#!/usr/bin/env python3
import subprocess
import time
import os
import sys
import linuxcnc
import matplotlib.pyplot as plt
import numpy as np

def setup_sampler():
    print("Checking if sampler is already loaded...")
    res = subprocess.run(["sudo", "env", "RTAPI_UID=1000", "halcmd", "show", "comp", "sampler"], capture_output=True, text=True)
    if "sampler" not in res.stdout:
        print("Loading sampler real-time component...")
        # Create/ensure signals are netted to joint pins first (ignore errors if already netted)
        subprocess.run(["sudo", "env", "RTAPI_UID=1000", "halcmd", "net", "x_CmdJerk", "joint.0.jerk-cmd"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["sudo", "env", "RTAPI_UID=1000", "halcmd", "net", "y_CmdVel", "joint.1.vel-cmd"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["sudo", "env", "RTAPI_UID=1000", "halcmd", "net", "y_CmdJerk", "joint.1.jerk-cmd"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        commands = [
            ["sudo", "env", "RTAPI_UID=1000", "halcmd", "loadrt", "sampler", "depth=300000,300000", "cfg=ffffffff,ffffffff"],
            ["sudo", "env", "RTAPI_UID=1000", "halcmd", "net", "x_CmdPos", "sampler.1.pin.0"],
            ["sudo", "env", "RTAPI_UID=1000", "halcmd", "net", "x_CmdVel", "sampler.1.pin.1"],
            ["sudo", "env", "RTAPI_UID=1000", "halcmd", "net", "x_CmdAcc", "sampler.1.pin.2"],
            ["sudo", "env", "RTAPI_UID=1000", "halcmd", "net", "x_CmdJerk", "sampler.1.pin.3"],
            ["sudo", "env", "RTAPI_UID=1000", "halcmd", "net", "y-CmdPos", "sampler.1.pin.4"],
            ["sudo", "env", "RTAPI_UID=1000", "halcmd", "net", "y_CmdVel", "sampler.1.pin.5"],
            ["sudo", "env", "RTAPI_UID=1000", "halcmd", "net", "y_CmdAcc", "sampler.1.pin.6"],
            ["sudo", "env", "RTAPI_UID=1000", "halcmd", "net", "y_CmdJerk", "sampler.1.pin.7"],
            ["sudo", "env", "RTAPI_UID=1000", "halcmd", "addf", "sampler.1", "servo-thread"]
        ]
        for cmd in commands:
            r = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if r.returncode != 0:
                print(f"Error executing {' '.join(cmd)}")
                sys.exit(1)
    else:
        print("Sampler already loaded.")

def run_test(output_path):
    print("Connecting to LinuxCNC...")
    try:
        c = linuxcnc.command()
        s = linuxcnc.stat()
    except Exception as e:
        print("Failed to connect to running LinuxCNC. Make sure it is started!")
        print(e)
        sys.exit(1)

    print("Putting machine into reset-estop and turning ON...")
    c.state(linuxcnc.STATE_ESTOP_RESET)
    time.sleep(0.1)
    c.state(linuxcnc.STATE_ON)
    time.sleep(0.1)
    c.mode(linuxcnc.MODE_AUTO)
    time.sleep(0.1)

    gcode_file = "/home/user/linuxcnc/configs/sim/axis/1_1001_fast.ngc"
    print(f"Opening G-code file: {gcode_file}")
    c.program_open(gcode_file)
    time.sleep(0.5)

    # Start halsampler logging to a file in the background (low-weight, no stdout flooding)
    log_file = "/tmp/sampler_data.txt"
    if os.path.exists(log_file):
        os.remove(log_file)
    
    print(f"Starting halsampler in background -> {log_file}...")
    sampler_proc = subprocess.Popen(["sudo", "env", "RTAPI_UID=1000", "halsampler", "-c", "1", "-t"], stdout=open(log_file, "w"))

    print("Launching G-code run...")
    c.auto(linuxcnc.AUTO_RUN, 0)
    time.sleep(0.5)

    # Wait for run to progress past the G3 spiral and deep into the G1 cloud (line 500)
    print("Monitoring execution...")
    try:
        while True:
            s.poll()
            if s.motion_line >= 500 or s.interp_state == linuxcnc.INTERP_IDLE:
                break
            print(f"Current motion line: {s.motion_line}", end="\r")
            time.sleep(0.1)
        print(f"\nExecution reached line {s.motion_line}. Stopping run and logging.")
    finally:
        c.abort()
        time.sleep(0.5)
        sampler_proc.terminate()
        sampler_proc.wait()
        print("Logging stopped.")

def parse_and_plot(log_file, plot_output):
    print(f"Parsing log file: {log_file}")
    data = []
    with open(log_file, "r") as f:
        for line in f:
            parts = line.strip().split()
            if not parts or len(parts) < 9:
                continue
            try:
                # fields: sample_num, x_pos, x_vel, x_acc, x_jerk, y_pos, y_vel, y_acc, y_jerk
                data.append([float(x) for x in parts])
            except ValueError:
                continue

    if not data:
        print("No valid data captured!")
        return

    data = np.array(data)
    
    # 250us sampling interval
    t = (data[:, 0] - data[0, 0]) * 0.000250 # in seconds
    
    x_pos, x_vel, x_acc, x_jerk = data[:, 1], data[:, 2], data[:, 3], data[:, 4]
    y_pos, y_vel, y_acc, y_jerk = data[:, 5], data[:, 6], data[:, 7], data[:, 8]

    print("Generating plots...")
    fig, axs = plt.subplots(2, 2, figsize=(15, 10))

    # 1. XY Trajectory
    axs[0, 0].plot(x_pos, y_pos, label='Trajectory', color='blue')
    axs[0, 0].set_title('XY Toolpath Trajectory')
    axs[0, 0].set_xlabel('X Command Position (mm)')
    axs[0, 0].set_ylabel('Y Command Position (mm)')
    axs[0, 0].grid(True)
    axs[0, 0].axis('equal')

    # 2. Velocity vs Time
    axs[0, 1].plot(t, x_vel, label='X Velocity', color='red', alpha=0.8)
    axs[0, 1].plot(t, y_vel, label='Y Velocity', color='green', alpha=0.8)
    axs[0, 1].set_title('Velocity Profile (250us cycles)')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Velocity (mm/s)')
    axs[0, 1].legend()
    axs[0, 1].grid(True)

    # 3. Acceleration vs Time
    axs[1, 0].plot(t, x_acc, label='X Accel', color='red', alpha=0.8)
    axs[1, 0].plot(t, y_acc, label='Y Accel', color='green', alpha=0.8)
    axs[1, 0].set_title('Acceleration Profile')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Acceleration (mm/s²)')
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    # 4. Jerk vs Time
    axs[1, 1].plot(t, x_jerk, label='X Jerk', color='red', alpha=0.8)
    axs[1, 1].plot(t, y_jerk, label='Y Jerk', color='green', alpha=0.8)
    axs[1, 1].set_title('Jerk Profile')
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('Jerk (mm/s³)')
    axs[1, 1].legend()
    axs[1, 1].grid(True)

    plt.tight_layout()
    plt.savefig(plot_output, dpi=150)
    print(f"Plot saved to: {plot_output}")

if __name__ == "__main__":
    setup_sampler()
    run_test("/tmp/sampler_data.txt")
    
    # Save the output image to the artifacts directory
    artifact_dir = "/home/user/.gemini/antigravity-cli/brain/6b8643c8-8af7-456f-a500-bfd126e81263"
    os.makedirs(artifact_dir, exist_ok=True)
    plot_output = os.path.join(artifact_dir, "tp1_performance.png")
    
    parse_and_plot("/tmp/sampler_data.txt", plot_output)
    print("Done!")
