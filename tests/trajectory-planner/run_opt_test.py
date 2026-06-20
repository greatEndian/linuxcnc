#!/usr/bin/env python3
import subprocess
import time
import os
import sys
import linuxcnc

def run_command(cmd_str):
    print(f"Running: {cmd_str}")
    full_cmd = f". /home/user/linuxcnc-dev/scripts/rip-environment && {cmd_str}"
    res = subprocess.run(full_cmd, shell=True, executable="/bin/bash", capture_output=True, text=True)
    if res.returncode != 0:
        print(f"Command failed: {res.stderr}")
    return res

def main():
    # 1. Start LinuxCNC in the background
    print("Starting LinuxCNC...")
    # Clean up any leftover lockfiles, realtime sessions and linuxcnc processes
    subprocess.run(["rm", "-f", "/tmp/linuxcnc.lock"])
    subprocess.run(["/home/user/linuxcnc-dev/scripts/realtime", "stop"])
    subprocess.run(["killall", "-9", "axis", "linuxcnc", "linuxcncsvr", "milltask", "rtapi_app"], stderr=subprocess.DEVNULL)
    
    # Start LinuxCNC using our custom simulation config (with exec to avoid shell wrappers)
    linuxcnc_proc = subprocess.Popen(
        ["/bin/bash", "-c", ". /home/user/linuxcnc-dev/scripts/rip-environment && exec linuxcnc /home/user/linuxcnc_sim_scurve/sim_scurve.ini"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    
    # 2. Wait for LinuxCNC to initialize
    print("Connecting to LinuxCNC...")
    stat = None
    cmd = None
    for i in range(30):
        try:
            stat = linuxcnc.stat()
            cmd = linuxcnc.command()
            stat.poll()
            print("Connected to LinuxCNC successfully!")
            break
        except Exception:
            time.sleep(0.5)
            
    if not stat or not cmd:
        print("Failed to start or connect to LinuxCNC!")
        linuxcnc_proc.terminate()
        sys.exit(1)

    # 3. Enable machine and home joints
    print("Enabling machine and homing joints...")
    cmd.state(linuxcnc.STATE_ESTOP_RESET)
    time.sleep(0.5)
    cmd.state(linuxcnc.STATE_ON)
    time.sleep(0.5)
    
    cmd.teleop_enable(0)
    time.sleep(0.1)
    cmd.home(0)
    cmd.home(1)
    cmd.home(2)
    
    # Wait for homing to complete
    homed = False
    for i in range(100):
        stat.poll()
        if stat.homed[0] and stat.homed[1] and stat.homed[2]:
            homed = True
            break
        time.sleep(0.2)
        
    if not homed:
        print("Homing timed out!")
        linuxcnc_proc.terminate()
        sys.exit(1)
    print("Machine is fully homed.")

    # 4. Start py_logger.py in background (with exec to avoid shell wrappers)
    print("Starting py_logger.py in background...")
    logger_proc = subprocess.Popen(
        ["/bin/bash", "-c", ". /home/user/linuxcnc-dev/scripts/rip-environment && exec python3 /home/user/linuxcnc-dev/tests/trajectory-planner/py_logger.py"],
        stdout=sys.stdout,
        stderr=sys.stderr
    )
    
    # Wait for logger HAL component to be ready
    time.sleep(2.0)
    
    # Connect HAL pins
    print("Connecting logger HAL pins...")
    run_command("halcmd net x_CmdPos logger.x_pos")
    run_command("halcmd net x_CmdVel logger.x_vel")
    run_command("halcmd net y-CmdPos logger.y_pos")
    run_command("halcmd net y_CmdVel joint.1.vel-cmd logger.y_vel")

    # 5. Open and run G-code
    gcode_file = sys.argv[1] if len(sys.argv) > 1 else "/home/user/linuxcnc/configs/sim/axis/1_1001_fast.ngc"
    print(f"Opening G-code: {gcode_file}")
    cmd.mode(linuxcnc.MODE_AUTO)
    time.sleep(0.1)
    cmd.program_open(gcode_file)
    time.sleep(0.5)
    
    print("Running G-code...")
    cmd.auto(linuxcnc.AUTO_RUN, 0)
    time.sleep(0.5)
    
    # Wait for G-code to run
    print("Monitoring execution...")
    try:
        while True:
            stat.poll()
            # Stop if we reach line 500 or if interpreter is idle (completed)
            if stat.motion_line >= 500 or stat.interp_state == linuxcnc.INTERP_IDLE:
                break
            print(f"Line: {stat.motion_line}  ", end="\r")
            time.sleep(0.1)
        print(f"\nExecution reached line {stat.motion_line}.")
    finally:
        cmd.abort()
        time.sleep(0.5)
        # Terminate logger
        logger_proc.terminate()
        logger_proc.wait()
        print("Logger stopped.")
        
        # Close LinuxCNC by killing child processes first to avoid deadlock
        subprocess.run(["killall", "-9", "axis", "linuxcnc", "linuxcncsvr", "milltask", "rtapi_app"], stderr=subprocess.DEVNULL)
        subprocess.run(["/home/user/linuxcnc-dev/scripts/realtime", "stop"], stderr=subprocess.DEVNULL)
        subprocess.run(["rm", "-f", "/tmp/linuxcnc.lock"])
        
        # Terminate and wait for the wrapper script process to exit
        try:
            linuxcnc_proc.terminate()
            linuxcnc_proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            linuxcnc_proc.kill()

    # 6. Generate Plot
    print("Generating plot...")
    plot_script = "/home/user/linuxcnc-dev/tests/trajectory-planner/plot_only.py"
    subprocess.run(["python3", plot_script])
    print("Done!")

if __name__ == "__main__":
    main()
