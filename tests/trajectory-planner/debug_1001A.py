#!/usr/bin/env python3
import subprocess
import time
import sys
import linuxcnc

def main():
    print("Starting LinuxCNC...")
    subprocess.run(["/home/user/linuxcnc-dev/scripts/realtime", "stop"])
    subprocess.run(["killall", "-9", "axis", "linuxcnc", "linuxcncsvr", "milltask", "rtapi_app"], stderr=subprocess.DEVNULL)
    subprocess.run(["rm", "-f", "/tmp/linuxcnc.lock"])

    linuxcnc_proc = subprocess.Popen(
        ["/bin/bash", "-c", ". /home/user/linuxcnc-dev/scripts/rip-environment && exec linuxcnc /home/user/linuxcnc_sim_scurve/sim_scurve.ini"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    
    time.sleep(3.0)
    try:
        stat = linuxcnc.stat()
        cmd = linuxcnc.command()
        err_channel = linuxcnc.error_channel()
        stat.poll()
        print("Connected.")
    except Exception as e:
        print(f"Failed to connect: {e}")
        out, err = linuxcnc_proc.communicate()
        print("STDOUT:", out)
        print("STDERR:", err)
        return

    # Enable machine
    cmd.state(linuxcnc.STATE_ESTOP_RESET)
    time.sleep(0.5)
    cmd.state(linuxcnc.STATE_ON)
    time.sleep(0.5)
    
    cmd.teleop_enable(0)
    time.sleep(0.1)
    cmd.home(0)
    cmd.home(1)
    cmd.home(2)
    
    # Wait for homing
    for i in range(20):
        stat.poll()
        if stat.homed[0] and stat.homed[1] and stat.homed[2]:
            break
        time.sleep(0.1)
        
    gcode = "/home/user/linuxcnc/configs/sim/axis/1001_A.ngc"
    print("Opening G-code...")
    cmd.mode(linuxcnc.MODE_AUTO)
    time.sleep(0.1)
    cmd.program_open(gcode)
    time.sleep(0.5)
    
    print("Running G-code...")
    cmd.auto(linuxcnc.AUTO_RUN, 0)
    
    # Monitor execution
    for i in range(40):
        time.sleep(0.1)
        stat.poll()
        print(f"Cycle {i}: line={stat.motion_line}, state={stat.interp_state}, exec_state={stat.exec_state}")
        # Check error channel
        err_msg = err_channel.poll()
        if err_msg:
            kind, text = err_msg
            print(f"  Error/Message: kind={kind}, text={text}")

    print("Stopping...")
    linuxcnc_proc.terminate()
    out, err = linuxcnc_proc.communicate()
    print("STDOUT:")
    print(out)
    print("STDERR:")
    print(err)

    subprocess.run(["killall", "-9", "axis", "linuxcnc", "linuxcncsvr", "milltask", "rtapi_app"], stderr=subprocess.DEVNULL)
    subprocess.run(["/home/user/linuxcnc-dev/scripts/realtime", "stop"], stderr=subprocess.DEVNULL)

if __name__ == "__main__":
    main()
