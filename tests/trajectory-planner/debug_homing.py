#!/usr/bin/env python3
import subprocess
import time
import sys
import linuxcnc

def main():
    print("Starting LinuxCNC...")
    subprocess.run(["/home/user/linuxcnc-dev/scripts/realtime", "stop"])
    subprocess.run(["killall", "-9", "axis", "linuxcnc", "milltask", "rtapi_app", "linuxcncsvr"], stderr=subprocess.DEVNULL)
    
    # Run in-place, capture output
    linuxcnc_proc = subprocess.Popen(
        ["/bin/bash", "-c", ". /home/user/linuxcnc-dev/scripts/rip-environment && linuxcnc /home/user/linuxcnc_sim_scurve/sim_scurve.ini"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    
    # Wait 3 seconds
    time.sleep(3.0)
    try:
        stat = linuxcnc.stat()
        cmd = linuxcnc.command()
        stat.poll()
        print("Connected.")
    except Exception as e:
        print(f"Failed to connect: {e}")
        linuxcnc_proc.terminate()
        out, err = linuxcnc_proc.communicate()
        print("STDOUT:")
        print(out)
        print("STDERR:")
        print(err)
        return

    print("Sending Estop Reset...")
    cmd.state(linuxcnc.STATE_ESTOP_RESET)
    time.sleep(1.0)
    stat.poll()
    print("Sending Machine ON...")
    cmd.state(linuxcnc.STATE_ON)
    time.sleep(1.0)
    stat.poll()

    print("Homing joint 0...")
    cmd.home(0)
    time.sleep(0.5)
    stat.poll()
    print("Homing joint 1...")
    cmd.home(1)
    time.sleep(0.5)
    stat.poll()
    print("Homing joint 2...")
    cmd.home(2)
    time.sleep(0.5)
    stat.poll()
    
    for i in range(5):
        time.sleep(0.5)
        stat.poll()
        print(f"  Step {i}: Homed status={stat.homed}")

    print("Stopping...")
    linuxcnc_proc.terminate()
    out, err = linuxcnc_proc.communicate()
    print("STDOUT:")
    print(out)
    print("STDERR:")
    print(err)

    subprocess.run(["killall", "-9", "axis", "linuxcnc", "milltask", "rtapi_app", "linuxcncsvr"], stderr=subprocess.DEVNULL)
    subprocess.run(["/home/user/linuxcnc-dev/scripts/realtime", "stop"], stderr=subprocess.DEVNULL)

if __name__ == "__main__":
    main()
