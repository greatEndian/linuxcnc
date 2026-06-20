#!/usr/bin/env python3
import hal
import time
import sys
import signal

# Clean exit handler
quit_logger = False
def sig_handler(signum, frame):
    global quit_logger
    quit_logger = True

signal.signal(signal.SIGINT, sig_handler)
signal.signal(signal.SIGTERM, sig_handler)

def main():
    global quit_logger
    print("Initializing Python HAL Logger...")
    comp = hal.component("logger")
    comp.newpin("x_pos", hal.HAL_FLOAT, hal.HAL_IN)
    comp.newpin("x_vel", hal.HAL_FLOAT, hal.HAL_IN)
    comp.newpin("y_pos", hal.HAL_FLOAT, hal.HAL_IN)
    comp.newpin("y_vel", hal.HAL_FLOAT, hal.HAL_IN)
    comp.ready()
    print("HAL Component 'logger' is ready. Connect pins and run G-code.")

    log_file = "/tmp/sampler_data.txt"
    with open(log_file, "w") as f:
        sample_num = 0
        t_start = time.time()
        while not quit_logger:
            # We log sample_num, x_pos, x_vel, y_pos, y_vel
            # To match the plot_only.py expectations: sample_num x_pos x_vel y_pos y_vel
            try:
                f.write(f"{sample_num} {comp['x_pos']:.6f} {comp['x_vel']:.6f} {comp['y_pos']:.6f} {comp['y_vel']:.6f}\n")
                f.flush()
            except Exception as e:
                print(f"Write error: {e}")
                break
            
            sample_num += 1
            
            # Precise 1ms sleep interval (compensating for execution time)
            t_next = t_start + sample_num * 0.001
            t_sleep = t_next - time.time()
            if t_sleep > 0:
                time.sleep(t_sleep)
            else:
                # If we fall behind, yield a bit
                time.sleep(0.0001)

    print(f"Logged {sample_num} samples to {log_file}.")

if __name__ == "__main__":
    main()
