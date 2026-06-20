#!/usr/bin/env python3
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def parse_and_plot(log_file, plot_output):
    print(f"Parsing log file: {log_file}")
    data = []
    if not os.path.exists(log_file):
        print(f"Log file {log_file} does not exist!")
        return

    with open(log_file, "r") as f:
        for line in f:
            parts = line.strip().split()
            if not parts or len(parts) < 5:
                continue
            try:
                # fields: sample_num, x_pos, x_vel, y_pos, y_vel
                data.append([float(x) for x in parts])
            except ValueError:
                continue

    if not data:
        print("No valid data captured!")
        return

    data = np.array(data)
    
    # 1ms sampling interval
    dt = 0.001000
    t = (data[:, 0] - data[0, 0]) * dt # in seconds
    
    x_pos = data[:, 1]
    x_vel = data[:, 2]
    y_pos = data[:, 3]
    y_vel = data[:, 4]

    # Calculate acceleration and jerk numerically
    x_acc = np.diff(x_vel, prepend=x_vel[0]) / dt
    y_acc = np.diff(y_vel, prepend=y_vel[0]) / dt
    
    x_jerk = np.diff(x_acc, prepend=x_acc[0]) / dt
    y_jerk = np.diff(y_acc, prepend=y_acc[0]) / dt

    print("Generating plots...")
    fig, axs = plt.subplots(2, 2, figsize=(15, 10))

    # 1. XY Trajectory
    axs[0, 0].plot(x_pos, y_pos, label='Trajectory', color='blue')
    axs[0, 0].set_title('XY Toolpath Trajectory')
    axs[0, 0].set_xlabel('X Command Position (inch)')
    axs[0, 0].set_ylabel('Y Command Position (inch)')
    axs[0, 0].grid(True)
    axs[0, 0].axis('equal')

    # 2. Velocity vs Time
    axs[0, 1].plot(t, x_vel, label='X Velocity', color='red', alpha=0.8)
    axs[0, 1].plot(t, y_vel, label='Y Velocity', color='green', alpha=0.8)
    axs[0, 1].set_title('Velocity Profile (250us cycles)')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Velocity (inch/s)')
    axs[0, 1].legend()
    axs[0, 1].grid(True)

    # 3. Acceleration vs Time
    axs[1, 0].plot(t, x_acc, label='X Accel (computed)', color='red', alpha=0.8)
    axs[1, 0].plot(t, y_acc, label='Y Accel (computed)', color='green', alpha=0.8)
    axs[1, 0].set_title('Acceleration Profile')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Acceleration (inch/s²)')
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    # 4. Jerk vs Time
    axs[1, 1].plot(t, x_jerk, label='X Jerk (computed)', color='red', alpha=0.8)
    axs[1, 1].plot(t, y_jerk, label='Y Jerk (computed)', color='green', alpha=0.8)
    axs[1, 1].set_title('Jerk Profile')
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('Jerk (inch/s³)')
    axs[1, 1].legend()
    axs[1, 1].grid(True)

    plt.tight_layout()
    plt.savefig(plot_output, dpi=150)
    print(f"Plot saved to: {plot_output}")

if __name__ == "__main__":
    artifact_dir = "/home/user/.gemini/antigravity-cli/brain/6b8643c8-8af7-456f-a500-bfd126e81263"
    os.makedirs(artifact_dir, exist_ok=True)
    plot_output = os.path.join(artifact_dir, "tp1_performance.png")
    parse_and_plot("/tmp/sampler_data.txt", plot_output)
    print("Done!")
