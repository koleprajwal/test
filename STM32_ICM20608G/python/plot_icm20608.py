"""
Live ICM-20608-G serial plotter for Windows 11.
Install: py -m pip install pyserial matplotlib
Run:     py plot_icm20608.py --port COM5
Use the COM port assigned to your USB-to-TTL adapter.
"""
import argparse
import csv
import time
from collections import deque
from datetime import datetime
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Windows COM port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--window", type=float, default=10.0, help="Plot window in seconds")
    parser.add_argument("--csv", default="icm20608_log.csv", help="CSV log filename")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.02)
    time.sleep(1.5)
    ser.reset_input_buffer()

    # Firmware prints diagnostic lines beginning with '#'; data rows have 8 fields.
    t = deque()
    accel = [deque(), deque(), deque()]
    gyro = [deque(), deque(), deque()]
    start_ms = None
    csv_file = open(args.csv, "w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow(["timestamp_ms", "accel_x_g", "accel_y_g", "accel_z_g",
                     "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "temperature_c"])

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
    lines_a = [ax1.plot([], [], label=n)[0] for n in ("Accel X", "Accel Y", "Accel Z")]
    lines_g = [ax2.plot([], [], label=n)[0] for n in ("Gyro X", "Gyro Y", "Gyro Z")]
    ax1.set_ylabel("Acceleration (g)")
    ax2.set_ylabel("Angular rate (deg/s)")
    ax2.set_xlabel("Time (s)")
    ax1.grid(True); ax2.grid(True)
    ax1.legend(loc="upper left"); ax2.legend(loc="upper left")
    status = fig.suptitle(f"Waiting for ICM-20608-G on {args.port}")

    def update(_frame):
        nonlocal start_ms
        # Drain all available lines so plotting doesn't make serial input backlog.
        while ser.in_waiting:
            try:
                line = ser.readline().decode("ascii", errors="ignore").strip()
            except serial.SerialException:
                return lines_a + lines_g
            if not line:
                continue
            if line.startswith("#"):
                status.set_text(line)
                continue
            parts = line.split(",")
            if len(parts) != 8:
                continue
            try:
                vals = [float(v) for v in parts]
            except ValueError:
                continue
            stamp = vals[0]
            if start_ms is None:
                start_ms = stamp
            x = (stamp - start_ms) / 1000.0
            t.append(x)
            for i in range(3):
                accel[i].append(vals[1+i])
                gyro[i].append(vals[4+i])
            writer.writerow(vals)
        if t:
            cutoff = t[-1] - args.window
            while t and t[0] < cutoff:
                t.popleft()
                for q in accel + gyro:
                    q.popleft()
            for i in range(3):
                lines_a[i].set_data(t, accel[i])
                lines_g[i].set_data(t, gyro[i])
            ax1.set_xlim(max(0, t[-1] - args.window), max(args.window, t[-1]))
            for axis, queues in ((ax1, accel), (ax2, gyro)):
                vals = [v for q in queues for v in q]
                if vals:
                    lo, hi = min(vals), max(vals)
                    pad = max((hi-lo)*0.15, 0.1)
                    axis.set_ylim(lo-pad, hi+pad)
            status.set_text(f"ICM-20608-G live | {len(t)} points in plot window | logging: {args.csv}")
        return lines_a + lines_g

    ani = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
    try:
        plt.tight_layout()
        plt.show()
    finally:
        ser.close()
        csv_file.close()

if __name__ == "__main__":
    main()
