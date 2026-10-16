"""Receive STEP records from the STM32 and plot their already-computed XYZ coordinates.
No step detection, counting, filtering, or coordinate estimation is performed here.
"""
import argparse
import csv
import sys
import time
from pathlib import Path

import serial
import matplotlib.pyplot as plt

FIELDS = [
    "record", "observed_leg_events", "estimated_total_steps", "time_ms",
    "ax_mg", "ay_mg", "az_mg", "gx_cdeg_s", "gy_cdeg_s", "gz_cdeg_s",
    "x_mm", "y_mm", "z_mm",
]


def main():
    parser = argparse.ArgumentParser(description="Plot STM32-computed step XYZ records in 3D")
    parser.add_argument("--port", required=True, help="Windows COM port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--csv", default="step_records.csv", help="Output CSV filename")
    args = parser.parse_args()

    output_path = Path(args.csv)
    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    time.sleep(1.5)  # USB-to-TTL serial adapters may reset/reopen when port opens
    print(f"Connected to {args.port} at {args.baud} baud. Waiting for STEP records; Ctrl+C to stop.")
    print("The PC only parses records and plots the X/Y/Z coordinates sent by the STM32.")

    plt.ion()
    fig = plt.figure("STM32 Step Map")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Estimated IMU trajectory at accepted step events")
    xs, ys, zs = [], [], []
    line, = ax.plot([], [], [], "-o", markersize=4)
    status = ax.text2D(0.02, 0.95, "Waiting for steps...", transform=ax.transAxes)

    file_exists = output_path.exists() and output_path.stat().st_size > 0
    csv_file = output_path.open("a", newline="", encoding="utf-8")
    writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
    if not file_exists:
        writer.writeheader()
        csv_file.flush()

    try:
        while plt.fignum_exists(fig.number):
            raw = ser.readline()
            if not raw:
                plt.pause(0.01)
                continue
            text = raw.decode("ascii", errors="replace").strip()
            if not text:
                continue
            print(text)
            parts = text.split(",")
            if len(parts) != 13 or parts[0] != "STEP":
                # Ignore BOOT/READY/ERROR messages; they are not step records.
                continue
            try:
                row = dict(zip(FIELDS, parts))
                # Firmware transmits millimetres and fixed-point sensor integers.
                x = int(row["x_mm"]) / 1000.0
                y = int(row["y_mm"]) / 1000.0
                z = int(row["z_mm"]) / 1000.0
                event_count = int(row["observed_leg_events"])
                total_estimate = int(row["estimated_total_steps"])
            except (ValueError, KeyError):
                print("Skipping malformed STEP record", file=sys.stderr)
                continue

            writer.writerow(row)
            csv_file.flush()
            xs.append(x); ys.append(y); zs.append(z)
            line.set_data(xs, ys)
            line.set_3d_properties(zs)
            status.set_text(f"Observed-leg events: {event_count} | Estimated total footfalls: {total_estimate}")
            # Set useful bounds without performing any coordinate estimation.
            ax.set_xlim(min(xs) - 0.25, max(xs) + 0.25)
            ax.set_ylim(min(ys) - 0.25, max(ys) + 0.25)
            zmin, zmax = min(zs), max(zs)
            ax.set_zlim(zmin - 0.15, zmax + 0.15 if zmax > zmin else zmin + 0.30)
            fig.canvas.draw_idle()
            plt.pause(0.001)
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        csv_file.close()
        ser.close()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
