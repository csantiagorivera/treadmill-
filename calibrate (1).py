#!/usr/bin/env python3
"""
Treadmill feedforward calibration
----------------------------------------------
Talks to the ESP32 over plain serial (no ROS needed), triggers the firmware's
duty sweep ('C'), collects 'CAL,duty,measured_v' lines until 'CAL,DONE', then
least-squares fits  v = a*duty + b  and prints the firmware constants:

    FF_SLOPE     = 1/a            (duty per m/s, ignoring offset)
    FF_INTERCEPT = -b/a

Paste those into treadmill_sync_firmware.ino.

Usage:
    python3 calibrate.py --port /dev/ttyUSB0
Safety: only run with the E-stop reachable and the belt clear. The sweep WILL
drive the belt up to near max speed.
"""

import argparse
import sys
import time

import serial
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--out', default='calibration.csv')
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=2.0)
    time.sleep(2.0)  # let the board reset/settle after port open
    ser.reset_input_buffer()

    input('Belt clear? E-stop in reach? Press Enter to start the sweep... ')
    ser.write(b'C\n')

    duties, vels = [], []
    print('duty,measured_v')
    deadline = time.time() + 120  # safety timeout
    while time.time() < deadline:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if not line:
            continue
        if line == 'CAL,DONE':
            break
        if line.startswith('CAL,'):
            try:
                _, d, v = line.split(',')
                d, v = int(d), float(v)
            except ValueError:
                continue
            duties.append(d)
            vels.append(v)
            print(f'{d},{v:.4f}')
        elif line.startswith('#'):
            print(f'[esp32] {line[1:].strip()}', file=sys.stderr)

    ser.write(b'X\n')  # ensure stop
    ser.close()

    if len(duties) < 3:
        print('Not enough points collected — check wiring/E-stop.', file=sys.stderr)
        sys.exit(1)

    duties = np.array(duties, dtype=float)
    vels = np.array(vels, dtype=float)

    # fit v = a*duty + b  (only points where the belt actually moved)
    mask = vels > 0.02
    a, b = np.polyfit(duties[mask], vels[mask], 1)

    np.savetxt(args.out, np.column_stack([duties, vels]),
               delimiter=',', header='duty,measured_v', comments='')

    ff_slope = 1.0 / a
    ff_intercept = -b / a
    print('\n--- Paste into treadmill_sync_firmware.ino ---')
    print(f'float FF_SLOPE     = {ff_slope:.2f}f;   // duty per m/s')
    print(f'float FF_INTERCEPT = {ff_intercept:.2f}f;')
    print(f'// fit: v = {a:.6f}*duty + {b:.4f}   (R/ saved to {args.out})')
    vmax = a * duties.max() + b
    print(f'// observed max belt velocity at full duty ~= {vmax:.2f} m/s '
          f'({vmax*3.6:.1f} km/h) -> set V_MAX accordingly')


if __name__ == '__main__':
    main()
