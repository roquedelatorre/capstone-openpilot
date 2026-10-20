#!/usr/bin/env python3
"""
angle_check.py - live steering-angle readout for ground-truth scale check.

Decodes STEER_ANGLE from 0x185 (STEER_TORQUE_SENSOR) exactly as the openpilot
Nissan port does, and prints it live. Turn the wheel to known positions and
compare the printed value to reality to confirm the DBC scale is correct.

READ-ONLY: receives CAN only. Transmits nothing, changes no safety mode,
controls nothing.

Run on the device with openpilot stopped, ignition on:
    sudo systemctl stop comma
    python capstone_26/angle_check.py
Ctrl-C to stop.
"""
"positibe steering is LEFT and negative steering is RIGHT"

import time


def main():
    from panda import Panda
    p = Panda()
    p.set_heartbeat_disabled()   # so a long read session doesn't trip the panda

    print("Live STEER_ANGLE from 0x185 (the signal the port uses for control).")
    print("Decode: deg = 1310 - raw*0.01   (DBC: 18-bit, factor -0.01, offset 1310)")
    print()
    print("Procedure:")
    print("  1. Center the wheel (straight) -> should read ~0")
    print("  2. Turn the wheel exactly 90 deg one way -> should read ~+/-90")
    print("  3. Turn toward full lock -> compare magnitude")
    print("If the printed degrees match the wheel, the scale is correct.")
    print("(Sign may be inverted - that's fine, just note which way is +.)")
    print()

    last = 0.0
    try:
        while True:
            for m in p.can_recv():
                addr, dat, bus = m[0], m[-2], m[-1]
                if addr == 0x185 and bus == 0 and len(dat) >= 5:
                    raw = (dat[2] << 10) | (dat[3] << 2) | (dat[4] >> 6)
                    deg = 1310 - raw * 0.01
                    now = time.time()
                    if now - last > 0.25:
                        last = now
                        bar = int(max(-40, min(40, deg / 5)))
                        pointer = " " * (40 + bar) + "|"
                        print(f"STEER_ANGLE = {deg:8.2f} deg   {pointer}")
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nstopped.")


if __name__ == "__main__":
    main()