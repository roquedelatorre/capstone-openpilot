#!/usr/bin/env python3
"""
port_probe.py - resolve the camera-vs-car bus topology for the Leaf port.

With the harness relay CLOSED (stock), the car side and camera side are
bridged, so every message appears on both bus 0 and bus 2 and you cannot tell
where it originates. This opens the relay to isolate the camera, captures which
bus each message lands on, then CLOSES the relay again (in a finally, so it is
never left open).

  camera-side messages  -> appear only on bus 2 (openpilot impersonates these)
  car-side messages     -> appear only on bus 0 (openpilot reads these)

Run with openpilot stopped, ignition ON, car in PARK:

    sudo systemctl stop comma
    python capstone_26/port_probe.py

This script only reads the bus and toggles the relay. It transmits nothing.
"""

import time

# messages the port cares about, for the summary
WATCH = {
    0x185: "STEER_TORQUE_SENSOR (angle feedback)",
    0x169: "LKAS (steer command target)",
    0x285: "WHEEL_SPEEDS_REAR",
    0x239: "CRUISE_THROTTLE",
    0x30f: "CRUISE_STATE",
    0x2b1: "PROPILOT_HUD",
    0x4cb: "LKAS_SETTINGS",
    0x4cc: "PROPILOT_HUD_INFO_MSG",
    0x358: "LIGHTS (blinkers)",
    0x60d: "DOORS_LIGHTS",
    0x421: "GEARBOX",
}


def capture(panda, seconds):
    seen = {}
    t = time.monotonic()
    while time.monotonic() - t < seconds:
        for m in panda.can_recv():
            bus = m[-1]
            if bus < 128:                      # skip panda TX echoes
                seen.setdefault(m[0], set()).add(bus)
    return seen


def main():
    from panda import Panda
    panda = Panda()
    panda.set_heartbeat_disabled()

    # liveness check first - a dead/asleep bus makes every result meaningless
    live = capture(panda, 2.0)
    if 0x185 not in live:
        print("!! No STEER_TORQUE (0x185) seen - bus looks asleep.")
        print("   Turn the ignition ON and wait for the dash to come up, then re-run.")
        return
    print(f"bus is live ({len(live)} ids seen). opening relay...")

    try:
        panda.force_relay_drive(True, False)   # OPEN - isolate the camera
        time.sleep(0.5)
        seen = capture(panda, 5.0)
    finally:
        panda.force_relay_drive(False, False)  # CLOSE - back to stock
        print("relay closed (back to stock).")

    print()
    print("=== bus topology with camera isolated (relay open) ===")
    print(f"{'ADDR':<7}{'BUSES':<10}{'ORIGIN':<12}MESSAGE")
    for addr, name in WATCH.items():
        buses = sorted(seen.get(addr, []))
        if buses == [2]:
            origin = "CAMERA"
        elif buses == [0]:
            origin = "CAR"
        elif not buses:
            origin = "absent"
        else:
            origin = "both/fwd"
        bstr = ",".join(str(b) for b in buses) if buses else "-"
        print(f"0x{addr:03X}  {bstr:<10}{origin:<12}{name}")

    print()
    print("CAMERA = openpilot must synthesize it (expect 0x169 here).")
    print("CAR    = openpilot reads it (expect 0x185, 0x285, 0x239 here).")
    print("both/fwd = a gateway forwards it; note which for the parser map.")


if __name__ == "__main__":
    main()