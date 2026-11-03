"""Read-only probe: what drives LKAS_STATUS?

TRANSMITS NOTHING. Never calls can_send(). Safe to run repeatedly.

WHY THIS EXISTS
---------------
Two runs of steering_impersonate.py, six minutes apart, neither of which
transmitted a single frame:

    safety mode 0  (silent)     LKAS_STATUS = 5   frames counted twice
    safety mode 17 (allOutput)  LKAS_STATUS = 14  frames counted once,
                                                  camera msgs only on bus 2

The only thing that changed was the safety mode. That points at the intercept
relay: with the relay CLOSED, panda bus 0 and bus 2 are the same electrical
segment, so every frame is reported on both and counts double. With it OPEN
the segments are separate and each frame is reported once, on its own side.

So the hypothesis is:
    setting a non-silent safety mode opens the relay, and
    LKAS_STATUS 14 just means "the ADAS camera is not on the bus"

If that is right, LKAS_STATUS 14 is not a fault we caused and not something an
ignition cycle fixes - it is the car correctly noticing the camera is gone, and
it should clear the moment the relay closes again.

This probe separates the two variables so you can move one at a time:

    --mode      silent | alloutput        which safety mode to set
    --relay     leave | closed | open     whether to force the relay after

THE MATRIX TO FILL IN
---------------------
Run these in order, 15 s each, and write down LKAS_STATUS for each:

  1  --mode silent    --relay leave      baseline, expect 5
  2  --mode alloutput --relay leave      the suspect, expect 14
  3  --mode alloutput --relay closed     THE INTERESTING ONE
  4  --mode silent    --relay leave      does it come back to 5?

Row 3 is the one that matters. If forcing the relay closed under allOutput
gives LKAS_STATUS 5, then we can transmit with the camera still present, and
the path forward is completely different from what we have been doing.

Row 4 is the control. If it returns to 5, then 14 is live state tied to the
relay, NOT a latched fault - and every "status was already 14 before we
transmitted" reading we have was just the relay still being open from the run
before.

Stop openpilot first:  sudo systemctl stop comma
"""

import argparse
import collections
import time

from opendbc.car.structs import CarParams
from panda import Panda

# messages worth watching, and why
WATCH = {
    0x169: "LKAS          (camera's own steering command)",
    0x185: "STEER_TORQUE  (carries LKAS_STATUS)",
    0x280: "CANCEL_MSG",
    0x2B1: "PROPILOT_HUD  (camera)",
    0x4CC: "PROPILOT_HUD_INFO_MSG (camera)",
    0x30F: "CRUISE_STATE  (camera/ADAS)",
    0x4CB: "LKAS_SETTINGS (camera/ADAS)",
}

LKAS_STATUS_NAMES = {5: "INACTIVE", 6: "ACTIVE", 9: "FAULT", 14: "UNKNOWN-14"}

MODES = {
    "silent": CarParams.SafetyModel.silent,
    "alloutput": CarParams.SafetyModel.allOutput,
}


def unpack(msg):
    """panda reports the bus with flag bits in the top nibble - mask them."""
    if len(msg) == 3:
        addr, data, bus = msg[0], msg[1], msg[2]
    else:
        addr, data, bus = msg[0], msg[2], msg[3]
    return addr, bytes(data), bus & 0x0F


def decode_185(data):
    if len(data) < 8:
        return None
    raw = (data[2] << 10) | (data[3] << 2) | (data[4] >> 6)
    return 1310 - (raw * 0.01), (data[6] >> 4) & 0x0F


def main():
    p = argparse.ArgumentParser(description="read-only LKAS_STATUS probe")
    p.add_argument("--mode", choices=sorted(MODES), default="silent",
                   help="safety mode to set (default silent)")
    p.add_argument("--relay", choices=["leave", "closed", "open"],
                   default="leave",
                   help="force the relay after setting the mode, or leave it "
                        "to whatever the mode did (default leave)")
    p.add_argument("--seconds", type=float, default=15.0)
    args = p.parse_args()

    panda = Panda()
    panda.set_heartbeat_disabled()

    print()
    print(f"setting safety mode: {args.mode} "
          f"({MODES[args.mode]})")
    panda.set_safety_mode(MODES[args.mode])
    time.sleep(0.3)

    if args.relay == "closed":
        print("forcing relay CLOSED (camera connected to car)")
        panda.force_relay_drive(False, False)
        time.sleep(0.5)
    elif args.relay == "open":
        print("forcing relay OPEN (camera isolated)")
        panda.force_relay_drive(True, False)
        time.sleep(0.5)
    else:
        print("relay: left as the safety mode set it")

    h = panda.health()
    print(f"health: safety_mode {h.get('safety_mode')}  "
          f"harness {h.get('car_harness_status')}  "
          f"ignition {h.get('ignition_line')}  "
          f"controls_allowed {h.get('controls_allowed')}")

    print()
    print(f"listening {args.seconds:.0f} s. Transmitting nothing.")
    print()

    # addr -> bus -> count. The bus split is the relay-state tell.
    counts = collections.defaultdict(lambda: collections.Counter())
    statuses = collections.Counter()
    angle = None
    last_print = 0.0

    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        for msg in panda.can_recv():
            addr, data, bus = unpack(msg)
            counts[addr][bus] += 1
            if addr == 0x185:
                decoded = decode_185(data)
                if decoded is not None:
                    angle, status = decoded
                    statuses[status] += 1

        # live status line once a second
        now = time.monotonic()
        if now - last_print >= 1.0:
            last_print = now
            if statuses:
                cur = statuses.most_common(1)[0][0]
                print(f"  LKAS_STATUS {cur} "
                      f"({LKAS_STATUS_NAMES.get(cur, '?')})   "
                      f"angle {angle:+.2f}")

    print()
    print("=== LKAS_STATUS values seen ===")
    if not statuses:
        print("  no 0x185 at all - the car may be asleep")
    for status, n in statuses.most_common():
        print(f"  {status:>3} ({LKAS_STATUS_NAMES.get(status, '?'):<10}) "
              f"{n:>6} frames")

    print()
    print("=== watched messages, by bus ===")
    print("  two buses with similar counts = relay CLOSED (segments bridged)")
    print("  one bus only                  = relay OPEN  (segments separate)")
    print()
    for addr, label in WATCH.items():
        if addr in counts:
            spread = "  ".join(f"bus {b}: {n}"
                               for b, n in sorted(counts[addr].items()))
            print(f"  0x{addr:03X} {label:<42} {spread}")
        else:
            print(f"  0x{addr:03X} {label:<42} NOT SEEN")

    print()
    total_buses = collections.Counter()
    for addr in counts:
        for b, n in counts[addr].items():
            total_buses[b] += n
    print("all traffic by bus:",
          "  ".join(f"bus {b}: {n}" for b, n in sorted(total_buses.items())))

    # leave the panda somewhere harmless
    panda.set_safety_mode(CarParams.SafetyModel.silent)
    print()
    print("safety mode returned to silent. Nothing was transmitted.")


if __name__ == "__main__":
    main()