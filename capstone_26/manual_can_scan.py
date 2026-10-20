# ADDED (Saul) - bus scanner. Read-only, SAFETY_SILENT.
#
# Why this exists: 0x169 does not appear in any capture we have. Until we
# know whether the camera transmits it, and on which bus, steering is
# guesswork - LKAS_BUS in manual_nissancan.py is currently a placeholder.
# This also dumps frames in the format verify_checksums.py expects, which
# is how we settle the init 0x00 vs 0xFF question on the LKAS checksum.
#
# Run on the comma, openpilot stopped:
#
#   python capstone_26/manual_can_scan.py --seconds 10
#   python capstone_26/manual_can_scan.py --id 0x169 --frames 20

import argparse
import collections
import time

# IDs we care about, and what we believe about each
KNOWN = {
    0x002: "STEER_ANGLE_SENSOR  (wheel position, confirmed)",
    0x169: "LKAS                (steering command - NOT yet seen)",
    0x185: "STEER_TORQUE_SENSOR (angle + driver torque feedback)",
    0x27A: "TURN_SIGNALS        (lamp state, bus 2, ~47.5 Hz)",
    0x3AF: "HAZARD_SWITCH       (button, bus 0, ~10 Hz)",
    0x60D: "DOORS_LIGHTS        (bus 0, ~10.3 Hz)",
}


def open_panda():
    from panda import Panda
    from opendbc.car.structs import CarParams

    p = Panda()
    p.set_heartbeat_disabled()
    # SILENT cannot transmit. This script never puts a frame on the bus.
    p.set_safety_mode(CarParams.SafetyModel.silent)
    return p


def unpack(msg):
    """panda returns (addr, data, bus) on current firmware and
    (addr, busTime, data, src) on older. Handle both."""
    if len(msg) == 3:
        return msg[0], msg[1], msg[2]
    return msg[0], msg[2], msg[3]


def scan(seconds):
    p = open_panda()
    print(f"listening {seconds:.0f} s, nothing will be transmitted\n")

    # (bus, addr) -> [count, length, first_bytes, last_bytes]
    seen = collections.OrderedDict()
    end = time.monotonic() + seconds

    while time.monotonic() < end:
        for msg in p.can_recv():
            addr, data, bus = unpack(msg)
            key = (bus, addr)
            if key not in seen:
                seen[key] = [0, len(data), bytes(data), bytes(data)]
            seen[key][0] += 1
            seen[key][3] = bytes(data)

    if not seen:
        print("NOTHING RECEIVED.")
        print("  - is openpilot still running and holding the panda?")
        print("  - is the car awake? most buses go quiet when it sleeps")
        print("  - check harness status with blinker_bringup.py --check")
        return

    print(f"{'bus':>4} {'ID':>7} {'len':>4} {'count':>7} {'Hz':>7}  note")
    print("-" * 70)
    for (bus, addr), (count, length, first, last) in sorted(seen.items()):
        rate = count / seconds
        note = KNOWN.get(addr, "")
        changing = "" if first == last else "  (bytes changing)"
        print(f"{bus:>4} 0x{addr:05X} {length:>4} {count:>7} {rate:>7.1f}  "
              f"{note}{changing}")

    print()
    for addr, label in KNOWN.items():
        buses = [b for (b, a) in seen if a == addr]
        if buses:
            print(f"  0x{addr:03X} FOUND on bus {buses}  - {label}")
        else:
            print(f"  0x{addr:03X} not seen          - {label}")


def dump(target_id, n_frames):
    p = open_panda()
    print(f"collecting {n_frames} frames of 0x{target_id:03X}\n")

    out = []
    end = time.monotonic() + 15.0

    while len(out) < n_frames and time.monotonic() < end:
        for msg in p.can_recv():
            addr, data, bus = unpack(msg)
            if addr == target_id:
                out.append((bus, bytes(data)))
                if len(out) >= n_frames:
                    break

    if not out:
        print(f"0x{target_id:03X} never appeared in 15 s.")
        return

    print(f"seen on bus {sorted({b for b, _ in out})}\n")
    print("paste into verify_checksums.py:\n")
    for bus, data in out:
        hexed = " ".join(f"{b:02X}" for b in data)
        print(f'    "{hexed}",')


def main():
    ap = argparse.ArgumentParser(description="read-only CAN bus scan")
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="how long to listen for the summary scan")
    ap.add_argument("--id", type=lambda s: int(s, 0), default=None,
                    help="dump raw frames for one ID instead, e.g. 0x169")
    ap.add_argument("--frames", type=int, default=20,
                    help="how many frames to dump with --id")
    args = ap.parse_args()

    if args.id is not None:
        dump(args.id, args.frames)
    else:
        scan(args.seconds)


if __name__ == "__main__":
    main()