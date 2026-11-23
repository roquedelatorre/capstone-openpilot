"""What does the ADAS camera actually transmit? Read-only census.

WHY THIS EXISTS
---------------
On 2026-10-28 a dry run of steer_camera_isolated.py produced this, with
NOTHING transmitted:

    relay closed:  LKAS_STATUS = 5  (INACTIVE)
    relay open:    LKAS_STATUS = 14 (UNKNOWN-14)

That settles the question we have been chasing for a week. LKAS_STATUS 14 is
not the EPS rejecting our frames. It is what the EPS reports when the camera
is gone. Every run that showed 14 had the relay open; every run that showed
5 or 6 had it closed. We kept editing the frame because we thought the frame
caused it.

The same run also proved the topology:

    relay closed:  0x169 on bus 0 (309), bus 2 (309)
    relay open:    0x169 on bus 2 (600), NONE on bus 0

So the camera sits behind the relay, bus 0 is the car side, and opening the
relay makes us the only possible source of 0x169 - which is what we want -
at the cost of putting the EPS straight into a fault state, which is not.

The resolution has to be impersonation: open the relay AND provide everything
the camera was providing, so the EPS never notices it left. openpilot does
exactly this. We have been sending one message out of however many the camera
actually puts on the wire.

This script finds out how many, by difference rather than by guessing.

METHOD
------
  1. relay CLOSED  - census every address on bus 0. This is the car plus the
                     camera, bridged.
  2. relay OPEN    - census bus 0 again (car alone) and bus 2 (camera alone).
  3. subtract.

Addresses on bus 0 when closed but absent when open are the camera's
contribution. Addresses live on bus 2 when open are the camera transmitting
into the void. Those two sets should agree, and where they agree is the exact
list we must impersonate.

It also samples LKAS_STATUS throughout, so the 5 -> 14 -> 5 transition is
recorded with timestamps rather than inferred.

TRANSMITS NOTHING. The panda goes to allOutput only because the relay cannot
be moved otherwise; no transmitter is ever constructed.

    sudo systemctl stop comma
    python camera_census.py

The dash will raise a chassis / ProPILOT warning while the relay is open.
It clears on an ignition cycle.
"""

import argparse
import atexit
import time

from manual_can_interface import CANInterface

EPS_ADDR = 0x185
LKAS_ADDR = 0x169
LKAS_STATUS_NAMES = {5: "INACTIVE", 6: "ACTIVE", 9: "FAULT", 14: "NO-CAMERA?"}

# Addresses openpilot's Nissan port transmits, for cross-reference. If the
# census turns up an address here that we are not sending, that is a direct
# lead. If it turns up one NOT here, the 2025 car wants something the 2018
# port never had to provide.
OPENPILOT_TX = {
    0x169: "LKAS (steering command)",
    0x2b1: "PROPILOT_HUD",
    0x4cc: "PROPILOT_HUD_INFO_MSG",
    0x280: "CANCEL_MSG",
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


def census(panda, seconds, label):
    """Count every (bus, addr) for `seconds`. Returns a dict and a status log.

    Nothing is transmitting, so this drains flat out - no sleep.
    """
    counts = {}      # bus -> addr -> count
    samples = {}     # (bus, addr) -> last payload
    status_log = []  # (elapsed, status, angle)
    last_status = object()

    t0 = time.monotonic()
    end = t0 + seconds
    while time.monotonic() < end:
        for msg in panda.can_recv():
            addr, data, bus = unpack(msg)
            counts.setdefault(bus, {})
            counts[bus][addr] = counts[bus].get(addr, 0) + 1
            samples[(bus, addr)] = data
            if addr == EPS_ADDR:
                decoded = decode_185(data)
                if decoded is not None:
                    angle, status = decoded
                    if status != last_status:
                        status_log.append(
                            (time.monotonic() - t0, status, angle))
                        last_status = status

    total = sum(sum(a.values()) for a in counts.values())
    print(f"   {label}: {total} frames in {seconds:.1f} s across "
          f"{len(counts)} buses")
    return counts, samples, status_log


def show_status_log(log, seconds):
    if not log:
        print("   LKAS_STATUS: never seen (no 0x185 at all)")
        return
    print("   LKAS_STATUS transitions:")
    for t, status, angle in log:
        name = LKAS_STATUS_NAMES.get(status, "?")
        print(f"     t+{t:5.2f}s  status {status:>2} ({name})  "
              f"wheel {angle:+.2f} deg")
    if len(log) == 1:
        print(f"     (held for the whole {seconds:.1f} s)")


def rate(n, seconds):
    return n / seconds if seconds else 0.0


def main():
    p = argparse.ArgumentParser(
        description="read-only census of what the ADAS camera transmits")
    p.add_argument("--seconds", type=float, default=6.0, metavar="SEC",
                   help="census window for each phase (default 6)")
    p.add_argument("--min-rate", type=float, default=0.5, metavar="HZ",
                   help="ignore addresses slower than this when deciding "
                        "something vanished (default 0.5)")
    args = p.parse_args()

    if args.seconds > 30:
        p.error("30 s per phase is plenty")

    # allow_output must be True or set_relay() refuses. Nothing is ever
    # transmitted - no transmitter is constructed anywhere in this file.
    can = CANInterface(simulation=False, allow_output=True)
    panda = can.panda

    closed = {"done": False}

    def close_once():
        if closed["done"]:
            return
        closed["done"] = True
        try:
            can.set_relay(False)
            print("failsafe: relay closed on exit")
        except Exception as exc:
            print(f"!! FAILSAFE COULD NOT CLOSE THE RELAY: {exc}")
            print("   Camera may still be isolated. Ignition cycle the car.")

    atexit.register(close_once)

    print()
    print("READ-ONLY. No transmitter is constructed in this script.")
    print()

    # ---- phase 1: relay closed, car + camera bridged
    print(f"phase 1 - relay CLOSED, {args.seconds:.1f} s census")
    can.set_relay(False)
    time.sleep(0.8)
    c_counts, c_samples, c_log = census(panda, args.seconds, "closed")
    show_status_log(c_log, args.seconds)

    # ---- phase 2: relay open, camera isolated
    print()
    print(f"phase 2 - relay OPEN, {args.seconds:.1f} s census")
    print("          (dash warning expected - clears on ignition cycle)")
    can.set_relay(True)
    time.sleep(1.2)
    o_counts, o_samples, o_log = census(panda, args.seconds, "open")
    show_status_log(o_log, args.seconds)

    # put it back before any analysis, so a crash in the reporting code
    # cannot leave the camera isolated
    print()
    can.set_relay(False)
    closed["done"] = True
    time.sleep(0.8)

    print()
    print(f"phase 3 - relay CLOSED again, {args.seconds:.1f} s census")
    print("          (does LKAS_STATUS go back to 5? if so, 14 is reversible")
    print("           and camera-presence is the whole story)")
    _r_counts, _r_samples, r_log = census(panda, args.seconds, "reclosed")
    show_status_log(r_log, args.seconds)

    # ---------------- analysis ----------------
    car_closed = c_counts.get(0, {})
    car_open = o_counts.get(0, {})
    cam_open = o_counts.get(2, {})

    print()
    print("=" * 62)
    print("WHAT THE CAMERA CONTRIBUTES TO THE CAR-SIDE BUS")
    print("=" * 62)
    print()
    print("Addresses present on bus 0 with the relay CLOSED that are gone")
    print("or starved with it OPEN. These are the camera's, and this is the")
    print("list we must transmit ourselves while impersonating it.")
    print()

    vanished = []
    for addr, n in sorted(car_closed.items()):
        r_closed = rate(n, args.seconds)
        r_open = rate(car_open.get(addr, 0), args.seconds)
        if r_closed >= args.min_rate and r_open < args.min_rate:
            vanished.append((addr, r_closed, cam_open.get(addr, 0)))

    if not vanished:
        print("   none - nothing vanished from bus 0. That would mean the")
        print("   camera contributes nothing the car side can see, which")
        print("   contradicts the 0x169 result. Re-run before trusting it.")
    else:
        print(f"   {'addr':<8} {'Hz (closed)':>12} {'on bus 2 open':>15}  note")
        for addr, r_closed, cam_n in vanished:
            note = OPENPILOT_TX.get(addr, "")
            flag = "" if addr in OPENPILOT_TX else "  <-- NOT in openpilot TX"
            print(f"   0x{addr:03x}   {r_closed:>11.1f}  "
                  f"{rate(cam_n, args.seconds):>14.1f}  {note}{flag}")

    print()
    print("=" * 62)
    print("EVERYTHING THE CAMERA TRANSMITS (bus 2, relay OPEN)")
    print("=" * 62)
    print()
    if not cam_open:
        print("   nothing - bus 2 was silent with the relay open. That would")
        print("   mean the camera is not on bus 2, which contradicts the")
        print("   0x169 result. Re-run before trusting it.")
    else:
        print(f"   {'addr':<8} {'Hz':>7} {'DLC':>4}  sample payload")
        for addr, n in sorted(cam_open.items()):
            data = o_samples.get((2, addr), b"")
            hexed = " ".join(f"{b:02x}" for b in data)
            mark = " *" if addr in OPENPILOT_TX else ""
            print(f"   0x{addr:03x}   {rate(n, args.seconds):>6.1f} "
                  f"{len(data):>4}  {hexed}{mark}")
        print()
        print("   * = openpilot's Nissan port already transmits this one")

    missing = [a for a in sorted(cam_open) if a not in OPENPILOT_TX]
    if missing:
        print()
        print("   The camera transmits these and openpilot's Nissan port does")
        print("   NOT. On a 2018 Leaf that was fine. If the 2025 EPS needs any")
        print("   of them to consider the camera present, this is why the port")
        print("   does not work on our car:")
        print("     " + ", ".join(f"0x{a:03x}" for a in missing))

    print()
    print("=" * 62)
    print("DID 14 REVERSE?")
    print("=" * 62)

    def final_status(log):
        return log[-1][1] if log else None

    s_closed, s_open, s_re = (final_status(c_log), final_status(o_log),
                              final_status(r_log))
    print()
    print(f"   closed   -> {s_closed} "
          f"({LKAS_STATUS_NAMES.get(s_closed, '?')})")
    print(f"   open     -> {s_open} "
          f"({LKAS_STATUS_NAMES.get(s_open, '?')})")
    print(f"   reclosed -> {s_re} "
          f"({LKAS_STATUS_NAMES.get(s_re, '?')})")
    print()
    if s_closed == 5 and s_open == 14 and s_re == 5:
        print("   Confirmed and reversible. LKAS_STATUS 14 means 'camera")
        print("   missing', nothing more. It tracks the relay, not our")
        print("   frames, and it clears by itself when the camera returns.")
        print("   Every earlier run that showed 14 was telling us this.")
    elif s_re == 14:
        print("   14 did NOT clear when the camera came back. The EPS has")
        print("   latched a fault. Ignition cycle the car, and pull DTCs with")
        print("   a scan tool BEFORE you do - a latched fault is a stored")
        print("   code, and it will tell us in plain words what it wants.")
    else:
        print("   Not the pattern expected. Write down all three values and")
        print("   re-run once before building anything on them.")

    print()
    can.print_summary()


if __name__ == "__main__":
    main()