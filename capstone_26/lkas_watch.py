"""Read-only 0x185 monitor for the 2025 Leaf.

Transmits nothing. Opens the panda with allow_output=False, never touches
the intercept relay, and never imports the TX path. There is no flag that
makes this script send a frame.

Purpose: answer the three things we still don't know before step 9.

  1. Does STEER_ANGLE track 1:1 across the FULL lock range? We verified the
     lane-keeping range on 2026-10-16; this covers the rest.
  2. What is LKAS_STATUS 14? We saw it once on this car with the relay open
     and the chassis faulted. It is not in the 2018 enum. Log every distinct
     value with the car state at the time.
  3. What does the EPS report when it is genuinely accepting commands?
     Watch with the relay CLOSED while stock ProPILOT is engaged on a drive.
     That is our reference for what "working" looks like.

Usage:
    python lkas_watch.py
    python lkas_watch.py --csv lock_sweep.csv --note "full lock L to R, park"
    python lkas_watch.py --csv propilot.csv --note "stock PP engaged, hwy"

Ctrl+C to stop. A summary of every distinct status seen prints at the end.
"""

import argparse
import csv
import time
from collections import Counter

from manual_can_interface import CANInterface

# 5/6/9 are the 2018 Leaf enum. 14 is ours, unexplained - that is the point
# of this script. Anything not listed prints as UNKNOWN and is counted.
LKAS_STATUS_NAMES = {
    5: "INACTIVE",
    6: "ACTIVE",
    9: "FAULT",
    14: "UNKNOWN-14",
}


def unpack(msg):
    """panda returns (addr, data, bus) or (addr, busTime, data, src).
    The bus byte carries flag bits in the top nibble, so mask them off -
    that is why the scan reported buses 128 / 130 / 192 / 194."""
    if len(msg) == 3:
        addr, data, bus = msg[0], msg[1], msg[2]
    else:
        addr, data, bus = msg[0], msg[2], msg[3]
    return addr, data, bus & 0x0F


# 0x185 STEER_TORQUE_SENSOR
#   STEER_ANGLE   23|18@0+  scale -0.01  offset 1310
#   LKAS_STATUS   55|4@0+
def decode_185(data):
    if len(data) < 8:
        return None
    raw = (data[2] << 10) | (data[3] << 2) | (data[4] >> 6)
    angle = 1310 - (raw * 0.01)
    status = (data[6] >> 4) & 0x0F
    return angle, status


def main():
    p = argparse.ArgumentParser(
        description="read-only 0x185 angle + LKAS_STATUS monitor")
    p.add_argument("--csv", metavar="PATH", default=None,
                   help="append every sample to this CSV")
    p.add_argument("--note", default="",
                   help="free-text tag written into every CSV row, e.g. "
                        "'full lock sweep, park' or 'stock PP engaged'")
    p.add_argument("--bus", type=int, default=None, metavar="N",
                   help="only accept 0x185 from this bus (default: any)")
    args = p.parse_args()

    # allow_output is hard-coded False. Do not add a flag for this.
    can = CANInterface(simulation=False, allow_output=False)
    panda = can.panda

    writer = None
    fh = None
    if args.csv:
        fh = open(args.csv, "a", newline="")
        writer = csv.writer(fh)
        if fh.tell() == 0:
            writer.writerow(
                ["wall_time", "mono_s", "bus", "angle_deg", "status",
                 "status_name", "raw_hex", "note"])

    print()
    print("read-only. Nothing will be transmitted. Relay untouched.")
    if args.csv:
        print(f"logging to {args.csv}")
    print("Ctrl+C to stop.")
    print()

    status_counts = Counter()
    first_seen = {}
    angle_min = None
    angle_max = None
    samples = 0
    last_print = 0.0
    t0 = time.monotonic()

    try:
        while True:
            for msg in panda.can_recv():
                addr, data, bus = unpack(msg)
                if addr != 0x185:
                    continue
                if args.bus is not None and bus != args.bus:
                    continue

                result = decode_185(data)
                if result is None:
                    continue

                angle, status = result
                now = time.monotonic()
                elapsed = now - t0
                name = LKAS_STATUS_NAMES.get(status, "UNKNOWN")

                samples += 1
                status_counts[status] += 1
                if status not in first_seen:
                    first_seen[status] = (elapsed, angle)
                    # a status we have never seen is the interesting event -
                    # print it immediately, out of band from the rate limit
                    print(f"  [{elapsed:7.1f}s] NEW STATUS {status} ({name}) "
                          f"at {angle:+.2f} deg  raw={bytes(data).hex()}")
                if angle_min is None or angle < angle_min:
                    angle_min = angle
                if angle_max is None or angle > angle_max:
                    angle_max = angle

                if writer:
                    writer.writerow([
                        time.strftime("%Y-%m-%dT%H:%M:%S"),
                        f"{elapsed:.3f}", bus, f"{angle:.2f}", status, name,
                        bytes(data).hex(), args.note,
                    ])

                # console at ~5 Hz so the terminal stays readable at 100 Hz
                if now - last_print >= 0.2:
                    last_print = now
                    print(f"  {angle:+8.2f} deg   bus {bus}   "
                          f"status {status} ({name})")

            time.sleep(0.005)

    except KeyboardInterrupt:
        print()
        print("stopped. Nothing was transmitted.")

    finally:
        if fh:
            fh.flush()
            fh.close()

    print()
    print(f"samples: {samples} over {time.monotonic() - t0:.1f}s")
    if angle_min is not None:
        print(f"angle range: {angle_min:+.2f} to {angle_max:+.2f} deg "
              f"(span {angle_max - angle_min:.2f})")
    print()
    print("LKAS_STATUS values seen:")
    for status, count in sorted(status_counts.items()):
        name = LKAS_STATUS_NAMES.get(status, "UNKNOWN")
        at, angle = first_seen[status]
        print(f"  {status:3d} ({name:11s})  {count:7d} frames   "
              f"first at {at:.1f}s, {angle:+.2f} deg")
    unknown = [s for s in status_counts if s not in LKAS_STATUS_NAMES]
    if unknown:
        print()
        print(f"undocumented status values: {unknown}")
        print("write down what the car was doing when each appeared.")


if __name__ == "__main__":
    main()