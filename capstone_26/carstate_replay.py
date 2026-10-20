#!/usr/bin/env python3
"""
carstate_replay.py - validate Nissan Leaf CarState decoding on a logged route.

Replays logged CAN from a route through openpilot's Nissan CarInterface
(forced to NISSAN_LEAF) and prints the decoded CarState once per second.

READ-ONLY: it decodes logged data only. It does not connect to the panda,
does not transmit, and does not control anything. Safe to run anywhere the
openpilot libraries are importable.

This is the offline de-risking step: if vEgo, steering angle, gear, and
blinkers decode sanely from the 2025's real logged data through the existing
Leaf port, then CarState works and only the TX side remains.

Run on the comma device, from /data/openpilot:

    python carstate_replay.py /data/media/0/realdata/00000020--6df23dc6a8--10

Pick a segment where you were actually driving (not 'boot' or --0), so there
is motion to check. Pass either the segment folder or its rlog file.
"""
import os
import sys
from collections import defaultdict

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.logreader import LogReader

PLATFORM = "NISSAN_LEAF"   # force the existing port; sidesteps FW fingerprinting


def resolve(path):
    # accept a segment dir or a direct rlog file
    if os.path.isdir(path):
        for name in ("rlog.zst", "rlog.bz2", "rlog"):
            cand = os.path.join(path, name)
            if os.path.exists(cand):
                return cand
        raise SystemExit(f"no rlog found in {path}")
    return path


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    path = resolve(sys.argv[1])

    lr = LogReader(path, only_union_types=True, sort_by_time=True)
    can_msgs = [m for m in lr if m.which() == "can"]
    if not can_msgs:
        print("No CAN messages in", path)
        return
    print(f"{len(can_msgs)} CAN frames loaded from {path}")

    # build a message fingerprint per bus (addr -> length), buses 0..2
    fp = defaultdict(dict)
    for m in can_msgs:
        for c in m.can:
            if c.src < 3:
                fp[c.src][c.address] = len(c.dat)
    fingerprint = {b: dict(fp.get(b, {})) for b in range(3)}

    CarInterface = interfaces[PLATFORM]
    CP = CarInterface.get_params(PLATFORM, fingerprint, [], False, False, False)
    CI = CarInterface(CP)
    CC = structs.CarControl().as_reader()

    print(f"\nforced platform: {PLATFORM}   steerControlType: {CP.steerControlType}")
    print(f"{'t(s)':>6} {'vEgo':>6} {'angle':>8} {'drvTrq':>7} {'gear':>7} "
          f"{'L':>2} {'R':>2} {'stop':>4} {'belt':>4} {'lkasFlt':>7}")

    t0 = can_msgs[0].logMonoTime
    last = -1.0
    for msg in can_msgs:
        frames = [CanData(c.address, c.dat, c.src) for c in msg.can]
        CS = CI.update([(msg.logMonoTime, frames)])
        CI.apply(CC, msg.logMonoTime)

        t = (msg.logMonoTime - t0) / 1e9
        if t - last >= 1.0:
            last = t
            print(f"{t:6.1f} {CS.vEgo:6.1f} {CS.steeringAngleDeg:8.1f} "
                  f"{CS.steeringTorque:7.1f} {str(CS.gearShifter):>7} "
                  f"{int(CS.leftBlinker):>2} {int(CS.rightBlinker):>2} "
                  f"{int(CS.standstill):>4} {int(CS.seatbeltUnlatched):>4} "
                  f"{int(CS.steerFaultTemporary):>7}")

    print("\ndone. Sanity checks:")
    print(" - vEgo should track your actual speed and be ~0 at standstill")
    print(" - angle should swing both signs through turns, ~0 straight")
    print(" - gear should read your actual gear (D/R/P/N/B), not garbage")
    print(" - L/R blinkers should flip when you signalled")
    print(" If those hold, CarState decoding is good on the 2025.")


if __name__ == "__main__":
    main()