#!/usr/bin/env python3
"""
can_diff.py - record CAN traffic under two conditions and diff them.

The workflow for finding which message carries a given function:

    python capstone_26/can_diff.py record idle      --seconds 5
    # now hold the stalk / press the button / open the door
    python capstone_26/can_diff.py record left_held --seconds 5
    python capstone_26/can_diff.py diff idle left_held

The diff prints every (bus, addr) whose payload bytes differ between the two
recordings, and for each one which byte positions changed. Counter and
checksum bytes will show up as "always changing" in both recordings and are
filtered out by default, so what's left is signal.

Requires openpilot to be stopped (pandad holds the panda otherwise):
    sudo systemctl stop comma
"""

import argparse
import json
import os
import time
from collections import defaultdict

STORE = "/tmp/can_diff"


def recordings_path(label):
    return os.path.join(STORE, f"{label}.json")


def do_record(label, seconds, buses):
    from panda import Panda

    os.makedirs(STORE, exist_ok=True)
    p = Panda()

    # (bus, addr) -> {hex payload: count}
    seen = defaultdict(lambda: defaultdict(int))

    print(f"recording '{label}' for {seconds:.1f}s ...")
    t = time.time()
    frames = 0
    while time.time() - t < seconds:
        for m in p.can_recv():
            addr, dat, bus = m[0], m[-2], m[-1]
            if bus >= 128:          # panda's echo of its own transmits
                continue
            if buses and bus not in buses:
                continue
            seen[(bus, addr)][bytes(dat).hex()] += 1
            frames += 1

    out = {f"{bus}:{addr}": dict(payloads) for (bus, addr), payloads in seen.items()}
    with open(recordings_path(label), "w") as f:
        json.dump(out, f)

    print(f"{frames} frames, {len(out)} distinct (bus, id) -> {recordings_path(label)}")


def load(label):
    path = recordings_path(label)
    if not os.path.exists(path):
        raise SystemExit(f"no recording named '{label}' (looked in {path})")
    with open(path) as f:
        return json.load(f)


def changing_bytes(payloads):
    """Byte positions that vary within a single recording (counters, checksums)."""
    blobs = [bytes.fromhex(h) for h in payloads]
    if not blobs:
        return set()
    width = max(len(b) for b in blobs)
    varying = set()
    for i in range(width):
        vals = {b[i] for b in blobs if len(b) > i}
        if len(vals) > 1:
            varying.add(i)
    return varying


def do_diff(label_a, label_b, show_noisy):
    a, b = load(label_a), load(label_b)

    keys = sorted(set(a) | set(b), key=lambda k: (int(k.split(":")[0]), int(k.split(":")[1])))
    hits = 0

    for key in keys:
        bus, addr = (int(x) for x in key.split(":"))
        pa, pb = a.get(key, {}), b.get(key, {})

        if not pa:
            print(f"bus {bus}  0x{addr:03X}  ONLY IN {label_b}")
            hits += 1
            continue
        if not pb:
            print(f"bus {bus}  0x{addr:03X}  ONLY IN {label_a}")
            hits += 1
            continue

        if set(pa) == set(pb):
            continue

        # which byte positions differ between the two recordings, ignoring the
        # positions that were already churning inside each one
        noisy = changing_bytes(pa) | changing_bytes(pb)
        blobs_a = [bytes.fromhex(h) for h in pa]
        blobs_b = [bytes.fromhex(h) for h in pb]
        width = max(len(x) for x in blobs_a + blobs_b)

        interesting = []
        for i in range(width):
            va = {x[i] for x in blobs_a if len(x) > i}
            vb = {x[i] for x in blobs_b if len(x) > i}
            if va != vb and (show_noisy or i not in noisy):
                interesting.append((i, sorted(va), sorted(vb)))

        if not interesting:
            continue

        hits += 1
        print(f"bus {bus}  0x{addr:03X}")
        for i, va, vb in interesting:
            fa = " ".join(f"{v:02X}" for v in va[:6]) + ("..." if len(va) > 6 else "")
            fb = " ".join(f"{v:02X}" for v in vb[:6]) + ("..." if len(vb) > 6 else "")
            print(f"    byte {i}:  {label_a}: {fa}   ->   {label_b}: {fb}")

    if not hits:
        print("no differences (try a longer recording, or check the condition actually changed)")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    rec = sub.add_parser("record", help="record traffic under one condition")
    rec.add_argument("label")
    rec.add_argument("--seconds", type=float, default=5.0)
    rec.add_argument("--buses", default="", help="comma-separated, e.g. 0,2 (default: all)")

    dif = sub.add_parser("diff", help="compare two recordings")
    dif.add_argument("label_a")
    dif.add_argument("label_b")
    dif.add_argument("--show-noisy", action="store_true",
                     help="also show byte positions that churn on their own "
                          "(counters, checksums)")

    args = ap.parse_args()

    if args.cmd == "record":
        buses = {int(x) for x in args.buses.split(",") if x.strip() != ""}
        do_record(args.label, args.seconds, buses)
    else:
        do_diff(args.label_a, args.label_b, args.show_noisy)


if __name__ == "__main__":
    main()