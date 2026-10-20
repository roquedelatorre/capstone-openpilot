"""Dump the car's ECU FW versions and emit a FW_VERSIONS block.

Step 8 of the port notes: get NISSAN_LEAF_2025 to auto-identify so we can
drop FINGERPRINT=NISSAN_LEAF_2025 SKIP_FW_QUERY=1 from the launch line.

Run on the device, with openpilot running and CarParams set.

TWO TERMINALS. The stack must stay up - card sets CarParams tens of seconds
after launch, so Ctrl-C'ing the launch script gets you nothing.

  terminal 1:  cd /data/openpilot
               FINGERPRINT=NISSAN_LEAF_2025 ./launch_openpilot.sh
               (leave it running)

  terminal 2:  python capstone_26/fw_dump.py --wait

--wait polls until CarParams shows up, so you can start it first and then
launch. Car ignition ON, and stop any capstone_26 script first - they hold
the panda exclusively and the FW query needs it.

    python fw_dump.py
    python fw_dump.py --wait
    python fw_dump.py --wait --out fw_block.py

IMPORTANT - the FW query has to actually have happened for this to produce
anything. SKIP_FW_QUERY=1 skips it and leaves carFw empty, which is why the
launch line above omits it.

Paste the emitted block into opendbc/car/nissan/fingerprints.py and merge it
into the existing FW_VERSIONS dict. Then relaunch with NO env vars at all and
confirm CarParams still reports NISSAN_LEAF_2025.
"""

import argparse
import os
import subprocess
import time
from collections import defaultdict

from cereal import car
from openpilot.common.params import Params

PLATFORM = "NISSAN_LEAF_2025"


def first_attr(obj, names, default="<unavailable>"):
    """capnp raises AttributeError on a missing member, and field names have
    moved between versions (carName -> brand). Try each name in turn."""
    for name in names:
        try:
            return getattr(obj, name)
        except AttributeError:
            continue
    return default


def param_age():
    """When was CarParams last written, and is a stack running now?

    CarParams persists after openpilot exits, so a run launched with
    SKIP_FW_QUERY=1 leaves behind a CarParams with an empty carFw that
    looks exactly like a failed FW query. Return (age_seconds, path,
    card_running) - any may be None if it can't be determined.
    """
    age = path = None
    for d in ("/dev/shm/params/d", "/data/params/d"):
        candidate = os.path.join(d, "CarParams")
        if os.path.exists(candidate):
            path = candidate
            age = time.time() - os.path.getmtime(candidate)
            break

    card_running = None
    try:
        out = subprocess.run(["pgrep", "-af", "card"], capture_output=True,
                             text=True, timeout=5).stdout
        card_running = any("card" in line and "pgrep" not in line
                           for line in out.splitlines())
    except Exception:
        pass

    return age, path, card_running


def build_block(fw, platform):
    """Group carFw entries and format them as a FW_VERSIONS block.

    Pure function - no capnp, no Params - so it can be unit tested off-car.
    Returns (block_text_or_None, skipped_logging_count).
    """
    # group by (ecu, address, subAddress) exactly as fingerprints.py keys it
    grouped = defaultdict(list)
    skipped_logging = 0
    for f in fw:
        # Nissan queries bus 0 with logging=False and bus 1 with
        # logging=True. Only non-logging entries are used for matching.
        if first_attr(f, ("logging",), False):
            skipped_logging += 1
            continue
        ecu = first_attr(f, ("ecu",), "unknown")
        address = first_attr(f, ("address",), 0)
        sub = first_attr(f, ("subAddress",), None)
        version = first_attr(f, ("fwVersion",), b"")
        grouped[(str(ecu), address, sub)].append(bytes(version))

    if not grouped:
        return None, skipped_logging

    lines = [f"  CAR.{platform}: {{"]
    # sort by address, then ecu name - stable and matches how the existing
    # fingerprints.py blocks read
    for (ecu, address, sub) in sorted(grouped, key=lambda k: (k[1], k[0])):
        versions = grouped[(ecu, address, sub)]
        sub_repr = "None" if not sub else hex(sub)
        lines.append(f"    (Ecu.{ecu}, {hex(address)}, {sub_repr}): [")
        for v in sorted(set(versions)):
            lines.append(f"      {v!r},")
        lines.append("    ],")
    lines.append("  },")
    return "\n".join(lines), skipped_logging


def main():
    p = argparse.ArgumentParser(description="dump carFw as a FW_VERSIONS block")
    p.add_argument("--out", metavar="PATH", default=None,
                   help="also write the block to this file")
    p.add_argument("--platform", default=PLATFORM)
    p.add_argument("--wait", type=float, nargs="?", const=180.0, default=None,
                   metavar="SEC",
                   help="poll until CarParams appears, up to SEC seconds "
                        "(default 180). Start this FIRST, then launch "
                        "openpilot in another terminal.")
    args = p.parse_args()

    params = Params()
    raw = params.get("CarParams", block=False)

    if raw is None and args.wait:
        print(f"waiting up to {args.wait:.0f}s for CarParams ...")
        print("(launch openpilot in another terminal now, and LEAVE IT UP -")
        print(" Ctrl-C kills the stack before it fingerprints)")
        deadline = time.monotonic() + args.wait
        while raw is None and time.monotonic() < deadline:
            time.sleep(1.0)
            raw = params.get("CarParams", block=False)
            left = deadline - time.monotonic()
            print(f"\r  {left:5.0f}s left ...", end="", flush=True)
        print()

    if raw is None:
        print("CarParams is not set.")
        print()
        print("Most likely the stack was not up long enough. card runs the FW")
        print("query and sets CarParams tens of seconds after launch - Ctrl-C")
        print("before then and nothing is ever written.")
        print()
        print("Leave openpilot running in one terminal:")
        print("  FINGERPRINT=NISSAN_LEAF_2025 ./launch_openpilot.sh")
        print("and run this in a second terminal.")
        print()
        print("Other things to check:")
        print("  - car ignition ON (the FW query needs the ECUs awake)")
        print("  - nothing else holding the panda (stop the capstone_26")
        print("    scripts first - they take exclusive USB)")
        print("  - card did not die on startup: override.toml must still")
        print("    have the NISSAN_LEAF_2025 key or get_std_params raises")
        print("    KeyError and CarParams is never set")
        return

    # CarParams.from_bytes is a context manager, not a plain object
    with car.CarParams.from_bytes(raw) as cp:
        platform = cp.carFingerprint
        fw = list(cp.carFw)

        print(f"carFingerprint: {platform}")
        print(f"brand:          {first_attr(cp, ('brand', 'carName'))}")
        print(f"carFw entries:  {len(fw)}")
        print()

        if not fw:
            age, path, card_running = param_age()

            print("carFw is EMPTY.")
            print()
            if path:
                print(f"CarParams at {path}")
                if age is not None:
                    mins = age / 60.0
                    print(f"last written {mins:.1f} min ago "
                          f"({'just now' if age < 120 else 'STALE?'})")
            if card_running is False:
                print()
                print(">> No card process is running. CarParams PERSISTS after")
                print("   openpilot exits, so this is almost certainly a")
                print("   LEFTOVER from an earlier SKIP_FW_QUERY=1 run - not")
                print("   a failed query. Launch openpilot, leave it up, and")
                print("   run this again while it is still running.")
                return
            print()
            print("If the stack really is running and the query really ran:")
            print("  - car ignition ON (asleep ECUs do not answer)")
            print("  - SKIP_FW_QUERY must be UNSET. Note FINGERPRINT alone")
            print("    does NOT skip the query - it is applied after it.")
            print("  - clear the stale param and relaunch so nothing cached")
            print("    is reused:  python -c \"from openpilot.common.params"
                  " import Params; Params().remove('CarParams')\"")
            return

        block, skipped_logging = build_block(fw, args.platform)

        if skipped_logging:
            print(f"(skipped {skipped_logging} logging-only entries - bus 1, "
                  f"not used for matching)")
            print()

        if block is None:
            print("every entry was logging-only. Nothing to key a FW_VERSIONS")
            print("block on. The bus 0 query got no responses.")
            return

        print("paste into opendbc/car/nissan/fingerprints.py, inside FW_VERSIONS:")
        print()
        print(block)
        print()

        if platform != args.platform:
            print(f"NOTE: the running platform is {platform}, not "
                  f"{args.platform}.")
            print("These FW versions came from the car either way, but make")
            print("sure you key the block under the platform you intend.")

        print("Also check: does this FW set collide with the existing")
        print("NISSAN_LEAF block? Our notes say the 2025 FUZZY-matches")
        print("NISSAN_LEAF. If an exact-match entry here is also present")
        print("under NISSAN_LEAF, the match stays ambiguous. Relaunch with")
        print("no env vars and confirm CarParams reports the 2025.")

    if args.out:
        with open(args.out, "w") as f:
            f.write(block + "\n")
        print()
        print(f"written to {args.out}")


if __name__ == "__main__":
    main()