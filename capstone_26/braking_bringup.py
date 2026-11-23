"""Nissan Leaf 2025 capstone brake diagnostics (master branch companion).

Receive-only CAN monitor and offline request-state simulation.
No CAN transmit, Panda safety changes, relay control, or brake actuation.

Examples:
    python braking_bringup.py --simulate --pressure 120 --seconds 4
    python braking_bringup.py --watch --seconds 15
    python braking_bringup.py --watch --seconds 20 --csv brake_capture.csv
"""

import argparse
import csv
import time
from collections import Counter

# Observed in capstone_26/nissan_leaf_2025_capstone.dbc (master).
# Names imply possible functions, not verified command semantics.
OBSERVED_IDS = {
    0x1C3: "PROPILOT_BRAKE", 0x1CB: "OBS_BRAKE_ASSOC_1CB",
    0x1CC: "BRAKE_PEDAL", 0x28E: "OBS_AUTO_BRAKE_ASSOC_28E",
    0x2AA: "OBS_BRAKE_ASSOC_2AA", 0x2B0: "OBS_BRAKE_ASSOC_2B0",
    0x2F9: "OBS_AUTO_BRAKE_ASSOC_2F9",
}


def unpack(msg):
    """Normalize Panda's 3- or 4-element receive tuples."""
    if len(msg) == 3:
        addr, data, src = msg
    elif len(msg) == 4:
        addr, _bus_time, data, src = msg
    else:
        raise ValueError(f"unexpected Panda CAN tuple length: {len(msg)}")
    return int(addr), bytes(data), int(src) & 0x0F


def report_frame(addr, payload, bus):
    """Raw data only; interpretations need independent validation."""
    name = OBSERVED_IDS[addr]
    return f"bus {bus}  ID 0x{addr:03X}  {name:<24} [{len(payload)}]  {payload.hex(' ').upper()}"


def simulation(pressure, seconds):
    """Offline state-machine demonstration; no CAN bytes are generated."""
    if not 0 <= pressure <= 511:
        raise ValueError("pressure must be between 0 and 511 (raw DBC range)")
    if not 0 < seconds <= 30:
        raise ValueError("seconds must be greater than zero and at most 30")
    print("OFFLINE SIMULATION ONLY: no hardware and no CAN transmission")
    print("The pressure input is an arbitrary RAW field value, not psi/bar.")
    print(f"requested raw value: {pressure}; requested hold: {seconds:.1f} s")
    print("phase 1: simulated inactive")
    print(f"phase 2: simulated active, raw magnitude={pressure}")
    print("phase 3: simulated release, raw magnitude=0")
    print("No assertion is made that these states would actuate a real vehicle.")


def watch(seconds, csv_path):
    """Observe existing traffic without calling can_send or changing safety mode."""
    from manual_can_interface import CANInterface

    can = CANInterface(simulation=False, allow_output=False)
    if can.panda is None:
        raise RuntimeError("Panda not available")

    counts = Counter()
    stream = open(csv_path, "w", newline="", encoding="utf-8") if csv_path else None
    try:
        writer = csv.writer(stream) if stream else None
        if writer:
            writer.writerow(["elapsed_s", "bus", "can_id", "name", "dlc", "data_hex"])
        print("RECEIVE-ONLY: relay unchanged; no CAN transmission")
        print(f"monitoring for {seconds:.1f} s; Ctrl+C to stop")
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            messages = can.panda.can_recv()
            if not messages:
                time.sleep(0.01)
                continue
            for msg in messages:
                addr, payload, bus = unpack(msg)
                if addr not in OBSERVED_IDS:
                    continue
                counts[(addr, bus)] += 1
                elapsed = time.monotonic() - start
                print(report_frame(addr, payload, bus))
                if writer:
                    writer.writerow([f"{elapsed:.6f}", bus, f"0x{addr:03X}",
                                     OBSERVED_IDS[addr], len(payload), payload.hex().upper()])
    except KeyboardInterrupt:
        print("\nmonitor stopped")
    finally:
        if stream:
            stream.close()
        print("\nObserved frame counts:")
        for (addr, bus), count in sorted(counts.items()):
            print(f"  0x{addr:03X} bus {bus}: {count}")
        print("No messages were transmitted by this script.")


def main():
    parser = argparse.ArgumentParser(description="Leaf braking observation and offline simulation")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--watch", action="store_true", help="read-only Panda capture")
    mode.add_argument("--simulate", action="store_true", help="offline braking state simulation")
    parser.add_argument("--pressure", type=int, default=100,
                        help="offline-only raw magnitude 0..511 (default 100)")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--csv", help="optional output path for receive-only captures")
    args = parser.parse_args()
    if not 0 < args.seconds <= 30:
        parser.error("--seconds must be between 0 and 30")
    if args.watch:
        if args.csv is not None and not args.csv.lower().endswith(".csv"):
            parser.error("--csv path must end in .csv")
        watch(args.seconds, args.csv)
    else:
        if args.csv:
            parser.error("--csv is for --watch only")
        simulation(args.pressure, args.seconds)


if __name__ == "__main__":
    main()
