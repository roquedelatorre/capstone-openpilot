"""Steering test that impersonates the camera, the way openpilot does.

WHY THIS EXISTS
---------------
Our earlier attempts sent 0x169 and nothing else. opendbc's nissan
carcontroller.py sends FOUR messages every control cycle for a Leaf:

    LKAS                  0x169   100 Hz   built from scratch
    CANCEL_MSG            0x280    50 Hz   copy-and-modify of the stock msg
    PROPILOT_HUD          0x2B1    50 Hz   copy-and-modify of the stock msg
    PROPILOT_HUD_INFO_MSG 0x4CC     2 Hz   copy-and-modify of the stock msg

The copy-and-modify part is the point. openpilot reads the camera's own
messages and retransmits altered versions, so when the intercept relay opens
and the camera is cut off, the rest of the ADAS conversation keeps going and
only the steering command is substituted.

When we opened the relay and sent only 0x169, the entire ADAS system vanished
from the EPS's point of view and a bare steering command arrived from nowhere.
LKAS_STATUS 6 told us the frame parsed. It did not tell us the system was in a
state to act on it.

This script does it properly:

  phase 0  verify LKAS_STATUS reads 5 (INACTIVE) - abort if not
  phase 1  relay CLOSED, listen, latch the camera's 0x280 / 0x2B1 / 0x4CC
  phase 2  relay OPEN, stream our 0x169 PLUS the latched camera messages
  phase 3  release, close relay, always, including on Ctrl+C

LIMITATION: phase 2 replays the latched camera frames VERBATIM. openpilot
modifies a few HUD fields; we do not. For a stationary bring-up that should be
enough to keep the EPS's context alive, but it means the HUD content is frozen
at whatever it was during phase 1. We have ALSO observed that the payloads are
not static between runs - PROPILOT_HUD went 01 00 54 00 00 10 1E 00 on one run
to 01 00 FF 00 00 00 1E 00 on the next - so a replayed frame may carry a
counter or checksum we have not identified. Watch the status output.

Car in park. Wheels chocked. Hands off the wheel.

    python steering_impersonate.py --listen 5 --delta 10 --seconds 15
    python steering_impersonate.py --listen 5 --dry-run    # no TX, no relay
"""

import argparse
import functools
import threading
import time
import csv

from manual_can_interface import CANInterface
from manual_nissancan import CarController_Manual, LKAS_BUS
from manual_tx_loop import PeriodicTransmitter, SteeringHold

LKAS_RATE_HZ = 100.0

# What openpilot sends alongside 0x169, how often, and ON WHICH BUS.
#
# FIXED - we used to replay each message on whatever bus we observed it on.
# That is wrong. The bus a message ORIGINATES on is not the bus openpilot
# TRANSMITS it on. These bus numbers are read straight out of nissancan.py:
#
#   create_cancel_msg         -> make_can_msg("CANCEL_MSG", 2, ...)
#   create_lkas_hud_msg       -> make_can_msg("PROPILOT_HUD", 0, ...)
#   create_lkas_hud_info_msg  -> make_can_msg("PROPILOT_HUD_INFO_MSG", 0, ...)
#
# and they match NISSAN_TX_MSGS in opendbc/safety/modes/nissan.h.
#
# The camera emits 0x2B1 and 0x4CC on bus 2 (camera side). openpilot sends its
# replacements on bus 0, because that is the side the EPS listens on. With the
# relay CLOSED the two segments are bridged, so replaying on the observed bus
# appeared to work. With the relay OPEN it does not - the frames land on the
# isolated camera segment where nothing is listening.
#
# addr -> (name, rate_hz, tx_bus)

def log_can_frame(writer, addr, data, bus):
    writer.writerow([
        time.monotonic(),
        bus,
        f"0x{addr:03X}",
        bytes(data).hex(" ").upper()
    ])

CAMERA_MESSAGES = {
    0x280: ("CANCEL_MSG", 50.0, 2),
    0x2B1: ("PROPILOT_HUD", 50.0, 0),
    0x4CC: ("PROPILOT_HUD_INFO_MSG", 2.0, 0),
}

LKAS_STATUS_NAMES = {
    5: "Observed 5",
    6: "Observed 6",
    9: "Observed 9",
    14: "Observed 14"
}
# LKAS_STATUS must read this at rest for a run to mean anything.
LKAS_STATUS_READY = 5


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


def unpack(msg):
    """panda returns (addr, data, bus) or (addr, busTime, data, src).
    The bus byte carries flag bits in the top nibble - that is why a scan
    reports buses 128 / 130 / 192 / 194. Mask them off."""
    if len(msg) == 3:
        addr, data, bus = msg[0], msg[1], msg[2]
    else:
        addr, data, bus = msg[0], msg[2], msg[3]
    return addr, bytes(data), bus & 0x0F


class CameraEcho:
    """Latches the most recent copy of each camera message, then replays it.

    Latching happens with the relay CLOSED, while the camera is still on the
    bus. Once the relay opens the camera is gone and these copies are the only
    thing keeping the EPS's ADAS context alive.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.latched = {}        # addr -> (bytes, observed_bus)
        self.seen_counts = {}    # addr -> how many we observed

    def observe(self, addr, data, bus):
        if addr not in CAMERA_MESSAGES:
            return
        with self.lock:
            self.latched[addr] = (data, bus)
            self.seen_counts[addr] = self.seen_counts.get(addr, 0) + 1

    def builder(self, addr):
        """Zero-arg callable for PeriodicTransmitter.

        CHANGED - transmits on the bus openpilot uses, taken from
        CAMERA_MESSAGES, NOT the bus we happened to observe the frame on.
        """
        tx_bus = CAMERA_MESSAGES[addr][2]

        def build():
            with self.lock:
                entry = self.latched.get(addr)
            if entry is None:
                return None
            data, _observed_bus = entry
            return (addr, data, tx_bus)
        return build

    def report(self):
        with self.lock:
            latched = dict(self.latched)
            counts = dict(self.seen_counts)

        print()
        print("camera messages latched during phase 1:")
        missing = []
        for addr, (name, rate, tx_bus) in sorted(CAMERA_MESSAGES.items()):
            if addr in latched:
                data, observed_bus = latched[addr]
                hexed = " ".join(f"{b:02X}" for b in data)
                # show both buses - if they differ, the relay was closed and
                # bridging them, which is expected during phase 1
                print(f"  0x{addr:03X} {name:<22} seen on bus {observed_bus} "
                      f"-> will TX on bus {tx_bus}  "
                      f"{counts.get(addr, 0):>4} seen   {hexed}")
            else:
                print(f"  0x{addr:03X} {name:<22} NOT SEEN")
                missing.append(name)
        return missing


def main():
    p = argparse.ArgumentParser(description="camera-impersonating steering test")
    p.add_argument("--listen", type=float, default=5.0, metavar="SEC",
                   help="phase 1 duration, relay closed, latching (default 5)")
    p.add_argument("--delta", type=float, default=10.0, metavar="DEG",
                   help="degrees relative to current wheel angle (default 10)")
    p.add_argument("--seconds", type=float, default=15.0, metavar="SEC",
                   help="phase 2 hold duration (default 15)")
    p.add_argument("--torque", type=float, default=1.0, metavar="NM",
                   help="MAX_TORQUE. opendbc's LKAS_MAX_TORQUE is 1 and its "
                        "comment calls that 'easy to overpower' - 1.0 is the "
                        "value openpilot actually ships (default 1.0)")
    p.add_argument(
                    "--dry-run",
                    action="store_true",
                    default=True,)

    p.add_argument("--no-echo", action="store_true",
                   help="open the relay but send ONLY 0x169, reproducing our "
                        "earlier failed attempt. For comparison.")
    p.add_argument("--ignore-status", action="store_true",
                   help="run even if LKAS_STATUS is not 5 at rest. Only for "
                        "deliberately testing what a latched fault does.")
    p.add_argument("--dbc", default=None)
    args = p.parse_args()

    if abs(args.delta) > 30:
        p.error("keep it small - 30 deg is plenty")
    if args.seconds > 30:
        p.error("30 s is plenty")
    if not 0.0 <= args.torque <= 2.5:
        p.error("MAX_TORQUE must be 0 to 2.5")

    can = CANInterface(
        simulation=False,
        allow_output=False
    )

    panda = can.panda

    echo = CameraEcho()

    # ---- phase 1: relay closed, listen and latch -------------------------
    print()
    print(f"phase 1: listening {args.listen:.1f} s with the relay CLOSED.")
    print("nothing is transmitted during this phase.")

    start_angle = None
    status = None
    end = time.monotonic() + args.listen

    #while time.monotonic() < end:
    #    for msg in panda.can_recv():
    #        addr, data, bus = unpack(msg)
    #        echo.observe(addr, data, bus)
    #        if addr == 0x185:
    #            decoded = decode_185(data)
    #            if decoded is not None:
    #                start_angle, status = decoded

    with open("steering_diagnostics.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Bus", "CAN_ID", "Data"])

        while time.monotonic() < end:
            for msg in panda.can_recv():
                addr, data, bus = unpack(msg)

                log_can_frame(writer, addr, data, bus)
                echo.observe(addr, data, bus)

                if addr == 0x185:
                    decoded = decode_185(data)
                    if decoded is not None:
                        start_angle, status = decoded

    missing = echo.report()

    if start_angle is None:
        print()
        print("no 0x185 received - cannot seed the ramp. Is the car awake?")
        print("Is openpilot holding the panda? Nothing was transmitted.")
        return

    print()
    print(f"wheel is at {start_angle:+.2f} deg")
    print(f"LKAS_STATUS = {status} ({LKAS_STATUS_NAMES.get(status, 'unknown')})")

    # ---- phase 0 check, done here because we need a 0x185 reading first --
    # NEW - a latched LKAS_STATUS 14 invalidates the whole run. We have seen
    # it read 14 BEFORE transmitting, left over from an earlier attempt. The
    # EPS will not act from that state, so the test tells us nothing. An
    # ignition cycle clears it.
    if status != LKAS_STATUS_READY and not args.ignore_status:
        print()
        print(f"!! LKAS_STATUS is {status}, expected {LKAS_STATUS_READY} "
              f"(INACTIVE) at rest.")
        print("   The system is already in a state it will not act from, so")
        print("   this run would prove nothing. This is usually a fault")
        print("   latched by a previous attempt.")
        print()
        print("   Ignition cycle the car, then run this again.")
        print("   To override anyway: --ignore-status")
        print()
        print("nothing was transmitted, relay untouched.")
        return

    if missing and not args.no_echo:
        print()
        print(f"WARNING: did not see {', '.join(missing)}.")
        print("Those will not be replayed, so the EPS will still be missing")
        print("part of its ADAS context. Try a longer --listen, or check that")
        print("the camera is actually transmitting them.")

    if args.dry_run:
        print()
        print("dry run complete. Nothing was transmitted, relay untouched.")
        return

    target = start_angle + args.delta

    print()
    print("phase 2:")
    print(f"  relay will OPEN (camera isolated, ProPILOT off)")
    print(f"  0x169 at {LKAS_RATE_HZ:.0f} Hz on bus {LKAS_BUS}, "
          f"{start_angle:+.2f} -> {target:+.2f} deg")
    print(f"  MAX_TORQUE {args.torque:.2f}")
    if args.no_echo:
        print("  --no-echo: camera messages will NOT be replayed")
    else:
        for addr, (name, rate, tx_bus) in sorted(CAMERA_MESSAGES.items()):
            state = "replaying" if addr in echo.latched else "NOT LATCHED"
            print(f"  0x{addr:03X} {name:<22} {rate:>5.0f} Hz  "
                  f"bus {tx_bus}   {state}")
    print(f"  holding {args.seconds:.1f} s")
    print()
    print("CAR IN PARK. WHEELS CHOCKED. HANDS OFF THE WHEEL.")
    if input("type GO to continue: ").strip() != "GO":
        raise SystemExit("aborted")

    nissan = CarController_Manual(args.dbc)
    nissan.create_steering_control = functools.partial(
        nissan.create_steering_control, max_torque=args.torque)

    steering = SteeringHold(nissan, rate_hz=LKAS_RATE_HZ, timeout_s=0)
    steering.current = start_angle
    steering.target = start_angle

    tx = PeriodicTransmitter(can, base_rate_hz=LKAS_RATE_HZ)

    relay_opened = False
    try:
        can.set_relay(True)
        relay_opened = True
        time.sleep(0.2)

        # register the camera echoes FIRST so the ADAS conversation never
        # has a gap, then the steering command
        if not args.no_echo:
            for addr, (name, rate, tx_bus) in CAMERA_MESSAGES.items():
                if addr in echo.latched:
                    tx.set(name, echo.builder(addr), rate_hz=rate)

        tx.set("LKAS", steering.build, rate_hz=LKAS_RATE_HZ)
        tx.start()

        # let the stream settle at the current angle before asking for motion
        time.sleep(0.8)

        steering.engage(target)
        print(f"engaged, ramping at {steering.MAX_RATE_DEG_S:.0f} deg/s")
        print()

        end = time.monotonic() + args.seconds
        faulted = False

        while time.monotonic() < end:
            reading = None
            deadline = time.monotonic() + 0.25
            while time.monotonic() < deadline:
                for msg in panda.can_recv():
                    addr, data, _ = unpack(msg)
                    if addr == 0x185:
                        decoded = decode_185(data)
                        if decoded is not None:
                            reading = decoded

            if reading:
                angle, status = reading
                name = LKAS_STATUS_NAMES.get(status, "?")
                print(f"  commanded {steering.current:+8.2f}   "
                      f"measured {angle:+8.2f}   status {status} ({name})")

                if status == 9:
                    print("Steering status changed to 9 (meaning unverified). Stopping.")
                    faulted = True
                    break

                # NEW - 14 is the latching state we keep landing in. Stop as
                # soon as we see it rather than streaming into a dead system
                # for the rest of the hold.
                if status == 14:
                    print()
                    print("!! LKAS_STATUS went to 14. The EPS has dropped out")
                    print("   of a state it will act from. Stopping - further")
                    print("   streaming proves nothing. Ignition cycle before")
                    print("   the next run.")
                    faulted = True
                    break

        if not faulted:
            print()
            print("hold finished.")

    except KeyboardInterrupt:
        print("\ninterrupted")

    finally:
        steering.release()
        time.sleep(0.3)
        tx.stop()
        if relay_opened:
            can.set_relay(False)
        print()
        tx.notices.print_all()
        tx.print_stats()
        print()
        can.print_summary()


if __name__ == "__main__":
    main()