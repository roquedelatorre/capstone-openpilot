"""Steering test with the camera ISOLATED - the configuration openpilot uses.

WHY THIS EXISTS
---------------
steer_relay_closed.py ran clean on 2026-10-28: 99.7 Hz, 1313 frames, relay
verifiably closed, LKAS_STATUS 6, wheel did not move. But that run also
printed:

    traffic by bus: bus 0: 14308  bus 1: 2014  bus 2: 14307
    bus balance 1.00 - relay is closed

A balance of 1.00 means the panda is bridging bus 0 and bus 2 - they are
electrically the same wire. So the bus 2 run and the bus 0 run were the same
experiment twice. The bus number cannot matter until the relay opens.

The second thing that run showed: status dipped 6 -> 5 three times during the
hold, which never happened on the earlier run. With the relay closed the real
camera is still putting its own 0x169 on the bus at ~94 Hz with LKA_ACTIVE
clear. The EPS is being handed two contradictory commands for one message ID
at ~200 Hz combined, half of them saying inactive. That is not a situation the
EPS was built for, and "no motion" is the least surprising outcome of it.

openpilot never does this. It OPENS the intercept relay, which cuts the camera
off the car-side bus, and then it is the only source of 0x169. That is the one
variable we have never changed.

WHAT THIS DOES DIFFERENTLY
--------------------------
  - opens the intercept relay instead of forcing it closed
  - proves the relay opened, by requiring the bus counts to go LOPSIDED
    (the opposite of the closed-relay check) and by requiring 0x169 to
    disappear from the car-side bus
  - then transmits 0x169 as the only source

WHAT TO EXPECT
--------------
  1. the wheel moves            -> steering works; the camera was winning
                                   arbitration all along
  2. LKAS_STATUS stops updating -> 0x185 lives on the camera side of the
                                   relay, and our bus mapping is backwards
  3. LKAS_STATUS 9 (FAULT)      -> the EPS wants camera messages it is no
                                   longer getting. Re-run with --hud
  4. status 6, no motion        -> the strongest negative we can produce. At
                                   that point the frame, rate, bus and ADAS
                                   context are all correct and the answer is
                                   in the EPS, not in our code. That is when
                                   the scan tool stops being optional

SIDE EFFECT - read this
-----------------------
Isolating the camera will probably raise a ProPILOT or front-camera warning on
the dash, because from the car's point of view the camera just went silent.
It clears on an ignition cycle. The relay is closed again in the finally
block, on every exit path including Ctrl-C.

    sudo systemctl stop comma
    python steer_camera_isolated.py --delta 15 --seconds 12
"""

import argparse
import atexit
import functools
import time

from manual_can_interface import CANInterface
from manual_nissancan import CarController_Manual, LKAS_BUS
from manual_tx_loop import PeriodicTransmitter, SteeringHold

LKAS_RATE_HZ = 100.0
LKAS_ADDR = 0x169
EPS_ADDR = 0x185
LKAS_STATUS_NAMES = {5: "INACTIVE", 6: "ACTIVE", 9: "FAULT", 14: "UNKNOWN-14"}
LKAS_STATUS_READY = 5

# How long to sleep between can_recv() polls while transmitting. A tight loop
# saturates the SPI link and starves the TX thread - that cost us a 44 Hz run
# and a PandaSpiNackResponse that killed the thread outright.
RECV_POLL_SLEEP_S = 0.02


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


def survey(panda, seconds):
    """Read for `seconds`. Returns (angle, status, per_bus, addr_by_bus).

    Nothing is transmitting during a survey, so this drains flat out.
    """
    angle = status = None
    per_bus = {}
    addr_by_bus = {}
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for msg in panda.can_recv():
            addr, data, bus = unpack(msg)
            per_bus[bus] = per_bus.get(bus, 0) + 1
            addr_by_bus.setdefault(addr, {})
            addr_by_bus[addr][bus] = addr_by_bus[addr].get(bus, 0) + 1
            if addr == EPS_ADDR:
                decoded = decode_185(data)
                if decoded is not None:
                    angle, status = decoded
        # no sleep: nothing is competing for the SPI link yet
    return angle, status, per_bus, addr_by_bus


def listen(panda, seconds):
    """Read for `seconds` while the TX thread is running. Throttled."""
    angle = status = None
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for msg in panda.can_recv():
            addr, data, _bus = unpack(msg)
            if addr == EPS_ADDR:
                decoded = decode_185(data)
                if decoded is not None:
                    angle, status = decoded
        time.sleep(RECV_POLL_SLEEP_S)
    return angle, status


def arm_relay_failsafe(can):
    """Close the relay on ANY exit, including an unhandled exception.

    The dry-run and early-return paths leave main() before the try/finally,
    and an exception during the survey would leave the camera isolated with
    nobody putting it back. atexit covers every one of those.
    """
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
            print("   The camera may still be isolated. Ignition cycle the")
            print("   car before driving it.")

    atexit.register(close_once)
    return close_once


def where_seen(addr_by_bus, addr):
    """'bus 0 (471), bus 2 (468)' or 'nowhere'. Kept out of the f-strings
    on purpose - a nested multiline f-string expression needs Python 3.12."""
    hits = addr_by_bus.get(addr, {})
    if not hits:
        return "nowhere"
    return ", ".join(f"bus {b} ({n})" for b, n in sorted(hits.items()))


def find_builder(nissan, names):
    """Return the first bound method on `nissan` matching one of `names`.

    The device copy of manual_nissancan.py has drifted from the one in git, so
    do not assume a method name. Missing is fine - the caller skips it.
    """
    for name in names:
        fn = getattr(nissan, name, None)
        if callable(fn):
            return name, fn
    return None, None


def main():
    p = argparse.ArgumentParser(
        description="steering test with the ADAS camera isolated")
    p.add_argument("--delta", type=float, default=15.0, metavar="DEG",
                   help="degrees relative to the current wheel angle "
                        "(default 15)")
    p.add_argument("--seconds", type=float, default=12.0)
    p.add_argument("--torque", type=float, default=1.0, metavar="NM",
                   help="MAX_TORQUE. opendbc ships 1.0 (default 1.0)")
    p.add_argument("--listen", type=float, default=5.0, metavar="SEC",
                   help="baseline survey before transmitting (default 5)")
    p.add_argument("--hud", action="store_true",
                   help="also transmit the camera's HUD messages (0x2b1 at "
                        "10 Hz, 0x4cc at 2 Hz). Use this if the EPS faults "
                        "once the camera is isolated - it means the car wants "
                        "those messages to keep existing")
    p.add_argument("--ignore-status", action="store_true",
                   help="transmit even if LKAS_STATUS is not 5 at rest")
    p.add_argument("--dry-run", action="store_true",
                   help="open the relay, survey, report, close it again. "
                        "Transmits nothing")
    p.add_argument("--dbc", default=None)
    args = p.parse_args()

    if abs(args.delta) > 40:
        p.error("keep it under 40 deg")
    if args.seconds > 30:
        p.error("30 s is plenty")
    if not 0.0 <= args.torque <= 2.5:
        p.error("MAX_TORQUE must be 0 to 2.5")

    # allow_output must be True even for a dry run. CANInterface.set_relay()
    # refuses to move the relay unless allow_output is set, and a dry run has
    # to move the relay - surveying the isolated bus IS the dry run. Passing
    # allow_output=not args.dry_run made --dry-run trip that guard before it
    # reached the survey.
    #
    # Dry run safety does not come from the safety mode here, it comes from
    # control flow: the dry-run branch returns before the transmitter is ever
    # constructed, so there is nothing to send. The relay is closed again on
    # the way out either way.
    can = CANInterface(simulation=False, allow_output=True)
    panda = can.panda

    if args.dry_run:
        print()
        print("DRY RUN - the panda goes to allOutput so the relay can move,")
        print("          but this run returns before building a transmitter.")
        print("          Nothing is sent. The relay is closed again at exit.")

    # --- baseline with the relay CLOSED, so we have something to compare to
    print()
    print("step 1 - relay CLOSED, measuring the stock bus")
    can.set_relay(False)
    time.sleep(0.5)
    _, _, closed_buses, closed_addrs = survey(panda, 2.0)
    c0, c2 = closed_buses.get(0, 0), closed_buses.get(2, 0)
    print("   traffic by bus:",
          "  ".join(f"bus {b}: {n}" for b, n in sorted(closed_buses.items())))
    if c0 and c2:
        print(f"   balance {min(c0, c2) / max(c0, c2):.2f} "
              f"(expect ~1.00 - the panda is bridging the two)")
    print(f"   0x169 seen on: {where_seen(closed_addrs, LKAS_ADDR)}")

    # --- now OPEN it
    print()
    print("step 2 - opening the intercept relay. The camera comes OFF the")
    print("         car-side bus. Expect a dash warning; it clears on an")
    print("         ignition cycle.")
    close_relay = arm_relay_failsafe(can)
    can.set_relay(True)
    time.sleep(1.0)

    can.print_health()

    print()
    print(f"step 3 - surveying for {args.listen:.1f} s with the camera isolated")
    start_angle, status, open_buses, open_addrs = survey(panda, args.listen)

    print()
    print("   traffic by bus:",
          "  ".join(f"bus {b}: {n}" for b, n in sorted(open_buses.items())))

    o0, o2 = open_buses.get(0, 0), open_buses.get(2, 0)

    # The check INVERTS compared to the closed-relay script. Closed, we demand
    # symmetry. Open, we demand asymmetry: bus 0 should carry the whole car and
    # bus 2 only the handful of messages the camera emits by itself.
    if o0 == 0:
        print()
        print("!! bus 0 is silent. The car side is on the other bus than we")
        print("   think, or the relay did not open the way we assume.")
        print("   Closing the relay. Nothing was transmitted.")
        can.set_relay(False)
        return

    balance = (min(o0, o2) / max(o0, o2)) if o2 else 0.0
    print(f"   balance {balance:.2f} (expect WELL under 0.50 - if it is still")
    print(f"            near 1.00 the relay did not actually open)")
    if balance > 0.5:
        print()
        print("!! the buses are still bridged. The relay did not open.")
        print("   Closing it and stopping - transmitting now would just")
        print("   repeat the run we already have.")
        can.set_relay(False)
        return

    # The direct proof that the camera is off the car-side bus.
    lkas_open = open_addrs.get(LKAS_ADDR, {})
    print(f"   0x169 now seen on: {where_seen(open_addrs, LKAS_ADDR)}")
    if lkas_open.get(0, 0) > 0:
        print()
        print("   !! 0x169 is STILL arriving on bus 0 with the relay open.")
        print("      Either the camera is not behind the relay or bus 0 is")
        print("      the camera side. This matters - write it down.")
    else:
        print("   -> the camera's 0x169 no longer reaches the car side. We")
        print("      are about to be the only source of it.")

    if start_angle is None:
        print()
        print("!! no 0x185 with the relay open - the EPS report is on the")
        print("   camera side of the relay, so we cannot measure the wheel in")
        print("   this configuration. That is outcome 2 and it is a real")
        print("   finding. Closing the relay. Nothing was transmitted.")
        can.set_relay(False)
        return

    print()
    print(f"wheel is at {start_angle:+.2f} deg")
    print(f"LKAS_STATUS = {status} ({LKAS_STATUS_NAMES.get(status, '?')})")

    if args.dry_run:
        print()
        print("dry run - closing the relay, transmitting nothing.")
        can.set_relay(False)
        can.print_summary()
        return

    if status != LKAS_STATUS_READY and not args.ignore_status:
        print()
        print(f"!! LKAS_STATUS is {status}, expected {LKAS_STATUS_READY} at "
              f"rest.")
        print("   If it is 9 the EPS faulted on losing the camera - re-run")
        print("   with --hud. Override with --ignore-status.")
        can.set_relay(False)
        return

    target = start_angle + args.delta

    nissan = CarController_Manual(args.dbc)
    nissan.create_steering_control = functools.partial(
        nissan.create_steering_control, max_torque=args.torque)

    hud_plan = []
    if args.hud:
        for label, rate, names in (
            ("HUD", 10.0, ("create_propilot_hud", "create_hud",
                           "create_propilot_hud_msg")),
            ("HUD_INFO", 2.0, ("create_propilot_hud_info",
                               "create_propilot_hud_info_msg",
                               "create_hud_info")),
        ):
            found, fn = find_builder(nissan, names)
            if fn is None:
                print(f"   (no builder for {label} on this copy of "
                      f"manual_nissancan.py - skipping it)")
            else:
                hud_plan.append((label, rate, fn, found))

    print()
    print("about to transmit:")
    print(f"  0x169 at {LKAS_RATE_HZ:.0f} Hz on bus {LKAS_BUS}")
    for label, rate, _fn, found in hud_plan:
        print(f"  {label} at {rate:.0f} Hz  (via {found}())")
    print(f"  {start_angle:+.2f} -> {target:+.2f} deg, MAX_TORQUE "
          f"{args.torque:.2f}")
    print(f"  holding {args.seconds:.1f} s")
    print("  CAMERA IS ISOLATED - we are the only source of 0x169")
    print()
    print("CAR IN PARK. WHEELS CHOCKED. HANDS OFF THE WHEEL.")
    print("A hand on the wheel invalidates the result - that has already")
    print("cost us one false positive.")
    if input("type GO to continue: ").strip() != "GO":
        can.set_relay(False)
        raise SystemExit("aborted")

    steering = SteeringHold(nissan, rate_hz=LKAS_RATE_HZ, timeout_s=0)
    steering.current = start_angle
    steering.target = start_angle

    tx = PeriodicTransmitter(can, base_rate_hz=LKAS_RATE_HZ)

    try:
        tx.set("LKAS", steering.build, rate_hz=LKAS_RATE_HZ)
        for label, rate, fn, _found in hud_plan:
            tx.set(label, fn, rate_hz=rate)
        tx.start()
        time.sleep(0.8)          # let the stream settle at the current angle

        steering.engage(target)
        print(f"engaged, ramping at {steering.MAX_RATE_DEG_S:.0f} deg/s")
        print()

        seen_statuses = {}
        max_dev = 0.0
        end = time.monotonic() + args.seconds

        def tick_count():
            for name in ("ticks", "tick_count", "n_ticks", "count"):
                v = getattr(tx, name, None)
                if isinstance(v, int):
                    return v
            return None

        last_ticks = tick_count()
        stalled_polls = 0
        lost_eps_polls = 0

        while time.monotonic() < end:
            angle, status = listen(panda, 0.25)

            now_ticks = tick_count()
            if now_ticks is not None and last_ticks is not None:
                if now_ticks == last_ticks:
                    stalled_polls += 1
                else:
                    stalled_polls = 0
                last_ticks = now_ticks
                if stalled_polls >= 3:
                    print()
                    print("!! the transmitter has stopped ticking - we are no")
                    print("   longer sending. Anything past this point says")
                    print("   nothing about the EPS. Stopping.")
                    break

            if angle is None:
                # 0x185 going quiet mid-run is itself a result: the EPS stopped
                # reporting once we started commanding it.
                lost_eps_polls += 1
                if lost_eps_polls >= 4:
                    print()
                    print("!! 0x185 has gone quiet for ~1 s while we were")
                    print("   transmitting. The EPS stopped reporting. Write")
                    print("   this down - it is a response, not a non-event.")
                    break
                continue
            lost_eps_polls = 0

            seen_statuses[status] = seen_statuses.get(status, 0) + 1
            dev = abs(angle - start_angle)
            max_dev = max(max_dev, dev)

            name = LKAS_STATUS_NAMES.get(status, "?")
            print(f"  commanded {steering.current:+8.2f}   "
                  f"measured {angle:+8.2f}   moved {dev:5.2f}   "
                  f"status {status} ({name})")

            if status == 9:
                print()
                print("!! EPS reported FAULT (9). Stopping.")
                print("   Our frames are reaching it and it is rejecting")
                print("   them. If this started only once the camera was")
                print("   isolated, re-run with --hud.")
                break

            if status == 14:
                print()
                print("!! LKAS_STATUS went to 14 while transmitting.")
                print("   Reproduced the mystery value on demand. Stopping.")
                break

        print()
        print("=== result ===")
        print(f"max deviation from start: {max_dev:.2f} deg")
        for s, n in sorted(seen_statuses.items()):
            print(f"  status {s} ({LKAS_STATUS_NAMES.get(s, '?')}): "
                  f"{n} samples")

        # Judge the run before judging the car.
        ticks = tick_count()
        if ticks is not None:
            actual_hz = ticks / max(1e-6, args.seconds)
            print()
            print(f"transmit rate: {actual_hz:.1f} Hz of {LKAS_RATE_HZ:.0f} "
                  f"target")
            if actual_hz < 0.9 * LKAS_RATE_HZ:
                print()
                print("!! THIS RUN IS NOT VALID. We did not hold the rate.")
                print("   Fix the rate before drawing any conclusion from the")
                print("   wheel not moving.")
                return

        if max_dev < 1.0:
            print()
            print("The wheel did not move. Under 1 deg is noise.")
            print()
            print("This is the strongest negative we can produce from")
            print("software: correct frame, correct checksum, 100 Hz, correct")
            print("bus, camera isolated, status 6, no motion. Every variable")
            print("we control is now ruled out. The next move is a")
            print("Nissan-capable scan tool on the EPS and ADAS modules -")
            print("pull DTCs BEFORE the next ignition cycle, they may clear.")
        else:
            print()
            print(f"The wheel moved {max_dev:.2f} deg. Before calling this a")
            print("success: did anyone touch the wheel? Re-run it twice.")

    except KeyboardInterrupt:
        print("\ninterrupted")

    finally:
        steering.release()
        time.sleep(0.3)
        tx.stop()
        # Always hand the car back the way we found it.
        can.set_relay(False)
        print()
        tx.notices.print_all()
        tx.print_stats()
        print()
        can.print_summary()


if __name__ == "__main__":
    main()