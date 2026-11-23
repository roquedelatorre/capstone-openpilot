"""Toggle the hazards with the 0x3AF button command and READ BACK 0x27A to
prove the car acted on it. The blinker equivalent of steer_position.py:
command on one message, measured feedback on another.

WHY THIS AND NOT blinker_bringup.py --left
-------------------------------------------
blinker_bringup.py streams 0x27A. Our own DBC says that is the wrong idea:

    CM_ BO_ 634 "... Its fields are state/observation definitions, NOT
                 validated actuation commands."
    CM_ BO_ 1549 "... Like 0x27A this message REPORTS lamp state."

0x27A is the car telling the bus what the lamps are doing. Nothing drives a
lamp off the back of it, so streaming it lights nothing - which is exactly
what you saw. Steering was different: 0x169 is a real command the EPS acts
on, which is why it moved.

The one lamp COMMAND we have reverse engineered is the hazard button:

    CM_ BO_ 943 "... This is the COMMAND side of the hazard function: all 35
                 sustained hazard periods in 0x27A began 20-30 ms after a
                 pulse here."

It is a momentary press, the same pulse for on and off, so it TOGGLES, and it
drives BOTH sides at once. There is no per-side (left-only / right-only)
command in the DBC - the turn-stalk command, if it is even on the bus, has
not been captured. So "a blinker like steer_position" is, concretely, a
hazard toggle with closed-loop confirmation. This script is that.

THE OTHER REASON blinker_bringup.py WAS SILENT
----------------------------------------------
CANInterface.__init__ has set_safety_mode and set_heartbeat_disabled
commented out. With pandad stopped and no safety mode set, the panda is in
SAFETY_SILENT (or a Nissan model that only permits 0x169), so 0x3AF and 0x27A
are dropped before they reach the wire. 0x169 went out because it is in
openpilot's TX allowlist; the hazard button is not. This script sets the
panda to ALLOUTPUT and disables the heartbeat itself (after you type GO),
which is what lets the command actually transmit. ALLOUTPUT turns the panda's
safety filtering OFF, so this is a garage-only tool - it restores SILENT on
exit.

WHAT IT DOES
------------
  1. self-test: builds the 0x3AF press frame and a 0x27A off frame and checks
     them against frames derived from the DBC, the same gate steer_position
     uses before it trusts its encoder.
  2. baseline: listens a few seconds, decodes 0x27A, and reports the current
     lamp state - steady (hazards off) or flashing (hazards already on) - plus
     the frame rate and bus, which confirms the harness mapping.
  3. prints the plan and waits for GO.
  4. sets the panda to ALLOUTPUT, pulses 0x3AF, and watches 0x27A come back,
     timing how long until the lamp state changes and confirming the new
     state. A per-toggle table at the end shows before -> after and latency.

USAGE
-----
Read-only first contact, transmits nothing, no GO, no safety-mode change:
    python hazard_toggle.py --report-only

One toggle (this flips hazards; run it again to flip them back):
    python hazard_toggle.py

The demo - toggle a few times and watch it track each flip:
    python hazard_toggle.py --cycles 4 --hold 4

Stop openpilot first either way, or pandad holds the panda:
    sudo systemctl stop comma
"""

import argparse
import atexit
import time

from manual_can_interface import CANInterface
from manual_nissancan import (CarController_Manual, HAZARD_SWITCH_BUS,
                              HAZARD_RATE_HZ, HAZARD_PULSE_S)

TURN_ADDR = 0x27A      # TURN_SIGNALS_27A, 7 bytes - REPORT, we only read it
HAZARD_ADDR = 0x3AF    # HAZARD_SWITCH, 8 bytes - COMMAND, we send it

# Frames derived straight from the DBC, independent of the packer, so the
# self-test catches a DBC that has drifted on the device.
#   0x27A off, counter 0: byte2=SET_0x4D=0x4D, blinker bits (byte3)=0,
#   checksum = (sum(bytes[0:6]) + 0x7C) & 0xFF = 0xC9
REF_27A_OFF = bytes.fromhex("00004d000000c9")
#   0x3AF pressed: HAZARD_BUTTON is byte0 bit7, rest constant 0x00
REF_3AF_PRESS = bytes.fromhex("8000000000000000")


def unpack(msg):
    """panda reports the bus with flag bits in the top nibble - mask them.
    Same helper steer_position uses."""
    if len(msg) == 3:
        addr, data, bus = msg[0], msg[1], msg[2]
    else:
        addr, data, bus = msg[0], msg[2], msg[3]
    return addr, bytes(data), bus & 0x0F


def decode_27a(data):
    """Return (left, right, counter) from a 0x27A frame, or None."""
    if len(data) < 7:
        return None
    left = data[3] & 0x01
    right = (data[3] >> 1) & 0x01
    counter = data[5] & 0x0F
    return left, right, counter


def self_test(nissan):
    # first call on a fresh controller uses counter 0, matching the reference
    built_27a = bytes(nissan.create_turn_signal_control(False, False)[1])
    built_3af = bytes(nissan.create_hazard_control(True)[1])
    ok_27a = built_27a == REF_27A_OFF
    ok_3af = built_3af == REF_3AF_PRESS

    def show(label, built, ref, ok):
        print(f"   {label}")
        print(f"      from DBC : {' '.join(f'{b:02x}' for b in ref)}")
        print(f"      we build : {' '.join(f'{b:02x}' for b in built)}")
        print(f"      -> {'MATCH' if ok else 'MISMATCH'}")

    print("encoder self-test against frames derived from the DBC:")
    show("0x27A off (counter 0)", built_27a, REF_27A_OFF, ok_27a)
    show("0x3AF pressed", built_3af, REF_3AF_PRESS, ok_3af)
    if not (ok_27a and ok_3af):
        print()
        print("Refusing to run - the packer does not reproduce the frames the")
        print("DBC describes. Check nissan_leaf_2025_capstone.dbc on this device.")
    return ok_27a and ok_3af


def classify(samples):
    """samples is a list of (t, left, right). Returns (flashing, transitions,
    both_together). Flashing = the bits toggled; both_together = left and
    right moved on the same edges (the hazard signature, not a single turn)."""
    transitions = 0
    both = 0
    last = None
    for _t, l, r in samples:
        st = (l, r)
        if last is not None and st != last:
            transitions += 1
            if (l != last[0]) and (r != last[1]):
                both += 1
        last = st
    flashing = transitions >= 2
    both_together = both >= 1
    return flashing, transitions, both_together


def listen(panda, seconds):
    """Collect 0x27A samples for a while. Returns (samples, frames, buses)."""
    samples = []
    buses = {}
    t0 = time.monotonic()
    end = t0 + seconds
    while time.monotonic() < end:
        for m in panda.can_recv():
            addr, data, bus = unpack(m)
            if addr != TURN_ADDR:
                continue
            d = decode_27a(data)
            if d is None:
                continue
            left, right, _c = d
            samples.append((time.monotonic() - t0, left, right))
            buses[bus] = buses.get(bus, 0) + 1
        time.sleep(0.002)
    return samples, len(samples), buses


def do_toggle(panda, nissan, bus, width, rate, observe):
    """Send one 0x3AF pulse and watch 0x27A for the whole window. Returns
    (pre_flashing, post_flashing, latency_s, together, n_frames)."""
    press = (HAZARD_ADDR, bytes(nissan.create_hazard_control(True)[1]), bus)
    release = (HAZARD_ADDR, bytes(nissan.create_hazard_control(False)[1]), bus)

    period = 1.0 / rate
    t0 = time.monotonic()
    press_end = t0 + width
    tail_end = press_end + max(0.1, 2.0 / rate)   # a few "released" frames
    watch_end = press_end + observe
    next_send = t0

    pre = []            # before the first observed change
    post = []           # after it
    last_state = None
    first_change_t = None

    while time.monotonic() < watch_end:
        now = time.monotonic()

        if now < tail_end and now >= next_send:
            panda.can_send(*(press if now < press_end else release))
            next_send += period

        for m in panda.can_recv():
            addr, data, b = unpack(m)
            if addr != TURN_ADDR:
                continue
            d = decode_27a(data)
            if d is None:
                continue
            left, right, _c = d
            st = (left, right)
            rec = (now - t0, left, right)
            if first_change_t is None:
                pre.append(rec)
                if last_state is not None and st != last_state:
                    first_change_t = now - t0
                    post.append(rec)
            else:
                post.append(rec)
            last_state = st

        time.sleep(0.002)

    pre_flashing, _pt, _pb = classify(pre)
    post_flashing, _xt, together = classify(post)
    return pre_flashing, post_flashing, first_change_t, together, len(pre) + len(post)


def main():
    p = argparse.ArgumentParser(
        description="toggle hazards (0x3AF) and confirm via 0x27A readback")
    p.add_argument("--report-only", action="store_true",
                   help="listen and decode current lamp state, transmit "
                        "nothing, no safety-mode change, no GO")
    p.add_argument("--cycles", type=int, default=1, metavar="N",
                   help="how many times to toggle (default 1)")
    p.add_argument("--hold", type=float, default=4.0, metavar="SEC",
                   help="seconds to watch 0x27A after each toggle (default 4)")
    p.add_argument("--width", type=float, default=HAZARD_PULSE_S, metavar="SEC",
                   help=f"press width (default {HAZARD_PULSE_S}, the measured "
                        "median)")
    p.add_argument("--rate", type=float, default=HAZARD_RATE_HZ, metavar="HZ",
                   help=f"0x3AF send rate (default {HAZARD_RATE_HZ:.0f}, as "
                        "captured)")
    p.add_argument("--listen", type=float, default=4.0, metavar="SEC",
                   help="baseline listen before toggling (default 4)")
    p.add_argument("--bus", type=int, default=None, metavar="N",
                   help="force the 0x3AF bus (default: auto from harness)")
    p.add_argument("--max-cycles", type=int, default=20, metavar="N")
    args = p.parse_args()

    if args.cycles < 1 or args.cycles > args.max_cycles:
        p.error(f"--cycles must be 1..{args.max_cycles}")
    if args.width > 2.0:
        p.error("a button press is well under 2 s")
    if args.hold > 30:
        p.error("30 s per watch window is plenty")

    nissan = CarController_Manual()

    print()
    if not self_test(nissan):
        return
    print()

    can = CANInterface(simulation=False, allow_output=True)
    panda = can.panda

    # work out which bus the hazard button lives on. A flipped comma harness
    # swaps 0 and 2; the button is captured on bus 0 normally.
    harness = None
    try:
        harness = panda.health().get("car_harness_status")
    except Exception:
        pass
    if args.bus is not None:
        hazard_bus = args.bus
    elif harness == 2:
        hazard_bus = 2
        print("harness is FLIPPED - sending 0x3AF on bus 2 instead of 0")
    else:
        hazard_bus = HAZARD_SWITCH_BUS

    # ---- baseline, read-only
    print(f"baseline - listening {args.listen:.1f} s on 0x27A")
    samples, frames, buses = listen(panda, args.listen)
    if frames == 0:
        print("   no 0x27A seen. Is the car awake? Is pandad stopped?")
        print("   (0x27A is a passive report - if it is absent here, the bus")
        print("    mapping or the harness is the problem, not the command.)")
        return

    flashing, transitions, _both = classify(samples)
    rate_hz = frames / max(1e-6, args.listen)
    bus_str = ", ".join(f"bus {b} x{n}" for b, n in sorted(buses.items()))
    state = "FLASHING (hazards already on)" if flashing else "steady (lamps off)"
    print(f"   0x27A: {frames} frames ({rate_hz:.1f} Hz), {bus_str}")
    print(f"   current lamp state: {state}")

    if args.report_only:
        print()
        print("report-only: nothing transmitted, panda untouched.")
        return

    # ---- plan
    print()
    print("plan:")
    expect = flashing
    for i in range(1, args.cycles + 1):
        print(f"   {i}. pulse 0x3AF {args.width:.2f}s on bus {hazard_bus} "
              f"-> expect hazards {'OFF' if expect else 'ON'}, "
              f"watch 0x27A {args.hold:.1f}s")
        expect = not expect
    print(f"   this sets the panda to ALLOUTPUT (safety filtering OFF) so the")
    print(f"   button command can transmit, and restores SILENT on exit.")
    print()
    print("CAR IN PARK. This only blinks lamps - harmless - but confirm the")
    print("dash indicator is where you can see it.")
    if input("type GO to continue: ").strip() != "GO":
        return

    # ---- arm the panda. This is the piece CANInterface leaves commented out.
    from panda import Panda
    silent = getattr(Panda, "SAFETY_SILENT", 0)
    alloutput = getattr(Panda, "SAFETY_ALLOUTPUT", 17)

    restored = {"done": False}

    def disarm():
        if restored["done"]:
            return
        restored["done"] = True
        try:
            panda.set_safety_mode(silent)
            print("failsafe: panda returned to SAFETY_SILENT")
        except Exception as exc:
            print(f"!! could not restore SILENT: {exc}")
            print("   Restart pandad (start openpilot) to reset the panda.")

    atexit.register(disarm)

    try:
        panda.set_heartbeat_disabled()
    except Exception as exc:
        print(f"note: set_heartbeat_disabled failed ({exc}); if TX dies after")
        print("a few seconds that is the panda reverting to SILENT on no heartbeat.")
    panda.set_safety_mode(alloutput)
    time.sleep(0.2)
    mode_now = None
    try:
        mode_now = panda.health().get("safety_mode")
    except Exception:
        pass
    print(f"panda safety mode now {mode_now} (ALLOUTPUT = {alloutput})")
    if mode_now is not None and mode_now != alloutput:
        print("!! safety mode did not take - the command will not transmit.")
        print("   Is pandad really stopped?")

    results = []
    try:
        for i in range(1, args.cycles + 1):
            print()
            print(f"--- toggle {i}/{args.cycles}")
            pre, post, latency, together, n = do_toggle(
                panda, nissan, hazard_bus, args.width, args.rate, args.hold)
            lat = f"{latency * 1000:.0f} ms" if latency is not None else "none"
            changed = pre != post
            print(f"    before: {'flashing' if pre else 'steady'}   "
                  f"after: {'flashing' if post else 'steady'}   "
                  f"change seen in {lat}")
            if post and not together:
                print("    (one side only - that is a turn signal, not hazards)")
            if not changed and latency is None:
                print("    no change observed - see the notes at the end")
            results.append((i, pre, post, latency, together, n))

    except KeyboardInterrupt:
        print("\ninterrupted")

    finally:
        disarm()

        print()
        print("=== result ===")
        print(f"  {'#':>2}  {'before':>9}  {'after':>9}  {'latency':>9}  "
              f"{'both?':>5}  {'frames':>6}")
        for i, pre, post, latency, together, n in results:
            lat = f"{latency * 1000:.0f} ms" if latency is not None else "-"
            print(f"  {i:>2}  {'flashing' if pre else 'steady':>9}  "
                  f"{'flashing' if post else 'steady':>9}  {lat:>9}  "
                  f"{'yes' if together else 'no':>5}  {n:>6}")

        # where did we end up?
        if results:
            ended_on = results[-1][2]
            print()
            if ended_on:
                print("Hazards are ON. Run the script once more (or with an even")
                print("--cycles) to turn them back off.")
            else:
                print("Hazards are OFF.")
            toggled = [r for r in results if r[1] != r[2]]
            if toggled and all(r[3] is not None for r in toggled):
                print("Each pulse flipped the state and 0x27A confirmed it - a")
                print("closed loop: command on 0x3AF, state read back on 0x27A.")
            elif not toggled:
                print("Nothing changed. Most likely the safety mode did not take")
                print("(pandad still running), the harness bus is not what we")
                print("assumed (try --bus 2), or this car ignores the pulse while")
                print("stationary. --report-only confirms 0x27A is readable.")

        print()
        can.print_summary()


if __name__ == "__main__":
    main()