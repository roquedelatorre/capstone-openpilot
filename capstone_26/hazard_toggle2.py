"""Toggle hazards on 0x3AF and confirm it worked by reading back 0x27A.

Fixed version of Roque's hazard_toggle.py. His encoding is correct - both
self-tests match the DBC and the 0x27A sum checksum reproduces a real
captured frame. What was broken was the measurement, plus two things the
script never controlled.

WHAT WAS WRONG
--------------
1. pre_flashing could never be true. The original did:

       if first_change_t is None:
           pre.append(rec)
           if last_state is not None and st != last_state:
               first_change_t = now - t0      # pre stops growing here

   so `pre` held at most ONE transition, while classify() needs two:

       flashing = transitions >= 2

   Consequences: turning hazards OFF always reported "no change observed",
   because changed = (pre != post) and pre was pinned False. And latency was
   measured against whatever edge happened first, which for already-flashing
   lamps is just the next blink, not our command.

   Fixed by watching a QUIET window before transmitting anything, so `pre` is
   a real measurement of the starting state.

2. No relay control. Every steering script we have forces the relay to a
   known state first, because we lost a week to assuming it. This one never
   touched it, and 0x27A is a camera-side message - the census on 2026-10-28
   caught it in the camera's own transmit list at 60 Hz. With the relay open,
   the command goes to one segment and the feedback comes from another.

3. No contention check. Steering failed for a week because the camera was
   transmitting 0x169 while we transmitted ours and the EPS got two
   contradictory commands. If a body module puts 0x3AF on the bus at rest,
   the hazard button has the identical problem. The baseline now counts
   0x3AF per bus and says so.

Also added --bus both, which tries bus 0 and bus 2 in turn and reports which
one the car responded to. manual_nissancan.py warns that a flipped harness
swaps 0 and 2, and the harness read FLIPPED on 2026-10-28 and normal on
2026-11-09, so the captured bus numbers may not apply today.

FIXES IN THIS REVISION (2026-11-09)
-----------------------------------
A. Relay call no longer crashes the run. CANInterface.set_relay() raises on a
   live panda, and it sat in front of everything, so every non-report run died
   before it sent a frame. Hazards do not need the relay, so it now warns and
   continues.
B. Pulses go through CANInterface.send() instead of panda.can_send() directly,
   so sent_log, print_summary() and export_c() actually see the frames. Before,
   the end-of-run summary always said "No messages sent".

USAGE
-----
Read-only, transmits nothing, no safety-mode change:
    python hazard_toggle2.py --report-only

One toggle on the bus manual_nissancan.py says (0):
    python hazard_toggle2.py

Try both buses and see which the car answers on:
    python hazard_toggle2.py --bus both

The demo:
    python hazard_toggle2.py --cycles 4 --hold 4

Stop openpilot first or pandad holds the panda:
    sudo systemctl stop comma

WATCH THE LAMPS, not just the table. The table is better than it was but
your eyes are the ground truth.
"""

import argparse
import atexit
import time

from manual_can_interface import CANInterface
from manual_nissancan import (CarController_Manual, HAZARD_SWITCH_BUS,
                              HAZARD_RATE_HZ, HAZARD_PULSE_S)

TURN_ADDR = 0x27A      # REPORT - we only ever read this
HAZARD_ADDR = 0x3AF    # COMMAND - this is what we send

# Frames derived straight from the DBC, independent of the packer, so the
# self-test catches a DBC that has drifted on the device.
REF_27A_OFF = bytes.fromhex("00004d000000c9")
REF_3AF_PRESS = bytes.fromhex("8000000000000000")

# A quiet window has to span at least two blink edges to be classified.
# Hazards flash near 1.5 Hz, so a half period is about 0.33 s - 2.5 s gives
# roughly seven edges, which is plenty of margin.
MIN_WINDOW_S = 2.5


def unpack(msg):
    """panda reports the bus with flag bits in the top nibble - mask them."""
    if len(msg) == 3:
        addr, data, bus = msg[0], msg[1], msg[2]
    else:
        addr, data, bus = msg[0], msg[2], msg[3]
    return addr, bytes(data), bus & 0x0F


def decode_27a(data):
    """(left, right, counter) from a 0x27A frame, or None."""
    if len(data) < 7:
        return None
    return data[3] & 0x01, (data[3] >> 1) & 0x01, data[5] & 0x0F


def self_test(nissan):
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
        print("DBC describes. Check nissan_leaf_2025_capstone.dbc.")
    return ok_27a and ok_3af


def classify(samples, window_s):
    """samples: list of (t, left, right).

    Returns (flashing, transitions, both_together). Refuses to call anything
    flashing if the window was too short to hold two blink edges - the
    original reported on windows of a few milliseconds.
    """
    transitions = 0
    both = 0
    last = None
    for _t, left, right in samples:
        st = (left, right)
        if last is not None and st != last:
            transitions += 1
            if left != last[0] and right != last[1]:
                both += 1
        last = st
    flashing = transitions >= 2 and window_s >= MIN_WINDOW_S
    return flashing, transitions, both >= 1


def watch(panda, seconds, on_frame=None):
    """Collect 0x27A for `seconds`. Returns (samples, buses).

    on_frame(now) is called every loop so a caller can transmit while we
    watch, without a second timing loop fighting this one for the SPI link.
    """
    samples = []
    buses = {}
    t0 = time.monotonic()
    end = t0 + seconds
    while time.monotonic() < end:
        now = time.monotonic()
        if on_frame is not None:
            on_frame(now)
        for m in panda.can_recv():
            addr, data, bus = unpack(m)
            if addr != TURN_ADDR:
                continue
            d = decode_27a(data)
            if d is None:
                continue
            left, right, _c = d
            samples.append((now - t0, left, right))
            buses[bus] = buses.get(bus, 0) + 1
        time.sleep(0.002)
    return samples, buses


def survey(panda, seconds):
    """Baseline: where is 0x27A, and is 0x3AF already on the bus?"""
    per = {}
    t0 = time.monotonic()
    while time.monotonic() < t0 + seconds:
        for m in panda.can_recv():
            addr, _data, bus = unpack(m)
            per[(addr, bus)] = per.get((addr, bus), 0) + 1
        time.sleep(0.002)
    return per


def pulse_sender(can, nissan, bus, width, rate):
    """Returns a callable for watch()'s on_frame that sends one 0x3AF press
    of `width` seconds at `rate` Hz, then a short released tail, then stops.

    CHANGED - sends through CANInterface.send() instead of panda.can_send()
    directly, so every frame lands in can.sent_log and the end-of-run summary,
    print_log and export_c actually see them. verbose=False keeps the 10 Hz
    pulse from flooding the terminal.
    """
    press = (HAZARD_ADDR, bytes(nissan.create_hazard_control(True)[1]), bus)
    release = (HAZARD_ADDR, bytes(nissan.create_hazard_control(False)[1]), bus)
    period = 1.0 / rate
    state = {"t0": None, "next": 0.0, "sent": 0}

    def send(now):
        if state["t0"] is None:
            state["t0"] = now
            state["next"] = now
        elapsed = now - state["t0"]
        tail_end = width + max(0.1, 2.0 / rate)
        if elapsed > tail_end or now < state["next"]:
            return
        try:
            can.send(press if elapsed < width else release, verbose=False)
            state["sent"] += 1
        except Exception:
            pass
        # never let a slow loop bank up a burst of catch-up sends
        state["next"] = max(now + period, state["next"] + period)

    send.state = state
    return send


def do_toggle(can, nissan, bus, width, rate, pre_s, post_s):
    """Quiet baseline, then pulse, then watch. Returns a result dict."""
    panda = can.panda
    pre_samples, _b = watch(panda, pre_s)
    pre_flashing, pre_tr, _pb = classify(pre_samples, pre_s)

    sender = pulse_sender(can, nissan, bus, width, rate)
    post_samples, _b2 = watch(panda, post_s, on_frame=sender)
    post_flashing, post_tr, together = classify(post_samples, post_s)

    # latency: first state change after the pulse began
    latency = None
    last = None
    for t, left, right in post_samples:
        st = (left, right)
        if last is not None and st != last:
            latency = t
            break
        last = st

    return {
        "pre": pre_flashing, "post": post_flashing,
        "pre_tr": pre_tr, "post_tr": post_tr,
        "latency": latency, "together": together,
        "sent": sender.state["sent"],
        "frames": len(pre_samples) + len(post_samples),
    }


def main():
    p = argparse.ArgumentParser(
        description="toggle hazards (0x3AF), confirm via 0x27A readback")
    p.add_argument("--report-only", action="store_true",
                   help="survey the bus and stop. Transmits nothing, does not "
                        "touch the relay or the safety mode")
    p.add_argument("--cycles", type=int, default=1, metavar="N")
    p.add_argument("--hold", type=float, default=4.0, metavar="SEC",
                   help="seconds to watch 0x27A after each pulse (default 4)")
    p.add_argument("--pre", type=float, default=MIN_WINDOW_S, metavar="SEC",
                   help=f"quiet window before each pulse (default "
                        f"{MIN_WINDOW_S}). Below {MIN_WINDOW_S} the starting "
                        f"state cannot be classified")
    p.add_argument("--width", type=float, default=HAZARD_PULSE_S, metavar="SEC",
                   help=f"press width (default {HAZARD_PULSE_S}, the measured "
                        "median of 110 real presses)")
    p.add_argument("--rate", type=float, default=HAZARD_RATE_HZ, metavar="HZ")
    p.add_argument("--listen", type=float, default=4.0, metavar="SEC",
                   help="baseline survey length (default 4)")
    p.add_argument("--bus", default=str(HAZARD_SWITCH_BUS), metavar="N",
                   help='0, 2, or "both" to try each in turn (default '
                        f'{HAZARD_SWITCH_BUS})')
    p.add_argument("--relay", choices=("closed", "leave"), default="closed",
                   help="relay state before testing. closed (default) puts "
                        "the camera back on the vehicle bus, which is where "
                        "0x27A comes from")
    args = p.parse_args()

    if args.bus == "both":
        buses = [0, 2]
    else:
        try:
            buses = [int(args.bus)]
        except ValueError:
            p.error('--bus must be 0, 2 or "both"')
    if args.cycles < 1 or args.cycles > 20:
        p.error("--cycles must be 1..20")
    if args.width > 2.0:
        p.error("a button press is well under 2 s")
    if args.hold > 30 or args.pre > 30:
        p.error("30 s per window is plenty")

    nissan = CarController_Manual()

    print()
    if not self_test(nissan):
        return
    print()

    can = CANInterface(simulation=False, allow_output=True)
    panda = can.panda

    relay_touched = {"done": False}

    def restore():
        if relay_touched["done"]:
            return
        relay_touched["done"] = True
        try:
            from panda import Panda
            panda.set_safety_mode(getattr(Panda, "SAFETY_SILENT", 0))
            print("failsafe: panda returned to SAFETY_SILENT")
        except Exception as exc:
            print(f"!! could not restore SILENT: {exc}")

    atexit.register(restore)

    if args.relay == "closed" and not args.report_only:
        # CHANGED - CANInterface.set_relay() raises on a live panda ("Live
        # relay control disabled pending vehicle safety validation"), and this
        # call used to sit in front of everything, so every non-report run died
        # right here before sending a thing. Hazards do not need the relay: in
        # its normal CLOSED state both bus segments are bridged, so the 0x27A
        # readback works. Warn and continue instead of crashing.
        try:
            print("requesting relay CLOSED (0x27A readback wants the camera "
                  "on the vehicle bus)")
            can.set_relay(False)
            time.sleep(0.8)
        except RuntimeError as exc:
            print(f"relay left as-is: {exc}")
            print("fine for hazards - the baseline below shows whether the "
                  "buses are actually bridged.")

    # ---- baseline survey
    print()
    print(f"baseline - surveying {args.listen:.1f} s")
    per = survey(panda, args.listen)

    totals = {}
    for (_a, b), n in per.items():
        totals[b] = totals.get(b, 0) + n
    print("   total frames by bus:",
          "  ".join(f"bus {b}: {n}" for b, n in sorted(totals.items())))

    b0, b2 = totals.get(0, 0), totals.get(2, 0)
    if b0 and b2:
        ratio = min(b0, b2) / max(b0, b2)
        print(f"   bus balance {ratio:.2f} - "
              f"{'relay CLOSED, buses bridged' if ratio > 0.5 else 'relay OPEN, segments separate'}")
    else:
        print("   one bus is silent - the relay is open, or the harness is "
              "not what we think")

    def where(addr):
        hits = {b: n for (a, b), n in per.items() if a == addr}
        return hits

    turn_hits = where(TURN_ADDR)
    haz_hits = where(HAZARD_ADDR)
    print(f"   0x27A on: {turn_hits or 'ABSENT'}")
    print(f"   0x3AF on: {haz_hits or 'absent (good - nobody else is '
                                      'transmitting the button)'}")

    if haz_hits:
        print()
        print("   !! 0x3AF IS ALREADY ON THE BUS. Another module is")
        print("      transmitting the hazard button at rest. When we send")
        print("      'pressed' and it sends 'not pressed', the receiver gets")
        print("      contradictory commands - this is exactly what stopped")
        print("      steering working for a week. If the toggles below do")
        print("      nothing, this is the reason, and the fix is the same one")
        print("      steering needed: isolate the other source.")

    if not turn_hits:
        print()
        print("   no 0x27A at all - we have no way to confirm anything.")
        print("   Fix that before testing the command.")
        return

    # starting state, measured over a window long enough to classify
    print()
    print(f"   reading starting lamp state over {args.pre:.1f} s")
    s0, _b = watch(panda, args.pre)
    start_flashing, start_tr, _t = classify(s0, args.pre)
    print(f"   {len(s0)} frames, {start_tr} state changes -> "
          f"{'FLASHING (hazards on)' if start_flashing else 'steady (lamps off)'}")

    if args.report_only:
        print()
        print("report-only: nothing transmitted, panda untouched.")
        return

    # ---- plan
    print()
    print("plan:")
    for bus in buses:
        for i in range(1, args.cycles + 1):
            print(f"   bus {bus}, pulse {i}: {args.pre:.1f}s quiet, "
                  f"{args.width:.2f}s press, {args.hold:.1f}s watch")
    print("   panda goes to ALLOUTPUT so the button command can transmit,")
    print("   and back to SILENT on exit.")
    print()
    print("CAR IN PARK. This only blinks lamps. WATCH THE LAMPS - they are")
    print("the ground truth, the table is only a cross-check.")
    if input("type GO to continue: ").strip() != "GO":
        return

    from panda import Panda
    alloutput = getattr(Panda, "SAFETY_ALLOUTPUT", 17)
    try:
        panda.set_heartbeat_disabled()
    except Exception as exc:
        print(f"note: set_heartbeat_disabled failed ({exc}); if TX stops after")
        print("a few seconds that is the panda reverting on no heartbeat.")
    panda.set_safety_mode(alloutput)
    time.sleep(0.2)
    mode_now = None
    try:
        mode_now = panda.health().get("safety_mode")
    except Exception:
        pass
    print(f"panda safety mode now {mode_now} (ALLOUTPUT = {alloutput})")
    if mode_now is not None and mode_now != alloutput:
        print("!! safety mode did not take - nothing will transmit.")
        print("   Is pandad really stopped?  sudo systemctl stop comma")

    results = []
    try:
        for bus in buses:
            for i in range(1, args.cycles + 1):
                print()
                print(f"--- bus {bus}, pulse {i}/{args.cycles}")
                r = do_toggle(can, nissan, bus, args.width, args.rate,
                              args.pre, args.hold)
                r["bus"] = bus
                r["n"] = i
                lat = (f"{r['latency'] * 1000:.0f} ms"
                       if r["latency"] is not None else "none")
                print(f"    sent {r['sent']} frames of 0x3AF on bus {bus}")
                print(f"    before: {'flashing' if r['pre'] else 'steady'}"
                      f" ({r['pre_tr']} changes)   "
                      f"after: {'flashing' if r['post'] else 'steady'}"
                      f" ({r['post_tr']} changes)   first change {lat}")
                if r["pre"] != r["post"]:
                    print("    STATE CHANGED")
                    if r["post"] and not r["together"]:
                        print("    (one side only - that is a turn signal, "
                              "not hazards)")
                else:
                    print("    no state change")
                results.append(r)

    except KeyboardInterrupt:
        print("\ninterrupted")

    finally:
        restore()

        print()
        print("=== result ===")
        print(f"  {'bus':>3} {'#':>2}  {'before':>9}  {'after':>9}  "
              f"{'latency':>9}  {'both?':>5}  {'sent':>5}")
        for r in results:
            lat = (f"{r['latency'] * 1000:.0f} ms"
                   if r["latency"] is not None else "-")
            print(f"  {r['bus']:>3} {r['n']:>2}  "
                  f"{'flashing' if r['pre'] else 'steady':>9}  "
                  f"{'flashing' if r['post'] else 'steady':>9}  {lat:>9}  "
                  f"{'yes' if r['together'] else 'no':>5}  {r['sent']:>5}")

        changed = [r for r in results if r["pre"] != r["post"]]
        print()
        if changed:
            worked = sorted({r["bus"] for r in changed})
            print(f"The car responded on bus {', '.join(map(str, worked))}.")
            if len(buses) > 1:
                print(f"Set HAZARD_SWITCH_BUS = {worked[0]} in "
                      f"manual_nissancan.py.")
            print("Command on 0x3AF, state confirmed on 0x27A - closed loop.")
            if results[-1]["post"]:
                print("Hazards are ON. Run once more to turn them off.")
        else:
            print("No state change on any bus tried.")
            if haz_hits:
                print("0x3AF was already on the bus at baseline, so start")
                print("there - we are competing with another transmitter.")
            elif len(buses) == 1:
                print("Try --bus both before concluding anything.")
            else:
                print("Both buses tried. Next: confirm the lamps really did")
                print("not blink (the table can only see 0x27A), then check")
                print("whether this car gates the hazard button on something")
                print("else - ignition state, or a different command entirely.")

        print()
        can.print_summary()


if __name__ == "__main__":
    main()