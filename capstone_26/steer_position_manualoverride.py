"""Command the steering to an ABSOLUTE angle, not a relative change.

steer_forward.py --delta -20 from +45 deg -> goes to +25
steer_position.py --angle -20 from +45 deg -> goes to -20

Same forwarding and substitution as steer_forward.py, which is what moved
 the wheel on 2026-11-04. The only difference is how the target is worked out,
 and that you can give it a list of positions to walk through.

VERIFIED 2026-11-04 (with steer_forward.py, identical frame path):
run 1 start +5.98 commanded +20.98 reached +19.70 moved 13.72
run 2 start +3.15 commanded +18.15 reached +17.02 moved 13.87
run 3 negative delta - wheel moved right, direction confirmed

Two different start angles, same displacement to 0.15 deg, and the wheel
held +17.02 to a hundredth of a degree for six seconds. Reversing the sign
reversed the direction. That is a real control loop, not drift.

SIGN CONVENTION
---------------
Positive is LEFT, negative is RIGHT - the same convention angle_check.py
uses and the one the DBC encodes (scale -0.01, offset 1310).

WHAT IT DOES
------------
Opens the harness relay so the camera is off the vehicle bus, then forwards
every camera message from bus 2 to bus 0 itself, substituting our own 0x169
in place of the camera's as each one arrives and reusing the camera's
counter. The EPS sees the cadence, phase and counter sequence it expects,
and a steering command that is ours.

Why that is necessary, in one line each:
- relay closed: the camera's own 0x169 competes with ours at ~200 Hz
  combined, half of it saying inactive, and the wheel does not move
- relay open with no forwarding: the EPS reports LKAS_STATUS 14, which
  means "no camera", and ignores everything
- relay open WITH forwarding: the EPS is satisfied and we are the only
  source of the steering command. This is that case.

DRIVER OVERRIDE
---------------
Grab the wheel while it is moving and the car lets go, for the rest of the
run. Detection is openpilot's, not invented here: STEER_TORQUE_DRIVER out of
0x185, averaged over 12 samples, tripping above 1.0 Nm - TORQUE_SAMPLES and
STEER_THRESHOLD straight out of carstate.py and values.py. The average is
what stops a bump from false-triggering.

The resting torque reading is measured during the baseline and used as zero,
because the sensor does not sit at exactly 0.00 Nm with nobody touching it.

On release we do NOT stop transmitting. We keep sending 0x169 at the same
cadence with LKA_ACTIVE clear, tracking whatever angle the driver puts the
wheel at - the same thing the real camera's frames do at rest. Going silent
would drop LKAS_STATUS and the EPS would report its camera missing.

Note this is deliberately different from what openpilot does. openpilot
scales its torque down and keeps steering, so a nudge on the motorway does
not disengage. We let go completely, which is what was asked for and is the
right behaviour for a parked demonstration.

USAGE
-----
One position:
python steer_position_manualoverride.py --angle -20 --seconds 10

A sequence, which is the demo:
python steer_position_manualoverride.py --sequence "15,0,-15,0" --hold 4

Each entry is an absolute wheel angle in degrees. The wheel ramps to it,
holds for --hold seconds, then moves to the next. A per-waypoint table at
the end shows commanded against reached.

NOTES FROM THE WORKING RUNS
---------------------------
- it lands about 1.2 deg short at a 15 deg move. The EPS pulls until
  self-centering balances the torque cap, and opendbc's own comment on
  LKAS_MAX_TORQUE = 1 is "a value of 1 is easy to overpower". Expect
  bigger targets to fall further short; --torque raises it
- the dash will show chassis and camera warnings while the relay is open.
  The EPS accepts and acts on commands anyway - confirmed 2026-11-04.
  They clear on an ignition cycle
- we capture ~90-100 Hz of the camera's ~120 Hz and it works anyway, so
  the dropped frames are not worth chasing

sudo systemctl stop comma
python steer_position.py --sequence "15,0,-15,0" --hold 4
"""

import argparse
import atexit
import time
from collections import deque

from manual_can_interface import CANInterface

LKAS_ADDR = 0x169
EPS_ADDR = 0x185
CAM_BUS = 2
CAR_BUS = 0

LKAS_STATUS_NAMES = {
    5: "INACTIVE", 6: "ACTIVE", 9: "FAULT",
    11: "undocumented-11", 14: "NO-CAMERA",
}

# DRIVER OVERRIDE - taken from openpilot, not invented here.
#
# opendbc/car/nissan/carstate.py:11 TORQUE_SAMPLES = 12
# opendbc/car/nissan/values.py:24 STEER_THRESHOLD = 1.0
# carstate.py:100-102
# self.steeringTorqueSamples.append(ret.steeringTorque)
# # Filtering driver torque to prevent steeringPressed false positives
# ret.steeringPressed = bool(abs(sum(self.steeringTorqueSamples)
# / TORQUE_SAMPLES) > STEER_THRESHOLD)
#
# The average over 12 samples is what stops a bump or a bit of noise from
# tripping it. A single sample would false-trigger constantly.
TORQUE_SAMPLES = 12
STEER_THRESHOLD = 1.0  # Nm

# NOTE on how openpilot itself reacts, which is NOT what we do here.
# carcontroller.py scales its torque down instead of letting go:
#
# # Start scaling torque at STEER_THRESHOLD down to 0.2. If we don't scale
# # this low, EPS will temp fault from high driver and LKAS torque
# lkas_max_torque = max(LKAS_MIN_TORQUE,
# LKAS_MAX_TORQUE - 0.6 * max(0, abs(torque) - STEER_THRESHOLD))
#
# It keeps steering with less force so a driver nudge does not disengage on
# the motorway. We want the opposite: the person takes over, fully, and the
# car stops steering. That also sidesteps the EPS temp fault the comment
# warns about, because our torque goes to zero rather than fighting.

# A real frame from this car's camera, 2026-10-28. The encoder is checked
# against it before the script will touch the car.
# 7f 82 00 80 00 80 05 e7 = +4.32 deg, torque 0, inactive, counter 5
REFERENCE_FRAME = bytes.fromhex("7f820080008005e7")
REFERENCE_ANGLE = 4.32
REFERENCE_COUNTER = 5


def _gen_crc8_table(poly):
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return table


_CRC8J1850 = _gen_crc8_table(0x1D)


def nissan_checksum(data):
    """opendbc's mk_crc8_fun(CRC8J1850, init_crc=0x00, xor_out=0xFF).

    init_reg = init_crc ^ xor_out = 0xFF, so the register seeds at 0xFF
    despite the argument being named init_crc=0x00.
    """
    crc = 0xFF
    for b in data:
        crc = _CRC8J1850[crc ^ b]
    return crc ^ 0xFF


def encode_lkas(angle_deg, torque_nm, active, counter):
    """Build 0x169. DESIRED_ANGLE is 18 bits big-endian across bytes 0, 1 and
    the top 2 bits of byte 2, scale -0.01, offset 1310.
    """
    raw = int(round((1310.0 - angle_deg) * 100.0))
    raw = max(0, min(raw, (1 << 18) - 1))

    data = bytearray(8)
    data[0] = (raw >> 10) & 0xFF
    data[1] = (raw >> 2) & 0xFF
    data[2] = (raw & 0x03) << 6
    data[3] = 0x80  # SET_0x80_2
    data[4] = int(round(torque_nm * 100.0)) & 0xFF if active else 0x00
    data[5] = 0x80  # SET_0x80
    data[6] = ((1 if active else 0) << 4) | (counter & 0x0F)
    data[7] = nissan_checksum(bytes(data[:7]))
    return bytes(data)


def lkas_counter(data):
    return data[6] & 0x0F


def decode_185(data):
    if len(data) < 8:
        return None
    raw = (data[2] << 10) | (data[3] << 2) | (data[4] >> 6)
    return 1310.0 - (raw * 0.01), (data[6] >> 4) & 0x0F


def driver_torque(data):
    """STEER_TORQUE_DRIVER out of 0x185, in Nm.

    DBC: SG_ STEER_TORQUE_DRIVER : 7|12@0+ (-0.01,20.47) [0|4095] "Nm"

    Start bit 7 big-endian, 12 bits long: all of byte 0, then the top nibble
    of byte 1. Scale -0.01, offset 20.47.
    """
    if len(data) < 2:
        return None
    raw = (data[0] << 4) | (data[1] >> 4)
    return 20.47 - raw * 0.01


def unpack(msg):
    """panda reports the bus with flag bits in the top nibble - mask them."""
    if len(msg) == 3:
        addr, data, bus = msg[0], msg[1], msg[2]
    else:
        addr, data, bus = msg[0], msg[2], msg[3]
    return addr, bytes(data), bus & 0x0F


def self_test():
    built = encode_lkas(REFERENCE_ANGLE, 0.0, False, REFERENCE_COUNTER)
    ok = built == REFERENCE_FRAME
    print("encoder self-test against a real frame from this car's camera:")
    print(f" camera sent : {' '.join(f'{b:02x}' for b in REFERENCE_FRAME)}")
    print(f" we build : {' '.join(f'{b:02x}' for b in built)}")
    print(f" -> {'MATCH' if ok else 'MISMATCH'}")
    if not ok:
        print()
        print("Refusing to run - the encoder cannot reproduce a frame we")
        print("captured from this car.")
    return ok


class Sender:
    """Batch where possible. One-at-a-time starves RX on the SPI link."""

    def __init__(self, panda):
        self.panda = panda
        self.many = None
        self.errors = 0

    def send(self, frames):
        if not frames:
            return
        if self.many is not False:
            try:
                self.panda.can_send_many([(a, d, b) for a, d, b in frames])
                self.many = True
                return
            except TypeError:
                self.many = False
            except Exception:
                self.many = False
        for addr, data, bus in frames:
            try:
                self.panda.can_send(addr, data, bus)
            except Exception:
                self.errors += 1


def parse_sequence(text):
    out = []
    for piece in text.replace(" ", "").split(","):
        if not piece:
            continue
        out.append(float(piece))
    return out


def main():
    p = argparse.ArgumentParser(
        description="command the steering to absolute angles")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--angle", type=float, metavar="DEG",
                   help="one absolute wheel angle. Positive is LEFT")
    g.add_argument("--sequence", metavar="LIST",
                   help='absolute angles to walk through, e.g. "15,0,-15,0"')
    p.add_argument("--hold", type=float, default=4.0, metavar="SEC",
                   help="sequence mode: seconds to hold each position "
                        "once reached (default 4)")
    p.add_argument("--seconds", type=float, default=10.0, metavar="SEC",
                   help="single-angle mode: how long to hold (default 10)")
    p.add_argument("--torque", type=float, default=1.0, metavar="NM",
                   help="MAX_TORQUE. opendbc ships 1.0, which its own comment "
                        "calls easy to overpower")
    p.add_argument("--rate", type=float, default=100.0, metavar="DEG_S",
                   help="ramp rate (default 100). The panda's own limit at "
                        "0 m/s is 5 deg per frame, about 500 deg/s")
    p.add_argument("--max-angle", type=float, default=600.0, metavar="DEG",
                   help="refuse any target beyond this. Default 600, which "
                        "is opendbc's own ceiling (NISSAN_STEERING_LIMITS "
                        ".max_angle = 60000 at 0.01 deg/count). Note that 600 "
                        "is PAST this car's mechanical steering lock, so it "
                        "is a sanity limit, not a reachable position")
    p.add_argument("--listen", type=float, default=4.0, metavar="SEC")
    p.add_argument("--override", type=float, default=STEER_THRESHOLD,
                   metavar="NM",
                   help=f"driver torque that counts as taking over, averaged "
                        f"over {TORQUE_SAMPLES} samples (default "
                        f"{STEER_THRESHOLD}, openpilot's own STEER_THRESHOLD). "
                        f"Lower is more sensitive")
    p.add_argument("--no-override", action="store_true",
                   help="disable driver override. Do not use with anyone "
                        "near the wheel")
    args = p.parse_args()

    targets = ([args.angle] if args.angle is not None
               else parse_sequence(args.sequence))
    if not targets:
        p.error("no angles given")
    for t in targets:
        if abs(t) > abs(args.max_angle):
            p.error(f"target {t:+.1f} exceeds --max-angle "
                    f"{args.max_angle:.0f}")
    if not 0.0 <= args.torque <= 2.5:
        p.error("--torque must be 0 to 2.5")

    hold_each = args.seconds if args.angle is not None else args.hold
    if hold_each > 30:
        p.error("30 s per position is plenty")

    print()
    if not self_test():
        return
    print()

    can = CANInterface(simulation=False, allow_output=True)
    panda = can.panda
    sender = Sender(panda)

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
            print(" Camera may still be isolated. Ignition cycle the car.")

    atexit.register(close_once)

    # ---- baseline, relay closed
    print(f"baseline - relay CLOSED, listening {args.listen:.1f} s")
    can.set_relay(False)
    time.sleep(0.6)

    start_angle = None
    status = None
    rest_torques = []
    end = time.monotonic() + args.listen
    while time.monotonic() < end:
        for m in panda.can_recv():
            addr, data, _bus = unpack(m)
            if addr == EPS_ADDR:
                d = decode_185(data)
                if d is not None:
                    start_angle, status = d
                t = driver_torque(data)
                if t is not None:
                    rest_torques.append(t)

    if start_angle is None:
        print(" no 0x185 - is the car awake? Is pandad stopped?")
        return

    print(f" wheel {start_angle:+.2f} deg, LKAS_STATUS {status} "
          f"({LKAS_STATUS_NAMES.get(status, '?')})")

    # The torque sensor does not read exactly zero at rest, so take the
    # resting value as our zero and measure the driver against that.
    torque_zero = (sum(rest_torques) / len(rest_torques)) if rest_torques else 0.0
    if rest_torques:
        spread = max(rest_torques) - min(rest_torques)
        print(f" driver torque at rest {torque_zero:+.2f} Nm "
              f"(spread {spread:.2f}) - using that as zero")
        if spread > args.override and not args.no_override:
            print(f" !! resting noise is wider than the {args.override:.2f} Nm")
            print(" override threshold. Something is touching the wheel,")
            print(" or the threshold is too low. It will trip instantly.")
    elif not args.no_override:
        print(" !! no torque readings - override cannot work. Stopping.")
        return
    if status == 14:
        print(" (14 means the EPS has not finished recovering from an")
        print(" earlier isolation. It clears on its own in a couple of")
        print(" minutes - wait, or the run will be hard to read.)")

    print()
    print("plan - ABSOLUTE positions, positive is LEFT:")
    prev = start_angle
    total = 0.0
    for i, t in enumerate(targets, 1):
        travel = abs(t - prev)
        ramp_s = travel / max(1e-6, args.rate)
        total += ramp_s + hold_each
        print(f" {i}. {prev:+7.2f} -> {t:+7.2f} "
              f"({travel:5.2f} deg, ramp {ramp_s:4.2f} s, hold "
              f"{hold_each:.1f} s)")
        prev = t
    print(f" MAX_TORQUE {args.torque:.2f}, ramp {args.rate:.0f} deg/s")
    print(f" about {total:.0f} s in total")

    biggest = max(abs(t) for t in targets)
    if biggest > 180.0:
        print()
        print(f" !! largest target is {biggest:.0f} deg. A Leaf is roughly")
        print(" 2.5-3 turns lock to lock, so somewhere past ~450-540 the")
        print(" wheel is against its mechanical stop and the EPS is just")
        print(" pulling at it. Harmless at this torque, but it will read")
        print(" as a large error rather than a reached position.")
        print(" Work up in steps and watch the error column instead of")
        print(" jumping straight to the ceiling.")
    if args.no_override:
        print()
        print(" !! DRIVER OVERRIDE DISABLED. The wheel will keep being")
        print(" driven no matter who grabs it. Nobody near the wheel.")
    else:
        print(f" driver override ON at {args.override:.2f} Nm averaged over "
              f"{TORQUE_SAMPLES} samples")
        print(" grab the wheel and the car lets go, for the rest of the run")
    print()
    print("CAR IN PARK. WHEELS CHOCKED.")
    if input("type GO to continue: ").strip() != "GO":
        return

    print()
    print("opening relay - camera isolated, we are its only route to the car")
    can.set_relay(True)
    time.sleep(0.4)

    results = []
    commanded = start_angle
    seen_statuses = {}
    measured = start_angle
    cam_169 = 0
    fwd = 0
    t0 = time.monotonic()

    # Driver override state. Once released we stay released - the person has
    # taken over, and quietly grabbing control back is the last thing a car
    # should do.
    torque_window = deque([0.0] * TORQUE_SAMPLES, TORQUE_SAMPLES)
    released = {"at": None, "torque": 0.0, "angle": None, "target": None}
    peak_torque = 0.0

    try:
        for index, target in enumerate(targets, 1):
            print()
            print(f"--- {index}/{len(targets)} to {target:+.2f} deg")
            leg_start = measured
            phase = "ramp"
            hold_until = None
            last_report = 0.0
            leg_t0 = time.monotonic()
            # generous ceiling so a stalled ramp cannot hang the run
            ramp_deadline = (leg_t0 + abs(target - commanded)
                             / max(1e-6, args.rate) + 6.0)
            last_tick = time.monotonic()

            while True:
                now = time.monotonic()
                dt = now - last_tick
                last_tick = now

                if phase == "ramp":
                    step = args.rate * dt
                    if commanded < target:
                        commanded = min(target, commanded + step)
                    elif commanded > target:
                        commanded = max(target, commanded - step)
                    if abs(commanded - target) < 0.01 or now > ramp_deadline:
                        commanded = target
                        phase = "hold"
                        hold_until = now + hold_each
                        print(f" at command {target:+.2f}, holding "
                              f"{hold_each:.1f} s")
                elif now >= hold_until:
                    break

                # Once the driver has it, finish this leg quietly: keep
                # streaming inactive frames for a moment so the EPS sees a
                # clean handover, then stop. Remaining waypoints are skipped.
                if released["at"] is not None:
                    if released.get("stop_at") is None:
                        released["stop_at"] = now + 1.0
                    elif now >= released["stop_at"]:
                        break

                out = []
                for m in panda.can_recv():
                    addr, data, bus = unpack(m)

                    if addr == EPS_ADDR:
                        d = decode_185(data)
                        if d is not None:
                            measured, st = d
                            seen_statuses[st] = seen_statuses.get(st, 0) + 1

                        t = driver_torque(data)
                        if t is not None:
                            applied = t - torque_zero
                            torque_window.append(applied)
                            avg = abs(sum(torque_window) / TORQUE_SAMPLES)
                            peak_torque = max(peak_torque, avg)
                            if (not args.no_override
                                    and released["at"] is None
                                    and avg > args.override):
                                released["at"] = now - t0
                                released["torque"] = avg
                                released["angle"] = measured
                                released["target"] = target
                                print()
                                print(f" *** DRIVER OVERRIDE at "
                                      f"{avg:.2f} Nm ***")
                                print(f" releasing the wheel at "
                                      f"{measured:+.2f} deg. You have it.")

                    if bus != CAM_BUS:
                        continue

                    if addr == LKAS_ADDR:
                        cam_169 += 1
                        if released["at"] is not None:
                            # Keep streaming, but inactive and tracking the
                            # wheel wherever the driver puts it - exactly what
                            # the camera's own frames do at rest. Going silent
                            # instead would drop LKAS_STATUS and make the EPS
                            # think its camera vanished.
                            out.append((LKAS_ADDR,
                                        encode_lkas(measured, 0.0, False,
                                                    lkas_counter(data)),
                                        CAR_BUS))
                        else:
                            out.append((LKAS_ADDR,
                                        encode_lkas(commanded, args.torque,
                                                    True, lkas_counter(data)),
                                        CAR_BUS))
                        continue

                    out.append((addr, data, CAR_BUS))
                    fwd += 1

                sender.send(out)

                if now - last_report >= 1.0:
                    last_report = now
                    st = (max(seen_statuses, key=seen_statuses.get)
                          if seen_statuses else None)
                    avg_now = abs(sum(torque_window) / TORQUE_SAMPLES)
                    tag = "RELEASED" if released["at"] is not None else phase
                    print(f" t+{now - leg_t0:5.1f}s {tag:<8} "
                          f"cmd {commanded:+7.2f} wheel {measured:+7.2f} "
                          f"err {target - measured:+6.2f} "
                          f"driver {avg_now:4.2f} Nm "
                          f"status {st} ({LKAS_STATUS_NAMES.get(st, '?')})")

            results.append((target, leg_start, measured))
            print(f" reached {measured:+.2f} "
                  f"(error {target - measured:+.2f} deg)")

            if released["at"] is not None:
                remaining = len(targets) - index
                if remaining:
                    print(f" skipping the remaining {remaining} position"
                          f"{'s' if remaining != 1 else ''} - the driver has "
                          f"the wheel")
                break

    except KeyboardInterrupt:
        print("\ninterrupted")

    finally:
        close_once()
        elapsed = max(1e-6, time.monotonic() - t0)

        print()
        print("=== result ===")
        print(f"elapsed {elapsed:.1f} s "
              f"camera 0x169 {cam_169} ({cam_169 / elapsed:.1f} Hz of ~120) "
              f"forwarded {fwd}")
        if sender.errors:
            print(f"send errors: {sender.errors}")

        if results:
            print()
            print(f" {'#':>2} {'from':>8} {'commanded':>10} "
                  f"{'reached':>8} {'error':>7} {'moved':>7}")
            for i, (target, leg_start, reached) in enumerate(results, 1):
                print(f" {i:>2} {leg_start:+8.2f} {target:+10.2f} "
                      f"{reached:+8.2f} {target - reached:+7.2f} "
                      f"{reached - leg_start:+7.2f}")

        print()
        print("LKAS_STATUS seen:")
        for st, n in sorted(seen_statuses.items()):
            print(f" {st:>2} ({LKAS_STATUS_NAMES.get(st, '?')}): {n}")

        print()
        if args.no_override:
            print("driver override was DISABLED for this run")
        elif released["at"] is None:
            print(f"driver override armed at {args.override:.2f} Nm and never "
                  f"tripped")
            print(f" highest driver torque seen: {peak_torque:.2f} Nm")
        else:
            print(f"DRIVER TOOK OVER at t+{released['at']:.1f}s")
            print(f" torque {released['torque']:.2f} Nm against a "
                  f"{args.override:.2f} Nm threshold")
            print(f" released at {released['angle']:+.2f} deg, on the way "
                  f"to {released['target']:+.2f}")
            print(" we kept sending inactive frames after that, so the EPS")
            print(" saw a clean handover rather than its camera going quiet")

        if results and released["at"] is None:
            worst = max(abs(t - r) for t, _s, r in results)
            print()
            if worst < 2.0:
                print(f"Every position reached within {worst:.2f} deg. The")
                print("steering is following absolute commands.")
            else:
                print(f"Worst error {worst:.2f} deg. If the larger targets")
                print("are the ones falling short, that is the torque cap -")
                print("try --torque 1.5 and compare the error column.")

        print()
        can.print_summary()


if __name__ == "__main__":
    main()
