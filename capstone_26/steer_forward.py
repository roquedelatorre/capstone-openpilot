"""Steering with the camera isolated AND its messages forwarded by us.

WHY THIS EXISTS
---------------
The 2026-10-28 census settled the mechanism. Two facts:

  1. allOutput with the default param DISABLES the panda's own forwarding:

         // opendbc/safety/modes/defaults.h
         const uint16_t ALLOUTPUT_PARAM_PASSTHROUGH = 1;
         return (safety_config){NULL, 0, NULL, 0, !alloutput_passthrough};
                                                  ^^^^^^^^^^^^^^^^^^^^^^
                                                  disable_forwarding

     param 0 -> disable_forwarding = true. Every test we ran was in this
     mode, so opening the relay cut the camera off completely and the EPS
     reported LKAS_STATUS 14, which simply means "no camera".

  2. nissan safety mode WOULD forward in firmware, and blocks exactly the
     camera's 0x169/0x2b1/0x4cc so openpilot can replace them. But it gates
     TX on controls_allowed, which comes from the car's own cruise state, and
     a Leaf will not engage cruise in park. Closed path for a parked demo.

So we do in Python what nissan mode does in firmware: open the relay, forward
the camera's messages from bus 2 to bus 0, and substitute our own 0x169.

THE SUBSTITUTION TRICK
----------------------
Rather than running our own 100 Hz timer, we replace each camera 0x169 the
moment it arrives, reusing ITS counter value. The EPS then sees the same
120 Hz cadence, the same phase and an unbroken counter sequence it has always
seen - we only change the angle, the torque and the active bit. That removes
rate and jitter from the list of things that could explain a failure, which
cost us several runs already.

MODES - run passthrough FIRST
-----------------------------
  --mode passthrough   (default) forward everything unchanged, including the
                       camera's own 0x169. Sends nothing of our own. If this
                       does not hold LKAS_STATUS at 5, our forwarder is too
                       slow or too lossy and no steering result would mean
                       anything. THIS IS THE VALIDATION STEP.

  --mode steer         forward everything except 0x169, and substitute ours.

SELF-TEST
---------
Before touching the car this verifies its own encoder against a frame really
captured from this car's camera on 2026-10-28:

    7f 82 00 80 00 80 05 e7   = +4.32 deg, torque 0, inactive, counter 5

If the encoder cannot reproduce that byte for byte, the script exits. The
checksum is CRC-8/SAE-J1850 as opendbc computes it: table poly 0x1D, register
seeded 0xFF (which is how init_crc=0x00 ^ xor_out=0xFF works out), final
XOR 0xFF.

    sudo systemctl stop comma
    python steer_forward.py --mode passthrough --seconds 15
    python steer_forward.py --mode steer --delta 15 --seconds 12

The dash will warn while the relay is open. It clears on an ignition cycle,
and LKAS_STATUS 14 recovers on its own within a few minutes - we watched it
do exactly that.
"""

import argparse
import atexit
import time

from manual_can_interface import CANInterface

LKAS_ADDR = 0x169
EPS_ADDR = 0x185
CAM_BUS = 2
CAR_BUS = 0

LKAS_STATUS_NAMES = {5: "INACTIVE", 6: "ACTIVE", 9: "FAULT",
                     11: "undocumented-11", 14: "NO-CAMERA"}

# A real frame from this car's camera, with its decoded meaning. The encoder
# is checked against this before the script will run.
REFERENCE_FRAME = bytes.fromhex("7f820080008005e7")
REFERENCE_ANGLE = 4.32
REFERENCE_TORQUE = 0.0
REFERENCE_ACTIVE = False
REFERENCE_COUNTER = 5


# ---------------------------------------------------------------- checksum

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
    """Exactly opendbc's mk_crc8_fun(CRC8J1850, init_crc=0x00, xor_out=0xFF).

    init_reg = init_crc ^ xor_out = 0x00 ^ 0xFF = 0xFF, so the register is
    seeded 0xFF despite the argument being named init_crc=0x00.
    """
    crc = 0xFF
    for b in data:
        crc = _CRC8J1850[crc ^ b]
    return crc ^ 0xFF


# ---------------------------------------------------------------- 0x169

def encode_lkas(angle_deg, torque_nm, active, counter):
    """Build 0x169. Verified against a real camera frame - see REFERENCE_FRAME.

    DESIRED_ANGLE is 18 bits big-endian across bytes 0,1 and the top 2 bits
    of byte 2, scale -0.01, offset 1310.
    """
    raw = int(round((1310.0 - angle_deg) * 100.0))
    raw = max(0, min(raw, (1 << 18) - 1))

    data = bytearray(8)
    data[0] = (raw >> 10) & 0xFF
    data[1] = (raw >> 2) & 0xFF
    data[2] = (raw & 0x03) << 6
    data[3] = 0x80                                    # SET_0x80_2
    data[4] = int(round(torque_nm * 100.0)) & 0xFF if active else 0x00
    data[5] = 0x80                                    # SET_0x80
    data[6] = ((1 if active else 0) << 4) | (counter & 0x0F)
    data[7] = nissan_checksum(bytes(data[:7]))
    return bytes(data)


def decode_lkas_angle(data):
    raw = (data[0] << 10) | (data[1] << 2) | (data[2] >> 6)
    return 1310.0 - raw * 0.01


def lkas_counter(data):
    return data[6] & 0x0F


def decode_185(data):
    if len(data) < 8:
        return None
    raw = (data[2] << 10) | (data[3] << 2) | (data[4] >> 6)
    return 1310.0 - (raw * 0.01), (data[6] >> 4) & 0x0F


def self_test():
    """Reproduce a real camera frame, or refuse to run."""
    built = encode_lkas(REFERENCE_ANGLE, REFERENCE_TORQUE,
                        REFERENCE_ACTIVE, REFERENCE_COUNTER)
    ok = built == REFERENCE_FRAME
    print("encoder self-test against a real frame from this car's camera:")
    print(f"   camera sent : {' '.join(f'{b:02x}' for b in REFERENCE_FRAME)}")
    print(f"   we build    : {' '.join(f'{b:02x}' for b in built)}")
    print(f"   -> {'MATCH' if ok else 'MISMATCH'}")
    if not ok:
        print()
        print("Refusing to run. If the encoder cannot reproduce a frame we")
        print("captured from the car, nothing it sends can be trusted.")
    return ok


# ---------------------------------------------------------------- panda io

def unpack(msg):
    """panda reports the bus with flag bits in the top nibble - mask them."""
    if len(msg) == 3:
        addr, data, bus = msg[0], msg[1], msg[2]
    else:
        addr, data, bus = msg[0], msg[2], msg[3]
    return addr, bytes(data), bus & 0x0F


class Sender:
    """Batch sender that works out which panda API this build exposes."""

    def __init__(self, panda):
        self.panda = panda
        self.many = None        # None = not yet decided, True/False after
        self.errors = 0

    def send(self, frames):
        """frames: list of (addr, data, bus). Returns number sent."""
        if not frames:
            return 0
        if self.many is not False:
            try:
                self.panda.can_send_many([(a, d, b) for a, d, b in frames])
                self.many = True
                return len(frames)
            except TypeError:
                self.many = False
            except Exception:
                # a real bus error, not an API mismatch - fall through to
                # one-at-a-time so a single bad frame cannot drop the batch
                self.many = False

        sent = 0
        for addr, data, bus in frames:
            try:
                self.panda.can_send(addr, data, bus)
                sent += 1
            except Exception:
                self.errors += 1
        return sent


# ---------------------------------------------------------------- main

def main():
    p = argparse.ArgumentParser(
        description="forward the camera's messages and substitute our 0x169")
    p.add_argument("--mode", choices=("passthrough", "steer"),
                   default="passthrough",
                   help="passthrough forwards everything unchanged and sends "
                        "nothing of our own - run this FIRST to validate the "
                        "forwarder (default). steer substitutes our 0x169")
    p.add_argument("--delta", type=float, default=15.0, metavar="DEG",
                   help="steer mode: degrees from the current angle "
                        "(default 15)")
    p.add_argument("--torque", type=float, default=1.0, metavar="NM",
                   help="steer mode: MAX_TORQUE, opendbc ships 1.0")
    p.add_argument("--rate", type=float, default=100.0, metavar="DEG_S",
                   help="steer mode: ramp rate (default 100). The panda's "
                        "own limit at 0 m/s is 5 deg per frame, so 500 deg/s")
    p.add_argument("--seconds", type=float, default=12.0)
    p.add_argument("--listen", type=float, default=4.0, metavar="SEC",
                   help="baseline listen with the relay closed (default 4)")
    args = p.parse_args()

    if abs(args.delta) > 40:
        p.error("keep --delta under 40 deg")
    if args.seconds > 30:
        p.error("30 s is plenty")
    if not 0.0 <= args.torque <= 2.5:
        p.error("--torque must be 0 to 2.5")

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
            print("   Camera may still be isolated. Ignition cycle the car.")

    atexit.register(close_once)

    # ---- baseline, relay closed
    print(f"baseline - relay CLOSED, listening {args.listen:.1f} s")
    can.set_relay(False)
    time.sleep(0.6)

    start_angle = None
    status = None
    end = time.monotonic() + args.listen
    while time.monotonic() < end:
        for msg in panda.can_recv():
            addr, data, _bus = unpack(msg)
            if addr == EPS_ADDR:
                decoded = decode_185(data)
                if decoded is not None:
                    start_angle, status = decoded

    if start_angle is None:
        print("   no 0x185 - is the car awake? Is pandad stopped?")
        return

    print(f"   wheel {start_angle:+.2f} deg, LKAS_STATUS {status} "
          f"({LKAS_STATUS_NAMES.get(status, '?')})")
    if status == 14:
        print("   (14 at rest means the EPS has not finished recovering from")
        print("    an earlier isolation. Wait a couple of minutes - it clears")
        print("    on its own - or the run will be hard to interpret.)")

    target = start_angle + args.delta

    print()
    print(f"mode: {args.mode}")
    if args.mode == "passthrough":
        print("   forwarding bus 2 -> bus 0 unchanged, including the camera's")
        print("   own 0x169. WE SEND NOTHING OF OUR OWN.")
        print("   Success here = LKAS_STATUS stays 5, not 14. That proves the")
        print("   forwarder keeps the EPS believing the camera is present.")
    else:
        print(f"   forwarding bus 2 -> bus 0, substituting 0x169")
        print(f"   {start_angle:+.2f} -> {target:+.2f} deg, MAX_TORQUE "
              f"{args.torque:.2f}, ramp {args.rate:.0f} deg/s")
        print("   our frame reuses the camera's counter, so the EPS sees the")
        print("   same cadence and an unbroken sequence")
    print(f"   holding {args.seconds:.1f} s")
    print()
    print("CAR IN PARK. WHEELS CHOCKED. HANDS OFF THE WHEEL.")
    if input("type GO to continue: ").strip() != "GO":
        return

    print()
    print("opening relay - camera isolated, we are now its only route to the car")
    can.set_relay(True)
    time.sleep(0.4)

    # ---- the forwarding loop
    fwd_counts = {}
    subbed = 0
    seen_statuses = {}
    max_dev = 0.0
    current = start_angle
    last_ramp = time.monotonic()
    last_report = time.monotonic()
    measured = start_angle
    cam_169 = 0

    t0 = time.monotonic()
    end = t0 + args.seconds

    try:
        while time.monotonic() < end:
            now = time.monotonic()

            # ramp our commanded angle
            if args.mode == "steer":
                step = args.rate * (now - last_ramp)
                last_ramp = now
                if current < target:
                    current = min(target, current + step)
                elif current > target:
                    current = max(target, current - step)

            out = []
            for msg in panda.can_recv():
                addr, data, bus = unpack(msg)

                if addr == EPS_ADDR:
                    decoded = decode_185(data)
                    if decoded is not None:
                        measured, st = decoded
                        seen_statuses[st] = seen_statuses.get(st, 0) + 1
                        max_dev = max(max_dev, abs(measured - start_angle))

                if bus != CAM_BUS:
                    continue

                if addr == LKAS_ADDR:
                    cam_169 += 1
                    if args.mode == "steer":
                        # substitute, reusing the camera's counter
                        out.append((LKAS_ADDR,
                                    encode_lkas(current, args.torque, True,
                                                lkas_counter(data)),
                                    CAR_BUS))
                        subbed += 1
                        continue
                    # passthrough: forward the camera's own frame untouched

                out.append((addr, data, CAR_BUS))
                fwd_counts[addr] = fwd_counts.get(addr, 0) + 1

            sender.send(out)

            if now - last_report >= 1.0:
                last_report = now
                st = max(seen_statuses, key=seen_statuses.get) if seen_statuses else None
                print(f"  t+{now - t0:5.1f}s  fwd {sum(fwd_counts.values()):6d}"
                      f"  sub {subbed:5d}  cam169 {cam_169:5d}"
                      f"  wheel {measured:+7.2f}  moved {abs(measured - start_angle):5.2f}"
                      f"  status {st} ({LKAS_STATUS_NAMES.get(st, '?')})")

    except KeyboardInterrupt:
        print("\ninterrupted")

    finally:
        close_once()
        elapsed = max(1e-6, time.monotonic() - t0)

        print()
        print("=== result ===")
        print(f"elapsed {elapsed:.1f} s")
        print(f"camera 0x169 seen: {cam_169} ({cam_169 / elapsed:.1f} Hz - the")
        print(f"                   camera runs ~120 Hz, so well under that")
        print(f"                   means we are dropping frames)")
        print(f"frames forwarded : {sum(fwd_counts.values())} "
              f"({sum(fwd_counts.values()) / elapsed:.0f} /s)")
        if args.mode == "steer":
            print(f"0x169 substituted: {subbed} ({subbed / elapsed:.1f} Hz)")
        if sender.errors:
            print(f"send errors      : {sender.errors}")
        print(f"send path        : "
              f"{'can_send_many' if sender.many else 'can_send one at a time'}")

        print()
        print("forwarded by address:")
        for addr, n in sorted(fwd_counts.items()):
            print(f"   0x{addr:03x}  {n:6d}  {n / elapsed:6.1f} Hz")

        print()
        print("LKAS_STATUS seen:")
        for st, n in sorted(seen_statuses.items()):
            print(f"   {st:>2} ({LKAS_STATUS_NAMES.get(st, '?')}): {n}")

        print()
        print(f"max wheel deviation: {max_dev:.2f} deg")

        statuses = set(seen_statuses)
        if args.mode == "passthrough":
            if 14 in statuses and 5 not in statuses and 6 not in statuses:
                print()
                print("FORWARDER FAILED. The EPS still reports no camera, so")
                print("our Python relay is too slow or too lossy. Check the")
                print("camera 0x169 rate above - if it is well under 120 Hz we")
                print("are not keeping up, and a steer run would prove nothing.")
            elif 5 in statuses or 6 in statuses:
                print()
                print("FORWARDER WORKS. The EPS believes the camera is present")
                print("while we are the only thing connecting them. Now run:")
                print("   python steer_forward.py --mode steer --delta 15")
            else:
                print()
                print("No usable status samples. Re-run before concluding.")
        else:
            if max_dev >= 1.0:
                print()
                print(f"The wheel moved {max_dev:.2f} deg. Did anyone touch it?")
                print("Re-run twice before believing it.")
            elif 14 in statuses:
                print()
                print("Still reporting no camera - the forwarder did not hold")
                print("up under the extra load of substituting. Run")
                print("--mode passthrough and compare the forward rates.")
            else:
                print()
                print("No motion, but the EPS stayed out of its no-camera")
                print("state. That is the cleanest negative available to us:")
                print("verified frame, camera present, correct bus, sole")
                print("source of 0x169, matched cadence and counter. Software")
                print("is exhausted - the next step is a scan tool on the EPS.")

        print()
        can.print_summary()


if __name__ == "__main__":
    main()