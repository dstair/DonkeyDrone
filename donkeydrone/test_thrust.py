#!/usr/bin/env python3
"""
Thrust / yaw test: validate BF SITL + Gazebo behavior.

Modes (`--mode`):
    thrust   Ramp throttle 1000→2000 PWM, 0.5s per step. Used to find
             hover PWM and tune `motorConstant` in model.sdf.
    yaw      Hold throttle=1000 (motors should be idle) and apply a
             full yaw stick. Watch for altitude climb — that's the
             "yaw shoots drone into sky" bug. With AIRMODE disabled
             in BF, altitude should stay ~0.
    yaw-airborne
             Climb to altitude first, then apply yaw at hover throttle,
             then cut CH3 to 1000 while keeping yaw. Checks the user-
             reported bug: "after yaw input, drone climbs and won't
             come down even with throttle at 1000."
    hover    Fine sweep near hover (default 1450-1550, 5 PWM steps,
             2s per step) with altitude rate per PWM. Pick the PWM
             closest to 0 m/s rate as your DRONE_HOVER_THROTTLE.
    both     Run yaw test first, then thrust ramp (default).

Easiest way to run (brings up the stack, runs the test, tears it down):

    ./scripts/test_thrust.sh
    ./scripts/test_thrust.sh --mode=yaw

This test sends its OWN RC packets to BetaFlight SITL on UDP 9004, so
drone_manage.py must NOT be running — the two would fight over the port.
"""

import argparse
import logging
import socket
import struct
import time

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
logger = logging.getLogger(__name__)

RC_HOST = "127.0.0.1"
RC_PORT = 9004

POSE_TOPIC = "/world/drone_course/dynamic_pose/info"


def send_rc(rc_sock, channels):
    """Send 40-byte RC packet: timestamp + 16 x uint16."""
    timestamp = time.time()
    packet = struct.pack("<d", timestamp)
    for ch in channels:
        packet += struct.pack("<H", int(ch))
    rc_sock.sendto(packet, (RC_HOST, RC_PORT))


def make_channels(throttle_pwm, armed=True, yaw_pwm=1500):
    """Create 16-channel RC packet with throttle, yaw, and arm/disarm."""
    channels = [1000] * 16
    channels[0] = 1500  # CH1 roll (centered)
    channels[1] = 1500  # CH2 pitch (centered - no forward tilt)
    channels[2] = throttle_pwm  # CH3 throttle
    channels[3] = yaw_pwm  # CH4 yaw
    channels[4] = 2000 if armed else 1000  # CH5 AUX1 arm switch
    channels[5] = 2000  # CH6 AUX2 = angle mode
    return channels


_pose_state = {"node": None, "latest": None}


def _init_pose_subscriber():
    """Subscribe once at startup; the callback keeps the latest drone pose in
    a shared slot. Previous per-call subscription missed most samples because
    the 150ms settle wasn't long enough for a fresh subscription to receive.
    """
    if _pose_state["node"] is not None:
        return True
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
    except Exception as e:
        logger.warning("gz-transport not available: %s", e)
        return False

    node = Node()

    def on_pose(msg):
        for p in msg.pose:
            if "betaloop" in p.name.lower():
                _pose_state["latest"] = (p.position.x, p.position.y, p.position.z)
                return

    if not node.subscribe(Pose_V, POSE_TOPIC, on_pose):
        logger.warning("Could not subscribe to %s", POSE_TOPIC)
        return False

    _pose_state["node"] = node
    return True


def query_pose():
    """Return latest drone pose (x, y, z) or None if unknown."""
    return _pose_state["latest"]


def hold(rc_sock, channels, duration_s):
    """Send channels at 50Hz for duration_s seconds."""
    n = int(duration_s / 0.02)
    for _ in range(n):
        send_rc(rc_sock, channels)
        time.sleep(0.02)


def arm_sequence(rc_sock):
    """Disarm→arm boot sequence BF SITL requires to clear NOT_DISARMED flag."""
    logger.info("Sending disarm packets (1s)...")
    hold(rc_sock, make_channels(1000, armed=False), 1.0)
    logger.info("Arming and idling...")
    hold(rc_sock, make_channels(1000, armed=True), 1.0)


def run_yaw_test(rc_sock, throttle_pwm=1000, yaw_pwm=2000, duration_s=3.0):
    """Test whether yaw input causes altitude to climb at the given throttle.

    At throttle=1000 this validates AIRMODE-disabled state (motors idle, yaw
    should do nothing). At throttle=hover_pwm this validates the quadrotor
    motor-mixer asymmetry (yaw intrinsically adds thrust via ω²)."""
    logger.info("=== Yaw bleed test: throttle=%d, yaw=%d, %.1fs ===",
                throttle_pwm, yaw_pwm, duration_s)

    # Baseline: same throttle, yaw centered
    logger.info("Baseline: throttle=%d, yaw centered, 2s", throttle_pwm)
    hold(rc_sock, make_channels(throttle_pwm, armed=True, yaw_pwm=1500), 2.0)
    baseline_alt_start = query_pose()[2] if query_pose() else None
    hold(rc_sock, make_channels(throttle_pwm, armed=True, yaw_pwm=1500), duration_s)
    baseline_alt_end = query_pose()[2] if query_pose() else None
    baseline_delta = (
        baseline_alt_end - baseline_alt_start
        if baseline_alt_start is not None and baseline_alt_end is not None
        else None
    )
    logger.info("Baseline: %s → %s (delta %s)",
                f"{baseline_alt_start:.3f}m" if baseline_alt_start is not None else "?",
                f"{baseline_alt_end:.3f}m" if baseline_alt_end is not None else "?",
                f"{baseline_delta:+.3f}m" if baseline_delta is not None else "?")

    # Reset throttle low briefly to try to settle back down (only effective
    # at low altitudes; above ~0.5m the drone keeps coasting upward).
    logger.info("Resetting (CH3=1000, 2s)...")
    hold(rc_sock, make_channels(1000, armed=True), 2.0)

    # Yaw applied
    logger.info("Yaw applied: throttle=%d, yaw=%d, sampling altitude %.1fs...",
                throttle_pwm, yaw_pwm, duration_s)
    hold(rc_sock, make_channels(throttle_pwm, armed=True, yaw_pwm=1500), 1.0)  # settle
    yaw_alt_start = query_pose()[2] if query_pose() else None
    samples = []
    n_samples = int(duration_s / 0.25)
    for i in range(n_samples):
        hold(rc_sock, make_channels(throttle_pwm, armed=True, yaw_pwm=yaw_pwm), 0.25)
        pos = query_pose()
        alt = pos[2] if pos else None
        samples.append(((i + 1) * 0.25, alt))

    yaw_alt_end = samples[-1][1] if samples else None
    yaw_delta = (
        yaw_alt_end - yaw_alt_start
        if yaw_alt_start is not None and yaw_alt_end is not None
        else None
    )

    for t, alt in samples:
        print(f"  yaw=on t={t:.2f}s alt={alt:.3f}m" if alt is not None
              else f"  yaw=on t={t:.2f}s alt=?")

    logger.info("--- Yaw test summary ---")
    logger.info("Baseline delta: %s",
                f"{baseline_delta:+.3f}m" if baseline_delta is not None else "?")
    logger.info("Yaw-applied delta: %s (start=%s end=%s)",
                f"{yaw_delta:+.3f}m" if yaw_delta is not None else "?",
                f"{yaw_alt_start:.3f}m" if yaw_alt_start is not None else "?",
                f"{yaw_alt_end:.3f}m" if yaw_alt_end is not None else "?")
    if baseline_delta is not None and yaw_delta is not None:
        bleed = yaw_delta - baseline_delta
        verdict = "YAW BLEED (thrust rises on yaw)" if bleed > 0.2 else "clean"
        logger.info("Yaw-induced altitude bleed: %+.3fm → %s", bleed, verdict)


def _sample_alt_over(rc_sock, channels, duration_s, dt=0.25):
    """Hold `channels` for duration_s, sampling altitude every dt seconds."""
    samples = []
    n = int(duration_s / dt)
    for i in range(n):
        hold(rc_sock, channels, dt)
        pos = query_pose()
        alt = pos[2] if pos else None
        samples.append(((i + 1) * dt, alt))
    return samples


def _print_samples(label, samples):
    for t, alt in samples:
        print(f"  {label} t={t:.2f}s alt={alt:.3f}m" if alt is not None
              else f"  {label} t={t:.2f}s alt=?")


def run_yaw_airborne_test(rc_sock, hover_pwm=1495, climb_pwm=1600,
                          climb_s=4.0, phase_s=3.0, yaw_pwm=2000):
    """Reproduce "yaw while airborne → won't descend" bug.

    Phases (arm sequence already done by main):
      A. Climb to altitude at `climb_pwm` for `climb_s` seconds.
      B. Baseline descent: CH3=1000, yaw centered, sample altitude for phase_s.
         Re-climb after so phase C starts from altitude too.
      C. Yaw at hover: CH3=hover_pwm, yaw=yaw_pwm — does it climb vs. baseline hover?
      D. Yaw + throttle cut: CH3=1000, yaw=yaw_pwm — does it descend at all?

    The bug manifests as: phase D altitude stays ~flat or keeps climbing,
    while phase B descends normally.
    """
    logger.info("=== Yaw-airborne test (hover=%d climb=%d yaw=%d) ===",
                hover_pwm, climb_pwm, yaw_pwm)

    # --- Phase A: climb ---
    logger.info("A: climbing at CH3=%d for %.1fs", climb_pwm, climb_s)
    a_samples = _sample_alt_over(rc_sock, make_channels(climb_pwm, armed=True), climb_s)
    _print_samples("A-climb", a_samples)
    alt_after_climb = a_samples[-1][1] if a_samples else None
    logger.info("A: altitude after climb = %s",
                f"{alt_after_climb:.3f}m" if alt_after_climb is not None else "?")

    # --- Phase B: baseline descent, no yaw ---
    logger.info("B: baseline descent, CH3=1000 yaw=centered, %.1fs", phase_s)
    pos = query_pose()
    b_start = pos[2] if pos else None
    b_samples = _sample_alt_over(rc_sock, make_channels(1000, armed=True, yaw_pwm=1500), phase_s)
    _print_samples("B-cut-no-yaw", b_samples)
    b_end = b_samples[-1][1] if b_samples else None

    # --- Re-climb so phase C/D start from altitude again ---
    logger.info("Re-climbing at CH3=%d for %.1fs", climb_pwm, climb_s)
    hold(rc_sock, make_channels(climb_pwm, armed=True), climb_s)
    pos = query_pose()
    alt_after_reclimb = pos[2] if pos else None
    logger.info("Altitude after re-climb = %s",
                f"{alt_after_reclimb:.3f}m" if alt_after_reclimb is not None else "?")

    # --- Phase C: yaw at hover throttle ---
    logger.info("C: yaw at hover, CH3=%d yaw=%d, %.1fs", hover_pwm, yaw_pwm, phase_s)
    pos = query_pose()
    c_start = pos[2] if pos else None
    c_samples = _sample_alt_over(rc_sock, make_channels(hover_pwm, armed=True, yaw_pwm=yaw_pwm), phase_s)
    _print_samples("C-yaw-hover", c_samples)
    c_end = c_samples[-1][1] if c_samples else None

    # --- Phase D: throttle cut while keeping yaw ---
    logger.info("D: throttle cut WITH yaw held, CH3=1000 yaw=%d, %.1fs", yaw_pwm, phase_s)
    pos = query_pose()
    d_start = pos[2] if pos else None
    d_samples = _sample_alt_over(rc_sock, make_channels(1000, armed=True, yaw_pwm=yaw_pwm), phase_s)
    _print_samples("D-cut-yaw-held", d_samples)
    d_end = d_samples[-1][1] if d_samples else None

    def delta(s, e):
        return e - s if s is not None and e is not None else None

    b_d = delta(b_start, b_end)
    c_d = delta(c_start, c_end)
    d_d = delta(d_start, d_end)

    logger.info("--- Yaw-airborne summary ---")
    logger.info("B baseline (cut, no yaw):   start=%s end=%s delta=%s",
                f"{b_start:.3f}m" if b_start is not None else "?",
                f"{b_end:.3f}m" if b_end is not None else "?",
                f"{b_d:+.3f}m" if b_d is not None else "?")
    logger.info("C yaw at hover:             start=%s end=%s delta=%s",
                f"{c_start:.3f}m" if c_start is not None else "?",
                f"{c_end:.3f}m" if c_end is not None else "?",
                f"{c_d:+.3f}m" if c_d is not None else "?")
    logger.info("D cut WHILE yaw held:       start=%s end=%s delta=%s",
                f"{d_start:.3f}m" if d_start is not None else "?",
                f"{d_end:.3f}m" if d_end is not None else "?",
                f"{d_d:+.3f}m" if d_d is not None else "?")

    # Phase C is the primary signal: at hover PWM, yaw should NOT add net
    # thrust. Any significant climb in C means the motor mixer is producing
    # more-than-hover thrust when yaw is commanded — the "yaw shoots drone
    # into sky" bug.
    if c_d is not None:
        if c_d > 1.0:
            logger.info("Verdict C (yaw at hover): BUG — climbed %+.2fm at hover+yaw", c_d)
        elif c_d < -1.0:
            logger.info("Verdict C (yaw at hover): yaw reduces thrust (%+.2fm)", c_d)
        else:
            logger.info("Verdict C (yaw at hover): clean (%+.2fm)", c_d)
    if b_d is not None and d_d is not None:
        # Phase D interpretation depends on whether climb came from motors
        # or just momentum carried over from C. In sim, drag is low so a
        # drone with upward velocity coasts for a while even at CH3=1000.
        if d_d > 1.0:
            logger.info("Verdict D (throttle cut + yaw): still climbing %+.2fm — "
                        "likely phase-C momentum carry-over, not a new thrust bug",
                        d_d)
        elif d_d > b_d * 0.5:
            logger.info("Verdict D: partial descent (%+.3f vs baseline %+.3f)", d_d, b_d)
        else:
            logger.info("Verdict D: descends similarly to baseline")


def run_hover_sweep(rc_sock, pwm_low=1450, pwm_high=1550, step=5, hold_s=2.0):
    """Fine-grained sweep near hover. Reports altitude rate (m/s) per PWM so
    you can pick DRONE_HOVER_THROTTLE = the PWM closest to 0 m/s rate.

    Each step:
      1. Reset to CH3=1000 for 1s to let the drone settle on the ground.
      2. Hold target PWM for hold_s seconds.
      3. Sample altitude at start and end; rate = (end - start) / hold_s.
    """
    logger.info("=== Hover sweep: %d-%d PWM, step %d, hold %.1fs ===",
                pwm_low, pwm_high, step, hold_s)
    results = []
    for pwm in range(pwm_low, pwm_high + 1, step):
        # Settle back to ground so each step starts from the same state
        hold(rc_sock, make_channels(1000, armed=True), 1.0)
        pos_start = query_pose()
        alt_start = pos_start[2] if pos_start else None

        hold(rc_sock, make_channels(pwm, armed=True), hold_s)
        pos_end = query_pose()
        alt_end = pos_end[2] if pos_end else None

        if alt_start is not None and alt_end is not None:
            rate = (alt_end - alt_start) / hold_s
            results.append((pwm, alt_start, alt_end, rate))
            print(f"  pwm={pwm} alt {alt_start:.3f}→{alt_end:.3f}m  rate={rate:+.3f} m/s")
        else:
            results.append((pwm, alt_start, alt_end, None))
            print(f"  pwm={pwm} alt ?→?  rate=?")

    # Find PWM with smallest absolute rate (closest to true hover)
    valid = [r for r in results if r[3] is not None]
    if valid:
        best = min(valid, key=lambda r: abs(r[3]))
        logger.info("--- Hover sweep summary ---")
        logger.info("Best hover candidate: PWM=%d (rate=%+.3f m/s)", best[0], best[3])
        logger.info("Set DRONE_HOVER_THROTTLE = %d in drone_config.py", best[0])
    else:
        logger.info("No valid samples — check pose subscription.")


def run_thrust_ramp(rc_sock):
    """Ramp throttle 1000→2000, measure altitude at each step."""
    logger.info("=== Thrust ramp test ===")
    logger.info("Ramping throttle from 1000 to 2000...")
    throttle_levels = list(range(1000, 2001, 50))

    results = []
    for pwm in throttle_levels:
        hold(rc_sock, make_channels(pwm, armed=True), 0.5)
        try:
            pos = query_pose()
            if pos:
                alt = pos[2]
                results.append((pwm, alt))
                print(f"throttle={pwm}, altitude={alt:.3f}m")
            else:
                results.append((pwm, None))
                print(f"throttle={pwm}, altitude=?")
        except Exception as e:
            results.append((pwm, None))
            print(f"throttle={pwm}, error: {e}")

    logger.info("--- Thrust ramp summary ---")
    hover_pwm = None
    for pwm, alt in results:
        if alt is not None and alt > 0.2:
            if hover_pwm is None:
                hover_pwm = pwm
            print(f"  {pwm} PWM -> {alt:.3f}m")
    if hover_pwm:
        logger.info("Approximate hover at ~%d PWM", hover_pwm)
    else:
        logger.info("No clear hover detected.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["thrust", "yaw", "yaw-airborne", "hover", "both"], default="both",
                        help="Which test(s) to run (default: both)")
    parser.add_argument("--hover-low", type=int, default=1450)
    parser.add_argument("--hover-high", type=int, default=1550)
    parser.add_argument("--hover-step", type=int, default=5)
    parser.add_argument("--yaw-throttle", type=int, default=1000,
                        help="CH3 PWM during yaw test (default 1000 = airmode check; use hover PWM to test motor-mixer asymmetry)")
    parser.add_argument("--yaw-pwm", type=int, default=2000,
                        help="CH4 PWM during yaw test (default 2000 = full right)")
    parser.add_argument("--airborne-hover", type=int, default=1495,
                        help="Hover PWM for yaw-airborne test")
    parser.add_argument("--airborne-climb", type=int, default=1550,
                        help="Climb PWM for yaw-airborne test (just above hover)")
    parser.add_argument("--airborne-climb-s", type=float, default=2.0)
    parser.add_argument("--airborne-phase-s", type=float, default=3.0)
    args = parser.parse_args()

    logger.info("Opening RC socket...")
    rc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    logger.info("Subscribing to pose topic...")
    _init_pose_subscriber()

    logger.info("Waiting for BetaFlight to come up...")
    time.sleep(2)

    # Pump idle disarm packets for a few seconds before arming. BF can sit
    # with ARMING_DISABLED_RX_FAILSAFE and ARMING_DISABLED_ANGLE set for
    # several seconds after (re)boot — until RC packets flow and the attitude
    # estimator converges from Gazebo IMU. 1s isn't always enough.
    logger.info("Pumping idle packets for 6s to clear failsafe/attitude...")
    hold(rc_sock, make_channels(1000, armed=False), 6.0)

    arm_sequence(rc_sock)

    if args.mode in ("yaw", "both"):
        run_yaw_test(rc_sock, throttle_pwm=args.yaw_throttle, yaw_pwm=args.yaw_pwm)

    if args.mode == "yaw-airborne":
        run_yaw_airborne_test(rc_sock,
                              hover_pwm=args.airborne_hover,
                              climb_pwm=args.airborne_climb,
                              climb_s=args.airborne_climb_s,
                              phase_s=args.airborne_phase_s,
                              yaw_pwm=args.yaw_pwm)

    if args.mode == "hover":
        run_hover_sweep(rc_sock, args.hover_low, args.hover_high, args.hover_step)

    if args.mode in ("thrust", "both"):
        run_thrust_ramp(rc_sock)

    logger.info("Disarming...")
    hold(rc_sock, make_channels(1000, armed=False), 0.5)
    rc_sock.close()


if __name__ == "__main__":
    main()
