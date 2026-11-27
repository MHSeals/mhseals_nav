"""The heading guard: a healthy compass passes, a field swing or a jump is held on the gyro.

A new FCU heading is taken only after it has stood, and only at a bounded slew.
"""
import math
from types import SimpleNamespace as S

import pytest

from mhseals_nav.heading_guard import HeadingGuard, guarded, quaternion, wrap, yaw_of
from mhseals_nav.odometry_configuration import resolve_odometry

DT = 0.01   # 100 Hz, as MAVROS sends ATTITUDE_QUATERNION and RAW_IMU


def params(**override):
    return {**resolve_odometry().guard_parameters, **override}


def run(guard, start, seconds, heading, rate=0.0, field=1.0):
    """Feed ``seconds`` of samples from ``start``; ``heading`` and ``field`` may be callables.

    Return [(t, output heading, extra variance)].
    """
    out, t = [], start
    while t < start + seconds - 1e-9:
        f = field(t) if callable(field) else field
        guard.mag(t, 0.25 * f, 0.0, 0.4 * f)
        h = heading(t) if callable(heading) else heading
        out.append((t, *guard.imu(t, h, rate)))
        t = round(t + DT, 6)
    return out


def seeded(p=None, heading=0.3):
    guard = HeadingGuard(p or params())
    run(guard, 0.0, 6.0, heading)      # past the seed: the usual field is known
    return guard


def test_a_healthy_compass_passes_straight_through_even_while_turning():
    guard = seeded()
    rate = math.radians(20)
    rows = run(guard, 6.0, 20.0, lambda t: wrap(0.3 + rate * (t - 6.0)), rate=rate)
    assert guard.trusted and guard.reason == 'ok'
    assert all(abs(wrap(h - wrap(0.3 + rate * (t - 6.0)))) < 1e-6 for t, h, _ in rows)
    assert all(extra is None for *_, extra in rows)


def test_a_heading_jump_without_rotation_is_held_then_taken_slowly_once_it_has_lasted():
    p = params()
    guard = seeded(p)
    jumped = 0.3 + math.radians(140)                                # the 11-27 jump
    rows = run(guard, 6.0, 0.5, 0.3) + run(guard, 6.5, 45.0, jumped)
    jump = [r for r in rows if r[0] >= 6.5]
    held = [r for r in jump if r[0] < 6.5 + p['window'] + p['recover'] + p['accept']]
    assert all(abs(wrap(h - 0.3)) < 1e-6 for _, h, _ in held)       # the gyro's heading, held
    assert not guard.trusted or guard.reason == 'ok'
    steps = [abs(wrap(b[1] - a[1])) for a, b in zip(jump, jump[1:])]
    assert max(steps) <= p['slew'] * DT + 1e-9                      # never a step, only a slew
    assert abs(wrap(rows[-1][1] - jumped)) < math.radians(1)        # finally taken


def test_a_field_swing_distrusts_at_once_and_trust_returns_only_after_recover():
    p = params()
    guard = seeded(p)
    burst = (8.0, 10.0)
    def swing(t):                       # the 80 % burst of 11-27
        return 1.9 if burst[0] <= t < burst[1] else 1.0

    def wander(t):                      # small: under the jump check
        return 0.3 + (0.05 if burst[0] <= t < burst[1] else 0.0)
    rows = run(guard, 6.0, 10.0, wander, field=swing)
    during = [r for r in rows if burst[0] + DT <= r[0] < burst[1]]
    assert during and all(abs(wrap(h - 0.3)) < 1e-6 for _, h, _ in during)
    assert all(extra is not None for *_, extra in during)
    extras = [extra for *_, extra in during]
    assert extras == sorted(extras) and extras[-1] > extras[0]      # the yaw sigma grows
    after = [r for r in rows if r[0] >= burst[1]]
    first_trusted = next(t for t, _, extra in after if extra is None)
    assert first_trusted == pytest.approx(burst[1] + p['recover'], abs=2 * DT)


def test_without_a_magnetometer_only_the_jump_check_guards():
    guard = HeadingGuard(params())
    rows = [(t, *guard.imu(t, 0.3, 0.0)) for t in (i * DT for i in range(300))]
    assert guard.trusted and 'no magnetometer' in guard.reason
    assert all(abs(wrap(h - 0.3)) < 1e-6 for _, h, _ in rows)
    held = [guard.imu(3.0 + i * DT, 0.3 + math.radians(90), 0.0)[0] for i in range(10)]
    assert all(abs(wrap(h - 0.3)) < 1e-6 for h in held) and not guard.trusted


def test_the_heading_wraps_across_plus_minus_pi():
    start = math.pi - 0.2
    guard = seeded(heading=start)
    rate = math.radians(30)
    rows = run(guard, 6.0, 2.0, lambda t: wrap(start + rate * (t - 6.0)), rate=rate)
    assert guard.trusted and rows[-1][1] < 0                        # crossed to the -pi side
    assert all(abs(wrap(h - wrap(start + rate * (t - 6.0)))) < 1e-6 for t, h, _ in rows)


def test_the_imu_message_keeps_roll_pitch_and_rates_and_takes_the_guarded_yaw():
    guard = seeded()
    x, y, z, w = quaternion(0.05, -0.03, 0.3)
    message = S(orientation=S(x=x, y=y, z=z, w=w), orientation_covariance=[0.04] * 9,
                angular_velocity=S(x=0.1, y=0.2, z=0.0), linear_acceleration=S(x=0., y=0., z=9.8))
    out = guarded(message, guard, 6.0)
    roll, pitch, yaw = yaw_of(out.orientation)
    assert (roll, pitch, yaw) == pytest.approx((0.05, -0.03, 0.3), abs=1e-9)
    assert out.angular_velocity == message.angular_velocity and out is not message
    x, y, z, w = quaternion(0.05, -0.03, 0.3 + math.radians(140))
    message.orientation = S(x=x, y=y, z=z, w=w)
    out = guarded(message, guard, 6.01)
    assert yaw_of(out.orientation)[2] == pytest.approx(0.3, abs=1e-9)          # held
    assert out.orientation_covariance[8] >= 0.04 and message.orientation_covariance[8] == 0.04


def test_a_long_bent_field_holds_at_most_hold_then_follows_slowly_and_refuses_jumps():
    p = params(hold=5.0)
    guard = seeded(p)
    drift = math.radians(2)             # the FCU heading creeps 2 deg/s in a bent field
    rows = run(guard, 6.0, 40.0, lambda t: 0.3 + drift * (t - 6.0), field=3.0)
    held = [r for r in rows if r[0] < 6.0 + p['hold']]
    assert all(abs(wrap(h - 0.3)) < 1e-6 for _, h, _ in held)                   # held for ``hold``
    assert guard.following and not guard.trusted
    assert abs(wrap(rows[-1][1] - (0.3 + drift * 40.0))) < math.radians(1)      # caught up, slowly
    steps = [abs(wrap(b[1] - a[1])) for a, b in zip(rows, rows[1:])]
    assert max(steps) <= p['slew'] * DT + drift * DT + 1e-9
    late = 6.0 + p['hold'] + DT
    extras = {round(e, 12) for t, _, e in rows if e is not None and t > late}
    assert len(extras) == 1                                         # the variance stops growing
    before = rows[-1][1]
    jump = run(guard, 46.0, 0.5, lambda t: 0.3 + drift * (t - 6.0) + math.radians(90), field=3.0)
    assert all(abs(wrap(h - before)) < math.radians(2) for _, h, _ in jump)   # still refused


def test_a_jump_while_following_never_passes_through_as_a_step():
    p = params(hold=2.0)
    guard = seeded(p)
    rows = run(guard, 6.0, 20.0, 0.3, field=3.0)        # bent field: held, then following
    assert guard.following
    rows += run(guard, 26.0, 30.0, 0.3 + math.radians(120), field=3.0)   # jumps while following
    steps = [abs(wrap(b[1] - a[1])) for a, b in zip(rows, rows[1:])]
    assert max(steps) <= p['slew'] * DT + 1e-9                    # slewed, never stepped
