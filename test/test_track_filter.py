"""Pure CV track filter: convergence, association, hysteresis, decay, lifetime."""

import copy
import math

import numpy as np
import pytest

from tracker_fixtures import CFG, fixed_pose, frame, ob, yawing_pose
from mhseals_nav.track_filter import (
    MOTION_MOVING, MOTION_STATIC, MOTION_UNKNOWN, SOURCE_LIVE, SOURCE_PRIOR, TrackFilter)

RATE = 15.0


def run(f, scene, seconds, start=0.0, pose_at=fixed_pose, noise=0.0, seed=7):
    """scene(t) -> list of observations; returns the last time stepped."""
    rng = np.random.default_rng(seed)
    t = start
    for k in range(int(seconds * RATE)):
        t = start + (k + 1) / RATE
        obs = scene(t)
        for o in obs:
            o.position.x += rng.normal(0, noise)
            o.position.y += rng.normal(0, noise)
        f.step(frame(t, obs), t, pose_at)
    return t


def test_cv_filter_converges_on_a_moving_boat():
    f = TrackFilter(CFG)
    run(f, lambda t: [ob('yellow_target_boat', 5 + 0.8 * t, 2.0)], 10.0, noise=0.05)
    (track,) = f.tracks
    assert track.x[2] == pytest.approx(0.8, abs=0.1)
    assert abs(track.x[3]) < 0.1
    assert track.motion == MOTION_MOVING
    assert track.x[0] == pytest.approx(5 + 0.8 * 10.0, abs=0.2)


def test_candidate_confirms_after_hits_and_dies_unconfirmed():
    f = TrackFilter(CFG)
    f.step(frame(1.0, [ob('red_buoy', 5, 0)]), 1.0, fixed_pose)
    assert f.tracks == [] and len(f.all) == 1
    f.step(frame(2.5, []), 2.5, fixed_pose)
    assert f.all == []                                 # candidate.timeout
    for k in range(CFG['candidate']['hits']):
        f.step(frame(3 + k / RATE, [ob('red_buoy', 5, 0)]), 3 + k / RATE, fixed_pose)
    assert len(f.tracks) == 1 and f.tracks[0].motion == MOTION_STATIC


def test_id_pass_beats_a_nearer_track_without_the_id():
    f = TrackFilter(CFG)
    run(f, lambda t: [ob('red_buoy', 5.0, 0.0, 'a'), ob('red_buoy', 5.0, 1.2, 'b')], 1.0)
    by_id = {t.backend_id: t.id for t in f.tracks}
    # 'b' now reported next to track 'a': the id still decides, within gate_chi2_id.
    t = 1.0 + 1 / RATE
    f.step(frame(t, [ob('red_buoy', 5.0, 0.55, 'b'), ob('red_buoy', 5.0, 1.0, 'a')]),
           t, fixed_pose)
    assert {tr.backend_id: tr.id for tr in f.tracks} == {'a': by_id['b'], 'b': by_id['a']} or \
        {tr.id: tr.backend_id for tr in f.tracks} == {by_id['a']: 'a', by_id['b']: 'b'}
    assert f.assoc['id'] > 0
    track_b = next(tr for tr in f.tracks if tr.id == by_id['b'])
    assert track_b.backend_id == 'b'


def test_gate_pass_prefers_the_nearest_pairs_globally():
    f = TrackFilter(CFG)
    run(f, lambda t: [ob('red_buoy', 5.0, 0.0), ob('red_buoy', 5.0, 2.0)], 1.0)
    ids = sorted((tr.x[1], tr.id) for tr in f.tracks)
    t = 1.0 + 1 / RATE
    f.step(frame(t, [ob('red_buoy', 5.0, 1.9), ob('red_buoy', 5.0, 0.1)]), t, fixed_pose)
    assert sorted((round(tr.x[1]), tr.id) for tr in f.tracks) == [(0, ids[0][1]), (2, ids[1][1])]


@pytest.mark.parametrize('rng, gap', [(10.0, 2.0), (15.0, 3.0), (20.0, 4.0), (20.0, 2.0)])
def test_second_same_class_buoy_beside_a_tracked_one_gets_its_own_track(rng, gap):
    f = TrackFilter(CFG)
    t = run(f, lambda t: [ob('red_buoy', rng, 0.0)], 2.0, noise=0.05)
    # The neighbour comes into view: the first track takes its own observation, so it
    # must not count against the neighbour's exclusion gate.
    run(f, lambda t: [ob('red_buoy', rng, 0.0), ob('red_buoy', rng, gap)], 2.0, start=t,
        noise=0.05)
    assert sorted(round(tr.x[1] / gap) for tr in f.tracks) == [0, 1]


def test_class_mismatch_does_not_associate():
    f = TrackFilter(CFG)
    run(f, lambda t: [ob('red_buoy', 5.0, 0.0)], 1.0)
    run(f, lambda t: [ob('green_buoy', 5.0, 0.0)], 1.0, start=1.0)
    assert sorted(t.class_id for t in f.tracks) == ['green_buoy', 'red_buoy']


def test_hysteresis_static_to_moving_and_back():
    cfg = copy.deepcopy(CFG)
    f = TrackFilter(cfg)
    run(f, lambda t: [ob('red_buoy', 5.0, 0.0)], 3.0)
    (track,) = f.tracks
    assert track.motion == MOTION_STATIC
    move_s = cfg['class_groups']['buoy']['move_s']
    # A buoy that starts drifting at 1 m/s: moving only after move_s of evidence.
    flips = []
    t = run(f, lambda t: [ob('red_buoy', 5.0 + 1.0 * (t - 3.0), 0.0)], 8.0, start=3.0)
    assert track.motion == MOTION_MOVING
    run(f, lambda t: [ob('red_buoy', 5.0 + 1.0 * 8.0, 0.0)], 20.0, start=t)
    assert track.motion == MOTION_STATIC
    assert move_s > 1 / RATE and not flips


def test_unknown_class_needs_n_unknown_min_hits():
    cfg = copy.deepcopy(CFG)
    cfg['motion']['n_unknown_min'] = 50
    f = TrackFilter(cfg)
    run(f, lambda t: [ob('mystery', 5.0, 0.0)], 1.0)
    (track,) = f.tracks
    assert track.hits < 50 and track.motion == MOTION_UNKNOWN
    run(f, lambda t: [ob('mystery', 5.0, 0.0)], 4.0, start=1.0)
    assert track.motion == MOTION_STATIC


def test_static_wobble_within_floor_stays_static():
    f = TrackFilter(CFG)
    run(f, lambda t: [ob('red_buoy', 5.0 + 0.1 * math.sin(t), 0.0)], 30.0, noise=0.05)
    (track,) = f.tracks
    assert track.motion == MOTION_STATIC
    assert track.speed_bound >= CFG['motion']['floor_mps']


def test_confidence_decays_with_tau_and_timeout_drops():
    f = TrackFilter(CFG)
    t = run(f, lambda t: [ob('yellow_target_boat', 5 + t, 0.0)], 3.0)
    (track,) = f.tracks
    tau = CFG['tau']['moving']
    assert f.confidence(track, t + tau) == pytest.approx(track.score * math.exp(-1), rel=1e-6)
    f.expire(t + CFG['timeout']['moving'] + 0.01)
    assert f.tracks == []


def test_static_track_unseen_in_fov_drops_and_out_of_fov_survives():
    f = TrackFilter(CFG)
    t = run(f, lambda t: [ob('red_buoy', 5.0, 0.0), ob('red_buoy', -5.0, 0.0)], 2.0)
    # Only the one ahead (inside the FOV) is now missing.
    t = run(f, lambda t: [], CFG['miss_in_fov_s'] + 0.5, start=t)
    assert [round(tr.x[0]) for tr in f.tracks] == [-5]
    assert t > 0


def test_prior_adopted_on_first_hit_keeps_id():
    f = TrackFilter(CFG)
    prior = f.add_prior('red_buoy', (5.0, 0.0), 0.3, 0.0)
    assert prior.source == SOURCE_PRIOR and prior.motion == MOTION_STATIC and prior.hits == 0
    assert f.confidence(prior, 0.0) <= CFG['course_map']['prior_confidence']
    f.step(frame(0.1, [ob('red_buoy', 5.2, 0.1)]), 0.1, fixed_pose)
    assert [(t.id, t.source) for t in f.tracks] == [(prior.id, SOURCE_LIVE)]


def test_reordered_stamp_and_missing_tf_are_dropped():
    f = TrackFilter(CFG)
    assert f.step(frame(2.0, [ob('red_buoy', 5, 0)]), 2.0, fixed_pose)
    assert not f.step(frame(1.0, [ob('red_buoy', 5, 0)]), 2.0, fixed_pose)

    def no_tf(t, frame_id):
        raise LookupError(frame_id)
    assert not f.step(frame(3.0, [ob('red_buoy', 5, 0)]), 3.0, no_tf)


def test_backend_velocity_is_used_when_enabled():
    cfg = copy.deepcopy(CFG)
    cfg['use_backend_velocity'] = True
    f = TrackFilter(cfg)
    run(f, lambda t: [ob('yellow_target_boat', 5 + t, 0.0, velocity=(1.0, 0.0))], 0.5)
    (track,) = f.tracks
    assert track.x[2] == pytest.approx(1.0, abs=0.1)


def test_coasting_static_track_holds_its_last_measured_position():
    f = TrackFilter(CFG)
    # Behind the camera, so the in-FOV miss rule does not drop it while it coasts.
    t = run(f, lambda t: [ob('red_buoy', -8.0, 2.0)], 3.0)
    (track,) = f.tracks
    assert track.motion == MOTION_STATIC
    track.x[2:] = (0.4, -0.3)          # a lag- or noise-induced velocity estimate
    seen, P0 = track.seen_xy.copy(), track.P[0, 0]
    assert seen == pytest.approx([-8.0, 2.0], abs=0.05)
    t = run(f, lambda t: [], 10.0, start=t)
    assert track.x[:2] == pytest.approx(seen, abs=1e-9)
    assert track.seen_xy == pytest.approx(seen, abs=1e-9)
    assert track.P[0, 0] > P0                       # uncertainty still grows
    run(f, lambda t: [ob('red_buoy', -8.0, 2.0)], 1.0, start=t)
    assert [tr.id for tr in f.tracks] == [track.id]


def test_coasting_moving_track_still_predicts():
    f = TrackFilter(CFG)
    t = run(f, lambda t: [ob('yellow_target_boat', -8.0 - 0.8 * t, 2.0)], 3.0)
    (track,) = f.tracks
    x0 = track.x[0]
    run(f, lambda t: [], 1.0, start=t)
    assert track.x[0] == pytest.approx(x0 - 0.8, abs=0.15)


def pause_then_cross(speed, pause=4.0, rng=10.0):
    """Cross, pause long enough to turn STATIC, then cross again at ``speed``."""
    def y(t):
        return 0.8 * min(t, 2.0) + speed * max(t - 2.0 - pause, 0.0)
    return lambda t: [ob('yellow_target_boat', rng, -3.0 + y(t))]


@pytest.mark.parametrize('speed', [0.5, 0.65, 0.8])
@pytest.mark.parametrize('rate', [10.0, 15.0, 30.0])
def test_boat_that_pauses_then_crosses_turns_moving(speed, rate, monkeypatch):
    monkeypatch.setitem(globals(), 'RATE', rate)
    f = TrackFilter(CFG)
    t = run(f, pause_then_cross(speed), 6.0, noise=0.05)
    (track,) = f.tracks
    assert track.motion == MOTION_STATIC          # the pause was long enough
    moving_at = None
    while t < 30.0 and moving_at is None:
        t = run(f, pause_then_cross(speed), 0.5, start=t, noise=0.05)
        if track.motion == MOTION_MOVING:
            moving_at = t
    assert moving_at is not None and moving_at - 6.0 <= 6.0, moving_at


@pytest.mark.parametrize('rate', [10.0, 15.0, 30.0])
def test_hysteresis_is_counted_in_seconds(rate, monkeypatch):
    monkeypatch.setitem(globals(), 'RATE', rate)
    f = TrackFilter(CFG)
    t = run(f, lambda t: [ob('yellow_target_boat', 10.0, 0.0)], 1.0)
    (track,) = f.tracks
    static_s = CFG['class_groups']['boat']['static_s']
    while track.motion != MOTION_STATIC:
        t = run(f, lambda t: [ob('yellow_target_boat', 10.0, 0.0)], 1 / rate, start=t)
    # Static evidence starts once the young bound is narrow enough; from then on the
    # flip takes static_s of updates whatever the frame rate.
    assert track.static_s == pytest.approx(static_s, abs=1.01 / rate)


def reversal_yaw(rate=0.4, start=3.0, ramp=3.0):
    """Yaw of a turn that reverses: +rate, a linear ramp to -rate, then -rate."""
    ts = np.arange(0.0, 30.0, 1e-3)
    r = rate * (1 - 2 * np.clip((ts - start) / ramp, 0, 1))
    yaw = np.concatenate([[0.0], np.cumsum(r[:-1]) * 1e-3])
    return lambda t: float(np.interp(t, ts, yaw))


@pytest.mark.parametrize('lag', [0.2, 0.3])
def test_young_boat_stays_moving_through_a_lagged_yaw_reversal(lag):
    """A lag fakes rho*lag*dr/dt, largest where the yaw rate crosses zero (R3 in §8)."""
    yaw = reversal_yaw()
    pose_at = yawing_pose(yaw)
    rng = np.random.default_rng(7)
    f = TrackFilter(CFG)
    motions = []
    for k in range(int(10.0 * RATE)):
        t = 1.5 + (k + 1) / RATE
        world = np.array([8.0, -4.0 + 0.8 * (t - 1.5)])
        c, s = math.cos(yaw(t - lag)), math.sin(yaw(t - lag))   # heading lags truth
        local = np.array([[c, s], [-s, c]]) @ world + rng.normal(0, 0.05, 2)
        f.step(frame(t, [ob('yellow_target_boat', *local)]), t, pose_at)
        motions += [(t, tr.motion) for tr in f.tracks]
    assert motions and all(m == MOTION_MOVING for t, m in motions if t > motions[0][0] + 2.0)


def test_young_boat_track_does_not_turn_static():
    f = TrackFilter(CFG)
    run(f, lambda t: [ob('yellow_target_boat', 8 + 0.8 * t, 3.0)], 6.0, noise=0.05)
    (track,) = f.tracks
    assert track.motion == MOTION_MOVING and track.static_s == 0
