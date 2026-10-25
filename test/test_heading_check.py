"""Heading lag recovery from static-track residuals."""

import math

import numpy as np
import pytest

from tracker_fixtures import CFG, frame, ob, yawing_pose
from mhseals_nav.heading_check import ERROR, OK, WARN, HeadingCheck
from mhseals_nav.track_filter import TrackFilter

HC = CFG['heading_check']


def synthetic(tau, n=600, tracks=4, seed=3, noise=0.002):
    rng = np.random.default_rng(seed)
    check = HeadingCheck(HC)
    for k in range(n):
        t = k / 15
        r = 0.4 * math.sin(0.5 * t)
        check.add(t, -tau * r + rng.normal(0, noise), r, k % tracks)
    return check, t


@pytest.mark.parametrize('tau', [0.0, 0.05, 0.1, 0.3])
def test_lag_recovered_within_20_ms(tau):
    check, t = synthetic(tau)
    result = check.result(t)
    assert result['excited']
    assert result['lag_s'] == pytest.approx(tau, abs=0.02)


def test_levels():
    assert synthetic(0.0)[0].result(40)['level'] == OK
    assert synthetic(0.1)[0].result(40)['level'] == WARN
    result = synthetic(0.3)[0].result(40)
    assert result['level'] == ERROR and result['agree'] == 1.0 and result['rho'] < -0.5


def test_insufficient_excitation_is_ok():
    check = HeadingCheck(HC)
    for k in range(200):
        check.add(k / 15, 0.01, 0.01, k % 3)        # straight line: no yaw-rate spread
    result = check.result(200 / 15)
    assert result['level'] == OK and not result['excited']


def test_filter_samples_recover_a_lagged_heading():
    """End to end: static buoys seen through a heading that lags truth by 0.2 s."""
    tau = 0.2

    def yaw(t):
        return 0.8 * math.sin(0.4 * t)

    lagged = yawing_pose(lambda t: yaw(t - tau))
    truth = yawing_pose(yaw)
    f, check = TrackFilter(CFG), HeadingCheck(HC)
    buoys = [(8.0, 1.0), (10.0, -3.0), (6.0, 4.0)]
    t = 0.0
    for k in range(int(60 * 15)):
        t = k / 15
        R, _ = truth(t, 'cam')
        obs = []
        for bx, by in buoys:
            local = R.T @ np.array([bx, by, 0.0])
            obs.append(ob('red_buoy', local[0], local[1]))

        def pose_at(s, frame_id):
            return (truth if frame_id == 'base_link' else lagged)(s, frame_id)
        f.step(frame(t, obs), t, pose_at)
        for sample in f.heading_samples:
            check.add(*sample)
    result = check.result(t)
    assert result['excited'] and result['lag_s'] > 0.1
    assert all(tr.motion == 1 for tr in f.tracks)
