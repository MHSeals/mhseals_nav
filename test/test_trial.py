"""trial: courses are relative to the start pose, and the metrics score a synthetic run."""
import math

import pytest

from mhseals_nav import trial


def test_every_course_loads_and_scales():
    assert set(trial.courses()) >= {'line', 'square', 'zigzag', 'pool-line', 'pool-square', 'pool-figure8', 'home'}
    points, _, box = trial.load_course('square', 0.5)
    assert points == [(5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)] and box is None
    points, _, box = trial.load_course('pool-line', 0.5)       # fractions of the file's 6 m length
    assert points == [(3.0, 0.0), (0.0, 0.0)] and box == '9x4'
    with pytest.raises(ValueError, match='no course'):
        trial.load_course('circle')


def test_the_course_turns_with_the_start_heading():
    poses = trial.to_world([(10, 0), (10, 10)], (100.0, 50.0, math.pi / 2))     # facing +y
    assert poses[0][:2] == pytest.approx((100.0, 60.0)) and poses[0][2] == pytest.approx(math.pi / 2)
    assert poses[1][:2] == pytest.approx((90.0, 60.0)) and poses[1][2] == pytest.approx(math.pi)


def test_metrics_of_a_synthetic_run():
    route = [(0.0, 0.0), (10.0, 0.0)]
    # 0.5 m to the left halfway, back on the line at the end, 0.2 m short; yaw rate + - + (2 flips)
    samples = [(0.0, 0.0, 0.0, (0.0, 0.0, 0.0), None),
               (5.0, 5.0, 0.5, (0.6, 0.0, 0.2), (0.5, 0.0, 0.1)),
               (8.0, 8.0, -0.25, (0.4, 0.0, -0.2), (0.4, 0.1, -0.1)),
               (10.0, 9.8, 0.0, (0.1, 0.0, 0.1), (0.4, 0.0, 0.1))]
    m = trial.metrics(samples, route)
    assert m['time'] == 10.0 and m['flips'] == 2 and m['vmax'] == 0.6
    assert m['xte_max'] == pytest.approx(0.5) and m['xte_mean'] == pytest.approx(0.75 / 4)
    assert m['final'] == pytest.approx(0.2)
    (vx_mean, vx_max), (vy_mean, vy_max), (wz_mean, wz_max) = m['track']
    assert vx_mean == pytest.approx(0.4 / 3) and vx_max == pytest.approx(0.3)
    assert (vy_mean, vy_max) == pytest.approx((0.1 / 3, 0.1)) and (wz_mean, wz_max) == pytest.approx((0.2 / 3, 0.1))
    assert trial.sign_changes([0.2, 0.01, -0.01, 0.2]) == 0                    # inside the deadband


def test_done_only_when_every_leg_is_reached():
    assert trial.verdict(4, 4) == 'done'
    assert trial.verdict(0, 4) == 'incomplete 0/4'


def test_the_log_line_and_parameter_hash():
    from datetime import datetime
    m = dict(time=61.2, xte_mean=0.31, xte_max=0.9, final=0.42, flips=3, vmax=0.45,
             track=[(0.05, 0.21), (0.02, 0.09), None])
    line = trial.log_line('trial', trial.summary('square', 0.5, m, 'done', 'ab12cd'), datetime(2026, 11, 24, 13, 5, 12))
    assert line == ('13:05:12  trial   square x0.5 done time=61.2s xte=0.31/0.90m final=0.42m flips=3 '
                    'vmax=0.45m/s track=0.05/0.21 0.02/0.09 - ctrl=ab12cd')
    assert trial.params_hash({'a': 1, 'b': 0.5}) == trial.params_hash({'b': 0.5, 'a': 1}) != trial.params_hash({'a': 2})


def test_a_stop_signal_raises_once_then_handlers_restore():
    """SIGTERM (an ssh drop is SIGHUP) raises Stop so the goal is cancelled; a second one doesn't interrupt the cancel."""
    import os
    import signal
    before = signal.getsignal(signal.SIGTERM)
    previous = trial.install_stop_handlers()
    try:
        with pytest.raises(trial.Stop, match='SIGTERM'):
            os.kill(os.getpid(), signal.SIGTERM)
        os.kill(os.getpid(), signal.SIGHUP)                 # ignored: the cancel must run
    finally:
        trial.restore_handlers(previous)
    assert signal.getsignal(signal.SIGTERM) is before


def test_the_box_is_centred_on_the_course_and_turns_with_the_start():
    points, _, box = trial.load_course('pool-line')
    box, centre = trial.parse_box(box), trial.box_centre(points)
    start = (10.0, 20.0, math.pi / 2)                          # facing +y; the line runs 6 m up y
    assert trial.outside_box((10.0, 26.0), start, box, centre) is None
    assert trial.outside_box((10.0, 28.0), start, box, centre) == 'left the 9x4 m box 0.50 m ahead'
    assert trial.outside_box((7.5, 23.0), start, box, centre) == 'left the 9x4 m box 0.50 m left'
    assert trial.box_margin([(10.0, 26.0), (9.0, 23.0)], start, box, centre) == pytest.approx(1.0)
    with pytest.raises(ValueError, match='LENGTHxWIDTH'):
        trial.parse_box('9 by 4')


def test_home_reads_the_start_of_todays_last_trial(tmp_path):
    with pytest.raises(ValueError, match='run a course first'):
        trial.load_start(str(tmp_path))
    trial.save_start('map', (1.0, 2.0, 0.5), 'pool-square', str(tmp_path))
    assert trial.load_start(str(tmp_path)) == ('map', (1.0, 2.0, 0.5))
