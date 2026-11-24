"""trial: courses are relative to the start pose, and the metrics score a synthetic run."""
import math

import pytest

from mhseals_nav import trial


def test_every_course_loads_and_scales():
    assert set(trial.courses()) >= {'line', 'square', 'zigzag'}
    points, _ = trial.load_course('square', 0.5)
    assert points == [(5.0, 0.0), (5.0, 5.0), (0.0, 5.0), (0.0, 0.0)]
    with pytest.raises(ValueError, match='no course'):
        trial.load_course('circle')


def test_the_course_turns_with_the_start_heading():
    poses = trial.to_world([(10, 0), (10, 10)], (100.0, 50.0, math.pi / 2))     # facing +y
    assert poses[0][:2] == pytest.approx((100.0, 60.0)) and poses[0][2] == pytest.approx(math.pi / 2)
    assert poses[1][:2] == pytest.approx((90.0, 60.0)) and poses[1][2] == pytest.approx(math.pi)


def test_metrics_of_a_synthetic_run():
    route = [(0.0, 0.0), (10.0, 0.0)]
    # 0.5 m to the left halfway, back on the line at the end, 0.2 m short; yaw rate + - + (2 flips)
    samples = [(0.0, 0.0, 0.0, 0.0, 0.0), (5.0, 5.0, 0.5, 0.6, 0.2), (8.0, 8.0, -0.25, 0.4, -0.2),
               (10.0, 9.8, 0.0, 0.1, 0.1)]
    m = trial.metrics(samples, route)
    assert m['time'] == 10.0 and m['flips'] == 2 and m['vmax'] == 0.6
    assert m['xte_max'] == pytest.approx(0.5) and m['xte_mean'] == pytest.approx(0.75 / 4)
    assert m['final'] == pytest.approx(0.2)
    assert trial.sign_changes([0.2, 0.01, -0.01, 0.2]) == 0                    # inside the deadband


def test_the_log_line_and_parameter_hash():
    from datetime import datetime
    m = dict(time=61.2, xte_mean=0.31, xte_max=0.9, final=0.42, flips=3, vmax=0.45)
    line = trial.log_line('trial', trial.summary('square', 0.5, m, 'done', 'ab12cd'), datetime(2026, 11, 24, 13, 5, 12))
    assert line == ('13:05:12  trial   square x0.5 done time=61.2s xte=0.31/0.90m final=0.42m flips=3 '
                    'vmax=0.45m/s ctrl=ab12cd')
    assert trial.params_hash({'a': 1, 'b': 0.5}) == trial.params_hash({'b': 0.5, 'a': 1}) != trial.params_hash({'a': 2})
