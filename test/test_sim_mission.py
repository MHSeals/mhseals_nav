"""Sim mission gate: per-goal time and in-place rotation (live part needs $SIMRIG)."""
import os
from pathlib import Path

import pytest

from mhseals_nav.mission_check import load_gates, read_cmds, split_goals, goal_stats, violations

GATES = Path(__file__).resolve().parents[1] / 'config/mission_gates.yaml'


def _goal(start, seconds, rotate_s=0.0, reverse_s=0.0):
    """10 Hz commands: rotate_s of pure rotation, then reverse_s backwards, then forward."""
    n = int(seconds * 10)
    r, b = rotate_s * 10, (rotate_s + reverse_s) * 10
    return [(start + i / 10, 0.0 if i < r else -0.3 if i < b else 0.3, 0.0,
             0.2 if i < r else 0.0) for i in range(n)]


def _mission(*goals):
    out, t = [], 0.0
    for seconds, rotate_s, *rev in goals:
        out += _goal(t, seconds, rotate_s, *rev)
        t += seconds + 5
    return out


def test_goals_split_on_gaps_and_stats_count_rotation():
    goals = split_goals(_mission((30, 10), (20, 0)))
    assert len(goals) == 2
    stats = goal_stats(goals[0])
    assert stats['duration'] == pytest.approx(30, abs=0.2)
    assert stats['rotate'] == pytest.approx(10, abs=0.5)


GOOD = [(40, 5), (40, 5), (20, 0), (20, 0)]


def test_good_mission_passes_and_slow_or_spinning_goals_fail():
    gates = load_gates(GATES)
    assert violations(_mission(*GOOD), gates) == []
    assert any('in-place' in v for v in violations(_mission(*GOOD[:3], (20, 15)), gates))
    assert any('goal 2' in v for v in violations(_mission(GOOD[0], (60, 5), *GOOD[2:]), gates))
    assert any('expected' in v for v in violations(_mission((30, 5)), gates))


def test_time_bounds_follow_goal_order():
    gates = load_gates(GATES)
    assert violations(_mission(GOOD[0], GOOD[1], (30, 0), GOOD[3]), gates) == [
        'goal 3: 30 s > 25 s']
    assert violations(_mission(GOOD[0], (44, 5), (24, 0), (24, 0)), gates) == []


def test_reversing_goal_fails():
    gates = load_gates(GATES)
    assert violations(_mission(GOOD[0], (40, 0, 10), *GOOD[2:]), gates) == [
        'goal 2: reversing 25% of the time > 15%']
    assert violations(_mission(GOOD[0], (40, 0, 4), *GOOD[2:]), gates) == []


@pytest.mark.skipif(not os.environ.get('SIMRIG'), reason='needs a sim-rig session in $SIMRIG')
def test_sim_session_meets_gates():
    """$SIMRIG is a run_sim.sh session dir recorded while the four standard goals were sent."""
    cmds = read_cmds(Path(os.environ['SIMRIG']) / 'bag')
    assert violations(cmds, load_gates(GATES)) == []
