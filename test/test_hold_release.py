"""A watchdog releases its own hold once its condition clears with the FCU disarmed.

11-01: link and fence holds had to be cleared by hand four times after their condition
had already cleared. Armed, or with the FCU state unknown, the hold stays for the operator.
"""

import json
from pathlib import Path

from mhseals_nav.link_watchdog import LinkWatchdog
import pytest
import yaml


def config(name):
    return yaml.safe_load((Path(__file__).parents[1] / 'config' / name)
                          .read_text())['/**']['ros__parameters']


LINK = dict(config('link_watchdog.yaml'), operator_hosts=['opstation'])
AFTER = LINK['release_after']


def held():
    """A link hold sent at 0.0 (no heartbeat yet), then the operator back at 0.5."""
    dog = LinkWatchdog(LINK, own_host='blastoise')
    dog.command(0.0)
    assert dog.tick(0.0) and dog.holding
    dog.heartbeat(0.5, 'opstation')
    return dog


def beat(dog, until, armed=False):
    """Heartbeats and FCU state every 0.5 s to ``until``; return when release fired."""
    t, fired = 0.5, []
    while t <= until:
        dog.heartbeat(t, 'opstation')
        dog.fcu(t, armed)
        if dog.release(t):
            fired.append(t)
        t += 0.5
    return fired


def test_link_and_fence_configs_release_their_own_holds():
    for name in ('link_watchdog.yaml', 'fence_watchdog.yaml'):
        cfg = config(name)
        assert cfg['release_after'] > 0 and cfg['fcu_state_topic'] == '/mavros/state'
        assert cfg['fcu_state_max_age'] > 1.0


def test_released_once_after_clear_for_release_after_while_disarmed():
    dog = held()
    fired = beat(dog, 0.5 + AFTER + 2.0)
    assert fired == [0.5 + AFTER] and not dog.holding


def test_never_released_while_armed_or_with_the_fcu_state_unknown_or_stale():
    dog = held()
    assert beat(dog, 20.0, armed=True) == [] and dog.holding
    dog = held()
    for t in (1.0, 5.0, 20.0):           # no FCU state at all
        dog.heartbeat(t, 'opstation')
        assert not dog.release(t)
    dog.fcu(0.0, False)
    late = LINK['fcu_state_max_age'] + 0.5
    dog.heartbeat(late + AFTER, 'opstation')
    assert not dog.release(late + AFTER)  # a disarmed state that stopped coming


def test_the_condition_going_bad_again_restarts_the_clear_time():
    dog = held()
    dog.fcu(1.0, False)
    assert not dog.release(1.0)
    stale = 1.0 + LINK['max_age_s'] + 0.1  # heartbeat gone again
    dog.fcu(stale, False)
    assert not dog.release(stale)
    dog.heartbeat(stale + 0.1, 'opstation')
    dog.fcu(stale + 0.1, False)
    assert not dog.release(stale + 0.1)
    dog.fcu(stale + 0.1 + AFTER, False)
    dog.heartbeat(stale + 0.1 + AFTER, 'opstation')
    assert dog.release(stale + 0.1 + AFTER)


def test_nothing_to_release_without_a_hold_or_after_the_operator_cleared_it():
    dog = LinkWatchdog(LINK, own_host='blastoise')
    assert beat(dog, 10.0) == []           # never held
    dog = held()
    dog.mux(1.0, json.dumps({'input': 'idle', 'holds': []}))   # U cleared everything
    assert beat(dog, 10.0) == [] and not dog.holding


def test_off_without_release_after():
    dog = LinkWatchdog(dict(LINK, release_after=None), own_host='blastoise')
    dog.command(0.0)
    assert dog.tick(0.0)
    assert beat(dog, 20.0) == []


@pytest.mark.parametrize('state', ['not json', json.dumps({'input': 'hold'})])
def test_a_mux_state_without_a_holds_list_does_not_stop_the_release(state):
    dog = held()
    dog.mux(0.5, state)
    assert beat(dog, 0.5 + AFTER) == [0.5 + AFTER]
