"""Live parameter changes are validated whole and applied through one helper."""
from types import SimpleNamespace as S

import pytest

from mhseals_nav.mavros_setup import parse_rates, request_rates
from mhseals_nav.utils.live_params import (
    LiveParams, at_least, fraction, one_of, positive, unit)

pytest.importorskip('rcl_interfaces')


class FakeNode:
    def __init__(self, **parameters):
        self.parameters, self.callback = parameters, None

    def get_parameter(self, name):
        return S(value=self.parameters[name])

    def add_on_set_parameters_callback(self, callback):
        self.callback = callback

    def set(self, **changes):
        return self.callback([S(name=k, value=v) for k, v in changes.items()])


@pytest.mark.parametrize('rule, value', [
    (positive, 0.), (positive, float('nan')), (positive, float('inf')), (positive, True),
    (fraction, 0.), (fraction, 1.5), (unit, -.1), (at_least(1), 0), (one_of('a', 'b'), 'c')])
def test_rules_reject_invalid_values(rule, value):
    with pytest.raises(ValueError):
        rule(value)


def test_rejected_set_keeps_every_accepted_value():
    node = FakeNode(age=.5, alpha=.5)
    seen = []
    live = LiveParams(node, {'age': positive, 'alpha': fraction},
                      lambda values, changed: seen.append(changed))
    result = node.set(age=2., alpha=3.)               # one bad value refuses the whole set
    assert not result.successful and 'alpha' in result.reason
    assert (live['age'], live['alpha'], seen) == (.5, .5, [])
    assert node.set(age=2.).successful
    assert (live['age'], seen) == (2., [{'age': 2.}])


def test_joint_check_sees_the_merged_values_and_unlisted_names_pass():
    def joint(values):
        if values['low'] >= values['high']:
            raise ValueError('low must be below high')
    node = FakeNode(low=1., high=2.)
    live = LiveParams(node, {'low': positive, 'high': positive}, joint=joint)
    assert not node.set(low=3.).successful
    assert node.set(low=3., high=4., other='x').successful
    assert live['low'] == 3.


def test_invalid_start_value_raises():
    with pytest.raises(ValueError):
        LiveParams(FakeNode(age=-1.), {'age': positive})


def test_message_rate_change_calls_set_message_interval():
    calls, node = [], FakeNode(message_rates=['33:20.0'])
    live = LiveParams(node, {'message_rates': parse_rates},
                      lambda values, changed: request_rates(
                          values['message_rates'], lambda i, hz: calls.append((i, hz)) or True,
                          lambda line: None))
    assert not node.set(message_rates=['33:fast']).successful
    assert not node.set(message_rates=['33:nan']).successful
    assert calls == []
    assert node.set(message_rates=['33:10.0', '1:2.0']).successful
    assert calls == [(33, 10.), (1, 2.)] and live['message_rates'] == calls


def test_restart_only_names_are_refused_unless_unchanged():
    node = FakeNode(age=.5, frame='map')
    live = LiveParams(node, {'age': positive}, restart_only=('frame',))
    result = node.set(frame='odom')
    assert not result.successful and result.reason == 'restart required: frame'
    assert node.set(frame='map', age=2.).successful      # same value, e.g. a param load
    assert live['age'] == 2.
