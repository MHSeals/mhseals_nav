"""Validate and apply ROS parameter changes while a node runs.

A node lists a rule per live parameter. A rule takes the new value and returns
it (coerced) or raises ValueError. A bad ``ros2 param set`` is refused whole and
the node keeps its old values. ``values`` always holds the accepted ones.
"""
import math


def finite(value):
    if isinstance(value, bool) or not math.isfinite(value):
        raise ValueError('must be a finite number')
    return float(value)


def positive(value):
    if finite(value) <= 0:
        raise ValueError('must be positive')
    return float(value)


def fraction(value):
    """A number in (0, 1]."""
    if not 0 < finite(value) <= 1:
        raise ValueError('must be in (0, 1]')
    return float(value)


def unit(value):
    """A number in [0, 1]."""
    if not 0 <= finite(value) <= 1:
        raise ValueError('must be in [0, 1]')
    return float(value)


def at_least(minimum):
    """Rule for a number no smaller than ``minimum``."""
    def rule(value):
        if finite(value) < minimum:
            raise ValueError(f'must be at least {minimum:g}')
        return value
    return rule


def one_of(*choices):
    def rule(value):
        if value not in choices:
            raise ValueError('must be one of ' + ', '.join(map(str, choices)))
        return value
    return rule


def check_changes(rules, changes, current=None, joint=None):
    """Validated copy of ``changes`` (names without a rule are ignored).

    ``joint`` sees the merged values and raises ValueError for a bad combination.
    """
    checked = {}
    for name, value in changes.items():
        if name in rules:
            try:
                checked[name] = rules[name](value)
            except (ValueError, TypeError) as error:
                raise ValueError(f'{name} {error}') from None
    if joint and checked:
        joint({**(current or {}), **checked})
    return checked


class LiveParams:
    """Accepted values of the rule-covered parameters, refreshed by param sets.

    ``on_change(values, changed)`` runs after a set is accepted. The node must
    have declared every parameter in ``rules`` first. ``restart_only`` names
    declared parameters the node reads once at start: a set that would change
    one is refused, so a read-back never reports a value the node is not using.
    """

    def __init__(self, node, rules, on_change=None, joint=None, restart_only=()):
        self.rules, self.on_change, self.joint = rules, on_change, joint
        self.startup = {name: node.get_parameter(name).value for name in restart_only}
        self.values = check_changes(
            rules, {name: node.get_parameter(name).value for name in rules})
        if joint:
            joint(self.values)
        node.add_on_set_parameters_callback(self._set)

    def __getitem__(self, name):
        return self.values[name]

    def _set(self, parameters):
        from rcl_interfaces.msg import SetParametersResult
        for p in parameters:
            if p.name in self.startup and p.value != self.startup[p.name]:
                return SetParametersResult(
                    successful=False, reason=f'restart required: {p.name}')
        try:
            changed = check_changes(self.rules, {p.name: p.value for p in parameters},
                                    self.values, self.joint)
        except ValueError as error:
            return SetParametersResult(successful=False, reason=str(error))
        self.values.update(changed)
        if changed and self.on_change:
            self.on_change(self.values, changed)
        return SetParametersResult(successful=True)
