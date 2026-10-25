"""Shared startup field rules for strict validation and read-only descriptions."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Rule:
    """Finite field constraint; bounds are schema limits, never physical qualification."""

    kind: str
    minimum: float | int | None = None
    exclusive: bool = False
    maximum: int | None = None
    choices: tuple = ()
    size: int = 0

    def check(self, value):
        """Validate and normalize the value."""
        if self.kind == 'string':
            valid = type(value) is str and value in self.choices
        elif self.kind == 'boolean':
            valid = type(value) is bool
        elif self.kind == 'sequence':
            valid = (type(value) is list and len(value) == self.size
                     and all(type(v) is bool for v in value))
        else:
            try:
                valid = (type(value) is int if self.kind == 'integer' else
                         type(value) in (int, float) and math.isfinite(value))
            except OverflowError:
                valid = False
            if valid and self.minimum is not None:
                valid = value > self.minimum if self.exclusive else value >= self.minimum
            if valid and self.maximum is not None:
                valid = value <= self.maximum
        if not valid:
            raise ValueError('invalid startup value: ' + self.describe())
        return (float(value) if self.kind == 'number' else
                tuple(value) if self.kind == 'sequence' else value)

    def describe(self):
        """Human-readable constraints generated from the validator's rule."""
        if self.choices:
            return 'string in ' + repr(self.choices)
        if self.kind == 'sequence':
            return f'flat sequence of exactly {self.size} actual booleans; replaces whole vector'
        if self.kind == 'boolean':
            return 'actual boolean (no numeric/string coercion)'
        text = ('positive signed int64' if self.kind == 'integer' else
                'finite double; integer spelling normalizes to double')
        if self.minimum is not None:
            text += (' > ' if self.exclusive else ' >= ') + str(self.minimum)
        if self.maximum is not None:
            text += ' <= ' + str(self.maximum)
        return text
