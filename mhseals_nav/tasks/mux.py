"""command_mux's latched ~/state as the task runner reads it: who has the boat.

The runner drives only while the mux's input is auto. An input that outranks auto (rc, manual
or a hold), or a state that is missing, stale or unreadable, ends or refuses the task: without a
live mux nothing the task sends drives the boat.
"""

import json

# command_mux ~/state inputs that outrank auto: while one has the mux, /cmd_vel reaches nothing.
OUTRANK_AUTO = ('rc', 'manual', 'hold')


def mux_state(state):
    """Return a command_mux ~/state (its JSON) as (input, body); input None when unreadable."""
    try:
        body = json.loads(state)
        source = body['input']
    except (ValueError, TypeError, KeyError):
        body, source = {}, None
    return (source if isinstance(source, str) else None), body


def mux_suspends(source, age, limit):
    """Return why command_mux's ~/state stops a task, '' while auto may drive.

    ``source`` is the state's input (None unreadable), ``age`` its receipt age (None: none
    yet), ``limit`` the oldest state that still counts. Fails closed.
    """
    if age is None:
        return 'no command_mux state'
    if age > limit:
        return f'command_mux state older than {limit} s'
    if source is None:
        return 'unreadable command_mux state'
    return f'{source} has control' if source in OUTRANK_AUTO else ''
