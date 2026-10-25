"""Go/no-go gates from the boat's own state: GPS fix tier, drift, and thruster health.

The runner feeds ``Conditions`` from small live topics (MAVROS GPS_RAW_INT fix type,
the velocity controller's drift estimate, the applied command against the measured
velocity, and the effort output). Unknown inputs never block: in the simulator or on
a bench without MAVROS the gates stay open. Thresholds are in the mission YAML under
``conditions`` (tuning study, crane test/tuning-plant Tools/Tuning):
  precise tasks (docking) need RTK fixed; at float they rely on perception or skip;
  a drift estimate held above ``drift_max`` skips the remaining precise tasks;
  a velocity error held while the thrusters saturate (a dead or weak thruster) aborts
  the running task and skips precise ones.
Gates only stop or skip; they never start or resume anything.
"""

import math

from mhseals_nav.tasks import load_config
from mhseals_nav.tasks.behaviours import Leaf
from py_trees.common import Status


class Conditions:
    """The latest boat state and how long each bad condition has held.

    Settings and topics default to mission.yaml's ``conditions``; ``config`` overrides.
    """

    def __init__(self, config=None):
        self.cfg = {**load_config('mission')['conditions'], **(config or {})}
        self.fix = None
        self.since = {}  # condition -> time it began to hold

    def held(self, name, bad, now):
        """Return the seconds the condition has held (0 when it does not)."""
        if not bad:
            self.since.pop(name, None)
            return 0.0
        return now - self.since.setdefault(name, now)

    def drift(self, speed, now):
        self.drift_s = self.held('drift', speed > self.cfg['drift_max'], now)

    def tracking(self, command, measured, effort, now):
        """``command``/``measured`` are (vx, wz); ``effort`` the controller's output."""
        lin, ang = self.cfg['track_err_max']
        off = abs(command[0] - measured[0]) > lin or abs(command[1] - measured[1]) > ang
        saturated = any(abs(e) >= self.cfg['sat_effort'] for e in effort)
        self.weak_s = self.held('weak', off and saturated, now)

    def precise(self):
        """None when precise tasks may run, else the reason they may not."""
        if self.fix is not None and self.fix < self.cfg['precise_min_fix']:
            if self.fix < 5 or self.cfg['float_policy'] == 'skip':
                return f'GPS fix {self.fix} < {self.cfg["precise_min_fix"]}'
        if getattr(self, 'drift_s', 0.0) >= self.cfg['drift_s']:
            return f'drift over {self.cfg["drift_max"]} m/s'
        return self.thrusters()

    def thrusters(self):
        if getattr(self, 'weak_s', 0.0) >= self.cfg['track_err_s']:
            return 'thrusters saturate, velocity off'
        return None


class Gate(Leaf):
    """Condition before a task: FAILURE (the task is skipped) with the reason if unfit."""

    def __init__(self, name, precise):
        super().__init__(name, reads=('conditions',))
        self.precise = precise

    def update(self):
        conditions = self.board.get('conditions') if self.board.exists('conditions') else None
        reason = conditions and (conditions.precise() if self.precise
                                 else conditions.thrusters())
        if reason:
            self.feedback_message = reason
            self.decide('give_up', f'no-go: {reason}')
            return Status.FAILURE
        return Status.SUCCESS


class Watch(Gate):
    """Run beside a task: RUNNING while the thrusters keep up, FAILURE (abort) once not.

    Only a failing thruster aborts a running task. Drift and the fix tier only keep
    precise tasks from starting: stopping a dock half way would leave the boat in a bay.
    """

    def __init__(self, name, precise=False):
        super().__init__(name, precise=False)

    def update(self):
        return Status.RUNNING if super().update() == Status.SUCCESS else Status.FAILURE


def speed(vector):
    return math.hypot(vector.x, vector.y)
