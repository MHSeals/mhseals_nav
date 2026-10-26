"""Delivery actuators (water gun, ball dropper) as capabilities: none is built yet.

The mission's ``capabilities`` YAML says which exist; a task that ``needs`` one that is
off is worth no points (tasks/budget.py) and its Deliver leaf fails at once. A real
actuator implements the same four calls, so the trees need no change when it lands:
  available()      -> bool, the hardware answers
  start(target)    -> begin a delivery at target (label, x, y); never blocks
  state()          -> 'RUNNING' | 'HIT' (evidence: >= 3 s on the target, a ball in)
                      | 'UNVERIFIED' (actuated, no evidence) | 'FAILED'
  stop()           -> stop at once (preemption, a hold, the end of the run)
"""


class Unavailable:
    """The stand-in for an actuator not built: never available, never moves anything."""

    def __init__(self, name):
        self.name = name

    def available(self):
        return False

    def start(self, target):
        raise RuntimeError(f'{self.name} is not fitted')

    def state(self):
        return 'FAILED'

    def stop(self):
        pass


def fitted(capabilities, provided=None):
    """Return {capability: actuator}: those ``provided`` when on, else Unavailable."""
    provided = provided or {}
    return {name: provided[name] if on and name in provided else Unavailable(name)
            for name, on in (capabilities or {}).items()}
