"""RoboBoat tasks as py_trees behaviour trees on top of Nav2.

Nav2's XML trees decide *how to drive* (plan, follow, recover); these trees
decide *what to do* (which gates, which beacon, when to report). Leaves are
plain Python over a blackboard the runner fills with tracked objects, the
boat's pose and the time, so each one is testable without ROS.
"""

from copy import deepcopy
from functools import lru_cache


def load_config(name, overrides=None):
    """config/tasks/``name``.yaml (params.config_path: the share, else this source tree) with
    ``overrides``, read once per process by the package's one strict loader; callers get copies.
    """
    return {**deepcopy(_parsed(name)), **(overrides or {})}


@lru_cache(maxsize=None)
def _parsed(name):
    from mhseals_nav.tasks import params  # only the runner and tests
    return params.read(params.config_path(f'{name}.yaml'))
