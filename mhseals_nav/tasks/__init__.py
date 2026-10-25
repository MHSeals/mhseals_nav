"""RoboBoat tasks as py_trees behaviour trees on top of Nav2.

Nav2's XML trees decide *how to drive* (plan, follow, recover); these trees
decide *what to do* (which gates, which beacon, when to report). Leaves are
plain Python over a blackboard the runner fills with tracked objects, the
boat's pose and the time, so each one is testable without ROS.
"""

from pathlib import Path


def load_config(name, overrides=None):
    """config/tasks/``name``.yaml (this source tree, else the share) with ``overrides``."""
    import yaml  # only the runner and tests load config
    path = Path(__file__).resolve().parents[2] / 'config/tasks' / f'{name}.yaml'
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('mhseals_nav')) / 'config/tasks' / path.name
    return {**(yaml.safe_load(path.read_text(encoding='utf-8')) or {}), **(overrides or {})}
