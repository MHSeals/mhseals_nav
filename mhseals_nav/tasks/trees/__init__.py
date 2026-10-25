"""Task trees by name: RoboBoat 2025 and commissioning exercises.

Each ``build(params)`` returns a py_trees root. Task 5 (deliveries) is not attempted.
A task with ``approach: [x, y, yaw]`` (map frame, from the day's course layout)
first drives there, so it starts where its objects are in view.
"""

import math

from mhseals_nav.tasks import search
from mhseals_nav.tasks.behaviours import Navigate, PlanHypothesis, PlanPose
from mhseals_nav.tasks.trees import (
    circle_test, docking, follow_path, mission, nav_channel, pair_test,
    return_home, speed_challenge,
)
from py_trees.composites import Sequence
from py_trees.decorators import FailureIsSuccess


# What the recogniser calls each task (tasks/recognition.py).
KINDS = {'nav_channel': 'nav_channel', 'follow_path': 'follow_path', 'docking': 'dock',
         'speed_challenge': 'speed_gate', 'return_home': 'home_gate'}


def look_for(name, labels, params, **overrides):
    """Return the search subtree (tasks/search.py) with the task's ``search`` overrides."""
    return search.search(name, labels, search.load_config(
        {**(params.get('search') or {}), **overrides}))


def gate_options(params):
    """Return GateRun's YAML: the first gate's reach, its look (Scan's spread), corners."""
    spread = search.load_config(params.get('search'))['scan_spread_deg']
    return {'first_radius': params['first_gate_radius'], 'spread': math.radians(spread),
            'corner_side': params['corner_side']}


def approached(task, build):
    """Drive to the task first, its pose kept as ``<task>_approach``.

    The pose is the task's ``approach`` from the day's layout or, without one, a view
    of where the recogniser has seen that task; with neither, the task's own search
    finds it.
    """
    def wrapped(params):
        tree = build(params)
        key = f'{task}_approach'
        go = FailureIsSuccess(name=f'near {tree.name}', child=Navigate(
            # Getting near is the point; a goal Nav2 gives up on close by is fine.
            f'to {tree.name}', key, why=f'to the {task} area', phase='approach'))
        if params.get('approach'):
            plan = PlanPose(f'{tree.name} approach', key, params['approach'])
            return Sequence(f'{tree.name} from its approach', memory=True,
                            children=[plan, go, tree])
        if task not in KINDS:
            return tree
        seen = Sequence(f'{tree.name} where it was seen', memory=True, children=[
            PlanHypothesis(f'{tree.name} recognised', key, KINDS[task]), go])
        return Sequence(f'{tree.name} from where it was seen', memory=True, children=[
            FailureIsSuccess(name=f'{tree.name} if seen', child=seen), tree])
    return wrapped


TREES = {name: approached(name, build) for name, build in {
    'nav_channel': nav_channel.build,
    'follow_path': follow_path.build,
    'docking': docking.build,
    'speed_challenge': speed_challenge.build,
    'return_home': return_home.build,
    'pair_test': pair_test.build,
    'circle_test': circle_test.build,
}.items()}
TREES['mission'] = mission.build
