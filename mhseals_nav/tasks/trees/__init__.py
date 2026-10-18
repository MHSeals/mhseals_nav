"""Task trees by name: RoboBoat 2025 and commissioning exercises.

Each ``build(params)`` returns a py_trees root. Task 5 (deliveries) is not attempted.
A task with ``approach: [x, y, yaw]`` (map frame, from the day's course layout)
first drives there, so it starts where its objects are in view.
"""

from py_trees.composites import Sequence
from py_trees.decorators import FailureIsSuccess

from mhseals_nav.tasks.behaviours import Navigate, PlanPose
from mhseals_nav.tasks.trees import (
    circle_test, docking, follow_path, mission, nav_channel, pair_test,
    return_home, speed_challenge,
)


def approached(build):
    def wrapped(params):
        tree = build(params)
        if not params.get('approach'):
            return tree
        return Sequence(f'{tree.name} from its approach', memory=True, children=[
            PlanPose(f'{tree.name} approach', 'approach', params['approach']),
            # Getting near is the point; a goal Nav2 gives up on close by is fine.
            FailureIsSuccess(name=f'near {tree.name}',
                             child=Navigate(f'to {tree.name}', 'approach')),
            tree,
        ])
    return wrapped


TREES = {name: approached(build) for name, build in {
    'nav_channel': nav_channel.build,
    'follow_path': follow_path.build,
    'docking': docking.build,
    'speed_challenge': speed_challenge.build,
    'return_home': return_home.build,
    'pair_test': pair_test.build,
    'circle_test': circle_test.build,
}.items()}
TREES['mission'] = mission.build
