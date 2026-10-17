"""Task trees by name, RoboBoat 2025 (handbook section 3.2).

Each ``build(params)`` returns a py_trees root. Task 5 (deliveries) is not attempted.
"""

from mhseals_nav.tasks.trees import (
    docking, follow_path, mission, nav_channel, return_home, speed_challenge,
)

TREES = {
    'nav_channel': nav_channel.build,
    'follow_path': follow_path.build,
    'docking': docking.build,
    'speed_challenge': speed_challenge.build,
    'return_home': return_home.build,
    'mission': mission.build,
}
