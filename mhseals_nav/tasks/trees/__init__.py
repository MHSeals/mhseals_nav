"""Task trees by name. Each ``build(params)`` returns a py_trees root."""

from mhseals_nav.tasks.trees import debris, gates, speed_challenge, sprint

TREES = {
    'gates': gates.build,
    'debris': debris.build,
    'sprint': sprint.build,
    'speed_challenge': speed_challenge.build,
}
