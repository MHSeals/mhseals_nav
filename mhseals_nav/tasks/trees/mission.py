"""The whole run: every task in order, each under its own time limit.

The navigation channel must succeed (it is mandatory and first); a later task
that fails or runs out of time is reported as skipped and the run moves on,
ending with the return home.
"""

from py_trees.composites import Selector, Sequence

from mhseals_nav.tasks.behaviours import Deadline, Report


def build(params):
    from mhseals_nav.tasks.trees import TREES
    stages = []
    for index, name in enumerate(params['tasks']):
        settings = params[name]
        task = Deadline(f'{name} time', TREES[name](settings), settings['time_limit'])
        attempt = Sequence(f'{name} attempt', memory=True, children=[
            Report(f'{name} start', 'stage_started', stage=name),
            task,
            Report(f'{name} done', 'stage_completed', stage=name),
        ])
        if index == 0:
            stages.append(attempt)  # mandatory
        else:
            stages.append(Selector(f'{name} or skip', memory=True, children=[
                attempt, Report(f'{name} skipped', 'stage_skipped', stage=name)]))
    return Sequence('mission', memory=True, children=stages)
