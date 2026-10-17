"""Task 2, follow the path (mapping migration patterns).

Gate after gate of small red/green buoys until none is ahead, avoiding the
yellow buoys and vessels, then report how many yellow buoys (endangered
species) there were before leaving the task.
"""

from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Sequence

from mhseals_nav.tasks.behaviours import CountObjects, GateRun, ReportCount, WaitFor


def build(params):
    red, green = params['red'], params['green']
    path = Sequence('path', memory=True, children=[
        WaitFor('path visible', (red, green)),
        GateRun('follow the gates', 'path', red, green, search=params['search_distance'],
                max_width=params['max_gate_width']),
        ReportCount('report count', 'species', 'species_count',
                    spin=params['report'] == 'spin'),
    ])
    count = CountObjects('count species', 'species', params['count'])
    return Parallel('follow path', policy=ParallelPolicy.SuccessOnSelected([path]),
                    children=[path, count])
