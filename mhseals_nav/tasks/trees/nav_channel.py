"""Task 1, navigation channel: through two red/green pole gates, the whole hull across each.

Mandatory, and first in every scored run. Until both colours are in sight the boat
searches (tasks/search.py); if the second gate is not in sight after the first, it
looks along the unexplored bearings ahead and drives on until it is.
"""

from py_trees.composites import Sequence

from mhseals_nav.tasks.behaviours import GateRun


def build(params):
    from mhseals_nav.tasks.trees import gate_options, look_for
    red, green = params['red'], params['green']
    return Sequence('nav channel', memory=True, children=[
        look_for('find the channel', (red, green), params),
        GateRun('through the channel', 'channel', red, green, gates=params['gates'],
                search=params['search_distance'], max_width=params['max_gate_width'],
                red_side=params['red_side'], **gate_options(params)),
    ])
