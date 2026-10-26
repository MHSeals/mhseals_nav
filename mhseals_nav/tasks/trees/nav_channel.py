"""Task 1, navigation channel: through two red/green pole gates, the whole hull across each.

Mandatory, and first in every scored run. Until both colours are in sight the boat
searches (tasks/search.py); if the second gate is not in sight after the first, it
looks along the unexplored bearings ahead and drives on until it is, up to
``next_gate_search`` metres: one gate is never the channel.
"""

from mhseals_nav.tasks.behaviours import GateRun
from py_trees.composites import Sequence


def build(params):
    from mhseals_nav.tasks.trees import gate_options, look_for
    red, green = params['red'], params['green']
    return Sequence('nav channel', memory=True, children=[
        look_for('find the channel', (red, green), params),
        GateRun('through the channel', 'channel', red, green, gates=params['gates'],
                search=params['search_distance'], max_width=params['max_gate_width'],
                red_side=params['red_side'], next_search=params['next_gate_search'],
                **gate_options(params)),
    ])
