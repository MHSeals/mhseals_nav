"""Task 1, navigation channel: through two red/green pole gates, the whole hull across each.

Mandatory, and first in every scored run. If the second gate is not in sight
after the first, the boat drives on until it is.
"""

from py_trees.composites import Sequence

from mhseals_nav.tasks.behaviours import GateRun, WaitFor


def build(params):
    red, green = params['red'], params['green']
    return Sequence('nav channel', memory=True, children=[
        WaitFor('gate visible', (red, green)),
        GateRun('through the channel', 'channel', red, green, gates=params['gates'],
                search=params['search_distance'], max_width=params['max_gate_width']),
    ])
