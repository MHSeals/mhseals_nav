"""Commissioning: cross a pole-buoy pair, turn beyond it, and cross back."""

from mhseals_nav.tasks.behaviours import (
    GateRun, Navigate, PlanGatePass, PlanReturn, Report,
)
from py_trees.composites import Sequence


def build(params):
    """Remember one pair, accepting either colour order, for both crossings."""
    from mhseals_nav.tasks.trees import gate_options
    if params['clearance'] < 1.1:
        raise ValueError('clearance must be at least 1.1 m to clear the hull')
    return Sequence('pair test', memory=True, children=[
        GateRun('find and cross the pair', 'pair_gate',
                params['red'], params['green'], gates=1,
                search=params['search_distance'],
                min_width=params['min_gate_width'],
                max_width=params['max_gate_width'], **gate_options(params)),
        PlanGatePass('plan clearance', 'pair_out', 'pair_gate',
                     after=params['clearance']),
        Navigate('a few metres beyond', 'pair_out_end'),
        PlanReturn('plan turn', 'pair_turn', 'pair_out_end'),
        # Finish a separate single-pose turn before beginning the return route.
        Navigate('turn around', 'pair_turn', precise=True),
        Report('report turn', 'pair_turned'),
        PlanGatePass('plan crossing back', 'pair_back', 'pair_gate', back=True,
                     after=params['clearance']),
        Navigate('back through the same pair', 'pair_back'),
        Report('report return', 'pair_returned'),
    ])
