"""Task 2, follow the path (mapping migration patterns).

Gate after gate of small red/green buoys until none is ahead, avoiding the
yellow buoys and vessels, then report how many yellow buoys (endangered
species) there were before leaving the task.
"""

from mhseals_nav.tasks.behaviours import (
    CountObjects, GateRun, Navigate, PlanChainBack, PlanCircle, Remember, ReportCount,
    ReportObjects, SpeedCap,
)
from py_trees.behaviours import Success
from py_trees.common import ParallelPolicy
from py_trees.composites import Parallel, Selector, Sequence


def rescue(params, index):
    """Circle the nearest beacon not yet circled, 360 degrees; succeed if there is none."""
    lap = Sequence(f'rescue {index}', memory=True, children=[
        PlanCircle(f'plan rescue {index}', 'rescue', tuple(params['rescue']),
                   params['circle_radius'], params['circle_clockwise'],
                   points=params['circle_points'], done='rescued'),
        Navigate(f'to rescue {index}', 'rescue_entry', why='to the beacon'),
        Navigate(f'circle {index} first half', 'rescue', retries=1, why='round the beacon'),
        Navigate(f'circle {index} second half', 'rescue_rest', retries=1,
                 why='round the beacon'),
        Remember(f'rescued {index}', 'rescued', 'rescue_at'),
    ])
    return Selector(f'rescue {index} or none', memory=True, children=[
        lap, Success(f'no beacon {index}')])


def build(params):
    from mhseals_nav.tasks.trees import gate_options, look_for
    red, green = params['red'], params['green']
    path = Sequence('path', memory=True, children=[
        look_for('find the path', (red, green), params),
        # Slower through the small gates: the controller gave up at the turn out of one.
        SpeedCap('gate speed', GateRun(
            'follow the gates', 'path', red, green, search=params['search_distance'],
            max_width=params['max_gate_width'], red_side=params['red_side'],
            **gate_options(params)), params['speed']),
        # 2026 debris field: circle the green beacons, then back through the channel.
        *[rescue(params, i + 1) for i in range(params.get('rescues', 0))],
        *([PlanChainBack('plan the way back', 'path_back', 'path', params['corner_side'],
                         params['gate_exit']),
           SpeedCap('gate speed back', Navigate('back through the channel', 'path_back',
                                                retries=2, why='back through the channel'),
                    params['speed'])] if params.get('and_back') else []),
        ReportCount('report count', 'species', 'species_count',
                    spin=params['report'] == 'spin'),
    ])
    watchers = [CountObjects('count species', 'species', params['count'])]
    if params.get('report_labels'):  # 2026: every debris and indicator position, once
        watchers.append(ReportObjects('report debris', 'debris',
                                      tuple(params['report_labels'])))
    return Parallel('follow path', policy=ParallelPolicy.SuccessOnSelected([path]),
                    children=[path, *watchers])
