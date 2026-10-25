"""Task 3, docking: the empty bay under the banner of the day.

Find the banner, choose a bay under it (skipping any with a vessel in it),
stage in front, slide in with the precise goal checker, hold, and back out.
"""

from mhseals_nav.tasks.behaviours import BackOut, Hold, Navigate, PlanBay, PlanLook, Report
from py_trees.composites import Selector, Sequence


def plan_bay(params, name):
    return PlanBay(name, 'bay', params['banner'], params['banners'], params['vessels'],
                   berth=params['berth'], staging=params['staging'], depth=params['bay_depth'])


def build(params):
    from mhseals_nav.tasks.trees import look_for
    banner = params['banner']
    return Sequence('docking', memory=True, children=[
        look_for('find the dock', params['banners'], params, need='any'),
        look_for('find the banner', (banner,), params),
        # The camera may see only part of the dock: if no empty bay is in view yet,
        # look along it once and choose again.
        Selector('choose a bay', memory=True, children=[
            plan_bay(params, 'first look'),
            Sequence('look along the dock', memory=True, children=[
                PlanLook('plan look', 'look'),
                Navigate('look one way', 'look'),
                Navigate('look the other way', 'look_rest'),
                plan_bay(params, 'second look'),
            ]),
        ]),
        Navigate('to the bay', 'bay_staging', retries=2, why=f'line up on the {banner} bay'),
        # Nav2's stopped checker can refuse a berthed boat that waves keep stirring.
        Navigate('into the bay', 'bay_berth', precise=True, retries=2,
                 accept=params['berth_accept'], why='slide into the bay'),
        Report('report docked', 'docked', include=('bay',), banner=banner),
        Hold('hold in the bay', params['dock_time'], station=params['hold_redrive'],
             precise=params['hold_precise']),
        # Straight back to the staging pose, then the next goal turns clear of the dock;
        # planning there instead turns in the bay first and jams the boat crosswise.
        Selector('leave the bay', memory=True, children=[
            BackOut('back out', 'bay_staging', params['back_out_speed'],
                    params['back_out_time'], retries=1, accept=1.0,
                    why='back out of the bay'),
            Navigate('drive out', 'bay_staging', retries=1, accept=1.0,
                     why='drive out of the bay'),
        ]),
        Report('report undocked', 'undocked'),
    ])
