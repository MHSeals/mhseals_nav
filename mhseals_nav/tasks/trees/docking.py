"""Task 3, docking: the empty bay under the banner of the day.

Find the banner, choose a bay under it (skipping any with a vessel in it),
stage in front, choose again with the bay in view, slide in with the precise goal
checker, hold, and back out.
"""

from mhseals_nav.tasks.behaviours import (
    BackOut, Hold, Navigate, PlanBay, PlanLook, PlanSlip, Report,
)
from py_trees.composites import Selector, Sequence
from py_trees.decorators import Retry


def plan_bay(params, name, confirm=False):
    if params.get('indicators'):  # 2026: the lowest free green slip of either pier
        return PlanSlip(name, 'bay', params['indicators'], params['numbers'],
                        params['vessels'], prefer=params['prefer'], berth=params['berth'],
                        staging=params['staging'], depth=params['bay_depth'], confirm=confirm)
    return PlanBay(name, 'bay', params['banner'], params['banners'], params['vessels'],
                   berth=params['berth'], staging=params['staging'], depth=params['bay_depth'],
                   confirm=confirm)


def build(params):
    from mhseals_nav.tasks.trees import look_for
    banner = params.get('banner', 'slip')
    find = ([look_for('find the marina', tuple(params['indicators']), params, need='any')]
            if params.get('indicators') else
            [look_for('find the dock', params['banners'], params, need='any'),
             look_for('find the banner', (banner,), params)])
    return Sequence('docking', memory=True, children=[
        *find,
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
        # The choice may rest on memory (a vessel out of view is soon forgotten, a light
        # may have changed): at the staging pose, with the bay in view, choose again, and
        # line up on the new bay if that one is no longer free.
        Retry('a free bay', Sequence('line up', memory=True, children=[
            Navigate('to the bay', 'bay_staging', retries=2, why=f'line up on the {banner} bay'),
            plan_bay(params, 'confirm the bay', confirm=True),
        ]), params['confirm_tries']),
        # Nav2's stopped checker can refuse a berthed boat that waves keep stirring.
        Navigate('into the bay', 'bay_berth', precise=True, retries=2,
                 accept=params['berth_accept'], why='slide into the bay'),
        Report('report docked', 'docked', include=('bay', 'bay_number', 'bay_colour')
               if params.get('indicators') else ('bay',), banner=banner),
        Hold('hold in the bay', params['dock_time'], station=params['hold_redrive'],
             precise=params['hold_precise']),
        # Straight back to the staging pose, then the next goal turns clear of the dock;
        # planning there instead turns in the bay first and jams the boat crosswise.
        Selector('leave the bay', memory=True, children=[
            BackOut('back out', 'bay_staging', params['back_out_speed'],
                    params['back_out_time'], params['back_out_resends'],
                    params['back_out_pause'], retries=params['back_out_retries'], accept=1.0,
                    why='back out of the bay'),
            Navigate('drive out', 'bay_staging', retries=1, accept=1.0,
                     why='drive out of the bay'),
        ]),
        Report('report undocked', 'undocked'),
    ])
