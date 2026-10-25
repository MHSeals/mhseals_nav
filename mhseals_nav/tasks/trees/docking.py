"""Task 3, docking: the empty bay under the banner of the day.

Find the banner, choose a bay under it (skipping any with a vessel in it),
stage in front, slide in with the precise goal checker, hold, and back out.
"""

from py_trees.composites import Selector, Sequence

from mhseals_nav.tasks.behaviours import Hold, Navigate, PlanBay, PlanLook, Report


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
        Hold('hold in the bay', params['dock_time'], station=params['berth_accept']),
        Navigate('back out', 'bay_staging', retries=2, accept=1.0, why='back out of the bay'),
        Report('report undocked', 'undocked'),
    ])
