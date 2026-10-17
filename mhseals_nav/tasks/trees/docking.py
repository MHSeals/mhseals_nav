"""Task 3, docking: the empty bay under the banner of the day.

Find the banner, choose a bay under it (skipping any with a vessel in it),
stage in front, slide in with the precise goal checker, hold, and back out.
"""

from py_trees.composites import Sequence

from mhseals_nav.tasks.behaviours import Hold, Navigate, PlanBay, Report, Search


def build(params):
    banner = params['banner']
    return Sequence('docking', memory=True, children=[
        Search('find the dock', params['banners'], params['search_distance']),
        Search('find the banner', (banner,), params['search_distance']),
        PlanBay('choose a bay', 'bay', banner, params['banners'], params['vessels'],
                berth=params['berth'], staging=params['staging'], depth=params['bay_depth']),
        Navigate('to the bay', 'bay_staging'),
        Navigate('into the bay', 'bay_berth', precise=True),
        Report('report docked', 'docked', include=('bay',), banner=banner),
        Hold('hold in the bay', params['dock_time']),
        Navigate('back out', 'bay_staging'),
        Report('report undocked', 'undocked'),
    ])
