"""Task 6, harbor alert: on the horn, go to its zone's yellow buoy.

The horn listener is not built: the task ``needs`` the ``horn`` capability and the
mission's budget skips it while that is off. The listener will write ``alert_zone``
(1: one blast, the speed challenge's turn mark; 2: two blasts, the buoy off the
marina). Zone 1's buoy is the yellow one beside a red/green gate (``pair``), zone 2's
the one without. Phase 2 runs this as a supervisor that preempts the current task.
"""

import math

from mhseals_nav.tasks import geometry
from mhseals_nav.tasks.behaviours import Leaf, Navigate, Report
from py_trees.common import Status
from py_trees.composites import Sequence


class PlanZone(Leaf):
    """Write a view ``standoff`` m off the alert zone's buoy to ``key``."""

    def __init__(self, name, key, mark, pair, standoff=2.0):
        super().__init__(name, reads=('alert_zone',), writes=(key,))
        self.key, self.mark, self.pair, self.standoff = key, mark, pair, standoff

    def update(self):
        zone = self.board.get('alert_zone') if self.board.exists('alert_zone') else None
        pose = self.board.pose
        speed = geometry.paired(self.board.objects, self.pair)
        buoys = speed if zone == 1 else [o for o in self.board.objects
                                         if o[0] == self.mark and o not in speed]
        if pose is None or zone not in (1, 2) or not buoys:
            self.feedback_message = f'zone {zone}: buoy not known'
            return Status.FAILURE
        _, x, y = min(buoys, key=lambda o: math.dist(o[1:], pose[:2]))
        away = geometry.heading((x, y), pose)
        view = (x + self.standoff * math.cos(away), y + self.standoff * math.sin(away))
        self.board.set(self.key, [(*view, geometry.heading(view, (x, y)))], overwrite=True)
        return Status.SUCCESS


def build(params):
    return Sequence('harbor alert', memory=True, children=[
        PlanZone('plan the zone', 'alert_view', params['mark'], params['pair'],
                 params['standoff']),
        Navigate('to the zone', 'alert_view', accept=1.5, why='harbor alert zone'),
        Report('zone reached', 'alert_zone_reached'),
    ])
