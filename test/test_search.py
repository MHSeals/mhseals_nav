"""Search behaviours: coverage, view planning, sweeps and the search subtree."""

import importlib.util
import math

import pytest

pytestmark = pytest.mark.skipif(importlib.util.find_spec('py_trees') is None,
                                reason='py_trees is not installed')

if importlib.util.find_spec('py_trees') is not None:
    import py_trees
    from py_trees.common import Access, Status

    from mhseals_nav.tasks import geometry, search
    from mhseals_nav.tasks.tracks import Track

FOV = (math.radians(90), 10.0)


def cell(cover, x, y):
    return int((x - cover.lo[0]) // cover.cell), int((y - cover.lo[1]) // cover.cell)


def live(label, x, y, confidence=0.9, sigma=0.3):
    return Track(label, x, y, confidence, sigma, True, False, f'{label}{x}{y}')


def prior(label, x, y):
    return Track(label, x, y, 0.5, 2.0, True, True, f'p{label}{x}{y}')


@pytest.fixture
def cfg():
    py_trees.blackboard.Blackboard.clear()
    return search.load_config({'area': [[-40, -40], [40, 40]]})


def test_coverage_marks_the_cone_and_not_behind_the_boat():
    cover = search.Coverage([[-20, -20], [20, 20]], cell=1.0)
    cover.observe((0, 0, 0), FOV, now=5.0)
    seen = cover.seen > -math.inf
    assert seen[cell(cover, 5, 0)] and seen[cell(cover, 6, 3)]
    assert not seen[cell(cover, -5, 0)]  # behind
    assert not seen[cell(cover, 5, 8)] and not seen[cell(cover, 15, 0)]  # beside, too far
    weights = cover.unseen()
    assert weights[cell(cover, 5, 0)] == 0 and weights[cell(cover, -5, 0)] == 1
    assert cover.unseen(now=5 + cover.forget_s / 2)[cell(cover, 5, 0)] == pytest.approx(0.5)


def test_bearing_gaps_picks_the_unseen_side():
    cover = search.Coverage([[-30, -30], [30, 30]], cell=1.0)
    cover.observe((0, 0, math.pi / 4), FOV, now=1.0)  # the left side has been looked at
    bearing = cover.bearing_gaps((0, 0, 0), FOV, math.radians(90), 0.0)
    assert bearing < 0
    cover.seen[:] = 1.0
    assert cover.bearing_gaps((0, 0, 0), FOV, math.radians(90), 0.0) is None


def test_next_view_prefers_the_unexplored_side_and_uncertain_wanted_tracks(cfg):
    cover = search.Coverage(cfg['area'], cfg['cell'], forget_s=1e6)
    cover.seen[:, cover.ys > 0] = 0.0  # the left half has been seen
    view, score = search.next_view(cover, (0, 0, 0), FOV, 1.0, [], ('red',), cfg)
    assert view[1] + FOV[1] / 2 * math.sin(view[2]) < 0 and score > 0  # looks right
    cover.seen[:] = 0.0  # everything seen
    assert search.next_view(cover, (0, 0, 0), FOV, 1.0, [], ('red',), cfg) is None
    weak = Track('red', -12, 0, 0.3, 3.0, True, False, 'r')
    view, _ = search.next_view(cover, (0, 0, 0), FOV, 1.0, [weak], ('red',), cfg)
    assert math.dist(view[:2], (-12, 0)) <= FOV[1]
    assert abs(geometry.wrap(math.atan2(-view[1], -12 - view[0]) - view[2])) <= FOV[0] / 2
    assert search.next_view(cover, (0, 0, 0), FOV, 1.0, [weak], ('green',), cfg) is None


def test_view_pose_stands_off_on_the_boats_side_facing_the_target():
    pose = search.view_pose((10, 0), (0, 0, 1.0), 4.0)
    assert pose == pytest.approx((6, 0, 0))


def test_lawnmower_and_expanding_square_stay_inside_the_area():
    area = [[0, 0], [50, 20]]
    route = search.lawnmower(area, (48, 1, 0), spacing=8.0)
    assert all(0 <= x <= 50 and 0 <= y <= 20 for x, y, _ in route)
    assert route[0][0] > 40 and route[0][1] < 10  # from the corner nearest the boat
    assert len({round(y, 3) for _, y, _ in route}) >= 2  # several legs across
    square = search.expanding_square((45, 18, 0), 6.0, area)
    assert len(square) > 4 and all(0 <= x <= 50 and 0 <= y <= 20 for x, y, _ in square)


class Goal:
    def __init__(self, world, route, why):
        self.world, self.route, self.why = world, route, why
        self.at, self.cancelled, self.status = world.tick, False, 'SUCCEEDED'

    @property
    def done(self):
        return self.cancelled or self.world.tick >= self.at + self.world.travel

    def cancel(self):
        self.cancelled = True


class World:
    """Ticks a search tree with a fake Nav2, scripted tracks and a live coverage grid."""

    def __init__(self, root, cfg, pose=(0.0, 0.0, 0.0), tracks=(), travel=20):
        self.tick, self.pose, self.tracks, self.goals, self.travel = 0, pose, list(tracks), [], \
            travel
        self.cover = search.Coverage(cfg['area'], cfg['cell'], pose, cfg['radius'],
                                     cfg['forget_s'])
        self.fov = (math.radians(cfg['hfov_deg']), cfg['range_m'])
        self.tree = py_trees.trees.BehaviourTree(root)
        self.tree.setup(navigator=self, report=lambda event, **details: None)
        self.board = py_trees.blackboard.Client(name='world')
        for key in ('objects', 'tracks', 'pose', 'now', 'home', 'fov', 'coverage'):
            self.board.register_key(key=key, access=Access.WRITE)
        self.board.home, self.board.fov, self.board.coverage = pose, self.fov, self.cover

    def send(self, route, precise=False, quiet=False, why=''):
        self.goals.append(Goal(self, route, why))
        return self.goals[-1]

    def run(self, ticks=300, script=None):
        for _ in range(ticks):
            goal = self.goals[-1] if self.goals else None
            if goal and not goal.cancelled and self.tick >= goal.at + self.travel - 1:
                self.pose = goal.route[-1]  # arrives a tick before Nav2 says so
            if script:
                script(self)
            self.cover.observe(self.pose, self.fov, self.tick / 10)
            self.board.pose, self.board.now, self.board.tracks = (
                self.pose, self.tick / 10, self.tracks)
            self.board.objects = [(t.label, t.x, t.y) for t in self.tracks if not t.prior]
            self.tree.tick()
            self.tick += 1
            if self.tree.root.status in (Status.SUCCESS, Status.FAILURE):
                return self.tree.root.status.name
        return 'RUNNING'


def test_search_looks_at_the_prior_first_and_succeeds_when_it_appears(cfg):
    target = prior('red', 20, 10)
    world = World(search.search('find red', ['red'], cfg), cfg, tracks=[target])
    view = search.view_pose((20, 10), (0, 0, 0), cfg['standoff'])

    def appear(world):
        if math.dist(world.pose[:2], view[:2]) < 0.5:
            world.tracks = [live('red', 20, 10)]

    assert world.run(script=appear) == 'SUCCESS'
    assert len(world.goals) == 1 and world.goals[0].route == [view]
    assert world.goals[0].why == 'look at prior red'


def test_scan_turns_toward_the_unseen_side(cfg):
    world = World(search.Scan('scan', ['red'], cfg), cfg)
    world.cover.observe((0, 0, math.radians(50)), world.fov, 0.0)  # the left is seen
    world.run(ticks=1)
    goal = world.goals[0]
    assert goal.route[-1][1] < 0 and goal.why.startswith('look toward unexplored bearing -')
    assert len(goal.why) <= 60
    assert world.run() == 'FAILURE' and len(world.goals) <= cfg['max_scans']


@pytest.mark.parametrize('leaf', ['LookAtPriors', 'Scan', 'NextView', 'Sweep'])
def test_leaves_cancel_their_goal_when_stopped(cfg, leaf):
    world = World(getattr(search, leaf)(leaf, ['red'], cfg), cfg, tracks=[prior('red', 15, 0)])
    assert world.run(ticks=3) == 'RUNNING' and len(world.goals) == 1
    world.tree.root.stop(Status.INVALID)
    assert world.goals[0].cancelled


def test_no_goal_is_resent_every_tick(cfg):
    world = World(search.search('find red', ['red'], cfg), cfg)
    world.run(ticks=600)
    sent = [goal.at for goal in world.goals]
    assert 3 <= len(sent) <= 600 / 15
    assert all(later - earlier >= 5 for earlier, later in zip(sent, sent[1:]))
    assert all(len(goal.why) <= 60 for goal in world.goals)
