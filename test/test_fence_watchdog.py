"""Geofence watchdog: outside, or near the edge heading outward, holds; so does no fence."""

import itertools
import json
import math
from pathlib import Path
import time

from mhseals_nav.fence_watchdog import (FenceWatchdog, load_fence, outline, polygon, problem,
                                        project)
import pytest
import yaml

CFG = yaml.safe_load((Path(__file__).parents[1] / 'config/fence_watchdog.yaml')
                     .read_text())['/**']['ros__parameters']
BOX = [0.0, 0.0, 100.0, 0.0, 100.0, 50.0, 0.0, 50.0]      # map metres, x east, y north
MAP = dict(CFG, fence_frame='map', fence_points=BOX)


def at(dog, now, x, y, yaw=0.0, vx=0.0):
    dog.position(now, x, y)
    dog.motion(now, yaw, vx, 0.0)
    dog.command(now)
    return dog.tick(now)


def test_well_inside_requests_nothing_whatever_the_heading():
    dog = FenceWatchdog(MAP)
    for k, yaw in enumerate((0.0, 1.5, 3.1, -1.5)):
        assert not at(dog, k * 2.0, 50.0, 25.0, yaw, 1.0)


def test_outside_holds_only_under_autonomy():
    dog = FenceWatchdog(MAP)
    dog.position(0.0, 120.0, 25.0)
    dog.motion(0.0, math.pi, 0.0, 0.0)       # facing back in, still outside
    assert dog.reason(0.0) == 'outside the fence'
    assert not dog.tick(0.0)
    dog.command(0.0)
    assert dog.tick(0.0)


def test_near_the_edge_holds_only_heading_outward():
    dog = FenceWatchdog(MAP)
    edge = 100.0 - CFG['margin_m'] / 2      # inside the margin of the east edge
    assert at(dog, 0.0, edge, 25.0, 0.0, 0.0)                    # facing east, still
    dog = FenceWatchdog(MAP)
    assert not at(dog, 0.0, edge, 25.0, math.pi, 0.5)            # facing and moving west
    assert not at(dog, 2.0, edge, 25.0, math.pi / 2, 0.5)        # along the edge
    assert at(dog, 4.0, edge, 25.0, math.pi, -0.5)               # backing out east


def test_near_edge_reason_does_not_change_with_the_distance():
    """The node logs on a reason change: a distance in it would log every tick."""
    dog = FenceWatchdog(MAP)
    reasons = set()
    for k, x in enumerate((96.0, 97.0, 98.0)):
        assert at(dog, k * 1.0, x, 25.0)
        reasons.add(dog.reason(k * 1.0))
    assert reasons == {'within the margin heading outward'}
    assert dog.describe(dog.reason(2.0)) == 'within the margin heading outward (2.0 m)'
    assert dog.describe('outside the fence') == 'outside the fence'


def test_outward_angle_and_margin_bounds():
    dog = FenceWatchdog(MAP)
    inside = math.radians(CFG['outward_angle_deg']) - 0.05
    edge = 100.0 - CFG['margin_m'] + 0.1
    assert at(dog, 0.0, edge, 25.0, inside, 0.0)
    dog = FenceWatchdog(MAP)
    assert not at(dog, 0.0, edge, 25.0, inside + 0.1, 0.0)
    assert not at(dog, 2.0, 100.0 - CFG['margin_m'] - 0.1, 25.0, 0.0, 1.0)


def test_near_a_corner_both_edges_count():
    for yaw in (0.0, math.pi / 4, math.pi / 2):                   # east, NE, north
        dog = FenceWatchdog(MAP)
        assert at(dog, 0.0, 98.5, 47.0, yaw, 0.0)                 # 1.5 m from E, 3 m from N
    dog = FenceWatchdog(MAP)
    assert not at(dog, 0.0, 98.5, 47.0, -3 * math.pi / 4, 0.0)


def test_no_valid_fence_holds():
    for points in ([], [0.0, 0.0, 1.0, 1.0], BOX[:-1], BOX[:-2] + [0.0, float('nan')],
                   BOX[:-1] + ['x'], {'a': 1}):
        dog = FenceWatchdog(dict(MAP, fence_points=points))
        assert dog.poly is None
        assert at(dog, 0.0, 50.0, 25.0)
    assert polygon('utm', BOX) is None
    dog = FenceWatchdog({k: v for k, v in CFG.items() if k != 'fence_points'})
    assert at(dog, 0.0, 42.0, -71.0)


def test_stale_position_or_odometry_holds():
    dog = FenceWatchdog(MAP)
    dog.motion(0.0, 0.0, 0.0, 0.0)
    dog.command(0.0)
    assert dog.reason(0.0) == 'position missing or stale' and dog.tick(0.0)
    dog = FenceWatchdog(MAP)
    assert not at(dog, 0.0, 50.0, 25.0)
    late = CFG['max_age_s'] + 0.01
    dog.position(late, 50.0, 25.0)
    dog.command(late)
    assert dog.reason(late) == 'odometry missing or stale' and dog.tick(late)


def test_lat_lon_fence():
    lat0, lon0 = 42.0, -71.0
    d = 0.001                                # ~111 m north, ~83 m east
    dog = FenceWatchdog(dict(CFG, fence_frame='latlon', fence_points=[
        lat0, lon0, lat0, lon0 + d, lat0 + d, lon0 + d, lat0 + d, lon0]))
    east, north = project((lat0, lon0), lat0 + d, lon0 + d)
    assert east == pytest.approx(82.7, abs=0.2) and north == pytest.approx(111.2, abs=0.2)
    assert not at(dog, 0.0, lat0 + d / 2, lon0 + d / 2, 0.0, 1.0)
    assert at(dog, 2.0, lat0 + d / 2, lon0 + d + 1e-5, math.pi, 0.0)   # 0.8 m outside east
    dog = FenceWatchdog(dog.cfg)
    assert at(dog, 0.0, lat0 + d / 2, lon0 + d - 2e-5, 0.0, 0.0)      # 1.7 m in, facing east


def test_fence_file_overrides_the_package_fence(tmp_path):
    day = tmp_path / 'fence.yaml'
    day.write_text(yaml.safe_dump({'fence_frame': 'map', 'fence_points': BOX}))
    cfg = load_fence(dict(CFG, fence_file=str(day), fence_points=[1.0]))
    assert (cfg['fence_frame'], cfg['fence_points']) == ('map', BOX)
    for text in ('- 1\n', ': :\n', 'fence_frame: map\n'):
        day.write_text(text)
        assert FenceWatchdog(load_fence(dict(CFG, fence_file=str(day)))).poly is None
    assert load_fence(dict(CFG, fence_file=str(tmp_path / 'missing.yaml'))) == \
        dict(CFG, fence_file=str(tmp_path / 'missing.yaml'))


def test_unreadable_fence_file_holds(tmp_path, monkeypatch):
    """Even with a valid fence in the parameters: the day's file was meant to replace it."""
    day = tmp_path / 'fence.yaml'
    day.write_bytes(b'fence_frame: map\nfence_points: [\xff\xfe]\n')   # not UTF-8
    dog = FenceWatchdog(load_fence(dict(MAP, fence_file=str(day))))
    assert dog.poly is None and at(dog, 0.0, 50.0, 25.0)
    day.write_text(yaml.safe_dump({'fence_frame': 'map', 'fence_points': BOX}))
    day.chmod(0)                       # root still reads it, so also refuse the read itself

    def refuse(*_, **__):
        raise PermissionError(13, 'Permission denied', str(day))
    monkeypatch.setattr(Path, 'read_text', refuse)
    dog = FenceWatchdog(load_fence(dict(MAP, fence_file=str(day))))
    assert dog.poly is None and at(dog, 0.0, 50.0, 25.0)


STATE_KEYS = {'valid', 'frame', 'points', 'margin_m', 'outward_angle_deg', 'outward_speed_mps',
              'reason', 'error', 'stamp_ns'}


def test_state_of_a_map_fence_is_its_points():
    dog = FenceWatchdog(MAP)
    state = dog.state(0.0, 7)
    assert set(state) == STATE_KEYS
    assert state['valid'] and state['frame'] == 'map'
    assert state['points'] == BOX and state['stamp_ns'] == 7 and not state['error']
    assert state['margin_m'] == CFG['margin_m']
    assert state['reason'] == 'position missing or stale'
    at(dog, 0.0, 99.0, 25.0)
    assert dog.state(0.0)['reason'] == 'within the margin heading outward'
    assert json.loads(json.dumps(state)) == state


def test_state_without_a_fence_is_invalid():
    state = FenceWatchdog(dict(MAP, fence_points=[])).state(0.0)
    assert set(state) == STATE_KEYS
    assert not state['valid'] and state['points'] is None and state['error']


def test_state_of_a_lat_lon_fence_is_its_raw_lat_lon_whatever_the_position():
    """No map copy: the TUI checks goals in lat/lon (toLL), so a rotated map cannot shift it."""
    points = [42.0, -71.0, 42.0, -70.999, 42.001, -70.999, 42.001, -71.0]
    dog = FenceWatchdog(dict(CFG, fence_frame='latlon', fence_points=points))
    for state in (dog.state(0.0), dog.state(100.0)):         # no fix yet, then a stale one
        assert set(state) == STATE_KEYS and state['valid'] and state['frame'] == 'latlon'
        assert state['points'] == points
        dog.position(0.0, 42.0005, -70.9995)


LATLON = [42.0, -71.0, 42.0, -70.999, 42.001, -70.999, 42.001, -71.0]


def test_anchor_moves_a_map_fence_as_the_start_placed_it():
    dog = FenceWatchdog(MAP)
    assert dog.anchor(0.0) == (False, 'no boat position (missing or older than max_age_s)')
    at(dog, 0.0, 10.0, 20.0)
    ok, message = dog.anchor(0.0)
    assert ok and 'fence anchored at 10, 20 (map)' == message
    assert dog.cfg['fence_points'] == [10.0, 20.0, 110.0, 20.0, 110.0, 70.0, 10.0, 70.0]
    assert dog.state(0.0)['points'] == dog.cfg['fence_points']
    assert MAP['fence_points'] == BOX                       # the package config is untouched
    assert dog.reason(0.0) == 'on the fence'                # BOX's corner is the origin
    at(dog, 0.5, 105.0, 50.0)                               # outside the old box, in the new
    assert dog.reason(0.5) == ''
    at(dog, 1.0, 5.0, 50.0)                                 # in the old box, out of the new
    assert dog.reason(1.0) == 'outside the fence'
    at(dog, 1.5, 15.0, 25.0)                                # anchoring again is relative too
    assert dog.anchor(1.5)[0]
    assert dog.cfg['fence_points'][:2] == [15.0, 25.0]


def test_anchor_refuses_without_a_fresh_position_or_a_fence():
    dog = FenceWatchdog(MAP)
    at(dog, 0.0, 10.0, 20.0)
    ok, why = dog.anchor(CFG['max_age_s'] + 0.01)
    assert not ok and 'no boat position' in why and dog.cfg['fence_points'] == BOX
    dog = FenceWatchdog(dict(MAP, fence_points=[]))
    at(dog, 0.0, 10.0, 20.0)
    assert dog.anchor(0.0) == (False, 'no valid fence configured')


def test_anchor_refuses_while_moving_or_without_odometry():
    dog = FenceWatchdog(MAP)
    dog.position(0.0, 10.0, 20.0)
    ok, why = dog.anchor(0.0)
    assert not ok and 'no odometry' in why and dog.cfg['fence_points'] == BOX
    at(dog, 0.0, 10.0, 20.0, 1.0, CFG['anchor_max_speed_mps'] + 0.1)
    ok, why = dog.anchor(0.0)
    assert not ok and 'moving' in why and dog.cfg['fence_points'] == BOX
    at(dog, 0.5, 10.0, 20.0, 1.0, CFG['anchor_max_speed_mps'] / 2)
    assert dog.anchor(0.5)[0]


def test_anchor_keeps_a_lat_lon_course_area_fixed():
    dog = FenceWatchdog(dict(CFG, fence_frame='latlon', fence_points=LATLON))
    assert dog.anchor(0.0) == (True, 'course area fixed')      # no position needed
    at(dog, 0.0, 42.1, -71.2)
    assert dog.anchor(0.0) == (True, 'course area fixed')
    assert dog.cfg['fence_points'] == LATLON and dog.reason(0.0) == 'outside the fence'


def test_outline_of_a_map_fence_is_its_closed_points():
    assert outline('map', BOX) == [(0.0, 0.0), (100.0, 0.0), (100.0, 50.0), (0.0, 50.0),
                                   (0.0, 0.0)]
    assert outline('map', BOX[:-1]) is None and outline('utm', BOX) is None
    dog = FenceWatchdog(MAP)
    assert dog.outline(0.0) == outline('map', BOX)          # no position needed
    at(dog, 0.0, 10.0, 20.0)
    dog.anchor(0.0)
    assert dog.outline(100.0)[0] == (10.0, 20.0)


def test_outline_of_a_lat_lon_fence_is_placed_through_the_boat():
    assert outline('latlon', LATLON) is None                # nowhere to place it yet
    line = outline('latlon', LATLON, boat_raw=(42.0, -71.0), boat_map=(5.0, -3.0))
    assert len(line) == 5 and line[0] == line[-1] == (5.0, -3.0)
    east, north = project((42.0, -71.0), 42.001, -70.999)
    assert line[2] == pytest.approx((5.0 + east, -3.0 + north))
    dog = FenceWatchdog(dict(CFG, fence_frame='latlon', fence_points=LATLON))
    dog.position(0.0, 42.0, -71.0)
    assert dog.outline(0.0) is None                         # no map pose yet
    dog.map_position(0.0, 5.0, -3.0)
    assert dog.outline(0.0) == line
    assert dog.outline(CFG['max_age_s'] + 0.01) is None     # stale


def test_back_inside_holds_no_more():
    dog = FenceWatchdog(MAP)
    assert at(dog, 0.0, 120.0, 25.0)
    assert not at(dog, 0.5, 50.0, 25.0)
    assert not dog.reason(0.5)


BOWTIE = [42.0, -71.0, 42.001, -70.999, 42.0, -70.999, 42.001, -71.0]


def test_corners_must_be_a_simple_polygon():
    assert problem('latlon', LATLON) == ''
    assert 'simple' in problem('latlon', BOWTIE)
    assert 'simple' in problem('latlon', LATLON[:2] * 2 + LATLON[4:6])          # repeated
    assert 'simple' in problem('latlon', [42.0, -71.0, 42.0005, -71.0, 42.001, -71.0])
    assert 'at least 3' in problem('latlon', LATLON[:4])
    assert 'finite' in problem('latlon', LATLON[:-1] + [float('nan')])
    assert 'outside lat' in problem('latlon', [91.0] + LATLON[1:])
    assert polygon('map', [0, 0, 10, 10, 10, 0, 0, 10]) is None             # map too


def test_course_corners_replace_the_fence_whole_or_not_at_all():
    dog = FenceWatchdog(MAP)
    dog.fix(0.0, 42.0005, -70.9995)                  # kept while the fence is in map
    at(dog, 0.0, 50.0, 25.0)
    assert 'simple' in dog.set_corners(BOWTIE)
    assert dog.frame == 'map' and dog.cfg['fence_points'] == BOX
    assert dog.set_corners(LATLON) == ''
    assert dog.frame == 'latlon' and dog.state(0.0)['points'] == LATLON
    assert dog.reason(0.0) == ''                     # placed at once by the fix it had
    dog.fix(0.5, 42.002, -70.9995)
    assert dog.reason(0.5) == 'outside the fence'


def test_fence_file_or_parameter_course_corners_are_a_lat_lon_area(tmp_path):
    day = tmp_path / 'fence.yaml'
    day.write_text(yaml.safe_dump({'course': {'corners': LATLON}}))
    for cfg in (dict(MAP, fence_file=str(day)), dict(MAP, fence_file='', course={
            'corners': LATLON})):
        cfg = load_fence(cfg)
        assert cfg['fence_frame'] == 'latlon' and cfg['fence_points'] == LATLON


def test_fence_file_course_on_exit_applies_at_startup_or_warns(tmp_path, caplog):
    """The TUI saves course.on_exit in the fence file; a restart keeps it, a bad one warns."""
    day = tmp_path / 'fence.yaml'
    day.write_text(yaml.safe_dump({'course': {'corners': LATLON, 'on_exit': 'return'}}))
    cfg = load_fence(dict(MAP, fence_file=str(day)))
    assert cfg['on_exit'] == 'return' and FenceWatchdog(cfg).cfg['on_exit'] == 'return'
    assert CFG['on_exit'] == 'alert' and not caplog.records
    for bad in ('drift', ['return'], 3):
        day.write_text(yaml.safe_dump({'course': {'corners': LATLON, 'on_exit': bad}}))
        caplog.clear()
        cfg = load_fence(dict(MAP, fence_file=str(day)))
        assert cfg['on_exit'] == 'alert' and FenceWatchdog(cfg).poly is not None
        assert [r.levelname for r in caplog.records] == ['WARNING']
        assert 'course.on_exit' in caplog.text and 'stays alert' in caplog.text
    day.write_text(yaml.safe_dump({'course': {'corners': LATLON}}))
    caplog.clear()
    assert load_fence(dict(MAP, fence_file=str(day)))['on_exit'] == 'alert' and not caplog.records


def test_on_exit_is_alert_or_return():
    dog = FenceWatchdog(MAP)
    assert dog.set_on_exit('return') == '' and dog.cfg['on_exit'] == 'return'
    assert 'neither' in dog.set_on_exit('drift') and dog.cfg['on_exit'] == 'return'
    with pytest.raises(ValueError):
        FenceWatchdog(dict(MAP, on_exit='drift'))


def test_status_says_inside_and_the_distance_to_the_edge():
    dog = FenceWatchdog(MAP)
    assert dog.status(0.0) == {'inside': None, 'distance_to_edge': None, 'frame': 'map',
                               'on_exit': 'alert', 'corners': BOX, 'returning': False,
                               'paused': False}
    at(dog, 0.0, 90.0, 25.0)
    assert (dog.status(0.0)['inside'], dog.status(0.0)['distance_to_edge']) == (True, 10.0)
    at(dog, 0.5, 103.0, 25.0)
    assert (dog.status(0.5)['inside'], dog.status(0.5)['distance_to_edge']) == (False, 3.0)
    assert dog.status(5.0)['inside'] is None                        # stale
    assert FenceWatchdog(dict(MAP, fence_points=[])).status(0.0)['corners'] is None
    json.dumps(dog.status(0.5))


RETURN = dict(MAP, on_exit='return')
LEAD = CFG['return_lead']


def leave(dog, now):
    """Just east of the fence under autonomy; whether a hold went out."""
    return at(dog, now, 104.0, 25.0)


def left(on_exit='return'):
    """Outside with the return goal out (past return_lead)."""
    dog = FenceWatchdog(dict(RETURN, on_exit=on_exit))
    leave(dog, 0.0)
    leave(dog, LEAD)
    dog.orders.clear()
    return dog


def mux(dog, now, *causes, source='auto'):
    dog.mux(now, json.dumps({'input': source, 'holds': [{'cause': c} for c in causes]}))


def test_return_pauses_the_runners_then_sends_one_slow_goal_inside():
    dog = FenceWatchdog(RETURN)
    assert not leave(dog, 0.0)
    assert dog.orders == [] and dog.pause() == {
        'paused': True, 'cause': 'geofence', 'by': 'fence_watchdog',
        'why': 'returning inside the fence'}
    assert not leave(dog, LEAD)                     # the runners had return_lead to yield
    (order, goal), = dog.orders
    assert order == 'go' and goal == pytest.approx((100.0 - CFG['return_inset'], 25.0, math.pi))
    assert dog.status(LEAD)['returning'] and dog.status(LEAD)['paused']
    dog.orders.clear()
    for k in range(1, 10):                          # outside a while: no hold, no second goal
        assert not leave(dog, LEAD + k * 1.0)
    assert dog.orders == []
    assert not at(dog, 11.0, 95.0, 25.0, math.pi, 0.5)       # back in, heading in
    assert dog.returning is not None                         # on to the interior point


def test_back_inside_the_return_holds_and_stays_paused_until_released():
    dog = left()
    at(dog, 5.0, 95.0, 25.0, math.pi, 0.5)
    dog.return_result(6.0, 'succeeded')
    assert dog.orders == [('cancel', 'goal succeeded')] and dog.returning is None
    paused = dog.pause()
    assert paused['paused'] and paused['why'].startswith('back inside the fence')
    assert 'release the geofence hold' in dog.reason(6.0)
    assert dog.tick(6.0)                             # holds whatever the commands
    assert not dog.tick(6.0 + CFG['repeat_s'] / 2)
    mux(dog, 6.5, 'geofence', source='hold')         # the mux holds: it stops asking
    assert not dog.tick(6.5 + CFG['repeat_s'])
    dog.fcu(10.0, False)                             # disarmed and clear: still its hold
    for k in range(10):
        at(dog, 10.0 + k, 95.0, 25.0)
    assert dog.pause()['paused']
    mux(dog, 21.0)                                   # U or boat_agent release: resume
    assert not at(dog, 21.0, 95.0, 25.0)
    assert not dog.pause()['paused'] and dog.reason(21.0) == ''


def test_station_keeping_after_the_return_never_starts_a_new_return():
    dog = left()
    dog.return_result(1.0, 'succeeded')              # ended outside: failed, alert holds
    assert dog.parked and leave(dog, 1.0)
    dog.orders.clear()
    mux(dog, 1.5, 'geofence', source='hold')
    for k in range(5):                               # parked: no new return goal
        leave(dog, 2.0 + k)
    assert dog.orders == []


def test_return_refused_while_the_runners_wind_down_is_sent_again():
    dog = left()
    dog.return_result(1.0, 'rejected')
    assert dog.orders == [] and dog.returning is not None
    leave(dog, 1.0 + LEAD)
    assert [o for o, _ in dog.orders] == ['go']
    for k in range(2, CFG['return_tries'] + 1):
        dog.orders.clear()
        leave(dog, k * 2.0)
        dog.return_result(k * 2.0, 'rejected')
        leave(dog, k * 2.0 + LEAD)
    assert [o for o, _ in dog.orders] == ['cancel', 'failed']


def test_return_not_back_inside_in_time_holds_until_inside():
    dog = left()
    assert not leave(dog, CFG['return_timeout'])
    assert leave(dog, CFG['return_timeout'] + 0.1)
    assert [o for o, _ in dog.orders] == ['cancel', 'failed']
    dog.orders.clear()
    mux(dog, CFG['return_timeout'] + 1.0, 'geofence', source='hold')
    mux(dog, CFG['return_timeout'] + 1.5)                        # released, still outside
    assert leave(dog, CFG['return_timeout'] + 2.0)               # the hold repeats as alert
    assert dog.orders == [] and dog.returning is None
    at(dog, 70.0, 50.0, 25.0)                                    # inside: return armed again
    assert not leave(dog, 71.0) and not leave(dog, 71.0 + LEAD)
    assert dog.orders[0][0] == 'go'


def test_return_gives_way_to_the_operator_then_holds():
    dog = left()
    mux(dog, 1.0, source='manual')
    assert leave(dog, 1.0)                           # the hold waits in the mux for auto
    assert [o for o, _ in dog.orders] == ['cancel', 'failed']


def test_return_goal_aborted_or_replaced_holds():
    dog = left()
    dog.return_result(1.0, 'aborted or replaced')
    assert dog.orders == [('cancel', 'goal aborted or replaced'),
                          ('failed', 'the return goal ended outside (aborted or replaced)')]
    assert leave(dog, 1.0)


def test_return_stops_on_alert_or_stale_position():
    dog = left()
    dog.set_on_exit('alert')
    assert leave(dog, 1.0) and dog.orders == [('cancel', 'on_exit is alert')]
    assert not dog.pause()['paused']                 # alert's hold pauses through the mux
    dog = left()
    dog.command(2.0)
    assert dog.tick(2.0) and [o for o, _ in dog.orders] == ['cancel', 'failed']


def test_return_goal_of_a_lat_lon_course_area_is_in_map():
    cfg = dict(CFG, fence_frame='latlon', fence_points=LATLON, on_exit='return')
    lat, lon = 42.0005, -70.9989                      # east of the east edge (-70.999)
    dog = FenceWatchdog(cfg)
    dog.map_position(0.0, 7.0, -2.0)
    assert not at(dog, 0.0, lat, lon)
    dog.map_position(LEAD, 7.0, -2.0)
    assert not at(dog, LEAD, lat, lon)
    (order, (x, y, yaw)), = dog.orders
    east = project((lat, lon), lat, -70.999)[0] - CFG['return_inset']
    assert (x, y, yaw) == pytest.approx((7.0 + east, -2.0, math.pi), abs=0.01)
    dog = FenceWatchdog(cfg)                          # no map pose: hold as alert
    assert at(dog, 0.0, lat, lon) and dog.orders[0][0] == 'failed'


def test_node_holds_outside_a_map_fence():
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from ros_domain import require_isolated_domain
    from watchdog_cycle import flip_twice
    require_isolated_domain()
    from mhseals_nav.fence_watchdog import node_class
    rclpy.init()
    try:
        dog = node_class()(parameter_overrides=[
            Parameter('fence_file', value=''), Parameter('fence_frame', value='map'),
            Parameter('fence_points', value=BOX)])
        probe = rclpy.create_node('fence_probe')
        got = []
        probe.create_subscription(String, CFG['request_topic'],
                                  lambda m: got.append(json.loads(m.data)), 10)
        cmd = probe.create_publisher(Twist, CFG['command_topic'], 10)
        odom = probe.create_publisher(Odometry, CFG['odom_topic'], 10)
        ex = SingleThreadedExecutor()
        ex.add_node(dog)
        ex.add_node(probe)

        def odometry(x):
            msg = Odometry()
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, 25.0
            msg.pose.pose.orientation.w = 1.0
            msg.twist.twist.linear.x = 1.0
            return msg

        def drive(x, seconds, until=None, autonomy=True):
            end = time.monotonic() + seconds
            while time.monotonic() < end and not (until and until()):
                odom.publish(odometry(x))
                if autonomy:
                    cmd.publish(Twist())
                ex.spin_once(timeout_sec=0.05)

        drive(50.0, 0.5, autonomy=False)        # odometry before autonomy starts
        drive(50.0, 1.0)
        assert dog.logic.poly is not None and dog.logic.p == (50.0, 25.0) and not got
        drive(101.0, 3.0, until=lambda: len(got) >= 2)
        assert len(got) >= 2

        def at_x(xs):
            xs = iter(xs)

            def feed():
                odom.publish(odometry(next(xs)))
                cmd.publish(Twist())
            return feed

        # Near the east edge heading east, the distance changing every message.
        near = at_x(97.0 + 0.01 * (k % 100) for k in itertools.count())
        flip_twice(ex, got, near, at_x(itertools.repeat(50.0)))
        assert {g['op'] for g in got} == {'hold'}
        assert {g['cause'] for g in got} == {'geofence'}
        assert {g['by'] for g in got} == {'fence_watchdog'} and all(g['token'] for g in got)

        # The fence it enforces is latched on fence_topic: a late subscriber gets it.
        fences = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        drive(50.0, 1.2)
        probe.create_subscription(String, CFG['fence_topic'],
                                  lambda m: fences.append(json.loads(m.data)), latched)
        drive(50.0, 1.0, until=lambda: fences)
        assert fences and fences[0]['valid'] and fences[0]['points'] == BOX

        # A failing publish never stops the holds.
        def broken(*_, **__):
            raise ValueError('boom')
        dog.logic.state = broken
        got.clear()
        drive(101.0, 3.0, until=lambda: len(got) >= 2)
        assert len(got) >= 2
    finally:
        rclpy.try_shutdown()


def test_node_anchor_service_moves_the_latched_marker():
    rclpy = pytest.importorskip('rclpy')
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_srvs.srv import Trigger
    from visualization_msgs.msg import Marker
    from ros_domain import require_isolated_domain
    require_isolated_domain()
    from mhseals_nav.fence_watchdog import node_class
    rclpy.init()
    try:
        dog = node_class()(parameter_overrides=[
            Parameter('fence_file', value=''), Parameter('fence_frame', value='map'),
            Parameter('fence_points', value=BOX)])
        probe = rclpy.create_node('fence_anchor_probe')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        markers = []
        probe.create_subscription(Marker, CFG['marker_topic'], markers.append, latched)
        odom = probe.create_publisher(Odometry, CFG['odom_topic'], 10)
        anchor = probe.create_client(Trigger, CFG['anchor_service'])
        ex = SingleThreadedExecutor()
        ex.add_node(dog)
        ex.add_node(probe)

        def spin(seconds, until):
            end = time.monotonic() + seconds
            while time.monotonic() < end and not until():
                ex.spin_once(timeout_sec=0.05)

        def call():
            future = anchor.call_async(Trigger.Request())
            spin(3.0, future.done)
            return future.result()

        spin(3.0, lambda: markers)
        first = markers[-1]
        assert first.type == Marker.LINE_STRIP and first.header.frame_id == 'map'
        assert [(p.x, p.y) for p in first.points] == outline('map', BOX)
        assert anchor.wait_for_service(timeout_sec=3.0)
        refused = call()
        assert not refused.success and 'no boat position' in refused.message

        msg = Odometry()
        msg.pose.pose.position.x, msg.pose.pose.position.y = 10.0, 20.0
        msg.pose.pose.orientation.w = 1.0
        odom.publish(msg)
        spin(1.0, lambda: dog.logic.p == (10.0, 20.0))
        markers.clear()
        done = call()
        assert done.success and 'anchored' in done.message
        spin(2.0, lambda: markers)
        assert [(p.x, p.y) for p in markers[-1].points][0] == (10.0, 20.0)
    finally:
        rclpy.try_shutdown()


def test_node_course_corners_on_exit_status_and_red_marker():
    rclpy = pytest.importorskip('rclpy')
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker
    from ros_domain import require_isolated_domain
    require_isolated_domain()
    from mhseals_nav.fence_watchdog import node_class
    rclpy.init()
    try:
        dog = node_class()(parameter_overrides=[
            Parameter('fence_file', value=''), Parameter('fence_frame', value='map'),
            Parameter('fence_points', value=BOX)])
        probe = rclpy.create_node('fence_course_probe')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        statuses, markers = [], []
        probe.create_subscription(String, CFG['status_topic'],
                                  lambda m: statuses.append(json.loads(m.data)), 10)
        probe.create_subscription(Marker, CFG['marker_topic'], markers.append, latched)
        odom = probe.create_publisher(Odometry, CFG['odom_topic'], 10)
        gps = probe.create_publisher(NavSatFix, CFG['gps_topic'], 10)
        ex = SingleThreadedExecutor()
        ex.add_node(dog)
        ex.add_node(probe)

        def spin(seconds, until, feed=None):
            end = time.monotonic() + seconds
            while time.monotonic() < end and not until():
                if feed:
                    feed()
                ex.spin_once(timeout_sec=0.05)

        spin(2.0, lambda: statuses)
        assert statuses[-1]['frame'] == 'map' and statuses[-1]['corners'] == BOX
        refused = dog.set_parameters_atomically([Parameter('course.corners', value=BOWTIE),
                                                 Parameter('on_exit', value='return')])
        assert not refused.successful and 'simple' in refused.reason
        assert dog.logic.frame == 'map' and dog.get_parameter('on_exit').value == 'alert'
        done = dog.set_parameters_atomically([Parameter('course.corners', value=LATLON),
                                              Parameter('on_exit', value='return')])
        assert done.successful
        assert list(dog.get_parameter('course.corners').value) == LATLON
        spin(2.0, lambda: statuses[-1]['corners'] == LATLON)
        assert statuses[-1]['frame'] == 'latlon' and statuses[-1]['on_exit'] == 'return'

        def feed():                     # east of the course area, no autonomy
            fix = NavSatFix(latitude=42.0005, longitude=-70.9985)
            fix.status.status = 0
            gps.publish(fix)
            msg = Odometry()
            msg.pose.pose.orientation.w = 1.0
            odom.publish(msg)
        spin(3.0, lambda: statuses[-1]['inside'] is False and markers and
             markers[-1].color.g == 0.0, feed)
        assert statuses[-1]['inside'] is False and statuses[-1]['distance_to_edge'] > 30
        assert (markers[-1].color.r, markers[-1].color.g) == (1.0, 0.0)
        assert not statuses[-1]['returning']           # no autonomy: nothing to return
    finally:
        rclpy.try_shutdown()


def test_node_return_sends_a_capped_goal_then_holds_when_it_ends_outside():
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.msg import SpeedLimit
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionServer
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from ros_domain import require_isolated_domain
    require_isolated_domain()
    from mhseals_nav.fence_watchdog import node_class
    rclpy.init()
    try:
        dog = node_class()(parameter_overrides=[
            Parameter('fence_file', value=''), Parameter('fence_frame', value='map'),
            Parameter('fence_points', value=BOX), Parameter('on_exit', value='return')])
        probe = rclpy.create_node('fence_return_probe')
        goals, limits, holds, pauses = [], [], [], []

        def execute(handle):            # Nav2 stand-in: done at once, the boat still outside
            goals.append(handle.request.pose)
            handle.succeed()
            return NavigateToPose.Result()
        ActionServer(probe, NavigateToPose, CFG['return_action'], execute)
        probe.create_subscription(SpeedLimit, CFG['speed_limit_topic'],
                                  lambda m: limits.append(m.speed_limit), 10)
        probe.create_subscription(String, CFG['request_topic'],
                                  lambda m: holds.append(json.loads(m.data)), 10)
        probe.create_subscription(String, CFG['pause_topic'],
                                  lambda m: pauses.append(json.loads(m.data)), 10)
        cmd = probe.create_publisher(Twist, CFG['command_topic'], 10)
        odom = probe.create_publisher(Odometry, CFG['odom_topic'], 10)
        ex = SingleThreadedExecutor()
        ex.add_node(dog)
        ex.add_node(probe)
        end = time.monotonic() + 3.0
        while time.monotonic() < end and not (goals and holds and 0.0 in limits
                                                and len(pauses) > 1):
            msg = Odometry()
            msg.pose.pose.position.x, msg.pose.pose.position.y = 104.0, 25.0
            msg.pose.pose.orientation.w = 1.0
            odom.publish(msg)
            cmd.publish(Twist())
            ex.spin_once(timeout_sec=0.05)
        pose = goals[0].pose
        assert (pose.position.x, pose.position.y) == (100.0 - CFG['return_inset'], 25.0)
        assert goals[0].header.frame_id == CFG['marker_frame']
        assert limits[0] == CFG['return_speed'] and 0.0 in limits     # capped, then lifted
        assert holds and holds[0]['op'] == 'hold' and holds[0]['cause'] == 'geofence'
        assert pauses[0]['paused'] and pauses[0]['why'] == 'returning inside the fence'
        assert pauses[-1]['paused'] and 'release the geofence hold' in pauses[-1]['why']
    finally:
        rclpy.try_shutdown()
