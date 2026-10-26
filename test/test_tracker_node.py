"""object_tracker node: observations + TF in, tracks, objects, grid, status and markers out."""

import time

import pytest

rclpy = pytest.importorskip('rclpy')


@pytest.fixture
def ros(tmp_path, monkeypatch):
    from ros_domain import require_isolated_domain
    require_isolated_domain()
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path))
    rclpy.init()
    yield tmp_path
    rclpy.try_shutdown()


def test_tracker_publishes_every_output(ros):
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import SingleThreadedExecutor
    from tf2_ros import TransformBroadcaster
    from vision_msgs.msg import Detection3DArray
    from visualization_msgs.msg import MarkerArray

    from mhseals_nav.msg import Observation, ObservationArray, Track, TrackArray
    from mhseals_nav.object_tracker import (
        LATCHED, OBSERVATIONS, RELIABLE, ObjectTracker)

    tracker = ObjectTracker()
    probe = rclpy.create_node('tracker_probe')
    got = {}

    def keep(name):
        return lambda msg: got.__setitem__(name, msg)
    probe.create_subscription(TrackArray, '/perception/tracks', keep('tracks'), RELIABLE)
    probe.create_subscription(Detection3DArray, '/objects/map', keep('objects'), RELIABLE)
    probe.create_subscription(OccupancyGrid, '/perception/track_grid', keep('grid'), LATCHED)
    probe.create_subscription(DiagnosticArray, '/perception/status', keep('status'), RELIABLE)
    probe.create_subscription(MarkerArray, '/tracked_objects', keep('markers'), RELIABLE)
    probe.create_subscription(MarkerArray, '/perception/detection_markers', keep('boxes'),
                              RELIABLE)
    pub = probe.create_publisher(ObservationArray, '/perception/observations', OBSERVATIONS)
    tf = TransformBroadcaster(probe)
    ex = SingleThreadedExecutor()
    ex.add_node(tracker)
    ex.add_node(probe)

    def transforms(stamp):
        out = []
        for parent, child, x in (('map', 'odom', 10.0), ('odom', 'base_link', 1.0),
                                 ('base_link', 'cam', 0.5)):
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
            t.transform.translation.x = x
            t.transform.rotation.w = 1.0
            out.append(t)
        return out

    end = time.monotonic() + 8.0
    while time.monotonic() < end:
        stamp = tracker.get_clock().now().to_msg()
        tf.sendTransform(transforms(stamp))
        ex.spin_once(timeout_sec=0.02)
        msg = ObservationArray(backend='sim')
        msg.header.stamp, msg.header.frame_id = stamp, 'cam'
        ob = Observation(class_id='red_buoy', score=0.9)
        ob.position.x = 4.0
        ob.size.x = ob.size.y = ob.size.z = 0.4
        msg.observations.append(ob)
        pub.publish(msg)
        for _ in range(3):
            ex.spin_once(timeout_sec=0.02)
        tracks = got.get('tracks')
        if tracks and tracks.tracks and {'grid', 'status', 'markers', 'boxes'} <= set(got):
            break
    (track,) = got['tracks'].tracks
    assert got['tracks'].header.frame_id == 'map'
    assert track.class_id == 'red_buoy' and track.source == Track.SOURCE_LIVE
    assert track.motion == Track.MOTION_STATIC
    assert track.pose.pose.position.x == pytest.approx(15.5, abs=0.2)   # 10 + 1 + 0.5 + 4
    (det,) = got['objects'].detections
    assert det.results[0].hypothesis.class_id == 'red_buoy'
    grid = got['grid']
    assert grid.header.frame_id == 'odom' and grid.info.width == 160
    assert max(grid.data) == 100 and min(grid.data) == 0
    names = {s.name for s in got['status'].status}
    assert names == {'perception/sim/tracker', 'perception/sim/heading'}
    tracker_stage = next(s for s in got['status'].status if s.name.endswith('tracker'))
    keys = {kv.key for kv in tracker_stage.values}
    assert {'hz', 'latency_ms', 'tracks_static', 'tracks_prior', 'grid_age_s',
            'conf_red_buoy', 'n_red_buoy', 'id_assoc_share'} <= keys
    assert {m.ns for m in got['markers'].markers} >= {'tracked_objects', 'tracked_labels'}
    assert {m.ns for m in got['boxes'].markers} >= {'detection_boxes'}
    tracker.destroy_node()


def test_publish_on_update_sends_tracks_after_an_update_at_most_rate_hz(ros):
    from mhseals_nav.msg import ObservationArray
    from mhseals_nav.object_tracker import ObjectTracker

    tracker = ObjectTracker()
    assert tracker.cfg['publish_on_update']
    sent, stamp = [], [1.0]
    # drain steps the filter to the newest frame's stamp; unchanged stamp = no update.
    tracker.drain = lambda: setattr(tracker.filter, 'last_stamp', stamp[0])
    tracker.publish_tracks = lambda: (sent.append(stamp[0]),
                                      setattr(tracker, 'published_at', time.monotonic()))
    tracker.on_observations(ObservationArray())
    assert sent == [1.0]                      # at once, not on the next tick
    stamp[0] = 2.0
    tracker.on_observations(ObservationArray())
    tracker.tick()
    assert sent == [1.0]                      # within 0.9 / rate_hz of the last: held
    tracker.published_at -= 1.0
    tracker.on_observations(ObservationArray())
    assert sent == [1.0]                      # due, but no new update
    tracker.tick()
    assert sent == [1.0, 2.0]                 # the timer still publishes when due
    tracker.cfg['publish_on_update'] = False
    tracker.tick()
    assert sent == [1.0, 2.0, 2.0]            # off: every tick, as before
    tracker.destroy_node()


def test_course_map_saved_through_toll_and_loaded_as_priors(tmp_path, monkeypatch):
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from robot_localization.srv import FromLL, ToLL
    from tf2_ros import TransformBroadcaster
    from ros_domain import require_isolated_domain

    from mhseals_nav import course_map
    from mhseals_nav.msg import Observation, ObservationArray, Track
    from mhseals_nav.object_tracker import OBSERVATIONS, ObjectTracker

    require_isolated_domain()
    monkeypatch.setenv('MHSEALS_DATA', str(tmp_path))
    rclpy.init(args=['--ros-args', '-p', 'course_map.save_period_s:=0.3',
                     '-p', 'course_map.save_min_hits:=3', '-p', 'miss_in_fov_s:=0.5'])
    try:
        probe = rclpy.create_node('ll_probe')

        def to_ll(request, response):
            response.ll_point.latitude = 42.0 + request.map_point.y * 1e-5
            response.ll_point.longitude = -71.0 + request.map_point.x * 1e-5
            return response

        def from_ll(request, response):
            response.map_point.x = (request.ll_point.longitude + 71.0) * 1e5
            response.map_point.y = (request.ll_point.latitude - 42.0) * 1e5
            return response
        probe.create_service(ToLL, '/toLL', to_ll)
        probe.create_service(FromLL, '/fromLL', from_ll)
        tf = TransformBroadcaster(probe)
        pub = probe.create_publisher(ObservationArray, '/perception/observations', OBSERVATIONS)
        gps = probe.create_publisher(Odometry, '/odom/gps', 1)
        datum = {'set': False}       # navsat publishes odometry/gps once its datum is set

        def spin(tracker, seconds, observe, until):
            ex = SingleThreadedExecutor()
            ex.add_node(tracker)
            ex.add_node(probe)
            end = time.monotonic() + seconds
            while time.monotonic() < end and not until():
                stamp = tracker.get_clock().now().to_msg()
                out = []
                for parent, child, x in (('map', 'odom', 2.0), ('odom', 'base_link', 0.0),
                                         ('base_link', 'cam', 0.0)):
                    t = TransformStamped()
                    t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
                    t.transform.translation.x = x
                    t.transform.rotation.w = 1.0
                    out.append(t)
                tf.sendTransform(out)
                if datum['set']:
                    gps.publish(Odometry())
                if observe:                 # True: a buoy and a boat; 'nothing': empty frames
                    msg = ObservationArray(backend='sim')
                    msg.header.stamp, msg.header.frame_id = stamp, 'cam'
                    if observe is True:
                        ob = Observation(class_id='green_buoy', score=0.9)
                        ob.position.x, ob.position.y = 5.0, 1.0
                        msg.observations.append(ob)
                        # A boat holding still turns STATIC, but is never a course prior.
                        boat = Observation(class_id='yellow_target_boat', score=0.9)
                        boat.position.x, boat.position.y = 5.0, -2.0
                        msg.observations.append(boat)
                    pub.publish(msg)
                for _ in range(3):
                    ex.spin_once(timeout_sec=0.02)
            ex.remove_node(tracker)
            ex.remove_node(probe)

        first = ObjectTracker()
        file = course_map.path('boat')
        mark = {}

        def saved_after_boat_static():
            if 'at' not in mark and any(t.class_id == 'yellow_target_boat' and
                                        t.motion == Track.MOTION_STATIC
                                        for t in first.filter.tracks):
                mark['at'] = time.monotonic()
            return file.exists() and 'at' in mark and time.monotonic() - mark['at'] > 1.0
        spin(first, 12.0, True, saved_after_boat_static)
        assert 'at' in mark, 'the still boat never turned STATIC'
        first.destroy_node()
        rows = course_map.load(file, course_map.datetime.now(course_map.timezone.utc), 3)
        assert [r['class_id'] for r in rows] == ['green_buoy']
        assert rows[0]['lon'] == pytest.approx(-71.0 + 7.0e-5, abs=1e-6)   # map x = 2 + 5

        second = ObjectTracker()
        spin(second, 2.5, False, lambda: second.filter.tracks)
        assert not second.filter.tracks, 'priors adopted before the navsat datum'
        datum['set'] = True
        spin(second, 5.0, False, lambda: second.filter.tracks)
        (prior,) = second.filter.tracks
        assert prior.source == Track.SOURCE_PRIOR and prior.hits == 0
        assert prior.x[:2] == pytest.approx([5.0, 1.0], abs=0.05)          # back in odom

        class Failed:
            def result(self):
                raise RuntimeError('fromLL failed')
        lost = dict(rows[0], class_id='red_buoy')
        second.unadopted[id(lost)] = lost
        second.adopt_prior(lost, Failed())                  # warns, keeps the entry
        assert len(second.filter.tracks) == 1
        second.write_course_map()
        now = course_map.datetime.now(course_map.timezone.utc)
        assert sorted(r['class_id'] for r in course_map.load(file, now, 3)) == \
            ['green_buoy', 'red_buoy']
        del second.unadopted[id(lost)]

        # The buoy is gone: seen nowhere while in view, it drops and the map empties.
        spin(second, 5.0, 'nothing', lambda: not second.filter.tracks)
        assert not second.filter.tracks
        second.write_course_map()
        assert course_map.load(file, now, 3) == []
        second.destroy_node()
    finally:
        rclpy.try_shutdown()
