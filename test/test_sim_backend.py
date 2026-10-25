"""sim_backend pure parts: injection geometry, schedules, truth ids, phantoms, ZED packet."""
import math
from importlib.util import find_spec
from pathlib import Path

from mhseals_nav import sim_backend as sb
import numpy as np
import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'sim_backend.yaml'
UNITY_AXES = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # x right, y fwd


def cfg():
    return yaml.safe_load(CONFIG.read_text())['/**']['ros__parameters']


def obj(p, v=(0.0, 0.0, 0.0), label='red_buoy', truth_id=None):
    return {'label': label, 'score': 0.8, 'p': np.array(p, float),
            'size': np.array([.4, .4, .6]), 'v': None if v is None else np.array(v, float),
            'id': truth_id}


def test_nest_rebuilds_the_yaml_shape_from_parameter_names():
    assert sb.nest({'a.b.c': 1, 'a.d': 2, 'e': 3}) == {'a': {'b': {'c': 1}, 'd': 2}, 'e': 3}


def test_injection_uses_live_params_without_a_case_and_rows_with_one():
    c = cfg()
    c['inject']['heading_bias_deg'] = 2.0
    assert sb.injection(c, 'none', 1e6)['heading_bias_deg'] == 2.0
    rows = [sb.injection(c, 'metamorphic', t) for t in (0, 59.9, 60, 150, 200, 299)]
    assert [r['heading_bias_deg'] for r in rows] == [0, 0, 5, 0, 0, 0]
    assert [r['heading_lag_s'] for r in rows] == [0, 0, 0, 0.1, 0, 0.3]
    assert rows[4]['extrinsic_yaw_deg'] == 3 and rows[4]['stamp_offset_s'] == 0.1


def test_metamorphic_case_matches_the_spec_schedule():
    rows = cfg()['cases']['metamorphic']
    assert rows['at_s'] == [0, 60, 120, 180, 240]
    assert len({len(v) for v in rows.values()}) == 1


def test_heading_error_is_bias_minus_lag_times_yaw_rate():
    inj = {'heading_bias_deg': 5.0, 'heading_lag_s': 0.2}
    assert sb.heading_error(inj, 0.5) == pytest.approx(math.radians(5) - 0.1)


def test_heading_error_turns_points_about_base_link_and_velocities_with_them():
    t_bc = np.array([1.0, 0.0, 0.0])
    [out] = sb.inject([obj([1.0, 0.0, 0.0], v=[1.0, 0.0, 0.0])], np.eye(3), t_bc,
                      math.pi / 2, 0.0)
    assert out['p'] == pytest.approx([-1.0, 2.0, 0.0])
    assert out['v'] == pytest.approx([0.0, 1.0, 0.0])


def test_extrinsic_error_turns_points_about_the_camera():
    [out] = sb.inject([obj([1.0, 0.0, 0.0], v=None)], np.eye(3), np.array([1.0, 0, 0]),
                      0.0, math.pi / 2)
    assert out['p'] == pytest.approx([0.0, 1.0, 0.0]) and out['v'] is None


@pytest.mark.parametrize('heading, extrinsic', [(0.1, 0.0), (0.0, -0.05), (0.07, 0.03)])
def test_errors_are_yaw_rotations_in_the_world_whatever_the_camera_axes(heading, extrinsic):
    """Lift injected points as turns about base_link (heading) or the camera (extrinsic).

    Holds with Unity's camera axes (x right, y forward) as with any other.
    """
    t_bc = np.array([0.6, 0.1, 0.4])
    p = np.array([0.5, 8.0, -0.3])  # 8 m ahead
    [out] = sb.inject([obj(p)], UNITY_AXES, t_bc, heading, extrinsic)
    base = UNITY_AXES @ p + t_bc
    expected = sb.rz(heading) @ (sb.rz(extrinsic) @ (base - t_bc) + t_bc)
    assert UNITY_AXES @ out['p'] + t_bc == pytest.approx(expected)


def test_camera_and_world_lifts_round_trip():
    pose = (sb.rz(0.7), np.array([3.0, -2.0, 0.1]))
    p = np.array([1.0, 5.0, 0.2])
    p_w = sb.to_world(p, pose, UNITY_AXES, np.array([.5, 0, .3]))
    assert sb.to_camera(p_w, pose, UNITY_AXES, np.array([.5, 0, .3])) == pytest.approx(p)


def test_fov_follows_base_forward_and_range():
    fov = {'hfov_deg': 110.0, 'range_m': 20.0}
    assert sb.in_fov(np.array([0.0, 10.0, 0.0]), UNITY_AXES, fov)
    assert not sb.in_fov(np.array([0.0, -10.0, 0.0]), UNITY_AXES, fov)   # behind
    assert not sb.in_fov(np.array([10.0, 5.0, 0.0]), UNITY_AXES, fov)    # 63 deg off axis
    assert not sb.in_fov(np.array([0.0, 21.0, 0.0]), UNITY_AXES, fov)


def test_truth_match_takes_the_nearest_same_label_within_reach():
    elements = [{'label': 'red_buoy', 'x': 0.0, 'y': 0.0},
                {'label': 'green_buoy', 'x': 0.1, 'y': 0.0},
                {'label': 'red_buoy', 'x': 0.5, 'y': 0.0}]
    assert sb.match_truth('red_buoy', (0.4, 0.0), elements, 1.0) == 2
    assert sb.match_truth('green_buoy', (0.4, 0.0), elements, 1.0) == 1
    assert sb.match_truth('red_buoy', (3.0, 0.0), elements, 1.0) is None


def test_phantoms_circle_at_their_speed():
    ph = cfg()['phantoms']
    for t in (0.0, 3.3, 17.0):
        [(label, p, v, size)] = sb.phantoms(ph, t)
        assert np.linalg.norm(v) == pytest.approx(0.8)
        assert np.linalg.norm(p[:2] - ph['boat']['centre']) == pytest.approx(3.0)
        assert float(np.dot(v, p - [*ph['boat']['centre'], 0])) == pytest.approx(0, abs=1e-9)


def test_scene_adds_truth_ids_and_only_phantoms_in_view():
    c = cfg()
    centre = np.array([*c['phantoms']['boat']['centre'], 0.0])
    elements = [{'label': 'red_buoy', 'x': 5.0, 'y': 0.0}]
    looking_at = (np.eye(3), centre - [8.0, 0.0, 0.0])            # phantom ahead
    looking_away = (sb.rz(math.pi), centre - [8.0, 0.0, 0.0])
    det = [('red_buoy', 0.9, np.array([5.0, 8.0, 0.0]), np.ones(3))]
    pose = (np.eye(3), np.array([0.0, 0.0, 0.0]))
    buoy, *_ = sb.scene([('red_buoy', 0.9, np.array([0.0, 5.0, 0.0]), np.ones(3))], 0.0,
                        pose, UNITY_AXES, np.zeros(3), c, elements)
    assert buoy['id'] == 0 and buoy['v'] == pytest.approx([0, 0, 0])
    seen = sb.scene(det, 1.0, looking_at, UNITY_AXES, np.zeros(3), c, [])
    assert [o['id'] for o in seen] == [None, sb.PHANTOM_ID]
    assert np.linalg.norm(seen[1]['v']) == pytest.approx(0.8)
    assert len(sb.scene(det, 1.0, looking_away, UNITY_AXES, np.zeros(3), c, [])) == 1
    assert len(sb.scene(det, 1.0, None, UNITY_AXES, np.zeros(3), c, elements)) == 1


def test_zed_packet_carries_ids_tracking_ok_and_a_box_around_the_centre():
    K = sb.pinhole((1280, 720), 110.0)
    packet = sb.zed_packet(1_500_000_000, 'front_camera_link',
                           [obj([0.5, 6.0, 0.0], truth_id=3), obj([0.0, 6.0, 0.0])], UNITY_AXES, K,
                           (1280, 720))
    assert packet['header'] == {'stamp': {'sec': 1, 'nanosec': 500_000_000},
                                'frame_id': 'front_camera_link'}
    first, second = packet['objects']
    assert (first['label_id'], second['label_id']) == (3, -1)
    assert first['tracking_state'] == 1 and first['confidence'] == pytest.approx(80)
    (u0, v0), _, (u1, v1), _ = [c['kp'] for c in first['bounding_box_2d']['corners']]
    u, v = sb.project(np.array(first['position']), UNITY_AXES, K)
    assert u0 < u < u1 and v0 < v < v1 and u > 640  # right of the optical axis


@pytest.mark.skipif(find_spec('rclpy') is None or find_spec('mhseals_nav.zed_backend') is None,
                    reason='zed mode needs ROS and zed_backend (nav-camera)')
def test_zed_mode_runs_the_boat_conversion():
    from mhseals_nav.msg import Observation
    objects = [obj([0.5, 6.0, 0.1], v=[0.2, 0.0, 0.0], truth_id=3), obj([0.0, 9.0, 0.0])]
    K = sb.pinhole((1280, 720), 110.0)
    zed = sb.observations(objects, 2_000_000_000, 'cam', 'zed', UNITY_AXES, K, (1280, 720))
    plain = sb.observations(objects, 2_000_000_000, 'cam', 'plain', UNITY_AXES, K, (1280, 720))
    assert zed.backend == plain.backend == 'sim'
    assert [o.backend_id for o in zed.observations] == ['3', '']
    assert zed.observations[0].velocity_kind == Observation.VELOCITY_WORLD
    assert zed.observations[0].velocity.x == pytest.approx(0.2)
    assert all(o.has_box2d for o in zed.observations)
    assert not any(o.has_box2d or o.backend_id or o.velocity_kind for o in plain.observations)
    for z, p in zip(zed.observations, plain.observations):
        assert (z.position.x, z.position.y) == pytest.approx((p.position.x, p.position.y))
