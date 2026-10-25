"""ZED objects dict -> ObservationArray, and the node's gates and status."""

import math
import time
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
import rclpy

from mhseals_nav.msg import Observation
from mhseals_nav.zed_backend import ZedBackend, convert_objects, covariance
from ros_domain import require_isolated_domain


def zed_object(**overrides):
    obj = dict(label='red_buoy', label_id=7, confidence=85.0, position=[4.0, 1.0, -.2],
               position_covariance=[.04, .01, 0.0, .09, 0.0, .01], velocity=[.5, -.1, 0.0],
               tracking_available=True, tracking_state=1,
               bounding_box_2d=dict(corners=[dict(kp=p) for p in
                                             ([320, 180], [480, 180], [480, 360], [320, 360])]),
               bounding_box_3d=dict(corners=[]), dimensions_3d=[.4, .9, .4])
    obj.update(overrides)
    return obj


def packet(*objects, sec=100):
    return dict(header=dict(frame_id='front_left_camera_frame', stamp=dict(sec=sec, nanosec=5)),
                objects=list(objects))


def test_tracking_on_maps_every_field():
    array = convert_objects(packet(zed_object()), (1280, 720), 'world')
    assert array.header.frame_id == 'front_left_camera_frame'
    assert (array.header.stamp.sec, array.header.stamp.nanosec) == (100, 5)
    (obs,) = array.observations
    assert obs.class_id == 'red_buoy' and obs.backend_id == '7'
    assert obs.score == pytest.approx(.85)
    assert (obs.position.x, obs.position.y, obs.position.z) == pytest.approx((4.0, 1.0, -.2))
    assert (obs.size.x, obs.size.y, obs.size.z) == pytest.approx((.4, .9, .4))
    assert obs.velocity_kind == Observation.VELOCITY_WORLD
    assert (obs.velocity.x, obs.velocity.y) == pytest.approx((.5, -.1))
    assert obs.has_box2d and list(obs.box2d) == pytest.approx([.25, .25, .375, .5])


def test_tracking_off_gives_no_ids_and_keeps_every_detection():
    array = convert_objects(packet(zed_object(tracking_available=False, tracking_state=0,
                                              label_id=-1)), (1280, 720), 'none')
    (obs,) = array.observations
    assert obs.backend_id == ''
    assert obs.velocity_kind == Observation.VELOCITY_NONE
    assert (obs.velocity.x, obs.velocity.y, obs.velocity.z) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('state', [0, 2, 3])
def test_predicted_or_lost_tracks_are_skipped(state):
    assert not convert_objects(packet(zed_object(tracking_state=state)), None, 'none').observations


def test_negative_label_id_is_no_id():
    (obs,) = convert_objects(packet(zed_object(label_id=-1)), None, 'none').observations
    assert obs.backend_id == ''


def test_covariance_expands_upper_triangle_symmetrically():
    assert covariance([1, 2, 3, 4, 5, 6]) == [1, 2, 3, 2, 4, 5, 3, 5, 6]
    (obs,) = convert_objects(packet(zed_object()), None, 'none').observations
    assert list(obs.position_covariance) == pytest.approx(
        [.04, .01, 0, .01, .09, 0, 0, 0, .01])
    (none,) = convert_objects(packet(zed_object(position_covariance=[0] * 6)), None,
                              'none').observations
    assert not any(none.position_covariance)


def test_box2d_needs_an_image_size_and_is_clamped():
    (obs,) = convert_objects(packet(zed_object()), None, 'none').observations
    assert not obs.has_box2d
    (obs,) = convert_objects(packet(zed_object()), (400, 300), 'none').observations
    assert list(obs.box2d) == pytest.approx([.8, .6, 1.0, 1.0])


def test_3d_corners_set_centre_and_size_in_the_header_frame():
    corners = [dict(kp=[x, y, z]) for x in (3.8, 4.2) for y in (.5, 1.5) for z in (-.5, .1)]
    (obs,) = convert_objects(packet(zed_object(bounding_box_3d=dict(corners=corners))),
                             None, 'none').observations
    assert (obs.position.x, obs.position.y, obs.position.z) == pytest.approx((4.0, 1.0, -.2))
    assert (obs.size.x, obs.size.y, obs.size.z) == pytest.approx((.4, 1.0, .6))


def test_non_finite_object_is_skipped_and_the_frame_kept():
    array = convert_objects(packet(zed_object(position=[math.nan, 0, 0]), zed_object()),
                            None, 'none')
    assert len(array.observations) == 1


def test_empty_frame_is_still_an_array():
    array = convert_objects(packet(), None, 'none')
    assert array.observations == [] and array.header.frame_id


@pytest.mark.parametrize('bad', [dict(frame_id=''), dict(stamp=dict(sec=1, nanosec=2 ** 31))])
def test_malformed_header_raises(bad):
    raw = packet()
    raw['header'].update(bad)
    with pytest.raises(ValueError):
        convert_objects(raw, None, 'none')


def test_unknown_velocity_kind_raises():
    with pytest.raises(KeyError):
        convert_objects(packet(), None, 'camera')


@pytest.fixture
def backend():
    require_isolated_domain()
    rclpy.init()  # no overrides: every value below comes from config/zed_backend.yaml
    node = ZedBackend()
    node.pub = SimpleNamespace(sent=[], publish=lambda m: node.pub.sent.append(m))
    node.status_pub = SimpleNamespace(sent=[], publish=lambda m: node.status_pub.sent.append(m))
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_node_publishes_fresh_frames_and_drops_stale_and_reordered(backend):
    backend.camera_info(SimpleNamespace(width=640, height=360))
    assert backend.camera_size == (1280, 720)
    now = int(time.time())
    backend.publish_packet(packet(zed_object(), sec=now - 30))  # stale
    backend.publish_packet(packet(zed_object(), sec=now))
    backend.publish_packet(packet(zed_object(), sec=now))      # same stamp: reorder
    backend.publish_packet(dict(header=1))                      # malformed
    (sent,) = backend.pub.sent
    assert sent.backend == 'zed_native' and 0 <= sent.processing_latency < 1.0
    assert sent.observations[0].has_box2d
    backend.ids = True
    backend.report()
    (stage,) = backend.status_pub.sent[0].status
    values = {v.key: v.value for v in stage.values}
    assert stage.name == 'perception/zed_native/backend'
    assert values['drop_reorder'] == '1' and values['drop_stale'] == '1'
    assert values['drop_malformed'] == '1' and values['objects'] == '1.0'
    assert values['frame'] == 'front_left_camera_frame' and values['ids'] == 'true'
    assert values['velocity'] == 'none' and values['hfov_deg'] == '110'
    assert set(values) >= {'hz', 'latency_ms', 'latency_ms_p95', 'proc_ms', 'range_m'}


def test_node_holds_no_default_values():
    import ast
    from pathlib import Path
    """Defaults live in config/zed_backend.yaml only (the backend fixture passes none)."""
    text = (Path(__file__).parents[1] / 'mhseals_nav/zed_backend.py').read_text()
    declared = [c for c in ast.walk(ast.parse(text)) if isinstance(c, ast.Call)
                and getattr(c.func, 'attr', '') == 'declare_parameter']
    assert all(not isinstance(arg, ast.Constant) for call in declared for arg in call.args[1:])
    assert '/front/zed_node' not in text and 'zed_native' not in text


def test_box2d_scale_matches_the_wrapper_publish_downscale():
    import yaml
    from pathlib import Path
    config = Path(__file__).parents[1] / 'config'
    ours = yaml.safe_load((config / 'zed_backend.yaml').read_text())['/**']['ros__parameters']
    wrapper = yaml.safe_load((config / 'zed_boat.yaml').read_text())['/**']['ros__parameters']
    general = wrapper['general']
    scale = general['pub_downscale_factor'] if general['pub_resolution'] == 'CUSTOM' else 1.0
    assert ours['box2d_scale'] == scale
    assert ours['fov']['range_m'] == wrapper['object_detection']['max_range']
