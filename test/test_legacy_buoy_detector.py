"""Check real YOLO output geometry and registered-depth projection on JP5."""

import numpy as np
import pytest

from mhseals_nav.legacy_buoy_detector import (
    decode_boxes, prepare_image, project_box,
)


def test_letterbox_preserves_geometry_and_converts_bgr_to_rgb():
    image = np.zeros((360, 640, 3), np.uint8)
    image[:, :, 2] = 255
    tensor, geometry = prepare_image(image, (640, 640))
    assert tensor.shape == (1, 3, 640, 640)
    assert tensor.dtype == np.float32
    assert geometry == (1.0, 0, 140, 360, 640)
    assert tensor[0, :, 200, 200] == pytest.approx([1, 0, 0])
    assert tensor[0, :, 0, 0] == pytest.approx([114 / 255] * 3)


def test_yolo11_boxes_keep_distinct_classes_and_suppress_duplicates():
    output = np.zeros((1, 17, 3), np.float32)
    output[0, :4, :] = np.array([
        [250, 250, 250], [340, 340, 340],
        [100, 100, 100], [200, 200, 200],
    ])
    output[0, 4 + 7, :2] = [.9, .8]
    output[0, 4 + 10, 2] = .7
    boxes = decode_boxes(output, (1., 0, 140, 360, 640), [.2] * 13)
    assert {item[0] for item in boxes} == {7, 10}
    assert len(boxes) == 2
    assert boxes[0][1] == pytest.approx(.9)
    assert boxes[0][2] == pytest.approx([200, 100, 100, 200])


def test_depth_projection_uses_optical_frame_metres_and_rejects_no_depth():
    depth = np.ones((360, 640), np.float32)
    k = [260., 0., 320., 0., 260., 180., 0., 0., 1.]
    position, dimensions = project_box((200, 100, 100, 200), depth, k)
    assert position == pytest.approx([-70 / 260, 20 / 260, 1.])
    assert dimensions == pytest.approx([100 / 260, 200 / 260, .05])
    depth[:] = np.nan
    assert project_box((200, 100, 100, 200), depth, k) is None


def test_box_depth_uses_surface_spread_without_background_or_invalid_samples():
    depth = np.full((100, 100), np.nan, np.float32)
    region = depth[42:59, 42:59]
    region[:] = np.linspace(1.1, 1.3, region.size).reshape(region.shape)
    region.flat[:10] = [np.nan, np.inf, -1, 0, 25, 8, 8, 8, 8, 8]
    k = [100., 0., 50., 0., 100., 50., 0., 0., 1.]
    position, dimensions = project_box((30, 30, 40, 40), depth, k)
    assert position[:2] == pytest.approx([0, 0])
    assert 1.19 < position[2] < 1.21
    assert .15 < dimensions[2] < .17


def test_box_depth_does_not_expand_to_the_background():
    depth = np.full((100, 100), 1., np.float32)
    depth[42:59, 42:59].flat[::4] = 8.
    k = [100., 0., 50., 0., 100., 50., 0., 0., 1.]
    position, dimensions = project_box((30, 30, 40, 40), depth, k)
    assert position == pytest.approx([0, 0, 1])
    assert dimensions == pytest.approx([.4, .4, .05])


def test_box_depth_is_bounded_when_nearby_surfaces_share_the_crop():
    depth = np.ones((100, 100), np.float32)
    depth[42:59, 42:59] = np.linspace(.65, 1.35, 289).reshape(17, 17)
    k = [100., 0., 50., 0., 100., 50., 0., 0., 1.]
    _, dimensions = project_box((30, 30, 40, 40), depth, k)
    assert dimensions == pytest.approx([.4, .4, .4])


def test_invalid_model_output_is_rejected_instead_of_mislabelled():
    with pytest.raises(ValueError):
        decode_boxes(np.zeros((1, 84, 8400)),
                     (1., 0, 140, 360, 640), [.2] * 13)
