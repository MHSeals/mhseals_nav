"""Check YOLO output geometry and SDK custom-box input on JP5."""

import numpy as np
import pytest

from mhseals_nav.legacy_buoy_detector import (
    box_corners, decode_boxes, prepare_image,
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


def test_sdk_pixel_corners_keep_clockwise_order_and_image_coordinates():
    assert box_corners([200., 100., 100., 200.]) == [
        [200, 100], [300, 100], [300, 300], [200, 300]]


def test_invalid_model_output_is_rejected_instead_of_mislabelled():
    with pytest.raises(ValueError):
        decode_boxes(np.zeros((1, 84, 8400)),
                     (1., 0, 140, 360, 640), [.2] * 13)
