"""Check YOLO output geometry and SDK custom-box input on JP5."""

import numpy as np
import pytest

from mhseals_nav.legacy_buoy_detector import (
    box_corners, decode_boxes, detect_image, prepare_image, suppress_boxes,
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


def test_detail_crops_restore_original_coordinates_and_merge_repeated_objects():
    class Model:
        shape = (640, 640)
        calls = 0

        def infer(self, tensor):
            self.calls += 1
            result = np.zeros((1, 17, 1), np.float32)
            # Same green buoy in the two overlapping middle/right crops.
            if self.calls in (3, 4):
                x = 560 if self.calls == 3 else 80 / 3
                result[0, :4, 0] = [x, 320, 20, 60]
                result[0, 4 + 7, 0] = .8 if self.calls == 3 else .7
            return result

    model = Model()
    found = detect_image(model, np.zeros((1080, 1920, 3), np.uint8),
                         [.2] * 13, tile_size=720)
    assert model.calls == 4
    assert len(found) == 1
    assert found[0][0] == 7
    assert found[0][1] == pytest.approx(.8)
    assert found[0][2] == pytest.approx([1218.75, 506.25, 22.5, 67.5])


@pytest.mark.parametrize('shape,tile_size', [((360, 640), 720), ((1080, 1920), 0)])
def test_single_pass_for_small_images_or_disabled_crops(shape, tile_size):
    class Model:
        shape = (640, 640)
        calls = 0

        def infer(self, tensor):
            self.calls += 1
            return np.zeros((1, 17, 1), np.float32)

    model = Model()
    assert detect_image(model, np.zeros((*shape, 3), np.uint8),
                        [.2] * 13, tile_size) == []
    assert model.calls == 1


def test_crop_merging_keeps_distinct_colours_at_the_same_position():
    box = [100., 200., 20., 60.]
    found = suppress_boxes([(7, .8, box), (7, .7, box), (10, .6, box)])
    assert [item[0] for item in found] == [7, 10]
