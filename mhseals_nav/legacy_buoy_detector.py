"""SDK 4 custom inference: TensorRT YOLO11 -> synchronized SDK 3D boxes.

The SDK 4 wrapper cannot run CUSTOM_YOLOLIKE_BOX_OBJECTS. Keep its camera and
calibration; feed 2D detections back to its CUSTOM_BOX_OBJECTS input.
"""

import ctypes
import hashlib
from pathlib import Path
import subprocess

import cv2
import numpy as np
import yaml


def prepare_image(image: np.ndarray, shape: tuple) -> tuple:
    """Letterbox BGR pixels to the engine's NCHW RGB input."""
    height, width = image.shape[:2]
    target_h, target_w = shape
    ratio = min(target_w / width, target_h / height)
    new_w, new_h = round(width * ratio), round(height * ratio)
    left, top = (target_w - new_w) // 2, (target_h - new_h) // 2
    canvas = np.full((target_h, target_w, 3), 114, np.uint8)
    canvas[top:top + new_h, left:left + new_w] = cv2.resize(
        cv2.cvtColor(image, cv2.COLOR_BGR2RGB), (new_w, new_h))
    tensor = np.ascontiguousarray(canvas.transpose(2, 0, 1)[None],
                                 dtype=np.float32) / 255.0
    return tensor, (ratio, left, top, height, width)


def decode_boxes(output: np.ndarray, geometry: tuple, thresholds: list) -> list:
    """Decode YOLO11 without objectness, preserving classes through NMS."""
    if (output.ndim != 3 or output.shape[0] != 1
            or output.shape[1] != 4 + len(thresholds)):
        raise ValueError('Expected YOLO11 [1, 4 + class_count, anchors] output')
    prediction = output[0].T
    classes = prediction[:, 4:].argmax(axis=1)
    scores = prediction[:, 4:].max(axis=1)
    keep = (np.isfinite(prediction).all(axis=1)
            & (scores >= np.asarray(thresholds)[classes]))
    prediction, classes, scores = prediction[keep], classes[keep], scores[keep]
    ratio, left, top, height, width = geometry
    xy1 = (prediction[:, :2] - prediction[:, 2:4] / 2 - [left, top]) / ratio
    xy2 = (prediction[:, :2] + prediction[:, 2:4] / 2 - [left, top]) / ratio
    xy1 = np.clip(xy1, [0, 0], [width, height])
    xy2 = np.clip(xy2, [0, 0], [width, height])
    boxes = np.concatenate((xy1, xy2 - xy1), axis=1)
    return suppress_boxes([
        (int(category), float(score), box.tolist())
        for category, score, box in zip(classes, scores, boxes)
        if (box[2:] > 1).all()])


def suppress_boxes(boxes: list) -> list:
    """Suppress same-class duplicates, including overlapping inference crops."""
    found = []
    for category in sorted({item[0] for item in boxes}):
        candidates = [item for item in boxes if item[0] == category]
        selected = cv2.dnn.NMSBoxes([item[2] for item in candidates],
                                   [item[1] for item in candidates], 0.0, 0.45)
        found.extend(candidates[int(i)]
                     for i in np.asarray(selected, dtype=int).reshape(-1))
    return sorted(found, key=lambda item: -item[1])[:256]


def tile_origins(height: int, width: int, tile_size: int, tile_count: int,
                 tile_center: float) -> list:
    """Top-left (x, y) of the detail crops: one row centred on the water band."""
    y = min(max(round(tile_center * height - tile_size / 2), 0), height - tile_size)
    return [(int(x), y) for x in np.linspace(0, width - tile_size, tile_count, dtype=int)]


def detect_image(model, pixels: np.ndarray, thresholds: list, tile_size: int = 0,
                 tile_count: int = 3, tile_center: float = .5) -> list:
    """Keep the full view and detail crops around the water band, which sits
    at tile_center of the image height (the horizon follows camera pitch)."""
    height, width = pixels.shape[:2]
    views = [(0, 0, pixels)]
    if tile_size > 0 and min(height, width) > tile_size:
        for x, y in tile_origins(height, width, tile_size, tile_count, tile_center):
            views.append((x, y, pixels[y:y + tile_size, x:x + tile_size]))
    boxes = []
    for x, y, view in views:
        tensor, geometry = prepare_image(view, model.shape)
        for category, score, (bx, by, bw, bh) in decode_boxes(
                model.infer(tensor), geometry, thresholds):
            boxes.append((category, score, [x + bx, y + by, bw, bh]))
    return suppress_boxes(boxes)


def box_corners(box: list):
    """SDK pixel corners: top left, top right, bottom right, bottom left."""
    x, y, width, height = box
    return [[int(u), int(v)] for u, v in (
        (x, y), (x + width, y), (x + width, y + height), (x, y + height))]


class TensorRTModel:
    """Own GPU buffers and cache a plan for this model, GPU and TRT version."""

    def __init__(self, model_file: str, cache_dir: str, logger):
        import tensorrt as trt

        self.cuda = ctypes.CDLL('libcudart.so')
        self.cuda.cudaSetDevice.argtypes = [ctypes.c_int]
        self.cuda.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p),
                                        ctypes.c_size_t]
        self.cuda.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                                        ctypes.c_size_t, ctypes.c_int]
        self.cuda.cudaFree.argtypes = [ctypes.c_void_p]
        self.check(self.cuda.cudaSetDevice(0))
        driver = ctypes.CDLL('libcuda.so.1')
        name = ctypes.create_string_buffer(128)
        self.check(driver.cuDeviceGetName(name, len(name), 0))
        key = hashlib.sha256(Path(model_file).read_bytes()
                             + trt.__version__.encode() + name.value).hexdigest()[:20]
        cache = Path(cache_dir)
        cache.mkdir(parents=True, exist_ok=True)
        plan = cache / (key + '.engine')
        if not plan.exists():
            log = cache / (key + '.log')
            logger.info('Building the buoy TensorRT plan once; this can take '
                        'tens of minutes on JP5. Log: ' + str(log))
            temporary = plan.with_suffix('.tmp')
            with log.open('w') as stream:
                subprocess.run([
                    '/usr/src/tensorrt/bin/trtexec', '--onnx=' + model_file,
                    '--saveEngine=' + str(temporary), '--fp16',
                    '--memPoolSize=workspace:256', '--avgTiming=1', '--buildOnly',
                ], stdout=stream, stderr=subprocess.STDOUT, check=True,
                    timeout=3600)
            temporary.replace(plan)
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(plan.read_bytes())
        if self.engine is None:
            raise RuntimeError('Invalid TensorRT cache; remove ' + str(plan))
        self.context = self.engine.create_execution_context()
        if self.engine.num_bindings != 2:
            raise ValueError('Expected one image input and one YOLO output')
        self.buffers, self.pointers = [], []
        for i in range(2):
            if self.engine.get_binding_dtype(i) != trt.float32:
                raise ValueError('Expected float32 engine I/O')
            buffer = np.empty(self.engine.get_binding_shape(i), np.float32)
            pointer = ctypes.c_void_p()
            self.check(self.cuda.cudaMalloc(ctypes.byref(pointer), buffer.nbytes))
            self.buffers.append(buffer)
            self.pointers.append(pointer)
        self.input = next(i for i in range(2) if self.engine.binding_is_input(i))
        self.output = 1 - self.input
        if self.buffers[self.input].shape[:2] != (1, 3):
            raise ValueError('Expected a static batch-one RGB model')
        self.shape = self.buffers[self.input].shape[2:]

    @staticmethod
    def check(result: int) -> None:
        """Fail immediately on a CUDA API error."""
        if result != 0:
            raise RuntimeError('CUDA error ' + str(result))

    def infer(self, tensor: np.ndarray) -> np.ndarray:
        """Synchronously copy, infer and retrieve one bounded image batch."""
        source, target = self.buffers[self.input], self.buffers[self.output]
        source[:] = tensor
        self.check(self.cuda.cudaMemcpy(self.pointers[self.input],
                                       source.ctypes.data, source.nbytes, 1))
        if not self.context.execute_v2([p.value for p in self.pointers]):
            raise RuntimeError('TensorRT execution failed')
        self.check(self.cuda.cudaMemcpy(target.ctypes.data,
                                       self.pointers[self.output], target.nbytes, 2))
        return target

    def close(self) -> None:
        """Release the GPU allocations when the ROS node stops."""
        for pointer in self.pointers:
            self.cuda.cudaFree(pointer)


class LegacyBuoyDetector:
    """Keep ROS I/O out of geometry and inference so both can be verified."""

    def __init__(self):
        import rclpy
        from ament_index_python.packages import get_package_share_directory
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image, CompressedImage
        from zed_interfaces.msg import Object, ObjectsStamped

        self.node = rclpy.create_node('buoy_detector')
        share = Path(get_package_share_directory('mhseals_nav')) / 'config'
        defaults = dict(model_file=str(share / 'epoch60.onnx'),
                        class_config=str(share / 'custom_object_detection.yaml'),
                        engine_cache='/usr/local/zed/resources/astro_buoys',
                        camera_topic='/front/zed_node', tile_size=0,
                        tile_count=3, tile_center=.5)
        for key, value in defaults.items():
            self.node.declare_parameter(key, value)
        values = {key: self.node.get_parameter(key).value for key in defaults}
        with open(values['class_config']) as stream:
            config = yaml.safe_load(stream)['object_detection']
        classes = [config['class_%03d' % i]
                   for i in range(config['custom_class_count'])]
        self.labels = [item['label'] for item in classes]
        self.thresholds = [item['confidence_threshold'] / 100
                           if item['enabled'] else 1.1 for item in classes]
        self.model = TensorRTModel(values['model_file'], values['engine_cache'],
                                   self.node.get_logger())
        cv2.setNumThreads(1)
        self.model.infer(np.zeros((1, 3, *self.model.shape), np.float32))
        self.tile_size = int(values['tile_size'])
        self.tile_count, self.tile_center = int(values['tile_count']), values['tile_center']
        self.object_type, self.array_type = Object, ObjectsStamped
        self.jpeg_type = CompressedImage
        prefix = values['camera_topic'].rstrip('/')
        self.objects = self.node.create_publisher(
            ObjectsStamped, prefix + '/obj_det/custom_boxes', 1)
        self.debug = self.node.create_publisher(
            CompressedImage, '~/image/compressed', 1)
        self.subscription = self.node.create_subscription(
            Image, prefix + '/rgb/image_rect_color', self.image_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

    def image_callback(self, image) -> None:
        """Reply with 2D boxes for this exact image; the wrapper owns all 3D."""
        age = self.node.get_clock().now().nanoseconds * 1e-9 - (
            image.header.stamp.sec + image.header.stamp.nanosec * 1e-9)
        if not -.1 <= age <= .5:
            self.node.get_logger().warning(
                'Dropping stale image (age %.3f s); check host clocks '
                'and camera latency' % age, throttle_duration_sec=5.0)
            return
        channels = {'bgra8': 4, 'rgba8': 4, 'bgr8': 3, 'rgb8': 3}[image.encoding]
        pixels = np.ndarray((image.height, image.width, channels), np.uint8,
                            buffer=image.data, strides=(image.step, channels, 1))
        if image.encoding in ('rgb8', 'rgba8'):
            pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR
                                  if channels == 3 else cv2.COLOR_RGBA2BGR)
        elif channels == 4:
            pixels = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
        boxes = detect_image(self.model, pixels, self.thresholds, self.tile_size,
                             self.tile_count, self.tile_center)
        array = self.array_type()
        array.header = image.header
        annotated = pixels.copy() if self.debug.get_subscription_count() else None
        for category, score, box in boxes:
            obj = self.object_type()
            obj.label, obj.label_id = self.labels[category], category
            obj.confidence = score * 100
            for corner, point in zip(obj.bounding_box_2d.corners, box_corners(box)):
                corner.kp = point
            array.objects.append(obj)
            if annotated is not None:
                x, y, w, h = map(int, box)
                cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(annotated, '%s %.2f' % (obj.label, score),
                            (x, max(15, y - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                            .45, (0, 255, 0), 1)
        self.objects.publish(array)
        if annotated is not None:
            message = self.jpeg_type()
            message.header, message.format = image.header, 'jpeg'
            if annotated.shape[1] > 1280:
                annotated = cv2.resize(annotated, (
                    1280, round(annotated.shape[0] * 1280 / annotated.shape[1])),
                    interpolation=cv2.INTER_AREA)
            message.data = cv2.imencode('.jpg', annotated, [
                cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
            self.debug.publish(message)


def main(args=None) -> None:
    """Run the SDK 4 adapter in the camera's ROS domain."""
    import rclpy

    rclpy.init(args=args)
    detector = None
    try:
        detector = LegacyBuoyDetector()
        rclpy.spin(detector.node)
    except KeyboardInterrupt:
        pass
    finally:
        if detector is not None:
            detector.model.close()
            detector.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
