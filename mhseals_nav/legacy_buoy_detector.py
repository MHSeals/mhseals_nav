"""SDK 4 camera shim: TensorRT YOLO11 + registered depth -> ZED objects.

The SDK 4 wrapper cannot run CUSTOM_YOLOLIKE_BOX_OBJECTS. Keep its camera and
calibration, and publish the same small object interface consumed by the bridge.
"""

import ctypes
from collections import deque
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
    found = []
    for category in np.unique(classes):
        indices = np.flatnonzero((classes == category)
                                 & (boxes[:, 2:] > 1).all(axis=1))
        if not len(indices):
            continue
        selected = cv2.dnn.NMSBoxes(boxes[indices].tolist(),
                                   scores[indices].tolist(), 0.0, 0.45)
        for index in np.asarray(selected, dtype=int).reshape(-1):
            i = indices[index]
            found.append((int(category), float(scores[i]), boxes[i].tolist()))
    return sorted(found, key=lambda item: -item[1])[:256]


def project_box(box: list, depth: np.ndarray, k: list):
    """Use finite centre-region depth; return metres in the optical frame."""
    x, y, width, height = box
    u, v = x + width / 2, y + height / 2
    region = depth[
        max(0, int(v - height * .2)):min(depth.shape[0], int(v + height * .2) + 1),
        max(0, int(u - width * .2)):min(depth.shape[1], int(u + width * .2) + 1)]
    valid = region[np.isfinite(region) & (region > .3) & (region < 20.)]
    if valid.size < 3 or k[0] <= 0 or k[4] <= 0:
        return None
    z = float(np.median(valid))
    return [(u - k[2]) * z / k[0], (v - k[5]) * z / k[4], z]


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
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, CameraInfo, CompressedImage
        from zed_interfaces.msg import Object, ObjectsStamped

        self.node = rclpy.create_node('buoy_detector')
        share = Path(get_package_share_directory('mhseals_nav')) / 'config'
        defaults = dict(model_file=str(share / 'epoch60.onnx'),
                        class_config=str(share / 'custom_object_detection.yaml'),
                        engine_cache='/usr/local/zed/resources/astro_buoys',
                        camera_topic='/front/zed_node', rate=10.0)
        for key, value in defaults.items():
            self.node.declare_parameter(key, value)
        values = {key: self.node.get_parameter(key).value for key in defaults}
        if not 0 < values['rate'] <= 30:
            raise ValueError('rate must be between zero and 30 Hz')
        with open(values['class_config']) as stream:
            config = yaml.safe_load(stream)['object_detection']
        classes = [config['class_%03d' % i]
                   for i in range(config['custom_class_count'])]
        self.labels = [item['label'] for item in classes]
        self.thresholds = [item['confidence_threshold'] / 100
                           if item['enabled'] else 1.1 for item in classes]
        self.model = TensorRTModel(values['model_file'], values['engine_cache'],
                                   self.node.get_logger())
        self.object_type, self.array_type = Object, ObjectsStamped
        self.jpeg_type = CompressedImage
        self.images, self.depths = deque(maxlen=3), deque(maxlen=3)
        self.info = None
        prefix = values['camera_topic'].rstrip('/')
        self.objects = self.node.create_publisher(
            ObjectsStamped, prefix + '/obj_det/objects', 10)
        self.debug = self.node.create_publisher(
            CompressedImage, '~/image/compressed', 1)
        self.subscriptions = [
            self.node.create_subscription(Image, prefix + '/rgb/image_rect_color',
                                          self.images.append, qos_profile_sensor_data),
            self.node.create_subscription(Image, prefix + '/depth/depth_registered',
                                          self.depths.append, qos_profile_sensor_data),
            self.node.create_subscription(CameraInfo, prefix + '/rgb/camera_info',
                                          self.camera_info, qos_profile_sensor_data)]
        self.timer = self.node.create_timer(1 / values['rate'], self.tick)

    def camera_info(self, message) -> None:
        """Retain the camera's actual rectified intrinsics."""
        self.info = message

    def tick(self) -> None:
        """Process only a fresh image/depth pair with the same acquisition time."""
        pair = next(((image, depth) for image in reversed(self.images)
                     for depth in reversed(self.depths)
                     if image.header.stamp == depth.header.stamp), None)
        if pair is None or self.info is None:
            return
        image, depth = pair
        self.images.clear()
        self.depths.clear()
        age = self.node.get_clock().now().nanoseconds * 1e-9 - (
            image.header.stamp.sec + image.header.stamp.nanosec * 1e-9)
        if not -.1 <= age <= .5:
            self.node.get_logger().warning(
                'Dropping stale RGB-D pair (age %.3f s); check host clocks '
                'and camera latency' % age, throttle_duration_sec=5.0)
            return
        if (image.header.frame_id != depth.header.frame_id
                or image.header.frame_id != self.info.header.frame_id
                or (image.width, image.height) != (depth.width, depth.height)
                or (image.width, image.height) != (self.info.width, self.info.height)
                or depth.encoding != '32FC1'):
            raise ValueError('Expected co-registered colour/depth and intrinsics')
        channels = {'bgra8': 4, 'rgba8': 4, 'bgr8': 3, 'rgb8': 3}[image.encoding]
        pixels = np.ndarray((image.height, image.width, channels), np.uint8,
                            buffer=image.data, strides=(image.step, channels, 1))
        if image.encoding in ('rgb8', 'rgba8'):
            pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR
                                  if channels == 3 else cv2.COLOR_RGBA2BGR)
        elif channels == 4:
            pixels = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
        depths = np.ndarray((depth.height, depth.width),
                            '>f4' if depth.is_bigendian else '<f4',
                            buffer=depth.data, strides=(depth.step, 4))
        tensor, geometry = prepare_image(pixels, self.model.shape)
        boxes = decode_boxes(self.model.infer(tensor), geometry, self.thresholds)
        array = self.array_type()
        array.header = image.header
        annotated = pixels.copy() if self.debug.get_subscription_count() else None
        for category, score, box in boxes:
            position = project_box(box, depths, self.info.k)
            if position is None:
                continue
            obj = self.object_type()
            obj.label, obj.label_id = self.labels[category], category
            obj.confidence = score * 100
            obj.position = list(map(float, position))
            obj.dimensions_3d = [box[2] * position[2] / self.info.k[0],
                                 box[3] * position[2] / self.info.k[4], 0.0]
            # These are fresh measurements; the ODROID owns temporal tracking.
            obj.tracking_available = False
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
            message.data = cv2.imencode('.jpg', annotated)[1].tobytes()
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
