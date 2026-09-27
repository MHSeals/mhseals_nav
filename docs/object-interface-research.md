# Object interface notes

Primary-source review, 2026-09-27. No boat/network hardware test was performed.

- **ZED fields:** `Object.position` is `float32[3]`, not a geometry
  `Point`. Confidence is on a 1–99 scale. Tracking states are OFF=0,
  OK=1, SEARCHING=2 (estimated after occlusion), TERMINATE=3. Do not
  refresh a measured track indefinitely from estimated/terminated observations.
  See [Object.msg](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/Object.msg)
  and [ObjectsStamped.msg](https://github.com/stereolabs/zed-ros2-interfaces/blob/master/msg/ObjectsStamped.msg).
- **Frame/time:** the wrapper publishes the left-camera frame and the frame
  timestamp (or publication time when explicitly configured). Preserve the
  message header; do not relabel coordinates `camera_link`. Transform at the
  observation timestamp through the calibrated camera-to-base and localization
  TF chain, not using the latest boat pose. See the
  [wrapper publisher](https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_components/src/zed_camera/src/zed_camera_component_objdet.cpp).
- **Cross-distribution contract:** Foxy's
  [ObjectHypothesisWithPose](https://github.com/ros-perception/vision_msgs/blob/foxy/vision_msgs/msg/ObjectHypothesisWithPose.msg)
  has flat `id`/`score`; current
  [ObjectHypothesisWithPose](https://github.com/ros-perception/vision_msgs/blob/ros2/vision_msgs/msg/ObjectHypothesisWithPose.msg)
  uses nested `hypothesis.class_id`/`score`. A matching ROS topic name is not
  sufficient for a typed relay. An explicit ZED JSON adapter using
  [rosbridge subscribe/publish](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)
  can reconstruct local Jazzy messages without exposing the legacy DDS domain.
  Restrict this unauthenticated interface to the trusted boat network. Native
  Jazzy/simulation producers should publish the same local detection contract
  directly; do not make them depend on the legacy transport.
- **Visualization is not collision avoidance:** `MarkerArray` does not enter
  a Nav2 costmap. The
  [ObstacleLayer](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/plugins/obstacle_layer.cpp)
  marks cells and separately raytraces clearing observations. Its
  [observation buffer](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/src/observation_buffer.cpp)
  expiration does not erase previously marked cells; zero persistence retains
  the latest observation. Sparse buoy centroids are not free-space measurements.
  Keep semantic tracking/markers separate unless an explicitly clearing
  object-costmap integration is implemented and tested.

Local review found an array/Point mismatch in the converter, latest-time TF,
unbounded retained tracks/candidates, and promoted candidates not being removed.
Required regression checks: moving/rotating boat with a fixed world buoy,
missing TF, empty detections/source outage, stale/out-of-order timestamps,
candidate promotion, class-aware association, expiration/marker deletion, and
native versus legacy-adapted equivalent inputs. Software tests cannot establish
camera extrinsics or real-world localization accuracy.

## Pinned JetPack 5 / ZED 4 limitations

The image pins wrapper `77a043a6d7fb5802ac49562e61efc1a4d6b5fc3f`.
Its [launch file](https://github.com/stereolabs/zed-ros2-wrapper/blob/77a043a6d7fb5802ac49562e61efc1a4d6b5fc3f/zed_wrapper/launch/zed_camera.launch.py)
defaults to `camera_name:=zed`, `node_name:=zed_node`. Thus the default object
topic is `/zed/zed_node/obj_det/objects`, frame `zed_left_camera_frame`;
`camera_name:=zed2` changes these to `/zed2/zed_node/obj_det/objects` and
`zed2_left_camera_frame`. Its message package is **zed_interfaces**, not zed_msgs.

With the camera already running and depth enabled, enable its built-in detector:

```bash
ros2 service call /zed/zed_node/enable_obj_det std_srvs/srv/SetBool '{data: true}'
```

Use the actual camera namespace. Startup YAML instead uses
`object_detection.od_enabled: true` and `object_detection.model`.
The [pinned implementation](https://github.com/stereolabs/zed-ros2-wrapper/blob/77a043a6d7fb5802ac49562e61efc1a4d6b5fc3f/zed_components/src/zed_camera/src/zed_camera_component.cpp)
only accepts built-in models preceding `CUSTOM_BOX_OBJECTS`; its launch file has
no `custom_onnx_file` or `custom_object_detection_config_path` arguments.
Consequently this repo's current `CUSTOM_YOLOLIKE_BOX_OBJECTS` buoy configuration
**cannot run through this pinned legacy wrapper**. Enabling built-in detection
does not enable buoy recognition. Custom buoy inference needs a compatible
separate detector producing the common detection interface, or a separately
validated wrapper/SDK upgrade; transport alone cannot provide missing detections.

The image installs rosbridge via apt, without a version pin. The
[Foxy release manifest](https://github.com/ros/rosdistro/blob/master/foxy/distribution.yaml)
lists 1.3.1; its
[subscriber implementation](https://github.com/RobotWebTools/rosbridge_suite/blob/1.3.1/rosbridge_library/src/rosbridge_library/internal/subscribers.py)
chooses TRANSIENT_LOCAL only if such a publisher is discovered when subscribing.
Start the camera/robot-state publisher first, wait until `/tf_static` is visible,
then connect the object bridge. If connected too early, restart/reconnect it
after publisher discovery. This allows retained static transforms to be received;
the bridge must cache/rebroadcast the complete static camera subtree for local
late subscribers. Runtime apt version and actual TF receipt still need checking
on the boat.
