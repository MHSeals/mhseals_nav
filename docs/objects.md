# Objects and optional lidar

Run localization/Nav2 on Jazzy. Keep Foxy on domain 142 and Jazzy on 42;
do not bridge their DDS graphs. When all machines use Jazzy, use native ROS
topics and omit the legacy transport.

## Lidar

`navigation.launch.py`, `slam_nav.launch.py` and `robot.launch.py` accept
`use_lidar:=false`. This explicitly disables both lidar obstacle layers;
the default `true` requires fresh `/points` and stops commands when lidar
goes stale. There is **no automatic fallback to blind navigation**.
`sensors_ignore:=lidar` only skips the driver: use both options when no lidar
is available, but only `sensors_ignore` when another computer supplies it.
Neither markers nor camera detections replace range-based collision avoidance.

```sh
ros2 launch mhseals_nav slam_nav.launch.py sim:=false use_lidar:=false
```

This requires existing `map -> odom -> base_link` TF and `/odom/local`.
Nav2 emits velocity targets on `/nav/cmd_vel`, not normalized PWM effort.
See [actuator requirements](nav2-commissioning.md#launch-and-interface).

## One input-independent tracker

The input is Jazzy `vision_msgs/msg/Detection3DArray` on `/detections`:
metres, real acquisition timestamps, actual coordinate frame in the header,
positions in `bbox.center`, labels in `results[].hypothesis.class_id`.
CRANE can publish this directly; do not start a ZED converter for simulated
detections. Use exactly one producer and one tracker per boat.

```sh
# CRANE supplies /clock, detections and its TF/localization:
ros2 launch mhseals_nav objects.launch.py use_sim_time:=true
# Native Jazzy ZED, if sensors.launch.py is NOT already converting it:
ros2 launch mhseals_nav objects.launch.py source:=zed
```

If `sensors.launch.py` already provides `/detections`, omit `source:=zed`.
Run the native converter on a machine with the ZED message package (normally
the camera host); only standard `/detections` needs to reach the Jazzy nav host.
Alternatively `robot.launch.py enable_object_tracking:=true` starts the tracker;
do not also run `objects.launch.py`. `odom.launch.py` now does localization only.

Outputs are `/objects/map` (confirmed centroids/labels in Detection3DArray, stable local
track IDs) and `/tracked_objects` (MarkerArray). In RViz/Foxglove, use fixed
frame **map** and add `/tracked_objects` plus `/tf` and `/tf_static`.
Tracking uses observation-time TF into `odom`, then current `map <- odom` for
publication: boat motion and localization corrections are handled separately.
Class-aware nearest-neighbor association confirms 3 observations over 0.2 s;
closely crossing same-class objects can swap IDs. Tracks expire after
3 s unseen, candidates after 1 s. Missing TF retries for up to 1 s; stale data
is dropped. Markers are explicitly deleted and have a short lifetime. These
are ROS parameters on `object_tracker`, not permanent surveyed landmarks.
No yaw-rate blackout is applied. Source loss, occlusion and out-of-view objects
all age out; detecting true removal requires renewed sensor evidence.

## Legacy Jetson → Jazzy

Start the camera first on Foxy, using the same `front` mount name as the boat
URDF. Disable its localization TF; Jazzy EKFs own the boat pose:

```sh
ROS_DOMAIN_ID=142 ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i camera_name:=front publish_tf:=false publish_map_tf:=false
ROS_DOMAIN_ID=142 ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

On Jazzy (install dependency `python3-websocket` if updating an older image):

```sh
ROS_DOMAIN_ID=42 ros2 launch mhseals_nav objects.launch.py source:=bridge rosbridge_url:=ws://squirtle-jetson.local:9090 camera_root_frame:=front_camera_link
```

The bridge reconstructs local Jazzy messages from ZED JSON; it does not relay
incompatible Foxy vision messages. Only `/front/zed_node/obj_det/objects` and
the `/tf_static` subtree **below** `front_camera_link` cross the boundary.
No remote map/odom/base transform, command, service or Nav2 action is bridged.
`objects_topic`, `camera_root_frame` and `detections_topic` are launch arguments.
Start after the camera's static publisher is discovered; reconnect/restart if
an older rosbridge missed transient-local TF during startup. Use a trusted LAN
only; rosbridge is not authenticated. Synchronize host clocks before running.

Jazzy must already publish the measured `base_link -> front_camera_link` mount
(the boat URDF does). Never rename a detection's frame to bypass missing TF.
For another camera name, supply its calibrated mount TF and adjust both topic
and root. Verify:

```sh
ros2 run tf2_ros tf2_echo map front_left_camera_frame
ros2 topic hz /detections
ros2 topic echo /objects/map --once
```

**Detector limitation:** the pinned Foxy/ZED 4 wrapper cannot load this repo's
custom buoy ONNX configuration. Its built-in detector can be enabled with
`ros2 service call /front/zed_node/enable_obj_det std_srvs/srv/SetBool '{data: true}'`,
but built-in classes are not buoy recognition. Real buoy input requires the
compatible Jazzy/custom detector or another producer of the canonical
`/detections` interface. Transport/tracking does not solve that model limitation.
See [primary-source findings](object-interface-research.md).

## Validation

Tests cover array conversion, source reconnect, timestamped translation/rotation,
map corrections, class association, expiry/deletion, missing TF and replayed
packets. `test/nav2_runtime_probe.py` exercises real Nav2 servers with lidar
loss; `--no-lidar` tests deliberate operation without `/points`. Run runtime
tests only in an isolated container (`--network none`, `ROS_DOMAIN_ID=231`).
No test publisher is installed as an operator command. Camera calibration,
buoy classification and propulsion still require real-boat validation.

After pulling into an existing workspace, install `python3-websocket` on the
Jazzy bridge host/container, then rebuild `mhseals_nav` and source
`install/setup.bash`. Devcontainer help follows the mounted helper file;
standalone older images can read `.devcontainer/dev.helper.txt` until rebuilt.
