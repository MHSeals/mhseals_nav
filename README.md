# mhseals_nav

### Overview

This package serves as the core navigation system for the ASV, interacting with the simulation and hardware interfaces established in [`crane_sim`](https://github.com/1unarzDev/crane_sim) and [`mhseals_hardware`](https://github.com/mhseals/mhseals_hardware). It encompasses sensor data fusion, localization, object detection and tracking, task detection, path generation, path following, and velocity outputs.

### Usage

Launch files separate the main responsibilities:

- `robot.launch.py` composes sensors, localization, navigation, and objects. `enable_slam:=true` additionally starts the lidar-cloud RTABMap mapper; navigation and EKF localization run either way. Generally used for simulation; on the boat, launch components on their respective hosts.
- `sensors.launch.py` starts the Velodyne driver and lidar relay with `sim:=false`, or the ROS TCP endpoint with `sim:=true`. It never starts the camera.
- `camera.launch.py` is the only launch file the camera host (Jetson Orin, JetPack 6, ZED SDK 5, domain 42) runs. It starts the ZED wrapper with `config/zed_boat.yaml` (HD720, positional tracking off, object detection on) and `config/custom_object_detection.yaml`, publishes no pose TF, and starts lazy relays: `/front_camera/camera_info` and a 2 Hz `/front_camera/preview/compressed`. `slam:=true` adds raw `/front_camera/rgb/image` and `/front_camera/depth/image`. Start it through the container (`docker exec roboboat_dev bash -c 'ros2 launch mhseals_nav camera.launch.py'`) so `ASTRO_DDS_ADDRESS` is set; the launch then pins its camera DDS profile (`config/camera_dds.xml`, 1400-byte messages) to that address.
- `odom.launch.py` starts MAVROS and measurement relays (`fcu:=true`), robot TF, and `ekf.launch.py` (dual EKFs and Navsat; `start_ekf:=false` leaves them to a separate launch, and `~/.config/mhseals/ekf.yaml` overrides parameters per node).
- `navigation.launch.py` starts Nav2 servers and lifecycle management. SLAM is optional.
- `slam.launch.py` starts RTABMap mapping, leaving localization TF with the EKFs.
- `objects.launch.py` always tracks `/detections`. With `sim:=false`, it also converts ZED objects into that interface, remapping the adapter input to `objects_topic`. On the boat it takes no rosbridge arguments: the Orin's wrapper is on the same graph, so the converter subscribes `/front/zed_node/obj_det/objects` directly (`rosbridge_url` and `camera_root_frame` are only for the retiring JetPack 5 host). `config/zed_boat.yaml` sets `prediction_timeout: 0` and `custom_object_detection.yaml` sets `prediction_timeout_s: 0` per class, so `/detections` stays raw.
- `legacy_zed.launch.py` starts the retiring JetPack 5 / ZED SDK 4 camera, TensorRT buoy detector, and rosbridge. Never run it together with `camera.launch.py`; start each camera only once.

```bash
ros2 launch mhseals_nav <launch_name>.launch.py param1:=value1 param2:=value2
ros2 launch mhseals_nav robot.launch.py --show-args
ros2 launch mhseals_nav robot.launch.py sim:=true
```

Start each component once. `robot` includes navigation, so running it also starts Nav2. `sim:=true` requires the simulator's `/clock`; use `sim:=false` on the boat.

`/cmd_vel` is the final smoothed navigation output. Nav2 commands are velocity targets; the hardware PWM driver interprets Twist as effort, so autonomous propulsion requires a calibrated velocity controller.

### Parameters

| Parameter | Launch files | Default / purpose |
| --- | --- | --- |
| `sim` | `robot`, `sensors`, `odom`, `navigation`, `slam`, `objects` | `true`; simulation time requires `/clock`. In `objects`, also selects simulated detections rather than ZED input |
| `use_lidar` | `robot`, `navigation` | `true`; require `/points` for obstacle avoidance. `false` disables both obstacle layers for supervised testing; cameras do not replace them |
| `sensors_ignore` | `robot`, `sensors` | Empty; comma-separated local drivers to skip, e.g. `lidar`. Does not disable Nav2 obstacle layers |
| `enable_slam` | `robot` | `false`; include `slam.launch.py` for lidar-cloud RTABMap mapping from `/points`. It leaves TF ownership with the EKFs and does not change Nav2 localization |
| `fcu` | `robot`, `odom` | `true`; start MAVROS and its relays. `false` when a simulator (crane_ml `--crane-roboboat-contract`) publishes `/odom/mavros`, `/imu/raw` and `/gps/fix` itself, so simulation needs no ArduPilot |
| `fcu_url` | `robot`, `odom` | Empty selects SITL TCP in simulation or the Cube's `/dev/serial/by-id/usb-CubePilot_CubeOrange+_*-if00` on hardware |
| `camera_name` | `robot`, `camera`, `odom`, `slam` | `front`; camera namespace and topic prefix |
| `camera_model`, `slam` | `camera` | `zed2i`, `false`; `slam:=true` also relays raw RGB and depth |
| `detections_topic`, `objects_topic` | `objects` | `/detections`, `/front/zed_node/obj_det/objects`; tracker input and ZED source |
| `rosbridge_url`, `camera_root_frame` | `objects` | Empty; with `sim:=false`, select the legacy JetPack 5 transport and its calibrated camera subtree root. Leave unset on the boat |

Configuration paths, device addresses, and rate overrides are listed by `ros2 launch mhseals_nav <launch_name>.launch.py --show-args`. Keeping `use_lidar:=true` while skipping the local driver allows another host or simulator to supply `/points`. Disabling LiDAR is explicit; stale data never triggers an automatic fallback.

### Architecture

[`astro_dock`](https://github.com/1unarzDev/astro_dock) provides the ROS tooling and container workspace for these components. Simulation and hardware producers satisfy the interfaces below.

The architecture is deliberately topic based: simulated or physical sensors feed
localization, object tracking, and navigation; the global EKF owns `map → odom`,
the local EKF owns `odom → base_link`, and sensor mounts come from the robot
description. Keep the boat on one ROS domain; the Orin camera host joins it directly, so consumers bind only to the canonical names (`/detections`, `/front_camera/*`) and never to `/front/zed_node/*`, except the objects converter.

Required interfaces depend on which components are running:

| Consumer | Required topics / interfaces |
| --- | --- |
| Localization | `/odom/mavros` (`Odometry`, body velocity), `/imu/raw` (`Imu`, FCU heading/yaw rate), `/gps/fix` (`NavSatFix`) |
| Navigation | `/odom/local` (`Odometry`), `/points` (`PointCloud2`, when `use_lidar:=true`), navigation action goals in `map` |
| Object tracking | `/detections` (`Detection3DArray`, metres, acquisition timestamps, actual source frame, labels in `results[].hypothesis.class_id`) |
| Optional SLAM | `/points` (`PointCloud2`), `odom → base_link` TF, and synchronized clocks; camera topics are disabled in the lidar-only profile |
| Simulation | `/clock`, plus the inputs required by the enabled components |

Navigation and tracking also require `/tf` and `/tf_static` with `map → odom → base_link → sensor` connectivity. The global EKF owns `map → odom`, the local EKF owns `odom → base_link`, and the robot description owns sensor mounts. Preserve measurement frame IDs and timestamps, and synchronize clocks across hosts. `/imu/raw` is the legacy name for FCU-fused MAVROS `/imu/data`, rather than `/imu/data_raw`.

Navigation uses MPPI with the Omni model at 10 Hz (the boat can sway) and rolling costmaps of decaying voxels, fed by the LiDAR. Camera tracks retain class and position for task planning and RViz; they do not synthesize obstacle geometry. The characterization's Nav2 overlay gives MPPI the boat's measured limits at the tested effort. RTABMap mapping consumes the live lidar cloud and external EKF odometry; it does not publish a competing localization transform.

The rest of this section describes the retiring JetPack 5 host. On JetPack 5, SDK 4 cannot run `CUSTOM_YOLOLIKE_BOX_OBJECTS`. The legacy detector runs the checked-in 640-pixel ONNX model with TensorRT 8 and sends 2D boxes to the SDK 4 wrapper's `CUSTOM_BOX_OBJECTS` input. The JP5 image includes a wrapper patch that holds each grabbed frame until its matching inference arrives (up to 500 ms); the SDK supplies positions and eight 3D box corners in the reported camera frame. The bridge derives RViz extents from those corners, including depth; no surface-thickness estimate is added. Its first start compiles a GPU-specific FP16 engine; the ZED resources volume retains that cache across container rebuilds. TensorRT/CUDA come from JetPack; OpenCV, NumPy, and PyYAML are required.

The legacy profile uses native 1080p, automatic exposure/white balance, and a camera-scoped 64 MiB DDS shared-memory transport. Alongside the full image, three overlapping 720-pixel crops preserve small buoys near the central water band; same-class duplicates are merged in original pixel coordinates. Use `tile_size:=0` to disable detail crops; `tile_count` sets how many and `tile_center` (fraction of image height, default 0.5) aims the row at the water band, since the horizon follows camera pitch. The optional JPEG preview is capped at 1280 pixels wide to keep it from delaying inference. Stereo settings in `config/legacy_zed.yaml` use a 0.5–20 m range and stricter confidence filtering; backlighting can still prevent reliable depth.

```bash
# Squirtle container (Foxy, domain 142), after building:
source /opt/zed_ros2/setup.bash && source install/setup.bash
ros2 launch mhseals_nav legacy_zed.launch.py
# ODROID host: resolve the current address, then pass it into its Jazzy container:
JETSON_IP=$(getent ahostsv4 squirtle-jetson.local | awk 'NR==1 {print $1}')
docker exec -it roboboat_dev bash -lc "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch mhseals_nav objects.launch.py sim:=false rosbridge_url:=ws://${JETSON_IP}:9090 camera_root_frame:=front_camera_link"
```

Run the ODROID's odometry and sensors as usual. The bridge reconnects after connection loss, retries subscriptions when the camera starts later, and relays only object measurements and camera-internal static TF; ODROID localization and the robot description retain TF ownership. Just like Unity, its tracker publishes `/objects/map` (`Detection3DArray`, `map`) for tasks and `/tracked_objects` (`MarkerArray`) for RViz. Confirmed objects expire after 30 seconds without observations. Keep host clocks synchronized; stale measurements are rejected. The optional annotated camera view is `/buoy_detector/image/compressed`. Low light can prevent detection or stereo depth; indoors, GPS corrections can move `map`, so use `odom` to assess local stability.

The hardware sensor launch also starts the boat's LiDAR relay. It reduces clouds before they cross the LAN and publishes one reliable `/points` writer on normal DDS for both ODROID navigation and remote viewers. Viewers subscribe directly; stop older viewer relays to avoid duplicate scans. astro_dock's default DDS profile keeps map, costmap and cloud packets below the LAN MTU.

RViz markers retain their track IDs across update gaps and expire with `track_timeout`; a temporary TF lookup failure does not delete valid tracks. The ZED converter rejects repeated, reordered, or more than one-second-old packets. The tracker transforms accepted boxes into `odom` at acquisition time, then publishes outlines and 10 cm confidence percentages in `map` on `/detections/markers`, using the same current map correction as semantic tracks. This keeps the laptop independent of delayed camera TF. Raw boxes clear after one second without a fresh packet. Use RViz's standard `MarkerArray` display for this topic to avoid the detection-array outline plugin's origin jump on failed transforms. `/objects/map` retains classed positions and the latest observed confidence, without box geometry; use `/tracked_objects` to visualize that semantic history separately from live detections.

### Mapping and rates

The validated `crane_ml` RoboBoat stack is the reference for the live navigation contract: authoritative `odom → base_link` at 50 Hz, a 10 Hz 3-D LiDAR cloud, and LiDAR-fed rolling costmaps at 10 Hz local / 5 Hz global. It deliberately validates authoritative-odometry navigation rather than SLAM ([baseline](https://github.com/1unarzDev/crane_ml/blob/main/Docs/RoboBoatNavigationBaseline.md)). This package preserves that contract on the boat while adding optional lidar-cloud RTABMap mapping.

On the physical ArduPilot path, `odom.launch.py` starts `mavros_setup`, which requests the MAVLink message rates (`GLOBAL_POSITION_INT` 20 Hz, `ATTITUDE`/`ATTITUDE_QUATERNION` 20, `GPS_RAW_INT` 5, `RAW_IMU` 10, `RC_CHANNELS` 25, `SYS_STATUS` and `EKF_STATUS_REPORT` 2) and repeats them whenever `/mavros/state` reconnects, adapts `/mavros/global_position/local` to `/odom/mavros`, and publishes the local EKF at 30 Hz. The adapter rotates ENU linear velocity into body FLU using the message’s FCU attitude and replaces unknown covariance with `mavros_velocity_stddev` (default 0.3 m/s per axis; in the default GPS velocity mode each `GPS_RAW_INT` sample is fused once with the receiver's `vel_acc` as sigma). Simulation with `fcu:=false` already supplies body velocity. `/mavros/local_position/odom` is silent on the rover profile and unused. The VLP16 driver runs at 600 RPM (10 scans/s); `sensors.launch.py` gives the driver/transform pair the large-data profile in `config/fastdds_large_data.xml`, while odometry and TF use normal DDS. On the sensor host, the `lidar_relay` executable republishes the cloud on normal DDS with one reliable writer compatible with both RViz and Nav2, so each timestamp represents one scan. Its bounded cloud uses an odd sampling stride to retain all 16 laser rings.

The FCU must supply a calibrated absolute heading: ROS ENU yaw = 90° − compass heading (195° bow heading means −105° ENU). Check `/mavros/global_position/compass_hdg` outside, away from metal, against the physical bow. Enable the intended calibrated compass in Mission Planner; board orientation and compass rotation describe their actual mounts. Do not hide an FCU heading error with URDF rotation or `navsat_transform.yaw_offset`.

Use `navigation.launch.py limits_file:=<measured-limits.yaml>` for characterization output; it merges with the current `params_file` and recalculates pruning distance against the current controller horizon. Regenerate old full-profile characterization files.

Start mapping only after the sensor and odometry stacks are live:

```bash
ros2 launch mhseals_nav slam.launch.py sim:=false
ros2 topic hz /points
ros2 topic hz /odom/local
ros2 topic echo /info --once
```

RTABMap should report a live input frequency and publish `/map` while the boat moves. `slam.launch.py` disables RGB/depth synchronization because those camera topics live on the Jetson's separate ROS domain. Nav2 paths and costmaps remain independent of RTABMap and use the EKF `map → odom → base_link` tree.

The checked-in Nav2 profile updates the local costmap at 10 Hz and the global costmap at 2 Hz (published at 1 Hz) with 0.20 m cells, below the `crane_ml` baseline of 5 Hz, to spare the ODROID; planning runs at 1 Hz. Restart `navigation.launch.py` after changing these parameters; an already active lifecycle node retains its previous values.

### FCU parameters

`config/fcu/blastoise.param` is the FCU's as-found state on 2026-10-22 (ArduRover 4.6.3, Cube Orange+), still holding the test-night values `EK3_SRC1_YAW=8` (GSF) and `COMPASS_USE=0`: no compass is used, so the EKF yaw stays 0 and the EKF sits in `CONST_POS_MODE`. Odometry from that window is invalid for heading and velocity. Diff each write against the file, write the batch on land, reboot, then snapshot again. Target values:

| Param | Now | Target | Why |
|---|---|---|---|
| `EK3_SRC1_YAW` | 8 | 1 | Compass yaw |
| `COMPASS_USE` | 0 | 1 | The Here4 (RM3100) is the cleanest, external compass |
| `COMPASS_USE2`, `COMPASS_USE3` | 0 | 0 | The internal mag reads 3x the expected horizontal field |
| `COMPASS_OFS_X/Y/Z` | -529/-873/864 | from the fixed-yaw calibration (MAV_CMD 42006), never hand-typed | The stored offsets give \|B\| 1669 mG against about 470 expected |
| `GPS1_POS_X/Y/Z` | 0 | measured Here4 offset from the Cube (FRD, m) | Lever arm for RTK and velocity under yaw rate |
| `SYSID_MYGCS` | 42 | 255 | Match MAVROS `system_id` |
| `GPS_DRV_OPTIONS` | 0 | 64 for one diagnostic boot, then 0 | Logs each injected RTCM frame |
| `LOG_DISARMED` | 0 | 1 during RTK and calibration tests only | Logs while on the trailer |

Leave `EK3_MAG_CAL`, the `EK3_GSF_*` masks (the emergency yaw reset), `GPS_INJECT_TO` and `SR0_*` alone: message rates come from `mavros_setup`, not the FCU. Once the compass yaw is restored, lower MAVROS's `/mavros/imu` `orientation_stdev` from its 1.0 rad default so the EKF can use it.

### Tasks

RoboBoat tasks are [py_trees](https://py-trees.readthedocs.io) behaviour trees in `mhseals_nav/tasks/`, layered on Nav2: Nav2's XML trees decide *how to drive* (plan, follow, recover), the task trees decide *what to do* (which gates, which beacon, when to report). Leaves are plain Python over a blackboard of tracked objects (`/objects/map`), the boat's map pose and the time, so they are testable without ROS.

```bash
ros2 launch mhseals_nav task.launch.py task:=nav_channel   # or follow_path, docking, speed_challenge, return_home
ros2 run mhseals_nav run_task --task mission --params config/tasks/mission_crane.yaml --ros-args -p use_sim_time:=true
```

The RoboBoat 2025 tasks (handbook section 3.2; task 5, the deliveries, is not attempted):

| Task | Tree |
| --- | --- |
| `nav_channel` | Through two red/green pole gates; sweeps and drives on if the second is out of view |
| `follow_path` | Gate after gate of small red/green buoys until none is ahead, avoiding yellow buoys and vessels; reports the yellow count |
| `docking` | Finds the banner of the day, picks an empty bay under it, stages, slides in, holds, backs out |
| `speed_challenge` | Through the gate, finds the blue mark, circles it, back through the gate; reports the black count (no light panel yet) |
| `return_home` | Back through the black-buoy gate by the start |
| `mission` | All of them in order, each under a time limit; a failed task is skipped, the channel is required |

A task with `approach: [x, y, yaw]` first drives to that map pose, taken from the day's course layout. Counts are reported as an event for the operator display, or with `report: spin` as one turn per object.

Parameters (class labels, distances, routes, timeouts) are in `config/tasks/<task>.yaml`. Each run publishes `/task/events` (JSON, one per event), a latched `/task/status` with the running leaf, and exits 0 on success, 1 on failure, 2 on timeout. `mhseals_testing`'s mission rehearsal launches these and scores the events.

Two commissioning tasks use the same stack; start localization, object tracking and Nav2 first:

```bash
ros2 launch mhseals_nav task.launch.py task:=pair_test
ros2 launch mhseals_nav task.launch.py task:=circle_test
```

`pair_test` crosses the nearest red/green pole pair ahead in either colour order, continues 3 m beyond its centre, turns around, then crosses the same pair back. `circle_test` searches ahead for a yellow buoy, approaches its near side, circles once at a 3 m radius, then returns to the run's initial map position and heading. The pair search sweeps and advances up to 10 m; the yellow search advances up to 15 m. Missing targets fail the test. Adjust distances, circle direction and timeouts in their task YAMLs (`params_file:=...`); add `sim:=true` in simulation. Run one task at a time, or choose it under `t` in `boat_test` for supervised operation and scoring.

### Boat tests

[`mhseals_testing`](https://github.com/mhseals/mhseals_testing) checks this stack's sensors and TF, runs thruster trials and characterization, and scores navigation and missions. Its `measurement.launch.py` includes `odom.launch.py` (with `fcu:=false` in simulation), so localization starts the same way in tests as in operation:

```bash
ros2 run mhseals_testing boat_test --start-stack   # the flight controller is detected
```

The suite's characterization writes a Nav2 overlay with the measured velocity, acceleration, deceleration, and combined-axis limits; pass it as `navigation.launch.py params_file:=...`. Nav2 commands are SI velocities, so they need `calibrated_velocity_controller` in front of the normalized-effort PWM node (`boat_test --velocity-control`).

### Development

Develop inside the [`astro_dock`](https://github.com/1unarzDev/astro_dock) ROS container; its README covers setup and simulation connections. From the workspace root:

```bash
colcon build --symlink-install --packages-select mhseals_nav mhseals_hardware
source install/setup.bash
colcon test --packages-select mhseals_nav mhseals_hardware
colcon test-result --verbose

ros2 node list
ros2 topic hz /odom/local
ros2 topic hz /points
ros2 run tf2_ros tf2_echo map base_link
```

`setup.py` registers `console_scripts` for `ros2 run mhseals_nav <executable>` and installs launch files, configurations, robot descriptions, and behavior trees. Register new runnable nodes there, declare dependencies in `package.xml`, and rebuild/source after installed assets change. Tuning lives in `config/`, physical frames in `description/`, and navigation behavior in `mhseals_nav/behavior_trees/`.

Keep changes lean and scoped. Open a pull request explaining what changed, why, and how it was tested, including any interface changes. Update the README when usage or architecture changes. Run ROS runtime probes in an isolated container/domain without boat hardware.

Read-only odometry/localization catalog constraints are generated from the compiler's
rules and package YAML: [startup catalog guide](docs/startup-catalog-constraints.md).
The installed copy is under `share/mhseals_nav/docs/`; it grants no runtime authority.
