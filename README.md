# mhseals_nav

### Overview

This package serves as the core navigation system for the ASV, interacting with the simulation and hardware interfaces established in [`crane_sim`](https://github.com/1unarzDev/crane_sim) and [`mhseals_hardware`](https://github.com/mhseals/mhseals_hardware). It encompasses sensor data fusion, localization, object detection and tracking, task detection, path generation, path following, and velocity outputs.

### Usage

Launch files separate the main responsibilities:

- `robot.launch.py` composes sensors, localization, navigation, and objects. `enable_slam:=true` additionally starts the lidar-cloud RTABMap mapper; navigation and EKF localization run either way. Generally used for simulation; on the boat, launch components on their respective hosts.
- `sensors.launch.py` starts Velodyne/ZED drivers with `sim:=false`, or the ROS TCP endpoint with `sim:=true`. Use `sensors_ignore:=camera` on ODROID and `sensors_ignore:=lidar` on Jetson.
- `odom.launch.py` starts MAVROS and measurement relays (`fcu:=true`), dual EKFs, Navsat, and robot TF.
- `navigation.launch.py` starts Nav2 servers and lifecycle management. SLAM is optional.
- `slam.launch.py` starts RTABMap mapping, leaving localization TF with the EKFs.
- `objects.launch.py` always tracks `/detections`. With `sim:=false`, it also converts ZED objects into that interface, remapping the adapter input to `objects_topic`. Set `rosbridge_url` and `camera_root_frame` for a legacy ZED host.
- `legacy_zed.launch.py` starts the JetPack 5 / ZED SDK 4 camera, TensorRT buoy detector, and rosbridge. Use it instead of the camera part of `sensors.launch.py`; start each camera only once.

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
| `sensors_ignore` | `robot`, `sensors` | Empty; comma-separated local drivers to skip, e.g. `camera,lidar`. Does not disable Nav2 obstacle layers |
| `enable_slam` | `robot` | `false`; include `slam.launch.py` for lidar-cloud RTABMap mapping from `/points`. It leaves TF ownership with the EKFs and does not change Nav2 localization |
| `fcu` | `robot`, `odom` | `true`; start MAVROS and its relays. `false` when a simulator (crane_ml `--crane-roboboat-contract`) publishes `/odom/mavros`, `/imu/raw` and `/gps/fix` itself, so simulation needs no ArduPilot |
| `fcu_url` | `robot`, `odom` | Empty selects SITL TCP in simulation or `/dev/ttyACM0` at 57600 baud on hardware |
| `camera_name` | `robot`, `sensors`, `odom`, `slam` | `front`; camera namespace and topic prefix |
| `detections_topic`, `objects_topic` | `objects` | `/detections`, `/front/zed_node/obj_det/objects`; tracker input and ZED source |
| `rosbridge_url`, `camera_root_frame` | `objects` | Empty; with `sim:=false`, select legacy transport and its calibrated camera subtree root |

Configuration paths, device addresses, and rate overrides are listed by `ros2 launch mhseals_nav <launch_name>.launch.py --show-args`. Keeping `use_lidar:=true` while skipping the local driver allows another host or simulator to supply `/points`. Disabling LiDAR is explicit; stale data never triggers an automatic fallback.

### Architecture

[`astro_dock`](https://github.com/1unarzDev/astro_dock) provides the ROS tooling and container workspace for these components. Simulation and hardware producers satisfy the interfaces below.

The architecture is deliberately topic based: simulated or physical sensors feed
localization, object tracking, and navigation; the global EKF owns `map → odom`,
the local EKF owns `odom → base_link`, and sensor mounts come from the robot
description. Keep this graph in one ROS domain per ROS release and use the
explicit camera bridge when the Jetson runs a different release.

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

On JetPack 5, SDK 4 cannot run `CUSTOM_YOLOLIKE_BOX_OBJECTS`. The legacy detector runs the checked-in 640-pixel ONNX model with TensorRT 8, matches RGB and registered depth by acquisition timestamp, and publishes metric positions in the actual optical frame. Its first start compiles a GPU-specific FP16 engine; the ZED resources volume retains that cache across container rebuilds. TensorRT/CUDA come from JetPack; OpenCV, NumPy, and PyYAML are required.

```bash
# Squirtle container (Foxy, domain 142), after building:
source /opt/zed_ros2/setup.bash && source install/setup.bash
ros2 launch mhseals_nav legacy_zed.launch.py
# ODROID host: resolve the current address, then pass it into its Jazzy container:
JETSON_IP=$(getent ahostsv4 squirtle-jetson.local | awk 'NR==1 {print $1}')
docker exec -it roboboat_dev bash -lc "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch mhseals_nav objects.launch.py sim:=false rosbridge_url:=ws://${JETSON_IP}:9090 camera_root_frame:=front_camera_link"
```

Run the ODROID's odometry and sensors as usual. The bridge reconnects after connection loss, retries subscriptions when the camera starts later, and relays only object measurements and camera-internal static TF; ODROID localization and the robot description retain TF ownership. Just like Unity, its tracker publishes `/objects/map` (`Detection3DArray`, `map`) for tasks and `/tracked_objects` (`MarkerArray`) for RViz. Confirmed objects expire after 30 seconds without observations. Keep host clocks synchronized; stale measurements are rejected. The optional annotated camera view is `/buoy_detector/image/compressed`. Low light can prevent detection or stereo depth; indoors, GPS corrections can move `map`, so use `odom` to assess local stability.

The hardware sensor launch also starts the local LiDAR relay. Each host's relay publishes locally, so ODROID navigation works without a viewer and another viewer's relay cannot duplicate its scans.

RViz markers retain their track IDs across update gaps and expire with `track_timeout`; a temporary TF lookup failure does not delete valid tracks.

### Mapping and rates

The validated `crane_ml` RoboBoat stack is the reference for the live navigation contract: authoritative `odom → base_link` at 50 Hz, a 10 Hz 3-D LiDAR cloud, and LiDAR-fed rolling costmaps at 10 Hz local / 5 Hz global. It deliberately validates authoritative-odometry navigation rather than SLAM ([baseline](https://github.com/1unarzDev/crane_ml/blob/main/Docs/RoboBoatNavigationBaseline.md)). This package preserves that contract on the boat while adding optional lidar-cloud RTABMap mapping.

On the physical ArduPilot path, `odom.launch.py` requests `GLOBAL_POSITION_INT` (MAVLink message 33) at 20 Hz, adapts `/mavros/global_position/local` to `/odom/mavros`, and publishes the local EKF at 30 Hz. The adapter rotates ENU linear velocity into body FLU using the message’s FCU attitude and replaces unknown covariance with `mavros_velocity_stddev` (default 0.2 m/s per axis). Simulation with `fcu:=false` already supplies body velocity. The previous `/mavros/local_position/odom` topic can be advertised without emitting samples on the rover profile. The VLP16 driver runs at 600 RPM (10 scans/s); `sensors.launch.py` gives the driver/transform pair the large-data profile in `config/fastdds_large_data.xml`, while odometry and TF use normal DDS. On the viewer, the `lidar_relay` executable republishes the cloud on normal DDS with one reliable writer compatible with both RViz and Nav2, so each timestamp represents one scan. Its bounded cloud uses an odd sampling stride to retain all 16 laser rings.

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

The checked-in Nav2 profile updates the local costmap at 10 Hz and the global costmap at 5 Hz, matching the `crane_ml` commissioning baseline. Restart `navigation.launch.py` after changing these parameters; an already active lifecycle node retains its previous values.

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
