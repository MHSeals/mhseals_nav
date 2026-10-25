# mhseals_nav

### Overview

This package serves as the core navigation system for the ASV, interacting with the simulation and hardware interfaces established in [`crane_sim`](https://github.com/1unarzDev/crane_sim) and [`mhseals_hardware`](https://github.com/mhseals/mhseals_hardware). It encompasses sensor data fusion, localization, object detection and tracking, task detection, path generation, path following, and velocity outputs.

### Usage

Launch files separate the main responsibilities:

- `robot.launch.py` composes sensors, localization, navigation, and objects. `enable_slam:=true` additionally starts the lidar-cloud RTABMap mapper; navigation and EKF localization run either way. Generally used for simulation; on the boat, launch components on their respective hosts.
- `sensors.launch.py` starts the Velodyne driver and lidar relay with `sim:=false`, or the ROS TCP endpoint with `sim:=true`. It never starts the camera.
- `device.launch.py device:=<profile>` is how any computer starts its share of the stack. A device profile in `config/devices/` lists its parts, each an include of a launch file here with arguments, or one `ros2 run`. The profiles are `odroid` (the boat computer, with the hold-only link, fence and FCU watchdogs of `watchdogs.launch.py`), `jetson` (camera, `objects.launch.py backend:=zed`, the lidar and mhseals_testing's `host_diagnostic`), `operator` (the RTK base and the operator heartbeat) and `sim` (both computers on one machine; the watchdogs need no operator or FCU and fence a map-frame box). `only:=a,b` starts a subset, and `overrides:='<json>'` is the session layer over `$ROS_WS/config/devices/<profile>.yaml` and `~/.config/mhseals/devices/<profile>.yaml`. The profiles are the only place a part is assigned to a computer; mhseals_testing derives which host runs what from them. Start it in the container (`docker exec roboboat_dev bash -c 'ros2 launch mhseals_nav device.launch.py device:=jetson'`) so `ASTRO_DDS_ADDRESS` pins the camera's DDS profile; the TUI does this on demand. The thruster driver and `command_mux` are never parts: they start with arming in the TUI.
- `camera.launch.py` starts the ZED wrapper with `config/zed_boat.yaml` and `config/custom_object_detection.yaml`, plus lazy `/front_camera/camera_info` and a 2 Hz preview; `slam:=true` adds raw RGB and depth.
- `odom.launch.py` starts MAVROS and measurement relays (`fcu:=true`), robot TF, and `ekf.launch.py` (dual EKFs and Navsat; `start_ekf:=false` leaves them to a separate launch, and `~/.config/mhseals/ekf.yaml` overrides parameters per node).
- `navigation.launch.py` starts Nav2 servers and lifecycle management. SLAM is optional.
- `slam.launch.py` starts RTABMap mapping, leaving localization TF with the EKFs.
- `objects.launch.py` starts one camera backend (`backend:=zed|sim`), the tracker and the perception costmap. With `sim:=false`, `robot.launch.py` starts neither sensors nor perception; the Orin runs them.

```bash
ros2 launch mhseals_nav <launch_name>.launch.py param1:=value1 param2:=value2
ros2 launch mhseals_nav robot.launch.py --show-args
ros2 launch mhseals_nav robot.launch.py sim:=true
```

Start each component once. `robot` includes navigation, so running it also starts Nav2. `sim:=true` requires the simulator's `/clock`; use `sim:=false` on the boat.

`/cmd_vel` is the final smoothed navigation output. Nav2 commands are velocity targets; the hardware PWM driver interprets Twist as effort, so autonomous propulsion requires a calibrated velocity controller.

While a task runs, the runner sends zero velocity to `/cmd_vel_nav` whenever Nav2 has been silent for 0.3 s (between goals, after an abort, during the recovery Wait), so the boat keeps station. These zeros keep `/cmd_vel` flowing, so the link, fence, FCU and costmap watchdogs treat the task as autonomous from its start: a stale operator link or perception fault now holds the boat during those gaps, and the operator clears it with U. The zeros stop at an override, a refusal or the end of the run.

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
| `backend` | `objects` | `sim`; `zed` reads `config/zed_backend.yaml`, `sim` reads `config/sim_backend.yaml` |

Other arguments and defaults are listed by `--show-args`. Keeping `use_lidar:=true` while skipping the local driver lets another host or simulator supply `/points`; stale data never triggers a fallback.

### Architecture

[`astro_dock`](https://github.com/1unarzDev/astro_dock) provides the ROS container workspace. Everything is topic based; simulated or physical producers satisfy the interfaces below.

#### Data flow

- Sensors: MAVROS (FCU odometry, IMU, GPS), the VLP16 LiDAR and the ZED camera.
- Localization: a local EKF (`odom → base_link`) and a global EKF with navsat (`map → odom`).
- Navigation: Nav2 with MPPI (Omni model, 10 Hz) and rolling LiDAR costmaps. Camera tracks never become obstacle geometry.
- Perception: ZED detections → `/detections` → tracker → `/objects/map` (tasks) and `/tracked_objects` (RViz).
- Tasks: py_trees trees consume `/objects/map` and the map pose, and send Nav2 goals.
- One ROS domain for the whole boat; consumers bind only to the canonical names (`/detections`, `/front_camera/*`), never `/front/zed_node/*`.

#### Interfaces

| Consumer | Required topics |
| --- | --- |
| Localization | `/odom/mavros` (`Odometry`, body velocity), `/imu/raw` (`Imu`, FCU-fused MAVROS `/imu/data`), `/gps/fix` |
| Navigation | `/odom/local`, `/points` (when `use_lidar:=true`), action goals in `map` |
| Object tracking | `/detections` (`Detection3DArray`, metres, acquisition timestamps, labels in `results[].hypothesis.class_id`) |
| SLAM (optional) | `/points`, `odom → base_link` TF, synchronized clocks |
| Simulation | `/clock` plus the inputs of the enabled components |

#### TF ownership

- `map → odom`: global EKF
- `odom → base_link`: local EKF
- `base_link → sensors`: robot description

Preserve frame IDs and timestamps, and synchronize clocks across hosts.

#### Perception

- Canonical outputs: `/detections`, `/objects/map`, `/tracked_objects`, `/detections/markers`, `/front_camera/camera_info`, `/front_camera/preview/compressed`.
- Orin path: the ZED wrapper (`config/zed_boat.yaml`) runs YOLO-like custom detection on the GPU with object tracking (GEN_1 positional tracking whose pose is never published), and `zed_backend` turns `/front/zed_node/obj_det/objects` into `/perception/observations`, one array per frame, dropping predicted objects and repeated, reordered or over-1 s-old frames. If tracking misses 12 Hz or costs over 5 ms, set `pos_tracking_enabled` and `enable_tracking` false in `zed_boat.yaml`.
- The tracker transforms boxes into `odom` at acquisition time, and confirmed tracks expire after 30 s without observations.

The JetPack 5 / Foxy camera host lives on the `legacy/jp5-foxy` branch.

#### LiDAR relay and markers

- `sensors.launch.py` also starts `lidar_relay`, which downsamples the cloud and republishes `/points` with one reliable writer for Nav2 and RViz. Stop older viewer relays to avoid duplicates.
- Use RViz's `MarkerArray` display for `/detections/markers`; it shows outlines and confidence in `map`.

### Mapping and rates

| Stream | Rate | Set in |
| --- | --- | --- |
| `GLOBAL_POSITION_INT` | 20 Hz | `config/mavros_setup.yaml` |
| `ATTITUDE`, `ATTITUDE_QUATERNION` | 50 Hz | `config/mavros_setup.yaml` |
| `RC_CHANNELS` | 25 Hz | `config/mavros_setup.yaml` |
| `RAW_IMU` | 10 Hz | `config/mavros_setup.yaml` |
| `GPS_RAW_INT` | 5 Hz | `config/mavros_setup.yaml` |
| `SYS_STATUS`, `EKF_STATUS_REPORT` | 2 Hz | `config/mavros_setup.yaml` |
| Local EKF | 30 Hz | `config/ekf_local.yaml` |
| Global EKF | 20 Hz | `config/ekf_global.yaml` |
| VLP16 scans | 10 Hz (600 RPM) | `config/lidar.yaml` |
| Local costmap | 10 Hz | `config/nav2_params.yaml` |
| Global costmap | 2 Hz (published 1 Hz) | `config/nav2_params.yaml` |
| Planner | 1 Hz | `config/nav2_params.yaml` |
| Controller | 10 Hz | `config/nav2_params.yaml` |

Rates are requested again whenever the FCU reconnects. Restart `navigation.launch.py` after changing Nav2 parameters.

**Heading:** ROS ENU yaw = 90° − compass heading (195° bow heading is −105°). Never hide a heading error with the URDF or `navsat_transform.yaw_offset`; fix the FCU.

Start mapping once sensors and odometry are live:

```bash
ros2 launch mhseals_nav slam.launch.py sim:=false
ros2 topic hz /points
ros2 topic hz /odom/local
ros2 topic echo /info --once
```

RTABMap should report a live input rate and publish `/map` while the boat moves. It does not publish TF; Nav2 keeps using the EKF tree.

`navigation.launch.py limits_file:=<measured-limits.yaml>` merges characterization limits into the Nav2 parameters.

### FCU parameters

`config/fcu/blastoise.param` is the reference snapshot (ArduRover, Cube Orange+).

1. Diff the intended change against the snapshot.
2. Write on land while disarmed, then reboot.
3. Snapshot again into `config/fcu/`.

Heading comes from the calibrated external compass (fixed-yaw magcal) with the internal compass disabled. Message rates come from `mavros_setup`, not the `SR*` params.

| Param | Value | Why |
| --- | --- | --- |
| `EK3_SRC1_YAW` | 1 | Compass yaw |
| `COMPASS_USE` | 1 | External compass |
| `COMPASS_USE2`, `COMPASS_USE3` | 0 | Internal compass disabled |
| `COMPASS_OFS_X/Y/Z` | from magcal | Never hand-typed |
| `SYSID_MYGCS` | 255 | Matches MAVROS `system_id` |
| `GPS1_POS_X/Y` | `gps_link` x, −y | Antenna lever arm; the URDF is the source, FRD negates y |
| `INS_POS1..3_X/Y` | `imu_link` x, −y | Cube IMU offset from the CoM, same source |

### Tasks

RoboBoat tasks are [py_trees](https://py-trees.readthedocs.io) behaviour trees in `mhseals_nav/tasks/`, layered on Nav2: Nav2's XML trees decide *how to drive* (plan, follow, recover), the task trees decide *what to do* (which gates, which beacon, when to report). Leaves are plain Python over a blackboard of tracked objects (`/objects/map`), the boat's map pose and the time, so they are testable without ROS.

```bash
ros2 launch mhseals_nav task.launch.py task:=nav_channel   # or follow_path, docking, speed_challenge, return_home
ros2 run mhseals_nav run_task --task mission --params config/tasks/mission_crane.yaml --ros-args -p use_sim_time:=true
```

The RoboBoat 2025 tasks (handbook section 3.2; task 5, the deliveries, is not attempted):

| Task | Tree |
| --- | --- |
| `nav_channel` | Through two red/green pole gates in one route; looks along unexplored bearings if the second is out of view |
| `follow_path` | Gate after gate of small red/green buoys, the visible chain as one route, until none is ahead; reports the yellow count |
| `docking` | Finds the banner of the day, picks an empty bay under it, stages, slides in, holds, backs out |
| `speed_challenge` | Through the gate, finds the blue mark, then the lap and the return through the gate as one route; reports the black count (no light panel yet) |
| `return_home` | Back through the black-buoy gate by the start |
| `mission` | All of them in order, each under a time limit and up to its `attempts`; a failed task is then skipped, the channel is required |

A task with `approach: [x, y, yaw]` first drives to that map pose, taken from the day's course layout; without one it drives to where the recogniser has seen that task, if anywhere. Counts are reported as an event for the operator display, or with `report: spin` as one turn per object.

When a task's objects are not in sight, it searches rather than waits (`tasks/search.py`, `config/tasks/search.yaml`): it looks at course-map priors and low-confidence tracks first, then turns the camera toward the least-seen bearing ahead, then picks next views by information gain over a coverage grid of where the camera has looked, and finally sweeps the course area. `tasks/tracks.py` reads `/objects/map` today and `/perception/tracks` (with priors and covariance) when it is published. `tasks/recognition.py` infers which task stands where from object configurations (`config/tasks/recognition.yaml`).

Parameters (class labels, distances, routes, timeouts) are in `config/tasks/<task>.yaml`. Each run publishes:
- `/task/events`: JSON `mhseals.task_events/1`, one per event; `decision` events carry a short reason for the operator.
- `/task/status`: latched JSON `mhseals.mission_status/1` with the stage, phase, goal and why, and per-stage attempts.
- `/task/hypotheses`: latched JSON `mhseals.task_hypotheses/1` (fixtures in `test/fixtures/`).

It exits 0 on success, 1 on failure, 2 on timeout and 3 when a human override (rc, keyboard, hold) stopped it; a Nav2 goal cancelled by anyone but the task counts as an override. A task never resumes after an override, and one started while a human has control refuses to start (exit 4, "human has control"). On exit it waits up to `exit_settle_s` (`config/tasks/runner.yaml`) until Nav2 holds none of its goals. `mhseals_testing`'s mission rehearsal launches these and scores the events.

Two commissioning tasks use the same stack; start localization, object tracking and Nav2 first:

```bash
ros2 launch mhseals_nav task.launch.py task:=pair_test
ros2 launch mhseals_nav task.launch.py task:=circle_test
```

`pair_test` crosses the nearest red/green pole pair and back; `circle_test` circles the nearest yellow buoy and returns to its start pose. Distances, direction and timeouts are in their task YAMLs (`params_file:=...`); add `sim:=true` in simulation. Run one task at a time.

### Boat tests

[`mhseals_testing`](https://github.com/mhseals/mhseals_testing) checks this stack's sensors and TF, runs thruster trials and characterization, and scores navigation and missions. Its `measurement.launch.py` includes `odom.launch.py` (with `fcu:=false` in simulation), so localization starts the same way in tests as in operation:

```bash
ros2 run mhseals_testing boat_test --start-stack   # the flight controller is detected
```

The suite's characterization writes a Nav2 overlay with the measured velocity, acceleration, deceleration, and combined-axis limits; pass it as `navigation.launch.py params_file:=...`. Nav2 commands are SI velocities, so they need `calibrated_velocity_controller` in front of the normalized-effort PWM node (`boat_test --velocity-control`).

### Tuning

MPPI plans as if its command were the velocity. Tuning measures what sits in between and turns each part into a key by a rule. Steady plant differences (mass, drag, thrust) are absorbed by the velocity loop and need no Nav2 change; delay, noise and geometry do. `T = tau + L` per axis.

| Measure | How | Parameter: rule |
| --- | --- | --- |
| Dead time `L` | Characterize yaw onsets, first-order-plus-dead-time fit | `wz_max`, `rotate_to_heading_angular_vel`: `yaw_tol / (2 T_yaw)`; fix latency first if `L` > 0.3 s |
| Lag `tau` per axis | Controller steps, rise time / ln 9 | `vx_max`, `vy_max`: `D / (T (k/2 + 1))`, stop within `D` = 1 m; accelerations `v / (k T)`, `k` = 3 |
| Neutral velocity SD `sigma` | Characterize neutral phase (wave floor) | stopped velocities at least `2 sigma`; velocity `kp` at most `0.0625 / (2 sigma)` (hardware owner) |
| Odometry noise in the plan | `/cmd_vel_nav` sign changes | `temperature` up, `*_std` down, each std under its axis limit |
| Rotation-centre offset `d` | Characterize yaw rows | xy goal tolerances at least `2 sigma_p + l + 1.6 d` (the final in-place turn starts inside them); in-place turns at most 0.3 rad/s while `d` > 0.08 m |
| GPS SD `sigma_p` per fix tier, antenna offset `l` | 5 min moored per tier; tape | xy goal tolerances at least `2 sigma_p + l`; precise goals (docking) only at RTK fixed |

The characterization's overlay takes the smaller of the measured limit and the rule (`nav2_rules` in mhseals_testing's `config/characterize.yaml`). Verify every change in crane_ml before the boat. The derivations, evidence and acceptance thresholds are in mhseals_testing `docs/tuning.tex` (`make tuning` in `docs/`).

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

Keep changes lean and scoped; update this README when usage or architecture changes.
