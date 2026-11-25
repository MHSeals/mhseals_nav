# mhseals_nav

The boat's bringup: localization (MAVROS and the EKFs), perception (camera and lidar objects), Nav2, the
RoboBoat task trees and RTK corrections. It runs the same against the real boat and the
[crane_ml](https://github.com/1unarzDev/crane_ml) simulator; the thrusters are
[mhseals_hardware](https://github.com/MHSeals/mhseals_hardware).

```bash
astro up                      # localization, perception and drive, each on its host
astro up nav                  # Nav2 on the boat computer (reads the applied nav2 limits)
astro task --task nav_channel # run one task tree
astro trial line --scale 0.5  # drive a 10 m out-and-back through Nav2 and print its metrics
astro rtk status              # RTK fix, satellites and correction source
```

Commands ([`help.yaml`](help.yaml), `astro help`): [device](#device) · [task](#tasks) · [trial](#trials) ·
[rtk](#rtk-corrections). Also: [Tuning Nav2](#tuning-nav2) · [Architecture](#architecture) ·
[FCU parameters](#fcu-parameters) · [Development](#development)

### Stacks

`astro up` starts the vessel profile's `astro.stacks` (profiles repo, `roboboat/profile.yaml`), each on its
host and each one `device` part or a drive launch:

| Stack | Host | Starts |
| --- | --- | --- |
| `localization` | boat computer | `device:=odroid only:=localization` |
| `perception` | camera host | `device:=jetson` (camera, objects, lidar, host_diagnostic) |
| `drive` | boat computer | `ros2 launch mhseals_hardware drive.launch.py` |
| `nav` (on request) | boat computer | `device:=odroid only:=navigation` |
| `mapping`, `mission` (on request) | boat computer | `only:=mapping`; `run_task --task mission` |

### device

`ros2 launch mhseals_nav device.launch.py device:=PROFILE [only:=a,b] [overrides:='<json>']` starts a
computer's parts; it is the only launch file meant to be started by hand. A profile in `config/devices/`
lists the parts, each an include of a launch file here (with arguments) or one `ros2 run`:

| Profile | Parts |
| --- | --- |
| `odroid` (boat computer) | `localization` (odom.launch.py: MAVROS and relays, robot TF, and the EKFs and navsat_transform through ekf.launch.py), `mapping` (RTAB-Map), `navigation` (Nav2), `mission` (task.launch.py; no task unless given one) |
| `jetson` (camera host) | `camera` (ZED), `objects` (camera backend, tracker, perception costmap), `lidar` (Velodyne and `lidar_relay`), `host_diagnostic` |
| `operator` | `rtk` (`rtk base`) |
| `sim` | the odroid and jetson parts against crane_ml (`fcu:=false`, `backend:=sim`; no camera, lidar or host_diagnostic) plus `sim_bridge` (the Unity ROS-TCP endpoint; its port is `ros_port`) |

`ros2 run mhseals_nav device_parts [PROFILE]` prints the parts `only:=` takes (`part<TAB>what it
starts`). Layers, lowest first: the package profile and its `extends`, `$ROS_WS/config/devices/PROFILE.yaml`,
`~/.config/mhseals/devices/PROFILE.yaml`, then `overrides`; a layer that sets a part the profile lacks
(without its own launch or run) is refused with the part's name. The drive stack (thruster driver,
`command_mux`) is never a part. `sim` defaults to `false`; the `sim` profile sets it, and `sim:=true` needs
the simulator's `/clock`.

The other launch files are the parts' internals: `odom.launch.py`, `ekf.launch.py` (the EKFs alone, to
restart them with MAVROS up; `~/.config/mhseals/ekf.yaml` overrides per node), `camera.launch.py`,
`objects.launch.py`, `sensors.launch.py` (the lidar with `sim:=false`, the Unity endpoint with
`sim:=true`), `navigation.launch.py` (Nav2; its limits are the hardware state file's
`characterization.nav2_limits`, `hardware_config:=`, else `$MHSEALS_HARDWARE_CONFIG`, else
`~/.config/mhseals/hardware.yaml`, clipped to `config/nav2_ceilings.yaml`; restart it after changing Nav2
parameters; `preset:=pool|lake` merges `config/nav2_presets/<name>.yaml` over the base, its `replace` blocks
replacing the base's: the pool preset runs Regulated Pure Pursuit instead of MPPI), `slam.launch.py` and `task.launch.py`.

`/cmd_vel` is the final smoothed navigation output. Nav2 commands are velocity targets; the hardware PWM driver interprets Twist as effort, so autonomous propulsion requires a calibrated velocity controller.

### Parameters

`ros2 launch mhseals_nav <file> --show-args` lists every argument; `help.yaml` documents the commands for astro's help browser. Lidar reaches Nav2 only inside `/perception/costmap`; a no-lidar run is a perception-side choice (`perception_costmap.yaml` lidar_layer), not a Nav2 argument.

### Architecture

[`astro_dock`](https://github.com/1unarzDev/astro_dock) provides the ROS container workspace. Everything is topic based; simulated or physical producers satisfy the interfaces below.

#### Data flow

- Sensors: MAVROS (FCU odometry, IMU, GPS), the VLP16 LiDAR and the ZED camera.
- Localization: a local EKF (`odom → base_link`) and a global EKF with navsat (`map → odom`).
- Navigation: Nav2 with MPPI (Omni model, 10 Hz) and rolling LiDAR costmaps. Camera tracks never become obstacle geometry.
- Perception: camera backend → `/perception/observations` → tracker → `/perception/tracks` (tasks) and `/tracked_objects` (RViz).
- Tasks: py_trees trees consume `/perception/tracks` and the map pose, and send Nav2 goals.
- One ROS domain for the whole boat; consumers bind only to the canonical names (`/perception/*`, `/front_camera/*`), never `/front/zed_node/*`.

#### Interfaces

| Consumer | Required topics |
| --- | --- |
| Localization | `/odom/mavros` (`Odometry`, body velocity), `/imu/raw` (`Imu`, FCU-fused MAVROS `/imu/data`), `/gps/fix` |
| Navigation | `/odom/local`, `/perception/costmap`, action goals in `map` |
| Object tracking | `/perception/observations` (`mhseals_nav/ObservationArray`, metres, acquisition timestamps) |
| SLAM (optional) | `/points`, `odom → base_link` TF, synchronized clocks |
| Simulation | `/clock` plus the inputs of the enabled components |

#### TF ownership

`map → odom` is the global EKF, `odom → base_link` the local EKF, `base_link → sensors` the robot description. Preserve frame IDs and timestamps, and synchronize clocks across hosts.

#### Perception

- Canonical outputs: `/perception/observations`, `/perception/tracks`, `/tracked_objects`, `/perception/detection_markers`, `/front_camera/camera_info`, `/front_camera/preview/compressed`.
- Orin path: the ZED wrapper (`config/zed_boat.yaml`) runs YOLO-like custom detection on the GPU with object tracking (GEN_1 positional tracking whose pose is never published), and `zed_backend` turns `/front/zed_node/obj_det/objects` into `/perception/observations`, one array per frame, dropping predicted objects and repeated, reordered or over-1 s-old frames. If tracking misses 12 Hz or costs over 5 ms, set `pos_tracking_enabled` and `enable_tracking` false in `zed_boat.yaml`.
- The tracker transforms boxes into `odom` at acquisition time, and confirmed tracks expire after 30 s without observations.

#### LiDAR relay and markers

- The `lidar` part (`sensors.launch.py`) also starts `lidar_relay`, which downsamples the cloud and republishes `/points` with one reliable writer for Nav2 and RViz. Stop older viewer relays to avoid duplicates.
- In sim, `tune.smear: true` in `config/lidar.yaml` smears crane's instantaneous scans over a 0.1 s VLP16 sweep with the true motion (`lidar_smear`), so the relay's deskew can be checked.
- Use RViz's `MarkerArray` display for `/perception/detection_markers`; it shows outlines and confidence in `map`.

### Mapping and rates

Rates are set in `config/mavros_setup.yaml` (FCU streams), `config/ekf_*.yaml` (EKFs) and `config/nav2_params.yaml` (costmaps, planner, controller); the VLP16 scan rate is the vessel profile's `lidar.rpm`. Rates are requested again whenever the FCU reconnects. Restart `navigation.launch.py` after changing Nav2 parameters.

**Heading:** ROS ENU yaw = 90° − compass heading (195° bow heading is −105°). Never hide a heading error with the URDF or `navsat_transform.yaw_offset`; fix the FCU.

`slam.launch.py` (RTABMap) should report a live input rate and publish `/map` while the boat moves. It does not publish TF; Nav2 keeps using the EKF tree.

### FCU parameters

`config/fcu/blastoise.param` is the reference snapshot (ArduRover, Cube Orange+); the `*.proposed.param` and `blastoise_gnss_proposal.param` files are unapplied proposals, each line with the old value and why. Diff a change against the snapshot, write on land while disarmed, reboot, then snapshot again into `config/fcu/`. Message rates come from `config/mavros_setup.yaml`, not the `SR*` params.

### Tasks

RoboBoat tasks are [py_trees](https://py-trees.readthedocs.io) behaviour trees in `mhseals_nav/tasks/`, layered on Nav2: Nav2's XML trees decide *how to drive* (plan, follow, recover), the task trees decide *what to do* (which gates, which beacon, when to report). Leaves are plain Python over a blackboard of tracked objects (`/perception/tracks`), the boat's map pose and the time, so they are testable without ROS. `tasks/behaviours.py` holds the driving leaves and decorators, `tasks/plans.py` the `Plan*` leaves that only write routes.

```bash
astro task --task nav_channel      # ros2 run mhseals_nav run_task --task nav_channel
ros2 run mhseals_nav run_task --list
ros2 run mhseals_nav run_task --task mission --params config/tasks/mission_crane.yaml --ros-args -p use_sim_time:=true
```

`--profile qualifying|finals` overlays `config/tasks/profiles/<name>.yaml` on the mission. While a task runs, the runner sends zero velocity to `/cmd_vel_nav` whenever Nav2 has been silent for 0.3 s (between goals, after an abort, during the recovery Wait), so the boat keeps station; the zeros stop at an override, a refusal or the end of the run.

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

When a task's objects are not in sight, it searches rather than waits (`tasks/search.py`, `config/tasks/search.yaml`): it looks at course-map priors and low-confidence tracks first, then turns the camera toward the least-seen bearing ahead, then picks next views by information gain over a coverage grid of where the camera has looked, and finally sweeps the course area. `tasks/tracks.py` reads `/perception/tracks` (with priors and covariance). `tasks/recognition.py` infers which task stands where from object configurations (`config/tasks/recognition.yaml`).

Parameters (class labels, distances, routes, timeouts) are in `config/tasks/<task>.yaml`. Each run publishes:
- `/task/events`: JSON `mhseals.task_events/1`, one per event; `decision` events carry a short reason for the operator.
- `/task/status`: latched JSON `mhseals.mission_status/1` with the stage, phase, goal and why, and per-stage attempts.
- `/task/hypotheses`: latched JSON `mhseals.task_hypotheses/1` (fixtures in `test/fixtures/`).

It exits 0 on success, 1 on failure, 2 on timeout and 3 when a human override (command_mux's latched state, `runner.yaml` `mux_state_topic`, naming rc, manual or hold, or no state within `mux_state_max_age`) stopped it; a Nav2 goal cancelled by anyone but the task counts as an override. A task never resumes after an override, and one started while a human has control refuses to start (exit 4, "human has control"). On exit it waits up to `exit_settle_s` (`config/tasks/runner.yaml`) until Nav2 holds none of its goals.

Commissioning tasks `pair_test` (cross the nearest red/green pole pair and back) and `circle_test` (circle the nearest yellow buoy) use the same stack and `config/tasks/<task>.yaml`; start localization, tracking and Nav2 first and run one task at a time.

### RTK corrections

`astro rtk base` (`ros2 run mhseals_nav rtk base`; the operator profile's `rtk` part) runs on the operator station: it reads a USB base receiver (Here+ M8P or Here4 F9P) and/or a public NTRIP caster and publishes RTCM to MAVROS (`/mavros/gps_rtk/send_rtcm`), with a latched JSON status on `/rtk/status`.

```bash
ros2 run mhseals_nav rtk base  --source ntrip          # NTRIP only; auto (default) uses both, local the USB base only
ros2 run mhseals_nav rtk base  --survey                # USB base (found by itself, or PORT): new survey-in (--known SITE, --resume)
ros2 run mhseals_nav rtk base  --pin MOUNT             # stream one NTRIP mountpoint (MOUNT or CASTER/MOUNT)
ros2 run mhseals_nav rtk status [--watch]             # rover fix, source, survey %, what is sent; exit 1 unless RTCM reaches a subscriber
```

The NTRIP mountpoint is ranked by distance from the rough fix (rover, else base, else the last one saved), plus penalties for older message sets, single band and fewer constellations, so it works wherever a caster has stations. Casters and weights are `config/ntrip.yaml`; station settings and saved sites are `~/.config/mhseals/rtk.yaml`; credentials are in `~/.config/astro/secrets.env` (mode 600), which the dev container must mount. RTK2go needs only an e-mail address as its user, no password: `RTK2GO_USER=<your e-mail>` in that file (FPRN: `FPRN_USER`, `FPRN_PASSWORD`; EarthScope: `EARTHSCOPE_USER`, `EARTHSCOPE_PASSWORD`). A mountpoint is ranked by its own position; only one the sourcetable marks as a network (solution 1 and a NET record) and that has no position takes its network's nearest station. A caster that refuses (HTTP 400/401/403, "not allowed") rests whole for `auth_cooldown_s`, with its reason in the status.

With no port and no `device` in the settings, `rtk base` takes the first u-blox receiver (`/dev/serial/by-id/*u-blox*`, USB vendor 1546, else a ttyACM of unknown vendor) and says which. The F9P's firmware is read from MON-VER: the L1/L2 HPG firmware gets its signal keys, the HPG L1L5 firmware keeps its own signals, and RTCM goes out for the constellations it lists. A refused configuration stops the base with one line naming the message and the fix; it is never retried. While running it prints a line when its state changes, e.g. `survey 3:12  acc 1.84 m (target 2.0)  sats 26  -> sending RTCM 1005/1077/... to /mavros/gps_rtk/send_rtcm (1 subscriber)`, or `-> not sending:` and the reason (no position yet, no credentials for a caster, a pin not in any sourcetable, a refused config, no subscriber on the RTCM topic).

### Trials

`astro trial COURSE` (`ros2 run mhseals_nav trial`; `--list` prints the courses) drives a course from `config/trials/` (`line` 20 m out and back, `square` 10 m, `zigzag`) through Nav2's NavigateThroughPoses, placed relative to the boat's start pose. Start Nav2 first (`astro up nav`).

```bash
astro trial square --scale 0.5
```

It drives one leg per pose (a closed course ends at its start, and Nav2's goal checker only looks at a path's last pose) and reports `done` only when every leg ends within `--tolerance` (0.5 m) of its pose, else `incomplete k/n`. It prints a progress line every second, then time, mean/max cross-track error (`xte`), final position error, yaw-rate sign changes, max speed and `track=vx vy wz` (mean/max of Nav2's `/cmd_vel` minus the odometry body velocity).

**Read `track=` first.** A large track error means the boat isn't doing what Nav2 asked: fix the control layers first ([CONTROL.md](https://github.com/MHSeals/mhseals_hardware/blob/main/docs/CONTROL.md), L3–L6). A small track error with a bad `xte` means: tune Nav2. The line is appended to the day log (`$ASTRO_DATA/<date>/log.txt`) and published on `/events`, with a hash of `controller_server`'s live parameters so runs can be compared by tuning. Bag presets for `astro rec` are `config/bags.yaml`.

### Tuning Nav2

Work in this order; each step assumes the ones before it.

1. **Velocity loop**: `astro test steps` (surge, sway, yaw) until `track=` in a trial is small.
2. **Limits**: `astro test nav2`, then `astro apply nav2`. Nav2 reads them at launch, so restart it (`astro down nav && astro up nav`).
3. **Rotation shim**: `FollowPath.angular_dist_threshold` (rad of heading error before the boat turns in place).
4. **MPPI's few knobs**: the horizon (`time_steps` × `model_dt`; `model_dt` follows `controller_frequency`), `temperature`, and the `cost_weight` of `PathFollowCritic`, `PathAlignCritic`, `GoalCritic` and `PreferForwardCritic`.
5. Leave the planner alone.

The manual loop, one change at a time (a live `ros2 param set` lasts until Nav2 restarts; put the keeper in `config/nav2_params.yaml`):

```bash
ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 14.0
astro mark "PathAlign 14"
astro trial line
```

The trial line carries a hash of `controller_server`'s parameters, so `astro log` groups runs by tuning.

### Validated parameters

Evidence for `config/nav2_params.yaml` as of 2026-11-24. **Nav2 has never reached a goal on the water.** The only field attempt (10-19) ran plain MPPI on raw effort with no velocity loop; it spun about 2.3 turns, and the goal was aborted. Every row below is backed by sim only (crane_ml `astro sim`, seed-1 course, current 0, `astro trial line --scale 0.5`). Square, zigzag, scale 1.0 and the default current are **not yet validated**.

| Group | Value | Evidence |
|---|---|---|
| Limits (base) | vx 0.4, vy 0.15, wz 0.3; ax/ay 0.3, az 0.5 | Chosen 10-23 as "field" values, but no field run used them. **Limits rule:** the measured `characterization.nav2_limits` velocities only lower the configured ones (never raise them); its accelerations replace the configured ones unless the preset states its own, which win |
| Accelerations | ax/ay 0.3, az 0.5 | Sim 11-24: with the sim's `nav2_limits` (0.09/0.10/0.25), line x0.5 never finished. Set live to 0.3/0.3/0.5, it is **done** in 128 s (xte 0.34/0.71 m, final 0.12 m, 6 flips) |
| Rotation shim | `angular_dist_threshold` 0.8, once per approach, to the goal yaw | Sim 11-24: at 1.8 the boat crabbed and looped near goals (line x0.5 incomplete twice, 180 s timeouts) |
| Heading critics | PathAngle weight 4.0, max angle 0.4 rad, thresholds 1.5 m (PathAngle, PathAlign, GoalAngle) | Sim 11-24: same runs as the shim row. Before the change, heading drifted 68 deg on a 10 m leg |
| MPPI horizon/batch | 48 x 0.1 s, 600 samples, 10 Hz | Sim only. Even on the laptop the loop once missed 10 Hz (6 Hz); not measured on the ODROID |
| Goal checkers | xy 0.4 / yaw 0.5 (precise 0.3 / 0.25) | Sim. Needs an RTK fix: without one GPS wander is about 1 m |
| Footprint | hull.yaml 1.063 x 0.895 m + 0.1 padding | Taken from the simplified CAD collision mesh. The visual hull mesh is 1.29 m long (the boat is about 1.34 m), so the bow and stern are about 0.12-0.14 m short |
| Inflation | radius 1.5, scaling 3.0 | The sim entry gate leaves 1.64 m between lethal discs (buoy 0.23 m + 0.1 m growth). An aligned boat has +-0.27 m of margin; at 15 deg off the path, +-0.13 m. MPPI still aborts at the gate once per run ("Optimizer fail"), and the BT recovers |

Velocity loop (mhseals_hardware, sim): yaw kp/ki 0.6/0.4 overshot 87/93 % in `astro test steps-yaw --space small`; 0.1/0.1 gives 4/11 %, and the feedforward alone 0 %. The boat needs its own steps-yaw run before Nav2.

### Development

Develop inside the [`astro_dock`](https://github.com/1unarzDev/astro_dock) ROS container; its README covers setup and simulation connections. From the workspace root:

```bash
colcon build --symlink-install --packages-select mhseals_nav mhseals_hardware
source install/setup.bash
colcon test --packages-select mhseals_nav mhseals_hardware
colcon test-result --verbose
python3 -m pytest -q src/mhseals_nav/test   # the same suite without colcon; ROS-only tests skip on a host
```

`CMakeLists.txt` generates the `ros2 run mhseals_nav <executable>` scripts and installs launch files, configurations, robot descriptions, and behavior trees. Register new runnable nodes there, declare dependencies in `package.xml`, and rebuild/source after installed assets change. Tuning lives in `config/`, physical frames in `description/`, and navigation behavior in `mhseals_nav/behavior_trees/`.

