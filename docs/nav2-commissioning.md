# Boat Nav2 commissioning profile

## Comparison with CRANE

Compared against the local `/home/lunarz/crane_ml/Tools/Performance` checkout.
Its `nav2_controller_fixture.yaml` and launcher are present; the requested
`nav2_roboboat_validated_fixture.yaml`, `nav2_roboboat_distance_replanning.xml`
and `Docs/RoboBoatNavigationBaseline.md` are absent in that checkout.
Do not assume this comparison covers those newer study results.

| Area | Available CRANE fixture | Previous mhseals_nav | New boat baseline |
| --- | --- | --- | --- |
| Controller | RPP, 0.15 m/s, 10 Hz | MPPI, 0.9 m/s, several ineffective parameter names | MPPI, 0.4 m/s and 0.4 rad/s setpoint limits, 20 Hz |
| MPPI budget | N/A | 3000 samples, 50 steps, 0.15 s, incorrect `iterations` | 1000 samples, 60 steps, 0.05 s, `iteration_count: 1` |
| Costmaps | Rolling, no map server, 0.8 m radius | Static-map dependency; 0.4/0.5 m radii | Rolling 12 m local / 60 m global, provisional 0.8 m radius |
| Global reference | odom | map | map, provided by GPS/IMU EKF |
| Planner | NavFn A* | Hybrid Dubins with assumed 0.6 m turning radius | NavFn A*, no unmeasured fixed-turn-radius assumption |
| Recovery | Stock motion behaviors | Fast reverse/spin; malformed custom recovery tree | Two wait/retry attempts, no automatic blind motion or costmap clearing |
| Actuation | Explicit Unity adapter | Implicit MAVROS converter + shared cmd_vel | Dedicated `/nav/cmd_vel`, opt-in actuator integration |

MPPI critic parameters now use `cost_weight`, not ignored `.weight` names.
The controller consumes `/odom/local`; progress checker uses Jazzy's plural
`progress_checker_plugins`. Both NavigateToPose and NavigateThroughPoses trees
are installed and resolved through the package share path. Replanning is at
1 Hz rather than relying on movement: a stopped boat still needs to react to
new obstacles. The old custom BT had a RecoveryNode with too many children,
and the YAML referenced a nonexistent absolute path.

Nav2's server set is explicit so upstream bringup additions (route/docking/
collision-monitor servers) cannot silently require unrelated configuration.
RTABMap is optional and disabled by default. GPS navigation does not require it.
The detection tracker is also opt-in (`enable_object_tracking:=true`); it is
not an obstacle source for these costmaps. The local CUDA image currently has
a NumPy/SciPy incompatibility in that optional tracker, so it is not validated
by the navigation test. This change does not fix its Python dependency stack.
The velocity smoother uses the same limits as MPPI. OPEN_LOOP here describes
the smoother, **not** permission to operate physical thrust without feedback.

## Launch and interface

After building/sourcing the workspace and bringing up verified sensors/odometry:

```bash
ros2 launch mhseals_nav slam_nav.launch.py sim:=false
```

Requires fresh `/points` (`PointCloud2`), `/odom/local` (`Odometry`) and
`map -> odom -> base_link -> sensor` transforms. Send goals to `/navigate_to_pose`
or `/navigate_through_poses` in `map`. For simulation use `sim:=true` and publish
`/clock`; remap simulated odometry and TF to this contract. CRANE's `/crane/odom`
and odom-frame goal fixture are not drop-in replacements for GPS localization.
Its direct FollowPath mode also names `goal_checker`; this profile uses
`general_goal_checker` (NavigateToPose selects it through the tree).

Output is `geometry_msgs/Twist` on **`/nav/cmd_vel`**, after smoothing.
The internal controller stream is `/cmd_vel_nav`. Do not drive actuators from it.
`cmd_vel_topic:=...` explicitly selects the final output. Manual tools continue
to use their existing `/cmd_vel`; this change does not alter their behavior.

**Blocking hardware issue:** the current direct-pin driver maps Twist values
directly to PWM effort. Nav2 expects measured m/s and rad/s velocity tracking.
Thus a 0.4 m/s command is not a measured speed cap. A calibrated feedback
controller with odometry-loss timeout, output saturation, anti-windup and RC/
E-stop arbitration must sit between Nav2 and the mixer before autonomous use.
Do not merely remap `/nav/cmd_vel` onto the PWM driver's topic.
The old MAVROS converter is opt-in (`enable_mavros_velocity:=true`); only use
after verifying FCU control mode, body/world-frame semantics and failsafes.
Do not run two actuator backends or competing manual/autonomous publishers.

## Before water testing

* Measure the complete hull/payload footprint. The 0.8 m circle is provisional,
  not a certified collision envelope. For a polygon set CostCritic's
  `consider_footprint: true` and supply the same footprint to both costmaps.
* Verify REP-103 x-forward/y-left axes physically. The existing URDF places
  port/starboard hulls along x and front/rear thrusters along y; that is a
  potential CAD/body-frame mismatch, not something to rotate blindly in YAML.
* Measure acceleration, braking/coasting, yaw response, drift, GPS noise and
  end-to-end latency. The 0.2 m/s² / 0.3 rad/s² limits and 1 m goal tolerance
  are starting assumptions. Neutral thrust is not an active brake.
* Filter hull returns and water reflections before `/points`. Height cutoffs
  apply in the costmap frame; verify actual sensor height and waterline. Confirm
  low buoys survive filtering. A live cloud can still be blind or inaccurate.
* Observation period limit is 0.5 seconds. Sensor loss should stop Nav2 commands;
  retain an independent downstream watchdog and physical E-stop. Inflation
  (1.2 m) is a cost preference, not a guaranteed stopping-distance margin.
  Sensor timeout, controller timeout and smoothing add delay before terminal
  zero; the test allows four seconds. Do not treat this as an emergency stop.
* The mapless rolling map assumes unseen cells are free. It provides no shoreline,
  geofence or global obstacle memory. Keep goals inside its 30 m half-width with
  margin (e.g. <=20 m); split distant missions into local subgoals. Add a surveyed
  map/geofence for constrained water rather than trusting empty cells.
* Check MPPI deadline misses on the actual Odroid under sensor load. Lower sample
  count if needed; if changing controller frequency, change model_dt with it.
  Desktop performance does not establish ARM64 real-time headroom.

## Repeatable validation

Static contract: `pytest -q test/test_nav2_contract.py test/test_launch_boolean.py`.
Runtime: source ROS and run `ROS_DOMAIN_ID=231 python3 test/nav2_runtime_probe.py`
**only in a network-isolated container with no hardware devices**. The probe
starts the actual server launch, supplies synthetic TF/odometry/clouds, sends
navigation actions, checks output bounds, and withdraws lidar observations.
Its ideal kinematic plant is not Unity or a hydrodynamic simulation. Passing
does not validate GPS, physical TF alignment, obstacle detection, thrust control
or on-water behavior. No physical actuators are used by this test.
After installing/sourcing mhseals_nav, add `--installed-launch` to exercise
the public `slam_nav.launch.py` and installed behavior-tree paths as well.
