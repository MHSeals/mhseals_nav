# Selected navigation catalog

Supply installed `config/catalog.yaml` bytes as `policy_raw` and the selected
installed parameter YAML bytes/resource path to `navigation_catalog`. No files
are opened by this API. Policy YAML carries authoring, tier, safety and evidence
contracts; Python expands the existing compiler's finite selected-plugin closure
and records actual resource SHA256, pointers and source lines. No defaults are
copied. `REGISTRY` binds existing callables only; compilation receives complete
`(base, explicit_overlay)` mappings, never isolated scalar values. Validator input
paths enumerate the affected canonical closure, not a callable argument adapter.

All upstream consumer builds, selection predicates, runtime safety/state and
write capabilities remain unresolved. Every row stays read-only. A compiler
parity recipe declares no achieved ROS behavior. Supplied unqualified consumer
build IDs refuse. Candidate tiers remain unsupported pending owner evidence.

MPPI selection follows actual plugin identity (including a renamed shim instance).
Explicit other controllers receive no MPPI descendants. The optional smoother
appears only when its parameter section is selected; actual process selection
and runtime evidence remain unavailable. Behavior rotational limits have their
own single-leaf closures. Frequency controls model_dt; conflicting explicit
model_dt is rejected by the existing compiler. Noise, shim and smoother outputs
remain subject to the compiler's canonical conflict rules.

Remaining inventory includes all unlisted MPPI critic/plugin descendants,
progress/goal checkers, costmaps/STVL/inflation, planner, BT/waypoint execution,
smoother feedback/deadband/rates, other behavior parameters, frames/clock/topics,
EKF/navsat, MAVROS, sensors/perception/tasks and launch/environment/build fan-out.
These need separate bounded owner traces and descriptors; this slice does not
claim the historical nav-launch audit is fully reconciled.

## MAVROS desired intervals

`mhseals_nav.catalog.mavros_catalog` registers the named `mavros.intervals`
family using the same `DescriptorBundle` convention as `navigation_catalog`.
Callers supply selected installed bytes, the package-relative resource name and
`config/catalog.yaml` policy bytes. Defaults and source hashes come only from
`/mavros_setup/ros__parameters/message_rates`; there is no catalog rate table.
Consumer build and runtime safety references remain unsupported/unknown blockers.
This descriptor is read-only and does not enroll this family in managed hot writes.

`config/mavros_setup.yaml` is the startup authority for both `odom.launch.py`
and the direct `mavros_setup` executable. `robot.launch.py` forwards
`mavros_setup_params_file` separately from `mavros_params_file` (plugin settings).
The selected file is required: empty selection, missing files, malformed YAML,
duplicate keys/IDs, extra keys, non-string entries, nonintegral/out-of-range IDs,
and nonfinite/unsupported negative rates fail before setup Node construction.
IDs are MAVLink 2 unsigned 24-bit integers. Positive Hz requests a rate, zero
requests firmware default policy, and exactly -1 requests disabling a stream.
This packet restricts the negative sentinel to -1; pinned MAVROS transport
conversion and FCU behavior still require the separate transport evidence gate.
No negative value means observed delivery stopped.

Startup precedence is selected package/deployment file, then explicitly supplied
`mavros_odom_rate` (33), `mavros_imu_rate` (30 and 31), `mavros_gps_rate` (24),
and `mavros_rc_rate` (65). All aliases default to empty at both include levels;
an explicit old-default value is still an override. The final complete rate map
is validated before construction. The selected file is passed first and that
validated map last. Direct executable ROS parameter overrides are validated
before subscriptions/client/worker creation. The installed package default is
loaded before rclpy initialization. No runtime reloader is added.

| Message ID | MAVLink source | Raw ROS consumer | Downstream contract |
| --- | --- | --- | --- |
| 33 | GLOBAL_POSITION_INT | /mavros/global_position/local | /odom/mavros adapter |
| 30 | ATTITUDE | MAVROS IMU attitude processing | attitude companion to 31 |
| 31 | ATTITUDE_QUATERNION | /mavros/imu/data | /imu/raw relay |
| 24 | GPS_RAW_INT | /mavros/global_position/raw/fix | /gps/fix relay |
| 27 | RAW_IMU | /mavros/imu/mag | compass diagnostics |
| 1 | SYS_STATUS | MAVROS system status | sensor health |
| 193 | EKF_STATUS_REPORT | /uas1/mavlink_source | /fcu/ekf_status decoder |
| 65 | RC_CHANNELS | /mavros/rc/in | RC arbitration and rate diagnostics |

The shared IMU alias historically controls both attitude IDs; the package raises
both to 50 Hz following the approved 31=50 Hz attitude sampling decision and
retains that paired behavior. Other package rates retain the existing requests.
The target remains FCU system 1/autopilot component 1 by existing default;
connection identity is a separate future descriptor, not a new YAML setting.

The closure is desired list -> validated alias projection -> setup parameter ->
existing reconnect/boot worker -> `/mavros/set_message_interval`. No reconnect
trigger, retry policy, RC monitor threshold, request result handling, or worker
reset bookkeeping changes here. Dropping an ID creates a later reset obligation;
it is not evidence that the FCU stopped a stream. Disabling/resetting required
streams can remove control/health inputs. No concrete startup required-ID policy
exists here; guarded runtime authority policy remains pending and no diagnostic
minimum is promoted to a safety floor. The existing `LiveParams` setter remains
legacy/unmanaged. Its permissive parser is not an alternate startup validator.
Evidence is **requested/unverified**, including accepted ACKs: these offline
construction tests establish propagation, not request delivery or behavior.
