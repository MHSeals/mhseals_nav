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

M01b adds `mavros.request_policy.*` from the same selected YAML: positive integer
`retry_attempts`, nonnegative seconds `retry_delay`, and positive seconds
`response_timeout`, `service_wait`, `reboot_dedupe_window`. Every selected layer
is validated before explicit overrides; final ROS values are validated before
clients/subscriptions/workers. Startup policy is immutable and setters require
restart through LiveParams. ACK/request outcome remains pending/unverified;
M01c still must repair partial failures, sent bookkeeping and reboot dedupe.

## Lidar startup layers

`mhseals_nav.lidar_catalog.lidar_catalog` returns the same read-only
`DescriptorBundle` convention for `lidar.model`, `lidar.frame`, `lidar.rpm`,
`lidar.calibration` and `lidar.endpoint`. Supply complete installed baseline
`config/lidar.yaml` bytes/resource and package `config/catalog.yaml` policy.
Defaults, locations and source hash come from those bytes. The finite supported
model is currently VLP16; other model/calibration contracts remain unsupported.
Adjacent `sensors.selection` and `sensors.sim_endpoint` rows explicitly inventory
existing launch-only fields as unsupported, with missing authority blockers.
No live mutation tier, qualified upstream version or behavior evidence is claimed.

The shared core parser/resolver supplies strict types, duplicate/merge-key
rejection, located leaf origins and layer precedence. Startup always requires
installed `config/lidar.yaml`, then applies optional `lidar_deployment_file`,
optional `lidar_params_file`, and genuinely explicit `velodyne_ip`/`velodyne_port`
aliases. All optional selections/aliases default to empty (no layer/override).
A nonempty selected path that is missing, unreadable or invalid refuses startup;
it never falls back. Partial YAML overlays use the `lidar` shape of the baseline;
unknown keys/nulls refuse. RPM permits finite positive integer/float values;
port is an integer1..65535; IP is IPv4; model/frame are exact strings.

The historical address conflict is preserved as explicit deployment selection:
standalone sensors use the canonical endpoint, while full robot selects
`config/lidar_robot.yaml`, which contains only its historical IP override.
Neither address was inferred from current boat networking. A custom selected
file overrides that deployment layer, and an explicit alias overrides both.
There is one common defaults table and one visible deployment value, not
conflicting launcher constants. Resolver output includes every source path/hash
and per-leaf ordered origins, including ephemeral explicit aliases.

The validated model/frame project identically to driver and transform; RPM and
endpoint project only to driver, calibration only to transform. Calibration can
use `{package: velodyne_pointcloud, path: params/VLP16db.yaml}` or a `{file: ...}`
reference. Relative files resolve against the YAML layer that authored that
reference. Package paths resolve through the installed ament index, with no
absolute/traversal package-relative paths. The bounded supported calibration
schema requires the VLP16db top-level fields, sixteen unique integer laser IDs
and finite correction fields; a wrong-model shape, duplicate key/ID, missing
file or corrupt YAML refuses before either physical node is constructed.
Resolver output records resolved calibration path and SHA256 of validated bytes.
Those identities describe inputs at resolution time, not a claim that a later
upstream file read or device behavior was observed; immutable process snapshots
and pinned consumer behavior remain later gates.

The relay preserves cloud headers through `/points`; the existing large-data
transport environment and transform remapping remain paired. Startup-only
changes require restarting the lidar driver/transform pair and affect downstream
cloud consumers (relay, Nav2, RTABMap/DLIO when selected). No stop/restart is
performed by this resolver. Simulation creates only its existing ROS TCP
endpoint; ignored physical lidar constructs nothing and does not resolve unused
physical configuration. `sensors.selection`/`sensors.sim_endpoint` are separate
remaining configuration work, rather than silently supported new YAML keys.

Frame changes have a deliberately limited scope: standalone sensor-local frame
projection is supported, but full robot forwards a fixed `lidar_link` contract
and refuses a different selected frame before physical construction. Blastoise
URDF's fixed `lidar_joint` joins `sensor_mount_link` to `lidar_link`; omni_boat
also has a fixed lidar link and DLIO names `lidar_link`. No URDF/static transform
or complete Nav2/EKF/TF fan-out is rewritten. Full-stack shared-frame migration
remains M06; a standalone custom frame requires a separately coherent external
TF/downstream setup. These tests prove requested node inputs only, not driver
startup, scan delivery, calibration accuracy, TF behavior or live reconfiguration.
