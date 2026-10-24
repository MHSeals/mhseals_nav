# Localization startup inputs

The package's three existing `config/ekf_local.yaml`, `ekf_global.yaml` and
`navsat_transform.yaml` files remain authoritative defaults. Full robot, odometry
include and standalone `ekf.launch.py` forward the same three base selections and
`ekf_overlay_file`. Selected bases replace their corresponding files. Each base
must state the fixed graph identities; omitted supported scalar fields remain
omitted rather than copying upstream example defaults.

An empty `ekf_overlay_file` inherits the optional standard
`~/.config/mhseals/ekf.yaml`. Only a missing implicit default is tolerated. An
explicit path must exist even when it spells the standard location. Existing
invalid/unreadable defaults fail. Empty does not disable an existing standard
overlay; no separate disable sentinel is provided in this packet. An empty
explicit overlay document supplies no overrides.

Overlay sections name only `ekf_local`, `ekf_global`, `navsat_transform`, with
flat parameter mappings or a sole `ros__parameters` wrapper per section. Every
base and supplied overlay field is independently validated before precedence.
Duplicates, aliases/cycles, merge/nonstring keys, unknown fields and mixed
wrappers fail. Fusion arrays replace whole arrays and require exactly 15 actual
booleans: x/y/z, roll/pitch/yaw, linear velocities, angular velocities,
accelerations. New sensor slots and covariance keys are unsupported.

Known doubles normalize finite integers/floats to floats; booleans remain exact
booleans. Frequency must be positive, timeout/delay nonnegative, and shipped
queue sizes positive integers. Signed finite transform offsets, yaw offsets and
declination remain supported. These local policies do not assert undocumented
upstream maximum rates, queue sizes or angle ranges. Omitted covariance matrices
stay omitted. Matrix shape alone would not prove symmetry or positive
semidefiniteness; covariance support needs separate source/math review.

The graph stays map=`map`, odom=`odom`, base=`base_link`; local world=`odom`,
global world=`map`; odom0=`/odom/mavros`, imu0=`/imu/raw`, global
odom1=`/odom/gps`. Changes in a selected base or overlay refuse. Navsat's three
shipped frame strings are preserved but their direct parameter consumption is
**unverified** by the installed 3.8.3 example/header evidence. They are not
advertised as effective frame controls. `predict_to_current_time` consumption
also needs exact implementation trace. The compiler exposes these uncertainties
in `unverified_consumers`; changing the shared graph is outside this boundary.

`compile_localization` returns immutable per-node mappings and exact selected
paths/SHA256 identities, plus explicit/default overlay provenance and the clock
selection. All three nodes compile before any Node/TimerAction construction.
Nodes receive fresh parameter dictionaries rather than original YAML paths, so
later file edits cannot alter the validated launch inputs. The explicit `sim`
selector must be true/false; hidden YAML `use_sim_time` is rejected. The existing
immediate-navsat/delayed-EKF sequence and fixed remappings remain intact.

This is fresh-start input validation, not frozen session replay, runtime EKF
restart activation, datum preservation, authority or behavior verification.
Changing fusion masks, wait-for-datum or odometry-yaw still needs operational
readiness/heading review. Installed robot_localization 3.8.3 documentation
substantiates supported shapes, not an exhaustive C++ declaration/consumption
contract. TUI full-stack profile frame/topic agreement and protected restart
policy remain the testing owner's separate launch/preflight obligation; direct
Unity continues to bypass this localization path.
