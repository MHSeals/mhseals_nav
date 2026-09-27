# GPS/IMU localization contract

This is a narrow correction of the existing dual-EKF setup, not a covariance
retune. Camera, DLIO and RTABMap odometry remain excluded from both filters.

| Component | Measurements used | Output / transform |
| --- | --- | --- |
| Local EKF, 80 Hz | FCU body vx/vy; FCU IMU yaw/yaw-rate | `/odom/local`, `odom -> base_link` |
| Navsat, 20 Hz | raw GPS fix, earth-referenced FCU heading, global odometry | `/odom/gps` position |
| Global EKF, 20 Hz | local velocity/heading, GPS x/y only | `/odom/global`, `map -> odom` |

`/imu/raw` is an old topic name for **MAVROS `/imu/data`**, which contains
FCU-fused orientation. It is not `/mavros/imu/data_raw`. Do not pass that heading
through magnetometer-free Madgwick before Navsat; doing so loses the heading
reference needed for geographic alignment. MAVROS already converts NED/FRD
to ENU/FLU; its IMU frame is `base_link`, matching the upstream MAVROS contract.
Verify FCU mounting/orientation calibration physically rather than inventing
an `imu_link` transform. GPS antenna lever-arm calibration remains a separate
physical measurement; the existing FCU base-link convention is retained.

Corrected defects:

- IMU masks must contain 15 booleans in robot_localization state order.
- Navsat GPS odometry does not measure yaw, velocity or angular velocity.
- The normal launch now connects `/mavros/global_position/raw/fix` to `/gps/fix`.
- Navsat receives the ROS 2 `imu` topic, not only the older `imu/data` name.
- MAVROS plugin frame parameters are scoped to their nodes, not a wildcard
  `frame_id: map` that could label all IMU data in the map frame.
- Local odometry fuses velocity instead of FCU absolute GPS-derived position;
  global position comes from GPS, avoiding duplicate position fusion.
- TFs are not arbitrarily future-dated by 0.3 seconds. Diagnostics are enabled.
- The requested attitude stream is MAVLink ATTITUDE_QUATERNION (31), not only
  HIGHRES_IMU (105), which drives the raw acceleration stream.

Nav2's controller is configured at 20 Hz, so 80 Hz local and 20 Hz global filter
rates are adequate **targets**. They do not prove actual throughput, fresh
inputs, or low transport latency. Do not increase requested MAVLink rates beyond
the FCU connection's bandwidth; USB and a 57600-baud UART are not interchangeable.
The `sensor_timeout` setting is a prediction threshold, not a propulsion stop.
IMU linear acceleration is left unfused until bias/noise is characterized.

## Validation without propulsion

```sh
python3 -m pytest test/test_ekf_contract.py test/test_launch_boolean.py
# Inside a sourced ROS container, isolated from boat control:
ROS_DOMAIN_ID=231 python3 test/ekf_runtime_probe.py
```

The runtime probe starts only EKFs/Navsat, sends synthetic stationary IMU/GPS
and odometry, checks sustained output rates, and stops its own processes. It
does not publish cmd_vel or access hardware. Synthetic throughput is not a
substitute for a real-sensor bag and a heading/position accuracy check.

Before navigation, with propulsion disabled, verify actual `/imu/raw`,
`/gps/fix`, `/odom/local`, `/odom/global` rates, `/diagnostics`, valid GPS fix
status, realistic nonzero covariances, and `map -> odom -> base_link` TFs.
Synchronize clocks across ODROID/Jetson/FCU. Test sensor dropout and recovery;
do not assume EKF prediction alone makes GPS loss safe for autonomous motion.
