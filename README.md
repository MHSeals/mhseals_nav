# mhseals_nav

See [Nav2 commissioning](docs/nav2-commissioning.md) for the MPPI boat profile,
CRANE comparison, launch commands, actuator-interface requirements and tests.
See [EKF contract](docs/ekf-contract.md) for localization inputs and TF ownership.
See [objects and optional lidar](docs/objects.md) for CRANE/ZED tracking,
the Foxy→Jazzy bridge, visualization and `use_lidar:=false` operation.

On the ODROID, `ros2 run mhseals_hardware thruster_pwm_node` subscribes to
`/cmd_vel`; `ros2 run mhseals_hardware keyboard_control` publishes manual effort.
These commands arm real outputs: secure the boat and clear/submerge propellers.
Nav2's `/nav/cmd_vel` is physical velocity, not PWM effort; do not directly
remap it to this driver without a calibrated velocity controller.
