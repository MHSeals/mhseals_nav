"""Hold the boat through command_mux when the inputs the EKFs fuse go stale.

The EKFs fuse /imu/raw, /gps/fix and /odom/mavros, which our relays and the odometry
adapter publish from MAVROS. With MAVROS alive but those dead, fcu_watchdog (raw MAVROS)
sees nothing wrong while the EKFs predict on and the boat drifts (sim fault F6). Bad: any
of them missing or older than its max age, by receipt time.
"""

from mhseals_nav.hold_watchdog import fresh, HoldGate, node_class as watchdog_class, run

INPUTS = ('imu', 'gps', 'odom')


class EkfInputWatchdog(HoldGate):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.at = dict.fromkeys(INPUTS)

    def receive(self, name, now):
        self.at[name] = now

    def stale_inputs(self, now):
        """Topics of the inputs that are missing or older than their limit."""
        cfg = self.cfg
        return [cfg[f'{name}_topic'] for name in INPUTS
                if not fresh(self.at[name], now, cfg[f'max_{name}_age_s'])]

    def reason(self, now):
        stale = self.stale_inputs(now)
        return f"EKF inputs missing or stale: {', '.join(stale)}" if stale else ''


def node_class():
    import time
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, NavSatFix

    def subscribe(node):
        for name, kind in zip(INPUTS, (Imu, NavSatFix, Odometry)):
            node.create_subscription(
                kind, node.cfg[f'{name}_topic'],
                lambda _, name=name: node.logic.receive(name, time.monotonic()),
                qos_profile_sensor_data)

    return watchdog_class('ekf_input_watchdog', 'ekf_input_watchdog.yaml', EkfInputWatchdog,
                          subscribe)


def main(args=None):
    run(node_class, args)


if __name__ == '__main__':
    main()
