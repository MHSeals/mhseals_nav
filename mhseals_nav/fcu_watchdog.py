"""Hold the boat through command_mux when MAVROS or the FCU link is gone.

MAVROS does not restart after an FCU reboot, and the EKFs keep publishing fresh-stamped
constant-velocity predictions without it, so the controller's stale-odometry stop never
fires. Bad: /mavros/state missing, stale or not connected, or the MAVROS IMU or raw GPS
fix stale.
"""

from mhseals_nav.hold_watchdog import fresh, HoldGate, node_class as watchdog_class, run


class FcuWatchdog(HoldGate):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.state_at = None
        self.connected = False
        self.imu_at = None
        self.gps_at = None

    def state(self, now, connected):
        self.state_at, self.connected = now, connected

    def imu(self, now):
        self.imu_at = now

    def gps(self, now):
        self.gps_at = now

    def reason(self, now):
        cfg = self.cfg
        if not fresh(self.state_at, now, cfg['max_state_age_s']):
            return 'MAVROS state missing or stale'
        if not self.connected:
            return 'MAVROS reports the FCU disconnected'
        stale = [name for name, at, limit in (
            ('IMU', self.imu_at, cfg['max_imu_age_s']),
            ('GPS', self.gps_at, cfg['max_gps_age_s'])) if not fresh(at, now, limit)]
        return f"MAVROS {', '.join(stale)} missing or stale" if stale else ''


def node_class():
    import time
    from mavros_msgs.msg import State
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, NavSatFix

    def subscribe(node):
        cfg, logic = node.cfg, node.logic
        node.create_subscription(
            State, cfg['state_topic'],
            lambda m: logic.state(time.monotonic(), m.connected), qos_profile_sensor_data)
        node.create_subscription(
            Imu, cfg['imu_topic'], lambda _: logic.imu(time.monotonic()),
            qos_profile_sensor_data)
        node.create_subscription(
            NavSatFix, cfg['gps_topic'], lambda _: logic.gps(time.monotonic()),
            qos_profile_sensor_data)

    return watchdog_class('fcu_watchdog', 'fcu_watchdog.yaml', FcuWatchdog, subscribe)


def main(args=None):
    run(node_class, args)


if __name__ == '__main__':
    main()
