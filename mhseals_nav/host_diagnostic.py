"""host_diagnostic: the camera host's temperature, power mode and free disk on /diagnostics.

Runs on the camera computer as the jetson device profile's ``host_diagnostic`` part
(config/devices/jetson.yaml) and publishes one DiagnosticStatus with ``hardware_id: host:<host_name()>``
every 2 s, so the boat reads it without SSH (mhseals_testing's preflight probe is the reader).
Reads sysfs and the nvpmodel status only; ``nvpmodel``, ``jetson_clocks`` and the fan are
never touched. A missing source (a container without /var/lib/nvpmodel) is left out.
"""

import glob
from pathlib import Path
import shutil
import sys

from mhseals_nav.link_watchdog import host_name

PERIOD_S = 2.0
HOST_PREFIX = 'host:'
THERMAL_ZONES = '/sys/class/thermal/thermal_zone*/temp'
NVPMODEL_STATUS = '/var/lib/nvpmodel/status'
NVPMODEL_CONF = '/etc/nvpmodel.conf'


def host_values(zones=THERMAL_ZONES, status=NVPMODEL_STATUS, conf=NVPMODEL_CONF, data='/'):
    """This host's readings as strings. Unreadable sources are left out, so the boat reads them
    as unknown."""
    values = {}
    temperatures = []
    for zone in glob.glob(zones):
        try:
            temperatures.append(int(Path(zone).read_text()) / 1000)
        except (OSError, ValueError):
            pass
    if temperatures:
        values['temp_max_c'] = f'{max(temperatures):.1f}'
    try:
        mode = Path(status).read_text().split('pmode:', 1)[1].split()[0].lstrip('0') or '0'
        names = {}
        for line in Path(conf).read_text().splitlines():
            if 'POWER_MODEL' in line and 'ID=' in line and 'NAME=' in line:
                fields = dict(part.split('=', 1) for part in line.strip('<> ').split() if '=' in part)
                names[fields['ID']] = fields['NAME']
        values['power_mode'] = names.get(mode, f'mode {mode}')
    except (OSError, IndexError, KeyError):
        pass
    try:
        values['disk_free_gb'] = f'{shutil.disk_usage(data).free / 1e9:.1f}'
    except OSError:
        pass
    return values


def main(argv=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    rclpy.init(args=argv if argv is not None else sys.argv)
    node = rclpy.create_node('host_diagnostic')
    publisher = node.create_publisher(DiagnosticArray, '/diagnostics', 10)
    host = host_name()
    data = node.declare_parameter('data_directory', '/').value

    def publish():
        values = host_values(data=data)
        message = DiagnosticArray()
        message.header.stamp = node.get_clock().now().to_msg()
        message.status = [DiagnosticStatus(
            level=DiagnosticStatus.OK, name='host_diagnostic', hardware_id=HOST_PREFIX + host,
            message=', '.join(f'{k} {v}' for k, v in values.items()),
            values=[KeyValue(key=k, value=v) for k, v in values.items()])]
        publisher.publish(message)

    node.create_timer(PERIOD_S, publish)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # launch's SIGINT/SIGTERM shuts the context down
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
