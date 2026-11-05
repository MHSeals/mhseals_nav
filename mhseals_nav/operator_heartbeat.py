"""operator_heartbeat: /operator/heartbeat (std_msgs/String, the host name) for link_watchdog.

Runs on the operator's computer as the operator device profile's ``operator_heartbeat`` part
(config/devices/operator.yaml). The boat's link_watchdog counts only beats naming one
of its ``operator_hosts`` and never the boat itself, so a stray copy on a boat computer cannot
hide a lost link; that filter lives there, not here. The name is this machine's
(``ASTRO_HOSTNAME``, else the host name, short like link_watchdog reads it): a container from before
prebuild named containers after their machine is ``roboboat``, which no operator_hosts lists,
so there ``ASTRO_HOSTNAME`` names the machine. Never the registry's address match, which on the
gateway address can name another computer.
"""

import sys

from mhseals_nav.link_watchdog import host_name
from mhseals_nav.yaml_params import section_config, WATCHDOGS

BEATS_PER_AGE = 6   # beats within link_watchdog's max_age: 3 s -> every 0.5 s


def link():
    """(heartbeat_topic, period): link_watchdog's section of watchdogs.yaml, the receiver's."""
    cfg = section_config(WATCHDOGS, 'link_watchdog')
    return cfg['heartbeat_topic'], cfg['max_age'] / BEATS_PER_AGE


def bind(node):
    from std_msgs.msg import String
    topic, period = link()
    pub = node.create_publisher(String, topic, 1)
    msg = String(data=host_name())
    return node.create_timer(period, lambda: pub.publish(msg))


def main(argv=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=argv)
    node = rclpy.create_node('operator_heartbeat')
    bind(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # launch's SIGINT/SIGTERM shuts the context down
        pass
    finally:
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
