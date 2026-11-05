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

import os
import socket
import sys

from mhseals_nav.link_watchdog import short

TOPIC = '/operator/heartbeat'
PERIOD_S = 0.5      # link_watchdog holds after max_age_s (3 s) without a beat


def bind(node, period=PERIOD_S):
    from std_msgs.msg import String
    pub = node.create_publisher(String, TOPIC, 1)
    msg = String(data=short(os.environ.get('ASTRO_HOSTNAME') or socket.gethostname()))
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
