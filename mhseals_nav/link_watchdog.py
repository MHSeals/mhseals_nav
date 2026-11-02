"""Hold the boat through command_mux when the operator's heartbeat goes stale.

The FCU's GCS failsafe is off and nothing on the boat drives through it, so without this a
lost operator link leaves Nav2 and the tasks driving. The operator host publishes its name
on heartbeat_topic at about 2 Hz. Only beats naming one of operator_hosts count, and never
one naming the boat itself: a TUI on the boat over SSH is not the operator. The name
``station`` in operator_hosts stands for whichever computer last took the station
(``hw.sh station take`` writes its name to station_file on every boat host); with no such
file, any host but the boat counts. Enforced once a heartbeat has arrived this session, or
from the start when require_operator is set; never with no operator ever seen otherwise.
"""

from pathlib import Path
import socket

from mhseals_nav.hold_watchdog import fresh, HoldGate, node_class as watchdog_class, run
import yaml

STATION = 'station'   # the operator_hosts entry resolved from station_file


def short(host):
    """'Laptop-A.local' -> 'laptop-a'."""
    return str(host).strip().lower().split('.')[0]


class Station:
    """The operator named in the station file, re-read whenever its mtime changes."""

    def __init__(self, path, log):
        self.path = Path(path).expanduser()
        self.log = log            # log(message, problem); the node swaps in its logger
        self.stamp = 'unread'
        self.operator = None

    def current(self):
        """Return the operator's short name, or None (no usable file: accept any host)."""
        try:
            info = self.path.stat()
            # Size too: a write right after the truncate can share the coarse mtime tick.
            stamp = (info.st_mtime_ns, info.st_size)
        except OSError:
            stamp = None
        if stamp != self.stamp:
            self.stamp = stamp
            self.operator = self.read() if stamp is not None else None
            if stamp is None:
                self.log(f'no station file at {self.path}: accepting a heartbeat from any '
                         'host but this one until a station is taken', True)
        return self.operator

    def read(self):
        try:
            name = short((yaml.safe_load(self.path.read_text()) or {}).get('operator') or '')
        except (OSError, ValueError, yaml.YAMLError, AttributeError) as err:  # bad UTF-8 too
            name, why = '', f'unreadable ({err})'
        else:
            why = 'names no operator'
        if not name:
            self.log(f'{self.path} {why}: accepting a heartbeat from any host but this one',
                     True)
            return None
        self.log(f'operator station: {name}', False)
        return name


class LinkWatchdog(HoldGate):
    def __init__(self, cfg, own_host=None, log=None):
        super().__init__(cfg)
        self.beat_at = None
        self.own = short(socket.gethostname() if own_host is None else own_host)
        names = {short(h) for h in cfg['operator_hosts']}
        self.literal = names - {STATION, self.own, ''}
        self.station = None
        if STATION in names:
            self.station = Station(cfg['station_file'], log or (lambda _msg, _bad: None))

    def hosts(self):
        """The host names that count, or None for any host but the boat itself."""
        if self.station is None:
            return self.literal
        operator = self.station.current()
        return None if operator is None else self.literal | ({operator} - {self.own})

    def heartbeat(self, now, host):
        """Count a beat named after an operator host; return whether it counted."""
        name, hosts = short(host), self.hosts()
        if name in ('', self.own) or (hosts is not None and name not in hosts):
            return False
        self.beat_at = now
        return True

    def reason(self, now):
        if self.beat_at is None:
            return 'no operator heartbeat yet' if self.cfg['require_operator'] else ''
        if not fresh(self.beat_at, now, self.cfg['max_age']):
            return f"operator heartbeat older than {self.cfg['max_age']} s"
        return ''


def node_class():
    import time
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    def subscribe(node):
        ignored = set()

        def log(message, problem):
            if problem:
                node.get_logger().warning(message)
            else:
                node.get_logger().info(message)

        if node.logic.station is not None:
            node.logic.station.log = log

        def on_beat(msg):
            first = node.logic.beat_at is None
            if node.logic.heartbeat(time.monotonic(), msg.data):
                if first:
                    node.get_logger().info(f'operator heartbeat from {msg.data}')
            elif msg.data not in ignored and len(ignored) < 16:  # bounded
                ignored.add(msg.data)
                node.get_logger().warning(
                    f'ignoring heartbeat from {msg.data!r}: not one of '
                    f'{sorted(node.logic.hosts() or ())} (and never this host)')

        node.create_subscription(String, node.cfg['heartbeat_topic'], on_beat,
                                 qos_profile_sensor_data)

    return watchdog_class('link_watchdog', LinkWatchdog, subscribe)


def main(args=None):
    run(node_class, args)


if __name__ == '__main__':
    main()
