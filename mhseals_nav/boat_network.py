"""The boat LAN facts from astro_dock's ``$ROS_WS/config/boat-network.env``, the only address source.

One parser for the shell env file (``KEY=value``, comments and quotes as the shell reads them):
``value('LIDAR_IP')`` and ``leases()`` (STATIC_LEASES as hostname -> address). A missing file
reads as empty, so callers decide whether a missing value is an error. ROS-free.
"""
import os
from pathlib import Path
import shlex


def path():
    """``$ROS_WS/config/boat-network.env`` (None without ROS_WS)."""
    workspace = os.environ.get('ROS_WS')
    return Path(workspace) / 'config' / 'boat-network.env' if workspace else None


def values(source=None):
    """``{KEY: value}`` from the env file; {} if missing. A line that does not parse is left out."""
    source = source or path()
    try:
        text = Path(source).read_text() if source else ''
    except OSError:
        return {}
    found = {}
    for line in text.splitlines():
        name, equals, value = line.partition('=')
        name = name.strip()
        if not equals or not name.isidentifier():
            continue
        try:
            words = shlex.split(value, comments=True)
        except ValueError:      # an unclosed quote: no value, never a crash
            continue
        found[name] = words[0] if words else ''
    return found


def value(key, source=None):
    """One value, or '' when the file or the key is missing."""
    return values(source).get(key, '')


def leases(source=None):
    """``{hostname: address}`` from STATIC_LEASES (``MAC=hostname=address ...``)."""
    found = (lease.split('=') for lease in value('STATIC_LEASES', source).split())
    return {lease[1]: lease[2] for lease in found if len(lease) == 3}
