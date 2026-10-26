"""The boat LAN facts from the vessel's network env, the only address source.

The file is ``$ASTRO_NETWORK_ENV`` (astro_dock exports it from the vessel's profile; ``hw.sh env``
prints it), else the legacy ``$ROS_WS/config/boat-network.env``.
One parser for the shell env file (``KEY=value``, comments and quotes as the shell reads them):
``value('LIDAR_IP')`` and ``leases()`` (STATIC_LEASES as hostname -> address). A missing file
reads as empty, so callers decide whether a missing value is an error. ROS-free.
"""
import os
from pathlib import Path
import shlex
import sys

_warned = set()


def path():
    """``$ASTRO_NETWORK_ENV``, else ``$ROS_WS/config/boat-network.env`` (None without either).

    A set ASTRO_NETWORK_ENV wins even when its file is missing (warned once on stderr), so a
    wrong profile never silently falls back to another network.
    """
    explicit = os.environ.get('ASTRO_NETWORK_ENV')
    if explicit:
        chosen = Path(explicit).expanduser()
        if not chosen.is_file() and str(chosen) not in _warned:
            _warned.add(str(chosen))
            print(f'mhseals_nav: ASTRO_NETWORK_ENV={chosen} is missing; '
                  'no boat network addresses (hw.sh env prints the right path)', file=sys.stderr)
        return chosen
    workspace = os.environ.get('ROS_WS')
    return Path(workspace) / 'config' / 'boat-network.env' if workspace else None


def values(source=None):
    """Return ``{KEY: value}`` from the env file ({} if missing); unparsable lines are left out."""
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
