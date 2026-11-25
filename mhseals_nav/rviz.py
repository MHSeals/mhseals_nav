"""rviz [CONFIG]: open RViz with rviz2/<CONFIG>.rviz (nav by default) and the workspace theme.

The theme is $ROS_WS/config/dark.qss when it exists. Runs on this machine: RViz is a window for the
person at it, so it is never forwarded to a robot computer.
"""

import os
from pathlib import Path
import sys

from mhseals_nav.layers import package_path


def configs():
    return sorted(path.stem for path in package_path('rviz2').glob('*.rviz'))


def argv(name, env=None):
    """The rviz2 command line for config NAME."""
    env = os.environ if env is None else env
    path = package_path('rviz2', f'{name}.rviz')
    if not path.is_file():
        raise SystemExit(f'rviz: no config {name!r}; have: {", ".join(configs())}')
    out = ['rviz2', '-d', str(path)]
    theme = Path(env.get('ROS_WS', '.')) / 'config' / 'dark.qss'
    return out + (['--stylesheet', str(theme)] if theme.is_file() else [])


def main(args=None):
    args = sys.argv[1:] if args is None else args
    if args[:1] == ['--list']:
        print('\n'.join(configs()))
        return 0
    name = args[0] if args and not args[0].startswith('-') else 'nav'
    rest = args[1:] if args and not args[0].startswith('-') else args
    command = argv(name) + rest
    os.execvp(command[0], command)


if __name__ == '__main__':
    sys.exit(main())
