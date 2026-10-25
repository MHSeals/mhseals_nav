"""Device profiles: what one computer runs, as parts built from this package's launch files.

``config/devices/<name>.yaml`` is the only place a responsibility is assigned to a computer
(device-topology.md); mhseals_testing derives the part -> host map from these files. Layers,
lowest first: the package profile (and its ``extends``), ``$ROS_WS/config/devices/<name>.yaml``
(deployment), ``~/.config/mhseals/devices/<name>.yaml`` (operator), then the session's
``overrides``. The thruster driver and command_mux are never parts: they start with arming.
"""
import json
import os
from pathlib import Path

from .navigation_config import load_strict

KEYS = {'extends', 'requires', 'args', 'parts'}
OVERLAY_KEYS = KEYS - {'extends'}
PART_KEYS = {'launch', 'run', 'args', 'nodes', 'enabled', 'kind', 'with'}
KINDS = ('sensing', 'commander')
COMMANDER_LAUNCHES = {'navigation.launch.py', 'task.launch.py', 'robot.launch.py'}
DRIVERS = {'thruster_pwm_node', 'command_mux'}


def share():
    """Return the installed share directory, else the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('mhseals_nav'))
    except (ImportError, LookupError):
        return Path(__file__).resolve().parents[1]


def available():
    return sorted(path.stem for path in (share() / 'config' / 'devices').glob('*.yaml'))


def _names(value, where):
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ValueError(f'{where}: list of names')
    return value


def _arguments(value, where):
    if not isinstance(value, dict) or not all(
            isinstance(item, (str, int, float, bool)) for item in value.values()):
        raise ValueError(f'{where}: mapping of launch argument to a string, number or boolean')
    return value


def validate_layer(layer, where, keys=KEYS):
    """Refuse unknown keys and wrong types in one layer; returns it ({} for an empty file)."""
    if layer is None:
        return {}
    if not isinstance(layer, dict) or not set(layer) <= keys:
        raise ValueError(f'{where}: mapping with keys from {sorted(keys)}')
    _names(layer.get('extends', []), f'{where}.extends')
    _names(layer.get('requires', []), f'{where}.requires')
    _arguments(layer.get('args', {}), f'{where}.args')
    parts = layer.get('parts', {})
    if not isinstance(parts, dict):
        raise ValueError(f'{where}.parts: mapping of part name to settings')
    for name, spec in parts.items():
        here = f'{where}.parts.{name}'
        if not isinstance(spec, dict) or not set(spec) <= PART_KEYS:
            raise ValueError(f'{here}: mapping with keys from {sorted(PART_KEYS)}')
        # A bare file name only: a path (./navigation.launch.py) would dodge the commander check.
        launch = spec.get('launch')
        if 'launch' in spec and not (isinstance(launch, str) and Path(launch).name == launch
                                     and launch.endswith('.launch.py')):
            raise ValueError(
                f'{here}.launch: a launch file name in this package (<name>.launch.py)')
        if 'run' in spec and len(_names(spec['run'], f'{here}.run')) < 2:
            raise ValueError(f'{here}.run: [package, executable, arguments...]')
        _arguments(spec.get('args', {}), f'{here}.args')
        _names(spec.get('nodes', []), f'{here}.nodes')
        _names(spec.get('with', []), f'{here}.with')
        if not isinstance(spec.get('enabled', True), bool):
            raise ValueError(f'{here}.enabled: true or false')
        if spec.get('kind', 'sensing') not in KINDS:
            raise ValueError(f'{here}.kind: one of {list(KINDS)}')
    return layer


def merge(base, layer):
    """Overlay ``layer``: requires replaces, args per key, parts per part and per field."""
    out = {'requires': list(layer.get('requires', base.get('requires', []))),
           'args': {**base.get('args', {}), **layer.get('args', {})},
           'parts': {name: dict(spec) for name, spec in base.get('parts', {}).items()}}
    for name, spec in layer.get('parts', {}).items():
        part = out['parts'].setdefault(name, {})
        if 'args' in spec:
            spec = dict(spec, args={**part.get('args', {}), **spec['args']})
        if 'launch' in spec:
            part.pop('run', None)
        if 'run' in spec:
            part.pop('launch', None)
        part.update(spec)
    return out


def _read(path, keys):
    path = Path(path).expanduser()
    return validate_layer(load_strict(path.read_bytes(), path), str(path), keys)


def _overlay_paths(name, deployment_file, startup_file):
    """Return the deployment and operator layers of ``name`` that exist."""
    workspace = os.environ.get('ROS_WS')
    config = Path(os.environ.get('XDG_CONFIG_HOME') or Path.home() / '.config')
    found = []
    for given, default in ((deployment_file, workspace and Path(workspace) / 'config' / 'devices'),
                           (startup_file, config / 'mhseals' / 'devices')):
        if given:
            found.append(Path(given).expanduser().resolve(strict=True))
        elif default and (Path(default) / f'{name}.yaml').is_file():
            found.append(Path(default) / f'{name}.yaml')
    return found


def _package(name, chain):
    if name in chain:
        raise ValueError(f'device profile extends itself: {" -> ".join((*chain, name))}')
    path = share() / 'config' / 'devices' / f'{name}.yaml'
    if not path.is_file():
        raise ValueError(f'no device profile {name!r}; have {", ".join(available())}')
    layer = _read(path, KEYS)
    merged = {}
    for parent in layer.get('extends', []):
        merged = merge(merged, _package(parent, (*chain, name)))
    return merge(merged, layer)


def load(name, deployment_file='', startup_file='', overrides=None):
    """Return the merged, validated profile ``name``: {'requires', 'args', 'parts'}.

    ``overrides`` is the session layer, a mapping or its JSON text (the launch argument).
    """
    merged = _package(name, ())
    for path in _overlay_paths(name, deployment_file, startup_file):
        merged = merge(merged, _read(path, OVERLAY_KEYS))
    if isinstance(overrides, str):
        try:
            overrides = json.loads(overrides) if overrides.strip() else None
        except ValueError as error:
            raise ValueError(f'overrides: not JSON ({error})') from None
    if overrides:
        merged = merge(merged, validate_layer(overrides, 'overrides', OVERLAY_KEYS))
    check(merged, name)
    return merged


def check(profile, name):
    """Each part runs one launch file of this package or one executable; drivers never.

    A part's ``with`` names other sensing parts of the profile that start beside it.
    """
    launches = share() / 'launch'
    for part, spec in profile['parts'].items():
        here = f'{name}.parts.{part}'
        for other in spec.get('with', ()):
            if other == part or other not in profile['parts']:
                raise ValueError(f'{here}.with: {other!r} is not another part of {name}')
            if profile['parts'][other].get('kind', 'sensing') != 'sensing':
                raise ValueError(f'{here}.with: {other!r} can command motion; start it on its own')
            if profile['parts'][other].get('with'):
                raise ValueError(f'{here}.with: {other!r} brings parts of its own')
        if ('launch' in spec) == ('run' in spec):
            raise ValueError(f'{here}: exactly one of launch or run')
        if 'run' in spec:
            if DRIVERS & set(spec['run']):
                raise ValueError(f'{here}: the thruster driver and command_mux start with '
                                 'arming in the TUI, never as a device part')
            if spec.get('args'):
                raise ValueError(f'{here}.args: give a run part its arguments in run')
            continue
        if not (launches / spec['launch']).is_file():
            raise ValueError(f'{here}.launch: no {spec["launch"]} in mhseals_nav/launch')
        if spec['launch'] in COMMANDER_LAUNCHES and spec.get('kind', 'sensing') != 'commander':
            raise ValueError(f'{here}: {spec["launch"]} can command motion; set kind: commander')


def select(profile, only=''):
    """[(part, spec)] to start: every enabled part, or the named ones (each must be enabled).

    A named part brings its enabled ``with`` parts (the boat's watchdogs come up with odometry).
    """
    names = [item.strip() for item in only.split(',') if item.strip()]
    if not names:
        return [(name, spec) for name, spec in profile['parts'].items()
                if spec.get('enabled', True)]
    for name in names:
        if name not in profile['parts']:
            raise ValueError(f'only: no part {name!r}; have {", ".join(profile["parts"])}')
        if not profile['parts'][name].get('enabled', True):
            raise ValueError(f'only: part {name!r} is disabled in this profile')
    brought = [other for name in names for other in profile['parts'][name].get('with', ())
               if profile['parts'][other].get('enabled', True)]
    return [(name, profile['parts'][name]) for name in dict.fromkeys([*names, *brought])]


def launch_arguments(profile, spec):
    """Return the include's arguments: the profile's ``args`` over the part's, as strings."""
    merged = {**profile.get('args', {}), **spec.get('args', {})}
    return {key: (str(value).lower() if isinstance(value, bool) else str(value))
            for key, value in merged.items()}
