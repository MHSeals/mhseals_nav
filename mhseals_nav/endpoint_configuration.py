"""Startup endpoint selection; discovery observations never prove connectivity."""
from dataclasses import dataclass
from hashlib import sha256
from ipaddress import IPv4Address
from pathlib import Path
import re
from types import MappingProxyType

import yaml

from .navigation_config import StrictOverlayLoader

CONTEXTS = ('standalone', 'full_robot', 'tui')
LAYER_ARGUMENTS = ('endpoint_deployment_file', 'endpoint_startup_file')
ALIASES = ('fcu_url', 'gcs_url', 'ros_ip', 'ros_port')
UNSET_GCS = '__inherit__'
STRATEGIES = ('odom_cube_if00', 'tui_stable_hints')


def freeze(value):
    if isinstance(value, dict):
        return MappingProxyType({key: freeze(item) for key, item in value.items()})
    return value


def discover_device(strategy, dev_root=Path('/dev')):
    """Execute the two fixed historical selection policies, without opening devices."""
    root = Path(dev_root)
    if strategy == 'odom_cube_if00':
        candidates = sorted(root.glob('serial/by-id/usb-CubePilot_CubeOrange+_*-if00'))
        return candidates[0] if candidates else None
    if strategy != 'tui_stable_hints':
        raise ValueError('unsupported serial discovery strategy')
    # Equivalent to hardware.fcu.detect_fcu_device; no hardware runtime dependency.
    by_id = root / 'serial' / 'by-id'
    represented = set()
    if by_id.is_dir():
        links = sorted(path for path in by_id.iterdir() if path.is_symlink() or path.is_file())
        hinted = [path for path in links if any(hint in path.name.lower()
                  for hint in ('ardupilot', 'pixhawk', 'px4', 'cube', 'holybro'))]
        if hinted:
            return hinted[0]
        represented = {path.resolve() for path in links}
    candidates = sorted((*root.glob('ttyACM*'), *root.glob('ttyUSB*')))
    candidates = [path for path in candidates if path.resolve() not in represented]
    return candidates[0] if candidates else None


def boolean(value):
    """Match the established strict launch vocabulary without truthiness coercion."""
    if type(value) is bool:
        return value
    if not isinstance(value, str) or value.strip().lower() not in ('true', 'false', '1', '0'):
        raise ValueError('endpoint sim/fcu selector must be a boolean')
    return value.strip().lower() in ('true', '1')


@dataclass(frozen=True)
class EndpointConfiguration:
    values: object
    inputs: tuple
    overrides: object
    context: str

    @property
    def unity_parameters(self):
        return {'ROS_IP': self.values['unity']['ip'], 'ROS_TCP_PORT': self.values['unity']['port']}

    def mavros_parameters(self, sim, dev_root=Path('/dev')):
        """Return parameters and separately labeled local discovery observation."""
        sim = boolean(sim)
        selected = self.values['contexts'][self.context]
        explicit = self.values['fcu_url']
        observation = None
        if explicit:
            url = explicit
        elif sim:
            url = self.values['simulation_fcu_url']
        else:
            device = discover_device(selected['discovery'], dev_root)
            path = str(device) if device is not None else self.values['serial_fallback']
            url = f"serial://{path}:{selected['baud']}"
            observation = freeze({'kind': 'local-path-observation', 'path': path,
                                  'discovery': selected['discovery'], 'fallback': device is None})
        return {'fcu_url': url, 'gcs_url': selected['gcs_url'], 'use_sim_time': sim}, observation


def validate_url(value, gcs=False):
    """Check bounded libmavconn2.16 syntax without resolving hosts or opening links."""
    if (type(value) is not str or not value or
            any(ord(char) < 32 or ord(char) == 127 for char in value)):
        raise ValueError('endpoint URL must be a nonempty string without control characters')
    if '://' not in value:
        if not value.startswith('/'):
            raise ValueError('schemeless serial endpoint must be an absolute path')
        scheme, path = 'serial', value
        host, query = '', ''
    else:
        scheme, rest = value.split('://', 1)
        scheme = scheme.lower()
        host, slash, remainder = rest.partition('/')
        path, mark, query = ('/' + remainder if slash else '').partition('?')
        if scheme not in ('serial', 'serial-hwfc', 'udp', 'udp-b', 'udp-pb', 'tcp', 'tcp-l'):
            raise ValueError('unsupported libmavconn endpoint scheme')
    if scheme.startswith('serial'):
        if not path.startswith('/') or path == '/':
            raise ValueError('serial endpoint requires an absolute device path')
        _host_pair(path, 2147483647, serial=True)
    elif scheme.startswith('udp'):
        if host.count('@') != 1:
            raise ValueError('UDP endpoint requires bind@remote separator')
        for pair in host.split('@'):
            _host_pair(pair, 65535)
    else:
        _host_pair(host, 65535)
    # libmavconn warns/ignores unknown query text, but validates ids= if a comma exists.
    if 'ids=' in query:
        ids = query.split('ids=', 1)[1]
        if ',' in ids:
            for item in ids.split(',', 1):
                if not re.fullmatch(r'[+]?\d+', item) or not 0 <= int(item) <= 255:
                    raise ValueError('endpoint ids must be uint8 values')


def _host_pair(value, maximum, serial=False):
    host, separator, number = value.partition(':')
    if not serial and host and not re.fullmatch(r'[A-Za-z0-9_.-]+', host):
        raise ValueError('endpoint host must be an IPv4 literal or hostname')
    if separator:
        if not re.fullmatch(r'[+]?\d+', number) or not 1 <= int(number) <= maximum:
            raise ValueError('endpoint port/baud outside parser integer representation')


def validate_layer(layer):
    """Validate every supplied leaf independently before precedence can hide it."""
    if type(layer) is not dict or set(layer) - {
            'fcu_url', 'simulation_fcu_url', 'serial_fallback', 'unity', 'contexts'}:
        raise ValueError('unsupported endpoint startup mapping')
    for name, value in layer.items():
        if name in ('fcu_url', 'simulation_fcu_url'):
            if name == 'fcu_url' and value == '':
                continue
            validate_url(value)
        elif name == 'serial_fallback':
            if (type(value) is not str or not value.startswith('/') or
                    any(ord(char) < 32 or ord(char) == 127 for char in value) or ':' in value):
                raise ValueError('serial fallback must be absolute without controls or colon')
        elif name == 'unity':
            if type(value) is not dict or set(value) - {'ip', 'port'}:
                raise ValueError('unsupported Unity endpoint mapping')
            for key, item in value.items():
                if key == 'ip':
                    if type(item) is not str:
                        raise ValueError('Unity IP must be an IPv4 literal string')
                    IPv4Address(item)
                elif type(item) is not int or not 1 <= item <= 65535:
                    raise ValueError('Unity port must be an integer in 1..65535')
        else:
            if type(value) is not dict or set(value) - set(CONTEXTS):
                raise ValueError('unsupported endpoint context')
            for context, settings in value.items():
                if type(settings) is not dict or set(settings) - {'discovery', 'baud', 'gcs_url'}:
                    raise ValueError('unsupported endpoint context mapping')
                for key, item in settings.items():
                    if key == 'discovery' and item not in STRATEGIES:
                        raise ValueError('unsupported serial discovery strategy')
                    if key == 'baud' and (type(item) is not int or not 1 <= item <= 2147483647):
                        raise ValueError('baud must be an integer in 1..INT_MAX')
                    if key == 'gcs_url' and item != '':
                        validate_url(item, gcs=True)


def merge(target, layer):
    for key, value in layer.items():
        if isinstance(value, dict):
            merge(target.setdefault(key, {}), value)
        else:
            target[key] = value


def resolve_endpoints(package_file=None, deployment_file='', operator_file='', aliases=None,
                      context='standalone'):
    """Resolve package < deployment < operator < explicit compatibility aliases."""
    if context not in CONTEXTS:
        raise ValueError('endpoint context must be standalone, full_robot or tui')
    if package_file is None:
        from ament_index_python.packages import get_package_share_directory
        package_file = (Path(get_package_share_directory('mhseals_nav')) /
                        'config/endpoint_startup.yaml')
    if not package_file:
        raise ValueError('package endpoint authority must select a file')
    values, inputs = {}, []
    for label, selected in (('package', package_file), ('deployment', deployment_file),
                            ('operator', operator_file)):
        if not selected:
            continue
        path = Path(selected).expanduser().resolve(strict=True)
        raw = path.read_bytes()
        layer = yaml.load(raw, Loader=StrictOverlayLoader)
        validate_layer(layer)
        merge(values, layer)
        if label == 'package':
            complete = (set(values) == {'fcu_url', 'simulation_fcu_url', 'serial_fallback',
                                        'unity', 'contexts'} and
                        set(values['unity']) == {'ip', 'port'} and
                        set(values['contexts']) == set(CONTEXTS) and
                        all(set(item) == {'discovery', 'baud', 'gcs_url'}
                            for item in values['contexts'].values()))
            if not complete:
                raise ValueError('complete package endpoint authority required')
        inputs.append(freeze({
            'path': str(path), 'sha256': sha256(raw).hexdigest(), 'layer': label}))
    explicit = {}
    for name, value in (aliases or {}).items():
        if name not in ALIASES:
            raise ValueError('unknown endpoint alias: ' + name)
        if value == (UNSET_GCS if name == 'gcs_url' else ''):
            continue
        if name == 'ros_port':
            if type(value) is str:
                try:
                    value = int(value)
                except ValueError as error:
                    raise ValueError('Unity port alias must be an integer') from error
            layer = {'unity': {'port': value}}
        elif name == 'ros_ip':
            layer = {'unity': {'ip': value}}
        elif name == 'gcs_url':
            layer = {'contexts': {context: {'gcs_url': value}}}
        else:
            layer = {'fcu_url': value}
        validate_layer(layer)
        merge(values, layer)
        explicit[name] = value
    validate_layer(values)
    return EndpointConfiguration(freeze(values), tuple(inputs), freeze(explicit), context)


def from_launch(context):
    """Read endpoint arguments without coupling the pure resolver to ROS imports."""
    from launch.substitutions import LaunchConfiguration

    def get(name):
        return LaunchConfiguration(name).perform(context)

    return resolve_endpoints(deployment_file=get('endpoint_deployment_file'),
                             operator_file=get('endpoint_startup_file'),
                             aliases={name: get(name) for name in ALIASES},
                             context=get('endpoint_context'))
