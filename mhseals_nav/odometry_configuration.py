"""Pure, startup-only odometry layers; identities describe inputs, never live state."""
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
from types import MappingProxyType

import yaml

from .navigation_config import StrictOverlayLoader
from .startup_rules import Rule

ALIASES = ('robot_state_publish_frequency', 'mavros_velocity_source',
           'mavros_velocity_stddev', 'mavros_velocity_max_age',
           'mavros_velocity_min_stddev', 'mavros_velocity_no_fix_stddev', 'ekf_delay')
LAYER_ARGUMENTS = ('odometry_deployment_file', 'odometry_startup_file')
KEYS = frozenset(ALIASES[:-1]) | {'ekf_delay_standalone', 'ekf_delay_included'}
ADAPTER_PARAMETERS = {
    'velocity_source': 'mavros_velocity_source',
    'velocity_stddev': 'mavros_velocity_stddev',
    'gps_velocity_max_age': 'mavros_velocity_max_age',
    'gps_velocity_min_stddev': 'mavros_velocity_min_stddev',
    'velocity_stddev_nofix': 'mavros_velocity_no_fix_stddev',
}


@dataclass(frozen=True)
class OdometryConfiguration:
    values: object
    inputs: tuple
    context: str

    @property
    def delay(self):
        return self.values['ekf_delay_' + self.context]

    @property
    def adapter_parameters(self):
        return {target: self.values[key] for target, key in ADAPTER_PARAMETERS.items()}


def field_rule(key):
    """The owned schema rule, shared by validation and catalog presentation."""
    if key not in KEYS:
        raise ValueError('unknown odometry startup field: ' + key)
    return (Rule('string', choices=('gps', 'fcu')) if key == 'mavros_velocity_source' else
            Rule('number', minimum=0, exclusive=not key.startswith('ekf_delay_')))


def validate(values):
    """Validate every supplied startup field before precedence."""
    if not isinstance(values, dict) or set(values) - KEYS:
        raise ValueError('unsupported odometry startup mapping')
    for key, value in values.items():
        field_rule(key).check(value)


def resolve_odometry(package_file=None, deployment_file='', operator_file='',
                     aliases=None, context='standalone'):
    """Resolve package < deployment < operator < explicitly supplied aliases."""
    if context not in ('standalone', 'included'):
        raise ValueError('EKF startup context must be standalone or included')
    if package_file is None:
        from ament_index_python.packages import get_package_share_directory
        package_file = (Path(get_package_share_directory('mhseals_nav')) /
                        'config/odometry_startup.yaml')
    values, inputs = {}, []
    for label, selected in (('package', package_file), ('deployment', deployment_file),
                            ('operator', operator_file)):
        if not selected:
            continue
        path = Path(selected).expanduser().resolve(strict=True)
        raw = path.read_bytes()
        layer = yaml.load(raw, Loader=StrictOverlayLoader)
        validate(layer)
        if label == 'package' and set(layer) != KEYS:
            raise ValueError('package odometry startup mapping must be complete')
        values.update(layer)
        inputs.append(MappingProxyType({
            'path': str(path), 'sha256': sha256(raw).hexdigest(), 'layer': label}))
    for key, raw in (aliases or {}).items():
        if key not in ALIASES:
            raise ValueError(f'unknown odometry alias: {key}')
        if raw == '' or raw is None:
            continue
        target = 'ekf_delay_' + context if key == 'ekf_delay' else key
        value = raw if key == 'mavros_velocity_source' else (
            float(raw) if isinstance(raw, str) else raw)
        validate({target: value})
        values[target] = value
    validate(values)
    values = {key: value if key == 'mavros_velocity_source' else float(value)
              for key, value in values.items()}
    return OdometryConfiguration(MappingProxyType(values), tuple(inputs), context)


def from_launch(context):
    """Thin launch adapter; keep the resolver independent of ROS imports."""
    from launch.substitutions import LaunchConfiguration

    def get(name):
        return LaunchConfiguration(name).perform(context)
    return resolve_odometry(deployment_file=get('odometry_deployment_file'),
                            operator_file=get('odometry_startup_file'),
                            aliases={key: get(key) for key in ALIASES},
                            context=get('ekf_startup_context'))
