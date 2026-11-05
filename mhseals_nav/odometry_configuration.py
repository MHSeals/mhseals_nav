"""Pure, startup-only odometry layers; identities describe inputs, never live state."""
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType


from . import layers
from .startup_rules import Rule

ALIASES = ('robot_state_publish_frequency', 'mavros_velocity_source',
           'mavros_velocity_stddev', 'mavros_velocity_max_age',
           'mavros_velocity_min_stddev', 'mavros_velocity_no_fix_stddev',
           'mavros_velocity_min_fix_type', 'mavros_velocity_max_acc', 'ekf_delay')
LAYER_ARGUMENTS = ('odometry_deployment_file', 'odometry_startup_file')
KEYS = frozenset(ALIASES[:-1]) | {'ekf_delay_standalone', 'ekf_delay_included'}
ADAPTER_PARAMETERS = {
    'velocity_source': 'mavros_velocity_source',
    'velocity_stddev': 'mavros_velocity_stddev',
    'gps_velocity_max_age': 'mavros_velocity_max_age',
    'gps_velocity_min_stddev': 'mavros_velocity_min_stddev',
    'velocity_stddev_nofix': 'mavros_velocity_no_fix_stddev',
    'gps_velocity_min_fix_type': 'mavros_velocity_min_fix_type',
    'gps_velocity_max_acc': 'mavros_velocity_max_acc',
}
FIX_TYPE = 'mavros_velocity_min_fix_type'  # a receiver fix_type: an integer 0-6
INCLUSIVE = frozenset({FIX_TYPE, 'mavros_velocity_max_acc'})


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
    """Return the owned schema rule."""
    if key not in KEYS:
        raise ValueError('unknown odometry startup field: ' + key)
    if key == FIX_TYPE:
        return Rule('integer', minimum=0, maximum=6)
    return (Rule('string', choices=('gps', 'fcu')) if key == 'mavros_velocity_source' else
            Rule('number', minimum=0,
                 exclusive=not (key.startswith('ekf_delay_') or key in INCLUSIVE)))


def validate(values):
    """Validate every supplied startup field before precedence."""
    if not isinstance(values, dict) or set(values) - KEYS:
        raise ValueError('unsupported odometry startup mapping')
    for key, value in values.items():
        field_rule(key).check(value)


def validate_package(layer, label):
    """Validate a layer; the package one must state every field."""
    validate(layer)
    if label == 'package' and set(layer) != KEYS:
        raise ValueError('package odometry startup mapping must be complete')


def resolve_odometry(package_file=None, deployment_file='', operator_file='',
                     aliases=None, context='standalone'):
    """Resolve package < deployment < operator < explicitly supplied aliases."""
    if context not in ('standalone', 'included'):
        raise ValueError('EKF startup context must be standalone or included')
    if package_file is None:
        from ament_index_python.packages import get_package_share_directory
        package_file = (Path(get_package_share_directory('mhseals_nav')) /
                        'config/odometry_startup.yaml')
    values, inputs = layers.resolve(package_file, deployment_file, operator_file,
                                    validate=validate_package)
    for key, raw in (aliases or {}).items():
        if key not in ALIASES:
            raise ValueError(f'unknown odometry alias: {key}')
        if raw == '' or raw is None:
            continue
        target = 'ekf_delay_' + context if key == 'ekf_delay' else key
        convert = int if key == FIX_TYPE else float
        value = raw if key == 'mavros_velocity_source' else (
            convert(raw) if isinstance(raw, str) else raw)
        validate({target: value})
        values[target] = value
    validate(values)
    values = {key: value if key in ('mavros_velocity_source', FIX_TYPE) else float(value)
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
