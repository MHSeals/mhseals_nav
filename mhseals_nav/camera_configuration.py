"""Pure startup preview selection; source identities never imply delivered frames."""
from dataclasses import dataclass
from hashlib import sha256
import math
from pathlib import Path
from types import MappingProxyType


from .navigation_config import load_strict


@dataclass(frozen=True)
class CameraConfiguration:
    preview_hz: float
    inputs: tuple
    override: object


def rate(value):
    """Accept finite positive numbers, refuse booleans and normalize ROS doubles."""
    try:
        valid = type(value) in (int, float) and math.isfinite(value) and value > 0
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError('preview_hz must be a finite positive number')
    return float(value)


def resolve_camera(package_file=None, deployment_file='', operator_file='', preview_hz=''):
    """Resolve package < explicit deployment < operator < nonempty launch alias."""
    if package_file is None:
        from ament_index_python.packages import get_package_share_directory
        package_file = (Path(get_package_share_directory('mhseals_nav')) /
                        'config/camera_startup.yaml')
    if not package_file:
        raise ValueError('camera package authority must select a file')
    inputs = []
    for layer, selected in (('package', package_file), ('deployment', deployment_file),
                            ('operator', operator_file)):
        if not selected:
            continue
        path = Path(selected).expanduser().resolve(strict=True)
        raw = path.read_bytes()
        data = load_strict(raw, path)
        if type(data) is not dict or set(data) != {'preview_hz'}:
            raise ValueError('camera startup must contain only preview_hz')
        resolved = rate(data['preview_hz'])
        inputs.append(MappingProxyType({
            'path': str(path), 'sha256': sha256(raw).hexdigest(), 'layer': layer}))
    if preview_hz != '':
        value = preview_hz
        if type(value) is str:
            try:
                value = float(value)
            except ValueError as error:
                raise ValueError('preview_hz alias must be a finite positive number') from error
        resolved = rate(value)
    return CameraConfiguration(resolved, tuple(inputs), preview_hz)


def from_launch(context):
    """Keep launch imports outside the pure selection boundary."""
    from launch.substitutions import LaunchConfiguration
    return resolve_camera(
        deployment_file=LaunchConfiguration('camera_deployment_file').perform(context),
        operator_file=LaunchConfiguration('camera_startup_file').perform(context),
        preview_hz=LaunchConfiguration('preview_hz').perform(context))
