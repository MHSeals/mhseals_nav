"""Pure startup preview selection; source identities never imply delivered frames.

The package layer is config/camera.yaml (``tune.preview_rate``); deployment and operator layers
are overlays of it (``tune: {preview_rate: ...}``). An old ``preview_hz: ...`` layer reads for
one release, with a warning.
"""
from dataclasses import dataclass
from hashlib import sha256
import logging
import math
from pathlib import Path
from types import MappingProxyType


from .navigation_config import load_strict


log = logging.getLogger(__name__)


@dataclass(frozen=True)
class CameraConfiguration:
    preview_rate: float
    inputs: tuple
    override: object


def rate(value):
    """Accept finite positive numbers, refuse booleans and normalize ROS doubles."""
    try:
        valid = type(value) in (int, float) and math.isfinite(value) and value > 0
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError('preview_rate must be a finite positive number')
    return float(value)


def preview(data, path):
    """A layer's preview rate: tune.preview_rate (camera.yaml's shape) or an old preview_hz."""
    if type(data) is dict and set(data) == {'preview_hz'}:
        log.warning('%s: preview_hz is deprecated, write tune: {preview_rate: ...}', path)
        return rate(data['preview_hz'])
    if (type(data) is not dict or not set(data) <= {'tune', 'backend'} or
            type(data.get('tune')) is not dict or set(data['tune']) != {'preview_rate'}):
        raise ValueError('camera startup must hold tune.preview_rate only (camera.yaml: and backend)')
    return rate(data['tune']['preview_rate'])


def resolve_camera(package_file=None, deployment_file='', operator_file='', preview_rate=''):
    """Resolve package < explicit deployment < operator < nonempty launch alias."""
    if package_file is None:
        from ament_index_python.packages import get_package_share_directory
        package_file = (Path(get_package_share_directory('mhseals_nav')) /
                        'config/camera.yaml')
    if not package_file:
        raise ValueError('camera package authority must select a file')
    inputs = []
    for layer, selected in (('package', package_file), ('deployment', deployment_file),
                            ('operator', operator_file)):
        if not selected:
            continue
        path = Path(selected).expanduser().resolve(strict=True)
        raw = path.read_bytes()
        resolved = preview(load_strict(raw, path), path)
        inputs.append(MappingProxyType({
            'path': str(path), 'sha256': sha256(raw).hexdigest(), 'layer': layer}))
    if preview_rate != '':
        value = preview_rate
        if type(value) is str:
            try:
                value = float(value)
            except ValueError as error:
                raise ValueError('preview_rate alias must be a finite positive number') from error
        resolved = rate(value)
    return CameraConfiguration(resolved, tuple(inputs), preview_rate)


def from_launch(context):
    """Keep launch imports outside the pure selection boundary."""
    from launch.substitutions import LaunchConfiguration
    return resolve_camera(
        deployment_file=LaunchConfiguration('camera_deployment_file').perform(context),
        operator_file=LaunchConfiguration('camera_startup_file').perform(context),
        preview_rate=LaunchConfiguration('preview_rate').perform(context))
