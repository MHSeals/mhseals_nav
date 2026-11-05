"""The Python interface other packages import (README "Python interface"): a rename fails here.

mhseals_testing's harness, fastsim adapter and fusion tools reuse these pure cores. Refresh the
list from an AST scan of its imports and attribute uses when it starts using another name.
"""

import importlib
import importlib.util

import pytest

PINNED = {
    'mhseals_nav': ('boat_network', 'lidar_fusion', 'navigation_config', 'startup_projection'),
    'mhseals_nav.boat_network': ('leases', 'path', 'values'),
    'mhseals_nav.device_profile': ('load',),
    'mhseals_nav.endpoint_configuration': ('LAYER_ARGUMENTS', 'UNSET_GCS', 'boolean',
                                           'from_launch', 'resolve_endpoints'),
    'mhseals_nav.fence': ('closest', 'inside', 'polygon', 'problem', 'project'),
    'mhseals_nav.lidar_configuration': ('CORRECTIONS', 'resolve_lidar', 'validate_calibration'),
    'mhseals_nav.lidar_fusion': ('axes_matrix', 'choose_scan', 'cloud_arrays', 'compensate',
                                 'crop_mask', 'delta_knots', 'extrinsic_samples', 'load_yaml',
                                 'matrix_rpy', 'model_rows', 'quat_matrix', 'rpy_matrix',
                                 'small_rotation', 'tf_gap'),
    'mhseals_nav.localization_configuration': ('compile_localization',),
    'mhseals_nav.mavros_configuration': ('ALIASES', 'parameters_from_yaml', 'validate_setup'),
    'mhseals_nav.msg': ('ObservationArray', 'TrackArray'),
    'mhseals_nav.navigation_config': ('derived', 'load_ceilings', 'load_overlay',
                                      'lower_navigation', 'merge_limits', 'mppi_controller',
                                      'persistent_limits_path', 'validate_navigation_base',
                                      'validated_merge_limits', 'within_ceilings'),
    'mhseals_nav.odometry_configuration': ('ALIASES', 'LAYER_ARGUMENTS', 'from_launch'),
    'mhseals_nav.robot_geometry': ('RobotGeometry', 'description_path', 'matrix_quaternion'),
    'mhseals_nav.startup_projection': ('SERVERS', 'ceilings_path', 'compile_navigation',
                                       'effective_limits'),
    'mhseals_nav.tasks': ('params', 'tracks'),
    'mhseals_nav.tasks.host': ('MissionHost',),
    'mhseals_nav.tasks.params': ('config_path', 'load_params', 'overlay', 'timeslot'),
    'mhseals_nav.tasks.tracks': ('Track', 'from_track_array'),
    'mhseals_nav.track_filter': ('MOTION_STATIC', 'SOURCE_PRIOR', 'TrackFilter'),
}
# Built by colcon (rosidl), so only present in a ROS build.
NEEDS_ROS = {'mhseals_nav.msg'}


@pytest.mark.parametrize('module', sorted(PINNED))
def test_pinned_names_exist(module):
    if module in NEEDS_ROS and importlib.util.find_spec('rclpy') is None:
        pytest.skip('generated messages need a ROS build')
    try:
        loaded = importlib.import_module(module)
    except ModuleNotFoundError as error:
        if (error.name or '').split('.')[0] == 'mhseals_nav':
            raise  # a pinned module moved or was renamed
        pytest.skip(f'{module} needs {error.name}')
    # A name is an attribute or a submodule (``from mhseals_nav import boat_network``).
    missing = [name for name in PINNED[module] if not hasattr(loaded, name)
               and importlib.util.find_spec(f'{module}.{name}') is None]
    assert not missing, f'{module} no longer has {missing}: update mhseals_testing and README'
