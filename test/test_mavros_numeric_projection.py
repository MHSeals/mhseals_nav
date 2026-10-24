"""Project accepted policy numbers with the node's declared ROS scalar types."""
from pathlib import Path

from mhseals_nav.mavros_configuration import installed_setup_defaults, POLICY_FIELDS, resolve_setup
import pytest
from rclpy.parameter import Parameter
from test_mavros_inheritance import layer
from test_mavros_policy import inert_main
from test_mavros_startup import factory
import yaml

ROOT = Path(__file__).parents[1]
DOUBLES = [key for key, rule in POLICY_FIELDS.items() if rule in ('positive', 'nonnegative')]


@pytest.mark.parametrize('key', DOUBLES)
def test_integer_policy_launch_and_worker_project_as_double(tmp_path, monkeypatch, key):
    """Integer-spelled partial YAML and rate aliases retain DOUBLE compatibility."""
    selected = layer(tmp_path, {key: 1})
    projected = resolve_setup(selected, {'mavros_imu_rate': '12'})
    node = factory()['create_mavros_setup_node']({
        'mavros_setup_params_file': str(selected), 'mavros_odom_rate': '',
        'mavros_imu_rate': '12', 'mavros_gps_rate': '', 'mavros_rc_rate': ''})[0]
    assert type(node.parameters[1][key]) is float
    assert Parameter(key, value=node.parameters[1][key]).type_ == Parameter.Type.DOUBLE
    assert Parameter(key, value=installed_setup_defaults()[key]).type_ == Parameter.Type.DOUBLE
    seen = inert_main(monkeypatch, projected)
    cells = dict(zip(seen.worker.__code__.co_freevars, seen.worker.__closure__))
    assert type(cells['policy'].cell_contents[key]) is float
    assert '30:12' in projected['message_rates']
    for integer in ('retry_attempts', 'rc_min_channels', 'fcu_sysid'):
        assert type(projected[integer]) is int


def test_integer_package_defaults_declare_doubles(tmp_path, monkeypatch):
    """Direct node declaration defaults normalize package integer spellings too."""
    from ament_index_python import packages
    data = yaml.safe_load((ROOT / 'config/mavros_setup.yaml').read_text())
    for key in DOUBLES:
        data['mavros_setup']['ros__parameters'][key] = 1
    config = tmp_path / 'config'
    config.mkdir()
    (config / 'mavros_setup.yaml').write_text(yaml.safe_dump(data))
    monkeypatch.setattr(packages, 'get_package_share_directory', lambda name: str(tmp_path))
    defaults = installed_setup_defaults()
    seen = inert_main(monkeypatch)
    for key in DOUBLES:
        assert type(defaults[key]) is float
        assert type(seen.params[key]) is float
        assert Parameter(key, value=seen.params[key]).type_ == Parameter.Type.DOUBLE
