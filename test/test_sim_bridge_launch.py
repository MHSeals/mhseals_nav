"""sim_bridge.launch.py: the Unity endpoint and the optional lidar smear, in the launch harness."""
from pathlib import Path

from launch_harness import executables, harness, named  # noqa: F401
from mhseals_nav.layers import load_strict
import pytest
import yaml

ROOT = Path(__file__).parents[1]


@pytest.fixture
def bridge(harness):  # noqa: F811
    module = harness.load('sim_bridge.launch.py')
    module.smear_enabled = lambda: False   # the packaged default, whatever config/lidar.yaml says
    return module


@pytest.mark.parametrize('data,expected', [
    ({'unity': {'ip': '0.0.0.0'}}, {'ROS_IP': '0.0.0.0', 'ROS_TCP_PORT': 6767}),
    ({'unity': {'port': 7999}}, {'ROS_IP': '127.0.0.1', 'ROS_TCP_PORT': 7999}),
])
def test_unity_yaml_reaches_the_endpoint_typed(
        harness, bridge, tmp_path, data, expected):  # noqa: F811
    operator = tmp_path / 'endpoint.yaml'
    operator.write_text(yaml.safe_dump(data))
    (endpoint,) = harness.run('sim_bridge.launch.py', endpoint_startup_file=str(operator))
    assert endpoint['executable'] == 'default_server_endpoint'
    assert endpoint['parameters'] == [expected]
    assert type(endpoint['parameters'][0]['ROS_TCP_PORT']) is int


def test_the_port_argument_wins_over_the_yaml(harness, bridge):  # noqa: F811
    """crane_ml passes each instance's endpoint port as ros_port."""
    (endpoint,) = harness.run('sim_bridge.launch.py', ros_port='10001')
    assert endpoint['parameters'][0]['ROS_TCP_PORT'] == 10001


@pytest.mark.parametrize('text', ['unity: {port: false}\n', "fcu_url: 'tcp://host:１２３４'\n"])
def test_an_invalid_endpoint_layer_fails_before_any_node(
        harness, bridge, tmp_path, text):  # noqa: F811
    operator = tmp_path / 'endpoint.yaml'
    operator.write_text(text)
    with pytest.raises(ValueError):
        harness.run('sim_bridge.launch.py', endpoint_startup_file=str(operator))
    assert harness.built == []


@pytest.mark.parametrize('enabled', [True, False])
def test_the_smear_moves_the_simulated_cloud_and_spreads_it(
        harness, bridge, enabled):  # noqa: F811
    bridge.smear_enabled = lambda: enabled
    nodes = harness.run('sim_bridge.launch.py')
    endpoint = named(nodes, 'default_server_endpoint')
    if enabled:
        assert endpoint['remappings'] == [('/points', '/points/instant')]
        assert executables(nodes) == ['default_server_endpoint', 'lidar_smear']
    else:
        assert endpoint['remappings'] == []
        assert executables(nodes) == ['default_server_endpoint']


def test_the_smear_switch_is_config_lidar_yaml(harness):  # noqa: F811
    path = ROOT / 'config/lidar.yaml'
    packaged = bool(load_strict(path.read_text(), path)['tune']['smear'])
    assert harness.load('sim_bridge.launch.py').smear_enabled() is packaged
