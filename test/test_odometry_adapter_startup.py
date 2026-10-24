"""Invoke actual adapter main with inert ROS imports and no graph or hardware."""
import ast
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from mhseals_nav import odometry_configuration

ROOT = Path(__file__).parents[1]


def adapter_main(monkeypatch, declarations, events):
    def module(name, **members):
        value = ModuleType(name)
        value.__dict__.update(members)
        monkeypatch.setitem(sys.modules, name, value)
    class Node:
        def __init__(self, name):
            events.append('node')
        def declare_parameter(self, name, value):
            declarations[name] = value
            return SimpleNamespace(value=value)
        def create_publisher(self, *args):
            return object()
        def create_subscription(self, *args):
            pass
        def destroy_node(self):
            events.append('destroy')
    module('rclpy', init=lambda **kw: events.append('init'),
           spin=lambda node: events.append('spin'), try_shutdown=lambda: None)
    module('rclpy.node', Node=Node)
    module('rclpy.qos', qos_profile_sensor_data=object())
    module('rclpy.executors', ExternalShutdownException=type('Shutdown', (Exception,), {}))
    module('nav_msgs.msg', Odometry=object)
    module('mavros_msgs.msg', GPSRAW=object)
    source = ROOT / 'mhseals_nav/mavros_odometry.py'
    main = next(x for x in ast.parse(source.read_text()).body
                if isinstance(x, ast.FunctionDef) and x.name == 'main')
    namespace = {'__package__': 'mhseals_nav', 'LiveParams': lambda *args: {},
                 'positive': object(), 'Adapter': lambda *args: object()}
    exec(compile(ast.Module(body=[main], type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['main']


def test_main_declares_all_adapter_values_from_yaml_before_spin(tmp_path, monkeypatch):
    share = tmp_path / 'share'
    (share / 'config').mkdir(parents=True)
    raw = (ROOT / 'config/odometry_startup.yaml').read_text()
    raw = raw.replace('0.3', '2').replace('1.5', '3').replace('0.05', '4').replace('1.0', '5')
    (share / 'config/odometry_startup.yaml').write_text(raw)
    package = ModuleType('ament_index_python.packages')
    package.get_package_share_directory = lambda name: str(share)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', package)
    declarations, events = {}, []
    adapter_main(monkeypatch, declarations, events)()
    expected = odometry_configuration.resolve_odometry().adapter_parameters
    assert {key: declarations[key] for key in expected} == expected
    assert all(type(declarations[key]) is float for key in expected if key != 'velocity_source')
    assert events == ['init', 'node', 'spin', 'destroy']


def test_main_invalid_package_fails_before_ros_init_or_node(tmp_path, monkeypatch):
    (tmp_path / 'config').mkdir()
    (tmp_path / 'config/odometry_startup.yaml').write_text('mavros_velocity_stddev: false\n')
    package = ModuleType('ament_index_python.packages')
    package.get_package_share_directory = lambda name: str(tmp_path)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', package)
    declarations, events = {}, []
    with pytest.raises(ValueError):
        adapter_main(monkeypatch, declarations, events)()
    assert declarations == {}
    assert events == []
