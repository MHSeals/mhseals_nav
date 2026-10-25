"""navigation.launch.py starts the stale-costmap watchdog only alongside a command_mux."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

ROOT = Path(__file__).parents[1]


def description():
    spec = importlib.util.spec_from_file_location(
        'navigation_launch', ROOT / 'launch/navigation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


@pytest.mark.parametrize('mux,started', [(None, False), ('false', False), ('true', True)])
def test_watchdog_follows_command_mux(mux, started):
    entities = description().entities
    context = LaunchContext()
    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    if mux is not None:
        context.launch_configurations['command_mux'] = mux
    (watchdog,) = [e for e in entities if isinstance(e, Node)]
    assert watchdog.node_executable == 'costmap_watchdog'
    assert watchdog.condition.evaluate(context) is started
