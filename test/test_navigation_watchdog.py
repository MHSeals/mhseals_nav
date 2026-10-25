"""navigation.launch.py starts the costmap watchdog and approach guard only beside a mux."""

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
    nodes = [e for e in entities if isinstance(e, Node)]
    assert [n.node_executable for n in nodes] == ['costmap_watchdog', 'approach_guard']
    for node in nodes:
        assert node.condition.evaluate(context) is started


@pytest.mark.parametrize('sim', ['true', 'false'])
def test_approach_guard_enforces_in_sim_and_only_logs_on_the_boat(sim):
    from launch.utilities import perform_substitutions
    entities = description().entities
    context = LaunchContext()
    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations['sim'] = sim
    (guard,) = [e for e in entities
                if isinstance(e, Node) and e.node_executable == 'approach_guard']
    (overrides,) = [p for p in guard._Node__parameters if isinstance(p, dict)]
    (value,) = [v for k, v in overrides.items()
                if perform_substitutions(context, list(k)) == 'enforce']
    assert perform_substitutions(context, list(value.value)) == sim
