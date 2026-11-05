"""navigation.launch.py starts the costmap watchdog and approach guard only beside a mux."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

ROOT = Path(__file__).parents[1]


def guards(**arguments):
    """Return the guard nodes navigation.launch.py starts for ``arguments`` over its defaults."""
    spec = importlib.util.spec_from_file_location(
        'navigation_launch', ROOT / 'launch/navigation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    context.launch_configurations.update(arguments)
    module.Node = lambda **kwargs: kwargs
    return {node['executable']: node['parameters'][0] for node in module.guard_nodes(context)}


@pytest.mark.parametrize('mux,started', [(None, False), ('false', False), ('true', True),
                                         ('1', True)])
def test_watchdog_follows_command_mux(mux, started):
    nodes = guards(**({} if mux is None else {'command_mux': mux}))
    assert list(nodes) == (['costmap_watchdog', 'approach_guard'] if started else [])


def test_perception_false_skips_the_costmap_watchdog():
    assert list(guards(command_mux='true', perception='false')) == ['approach_guard']


def test_invalid_flag_raises():
    with pytest.raises(ValueError, match='invalid launch boolean'):
        guards(command_mux='yes')


@pytest.mark.parametrize('sim', ['true', 'false'])
def test_sim_sets_only_the_clock_and_the_yaml_keeps_enforce(sim):
    guard = guards(command_mux='true', sim=sim)['approach_guard']
    assert guard == {'use_sim_time': sim == 'true'}


@pytest.mark.parametrize('enforce,value', [('true', True), ('0', False)])
def test_approach_enforce_overrides_the_yaml(enforce, value):
    guard = guards(command_mux='true', approach_enforce=enforce)['approach_guard']
    assert guard['enforce'] is value
