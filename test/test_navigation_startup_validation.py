"""Actual validation and startup factory failures, without ROS imports."""
import ast
from pathlib import Path
import pytest
import yaml
from mhseals_nav.navigation_config import load_overlay, validated_merge_limits

ROOT = Path(__file__).parents[1]


@pytest.fixture
def base():
    return yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())


@pytest.mark.parametrize('patch', [
    {'velocity_smoother': {'ros__parameters': {'max_velocity': []}}},
    {'controller_server': {'ros__parameters': {'FollowPath': {'time_steps': -1}}}},
    {'controller_server': []},
    {'controller_server': {'ros__parameters': {'controller_frequency': float('nan')}}},
    {'velocity_smoother': {'ros__parameters': {'feedback': 'bad'}}},
    {'unknown_server': {}},
])
def test_invalid_requested_values_fail(base, patch):
    with pytest.raises(ValueError):
        validated_merge_limits(base, patch)


@pytest.mark.parametrize('text', ['[', '[]', 'false', 'a: 1\na: 2', 'a: &x {}\nb: *x', '<<: {}'])
def test_invalid_existing_file_is_not_absent(tmp_path, text):
    path = tmp_path / 'overlay.yaml'
    path.write_text(text)
    with pytest.raises((ValueError, yaml.YAMLError)):
        load_overlay(path, strict=True)


def test_absent_optional_file_and_valid_overlay(base, tmp_path):
    assert load_overlay(tmp_path / 'absent', strict=True) == {}
    patch = {'controller_server': {'ros__parameters': {'controller_frequency': 20}}}
    result = validated_merge_limits(base, patch)
    assert result['controller_server']['ros__parameters']['FollowPath']['model_dt'] == .05
    with pytest.raises(ValueError):
        load_overlay(tmp_path, strict=True)  # existing but unreadable as a file


def test_actual_navigation_factory_fails_before_node(base, tmp_path):
    source = ROOT / 'launch/navigation.launch.py'
    tree = ast.parse(source.read_text())
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'navigation_nodes')
    base_path = tmp_path / 'base.yaml'
    base_path.write_text(yaml.safe_dump(base))
    overlay = tmp_path / 'bad.yaml'
    overlay.write_text('controller_server: []')
    class Configuration:
        def __init__(self, name): self.name = name
        def perform(self, context): return context[self.name]
    # Inert imports used by the actual factory; never load launch or rclpy.
    import sys
    import types
    from unittest.mock import patch
    actions = types.ModuleType('launch.actions')
    actions.RegisterEventHandler = lambda *a, **kw: None
    events = types.ModuleType('launch.event_handlers')
    events.OnShutdown = lambda **kw: None
    import os
    import tempfile
    ns = dict(LaunchConfiguration=Configuration, os=os, yaml=yaml, tempfile=tempfile,
              load_overlay=load_overlay, validated_merge_limits=validated_merge_limits,
              Node=lambda **kw: pytest.fail('Node constructed before validation'))
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), ns)
    with patch.dict(sys.modules, {'launch.actions': actions, 'launch.event_handlers': events}):
        with pytest.raises(ValueError):
            ns['navigation_nodes']({'params_file': str(base_path), 'limits_file': str(overlay)})
