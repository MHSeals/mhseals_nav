"""Run the actual robot factory and compare the optional TUI source contract."""
import ast
import os
from pathlib import Path
from types import SimpleNamespace
import pytest
from mhseals_nav.navigation_config import persistent_limits_path


def robot_factory():
    source = Path(__file__).parents[1] / 'launch/robot.launch.py'
    tree = ast.parse(source.read_text())
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef))
    class Configuration:
        def __init__(self, name): self.name = name
        def perform(self, context): return context[self.name]
    def record(*args, **kwargs): return SimpleNamespace(args=args, **kwargs)
    ns = {name: record for name in (
        'DeclareLaunchArgument', 'IncludeLaunchDescription', 'TimerAction',
        'PythonLaunchDescriptionSource', 'IfCondition')}
    ns.update(LaunchConfiguration=Configuration, LaunchDescription=lambda x: x,
              PathJoinSubstitution=lambda x: '/'.join(map(str, x)),
              FindPackageShare=lambda x: x, persistent_limits_path=persistent_limits_path)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), ns)
    return ns['generate_launch_description']()


@pytest.mark.parametrize('explicit', [None, '/tmp/nondefault limits.yaml', ''])
def test_robot_declares_and_forwards_overlay(monkeypatch, explicit):
    monkeypatch.setenv('MHSEALS_NAV2', '/tmp/persistent.yaml')
    entities = robot_factory()
    declared = {e.args[0]: e.default_value for e in entities if hasattr(e, 'default_value')}
    assert declared['limits_file'] == '/tmp/persistent.yaml'
    timer = next(e for e in entities if getattr(e, 'period', None) == 5.0)
    include = timer.actions[0]
    args = dict(include.launch_arguments)
    context = dict(declared)
    if explicit is not None: context['limits_file'] = explicit
    assert args['limits_file'].perform(context) == context['limits_file']


@pytest.mark.parametrize('override,xdg', [(None, None), (None, '/tmp/xdg'), ('~/custom.yaml', '/tmp/xdg')])
def test_tui_actual_path_and_component_argv(monkeypatch, override, xdg):
    root = os.environ.get('MHSEALS_TESTING_SOURCE')
    if not root: pytest.skip('set MHSEALS_TESTING_SOURCE for cross-repository TUI contract probe')
    for key, value in [('MHSEALS_NAV2', override), ('XDG_CONFIG_HOME', xdg)]:
        if value is None: monkeypatch.delenv(key, raising=False)
        else: monkeypatch.setenv(key, value)
    root = Path(root) / 'mhseals_testing'
    def functions(file, names, namespace):
        tree = ast.parse(file.read_text())
        nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(file), 'exec'), namespace)
    tui = dict(os=os, Path=Path)
    functions(root / 'nav2_limits.py', {'path', 'launch_argument'}, tui)
    component = dict(nav2_limits=SimpleNamespace(launch_argument=tui['launch_argument']),
                     Component=lambda *args: args)
    functions(root / 'harness/components.py', {'components'}, component)
    profile = SimpleNamespace(components={'navigation': {
        'launch': ['mhseals_nav', 'navigation.launch.py'], 'limits': True}})
    argv = component['components'](profile)[0][2]
    assert argv[-1] == f'limits_file:={persistent_limits_path()}'
