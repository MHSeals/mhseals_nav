"""Compare the optional TUI source contract with the persistent Nav2 overlay and endpoints."""
import ast
import os
from pathlib import Path
from types import SimpleNamespace

from mhseals_nav.navigation_config import persistent_limits_path
import pytest

ROOT = Path(__file__).parents[1]


@pytest.mark.parametrize('override,xdg', [
    (None, None), (None, '/tmp/xdg'), ('~/custom.yaml', '/tmp/xdg')])
def test_tui_actual_path_and_component_argv(monkeypatch, override, xdg, tmp_path):
    root = os.environ.get('MHSEALS_TESTING_SOURCE')
    if not root:
        pytest.skip('set MHSEALS_TESTING_SOURCE for cross-repository TUI contract probe')
    for key, value in [('MHSEALS_NAV2', override), ('XDG_CONFIG_HOME', xdg)]:
        if value is None:
            monkeypatch.delenv(key, raising=False)
        else:
            monkeypatch.setenv(key, value)
    root = Path(root) / 'mhseals_testing'

    def functions(file, names, namespace):
        tree = ast.parse(file.read_text())
        nodes = [n for n in tree.body if
                 (isinstance(n, ast.FunctionDef) and n.name in names) or
                 (isinstance(n, ast.Assign) and any(
                     isinstance(target, ast.Name) and target.id in names for target in n.targets))]
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(file), 'exec'), namespace)
    tui = {'os': os, 'Path': Path}
    functions(root / 'nav2_limits.py', {'path', 'launch_argument'}, tui)
    component = {'nav2_limits': SimpleNamespace(launch_argument=tui['launch_argument']),
                 'Component': lambda *args: args}
    functions(root / 'harness/components.py',
              {'components', 'odometry_arguments', 'validate_localization_profile',
               'ODOMETRY_INPUTS', 'ODOMETRY_LAUNCHES', 'endpoint_arguments',
               'ENDPOINT_LAUNCHES', 'ENDPOINT_INPUTS'},
              component)
    from mhseals_nav.endpoint_configuration import resolve_endpoints
    authority = resolve_endpoints(ROOT / 'config/endpoint_startup.yaml', context='tui')
    selected = tmp_path / 'endpoint.yaml'
    port = authority.unity_parameters['ROS_TCP_PORT'] + 1
    selected.write_text(f'unity: {{port: {port}}}\n')
    profile = SimpleNamespace(
        map_frame='map', odom_frame='odom', base_frame='base_link',
        odometry='/odom/local', use_sim_time=False,
        components={
            'navigation': {'launch': ['mhseals_nav', 'navigation.launch.py'], 'limits': True},
            'odometry': {'launch': ['mhseals_nav', 'odom.launch.py'],
                         'args': {'sim': 'false', 'endpoint_startup_file': str(selected)}},
            'sensors': {'launch': ['mhseals_nav', 'sensors.launch.py']}})
    children = component['components'](profile)
    argv = children[0][2]
    assert argv[-1] == f'limits_file:={persistent_limits_path()}'
    for child in children[1:]:
        arguments = dict(item.split(':=', 1) for item in child[2][4:])
        assert arguments['endpoint_context'] == authority.context
        requested = resolve_endpoints(
            ROOT / 'config/endpoint_startup.yaml',
            operator_file=arguments['endpoint_startup_file'],
            context=arguments['endpoint_context'])
        assert requested.unity_parameters['ROS_TCP_PORT'] == port
        assert requested.values['contexts'] == authority.values['contexts']
        assert requested.inputs[0] == authority.inputs[0]
        assert requested.inputs[-1]['path'] == str(selected)
