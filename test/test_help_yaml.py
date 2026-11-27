"""help.yaml (astro's help browser): device.launch.py is the one launch entry, and every choice is real."""
from pathlib import Path
import re

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
COMMANDS = {c['name']: c for c in yaml.safe_load((ROOT / 'help.yaml').read_text())['commands']}


def params(name):
    return {p['name']: p for p in COMMANDS[name].get('params', [])}


def test_device_is_the_only_launch_and_its_arguments_are_declared():
    launches = [c for c in COMMANDS.values() if c['run'].startswith('ros2 launch')]
    assert [c['run'] for c in launches] == ['ros2 launch mhseals_nav device.launch.py']
    source = (ROOT / 'launch/device.launch.py').read_text()
    for name in params('device'):
        assert re.search(r"""['"]%s['"]""" % name, source), name
    from mhseals_nav import device_parts
    for name in ('device', 'only'):   # the help page lists both, from the profiles
        assert params('device')[name]['list'] is True
        command = params('device')[name]['choices_from'].split()
        assert command[:4] == ['ros2', 'run', 'mhseals_nav', 'device_parts']
        found = [line.split('\t')[0] for line in device_parts.lines(*command[4:])]
        assert found and all(found)
    assert [line.split('\t')[0] for line in device_parts.lines('--devices')] == sorted(
        p.stem for p in (ROOT / 'config/devices').glob('*.yaml'))


def test_every_entry_has_a_level_and_a_host():
    for name, command in COMMANDS.items():
        assert command['level'] in ('core', 'setup'), name
        assert command.get('host', 'here') in ('boat_computer', 'operator', 'here'), name   # none: runs where you are


def test_task_profiles_match_the_files():
    assert set(params('task')['--profile']['choices']) == {
        p.stem for p in (ROOT / 'config/tasks/profiles').glob('*.yaml')}


@pytest.mark.parametrize('name, param, expected', [
    ('task', '--task', lambda: __import__('mhseals_nav.tasks.trees', fromlist=['TREES']).TREES),
    ('trial', 'course', lambda: [p.stem for p in (ROOT / 'config/trials').glob('*.yaml')] + ['home']),
])
def test_choices_from_prints_the_real_choices(name, param, expected, capsys):
    command = params(name)[param]['choices_from'].split()
    assert command[:3] == ['ros2', 'run', 'mhseals_nav'] and command[-1] == '--list'
    if command[3] == 'run_task':
        from mhseals_nav.tasks.interface import main
        main(['run_task', '--list'])
    else:
        from mhseals_nav.trial import main
        assert main(['--list']) == 0
    assert capsys.readouterr().out.split() == sorted(expected())
