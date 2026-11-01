"""help.yaml (astro_dock's help browser) lists every launch file and only real arguments."""
from pathlib import Path
import re

import yaml

ROOT = Path(__file__).resolve().parents[1]
# Not listed: its mode check compares an unquoted name (PythonExpression), so it cannot run.
UNLISTED = {'log.launch.py'}
COMMANDS = yaml.safe_load((ROOT / 'help.yaml').read_text())['commands']


def launches():
    """{launch file: its help.yaml command} of every 'ros2 launch mhseals_nav' entry."""
    found = {}
    for command in COMMANDS:
        words = command.get('run', '').split()
        if words[:3] == ['ros2', 'launch', 'mhseals_nav']:
            found[words[3]] = command
    return found


def choices(name):
    return next(set(p['choices']) for c in COMMANDS for p in c.get('params', [])
                if c['name'] == name and 'choices' in p)


def test_every_launch_file_is_listed():
    files = {path.name for path in (ROOT / 'launch').glob('*.launch.py')}
    assert set(launches()) == files - UNLISTED


def test_listed_arguments_are_declared_by_their_launch():
    for launch, command in launches().items():
        source = (ROOT / 'launch' / launch).read_text()
        for param in command.get('params', []):
            assert re.search(r"""['"]%s['"]""" % param['name'], source), (launch, param['name'])


def test_choices_match_the_code():
    trees = (ROOT / 'mhseals_nav/tasks/trees/__init__.py').read_text()
    names = set(re.findall(r"^\s+'(\w+)': \w+\.build", trees, re.M)) | {'mission'}
    assert choices('task.launch') == names
    assert choices('run_task') == names
    assert choices('device.launch') == {p.stem for p in (ROOT / 'config/devices').glob('*.yaml')}
    profiles = ROOT / 'config/tasks/profiles'
    assert {p['name']: set(p.get('choices', [])) for c in COMMANDS if c['name'] == 'run_task'
            for p in c['params']}['--profile'] == {p.stem for p in profiles.glob('*.yaml')}
