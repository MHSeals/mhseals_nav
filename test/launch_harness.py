"""Run this package's launch files without ROS: fake launch modules, inspectable records.

``Harness(monkeypatch).run('perception.launch.py', camera='sim')`` loads ``launch/<file>`` as a
real module with fake ``launch`` and ``launch_ros`` modules in ``sys.modules``, gives every
declared argument its default, overlays the given ones and runs the OpaqueFunctions (through
GroupAction and TimerAction). It returns the flat list of what would start: Node,
IncludeLaunchDescription and ExecuteProcess records. Each record is a dict of the constructor's
keywords with ``kind``, ``timer`` (the enclosing TimerAction's period, else None) and ``env`` (the
SetEnvironmentVariable values of its GroupAction scope). ``built`` lists every Node constructed,
so a test can show that bad input fails before any.
"""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType

import pytest

ROOT = Path(__file__).parents[1]
CONTAINERS = ('GroupAction', 'TimerAction')
STARTS = ('Node', 'IncludeLaunchDescription', 'ExecuteProcess')


class Record(dict):
    """One constructed launch object: its keywords, its kind and its positional arguments."""

    def __init__(self, kind, args=(), **kwargs):
        super().__init__(kwargs)
        self.kind, self.args = kind, args
        self.timer, self.env = None, {}

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError:
            raise AttributeError(name) from None

    def __repr__(self):
        return f'{self.kind}({dict.__repr__(self)})'


class Share(str):
    """FindPackageShare(package): text that remembers the package."""

    def __new__(cls, package):
        found = super().__new__(cls, f'<share {package}>')
        found.package = package
        return found


class LaunchConfiguration:
    def __init__(self, name, default=None):
        self.name, self.default = name, default

    def perform(self, context):
        if self.name not in context and self.default is not None:
            return self.default
        return context[self.name]


def performed(value, context):
    """Return a launch value as text: strings stay, substitutions perform, lists join."""
    if hasattr(value, 'perform'):
        return value.perform(context)
    if isinstance(value, (list, tuple)):
        return ''.join(performed(part, context) for part in value)
    return value


class Harness:
    def __init__(self, monkeypatch):
        self.monkeypatch = monkeypatch
        self.built = []
        self.modules = {}
        self.shutdown = []
        self._fake_modules()

    def _fake_modules(self):
        def kind(name, keys=()):
            def construct(*args, **kwargs):
                record = Record(name, args, **dict(zip(keys, args)), **kwargs)
                if name == 'Node':
                    self.built.append(record)
                return record
            construct.__name__ = name
            return construct

        def module(name, **members):
            found = ModuleType(name)
            found.__dict__.update(members)
            self.monkeypatch.setitem(sys.modules, name, found)
            return found

        def join(parts):
            return '/'.join(str(part) for part in parts)

        actions = {name: kind(name, keys) for name, keys in (
            ('DeclareLaunchArgument', ('name',)), ('OpaqueFunction', ()),
            ('TimerAction', ()), ('GroupAction', ()), ('IncludeLaunchDescription', ('source',)),
            ('SetEnvironmentVariable', ('name', 'value')), ('LogInfo', ()),
            ('RegisterEventHandler', ('handler',)), ('ExecuteProcess', ()))}
        package = module('launch', LaunchDescription=lambda entities: list(entities),
                         LaunchContext=dict)
        package.actions = module('launch.actions', **actions)
        package.substitutions = module('launch.substitutions',
                                       LaunchConfiguration=LaunchConfiguration,
                                       PathJoinSubstitution=join)
        package.conditions = module('launch.conditions', IfCondition=kind('IfCondition'),
                                    UnlessCondition=kind('UnlessCondition'))
        package.event_handlers = module('launch.event_handlers', OnShutdown=kind('OnShutdown'))
        package.launch_description_sources = module(
            'launch.launch_description_sources',
            PythonLaunchDescriptionSource=kind('PythonLaunchDescriptionSource', ('location',)))
        ros = module('launch_ros')
        ros.actions = module('launch_ros.actions', Node=kind('Node'))
        ros.substitutions = module('launch_ros.substitutions',
                                   FindPackageShare=Share)

    def close(self):
        """Run the OnShutdown handlers the runs registered (navigation's temp parameter file)."""
        for action in self.shutdown:
            action['function'](None)
        self.shutdown = []

    def load(self, filename):
        """Load the launch file as a module (one per harness, so a test can patch its names)."""
        if filename not in self.modules:
            path = ROOT / 'launch' / filename
            name = 'launch_file_' + filename.replace('.', '_')
            spec = importlib.util.spec_from_file_location(name, path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self.modules[filename] = module
        return self.modules[filename]

    def declared(self, filename):
        """Return {argument: default} of the file's DeclareLaunchArgument entities."""
        return {entity['name']: performed(entity.get('default_value'), {})
                for entity in self.load(filename).generate_launch_description()
                if isinstance(entity, Record) and entity.kind == 'DeclareLaunchArgument'}

    def context(self, filename, **arguments):
        """Return what a run sees: the declared defaults, then ``arguments``."""
        found = {**self.declared(filename), **arguments}
        missing = sorted(name for name, value in found.items() if value is None)
        if missing:
            pytest.fail(f'{filename}: no value for required arguments {missing}')
        return found

    def run(self, filename, **arguments):
        """[Record] of what the file starts with ``arguments``, in launch order."""
        context = self.context(filename, **arguments)
        found = []

        def walk(entities, timer=None, env=None):
            for entity in entities or ():
                if not isinstance(entity, Record):
                    continue
                if entity.kind == 'OpaqueFunction':
                    walk(entity['function'](context), timer, env)
                elif entity.kind == 'GroupAction':
                    scope = dict(env or {})
                    for action in entity['actions']:
                        if isinstance(action, Record) and action.kind == 'SetEnvironmentVariable':
                            scope[performed(action['name'], context)] = performed(
                                action['value'], context)
                    walk(entity['actions'], timer, scope)
                elif entity.kind == 'TimerAction':
                    walk(entity['actions'], entity['period'], env)
                elif entity.kind == 'RegisterEventHandler':
                    self.shutdown += entity['handler'].get('on_shutdown', [])
                elif entity.kind in STARTS:
                    entity.timer, entity.env = timer, dict(env or {})
                    found.append(entity)
        walk(self.load(filename).generate_launch_description())
        return found


def executables(records):
    """Return the executables of the Node records, in order."""
    return [record['executable'] for record in records if record.kind == 'Node']


def packages(records):
    """Return the ROS packages the records start: Nodes' and includes' FindPackageShare."""
    found = set()
    for record in records:
        if record.kind == 'Node':
            found.add(record['package'])
        elif record.kind == 'IncludeLaunchDescription':
            location = record['source']['location']
            parts = location if isinstance(location, list) else [location]
            found |= {part.package for part in parts if isinstance(part, Share)}
    return found


def included(record):
    """Return (launch file name, {argument: value}) of an IncludeLaunchDescription."""
    return (Path(performed(record['source']['location'], {})).name,
            dict(record.get('launch_arguments', ())))


def named(records, name):
    """Return the one Node record whose name (else executable) is ``name``."""
    (found,) = [record for record in records if record.kind == 'Node'
                and record.get('name', record.get('executable')) == name]
    return found


@pytest.fixture
def harness(monkeypatch):
    """Give a Harness whose fake launch modules leave sys.modules when the test ends."""
    found = Harness(monkeypatch)
    yield found
    found.close()
