"""Every ROS package a launch file starts is in package.xml, or only a profile that requires it starts it."""
import ast
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from mhseals_nav import device_profile
import pytest

ROOT = Path(__file__).parents[1]
# Host capability (a device profile's requires) -> the packages that host installs for it.
GATED = {'zed': {'zed_wrapper'}}
USES = re.compile(r"""(?:package=|FindPackageShare\()['"](\w+)['"]""")
INCLUDES = re.compile(r"""'launch',\s*'(\w+\.launch\.py)'""")


@pytest.fixture(autouse=True)
def source_tree(monkeypatch):
    monkeypatch.setattr(device_profile, 'share', lambda: ROOT)


def launched():
    """{package: {launch files that start it}}; navigation.launch.py starts SERVERS by variable."""
    used = {}
    for path in (ROOT / 'launch').glob('*.launch.py'):
        for package in set(USES.findall(path.read_text())) - {'mhseals_nav'}:
            used.setdefault(package, set()).add(path.name)
    source = (ROOT / 'mhseals_nav' / 'startup_projection.py').read_text()
    servers = next(node.value for node in ast.parse(source).body if isinstance(node, ast.Assign)
                   and node.targets[0].id == 'SERVERS')
    for package, _ in ast.literal_eval(servers):
        used.setdefault(package, set()).add('navigation.launch.py')
    return used


def declared():
    tags = ('depend', 'exec_depend')
    return {item.text for tag in tags for item in ET.parse(ROOT / 'package.xml').iter(tag)}


def test_every_launched_package_is_declared_or_gated():
    gated = set().union(*GATED.values())
    missing = sorted(set(launched()) - declared() - gated)
    assert not missing, f'add to package.xml or gate behind a profile requires: {missing}'


@pytest.mark.parametrize('capability,package', [
    (capability, package) for capability, packages in GATED.items() for package in packages])
def test_gated_package_starts_only_where_its_host_has_it(capability, package):
    assert package not in declared()
    files = launched()[package]
    included = {name for path in (ROOT / 'launch').glob('*.launch.py')
                for name in INCLUDES.findall(path.read_text())}
    assert not files & included, f'{package}: a launch file includes {files & included}'
    profiles = []
    for name in device_profile.available():
        profile = device_profile._package(name, ())
        if any(spec.get('launch') in files for _, spec in device_profile.select(profile)):
            profiles.append(name)
            assert capability in profile['requires'], f'{name} starts {package} without {capability}'
    assert profiles, f'no device profile starts {package}'
