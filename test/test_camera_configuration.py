"""Strict startup preview layers and immutable input identity without ROS."""
from hashlib import sha256
from pathlib import Path
import sys
from types import ModuleType

from mhseals_nav.camera_configuration import resolve_camera
import pytest

ROOT = Path(__file__).parents[1]
BASE = ROOT / 'config/camera.yaml'


def test_precedence_defaults_and_provenance(tmp_path):
    package, deployment, operator = [
        tmp_path / name for name in ('package', 'deployment', 'operator')]
    for path, hz in ((package, 3), (deployment, 4), (operator, 5)):
        path.write_text(f'tune: {{preview_rate: {hz}}}\n')
    assert resolve_camera(BASE).preview_rate == 2.
    assert resolve_camera(package).preview_rate == 3.
    assert resolve_camera(package, deployment).preview_rate == 4.
    result = resolve_camera(package, deployment, operator)
    assert result.preview_rate == 5. and type(result.preview_rate) is float
    assert resolve_camera(package, deployment, operator, '6').preview_rate == 6.
    assert resolve_camera(package, deployment, operator, '').preview_rate == 5.
    expected = {'path': str(operator), 'layer': 'operator',
                'sha256': sha256(operator.read_bytes()).hexdigest()}
    assert result.inputs[-1] == expected
    with pytest.raises(TypeError):
        result.inputs[-1]['path'] = 'modified'
    assert operator.read_text() == 'tune: {preview_rate: 5}\n'


@pytest.mark.parametrize('raw', [
    *(f'tune: {{preview_rate: {v}}}' for v in ('false', 'null', '.nan', '.inf', '0', '-1', '"2"')),
    'tune: {other: 2}', 'other: 2', '{}', 'null', '[]', 'tune: {preview_rate: 2, other: 2}',
    'tune: {preview_rate: 2, preview_rate: 3}', 'tune: {preview_rate: &x 2}\nother: *x',
    '<<: {tune: {preview_rate: 2}}', 'preview_hz: false', 'preview_hz: 2\nother: 2'])
@pytest.mark.parametrize('layer', ['package', 'deployment', 'operator'])
def test_bad_selected_layer_cannot_be_masked(tmp_path, raw, layer):
    path = tmp_path / 'invalid.yaml'
    path.write_text(raw)
    paths = [BASE, '', '']
    paths[['package', 'deployment', 'operator'].index(layer)] = path
    with pytest.raises((ValueError, __import__('yaml').YAMLError)):
        resolve_camera(*paths, preview_rate='9')


@pytest.mark.parametrize('value', [True, None, 0, -1, 'nan', 'bad', 10 ** 400])
def test_invalid_alias(value):
    with pytest.raises(ValueError):
        resolve_camera(BASE, preview_rate=value)


def test_installed_share_selection_from_unrelated_cwd(tmp_path, monkeypatch):
    share = tmp_path / 'share/mhseals_nav'
    (share / 'config').mkdir(parents=True)
    (share / 'config/camera.yaml').write_bytes(BASE.read_bytes())
    packages = ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = lambda name: str(share)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    monkeypatch.chdir(tmp_path)
    assert resolve_camera().inputs[0]['path'] == str(share / 'config/camera.yaml')
    with pytest.raises(FileNotFoundError):
        resolve_camera(operator_file=tmp_path / 'missing')


def test_actual_launch_adapter_uses_selected_files_and_alias(tmp_path, monkeypatch):
    from mhseals_nav.camera_configuration import from_launch
    package = ModuleType('ament_index_python.packages')
    package.get_package_share_directory = lambda name: str(ROOT)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', package)
    substitutions = ModuleType('launch.substitutions')

    class Configuration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]
    substitutions.LaunchConfiguration = Configuration
    monkeypatch.setitem(sys.modules, 'launch.substitutions', substitutions)
    path = tmp_path / 'selected.yaml'
    path.write_text('tune: {preview_rate: 7}\n')
    context = {'camera_deployment_file': '', 'camera_startup_file': str(path), 'preview_rate': ''}
    assert from_launch(context).preview_rate == 7.
    context['preview_rate'] = '8'
    result = from_launch(context)
    assert result.preview_rate == 8. and result.override == '8'


def test_an_old_preview_hz_layer_reads_for_one_release(tmp_path, caplog):
    path = tmp_path / 'old.yaml'
    path.write_text('preview_hz: 3\n')
    assert resolve_camera(BASE, path).preview_rate == 3.
    assert 'preview_hz is deprecated' in caplog.text
