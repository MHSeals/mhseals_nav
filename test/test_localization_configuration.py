"""Pure schema/graph validation; no robot_localization runtime is started."""
from hashlib import sha256
from pathlib import Path

from mhseals_nav.localization_configuration import compile_localization, NODES
import pytest
import yaml

ROOT = Path(__file__).parents[1]


def bases():
    return {name: ROOT / 'config' / (name + '.yaml') for name in NODES}


def test_shipped_values_types_immutable_projections_and_provenance(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    selected = compile_localization(bases(), use_sim_time=True)
    for name, path in bases().items():
        assert dict(selected.nodes[name]) == {
            k: tuple(v) if isinstance(v, list) else v
            for k, v in yaml.safe_load(path.read_bytes())['/**']['ros__parameters'].items()}
        params = selected.parameters(name)
        assert params['use_sim_time'] is True
        assert type(params['frequency']) is float
        with pytest.raises(TypeError):
            selected.nodes[name]['frequency'] = 99
    for record in selected.inputs[:3]:
        assert record['sha256'] == sha256(Path(record['path']).read_bytes()).hexdigest()
    assert selected.inputs[-1]['kind'] == 'overlay-absent'


def test_partial_scalar_vector_and_whole_array_replacement(tmp_path):
    path = tmp_path / 'overlay.yaml'
    vector = [True] + [False] * 14
    path.write_text(yaml.safe_dump({'ekf_local': {'frequency': 41, 'odom0_config': vector},
                                   'navsat_transform': {'ros__parameters': {'yaw_offset': -1}}}))
    result = compile_localization(bases(), str(path), True)
    assert result.parameters('ekf_local')['odom0_config'] == vector
    assert type(result.nodes['ekf_local']['frequency']) is float
    assert result.nodes['navsat_transform']['yaw_offset'] == -1.
    result.parameters('ekf_local')['odom0_config'][0] = False
    assert result.nodes['ekf_local']['odom0_config'][0] is True


@pytest.mark.parametrize('raw', [
    'ekf_local: {frequency: 1, frequency: 2}', 'ekf_local: &a {imu0_config: *a}',
    'ekf_local: {1: true}', 'future: {}', '/**: {}', 'ekf_local: null',
    'ekf_local: {ros__parameters: {}, frequency: 1}', 'ekf_local: {frequency: .nan}',
    'ekf_local: {frequency: true}', 'ekf_local: {frequency: 0}',
    'ekf_local: {imu0_queue_size: 0}', 'ekf_local: {imu0_queue_size: true}',
    'ekf_global: {process_noise_covariance: [1]}', 'ekf_local: {imu1: /imu/raw}',
    'ekf_local: {world_frame: map}', 'navsat_transform: {map_frame: other}',
    'ekf_global: {odom1: /different}', 'ekf_local: {use_sim_time: false}',
])
def test_invalid_supplied_overlay_refuses(tmp_path, raw):
    path = tmp_path / 'overlay.yaml'
    path.write_text(raw)
    with pytest.raises((ValueError, yaml.YAMLError)):
        compile_localization(bases(), str(path))


@pytest.mark.parametrize('vector', [[False] * 14, [False] * 16, [0] * 15, [[False]] * 15])
def test_vector_shape_and_actual_booleans(tmp_path, vector):
    path = tmp_path / 'overlay.yaml'
    path.write_text(yaml.safe_dump({'ekf_global': {'odom1_config': vector}}))
    with pytest.raises(ValueError):
        compile_localization(bases(), str(path))


def test_explicit_missing_vs_optional_and_invalid_default(tmp_path, monkeypatch):
    monkeypatch.setenv('HOME', str(tmp_path))
    path = tmp_path / '.config/mhseals/ekf.yaml'
    compile_localization(bases())
    with pytest.raises(FileNotFoundError):
        compile_localization(bases(), str(path))
    path.parent.mkdir(parents=True)
    path.write_text('ekf_global: null')
    with pytest.raises(ValueError):
        compile_localization(bases())


def test_changed_selected_base_cannot_redefine_graph(tmp_path):
    paths = bases()
    selected = tmp_path / 'global.yaml'
    selected.write_text(paths['ekf_global'].read_text().replace('/odom/mavros', '/other'))
    paths['ekf_global'] = selected
    with pytest.raises(ValueError):
        compile_localization(paths)
