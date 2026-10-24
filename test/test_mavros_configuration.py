"""Pure desired-rate startup validation; no ROS or FCU interaction."""
from pathlib import Path

from mhseals_nav.mavros_configuration import rates_from_yaml, resolve_setup, validate_rates
import pytest
import yaml


def selected(tmp_path, entries):
    path = tmp_path / 'rates.yaml'
    path.write_text(yaml.safe_dump({'mavros_setup': {
        'ros__parameters': {'message_rates': entries}}}))
    return str(path)


def test_package_authority():
    path = Path(__file__).parents[1] / 'config/mavros_setup.yaml'
    assert dict(rates_from_yaml(path.read_text())) == {
        33: 20, 30: 50, 31: 50, 24: 5, 27: 10, 1: 2, 193: 2, 65: 25}


@pytest.mark.parametrize('entries', [None, {}, [], [True], ['1:2', '1:3'],
                                       ['-1:2'], ['16777216:2'], ['1.0:2'],
                                       ['true:2'], ['1:nan'], ['1:inf'], ['1:-2'],
                                       ['1:true'], ['1:2:3']])
def test_reject_invalid_entries(entries):
    with pytest.raises(ValueError):
        validate_rates(entries)


def test_sentinel_contract():
    assert validate_rates(['1:0', '2:-1', '3:0.25']) == [(1, 0), (2, -1), (3, .25)]


@pytest.mark.parametrize('raw', ['mavros_setup: {}', '[]',
    'mavros_setup: {}\nmavros_setup: {}',
    'mavros_setup: {ros__parameters: {message_rates: ["1:2"], retries: 3}}'])
def test_reject_schema_and_duplicate_keys(raw):
    with pytest.raises((ValueError, yaml.YAMLError)):
        rates_from_yaml(raw)


def test_nondefault_and_explicit_alias(tmp_path):
    path = selected(tmp_path, ['33:7', '30:8', '31:9', '27:3', '1:4', '193:6'])
    absent = resolve_setup(path, {'mavros_imu_rate': ''})
    assert absent['message_rates'] == ['33:7', '30:8', '31:9', '27:3', '1:4', '193:6']
    explicit = resolve_setup(path, {'mavros_imu_rate': '20.0'})
    assert explicit['message_rates'][1:3] == ['30:20', '31:20']
    with pytest.raises(ValueError):
        resolve_setup(path, {'mavros_rc_rate': 'nan'})


@pytest.mark.parametrize('path', ['', '/does/not/exist'])
def test_no_fallback(path):
    with pytest.raises((ValueError, OSError)):
        resolve_setup(path)
