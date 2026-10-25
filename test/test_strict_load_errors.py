"""Strict-load failures in every section loader name the file and the line."""
from pathlib import Path

from mhseals_nav import mavros_configuration
from mhseals_nav.camera_configuration import resolve_camera
from mhseals_nav.endpoint_configuration import resolve_endpoints
from mhseals_nav.lidar_configuration import resolve_lidar
from mhseals_nav.localization_configuration import compile_localization, NODES
from mhseals_nav.mavros_configuration import resolve_setup
from mhseals_nav.odometry_configuration import resolve_odometry
import pytest

ROOT = Path(__file__).parents[1]


@pytest.fixture
def bad(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('a: 1\na: 2\n')
    return path


def check(error, path):
    assert str(path) in str(error.value)
    assert 'line ' in str(error.value)


def test_endpoint(bad):
    with pytest.raises(ValueError) as error:
        resolve_endpoints(bad)
    check(error, bad)


def test_odometry(bad):
    with pytest.raises(ValueError) as error:
        resolve_odometry(bad)
    check(error, bad)


def test_camera(bad):
    with pytest.raises(ValueError) as error:
        resolve_camera(bad)
    check(error, bad)


def test_mavros(bad, monkeypatch):
    monkeypatch.setattr(mavros_configuration, 'installed_setup_defaults', dict)
    with pytest.raises(ValueError) as error:
        resolve_setup(str(bad))
    check(error, bad)


def test_lidar(bad):
    with pytest.raises(ValueError) as error:
        resolve_lidar(str(bad), share=lambda name: ROOT)
    check(error, bad)


def test_localization(bad):
    with pytest.raises(ValueError) as error:
        compile_localization({name: bad for name in NODES})
    check(error, bad)
