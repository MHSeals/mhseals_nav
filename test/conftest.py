"""Shared pytest fixtures."""

import logging

import pytest

from ros_isolation import isolate_ros_env


LIDAR_IP = '192.168.0.201'


def pytest_configure(config):
    """Keep tests that start nodes off the live boat: private domain, localhost only."""
    isolate_ros_env()


@pytest.fixture(autouse=True)
def boat_network_env(tmp_path_factory, monkeypatch):
    """Point boat_network at a vessel profile with the lidar's address, never the real one.

    The profile is a fleet (its lidar.yaml: the address from the env) and a vessel extending it
    (profile.yaml lidar: model and rpm), beside the vessel's network env.
    """
    from mhseals_nav import boat_network
    root = tmp_path_factory.mktemp('profiles')
    (root / 'fleet').mkdir()
    (root / 'fleet/lidar.yaml').write_text('driver: lidar/vlp16\naddress: $LIDAR_IP\nrpm:\n')
    (root / 'vessel').mkdir()
    (root / 'vessel/profile.yaml').write_text(
        'extends: fleet\nlidar: {model: VLP16, rpm: 600}\n')
    env = root / 'vessel/network.env'
    env.write_text(f'LIDAR_IP={LIDAR_IP}\n')
    monkeypatch.setattr(boat_network, 'path', lambda: env)
    return env


@pytest.fixture(autouse=True)
def propagating_module_loggers():
    """Let caplog see mhseals_nav logs even when launch has replaced the logger class.

    The launch_testing plugin imports launch, whose LaunchLogger has propagate=False
    and its own handlers, so records never reach the root logger caplog listens on.
    """
    loggers = [logging.getLogger('mhseals_nav.navigation_config'),
               logging.getLogger('mhseals_nav.lidar_configuration'),
               logging.getLogger('mhseals_nav.camera_configuration')]
    saved = [logger.propagate for logger in loggers]
    for logger in loggers:
        logger.propagate = True
    yield
    for logger, propagate in zip(loggers, saved):
        logger.propagate = propagate


@pytest.fixture
def installed(tmp_path, monkeypatch):
    """Run as a plain install: layers.package_path answers from a fake share (returned)."""
    import sys
    from types import ModuleType
    from mhseals_nav import layers
    share = tmp_path / 'installed/share/mhseals_nav'
    (share / 'config').mkdir(parents=True)
    monkeypatch.setattr(layers, 'SOURCE', tmp_path / 'site-packages')
    package, packages = ModuleType('ament_index_python'), ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = {'mhseals_nav': str(share)}.__getitem__
    package.packages = packages
    monkeypatch.setitem(sys.modules, 'ament_index_python', package)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    return share
