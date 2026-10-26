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
    """Point boat_network at a boat-network.env with the lidar's address, never the real one."""
    from mhseals_nav import boat_network
    env = tmp_path_factory.mktemp('boat-network') / 'boat-network.env'
    env.write_text(f'LIDAR_IP={LIDAR_IP}\n')
    monkeypatch.setattr(boat_network, 'path', lambda: env)
    return env


@pytest.fixture(autouse=True)
def propagating_module_loggers():
    """Let caplog see mhseals_nav logs even when launch has replaced the logger class.

    The launch_testing plugin imports launch, whose LaunchLogger has propagate=False
    and its own handlers, so records never reach the root logger caplog listens on.
    """
    loggers = [logging.getLogger('mhseals_nav.navigation_config')]
    saved = [logger.propagate for logger in loggers]
    for logger in loggers:
        logger.propagate = True
    yield
    for logger, propagate in zip(loggers, saved):
        logger.propagate = propagate
