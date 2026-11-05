"""Shared pytest fixtures."""

import importlib.util
import logging
import re

import pytest

from ros_isolation import isolate_ros_env


LIDAR_IP = '192.168.0.201'


def pytest_configure(config):
    """Keep tests that start nodes off the live boat: private domain, localhost only."""
    isolate_ros_env()


NO_ROS = importlib.util.find_spec('rclpy') is None
MISSING = re.compile(r"No module named '([\w.]+)'|"
                     r"cannot import name '\w+' from '([\w.]+)' \(unknown location\)")
# Only ROS (and its linters) may be missing: a dropped pip dependency still fails.
ROS_MODULE = re.compile(r'rcl\w*|rosidl\w*|launch\w*|ament_\w+|\w+_(msgs|srvs|interfaces)|'
                        r'tf2\w*|xacro|message_filters|cv_bridge|builtin_interfaces')


def ros_missing(text):
    """The ROS module an ImportError lacks, on a host with no ROS (None for any other)."""
    match = MISSING.search(text) if NO_ROS else None
    name = match and (match[1] or match[2]).split('.')[0]
    return name if name and ROS_MODULE.fullmatch(name) else None


@pytest.hookimpl(hookwrapper=True)
def pytest_make_collect_report(collector):
    """With no ROS here, a test file that imports ROS (or the linters) skips, not errors."""
    outcome = yield
    report = outcome.get_result()
    name = report.failed and ros_missing(str(report.longrepr))
    if name:
        report.outcome = 'skipped'
        report.longrepr = (str(collector.path), 0, f'Skipped: needs {name} (no ROS here)')


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    """With no ROS here, a test that imports ROS skips, not fails (CI's ROS-free step)."""
    outcome = yield
    report = outcome.get_result()
    name = (call.excinfo is not None and call.excinfo.errisinstance(ImportError)
            and ros_missing(str(call.excinfo.value)))
    if name:
        report.outcome = 'skipped'
        report.longrepr = (str(item.path), item.location[1] or 0,
                           f'Skipped: needs {name} (no ROS here)')


@pytest.fixture(autouse=True)
def boat_network_env(tmp_path_factory, monkeypatch):
    """Point boat_network at a vessel profile with the lidar's address, never the real one.

    The profile is a fleet (its lidar.yaml: the address from the env) and a vessel extending it
    (profile.yaml lidar: model and rpm), beside the vessel's network env.
    """
    from mhseals_nav import boat_network
    root = tmp_path_factory.mktemp('profiles')
    (root / 'fleet').mkdir()
    (root / 'fleet/profile.yaml').write_text('name: fleet\n')
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
