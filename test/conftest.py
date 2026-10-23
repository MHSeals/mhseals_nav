"""Shared pytest fixtures."""

import logging

import pytest


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
