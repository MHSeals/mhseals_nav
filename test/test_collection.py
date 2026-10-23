"""Test modules must import without optional packages so a directory run collects them."""

import importlib
import sys

import pytest


def test_object_bridge_module_imports_without_websocket(monkeypatch):
    """A missing websocket package skips that one test, not the whole module."""
    pytest.importorskip("rclpy")
    monkeypatch.setitem(sys.modules, "websocket", None)
    monkeypatch.setitem(sys.modules, "websockets", None)
    sys.modules.pop("test_object_bridge", None)
    module = importlib.import_module("test_object_bridge")
    assert hasattr(module, "test_legacy_json_recovers_from_late_publisher_and_reconnects")
