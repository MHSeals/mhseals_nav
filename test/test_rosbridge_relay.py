"""RosbridgeSource health relay against a scripted rosbridge peer."""

import json
import logging
import sys
import time
import types

import pytest

from mhseals_nav.rosbridge_source import RosbridgeSource


class Timeout(Exception):
    pass


class FakeSocket:
    """Scripted rosbridge peer whose clock runs 2 s ahead of ours."""

    def __init__(self, inbox):
        self.inbox, self.sent, self.reply = list(inbox), [], None

    def send(self, raw):
        packet = json.loads(raw)
        self.sent.append(packet)
        if packet["op"] != "call_service":
            return
        if self.reply is not None:
            self.inbox.append(json.dumps(dict(op="service_response", id=packet["id"],
                                              values=self.reply, result=True)))
            return
        peer = time.time() + 2.0
        values = (dict(time=dict(secs=int(peer), nsecs=int(peer % 1 * 1e9)))
                  if packet["service"].endswith("get_time")
                  else dict(nodes=["/front/zed_node", "/rosapi"]))
        self.inbox.append(json.dumps(dict(op="service_response", id=packet["id"],
                                          service=packet["service"], values=values, result=True)))

    def recv(self):
        if self.inbox:
            return self.inbox.pop(0)
        time.sleep(0.02)
        raise Timeout

    def close(self):
        pass


@pytest.fixture
def fake_websocket(monkeypatch):
    module = types.SimpleNamespace(WebSocketTimeoutException=Timeout)
    module.sockets = []

    def create_connection(url, timeout):
        module.sockets.append(FakeSocket(module.inbox))
        module.sockets[-1].reply = module.reply
        return module.sockets[-1]

    module.create_connection, module.inbox, module.reply = create_connection, [], None
    monkeypatch.setitem(sys.modules, "websocket", module)
    return module


def publish(topic, msg):
    return json.dumps(dict(op="publish", topic=topic, msg=msg))


def wait_for(check, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if check():
            return
        time.sleep(0.02)
    raise AssertionError("timed out")


def test_source_subscribes_to_health_topics_and_measures_the_clock(fake_websocket):
    source = RosbridgeSource("ws://squirtle:9090", "/objects", logging.getLogger(), relay=True)
    try:
        wait_for(lambda: {"clock_offset_ms", "remote_nodes"} <= source.remote.keys())
    finally:
        source.close()
    sent = fake_websocket.sockets[0].sent
    subscribed = {p["topic"]: p for p in sent if p["op"] == "subscribe"}
    assert subscribed["/rosout"]["throttle_rate"] == 0 and subscribed["/rosout"]["queue_length"] == 20
    assert subscribed["/diagnostics"]["throttle_rate"] == 1000
    assert "/perception/status" in subscribed
    assert source.remote["clock_offset_ms"] == pytest.approx(2000, abs=100)
    assert source.remote["remote_nodes"] == ["/front/zed_node", "/rosapi"]
    assert not source.connected


def test_source_without_relay_keeps_the_original_subscriptions(fake_websocket):
    source = RosbridgeSource("ws://squirtle:9090", "/objects", logging.getLogger())
    try:
        wait_for(lambda: fake_websocket.sockets)
        time.sleep(0.2)
    finally:
        source.close()
    assert [p["topic"] for p in fake_websocket.sockets[0].sent] == ["/objects"]




def test_unreadable_rosapi_reply_and_side_channel_errors_keep_the_link_up(fake_websocket):
    fake_websocket.reply = dict(time="soon", nodes=7)
    fake_websocket.inbox.append(json.dumps(dict(op="status", level="error", id="call:/rosapi/nodes",
                                                msg="service does not exist")))
    fake_websocket.inbox.append(json.dumps(dict(op="status", level="error", id="objects:/rosout",
                                                msg="no publisher")))
    fake_websocket.inbox.append(publish("/objects", dict(n=1)))
    source = RosbridgeSource("ws://squirtle:9090", "/objects", logging.getLogger(), relay=True)
    try:
        wait_for(lambda: source.latest() is not None)
        wait_for(lambda: source.remote.get("remote_nodes") == [])
        assert source.connected and len(fake_websocket.sockets) == 1
    finally:
        source.close()
    assert "clock_offset_ms" not in source.remote


def test_objects_subscription_error_still_reconnects(fake_websocket):
    fake_websocket.inbox.append(json.dumps(dict(op="status", level="error", id="objects:/objects",
                                                msg="bad topic")))
    source = RosbridgeSource("ws://squirtle:9090", "/objects", logging.getLogger(), relay=True)
    try:
        wait_for(lambda: len(fake_websocket.sockets) == 1 and not source.connected)
    finally:
        source.close()
