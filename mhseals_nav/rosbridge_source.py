"""Read one ZED JSON topic across a DDS/distro boundary, with bounded backlog."""

import json
from queue import Empty, Full, Queue
from threading import Event, Thread
from time import monotonic, time

# Remote topics relayed for health: (throttle_rate ms, queue_length).
RELAY_TOPICS = {
    "/perception/status": (1000, 1),
    "/diagnostics": (1000, 1),
    "/rosout": (0, 20),
}
SERVICES = ("/rosapi/get_time", "/rosapi/nodes")
SERVICE_PERIOD = 10.0


class RosbridgeSource:
    """Transport only: ROS publication and validation stay on the executor."""

    def __init__(self, url, topic, logger, static_tf=False, relay=False):
        import websocket

        self.websocket = websocket
        self.url, self.topic, self.logger = url, topic, logger
        # topic -> (throttle_rate, queue_length), also the queue bound.
        self.limits = {topic: (100, 1)}
        if static_tf:
            self.limits["/tf_static"] = (0, 100)
        if relay:
            self.limits.update(RELAY_TOPICS)
        self.messages = {t: Queue(maxsize=n) for t, (_, n) in self.limits.items()}
        self.relay = relay
        # Health facts for the converter; the thread only ever rebinds values.
        self.connected = False
        self.packets = 0
        self.remote = {}
        self.calls = {}
        self.stop = Event()
        self.socket = None
        self.thread = Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            try:
                self.socket = self.websocket.create_connection(self.url, timeout=1.0)
                last_activity = {topic: 0.0 for topic in self.messages}
                self.connected = True
                self.calls = {}
                last_call = float("-inf")
                self.logger.info(f"Object bridge connected: {self.url} {self.topic}")
                while not self.stop.is_set():
                    # Foxy may silently reject subscriptions before a publisher exists.
                    # Reusing the id replaces the subscription, avoiding duplicates.
                    for topic, last in last_activity.items():
                        if monotonic() - last >= 5.0:
                            rate, length = self.limits[topic]
                            self.socket.send(json.dumps(dict(
                                op="subscribe", id=f"objects:{topic}", topic=topic,
                                throttle_rate=rate, queue_length=length,
                            )))
                            last_activity[topic] = monotonic()
                    if self.relay and monotonic() - last_call >= SERVICE_PERIOD:
                        last_call = monotonic()
                        self.call_services()
                    try:
                        raw = self.socket.recv()
                    except self.websocket.WebSocketTimeoutException:
                        continue
                    if not raw:
                        break
                    if len(raw) > 2_000_000:
                        raise ValueError("oversized object message")
                    packet = json.loads(raw)
                    if not isinstance(packet, dict):
                        continue
                    if packet.get("op") == "status" and packet.get("level") == "error":
                        if not self.is_side_channel(packet.get("id")):
                            raise ValueError(
                                packet.get("msg", "rosbridge subscription error")
                            )
                        # Health calls (e.g. rosapi absent) must never drop the objects.
                        self.logger.debug(f"Health side-channel error: {packet.get('msg')}")
                        continue
                    if packet.get("op") == "service_response":
                        self.on_response(packet)
                        continue
                    if (
                        packet.get("op") != "publish"
                        or packet.get("topic") not in self.messages
                    ):
                        continue
                    queue = self.messages[packet["topic"]]
                    # Only the objects need a refresh; other topics subscribe once.
                    last_activity[packet["topic"]] = (
                        monotonic() if packet["topic"] == self.topic else float("inf")
                    )
                    self.packets += packet["topic"] == self.topic
                    try:
                        queue.put_nowait(packet["msg"])
                    except Full:
                        try:
                            queue.get_nowait()
                        except Empty:
                            pass
                        queue.put_nowait(packet["msg"])
            except Exception as error:
                if not self.stop.is_set():
                    self.logger.warning(f"Object bridge reconnecting: {error}")
            finally:
                self.connected = False
                if self.socket is not None:
                    self.socket.close()
                    self.socket = None
            self.stop.wait(2.0)

    def is_side_channel(self, packet_id):
        """True for rosapi calls and relay subscriptions; only the objects are fatal."""
        packet_id = str(packet_id)
        return packet_id.startswith("call:") or (
            self.relay and packet_id in {f"objects:{t}" for t in RELAY_TOPICS})

    def call_services(self):
        """Ask rosapi for the peer clock and node list; answers arrive as packets."""
        for name in SERVICES:
            self.calls[name] = time()
            self.socket.send(json.dumps(dict(
                op="call_service", id=f"call:{name}", service=name, args={},
            )))

    def on_response(self, packet):
        """Fold a rosapi answer into self.remote (offset = peer clock - ours)."""
        name = str(packet.get("id", "")).removeprefix("call:")
        sent = self.calls.pop(name, None)
        values = packet.get("values")
        if sent is None or not packet.get("result", True) or not isinstance(values, dict):
            return
        now = time()
        if name == "/rosapi/get_time":
            stamp = values.get("time")
            if not isinstance(stamp, dict):
                return  # Foxy's reply shape is unverified; drop what we cannot read.
            try:
                peer = int(stamp.get("secs", stamp.get("sec", 0))) \
                    + int(stamp.get("nsecs", stamp.get("nanosec", 0))) * 1e-9
            except (TypeError, ValueError):
                return
            self.remote = {**self.remote,
                           "clock_offset_ms": (peer - (sent + now) / 2) * 1e3,
                           "clock_rtt_ms": (now - sent) * 1e3}
        else:
            nodes = values.get("nodes", [])
            self.remote = {**self.remote,
                           "remote_nodes": [str(n) for n in nodes] if isinstance(nodes, list) else []}

    def latest(self, topic=None):
        try:
            return self.messages[topic or self.topic].get_nowait()
        except Empty:
            return None

    def close(self):
        self.stop.set()
        self.thread.join(timeout=3.0)
