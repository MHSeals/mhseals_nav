"""Read one ZED JSON topic across a DDS/distro boundary, with bounded backlog."""

import json
from queue import Empty, Full, Queue
from threading import Event, Thread
from time import monotonic


class RosbridgeSource:
    """Transport only: ROS publication and validation stay on the executor."""

    def __init__(self, url, topic, logger, static_tf=False):
        import websocket

        self.websocket = websocket
        self.url, self.topic, self.logger = url, topic, logger
        self.messages = {topic: Queue(maxsize=1)}
        if static_tf:
            self.messages["/tf_static"] = Queue(maxsize=100)
        self.stop = Event()
        self.socket = None
        self.thread = Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            try:
                self.socket = self.websocket.create_connection(self.url, timeout=1.0)
                last_activity = {topic: 0.0 for topic in self.messages}
                self.logger.info(f"Object bridge connected: {self.url} {self.topic}")
                while not self.stop.is_set():
                    # Foxy may silently reject subscriptions before a publisher exists.
                    # Reusing the id replaces the subscription, avoiding duplicates.
                    for topic, last in last_activity.items():
                        if monotonic() - last >= 5.0:
                            self.socket.send(json.dumps(dict(
                                op="subscribe", id=f"objects:{topic}", topic=topic,
                                throttle_rate=100 if topic == self.topic else 0,
                                queue_length=1 if topic == self.topic else 100,
                            )))
                            last_activity[topic] = monotonic()
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
                        raise ValueError(
                            packet.get("msg", "rosbridge subscription error")
                        )
                    if (
                        packet.get("op") != "publish"
                        or packet.get("topic") not in self.messages
                    ):
                        continue
                    queue = self.messages[packet["topic"]]
                    # Static transforms need only one successful subscription.
                    last_activity[packet["topic"]] = (
                        monotonic() if packet["topic"] == self.topic else float("inf")
                    )
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
                if self.socket is not None:
                    self.socket.close()
                    self.socket = None
            self.stop.wait(2.0)

    def latest(self, topic=None):
        try:
            return self.messages[topic or self.topic].get_nowait()
        except Empty:
            return None

    def close(self):
        self.stop.set()
        self.thread.join(timeout=3.0)
