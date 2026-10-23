"""Build /perception/status stages for the ZED perception routes (nothing here touches ROS I/O)."""

from collections import deque
from urllib.parse import urlparse

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

SOURCE = "legacy_zed"
PREFIX = f"perception/{SOURCE}/"
WARN_LEVEL = 30  # rcl_interfaces/Log WARN; also catches ERROR and FATAL.


def status(name, host, values, level=DiagnosticStatus.OK, message="", source=SOURCE):
    """One DiagnosticStatus; values are rendered verbatim as strings."""
    out = DiagnosticStatus(
        name=f"perception/{source}/{name}", hardware_id=host, level=level, message=message
    )
    out.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
    return out


def relayed(packet, zed_only=False, source=SOURCE):
    """Remote statuses to forward in the source namespace; /diagnostics keeps ZED only."""
    prefix = f"perception/{source}/"
    out = []
    for item in packet.get("status", []):
        name, host = str(item.get("name", "")), str(item.get("hardware_id", ""))
        if not name or (zed_only and not host.startswith("Stereolabs")):
            continue
        status_out = DiagnosticStatus(
            name=prefix + (name[len(PREFIX):] if name.startswith(PREFIX) else name),
            hardware_id=host, message=str(item.get("message", "")),
            level=int(item.get("level", 0)).to_bytes(1, "little"),
        )
        status_out.values = [KeyValue(key=str(v["key"]), value=str(v["value"]))
                             for v in item.get("values", [])]
        out.append(status_out)
    return out


def warnings(packets):
    """Remote log lines at WARN or above, from rosout JSON."""
    return [p["msg"] for p in packets
            if isinstance(p.get("level"), int) and p["level"] >= WARN_LEVEL and p.get("msg")]


class StageClock:
    """Rate, latency and drop counters for the converter stage."""

    def __init__(self, window=10.0):
        self.window = window
        self.accepted = deque()  # (time, latency_s)
        self.drops = dict(stale=0, reorder=0, malformed=0)

    def accept(self, now, latency):
        self.accepted.append((now, latency))

    def drop(self, kind):
        self.drops[kind] += 1

    def values(self, now):
        while self.accepted and now - self.accepted[0][0] > self.window:
            self.accepted.popleft()
        out = {f"drop_{k}": v for k, v in self.drops.items()}
        if self.accepted:
            lat = sorted(l for _, l in self.accepted)
            out.update(hz=f"{len(lat) / self.window:.2f}",
                       latency_ms=f"{lat[len(lat) // 2] * 1e3:.0f}",
                       latency_ms_p95=f"{lat[min(len(lat) - 1, int(.95 * len(lat)))] * 1e3:.0f}")
        else:
            out["hz"] = "0.00"
        return out


class DetectorStage:
    """Per-second frame statistics for the detector stage; values() starts a new window."""

    def __init__(self, start):
        self.start = start
        self.frames = []  # (proc_s, latency_s, views, objects)
        self.drops = dict(stale=0)

    def accept(self, proc, latency, views, objects):
        self.frames.append((proc, latency, views, objects))

    def drop(self, kind):
        self.drops[kind] += 1

    def values(self, now):
        count, span = len(self.frames), max(now - self.start, 1e-6)
        out = dict(hz=f"{count / span:.2f}", **{f"drop_{k}": v for k, v in self.drops.items()})
        if count:
            proc, latency, views, objects = zip(*self.frames)
            out.update(proc_ms=f"{sum(proc) / count * 1e3:.0f}",
                       proc_ms_max=f"{max(proc) * 1e3:.0f}",
                       latency_ms=f"{sorted(latency)[count // 2] * 1e3:.0f}",
                       views=max(views), objects=f"{sum(objects) / count:.1f}")
        self.start, self.frames = now, []
        return out


def host(url):
    return urlparse(url).hostname or ""


def bridge_values(source):
    """Link state and rosapi facts; the caller adds the packet rate."""
    facts = dict(connected=str(source.connected).lower(), peer=urlparse(source.url).netloc)
    facts.update({k: (",".join(v) if isinstance(v, list) else f"{v:.1f}")
                  for k, v in source.remote.items()})
    return facts
