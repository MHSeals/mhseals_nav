"""Build /perception/status stages (nothing here touches ROS I/O)."""

from collections import deque

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def status(name, host, values, level=DiagnosticStatus.OK, message="", *, source):
    """One DiagnosticStatus named perception/<source>/<name>; values rendered as strings."""
    out = DiagnosticStatus(
        name=f"perception/{source}/{name}", hardware_id=host, level=level, message=message
    )
    out.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
    return out


class StageClock:
    """Rate, latency, object count and drop counters over a sliding window."""

    def __init__(self, window=10.0):
        self.window = window
        self.accepted = deque()  # (time, latency_s, objects)
        self.drops = dict(stale=0, reorder=0, malformed=0)

    def accept(self, now, latency, objects=0):
        self.accepted.append((now, latency, objects))

    def drop(self, kind):
        self.drops[kind] += 1

    def values(self, now):
        while self.accepted and now - self.accepted[0][0] > self.window:
            self.accepted.popleft()
        out = {f"drop_{k}": v for k, v in self.drops.items()}
        if self.accepted:
            lat = sorted(a[1] for a in self.accepted)
            count = len(lat)
            out.update(hz=f"{count / self.window:.2f}",
                       latency_ms=f"{lat[count // 2] * 1e3:.0f}",
                       latency_ms_p95=f"{lat[min(count - 1, int(.95 * count))] * 1e3:.0f}",
                       proc_ms=f"{sum(lat) / count * 1e3:.0f}",
                       objects=f"{sum(a[2] for a in self.accepted) / count:.1f}")
        else:
            out["hz"] = "0.00"
        return out
