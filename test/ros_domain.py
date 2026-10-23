"""Guard shared by the ROS tests: run on an isolated domain, whichever one the runner picked."""

import os

# Domain 0 is the default (shared with the boat network); Fast DDS rejects domains above 232.
MAX_DOMAIN = 232


def require_isolated_domain(environ=None):
    """Return the ROS domain id, failing unless it is an isolated one (1..232)."""
    raw = (os.environ if environ is None else environ).get("ROS_DOMAIN_ID", "")
    assert raw.isdigit() and 1 <= int(raw) <= MAX_DOMAIN, f"isolate ROS tests: ROS_DOMAIN_ID={raw!r}"
    return int(raw)
