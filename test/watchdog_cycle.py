"""Bad, good, bad, good on a live watchdog node: it stays up and only ever asks for holds."""

import time


def spin(ex, feed, seconds, until=None):
    """Call feed() and spin until seconds pass or until() is true."""
    end = time.monotonic() + seconds
    while time.monotonic() < end and not (until and until()):
        feed()
        ex.spin_once(timeout_sec=0.02)


def flip_twice(ex, got, bad, good, wait=4.0):
    """Feed bad, good, bad, good; each feed publishes inputs and an autonomy command.

    Each bad phase must bring a repeated hold, each good phase none once it settles. A
    callback that raises (rclpy's logger severity check) propagates out of spin_once.
    """
    for _ in range(2):
        n = len(got)
        spin(ex, bad, wait, until=lambda: len(got) >= n + 2)
        assert len(got) >= n + 2, 'no repeated hold while bad'
        spin(ex, good, 0.5)
        n = len(got)
        spin(ex, good, 1.0)
        assert len(got) == n, 'hold requested while good'
    assert ex.context.ok()
    assert got and {g['op'] for g in got} == {'hold'}
