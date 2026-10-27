"""The mission's speed profile: fast in open water, slower on approach and near buoys.

One cap for the whole run, by segment, from what is known around the boat (every track
the host holds, course-map priors included):

- ``near``: a known object within ``near_m``, any side (inside a task: gates, circles,
  the marina; leaving one just passed);
- ``approach``: a known object within ``approach_m`` ahead (within ``ahead_deg`` of the
  heading);
- ``open``: neither, and the track feed fresh (the straightaways between tasks, search
  legs across open water).

Not knowing is not open water: with no tracker message for ``stale_s`` (the tracker died,
or perception has not reported yet) the cap is the slowest, ``near``, as tracks are
forgotten a few seconds after their last sighting. And after ``near`` or ``approach`` the
boat keeps that speed for ``dwell_s`` and ``dwell_m`` before going faster: a buoy just
passed, or one that flickers out of the tracker, does not speed it up at once.

It is a cap (tasks/behaviours.py SpeedCap, Nav2's /speed_limit), never more than the
controller's own limit: a tree's tighter cap (the debris gates) still wins inside it.
Loading refuses a profile above the speed ceiling (config/nav2_ceilings.yaml vx_max),
as the launch refuses limits above it, or one whose speeds cannot stop in time: after
``stop.latency_s`` at full speed and a stop at ``stop.decel_mps2`` (the velocity
smoother's deceleration), ``open`` must stand still ``stop.margin_m`` short of an object
first seen at ``stop.detect_m`` or entering ``approach_m``, and ``approach`` short of
one at ``near_m``. The host never sends a cap above the controller's own limit (its
vx_max as launched: nav2_params.yaml, or the launch's limits overlay), so with today's
field limit the profile only ever caps at or below it.
"""

import math

from mhseals_nav.navigation_config import load_ceilings
from mhseals_nav.tasks.params import config_path

KEYS = {'open', 'approach', 'near', 'approach_m', 'near_m', 'ahead_deg', 'stale_s', 'dwell_s',
        'dwell_m', 'stop'}
STOP = {'decel_mps2', 'latency_s', 'margin_m', 'detect_m'}


def ceiling():
    """Return the forward speed ceiling in m/s: config/nav2_ceilings.yaml vx_max."""
    return float(load_ceilings(config_path('../nav2_ceilings.yaml'))['vx_max'])


def stopping_m(speed, stop):
    """Return the metres covered from ``speed`` m/s: latency at speed, then the stop."""
    return speed * stop['latency_s'] + speed * speed / (2.0 * stop['decel_mps2'])


def load(cfg, vx_ceiling):
    """Return the validated profile ``cfg``; ``vx_ceiling`` is the forward speed ceiling (m/s)."""
    if set(cfg) != KEYS or not isinstance(cfg['stop'], dict) or set(cfg['stop']) != STOP:
        raise ValueError(f'speed_profile: expected keys {sorted(KEYS)} and stop '
                         f'{sorted(STOP)}, got {sorted(cfg)}')
    for key, value in [*cfg.items(), *(('stop.' + k, v) for k, v in cfg['stop'].items())]:
        if key != 'stop' and (isinstance(value, bool) or not isinstance(value, (int, float))
                              or not math.isfinite(value) or value <= 0):
            raise ValueError(f'speed_profile.{key}: positive finite number, got {value!r}')
    if not cfg['near'] <= cfg['approach'] <= cfg['open'] <= vx_ceiling:
        raise ValueError(f'speed_profile: need near <= approach <= open <= the vx_max ceiling '
                         f'{vx_ceiling:g} m/s')
    if not cfg['near_m'] < cfg['approach_m']:
        raise ValueError('speed_profile: need near_m < approach_m')
    stop, margin = cfg['stop'], cfg['stop']['margin_m']
    for speed, room, what in (
            (cfg['open'], min(stop['detect_m'], cfg['approach_m']), 'open'),
            (cfg['approach'], cfg['near_m'], 'approach')):
        if stopping_m(speed, stop) + margin > room:
            raise ValueError(f'speed_profile.{what}: {speed:g} m/s needs '
                             f'{stopping_m(speed, stop):.1f} m to stop, more than '
                             f'{room:g} m less the {margin:g} m margin')
    return cfg


def segment(cfg, pose, tracks):
    """Return the segment's speed in m/s for a boat at ``pose`` (x, y, yaw) among ``tracks``."""
    x, y, yaw = pose
    half = math.radians(cfg['ahead_deg'])
    speed = cfg['open']
    for track in tracks or ():
        gap = math.hypot(track.x - x, track.y - y)
        if gap <= cfg['near_m']:
            return cfg['near']
        bearing = math.atan2(track.y - y, track.x - x) - yaw
        if gap <= cfg['approach_m'] and abs(math.atan2(math.sin(bearing),
                                                       math.cos(bearing))) <= half:
            speed = cfg['approach']
    return speed


class Profile:
    """The run's cap, by segment, held at a slower segment's speed for the dwell."""

    def __init__(self, cfg):
        self.cfg, self.held = cfg, None  # held: (speed, x, y, since) of the last slow segment

    def cap(self, pose, tracks, now, heard_at):
        """Return the cap in m/s; ``heard_at`` is the last tracker message's time, or None."""
        cfg = self.cfg
        if pose is None:
            return cfg['near']
        stale = heard_at is None or now - heard_at > cfg['stale_s']
        speed = cfg['near'] if stale else segment(cfg, pose, tracks)
        if self.held is not None:
            _, x, y, since = self.held
            if (now - since >= cfg['dwell_s']
                    and math.hypot(pose[0] - x, pose[1] - y) >= cfg['dwell_m']):
                self.held = None
        if speed < cfg['open'] and (self.held is None or speed <= self.held[0]):
            self.held = (speed, pose[0], pose[1], now)
        return min(speed, self.held[0]) if self.held is not None else speed
