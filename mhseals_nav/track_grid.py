"""Tracks as an OccupancyGrid for the perception costmap's track_layer (perception-v2 §5.7).

Pure numpy. Static, prior and unknown tracks are lethal discs, uncertainty included. A
moving track is lethal only over its hull; its uncertainty ring and the swath along its
predicted path are graded, so MPPI pays to cross ahead of it but a crossing never walls
off a gate.
"""

import math

import numpy as np

MOTION_MOVING = 2


def cells(cfg):
    return int(round(cfg['size_m'] / cfg['resolution']))


def grid_origin(centre_xy, cfg):
    """Lower-left corner, snapped to whole cells so a still object keeps its cells."""
    res, half = cfg['resolution'], cfg['size_m'] / 2
    return tuple(math.floor((c - half) / res) * res for c in centre_xy)


def _disc(grid, origin, res, xy, radius, value):
    n = grid.shape[0]
    lo = np.floor((np.asarray(xy) - radius - origin) / res).astype(int)
    hi = np.ceil((np.asarray(xy) + radius - origin) / res).astype(int)
    lo, hi = np.clip(lo, 0, n), np.clip(hi, 0, n)
    if np.any(hi <= lo):
        return
    xs = origin[0] + (np.arange(lo[0], hi[0]) + 0.5) * res
    ys = origin[1] + (np.arange(lo[1], hi[1]) + 0.5) * res
    inside = (xs[None, :] - xy[0]) ** 2 + (ys[:, None] - xy[1]) ** 2 <= radius ** 2
    block = grid[lo[1]:hi[1], lo[0]:hi[0]]
    np.maximum(block, np.where(inside, value, 0).astype(np.int8), out=block)


def _sigma(P):
    return math.sqrt(max(float(np.linalg.eigvalsh(P[:2, :2]).max()), 0.0))


def rasterize(tracks, centre_xy, cfg, now=None):
    """Return int8[H, W] (row = y) of costs 0..100 around ``centre_xy``.

    ``tracks`` carry ``class_id, motion, x=[px,py,vx,vy], P (4x4), t``; ``cfg`` is the
    tracker's ``grid`` section plus its ``radius`` table. Moving tracks are predicted
    from ``t`` to ``now`` first (the latency offset), then along the horizon.
    """
    n, res = cells(cfg), cfg['resolution']
    grid = np.zeros((n, n), dtype=np.int8)
    origin = np.array(grid_origin(centre_xy, cfg))
    radii, extra = cfg['radius'], cfg['max_extra_m']
    for track in tracks:
        radius = radii.get(track.class_id, radii['default'])
        if track.motion != MOTION_MOVING:
            _disc(grid, origin, res, track.x[:2], radius + min(2 * _sigma(track.P), extra), 100)
            continue
        latency = max((now if now is not None else track.t) - track.t, 0.0)
        p, v = track.x[:2], track.x[2:]
        # The prediction ends horizon_s after the last measurement: a stale track's swath
        # shrinks away instead of sliding ahead of it, and only its current disc remains.
        steps = int(max(cfg['horizon_s'] - latency, 0.0) / cfg['step_s'] + 1e-9)
        for k in range(steps + 1):
            t = k * cfg['step_s']
            dt = latency + t
            P = track.P[:2, :2] + dt * (track.P[:2, 2:] + track.P[2:, :2]) + \
                dt * dt * track.P[2:, 2:]
            sigma = math.sqrt(max(float(np.linalg.eigvalsh(P).max()), 0.0))
            value = cfg['ring_cost'] if k == 0 else min(99, max(
                cfg['min_cost'], int(100 * (1 - t / cfg['horizon_s']) ** cfg['gamma'])))
            _disc(grid, origin, res, p + v * dt, radius + min(2 * sigma, extra), value)
        # Only the hull is lethal (B inflates it); its uncertainty ring and swath are graded.
        _disc(grid, origin, res, p + v * latency, radius, 100)
    return grid
