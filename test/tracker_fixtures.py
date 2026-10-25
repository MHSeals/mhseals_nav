"""Duck-typed observations and a fake TF for the pure tracker tests."""

import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import yaml

CFG = yaml.safe_load((Path(__file__).parents[1] / 'config/tracker.yaml').read_text())[
    '/**']['ros__parameters']


def ob(cls, x, y, backend_id='', score=0.9, velocity=None):
    kind, v = (0, (0.0, 0.0)) if velocity is None else (2, velocity)
    return NS(class_id=cls, score=score, backend_id=backend_id,
              position=NS(x=x, y=y, z=0.0), position_covariance=[0.0] * 9,
              size=NS(x=0.3, y=0.3, z=0.3), velocity_kind=kind,
              velocity=NS(x=v[0], y=v[1], z=0.0), velocity_covariance=[0.0] * 9)


def frame(t, obs, frame_id='cam', backend='sim'):
    sec = math.floor(t)
    return NS(header=NS(stamp=NS(sec=sec, nanosec=int(round((t - sec) * 1e9))),
                        frame_id=frame_id), backend=backend, observations=obs)


def fixed_pose(t, frame_id):
    """Camera at the odom origin looking along +x."""
    return np.eye(3), np.zeros(3)


def yawing_pose(yaw_at):
    def pose_at(t, frame_id):
        yaw = yaw_at(t)
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), np.zeros(3)
    return pose_at
