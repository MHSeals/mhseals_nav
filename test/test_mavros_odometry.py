"""The real GLOBAL_POSITION_INT path must obey Odometry's body-twist contract."""
from copy import deepcopy
import math
from types import SimpleNamespace as S

import pytest

from mhseals_nav.mavros_odometry import body_odometry


def message(yaw, velocity, covariance=None):
    return S(header=S(frame_id='map', stamp=123), child_frame_id='base_link',
             pose=S(pose=S(orientation=S(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))),
             twist=S(twist=S(linear=S(x=velocity[0], y=velocity[1], z=0.),
                             angular=S(x=0., y=0., z=0.)),
                     covariance=covariance or [-1.] + [0.] * 35))


@pytest.mark.parametrize('yaw', [0., math.pi/2, math.radians(-105), math.pi])
def test_forward_motion_is_surge_at_every_heading(yaw):
    raw = message(yaw, (math.cos(yaw), math.sin(yaw)))
    original = deepcopy(raw)
    fixed = body_odometry(raw, .2)
    assert (fixed.twist.twist.linear.x, fixed.twist.twist.linear.y) == pytest.approx((1., 0.))
    assert raw == original  # never change a message used by another subscriber
    assert fixed.header == raw.header and fixed.pose == raw.pose
    assert fixed.child_frame_id == 'base_link'
    assert fixed.twist.covariance[0] == pytest.approx(.04)
    assert fixed.twist.covariance[7] == pytest.approx(.04)
    assert fixed.twist.covariance[35] > 1000  # angular velocity is not measured here


def test_known_world_covariance_rotates_with_velocity():
    cov = [0.] * 36
    cov[0], cov[7], cov[14] = .09, .16, .25
    fixed = body_odometry(message(math.pi/2, (0., 1.), cov), .2)
    assert (fixed.twist.covariance[0], fixed.twist.covariance[7]) == pytest.approx((.16, .09))


@pytest.mark.parametrize('value', [0., float('nan')])
def test_invalid_attitude_is_rejected(value):
    raw = message(0., (1., 0.))
    raw.pose.pose.orientation.w = value
    with pytest.raises(ValueError):
        body_odometry(raw, .2)
