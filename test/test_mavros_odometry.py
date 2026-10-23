"""The real GLOBAL_POSITION_INT path must obey Odometry's body-twist contract."""
from copy import deepcopy
import math
from types import SimpleNamespace as S

import pytest

from mhseals_nav.mavros_odometry import Adapter, body_odometry, gps_velocity, with_velocity


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


def test_gps_velocity_replaces_the_fcu_velocity_before_rotation():
    raw = message(math.pi / 2, (-1.1, -1.6))          # the FCU's drifting estimate
    fixed = body_odometry(with_velocity(raw, (0., .5, 0.), .3), .3)
    assert (fixed.twist.twist.linear.x, fixed.twist.twist.linear.y) == pytest.approx((.5, 0.))
    assert fixed.twist.covariance[0] == pytest.approx(.09)
    assert raw.twist.twist.linear.x == -1.1           # the original message is untouched


def sample(cog, vel=100, fix_type=3, vel_acc=210):
    return S(fix_type=fix_type, vel=vel, cog=cog, vel_acc=vel_acc)


@pytest.mark.parametrize('cog, east, north', [(0, 0., 1.), (9000, 1., 0.), (18000, 0., -1.)])
def test_gps_course_is_converted_to_enu(cog, east, north):
    velocity, _ = gps_velocity(sample(cog), .05, 1., .3)
    assert velocity == pytest.approx((east, north, 0.), abs=1e-9)


def test_gps_sigma_is_the_receivers_accuracy_with_a_floor():
    assert gps_velocity(sample(0, vel_acc=210), .05, 1., .3)[1] == pytest.approx(.21)
    assert gps_velocity(sample(0, vel_acc=10), .05, 1., .3)[1] == .05


def test_gps_zero_accuracy_means_unknown_not_perfect():
    assert gps_velocity(sample(0, vel_acc=0), .05, 1., .3)[1] == .3


@pytest.mark.parametrize('raw', [sample(0, fix_type=2), sample(0, vel=65535), sample(65535)])
def test_gps_without_a_usable_measurement_is_a_wide_zero_prior(raw):
    assert gps_velocity(raw, .05, 1., .3) == ((0., 0., 0.), 1.)


LIVE = dict(velocity_stddev=.3, gps_velocity_max_age=1.5, gps_velocity_min_stddev=.05,
            velocity_stddev_nofix=1.)


def raw_sample(sec, nanosec=0, **kwargs):
    return S(header=S(stamp=S(sec=sec, nanosec=nanosec)), **vars(sample(9000, **kwargs)))


def test_each_gps_sample_is_published_once_in_the_gps_frame():
    adapter = Adapter(LIVE, 'gps', 'gps_link')
    adapter.receive_gps(raw_sample(10), 10.)
    first = adapter.receive(message(0., (5., 5.)), 10.05)
    assert first.child_frame_id == 'gps_link'
    assert first.twist.twist.linear.x == pytest.approx(1.)  # GPS east, not the FCU's 5
    assert adapter.receive(message(0., (5., 5.)), 10.1) is None
    adapter.receive_gps(raw_sample(10, 200000000), 10.2)
    assert adapter.receive(message(0., (5., 5.)), 10.25) is not None


def test_stale_gps_velocity_publishes_the_zero_prior_instead_of_going_silent():
    adapter = Adapter(LIVE, 'gps', 'gps_link')
    adapter.receive_gps(raw_sample(10), 10.)
    # Every FCU message after the receiver stops carries v=0 with the no-fix sigma,
    # so the EKF decays to rest instead of holding the last velocity.
    for now in (11.6, 12., 40.):
        result = adapter.receive(message(0., (1., 0.)), now)
        assert result.child_frame_id == 'gps_link'
        assert result.twist.twist.linear.x == 0.
        assert result.twist.covariance[0] == pytest.approx(1.)
    assert adapter.receive(message(0., (1., 0.)), 5.).twist.covariance[0] == pytest.approx(1.)  # no sample yet


def test_ordinary_gps_gaps_at_5_hz_send_no_zero_prior():
    adapter = Adapter(LIVE, 'gps', 'gps_link')
    adapter.receive_gps(raw_sample(10), 10.)
    assert adapter.receive(message(0., (1., 0.)), 10.05) is not None
    # Field gaps reach 1.13 s: every FCU message in one is skipped, not a phantom stop.
    assert all(adapter.receive(message(0., (1., 0.)), 10.05 + .05 * i) is None for i in range(1, 22))
    adapter.receive_gps(raw_sample(11, 130000000), 11.13)
    assert adapter.receive(message(0., (1., 0.)), 11.15).twist.twist.linear.x == pytest.approx(1.)


@pytest.mark.parametrize('header_sec, stamped', [(10, True), (500, False)])
def test_gps_header_is_the_stamp_only_when_it_agrees_with_arrival(header_sec, stamped):
    adapter = Adapter(LIVE, 'gps', 'gps_link')
    adapter.receive_gps(raw_sample(header_sec), 10.)
    result = adapter.receive(message(0., (1., 0.)), 10.05)
    assert (result.header.stamp != 123) is stamped


def test_fcu_source_keeps_the_fcu_velocity_and_base_link():
    result = Adapter(LIVE, 'fcu', 'gps_link').receive(message(0., (2., 0.)), 10.)
    assert result.child_frame_id == 'base_link'
    assert result.twist.twist.linear.x == pytest.approx(2.)
