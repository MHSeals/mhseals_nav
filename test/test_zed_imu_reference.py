"""The ZED IMU reference: gyro variance floor and tilt-compensated magnetic heading."""
import math

import pytest

from mhseals_nav.zed_imu_reference import floored_covariance, heading_yaw

# A northern-hemisphere field in world ENU: 0.24 G toward north, 0.4 G down.
FIELD = (0., 0.24, -0.4)


def rotation(yaw, pitch, roll):
    """World-from-body rotation Rz(yaw) Ry(pitch) Rx(roll) as nested tuples."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def quaternion(yaw, pitch, roll):
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def body_field(yaw, pitch, roll):
    r = rotation(yaw, pitch, roll)
    return tuple(sum(r[i][j] * FIELD[i] for i in range(3)) for j in range(3))


@pytest.mark.parametrize('yaw, pitch, roll', [
    (0., 0., 0.), (math.pi / 2, 0., 0.), (1.0, 0.1, -0.2), (-2.5, -0.15, 0.3), (3.0, 0.2, 0.2)])
def test_heading_is_the_body_yaw_through_tilt(yaw, pitch, roll):
    # The ZED orientation's own yaw is arbitrary (no magnetometer in it); only its tilt is used.
    orientation = quaternion(0.7, pitch, roll)
    assert heading_yaw(body_field(yaw, pitch, roll), orientation, 0.) == pytest.approx(yaw, abs=1e-9)


def test_east_declination_turns_the_true_heading_clockwise():
    # Bow on magnetic north: ENU yaw pi/2. True north is 5 deg east of it, so the bow is at
    # true heading 5 deg, ENU yaw pi/2 - 5 deg.
    yaw = heading_yaw(body_field(math.pi / 2, 0., 0.), quaternion(0., 0., 0.), math.radians(5))
    assert yaw == pytest.approx(math.pi / 2 - math.radians(5))


def test_a_vertical_field_has_no_heading():
    with pytest.raises(ValueError):
        heading_yaw((0., 0., -0.5), quaternion(0., 0., 0.), 0.)


def test_gyro_variance_is_floored_and_unknown_becomes_the_floor():
    floor = 0.005
    assert floored_covariance([1e-8, 0, 0, 0, 1e-8, 0, 0, 0, 1e-8], floor)[8] == floor ** 2
    assert floored_covariance([1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3], floor)[8] == 1e-3
    unknown = floored_covariance([-1, 0, 0, 0, 0, 0, 0, 0, 0], floor)
    assert unknown == [floor ** 2, 0, 0, 0, floor ** 2, 0, 0, 0, floor ** 2]


def test_floor_keeps_the_fcu_in_charge_of_the_yaw_rate():
    """MAVROS's gyro sigma (0.02 deg/s) must be far below the ZED floor."""
    from pathlib import Path
    import yaml
    config = yaml.safe_load((Path(__file__).parents[1] / 'config/zed_imu_reference.yaml')
                            .read_text())['zed_imu_reference']['ros__parameters']
    mavros_gyro_sigma = math.radians(0.02)   # MAVROS imu angular_velocity_stdev default
    assert config['gyro_stddev_floor'] >= 10 * mavros_gyro_sigma
    assert config['heading_enabled'] is False   # until the ZED magcal (owner bench list)
