"""Horizon detector and waterline range on synthetic frames (pure numpy)."""
import math

from mhseals_nav import horizon, waterline
import numpy as np
import pytest

SIZE = (640, 360)
FX = SIZE[0] / 2 / math.tan(math.radians(55.0))
K = (FX, FX, SIZE[0] / 2 - 0.5, SIZE[1] / 2 - 0.5)
CFG = {'scale': 4, 'band_deg': 6.0, 'min_grad': 3.0, 'max_slope': 1.0, 'inlier_px': 1.0,
       'iterations': 60, 'min_columns': 0.4, 'min_inliers': 0.5, 'min_span': 0.6,
       'dip_deg': 0.0}


def frame(roll, pitch, seed=0, shore_px=0, buoys=3, noise=6.0, contrast=40.0):
    """Sky (bright, smooth) over water (darker, textured), optional tree band, buoys."""
    rng = np.random.default_rng(seed)
    u, v = np.meshgrid(np.arange(SIZE[0]), np.arange(SIZE[1]))
    row = horizon.horizon_row(u, roll, pitch, K)
    below = v - row
    img = np.where(below < 0, 170.0 - 0.1 * v, 170.0 - contrast + 8 * np.sin(u / 7.0 + v / 3.0))
    if shore_px:
        img = np.where((below < 0) & (below > -shore_px), 70.0, img)
    img = img + np.clip(below, -0.5, 0.5) * 0      # hard edge, no blur
    for k in range(buoys):
        cu, cv = rng.uniform(80, 560), rng.uniform(0, 30)
        rr = (u - cu) ** 2 + (v - (np.interp(cu, u[0], row[0]) + 10 + cv)) ** 2
        img = np.where(rr < 36, 30.0, img)
    img = img + rng.normal(0, noise, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def test_horizon_row_and_line_attitude_round_trip():
    for roll, pitch in [(0.0, 0.0), (0.1, -0.05), (-0.08, 0.07)]:
        us = np.array([10.0, 600.0])
        vs = horizon.horizon_row(us, roll, pitch, K)
        r, p = horizon.attitude_from_line((us[0], vs[0]), (us[1], vs[1]), K)
        assert r == pytest.approx(roll, abs=1e-9) and p == pytest.approx(pitch, abs=1e-9)


def test_nose_down_moves_the_horizon_up():
    assert horizon.horizon_row(K[2], 0.0, 0.05, K) < K[3] < horizon.horizon_row(K[2], 0, -.05, K)


@pytest.mark.parametrize('shore', [0, 25])
def test_detect_recovers_roll_and_pitch_on_synthetic_frames(shore):
    errors = []
    rng = np.random.default_rng(3)
    for seed in range(20):
        roll, pitch = rng.uniform(-0.12, 0.12), rng.uniform(-0.08, 0.08)
        prior = (roll + rng.normal(0, 0.02), pitch + rng.normal(0, 0.02))
        fix = horizon.detect(frame(roll, pitch, seed, shore_px=shore), K, CFG, prior=prior)
        assert fix is not None
        errors.append((fix['roll'] - roll, fix['pitch'] - pitch))
    rms = np.degrees(np.sqrt(np.mean(np.square(errors), axis=0)))
    assert rms[0] < 0.3 and rms[1] < 0.15, rms


def test_detect_refuses_a_frame_without_a_horizon():
    rng = np.random.default_rng(1)
    flat = rng.normal(120, 6, (SIZE[1], SIZE[0])).clip(0, 255).astype(np.uint8)
    assert horizon.detect(flat, K, CFG) is None


WL = {'camera_height_m': 0.325, 'height_sigma_m': 0.03, 'edge_sigma_px': 1.0,
      'min_depression_deg': 0.8, 'max_range_m': 30.0, 'centre_sigma_m': 0.1}


@pytest.mark.parametrize('roll,pitch', [(0.0, 0.0), (0.06, -0.04), (-0.05, 0.05)])
def test_waterline_range_inverts_the_projection(roll, pitch):
    """A contact point on the water plane, projected by a tilted camera, ranges back."""
    R = horizon.rotation(roll, pitch)
    for rho, bearing in [(5.0, 0.3), (12.0, -0.5), (20.0, 0.1)]:
        p_level = np.array([rho * math.cos(bearing), rho * math.sin(bearing), -0.325])
        pc = R.T @ p_level
        u, v = K[2] - K[0] * pc[1] / pc[0], K[3] - K[1] * pc[2] / pc[0]
        got, sigma = waterline.mono_range((u - 3, v - 10, u + 3, v), K, roll, pitch, 0.0, WL)
        assert got == pytest.approx(rho, rel=1e-6)
        assert sigma > 0


def test_waterline_sigma_grows_with_range_squared_and_refuses_the_horizon():
    sigmas = []
    for rho in (5.0, 10.0, 20.0):
        v = K[3] + K[1] * 0.325 / rho
        sigmas.append(waterline.mono_range((300, v - 5, 340, v), K, 0, 0, 0.0, WL)[1])
    # Between linear (heave) and quadratic (angle) growth.
    assert 2.5 < sigmas[1] / sigmas[0] < 4.0 and 2.5 < sigmas[2] / sigmas[1] < 4.0
    assert waterline.mono_range((300, K[3] - 5, 340, K[3] + 0.5), K, 0, 0, 0, WL) is None


def test_attitude_interpolates_imu_and_adopts_the_horizon_bias():
    att = waterline.Attitude({'imu_sigma_deg': 1.0, 'horizon_sigma_deg': 0.25,
                              'bias_tau_s': 1.0, 'fresh_s': 0.5, 'max_gap_s': 0.05,
                              'buffer_s': 2.0})
    for k in range(200):
        t = k * 0.01
        att.add_imu(t, 0.01 * t, 0.02)      # the IMU reads pitch 0.02 rad too high
    r, p, s, src = att.at(1.005)
    assert r == pytest.approx(0.01005) and src == 'imu'
    for k in range(30):
        att.add_horizon(1.0 + k * 0.03, 0.01 * (1.0 + k * 0.03), 0.0, 0.002)
    r, p, s, src = att.at(1.9)
    assert src == 'horizon' and abs(p) < 0.004 and s == pytest.approx(math.radians(0.25))
    assert att.at(5.0) is None              # past the buffer and the gap


def test_camera_roll_pitch_from_a_quaternion():
    roll, pitch = 0.1, -0.05
    cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), \
        math.sin(pitch / 2)
    q = (sr * cp, cr * sp, -sr * sp, cr * cp)      # yaw 0, ZYX
    r, p = waterline.camera_roll_pitch(q, np.eye(3))
    assert r == pytest.approx(roll) and p == pytest.approx(pitch)
