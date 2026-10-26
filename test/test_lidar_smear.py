"""lidar_smear: crane's instant scan made into a VLP16 sweep; lidar_relay's deskew undoes it."""

from types import SimpleNamespace

from mhseals_nav.lidar_relay import deskew, sweep_times
from mhseals_nav.lidar_smear import planar, smear
import numpy as np

W, V = 1.5, (1.5, 0.3)          # a fast turn while moving, base_link axes at the stamp
# A ring at 10 m off the seam behind the lidar (there a turning beam sees a point twice or never).
ANGLES = np.radians(np.arange(-165, 166, 5))
X, Y = 10 * np.cos(ANGLES), 10 * np.sin(ANGLES)


def poses(yaw0=0.7, at=(3., -2.)):
    """base_link in the world over the sweep, turning at W and moving at V (axes at the stamp)."""
    rows = []
    for t in np.linspace(-0.2, 0.05, 26):
        dx, dy = planar(0., 0., yaw0, V[0] * t, V[1] * t)
        rows.append((t, at[0] + dx, at[1] + dy, yaw0 + W * t))
    return rows


def test_each_point_is_where_the_moving_lidar_sees_it_at_its_time():
    lever = (0.29, 0.05, 0.02)
    rows = poses()
    px, py, t = smear(X, Y, rows, lever)
    assert t.min() >= -0.1 and t.max() <= 0. and np.ptp(t) > 0.08
    np.testing.assert_allclose(t, sweep_times(px, py), atol=1e-4)   # the 0.1 ms pose grid

    def lidar(at):
        b = [np.interp(at, [r[0] for r in rows], [r[i] for r in rows]) for i in (1, 2, 3)]
        return (*planar(*b, lever[0], lever[1]), b[2] + lever[2])
    world = planar(*lidar(0.), X, Y)
    seen = planar(*lidar(t), px, py)
    np.testing.assert_allclose(seen, world, atol=1e-9)
    assert np.hypot(px - X, py - Y).max() > 1.0          # w * 0.1 s * 10 m = 1.5 m of smear


def test_deskew_with_the_true_twist_undoes_the_smear():
    px, py, t = smear(X, Y, poses())
    data = np.stack([px, py, np.zeros_like(px), t], axis=1).astype('<f4')
    fields = [SimpleNamespace(name=n, offset=4 * i, datatype=7) for i, n in
              enumerate(('x', 'y', 'z', 'time'))]
    cloud = SimpleNamespace(header=None, height=1, width=len(px), fields=fields,
                            is_bigendian=False, point_step=16, row_step=16 * len(px),
                            data=data.tobytes(), is_dense=True)
    fixed = np.frombuffer(deskew(cloud, W, V).data, dtype='<f4').reshape(-1, 4)
    assert np.hypot(fixed[:, 0] - X, fixed[:, 1] - Y).max() < 2e-3   # float32 at 10 m


def test_points_without_a_return_stay_nan():
    px, py, t = smear(np.array([np.nan, 10.]), np.array([np.nan, 0.]), poses())
    assert np.isnan(px[0]) and np.isnan(py[0]) and t[0] == 0. and np.isfinite(px[1])


def test_an_empty_cloud_gives_empty_arrays():
    px, py, t = smear(np.empty(0), np.empty(0), poses())
    assert px.shape == py.shape == t.shape == (0,)
