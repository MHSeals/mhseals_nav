import math
import os
from pathlib import Path
import re
import shutil
import signal
import struct
import subprocess
import sys
import time
from types import SimpleNamespace

from mhseals_nav.lidar_relay import decimate
from ros_domain import require_isolated_domain

ROOT = Path(__file__).parents[1]


class Cloud(SimpleNamespace):
    pass


def test_decimation_preserves_all_interleaved_vlp16_rings():
    cloud = Cloud(header=object(), height=1, width=1600, point_step=22,
                  row_step=1600*22, fields=object(), is_bigendian=False, is_dense=True,
                  data=b''.join(struct.pack('<H', i % 16)+bytes(20) for i in range(1600)))
    reduced = decimate(cloud, 80*22)
    rings = {struct.unpack_from('<H', reduced.data, i*22)[0] for i in range(reduced.width)}
    assert rings == set(range(16))
    assert len(reduced.data) <= 80*22
    assert reduced.header is cloud.header and reduced.fields is cloud.fields
    assert reduced.row_step == reduced.width*reduced.point_step


def test_reader_keeps_latest_scan_without_reliable_network_backpressure():
    from rclpy.qos import ReliabilityPolicy
    from mhseals_nav.lidar_relay import _qos

    assert _qos(False).depth == 1
    assert _qos(False).reliability == ReliabilityPolicy.BEST_EFFORT
    assert _qos().depth == 1
    assert _qos().reliability == ReliabilityPolicy.RELIABLE


def vlp16_cloud(world, yaw_rate, velocity, period=0.1):
    """Scan static world points (x, y) as a VLP16 at the origin does while turning and moving.

    The sensor sweeps azimuth once per period, ending at the stamp (t = 0); a point hit at
    time t (-period..0) is stored in the sensor frame of that moment, with its t.
    """
    from sensor_msgs.msg import PointField
    import numpy as np
    rows = []
    for x, y in world:
        # Fixed point iteration: the time the beam reaches this point's sensor-frame azimuth.
        t = -period / 2
        for _ in range(50):
            yaw, px, py = yaw_rate * t, velocity[0] * t, velocity[1] * t
            c, s = math.cos(-yaw), math.sin(-yaw)
            sx, sy = c * (x - px) - s * (y - py), s * (x - px) + c * (y - py)
            t = -period * (1 - (math.atan2(sy, sx) % (2 * math.pi)) / (2 * math.pi))
        rows.append((sx, sy, 0.5, t))
    data = np.array(rows, dtype='<f4').tobytes()
    fields = [PointField(name=n, offset=4 * i, datatype=7, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'time'))]
    return Cloud(header=object(), height=1, width=len(rows), point_step=16,
                 row_step=16 * len(rows), fields=fields, is_bigendian=False, is_dense=True,
                 data=data)


def xy(cloud):
    import numpy as np
    return np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 4)[:, :2]


def test_deskew_puts_every_point_where_it_was_at_the_stamp():
    import numpy as np
    from mhseals_nav.lidar_relay import deskew
    angles = np.linspace(-math.pi, math.pi, 72, endpoint=False)
    world = [(10 * math.cos(a), 10 * math.sin(a)) for a in angles]   # a ring of buoys at 10 m
    w, v = 1.5, (1.5, 0.)            # a manual spin while moving
    cloud = vlp16_cloud(world, w, v)
    # At the stamp the sensor is at the origin with yaw 0, so truth is the world points.
    skewed = np.abs(xy(cloud) - np.array(world)).max()
    fixed = np.abs(xy(deskew(cloud, w, v)) - np.array(world)).max()
    assert skewed > 1.0              # up to w * 0.1 s * 10 m = 1.5 m before
    assert fixed < 1e-3


def test_deskew_turns_a_tilted_offset_lidar_about_the_body():
    """A lidar 0.3 m forward and tilted 5 deg in roll and pitch: deskew in base_link axes."""
    import numpy as np
    from sensor_msgs.msg import PointField
    from mhseals_nav.lidar_relay import deskew
    from mhseals_nav.robot_geometry import rpy_matrix
    R, mount_t = rpy_matrix(np.radians(5), np.radians(-5), 0.), np.array([0.3, 0., 0.25])
    w, v = 1.5, np.array([1.5, 0.3])
    angles = np.linspace(-math.pi, math.pi, 72, endpoint=False)
    world = np.stack([10 * np.cos(angles), 10 * np.sin(angles), np.full(72, 0.5)], axis=1)
    times = np.linspace(-0.1, 0., 72)
    rows, truth = [], (world - mount_t) @ R   # sensor frame at the stamp
    for p, t in zip(world, times):
        c, s = math.cos(-w * t), math.sin(-w * t)
        d = p - np.array([v[0] * t, v[1] * t, 0.])
        body = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])
        rows.append((*((body - mount_t) @ R), t))
    fields = [PointField(name=n, offset=4 * i, datatype=7, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'time'))]
    cloud = Cloud(header=object(), height=1, width=72, point_step=16, row_step=16 * 72,
                  fields=fields, is_bigendian=False, is_dense=True,
                  data=np.array(rows, dtype='<f4').tobytes())

    def xyz(c):
        return np.frombuffer(c.data, dtype='<f4').reshape(-1, 4)[:, :3]
    assert np.abs(xyz(deskew(cloud, w, v, mount=(R, mount_t))) - truth).max() < 1e-3
    assert np.abs(xyz(deskew(cloud, w, v)) - truth).max() > 0.05   # lidar taken as base_link


def test_deskew_uses_only_a_fresh_twist():
    from mhseals_nav.lidar_relay import fresh_twist, TWIST_MAX_AGE
    twist = {'w': 0.3, 'v': (1., 0.), 'at': 100.}
    assert fresh_twist({}, 100.) is None                     # no /odom/local yet
    assert fresh_twist(twist, 100. + TWIST_MAX_AGE / 2) == (0.3, (1., 0.))
    assert fresh_twist(twist, 100.01 + TWIST_MAX_AGE) is None  # ekf_local stalled: as scanned


def test_lidar_mount_composes_tf_static_chain():
    """The deskew mount comes from /tf_static alone: rail on base_link, lidar on the rail."""
    import numpy as np
    from geometry_msgs.msg import TransformStamped
    from mhseals_nav.lidar_relay import add_static, lidar_mount
    from tf2_msgs.msg import TFMessage

    def tf(parent, child, xyz, qz=0.0):
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        t.transform.rotation.z, t.transform.rotation.w = qz, math.sqrt(1 - qz * qz)
        return t
    static, mounts = {}, {}
    add_static(static, TFMessage(transforms=[
        tf('sensor_mount_link', 'lidar_link', (0., 0., 0.26))]))
    assert lidar_mount(static, mounts, 'lidar_link') is None    # rail not yet published
    add_static(static, TFMessage(transforms=[
        tf('base_link', 'sensor_mount_link', (0.29, 0., -0.01), math.sin(math.pi / 4))]))
    R, t = lidar_mount(static, mounts, 'lidar_link')
    assert np.allclose(t, (0.29, 0., 0.25)) and np.allclose(R @ (1., 0., 0.), (0., 1., 0.))


def test_deskew_leaves_clouds_without_point_times_alone():
    from mhseals_nav.lidar_relay import deskew
    from sensor_msgs.msg import PointField
    cloud = Cloud(fields=[PointField(name=n, offset=4 * i, datatype=7, count=1)
                          for i, n in enumerate('xyz')], data=b'')
    assert deskew(cloud, 1.0, (1.0, 0.)) is cloud


def test_installed_entry_point_survives_spawned_workers(tmp_path):
    """Run the relay through the wrapper CMake installs; spawn re-runs that file as __mp_main__."""
    require_isolated_domain()
    template = re.search(r'CONTENT "(.*?)"\n', (ROOT / 'CMakeLists.txt').read_text()).group(1)
    wrapper = tmp_path / 'lib' / 'mhseals_nav' / 'lidar_relay'
    wrapper.parent.mkdir(parents=True)
    script = template.replace('${PROJECT_NAME}', 'mhseals_nav').replace('${module}', 'lidar_relay')
    wrapper.write_text(script.encode().decode('unicode_escape'))
    # A private ament prefix resolves the share directory to this checkout's config.
    (tmp_path / 'share/ament_index/resource_index/packages').mkdir(parents=True)
    (tmp_path / 'share/ament_index/resource_index/packages/mhseals_nav').touch()
    shutil.copytree(ROOT / 'config', tmp_path / 'share/mhseals_nav/config')
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join([str(ROOT), env.get('PYTHONPATH', '')])
    env['AMENT_PREFIX_PATH'] = os.pathsep.join([str(tmp_path), env.get('AMENT_PREFIX_PATH', '')])
    relay = subprocess.Popen([sys.executable, str(wrapper), '--topic', '/lidar_relay_spawn_test'],
                             env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        # Either half dying makes the parent exit, so a live parent means both workers started.
        time.sleep(8)
        assert relay.poll() is None, relay.communicate()[0]
    finally:
        if relay.poll() is None:
            relay.send_signal(signal.SIGTERM)
        output = relay.communicate(timeout=15)[0]
    assert 'bootstrapping phase' not in output, output
