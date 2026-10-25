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
