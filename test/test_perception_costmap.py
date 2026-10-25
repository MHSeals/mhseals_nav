"""Real Nav2 costmaps: track grid -> Orin perception costmap -> B rolling StaticLayer.

Acceptance check for perception-v2 §6: a StaticLayer in a rolling costmap resamples the
odom-frame perception costmap through TF, keeps graded costs (trinary off), and follows
the robot when the rolling origin moves.
"""

from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
ROOT = Path(__file__).resolve().parents[1]


def b_local_costmap(path):
    """B's local_costmap params from nav2_params.yaml as a standalone node's file."""
    nav2 = yaml.safe_load((ROOT / 'config/nav2_params.yaml').read_text())
    params = nav2['local_costmap']['local_costmap']['ros__parameters']
    params['always_send_full_costmap'] = True   # observation only: the probe reads full maps
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    return params


def start(args, log):
    # The executable itself, not `ros2 run`, whose child would outlive terminate().
    from ament_index_python.packages import get_package_prefix
    package, executable, *rest = args
    path = Path(get_package_prefix(package)) / 'lib' / package / executable
    return subprocess.Popen([str(path)] + rest, stdout=log, stderr=subprocess.STDOUT)


def stop(procs):
    for proc in procs:
        if proc.poll() is None:
            proc.terminate()                # exact PID; never a pattern kill
    for proc in procs:
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


def cost_at(msg, x, y):
    info = msg.metadata if hasattr(msg, 'metadata') else msg.info
    i = int((x - info.origin.position.x) / info.resolution)
    j = int((y - info.origin.position.y) / info.resolution)
    width = info.size_x if hasattr(info, 'size_x') else info.width
    return msg.data[j * width + i]


def test_static_layer_in_rolling_costmaps_keeps_graded_costs_through_tf():
    from ros_domain import require_isolated_domain
    require_isolated_domain()
    from geometry_msgs.msg import TransformStamped
    from map_msgs.msg import OccupancyGridUpdate
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from tf2_ros import TransformBroadcaster

    tmp = Path(tempfile.mkdtemp())
    b_params = b_local_costmap(tmp / 'b.yaml')
    log = open(tmp / 'nodes.log', 'w')
    procs = [
        start(['nav2_costmap_2d', 'nav2_costmap_2d', '--ros-args', '-r', '__node:=costmap',
               '-r', '__ns:=/perception', '--params-file',
               str(ROOT / 'config/perception_costmap.yaml')], log),
        start(['nav2_costmap_2d', 'nav2_costmap_2d', '--ros-args', '-r', '__node:=local_costmap',
               '-r', '__ns:=/local_costmap', '--params-file', str(tmp / 'b.yaml')], log),
    ]
    # Bond ids are bare node names, so each manager lives in its node's namespace.
    procs += [start(['nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args',
                     '-r', '__node:=lifecycle_manager_test', '-r', f'__ns:=/{ns}',
                     '-p', 'autostart:=true', '-p', 'bond_timeout:=0.0',
                     '-p', f'node_names:=[{name}]'], log)
              for ns, name in (('perception', 'costmap'), ('local_costmap', 'local_costmap'))]
    rclpy.init()
    try:
        node = rclpy.create_node('costmap_probe')
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        grid_pub = node.create_publisher(OccupancyGrid, '/perception/track_grid', latched)
        got = {}
        node.create_subscription(OccupancyGrid, '/perception/costmap',
                                 lambda m: got.__setitem__('o', m), latched)

        def o_update(update):
            # O sends updates while its origin holds: apply them as B's StaticLayer does.
            o = got.get('o')
            if o is None:
                return
            data = np.array(o.data, dtype=np.int8).reshape(o.info.height, o.info.width)
            data[update.y:update.y + update.height, update.x:update.x + update.width] = \
                np.array(update.data, dtype=np.int8).reshape(update.height, update.width)
            o.data = data.ravel().tolist()
        node.create_subscription(OccupancyGridUpdate, '/perception/costmap_updates',
                                 o_update, 10)
        node.create_subscription(Costmap, '/local_costmap/costmap_raw',
                                 lambda m: got.__setitem__('b', m), 10)
        tf = TransformBroadcaster(node)
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        robot = [0.0, 0.0]

        def grid(centre):
            # Odom-frame 24 m grid like the tracker's: lethal 100 at (3, 1), graded 60 at (3, -2).
            msg = OccupancyGrid()
            msg.header.frame_id = 'odom'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.info.resolution, msg.info.width, msg.info.height = 0.15, 160, 160
            origin = np.floor((np.array(centre) - 12.0) / 0.15) * 0.15
            msg.info.origin.position.x, msg.info.origin.position.y = map(float, origin)
            msg.info.origin.orientation.w = 1.0
            data = np.zeros((160, 160), dtype=np.int8)
            for (x, y), value in (((3.0, 1.0), 100), ((3.0, -2.0), 60)):
                i, j = (np.array([x, y]) - origin) / 0.15
                data[int(j) - 2:int(j) + 3, int(i) - 2:int(i) + 3] = value
            msg.data = data.ravel().tolist()
            return msg

        def spin(seconds, until):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                t = TransformStamped()
                t.header.stamp = node.get_clock().now().to_msg()
                t.header.frame_id, t.child_frame_id = 'odom', 'base_link'
                t.transform.translation.x, t.transform.translation.y = robot
                t.transform.rotation.w = 1.0
                tf.sendTransform(t)
                ex.spin_once(timeout_sec=0.05)
                if until():
                    return True
            return False

        def b_ready():
            b = got.get('b')
            centred = b is not None and abs(
                b.metadata.origin.position.x + b_params['width'] / 2 - robot[0]) < 0.2
            return centred and cost_at(b, 3.0, 1.0) == 254

        def state():
            lines = (tmp / 'nodes.log').read_text().splitlines(True)
            return ''.join(line for line in lines if 'buffer updated' not in line)[-2500:]

        grid_pub.publish(grid(robot))
        assert spin(60.0, b_ready), state()
        o, b = got['o'], got['b']
        assert o.info.width == 160 and o.header.frame_id == 'odom'
        assert cost_at(o, 3.0, 1.0) == 100                   # lethal survives O
        assert 50 <= cost_at(o, 3.0, -2.0) <= 70             # graded, not trinary
        assert cost_at(b, 3.0, 1.0) == 254                   # lethal on B
        graded = cost_at(b, 3.0, -2.0)
        assert 120 <= graded < 253                           # graded survives B's StaticLayer
        assert b.header.frame_id == b_params['global_frame']

        # Roll: the robot moves 2 m; the same world cells keep their cost on B.
        robot[:] = [2.0, 0.0]
        grid_pub.publish(grid(robot))
        assert spin(30.0, b_ready), state()
        b = got['b']
        assert cost_at(b, 3.0, 1.0) == 254
        assert 120 <= cost_at(b, 3.0, -2.0) < 253
        assert cost_at(b, 0.0, 4.0) == 0                     # free water stays free
    finally:
        rclpy.try_shutdown()
        stop(procs)
        log.close()
        (tmp / 'nodes.log').unlink()
