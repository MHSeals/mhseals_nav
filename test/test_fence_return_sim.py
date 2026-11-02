"""Closed loop, headless: the fence's return against a station-keeping procedure outside.

A kinematic boat, a one-goal Nav2 stand-in (a new goal aborts the running one, as Nav2 does), a
command_mux stand-in (holds stop the boat) and a procedure that keeps station at a point outside
the fence and honours /autonomy/pause the way mhseals_testing's runner does. Without the pause the
procedure re-sends its goal whenever the fence's return replaced it, and the two fight.
"""

import json
import math
import threading
import time

import pytest
import yaml
from pathlib import Path

CFG = yaml.safe_load((Path(__file__).parents[1] / 'config/fence_watchdog.yaml')
                     .read_text())['/**']['ros__parameters']
BOX = [0.0, 0.0, 100.0, 0.0, 100.0, 50.0, 0.0, 50.0]
STATION = (106.0, 25.0)          # the procedure's point, outside the east edge
SPEED, RATE = 2.0, 20.0


def test_node_fence_return_pauses_station_keeping_then_holds_until_released():
    rclpy = pytest.importorskip('rclpy')
    from geometry_msgs.msg import PoseStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient, ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from ros_domain import require_isolated_domain
    require_isolated_domain()
    from mhseals_nav.fence_watchdog import node_class

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    rclpy.init()
    ex = MultiThreadedExecutor(num_threads=4)
    active = {'handle': None, 'on': True}
    try:
        dog = node_class()(parameter_overrides=[
            Parameter('fence_file', value=''), Parameter('fence_frame', value='map'),
            Parameter('fence_points', value=BOX), Parameter('on_exit', value='return')])
        world = rclpy.create_node('fence_sim_world')
        group = ReentrantCallbackGroup()
        lock = threading.Lock()
        boat = {'x': 104.0, 'y': 25.0, 'yaw': 0.0, 'cmd': (0.0, 0.0)}
        mux = {'holds': set(), 'released': False}
        log = {'procedure_goals': [], 'pauses': [], 'max_x_after_return': None}

        cmd_vel = world.create_publisher(Twist, CFG['command_topic'], 10)
        odom = world.create_publisher(Odometry, CFG['odom_topic'], 10)
        state = world.create_publisher(String, CFG['mux_state_topic'], latched)

        def on_request(msg):        # command_mux: holds stack until released
            body = json.loads(msg.data)
            with lock:
                if body['op'] == 'hold':
                    mux['holds'].add(body['cause'])
                elif body['op'] == 'release':
                    mux['holds'].discard(body['cause'])
        world.create_subscription(String, CFG['request_topic'], on_request, 10)

        def publish_state():
            with lock:
                holds = sorted(mux['holds'])
            state.publish(String(data=json.dumps(
                {'input': 'hold' if holds else 'auto', 'holds': [{'cause': c} for c in holds]})))

        def step():                 # the boat: the mux forwards auto unless a hold is on
            with lock:
                vx, vy = (0.0, 0.0) if mux['holds'] else boat['cmd']
                boat['x'] += vx / RATE
                boat['y'] += vy / RATE
                if vx or vy:        # it turns to where it goes (body velocity is surge)
                    boat['yaw'] = math.atan2(vy, vx)
            msg = Odometry()
            msg.pose.pose.position.x, msg.pose.pose.position.y = boat['x'], boat['y']
            msg.pose.pose.orientation.z = math.sin(boat['yaw'] / 2)
            msg.pose.pose.orientation.w = math.cos(boat['yaw'] / 2)
            msg.twist.twist.linear.x = math.hypot(vx, vy)
            odom.publish(msg)
        world.create_timer(1.0 / RATE, step, callback_group=group)
        world.create_timer(0.2, publish_state, callback_group=group)

        def execute(handle):        # Nav2: drive at SPEED to the goal; a newer goal aborts this
            active['handle'] = handle
            goal = handle.request.pose.pose.position
            while active['on']:
                if active['handle'] is not handle:
                    handle.abort()
                    return NavigateToPose.Result()
                if handle.is_cancel_requested:
                    handle.canceled()
                    return NavigateToPose.Result()
                dx, dy = goal.x - boat['x'], goal.y - boat['y']
                d = math.hypot(dx, dy)
                cmd = (0.0, 0.0) if d < 0.5 else (SPEED * dx / d, SPEED * dy / d)
                boat['cmd'] = cmd
                cmd_vel.publish(Twist())          # autonomy for the watchdog
                if d < 0.5:
                    handle.succeed()
                    return NavigateToPose.Result()
                time.sleep(1.0 / RATE)
            handle.abort()
            return NavigateToPose.Result()
        ActionServer(world, NavigateToPose, CFG['return_action'], execute,
                     callback_group=group, cancel_callback=lambda _: CancelResponse.ACCEPT)

        # The procedure: station keeping at STATION; paused while /autonomy/pause asks.
        client = ActionClient(world, NavigateToPose, CFG['return_action'], callback_group=group)
        proc = {'goal': None, 'paused': False, 'handle': None}

        def on_pause(msg):
            body = json.loads(msg.data)
            log['pauses'].append(body)
            proc['paused'] = body['paused']
            if body['paused'] and proc['handle'] is not None:
                proc['handle'].cancel_goal_async()
                proc['handle'] = None
        world.create_subscription(String, CFG['pause_topic'], on_pause, latched,
                                  callback_group=group)

        def send():
            pose = PoseStamped()
            pose.header.frame_id = CFG['marker_frame']
            pose.pose.position.x, pose.pose.position.y = STATION
            pose.pose.orientation.w = 1.0
            log['procedure_goals'].append(time.monotonic())
            future = client.send_goal_async(NavigateToPose.Goal(pose=pose))
            proc['goal'] = future
            future.add_done_callback(lambda f: proc.update(handle=f.result()))

        def keep_station():
            if proc['paused'] or mux['holds'] or not client.server_is_ready():
                return
            handle = proc['handle']
            done = proc['goal'] is None or (proc['goal'].done() and (
                handle is None or handle.status not in (1, 2)))   # not accepted or executing
            if done and math.dist((boat['x'], boat['y']), STATION) > 1.0:
                send()
        world.create_timer(0.5, keep_station, callback_group=group)

        ex.add_node(dog)
        ex.add_node(world)
        spinner = threading.Thread(target=ex.spin, daemon=True)
        spinner.start()

        def wait(predicate, seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end and not predicate():
                time.sleep(0.05)
            return predicate()

        # The return: paused, back inside, held by the fence, still paused.
        assert wait(lambda: any(p['paused'] for p in log['pauses']), 5.0)
        returned_at = time.monotonic()
        assert wait(lambda: 'geofence' in mux['holds'] and boat['x'] < 100.0, 15.0), boat
        assert log['pauses'][-1]['paused'], log['pauses']
        assert 'release the geofence hold' in log['pauses'][-1]['why']
        fights = [t for t in log['procedure_goals'] if t > returned_at]
        assert fights == [], 'the procedure re-sent its goal during the return'
        held_x = boat['x']
        time.sleep(1.0)
        assert boat['x'] == held_x and log['procedure_goals'][-1] < returned_at

        # The operator (U) or an agent releases the hold: the pause lifts, the procedure resumes.
        with lock:
            mux['holds'].discard('geofence')
        assert wait(lambda: not log['pauses'][-1]['paused'], 5.0), log['pauses']
        assert wait(lambda: log['procedure_goals'][-1] > returned_at, 5.0)
    finally:
        active['on'] = False        # ends the Nav2 stand-in's loop, then the executor
        time.sleep(0.2)
        ex.shutdown(timeout_sec=2.0)
        rclpy.try_shutdown()
