"""Isolated mavros_setup check: live message_rates sets go through the startup rule.

Runs the real node against a fake loopback /mavros/set_message_interval service; it never
talks to an FCU. Run with ROS_DOMAIN_ID set to an isolated domain and ROS_LOCALHOST_ONLY=1.
"""
import subprocess
import sys
import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

from ros_domain import require_isolated_domain

require_isolated_domain()
rclpy.init()
probe = rclpy.create_node('rate_probe')
requests = []


def answer(request, response):
    requests.append((request.message_id, request.message_rate))
    response.success = True
    return response


probe.create_service(MessageInterval, '/mavros/set_message_interval', answer)
state = probe.create_publisher(State, '/mavros/state', 10)
get = probe.create_client(GetParameters, '/mavros_setup/get_parameters')
put = probe.create_client(SetParameters, '/mavros_setup/set_parameters')


def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(probe, timeout_sec=.05)


def call(client, request):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(probe, future, timeout_sec=5)
    return future.result()


def read():
    return list(call(get, GetParameters.Request(names=['message_rates'])).values[0].string_array_value)


def write(entries):
    value = ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=entries)
    result = call(put, SetParameters.Request(
        parameters=[Parameter(name='message_rates', value=value)])).results[0]
    return result.successful, result.reason


node = subprocess.Popen([sys.executable, '-c', 'from mhseals_nav.mavros_setup import main; main()'])
try:
    assert get.wait_for_service(10) and put.wait_for_service(10), 'mavros_setup did not start'
    start = read()
    state.publish(State(connected=True))
    spin(1.5)
    assert len(requests) == len(start), (requests, start)  # startup edge asks once
    requests.clear()
    for bad in ([], ['33:5', '33:6'], ['33:-5']):
        ok, reason = write(bad)
        spin(.5)
        assert not ok and reason.startswith('message_rates '), (bad, ok, reason)
        assert read() == start, (bad, read())
        assert requests == [], (bad, requests)
    ok, reason = write(['33:10', '31:0'])
    spin(1.5)
    assert ok, reason
    assert read() == ['33:10', '31:0'] and requests == [(33, 10.), (31, 0.)], (read(), requests)
    print('mavros_setup live message_rates: invalid refused, readback unchanged, valid requested')
finally:
    node.terminate()
    node.wait(timeout=10)
    probe.destroy_node()
    rclpy.shutdown()
