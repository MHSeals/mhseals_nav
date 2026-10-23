#!/usr/bin/env python3
"""Compare /uas1/mavlink_source rates in a bag with the odom.launch request list.

Usage: check_rates_bag.py BAG_DIR   (needs ROS; exits 1 when an id is short)
"""
import collections
import sys

import rosbag2_py
from mavros_msgs.msg import Mavlink
from rclpy.serialization import deserialize_message

# Mirrors create_mavros_setup_node in launch/odom.launch.py at its default rates.
WANTED = {33: 20., 30: 20., 31: 20., 24: 5., 27: 10., 1: 2., 193: 2., 65: 25.}


def main(bag):
    from mhseals_nav.mavros_setup import rate_shortfalls
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/uas1/mavlink_source']))
    counts, stamps = collections.Counter(), []
    while reader.has_next():
        _, data, stamp = reader.read_next()
        msg = deserialize_message(data, Mavlink)
        if msg.sysid == 1 and msg.compid == 1:  # the FCU, not a routed GCS
            counts[msg.msgid] += 1
            stamps.append(stamp / 1e9)
    if not stamps:
        sys.exit('no FCU messages on /uas1/mavlink_source')
    short = rate_shortfalls(counts, max(stamps) - min(stamps) or 1., WANTED)
    for i, want, got in short:
        print(f'msg {i}: wanted {want:g} Hz, got {got:.1f} Hz')
    print('all requested rates confirmed' if not short else f'{len(short)} short')
    sys.exit(bool(short))


if __name__ == '__main__':
    main(sys.argv[1])
