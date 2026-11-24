"""RTK corrections from the operator station: a USB base receiver and/or public NTRIP, to MAVROS.

base: the USB receiver session and the status document; ntrip: casters, the distance-ranked mount
picker and the client; corrections: which source feeds the FCU; node: the ROS side; cli: rtk_base
and `rtk status`.
"""
