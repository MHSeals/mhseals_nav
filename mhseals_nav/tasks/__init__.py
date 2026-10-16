"""RoboBoat tasks as py_trees behaviour trees on top of Nav2.

Nav2's XML trees decide *how to drive* (plan, follow, recover); these trees
decide *what to do* (which gates, which beacon, when to report). Leaves are
plain Python over a blackboard the runner fills with tracked objects, the
boat's pose and the time, so each one is testable without ROS.
"""
