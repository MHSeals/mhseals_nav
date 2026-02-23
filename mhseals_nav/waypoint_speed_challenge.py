#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mavros_msgs.srv import SetMode, CommandBool, WaypointSetCurrent
from mavros_msgs.msg import State
from zed_msgs.msg import ObjectsStamped

# --- Mission waypoint start indices ---
# Must match sequence numbers uploaded via Mission Planner / QGC.
# Index 0 is HOME; actual mission items start at 1.
SEQ_A_START = 1  # isRed = False (Green)
SEQ_B_START = 13  # isRed = True  (Red)

# --- Detection config ---
# Label strings must match exactly what your ZED object detection model outputs.
# Adjust these if your custom model uses different class names.
RED_LABEL   = "Red"
GREEN_LABEL = "Green"


class WayPointSpeedChallenge(Node):

    def __init__(self):
        super().__init__('mission_switcher')

        # --- State ---
        self.fcu_connected   = False
        self.mission_started = False

        # QoS for MAVROS state — must be BEST_EFFORT to match publisher
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS for ZED — reliable, standard depth
        zed_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscriptions ---
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self._state_cb,
            mavros_qos
        )

        self.objects_sub = self.create_subscription(
            ObjectsStamped,
            '/front/zed_node/obj_det/objects',
            self._objects_cb,
            zed_qos
        )

        # --- MAVROS service clients ---
        self.set_mode_client = self.create_client(SetMode,            '/mavros/set_mode')
        self.arming_client   = self.create_client(CommandBool,        '/mavros/cmd/arming')
        self.set_wp_client   = self.create_client(WaypointSetCurrent, '/mavros/mission/set_current')

        self.get_logger().info("MissionSwitcher node started. Waiting for ZED detections...")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _state_cb(self, msg: State):
        self.fcu_connected = msg.connected

    def _objects_cb(self, msg: ObjectsStamped):
        """
        On each ZED detection message, look for the first object whose label
        is Red or Green. The first valid detection commits the mission choice.
        """
        if self.mission_started:
            return  # Already committed — ignore further detections

        for obj in msg.objects:
            label = obj.label.strip()
            if label == RED_LABEL:
                self.get_logger().info("Detected RED — starting Sequence B.")
                self.mission_started = True
                self.start_mission(isRed=True)
                return
            elif label == GREEN_LABEL:
                self.get_logger().info("Detected GREEN — starting Sequence A.")
                self.mission_started = True
                self.start_mission(isRed=False)
                return

    # ------------------------------------------------------------------
    # MAVROS helpers
    # ------------------------------------------------------------------

    def _wait_for_service(self, client, timeout_sec=10.0) -> bool:
        self.get_logger().info(f"Waiting for service: {client.srv_name}")
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f"Service {client.srv_name} not available after {timeout_sec}s"
            )
            return False
        return True

    def _wait_for_fcu(self, timeout_sec=30.0) -> bool:
        self.get_logger().info("Waiting for FCU connection...")
        start = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.fcu_connected:
                self.get_logger().info("FCU connected.")
                return True
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().error("Timed out waiting for FCU connection.")
                return False
        return False

    def _call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.get_logger().error(f"Service call to {client.srv_name} failed (no response).")
            return None
        return future.result()

    # ------------------------------------------------------------------
    # Mission logic
    # ------------------------------------------------------------------

    def start_mission(self, isRed: bool) -> bool:
        target_wp = SEQ_B_START if isRed else SEQ_A_START
        label     = "Sequence B (Red)" if isRed else "Sequence A (Green)"
        self.get_logger().info(f"Starting {label} — jumping to waypoint index {target_wp}")

        # 1. Confirm all services are up
        for client in [self.set_mode_client, self.arming_client, self.set_wp_client]:
            if not self._wait_for_service(client):
                return False

        # 2. Confirm FCU is connected
        if not self._wait_for_fcu():
            return False

        # 3. Set AUTO mode BEFORE arming
        self.get_logger().info("Setting mode to AUTO...")
        mode_req = SetMode.Request()
        mode_req.base_mode   = 0
        mode_req.custom_mode = 'AUTO'
        mode_resp = self._call_service(self.set_mode_client, mode_req)
        if mode_resp is None or not mode_resp.mode_sent:
            self.get_logger().error("Failed to set AUTO mode. Aborting.")
            return False

        # 4. Jump to the correct starting waypoint
        self.get_logger().info(f"Setting current waypoint to index {target_wp}...")
        wp_req        = WaypointSetCurrent.Request()
        wp_req.wp_seq = target_wp
        wp_resp       = self._call_service(self.set_wp_client, wp_req)
        if wp_resp is None or not wp_resp.success:
            self.get_logger().error("Failed to set current waypoint. Aborting.")
            return False

        # 5. Arm the vehicle
        self.get_logger().info("Arming vehicle...")
        arm_req       = CommandBool.Request()
        arm_req.value = True
        arm_resp      = self._call_service(self.arming_client, arm_req)
        if arm_resp is None or not arm_resp.success:
            self.get_logger().error("Arming failed. Check pre-arm conditions on FCU.")
            return False

        self.get_logger().info(f"Mission underway — {label} starting at WP {target_wp}.")
        return True


def main():
    rclpy.init()
    node = WayPointSpeedChallenge()

    try:
        # Spin indefinitely — mission triggers automatically from _objects_cb
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()