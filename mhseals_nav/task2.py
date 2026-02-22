from time import time
from turtle import color

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math

class Task2DebrisNavigator(Node):
    def __init__(self):
        super().__init__('task2_debris_navigator')
        self.group = ReentrantCallbackGroup()
        self.navigator = BasicNavigator()
        
        # State Machine
        self.state = "CHANNEL_TRANSIT" 
        self.is_navigating = False
        
        # Memory storage for the return trip and reporting
        self.entry_gate_poses = []
        self.reported_debris = set() 
        self.reported_beacons = set()
        self.gates_cleared = 0
        self.debris_locations = {}
        
        # Subscriptions and Timers
        self.marker_sub = self.create_subscription(
            MarkerArray, 
            '/tracked_objects', 
            self.marker_callback, 
            10, 
            callback_group=self.group)
        
        self.timer = self.create_timer(
            2.0, 
            self.state_machine_loop, 
            callback_group=self.group)
        
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Task 2 Debris Navigator is Active')

    def marker_callback(self, msg):
        red_channel = []
        green_channel = []

        for m in msg.markers:
            # ALWAYS REPORT: Black debris
            if "black_" in m.label and m.id not in self.reported_debris:
                self.reported_debris.add(m.id)
                self.get_logger().info(f"DISRUPTIVE REPORT: Debris {m.label} ({m.id}) at x={m.pose.position.x:.2f}, y={m.pose.position.y:.2f}")

            # ALWAYS REPORT: Light beacons
            if "light_buoy" in m.label and m.id not in self.reported_beacons:
                self.reported_beacons.add(m.id)
                status = "SURVIVOR" if "green" in m.label else "HAZARD"
                self.get_logger().info(f"DISRUPTIVE REPORT: {status} beacon ({m.label}) at x={m.pose.position.x:.2f}, y={m.pose.position.y:.2f}")
                
                # ONLY trigger the physical "INTERACT" state if it's a green survivor
                if "green" in m.label and self.state != "RETURN_TRANSIT":
                    self.state = "BEACON_INTERACT"
                    self.interact_pose = m.pose.position

            # STATE-SPECIFIC: Collect channel markers for navigation
            if self.state == "CHANNEL_TRANSIT":
                if m.label == "red_buoy": red_channel.append(m.pose.position)
                elif m.label == "green_buoy": green_channel.append(m.pose.position)

        # Update gate memory for the return trip
        if self.state == "CHANNEL_TRANSIT" and red_channel and green_channel:
            self.entry_gate_poses = self.calculate_all_visible_midpoints(red_channel, green_channel)

    def state_machine_loop(self):
        if self.is_navigating: return

        if self.state == "CHANNEL_TRANSIT" and self.entry_gate_poses:
            self.execute_channel_transit()
            
        elif self.state == "DEBRIS_SEARCH":
            self.get_logger().info("Scanning debris field for survivors...")
            time.sleep(3.0) 
            # If no survivor was found (which would have triggered BEACON_INTERACT), head home.
            if self.state == "DEBRIS_SEARCH":
                self.state = "RETURN_TRANSIT"
            
        elif self.state == "BEACON_INTERACT":
            self.execute_survivor_circle()
            
        elif self.state == "RETURN_TRANSIT":
            self.execute_return_trip()

    def execute_channel_transit(self):
        self.is_navigating = True
        self.get_logger().info("Transiting Channel...")
        self.navigator.goThroughPoses(self.entry_gate_poses)
        
        last_index = 0
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and feedback.current_pose_index > last_index:
                self.gates_cleared += 1
                last_index = feedback.current_pose_index
                self.get_logger().info(f"Gate {self.gates_cleared} cleared.")
                
                # Assume 4 gates max for the channel
                if self.gates_cleared >= 4:
                    self.get_logger().info("End of channel reached. Entering Debris Field.")
                    self.navigator.cancelTask()
                    self.state = "DEBRIS_SEARCH"
                    break

        self.is_navigating = False

    def execute_survivor_circle(self):
        self.is_navigating = True
        self.get_logger().info("Circling Green Beacon (Survivor)...")
        
        # Calculate a 3-meter radius circle around the green beacon
        circle_poses = self.calculate_circle_poses(self.interact_pose, radius=2.0, points=6)
        self.navigator.goThroughPoses(circle_poses)
        
        while not self.navigator.isTaskComplete():
            pass # Block until circle is complete
            
        if self.navigator.getResult() == TaskResult.SUCCEEDED:
            self.get_logger().info("Circle complete. Starting return trip.")
            self.state = "RETURN_TRANSIT"
            
        self.is_navigating = False

    def execute_return_trip(self):
        self.is_navigating = True
        self.get_logger().info("Navigating back through the channel...")
        
        # Reverse the gates we found on the way in
        return_path = list(reversed(self.entry_gate_poses))
        
        # Ensure the boat's orientation points back the way it came
        for pose in return_path:
            # Simple inversion of the quaternion (yaw + 180 degrees)
            q = pose.pose.orientation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            new_yaw = yaw + math.pi 
            
            pose.pose.orientation.z = math.sin(new_yaw / 2)
            pose.pose.orientation.w = math.cos(new_yaw / 2)

        self.navigator.goThroughPoses(return_path)
        
        while not self.navigator.isTaskComplete():
            pass

        if self.navigator.getResult() == TaskResult.SUCCEEDED:
            self.get_logger().info("TASK 2 COMPLETE: Returned to start.")
            self.state = "DONE"
            
        self.is_navigating = False

    def calculate_circle_poses(self, center_pos, radius, points):
        poses = []
        for i in range(points):
            angle = (2 * math.pi / points) * i
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            
            # Position
            pose.pose.position.x = center_pos.x + radius * math.cos(angle)
            pose.pose.position.y = center_pos.y + radius * math.sin(angle)
            
            # Orientation (tangent to the circle so the boat faces forward)
            tangent_angle = angle + (math.pi / 2)
            pose.pose.orientation.z = math.sin(tangent_angle / 2)
            pose.pose.orientation.w = math.cos(tangent_angle / 2)
            poses.append(pose)
            
        # Add the first point again to close the loop
        poses.append(poses[0])
        return poses

    def calculate_all_visible_midpoints(self, reds, greens):
        # [Same helper function from Task 1]
        gate_poses = []
        reds.sort(key=lambda p: p.x**2 + p.y**2)
        greens.sort(key=lambda p: p.x**2 + p.y**2)
        for r_pos, g_pos in zip(reds, greens):
            mid_x, mid_y = (r_pos.x + g_pos.x) / 2, (r_pos.y + g_pos.y) / 2
            yaw = math.atan2(r_pos.y - g_pos.y, r_pos.x - g_pos.x) + (math.pi / 2)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = mid_x, mid_y
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            gate_poses.append(pose)
        return gate_poses