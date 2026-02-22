import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import time

class Task3SprintNavigator(Node):
    def __init__(self):
        super().__init__('task3_sprint_navigator')
        self.group = ReentrantCallbackGroup()
        self.navigator = BasicNavigator()
        
        # State Machine
        self.state = "FIND_GATE" 
        self.is_navigating = False
        
        # Memory storage
        self.gate_pose = None
        self.beacon_detected = False
        
        # Timing for Disruptive Reporting
        self.start_time = 0.0
        self.response_time = 0.0
        
        # Subscriptions and Timers
        self.marker_sub = self.create_subscription(
            MarkerArray, 
            '/tracked_objects', 
            self.marker_callback, 
            10, 
            callback_group=self.group)
        
        self.timer = self.create_timer(
            0.5, 
            self.state_machine_loop, 
            callback_group=self.group) # Faster loop for sprint
        
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Task 3 Emergency Sprint is Active. Revving engines...')

    def marker_callback(self, msg):
        red_buoys = []
        green_buoys = []

        for m in msg.markers:
            # 1. Detect the Entry/Exit Gate
            if m.scale.z > 0.8 and not m.ns == "light_beacon":
                if m.color.r > 0.5: red_buoys.append(m.pose.position)
                if m.color.g > 0.5: green_buoys.append(m.pose.position)

            # 2. Detect the Light Beacon
            elif m.ns == "light_beacon" and not self.beacon_detected and self.state == "SEEK_BEACON":
                self.beacon_detected = True
                self.response_time = time.time() - self.start_time
                
                # Determine color and direction
                if m.color.r > 0.5:
                    color_str = "RED"
                    direction = "CCW" # Counter-clockwise (from the right)
                else:
                    color_str = "GREEN"
                    direction = "CW"  # Clockwise (from the left)

                # Disruptive Reporting: Color and Time of Response
                self.get_logger().info(f"DISRUPTIVE REPORT: {color_str} beacon detected. Response time: {self.response_time:.2f} seconds.")
                
                # Calculate the maneuver and trigger it
                circle_poses = self.calculate_sprint_maneuver(m.pose.position, direction)
                self.navigator.cancelTask() # Cancel forward drive
                self.state = "EXECUTE_MANEUVER"
                self.maneuver_poses = circle_poses

        # Update gate pose if we haven't locked it in yet
        if self.state == "FIND_GATE" and len(red_buoys) >= 1 and len(green_buoys) >= 1:
            gates = self.calculate_all_visible_midpoints(red_buoys, green_buoys)
            if gates:
                self.gate_pose = gates[0] # Take the closest pair

    def state_machine_loop(self):
        if self.is_navigating: return

        if self.state == "FIND_GATE" and self.gate_pose:
            self.execute_gate_entry()
            
        elif self.state == "SEEK_BEACON":
            self.drive_straight_to_find_beacon()
            
        elif self.state == "EXECUTE_MANEUVER":
            self.execute_circle_and_return()

    def execute_gate_entry(self):
        self.is_navigating = True
        self.get_logger().info("Sprint Gate identified. Punching it...")
        
        self.navigator.goToPose(self.gate_pose)
        
        while not self.navigator.isTaskComplete():
            pass # Block until we reach the gate
            
        if self.navigator.getResult() == TaskResult.SUCCEEDED:
            self.start_time = time.time() # Start the clock as we pass through
            self.get_logger().info("Gate cleared! Clock started. Scanning for beacon...")
            self.state = "SEEK_BEACON"
            
        self.is_navigating = False

    def drive_straight_to_find_beacon(self):
        self.is_navigating = True
        # Drive straight for 20 meters while the callback looks for the beacon
        straight_pose = self.calculate_forward_pose(20.0) 
        self.navigator.goToPose(straight_pose)
        
        # We don't block fully here because the marker_callback will cancel this 
        # task as soon as it sees the beacon.
        while not self.navigator.isTaskComplete() and self.state == "SEEK_BEACON":
            time.sleep(0.1) 
            
        self.is_navigating = False

    def execute_circle_and_return(self):
        self.is_navigating = True
        self.get_logger().info("Executing high-speed loop...")
        
        # We append the gate pose to the end of the maneuver so the boat smoothly exits
        # without needing to stop and replan.
        full_path = self.maneuver_poses + [self.gate_pose]
        self.navigator.goThroughPoses(full_path)
        
        while not self.navigator.isTaskComplete():
            pass
            
        if self.navigator.getResult() == TaskResult.SUCCEEDED:
            total_time = time.time() - self.start_time
            # Final Communications Report
            self.get_logger().info(f"TASK 3 COMPLETE! Exit gate cleared.")
            self.get_logger().info(f"COMMUNICATIONS REPORT: Total Task Time: {total_time:.2f} seconds.")
            
            self.destroy_node()
            rclpy.shutdown()
            
        self.is_navigating = False

    def calculate_sprint_maneuver(self, center_pos, direction, radius=3.0, points=6):
        poses = []
        # CW goes negative (0 to -2pi), CCW goes positive (0 to 2pi)
        angle_step = (2 * math.pi / points) if direction == "CCW" else -(2 * math.pi / points)
        
        # To make a smooth sprint, we don't start at angle 0, we start at the edge closest to the boat
        curr_pose = self.navigator.getCurrentPose().pose.position
        start_angle = math.atan2(curr_pose.y - center_pos.y, curr_pose.x - center_pos.x)

        for i in range(points + 1): # +1 to close the loop
            angle = start_angle + (angle_step * i)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = center_pos.x + radius * math.cos(angle)
            pose.pose.position.y = center_pos.y + radius * math.sin(angle)
            
            # Tangent orientation for smooth driving
            tangent_angle = angle + (math.pi / 2 if direction == "CCW" else -math.pi / 2)
            pose.pose.orientation.z = math.sin(tangent_angle / 2)
            pose.pose.orientation.w = math.cos(tangent_angle / 2)
            poses.append(pose)
            
        return poses

    def calculate_forward_pose(self, distance):
        curr_pose = self.navigator.getCurrentPose()
        q = curr_pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = curr_pose.pose.position.x + (distance * math.cos(yaw))
        pose.pose.position.y = curr_pose.pose.position.y + (distance * math.sin(yaw))
        pose.pose.orientation = q 
        return pose

    def calculate_all_visible_midpoints(self, reds, greens):
        # [Same standard helper function from Task 1]
        gate_poses = []
        reds.sort(key=lambda p: p.x**2 + p.y**2)
        greens.sort(key=lambda p: p.x**2 + p.y**2)
        if reds and greens:
            r_pos, g_pos = reds[0], greens[0]
            mid_x, mid_y = (r_pos.x + g_pos.x) / 2, (r_pos.y + g_pos.y) / 2
            yaw = math.atan2(r_pos.y - g_pos.y, r_pos.x - g_pos.x) + (math.pi / 2)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = mid_x
            pose.pose.position.y = mid_y
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            gate_poses.append(pose)
        return gate_poses

def main(args=None):
    rclpy.init(args=args)
    node = Task3SprintNavigator()
    # The navigator logic runs on its own timer thread
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except SystemExit:
        pass

if __name__ == '__main__':
    main()