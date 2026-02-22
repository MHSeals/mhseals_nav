import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math

class Task1GateNavigator(Node):
    def __init__(self):
        super().__init__('task1_gate_navigator')
        
        self.group = ReentrantCallbackGroup()
        self.navigator = BasicNavigator()
        self.latest_gate_poses = []
        self.is_navigating = False
        
        # Subscribe to tracked objects in map frame
        self.marker_sub = self.create_subscription(
            MarkerArray, 
            '/tracked_objects', 
            self.marker_callback, 
            10, 
            callback_group=self.group)
        
        # 2-second to not overload observations
        self.timer = self.create_timer(
            2.0, 
            self.navigation_loop, 
            callback_group=self.group)
        
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Task One Navigator is Active')
        
    # Nagivation logic here
    def execute_nav(self):
        self.is_navigating = True
        poses = self.latest_gate_poses
        self.navigator.goThroughPoses(poses)

        last_goal_index = 0
        
        # If we are still navigating, checks to see if we passed gate 1
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # feedback.current_pose_index is the goal the boat is GOING TO.
                if feedback.current_pose_index > last_goal_index:
                    self.log_gate_cleared(last_goal_index + 1, feedback.current_pose.pose.position)
                    last_goal_index = feedback.current_pose_index

        # Once we finished traveling to all goal poses
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            # If we only had 1 gate in the list, we just finished the Entry Gate.
            if len(poses) == 1:
                self.log_gate_cleared(1, self.navigator.getCurrentPose().pose.position)
                self.drive_straight_until_discovery()
            else:
                # We finished the whole list, so the last one was the Exit Gate.
                self.log_gate_cleared(2, self.navigator.getCurrentPose().pose.position)
                self.get_logger().info("TASK 1 COMPLETE: Both gates cleared.")
        
        self.is_navigating = False

    # logs the finished gate, reports pos, time, and label (Entry or Exit)
    def log_gate_cleared(self, gate_num, position):
        now = self.get_clock().now().to_msg()
        label = "ENTRY" if gate_num == 1 else "EXIT"
        self.get_logger().info(f'REPORT: {label} Gate Cleared.')
        self.get_logger().info(f'Time: {now.sec}s, Pos: x={position.x:.2f}, y={position.y:.2f}')

    # Calculates the angle to drive straight forward
    def calculate_forward_pose(self, distance):
        curr_pose = self.navigator.getCurrentPose()
        
        # Extract yaw from quaternion to project x and y
        q = curr_pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        # Weird position math lol
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = curr_pose.pose.position.x + (distance * math.cos(yaw))
        pose.pose.position.y = curr_pose.pose.position.y + (distance * math.sin(yaw))
        pose.pose.orientation = q 
        return pose

    # Loops our navigation stack when we aren't already going/finished
    def navigation_loop(self):
        # Stop if we are already in the execute_nav while-loop
        if self.is_navigating or not self.latest_gate_poses:
            return
        self.execute_nav()

    # If we don't see the second bouy pair, we drive stright until we do
    def drive_straight_until_discovery(self):
        self.get_logger().info('Gate 1 cleared. Gate 2 not found. Driving straight...')
        # Create a dummy pose 10 meters ahead to keep moving
        straight_pose = self.calculate_forward_pose(10.0) 
        self.navigator.goToPose(straight_pose)
    
    # Runs every time we see an object (2s delay), calculates bouy pairs
    def marker_callback(self, msg):
        # Filter for all visible tall buoys
        reds = [m.pose.position for m in msg.markers if m.label == "red_pole_buoy"]
        greens = [m.pose.position for m in msg.markers if m.label == "green_pole_buoy"]

        # Even if we only see ONE red and ONE green, we can form a gate
        if len(reds) >= 1 and len(greens) >= 1:
            new_gate_poses = self.calculate_all_visible_midpoints(reds, greens)
            
            # If we aren't moving yet, or if we found MORE gates than before
            if not self.is_navigating or len(new_gate_poses) > len(self.latest_gate_poses):
                self.latest_gate_poses = new_gate_poses
                self.get_logger().info(f"Updated plan: {len(self.latest_gate_poses)} gates found.")
    
    #Calculates all the possible midpts and returns the poses
    #Ngl no idea how this rlly works, just told to add it 
    def calculate_all_visible_midpoints(self, reds, greens):
        gate_poses = []
        
        #NGL this some wizardry math to me so lolololololololol
        # Sort by distance so we pair the closest ones first
        reds.sort(key=lambda p: p.x**2 + p.y**2)
        greens.sort(key=lambda p: p.x**2 + p.y**2)

        for r_pos, g_pos in zip(reds, greens):
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
    node = Task1GateNavigator()
    # The navigator logic starts in the timer/callbacks automatically
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except SystemExit: # Triggered by rclpy.shutdown()
        pass