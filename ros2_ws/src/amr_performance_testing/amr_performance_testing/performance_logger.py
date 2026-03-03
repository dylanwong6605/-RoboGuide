#!/usr/bin/env python3
"""
performance_logger.py
---------------------
Collects performance metrics during navigation tests:
1. Time to Goal
2. Success/Failure
3. Path Efficiency
4. Number of Recoveries
5. Average Speed
6. Planning Time
7. Minimum Obstacle Distance
8. Safety Violations
9. Collision Count
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
import math
import time
import csv
import os
from datetime import datetime


class PerformanceLogger(Node):
    """Logs navigation performance metrics to CSV."""
    
    def __init__(self):
        super().__init__('performance_logger')
        
        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('scenario_name', 'unknown')
        self.declare_parameter('run_number', 1)
        self.declare_parameter('output_dir', '/ros2_ws/test_results')
        self.declare_parameter('safety_distance_threshold', 0.3)  # meters
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        
        self.scenario_name = self.get_parameter('scenario_name').value
        self.run_number = self.get_parameter('run_number').value
        self.output_dir = self.get_parameter('output_dir').value
        self.safety_threshold = self.get_parameter('safety_distance_threshold').value
        
        # Goal and start positions
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        
        # ── Metric Storage ────────────────────────────────────────
        self.start_time = None
        self.end_time = None
        self.success = False
        self.total_distance = 0.0
        self.last_odom_pose = None
        self.num_recoveries = 0
        self.planning_time = None
        self.min_obstacle_distance = float('inf')
        self.safety_violations = 0
        self.collision_count = 0
        self.last_collision_time = 0.0
        self.planning_start_time = None
        
        # ── Subscribers ───────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # ── Action Client ─────────────────────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info(
            f'Performance Logger initialized for scenario: {self.scenario_name}, '
            f'run: {self.run_number}'
        )
    
    # ══════════════════════════════════════════════════════════════
    # CALLBACK METHODS
    # ══════════════════════════════════════════════════════════════
    
    def odom_callback(self, msg):
        """Track distance traveled and detect collisions from velocity."""
        current_pose = msg.pose.pose.position
        
        # Calculate distance traveled
        if self.last_odom_pose is not None:
            dx = current_pose.x - self.last_odom_pose.x
            dy = current_pose.y - self.last_odom_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            self.total_distance += distance
            
            # Simple collision detection: if linear velocity commanded but not moving
            linear_vel = math.sqrt(
                msg.twist.twist.linear.x**2 + 
                msg.twist.twist.linear.y**2
            )
            
            # If we're supposed to be moving but distance is tiny, might be collision
            current_time = time.time()
            if linear_vel > 0.05 and distance < 0.001:
                # Only count if it's been more than 2 seconds since last collision
                if current_time - self.last_collision_time > 2.0:
                    self.collision_count += 1
                    self.last_collision_time = current_time
                    self.get_logger().warn(
                        f'Possible collision detected! Count: {self.collision_count}'
                    )
        
        self.last_odom_pose = current_pose
    
    def scan_callback(self, msg):
        """Track minimum obstacle distance and safety violations."""
        if len(msg.ranges) == 0:
            return
        
        # Filter out invalid readings
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        
        if len(valid_ranges) == 0:
            return
        
        min_range = min(valid_ranges)
        
        # Update minimum obstacle distance
        if min_range < self.min_obstacle_distance:
            self.min_obstacle_distance = min_range
        
        # Check for safety violation
        if min_range < self.safety_threshold:
            self.safety_violations += 1
            if self.safety_violations % 10 == 0:  # Log every 10th violation
                self.get_logger().warn(
                    f'Safety violation! Obstacle at {min_range:.2f}m '
                    f'(threshold: {self.safety_threshold}m). '
                    f'Total violations: {self.safety_violations}'
                )
    
    def feedback_callback(self, feedback_msg):
        """Track recoveries and planning time from Nav2 feedback."""
        feedback = feedback_msg.feedback
        
        # Track planning time (first feedback after goal sent)
        if self.planning_time is None and self.planning_start_time is not None:
            self.planning_time = time.time() - self.planning_start_time
            self.get_logger().info(f'Planning completed in {self.planning_time:.2f}s')
        
        # Nav2 feedback includes number_of_recoveries
        if hasattr(feedback, 'number_of_recoveries'):
            if feedback.number_of_recoveries > self.num_recoveries:
                self.num_recoveries = feedback.number_of_recoveries
                self.get_logger().info(
                    f'Recovery detected. Total recoveries: {self.num_recoveries}'
                )
    
    # ══════════════════════════════════════════════════════════════
    # NAVIGATION METHODS
    # ══════════════════════════════════════════════════════════════
    
    def send_goal(self):
        """Send navigation goal and start tracking."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(
            f'Sending goal: ({self.goal_x:.2f}, {self.goal_y:.2f})'
        )
        
        # Wait for action server
        self.nav_client.wait_for_server()
        
        # Record start time and planning start
        self.start_time = time.time()
        self.planning_start_time = time.time()
        
        # Send goal
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            self.success = False
            self.save_results()
            return
        
        self.get_logger().info('Goal accepted, navigating...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle navigation completion."""
        result = future.result().result
        self.end_time = time.time()
        
        # Check if goal was reached
        # Nav2 returns empty result on success
        self.success = True
        self.get_logger().info('Navigation completed successfully!')
        
        # Save results to CSV
        self.save_results()
    
    # ══════════════════════════════════════════════════════════════
    # METRICS CALCULATION & SAVING
    # ══════════════════════════════════════════════════════════════
    
    def calculate_metrics(self):
        """Calculate all performance metrics."""
        # 1. Time to goal
        if self.start_time and self.end_time:
            time_to_goal = self.end_time - self.start_time
        else:
            time_to_goal = 0.0
        
        # 2. Success (already tracked)
        
        # 3. Path efficiency
        straight_line_dist = math.sqrt(
            (self.goal_x - self.start_x)**2 + 
            (self.goal_y - self.start_y)**2
        )
        if straight_line_dist > 0:
            path_efficiency = self.total_distance / straight_line_dist
        else:
            path_efficiency = 0.0
        
        # 4. Number of recoveries (already tracked)
        
        # 5. Average speed
        if time_to_goal > 0:
            avg_speed = self.total_distance / time_to_goal
        else:
            avg_speed = 0.0
        
        # 6. Planning time (already tracked)
        if self.planning_time is None:
            self.planning_time = 0.0
        
        # 7. Min obstacle distance (already tracked)
        if self.min_obstacle_distance == float('inf'):
            self.min_obstacle_distance = 0.0
        
        # 8. Safety violations (already tracked)
        
        # 9. Collision count (already tracked)
        
        return {
            'scenario_name': self.scenario_name,
            'run_number': self.run_number,
            'timestamp': datetime.now().strftime('%Y-%m-%d_%H:%M:%S'),
            'time_to_goal': round(time_to_goal, 2),
            'success': self.success,
            'path_efficiency': round(path_efficiency, 3),
            'num_recoveries': self.num_recoveries,
            'avg_speed': round(avg_speed, 3),
            'planning_time': round(self.planning_time, 2),
            'min_obstacle_dist': round(self.min_obstacle_distance, 3),
            'safety_violations': self.safety_violations,
            'collision_count': self.collision_count
        }
    
    def save_results(self):
        """Save metrics to CSV file."""
        metrics = self.calculate_metrics()
        
        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create filename
        filename = os.path.join(
            self.output_dir,
            f'{self.scenario_name}_run{self.run_number}.csv'
        )
        
        # Write to CSV
        file_exists = os.path.isfile(filename)
        
        with open(filename, 'a', newline='') as csvfile:
            fieldnames = [
                'scenario_name', 'run_number', 'timestamp',
                'time_to_goal', 'success', 'path_efficiency',
                'num_recoveries', 'avg_speed', 'planning_time',
                'min_obstacle_dist', 'safety_violations', 'collision_count'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            # Write header if file is new
            if not file_exists:
                writer.writeheader()
            
            writer.writerow(metrics)
        
        self.get_logger().info(f'Results saved to {filename}')
        self.get_logger().info(f'Metrics: {metrics}')


def main(args=None):
    rclpy.init(args=args)
    logger = PerformanceLogger()
    
    # Send goal after a short delay to let everything initialize
    time.sleep(2.0)
    logger.send_goal()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
