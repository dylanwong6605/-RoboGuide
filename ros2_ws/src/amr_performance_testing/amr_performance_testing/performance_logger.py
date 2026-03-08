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
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
import math
import time
import csv
import os
from datetime import datetime

MAX_GOAL_RETRIES = 3
RETRY_DELAY = 5.0


class PerformanceLogger(Node):
    """Logs navigation performance metrics to CSV."""

    def __init__(self):
        super().__init__('performance_logger')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('scenario_name', 'unknown')
        self.declare_parameter('run_number', 1)
        self.declare_parameter('output_dir', '/ros2_ws/test_results')
        self.declare_parameter('safety_distance_threshold', 0.3)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('wait_for_action_server', True)
        self.declare_parameter('wait_for_nav2_lifecycle', True)
        self.declare_parameter('amcl_settle_seconds', 10.0)

        self.scenario_name = self.get_parameter('scenario_name').value
        self.run_number = self.get_parameter('run_number').value
        self.output_dir = self.get_parameter('output_dir').value
        self.safety_threshold = self.get_parameter('safety_distance_threshold').value

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.wait_for_action_server = bool(self.get_parameter('wait_for_action_server').value)
        self.wait_for_nav2_lifecycle = bool(self.get_parameter('wait_for_nav2_lifecycle').value)
        self.amcl_settle_seconds = max(0.0, float(self.get_parameter('amcl_settle_seconds').value))

        # ── Metric Storage ────────────────────────────────────────
        self.start_time = None
        self.end_time = None
        self.goal_accepted = False       # Only track metrics after goal accepted
        self.success = False
        self.total_distance = 0.0
        self.last_odom_pose = None
        self.num_recoveries = 0
        self.planning_time = None
        self.first_feedback_received = False
        self.min_obstacle_distance = float('inf')
        self.safety_violations = 0
        self.collision_count = 0
        self.last_collision_time = 0.0
        self.planning_start_time = None
        self.goal_retries = 0
        self.results_saved = False

        # ── Subscribers ───────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # ── Action Client ─────────────────────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(
            f'Performance Logger initialized for scenario: {self.scenario_name}, '
            f'run: {self.run_number}'
        )

    # ══════════════════════════════════════════════════════════════
    # READINESS CHECKS
    # ══════════════════════════════════════════════════════════════

    def wait_for_nav2_lifecycle(self, timeout=120.0):
        """
        Poll Nav2 lifecycle nodes until they are all in 'active' state.
        This is the key check that prevents sending goals before Nav2 is ready.
        """
        nodes_to_check = [
            '/bt_navigator/get_state',
            '/controller_server/get_state',
            '/planner_server/get_state',
        ]

        self.get_logger().info('Waiting for Nav2 lifecycle nodes to become active...')
        start = time.time()

        while time.time() - start < timeout:
            all_active = True

            for service_name in nodes_to_check:
                client = self.create_client(GetState, service_name)

                if not client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().info(f'  Service {service_name} not available yet...')
                    all_active = False
                    self.destroy_client(client)
                    break

                req = GetState.Request()
                future = client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

                if future.result() is not None:
                    state_label = future.result().current_state.label
                    if state_label != 'active':
                        self.get_logger().info(
                            f'  {service_name} state: {state_label} (waiting for active)'
                        )
                        all_active = False
                else:
                    self.get_logger().info(f'  {service_name} did not respond')
                    all_active = False

                self.destroy_client(client)

                if not all_active:
                    break

            if all_active:
                elapsed = time.time() - start
                self.get_logger().info(
                    f'All Nav2 lifecycle nodes active after {elapsed:.1f}s'
                )
                return True

            time.sleep(3)

        self.get_logger().error(
            f'Timeout ({timeout}s) waiting for Nav2 lifecycle nodes!'
        )
        return False

    # ══════════════════════════════════════════════════════════════
    # CALLBACK METHODS
    # ══════════════════════════════════════════════════════════════

    def odom_callback(self, msg):
        """Track distance traveled and detect collisions from velocity.
        Only accumulates metrics AFTER the goal has been accepted."""
        if not self.goal_accepted:
            # Store pose for reference but don't accumulate distance
            self.last_odom_pose = msg.pose.pose.position
            return

        current_pose = msg.pose.pose.position

        if self.last_odom_pose is not None:
            dx = current_pose.x - self.last_odom_pose.x
            dy = current_pose.y - self.last_odom_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            self.total_distance += distance

            linear_vel = math.sqrt(
                msg.twist.twist.linear.x**2 +
                msg.twist.twist.linear.y**2
            )

            current_time = time.time()
            if linear_vel > 0.05 and distance < 0.001:
                if current_time - self.last_collision_time > 2.0:
                    self.collision_count += 1
                    self.last_collision_time = current_time
                    self.get_logger().warn(
                        f'Possible collision detected! Count: {self.collision_count}'
                    )

        self.last_odom_pose = current_pose

    def scan_callback(self, msg):
        """Track minimum obstacle distance and safety violations.
        Only tracks AFTER the goal has been accepted."""
        if not self.goal_accepted:
            return

        if len(msg.ranges) == 0:
            return

        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]

        if len(valid_ranges) == 0:
            return

        min_range = min(valid_ranges)

        if min_range < self.min_obstacle_distance:
            self.min_obstacle_distance = min_range

        if min_range < self.safety_threshold:
            self.safety_violations += 1
            if self.safety_violations % 10 == 0:
                self.get_logger().warn(
                    f'Safety violation! Obstacle at {min_range:.2f}m '
                    f'(threshold: {self.safety_threshold}m). '
                    f'Total violations: {self.safety_violations}'
                )

    def feedback_callback(self, feedback_msg):
        """Track recoveries and planning time from Nav2 feedback."""
        feedback = feedback_msg.feedback

        # Planning time = time from goal sent to first feedback received
        if not self.first_feedback_received and self.planning_start_time is not None:
            self.first_feedback_received = True
            self.planning_time = time.time() - self.planning_start_time
            self.get_logger().info(f'Planning completed in {self.planning_time:.2f}s')

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
            f'Sending goal: ({self.goal_x:.2f}, {self.goal_y:.2f}) '
            f'[attempt {self.goal_retries + 1}/{MAX_GOAL_RETRIES}]'
        )

        # Planning time starts from when we send the goal
        self.planning_start_time = time.time()

        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance or rejection with retry logic."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.goal_retries += 1
            self.get_logger().error(
                f'Goal rejected! (attempt {self.goal_retries}/{MAX_GOAL_RETRIES})'
            )

            if self.goal_retries < MAX_GOAL_RETRIES:
                self.get_logger().info(f'Retrying in {RETRY_DELAY}s...')
                self.retry_timer = self.create_timer(RETRY_DELAY, self._retry_goal_once)
            else:
                self.get_logger().error(
                    f'All {MAX_GOAL_RETRIES} goal attempts rejected. Recording failure.'
                )
                self.success = False
                self.start_time = time.time()
                self.end_time = time.time()
                self.save_results()
            return

        # Goal accepted — NOW start tracking metrics
        self.goal_accepted = True
        self.start_time = time.time()
        self.total_distance = 0.0          # Reset distance from this moment
        self.last_odom_pose = None         # Force fresh pose on next odom
        self.min_obstacle_distance = float('inf')  # Reset
        self.safety_violations = 0
        self.collision_count = 0

        self.get_logger().info('Goal accepted, navigating...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def _retry_goal_once(self):
        """Timer callback to retry sending the goal. Fires once then cancels."""
        self.retry_timer.cancel()
        self.get_logger().info('Retrying goal...')
        self.send_goal()

    def get_result_callback(self, future):
        """Handle navigation completion with proper status checking."""
        status = future.result().status
        self.end_time = time.time()

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.success = True
            self.get_logger().info('Navigation completed successfully!')
        elif status == GoalStatus.STATUS_ABORTED:
            self.success = False
            self.get_logger().error('Navigation aborted by Nav2 (planner/controller failure)')
        elif status == GoalStatus.STATUS_CANCELED:
            self.success = False
            self.get_logger().error('Navigation was canceled')
        else:
            self.success = False
            self.get_logger().error(f'Navigation ended with unexpected status: {status}')

        self.save_results()

    # ══════════════════════════════════════════════════════════════
    # METRICS CALCULATION & SAVING
    # ══════════════════════════════════════════════════════════════

    def calculate_metrics(self):
        """Calculate all performance metrics."""
        if self.start_time and self.end_time:
            time_to_goal = self.end_time - self.start_time
        else:
            time_to_goal = 0.0

        straight_line_dist = math.sqrt(
            (self.goal_x - self.start_x)**2 +
            (self.goal_y - self.start_y)**2
        )
        if straight_line_dist > 0 and self.total_distance > 0:
            path_efficiency = self.total_distance / straight_line_dist
        else:
            path_efficiency = 0.0

        if time_to_goal > 0:
            avg_speed = self.total_distance / time_to_goal
        else:
            avg_speed = 0.0

        if self.planning_time is None:
            self.planning_time = 0.0

        if self.min_obstacle_distance == float('inf'):
            self.min_obstacle_distance = 0.0

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
        """Save metrics to combined CSV file. Only runs once."""
        if self.results_saved:
            return
        self.results_saved = True

        metrics = self.calculate_metrics()

        os.makedirs(self.output_dir, exist_ok=True)

        filename = os.path.join(self.output_dir, 'combined_results.csv')
        file_exists = os.path.isfile(filename)

        with open(filename, 'a', newline='') as csvfile:
            fieldnames = [
                'scenario_name', 'run_number', 'timestamp',
                'time_to_goal', 'success', 'path_efficiency',
                'num_recoveries', 'avg_speed', 'planning_time',
                'min_obstacle_dist', 'safety_violations', 'collision_count'
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

            if not file_exists:
                writer.writeheader()

            writer.writerow(metrics)

        self.get_logger().info(f'Results saved to {filename}')
        self.get_logger().info(f'Metrics: {metrics}')

        # Exit cleanly so test_runner detects completion via poll()
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    logger = PerformanceLogger()

    # ── Phase 1: Optional wait for Nav2 action server ─────────────
    if logger.wait_for_action_server:
        logger.get_logger().info('Waiting for navigate_to_pose action server...')
        if not logger.nav_client.wait_for_server(timeout_sec=60.0):
            logger.get_logger().error('Action server not available after 60s!')
            logger.success = False
            logger.start_time = time.time()
            logger.end_time = time.time()
            logger.save_results()
            return
    else:
        logger.get_logger().info('Skipping action server wait (configured)')

    # ── Phase 2: Optional wait for Nav2 lifecycle ACTIVE state ────
    if logger.wait_for_nav2_lifecycle:
        nav2_ready = logger.wait_for_nav2_lifecycle(timeout=120.0)
        if not nav2_ready:
            logger.get_logger().error('Nav2 lifecycle nodes never became active!')
            logger.success = False
            logger.start_time = time.time()
            logger.end_time = time.time()
            logger.save_results()
            return
    else:
        logger.get_logger().info('Skipping Nav2 lifecycle wait (configured)')

    # ── Phase 3: Optional settle time for AMCL convergence ────────
    if logger.amcl_settle_seconds > 0.0:
        logger.get_logger().info(
            f'Waiting {logger.amcl_settle_seconds:.1f}s for AMCL to converge...'
        )
        time.sleep(logger.amcl_settle_seconds)
    else:
        logger.get_logger().info('Skipping AMCL settle delay (configured)')

    # ── Phase 4: Send the navigation goal ─────────────────────────
    logger.send_goal()

    try:
        rclpy.spin(logger)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()