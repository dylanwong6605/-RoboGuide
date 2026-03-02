#!/usr/bin/env python3
"""
target_mover.py  —  Universal person mover
-------------------------------------------
Reads a YAML config file to determine how many people exist and how each
one moves. Works with any world file — just swap the config.

Usage:
    python3 target_mover.py                        # uses person_config.yaml next to this script
    python3 target_mover.py --config /path/to.yaml

Movement types and their config params:
--------------------------------------------------------------
back_and_forth  axis (x/y), speed, distance, start_offset
circle          speed (linear m/s), angular (rad/s)
diagonal        angle (degrees), speed, distance, start_offset
waypoint        speed, loop (reverse/repeat), points [[x,y], ...]
random_walk     speed, change_interval (seconds)
patrol          speed, points [[x,y], ...]  -- loops back to start
figure_eight    speed, radius
zigzag          speed, angle (degrees), distance
oscillate       speed, distance  -- sways in place on Y axis
sprint_and_pause speed, distance, pause_duration (seconds)
stationary      no extra params
--------------------------------------------------------------
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import yaml
import argparse
import os
import math
import random


DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'person_config.yaml'
)

DT = 0.1  # timer period (10 Hz)


# ── Movement classes ──────────────────────────────────────────────────────────

class BackAndForthMover:
    """Bounces back and forth along a single axis (x or y)."""
    def __init__(self, cfg):
        self.axis      = cfg.get('axis', 'x')
        self.speed     = float(cfg.get('speed', 1.0))
        self.distance  = float(cfg.get('distance', 6.0))
        self.dist      = float(cfg.get('start_offset', 0.0))
        self.direction = 1

    def next_cmd(self):
        self.dist += self.speed * DT
        if self.dist >= self.distance:
            self.direction *= -1
            self.dist = 0.0
        msg = Twist()
        if self.axis == 'y':
            msg.linear.y = self.speed * self.direction
        else:
            msg.linear.x = self.speed * self.direction
        return msg


class CircleMover:
    """Drives in a continuous circle. radius = speed / angular."""
    def __init__(self, cfg):
        self.linear  = float(cfg.get('speed', 1.18))
        self.angular = float(cfg.get('angular', 0.393))

    def next_cmd(self):
        msg = Twist()
        msg.linear.x  = self.linear
        msg.angular.z = self.angular
        return msg


class DiagonalMover:
    """Moves diagonally at a given angle, bounces back."""
    def __init__(self, cfg):
        angle_deg      = float(cfg.get('angle', 45.0))
        angle_rad      = math.radians(angle_deg)
        self.speed     = float(cfg.get('speed', 1.0))
        self.distance  = float(cfg.get('distance', 6.0))
        self.dist      = float(cfg.get('start_offset', 0.0))
        self.direction = 1
        # Decompose into x/y velocity components
        self.vx = math.cos(angle_rad)
        self.vy = math.sin(angle_rad)

    def next_cmd(self):
        self.dist += self.speed * DT
        if self.dist >= self.distance:
            self.direction *= -1
            self.dist = 0.0
        msg = Twist()
        msg.linear.x = self.speed * self.vx * self.direction
        msg.linear.y = self.speed * self.vy * self.direction
        return msg


class WaypointMover:
    """
    Follows a list of (x,y) waypoints at a fixed speed using velocity control.
    loop: 'reverse' — walks back through waypoints in reverse order
    loop: 'repeat'  — teleports back to start and repeats (avoid unless needed)
    """
    def __init__(self, cfg):
        self.speed     = float(cfg.get('speed', 1.0))
        self.loop      = cfg.get('loop', 'reverse')
        raw            = cfg.get('points', [[1, 0]])
        self.waypoints = [list(p) for p in raw]
        self.forward   = True   # traversal direction
        self.index     = 0      # current target waypoint index
        # Track estimated position (starts at world pose, we just track delta)
        self.px = 0.0
        self.py = 0.0

    def next_cmd(self):
        if not self.waypoints:
            return Twist()

        target = self.waypoints[self.index]
        dx = target[0] - self.px
        dy = target[1] - self.py
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.1:
            # Reached waypoint — advance
            if self.loop == 'reverse':
                if self.forward:
                    if self.index < len(self.waypoints) - 1:
                        self.index += 1
                    else:
                        self.forward = False
                        self.index -= 1
                else:
                    if self.index > 0:
                        self.index -= 1
                    else:
                        self.forward = True
                        self.index += 1
            else:  # repeat
                self.index = (self.index + 1) % len(self.waypoints)
                if self.index == 0:
                    self.px = 0.0
                    self.py = 0.0

            target = self.waypoints[self.index]
            dx = target[0] - self.px
            dy = target[1] - self.py
            dist = math.sqrt(dx * dx + dy * dy) or 0.001

        # Move toward target
        vx = (dx / dist) * self.speed
        vy = (dy / dist) * self.speed
        self.px += vx * DT
        self.py += vy * DT

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        return msg


class RandomWalkMover:
    """Picks a new random direction every change_interval seconds."""
    def __init__(self, cfg):
        self.speed           = float(cfg.get('speed', 1.0))
        self.change_interval = float(cfg.get('change_interval', 3.0))
        self.elapsed         = 0.0
        self.vx              = self.speed
        self.vy              = 0.0
        self._pick_direction()

    def _pick_direction(self):
        angle    = random.uniform(0, 2 * math.pi)
        self.vx  = math.cos(angle) * self.speed
        self.vy  = math.sin(angle) * self.speed

    def next_cmd(self):
        self.elapsed += DT
        if self.elapsed >= self.change_interval:
            self._pick_direction()
            self.elapsed = 0.0
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        return msg


class PatrolMover:
    """Like waypoint but always loops forward back to the start."""
    def __init__(self, cfg):
        self.speed     = float(cfg.get('speed', 1.0))
        raw            = cfg.get('points', [[3, 0], [0, 3]])
        self.waypoints = [list(p) for p in raw]
        self.index     = 0
        self.px        = 0.0
        self.py        = 0.0

    def next_cmd(self):
        if not self.waypoints:
            return Twist()

        target = self.waypoints[self.index]
        dx = target[0] - self.px
        dy = target[1] - self.py
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.1:
            self.index = (self.index + 1) % len(self.waypoints)
            if self.index == 0:
                # Reset position tracking when looping
                self.px = 0.0
                self.py = 0.0
            target = self.waypoints[self.index]
            dx = target[0] - self.px
            dy = target[1] - self.py
            dist = math.sqrt(dx * dx + dy * dy) or 0.001

        vx = (dx / dist) * self.speed
        vy = (dy / dist) * self.speed
        self.px += vx * DT
        self.py += vy * DT

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        return msg


class FigureEightMover:
    """Drives a figure-eight pattern using two connected circles."""
    def __init__(self, cfg):
        self.speed   = float(cfg.get('speed', 1.0))
        self.radius  = float(cfg.get('radius', 2.0))
        self.angular = self.speed / self.radius
        self.t       = 0.0
        # Full figure-eight period = two circles
        self.period  = 2 * math.pi / self.angular

    def next_cmd(self):
        self.t += DT
        # Flip turn direction halfway through
        half = self.period / 2
        if (self.t % self.period) < half:
            angular = self.angular
        else:
            angular = -self.angular
        msg = Twist()
        msg.linear.x  = self.speed
        msg.angular.z = angular
        return msg


class ZigzagMover:
    """Alternates between two diagonal directions, making a Z pattern."""
    def __init__(self, cfg):
        self.speed    = float(cfg.get('speed', 1.0))
        angle_deg     = float(cfg.get('angle', 45.0))
        self.distance = float(cfg.get('distance', 4.0))
        self.dist     = 0.0
        angle_rad     = math.radians(angle_deg)
        # Two alternating directions: +angle and -angle
        self.directions = [
            ( math.cos(angle_rad),  math.sin(angle_rad)),
            ( math.cos(angle_rad), -math.sin(angle_rad)),
        ]
        self.leg = 0

    def next_cmd(self):
        self.dist += self.speed * DT
        if self.dist >= self.distance:
            self.leg  = (self.leg + 1) % 2
            self.dist = 0.0
        vx, vy = self.directions[self.leg]
        msg = Twist()
        msg.linear.x = vx * self.speed
        msg.linear.y = vy * self.speed
        return msg


class OscillateMover:
    """Sways in place on the Y axis — like someone shifting weight."""
    def __init__(self, cfg):
        self.speed     = float(cfg.get('speed', 0.3))
        self.distance  = float(cfg.get('distance', 0.5))
        self.dist      = 0.0
        self.direction = 1

    def next_cmd(self):
        self.dist += self.speed * DT
        if self.dist >= self.distance:
            self.direction *= -1
            self.dist = 0.0
        msg = Twist()
        msg.linear.y = self.speed * self.direction
        return msg


class SprintAndPauseMover:
    """Sprints a distance, pauses, then reverses. Repeats."""
    def __init__(self, cfg):
        self.speed         = float(cfg.get('speed', 2.0))
        self.distance      = float(cfg.get('distance', 4.0))
        self.pause_dur     = float(cfg.get('pause_duration', 2.0))
        self.dist          = 0.0
        self.pause_elapsed = 0.0
        self.direction     = 1
        self.pausing       = False

    def next_cmd(self):
        msg = Twist()
        if self.pausing:
            self.pause_elapsed += DT
            if self.pause_elapsed >= self.pause_dur:
                self.pausing       = False
                self.pause_elapsed = 0.0
                self.direction    *= -1
            # publish zero while paused
            return msg

        self.dist += self.speed * DT
        if self.dist >= self.distance:
            self.dist    = 0.0
            self.pausing = True
            return msg

        msg.linear.x = self.speed * self.direction
        return msg


class StationaryMover:
    """Stands still."""
    def __init__(self, cfg):
        pass

    def next_cmd(self):
        return Twist()


# ── Movement registry ─────────────────────────────────────────────────────────

MOVEMENT_TYPES = {
    'back_and_forth':  BackAndForthMover,
    'circle':          CircleMover,
    'diagonal':        DiagonalMover,
    'waypoint':        WaypointMover,
    'random_walk':     RandomWalkMover,
    'patrol':          PatrolMover,
    'figure_eight':    FigureEightMover,
    'zigzag':          ZigzagMover,
    'oscillate':       OscillateMover,
    'sprint_and_pause':SprintAndPauseMover,
    'stationary':      StationaryMover,
}


# ── ROS 2 Node ────────────────────────────────────────────────────────────────

class UniversalMover(Node):
    def __init__(self, config_path):
        super().__init__('target_mover')

        if not os.path.exists(config_path):
            self.get_logger().error(f'Config file not found: {config_path}')
            raise FileNotFoundError(config_path)

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        persons = config.get('persons', [])
        if not persons:
            self.get_logger().warn('No persons defined in config file.')

        self.movers = []
        for p in persons:
            name      = p['name']
            move_type = p.get('movement', 'stationary')

            if move_type not in MOVEMENT_TYPES:
                self.get_logger().warn(
                    f'{name}: unknown movement "{move_type}", defaulting to stationary'
                )
                move_type = 'stationary'

            pub   = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            mover = MOVEMENT_TYPES[move_type](p)
            self.movers.append((name, pub, mover))
            self.get_logger().info(f'  {name}: {move_type}')

        self.get_logger().info(
            f'Universal mover ready — {len(self.movers)} person(s) from {config_path}'
        )
        self.timer = self.create_timer(DT, self.timer_callback)

    def timer_callback(self):
        for _, pub, mover in self.movers:
            pub.publish(mover.next_cmd())


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Universal person mover')
    parser.add_argument('--config', '-c', default=DEFAULT_CONFIG,
                        help='Path to YAML config file')
    args, _ = parser.parse_known_args()

    rclpy.init()
    try:
        node = UniversalMover(args.config)
        rclpy.spin(node)
    except (FileNotFoundError, KeyboardInterrupt):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()