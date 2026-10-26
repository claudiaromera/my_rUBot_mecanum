#!/usr/bin/env python3
"""Holonomic right-wall follower with obstacle handling for the mecanum rUBot."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion


def clamp(value, low, high):
    return max(low, min(high, value))


class HolonomicObstacleWallFollower(Node):
    """LiDAR processing, state machine and control laws."""

    # If no scan arrives for this time (s), a zero command is published.
    SCAN_TIMEOUT = 0.5
    # Minimum angular speed (fraction of turn_speed) while turning to a long
    # wall: avoids the slow asymptotic end of a proportional controller.
    MIN_TURN_FRACTION = 0.35

    def __init__(self):
        super().__init__('holonomic_obstacle_wall_follower')

        self.follow_distance = float(
            self.declare_parameter('follow_distance', 0.35).value)
        self.max_front_length = float(
            self.declare_parameter('max_front_length', 0.50).value)
        self.forward_speed = float(
            self.declare_parameter('forward_speed', 0.20).value)
        self.lateral_speed = float(
            self.declare_parameter('lateral_speed', 0.20).value)
        self.turn_speed = float(
            self.declare_parameter('turn_speed', 1.00).value)
        self.detection_limit = float(
            self.declare_parameter('detection_limit', 1.20).value)
        self.lateral_gain = float(
            self.declare_parameter('lateral_gain', 2.0).value)
        self.alignment_gain = float(
            self.declare_parameter('alignment_gain', 2.0).value)

        self.RIGHT = 'RIGHT'
        self.FRONT = 'FRONT'
        self.FRONT_RIGHT = 'FRONT_RIGHT'
        self.BACK_RIGHT = 'BACK_RIGHT'

        # Angle intervals in degrees: [low, high); FRONT also includes +20.
        self.regions = {
            self.FRONT: (-20.0, 20.0),
            self.FRONT_RIGHT: (-70.0, -20.0),
            self.RIGHT: (-110.0, -70.0),
            self.BACK_RIGHT: (-160.0, -110.0),
        }

        self.diagonal_factor = 0.5
        self.region_confirmations = 3
        self.robust_rays = 3

        self.state = self.RIGHT
        self.obstacle_is_long = False
        self.passing_short_obstacle = False
        self.command = Twist()
        self.odom_pose = None
        self.front_start_pose = None
        self.last_scan_time = None

        self.region_candidate = None
        self.region_candidate_count = 0
        self.stable_region = None

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_command)

        self.get_logger().info(
            f'Holonomic obstacle wall follower started in state {self.state}.')

    # ------------------------------------------------------------------
    # Publishing / odometry
    # ------------------------------------------------------------------
    def publish_command(self):
        if self.last_scan_time is None:
            self.publisher.publish(Twist())
            return
        age = (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9
        self.publisher.publish(Twist() if age > self.SCAN_TIMEOUT
                               else self.command)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        o = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        if all(math.isfinite(v) for v in (position.x, position.y, yaw)):
            self.odom_pose = (position.x, position.y, yaw)

    # ------------------------------------------------------------------
    # LiDAR processing
    # ------------------------------------------------------------------
    def region_measurements(self, scan):
        """Return {region: (mean distance, mean angle_deg)} of the 3 nearest
        valid rays. (math.inf, None) when a region has no valid ray."""
        max_valid = min(self.detection_limit, scan.range_max) \
            if math.isfinite(scan.range_max) else self.detection_limit
        min_valid = max(scan.range_min, 0.0)

        samples = {name: [] for name in self.regions}
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            if r < min_valid or r > max_valid:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            deg = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
            for name, (low, high) in self.regions.items():
                inside = low <= deg < high
                if name == self.FRONT:
                    inside = low <= deg <= high
                if inside:
                    samples[name].append((r, deg))
                    break

        measurements = {}
        for name, values in samples.items():
            if not values:
                measurements[name] = (math.inf, None)
                continue
            values.sort(key=lambda item: item[0])
            nearest = values[:self.robust_rays]
            measurements[name] = (
                sum(v[0] for v in nearest) / len(nearest),
                sum(v[1] for v in nearest) / len(nearest),
            )
        return measurements

    def closest_region(self, measurements):
        """Closest region inside detection_limit (None if there is none)."""
        valid = {k: v[0] for k, v in measurements.items()
                 if v[0] < self.detection_limit}
        return min(valid, key=valid.get) if valid else None

    def confirmed_region(self, measurements):
        """Accept a new closest region after N consecutive scans."""
        closest = self.closest_region(measurements)
        if closest == self.region_candidate:
            self.region_candidate_count += 1
        else:
            self.region_candidate = closest
            self.region_candidate_count = 1
        if self.region_candidate_count >= self.region_confirmations:
            self.stable_region = self.region_candidate
        return self.stable_region

    def lateral_progress(self):
        if self.front_start_pose is None or self.odom_pose is None:
            return None
        x0, y0, yaw0 = self.front_start_pose
        dx = self.odom_pose[0] - x0
        dy = self.odom_pose[1] - y0
        return abs(-math.sin(yaw0) * dx + math.cos(yaw0) * dy)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'State: {self.state} -> {state}')
        self.state = state

    def make_twist(self, vx=0.0, vy=0.0, wz=0.0):
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            return Twist()
        cmd = Twist()
        cmd.linear.x = clamp(vx, -self.forward_speed, self.forward_speed)
        cmd.linear.y = clamp(vy, -self.lateral_speed, self.lateral_speed)
        cmd.angular.z = clamp(wz, -self.turn_speed, self.turn_speed)
        return cmd

    # ------------------------------------------------------------------
    # States
    # ------------------------------------------------------------------
    def follow_right(self, measurements, closest):
        if closest == self.FRONT and not self.passing_short_obstacle:
            if self.odom_pose is None:
                self.get_logger().warn(
                    'Frontal wall but no /odom yet: stopping.',
                    throttle_duration_sec=2.0)
                return Twist()
            self.front_start_pose = self.odom_pose
            self.obstacle_is_long = False
            self.set_state(self.FRONT)
            return self.follow_front(measurements, closest)

        if self.passing_short_obstacle and closest == self.BACK_RIGHT:
            self.set_state(self.BACK_RIGHT)
            return self.follow_back_right(measurements, closest)

        dist, angle_deg = measurements[self.RIGHT]
        if angle_deg is None:
            # Right wall not visible: keep moving forward, no corrections.
            return self.make_twist(self.forward_speed, 0.0, 0.0)

        vy = -self.lateral_gain * (dist - self.follow_distance)
        if self.passing_short_obstacle:
            # Closest point is the obstacle corner: do not align with it.
            wz = 0.0
        else:
            wz = self.alignment_gain * (math.radians(angle_deg) + math.pi / 2)
        return self.make_twist(self.forward_speed, vy, wz)

    def follow_front(self, measurements, closest):
        progress = self.lateral_progress()
        if progress is None:
            return Twist()
        if closest != self.FRONT:
            # Front cleared before max_front_length: short obstacle.
            self.passing_short_obstacle = True
            self.set_state(self.FRONT_RIGHT)
            return self.follow_front_right(measurements, closest)
        if progress >= self.max_front_length:
            # Long wall: turn left until it becomes the new right wall.
            self.obstacle_is_long = True
            self.set_state(self.FRONT_RIGHT)
            return self.follow_front_right(measurements, closest)
        return self.make_twist(0.0, self.lateral_speed, 0.0)

    def follow_front_right(self, measurements, closest):
        if self.obstacle_is_long:
            if closest == self.RIGHT:
                self.obstacle_is_long = False
                self.passing_short_obstacle = False
                self.set_state(self.RIGHT)
                return self.follow_right(measurements, closest)
            # Fast, fluid left turn: full turn_speed far from the goal and a
            # proportional reduction (with a floor) when the wall approaches
            # -90 deg. Never the slow constant rate that stalled before.
            wz = self.turn_speed
            region = closest if closest in (self.FRONT, self.FRONT_RIGHT) \
                else None
            if region is not None and measurements[region][1] is not None:
                err = math.radians(measurements[region][1]) + math.pi / 2
                wz = clamp(self.alignment_gain * err,
                           self.MIN_TURN_FRACTION * self.turn_speed,
                           self.turn_speed)
            return self.make_twist(0.0, 0.0, wz)

        # Short obstacle: diagonal motion until RIGHT is reached.
        if closest == self.RIGHT:
            self.set_state(self.RIGHT)  # passing_short_obstacle stays True
            return self.follow_right(measurements, closest)
        return self.make_twist(
            self.forward_speed,
            self.diagonal_factor * self.lateral_speed,
            0.0)

    def follow_back_right(self, measurements, closest):
        if closest == self.RIGHT:
            self.passing_short_obstacle = False
            self.set_state(self.RIGHT)
            return self.follow_right(measurements, closest)
        d_back = measurements[self.BACK_RIGHT][0]
        if math.isfinite(d_back):
            vy = -self.lateral_gain * (d_back - self.follow_distance)
        else:
            vy = -self.diagonal_factor * self.lateral_speed
        return self.make_twist(self.forward_speed, vy, 0.0)

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------
    def scan_callback(self, scan):
        self.last_scan_time = self.get_clock().now()
        measurements = self.region_measurements(scan)
        closest = self.confirmed_region(measurements)

        if self.state == self.RIGHT:
            cmd = self.follow_right(measurements, closest)
        elif self.state == self.FRONT:
            cmd = self.follow_front(measurements, closest)
        elif self.state == self.FRONT_RIGHT:
            cmd = self.follow_front_right(measurements, closest)
        elif self.state == self.BACK_RIGHT:
            cmd = self.follow_back_right(measurements, closest)
        else:
            cmd = Twist()
        self.command = cmd

    def stop(self):
        self.command = Twist()
        self.timer.cancel()
        try:
            self.publisher.publish(self.command)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = HolonomicObstacleWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()