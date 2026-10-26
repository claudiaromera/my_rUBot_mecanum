#!/usr/bin/env python3
"""Improved holonomic right-wall follower for the mecanum rUBot."""

import math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


class ImprovedHolonomicWallFollower(Node):
    """LiDAR processing, state machine and control laws."""

    FOLLOW_RIGHT = 'FOLLOW_RIGHT'
    FOLLOW_FRONT = 'FOLLOW_FRONT'
    BYPASS_OBSTACLE = 'BYPASS_OBSTACLE'
    TURN_TO_RIGHT = 'TURN_TO_RIGHT'

    BYPASS_FRONT_RIGHT = 'BYPASS_FRONT_RIGHT'
    BYPASS_RIGHT = 'BYPASS_RIGHT'
    BYPASS_BACK_RIGHT = 'BYPASS_BACK_RIGHT'

    # Angular regions in degrees (0 = front, negative = right side).
    # Each region is [low, high); the front region also includes +20 deg.
    REGIONS = {
        'front': (-20.0, 20.0),
        'front_right': (-70.0, -20.0),
        'right': (-110.0, -70.0),
        'back_right': (-160.0, -110.0),
    }

    # Number of nearest valid rays averaged per region.
    NEAREST_RAYS = 3
    # If no scan arrives for this time (s) a zero command is published.
    SCAN_TIMEOUT = 0.5

    def __init__(self):
        super().__init__('improved_holonomic_wall_follower')

        # Controller and geometry parameters.
        self.follow_distance = float(
            self.declare_parameter('follow_distance', 0.35).value
        )
        self.max_front_length = float(
            self.declare_parameter('max_front_length', 0.50).value
        )
        self.forward_speed = float(
            self.declare_parameter('forward_speed', 0.30).value
        )
        self.lateral_speed = float(
            self.declare_parameter('lateral_speed', 0.40).value
        )
        self.distance_gain = float(
            self.declare_parameter('distance_gain', 0.80).value
        )
        self.angle_gain = float(
            self.declare_parameter('angle_gain', 1.20).value
        )
        self.max_angular_speed = float(
            self.declare_parameter('max_angular_speed', 1.00).value
        )
        self.detection_limit = float(
            self.declare_parameter('detection_limit', 1.20).value
        )
        self.back_right_limit_factor = float(
            self.declare_parameter('back_right_limit_factor', 1.50).value
        )

        self.state = self.FOLLOW_RIGHT
        self.bypass_phase = self.BYPASS_FRONT_RIGHT
        self.command = Twist()
        self.odom_pose = None
        self.front_start_pose = None
        self.last_scan_time = None

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_command)

        self.get_logger().info(
            f'Improved holonomic wall follower started in state {self.state}.'
        )

    # ------------------------------------------------------------------
    # Publishing / odometry
    # ------------------------------------------------------------------
    def publish_command(self):
        """Publish the last command periodically (zero if scans are stale)."""
        if self.last_scan_time is None:
            self.publisher.publish(Twist())
            return
        age = (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9
        if age > self.SCAN_TIMEOUT:
            self.publisher.publish(Twist())
            return
        self.publisher.publish(self.command)

    def odom_callback(self, msg):
        """Store x, y and yaw from /odom."""
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        if not all(math.isfinite(v) for v in (position.x, position.y, yaw)):
            return
        self.odom_pose = (position.x, position.y, yaw)

    # ------------------------------------------------------------------
    # LiDAR processing
    # ------------------------------------------------------------------
    def process_scan(self, scan):
        """Return {region: (distance, angle_rad) or None}."""
        max_valid = self.detection_limit
        if math.isfinite(scan.range_max):
            max_valid = min(max_valid, scan.range_max)
        min_valid = max(scan.range_min, 0.0)

        samples = {name: [] for name in self.REGIONS}
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            if r < min_valid or r > max_valid:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            deg = math.degrees(angle)
            for name, (low, high) in self.REGIONS.items():
                inside = low <= deg < high
                if name == 'front':
                    inside = low <= deg <= high
                if inside:
                    samples[name].append((r, angle))
                    break

        regions = {}
        for name, values in samples.items():
            if not values:
                regions[name] = None
                continue
            values.sort(key=lambda item: item[0])
            nearest = values[:self.NEAREST_RAYS]
            distance = sum(v[0] for v in nearest) / len(nearest)
            angle = sum(v[1] for v in nearest) / len(nearest)
            regions[name] = (distance, angle)
        return regions

    @staticmethod
    def closest_region(regions):
        """Name of the region with the smallest distance (None if empty)."""
        valid = {k: v for k, v in regions.items() if v is not None}
        if not valid:
            return None
        return min(valid, key=lambda k: valid[k][0])

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'State: {self.state} -> {state}')
        self.state = state
        if state == self.BYPASS_OBSTACLE:
            self.bypass_phase = self.BYPASS_FRONT_RIGHT

    def set_bypass_phase(self, phase):
        if phase != self.bypass_phase:
            self.get_logger().info(
                f'Bypass phase: {self.bypass_phase} -> {phase}'
            )
        self.bypass_phase = phase

    def lateral_progress(self):
        """Odometric displacement along the initial lateral axis."""
        if self.front_start_pose is None or self.odom_pose is None:
            return None
        x0, y0, yaw0 = self.front_start_pose
        x, y, _ = self.odom_pose
        dx = x - x0
        dy = y - y0
        return abs(-math.sin(yaw0) * dx + math.cos(yaw0) * dy)

    def make_twist(self, vx=0.0, vy=0.0, wz=0.0):
        """Build a bounded Twist; any non-finite value gives a zero Twist."""
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            return Twist()
        cmd = Twist()
        cmd.linear.x = self.clamp(vx, self.forward_speed)
        cmd.linear.y = self.clamp(vy, self.lateral_speed)
        cmd.angular.z = self.clamp(wz, self.max_angular_speed)
        return cmd

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------
    def scan_callback(self, scan):
        """Read the LiDAR, update the state and calculate self.command."""
        self.last_scan_time = self.get_clock().now()

        regions = self.process_scan(scan)
        if all(v is None for v in regions.values()):
            self.get_logger().warn(
                'No valid LiDAR measurements: stopping.',
                throttle_duration_sec=2.0,
            )
            self.command = Twist()
            return

        if not self.update_state(regions):
            self.command = Twist()
            return
        self.command = self.compute_command(regions)

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------
    def update_state(self, regions):
        """Apply state transitions. Return False if it is unsafe to move."""
        front = regions['front']
        closest = self.closest_region(regions)
        front_detect = 1.5 * self.follow_distance
        front_clear = 2.0 * self.follow_distance  # hysteresis

        if self.state == self.FOLLOW_RIGHT:
            if front is not None and front[0] <= front_detect:
                if self.odom_pose is None:
                    self.get_logger().warn(
                        'Frontal wall detected but no /odom received yet: '
                        'stopping.',
                        throttle_duration_sec=2.0,
                    )
                    return False
                self.front_start_pose = self.odom_pose
                self.set_state(self.FOLLOW_FRONT)

        elif self.state == self.FOLLOW_FRONT:
            progress = self.lateral_progress()
            if progress is None:
                return False
            blocked = front is not None and front[0] < front_clear
            if blocked and progress >= self.max_front_length:
                self.set_state(self.TURN_TO_RIGHT)
            elif not blocked:
                self.set_state(self.BYPASS_OBSTACLE)

        elif self.state == self.BYPASS_OBSTACLE:
            if self.bypass_phase == self.BYPASS_FRONT_RIGHT:
                if closest == 'right':
                    self.set_bypass_phase(self.BYPASS_RIGHT)
            elif self.bypass_phase == self.BYPASS_RIGHT:
                if closest == 'back_right':
                    self.set_bypass_phase(self.BYPASS_BACK_RIGHT)
            elif self.bypass_phase == self.BYPASS_BACK_RIGHT:
                if closest == 'right':
                    self.set_state(self.FOLLOW_RIGHT)

        elif self.state == self.TURN_TO_RIGHT:
            if closest == 'right':
                self.set_state(self.FOLLOW_RIGHT)

        return True

    # ------------------------------------------------------------------
    # Control laws
    # ------------------------------------------------------------------
    def compute_command(self, regions):
        front = regions['front']
        right = regions['right']
        back_right = regions['back_right']
        fd = self.follow_distance

        if self.state == self.FOLLOW_RIGHT:
            if right is None:
                self.get_logger().warn(
                    'Right wall not visible: stopping.',
                    throttle_duration_sec=2.0,
                )
                return Twist()
            dist, angle = right
            vx = self.forward_speed
            vy = -self.distance_gain * (dist - fd)
            wz = self.angle_gain * (angle + math.pi / 2.0)
            return self.make_twist(vx, vy, wz)

        if self.state == self.FOLLOW_FRONT:
            if front is None:
                return Twist()
            dist, angle = front
            vx = self.distance_gain * (dist - fd)
            vy = self.lateral_speed
            wz = self.angle_gain * angle
            return self.make_twist(vx, vy, wz)

        if self.state == self.BYPASS_OBSTACLE:
            # Emergency stop if something appears too close in front.
            if front is not None and front[0] < 0.8 * fd:
                self.get_logger().warn(
                    'Obstacle too close during bypass: stopping.',
                    throttle_duration_sec=2.0,
                )
                return Twist()

            vx = 0.5 * self.forward_speed
            vy_nominal = 0.25 * self.lateral_speed

            if self.bypass_phase == self.BYPASS_FRONT_RIGHT:
                return self.make_twist(vx, vy_nominal, 0.0)

            if self.bypass_phase == self.BYPASS_RIGHT:
                if right is None:
                    return self.make_twist(vx, 0.0, 0.0)
                vy = -self.distance_gain * (right[0] - fd)
                vy = self.clamp(vy, 0.5 * self.lateral_speed)
                return self.make_twist(vx, vy, 0.0)

            # BYPASS_BACK_RIGHT
            vy_mag = vy_nominal
            limit = self.back_right_limit_factor * fd
            d_back = back_right[0] if back_right is not None \
                else self.detection_limit
            if d_back > limit:
                vy_mag += self.distance_gain * (d_back - limit)
            vy_mag = min(vy_mag, self.lateral_speed)
            return self.make_twist(vx, -vy_mag, 0.0)

        if self.state == self.TURN_TO_RIGHT:
            return self.make_twist(0.0, 0.0, 0.5 * self.max_angular_speed)

        return Twist()

    def stop(self):
        """Publish one zero command before destroying the node."""
        self.command = Twist()
        self.timer.cancel()
        try:
            self.publisher.publish(self.command)
        except Exception:
            pass


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = ImprovedHolonomicWallFollower()
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
