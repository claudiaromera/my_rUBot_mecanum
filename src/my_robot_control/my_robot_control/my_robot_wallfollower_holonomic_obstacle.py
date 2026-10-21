#!/usr/bin/env python3
"""Holonomic obstacle wall follower driven directly by laser regions."""

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
    """Follow the sequence FRONT -> FRONT_RIGHT -> RIGHT -> BACK_RIGHT."""

    def __init__(self):
        super().__init__('holonomic_obstacle_wall_follower')

        # Desired distance between the robot and the right wall (m).
        self.follow_distance = float(
            self.declare_parameter('follow_distance', 0.35).value
        )

        # Maximum lateral distance used to classify a short obstacle (m).
        self.max_front_length = float(
            self.declare_parameter('max_front_length', 0.50).value
        )

        # Forward velocity of the robot (m/s).
        self.forward_speed = float(
            self.declare_parameter('forward_speed', 0.25).value
        )

        # Maximum lateral velocity of the robot (m/s).
        self.lateral_speed = float(
            self.declare_parameter('lateral_speed', 0.30).value
        )

        # Angular velocity used when turning (rad/s).
        self.turn_speed = float(
            self.declare_parameter('turn_speed', 0.60).value
        )

        # Maximum LiDAR distance used to detect an obstacle (m).
        self.detection_limit = float(
            self.declare_parameter('detection_limit', 1.20).value
        )

        # Gain used to correct the distance to the right wall.
        self.lateral_gain = float(
            self.declare_parameter('lateral_gain', 0.5).value
        )

        # Gain used to keep the robot parallel to the right wall.
        self.alignment_gain = float(
            self.declare_parameter('alignment_gain', 2.0).value
        )

        # Names of the four states and LiDAR regions.
        self.RIGHT = 'RIGHT'
        self.FRONT = 'FRONT'
        self.FRONT_RIGHT = 'FRONT_RIGHT'
        self.BACK_RIGHT = 'BACK_RIGHT'

        # Simple fixed values used by the controller.
        self.diagonal_factor = 0.5
        self.region_confirmations = 3
        self.robust_rays = 3

        self.state = self.RIGHT
        self.obstacle_is_long = False
        self.command = Twist()
        self.odom_pose = None
        self.front_start_pose = None
        self.right_angle = None

        self.region_candidate = None
        self.region_candidate_count = 0
        self.stable_region = None
        self.last_status = None

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_command)

        self.get_logger().info(
            'Holonomic obstacle wall follower ready: '
            f'follow_distance={self.follow_distance:.2f} m, '
            f'max_front_length={self.max_front_length:.2f} m'
        )

    def publish_command(self):
        self.publisher.publish(self.command)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = [
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.odom_pose = (position.x, position.y, yaw)

    def region_measurements(self, scan):
        """Return a robust distance and angle for each region.

        Angles are normalised so scans expressed as either [-pi, pi] or
        [0, 2*pi] work in the same way. The mean of the nearest three valid
        rays prevents one noisy hardware return from controlling the state.
        """
        regions = {
            self.FRONT: [],
            self.FRONT_RIGHT: [],
            self.RIGHT: [],
            self.BACK_RIGHT: [],
        }

        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            raw_angle = scan.angle_min + index * scan.angle_increment
            angle = math.atan2(math.sin(raw_angle), math.cos(raw_angle))
            angle_deg = math.degrees(angle)

            if -20.0 <= angle_deg <= 20.0:
                regions[self.FRONT].append((distance, angle_deg))
            elif -70.0 <= angle_deg < -20.0:
                regions[self.FRONT_RIGHT].append((distance, angle_deg))
            elif -110.0 <= angle_deg < -70.0:
                regions[self.RIGHT].append((distance, angle_deg))
            elif -160.0 <= angle_deg < -110.0:
                regions[self.BACK_RIGHT].append((distance, angle_deg))

        # For each region, average the distance and angle of its nearest valid
        # LiDAR rays. This reduces the effect of one noisy measurement.
        measurements = {}
        angles = {}
        for name, rays in regions.items():
            nearest = sorted(rays, key=lambda ray: ray[0])[
                :self.robust_rays
            ]
            if nearest:
                measurements[name] = sum(
                    ray[0] for ray in nearest
                ) / len(nearest)
                angles[name] = sum(
                    ray[1] for ray in nearest
                ) / len(nearest)
            else:
                measurements[name] = math.inf
                angles[name] = None

        return measurements, angles

    def closest_region(self, measurements):
        visible = {
            name: distance
            for name, distance in measurements.items()
            if distance < self.detection_limit
        }
        return min(visible, key=visible.get) if visible else None

    def confirmed_region(self, measurements):
        """Debounce region changes over several consecutive laser scans."""
        candidate = self.closest_region(measurements)
        if candidate == self.region_candidate:
            self.region_candidate_count += 1
        else:
            self.region_candidate = candidate
            self.region_candidate_count = 1

        if self.region_candidate_count >= self.region_confirmations:
            self.stable_region = candidate
        return self.stable_region

    def lateral_progress(self):
        """Distance travelled along the initial robot lateral axis."""
        if self.odom_pose is None or self.front_start_pose is None:
            return None

        start_x, start_y, start_yaw = self.front_start_pose
        current_x, current_y, _ = self.odom_pose
        dx = current_x - start_x
        dy = current_y - start_y
        return abs(
            -math.sin(start_yaw) * dx
            + math.cos(start_yaw) * dy
        )

    def change_state(self, new_state, reason):
        if new_state != self.state:
            self.state = new_state
            self.get_logger().info(f'STATE {new_state}: {reason}')

    def follow_right(self, measurements, closest, cmd):
        right_distance = measurements[self.RIGHT]

        if closest == self.FRONT:
            if self.odom_pose is None:
                self.report('RIGHT: frontal obstacle; waiting for /odom')
                return

            self.front_start_pose = self.odom_pose
            self.obstacle_is_long = False
            self.change_state(self.FRONT, 'frontal obstacle detected')
            cmd.linear.y = self.lateral_speed
            return

        if closest == self.BACK_RIGHT:
            self.change_state(
                self.BACK_RIGHT,
                'minimum moved from RIGHT to BACK_RIGHT',
            )
            self.follow_back_right(measurements, closest, cmd)
            return

        if right_distance < self.detection_limit:
            distance_error = right_distance - self.follow_distance
            cmd.linear.x = self.forward_speed
            cmd.linear.y = clamp(
                -self.lateral_gain * distance_error,
                -self.lateral_speed,
                self.lateral_speed,
            )

            # The closest point on a parallel right wall should be at -90 deg.
            if self.right_angle is not None:
                alignment_error = math.radians(self.right_angle + 90.0)
                cmd.angular.z = clamp(
                    self.alignment_gain * alignment_error,
                    -self.turn_speed,
                    self.turn_speed,
                )

            self.report(
                f'RIGHT: distance={right_distance:.2f} m, '
                f'angle={self.right_angle:.1f} deg, '
                f'vy={cmd.linear.y:.2f} m/s, '
                f'w={cmd.angular.z:.2f} rad/s'
            )
        else:
            cmd.linear.x = 0.5 * self.forward_speed
            cmd.linear.y = -self.lateral_speed
            self.report('RIGHT: searching for wall')

    def follow_front(self, measurements, closest, cmd):
        progress = self.lateral_progress()
        if progress is None:
            self.report('FRONT: odometry unavailable; stopped')
            return

        if progress >= self.max_front_length:
            self.obstacle_is_long = True

        if closest == self.FRONT_RIGHT:
            obstacle_type = (
                'long wall' if self.obstacle_is_long else 'short obstacle'
            )
            self.change_state(
                self.FRONT_RIGHT,
                f'{obstacle_type}; progress={progress:.2f} m',
            )
            self.follow_front_right(measurements, closest, cmd)
            return

        if self.obstacle_is_long:
            if closest == self.RIGHT:
                self.change_state(self.RIGHT, 'long front wall reached RIGHT')
                self.follow_right(measurements, closest, cmd)
            else:
                cmd.angular.z = self.turn_speed
                self.report(
                    f'FRONT: long obstacle, turning; progress={progress:.2f} m'
                )
            return

        # Graceful handling if a low-rate scan skips FRONT_RIGHT.
        if closest == self.RIGHT:
            self.change_state(self.RIGHT, 'short obstacle reached RIGHT')
            self.follow_right(measurements, closest, cmd)
            return

        cmd.linear.y = self.lateral_speed
        self.report(
            f'FRONT: lateral progress={progress:.2f} m, closest={closest}'
        )

    def follow_front_right(self, measurements, closest, cmd):
        if closest == self.RIGHT:
            self.change_state(self.RIGHT, 'obstacle moved to RIGHT')
            self.follow_right(measurements, closest, cmd)
            return

        if self.obstacle_is_long:
            cmd.angular.z = self.turn_speed
            self.report(
                f'FRONT_RIGHT: long wall, turning; closest={closest}'
            )
            return

        if closest == self.BACK_RIGHT:
            self.change_state(
                self.BACK_RIGHT,
                'obstacle moved directly to BACK_RIGHT',
            )
            self.follow_back_right(measurements, closest, cmd)
            return

        cmd.linear.x = self.forward_speed
        cmd.linear.y = self.diagonal_factor * self.lateral_speed
        self.report(f'FRONT_RIGHT: diagonal +vx/+vy, closest={closest}')

    def follow_back_right(self, measurements, closest, cmd):
        if closest == self.RIGHT:
            self.change_state(self.RIGHT, 'right wall recovered')
            self.follow_right(measurements, closest, cmd)
            return

        cmd.linear.x = self.forward_speed
        cmd.linear.y = -self.diagonal_factor * self.lateral_speed
        self.report(f'BACK_RIGHT: diagonal +vx/-vy, closest={closest}')

    def scan_callback(self, scan):
        measurements, angles = self.region_measurements(scan)
        self.right_angle = angles[self.RIGHT]
        closest = self.confirmed_region(measurements)
        cmd = Twist()

        if self.state == self.RIGHT:
            self.follow_right(measurements, closest, cmd)
        elif self.state == self.FRONT:
            self.follow_front(measurements, closest, cmd)
        elif self.state == self.FRONT_RIGHT:
            self.follow_front_right(measurements, closest, cmd)
        else:
            self.follow_back_right(measurements, closest, cmd)

        self.command = cmd

    def report(self, status):
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def stop(self):
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
