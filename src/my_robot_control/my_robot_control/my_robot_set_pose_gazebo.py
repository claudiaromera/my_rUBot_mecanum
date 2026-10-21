#!/usr/bin/env python3
"""Move a Gazebo model to a new planar pose using Ignition set_pose."""

import math
import re
import subprocess

import rclpy
from rclpy.node import Node


class RobotSetPose(Node):
    """Call Gazebo once and then finish."""

    VALID_NAME = re.compile(r'^[A-Za-z0-9_.-]+$')

    def __init__(self):
        super().__init__('robot_set_pose_node')

        self.world_name = str(
            self.declare_parameter(
                'world_name', 'square4m_wall_ign'
            ).value
        )
        self.robot_name = str(
            self.declare_parameter('robot_name', 'rubot_mecanum').value
        )
        self.x = float(self.declare_parameter('x', 0.0).value)
        self.y = float(self.declare_parameter('y', 0.0).value)
        self.yaw = float(self.declare_parameter('yaw', 0.0).value)

        self.done = False
        self.timer = self.create_timer(0.1, self.set_pose)

    def set_pose(self):
        """Send the requested planar pose through Ignition Transport."""
        self.timer.cancel()

        if not self.VALID_NAME.fullmatch(self.world_name):
            self.get_logger().error('Invalid world_name parameter.')
            self.done = True
            return

        if not self.VALID_NAME.fullmatch(self.robot_name):
            self.get_logger().error('Invalid robot_name parameter.')
            self.done = True
            return

        half_yaw = 0.5 * self.yaw
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        request = (
            f'name: "{self.robot_name}", '
            f'position: {{x: {self.x}, y: {self.y}}}, '
            f'orientation: {{z: {qz}, w: {qw}}}'
        )
        command = [
            'ign', 'service',
            '-s', f'/world/{self.world_name}/set_pose',
            '--reqtype', 'ignition.msgs.Pose',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '5000',
            '--req', request,
        ]

        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=6.0,
                check=False,
            )
        except FileNotFoundError:
            self.get_logger().error(
                'The ign command was not found. This node requires '
                'Gazebo Ignition Fortress.'
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error('Gazebo set_pose service timed out.')
        else:
            output = (result.stdout or result.stderr).strip()
            if result.returncode == 0 and 'data: true' in output.lower():
                self.get_logger().info(
                    f'Moved {self.robot_name} to '
                    f'x={self.x:.2f}, y={self.y:.2f}, '
                    f'yaw={self.yaw:.2f} rad.'
                )
            elif result.returncode == 0:
                self.get_logger().info(
                    f'set_pose request completed: {output}'
                )
            else:
                self.get_logger().error(
                    f'set_pose failed ({result.returncode}): {output}'
                )
        finally:
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = RobotSetPose()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
