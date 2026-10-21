import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class DifferentialWallFollower(Node):
    def __init__(self):
        super().__init__('differential_wall_follower')

        # Parameters
        self.declare_parameter('follow_distance', 0.35)  # desired distance to right wall
        self.declare_parameter('forward_speed', 0.30)    # linear speed
        self.declare_parameter('turn_speed', 0.50)       # angular speed
        self.declare_parameter('tolerance', 0.05)        # band around follow_distance
        self.declare_parameter('right_detection_limit', 1.0)  # max distance to consider RIGHT wall visible

        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.v_lin = float(self.get_parameter('forward_speed').value)
        self.v_ang = float(self.get_parameter('turn_speed').value)
        self.tol = float(self.get_parameter('tolerance').value)
        self.right_detection_limit = float(
            self.get_parameter('right_detection_limit').value
        )

        # Last commanded twist (will be published periodically)
        self.cmd = Twist()

        # ROS 2 entities
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timers
        self.info_timer = self.create_timer(1.0, self.log_info)

        # Periodic cmd_vel publisher at 10 Hz (0.1 s)
        self.cmd_timer = self.create_timer(0.1, self.cmd_publish_timer_cb)

        self._state_action = "Idle"
        self._last_action_logged = None
        self._shutting_down = False

        self.get_logger().info(
            "Differential wall follower: RIGHT tolerance and BACK_RIGHT search."
        )

    #--------------------------------------------------------------------
    def stop(self):
        """Stop timers and publish one zero command before destroying the node."""
        self._shutting_down = True

        # Set last command to zero
        self.cmd = Twist()

        # Cancel timers safely
        for t in [self.info_timer, self.cmd_timer]:
            try:
                t.cancel()
            except Exception:
                pass

        # The ROS context is still active because main() handles Ctrl+C.
        try:
            self.publisher.publish(self.cmd)
        except Exception:
            pass

    #--------------------------------------------------------------------
    def cmd_publish_timer_cb(self):
        """Periodic publisher: send the latest cmd_vel at 10 Hz."""
        if self._shutting_down:
            return

        try:
            self.publisher.publish(self.cmd)
        except Exception:
            # If the context or publisher is invalid, ignore
            pass

    #--------------------------------------------------------------------
    def laser_callback(self, scan):
        """Compute control action from LIDAR and update self.cmd."""
        if self._shutting_down:
            return

        angle_min = math.degrees(scan.angle_min)
        angle_inc = math.degrees(scan.angle_increment)

        FRONT       = []
        FR_RIGHT    = []
        RIGHT       = []
        BACK_RIGHT  = []

        for i, d in enumerate(scan.ranges):
            if not math.isfinite(d):
                continue
            if d < scan.range_min or d > scan.range_max:
                continue

            ang = angle_min + i * angle_inc

            if -20 <= ang <= 20:
                FRONT.append(d)
            elif -70 <= ang < -20:
                FR_RIGHT.append(d)
            elif -110 <= ang < -70:
                RIGHT.append(d)
            elif -160 <= ang < -110:
                BACK_RIGHT.append(d)

        # Minimal distances
        # Mean of the three nearest rays: less sensitive to one noisy sample.
        def robust_min(values):
            nearest = sorted(values)[:3]
            return sum(nearest) / len(nearest) if nearest else float('inf')

        min_front = robust_min(FRONT)
        min_fr_right = robust_min(FR_RIGHT)
        min_right = robust_min(RIGHT)
        min_back_right = robust_min(BACK_RIGHT)

        twist = Twist()
        action = ""

        #----------------------------------------------------------
        # RULE 1: FRONT obstacle → turn left
        #----------------------------------------------------------
        if min_front < self.follow_distance:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 2.0 * self.v_ang
            action = f"FRONT {min_front:.2f} m → turn LEFT"

        #----------------------------------------------------------
        # RULE 2: FRONT-RIGHT obstacle → turn left
        #----------------------------------------------------------
        elif min_fr_right < self.follow_distance:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 2.0 * self.v_ang
            action = f"FRONT-RIGHT {min_fr_right:.2f} m → turn LEFT"

        #----------------------------------------------------------
        # RULE 3: RIGHT visible → control with tolerance band (no vy)
        #----------------------------------------------------------
        elif min_right < self.right_detection_limit:
            # error > 0 → too far; error < 0 → too close
            error = min_right - self.follow_distance

            if abs(error) <= self.tol:
                # Inside band: go straight
                twist.linear.x = self.v_lin
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                action = (
                    f"RIGHT ~OK ({min_right:.2f} m, target "
                    f"{self.follow_distance:.2f}±{self.tol:.2f}) → STRAIGHT"
                )

            elif error < 0:
                # Too close to right wall → slow forward + stronger left turn
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = 0.0
                twist.angular.z = self.v_ang * 1.0
                action = (
                    f"RIGHT too CLOSE ({min_right:.2f} m < "
                    f"{self.follow_distance:.2f}-{self.tol:.2f}) → "
                    f"forward + strong LEFT turn"
                )

            else:
                # Too far from right wall → slow forward + stronger right turn
                twist.linear.x = self.v_lin * 0.5
                twist.linear.y = 0.0
                twist.angular.z = -self.v_ang * 1.0
                action = (
                    f"RIGHT too FAR ({min_right:.2f} m > "
                    f"{self.follow_distance:.2f}+{self.tol:.2f}) → "
                    f"forward + strong RIGHT turn"
                )

        #----------------------------------------------------------
        # RULE 4: BACK-RIGHT → only if it is the most relevant wall
        #----------------------------------------------------------
        elif (
            min_right >= self.right_detection_limit
            and math.isfinite(min_back_right)
            and min_back_right < min_right
        ):
            twist.linear.x = self.v_lin * 0.1
            twist.linear.y = 0.0
            twist.angular.z = -2.0 * self.v_ang
            action = (
                f"BACK-RIGHT {min_back_right:.2f} m → "
                f"very slow + STRONG RIGHT turn (2*w)"
            )

        #----------------------------------------------------------
        # RULE 5: No wall visible -> search gently to the right
        #----------------------------------------------------------
        else:
            twist.linear.x = self.v_lin
            twist.linear.y = 0.0
            twist.angular.z = -0.5 * self.v_ang
            action = "No wall detected -> forward + gentle RIGHT search"

        # Update last commanded twist (periodic timer will publish it)
        self.cmd = twist

        # Logging (only on change)
        if action != self._last_action_logged:
            self.get_logger().info(action if action else "No action (stopped).")
            self._last_action_logged = action

        self._state_action = action if action else "Stopped (no wall detected)"

    #--------------------------------------------------------------------
    def log_info(self):
        if not self._shutting_down:
            self.get_logger().info(self._state_action)

def main(args=None):
    # Let Python raise KeyboardInterrupt before rclpy closes its context.
    # This keeps the publisher valid while stop() sends the zero command.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = DifferentialWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
