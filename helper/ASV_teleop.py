import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from pynput import keyboard


def clamp(value, min_value, max_value):
    """Clamp the value to the range [min_value, max_value]."""
    return max(min_value, min(value, max_value))


def convert_steering_to_pwm(steer):
    """Map steering value to PWM based on the given formula."""
    if steer >= 300.0:
        return 2000.0
    elif 0 <= steer < 300.0:
        return 1550.0 + (steer * 1.6667)
    elif -300.0 <= steer < 0:
        return 1450.0 + (steer * 1.6667)
    elif steer < -300.0:
        return 1000.0


def convert_thrust_to_pwm(thrust):
    """Convert thrust level to PWM signal."""
    if thrust < 0.0:
        pwm = 3.9 * thrust + 1450.0
        return clamp(pwm, 1000.0, 1500.0)
    else:
        pwm = 3.9 * thrust + 1550.0
        return clamp(pwm, 1500.0, 2000.0)


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        # Publishers
        self.publisher_ = self.create_publisher(Float64MultiArray, '/bada/ros_actuator', 10)
        self.mpcvis_pub = self.create_publisher(Float64MultiArray, '/ship/utm', 10)

        # Inputs
        self.pwm_steer = 0.0
        self.pwm_thrust = 0.0
        self.target_steer = 0.0
        self.target_thrust = 0.0

        # Rate limits (units per 0.1 s timer)
        self.max_steer_rate = 0.1     # deg/0.1s (≈100 deg/s)
        self.max_thrust_rate = 0.1     # throttle units per 0.1s

        # Timer 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def timer_callback(self):
        # --- Apply rate limiting ---
        steer_diff = self.target_steer - self.pwm_steer
        thrust_diff = self.target_thrust - self.pwm_thrust

        # Limit change per cycle
        steer_step = clamp(steer_diff, -self.max_steer_rate, self.max_steer_rate)
        thrust_step = clamp(thrust_diff, -self.max_thrust_rate, self.max_thrust_rate)

        # Update actual outputs gradually
        self.pwm_steer += steer_step
        self.pwm_thrust += thrust_step

        # Clamp absolute bounds
        self.pwm_steer = clamp(self.pwm_steer, -300.0, 300.0)
        self.pwm_thrust = clamp(self.pwm_thrust, -100.0, 100.0)

        # --- Publish actuator message ---
        actuator_msg = Float64MultiArray()
        actuator_msg.data = [
            convert_steering_to_pwm(self.pwm_steer),
            convert_thrust_to_pwm(self.pwm_thrust),
            0.0, 0.0
        ]
        self.publisher_.publish(actuator_msg)
        self.get_logger().info(
            f"Thrust={self.pwm_thrust:.1f} (target={self.target_thrust:.1f}), "
            f"Steer={self.pwm_steer:.1f} (target={self.target_steer:.1f})"
        )

        # Dummy visualization topic
        mpcvis_msg = Float64MultiArray()
        mpcvis_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.mpcvis_pub.publish(mpcvis_msg)

    def on_press(self, key):
        try:
            if key == keyboard.Key.up:
                self.target_thrust += 1.0
                self.get_logger().info(f"Target thrust ↑ {self.target_thrust}")
            elif key == keyboard.Key.down:
                self.target_thrust -= 1.0
                self.get_logger().info(f"Target thrust ↓ {self.target_thrust}")
            elif key == keyboard.Key.left:
                self.target_steer += 1.0
                self.get_logger().info(f"Target steer ← {self.target_steer}")
            elif key == keyboard.Key.right:
                self.target_steer -= 1.0
                self.get_logger().info(f"Target steer → {self.target_steer}")
            elif key == keyboard.Key.esc:
                self.listener.stop()
                self.get_logger().info("Escape pressed → stopping keyboard listener.")
            elif key == keyboard.KeyCode.from_char('r'):
                self.reset_values()
        except AttributeError:
            pass

    def reset_values(self):
        self.target_steer = 0.0
        self.target_thrust = 0.0
        self.get_logger().info("Reset target steer/thrust to zero.")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
