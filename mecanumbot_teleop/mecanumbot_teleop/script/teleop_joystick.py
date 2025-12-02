"""
Control Your TurtleBot3 friend - Mecanumbot !
---------------------------
Moving around:

 left button up/down: increase/decrease linear x velocity (Mecanumbot : ~ 0.26)
 left button left/right: decrease/increase linear y velocity (Mecanumbot : ~ 0.26)
 right button left/right: decrease/increase angular velocity (Mecanumbot : ~ 1.82)


Moving accessories:
        Y    
   X    A    B
        
Y/A : cam up/down
X/B : open/close gripper 

Back : force stop

CTRL-C to quit
"""


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from mecanumbot_msgs.msg import AccessMotorCmd

# --- constants (same as your original values) ---
MECANUMBOT_MAX_LIN_VEL = 0.234
MECANUMBOT_MAX_ANG_VEL = 1.092

MECANUMBOT_MIN_CAM_POS = 2.0
MECANUMBOT_MAX_CAM_POS = 8.6
MECANUMBOT_MID_CAM_POS = (MECANUMBOT_MIN_CAM_POS + MECANUMBOT_MAX_CAM_POS) / 2.0

MECANUMBOT_MIN_GRIPPER_POS = 1.6
MECANUMBOT_FRONT_GRIPPER_POS = 5.12
MECANUMBOT_MAX_GRIPPER_POS = 8.54

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.05
SENSITIVITY_THRESH = 0.05

PUBLISH_HZ = 5.0


def make_simple_profile(output_vel, input_vel, slop):
    if input_vel > output_vel:
        output_vel = min(input_vel, output_vel + slop)
    elif input_vel < output_vel:
        output_vel = max(input_vel, output_vel - slop)
    else:
        output_vel = input_vel
    return output_vel


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        return low_bound
    if input_vel > high_bound:
        return high_bound
    return input_vel


class MecanumbotJoyTeleop(Node):
    def __init__(self):
        super().__init__('mecanumbot_joystick')

        qos = QoSProfile(depth=10)
        self.pub_vel = self.create_publisher(Twist, '/mecanumbot/cmd_vel', qos)
        self.pub_accessory = self.create_publisher(AccessMotorCmd, '/mecanumbot/cmd_accessory_pos', qos)

        # Parameters: allow remapping of axes/buttons if needed
        self.declare_parameter('axes.linear_x', 1)
        self.declare_parameter('axes.linear_y', 0)
        self.declare_parameter('axes.angular', 3)

        # Buttons defaults for Xbox-style controllers:
        # A=0, B=1, X=2, Y=3, LB=4, RB=5, Back=6, Start=7
        self.declare_parameter('buttons.A', 0)
        self.declare_parameter('buttons.B', 1)
        self.declare_parameter('buttons.X', 2)
        self.declare_parameter('buttons.Y', 3)
        self.declare_parameter('buttons.Back', 6)

        # Sensitivity and profiling
        self.declare_parameter('sensitivity.deadzone', SENSITIVITY_THRESH)
        self.declare_parameter('smoothing.lin_step', LIN_VEL_STEP_SIZE)
        self.declare_parameter('smoothing.ang_step', ANG_VEL_STEP_SIZE)
        self.declare_parameter('max_lin_vel', MECANUMBOT_MAX_LIN_VEL)
        self.declare_parameter('max_ang_vel', MECANUMBOT_MAX_ANG_VEL)
        self.declare_parameter('publish_hz', PUBLISH_HZ)

        # Read params
        self.ax_lin_x_idx = int(self.get_parameter('axes.linear_x').value)
        self.ax_lin_y_idx = int(self.get_parameter('axes.linear_y').value)
        self.ax_ang_idx = int(self.get_parameter('axes.angular').value)

        self.btn_a = int(self.get_parameter('buttons.A').value)
        self.btn_b = int(self.get_parameter('buttons.B').value)
        self.btn_x = int(self.get_parameter('buttons.X').value)
        self.btn_y = int(self.get_parameter('buttons.Y').value)
        self.btn_back = int(self.get_parameter('buttons.Back').value)

        self.deadzone = float(self.get_parameter('sensitivity.deadzone').value)
        self.lin_step = float(self.get_parameter('smoothing.lin_step').value)
        self.ang_step = float(self.get_parameter('smoothing.ang_step').value)
        self.max_lin = float(self.get_parameter('max_lin_vel').value)
        self.max_ang = float(self.get_parameter('max_ang_vel').value)

        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.get_logger().info(
            f'Using axes: lin_x={self.ax_lin_x_idx}, lin_y={self.ax_lin_y_idx}, ang={self.ax_ang_idx}; '
            f'buttons: A={self.btn_a},B={self.btn_b},X={self.btn_x},Y={self.btn_y},Back={self.btn_back}'
        )

        # State
        self.target_vel = [0.0, 0.0, 0.0]   # [lin_x, lin_y, ang_z]
        self.control_vel = [0.0, 0.0, 0.0]
        self.control_access_pos = [
            MECANUMBOT_MAX_CAM_POS,
            MECANUMBOT_FRONT_GRIPPER_POS,
            MECANUMBOT_FRONT_GRIPPER_POS
        ]  # [cam, left_gripper, right_gripper]

        # Subscribe
        self.create_subscription(Joy, 'joy', self.joy_callback, qos)

        # Timer publish
        period = 1.0 / max(1.0, self.publish_hz)
        self.timer = self.create_timer(period, self.timer_publish)

        self.get_logger().info('Mecanumbot joystick teleop node started')

    def joy_axis_safe(self, axes, idx):
        try:
            return float(axes[idx])
        except (IndexError, TypeError):
            return 0.0

    def joy_button_safe(self, buttons, idx):
        try:
            return int(buttons[idx])
        except (IndexError, TypeError):
            return 0

    def joy_callback(self, joy_msg: Joy):
        # read axes (with safe indexing)
        x_in = self.joy_axis_safe(joy_msg.axes, self.ax_lin_x_idx)
        y_in = self.joy_axis_safe(joy_msg.axes, self.ax_lin_y_idx)
        ang_in = self.joy_axis_safe(joy_msg.axes, self.ax_ang_idx)

        # apply deadzone
        if abs(x_in) < self.deadzone:
            x_in = 0.0
        if abs(y_in) < self.deadzone:
            y_in = 0.0
        if abs(ang_in) < self.deadzone:
            ang_in = 0.0

        # map to target velocities
        self.target_vel[0] = x_in * self.max_lin
        self.target_vel[1] = y_in * self.max_lin
        self.target_vel[2] = ang_in * self.max_ang

        # accessory buttons: use Xbox mapping defaults:
        # Y -> cam up, A -> cam mid/down, X -> open gripper, B -> close gripper
        if self.joy_button_safe(joy_msg.buttons, self.btn_y) == 1:
            self.control_access_pos[0] = MECANUMBOT_MAX_CAM_POS
            self.get_logger().debug('Button Y pressed: cam -> MAX')
        elif self.joy_button_safe(joy_msg.buttons, self.btn_a) == 1:
            self.control_access_pos[0] = MECANUMBOT_MID_CAM_POS
            self.get_logger().debug('Button A pressed: cam -> MID')

        if self.joy_button_safe(joy_msg.buttons, self.btn_x) == 1:
            # open gripper (approx)
            self.control_access_pos[1] = (MECANUMBOT_MAX_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS) / 2.0
            self.control_access_pos[2] = (MECANUMBOT_MIN_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS) / 2.0
            self.get_logger().debug('Button X pressed: gripper -> OPEN')
        elif self.joy_button_safe(joy_msg.buttons, self.btn_b) == 1:
            # close gripper (approx)
            self.control_access_pos[1] = (MECANUMBOT_MIN_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS) / 2.0
            self.control_access_pos[2] = (MECANUMBOT_MAX_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS) / 2.0
            self.get_logger().debug('Button B pressed: gripper -> CLOSE')

        # emergency stop (Back button)
        if self.joy_button_safe(joy_msg.buttons, self.btn_back) == 1:
            self.target_vel = [0.0, 0.0, 0.0]
            self.control_vel = [0.0, 0.0, 0.0]
            self.get_logger().warn('Emergency STOP pressed (Back). Zeroing velocities.')

        # debug log
        self.get_logger().debug(f'Joy received axes={joy_msg.axes} buttons={joy_msg.buttons}')
        self.get_logger().debug(f'Updated target_vel={self.target_vel}')

    def timer_publish(self):
        '''# smoothly move control_vel towards target_vel
        self.control_vel[0] = make_simple_profile(self.control_vel[0], self.target_vel[0], self.lin_step / 2.0)
        self.control_vel[1] = make_simple_profile(self.control_vel[1], self.target_vel[1], self.lin_step / 2.0)
        self.control_vel[2] = make_simple_profile(self.control_vel[2], self.target_vel[2], self.ang_step / 2.0)'''
        self.control_vel = self.target_vel

        # enforce limits
        self.control_vel[0] = constrain(self.control_vel[0], -self.max_lin, self.max_lin)
        self.control_vel[1] = constrain(self.control_vel[1], -self.max_lin, self.max_lin)
        self.control_vel[2] = constrain(self.control_vel[2], -self.max_ang, self.max_ang)

        # publish twist
        twist = Twist()
        twist.linear.x = float(self.control_vel[0])
        twist.linear.y = float(self.control_vel[1])
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(self.control_vel[2])

        self.pub_vel.publish(twist)

        # publish accessory pos
        pos_msg = AccessMotorCmd()
        pos_msg.n_pos = float(self.control_access_pos[0])
        pos_msg.gl_pos = float(self.control_access_pos[1])
        pos_msg.gr_pos = float(self.control_access_pos[2])
        self.pub_accessory.publish(pos_msg)

        # occasional info log (reduce spam)
        # Use DEBUG to see everything; INFO every second
        self.get_logger().debug(f'Published Twist: x={twist.linear.x:.3f}, y={twist.linear.y:.3f}, z={twist.angular.z:.3f}')

    def destroy_node(self):
        # cancel timer and then destruct
        try:
            self.timer.cancel()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumbotJoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()