#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Darby Lim

import os
import select
import sys
import time
from geometry_msgs.msg import Twist
from mecanumbot_msgs.msg import AccessMotorCmd
import rclpy
from rclpy.clock import Clock
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty
MECANUMBOT_MAX_LIN_VEL = 0.26
MECANUMBOT_MAX_ANG_VEL = 1.82

MECANUMBOT_MIN_CAM_POS = 2.0
MECANUMBOT_MAX_CAM_POS = 8.6
MECANUMBOT_MID_CAM_POS = (MECANUMBOT_MIN_CAM_POS + MECANUMBOT_MAX_CAM_POS)/2

MECANUMBOT_MIN_GRIPPER_POS = 1.6
MECANUMBOT_FRONT_GRIPPER_POS = 5.12
MECANUMBOT_MAX_GRIPPER_POS = 8.54
SLEEP_TIME = 0.01
WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3 friend - Mecanumbot !
---------------------------
Moving around:
   q    w    e
   a    s    d
        x

        
w/x : increase/decrease linear x velocity (Mecanumbot : ~ 0.26)
q/e : decrease/increase angular velocity (Mecanumbot : ~ 1.82)
a/d : decrease/increase linear y velocity (Mecanumbot : ~ 0.26)

Moving accessories:
        i    
   j    k    l
        
i/k : cam up/down
j/l : open/close gripper 

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity):
    print('currently:\tlinear x velocity {0}\t linear y velocity {1}\t angular velocity {2} '.format(
        target_linear_x_velocity,
        target_linear_y_velocity,
        target_angular_velocity))


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
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MECANUMBOT_MAX_LIN_VEL, MECANUMBOT_MAX_LIN_VEL)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MECANUMBOT_MAX_ANG_VEL, MECANUMBOT_MAX_ANG_VEL)

def create_twistcmd(linear_x, linear_y, angular_z):
    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = linear_y
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular_z

    return twist

def create_poscmd(N,GL,GR):
    posi = AccessMotorCmd()
    posi.n_pos = N
    posi.gl_pos = GL
    posi.gr_pos = GR

    return posi

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    qos = QoSProfile(depth=0)
    node = rclpy.create_node('mecanumbot_keyboard')
    pub_vel = node.create_publisher(Twist, 'mecanumbot/cmd_vel', qos)
    pub_accessory = node.create_publisher(AccessMotorCmd, 'mecanumbot/cmd_accessory_pos', qos)
    status = 0
    target_linear_x_velocity  = 0.0
    target_linear_y_velocity  = 0.0
    target_angular_velocity   = 0.0
    control_linear_x_velocity = 0.0
    control_linear_y_velocity = 0.0
    control_angular_velocity  = 0.0

    control_access_pos = [MECANUMBOT_MAX_CAM_POS,MECANUMBOT_MAX_GRIPPER_POS,MECANUMBOT_MIN_GRIPPER_POS] #cam, left gripper, right gripper
    key_handled = False  
    try:
        print(msg)
        while (1):
            key = get_key(settings)
            if key == 'w':
                target_linear_x_velocity =\
                    check_linear_limit_velocity(target_linear_x_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
                key_handled = True
            elif key == 'x':
                target_linear_x_velocity =\
                    check_linear_limit_velocity(target_linear_x_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
                key_handled = True  
            elif key == 'a': # left, positive y direction
                target_linear_y_velocity =\
                    check_linear_limit_velocity(target_linear_y_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                key_handled = True
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
            elif key == 'd': # right, negative y direction
                target_linear_y_velocity =\
                    check_linear_limit_velocity(target_linear_y_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                key_handled = True
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
            elif key == 'q': # left rotation, positive angular direction
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                key_handled = True
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
            elif key == 'e': # right rotation, negative angular direction
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                key_handled = True
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
            elif key == ' ' or key == 's':
                target_linear_x_velocity = 0.0
                target_linear_y_velocity = 0.0
                control_linear_x_velocity = 0.0
                control_linear_y_velocity = 0.0
                target_angular_velocity = 0.0
                control_angular_velocity = 0.0
                key_handled = True
                print_vels(target_linear_x_velocity, target_linear_y_velocity, target_angular_velocity)
            elif key == 'i': #cam_up
                key_handled = True
                control_access_pos[0] = MECANUMBOT_MAX_CAM_POS
            elif key == 'k': #cam_down
                key_handled = True
                control_access_pos[0] = (MECANUMBOT_MIN_CAM_POS + MECANUMBOT_MAX_CAM_POS)/2
            elif key == 'j': #open gripper,rot around robot axis z
                key_handled = True
                control_access_pos[1] = MECANUMBOT_MAX_GRIPPER_POS
                control_access_pos[2] = MECANUMBOT_MIN_GRIPPER_POS
            elif key == 'l': #close gripper, rot around robot axis z
                key_handled = True  
                control_access_pos[1] = MECANUMBOT_MIN_GRIPPER_POS
                control_access_pos[2] = MECANUMBOT_MAX_GRIPPER_POS
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            control_linear_x_velocity = make_simple_profile(
                control_linear_x_velocity,
                target_linear_x_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))
            
            control_linear_y_velocity = make_simple_profile(
                control_linear_y_velocity,
                target_linear_y_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))
            twist = create_twistcmd(control_linear_x_velocity, control_linear_y_velocity, control_angular_velocity)
            pub_vel.publish(twist)
            accessory_motor_cmd = create_poscmd(control_access_pos[0],control_access_pos[1],control_access_pos[2])
            pub_accessory.publish(accessory_motor_cmd)
            if key_handled:
                key_handled = False
                time.sleep(SLEEP_TIME)
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub_vel.publish(twist)
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
