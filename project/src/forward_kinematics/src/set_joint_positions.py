#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Example: keyboard
"""
from __future__ import division, print_function
import argparse
import time
import random
from coord_transform import convert_image_to_robot_coords, convert_robot_coords_to_joint_angles

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION


def map_keyboard():
    right = intera_interface.Limb('right')
    grip_right = intera_interface.Gripper('right', CHECK_VERSION)
    rj = right.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)

    bindings = {
    #   key: (function, args, description)
        '9': (set_j, [left, lj[0], 0.1], "left_s0 increase"),
        '6': (set_j, [left, lj[0], -0.1], "left_s0 decrease"),
        '8': (set_j, [left, lj[1], 0.1], "left_s1 increase"),
        '7': (set_j, [left, lj[1], -0.1], "left_s1 decrease"),
        'o': (set_j, [left, lj[2], 0.1], "left_e0 increase"),
        'y': (set_j, [left, lj[2], -0.1], "left_e0 decrease"),
        'i': (set_j, [left, lj[3], 0.1], "left_e1 increase"),
        'u': (set_j, [left, lj[3], -0.1], "left_e1 decrease"),
        'l': (set_j, [left, lj[4], 0.1], "left_w0 increase"),
        'h': (set_j, [left, lj[4], -0.1], "left_w0 decrease"),
        'k': (set_j, [left, lj[5], 0.1], "left_w1 increase"),
        'j': (set_j, [left, lj[5], -0.1], "left_w1 decrease"),
        '.': (set_j, [left, lj[6], 0.1], "left_w2 increase"),
        'n': (set_j, [left, lj[6], -0.1], "left_w2 decrease"),
        ',': (grip_left.close, [], "left: gripper close"),
        'm': (grip_left.open, [], "left: gripper open"),
        '/': (grip_left.calibrate, [], "left: gripper calibrate"),

        '4': (set_j, [right, rj[0], 0.1], "right_s0 increase"),
        '1': (set_j, [right, rj[0], -0.1], "right_s0 decrease"),
        '3': (set_j, [right, rj[1], 0.1], "right_s1 increase"),
        '2': (set_j, [right, rj[1], -0.1], "right_s1 decrease"),
        'r': (set_j, [right, rj[2], 0.1], "right_e0 increase"),
        'q': (set_j, [right, rj[2], -0.1], "right_e0 decrease"),
        'e': (set_j, [right, rj[3], 0.1], "right_e1 increase"),
        'w': (set_j, [right, rj[3], -0.1], "right_e1 decrease"),
        'f': (set_j, [right, rj[4], 0.1], "right_w0 increase"),
        'a': (set_j, [right, rj[4], -0.1], "right_w0 decrease"),
        'd': (set_j, [right, rj[5], 0.1], "right_w1 increase"),
        's': (set_j, [right, rj[5], -0.1], "right_w1 decrease"),
        'v': (set_j, [right, rj[6], 0.1], "right_w2 increase"),
        'z': (set_j, [right, rj[6], -0.1], "right_w2 decrease"),
        'c': (grip_right.close, [], "right: gripper close"),
        'x': (grip_right.open, [], "right: gripper open"),
        'b': (grip_right.calibrate, [], "right: gripper calibrate"),
     }
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = intera_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                #expand binding to something like "set_j(right, 's0', 0.1)"
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))

def angles_to_dict(rj, angles):
    return {rj[i]: angles[i] for i in range(len(rj))}

def set_joints():
    right = intera_interface.Limb('right')
    right.set_joint_position_speed(0.1)

    rj = right.joint_names()
    image_coords = [((i % 4) * 50 - 100, (i // 4) * 50 - 100) for i in range(16)]
    random.shuffle(image_coords)
    for ic in image_coords:
        rc = convert_image_to_robot_coords(*ic)
        right.move_to_joint_positions(angles_to_dict(rj, convert_robot_coords_to_joint_angles(rc[0], rc[1], rc[2] + 50)))
        right.move_to_joint_positions(angles_to_dict(rj, convert_robot_coords_to_joint_angles(rc[0], rc[1], rc[2] - 22)))
        time.sleep(1)
        print(right.endpoint_pose()['position'])
        right.move_to_joint_positions(angles_to_dict(rj, convert_robot_coords_to_joint_angles(rc[0], rc[1], rc[2] + 50)))
    """
    rj = right.joint_names()
    while True:
        current_positions = [right.joint_angle(rj[i]) for i in range(len(rj))]
        # print("Start positions: ", current_positions)
        angles = raw_input("Enter 7 joint angles: ")
        angles = angles.strip().split(", ")
        angles = [float(angle) for angle in angles]
        while max([abs(angles[i] - current_positions[i]) for i in [0, 1, 3, 5, 6]]) > 0.005:
            # print("Current positions: ", current_positions)
            # print("Desired positions: ", angles)
            right.set_joint_positions({rj[i]: angles[i] for i in range(len(rj))})
            time.sleep(0.01)
            current_positions = [right.joint_angle(rj[i]) for i in range(len(rj))]
        # print("End positions: ", current_positions)
        print("End pose: ", right.endpoint_pose())
    """


def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on one of Baxter's arms. Each arm is represented
    by one side of the keyboard and inner/outer key pairings
    on each row for each joint.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    # arg_fmt = argparse.RawDescriptionHelpFormatter
    # parser = argparse.ArgumentParser(formatter_class=arg_fmt,
    #                                  description=main.__doc__,
    #                                  epilog=epilog)
    # args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    # map_keyboard()
    set_joints()
    print("Done.")


if __name__ == '__main__':
    main()
