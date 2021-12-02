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

import numpy as np
import cv2
import matplotlib.pyplot as plt

from coord_transform import convert_image_to_robot_coords, convert_robot_coords_to_joint_angles
import dots

import rospy
import tf2_ros

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

def angles_to_dict(rj, angles):
    return {rj[i]: angles[i] for i in range(len(rj))}

top_left_paper_tag = "ar_marker_3"
#bottom_right_paper_tag = "ar_marker_4"
camera_frame = "reference/right_hand_camera"

def set_joints():
    right = intera_interface.Limb('right')
    right.set_joint_position_speed(0.1)

    rj = right.joint_names()
    # image_coords = [((i % 10) * -20, (i // 10) * 20) for i in range(100)]
    # random.shuffle(image_coords)

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    top_left_to_camera = None
    right.move_to_joint_positions(angles_to_dict(rj, [0, 0, 0, 0, 0, 0, 0]))
    time.sleep(1)
    while top_left_to_camera is None:
        try:
            top_left_to_camera = tfBuffer.lookup_transform(top_left_paper_tag,camera_frame, rospy.Time())
        except :
            pass
    print(top_left_to_camera)

    t = top_left_to_camera
    t_x = t.transform.translation.x * 100 + 650
    t_y = t.transform.translation.y * 100
    t_z = 150 #t.transform.translation.z * 100
    
    print(t_x, t_y, t_z)
    #print(convert_robot_coords_to_joint_angles(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z))
    raw_input("enter to confirm coords")
    
    right.move_to_joint_positions(angles_to_dict(rj, convert_robot_coords_to_joint_angles(t_x, t_y, t_z)))
    return
    
    filename = 'sphere.jpg'
    img = np.array(cv2.imread(filename, 0))
    img = dots.contrast(img, 50, 200)

    dots_x, dots_y, size = dots.grey_dither(img, angle = 30, size = 50)
    # plt.plot(dots_x, dots_y, 'ko')
    # plt.show()

    """
    filename = 'sonof.png'
    img = np.array(cv2.imread(filename))
    R = 255 -img[:, :, 2]
    G = 255 -img[:, :, 1]
    B = 255 -img[:, :, 0]

    r_x, r_y, size = dots.grey_dither(R, angle = 30, size = 100)
    g_x, g_y, size = dots.grey_dither(G, angle = 45, size = 100)
    b_x, b_y, size = dots.grey_dither(B, angle = 60, size = 100)
    plt.plot(g_x, g_y, 'g.', alpha = .5)
    plt.plot(b_x, b_y, 'b.', alpha = .5)
    plt.plot(r_x, r_y, 'r.', alpha = .5)
    plt.show()
    return
    """
    
    image_coords = [(2 * dots_x[i], 2 * dots_y[i]) for i in range(len(dots_x))]
    # image_coords = [(-50, -50), (-50, 50), (50, 50), (50, -50), (-50, 50), (50, 50), (-50, -50)]
    # image_coords = image_coords[:1]
    
    #top_left_tobottom_right = tfBuffer.lookup_transform(top_left_paper_tag, bottom_right_paper_tag, rospy.Time())
    
    for i in range(len(image_coords)):
        ic = image_coords[i]
        print(i, ic)
        rc = convert_image_to_robot_coords(*ic)
        right.move_to_joint_positions(angles_to_dict(rj, convert_robot_coords_to_joint_angles(rc[0], rc[1], rc[2] - 30)))
        time.sleep(0.2)
        right.move_to_joint_positions(angles_to_dict(rj, convert_robot_coords_to_joint_angles(rc[0], rc[1], rc[2] - 50)), threshold=0.005)
        time.sleep(0.3)
        right.move_to_joint_positions(angles_to_dict(rj, convert_robot_coords_to_joint_angles(rc[0], rc[1], rc[2] - 30)))
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
