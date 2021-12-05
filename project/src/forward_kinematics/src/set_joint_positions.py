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

import tf
import rospy
import tf2_ros
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

def angles_to_dict(rj, angles):
    return {rj[i]: angles[i] for i in range(len(rj))}

top_left_paper_tag = "ar_marker_3"
# You will need to update the name and id of the ar tag if you change
# which one you are using.

#bottom_right_paper_tag = "ar_marker_4"
# For the future, when we use the distance between the ar markers to
# get the size of the paper from the ar tags themselves.

camera_frame = "reference/right_hand_camera"
# This should not change

def set_joints():
    right = intera_interface.Limb('right')
    right.set_joint_position_speed(0.1)

    rj = right.joint_names()
    # image_coords = [((i % 10) * -20, (i // 10) * 20) for i in range(100)]
    # random.shuffle(image_coords)

    tfBuffer = tf2_ros.Buffer()
    # The tf package creates and keeps track of recent transformations.
    # This is the buffer where they are stored.

    tfListener = tf2_ros.TransformListener(tfBuffer)
    # The tfListener variable is not used, but we need this line to create
    # and keep the TransformListener
    
    top_left_to_camera = None
    # tfListener does not always work the first 20-30 times we try to do
    # a tfBuffer lookup, so we need to repetitively try to get the
    # top_left_to_camera transformation.
    
    right.move_to_joint_positions(angles_to_dict(rj, [0, 0, 0, 0, 0, 0, 0]))
    # Puts the arm in the zero-angle configuration so that the camera is
    # pointing a the ar tag near the paper
    
    time.sleep(1)
    # Wait for the robot to get to the zero-angle configuration

    # COMMENTED CAMERA TO TAG TRANSFORMATION FOR NOW 
    # while top_left_to_camera is None:
    # # Since it takes 20-30 times to get sucessfully tfBuffer lookup, this
    # # loop is necessary
    #     try:
    #         top_left_to_camera = tfBuffer.lookup_transform(top_left_paper_tag,camera_frame, rospy.Time())
    #     except :
    #         pass
    # print(top_left_to_camera)

    # t = top_left_to_camera
    # TODO: Calculate the transformation from the robot base to the ar tag
    # using the transformation from the end effector camera to the ar tag
    # and the transformation from the base to the end effector given the
    # joint angles
    
    # t_x = t.transform.translation.x * 100 + 650
    # t_y = t.transform.translation.y * 100
    # t_z = 150 #t.transform.translation.z * 100
    
    # print(t_x, t_y, t_z)
    # print("PRINTING ROTATION")
    # print(t)
    print("PRINTING TRANSFORM FROM BASE TO AR TAG")
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    base_to_ar_tag = None
    while base_to_ar_tag is None:
    # Since it takes 20-30 times to get sucessfully tfBuffer lookup, this
    # loop is necessary
        try:
            base_to_ar_tag = tfBuffer.lookup_transform("base",top_left_paper_tag, rospy.Time())
        except :
            pass
    print(base_to_ar_tag)

    print("PRINTING TRANSFORM FROM BASE TO CAMERA")
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    base_to_camera = None
    while base_to_camera is None:
    # Since it takes 20-30 times to get sucessfully tfBuffer lookup, this
    # loop is necessary
        try:
            base_to_camera = tfBuffer.lookup_transform("base",camera_frame, rospy.Time())
        except :
            pass
    print(base_to_camera)

    # target goal is going to have the same transformation as the camera but the x, y would be the same as the base to AR tag x, y
    target_goal = base_to_ar_tag
    target_goal.transform.translation.z = base_to_camera.transform.translation.z

    # t = base_to_ar_tag
    # t_x = t.transform.translation.x * 100 + 650
    # t_y = t.transform.translation.y * 100
    # t_z = 150
    
    # print(t_x, t_y, t_z)

    raw_input("enter to confirm coords")
    # To make sure that the desired robot configuration is not  unreasonable
    # before moving there.  Just press enter.
    
    # FORWARD KINEMATICS
    # right.move_to_joint_positions(angles_to_dict(rj, convert_robot_coords_to_joint_angles(t_x, t_y, t_z)))

    # ATTEMPT TRYING INVERSE KINEMATICS
    rospy.wait_for_service('compute_ik')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    arm = 'right'
    move_gripper(compute_ik, arm, target_goal.transform.translation, target_goal.transform.rotation)
    return
    # I was having problems with the code below, so I exit prematurely
    
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
def move_gripper(compute_ik, arm, position, orientation):
    request = GetPositionIKRequest()
    request.ik_request.group_name = arm + "_arm"

    link = arm + "_gripper"

    request.ik_request.ik_link_name = link
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    
    # Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = position.x
    request.ik_request.pose_stamped.pose.position.y = position.y
    request.ik_request.pose_stamped.pose.position.z = position.z        
    request.ik_request.pose_stamped.pose.orientation.x = orientation.x
    request.ik_request.pose_stamped.pose.orientation.y = orientation.y
    request.ik_request.pose_stamped.pose.orientation.z = orientation.z
    request.ik_request.pose_stamped.pose.orientation.w = orientation.w
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
    try:
        # Send the request to the service
        response = compute_ik(request)
        
        # Print the response HERE
        print(response)
        group = MoveGroupCommander(arm + "_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # TRY THIS
        # Setting just the position without specifying the orientation
        ###group.set_position_target(position)

        # Plan IK and execute
        group.go()
            
    except rospy.ServiceException, e:
        print("Service call failed: %s")

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
