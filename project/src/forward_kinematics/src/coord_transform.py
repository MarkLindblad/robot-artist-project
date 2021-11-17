#!/usr/bin/env python


import math

def convert_image_to_robot_coords(px, py, x_ofs=600, y_ofs=0, z_ofs=200):
    """
    Input: px, py -- the number of millimeters from the center of the paper
    Output: rx, ry, rz -- the coordinates of the joint above the end effector
    in order to draw on the paper (in millimeters from the base of the robot)
    rx is front/back direction from robot base (front is positive, back is negative)
    ry is right/left direction
    rz is up/down direction
    """
    # x_ofs is distance from robot base to center of paper in rx direction
    # y_ofs is distance from robot base to center of paper in ry direction
    # z_ofs is distance from desired joint position above end effector to robot base in rz direction
    return py + x_ofs, -px + y_ofs, z_ofs

def convert_robot_coords_to_joint_angles(rx, ry, rz):
    """
    Input: rx, ry, rz -- the coordinates of the joint above the end effector
    in order to draw on the paper (in millimeters from the base of the robot)
    Output: 7 joint angles
    """
    flat_dist = math.sqrt(rx ** 2 + ry ** 2) - 81
    dist = math.sqrt(flat_dist ** 2 + (rz - 80 - 237) ** 2)
    tilt_down_angle = math.acos(flat_dist / dist)
    compress_angle = math.acos(dist / 2 / 400)
    j1 = -math.atan2(192.50 - 168.50 + 136.30, 81 + 140 + 260 + 126.50 + 273.50)
    j1 += math.atan2(ry, rx)
    j2 = tilt_down_angle - compress_angle
    j3 = 0.0
    j4 = 2 * compress_angle
    j5 = 0.0
    j6 = math.pi / 2 - tilt_down_angle - compress_angle
    j7 = math.pi / 2
    return j1, j2, j3, j4, j5, j6, j7

