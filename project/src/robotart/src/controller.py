#!/usr/bin/env python

from __future__ import division, print_function

import math

import intera_interface

import rospy

class Controller:

    def __init__(self):
        self.limb = intera_interface.Limb('right')
        self.joint_names = self.limb.joint_names()

    def joint_angles_to_dict(self, angles):
        return {self.joint_names[i]: angles[i] for i in range(len(self.joint_names))}

    def set_joint_angles(self, angles):
        angles_dict = self.joint_angles_to_dict(angles)
        # self.limb.move_to_joint_positions(angles_dict, timeout=5.0, threshold=0.005)
        prev_angles = self.limb.joint_angles()
        def test():
            cur_angles = self.limb.joint_angles()
            if all(abs(cur_angles[j] - angles_dict[j]) < 0.005 for j in cur_angles) \
                and all(cur_angles[j] == prev_angles[j] for j in cur_angles):
                return True
            prev_angles = cur_angles
            return False
        self.limb.move_to_joint_positions(angles_dict, timeout=5.0, threshold=0.0, test=test)

    def move_to_robot_coords(self, rx, ry, rz):
        self.set_joint_angles(self.convert_robot_coords_to_joint_angles(rx, ry, rz))

    def draw_image_point(self, ix, iy):
        rx, ry, rz = self.convert_image_to_robot_coords(ix, iy)
        self.move_to_robot_coords(rx, ry, rz - 50)
        rospy.sleep(0.3)
        self.move_to_robot_coords(rx, ry, rz - 70)
        rospy.sleep(0.3)
        self.move_to_robot_coords(rx, ry, rz - 50)

    def draw_image_line(self, ixs, iys, ixe, iye):
        self.limb.set_joint_position_speed(0.3)
        rxs, rys, rz = self.convert_image_to_robot_coords(ixs, iys)
        rxe, rye, rz = self.convert_image_to_robot_coords(ixe, iye)
        dist = math.sqrt((rxs - rxe) ** 2 + (rys - rye) ** 2)
        steps = int(dist // 1) + 1
        self.move_to_robot_coords(rxs, rys, rz - 5)
        self.limb.set_joint_position_speed(0.1)
        rospy.sleep(0.3)
        for i in range(steps + 1):
            rx = rxs + (rxe - rxs) * (i / steps)
            ry = rys + (rye - rys) * (i / steps)
            self.move_to_robot_coords(rx, ry, rz - 30)
            if i == 0:
                self.limb.set_joint_position_speed(0.3)
        self.limb.set_joint_position_speed(0.1)
        rospy.sleep(0.3)
        self.move_to_robot_coords(rxe, rye, rz - 5)

    @staticmethod
    def convert_image_to_robot_coords(px, py, x_ofs=600, y_ofs=0, z_ofs=150):
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

    @staticmethod
    def convert_robot_coords_to_joint_angles(rx, ry, rz):
        """
        Input: rx, ry, rz -- the coordinates of the joint above the end effector
        in order to draw on the paper (in millimeters from the base of the robot)
        Output: 7 joint angles
        """
        z_offset = 80 + 237 - rz
        flat_radius = math.sqrt(rx ** 2 + ry ** 2)
        flat_radius_y = 192.5 - 168.5 + 136.3
        flat_radius_x = math.sqrt(flat_radius ** 2 - flat_radius_y ** 2)
        tilt_down_angle = math.atan2(z_offset, flat_radius_x - 81)
        compressed_len = math.sqrt(z_offset ** 2 + (flat_radius_x - 81) ** 2)
        compress_angle = math.acos(compressed_len / 2 / 400)

        j1 = math.atan2(ry, rx)
        j1 -= math.atan2(flat_radius_y, flat_radius_x)
        j2 = tilt_down_angle - compress_angle
        j3 = 0.0
        j4 = 2 * compress_angle
        j5 = 0.0
        j6 = math.pi / 2 - tilt_down_angle - compress_angle
        j7 = math.pi / 2
        return j1, j2, j3, j4, j5, j6, j7

