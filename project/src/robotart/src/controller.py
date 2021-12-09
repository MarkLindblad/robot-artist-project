#!/usr/bin/env python

from __future__ import division, print_function

import math

import intera_interface

import rospy

class Controller:

    def __init__(self, up=-23, down=-43, x_image_offset=600, y_image_offset=0, z_image_offset=150):
        self.limb = intera_interface.Limb('right')
        self.joint_names = self.limb.joint_names()
        self.pen_up_offset = up
        self.pen_down_offset = down
        # offsets in the robot coordinates
        self.x_image_offset = x_image_offset
        self.y_image_offset = y_image_offset
        self.z_image_offset = z_image_offset

    def joint_angles_to_dict(self, angles):
        return {self.joint_names[i]: angles[i] for i in range(len(self.joint_names))}

    def set_joint_angles(self, angles):
        angles_dict = self.joint_angles_to_dict(angles)
        self.limb.move_to_joint_positions(angles_dict, timeout=15.0, threshold=0.005)
        # prev_angles = self.limb.joint_angles()
        # def test():
        #     cur_angles = self.limb.joint_angles()
        #     if all(abs(cur_angles[j] - angles_dict[j]) < 0.005 for j in cur_angles) \
        #         and all(abs(cur_angles[j] - prev_angles[j]) < 0.001 for j in cur_angles):
        #         return True
        #     for j in cur_angles:
        #         prev_angles[j] = cur_angles[j]
        #     return False
        # self.limb.move_to_joint_positions(angles_dict, timeout=15.0, threshold=0.0, test=test)

    def move_to_robot_coords(self, rx, ry, rz, camera=False):
        self.set_joint_angles(self.convert_robot_coords_to_joint_angles(rx, ry, rz, camera))

    def draw_image_point(self, ix, iy):
        rx, ry, rz = self.convert_image_to_robot_coords(ix, iy)
        self.move_to_robot_coords(rx, ry, rz + self.pen_up_offset)
        self.limb.set_joint_position_speed(0.1)
        rospy.sleep(0.3)
        self.move_to_robot_coords(rx, ry, rz + self.pen_down_offset)
        rospy.sleep(0.3)
        self.move_to_robot_coords(rx, ry, rz + self.pen_up_offset)
        self.limb.set_joint_position_speed(0.3)

    def draw_image_line(self, ixs, iys, ixe, iye):
        self.limb.set_joint_position_speed(0.3)
        rxs, rys, rz = self.convert_image_to_robot_coords(ixs, iys)
        rxe, rye, rz = self.convert_image_to_robot_coords(ixe, iye)
        dist = math.sqrt((rxs - rxe) ** 2 + (rys - rye) ** 2)
        steps = int(dist // 5) + 1
        self.move_to_robot_coords(rxs, rys, rz + self.pen_up_offset)
        self.limb.set_joint_position_speed(0.1)
        rospy.sleep(0.3)
        steps = 1 # TODO: REMOVE THIS
        for i in range(steps + 1):
            rx = rxs + (rxe - rxs) * (i / steps)
            ry = rys + (rye - rys) * (i / steps)
            self.move_to_robot_coords(rx, ry, rz + self.pen_down_offset)
            if i == 0:
                self.limb.set_joint_position_speed(0.1)
        self.limb.set_joint_position_speed(0.1)
        rospy.sleep(0.3)
        self.move_to_robot_coords(rxe, rye, rz + self.pen_up_offset)

    def convert_image_to_robot_coords(self, px, py):
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
        return py + self.x_image_offset, -px + self.y_image_offset, self.z_image_offset

    def convert_robot_coords_to_joint_angles(self, rx, ry, rz, camera=False):
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
        j6 = (0 if camera else math.pi / 2) - tilt_down_angle - compress_angle
        j7 = math.pi / 2 + 0.15 # 0.15 for calibration adjustment on Ada
        return j1, j2, j3, j4, j5, j6, j7

