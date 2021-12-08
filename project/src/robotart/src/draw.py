#!/usr/bin/env python

from __future__ import division, print_function

from controller import Controller
from find_ar_tags import find_ar_tags

import cv2
import numpy as np
from lines import crosshatch

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

def draw_lines(controller, width, height, lines):
    # controller = Controller()
    for i in range(len(lines)):
        print(i, lines[i])
        controller.draw_image_line(lines[i][0] * width - width / 2, lines[i][2] * height - height / 2, lines[i][1] * width - width / 2, lines[i][3] * height - height / 2)

def main():
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

    """
    rs.enable()
    controller = Controller()
    controller.draw_image_point(50, 150)
    return
    """

    """
    ar_tag_pos = find_ar_tags(["ar_marker_3", "ar_marker_4", "ar_marker_6", "ar_marker_7"])
    if ar_tag_pos is None:
        return
    x_pos = [ar_tag_pos[ar_tag][0] for ar_tag in ar_tag_pos]
    y_pos = [ar_tag_pos[ar_tag][1] for ar_tag in ar_tag_pos]
    x_pos.sort()
    y_pos.sort()
    x_center = sum(x_pos) / len(x_pos)
    y_center = sum(y_pos) / len(y_pos)
    height = x_pos[2] - x_pos[1] - 45
    width = y_pos[2] - y_pos[1] - 45
    print(x_center, y_center, width, height)
    """

    x_center, y_center, width, height = 707.07331582, -109.566561181, 110.33267366, 187.66238968

    controller = Controller(x_image_offset=x_center, y_image_offset=y_center)
    """
    # controller.limb.move_to_neutral()
    # controller.move_to_robot_coords(600, -100, 170, True)
    # controller.move_to_robot_coords(768, -64, 120, False)
    width *= 0.8
    height *= 0.8
    width = min(width, height)
    height = min(width, height)

    def rotate_point(x, y, theta):
        new_x = np.cos(theta) * x - np.sin(theta) * y
        new_y = np.sin(theta) * x + np.cos(theta) * y
        return new_x, new_y

    grid_size = 4
    for i in range(grid_size + 1):
        offset = width * i / grid_size
        sx, sy = rotate_point(-width / 2 + offset, -height / 2, np.pi * 0)
        ex, ey = rotate_point(-width / 2 + offset, height / 2, np.pi * 0)
        controller.draw_image_line(sx, sy, ex, ey)
    for i in range(grid_size + 1):
        offset = height * i / grid_size
        sx, sy = rotate_point(-width / 2, -height / 2 + offset, np.pi * 0)
        ex, ey = rotate_point(width / 2, -height / 2 + offset, np.pi * 0)
        controller.draw_image_line(sx, sy, ex, ey)

    # controller.draw_image_line(-width / 2, -height / 2,  width / 2,  height / 2)
    # controller.draw_image_line(-width / 2,  height / 2,  width / 2, -height / 2)
    # controller.draw_image_line(-width / 2, -height / 2,  width / 2, -height / 2)
    # controller.draw_image_line( width / 2, -height / 2,  width / 2,  height / 2)
    # controller.draw_image_line( width / 2,  height / 2, -width / 2,  height / 2)
    # controller.draw_image_line(-width / 2,  height / 2, -width / 2, -height / 2)

    # controller.draw_image_line(50, -50, -50, 50)
    # controller.draw_image_point(-50, -50)
    # controller.draw_image_point(-50, 50)
    # controller.draw_image_point(50, 50)
    # controller.draw_image_point(50, -50)
    return
    """

    """
    lines = []
    with open('ball2_lines.txt') as f:
        for line in f:
            line = line.strip()
            line = line[1:-1]
            line = line.split()
            line = [float(x) for x in line]
            lines.append(line)
    """
    img = np.zeros((200, 200))
    
    draw_lines(controller, width, height, crosshatch(img, blacks = 0, whites = 0.9, layers = 7, number = 7))
    print("Done.")


if __name__ == '__main__':
    main()
