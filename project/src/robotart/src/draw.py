#!/usr/bin/env python

from __future__ import division, print_function

from controller import Controller

import cv2
import numpy as np
from lines import crosshatch

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

def draw_lines(lines):
    controller = Controller()
    for i in range(len(lines)):
        print(i, lines[i])
        if lines[i][0] < 0.0 or lines[i][0] > 1.0 or lines[i][1] < 0.0 or lines[i][1] > 1.0 or lines[i][2] < 0.0 or lines[i][2] > 1.0 or lines[i][3] < 0.0 or lines[i][3] > 1.0:
            print("Bad line")
            continue
        controller.draw_image_line(lines[i][0] * 100 - 50, lines[i][2] * 100 - 50, lines[i][1] * 100 - 50, lines[i][3] * 100 - 50)

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
    rs.enable()

    # controller = Controller()
    # controller.limb.move_to_neutral()
    # controller.move_to_robot_coords(600, -100, 120, True)
    # controller.move_to_robot_coords(768, -64, 120, False)
    # controller.draw_image_line(-50, -50, 50, 50)
    # controller.draw_image_line(50, -50, -50, 50)
    # controller.draw_image_point(-50, -50)
    # controller.draw_image_point(-50, 50)
    # controller.draw_image_point(50, 50)
    # controller.draw_image_point(50, -50)

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
    img = np.array(cv2.imread('raven_huang.jpg', 0))
    draw_lines(crosshatch(img, blacks = 0, whites = 0.9, layers = 7, number = 50))
    print("Done.")


if __name__ == '__main__':
    main()
