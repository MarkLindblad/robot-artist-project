#!/usr/bin/env python

from __future__ import division, print_function

from controller import Controller

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

def draw_lines(lines):
    controller = Controller()
    for i in range(len(lines)):
        print(i, lines[i])
        controller.draw_image_line(lines[i][0] * 180 - 90, lines[i][2] * 180 - 90, lines[i][1] * 180 - 90, lines[i][3] * 180 - 90)

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

    """
    controller = Controller()
    controller.draw_image_line(-90, -90, 90, 90)
    controller.draw_image_line(90, -90, -90, 90)
    """
    lines = []
    with open('ball2_lines.txt') as f:
        for line in f:
            line = line.strip()
            line = line[1:-1]
            line = line.split()
            line = [float(x) for x in line]
            lines.append(line)
    draw_lines(lines)
    print("Done.")


if __name__ == '__main__':
    main()
