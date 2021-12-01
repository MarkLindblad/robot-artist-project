#!/usr/bin/env python

from __future__ import division, print_function

from controller import Controller

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

def draw_points(points):
    controller = Controller()
    for i in range(len(points)):
        ix, iy = points[i]
        print(i, ix, iy)
        controller.draw_image_point(ix, iy)

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

    draw_points([(100, 100), (100, -100), (-100, -100), (-100, 100), (100, 100)])
    print("Done.")


if __name__ == '__main__':
    main()
