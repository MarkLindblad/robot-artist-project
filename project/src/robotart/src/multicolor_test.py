#!/usr/bin/env python

from __future__ import print_function, division

import rospy

import math

import intera_interface
from intera_interface import CHECK_VERSION

from controller import Controller

def main():
    print("Initializing node... ")
    rospy.init_node("reset_pos_to_zero_node")
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

    controller = Controller(x_image_offset=550, y_image_offset=-200)
    controller.limb.set_joint_position_speed(0.1)

    for i in range(4):
        # draw
        controller.set_joint_angles([0.0] * 7)
        # controller.draw_image_point(0, 0)
        controller.draw_image_line(-50, -50, -50, 50)
        controller.draw_image_line(-50, 50, 50, 50)
        controller.draw_image_line(50, 50, 50, -50)
        controller.draw_image_line(50, -50, -50, -50)
        # rotate
        controller.threshold = 0.03
        controller.set_joint_angles([0.0] * 7)
        controller.set_joint_angles([0.0, 0.0, 0.0, 0.0, -3.14159 / 2, 3.14159 / 2, -0.2 + 3.14159 / 4])
        controller.set_joint_angles([0.0, 0.455, 0.0, 0.0, -3.14159 / 2, 3.14159 / 2, -0.2 + 3.14159 / 4])
        controller.timeout = 5.0
        controller.set_joint_angles([0.0, 0.455, 0.0, 0.0, -3.14159 / 2, 3.14159 / 2, -0.2 - 3.14159 / 4])
        controller.timeout = 15.0
        controller.set_joint_angles([0.0, 0.0, 0.0, 0.0, -3.14159 / 2, 3.14159 / 2, -0.2 - 3.14159 / 4])
        controller.set_joint_angles([0.0, 0.0, 0.0, 0.0, -3.14159 / 2, 3.14159 / 2, -0.2 + 3.14159 / 4])
        controller.threshold = 0.01
    # draw
    controller.set_joint_angles([0.0] * 7)
    controller.draw_image_line(-50, -50, -50, 50)
    controller.draw_image_line(-50, 50, 50, 50)
    controller.draw_image_line(50, 50, 50, -50)
    controller.draw_image_line(50, -50, -50, -50)
    controller.set_joint_angles([0.0] * 7)

    # print(controller.limb.joint_ordered_angles())
    # print(controller.limb.joint_angles())
    
    return

    """
    args = rospy.myargv()[1:]
    if len(args) != 7:
        print("Invalid args")
        return
    args = [float(x) for x in args]
    print(args)
    return
    """

    # controller.set_joint_angles([0.0] * 5 + [math.pi / 2, math.pi / 2])
    # rospy.sleep(1)
    # controller.set_joint_angles([0.5, 0.0, 0.0, 0.0, 0.0, math.pi / 2, math.pi / 2])
    # rospy.sleep(1)
    # controller.set_joint_angles([0.0] * 5 + [math.pi / 2, math.pi / 2])
    # rospy.sleep(1)
    # controller.set_joint_angles([0.0, 0.20, 0.0, 0.0, 0.0, math.pi / 2 - 0.20, math.pi / 2])
    # rospy.sleep(1)
    # controller.set_joint_angles([0.0] * 5 + [math.pi / 2, math.pi / 2])
    # rospy.sleep(1)
    # controller.set_joint_angles([0.0, -0.75, 0.0, 1.5, 0.0, math.pi / 2 - 0.75, math.pi / 2])
    # rospy.sleep(1)
    # controller.set_joint_angles([0.0] * 5 + [math.pi / 2, math.pi / 2])
    print("Done.")

if __name__ == '__main__':
    main()

