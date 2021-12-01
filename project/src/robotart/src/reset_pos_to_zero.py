#!/usr/bin/env python

from __future__ import print_function

import rospy

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

    controller = Controller()
    controller.set_joint_angles([0.0] * 7)
    print("Done.")

if __name__ == '__main__':
    main()

