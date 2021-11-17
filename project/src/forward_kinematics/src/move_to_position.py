#!/usr/bin/env python


import argparse
import time
import math

import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION


def convert_image_to_robot_coords(px, py, x_ofs=750, y_ofs=0, z_ofs=100):
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
    flat_dist = math.sqrt(

    j1 = -math.atan2(192.50 - 168.50 + 136.30, 81 + 140 + 260 + 126.50 + 273.50)
    j1 += math.atan2(ry, rx)
    j2 = something
    j3 = 0.0
    j4 = something
    j5 = 0.0
    j6 = something
    j7 = math.pi / 2


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
