#!/usr/bin/env python

from __future__ import division, print_function

from controller import Controller
from find_ar_tags import find_ar_tags

import cv2
import numpy as np
from lines import crosshatch
from lines import preview
from lines import cmyk_crosshatch
from lines import cmyk_preview
import rospy

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

def draw_lines(controller, width, height, lines):
    # controller = Controller()
    ll = len(lines)
    for i in range(len(lines)):
        print(i, "/", ll, lines[i])
        if lines[i][0] < -0.001 or lines[i][0] > 1.001 or lines[i][1] < -0.001 or lines[i][1] > 1.001 or lines[i][2] < -0.001 or lines[i][2] > 1.001 or lines[i][3] < -0.001 or lines[i][3] > 1.001 or np.isnan(lines[i][0]) or np.isnan(lines[i][1]) or np.isnan(lines[i][2]) or np.isnan(lines[i][3]):
            print("Bad line")
            continue
        controller.draw_image_line(lines[i][0] * width - width / 2, lines[i][2] * height - height / 2, lines[i][1] * width - width / 2, lines[i][3] * height - height / 2)

def main():
    lines = None

    # img = np.array(cv2.imread('snoof.jpg', 0))
    # lines = crosshatch(img, blacks = 0, whites = 0.6, layers = 5, number = 60)
    # preview(img, lines)
    
    # img = np.array(cv2.imread('sphere.jpg', 0))
    # lines = crosshatch(img, blacks = 0, whites = 0.9, layers = 20, number = 50)

    # img = np.array(cv2.imread('team.png', 0))
    # lines = crosshatch(img, blacks = 0, whites = 0.9, layers = 4, number = 50)

    # img = np.array(cv2.imread('48.png', 0))
    # lines = crosshatch(img, blacks = 0, whites = 0.9, layers = 7, number = 50)
    
    # img = np.array(cv2.imread('charlie-2.jpg', 0))
    # lines = crosshatch(img, blacks = 0, whites = 0.9, layers = 5, number = 25)
    # print(len(lines))
    # preview(img, lines)

    # img = np.array(cv2.GaussianBlur(cv2.imread('raven_huang.jpg', 0), (3, 3), cv2.BORDER_DEFAULT))
    # lines = crosshatch(img, blacks = 0, whites = 0.9, layers = 15, number = 100)

    # img = np.array(cv2.GaussianBlur(cv2.imread('ken.jpg', 0), (5, 5), cv2.BORDER_DEFAULT))
    # lines = crosshatch(img, blacks = 0, whites = 0.9, layers = 15, number = 60)

    # img = np.array(cv2.GaussianBlur(cv2.imread('1.jpg', 0), (19, 19), cv2.BORDER_DEFAULT))
    # lines = crosshatch(img, blacks = 0, whites = 0.75, layers = 8, number = 100)
    # print(len(lines))
    
    # img = np.array(cv2.imread('starry.png'))
    # clines, mlines, ylines, klines = cmyk_crosshatch(img, blacks = 0, whites = 0.9, layers = 5, number = 30)
    # c_clines = len(clines)
    # c_mlines = len(mlines)
    # c_ylines = len(ylines)
    # c_klines = len(klines)
    # print(c_clines + c_mlines + c_ylines + c_klines, c_clines, c_mlines, c_ylines, c_klines)
    # cmyk_preview(img, clines, mlines, ylines, klines)
    
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

    ### Height Calibration
    controller = Controller(x_image_offset=600, y_image_offset=0)
    controller.draw_image_point(0, 0)
    return

    
    # ar_tag_pos = find_ar_tags(["ar_marker_3", "ar_marker_4", "ar_marker_6", "ar_marker_7"])
    # if ar_tag_pos is None:
    #     return
    # x_pos = [ar_tag_pos[ar_tag][0] for ar_tag in ar_tag_pos]
    # y_pos = [ar_tag_pos[ar_tag][1] for ar_tag in ar_tag_pos]
    # x_pos.sort()
    # y_pos.sort()
    # x_center = sum(x_pos) / len(x_pos)
    # y_center = sum(y_pos) / len(y_pos)
    # height = x_pos[2] - x_pos[1] - 45
    # width = y_pos[2] - y_pos[1] - 45
    # print(x_center, y_center, width, height)

    # width *= 0.9
    # height *= 0.9
    # width = min(width, height)
    # height = min(width, height)

    # controller = Controller(x_image_offset=x_center, y_image_offset=y_center)
    controller = Controller(x_image_offset=750, y_image_offset=150)
    draw_lines(controller, 200, 200, lines)
    return

    #img = np.array(cv2.imread('raven_huang.jpg', 0))
    #lines = crosshatch(img, blacks = 0, whites = 0.9, layers = 8, number = 60)
    
    #preview(img, lines)

    # img = np.array(cv2.imread('raven_huang.jpg', 0))
    # lines = crosshatch(img, blacks = 0, whites = 0.9, layers = 7, number = 50)
    # preview(img, lines)
    # img = np.array(cv2.imread('sphere.jpg', 0))
    print("Starting drawing...")
    controller = Controller()
    draw_lines(controller, 180, 180, lines)
    return

    if lines is not None:
        cont = ""
        adjust_up = 0
        adjust_down = -10
        controller = Controller(adjust_up, adjust_down, x_image_offset=x_center, y_image_offset=y_center)
        controller.draw_image_point(50, 100)
        print(adjust_up, adjust_down)
        while cont != "y":
            adjust_up = int(raw_input("Enter up displacement: "))
            adjust_down = int(raw_input("Enter down displacement: "))
            controller = Controller(adjust_up, adjust_down, x_image_offset=x_center, y_image_offset=y_center)
            controller.draw_image_point(50, 100)
            print("(", adjust_up, adjust_down, ")")
            cont = raw_input("Enter y to exit.  Enter any other input to keep calibrating")
        draw_lines(controller, width, height, lines)
    else:
        """
        print(c_clines + c_mlines + c_ylines + c_klines, c_clines, c_mlines, c_ylines, c_klines)
        raw_input("CYAN       Put the cyan pen in the gripper and press enter to begin.")
        cont = ""
        adjust_up = 0
        adjust_down = -10
        controller = Controller(adjust_up, adjust_down, x_image_offset=x_center, y_image_offset=y_center)
        controller.draw_image_point(50, 100)
        print(adjust_up, adjust_down)
        while cont != "y":
            adjust_up = int(raw_input("Enter up displacement: "))
            adjust_down = int(raw_input("Enter down displacement: "))
            controller = Controller(adjust_up, adjust_down, x_image_offset=x_center, y_image_offset=y_center)
            controller.draw_image_point(50, 100)
            print("(", adjust_up, adjust_down, ")")
            cont = raw_input("Enter y to exit.  Enter any other input to keep calibrating")
        draw_lines(controller, width, height, clines)

        print(c_clines + c_mlines + c_ylines + c_klines, c_clines, c_mlines, c_ylines, c_klines)
        raw_input("MAGENTA    Put the magenta pen in the gripper and press enter to begin.")
        cont = ""
        adjust_up = 0
        adjust_down = -10
        controller = Controller(adjust_up, adjust_down, x_image_offset=x_center, y_image_offset=y_center)
        controller.draw_image_point(50, 100)
        print(adjust_up, adjust_down)
        while cont != "y":
            adjust_up = int(raw_input("Enter up displacement: "))
            adjust_down = int(raw_input("Enter down displacement: "))
            controller = Controller(adjust_up, adjust_down, x_image_offset=x_center, y_image_offset=y_center)
            controller.draw_image_point(50, 100)
            print("(", adjust_up, adjust_down, ")")
            cont = raw_input("Enter y to exit.  Enter any other input to keep calibrating")
        draw_lines(controller, width, height, mlines)

        print(c_clines + c_mlines + c_ylines + c_klines, c_clines, c_mlines, c_ylines, c_klines)
        raw_input("YELLOW     Put the yellow pen in the gripper and press enter to begin.")
        cont = ""
        adjust_up = 0
        adjust_down = -10
        controller = Controller(adjust_up, adjust_down, x_image_offset=x_center, y_image_offset=y_center)
        controller.draw_image_point(50, 100)
        print(adjust_up, adjust_down)
        while cont != "y":
            adjust_up = int(raw_input("Enter up displacement: "))
            adjust_down = int(raw_input("Enter down displacement: "))
            controller = Controller(adjust_up, adjust_down, x_image_offset=x_center, y_image_offset=y_center)
            controller.draw_image_point(50, 100)
            print("(", adjust_up, adjust_down, ")")
            cont = raw_input("Enter y to exit.  Enter any other input to keep calibrating")
        draw_lines(controller, width, height, ylines)
        """

        print(c_clines + c_mlines + c_ylines + c_klines, c_clines, c_mlines, c_ylines, c_klines)
        raw_input("BLACK      Put the black pen in the gripper and press enter to begin.")
        cont = ""
        adjust_up = 0
        adjust_down = -10
        controller = Controller(adjust_up, adjust_down, x_image_offset=x_center, y_image_offset=y_center)
        controller.draw_image_point(50, 100)
        print(adjust_up, adjust_down)
        while cont != "y":
            adjust_up = int(raw_input("Enter up displacement: "))
            adjust_down = int(raw_input("Enter down displacement: "))
            controller = Controller(adjust_up, adjust_down, x_image_offset=x_center, y_image_offset=y_center)
            controller.draw_image_point(50, 100)
            print("(", adjust_up, adjust_down, ")")
            cont = raw_input("Enter y to exit.  Enter any other input to keep calibrating")
        controller = Controller(adjust_up, adjust_down, x_image_offset=x_center, y_image_offset=y_center)
        draw_lines(controller, width, height, klines)

    return
    print("Done.")


if __name__ == '__main__':
    main()
