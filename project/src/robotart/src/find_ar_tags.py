#!/usr/bin/env python

from __future__ import division, print_function

from controller import Controller

import rospy
import tf2_ros

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

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

    ar_tags = ["ar_marker_3", "ar_marker_4", "ar_marker_6", "ar_marker_7"]
    ar_tags_translations = {}
    for ar_tag in ar_tags:
        ar_tags_translations[ar_tag] = {'x': [], 'y': [], 'z': []}

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    controller = Controller()
    for x in range(500, 750, 50):
        for y in range(-100, 150, 50):
            controller.move_to_robot_coords(x, y, 120, True)
            rospy.sleep(1)
            for ar_tag in ar_tags:
                try:
                    ar_tag_transform = tf_buffer.lookup_transform("base", ar_tag, rospy.Time())
                    ar_tags_translations[ar_tag]['x'].append(ar_tag_transform.transform.translation.x)
                    ar_tags_translations[ar_tag]['y'].append(ar_tag_transform.transform.translation.y)
                    ar_tags_translations[ar_tag]['z'].append(ar_tag_transform.transform.translation.z)
                except:
                    pass

    for ar_tag in ar_tags:
        if len(ar_tags_translations[ar_tag]['x']) == 0:
            print(ar_tag, "not found")
            return
    ar_tag_positions = {}
    for ar_tag in ar_tags:
        ar_tag_positions[ar_tag] = (1000 * sum(ar_tags_translations[ar_tag]['x']) / len(ar_tags_translations[ar_tag]['x']), 1000 * sum(ar_tags_translations[ar_tag]['y']) / len(ar_tags_translations[ar_tag]['y']))
        print(ar_tag, ":", ar_tag_positions[ar_tag])
        controller.move_to_robot_coords(ar_tag_positions[ar_tag][0], ar_tag_positions[ar_tag][1], 50, False)
        rospy.sleep(1)

if __name__ == '__main__':
    main()
