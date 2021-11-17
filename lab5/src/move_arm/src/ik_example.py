#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys

if sys.argv[1] == 'sawyer':
	from intera_interface import gripper as robot_gripper
else:
    from baxter_interface import gripper as robot_gripper

def move_gripper(compute_ik, arm, position, orientation):
    request = GetPositionIKRequest()
    request.ik_request.group_name = arm + "_arm"

    link = arm + "_gripper"

    request.ik_request.ik_link_name = link
    request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    
    # Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = position[0]
    request.ik_request.pose_stamped.pose.position.y = position[1]
    request.ik_request.pose_stamped.pose.position.z = position[2]        
    request.ik_request.pose_stamped.pose.orientation.x = orientation[0]
    request.ik_request.pose_stamped.pose.orientation.y = orientation[1]
    request.ik_request.pose_stamped.pose.orientation.z = orientation[2]
    request.ik_request.pose_stamped.pose.orientation.w = orientation[3]
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
    try:
        # Send the request to the service
        response = compute_ik(request)
        
        # Print the response HERE
        print(response)
        group = MoveGroupCommander(arm + "_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # TRY THIS
        # Setting just the position without specifying the orientation
        ###group.set_position_target(position)

        # Plan IK and execute
        group.go()
            
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def main(robo):
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    arm = 'left'
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    if robo == 'sawyer':
    	arm = 'right'
    left_gripper = robot_gripper.Gripper('left')
    above_translation = [0.758, 0.146, -0.053]
    above_rotation = [0.020, 1.000, -0.002, 0.011]
    pickup_translation = [0.747, 0.159, -0.161]
    pickup_rotation = [-0.003, 1.000, 0.022, -0.002]
    dropoff_translation = [0.779, 0.429, -0.108]
    dropoff_rotation = [0.005, 1.000, 0.027, -0.007]

    # callibrate grip
    left_gripper.calibrate()
    rospy.sleep(2.0)
    # move to above
    move_gripper(compute_ik, arm, above_translation, above_rotation)
    rospy.sleep(2.0)
    # move to pickup location
    move_gripper(compute_ik, arm, pickup_translation, pickup_rotation)
    rospy.sleep(2.0)
    # close grip
    left_gripper.close()
    rospy.sleep(2.0)
    # move to above
    move_gripper(compute_ik, arm, above_translation, above_rotation)
    rospy.sleep(2.0)
    # move to dropoff
    move_gripper(compute_ik, arm, dropoff_translation, dropoff_rotation)
    rospy.sleep(2.0)
    # open grip
    left_gripper.open()
    rospy.sleep(2.0)
    # move to above
    # move_gripper(compute_ik, arm, above_translation, above_rotation)

# Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1])

