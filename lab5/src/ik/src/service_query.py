#!/usr/bin/env python
import rospy
import numpy as np
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped

########## UNCOMMENT THIS LINE TO TEST WITH LAB 3 FORWARD KINEMATICS CODE ##########
from baxter_forward_kinematics import baxter_forward_kinematics_from_angles

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    while not rospy.is_shutdown():
        xyz = raw_input('Enter the xyz cordinates for gripper: ')
        x, y, z = xyz.split()
        x, y, z = float(x), float(y), float(z)
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_hand"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z
        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            # Send the request to the service
            for _ in range(10):
                response = compute_ik(request)
                if response.error_code.val != -31:
                    break
            
            # Print the response HERE
            print(response)
            # print(dir(response))

            ########## UNCOMMENT THIS LINE TO TEST WITH LAB 3 FORWARD KINEMATICS CODE ##########
            print(baxter_forward_kinematics_from_angles(np.array(response.solution.joint_state.position[1:8])))
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

# Python's syntax for a main() method
if __name__ == '__main__':
    main()

