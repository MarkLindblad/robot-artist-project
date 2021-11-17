#!/usr/bin/env python
import sys
from geometry_msgs.msg import Twist
import numpy as np
import rospy
from std_srvs.srv import Empty
from turtle_patrol.srv import Patrol  # Service type
from turtlesim.srv import TeleportAbsolute


def patrol_callback(request):
    turtle_name = sys.argv[1]
    rospy.wait_for_service('clear')
    rospy.wait_for_service('/' + turtle_name + '/teleport_absolute')
    clear_proxy = rospy.ServiceProxy('clear', Empty)
    teleport_proxy = rospy.ServiceProxy(
        '/' + turtle_name + '/teleport_absolute',
        TeleportAbsolute
    )
    vel = request.vel  # Linear velocity
    omega = request.omega  # Angular velocity
    x = request.x
    y = request.y
    theta = request.theta
    pub = rospy.Publisher(
        '/' + turtle_name + '/cmd_vel', Twist, queue_size=50)
    cmd = Twist()
    cmd.linear.x = vel
    cmd.angular.z = omega
    # Publish to cmd_vel at 5 Hz
    rate = rospy.Rate(5)
    # Teleport to initial pose
    teleport_proxy(x, y, theta)
    # Clear historical path traces
    clear_proxy()
    while not rospy.is_shutdown():
        pub.publish(cmd)  # Publish to cmd_vel
        rate.sleep()  # Sleep until 
    return cmd  # This line will never be reached

def patrol_server():
    turtle_name = sys.argv[1]
    # Initialize the server node for turtle_name
    rospy.init_node(turtle_name + '_patrol_server')
    # Register service
    rospy.Service(
        '/' + turtle_name + '/patrol',  # Service name
        Patrol,  # Service type
        patrol_callback  # Service callback
    )
    rospy.loginfo('Running patrol server...')
    rospy.spin() # Spin the node until Ctrl-C


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Invalid arguments")
        sys.exit()
    patrol_server()

