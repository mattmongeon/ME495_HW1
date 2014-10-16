#!/usr/bin/env python

import rospy
import roslib
import sys
import math
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute


time_s = 0.0
max_time_s = 10.0

frame_time_s = 0.02

#control_pub as rospy.Publisher
control_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)


def turtlesim_control():

    rospy.init_node("turtlesim_control", anonymous=True)

    # Wait for the turtle's teleport service to start up, then move on to orient it.
    rospy.wait_for_service("/turtle1/teleport_absolute")
    orient_turtle_service = rospy.ServiceProxy("/turtle1/teleport_absolute", TeleportAbsolute)
    orient_turtle_service(5.0, 5.0, 0.0)

    rospy.Timer(rospy.Duration(frame_time_s), timer_callback)

    rospy.spin()


twist_msg = Twist()

def timer_callback(event):
    global time_s, control_pub

    theta_rad = (2.0 * math.pi * time_s) / max_time_s

    twist_msg.linear.x = 3.0 * math.sin(2.0 * theta_rad)
    twist_msg.linear.y = 3.0 * math.sin(theta_rad)

    twist_msg.angular.z = math.sin(0.25 * theta_rad)

    time_s += frame_time_s
    if time_s > max_time_s:
        time_s = 0.0

    control_pub.publish(twist_msg)


def main(args):
    turtlesim_control()
   

if __name__ == '__main__':
    main(sys.argv)

