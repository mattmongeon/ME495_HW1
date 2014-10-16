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

control_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)


def turtlesim_control():

    rospy.init_node("turtlesim_control", anonymous=True)

    max_time_s = rospy.get_param("~max_time_s", 10.0)
    print "T = " + str(max_time_s)

    # Wait for the turtle's teleport service to start up, then move on to orient it.
    print "Waiting for turtle service..."
    rospy.wait_for_service("/turtle1/teleport_absolute")
    orient_turtle_service = rospy.ServiceProxy("/turtle1/teleport_absolute", TeleportAbsolute)
    orient_turtle_service(5.0, 5.0, 0.0)
    print "Turtle position set!"

    rospy.Timer(rospy.Duration(frame_time_s), timer_callback)

    rospy.spin()


twist_msg = Twist()
prev_theta_rad = 0.0

def timer_callback(event):
    global time_s, control_pub, prev_theta_rad

    theta_rad = (2.0 * math.pi * time_s) / max_time_s

    x_t = 3.0 * math.sin(2.0 * theta_rad)
    y_t = 3.0 * math.sin(theta_rad)

    v_x_t = ((12.0 * math.pi) / max_time_s) * math.cos(2.0 * theta_rad)
    v_y_t = ((6.0 * math.pi) / max_time_s) * math.cos(theta_rad)

    twist_msg.linear.x = math.sqrt(v_x_t**2 + v_y_t**2)

    # This is completely unsatisfying, because I wanted this to be solved
    # more elegantly, but I couldn't figure it out...
    ang_vel_rad_per_s = 2.0 * (theta_rad - prev_theta_rad) / frame_time_s
    if time_s > (max_time_s * 0.5):
        ang_vel_rad_per_s = -ang_vel_rad_per_s

    twist_msg.angular.z = ang_vel_rad_per_s

    prev_theta_rad = theta_rad

    print str(time_s) + ", " + ang_vel_rad_per_s


    time_s += frame_time_s
    if time_s > max_time_s:
        time_s = 0.0

    control_pub.publish(twist_msg)


def main(args):
    turtlesim_control()
   

if __name__ == '__main__':
    main(sys.argv)

