#!/usr/bin/env python

import rospy
import roslib
import sys
import math
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute


time_s = 0.0
max_time_s = 10.0
frame_time_s = 0.01  # Just for kicks we will run at 100 Hz.

twist_msg = Twist()

control_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)


def get_linear_velocities_m_per_s(cycle_pos):
    v_x_t = ((12.0 * math.pi) / max_time_s) * math.cos(2.0 * cycle_pos)
    v_y_t = ((6.0 * math.pi) / max_time_s) * math.cos(cycle_pos)

    return v_x_t, v_y_t


def get_angular_velocity_rad_per_s(cycle_pos):
    ang_vel_rad_per_s = 3.0 * math.sin(cycle_pos) + math.sin(3.0 * cycle_pos)
    ang_vel_rad_per_s *= 4.0 * math.pi

    ang_vel_rad_per_s /= max_time_s * (math.cos(2.0 * cycle_pos) + 4.0 * math.cos(4.0 * cycle_pos) + 5.0)

    return ang_vel_rad_per_s


def turtlesim_control():

    global max_time_s

    rospy.init_node("turtlesim_control", anonymous=True)

    max_time_s = rospy.get_param("~max_time_s", 10.0)
    print "T = " + str(max_time_s)

    # Wait for the turtle's teleport service to start up, then move on to orient it.
    print "Waiting for turtle service..."
    rospy.wait_for_service("/turtle1/teleport_absolute")
    orient_turtle_service = rospy.ServiceProxy("/turtle1/teleport_absolute", TeleportAbsolute)

    # The initial orientation should be based on the theta determined from the
    # initial velocities at t=0.
    v_x_t, v_y_t = get_linear_velocities_m_per_s(0.0)
    init_theta_rad = math.atan2(v_y_t, v_x_t)
    orient_turtle_service(5.0, 5.0, init_theta_rad)

    print "Turtle position set!"

    rospy.Timer(rospy.Duration(frame_time_s), timer_callback)

    rospy.spin()


def timer_callback(event):
    global time_s, control_pub

    cycle_pos = (2.0 * math.pi * time_s) / max_time_s

    v_x_t, v_y_t = get_linear_velocities_m_per_s(cycle_pos)
    twist_msg.linear.x = math.sqrt(v_x_t**2 + v_y_t**2)

    twist_msg.angular.z = get_angular_velocity_rad_per_s(cycle_pos)

    time_s += frame_time_s
    if time_s >= max_time_s:
        time_s = 0.0

    control_pub.publish(twist_msg)


def main(args):
    turtlesim_control()
   

if __name__ == '__main__':
    main(sys.argv)

