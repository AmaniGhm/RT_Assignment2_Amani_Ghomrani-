#! /usr/bin/env python

import rospy
import math
import time
from assignment_2_2023.msg import Vel_pos_xy

previous_print_time = 0
frequency_param = 1.0

def print_robot_info(msg):
    global previous_print_time, frequency_param
    time_period = (1.0/frequency_param) * 1000
    current_time = time.time() * 1000

    if current_time - previous_print_time > time_period:
        # Get the target position
        target_x = rospy.get_param("des_pos_x")
        target_y = rospy.get_param("des_pos_y")

        # get the current robot position
        current_x = msg.pos_x
        current_y = msg.pos_y

        # Calculate the distance and the speed
        distance = math.dist([target_x, target_y], [current_x, current_y])
        average_speed = math.sqrt(msg.vel_x**2 + msg.vel_z**2)

        # Print the data 
        print(' \n Robot is {:.3f}m far from goal'.format(float(distance)))
        print(' \n The average speed is {:.3f} m\s'.format(float(average_speed)))

        previous_print_time = current_time

if __name__ == "__main__":
    rospy.init_node('rob_position')
    pos_vel_subscriber = rospy.Subscriber("/pos_vel", Vel_pos_xy, print_robot_info)
    rospy.spin()
