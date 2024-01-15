#! /usr/bin/env python

import rospy
import math
import time
from assignment2_change.msg import RobotState
from assignment2_change.srv import GetDistSpeed, GetDistSpeedResponse

previous_print_time = 0
frequency_param = 1.0
distance = 0
average_speed = 0

def print_robot_info(msg):
    global previous_print_time, frequency_param, distance, average_speed
    time_period = (1.0/frequency_param) * 1000
    current_time = time.time() * 1000

    if current_time - previous_print_time > time_period:
        # Get the target position
        target_x = rospy.get_param("des_pos_x")
        target_y = rospy.get_param("des_pos_y")

        # get the current robot position
        current_x = msg.x
        current_y = msg.y

        # Calculate the distance and the speed
        distance = math.dist([target_x, target_y], [current_x, current_y])
        average_speed = math.sqrt(msg.vel_x**2 + msg.vel_z**2)

        # Print the data 
        print(' \n Robot is {:.3f}m far from goal'.format(float(distance)))
        print(' \n The average speed is {:.3f} m\s'.format(float(average_speed)))

        previous_print_time = current_time

def dist_velocity_callbk(rsp):
    global distance, average_speed

    # Service callback to update the distace and the average speed of the robot
    response = GetDistSpeedResponse()
    response.distance = distance
    response.average_speed = average_speed
    return response

if __name__ == "__main__":
    rospy.init_node('rob_position')

    # service that store the velocity and average speed of the robot
    distance_velocity_service = rospy.Service('distance_velocity_from_target', GetDistSpeed, dist_velocity_callbk)
    
    # Subscriber to the robot_state
    pos_vel_subscriber = rospy.Subscriber("/robot_state", RobotState, print_robot_info)
    rospy.spin()
