#! /usr/bin/env python
"""
.. module:: assignment1
   :platform: unix
   :synopsis: ROS node that prints updates on the robot's position and velocity.
.. moduleauthor:: Amani Ghomrani <angho34@gmail.com>

Subscribes to:
    /robot_state

Publishes to:
    None

Services:
    distance_velocity_from_target

Description:
    This ROS node subscribes to the /robot_state topic to receive updates on the robot's position and velocity.
    It calculates the distance from a target position and the average speed of the robot, printing this information
    periodically. Additionally, it provides a service to retrieve the current distance and average speed.
"""

import rospy
import math
import time
from assignment_2_2023.msg import RobotState
from assignment_2_2023.srv import GetDistSpeed, GetDistSpeedResponse

previous_print_time = 0
frequency_param = 1.0
distance = 0
average_speed = 0

def print_robot_info(msg):
    """
    Callback function for the /robot_state topic subscriber. It calculates the distance from the target
    position and the average speed of the robot, then prints this information periodically based on the
    frequency parameter.

    Args:
        msg (RobotState): Message containing the current state of the robot, including position and velocity.
    """
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
    """
    Service callback function to handle requests for the robot's distance from the target
    and its average speed.

    :param rsp: Request from the GetDistSpeed service.
    :type rsp: GetDistSpeedRequest
    :return: Response containing the distance and average speed of the robot.
    :rtype: GetDistSpeedResponse
    """
    global distance, average_speed

    # Service callback to update the distace and the average speed of the robot
    response = GetDistSpeedResponse()
    response.distance = distance
    response.average_speed = average_speed
    return response

if __name__ == "__main__":
    """
    Main function to initialize the ROS node, set up the service, and subscribe to the /robot_state topic.
    """
    rospy.init_node('rob_position')

    # service that store the velocity and average speed of the robot
    distance_velocity_service = rospy.Service('distance_velocity_from_target', GetDistSpeed, dist_velocity_callbk)
    
    # Subscriber to the robot_state
    pos_vel_subscriber = rospy.Subscriber("/robot_state", RobotState, print_robot_info)
    rospy.spin()
