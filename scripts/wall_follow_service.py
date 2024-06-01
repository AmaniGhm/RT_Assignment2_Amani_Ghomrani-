#! /usr/bin/env python
"""
.. module:: assignment1
   :platform: unix
   :synopsis: ROS node that implements a simple wall follower behavior.
.. moduleauthor:: Amani Ghomrani  <angho34@gmail.com>

Subscribes to:
    /scan (sensor_msgs/LaserScan): Laser scan data.

Publishes to:
    /cmd_vel (geometry_msgs/Twist): Twist commands for the robot.

Services:
    wall_follower_switch (std_srvs/SetBool): Service to activate/deactivate the wall follower behavior.

Description:
    This node implements a basic wall follower behavior using a scanning laser sensor. 
    It divides the laser scan into five regions (front, front-left, front-right, left, right) 
    and takes actions based on the minimum range values in each region. The node can be 
    activated and deactivated using a service named `wall_follower_switch`.
   
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False

pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}


def init_():
    """
    Initialize the ROS node and set up publishers, subscribers, and services.
    """
    global pub_, active_

    rospy.init_node('reading_laser')

    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)


def wall_follower_switch(req):
    """
    Callback function for the /wall_follower_switch service.

    Args:
        req (SetBool): Service request containing the activation/deactivation command.

    Returns:
        SetBoolResponse: Service response indicating success and a message.
    """
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


def clbk_laser(msg):
    """
    Callback function for the /scan topic.

    This function updates the `regions_` dictionary with the minimum range values 
    in each of the five defined regions (right, front-right, front, front-left, left).
    It then calls the `take_action` function to determine the appropriate movement 
    based on the sensor readings.

    Args:
        msg (LaserScan): Laser scan data message.
    """
    global regions_
    regions_ = {
        'right': min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front': min(min(msg.ranges[288:431]), 10),
        'fleft': min(min(msg.ranges[432:575]), 10),
        'left': min(min(msg.ranges[576:713]), 10),
    }

    take_action()


def change_state(state):
    """
    Updates the current state of the wall follower and prints a message indicating 
    the state transition.

    Args:
        state (int): The new state of the wall follower (0: find the wall, 1: turn left, 2: follow the wall).
    """
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    """
    This function analyzes the minimum range values in each region (`regions_`) 
    and determines the appropriate movement command for the robot based on pre-defined 
    conditions. It then publishes the calculated twist message (`msg`) to the `/cmd_vel` topic.

    The function defines several cases based on the sensor readings and sets the 
    linear and angular velocities (`linear_x` and `angular_z`) accordingly. 
    A state description variable (`state_description`) is used for logging purposes.
    """
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''

    d0 = 1
    d = 1.5

    if regions['front'] > d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d0 and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d0 and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)


def find_wall():
    """
    This function set a linear and angular velocity ro find the wall
    """
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.3
    return msg


def turn_left():
    """
    This function set an angular velocity to make the robot turn left
    """
    msg = Twist()
    msg.angular.z = 0.3
    return msg


def follow_the_wall():
    """
    This function set an lineaer velocity to make the robot follow the wall
    """
    global regions_

    msg = Twist()
    msg.linear.x = 0.5
    return msg

def main():
    """
    This function handles the robot rotation and linear velocity to make it follow the wall
    """
    init_()
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        else:
            msg = Twist()
            if state_ == 0:
                msg = find_wall()
            elif state_ == 1:
                msg = turn_left()
            elif state_ == 2:
                msg = follow_the_wall()
            else:
                rospy.logerr('Unknown state!')

            pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()