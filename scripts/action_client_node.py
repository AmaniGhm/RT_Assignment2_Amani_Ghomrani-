#!/usr/bin/env python
"""
.. module::assignment1
   :platform:unix
   :synopsys: A ROS node for setting and cancelling Robot Goal
.. module eauthor:: Amani Ghomrani <angho34@gmail.com>

Subscribes to:
    /odom

Publishes to:
    /robot_state

"""

import rospy
import actionlib
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import RobotState
# from assignment_2_2023.srv import Getlasttarget
from assignment_2_2023.msg import PlanningAction, PlanningGoal

class ActionClientNode:
    """
    A ROS node that acts as an action client for sending goals and receiving feedback.

    This node subscribes to the /odom topic to receive odometry messages and publishes 
    the robot state to the /robot_state topic. It also communicates with an action server 
    to send goals and cancel them if needed.
    """
    def __init__(self):
        """
        Initialize the ActionClientNode.

        This sets up the ROS node, action client, publisher, and subscriber.
        """
        rospy.init_node('action_client_node')

        # Action client
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Action server is available")

        # Publishers
        self.robot_state_pub = rospy.Publisher('/robot_state', RobotState, queue_size=10)

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo("Action client node initialized")

    def send_goal(self, target_x, target_y):
        """
        Send a goal to the action server.   

        :param target_x: The x-coordinate of the goal position.
        :type target_x: float
        :param target_y: The y-coordinate of the goal position.
        :type target_y: float
        """
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y= target_y
        self.client.send_goal(goal)


    def cancel_goal(self):
        """
        Cancel the current goal.
        """
        rospy.loginfo("Canceling goal...")
        self.client.cancel_goal()


    def odom_callback(self, odom_msg):
        """
        Callback function for /odom topic.

        This function processes the incoming odometry messages and publishes the robot state.

        :param odom_msg: The odometry message.
        :type odom_msg: nav_msgs.msg.Odometry
        """
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        vel_x = odom_msg.twist.twist.linear.x
        vel_z = odom_msg.twist.twist.angular.z

        robot_state_msg = RobotState()
        robot_state_msg.x = x
        robot_state_msg.y = y
        robot_state_msg.vel_x = vel_x
        robot_state_msg.vel_z = vel_z

        self.robot_state_pub.publish(robot_state_msg)

if __name__ == '__main__':
    """
    Main entry point of the script. 

    This creates an instance of ActionClientNode and continuously prompts the user 
    for goal coordinates to send to the action server. It also allows the user to 
    cancel the goal.
    """
    try:
        node = ActionClientNode()
        while not rospy.is_shutdown():

            # Take user input for goal parameters
            target_x = float(input("Enter the x-coordinate for the goal: "))
            target_y = float(input("Enter the y-coordinate for the goal: "))

            # Send the goal
            node.send_goal(target_x, target_y)

            # Get the state of the goal
            current_state = node.client.get_state()

            # Take user input for timing of goal cancellation or type 'cancel' to cancel immediately
            cancel_input = input("Type 'cancel' to cancel immediately: ")

            if cancel_input.lower() == 'cancel':
                # Cancel the goal immediately
                rospy.loginfo('Goal Cancelled!')
                node.cancel_goal()
                continue
            

            # Spin to keep the node alive
            # rospy.spin()

    except rospy.ROSInterruptException:
        pass

