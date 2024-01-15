#!/usr/bin/env python

import rospy
import actionlib
from nav_msgs.msg import Odometry
from assignment2_change.srv import CancelTrgt 
from assignment2_change.msg import RobotState , RobotPosFeet
# from assignment2_change.srv import Getlasttarget
from assignment2_change.msg import PlanningAction, PlanningGoal

class ActionClientNode:
    def __init__(self):
        rospy.init_node('action_client_node')

        # Adding the service that cancels the goal when called 
        self.service = rospy.Service('cancel_target', CancelTrgt, self.handle_cancel_target)


        # Action client
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Action server is available")

        # Publishers
        self.robot_state_pub = rospy.Publisher('/robot_state', RobotState, queue_size=10)

        # Adding the publisher in feet
        self.robot_state_pub = rospy.Publisher('/robot_position_in_feet', RobotState, queue_size=10)


        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo("Action client node initialized")

    # the cancelling service callback 
        
    def handle_cancel_target(self, res):
        # Service callback to update the goal
        self.client.cancel_goal()
        return True

    def send_goal(self, target_x, target_y):
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y= target_y
        self.client.send_goal(goal)


    def cancel_goal(self):
        rospy.loginfo("Canceling goal...")
        self.client.cancel_goal()


    def odom_callback(self, odom_msg):
        # Process /odom message and publish robot state
        # Example: Extracting x, y, vel_x, vel_z from odom_msg
        # Replace this with your actual logic based on /odom message structure
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        vel_x = odom_msg.twist.twist.linear.x
        vel_z = odom_msg.twist.twist.angular.z

        robot_state_msg = RobotState()
        robot_state_msg.x = x
        robot_state_msg.y = y
        robot_state_msg.vel_x = vel_x
        robot_state_msg.vel_z = vel_z

        # Updating the robot position in feet
        position_in_feet = RobotPosFeet()
        position_in_feet.x = x * 3.28
        position_in_feet.y = y * 3.28

        self.robot_state_pub.publish(position_in_feet)
        # Publishing the position in feet 
        self.robot_state_pub.publish(position_in_feet)

if __name__ == '__main__':
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

