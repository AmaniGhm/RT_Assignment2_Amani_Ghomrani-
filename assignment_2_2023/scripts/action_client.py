#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from assignment_2_2023.msg import Vel_pos_xy
import assignment_2_2023.msg
from std_srvs.srv import *
from nav_msgs.msg import Odometry
import actionlib 
import actionlib.msg

def update_current_robot_state(msg):

    global current_state_publisher
    # extract the current odometry position and velocity from the message
    position = msg.pose.pose.position
    velocity = msg.twist.twist.linear
    rotation = msg.twist.twist.angular

    # create a custom message to store and publish the current state
    current_state = Vel_pos_xy()
    current_state.pos_x = position.x
    current_state.pos_y = position.y
    current_state.vel_x = velocity.x
    current_state.vel_z = rotation.z
    current_state_publisher.publish(current_state)

    
def main():
    global current_state_publisher
    # create a publisher to send the current state of the robot
    current_state_publisher = rospy.Publisher("/pos_vel", Vel_pos_xy, queue_size = 10)

    # subscribe to the odometry topic to receive updates on the robot's position and velocity
    odom_subscriber = rospy.Subscriber("/odom", Odometry, update_current_robot_state)

    # create a client for the reaching_goal action server
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)
    rospy.loginfo('Waiting for server to start...')
    client.wait_for_server()

    while not rospy.is_shutdown():
        # get the desired x and y position from the user
        x_des = float(input("Please enter the desired X position: "))
        y_des = float(input("Please enter the desired Y position: "))
        rospy.loginfo("Target position is set!")

        goal = assignment_2_2023.msg.PlanningGoal()
        goal.target_pose.pose.position.x = x_des
        goal.target_pose.pose.position.y = y_des 
        client.send_goal(goal)
        
        # client.wait_for_result()


        # check if the user wants to cancel the goal
        c_input = input("Please type 'cancel' to cancel the goal: ")
        if (c_input == "cancel"):
            print("Goal canceled!")
            client.cancel_goal()
        else:
            continue

        current_state = client.get_state()

        if current_state == actionlib.GoalStatus.SUCCEEDED:
            # The goal has succeeded
            rospy.loginfo('Goal succeeded!')
            break
        elif current_state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED, actionlib.GoalStatus.LOST]:
            # The goal has aborted, rejected, or lost
            rospy.logerr('Goal failed !')
            break

if __name__ == '__main__':
    rospy.init_node('user_input')
    main()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()

