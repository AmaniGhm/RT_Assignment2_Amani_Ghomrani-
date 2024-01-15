#!/usr/bin/env python

import rospy
from assignment2_change.srv import CancelTrgt , CancelTrgtResponse
from assignment2_change.msg import PlanningActionGoal
from std_msgs.msg import Float64

# Initialize target coordinates
new_x = 0.0
new_y = 0.0

class GetLastTargetNode:
    def __init__(self):
        rospy.init_node('cancel_target_node')

        # Service server
        self.service = rospy.Service('cancel_target', CancelTrgt, self.handle_cancel_target)

        # Subscriber
        rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, self.goal_callback)


        rospy.loginfo("GetLastTargetNode initialized")

    def handle_cancel_target(self, res):
        global new_y, new_x
        # Service callback to update the goal
        response = CancelTrgtResponse()
        response.target_x = new_x
        response.target_y = new_y
        return response

    def goal_callback(self, msg):
        global new_y, new_x
        # Update target coordinates when a new goal is received
        new_x = msg.goal.target_pose.pose.position.x
        new_y = msg.goal.target_pose.pose.position.y

if __name__ == '__main__':
    try:
        node = GetLastTargetNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
