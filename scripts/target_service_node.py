#!/usr/bin/env python
"""
Subscribes to:
   /reaching_goal/goal (message type: PlanningActionGoal)

Services:
   get_last_target (service type: Getlasttarget) - Returns the latest target coordinates (target_x, target_y).

Description:
   This ROS node retrieves the last target coordinates received on the `/reaching_goal/goal` topic 
   and provides them as a service response. The service is named `get_last_target` and returns a message 
   of type `GetlasttargetResponse` containing the `target_x` and `target_y` values.
"""

import rospy
from assignment_2_2023.srv import Getlasttarget , GetlasttargetResponse
from assignment_2_2023.msg import PlanningActionGoal
from std_msgs.msg import Float64

# Initialize target coordinates
new_x = 0.0
new_y = 0.0

class GetLastTargetNode:
    """
    This class implements a ROS node that retrieves the last target coordinates 
    and provides them as a service response.
    """
    def __init__(self):
        rospy.init_node('get_last_target_node')

        # Service server (advertise /get_last_target service of type Getlasttarget)
        self.service = rospy.Service('get_last_target', Getlasttarget, self.handle_get_last_target)

        # Subscriber to /reaching_goal/goal topic (message type: PlanningActionGoal)
        rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, self.goal_callback)


        rospy.loginfo("GetLastTargetNode initialized")

    def handle_get_last_target(self, res):
        """
        Service callback function to handle requests to the /get_last_target service.

        Args:
            req (Getlasttarget): Service request object.

        Returns:
            GetlasttargetResponse: Service response object containing the last target coordinates.
        """
        global new_y, new_x
        response = GetlasttargetResponse()
        response.target_x = new_x
        response.target_y = new_y
        return response

    def goal_callback(self, msg):
        """
        Callback function triggered when a new message is received on the subscribed topic.

        Args:
            msg (PlanningActionGoal): Incoming message containing the new goal pose.
        """
        global new_y, new_x
        # Update target coordinates with the latest goal pose
        new_x = msg.goal.target_pose.pose.position.x
        new_y = msg.goal.target_pose.pose.position.y

if __name__ == '__main__':
    try:
        node = GetLastTargetNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
