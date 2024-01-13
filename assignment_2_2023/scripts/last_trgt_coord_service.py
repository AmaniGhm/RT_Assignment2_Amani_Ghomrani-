#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import Getlasttarget, GetlasttargetResponse
from assignment_2_2023.msg import PlanningActionGoal    

last_target = (0.0, 0.0)

def get_last_target_callback(msg):
    global last_target
    
    last_target[1] = msg.goal.target_pose.pose.position.pos_x
    last_target[2] = msg.goal.target_pose.pose.position.pos_y

def service_position_callback():
    global last_target

    response = GetlasttargetResponse()
    response.x = last_target[1]
    response.y = last_target[2]

    return response

def last_target_service():
    global last_target
    
    rospy.init_node('last_target_coord_service')

    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, get_last_target_callback)

    service = rospy.Service('/get_last_target_service', Getlasttarget, service_position_callback)
    

if __name__ == '__main__':
    last_target_service()