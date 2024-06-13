#! /usr/bin/env python3
import rospy
import actionlib
from my_moveit_planner.msg import NewAction, NewGoal

def action_client():
    client = actionlib.SimpleActionClient('Pick',NewAction)
    client.wait_for_server()
    goal = NewGoal()
    print('sending goal order')
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('action_client')
    rospy.loginfo('Action client started')
    result = action_client()
    print('result', result.sequence)