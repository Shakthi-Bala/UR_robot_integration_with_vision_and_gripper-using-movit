#!/usr/bin/env python3

import rospy
import actionlib
from my_moveit_planner.msg import NewAction, NewResult
import sys
import moveit_commander
from moveit_commander.exception import MoveItCommanderException

class TaskServer(object):
    result_ = NewResult()
    arm_goal = []

    def __init__(self, name):
        self.action_name_ = name
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_move_group_ = moveit_commander.MoveGroupCommander("manipulator")
        self.as_ = actionlib.SimpleActionServer(self.action_name_, NewAction, execute_cb=self.execute_cb, auto_start=False)
        self.as_.start()

    def execute_cb(self, goal):
        success = True
        # Defining the goals to have six joint values
        if goal.task_number == 0:
            self.arm_goal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        elif goal.task_number == 1:
            self.arm_goal = [0.116, 0.6, 0.3, 0.0, 0.0, 0.0]
        else:
            rospy.logerr('Invalid goal received')
            self.result_.success = False
            self.as_.set_aborted(self.result_)
            return

        try:
            joint_names = self.arm_move_group_.get_active_joints()

            if len(joint_names) != len(self.arm_goal):
                rospy.logerr('The number of joint targets does not match the number of active joints')
                self.result_.success = False
                self.as_.set_aborted(self.result_)
                return

            joint_goal = dict(zip(joint_names, self.arm_goal))
            rospy.loginfo(f'Setting joint targets: {joint_goal}')
            self.arm_move_group_.set_joint_value_target(joint_goal)

            self.arm_move_group_.go(wait=True)
            self.arm_move_group_.stop()
            self.arm_move_group_.clear_pose_targets()

            if self.as_.is_preempt_requested():
                rospy.loginfo('%s is Preempted' % self.action_name_)
                self.as_.set_preempted()
                success = False

        except MoveItCommanderException as e:
            rospy.logerr("MoveItCommanderException: %s" % str(e))
            self.result_.success = False
            self.as_.set_aborted(self.result_)
            return
        except Exception as e:
            rospy.logerr("Unexpected exception: %s" % str(e))
            self.result_.success = False
            self.as_.set_aborted(self.result_)
            return

        if success:
            self.result_.success = True
            rospy.loginfo('%s Succeeded' % self.action_name_)
            self.as_.set_succeeded(self.result_)

if __name__ == '__main__':
    rospy.init_node('task_server')
    server = TaskServer('task_server')
    rospy.spin()
