#!/usr/bin/env python3
import rospy
import roslib
import tf
import math 

from cv_bridge import CvBridge

from platform_msgs.msg import PlatformGoalAction, PlatformGoalGoal, PlatformGoalFeedback, PlatformGoalResult
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion

import message_filters

from move_group_python_interface import MoveGroupPythonInterface
from tf.transformations import quaternion_from_euler
from math import pi

import numpy as np
import cv2

from queue import Queue
from queue import Empty

from os.path import expanduser

import actionlib
home = expanduser("~")

import time

# Arm command
# Enumeration
# 0 Move
# 1 Stop
# 2 Reset
MOVE=0
STOP=1
RESET=2
ACTUATE = 3

class MoveItController():
    
    def __init__(self, name, planning_time=5.0):
        self._action_name = name
        self.ur = MoveGroupPythonInterface(planning_time=planning_time)

        self._as = actionlib.SimpleActionServer(name=self._action_name, ActionSpec=PlatformGoalAction, execute_cb=self.execute_cb,auto_start=False)

        # create messages that are used to publish feedback/result
        self._feedback = PlatformGoalFeedback()
        self._result   = PlatformGoalResult()

        self._as.start()

        current_state = self.ur.get_current_pose()
        rospy.loginfo(current_state)

        # current_state.position.x = 0.00
        # current_state.position.y = 0.75
        # current_state.position.z = 0.75
        # self.ur.go_to_pose_goal(current_state)

        # print(current_state)

    def execute_cb(self, goal):

        command = goal.command
        if command == MOVE or command == ACTUATE:
            target_pose = goal.target_pose
            link_id     = goal.link_id.data
            do_actuate  = (command == ACTUATE)

            rospy.loginfo("Goal")
            rospy.loginfo(target_pose)

            rospy.loginfo("Link ID")
            rospy.loginfo(link_id)
            
            self.ur.set_ee_link(link_id)
            plan = self.ur.go_to_pose_goal_cont(goal.target_pose)

            if not plan:
                rospy.logerr("Plan not found aborting ")
                #stop excess movement
                self.ur.stop_moving()
                self._as.set_aborted()
                return

            while not self.ur.at_goal(goal.target_pose):
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    rospy.loginfo("Preempted!")
                    #stop excess movement
                    self.ur.stop_moving()
                    success = False
                    break

                current_state = self.ur.get_current_state()
                rospy.loginfo(current_state)

                # Publish feedback
                # Feedback is it is moving and doing the current action
                self._feedback.status = MOVE
                self._as.publish_feedback(self._feedback)
                time.sleep(0.1)

                break
            else:
                #stop excess movement
                self.ur.stop_moving()

        elif command == STOP:
            self.ur.stop_moving()
        else:
            rospy.logerr("Unkown Command type: "+str(command))
          
        if self.ur.at_goal(goal.target_pose):
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._result.status = STOP
            self._as.set_succeeded(self._result)
        # else:
        #     self._as.set_aborted()


def main():
    rospy.init_node('moveit_server', anonymous=True)
    planning_time = rospy.get_param("~planning_time", 5.0)

    platform_controller = MoveItController("server", planning_time)

    while not rospy.is_shutdown():
        pass
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
