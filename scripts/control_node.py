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

from actionlib_msgs.msg import GoalStatusArray

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
    
    def __init__(self, name, move_group_name):
        self._action_name = name
        self.ur = MoveGroupPythonInterface(move_group_name)

        self._as = actionlib.SimpleActionServer(name=self._action_name, ActionSpec=PlatformGoalAction, execute_cb=self.execute_cb,auto_start=False)

        # create messages that are used to publish feedback/result
        self._feedback = PlatformGoalFeedback()
        self._result   = PlatformGoalResult()

        self._as.start()

        current_state = self.ur.get_current_pose()
        print(current_state)

    def execute_cb(self, goal):
        success = True

        status = 0
        command = goal.command
        if command == MOVE or command == ACTUATE:
            target_pose = goal.target_pose
            link_id     = goal.link_id.data
            do_actuate  = (command == ACTUATE)

            print("Goal")
            print(target_pose)

            print("Link ID")
            print(link_id)
            
            self.ur.set_ee_link(link_id)
            plan = self.ur.go_to_pose_goal_cont(goal.target_pose)

            if not plan:
                print("Plan not found aborting")
                #stop excess movement
                self.ur.stop_moving()
                success = False
            else:
                rospy.sleep(1)
                while True:
                    if self._as.is_preempt_requested():
                        self._as.set_preempted()
                        print("Goal Preempted")
                        #stop excess movement
                        self.ur.stop_moving()
                        success = False
                        break
                    
                    arm_status = rospy.wait_for_message("move_group/status", GoalStatusArray, timeout=None)
                    current_status = arm_status.status_list[len(arm_status.status_list)-1]

                    status = current_status.status
                    
                    if status == current_status.SUCCEEDED:
                        success = True
                        break
                    elif status != current_status.ACTIVE:
                        success = False
                        break

                    # Publish feedback
                    self._feedback.status = status
                    self._as.publish_feedback(self._feedback)
                    rospy.sleep(0.1)

            #stop excess movement
            self.ur.stop_moving()

        elif command == STOP:
            self.ur.stop_moving()
        else:
            print("Unkown Command type: "+str(command))
          
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._result.status = status
            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted()

def main():
    rospy.init_node('moveit_server', anonymous=True)

    move_group_name = rospy.get_param('~move_group_name', "manipulator")
    platform_controller = MoveItController(move_group_name+"_server", move_group_name)

    while not rospy.is_shutdown():
        pass
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
