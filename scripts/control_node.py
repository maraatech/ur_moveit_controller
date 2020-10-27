#!/usr/bin/env python
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

from Queue import Queue
from Queue import Empty

from os.path import expanduser
home = expanduser("~")

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
    
    def __init__(self, name):
        self._action_name = name
        self.ur = MoveGroupPythonInterface()

        self._as = actionlib.SimpleActionServer(name=self._action_name, ActionSpec=PlatformGoalAction, execute_cb=self.execute_cb,auto_start=False)

        # create messages that are used to publish feedback/result
        self._feedback = maara_platform_control.msg.PlatformGoalFeedback()
        self._result   = maara_platform_control.msg.PlatformGoalResult()

        self._as.start()

    def execute_cb(self, goal):
        success = True

        command = goal.command
        if command == MOVE or command == ACTUATE:
            target_pose = goal.target_pose
            link_id     = goal.link_id.data
            do_actuate  = (command == ACTUATE)

            print("Goal")
            print(target_pose)
            
            self.ur.set_ee_link(arm_command.ee_id.data)
            self.ur.go_to_pose_goal_cont(arm_command.target_pose)

            while not self.ur.at_goal(arm_command.target_pose):
                if self._as.is_preempt_requested():
                    self._as.set_preempted()
                    #stop excess movement
                    self.ur.stop_moving()
                    success = False
                    break
                
                current_state = self.ur.get_current_sate()
                print(current_state)
                # Publish feedback
                # Feedback is it is moving and doing the current action
                self._feedback.status = MOVE
                self._as.publish_feedback(self._feedback)
                time.sleep(0.1)

            #stop excess movement
            self.ur.stop_moving()

        elif command == STOP:
            self.ur.stop_moving()
        else:
            print("Unkown Command type: "+str(command))
          
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self.result.status = STOP
            self._as.set_succeeded(self._result)

def main():
    rospy.init_node('moveit_server', anonymous=True)
    platform_controller = MoveItController()

    while not rospy.is_shutdown():
        pass
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass