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
    
    def __init__(self, name):
        self._action_name = name
        self.ur = MoveGroupPythonInterface()

        self._as = actionlib.SimpleActionServer(name=self._action_name, ActionSpec=PlatformGoalAction, execute_cb=self.execute_cb,auto_start=False)

        # create messages that are used to publish feedback/result
        self._feedback = PlatformGoalFeedback()
        self._result   = PlatformGoalResult()

        self._as.start()

        current_state = self.ur.get_current_pose()
        rospy.loginfo(current_state)

    def monitor_status(self, topic="move_group/status"):
        while True:
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                rospy.loginfo("Goal Preempted")
                return False
            
            arm_status = rospy.wait_for_message(topic, GoalStatusArray, timeout=None)
            current_status = arm_status.status_list[len(arm_status.status_list)-1]
            status = current_status.status
            
            if status == current_status.SUCCEEDED:
                return True
            elif status != current_status.ACTIVE:
                return False

            # Publish feedback
            self._feedback.status = status
            self._as.publish_feedback(self._feedback)
            rospy.sleep(0.1)



    def execute_cb(self, goal):
        success = True

        status = 0
        command = goal.command
        if command == MOVE or command == ACTUATE:
            target_poses = goal.target_poses
            link_id     = goal.link_id.data
            do_actuate  = (command == ACTUATE)

            rospy.loginfo("Goals")
            rospy.loginfo(target_poses)

            rospy.loginfo("Link ID")
            rospy.loginfo(link_id)
            
            self.ur.set_ee_link(link_id)

            if goal.cartesian_path:
                plan = self.ur.follow_cartesian_path(goal.target_poses)
                status_topic = "scaled_pos_joint_traj_controller/follow_joint_trajectory/status"
            else:
                plan = self.ur.go_to_pose_goal_cont(goal.target_poses)
                status_topic = "move_group/status"
            
            
            if plan is None:
                rospy.loginfo("Plan not found aborting")
                #stop excess movement
                self.ur.stop_moving()
                success = False
            else:
                success = self.monitor_status(status_topic)


            #stop excess movement
            self.ur.stop_moving()

        elif command == STOP:
            self.ur.stop_moving()
        else:
            rospy.loginfo("Unkown Command type: "+str(command))
          
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._result.status = status
            self._as.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Aborted' % self._action_name)
            self._as.set_aborted()

def main():
    rospy.init_node('moveit_server', anonymous=True)
    platform_controller = MoveItController("server")

    while not rospy.is_shutdown():
        pass
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
