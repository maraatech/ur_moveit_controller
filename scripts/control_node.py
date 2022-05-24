#!/usr/bin/env python3
import rospy
import roslib
import tf
import math

from cv_bridge import CvBridge

from cares_msgs.msg import PlatformGoalAction, PlatformGoalGoal, PlatformGoalFeedback, PlatformGoalResult
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

class MoveItController():

    def __init__(self, name, move_group_name):
        self._action_name = name
        self.arm_interface = MoveGroupPythonInterface(move_group_name)
        current_state = self.arm_interface.get_current_pose()
        print(current_state)

        self.control_server = actionlib.SimpleActionServer(name=self._action_name, ActionSpec=PlatformGoalAction, execute_cb=self.execute_cb,auto_start=False)

        # create messages that are used to publish feedback/result
        self.feedback = PlatformGoalFeedback()
        self.result   = PlatformGoalResult()

        self.control_server.start()


    def execute_cb(self, goal):
        status = 0
        command = goal.command
        if command == goal.MOVE or command == goal.ACTUATE:
            target_pose = goal.target_pose
            link_id     = goal.link_id.data
            do_actuate  = (command == goal.ACTUATE)

            print("Goal")
            print(target_pose)

            print("Link ID")
            print(link_id)

            self.arm_interface.set_ee_link(link_id)
            plan = self.arm_interface.go_to_pose_goal_cont(goal.target_pose)

            if not plan:
                print("Plan not found aborting")
                #stop excess movement
                self.arm_interface.stop_moving()
                self.control_server.set_aborted()
                return

            r = rospy.Rate(10)
            while True:
                if rospy.is_shutdown():
                    self.control_server.set_aborted()
                    return

                if self.control_server.is_preempt_requested():
                    self.control_server.set_preempted()
                    print("Goal Preempted")
                    #stop excess movement
                    self.arm_interface.stop_moving()
                    return

                arm_status = rospy.wait_for_message("move_group/status", GoalStatusArray, timeout=None)
                current_status = arm_status.status_list[len(arm_status.status_list)-1]

                status = current_status.status
                if status == current_status.SUCCEEDED:
                    break
                elif status != current_status.ACTIVE:
                    self.control_server.set_aborted()
                    return

                # Publish feedback
                self.feedback.status = status
                self.control_server.publish_feedback(self.feedback)
                r.sleep()

            #stop excess movement
            self.arm_interface.stop_moving()

        elif command == goal.STOP:
            self.arm_interface.stop_moving()
        else:
            print("Unkown Command type: "+str(command))
            self.control_server.set_aborted()
            return

        rospy.loginfo('%s: Succeeded' % self._action_name)
        self.result.status = status
        self.control_server.set_succeeded(self.result)

def main():
    rospy.init_node('moveit_server', anonymous=True)

    move_group_name = rospy.get_param('~move_group_name', "manipulator")
    platform_controller = MoveItController(move_group_name+"_server", move_group_name)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
