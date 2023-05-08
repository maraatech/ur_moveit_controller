#!/usr/bin/env python3
import rospy
import roslib
import tf
import tf2_ros
import math

from cv_bridge import CvBridge

from cares_msgs.msg import PlatformGoalAction, PlatformGoalGoal, PlatformGoalFeedback, PlatformGoalResult
from cares_msgs.msg import PathPlanningConstraints
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import BoundingVolume
from moveit_msgs.msg import Constraints, PositionConstraint, JointConstraint, OrientationConstraint

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

import cares_lib_ros.utils as utils
from cares_lib_ros.action_client import ActionClient
from actionlib_msgs.msg import GoalStatusArray

class MoveItController():

    def __init__(self, name, move_group_name):
        self._action_name = name
        self.arm_interface = MoveGroupPythonInterface(move_group_name)
        current_state = self.arm_interface.get_current_pose()
        print(current_state)

        self.control_server = actionlib.SimpleActionServer(name=self._action_name, ActionSpec=PlatformGoalAction, execute_cb=self.execute_cb,auto_start=False)


        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))#tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # create messages that are used to publish feedback/result
        self.feedback = PlatformGoalFeedback()
        self.result   = PlatformGoalResult()

        self.control_server.start()


    def execute_cb(self, goal):
        status = 0
        command = goal.command
        if command == PlatformGoalGoal.MOVE or command == PlatformGoalGoal.ACTUATE:
            target_pose = goal.target_pose
            link_id     = goal.link_id.data
            do_actuate  = (command == goal.ACTUATE)

            if goal.path_constraints.allowed_planning_time: 
                self.arm_interface.move_group.set_planning_time(goal.path_constraints.allowed_planning_time)
        
            self.arm_interface.set_ee_link(link_id)
            # Set path constraints, if empty will set to empty array
            self.arm_interface.move_group.clear_path_constraints()
            constraints = goal.path_constraints.path_constraints
            print("Goal")
            print(goal)

            print("Link ID")
            print(link_id)
            if goal.path_constraints.fix_end_effector:
                orientation_constraint = self.generate_orientation_constraint(target_pose, link_id, 0.01)
                constraints.orientation_constraints.extend(orientation_constraint)

            if goal.path_constraints.volume_constraint:
                position_constraint = self.generate_position_constraint(goal.path_constraints.volume_constraint, target_pose, link_id)
                constraints.position_constraints.append(position_constraint)

            print(f"Setting volume constraints: {constraints}")
            self.arm_interface.move_group.set_path_constraints(constraints)               

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
                    self.arm_interface.stop_moving()
                    return

                if self.control_server.is_preempt_requested():
                    self.control_server.set_preempted()
                    print("Goal Preempted")
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

        elif command == PlatformGoalGoal.STOP:
            print("Stopping")
            self.arm_interface.stop_moving()
        else:
            print("Unkown Command type: "+str(command))
            self.control_server.set_aborted()
            return

        rospy.loginfo('%s: Succeeded' % self._action_name)
        self.result.status = status
        self.control_server.set_succeeded(self.result)



    def generate_orientation_constraint(self, target_pose, link_id, tolerance):
            orientation_constraint_1 = OrientationConstraint()
            orientation_constraint_1.header = target_pose.header
            orientation_constraint_1.link_name = link_id
            orientation_constraint_1.orientation = target_pose.pose.orientation
            orientation_constraint_1.weight = 1.0
            orientation_constraint_1.absolute_x_axis_tolerance = 0.01
            orientation_constraint_1.absolute_y_axis_tolerance = 0.01
            orientation_constraint_1.absolute_z_axis_tolerance = 0.01

            return [orientation_constraint_1]

    def generate_position_constraint(self, constraint_type, target_pose, link_id, tolerance=0.025):
        def orientate_around_z_axis(current_pose, target_pose):
            pose_arr = np.array([current_pose.x, current_pose.y, current_pose.z])
            target_arr = np.array([target_pose.x, target_pose.y, target_pose.z])
            orientation = utils.look_at_pose(pose_arr, target_arr, up=utils.World.up).orientation

            euler = tf.transformations.euler_from_quaternion(np.array([orientation.x, orientation.y, orientation.z, orientation.w]))
            #Offset to centre z axis rotation between current and target poses
            r = euler[0]
            p = euler[1] - math.pi/2
            y = euler[2]

            return Quaternion(*tf.transformations.quaternion_from_euler(r, p, y))

        current_pose_world = self.arm_interface.move_group.get_current_pose()
        current_pose = self.tf_buffer.transform(current_pose_world, target_pose.header.frame_id)
        print(f"Current Pose: {current_pose}")
        current_xyz = current_pose.pose.position
        target_xyz = target_pose.pose.position

        x_diff = abs(round(current_xyz.x, 2) - target_xyz.x) + tolerance
        y_diff = abs(round(current_xyz.y, 2) - target_xyz.y) + tolerance
        z_diff = abs(round(current_xyz.z, 2) - target_xyz.z) + tolerance

        position_constraint = PositionConstraint()
        position_constraint.header = target_pose.header
        position_constraint.link_name = link_id
        position_constraint.weight = 1.0


        primitive = SolidPrimitive()
        x = (round(current_xyz.x, 2) + target_xyz.x)/2
        y = (round(current_xyz.y, 2) + target_xyz.y)/2
        z = (round(current_xyz.z, 2) + target_xyz.z)/2
        
        primitive_pose = Pose()
        primitive_pose.orientation = Quaternion()
        primitive_pose.orientation.w = 1.0
        primitive_pose.position.x = x
        primitive_pose.position.y = y
        primitive_pose.position.z = z

        bounding_volume = BoundingVolume()
        if constraint_type == PathPlanningConstraints.BOX:
            primitive.type = 1
            primitive.dimensions = [x_diff, y_diff, z_diff]
        elif constraint_type == PathPlanningConstraints.SPHERE:
            primitive.type = 2
            primitive.dimensions = [max([x_diff, y_diff, z_diff])]
        elif constraint_type == PathPlanningConstraints.CYLINDER:
            primitive.type = 3

            current_pose_array = np.array([current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z])
            target_pose_array = np.array([target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z])
            cyclinder_height= math.dist(current_pose_array, target_pose_array) + tolerance*2
            primitive.dimensions = [cyclinder_height, 0.0125]
            primitive_pose.orientation = orientate_around_z_axis(current_xyz, target_xyz)
            
        bounding_volume.primitives = [primitive]
        bounding_volume.primitive_poses = [primitive_pose]


        position_constraint.constraint_region = bounding_volume

        return position_constraint




    def add_marker(self, primitive, pose_id):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.id = pose_id

        marker.type = marker.SPHERE
        marker.action = marker.ADD
        
        marker.pose = pose_goal
        t = rospy.Duration()
        marker.lifetime = t
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        #take this out later
        # while self.marker_publisher.get_num_connections() == 0:
        #   time.sleep(0.1)
        self.marker_publisher.publish(marker)


def main():
    rospy.init_node('moveit_server', anonymous=True)

    move_group_name = rospy.get_param('~move_group_name', "manipulator")
    platform_controller = MoveItController(f"/{move_group_name}_server", move_group_name)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
