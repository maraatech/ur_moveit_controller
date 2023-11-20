#!/usr/bin/env python3
import rospy
import roslib
import tf
import tf2_ros
import math

from cv_bridge import CvBridge

from cares_msgs.msg import PlatformGoalAction, PlatformGoalGoal, PlatformGoalFeedback, PlatformGoalResult
from cares_msgs.msg import PathPlanningConstraints
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion, PoseStamped
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

import yaml

import cares_lib_ros.utils as utils
from cares_lib_ros.action_client import ActionClient
from actionlib_msgs.msg import GoalStatusArray

from scipy.spatial.transform import Rotation   


def calculate_vector_angle(p1, p2):
  p1_array = np.array([p1.x, p1.y, p1.z])
  p2_array = np.array([p2.x, p2.y, p2.z])

  rotvec = np.cross(p1_array, p2_array)
  rotvec_normalized = rotvec/math.sqrt(rotvec[0]**2+rotvec[1]**2+rotvec[2]**2)

  return rotvec_normalized 

def convert_euler_to_rotvec(roll, pitch, yaw):
    x = math.cos(yaw)*math.cos(pitch)
    y = math.sin(yaw)*math.cos(pitch)
    z = math.sin(pitch)

    x2 = -math.cos(yaw)*math.sin(pitch)*math.sin(roll)-math.sin(yaw)*math.cos(roll)
    y2 = -math.sin(yaw)*math.sin(pitch)*math.sin(roll)+math.cos(yaw)*math.cos(roll)
    z2 =  math.cos(pitch)*math.sin(roll)

    return np.asarray([x, y, z]), np.asarray([x2, y2, z2])

class MoveItController():

    def __init__(self, name, move_group_name):
        self._action_name = name
        self.arm_interface = MoveGroupPythonInterface(move_group_name)
        current_state = self.arm_interface.get_current_pose()
        print(current_state)
        self.pos = 0
        self.control_server = actionlib.SimpleActionServer(name=self._action_name, ActionSpec=PlatformGoalAction, execute_cb=self.execute_cb,auto_start=False)

        self.default_max_planning_retries = rospy.get_param("~max_planning_retries", 0)
        self.default_max_planning_time = rospy.get_param("~max_planning_time", 0)
        self.default_retry_multiplier = rospy.get_param("~planning_time_retry_multiplier", 0)
        self.default_planning_time = rospy.get_param("~default_planning_time", 1.0)
        self.final_planning_time = rospy.get_param("~final_planning_time", 5.0)

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
            else:
                self.arm_interface.move_group.set_planning_time(self.default_planning_time)
            
            # Set path constraints, if empty will set to empty array
            self.arm_interface.move_group.clear_path_constraints()
            self.arm_interface.move_group.clear_pose_targets()

            self.arm_interface.set_ee_link(link_id)
            constraints = goal.path_constraints.path_constraints

            #current_pose_world = self.arm_interface.move_group.get_current_pose()
            # self.tf_listener.waitForTransform(target_pose.header.frame_id, link_id, rospy.Time.now())
            current_transform = self.tf_buffer.lookup_transform(target_pose.header.frame_id, link_id, rospy.Time(0))
            current_pose = PoseStamped(header=current_transform.header)
            current_pose.pose.position = current_transform.transform.translation
            current_pose.pose.orientation = current_transform.transform.rotation
            
            # print(f"Current Pose World: {current_pose_world}")
            # current_pose = self.tf_buffer.transform(current_pose_world, target_pose.header.frame_id)
            print(f"Current Pose: {current_pose}")
            print("Goal")
            print(goal)

            print("Link ID")
            print(link_id)
            # if goal.path_constraints.fix_end_effector:
            if goal.path_constraints.volume_constraint == PathPlanningConstraints.CYLINDER:
                orientation_constraint = self.generate_orientation_constraint(target_pose, link_id, 0.01)
            
            tolerance = 0.075
            if goal.path_constraints.volume_constraint != PathPlanningConstraints.CYLINDER:
                orientation_constraint = self.generate_orientation_constraint_between(current_pose, target_pose, link_id)
                tolerance = 0.025
           
            constraints.orientation_constraints.extend(orientation_constraint)

            if goal.path_constraints.volume_constraint:
                position_constraint = self.generate_position_constraint(goal.path_constraints.volume_constraint, current_pose, target_pose, link_id, tolerance=tolerance)
                position_constraint.header.stamp = rospy.Time.now()
                if goal.path_constraints.volume_constraint != PathPlanningConstraints.CYLINDER:
                    if goal.path_constraints.fix_end_effector:
                        position_constraint.weight = 0.75
                constraints.position_constraints.append(position_constraint)


            print(f"Setting volume constraints: {constraints}")
            self.arm_interface.move_group.set_path_constraints(constraints)            

            retries = 0
            plan = self.arm_interface.go_to_pose_goal_cont(goal.target_pose)
            planning_time = goal.path_constraints.allowed_planning_time

            max_retries = self.default_max_planning_retries if not goal.path_constraints.max_retries else goal.path_constraints.max_retries
            max_planning_time = self.default_max_planning_time if not goal.path_constraints.max_planning_time else goal.path_constraints.max_planning_time
            replanning_multiplier = self.default_retry_multiplier if not goal.path_constraints.replanning_multiplier else goal.path_constraints.replanning_multiplier

            if not plan:
                while not plan and retries < max_retries:
                    if goal.path_constraints.allowed_planning_time:
                        planning_time = goal.path_constraints.allowed_planning_time*replanning_multiplier**(retries+1)
                    else:
                        planning_time = self.default_planning_time*replanning_multiplier**(retries+1)
                    
                    if goal.path_constraints.max_planning_time:
                        planning_time = min(planning_time, goal.path_constraints.max_planning_time)
                    elif max_planning_time:
                        planning_time = min(planning_time, max_planning_time)
                    
                    self.arm_interface.move_group.set_planning_time(planning_time)
                    
                    plan = self.arm_interface.go_to_pose_goal_cont(goal.target_pose)
                    retries += 1
                    print(f"Planning Time: {planning_time}, Retry Attempts: {retries}")

                if not plan:
                    self.arm_interface.move_group.set_planning_time(self.final_planning_time)
                    print(f"Planning Time: {self.final_planning_time}, Retry Attempts: {retries}")
                    plan = self.arm_interface.go_to_pose_goal_cont(goal.target_pose)
                    if not plan:
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
                    if planning_time != goal.path_constraints.allowed_planning_time*replanning_multiplier**(retries+1):
                        self.execute_cb(goal)
                        return
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

    def generate_orientation_constraint_between(self, current_pose, target_pose, link_id, tolerance=(45)):
        self.pos += 1
        def get_rotvec(orientation, orientation2, axes="yzx", method=["s", "s"]):  

            keys = ["xyz", "xzy", "zxy", "zyx", "yxz", "yzx"]
            map_axes = {'method': {}}

            for axe in keys:
                for axe2 in keys:
                    euler = tf.transformations.euler_from_quaternion(np.array([orientation.x, orientation.y, orientation.z, orientation.w]), axes=method[0]+axe2)
                    euler2 = tf.transformations.euler_from_quaternion(np.array([orientation2.x, orientation2.y, orientation2.z, orientation2.w]), axes=method[0]+axe2)
                    
                    r = Rotation.from_euler(axe, np.asarray(euler)).as_matrix()
                    r2 = Rotation.from_euler(axe, np.asarray(euler2)).as_matrix()

                    rvec, _ = cv2.Rodrigues(np.dot(r, np.transpose(r2)))
                    rvec_reversed, _ = cv2.Rodrigues(np.dot(r2, np.transpose(r)))

                    rvec = np.asarray(rvec)
                    rvec = rvec.reshape((1, 3))
                    rotation = Rotation.from_rotvec(rvec)

                    map_axes['method'][axe2+"_"+axe] = {}
                    map_axes['method'][axe2+"_"+axe]["rvec"] = rvec.tolist()
                    map_axes['method'][axe2+"_"+axe]["reversed"] = rvec_reversed.tolist()
                    map_axes['method'][axe2+"_"+axe]["Euler A"] = list(euler)
                    map_axes['method'][axe2+"_"+axe]["Euler B"] = list(euler2)
            

            return np.asarray([euler[0], euler[1], euler[2]]), map_axes


        rotvec1, dict_1 = get_rotvec(current_pose.pose.orientation, target_pose.pose.orientation, axes="xyz", method=["r", "r"])


        rotvec = dict_1["method"]["yzx_yxz"]["rvec"][0]
        # print(f"rotvec: {rotvec}")
        # save_path = f"/home/dsmi923/{self.pos}.yaml"
        # with open(save_path, "w") as file:
        #     yaml.dump(dict_1, file)

        # differences = {}
        # sums  = {}
        # for key, value in dict_1.items():
        #     differences[key] = {}
        #     sums[key] = {}
        #     for axes, result in value.items():
        #         differences[key][axes] = abs(result - dict_2[key][axes])
        #         sums[key][axes] = abs(result + dict_2[key][axes])

        # print(f"differences: {differences}")
        # print(f"sums: {sums}")
        # angle_diff[0] = abs(-rotvec1[0] - rotvec2[0])
        # angle_diff[1] = abs(rotvec1[1] - rotvec2[1]) 
        # angle_diff[2] = abs(rotvec1[2] - rotvec2[2])

    
        orientation_constraint_1 = OrientationConstraint()
        orientation_constraint_1.header = current_pose.header
        orientation_constraint_1.link_name = link_id
        orientation_constraint_1.orientation = current_pose.pose.orientation
        orientation_constraint_1.weight = 1.0
        orientation_constraint_1.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation_constraint_1.absolute_x_axis_tolerance = abs(rotvec[2]) + math.radians(tolerance)
        orientation_constraint_1.absolute_y_axis_tolerance = abs(rotvec[1]) + math.radians(tolerance)
        orientation_constraint_1.absolute_z_axis_tolerance = abs(rotvec[0]) + math.radians(tolerance)

        orientation_constraint_2 = OrientationConstraint()
        orientation_constraint_2.header = target_pose.header
        orientation_constraint_2.link_name = link_id
        orientation_constraint_2.orientation = target_pose.pose.orientation
        orientation_constraint_2.parameterization = OrientationConstraint.ROTATION_VECTOR
        orientation_constraint_2.weight = 1.0
        orientation_constraint_2.absolute_x_axis_tolerance = abs(rotvec[2]) + math.radians(tolerance)
        orientation_constraint_2.absolute_y_axis_tolerance = abs(rotvec[1]) + math.radians(tolerance)
        orientation_constraint_2.absolute_z_axis_tolerance = abs(rotvec[0]) + math.radians(tolerance)

        return [orientation_constraint_1, orientation_constraint_2]

    def generate_position_constraint(self, constraint_type, current_pose, target_pose, link_id, tolerance=0.025):
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
