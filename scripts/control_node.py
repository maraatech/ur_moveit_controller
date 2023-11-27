#!/usr/bin/env python3
import rospy

import tf
import tf2_ros
import math

from cares_msgs.msg import PlatformGoalAction, PlatformGoalGoal, PlatformGoalFeedback, PlatformGoalResult
from cares_msgs.msg import PathPlanningConstraints
from geometry_msgs.msg import Pose, PoseStamped 

from move_group_python_interface import MoveGroupPythonInterface

from os.path import expanduser

import actionlib

from actionlib_msgs.msg import GoalStatusArray

import constraint_generation

class MoveItController():

    def __init__(self, name, move_group_name):
        self._action_name = name
        self.arm_interface = MoveGroupPythonInterface(move_group_name)
        
        self.pos = 0
        
        self.control_server = actionlib.SimpleActionServer(name=self._action_name, ActionSpec=PlatformGoalAction, execute_cb=self.execute_cb,auto_start=False)

        # Set default planning and retry conditions
        self.default_max_planning_retries = rospy.get_param("~max_planning_retries", 0)
        self.default_max_planning_time = rospy.get_param("~max_planning_time", 0)
        self.default_retry_multiplier = rospy.get_param("~planning_time_retry_multiplier", 0)
        self.default_planning_time = rospy.get_param("~default_planning_time", 1.0)
        self.final_planning_time = rospy.get_param("~final_planning_time", 5.0)
        
        # Set default path planning constraints for both volume and orientation
        self.default_orientation_constraint = rospy.get_param("~default_orientation_constraint", PathPlanningConstraints.BETWEEN_ORIENT)
        self.default_volume_constraint = rospy.get_param("~default_volume_constraint", PathPlanningConstraints.BOX)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))#tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # create messages that are used to publish feedback/result
        self.feedback = PlatformGoalFeedback()
        self.result   = PlatformGoalResult()

        self.control_server.start()
    
    def execute_cb(self, goal):
        status = 0
        command = goal.command
        self.current_goal = goal

        if command == PlatformGoalGoal.MOVE:
            self.set_arm_planning_settings(goal.target_pose, goal.path_constraints, goal.link_id.data)
            
            used_planning_time = self.find_plan_and_move_arm(goal.target_pose)
            if not used_planning_time:
                self.control_server.set_aborted()

            self.wait_for_arm_server(used_planning_time)
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

        self.marker_publisher.publish(marker)
    
    def find_plan_and_move_arm(self, target_pose):
        retries = 0
        plan = self.arm_interface.go_to_pose_goal_cont(target_pose)
        planning_time = self.planning_time

        while not plan and retries < self.max_retries:
            retries += 1
            planning_time = self.planning_time*self.replanning_multiplier**(retries)
            
            if self.max_planning_time:
                planning_time = min(planning_time, self.max_planning_time)
            
            self.arm_interface.move_group.set_planning_time(planning_time)
            
            plan = self.arm_interface.go_to_pose_goal_cont(target_pose)

            print(f"Planning Time: {planning_time}, Retry Attempts: {retries}")

        if not plan:
            self.arm_interface.move_group.set_planning_time(self.final_planning_time)
            print(f"Planning Time: {self.final_planning_time}, Retry Attempts: {retries}")
            plan = self.arm_interface.go_to_pose_goal_cont(target_pose)
            if not plan:
                self.arm_interface.stop_moving()
                return 0.0
        
        return planning_time

    def wait_for_arm_server(self, planning_time):
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
                return True
            elif status != current_status.ACTIVE:
                if planning_time != self.planning_time*self.replanning_multiplier**(self.max_retries+1):
                    self.execute_cb(self.current_goal)
                    return
                self.control_server.set_aborted()
                return False
        
            # Publish feedback
            self.feedback.status = status
            self.control_server.publish_feedback(self.feedback)
            r.sleep()

    def get_current_pose(self, target_frame_id, link_id):
        current_transform = self.tf_buffer.lookup_transform(target_frame_id , link_id, rospy.Time(0))
        current_pose = PoseStamped(header=current_transform.header)
        current_pose.pose.position = current_transform.transform.translation
        current_pose.pose.orientation = current_transform.transform.rotation

        return current_pose

    def set_arm_planning_settings(self, target_pose, path_planning_constraints, link_id):
        # Set planning time, if no planning time in request use default
        req_planning_time = path_planning_constraints.allowed_planning_time
        self.planning_time = req_planning_time if req_planning_time else self.default_planning_time
        
        self.arm_interface.move_group.set_planning_time(self.planning_time)
        
        # Generate and set path constraints, clear previous constraints

        constraints = self.calculate_planning_constraints(self.get_current_pose(target_pose.header.frame_id, link_id), 
                                                          target_pose, link_id, path_planning_constraints)

        self.arm_interface.move_group.clear_path_constraints()
        self.arm_interface.move_group.clear_pose_targets()
        self.arm_interface.move_group.set_path_constraints(constraints)
        print(f"Setting volume constraints: {constraints}")

        # Set end effector/target link id        
        self.arm_interface.set_ee_link(link_id)

        self.max_retries = self.default_max_planning_retries if not path_planning_constraints.max_retries else path_planning_constraints.max_retries
        self.max_planning_time = self.default_max_planning_time if not path_planning_constraints.max_planning_time else path_planning_constraints.max_planning_time
        self.replanning_multiplier = self.default_retry_multiplier if not path_planning_constraints.replanning_multiplier else path_planning_constraints.replanning_multiplier

    def calculate_planning_constraints(self, current_pose, target_pose, link_id, path_planning_constraints):
        constraints = path_planning_constraints.path_constraints

        orientation_constraint_type = path_planning_constraints.orientation_constraint_type if path_planning_constraints.orientation_constraint_type else self.default_orientation_constraint
        orientation_constraints = constraint_generation.generate_orientation_constraints(orientation_constraint_type,
                                                                                        current_pose, target_pose, 
                                                                                        link_id)
        
        vol_constraint_type = path_planning_constraints.volume_constraint_type if path_planning_constraints.volume_constraint_type else self.default_volume_constraint
        position_constraint = constraint_generation.generate_position_constraint(vol_constraint_type,
                                                                                current_pose, target_pose, link_id)

        print(orientation_constraints)

        constraints.orientation_constraints.extend(orientation_constraints)
        constraints.position_constraints.append(position_constraint)

        print(constraints)
        return constraints


    def validate_target_reached(self):
        #NOTE Make sure to add check if arm has physically moved
        pass

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
