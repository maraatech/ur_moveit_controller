import tf
import math
from scipy.spatial.transform import Rotation
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import BoundingVolume
from moveit_msgs.msg import Constraints, PositionConstraint, JointConstraint, OrientationConstraint
from cares_msgs.msg import PathPlanningConstraints
from cares_lib_ros import utils
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion, PoseStamped

import numpy as np
import cv2

def generate_orientation_constraints(constraint_type, current_pose, target_pose, link_id):
    print(f"Requested Orientation Constraint Type: {constraint_type}")
    if constraint_type == PathPlanningConstraints.FIXED_ORIENT:
        fixed_orientation_constraint = generate_fixed_orientation_constraint(target_pose, link_id)
        return [fixed_orientation_constraint]
    elif constraint_type == PathPlanningConstraints.BETWEEN_ORIENT:
        constraint_1, constraint_2 = generate_between_orientation_constraints(current_pose, target_pose, link_id)
        return [constraint_1, constraint_2]
    elif constraint_type == PathPlanningConstraints.NO_ORIENT:
        return []
    else:
        print("Orientation constraint not recognized")

def generate_position_constraint(constraint_type, current_pose, target_pose, link_id, tolerance=0.025, radius=0.0125):
    print(f"Requested Volume Constraint Type: {constraint_type}")
    position_constraint = PositionConstraint()
    position_constraint.header = target_pose.header
    position_constraint.link_name = link_id
    position_constraint.weight = 1.0
    
    primitive, primitive_pose = generate_shape_primitive_and_pose(constraint_type, current_pose, target_pose, tolerance, radius)

    bounding_volume = BoundingVolume()
    bounding_volume.primitives = [primitive] if primitive else []
    bounding_volume.primitive_poses = [primitive_pose] if primitive_pose else []

    position_constraint.constraint_region = bounding_volume

    return position_constraint

def generate_shape_primitive_and_pose(constraint_type, current_pose, target_pose, tolerance, radius):
    '''Function that returns both the shape primitive and respective both which are needed for
       path planning constraints'''
    def orientate_around_z_axis(current_pose, target_pose):
        '''Inner function to orient generated constraint around Z axis in order to have 
           constraint in line with orientation between target and current if needed based on primitive definitions'''
        pose_arr = np.array([current_pose.x, current_pose.y, current_pose.z])
        target_arr = np.array([target_pose.x, target_pose.y, target_pose.z])
        orientation = utils.look_at_pose(pose_arr, target_arr, up=utils.World.up).orientation

        euler = tf.transformations.euler_from_quaternion(utils.quaternion_to_array(orientation))
        # Offset to centre z axis rotation between current and target poses
        r = euler[0]
        p = euler[1] - math.pi/2
        y = euler[2]

        return Quaternion(*tf.transformations.quaternion_from_euler(r, p, y))
    
    primitive_pose = generate_primitive_pose(current_pose, target_pose)

    #Generate primitives based on constraint type
    primitive = SolidPrimitive()
    
    x_diff, y_diff, z_diff  = calculate_axis_differences(current_pose.pose.position, target_pose.pose.position, tolerance)
    
    if constraint_type == PathPlanningConstraints.BOX:
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [x_diff, y_diff, z_diff]
    elif constraint_type == PathPlanningConstraints.SPHERE:
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [max([x_diff, y_diff, z_diff])]
    elif constraint_type == PathPlanningConstraints.CYLINDER:
        cyclinder_height= utils.distance_between_points(current_pose.pose.position, target_pose.pose.position) + tolerance*2
        
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [cyclinder_height, radius]

        #Primitive pose needs to be orientated around z axis for cyclinder due to shape msg default orientation
        primitive_pose.orientation = orientate_around_z_axis(current_pose.pose.position, target_pose.pose.position)
    elif constraint_type == PathPlanningConstraints.NO_VOL:
        return None, None

    return primitive, primitive_pose

def generate_primitive_pose(current_pose, target_pose):
    '''Generates a primitive pose centered between two poses'''
    x_centered, y_centered, z_centered = calculate_centered_xyz(current_pose.pose.position, target_pose.pose.position)
    
    primitive_pose = Pose()
    primitive_pose.orientation = Quaternion()
    primitive_pose.orientation.w = 1.0
    primitive_pose.position.x = x_centered
    primitive_pose.position.y = y_centered
    primitive_pose.position.z = z_centered

    return primitive_pose

def generate_fixed_orientation_constraint(pose, link_id, tolerances=[0.01, 0.01, 0.01]):
    '''Generate a single orientation constraint based on pose orientation'''
    fixed_orientation_constraint = format_orientation_constraint(pose, tolerances, link_id) 

    return fixed_orientation_constraint

def generate_between_orientation_constraints(current_pose, target_pose, link_id, tolerance=(45)):
    '''Generate orientation constraint pair with the ROTVec tolerances set as the 3D rotation vector
       between orientations. This tricks moveit to only allow movement if rotation stays between
       current and target orientations(Plus tolerance)'''
    
    #Using rotating reference frames and yzx axes due to our standard orientation/definition for general joints
    rotvec = get_rotvec(current_pose.pose.orientation, target_pose.pose.orientation, axes=["yzx", "yzx"], method=["r", "r"])
    x_tolerance = abs(rotvec[2]) + math.radians(tolerance)
    y_tolerance = abs(rotvec[1]) + math.radians(tolerance)
    z_tolerance = abs(rotvec[0]) + math.radians(tolerance)
    
    tolerances = [x_tolerance, y_tolerance, z_tolerance]

    orientation_constraint_1 = format_orientation_constraint(current_pose, tolerances, link_id)

    orientation_constraint_2 = format_orientation_constraint(target_pose, tolerances, link_id)

    return orientation_constraint_1, orientation_constraint_2

def calculate_axis_differences(point_1, point_2, tolerance):
    '''Function to return xyz differences between points'''
    x_diff = abs(round(point_1.x, 2) - point_2.x) + tolerance
    y_diff = abs(round(point_1.y, 2) - point_2.y) + tolerance
    z_diff = abs(round(point_1.z, 2) - point_2.z) + tolerance

    return x_diff, y_diff, z_diff

def calculate_centered_xyz(point_1, point_2):
    '''Function to return center/halfway between two points'''
    x_centered = (round(point_1.x, 2) + point_2.x)/2
    y_centered = (round(point_1.y, 2) + point_2.y)/2
    z_centered = (round(point_1.z, 2) + point_2.z)/2

    return x_centered, y_centered, z_centered

def get_rotvec(orientation, orientation2, axes=["xyz", "xyz"], method=["s", "s"]): 
    '''Function to return rotation vector between two orientations
       based on given combination or static/rotating frame and axes order'''
    euler = tf.transformations.euler_from_quaternion(np.array([orientation.x, orientation.y, orientation.z, orientation.w]), axes=method[0]+axes[0])
    euler2 = tf.transformations.euler_from_quaternion(np.array([orientation2.x, orientation2.y, orientation2.z, orientation2.w]), axes=method[1]+axes[1])
    
    r = Rotation.from_euler(axes[0], np.asarray(euler)).as_matrix()
    r2 = Rotation.from_euler(axes[1], np.asarray(euler2)).as_matrix()

    rvec, _ = cv2.Rodrigues(np.dot(r, np.transpose(r2)))
    rvec = np.asarray(rvec)

    return np.asarray([rvec[0][0], rvec[1][0], rvec[2][0]])

def format_orientation_constraint(pose, tolerances, link_id, weight=1.0, parameterization=OrientationConstraint.ROTATION_VECTOR):
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header = pose.header
    orientation_constraint.link_name = link_id
    orientation_constraint.orientation = pose.pose.orientation
    orientation_constraint.weight = 1.0
    orientation_constraint.parameterization = parameterization
    orientation_constraint.absolute_x_axis_tolerance = tolerances[0]
    orientation_constraint.absolute_y_axis_tolerance = tolerances[1]
    orientation_constraint.absolute_z_axis_tolerance = tolerances[2]

    return orientation_constraint