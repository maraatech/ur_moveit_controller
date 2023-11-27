# ur_moveit_controller
Basic MoveIt interface based control node for a universal robotic arm - interface via cares_msgs/PlatformGoal for generic platform control.
This package is setup as the control node for a simple platform with a UR robot arm with any sensor on a simple base.
The full platform description will sit under the base platform package - e.g. mini_rig_ros.

## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.
See deployment for notes on how to deploy the project on a live system.

### Prerequisites
Base package dependencies

```
1) Ubuntu 20.04 - ROS Noetic - requires python 3 support

2) Pull master version of cares_msgs
   a) cd ~/catkin_ws/src
   b) git clone https://github.com/UoA-CARES/cares_msgs.git

3) Pull master version of cares_lib_ros
   a) cd ~/catkin_ws/src
   b) git clone https://github.com/UoA-CARES/cares_lib_ros.git

4) Install MoveIt
	a) sudo apt-get install ros-noetic-moveit

```

### Installing
A step by step series of examples that tell you how to get a development env running


Simple moveit control node for a UR robot arm operating stand-alone. 

```
git clone https://github.com/maraatech/ur_moveit_controller.git
cd ~/catkin_workspace && catkin_make
```
install python3 requirements

```
cd ~/catkin_ws/src/ur_moveit_controller
pip3 install -r requirements.txt
```

## Running
Running the moveit node is as simple as running below.
Note: this is intended to be run as part of a "platform"\_bringup package.

```
rosrun ur_moveit_controller control_node.py
```

## Controlling Platform
Control is set up and controlled as an action server. Please refer to cares_msgs/Action/PlatformGoal for msg format,

Note: This package is not required to be edited to control your platform. All control requests should be formatted and handled via your own scripts/packages. Default path planning constraints can be set in your "platform"\_bringup package launch file

### Example
Definitions:

link_id - End Effector ID to move to target_pose

planning_link - Reference frame for target pose

Example for platform goal:
```
#Setup imports
import tf 
import rospy
from cares_msgs.msg import PlatformGoalAction, PlatformGoalGoal, PlatformGoalFeedback, PathPlanningConstraints
from geometry_msgs.msg import Quaternion, PoseStamped, Point
from cares_lib_ros.action_client import ActionClient

....

#Setup Action Client
client = ActionClient(control_server_name, PlatformGoalAction, PlatformGoalFeedback)

....

#Setup PlatformGoal
goal = PlatformGoalGoal()
goal.link_id = link_id
goal.command = PlatformGoalGoal.MOVE

#Set target pose to desried xyz, and rpy
target_pose = PoseStamped()
target_pose.header.stamp = rospy.Time.now()
target_pose.header.frame_id = planning_link
target_pose.pose.position = Point(x=target_x, y=target_y, z=target_z)
target_pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(r, p, y))

# Set up constraints for fixed orientation and cyclindrical volumes with 1 second initial 
planning time and a max of 3 planning retries, 
doubling planning time every retry with a max planning time of 5 seconds

constraints = PathPlanningConstraints()
constraints.orientation_constraint_type = PathPlanningConstraints.FIXED_ORIENT
constraints.volume_constraint_type = PathPlanningConstraints.CYLINDER
constraints.allowed_planning_time = 1.0
constraints.replanning_multiplier = 2.0
constraints.max_retries = 3
constraints.max_planning_time = 5.0

goal.path_constraints = constraints
goal.target_pose = target_pose

....

# Send Goal
client.send_goal(goal)

....

# Monitor/Wait for Goal status
while not client.is_idle() and not rospy.is_shutdown():
   rospy.sleep(0.5)

result = client.status.result

```

## Path Planning Constraints
Types of constraints available:
1) Orientation Constraints:

   a) Between Orientation - constrains valid orientation for given link id to the region between the vector angles for current and target pose

   b) Fixed - Orientation - constrains valid oreintation to keep given link_id at a fixed orientation

   c) No Orientation - No orientation constraints
2) Volume Constraints:

   a) Box - constraints valid region to a box volume between current and target positions
   
   b) Cylinder -  constraints valid region to a cyclindrical volume between current and target positions

   c) Shere - constraints valid region to a spherical volume between current and target positions
   d) No Volume - No volume constraints
3) Time constraints:
* Allowed Planning Time - sets the time for a single plan attempt
* Max Retries - maximum number of planning attempts before giving up
* Replanning Multiplier - sets the multiplier for next attempts planning time if path planning fails to generate a valid plan
* Max Planning Time - sets the maximum allowable planning time
* Final Planning Time - sets the time for one final replan attempt if maximum number of retries is reached