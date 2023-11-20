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

## Running
Running the moveit node is as simple as running below.
Note: this is intended to be run as part of a "platform"\_bringup package.

```
rosrun ur_moveit_controller control_node.py
```
