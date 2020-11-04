# ur_moveit_controller

Simple moveit control node for a UR robot arm operating stand-alone. 

# Dependencies
```
cd ~/catkin_workspace/src
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git
git clone https://github.com/maraatech/ur_description.git
cd ..
catkin_make
source devel/setup.bash
```

## Installation
```
roscd ur5_moveit_config/config/
sed -i 's/name: \"\"/name: \"scaled_pos_joint_traj_controller\"/g' controllers.yaml
cd ~/catkin_workspace/src
git clone https://github.com/maraatech/ur_moveit_controller.git
cd ~/catkin_workspace && catkin_make
```
## Running
```
roslaunch ur_uoa_description startup.launch # see note
roslaunch ur5_moveit_config move_group.launch
rosrun ur_moveit_controller control_node.py 
```
### *note
may vary depending on the arm controller used.
