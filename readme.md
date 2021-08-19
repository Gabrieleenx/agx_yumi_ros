## YuMi in Algoryx 

## Maintainer 
* Gabriel Arslan Waltersson

## Table of Contents
* [General](#general)
* [ROScommunication](#ros_communication)
* [Dependencies](#dependencies)
* [Usage](#usage)

## General
This package allows for simulating yumi in Algoryx. It uses the same ROS topics as the ABB ros driver, but does not emulate all of the topics and services.

## ROS_communication

* Output 
```
/yumi/egm/egm_states
/yumi/egm/joint_states
```
* Input  
```
/yumi/egm/joint_group_velocity_controller/command
/yumi/rws/sm_addin/set_sg_command (rosservice)
/yumi/rws/sm_addin/run_sg_routine (rosservice)
```

## Dependencies
This package uses Ros melodic and python3, ROS and catkin has to be reconfigured for python3
* for pythpn3 ros packages 
```
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy
```

* Algoryx
```
https://www.algoryx.se/
```

* abb_robot_driver, used for importing msg types. 
```
https://github.com/ros-industrial/abb_robot_driver
```

## Usage
* compile, (not yet tested with catkin build, but only with catkin_make_isolated)
``` 
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
``` 

* start roscore
``` 
roscore
``` 
* source AGX
``` 
source /opt/Algoryx/AGX-2.30.4.0/setup_env.bash
``` 
* Start yumi Agx simulation
``` 
rosrun agx_ros yumiAGX.py 
``` 

* To test, publish a joint velocity [rad/s]
``` 
rostopic pub /yumi/egm/joint_group_velocity_controller/command std_msgs/Float64MultiArray "data: [0,0,0,0,0,0,-0.1,0,0,0,0,0,0,0.1]"
``` 

* To test grippers, set command

``` 
rosservice call /yumi/rws/sm_addin/set_sg_command "task: 'T_ROB_L'
command: 5
target_position: 20.0" 
``` 
* Run gripper command
``` 
rosservice call /yumi/rws/sm_addin/run_sg_routine "{}" 
``` 
The commands for the gripper are, 5 - move to (target_position only used for this [mm]), 6 - grip in, 7 - grip out 