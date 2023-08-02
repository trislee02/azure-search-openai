---
id: python_ros_moveit
title: Controlling QTrobot arms using MoveIt
hide_table_of_contents: true
---


import Icon from "@material-ui/core/Icon";
import Markdown from 'markdown-to-jsx';

:::info Overview
<Icon>signal_cellular_alt</Icon> <Markdown>&nbsp;**Level:**&nbsp; *Advanced*</Markdown>
<br/> <Icon> track_changes </Icon> <Markdown>&nbsp;**Goal:**&nbsp; *learn how to control QTrobot arms using MoveIt*</Markdown>
<br/> <Icon> task_alt </Icon> <Markdown>&nbsp;**Requirements:**</Markdown>

  - &nbsp;&nbsp;[Quick start with coding on QTrobot](/docs/intro_code)
  - &nbsp;&nbsp;[Create a ROS python project](/docs/tutorials/python/python_ros_project)
:::

This advanced tutorial demonstrate how to use ROS MoveIT to control QTrobot arms. The examples draw some shapes (i,e. rectangle and spirals) on the XY plane in robot frame.

<center>
<iframe width="800" height="480" src="https://www.youtube.com/embed/JVJMZNkcl6M" frameborder="0" allow="accelerometer; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</center>


<!-- ![QTrobot MoveIt](../../../static/img/qt_moveit.png) -->

## Preparation and requirements
Before running the example, please ensure that following setups of your QTrobot and the machine which you are running the example.

### QTrobot setup
By default QTrobot motors interface runs in *'normal'* mode. in normal mode, the motors control loop and joints state publisher run in low frequency (2-5Hz). More importantly the joint position values are in *degree*. To be able to use MoveIt with QTrobot, you need to configure it to run in *'advanced'* mode:
- joints position value is in *radian*
- motors main controller loop and joints state publisher runs in 30hz.
- required interfaces and controller such as `robot_state_publisher` and `JointTrajectoryController` are available

**Change motor launcher autostart script to run in advanced mode:**

```
nano ~/robot/autostart/start_qt_motor.sh
```

and change the corresponding line to look like the following and save and exit:

```
roslaunch qt_motor qt_motor_advanced.launch
```

Reboot the robot to run the advance motor interface.


**Check the advanced mode setup:**
After rebooting the reboot, you can check (from QTPC, QTRP or your machine) if the motor interface is running in the advanced mode:

joint state publisher frequency:
```
rostopic hz /qt_robot/joints/state
...
average rate: 30.041
	min: 0.029s max: 0.047s std dev: 0.00391s window: 29
```

joints value should be in radian:
```
rostopic echo /qt_robot/joints/state
...
position: [0.015707962851830046, 0.0, -0.6073745663782212, 1.569051024174513, -0.9896016991965904, -0.5689773095185405, -0.3455751785790718, -0.8360127383368947]
```

trajectory controller is running:
```
rostopic type  /qt_robot/left_arm_controller/follow_joint_trajectory/goal
...
control_msgs/FollowJointTrajectoryActionGoal
```

### Your machine setup (QTPC)

After checking and updating the QTrobot setup, you can install the iKfast solver plugin for MoveIt on the machine which you plan to run the example:

Get the latest version of QTrobot open software repository

```
cd ~/robot/code/software
git pull
```

If the folder doesn't exists:

```
cd ~/robot/code/
git clone https://github.com/luxai-qtrobot/software.git
```

build the plugins:
```
cd ~/catkin_ws/src
ln -s ~/software/plugins/qtrobot_ikfast_right_arm_plugin .
ln -s ~/software/plugins/qtrobot_ikfast_right_left_plugin .
cd ~/catkin_ws
catkin_make
```


## Build the motors_moveit

Assuming that you already have the `tutorial` repository (default path `~/robot/code/tutorials`):

```
cd ~/catkin_ws/src
ln -s ~/robot/code/tutorials/examples/motors_moveit ./
cd ~/catkin_ws
catkin_make
```

if you don't have it, then clone it and repeat the above process:

```
git clone https://github.com/luxai-qtrobot/tutorials
```

## How to run the examples
Launch `moveit_qtrobot.launch` to start move_group planner and rviz:

```
roslaunch motors_moveit moveit_qtrobot.launch
```

wait until rviz shows up, then run one of the following demos:

### Drawing rectangle
```
rosrun motors_moveit draw_rectangle.py joint_states:=/qt_robot/joints/state
```

### Drawing spiral
```
rosrun motors_moveit draw_spiral.py joint_states:=/qt_robot/joints/state
```
