---
id: python_ros_motors
title: Commanding QTrobot motors
hide_table_of_contents: true
---

# Commanding QTrobot motors


import Icon from "@material-ui/core/Icon";
import Markdown from 'markdown-to-jsx';

:::info Overview
<Icon>signal_cellular_alt</Icon> <Markdown>&nbsp;**Level:**&nbsp; *Intermediate*</Markdown>
<br/> <Icon> track_changes </Icon> <Markdown>&nbsp;**Goal:**&nbsp; *learn how to command and read motors with QTrobot Motor interface*</Markdown>
<br/> <Icon> task_alt </Icon> <Markdown>&nbsp;**Requirements:**</Markdown>

  - &nbsp;&nbsp;[Quick start with coding on QTrobot](/docs/intro_code)
  - &nbsp;&nbsp;[Create a ROS python project](/docs/tutorials/python/python_ros_project)
  - &nbsp;&nbsp;[QTrobot interfaces using ROS Topics](/docs/tutorials/python/python_ros_publish)
:::

In this tutorial you will learn how to command and read motors with [QTrobot Motor interface](/docs/api_ros#motor-interface) using python.

## Create a python project 
First we create a python project for our tutorial. let's call it `tutorial_qt_motors` and add the required python file: 

```
cd ~/catkin_ws/src
catkin_create_pkg tutorial_qt_motors sensor_msgs std_msgs rospy roscpp -D "Command QTrobot motors"
cd tutorial_qt_motors/src
touch tutorial_qt_motors_node.py
chmod +x tutorial_qt_motors_node.py
```

## Code

In this tutorial we will focus on reading and moving just one motor "HeadYaw".
Open the `tutorial_qt_motors_node.py` file and add the following code:

```python
#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

head_yaw_pos = 0
rospy.init_node('my_tutorial_node')
rospy.loginfo("my_tutorial_node started!")

head_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=10)
rospy.sleep(3.0)

def state_callback(msg):
    global head_yaw_pos
    head_yaw_pos = msg.position[msg.name.index("HeadYaw")]

rospy.Subscriber('/qt_robot/joints/state', JointState, state_callback)

if __name__ == '__main__':
    head_yaw_ref = 15.0
    while not rospy.is_shutdown():
        try:
            href = Float64MultiArray()
            href.data = [head_yaw_ref, 0]
            head_pub.publish(href)
            rospy.sleep(4)
            rospy.loginfo("Current position : %.2f" ,head_yaw_pos)
            head_yaw_ref = -15 if head_yaw_ref == 15 else 15
        except KeyboardInterrupt:
            pass
    rospy.loginfo("finsihed!")
```



## Explanation

First we imported `Float64MultiArray` from ROS standard message library. This message is used in `/qt_robot/head_position/command` to command the motors. Next we imported `JointState`, which we will need to read the joint positions. We define one global variable to save latest position of motor and we initialize ROS node. 

We define a ROS publisher for `/qt_robot/head_position/command`, which we will use to command the motors.

```python
head_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=10)
```

Next we define a callback function `state_callback`, which will be reading position of "HeadYaw" motor. With that we also define a ROS subscriber `/qt_robot/joints/state` to read this data.

```python
def state_callback(msg):
    global head_yaw_pos
    head_yaw_pos = msg.position[msg.name.index("HeadYaw")]

rospy.Subscriber('/qt_robot/joints/state', JointState, state_callback)
```

In the main we define starting reference position and new `Float64MultiArray` message, which includes reference position. We publish new position with `head_pub` and after some delay we print the current position and we change the reference position. The "HeadYaw" will move from 15 to -15 on repeat.

```python
if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            href = Float64MultiArray()
            href.data = [head_yaw_ref, 0]
            head_pub.publish(href)
            rospy.sleep(4)
            rospy.loginfo("Current position : %.2f" ,head_yaw_pos)
            head_yaw_ref = -15 if head_yaw_ref == 15 else 15
        except KeyboardInterrupt:
            pass
    rospy.loginfo("finsihed!")
```
