---
id: python_ros_subscribe
title: QTrobot interfaces using ROS Subscribe
hide_table_of_contents: true
---

# QTrobot interfaces using ROS Subscribe


import Icon from "@material-ui/core/Icon";
import Markdown from 'markdown-to-jsx';

:::info Overview
<Icon>signal_cellular_alt</Icon> <Markdown>&nbsp;**Level:**&nbsp; *Basic*</Markdown>
<br/> <Icon> track_changes </Icon> <Markdown>&nbsp;**Goal:**&nbsp; *learn how to read QTrobot interfaces in python via ROS Subscribers*</Markdown>
<br/> <Icon> task_alt </Icon> <Markdown>&nbsp;**Requirements:**</Markdown>

  - &nbsp;&nbsp;[Quick start with coding on QTrobot](/docs/intro_code)
  - &nbsp;&nbsp;[Create a ROS python project](/docs/tutorials/python/python_ros_project)
:::


If you have followed our previous tutorials, you should know how to start coding on QTrobot with python and got the basic knowledge of ROS framework. In this tutorial we will learn about how to read some of [QTrobot interfaces](/docs/api_ros#list-of-available-interfaces) using ROS Subscribe. 

## Create a python project 
First we create a python project for our tutorial. let's call it `tutorial_qt_subscribe` and add the required python file: 

```
cd ~/catkin_ws/src
catkin_create_pkg tutorial_qt_subscribe rospy roscpp -D "Read QTrobot data with ROS Subscribe"
cd tutorial_qt_subscribe/src
touch tutorial_qt_subscribe_node.py
chmod +x tutorial_qt_subscribe_node.py
```

## QTrobot nuitrack interface - Gestures
Open the `tutorial_qt_subscribe_node.py` file and add the following code:

```python
#!/usr/bin/env python
import sys
import rospy
from qt_nuitrack_app.msg import Gestures

def gesture_callback(msg):
    rospy.loginfo(msg)

if __name__ == '__main__':
    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    # define ros subscriber
    rospy.Subscriber('/qt_nuitrack_app/gestures', Gestures, gesture_callback)
   
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")

```

First we imported `Gestures` message type from `qt_nuitrack_app.msg` message library. This message type is used in communication with `/qt_nuitrack_app/gestures`. 

:::tip Tip
How do we know which messages an interface uses? well, There is a useful command in ROS which tells you that: 
```
rostopic info /qt_nuitrack_app/gestures
Type: qt_nuitrack_app/Gestures
...
```
:::

Then we created a subscriber for `/qt_nuitrack_app/gestures` with callback function `gesture_callback` to print any data that comes from that topic.
Stand in front of QTrobot and do one of this gestures ("SWIPE UP", "SWIPE DOWN", "SWIPE LEFT", "SWIPE RIGHT").
When QTrobot detects you, the script should print something like this:

```bash
[INFO] [1629900965.371938]: gestures:
  -
    id: 2
    name: "SWIPE UP"
```
