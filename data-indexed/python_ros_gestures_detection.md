---
id: python_ros_gestures
title: Human gesture detection
hide_table_of_contents: true
---


# Human gesture detection

import Icon from "@material-ui/core/Icon";
import Markdown from 'markdown-to-jsx';

:::info Overview
<Icon>signal_cellular_alt</Icon> <Markdown>&nbsp;**Level:**&nbsp; *Intermediate*</Markdown>
<br/> <Icon> track_changes </Icon> <Markdown>&nbsp;**Goal:**&nbsp; *learn how to detect Human gesture expressions with QTrobot Nuitrack interface*</Markdown>
<br/> <Icon> task_alt </Icon> <Markdown>&nbsp;**Requirements:**</Markdown>

  - &nbsp;&nbsp;[Quick start with coding on QTrobot](/docs/intro_code)
  - &nbsp;&nbsp;[Create a ROS python project](/docs/tutorials/python/python_ros_project)
  - &nbsp;&nbsp;[QTrobot interfaces using ROS Topics](/docs/tutorials/python/python_ros_publish)
  - &nbsp;&nbsp;[QTrobot interfaces using ROS Subscribers](/docs/tutorials/python/python_ros_subscribe)
:::


In this tutorial we will learn about how to read human gestures with [QTrobot Nuitrack interface](/docs/api_ros#human-3d-tracking-interface) and react to them. 

## Create a python project 
First we create a python project for our tutorial. let's call it `tutorial_qt_subscribe` and add the required python file: 

```
cd ~/catkin_ws/src
catkin_create_pkg tutorial_qt_gestures rospy roscpp -D "Reading human gestures movement"
cd tutorial_qt_gestures/src
touch tutorial_qt_gestures_node.py
chmod +x tutorial_qt_gestures_node.py
```

## Code
Open the `tutorial_qt_gestures_node.py` file and add the following code:

```python
#!/usr/bin/env python
import sys
import rospy
from qt_gesture_controller.srv import gesture_play
from qt_nuitrack_app.msg import Gestures

def gesture_callback(msg):
    if msg.gestures[0].name == "SWIPE RIGHT":
        gesturePlay("QT/swipe_right",0)
    elif msg.gestures[0].name == "SWIPE LEFT":
        gesturePlay("QT/swipe_left",0)

if __name__ == '__main__':
    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
    # define ros subscriber
    rospy.Subscriber('/qt_nuitrack_app/gestures', Gestures, gesture_callback)
   
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")

```

## Explanation

First we imported `Gestures` message type from `qt_nuitrack_app.msg` message library. This message type is used in communication with `/qt_nuitrack_app/gestures`. 

:::tip Tip

How do we know which messages an interface uses? well, There is a useful command in ROS which tells you that: 
```
rostopic info /qt_nuitrack_app/gestures
Type: qt_nuitrack_app/Gestures
...
```
:::

Then we created a subscriber for`/qt_nuitrack_app/gestures` with callback function `gesture_callback`.
In the callback we react accordingly to what is detected. If we detect gesture "SWIPE RIGHT" we also play a gesture "swipe_right". That will make QTrobot repeat your gestures.

```python
def gesture_callback(msg):
    if msg.gestures[0].name == "SWIPE RIGHT":
        gesturePlay("QT/swipe_right",0)
    elif msg.gestures[0].name == "SWIPE LEFT":
        gesturePlay("QT/swipe_left",0)
```

Stand in front of QTrobot and do one of the gestures "SWIPE LEFT" or "SWIPE RIGHT".
When QTrobot detects the gesture, it will repeat the same pre-recorded gesture.
