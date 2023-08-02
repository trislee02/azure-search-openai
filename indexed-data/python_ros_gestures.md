---
id: python_ros_record
title: QTrobot recording new gesture
hide_table_of_contents: true
---


import Icon from "@material-ui/core/Icon";
import Markdown from 'markdown-to-jsx';

:::info Overview
<Icon>signal_cellular_alt</Icon> <Markdown>&nbsp;**Level:**&nbsp; *Intermediate*</Markdown>
<br/> <Icon> track_changes </Icon> <Markdown>&nbsp;**Goal:**&nbsp; *learn how to record and play custom gesture with QTrobot Gesture interface*</Markdown>
<br/> <Icon> task_alt </Icon> <Markdown>&nbsp;**Requirements:**</Markdown>

  - &nbsp;&nbsp;[Quick start with coding on QTrobot](/docs/intro_code)
  - &nbsp;&nbsp;[Create a ROS python project](/docs/tutorials/python/python_ros_project)
  - &nbsp;&nbsp;[QTrobot interfaces using ROS Services](/docs/tutorials/python/python_ros_services)
:::

In this tutorial you will learn how to record and play custom gesture with [QTrobot Gesture interface](/docs/api_ros#gesture-interface) using python.

## Create a python project 
First we create a python project for our tutorial. let's call it `tutorial_qt_record` and add the required python file: 

```
cd ~/catkin_ws/src
catkin_create_pkg tutorial_qt_record rospy roscpp -D "Record new gestures"
cd tutorial_qt_record/src
touch tutorial_qt_record_node.py
chmod +x tutorial_qt_record_node.py
```

## Code

Lets see how we can record and play a new gesture. 

Open the `tutorial_qt_record_node.py` file and add the following code:

```python
#!/usr/bin/env python
import sys
import rospy
from qt_robot_interface.srv import *
from qt_gesture_controller.srv import *
from qt_motors_controller.srv import *

if __name__ == '__main__':
    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    gestureRecord = rospy.ServiceProxy('/qt_robot/gesture/record', gesture_record)
    gestureSave = rospy.ServiceProxy('/qt_robot/gesture/save', gesture_save)
    setControlMode = rospy.ServiceProxy('/qt_robot/motors/setControlMode', set_control_mode)
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)


    try:
        name = "my_gesture"
        parts = ["left_arm"]
        input('Press enter to START recording ...\n')
        speechSay('Press enter to START recording.')
        res = gestureRecord(parts, True, 0, 0)
        if not res.status:
            rospy.logfatal("Could not start recording gesture '%s' using '%s'." % (name, parts))
        speechSay('When you want to STOP recording, just press enter again')
        input('Press enter to STOP recording ...\n')
        res = gestureSave(name, "")
        if not res.status:
            rospy.logfatal("Could not save gesture '%s'." % name)
        else:
            rospy.loginfo("Gesture '%s' was recorded." % name)
            speechSay("Your gesture was recorded." % name)
        res = setControlMode(parts, 1)
        if not res.status:
            rospy.logfatal("Could not set control mode of '%s'." % parts)
        else:
            speechSay("Let's see what did you record.")
            gesturePlay(name, 0)

    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")
```



## Explanation

Let's dissect the code. First we import all from `qt_robot_interface.srv`, because we need `speech_say`. We also import `qt_gesture_controller.srv` and `qt_motors_controller.srv` for all gesture and motor services.

We define all services that we need to use. We will need services for recording, saving and playing gestures. Also we will use speech service and control mode to set the mode of QTrobot motors.

```python
speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
gestureRecord = rospy.ServiceProxy('/qt_robot/gesture/record', gesture_record)
gestureSave = rospy.ServiceProxy('/qt_robot/gesture/save', gesture_save)
setControlMode = rospy.ServiceProxy('/qt_robot/motors/setControlMode', set_control_mode)
gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
```

Next we name the gesture and in we select which body part we want to record. You can write `["left_arm","right_arm","head"]` to record with entire QTrobot. In this tutorial we will use just `left_arm`.

```python
name = "my_gesture"
parts = ["left_arm"]
```

Next we use `speechSay` service to let the user decide when to start the recording. After user pressed enter, With `gestureRecord` call we start recording the gesture with the `parts` that we selected. If there is any error on recording we print out and error message.

```python
input('Press enter to START recording ...\n')
speechSay('Press enter to START recording.')
res = gestureRecord(parts, True, 0, 0)
if not res.status:
    rospy.logfatal("Could not start recording gesture '%s' using '%s'." % (name, parts))
```

We use the same `speech_say` service for user interaction and when user decides to stop the recording we stop it using `gestureSave`, which saves the gesture with selected name on *QTRP* in folder `~/robot/data/gestures/`.

```python 
speechSay('When you want to STOP recording, just press enter again')
input('Press enter to STOP recording ...\n')
res = gestureSave(name, "")
if not res.status:
    rospy.logfatal("Could not save gesture '%s'." % name)
else:
    rospy.loginfo("Gesture '%s' was recorded." % name)
    speechSay("Your gesture was recorded." % name)
```

At the end we enable back the motors used for recording with `setControlMode` and if everything is ok we play the gesture that we recorded with `gesturePlay`.

```python
res = setControlMode(parts, 1)
if not res.status:
    rospy.logfatal("Could not set control mode of '%s'." % parts)
else:
    speechSay("Let's see what did you record.")
    gesturePlay(name, 0)
```