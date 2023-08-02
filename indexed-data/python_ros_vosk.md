---
id: python_ros_vosk
title: Offline speech recognition
hide_table_of_contents: true
---

# QTrobot Offline speech recognition

import Icon from "@material-ui/core/Icon";
import Markdown from 'markdown-to-jsx';

:::info Overview
<Icon>signal_cellular_alt</Icon> <Markdown>&nbsp;**Level:**&nbsp; *Advanced*</Markdown>
<br/> <Icon> track_changes </Icon> <Markdown>&nbsp;**Goal:**&nbsp; *learn how to use QTrobot Offline speech recognition*</Markdown>
<br/> <Icon> task_alt </Icon> <Markdown>&nbsp;**Requirements:**</Markdown>

  - &nbsp;&nbsp;[Quick start with coding on QTrobot](/docs/intro_code)
  - &nbsp;&nbsp;[Create a ROS python project](/docs/tutorials/python/python_ros_project)
  - &nbsp;&nbsp;[QTrobot interfaces using ROS Services](/docs/tutorials/python/python_ros_services)
:::


In this tutorial we will learn about how to use `QTrobot Offline speech recognition`. 

## Create a python project 
First we create a python project for our tutorial. let's call it `tutorial_qt_vosk` and add the required python file: 

```
cd ~/catkin_ws/src
catkin_create_pkg tutorial_qt_vosk rospy roscpp -D "Command QTrobot via ROS Services"
cd tutorial_qt_vosk/src
touch tutorial_qt_vosk_node.py
chmod +x tutorial_qt_vosk_node.py
```

## QTrobot speech service 

Following are some standard supported languages:

*  **en_US** (English)
*  **fr_FR** (French)
*  **de_DE** (German)
*  **es_ES** (Spanish)
*  **it_IT** (Italian)
*  **pt_PT** (Portuguese)
*  **nl_NL** (Dutch)

Open the `tutorial_qt_service_node.py` file and add the following code:

```python
#!/usr/bin/env python
import sys
import rospy
from qt_robot_interface.srv import *
from qt_vosk_app.srv import *

if __name__ == '__main__':

    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    # define a ros service
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/speech/recognize')

    try:
        # call a ros service with text message
        speechSay("Say something after the beep.")
        speechSay('#CAR HORN#')
        resp = recognize("en_US", ['blue', 'green', 'red'], 10)
        rospy.loginfo("I got: %s", resp.transcript)
        speechSay("You said %s " % resp.transcript)
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")

```

## Tips

1. When you run the code or when you call the service from command line always wait a least 1 second before saying something, so that microphone has time to open.
2. It might not work on the first call when you switch the language, second one will be ok.
3. Using options for better detection of words. If you want to get exactly 'yes' or 'no', you just need to call the service with this options ['yes','no']. Even if you say some really long sentence, it will detect just 'yes' or 'no'.



