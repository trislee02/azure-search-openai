---
id: python_ros_hands
title: Human hands detection
hide_table_of_contents: true
---

# Human hands detection

import Icon from "@material-ui/core/Icon";
import Markdown from 'markdown-to-jsx';

:::info Overview
<Icon>signal_cellular_alt</Icon> <Markdown>&nbsp;**Level:**&nbsp; *Intermediate*</Markdown>
<br/> <Icon> track_changes </Icon> <Markdown>&nbsp;**Goal:**&nbsp; *learn how to detect human hands status with QTrobot Nuitrack interface*</Markdown>
<br/> <Icon> task_alt </Icon> <Markdown>&nbsp;**Requirements:**</Markdown>

  - &nbsp;&nbsp;[Quick start with coding on QTrobot](/docs/intro_code)
  - &nbsp;&nbsp;[Create a ROS python project](/docs/tutorials/python/python_ros_project)
  - &nbsp;&nbsp;[QTrobot interfaces using ROS Topics](/docs/tutorials/python/python_ros_publish)
  - &nbsp;&nbsp;[QTrobot interfaces using ROS Subscribers](/docs/tutorials/python/python_ros_subscribe)
:::

In this tutorial we will learn how to detect human hands status with [QTrobot Nuitrack interface](/docs/api_ros#human-3d-tracking-interface) using ROS Subscribes. 

## Create a python project 
First we create a python project for our tutorial. let's call it `tutorial_qt_hands` and add the required python file: 

```
cd ~/catkin_ws/src
catkin_create_pkg tutorial_qt_hands std_msgs rospy roscpp -D "Reading human hands status"
cd tutorial_qt_hands/src
touch tutorial_qt_hands_node.py
chmod +x tutorial_qt_hands_node.py
```


## Code
Open the `tutorial_qt_hands_node.py` file and add the following code:

```python
#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from qt_nuitrack_app.msg import Hands
from qt_robot_interface.srv import *


def hands_callback(msg):
    if msg.hands[0].right_click:
        speech_pub.publish("Right hand")
        rospy.sleep(2)
    elif msg.hands[0].left_click:
        speech_pub.publish("Left hand")
        rospy.sleep(2)

if __name__ == '__main__':
    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
    speech_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=1)

    # define ros subscriber
    rospy.Subscriber('/qt_nuitrack_app/hands', Hands, hands_callback)
   
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")

```

First we imported `Hands` message type from `qt_nuitrack_app.msg` message library. This message type is used in communication with `/qt_nuitrack_app/hands`. 

:::tip Tip
How do we know which messages an interface uses? well, There is a useful command in ROS which tells you that: 

```
rostopic info /qt_nuitrack_app/hands
Type: qt_nuitrack_app/Hands
...
```
:::

Then we created a subscriber for`/qt_nuitrack_app/hands` with callback function `hands_callback`.
In the callback we react accordingly to what is detected. If we detect "right_click" QTrobot will say "Right hand". That will make QTrobot react to your hands.

```python
def hands_callback(msg):
    if msg.hands[0].right_click:
        speech_pub.publish("Right hand")
        rospy.sleep(2)
    elif msg.hands[0].left_click:
        speech_pub.publish("Left hand")
        rospy.sleep(2)
```

Stand in front of QTrobot and lift up your right arm pointing towards QTrobot with open hand. When you close your right hand the "right_click" will be detected.