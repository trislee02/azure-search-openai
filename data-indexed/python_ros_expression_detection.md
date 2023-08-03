---
id: python_ros_expression
title: Human facial expression detection
hide_table_of_contents: true
---

# Human facial expression detection

import Icon from "@material-ui/core/Icon";
import Markdown from 'markdown-to-jsx';

:::info Overview
<Icon>signal_cellular_alt</Icon> <Markdown>&nbsp;**Level:**&nbsp; *Intermediate*</Markdown>
<br/> <Icon> track_changes </Icon> <Markdown>&nbsp;**Goal:**&nbsp; *learn how to detect Human facial expressions with QTrobot Nuitrack interface*</Markdown>
<br/> <Icon> task_alt </Icon> <Markdown>&nbsp;**Requirements:**</Markdown>

  - &nbsp;&nbsp;[Quick start with coding on QTrobot](/docs/intro_code)
  - &nbsp;&nbsp;[Create a ROS python project](/docs/tutorials/python/python_ros_project)
  - &nbsp;&nbsp;[QTrobot interfaces using ROS Topics](/docs/tutorials/python/python_ros_publish)
  - &nbsp;&nbsp;[QTrobot interfaces using ROS Subscribers](/docs/tutorials/python/python_ros_subscribe)
:::

In this tutorial we will learn how to detect Human facial expressions with [QTrobot Nuitrack interface](/docs/api_ros#human-3d-tracking-interface) using ROS Subscribes. 

## Create a python project 
First we create a python project for our tutorial. let's call it `tutorial_qt_expressions` and add the required python file: 

```
cd ~/catkin_ws/src
catkin_create_pkg tutorial_qt_expressions std_msgs rospy roscpp -D "Reading human face expressions"
cd tutorial_qt_expressions/src
touch tutorial_qt_expressions_node.py
chmod +x tutorial_qt_expressions_node.py
```


## Code
Open the `tutorial_qt_expressions_node.py` file and add the following code:

```python
#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from qt_nuitrack_app.msg import Faces
from qt_robot_interface.srv import *


def face_callback(msg):
    emotions = [msg.faces[0].emotion_angry, msg.faces[0].emotion_happy, msg.faces[0].emotion_surprise]
    em = max(emotions)
    em_index = emotions.index(em)
    if em_index == 0 and em >= 0.9:
        speech_pub.publish("It looks like you are angry! This is my angry face")
        emotionShow("QT/angry")
        rospy.sleep(2)
    elif em_index == 1 and em >= 0.9:
        speech_pub.publish("You are happy! This is my happy face")
        emotionShow("QT/happy")
        rospy.sleep(2)
    elif em_index == 2 and em >= 0.9:
        speech_pub.publish("This is surprising!")
        emotionShow("QT/surprise")
        rospy.sleep(2)
    
    
if __name__ == '__main__':
    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
    speech_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=1)

    # define ros subscriber
    rospy.Subscriber('/qt_nuitrack_app/faces', Faces, face_callback)
   
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")

```

## Explanation

First we imported `Faces` message type from `qt_nuitrack_app.msg` message library. This message type is used in communication with `/qt_nuitrack_app/faces`. 

:::tip Tip

How do we know which messages an interface uses? well, There is a useful command in ROS which tells you that: 

```
rostopic info /qt_nuitrack_app/faces
Type: qt_nuitrack_app/Faces
...
```

:::

Then we created a subscriber for`/qt_nuitrack_app/faces` with callback function `face_callback`.
In the callback we react accordingly to what is detected. We check all the emotions that are detected and we use the one which has the highest rating. If we detect happy emotion we also make QTrobot show happy emotion.

```python
def face_callback(msg):
    emotions = [msg.faces[0].emotion_angry, msg.faces[0].emotion_happy, msg.faces[0].emotion_surprise]
    em = max(emotions)
    em_index = emotions.index(em)
    if em_index == 0 and em >= 0.9:
        speech_pub.publish("It looks like you are angry! This is my angry face")
        emotionShow("QT/angry")
        rospy.sleep(2)
    elif em_index == 1 and em >= 0.9:
        speech_pub.publish("You are happy! This is my happy face")
        emotionShow("QT/happy")
        rospy.sleep(2)
    elif em_index == 2 and em >= 0.9:
        speech_pub.publish("This is surprising!")
        emotionShow("QT/surprise")
        rospy.sleep(2)
```

Stand in front of QTrobot and smile or make an angry face. 
When QTrobot detects the emotion, it will repeat the same emotion as you showed.


