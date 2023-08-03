---
id: python_ros_services
title: QTrobot interfaces using ROS Services
hide_table_of_contents: true
---

# QTrobot interfaces using ROS Services

import Icon from "@material-ui/core/Icon";
import Markdown from 'markdown-to-jsx';

:::info Overview
<Icon>signal_cellular_alt</Icon> <Markdown>&nbsp;**Level:**&nbsp; *Basic*</Markdown>
<br/> <Icon> track_changes </Icon> <Markdown>&nbsp;**Goal:**&nbsp; *learn how to access QTrobot interfaces such as speech, emotion, gesture, etc. in python via ROS Services*</Markdown>
<br/> <Icon> task_alt </Icon> <Markdown>&nbsp;**Requirements:**</Markdown>

  - &nbsp;&nbsp;[Quick start with coding on QTrobot](/docs/intro_code)
  - &nbsp;&nbsp;[Create a ROS python project](/docs/tutorials/python/python_ros_project)
:::

If you have followed our previous tutorials, you should know how to start coding on QTrobot with python and got the basic knowledge of ROS framework. In this tutorial we will learn about how to access [QTrobot interfaces](/docs/api_ros#list-of-available-interfaces) such as speech and audio interface using ROS Services. 

## Create a python project 
First we create a python project for our tutorial. let's call it `tutorial_qt_service` and add the required python file: 

```
cd ~/catkin_ws/src
catkin_create_pkg tutorial_qt_service rospy roscpp -D "Command QTrobot via ROS Services"
cd tutorial_qt_service/src
touch tutorial_qt_service_node.py
chmod +x tutorial_qt_service_node.py
```

## QTrobot speech service 
Open the `tutorial_qt_service_node.py` file and add the following code:

```python
#!/usr/bin/env python
import sys
import rospy
from qt_robot_interface.srv import *

if __name__ == '__main__':
    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    # define a ros service
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    
    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/speech/say') 
   
    try:
        # call a ros service with text message
        speechSay("Hello! This is QT talking using text to speech")
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")

```

ROS Services are defined by srv files, which contains a request message and a response message. First we import all from `qt_robot_interface.srv`. This will import all srv files that are under `qt_robot_interface.srv`. We need to use `speech_say`. 

:::tip Tip
How do we know which parameters an interface uses? well, There is a useful command in ROS which tells you that: 
```
rosservice info /qt_robot/speech/say
Type: qt_robot_interface/speech_say
Args: message
...
```
:::

Then we defined a service `/qt_robot/speech/say` and call `rospy.wait_for_service()` to block until a service is available. 

Finally we called a ROS service with a text message to QTrobot speech interface which makes the robot read that message. 

## QTrobot talk text service 
The `/qt_robot/behavior/talkText` interface is similar to `/qt_robot/speech/say` interface with the only different that the talkText interface asks QTrobot to move his lips while reading the text messages. To try it, just add the following lines to our code:

```python
behaviorTalkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
rospy.wait_for_service('/qt_robot/behavior/talkText')
behaviorTalkText("I am QT robot!")
```

## QTrobot emotion service 
Now lets show an emotion on QTrobot face. QTrobot comes with plenty of predefined emotion animations. You can find the complete list of the available emotions either using the *QTrobot Educator app* or by looking into the `~/robot/data/emotions` folder in **QTRP**.  

Add the following lines to our code to show the 'happy' emotion under 'QT' category on QTrobot face: 

```python 
emotionShow = rospy.ServiceProxy('/qt_robot/emotion/show', emotion_show)
rospy.wait_for_service('/qt_robot/emotion/show')
emotionShow("QT/happy")
```

:::note Note
As it shown in the above example, you should **not** give the emotion's file extension (`.avi`) to the interface! 
:::

## QTrobot gesture service 
Now lets play a gesture with QTrobot. QTrobot comes with plenty of predefined gestures. You can find the complete list of the available gestures either using the *QTrobot Educator app* or by looking into the `~/robot/data/gestures` folder in **QTRP**.  

Add the following lines to our code to play the 'clapping' gesture under 'QT' category: 
```python 
from qt_gesture_controller.srv import gesture_play

gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)
rospy.wait_for_service('/qt_robot/gesture/play')
gesturePlay("QT/happy", 0)
```

:::note Note
As it shown in the above example, you should **not** give the gestures's file extension (`.xml`) to the interface! 
:::

## QTrobot audio service 
Now lets play an audio file on QTrobot. QTrobot comes with some audio examples. You can find the complete list of the available audios either using the *QTrobot Educator app* or by looking into the `~/robot/data/audios` folder in **QTRP**.  QTrobot can play both audio *wave* and *mp3* files. 


Add the following lines to our code to play the 'Komiku_Glouglou' audio file under 'QT' category: 
```python 
audioPlay = rospy.ServiceProxy('/qt_robot/audio/play', audio_play)
rospy.wait_for_service('/qt_robot/audio/play')
audioPlay("QT/Komiku_Glouglou", "")
```

:::note Note
As it shown in the above example, you do not need to give the audio's file extension (`.wav` or `.mp3`) to the interface! 
:::

