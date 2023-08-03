---
id: python_ros_publish
title: QTrobot interfaces using ROS Publishers
hide_table_of_contents: true
---


import Icon from "@material-ui/core/Icon";
import Markdown from 'markdown-to-jsx';

:::info Overview
<Icon>signal_cellular_alt</Icon> <Markdown>&nbsp;**Level:**&nbsp; *Basic*</Markdown>
<br/> <Icon> track_changes </Icon> <Markdown>&nbsp;**Goal:**&nbsp; *learn how to access QTrobot interfaces such as speech, emotion, gesture, etc. in python via ROS publishers*</Markdown>
<br/> <Icon> task_alt </Icon> <Markdown>&nbsp;**Requirements:**</Markdown>

  - &nbsp;&nbsp;[Quick start with coding on QTrobot](/docs/intro_code)
  - &nbsp;&nbsp;[Create a ROS python project](/docs/tutorials/python/python_ros_project)
:::

If you have followed our previous tutorials, you should know how to start coding on QTrobot with python and got the basic knowledge of ROS framework.  In this tutorial we will learn about how to access [QTrobot interfaces](/docs/api_ros#list-of-available-interfaces) such as speech and audio interface using ROS publishers. 

## Create a python project 
First we create a python project for our tutorial. let's call it `tutorial_qt_publisher` and add the required python file: 

```
cd ~/catkin_ws/src
catkin_create_pkg tutorial_qt_publisher std_msgs rospy roscpp -D "Command QTrobot via ROS Publishers"
cd tutorial_qt_publisher/src
touch tutorial_qt_publisher_node.py
chmod +x tutorial_qt_publisher_node.py
```

## QTrobot speech publisher  
Open the `tutorial_qt_publisher_node.py` file and add the following code:

```python
#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

   # creating a ros publisher
   speechSay_pub = rospy.Publisher('/qt_robot/speech/say', String, queue_size=10)
   rospy.sleep(3.0)

   # publish a text message to TTS
   speechSay_pub.publish("Hello! my name is QT!")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")

```

First we imported `std_msgs.msg` from ROS standard message library. This message is used in `/qt_robot/speech/say` to communicate with QTrobot speech (TTS) interface. 

:::tip Tip
How do we know which message type an interface uses? well, There is a useful command in ROS which tells you that: 
```bash
rostopic info /qt_robot/speech/say
Type: std_msgs/String
...
```
:::

Then we created a publisher to '/qt_robot/speech/say' topic and wait for few seconds to establish the connection with the interface. 

:::tip Tip
In the above example, for simplicity we used few seconds of delay using `sleep` function. A more appropriate way is to wait and check until the connection with a subscriber is established: 
```python 
# wait for publisher/subscriber connections
wtime_begin = rospy.get_time()
while (speechSay_pub.get_num_connections()) :
    rospy.loginfo("waiting for subscriber connections")
    if rospy.get_time() - wtime_begin > 5.0:
        rospy.logerr("Timeout while waiting for subscribers connection!")
        sys.exit()
    rospy.sleep(0.5)
```
:::

Finally we published a text message to QTrobot speech interface which make the robot read that message. 

## QTrobot talk text publisher 
The `/qt_robot/behavior/talkText` interface is similar to `/qt_robot/speech/say` interface with the only different that the talkText interface asks QTrobot to move his lips while reading the text messages. To try it, just add the following lines to our code:

```python
behaviorTalkText_pub = rospy.Publisher('/qt_robot/behavior/talkText', String, queue_size=10)
behaviorTalkText_pub.publish("Hello! my name is QT!")
```

## QTrobot emotion publisher 
Now lets show an emotion on QTrobot face. QTrobot comes with plenty of predefined emotion animations. You can find the complete list of the available emotions either using the *QTrobot Educator app* or by looking into the `~/robot/data/emotions` folder in **QTRP**.  

Add the following lines to our code to show the 'happy' emotion under 'QT' category on QTrobot face: 
```python 
emotionShow_pub = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)
emotionShow_pub.publish("QT/happy")
```

:::note Note
As it shown in the above example, you should **not** give the emotion's file extension (`.avi`) to the interface! 
:::

## QTrobot gesture publisher 
Now lets play a gesture with QTrobot. QTrobot comes with plenty of predefined gestures. You can find the complete list of the available gestures either using the *QTrobot Educator app* or by looking into the `~/robot/data/gestures` folder in **QTRP**.  

Add the following lines to our code to play the 'clapping' gesture under 'QT' category: 
```python 
gesturePlay_pub = rospy.Publisher('/qt_robot/gesture/play', String, queue_size=10)
gesturePlay_pub.publish("QT/clapping")
```

:::note Note
As it shown in the above example, you should **not** give the gestures's file extension (`.xml`) to the interface! 
:::

## QTrobot audio publisher 
Now lets play an audio file on QTrobot. QTrobot comes with some audio examples. You can find the complete list of the available audios either using the *QTrobot Educator app* or by looking into the `~/robot/data/audios` folder in **QTRP**.  QTrobot can play both audio *wave* and *mp3* files. 


Add the following lines to our code to play the 'Komiku_Glouglou' audio file under 'QT' category: 
```python 
audioPlay_pub = rospy.Publisher('/qt_robot/audio/play', String, queue_size=10)
audioPlay_pub.publish("QT/Komiku_Glouglou")
```

:::note Note
As it shown in the above example, you do not need to give the audio's file extension (`.wav` or `.mp3`) to the interface! 
:::

