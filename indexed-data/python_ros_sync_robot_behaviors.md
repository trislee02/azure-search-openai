---
id: python_ros_sync_robot_behaviors
title: Synchronizing QTrobot Behaviors
hide_table_of_contents: true
---

# Synchronizing QTrobot Behaviors

import Icon from "@material-ui/core/Icon";
import Markdown from 'markdown-to-jsx';

:::info Overview
<Icon>signal_cellular_alt</Icon> <Markdown>&nbsp;**Level:**&nbsp; *Intermediate*</Markdown>
<br/> <Icon> track_changes </Icon> <Markdown>&nbsp;**Goal:**&nbsp; *learn how to synchronize QTrobot behaviors/actions*</Markdown>
<br/> <Icon> task_alt </Icon> <Markdown>&nbsp;**Requirements:**</Markdown>

  - &nbsp;&nbsp;[Quick start with coding on QTrobot](/docs/intro_code)
  - &nbsp;&nbsp;[Create a ROS python project](/docs/tutorials/python/python_ros_project)
  - &nbsp;&nbsp;[QTrobot interfaces using ROS Topics](/docs/tutorials/python/python_ros_publish)
  - &nbsp;&nbsp;[QTrobot interfaces using ROS Subscribers](/docs/tutorials/python/python_ros_subscribe)
:::

In this tutorial we will learn how to synchronize QTrobot behaviors or actions. We want to make QTrobot talk and do a gesture at the same time and wait for both of them to finish.
There are multiple ways how to do it. 
One option is to use ROS Topics which are non-blocking and we would be able to execute both rostopics at the same time.

```python
talk_pub.publish("Hello! my name is QT!") #rostopic
gesture_play.publish("QT/happy") #rostopic
```

This will make QTrobot say **"Hello! my name is QT!"** and play a **"happy"** gesture at the same time. The issue is that we don't know how long it takes to finish and we can't wait for both of them finish. That means whatever is after **"gesture_play"** in the code, it will be executed without waiting for gesture_play/talk_pub to finish. This might work in certain situations, but for our use case we need to find something else.

The other option would be to use mix of ROS topics and services, because ROS services are blocking and will stop/block the execution of the program until that service call finishes.
In the same example above we could replace gesture play topic with a ros service call and the code would look like this:

```python
talk_pub.publish("Hello! my name is QT!") #rostopic
gesturePlay('QT/happy', 0) #rosservice
```

In this situation **talk_pub** and **gesturePlay** will execute at the same time, and the **gesturePlay** service call will block the execution of the program until it finishes playing the gesture. We come to an issue that, in this example, it doesn't matter how long or short the text is used for **talk_pub**, program will continue after the **"gesturePlay"** service is done. 

This doesn't solve our task, in which we need to wait for both to finish. So let's implement a synchronizer for our actions/behaviors, in which we will execute both gesturePlay and talkText at the same time and wait for both of them to finish.

## Create a python project 
First we create a python project for our tutorial. let's call it `tutorial_sync_qt_behaviors` and add the required python files: 

```
cd ~/catkin_ws/src
catkin_create_pkg tutorial_sync_qt_behaviors std_msgs rospy roscpp -D "Synchronizing QTrobot Behaviors"
cd tutorial_sync_qt_behaviors/src
touch tutorial_sync_qt_behaviors.py
touch synchronizer.py
chmod +x tutorial_sync_qt_behaviors.py
```

## Synchronizer

TaskSynchronizer is a simple class, using asyncio and threads, that enables us to run multiple tasks at the same time and wait for them to finish.
Open the `synchronizer.py` file and add the following code:

```python
import time
import asyncio
import concurrent.futures

class TaskSynchronizer():
    """
    A simple concurrent tasks synchornizer
    """

    def __init__(self, max_workers=5):
        self.loop = asyncio.get_event_loop()
        self.executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=max_workers)

    def __worker(self, *args):
        delay_exe = args[0][0]
        func = args[0][1]
        time.sleep(delay_exe)
        return func()

    async def __non_blocking(self, tasks):
        fs = []
        for task in tasks:
            fs.append(self.loop.run_in_executor(
                self.executor, self.__worker, task))
        done, pending = await asyncio.wait(fs=fs, return_when=asyncio.ALL_COMPLETED)
        results = [task.result() for task in done]
        return results

    def sync(self, tasks):
        """
        call this function with multiple tasks to run concurrently.
        tasks is a list of (delay, lamda function) tuple. for exmaple:
        tasks = [ (0, lambda: print("hello")), (3, lambda: print("world")), ...]
        returns a list of each lamda function return value
        """
        results = self.loop.run_until_complete(self.__non_blocking(tasks))
        return results
```


## Main Code

Open the `tutorial_sync_qt_behaviors.py` file and add the following code and please check the [explanation](/docs/tutorials/python/python_ros_sync_robot_behaviors#explanation) below:

```python
#!/usr/bin/env python
import sys
import rospy
from qt_robot_interface.srv import *
from qt_gesture_controller.srv import *
from synchronizer import TaskSynchronizer

if __name__ == '__main__':
    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

   # define a ros service
    talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
    gesturePlay = rospy.ServiceProxy('/qt_robot/gesture/play', gesture_play)

    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/behavior/talkText')
    rospy.wait_for_service('/qt_robot/gesture/play')

    # ceate an instance of TaskSynchronizer
    ts = TaskSynchronizer()

    # call talkText and gesturePlay at the same time
    # wait until both finish their jobs
    print('calling talkText and gesturePlay...')
    results = ts.sync([
        (0, lambda: talkText('Hello! my name is QT!')),
        (0, lambda: gesturePlay('QT/happy', 0))
    ])
    print('talkText and gesturePlay finished.')
```

### Explanation

ROS Services are defined by srv files, which contains a request message and a response message. First we import all from `qt_robot_interface.srv` and `qt_gesture_controller.srv`. This will import all srv files that are under `qt_robot_interface.srv` and `qt_gesture_controller.srv`.
In this example we want to call talkText and gesturePaly services at the same time, so we need `behavior_talk_text` and `gesture_play`.

:::tip Tip
How do we know which service an interface uses? well, There is a useful command in ROS which tells you that: 
```
Type: qt_robot_interface/behavior_talk_text
Args: message
```
:::

Next we import `TaskSynchronizer` class from `synchronizer`. This will allow as to create an instance of TaskSynchronizer and execute class function calls. 
In the main, after we define ros services that we need, we create an instance of `TaskSynchronizer`:

```python
# ceate an instance of TaskSynchronizer
ts = TaskSynchronizer()
```

We want to use `sync` function from `TaskSynchronizer`, to which we can pass multiple tasks that we want to run concurrently.
We provide an array of `tasks` to the function, which are tuples of delay and lambda function.
Task should look something like this:
`(0, lambda: print("hello"))`

First parameter **delay** -> **0**, will tell synchronizer when to execute second parameter **lambda function** -> **lambda: print("hello")**.

Array of task would look like this example:
` tasks = [ (0, lambda: Task1()), (0, lambda: Task2()), (0.6, lambda: Task3()), ...]`

[![](https://mermaid.ink/img/pako:eNpNjrtuwzAMRX9FINDNMCT5QUtz0albMxVaiFhJhVhyYNOAUyP_XjnJUE68h8QhNziOvQcLZ0rMLjnuif3HOEVise6Z1jC_8tunSyIXBx68OIToh5D8k7l0oPkilBWyEKqRMs4vpJ9I_kOVFa3MEHcGBUSf_aHPb2y7zQH_-Ogd2Nz2_kTLwA5cuudVWnj8uqUjWJ4WX8By3R9-D3SeKII90TBneqUEdoMVbIVlawy2dWdqVLIp4AZWKywRUaNstG7qTnX3An7HMQtk2Rk0XdWgUloaifph-34M95P3P0x1XKU?type=png)](https://mermaid.live/edit#pako:eNpNjrtuwzAMRX9FINDNMCT5QUtz0albMxVaiFhJhVhyYNOAUyP_XjnJUE68h8QhNziOvQcLZ0rMLjnuif3HOEVise6Z1jC_8tunSyIXBx68OIToh5D8k7l0oPkilBWyEKqRMs4vpJ9I_kOVFa3MEHcGBUSf_aHPb2y7zQH_-Ogd2Nz2_kTLwA5cuudVWnj8uqUjWJ4WX8By3R9-D3SeKII90TBneqUEdoMVbIVlawy2dWdqVLIp4AZWKywRUaNstG7qTnX3An7HMQtk2Rk0XdWgUloaifph-34M95P3P0x1XKU)

As you can see above in the graph, Task1 and Task2 start at the same time, because they don't have any delay set and Task3 will be executed with 600ms of delay.
Now when we now how `sync` function works, we can implement talkText and gesturePlay service calls like this:

```python
# call talkText and gesturePlay at the same time 
# wait until both finish their jobs
results = ts.sync([
    (0, lambda: talkText('Hello! my name is QT!')),
    (0, lambda: gesturePlay('QT/happy', 0))
])
```

If we wanted to execute gesturePlay a bit later then talkText, we could write something like this:

```python
# call gesturePlay 0.5s (500ms) after talkText 
# wait until both finish their jobs
results = ts.sync([
    (0, lambda: talkText('Hello! my name is QT!')),
    (0.5, lambda: gesturePlay('QT/happy', 0))
])
```

As explained above `sync` function waits (blocking) until all services finish their jobs and after that returns, so if we wanted to have two/three sets of actions/behaviors that QTrobot should execute, it might look something like this:

```python
# calling first set
results = ts.sync([
    (0, lambda: talkText('Hello! my name is QT!')),
    (0, lambda: gesturePlay('QT/happy', 0))
])
# after first set executes it will call second set
results = ts.sync([
    (0, lambda: talkText('It was nice meeting you!')),
    (0, lambda: gesturePlay('QT/bye', 0))
])
```

In this example we just used **gesturePlay** and **talkText**, but you can use any other ros services which you want to synchronize.



