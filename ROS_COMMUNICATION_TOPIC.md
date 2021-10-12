# **ROS TOPICS**

- ROS topics allows **unidirectional** communication between ROS nodes.
- When using ROS Topics ROS Node can be a **publisher, subscriber or both**.
- Publisher and Subscriber Nodes will exchange ROS Messages over a ROS Topic.
- ROS message is aq simple data structure(ex: integer, floating point, boolean, string, etc.). So a ROS message can hold data of various data-types.
- Let's take a terminology
    + You are ordering a **newspaper** named as **bhaskar**, Which is **published** by **bhaskar times**.
    + every morning **ramu kaka** will **deliver** **bhaskar** to you.
    + You like **bhaskar** because it is having **madhurima**, **chitralok** section.
- In this analogy we can relate.
    + >bhaskar times <---> Publisher Node
    + >You <---> Subcriber Node
    + >ramu kaka <---> Topic
    + >bhaskar <--> ROS message
    + >madhurima and chitralok <---> data fields defined in ROS messages

![ROS Topic](/home/kash/Pictures/topic.png "ROS Topic")

+ Basically ROS Publisher will ```brodcast``` the message and any Node can subsctibe it and take the information.
+ TOpics are named ```buses``` over which ```nodes``` exchange ```messages```. 
+ Topics have anonymous publish/subscribe sementics which decouples the production of information from it's consumption.
    - Inshort ```nodes``` are not aware of that who they are communicating. Insted of that they are intrested in a perticular data.

### **Topic Types***

+ Eac topic is strongly typed by the ```ROS Messages``` type used to publish to it and nodes can only receive messages with a matching type(or else you will get an error).
    + subscriber should clearly specify the message type before receiving a perticular message.
    + All ROS clients check to makesure that an MD5 computed from the ```msg files```  match. This check ensures that the ROS Nodes were compiled from consistent code bases.

### **Topic Transport**

> + ROS currently supports ```TCP/IP based and UDP- based ```message transport.
> + TCP based transport is known as ```TCPROS``` and streams message data over persistent TCP/IP connections.
> + TCPROS is the default transport used in ROS and is the only transport that client libraries are required to support. 
> + The UDP-based transport, which is known as UDPROS and is currently only supported in roscpp, separates messages into UDP packets. UDPROS is a low-latency, lossy transport, so is best suited for tasks like teleoperation.
> + ROS nodes negotiate the desired transport at runtime. For example, if a node prefers UDPROS transport but the other Node does not support it, it can fallback on TCPROS transport. This negotiation model enables new transports to be added over time as compelling use cases arise.

+ There is one command for viewing the topics and it's related information.

```
rostopic -h
```
![rostopic -h](/home/kash/Pictures/rostopic.png "rostopic -h")

``` 
rostopic type [topic]
```
+ Communication on topics happens by sending ROS messages between nodes. To communicate, the publisher and subscriber must send and receive the same ```type``` of message. This means that a topic ```type``` is defined by the message ```type``` published on it. The ```type``` of the message sent on a topic can be determined using rostopic ```type```.

> For understanding this in better form do the following:

+ Run ros master in one terminal
```
roscore
```
+ run below command in other terminal window.
```
rosrun turtlesim turtlesim_node
```
+ Now run:
```
rostopic list
```

>Output

![rostopic list](/home/kash/Pictures/t_topic.png "rostopic list")

+ Now run:
```
rostopic type /turtle1/cmd_vel
```
>Output

+ It will be the message type.
```

╭─ kash@pop-os  ~                        ✔  6828  00:52:09
╰─ rostopic type /turtle1/cmd_vel 
geometry_msgs/Twist
```

+ As we observed the type of message associated with ```/turtle1/cmd_vel``` topic is ```geometry_msgs/Twist```. let's look more details about this message.

+ Run:
```
rosmsg show geometry_msgs/Twist
```
>Output
```
╭─ kash@pop-os  ~                        ✔  6838  01:01:51
╰─ rosmsg show geometry_msgs/Twist

geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z

```
+ Messges consist of ```fields``` and ```constants```.
+ ```fields``` are dataypes and ```constants``` are representative values.
+  From the above output, you can observe that these field and constants are displayed twice. However, both of these sections, are separate since they have a different header or different sub-information from the same robot. The 2 headers seen are...
     
     +  ```geometry_msgs/Vector3 linear```: Describes the linear velocities of all the 3 axes.

     + ```geometry_msgs/Vector3 angular```: While this header describes, angular velocities of all 3 axes.

```
rostopic info /turtle1/cmd_vel
```
>Output

```
─ kash@pop-os  ~                      1 ↵  6841  01:03:1
╰─ rostopic info /turtle1/cmd_vel 
Type: geometry_msgs/Twist

Publishers: None

Subscribers: 
 * /turtlesim (http://pop-os:34703/)

```
+ The output of this command will display both:
    + message type
    + Publishing and subcribing Node.


```
rostopic pub...
```
+ you can figure out by your own
+ it's for publishing the value from terminal.
+ you can use ```-h``` for seeking help from terminal.

```
rostopic echo [TopicName]
```
+ It's for printing published message into the terminal itself

## **Example #1: Pub-Sub with Custom Message**

### **Aim**

To write a ````listener```` and ```talker ```node which should communicate with each other over a ROS Topic called ```my_topic ```using a custom ROS Message called ```myMessage``` with the following data fields of the following data types.

```
1. int32 id
2. string name
3. float32 temperature
4. float32 humidity
```
### **Steps**

#### **Create Custom ROS message**

+ Messages are just a simple text file with field type and field name per line.
+ They are stored in ```msg``` directory of our package.

1. Creare a file and name it ```myMessage.msg``` and store it into the ```msg``` folder of ```ros_basics``` package. If folder does not exist then create it.

2. Now fill the ```myMessage.msg``` file with the following content.

```
int32 id
string name
float32 temperature
float32 humidity
```
This is the format of the typical ```msg``` file.

3. Now open ```package.xml``` file of ```ros_basics``` and add the dependencies for our ```message_generarion``` and ```message_runtime``` as seen below.

```
<?xml version="1.0"?>
<package format="2">
  <name>pkg_ros_basics</name>
  <version>0.1.0</version>
  <description>The pkg_ey_ros_basics package</description>

  <maintainer email="eyantra@todo.todo">eyantra</maintainer>

  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>message_generation</build_depend>
  
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>message_runtime</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```
>```message_generation``` will actually depend on all default supported generators and makes sure that they are actually present. Otherwise e.g. when building Debians on the farm for a package containing messages it might not have any language specific message generators installed.

>```geometry_msgs``` provides messages for common geometric primatives such as ```points, vectors, and poses```. These primatives are designed to provide a common data type and facilitate interoperability throughout the system.

>In the same way, ```message_runtime``` is a package used for build-time dependencies to generate language bindings of messages. This means that is the package in charge of compiling and generating all the needed files when you add a custom msg type, service, action etc. in your package.

4. Now open your ```CMakeliist.txt``` of ```ros_basics``` and navigate to the following block of code in your file.

```
#add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```
Uncomment the Messages and add include the name of your message files. You can include multiple message files if require.

Now your ```CMakelist.txt``` should look like this:

```
cmake_minimum_required(VERSION 3.0.2)
project(pkg_ros_basics)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  message_generation
)

add_message_files(
  FILES
  myMessage.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control_turtle
 CATKIN_DEPENDS geometry_msgs rospy message_runtime

)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
```

5. After this build your package.

```
cd ~/katkin_ws
catkin_make
```

> Remember package.xml file is importent for specifying dependencies, if you haven't mentioned a perticular dependency and you have used it in CMakelist.txt then it will throw the error at the build time so make sure to observe each steps carefully! and if needed then please see all the dependencies propperly.

>Once package is build successfully you can see ```myMessage.h``` file lockated at ```~/katkin_ws/devel/include/ros_basics/myMessages.h``` This will be used by ROS Nodes to communicate over a ROS topic using ```myMessage``` ROS Message.


## **Code--> ROS Nodes**

### **Listener Node**

```node_myMsg_listener.py```

```
#!/usr/bin/env python3

import rospy
from pkg_ros_basics.msg import myMessage


def func_callback_topic_my_topic(myMsg):

    rospy.loginfo("Data Received: (%d, %s, %.2f, %.2f)", myMsg.id,
                  myMsg.name, myMsg.temperature, myMsg.humidity)


def main():

    # 1. Initialize the Subscriber Node.
    rospy.init_node('node_myMsg_listener', anonymous=True)

    # 2. Subscribe to the desired topic and attach a Callback Funtion to it.
    rospy.Subscriber("my_topic", myMessage, func_callback_topic_my_topic)

    # 3. spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

### **Talker Node**

```node_myMsg_talker.py```

```
#!/usr/bin/env python

import rospy
from pkg_ros_basics.msg import myMessage

import random


def main():
    
    # 1. Create a handle to publish messages to a topic.
    var_handle_pub = rospy.Publisher('my_topic', myMessage, queue_size=10)
    
    # 2. Initializes the ROS node for the process.
    rospy.init_node('node_myMsg_talker', anonymous=True)

    # 3. Set the Loop Rate 
    var_loop_rate = rospy.Rate(1) # 1 Hz : Loop will its best to run 1 time in 1 second
    
    # 4. Write the infinite Loop
    while not rospy.is_shutdown():
        obj_msg = myMessage()

        obj_msg.id = 1
        obj_msg.name = "my_message"
        obj_msg.temperature = 10 + random.random()
        obj_msg.humidity = 20 + random.random()

        rospy.loginfo("Publishing: ")
        rospy.loginfo(obj_msg)

        var_handle_pub.publish(obj_msg)

        var_loop_rate.sleep()



# Python Main
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```