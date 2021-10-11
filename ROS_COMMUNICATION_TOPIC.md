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