# USFSOAR AGSE

|  Section      | Currently Used|
| ------------- |:-------------:|
|  Hardware     | Raspberry Pi  |
|       OS      | Ubuntu 14.0.4       |
|       Robot OS        |      ROS      |
|   |      |

## Basic Setup

### Setting a static IP

Find current settings with 
```bash
$  route -n
```
Edit `/etc/network/interfaces`
```
# Change line 
iface eth0 inet dynamic

# New settings
iface eth0 inet static
address 192.168.1.42 	# Static IP
netmask 255.255.255.0
network 192.168.1.0
broadcast 192.168.1.255
gateway 192.168.1.1


```

Then restart networking services
```bash
$  sudo reboot now
```


Find all ethernet connected devices
```bash
$  ifconfig -a | grep eth
```

Scan for all IP addresses on the network
```bash
# Make sure nmap is installed
$  sudo apt-get -y install nmap

# Scan Network
$ nmap -sn 192.168.1.0/24

# Login to device
$ ssh <username>@<ip>
```

## To check environment variables
```
$	printenv | grep ROS
```
## Running on multiple computers
To connect multiple computers with ros over ssh, make sure all computers know where the master node is and that the master node can find the others by ip address.

On the roscore master node in the `~/.bashrc` file so that these commands load on every shell. The basic premise here is to let every node know who is the master ip node running roscore then in the second line broadcast the computers own ip.
```
IP1: 	192.168.1.42 	# raspberry pi
IP2:	192.168.1.173 	# laptop
```
In pi `~/.bashrc`
```
export ROS_MASTER_URI=http://192.168.1.42:11311
export ROS_IP=192.168.1.42
```
In the laptop `~/.bashrc`
```
export ROS_MASTER_URI=http://192.168.1.42:11311
export ROS_IP=192.168.1.173
```
This will only need to be done once on every machine as long as the ip addresses remain the same.


## The `~/.bashrc` file. 

AKA stuff you would have to do every time you open up a terminal but are too lazy to
```bash
# Source the ROS installation
$  source /opt/ros/indigo/setup.bash

# Source a package
$  source /home/athsmat/Rosstuff/mac_catkin_ws/devel/setup.bash

# Let your device know who the master node is (even if it is the master node)
$  export ROS_MASTER_URI=http://192.168.1.42:11311

# Let every other device know your IP
$  export ROS_IP=192.168.1.173

#
$

#
$

#
$

#
$

#
$


```


## Starting up the kinect

Remove the gspca modules so that the user-mode libfreenect drive can take over.
```bash
$  	sudo modprobe -r gspca_kinect

$  	sudo modprobe -r gspca_main
```
Launch files to open an OpenNI device and load all nodelets to convert raw depth/RGB/IR streams to depth images, disparity images, and (registered) point clouds.
```
$  	roslaunch openni_launch openni.launch
```
Now start `rqt` from laptop, or secondary device with ros desktop tools, and select `plugins` then `visualization` then `image view`.
```
$	rqt
```
## Finding ROS Drivers

Drivers for most things that will be needed can be found at https://github.com/ros-drivers/ . 

Steps to install:
1. Find driver (eg: https://github.com/ros-drivers/rosserial.git).
2. Clone into the `src` folder in your catkin workspace directory.
3. `cd` back to workspace directory.
4. Run `$ catkin_make`
5. Run `$ catkin_make install`


## Creating and Sourcing a Workspace. 

First create `ros_workspace` and `ros_workspace/src`.
```bash
$	cd ros_workspace/src
$	git clone https://github.com/ros-drivers/rosserial.git
$	cd ros_workspace
$	catkin_make
$	catkin_make install
$	source ros_workspace/install/setup.bash

# To make only a specific package
$  catkin_make --pkg package_name

```


## XBee Setup

### Installation

Install `rosserial` and `rosserial_xbee` for command line capabilities, this only needs to be done once on any given machine.
```
$	sudo apt-get -y install ros-indigo-rosserial
$	sudo apt-get install ros-indigo-rosserial-xbee
```
### Find the XBee

List all USB devices
```
$	lsusb | grep Future
```
Here are some ways that I have seen them listed 
	`Future Technology Devices International, Ltd FT232 USB-Serial (UART) IC`
	`Future Technology Devices International, Ltd Bridge(I2C/SPI/UART/FIFO)`

### Add the current user to the `dialout` group because that is the only thing that can write to `/dev/ttyUSB0`
```bash
$  sudo adduser $USER dialout
```
### Setup Coordinator and nodes

Use `-C` for Coordinater setup. This is the master node and should always have an id of `0`
```
$	rosrun rosserial_xbee setup_xbee.py -C /dev/ttyUSB0 0
```
Setup any other node `N` where `N = 1,2,3,...`
```
$	rosrun rosserial_xbee setup_xbee.py /dev/ttyUSB0 N
$	rosrun rosserial_xbee setup_xbee.py /dev/ttyUSB0 1 # Node 1 for example
```

Al rosserial_xbee network coordinators should have an ID of 0. It also sets up some default configurations.

*API mode `2` (a binary protocol with escape characters)

*Baud rate `57600 baud`

*Network ID `1331`

*Frequency Channel `D`


Nodes communicate at
-API mode `0`
-Baud rate `57600 baud`

Matts XBee Series 2 (ZigBee) 			013A20040F3BC74
SOAR/Carltons XBee Series 2 (ZigBee)	013A20040F9DFD4

After setting up each node individually, set up the XBee network of nodes. 1 2 here is the first two non-coordinator nodes that were setup.
```
$	rosrun rosserial_xbee xbee_network.py /dev/ttyUSB0 1 2
```
## Servo driver

We will be using the 16-Channel 12-bit PWM/Servo Driver (http://adafru.it/815). Assuming you have the I^2C hooked up correctly between the pi and the board, download the Adafruit PWM Servo Driver library.

```
 git clone https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code.git
 cd Adafruit-Raspberry-Pi-Python-Code
 cd Adafruit_PWM_Servo_Driver 
```

The code should autodetect the I2C address, if it does not either change the section in the code or refer to https://learn.adafruit.com/adafruit-16-channel-servo-driver-with-raspberry-pi/ for further instructions on locating the I2C address and/or resolving errors.

The servos can now be reached in the format of setPWM(self, channel, on, off). The channel is the 16 channels of the servo driver addressed from 0-15. The on and off sets a duty cycle through "ticks" on the pulse from 0 - 4095. An example of this is,

```
 pwm.setPWM(15, 1024, 3072)
```

This will activate the 16th servo at tick 1024 and deactivate it at tick 3072, giving a 50% duty cycle activating on the first quarter of the pulse and deactivating on the third quarter. 

## Arm Navigation (Moveit!)

 If the installation of ROS was a base installation or to download it again follow the code below.
 
 ```
 source /opt/ros/indigo/setup.bash
mkdir moveit
cd moveit
mkdir src
cd src/
wstool init .
wstool merge https://raw.github.com/ros-planning/moveit_docs/indigo-devel/moveit.rosinstall
wstool update
cd ..
```

Then ensure you have the proper dependencies.

```
rosdep install --from-paths src --ignore-src --rosdistro indigo -y
```

Tutorials can be found at: http://moveit.ros.org/documentation/tutorials/


## Transforms

Each robotic component has a unique position within the global map, translating between these positions is done via transforms. In ROS the `tf` package is the standard for keeping track of the changing coordinates. For example, translating between end effector coordinate to the visual sensor stream coordinate requires the use of a transform (e.g (x1, y1, z1) --> (x2, y2, z2)).


First, lets change directory to the highest catkin workspace and create a new package with a generic name like "robot setup transform," let's give it the dependecies `tf`, `roscpp`, and `geometry_msgs`

```
$	cd ros_workspace/src
$	catkin_create_pkg robot_setup_tf tf roscpp geometry_msgs
```

### Creating the Transform Broadcaster (Publisher)

In the `robot_setup_tf` package that was just created lets edit the `robot_setup_tf/src/tf_broadcaster.cpp` file that was created.

We want,

```
# include <ros/ros.h>
# include <tf/transform_broadcaster.h>
```

This calls on the `ros` header as well as the `transform_broadcaster`, ros provides the transform broadcaster to simplify the work of broadcasting transforms.

```
	int main(int argc, char** argv){
		ros::int(argc, argv, "robot_tf_publisher");
		ros::NodeHandle n; 
		ros::Rate r(100); \\ 100 Hz
```

This is just the standard main opener for cpp, along with the ros initialization of the `robot_tf_publisher` node. The NodeHandle opens communication access and when it is shut down, the node will shut down.

The `ros::Rate` attempts to set the hertz, or times per second, for the loop's activation.

```
	tf::TransformBroadcaster broadcaster;
	
	while(n.ok()){
		broadcaster.sendTransform(
		  tf::StampedTransform(
		    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),"End_Effector_Odom", "Kinect_PointCloud"));
		  r.sleep();
		  }
		 }

```

In the final bit of code, we declare the `TransformBroadcaster` object to send our messages across the network. Then we run a loop while the node `n` is "ok." This means it has not been shutdown or received the SIGINT. The loop then uses our broadcaster `TransformerBroadcaster` to send out our transforms. However, we must finally define the features of this transform. 

The first feature of the five features of the transform is the angular variation between the coordinates which is resolved using the Quaternion(pitch, roll, yaw) function, then a three vector(x, y, z) function which represents the vector of displacement from one origin of a coordinate system to another, a function giving the current time at which the transform is applicable, and then the parent to child relationship of the file going from the end effector's frame to the frame of the kinect. Finally, we tell the loop to `r.sleep()`. This tells the function to sleep during the time not required by the rate set earlier.

### Using the Transform (Subscribing) (To be edited - needs comments)

This section of the ReadMe explains the setting up of the transform subscribers.

```
#include <ros/ros.h>
#include <nav_msgs/odometry.h>
#include <sensor_msgs/pointcloud.h>
#include <tf/transform_listener.h>
```

Here, we bring in the tools we will be using, ros itself, the messages to be sent and the `tf/transform_listener.h` to aid us creating a subscriber to the `tf` topic.

```
void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the Kinect_PointCloud frame that we'd like to transform to the End_Effector_Odom frame
  sensor_msgs::PointCloud kinect_point;
  kinect_point.header.frame_id = "Kinect_PointCloud";

  //we'll just use the most recent transform available for our simple example
  kinect_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  kinect_point.point.x = 1.0;
  kinect_point.point.y = 0.2;
  kinect_point.point.z = 0.0;

  try{
    nav_msgs::Odometry end_effector;
    listener.transformPoint("end_effector_odom", kinect_point, end_effector);

    ROS_INFO("base_kinect: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        kinect_point.point.x, kinect_point.point.y, kinect_point.point.z,
        end_effector.point.x, end_effector.point.y, end_effector.point.z, end_effector.header.stamp.toSec());
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"Kinect_PointCloud\" to \"End_Effector_Odom\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
```


### Building the Code (To be edited)

This section creates the code that was written in the subscriber and publisher files.

We need to open to our CMakeLists.txt file that was created when we used roscreate-pkg and append the following onto the last line:

```
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
add_executable(tf_listener src/tf_listener.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
target_link_libraries(tf_listener ${catkin_LIBRARIES})
```

This activates the appropriate code at runtime and links their libraries to required libraries.

```
$ cd ros_workspace
$ catkin_make
```

`catkin_make` the package.

```
roscore
```

Run ros.

```
rosrun robot_setup_tf tf_broadcaster
rosrun robot_setup_tf tf_listener
```

Initialize our publisher and subscriber.

```
[ INFO] 1248138528.200823000: base_kinect: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1248138528.19
[ INFO] 1248138529.200820000: base_kinect: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1248138529.19
[ INFO] 1248138530.200821000: base_kinect: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1248138530.19
[ INFO] 1248138531.200835000: base_kinect: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1248138531.19
[ INFO] 1248138532.200849000: base_kinect: (1.00, 0.20. 0.00) -----> base_link: (1.10, 0.20, 0.20) at time 1248138532.19
```

Listen.

## Odometry

The odometry of a robotic component details the angular and linear position, velocity, and acceleration of that component. This allows the component to be mapped against a global frame.

### Odometry Transform

The odometry transform allows us to publish the odometry information in the kinect's frame, yielding efficient correspondence with the component's visual position. 

```
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
```

Above we have included ros, as well as our old friend transform broadcaster, and since we will be publishing `nav_msgs/Odometry` we will need to include the relevant header file.

```
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ```
  
In the previous section we are initializing our standard main c++ loop as well as creating a new ros node called `odometry_publisher`. We then open communication with the node using `NodeHandle`.
  
  ```
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
```

Here we have created a ros publisher and a `TransformBroadcaster` for our odometry information to be sent out on the ros network.

```
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
```

Above we set our initial component to origin in the component's local frame.

```
  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;
```

Above is dummy code to make a fake robot drive in a circle, this will need to be replaced with a further section on encoder feedback reading, preferably with and without interrupts, as well as once the hardware is interfaced.

```
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(500.0);
  ```
  
Here we have a few lines relating to the timing of the program. We declare two new variables and assign them to the current time. Additionally we set the rate of our odometry publishing to be 500 Hz.
  
  ```
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
```

While the node is in an okay state, lets use `spinOnce()` to check for any outstanding callbacks. Then lets update the current time variable to account for how long that callback may have taken.

```
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;
```

Lets compute the changes in our variables using a standard approach. We would want our updating velocities to input into these variables for the expression of change.

```
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
```

To smoothly handle 3D information we use Quaternions.

```
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "end_effector_odom";
```

Above we publish the information over tf, stamping it with the time and frames.

```
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
```

In the above passage we set the transform odometry to the current odometry and publish it using our broadcaster from earlier.

```
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
```

Now we want to publish the odometry over ros so we create the `odom` navagation message and stamp it's header with the current time and the frame it is coming from.

```
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "end_effector_odom";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
```

Finally, we fill in the ros message with the odometry data and publish it on the network.

```
    last_time = current_time;
    r.sleep();
  }
}
```
End the program and let the rate fill in the time gaps with the appropriate Hz.
