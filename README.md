# USFSOAR AGSE

|  Section      | Currently Used|
| ------------- |:-------------:|
|  Hardware     | Raspberry Pi  |
|       OS      | Ubuntu 14.0.4       |
|               |      ROS      |
| zebra stripes | are neat      |

##Usage

Before using any ros commands in any shell first run
```
source /opt/ros/indigo/setup.bash
``` 
# To check environment variables
```
$	printenv | grep ROS
```
 
# Start up the kinect
```
$  	sudo modprobe -r gspca_kinect
```
This ...
```
$  	sudo modprobe -r gspca_main
```
fdsfdsa
```
$  	roslaunch openni_launch openni.launch
```