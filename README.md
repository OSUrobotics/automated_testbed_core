# infrastructure-packages
#### All the Needed packages for the testing Infrastructure


## Setup:

0. Install catkin tools if you havent already: https://catkin-tools.readthedocs.io/en/latest/installing.html

1. Create a ROS workspace and name it infrastructure system 
```
mkdir -p ~/infrastructure_system/src
cd ~/catkin_ws/
catkin build
```
2. Install FlexBE Binaries and clone the app to your src folder
```
sudo apt install ros-$ROS_DISTRO-flexbe-behavior-engine
git clone https://github.com/FlexBE/flexbe_app.git 
```
3. Clone this repository into your src folder

### If you arent using hardware you can stop here otherwise:

4. Install rosserial and arduino: http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

5. Add the custom messages in the infrastucture_msgs package to rosserial: http://wiki.ros.org/rosserial_arduino/Tutorials/Adding%20Custom%20Messages



## How to use:

### Interfacing with an arm
You will need to create your own Moveit class that you can use within the action server located in hardware_flexbe_link its still very much under developement so Im not gonna go into too much detail until we do further testing, if you need to use it or need a Moveit class contact me and I can walk you through.
    
### Launching 
```
roslaunch infrastructure_flexbe_behaviors infrastructure_start_all.launch 

```
This will launch FlexBe and all of the necesary nodes for a full trial. There are 4 arguments you can pass to the launch function for different behavior:
```
collect_data:=true (This activates data collection and rosbags and video recordings (if a camera is plugged in) will be sent to folders in the data collection package

rosbag_path:=<path> (You can use this to change where bagfiles are stored on your machine, make sure to use a full path)
  
video_path:=<path> (You can use this to change where videos are stored on your machine, make sure to use a full path)
  
use_hardware:=true (This activates the rosserial node and allows you to connect with any of the physical hardware. BAUD is set to 57600)
```
They automatically default to false so you have to explicitly state them if you wish to use any combination of these.


### FlexBE
After running the launch file FlexBe will pop up to use the testbed or door:

#### Door
Load Behavior->Door_System_Behavior->Runtime Control->Change number of tests and trials to whatever youd like->Start Execution

#### Testbed
Load Behavior->Grasp_Reset_System_Behavior_Single->Runtime Control->Change number of tests and trials to whatever youd like(If you want the python action server make sure to change the topic from single_stage_as to: single_stage_as_py)->Start Execution


The other Behaviors and states can be used as templates or testing/debugging tools but will require some tinkering.


### You're ready to go!
This is still very much under developement so there will be bugs and things will break. I most likely forgot something on here as well so if you need to use it with an arm or have any questions feel free to contact me at navek@oregonstate.edu. Good Luck!
