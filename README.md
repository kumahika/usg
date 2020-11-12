# usg package
This is a ROS package for controlling the USG.

# 0. Video clips
<iframe width="560" height="315" src="https://www.youtube.com/embed/FSWc5AsxabE" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

# 1. Features
- Constant output of thrust.
- Continuous output of thrust.

**Thruster (Continuous control by DS4)**
![thruster](https://github.com/kumahika/usg/blob/master/img/continous_control.gif)

# 2. Installation

## 2.1 Hardware
This package assumes connecting a thruster and gripper to **Rasberry Pi GPIO**.
Please refer to the following sites for parts installation and circuit wiring.
- Thruster (use 13pin)
  - https://bluerobotics.com/store/thrusters/t200-thruster/

EX) Thruster circuit
![ex](https://github.com/kumahika/usg/blob/master/img/USM%20unit.png)

## 2.2 Software
**Step1 Make the raspi storage to a new SD card**

You can download raspi img file zip from [here](https://drive.google.com/drive/folders/1_n5wU1cHDeiD3O8nzzONRza-W1F2eZ3_?usp=basharing).
```bash
# Check your device path
$ sudo fdisk -l
  /dev/mmcblk0
# Unzip the img file and copy to a new SD card.
$ gzip -dc /home/user name/comapanion_ROS.gz | sudo dd bs=1M of=/dev/mmcblk0
```
If you've already had a raspi warkspace, you can skip above process.
But, in that case, don't forget edit `.bashrc` and git clone this repo.
Set your laptop's IP address to `192.168.2.1`
```bash
# ssh conection to Rasberrypi.
$ ssh pi@192.168.2.2
type passward (default is **companion**)

# In .bashrc file, add the following
$ vim .bashrc
export LD_LIBRARY_PATH=/usr/local/lib/
export PATH=$PATH:~/.local/bin
source /opt/ros/kinetic/setup.bash
export ROS_IP=192.168.2.2
export export ROS_MASTER_URI=http://192.168.2.1:11311
```

**Step2 Initial setting in Raspi**
```bash
# git clone and build
$ cd catkin_ws/src
$ git clone https://github.com/kumahika/usg.git
$ cd ..
$ catkin_make or catkin build

# Grant execute permission
$ cd ~/catkin_ws/src/usg/scripts
$ chmod +x thruster_static.py
# change all scripts that you need
```

# 3. ROS API (Usage)
## Nodes
- `thruster_static` node
 This node can control a thruster by static output.
 The output is fixed according to the rotation speed by adjusting the parameters.

<img src="https://github.com/kumahika/usg/blob/master/img/thrust.png" width="400"><img src="https://github.com/kumahika/usg/blob/master/img/spec.png" width="400">

- `thruster_dynamic` node

 The debug of this node is being sorted out now.


EX)
```bash
# In the laptop
$ export ROS_IP=192.168.2.1
$ export export ROS_MASTER_URI=http://192.168.2.1:11311
$ roscore

# In the raspi
# Execution of a thruster (Static output)
$ roslaunch usg thruster_static.launch

# Execution of a thruster (Continuous control by DS4)
$ roslaunch usg thruster_dynamic.launch

```

<!--If you use it with joy_screw_controller, you can control screw easily using DS4
<img src="https://github.com/naoki-sh/ruri/blob/master/img/simple_controller.png" width="750">
-->

## Topics
- thruster/pwm (std_msgs/int64) : Sequential commands
  - Stop : 0
  - Incriment : 1
  - Decriment : 2

## Prameters
- /thruster/flag_vel (int, default: 2) : The command for the desired thrust (see table above)

EX)
```bash
$ rosparam set /thruster/flag_vel '2'
```

# Author/Contributors
[Hikaru Kumamoto](https://kumahika.github.io/research/)

[Naoki Shirakura](https://github.com/naoki-sh)   
