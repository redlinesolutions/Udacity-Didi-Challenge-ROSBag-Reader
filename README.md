# Udacity-Didi-Challenge-ROSBag-Reader
Reads and displays values and images in a ROSBag file

## System Requirements

* Ubuntu 16.04 (native or via Docker)

## Software Requirements

* ROS
* Python 2.7
* XQuartz (if you are running on Mac via Docker)

## Required Python packages

Most of these can be installed with a simple 'pip install MODULE' command

* numpy
* opencv (pip install opencv-python)
* pygame
* rosbag
* rospkg
* cv_bridge (apt-get install ros-kinetic-cv-bridge)

## Command Line

Basic command:

`python view_rosbag_video.py --dataset <BAGFILE>`

To run via Docker on Mac:

* Make sure you have XQuartz running
* Install socat on your Mac (brew install socat)
* Run socat on your Mac host

`socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\"`

* Find out your host machine's IP address using 'ifconfig'
* Setup your DISPLAY environment to point to your host's XQuartz

`export DISPLAY=<HOST_IP_ADDRESS>:0`    
