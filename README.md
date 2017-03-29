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
* pyglet
* rosbag
* rospkg
* cv_bridge (apt-get install ros-kinetic-cv-bridge)

## KITTI to ROSbag conversion

Install Kitti2Bag

'pip install kitti2bag'

To convert, follow the directions on the Github page. Pay attention to the how the zip files are unpacked.

https://github.com/tomas789/kitti2bag

## Command Line

Basic command:

`python view_rosbag_video.py --dataset <BAGFILE>`

For visualizing only certain topics

`python view_rosbag_video.py --dataset <BAGFILE> --topics <TOPIC1>,<TOPIC2>,...`

To run via Docker on Mac:

* Install XQuartz running
* Make sure you have updated XQuartz settings for GLX

`defaults write org.macosforge.xquartz.X11 enable_iglx -bool true`

* Log Out and Log back Into Mac session
* Install socat on your Mac (brew install socat)

Then these steps once per Mac session:

* Run socat on your Mac host

`socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\"`

* Find out your host machine's IP address using 'ifconfig'
* Within your Docker container, setup your DISPLAY environment to point to your host's XQuartz

`export DISPLAY=<HOST_IP_ADDRESS>:0`    
