#!/usr/bin/python
"""
view_rosbag_video.py: version 0.1.0
Note:
Part of this code was copied and modified from github.com/comma.ai/research (code: BSD License)

Todo:
Update steering angle projection.  Current version is a hack from comma.ai's version
Update enable left, center and right camera selection.  Currently all three cameras are displayed.
Update to enable display of trained steering data (green) as compared to actual (blue projection).

History:
2016/10/06: Update to add --skip option to skip the first X seconds of data from rosbag.
2016/10/02: Initial version to display left, center, right cameras and steering angle.
"""

import pyglet
import argparse
import sys
import numpy as np
import rosbag
import datetime
import sensor_msgs.point_cloud2
from cv_bridge import CvBridge

#from keras.models import model_from_json

appTitle = "Udacity Team-SF: ROSbag viewer"
bridge = CvBridge()
video_max_width=875
size = None
startsec = 0
windowCamera = {}
windowLidar = None
windowRadar = None

def print_msg(msgType, topic, msg, time):

    t = time.to_sec()

    if 'sensor_msgs' in msgType or 'nav_msgs' in msgType:
        since_start = msg.header.stamp.to_sec()-startsec

    if msgType in ['sensor_msgs/PointCloud2']:

        points = sensor_msgs.point_cloud2.read_points(msg, skip_nans=True)
        arrPoints = []
        for point in points:
            pt_x = point[0]
            pt_y = point[1]
            pt_z = point[2]
            arrPoints.append(point)
        print(topic, msg.header.seq, since_start, 'nPoints=', len(arrPoints), t)

    elif msgType in ['sensor_msgs/NavSatFix']:
        print(topic, msg.header.seq, since_start, msg.latitude, msg.longitude, msg.altitude, t)

    elif msgType in ['nav_msgs/Odometry']:
        position = msg.pose.pose.position
        print(topic, msg.header.seq, since_start, position.x, position.y, position.z, t)

    elif topic == '/radar/range':

        print(topic, msg.header.seq, since_start, msg.radiation_type, msg.field_of_view, msg.min_range, msg.max_range, msg.range, t)

    elif msgType == 'sensor_msgs/Image':
        print(topic, msg.header.seq, msg.width, msg.height, since_start, t)
    elif msgType in ['sensor_msgs/CameraInfo']:
        print(topic, msg.header.seq, since_start, msg.width, msg.height, msg.distortion_model, t)
    else:
        pass
        #print(topic, msg.header.seq, t-msg.header.stamp, msg, t)

# ***** main loop *****
if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=appTitle)
  parser.add_argument('--dataset', type=str, default="dataset.bag", help='Dataset/ROS Bag name')
  parser.add_argument('--skip', type=int, default="0", help='skip seconds')
  parser.add_argument('--topics', type=str, default=None, help='Topic list to display')
  args = parser.parse_args()

  dataset = args.dataset
  skip = args.skip
  startsec = 0
  topics_list = args.topics.split(',') if args.topics else None

  print("reading rosbag ", dataset)
  bag = rosbag.Bag(dataset, 'r')
  topicTypesMap = bag.get_type_and_topic_info().topics

  for topic, msg, t in bag.read_messages(topics=topics_list):
    msgType = topicTypesMap[topic].msg_type
    if startsec == 0:
        startsec = t.to_sec()
        if skip < 24*60*60:
            skipping = t.to_sec() + skip
            print("skipping ", skip, " seconds from ", startsec, " to ", skipping, " ...")
        else:
            skipping = skip
            print("skipping to ", skip, " from ", startsec, " ...")
    else:
        window = None
        if t.to_sec() > skipping:
            print_msg(msgType, topic, msg, t)

            if msgType in ['sensor_msgs/Image']:

                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                cv_img = np.flipud(cv_img)

                if (size == None):
                    ratio = float(cv_img.shape[0]) / video_max_width
                    size = (int(ratio * cv_img.shape[1]), int(ratio * cv_img.shape[0]))
                    print(size)
                if (windowCamera.get(topic, None) == None):
                    windowCamera[topic] = pyglet.window.Window(size[0], size[1], caption=topic)

                window = windowCamera[topic]
                img = pyglet.image.ImageData(cv_img.shape[1], cv_img.shape[0], 'RGB', cv_img.tobytes())

            #elif topic in ['sensor_msgs/PointCloud2']:

                # render top down point cloud

            #    if (size == None):
            #        ratio = float(cv_img.shape[0]) / video_max_width
            #        size = (int(ratio * cv_img.shape[1]), int(ratio * cv_img.shape[0]))
            #        print(size)
            #        windowCamera = pyglet.window.Window(size[0], size[1])

            #    window = windowCamera
            #    img = pyglet.image.ImageData(cv_img.shape[1], cv_img.shape[0], 'RGB', cv_img.tobytes())

            #draw_path_on(imgleft, 0, angle_steers*20, (0,0,255), -10)
            #draw_path_on(imgcenter, 0, angle_steers*20, (0,0,255), -20)
            #draw_path_on(imgright, 0, angle_steers*20, (0,0,255), 0)

            if window:
                window.switch_to()
                window.dispatch_events()
                img.blit(0,0,width=size[0], height=size[1])
                window.flip()
