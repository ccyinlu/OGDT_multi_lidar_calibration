#!/usr/bin/env python

import time, sys, os
import argparse

from ros import rosbag
import roslib, rospy
roslib.load_manifest('sensor_msgs')

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import numpy as np 

def sync2bag():
    parser = argparse.ArgumentParser(description='Convert the synced lidar point cloud to rosbag')
    parser.add_argument('outdir', help='output dir for outputbag')
    parser.add_argument("dir", nargs = "?", default = os.getcwd(), help = "base directory of the dataset, if no directory passed the deafult is current working directory")
    parser.add_argument("-f", "--first_id", default=0, help = "first id of the sequence (between 0 - last).")
    parser.add_argument("-l", "--last_id", default=1, help = "last id of the sequence (between 0 - last).")

    args = parser.parse_args()

    first_id = int(args.first_id)
    last_id = int(args.last_id)

    lidar_front_topic = "/lidar2/points_raw"
    lidar_left_topic = "/lidar1/points_raw"
    lidar_right_topic = "/lidar3/points_raw"
    lidar_back_topic = "/lidar4/points_raw"

    compression = rosbag.Compression.NONE
    bag_filename = args.outdir + "/" + "multi_lidar_calib.bag"

    lidar_front_bin_path = args.dir + "/" + "lidar_front_bin"
    lidar_left_bin_path = args.dir + "/" + "lidar_left_bin"
    lidar_right_bin_path = args.dir + "/" + "lidar_right_bin"
    lidar_back_bin_path = args.dir + "/" + "lidar_back_bin"

    lidar_front_num = len(os.listdir(lidar_front_bin_path))

    if last_id > lidar_front_num:
        print("last id must be smaller then " + "{:0>2d}".format(lidar_front_num))
        exit()

    bag = rosbag.Bag(bag_filename, 'w', compression=compression)

    frame_id = 0
    frame_rate = 10.0

    lidar_fields = [  PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('intensity', 12, PointField.UINT16, 1),
            ]

    for index in range(first_id, last_id):
        print("current index: " + "{:0>6d}".format(index) + "[{:0>6d}-{:0>6d}]".format(first_id, last_id))
        frame_id = frame_id + 1
        lidar_front_bin_filename = lidar_front_bin_path + "/" + "{:0>6d}".format(index) + ".bin"
        lidar_left_bin_filename = lidar_left_bin_path + "/" + "{:0>6d}".format(index) + ".bin"
        lidar_right_bin_filename = lidar_right_bin_path + "/" + "{:0>6d}".format(index) + ".bin"
        lidar_back_bin_filename = lidar_back_bin_path + "/" + "{:0>6d}".format(index) + ".bin"

        stamp = rospy.rostime.Time.from_seconds(frame_id/frame_rate)

        # read the lidar file bin
        lidar_front_points = np.fromfile(lidar_front_bin_filename, dtype = np.float32).reshape(-1, 4)
        lidar_front_header = Header()
        lidar_front_header.frame_id = "ls_front"
        lidar_front_header.stamp = stamp
        lidar_front_pc2 = point_cloud2.create_cloud(lidar_front_header, lidar_fields, lidar_front_points)
        bag.write(lidar_front_topic, lidar_front_pc2, stamp)

        lidar_left_points = np.fromfile(lidar_left_bin_filename, dtype = np.float32).reshape(-1, 4)
        lidar_left_header = Header()
        lidar_left_header.frame_id = "ls_left"
        lidar_left_header.stamp = stamp
        lidar_left_pc2 = point_cloud2.create_cloud(lidar_left_header, lidar_fields, lidar_left_points)
        bag.write(lidar_left_topic, lidar_left_pc2, stamp)

        lidar_right_points = np.fromfile(lidar_right_bin_filename, dtype = np.float32).reshape(-1, 4)
        lidar_right_header = Header()
        lidar_right_header.frame_id = "ls_right"
        lidar_right_header.stamp = stamp
        lidar_right_pc2 = point_cloud2.create_cloud(lidar_right_header, lidar_fields, lidar_right_points)
        bag.write(lidar_right_topic, lidar_right_pc2, stamp)

        lidar_back_points = np.fromfile(lidar_back_bin_filename, dtype = np.float32).reshape(-1, 4)
        lidar_back_header = Header()
        lidar_back_header.frame_id = "ls_back"
        lidar_back_header.stamp = stamp
        lidar_back_pc2 = point_cloud2.create_cloud(lidar_back_header, lidar_fields, lidar_back_points)
        bag.write(lidar_back_topic, lidar_back_pc2, stamp)

    bag.close()

if __name__ == "__main__":
    sync2bag()