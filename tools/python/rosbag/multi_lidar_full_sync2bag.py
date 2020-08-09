#!/usr/bin/env python

import time, sys, os
import argparse

from ros import rosbag
import roslib, rospy
roslib.load_manifest('sensor_msgs')

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

import numpy as np 
import yaml

from tf.transformations import *
import math

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
    gnss_pose_topic = "/gnss_pose"

    lidar_concate_topic = "/concate_points_raw"

    compression = rosbag.Compression.NONE
    bag_filename = args.outdir + "/" + "multi_lidar_calib.bag"

    lidar_front_bin_path = args.dir + "/" + "lidar_front_bin"
    lidar_left_bin_path = args.dir + "/" + "lidar_left_bin"
    lidar_right_bin_path = args.dir + "/" + "lidar_right_bin"
    lidar_back_bin_path = args.dir + "/" + "lidar_back_bin"
    gnss_pose_path = args.dir + "/" + "vehicle_pose"

    lidar_1_to_2_gt_6dof_path = args.dir
    lidar_3_to_2_gt_6dof_path = args.dir
    lidar_4_to_2_gt_6dof_path = args.dir

    #####################################################################################################################
    # load the extrinsic parameters and form the corresponding transformation matrix
    lidar_1_to_2_gt_6dof_filename = lidar_1_to_2_gt_6dof_path + "/" + "lidar_1_to_2_gt_6dof.txt"
    lidar_3_to_2_gt_6dof_filename = lidar_3_to_2_gt_6dof_path + "/" + "lidar_3_to_2_gt_6dof.txt"
    lidar_4_to_2_gt_6dof_filename = lidar_4_to_2_gt_6dof_path + "/" + "lidar_4_to_2_gt_6dof.txt"
    with open(lidar_1_to_2_gt_6dof_filename) as f:
      lidar_1_to_2_gt_6dof = yaml.load(f)
    with open(lidar_3_to_2_gt_6dof_filename) as f:
      lidar_3_to_2_gt_6dof = yaml.load(f)
    with open(lidar_4_to_2_gt_6dof_filename) as f:
      lidar_4_to_2_gt_6dof = yaml.load(f)

    lidar_1_to_2_T = translation_matrix((float(lidar_1_to_2_gt_6dof['x']), float(lidar_1_to_2_gt_6dof['y']), float(lidar_1_to_2_gt_6dof['z'])))
    lidar_1_to_2_R = euler_matrix(float(lidar_1_to_2_gt_6dof['roll'])*math.pi/180, float(lidar_1_to_2_gt_6dof['pitch'])*math.pi/180, float(lidar_1_to_2_gt_6dof['yaw'])*math.pi/180, 'sxyz')
    lidar_1_to_2_M = concatenate_matrices(lidar_1_to_2_T, lidar_1_to_2_R)

    lidar_3_to_2_T = translation_matrix((float(lidar_3_to_2_gt_6dof['x']), float(lidar_3_to_2_gt_6dof['y']), float(lidar_3_to_2_gt_6dof['z'])))
    lidar_3_to_2_R = euler_matrix(float(lidar_3_to_2_gt_6dof['roll'])*math.pi/180, float(lidar_3_to_2_gt_6dof['pitch'])*math.pi/180, float(lidar_3_to_2_gt_6dof['yaw'])*math.pi/180, 'sxyz')
    lidar_3_to_2_M = concatenate_matrices(lidar_3_to_2_T, lidar_3_to_2_R)

    lidar_4_to_2_T = translation_matrix((float(lidar_4_to_2_gt_6dof['x']), float(lidar_4_to_2_gt_6dof['y']), float(lidar_4_to_2_gt_6dof['z'])))
    lidar_4_to_2_R = euler_matrix(float(lidar_4_to_2_gt_6dof['roll'])*math.pi/180, float(lidar_4_to_2_gt_6dof['pitch'])*math.pi/180, float(lidar_4_to_2_gt_6dof['yaw'])*math.pi/180, 'sxyz')
    lidar_4_to_2_M = concatenate_matrices(lidar_4_to_2_T, lidar_4_to_2_R)

    #####################################################################################################################

    # print(lidar_1_to_2_M)

    lidar_front_num = len(os.listdir(lidar_front_bin_path))

    if last_id > lidar_front_num:
        print("last id must be smaller then " + "{:0>2d}".format(lidar_front_num))
        exit()

    bag = rosbag.Bag(bag_filename, 'w', compression=compression)

    frame_id = 0
    frame_rate = 10.0

    # lidar_fields = [  PointField('x', 0, PointField.FLOAT32, 1),
    #             PointField('y', 4, PointField.FLOAT32, 1),
    #             PointField('z', 8, PointField.FLOAT32, 1),
    #             PointField('intensity', 12, PointField.UINT16, 1),
    #         ]
    
    lidar_fields = [  PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('intensity', 12, PointField.FLOAT32, 1),
            ]

    # # just for test
    # lidar_left_bin_filename = lidar_left_bin_path + "/" + "{:0>6d}".format(1) + ".bin"
    # lidar_left_points = np.fromfile(lidar_left_bin_filename, dtype = np.float32).reshape(-1, 4)
    # lidar_left_points_hom = lidar_left_points.copy()
    # lidar_left_points_hom[..., 3] = 1
    # # print(lidar_left_points)

    # lidar_1_to_2_points_hom = np.dot(lidar_1_to_2_M, lidar_left_points_hom.T)
    # lidar_1_to_2_points = lidar_1_to_2_points_hom.T
    # lidar_1_to_2_points[..., 3] = lidar_left_points[..., 3]
    # print(lidar_1_to_2_points)

    # points = np.concatenate((lidar_1_to_2_points, lidar_left_points),axis=0)
    # print(points)

    for index in range(first_id, last_id):
        print("current index: " + "{:0>6d}".format(index) + "[{:0>6d}-{:0>6d}]".format(first_id, last_id))
        frame_id = frame_id + 1
        lidar_front_bin_filename = lidar_front_bin_path + "/" + "{:0>6d}".format(index) + ".bin"
        lidar_left_bin_filename = lidar_left_bin_path + "/" + "{:0>6d}".format(index) + ".bin"
        lidar_right_bin_filename = lidar_right_bin_path + "/" + "{:0>6d}".format(index) + ".bin"
        lidar_back_bin_filename = lidar_back_bin_path + "/" + "{:0>6d}".format(index) + ".bin"
        gnss_pose_filename = gnss_pose_path + "/" + "{:0>6d}".format(index) + ".txt"

        stamp = rospy.rostime.Time.from_seconds(frame_id/frame_rate)

        with open(gnss_pose_filename) as f:
          gnss_pose_6dof = yaml.load(f)

        gnss_pose_header = Header()
        gnss_pose_header.frame_id = "gnss"
        gnss_pose_header.stamp = stamp
        gnss_poseStamp = PoseStamped()
        gnss_poseStamp.header = gnss_pose_header
        gnss_poseStamp.pose.position.x = gnss_pose_6dof['x']
        gnss_poseStamp.pose.position.y = gnss_pose_6dof['y']
        gnss_poseStamp.pose.position.z = gnss_pose_6dof['z']
        quat = quaternion_from_euler(float(gnss_pose_6dof['roll'])*math.pi/180, float(gnss_pose_6dof['pitch'])*math.pi/180, float(gnss_pose_6dof['yaw'])*math.pi/180, axes='sxyz')
        gnss_poseStamp.pose.orientation.x = quat[0]
        gnss_poseStamp.pose.orientation.y = quat[1]
        gnss_poseStamp.pose.orientation.z = quat[2]
        gnss_poseStamp.pose.orientation.w = quat[3]

        bag.write(gnss_pose_topic, gnss_poseStamp, stamp)

        # read the lidar file bin
        lidar_front_points = np.fromfile(lidar_front_bin_filename, dtype = np.float32).reshape(-1, 4)
        # print(lidar_front_points)
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

        # concate the point cloud from front, left, right and back
        # transform left points to front coordinates
        lidar_left_points_hom = lidar_left_points.copy()
        lidar_left_points_hom[..., 3] = 1
        lidar_1_to_2_points_hom = np.dot(lidar_1_to_2_M, lidar_left_points_hom.T)
        lidar_1_to_2_points = lidar_1_to_2_points_hom.T
        lidar_1_to_2_points[..., 3] = lidar_left_points[..., 3]

        # transform right points to front coordinates
        lidar_right_points_hom = lidar_right_points.copy()
        lidar_right_points_hom[..., 3] = 1
        lidar_3_to_2_points_hom = np.dot(lidar_3_to_2_M, lidar_right_points_hom.T)
        lidar_3_to_2_points = lidar_3_to_2_points_hom.T
        lidar_3_to_2_points[..., 3] = lidar_right_points[..., 3]

        # transform back points to front coordinates
        lidar_back_points_hom = lidar_back_points.copy()
        lidar_back_points_hom[..., 3] = 1
        lidar_4_to_2_points_hom = np.dot(lidar_4_to_2_M, lidar_back_points_hom.T)
        lidar_4_to_2_points = lidar_4_to_2_points_hom.T
        lidar_4_to_2_points[..., 3] = lidar_back_points[..., 3]

        lidar_concate_points = np.concatenate((lidar_front_points, lidar_1_to_2_points),axis=0)
        lidar_concate_points = np.concatenate((lidar_concate_points, lidar_3_to_2_points),axis=0)
        lidar_concate_points = np.concatenate((lidar_concate_points, lidar_4_to_2_points),axis=0)
        lidar_concate_header = Header()
        lidar_concate_header.frame_id = "ls_concate"
        lidar_concate_header.stamp = stamp
        lidar_concate_pc2 = point_cloud2.create_cloud(lidar_concate_header, lidar_fields, lidar_concate_points)
        bag.write(lidar_concate_topic, lidar_concate_pc2, stamp)

    bag.close()

if __name__ == "__main__":
    sync2bag()