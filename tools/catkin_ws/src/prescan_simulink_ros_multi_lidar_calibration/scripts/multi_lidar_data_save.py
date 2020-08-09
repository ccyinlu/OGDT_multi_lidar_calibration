#!/usr/bin/env python
import rospy
import argparse
# ROS Image message
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np

from tf.transformations import *
import math
import time

import os
import math
import json

import numpy as np

ros_to_pcl_method = None

save_dir_lidar_front = None
save_dir_lidar_left = None
save_dir_lidar_right = None
save_dir_lidar_back = None

save_dir_vehicle_status_pose = None

lidar_front_pose_relative_received = False
lidar_front_pose_relative_fp = None

lidar_left_pose_relative_received = False
lidar_left_pose_relative_fp = None

lidar_right_pose_relative_received = False
lidar_right_pose_relative_fp = None

lidar_back_pose_relative_received = False
lidar_back_pose_relative_fp = None

def ros_to_pcl(ros_cloud, method="fromList"):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZI
    
        Args:
            ros_cloud (PointCloud2): ROS PointCloud2 message
            
        Returns:
            pcl.PointCloud_PointXYZI: PCL XYZI point cloud
    """
    if method == "fromList":
        points_list = []

        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2], data[3]])

        pcl_data = pcl.PointCloud_PointXYZI()
        pcl_data.from_list(points_list)
    elif method == "fromNumpy":
        pc = ros_numpy.numpify(ros_cloud)
        points=np.zeros((pc.shape[0],4))
        points[:,0]=pc['x']
        points[:,1]=pc['y']
        points[:,2]=pc['z']
        points[:,3]=pc['intensity']
        pcl_data = pcl.PointCloud_PointXYZI(np.array(points, dtype = np.float32))

    return pcl_data 
   
def lidar_front_callback(msg):
    global save_dir_lidar_front
    try:
        # Convert your ROS pointCloud2 to PCL
        current_pointCloud_front = ros_to_pcl(msg, ros_to_pcl_method)
    except e:
        print(e)
    else:
        rospy.loginfo("lidar_front message received")
        lidar_front_file_path = os.path.join(save_dir_lidar_front, '%08.4f.pcd' % (msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0))
        pcl.save(current_pointCloud_front, lidar_front_file_path)

def lidar_left_callback(msg):
    global save_dir_lidar_left
    try:
        # Convert your ROS pointCloud2 to PCL
        current_pointCloud_left = ros_to_pcl(msg, ros_to_pcl_method)
    except e:
        print(e)
    else:
        rospy.loginfo("lidar_left message received")
        lidar_left_file_path = os.path.join(save_dir_lidar_left, '%08.4f.pcd' % (msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0))
        pcl.save(current_pointCloud_left, lidar_left_file_path)

def lidar_right_callback(msg):
    global save_dir_lidar_right
    try:
        # Convert your ROS pointCloud2 to PCL
        current_pointCloud_right = ros_to_pcl(msg, ros_to_pcl_method)
    except e:
        print(e)
    else:
        rospy.loginfo("lidar_right message received")
        lidar_right_file_path = os.path.join(save_dir_lidar_right, '%08.4f.pcd' % (msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0))
        pcl.save(current_pointCloud_right, lidar_right_file_path)

def lidar_back_callback(msg):
    global save_dir_lidar_back
    try:
        # Convert your ROS pointCloud2 to PCL
        current_pointCloud_back = ros_to_pcl(msg, ros_to_pcl_method)
    except e:
        print(e)
    else:
        rospy.loginfo("lidar_back message received")
        lidar_back_file_path = os.path.join(save_dir_lidar_back, '%08.4f.pcd' % (msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0))
        pcl.save(current_pointCloud_back, lidar_back_file_path)

def vehicle_status_pose_callback(msg):
    global save_dir_vehicle_status_pose
    rospy.loginfo("vehicle status pose message received")
    # save the vehicle pose to the file
    vehicle_status_pose_file_path = os.path.join(save_dir_vehicle_status_pose, '%08.4f.txt' % (msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0))
    vehicle_status_pose_relative_fp = open(vehicle_status_pose_file_path, 'w')
    if(vehicle_status_pose_relative_fp != None):
        vehicle_status_pose_relative_fp.write("#position unit meter, orientation unit degree\n")
        vehicle_status_pose_relative_fp.write("x: %.4f\n"%msg.pose.position.x)
        vehicle_status_pose_relative_fp.write("y: %.4f\n"%msg.pose.position.y)
        vehicle_status_pose_relative_fp.write("z: %.4f\n"%msg.pose.position.z)
        vehicle_quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        [vehicle_roll, vehicle_pitch, vehicle_yaw] = euler_from_quaternion(vehicle_quat)
        vehicle_status_pose_relative_fp.write("roll: %.4f\n"%(vehicle_roll * 180.0/math.pi))
        vehicle_status_pose_relative_fp.write("pitch: %.4f\n"%(vehicle_pitch * 180.0/math.pi))
        vehicle_status_pose_relative_fp.write("yaw: %.4f\n"%(vehicle_yaw * 180.0/math.pi))
        vehicle_status_pose_relative_fp.close()
    
def lidar_front_pose_relative_callback(msg):
    global lidar_front_pose_relative_received
    global lidar_front_pose_relative_fp
    if lidar_front_pose_relative_received == False:
        lidar_front_pose_relative_received = True
        rospy.loginfo("lidar_front_pose_relative_received message received")
        
        lidar_front_pose_relative_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        [roll_, pitch_, yaw_] = euler_from_quaternion(lidar_front_pose_relative_quat)
        
        # save the lidar_front relative pose to the file
        if(lidar_front_pose_relative_fp != None):
            lidar_front_pose_relative_fp.write("#position unit meter, orientation unit degree\n")
            lidar_front_pose_relative_fp.write("x: %.4f\n"%(msg.position.x))
            lidar_front_pose_relative_fp.write("y: %.4f\n"%(msg.position.y))
            lidar_front_pose_relative_fp.write("z: %.4f\n"%(msg.position.z))
            lidar_front_pose_relative_fp.write("roll: %.4f\n"%(roll_ * 180.0/math.pi))
            lidar_front_pose_relative_fp.write("pitch: %.4f\n"%(pitch_ * 180.0/math.pi))
            lidar_front_pose_relative_fp.write("yaw: %.4f\n"%(yaw_ * 180.0/math.pi))
            lidar_front_pose_relative_fp.close()
            rospy.loginfo("lidar_front_pose_relative_fp message has been writen to the file")

def lidar_left_pose_relative_callback(msg):
    global lidar_left_pose_relative_received
    global lidar_left_pose_relative_fp
    if lidar_left_pose_relative_received == False:
        lidar_left_pose_relative_received = True
        rospy.loginfo("lidar_left_pose_relative_received message received")
        
        lidar_left_pose_relative_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        [roll_, pitch_, yaw_] = euler_from_quaternion(lidar_left_pose_relative_quat)
        
        # save the lidar_left relative pose to the file
        if(lidar_left_pose_relative_fp != None):
            lidar_left_pose_relative_fp.write("#position unit meter, orientation unit degree\n")
            lidar_left_pose_relative_fp.write("x: %.4f\n"%(msg.position.x))
            lidar_left_pose_relative_fp.write("y: %.4f\n"%(msg.position.y))
            lidar_left_pose_relative_fp.write("z: %.4f\n"%(msg.position.z))
            lidar_left_pose_relative_fp.write("roll: %.4f\n"%(roll_ * 180.0/math.pi))
            lidar_left_pose_relative_fp.write("pitch: %.4f\n"%(pitch_ * 180.0/math.pi))
            lidar_left_pose_relative_fp.write("yaw: %.4f\n"%(yaw_ * 180.0/math.pi))
            lidar_left_pose_relative_fp.close()
            rospy.loginfo("lidar_left_pose_relative_fp message has been writen to the file")

def lidar_right_pose_relative_callback(msg):
    global lidar_right_pose_relative_received
    global lidar_right_pose_relative_fp
    if lidar_right_pose_relative_received == False:
        lidar_right_pose_relative_received = True
        rospy.loginfo("lidar_right_pose_relative_received message received")
        
        lidar_right_pose_relative_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        [roll_, pitch_, yaw_] = euler_from_quaternion(lidar_right_pose_relative_quat)
        
        # save the lidar_right relative pose to the file
        if(lidar_right_pose_relative_fp != None):
            lidar_right_pose_relative_fp.write("#position unit meter, orientation unit degree\n")
            lidar_right_pose_relative_fp.write("x: %.4f\n"%(msg.position.x))
            lidar_right_pose_relative_fp.write("y: %.4f\n"%(msg.position.y))
            lidar_right_pose_relative_fp.write("z: %.4f\n"%(msg.position.z))
            lidar_right_pose_relative_fp.write("roll: %.4f\n"%(roll_ * 180.0/math.pi))
            lidar_right_pose_relative_fp.write("pitch: %.4f\n"%(pitch_ * 180.0/math.pi))
            lidar_right_pose_relative_fp.write("yaw: %.4f\n"%(yaw_ * 180.0/math.pi))
            lidar_right_pose_relative_fp.close()
            rospy.loginfo("lidar_right_pose_relative_fp message has been writen to the file")

def lidar_back_pose_relative_callback(msg):
    global lidar_back_pose_relative_received
    global lidar_back_pose_relative_fp
    if lidar_back_pose_relative_received == False:
        lidar_back_pose_relative_received = True
        rospy.loginfo("lidar_back_pose_relative_received message received")
        
        lidar_back_pose_relative_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        [roll_, pitch_, yaw_] = euler_from_quaternion(lidar_back_pose_relative_quat)
        
        # save the lidar_back relative pose to the file
        if(lidar_back_pose_relative_fp != None):
            lidar_back_pose_relative_fp.write("#position unit meter, orientation unit degree\n")
            lidar_back_pose_relative_fp.write("x: %.4f\n"%(msg.position.x))
            lidar_back_pose_relative_fp.write("y: %.4f\n"%(msg.position.y))
            lidar_back_pose_relative_fp.write("z: %.4f\n"%(msg.position.z))
            lidar_back_pose_relative_fp.write("roll: %.4f\n"%(roll_ * 180.0/math.pi))
            lidar_back_pose_relative_fp.write("pitch: %.4f\n"%(pitch_ * 180.0/math.pi))
            lidar_back_pose_relative_fp.write("yaw: %.4f\n"%(yaw_ * 180.0/math.pi))
            lidar_back_pose_relative_fp.close()
            rospy.loginfo("lidar_back_pose_relative_fp message has been writen to the file")
    
if __name__ == '__main__':
    rospy.init_node('multi_lidar_data_saver')

    ##################################################
    # Parameter handling                             #
    ##################################################
    data_root = rospy.get_param('~data_root', '/home/bingo/ethan/multi-lidar-calibration/Dataset/prescan/scene1_gt')

    lidar_front_topic = rospy.get_param('~lidar_front_topic', '/lidar2/points_raw')
    lidar_left_topic = rospy.get_param('~lidar_left_topic', '/lidar1/points_raw')
    lidar_right_topic = rospy.get_param('~lidar_right_topic', '/lidar3/points_raw')
    lidar_back_topic = rospy.get_param('~lidar_back_topic', '/lidar4/points_raw')

    vehicle_status_pose_topic = rospy.get_param('~vehicle_status_pose_topic', '/vehicle_status_pose')

    lidar_front_pose_relative_topic = rospy.get_param('~lidar_front_pose_relative_topic', '/lidar_front_pose_relative')
    lidar_left_pose_relative_topic = rospy.get_param('~lidar_left_pose_relative_topic', '/lidar_left_pose_relative')
    lidar_right_pose_relative_topic = rospy.get_param('~lidar_right_pose_relative_topic', '/lidar_right_pose_relative')
    lidar_back_pose_relative_topic = rospy.get_param('~lidar_back_pose_relative_topic', '/lidar_back_pose_relative')

    start_index = rospy.get_param('~start_index', 0);
    ros_to_pcl_method = rospy.get_param('~ros_to_pcl_method', "fromNumpy");

    # mkdir or touch file for the received message
    save_dir_lidar_front = os.path.join(data_root, 'lidar_front')
    save_dir_lidar_left = os.path.join(data_root, 'lidar_left')
    save_dir_lidar_right = os.path.join(data_root, 'lidar_right')
    save_dir_lidar_back = os.path.join(data_root, 'lidar_back')

    save_dir_vehicle_status_pose = os.path.join(data_root, 'vehicle_pose')

    save_file_lidar_front_pose_relative = os.path.join(data_root, 'lidar_front_pose_relative.txt')
    save_file_lidar_left_pose_relative = os.path.join(data_root, 'lidar_left_pose_relative.txt')
    save_file_lidar_right_pose_relative = os.path.join(data_root, 'lidar_right_pose_relative.txt')
    save_file_lidar_back_pose_relative = os.path.join(data_root, 'lidar_back_pose_relative.txt')

    # create the dir if the dir not exist
    if not os.path.exists(save_dir_lidar_front):
        os.makedirs(save_dir_lidar_front)
    if not os.path.exists(save_dir_lidar_left):
        os.makedirs(save_dir_lidar_left)
    if not os.path.exists(save_dir_lidar_right):
        os.makedirs(save_dir_lidar_right)
    if not os.path.exists(save_dir_lidar_back):
        os.makedirs(save_dir_lidar_back)

    if not os.path.exists(save_dir_vehicle_status_pose):
        os.makedirs(save_dir_vehicle_status_pose)

    lidar_front_pose_relative_fp = open(save_file_lidar_front_pose_relative, 'w')
    lidar_left_pose_relative_fp = open(save_file_lidar_left_pose_relative, 'w')
    lidar_right_pose_relative_fp = open(save_file_lidar_right_pose_relative, 'w')
    lidar_back_pose_relative_fp = open(save_file_lidar_back_pose_relative, 'w')

    # Set up your subscriber and define its callback
    rospy.Subscriber(lidar_front_topic, PointCloud2, lidar_front_callback)
    rospy.Subscriber(lidar_left_topic, PointCloud2, lidar_left_callback)
    rospy.Subscriber(lidar_right_topic, PointCloud2, lidar_right_callback)
    rospy.Subscriber(lidar_back_topic, PointCloud2, lidar_back_callback)

    rospy.Subscriber(vehicle_status_pose_topic, PoseStamped, vehicle_status_pose_callback)

    rospy.Subscriber(lidar_front_pose_relative_topic, Pose, lidar_front_pose_relative_callback)
    rospy.Subscriber(lidar_left_pose_relative_topic, Pose, lidar_left_pose_relative_callback)
    rospy.Subscriber(lidar_right_pose_relative_topic, Pose, lidar_right_pose_relative_callback)
    rospy.Subscriber(lidar_back_pose_relative_topic, Pose, lidar_back_pose_relative_callback)

    # Spin until ctrl + c
    rospy.spin()