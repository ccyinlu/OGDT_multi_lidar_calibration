#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>

#define PI 3.141592653

class PointsSplitSave4
{
public:
  	PointsSplitSave4();

private:
	// typedef pcl::PointXYZI PointT;
  typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;

	typedef sensor_msgs::PointCloud2 PointCloudMsgT;
	typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT, PointCloudMsgT, PointCloudMsgT> SyncPolicyT;

	ros::NodeHandle node_handle_, private_node_handle_;
	message_filters::Subscriber<PointCloudMsgT> *cloud_subscriber_lidar_front_, *cloud_subscriber_lidar_left_, *cloud_subscriber_lidar_right_, *cloud_subscriber_lidar_back_;
	message_filters::Synchronizer<SyncPolicyT>* cloud_synchronizer_;

	std::string lidar_front_topic_;
	std::string lidar_left_topic_;
	std::string lidar_right_topic_;
	std::string lidar_back_topic_;

	std::string lidar_front_path_;
	std::string lidar_left_path_;
	std::string lidar_right_path_;
  std::string lidar_back_path_;

	int start_index_;

  void pointcloud_callback(const PointCloudMsgT::ConstPtr& cloud_msg_lidar_front, const PointCloudMsgT::ConstPtr& cloud_msg_lidar_left, const PointCloudMsgT::ConstPtr& cloud_msg_lidar_right, const PointCloudMsgT::ConstPtr& cloud_msg_lidar_back);
};

PointsSplitSave4::PointsSplitSave4()
  : node_handle_(), private_node_handle_("~")
{
	ROS_INFO("Inititalizing the params of PointsSplitSave4...");

	private_node_handle_.param<int>("start_index", start_index_, 1);

	private_node_handle_.param("lidar_front_path", lidar_front_path_, std::string(""));
	ROS_INFO("lidar_front_path: %s", lidar_front_path_.c_str());
	private_node_handle_.param("lidar_left_path", lidar_left_path_, std::string(""));
	ROS_INFO("lidar_left_path: %s", lidar_left_path_.c_str());
  private_node_handle_.param("lidar_right_path", lidar_right_path_, std::string(""));
	ROS_INFO("lidar_right_path: %s", lidar_right_path_.c_str());
  private_node_handle_.param("lidar_back_path", lidar_back_path_, std::string(""));
	ROS_INFO("lidar_back_path: %s", lidar_back_path_.c_str());
	
	private_node_handle_.param("lidar_front_topic", lidar_front_topic_, std::string("/lidar2/points_raw"));
	ROS_INFO("lidar front topic: %s", lidar_front_topic_.c_str());
	private_node_handle_.param("lidar_left_topic", lidar_left_topic_, std::string("/lidar1/points_raw"));
	ROS_INFO("lidar left topic: %s", lidar_left_topic_.c_str());
  private_node_handle_.param("lidar_right_topic", lidar_right_topic_, std::string("/lidar3/points_raw"));
	ROS_INFO("lidar right topic: %s", lidar_right_topic_.c_str());
  private_node_handle_.param("lidar_back_topic", lidar_back_topic_, std::string("/lidar4/points_raw"));
	ROS_INFO("lidar back topic: %s", lidar_back_topic_.c_str());
	
	cloud_subscriber_lidar_front_ = new message_filters::Subscriber<PointCloudMsgT>(node_handle_, lidar_front_topic_, 10);
	cloud_subscriber_lidar_left_ = new message_filters::Subscriber<PointCloudMsgT>(node_handle_, lidar_left_topic_, 10);
  cloud_subscriber_lidar_right_ = new message_filters::Subscriber<PointCloudMsgT>(node_handle_, lidar_right_topic_, 10);
  cloud_subscriber_lidar_back_ = new message_filters::Subscriber<PointCloudMsgT>(node_handle_, lidar_back_topic_, 10);
	
	cloud_synchronizer_ =
		new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100), *cloud_subscriber_lidar_front_, *cloud_subscriber_lidar_left_, *cloud_subscriber_lidar_right_, *cloud_subscriber_lidar_back_);
  cloud_synchronizer_->registerCallback(boost::bind(&PointsSplitSave4::pointcloud_callback, this, _1, _2, _3, _4));

  // check the lidar path existance, if not, then create them
  if (access(lidar_front_path_.c_str(), 0) == -1)
	{
    ROS_INFO("lidar_front_path is not existing, now make it");

		int flag=mkdir(lidar_front_path_.c_str(), 0777);

		if (flag == 0)
		{
			ROS_INFO("lidar_front_path made successfully");
		} else {
      ROS_ERROR("lidar_front_path made not successfully");
		}
  }

  if (access(lidar_left_path_.c_str(), 0) == -1)
	{
    ROS_INFO("lidar_left_path is not existing, now make it");

		int flag=mkdir(lidar_left_path_.c_str(), 0777);

		if (flag == 0)
		{
			ROS_INFO("lidar_left_path made successfully");
		} else {
      ROS_ERROR("lidar_left_path made not successfully");
		}
  }

  if (access(lidar_right_path_.c_str(), 0) == -1)
	{
    ROS_INFO("lidar_right_path is not existing, now make it");

		int flag=mkdir(lidar_right_path_.c_str(), 0777);

		if (flag == 0)
		{
			ROS_INFO("lidar_right_path made successfully");
		} else {
      ROS_ERROR("lidar_right_path made not successfully");
		}
  }

  if (access(lidar_back_path_.c_str(), 0) == -1)
	{
    ROS_INFO("lidar_back_path is not existing, now make it");

		int flag=mkdir(lidar_back_path_.c_str(), 0777);

		if (flag == 0)
		{
			ROS_INFO("lidar_back_path made successfully");
		} else {
      ROS_ERROR("lidar_back_path made not successfully");
		}
  }
}


void PointsSplitSave4::pointcloud_callback(const PointCloudMsgT::ConstPtr& cloud_msg_lidar_front,
                                            const PointCloudMsgT::ConstPtr& cloud_msg_lidar_left, 
                                            const PointCloudMsgT::ConstPtr& cloud_msg_lidar_right, 
                                            const PointCloudMsgT::ConstPtr& cloud_msg_lidar_back)
{
	PointCloudT::Ptr cloud_source_lidar_front(new PointCloudT);
  PointCloudT::Ptr cloud_source_lidar_left(new PointCloudT);
  PointCloudT::Ptr cloud_source_lidar_right(new PointCloudT);
  PointCloudT::Ptr cloud_source_lidar_back(new PointCloudT);

	pcl::fromROSMsg(*cloud_msg_lidar_front, *cloud_source_lidar_front);
	pcl::fromROSMsg(*cloud_msg_lidar_left, *cloud_source_lidar_left);
	pcl::fromROSMsg(*cloud_msg_lidar_right, *cloud_source_lidar_right);
	pcl::fromROSMsg(*cloud_msg_lidar_back, *cloud_source_lidar_back);

	// save the points
  char pcd_filename[128];

  sprintf(pcd_filename, "%s/%06d.pcd", lidar_front_path_.c_str(), start_index_);
	if(pcl::io::savePCDFileASCII (pcd_filename, *cloud_source_lidar_front)<0){
    ROS_ERROR("lidar_front index: %d saved error", start_index_);
  }

  sprintf(pcd_filename, "%s/%06d.pcd", lidar_left_path_.c_str(), start_index_);
	if(pcl::io::savePCDFileASCII (pcd_filename, *cloud_source_lidar_left)<0){
    ROS_ERROR("lidar_left index: %d saved error", start_index_);
  }

  sprintf(pcd_filename, "%s/%06d.pcd", lidar_right_path_.c_str(), start_index_);
	if(pcl::io::savePCDFileASCII (pcd_filename, *cloud_source_lidar_right)<0){
    ROS_ERROR("lidar_right index: %d saved error", start_index_);
  }

  sprintf(pcd_filename, "%s/%06d.pcd", lidar_back_path_.c_str(), start_index_);
	if(pcl::io::savePCDFileASCII (pcd_filename, *cloud_source_lidar_back)<0){
    ROS_ERROR("lidar_back index: %d saved error", start_index_);
  }

  start_index_++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PointsSplitSave4");
  PointsSplitSave4 node;
  ros::spin();
  return 0;
}
