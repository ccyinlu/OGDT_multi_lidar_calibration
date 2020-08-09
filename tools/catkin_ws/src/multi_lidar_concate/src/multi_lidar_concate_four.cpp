#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <yaml-cpp/yaml.h>

#define PI 3.141592653

class PointsConcatFilter4
{
public:
  	PointsConcatFilter4();

private:
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;

	typedef sensor_msgs::PointCloud2 PointCloudMsgT;
	typedef message_filters::sync_policies::ApproximateTime<PointCloudMsgT, PointCloudMsgT, PointCloudMsgT, PointCloudMsgT> SyncPolicyT;

	ros::NodeHandle node_handle_, private_node_handle_;
	message_filters::Subscriber<PointCloudMsgT> *cloud_subscriber_parent_, *cloud_subscriber_child_1_, *cloud_subscriber_child_2_, *cloud_subscriber_child_3_;
	message_filters::Synchronizer<SyncPolicyT>* cloud_synchronizer_;
	ros::Publisher concat_cloud_publisher_;

	std::string output_frame_;
	std::string lidar_parent_topic_;
	std::string lidar_child_1_topic_;
	std::string lidar_child_2_topic_;
	std::string lidar_child_3_topic_;
	std::string concat_points_topic_;

	bool use_yaml_config_;
	std::string lidar_child_1_yaml_path_;
	std::string lidar_child_2_yaml_path_;
	std::string lidar_child_3_yaml_path_;

	double transform_child1_2parent_x_;
	double transform_child1_2parent_y_;
	double transform_child1_2parent_z_;
	double transform_child1_2parent_roll_;
	double transform_child1_2parent_pitch_;
	double transform_child1_2parent_yaw_;

	double transform_child2_2parent_x_;
	double transform_child2_2parent_y_;
	double transform_child2_2parent_z_;
	double transform_child2_2parent_roll_;
	double transform_child2_2parent_pitch_;
	double transform_child2_2parent_yaw_;

	double transform_child3_2parent_x_;
	double transform_child3_2parent_y_;
	double transform_child3_2parent_z_;
	double transform_child3_2parent_roll_;
	double transform_child3_2parent_pitch_;
	double transform_child3_2parent_yaw_;

	Eigen::Matrix4f transform_child1_to_parent_;
	Eigen::Matrix4f transform_child2_to_parent_;
	Eigen::Matrix4f transform_child3_to_parent_;

	bool use_intensity_render_;

  	void pointcloud_callback(const PointCloudMsgT::ConstPtr& cloud_msg_parent, const PointCloudMsgT::ConstPtr& cloud_msg_child_1, const PointCloudMsgT::ConstPtr& cloud_msg_child_2, const PointCloudMsgT::ConstPtr& cloud_msg_child_3);
};

PointsConcatFilter4::PointsConcatFilter4()
  : node_handle_(), private_node_handle_("~")
{
	ROS_INFO("Inititalizing the params of PointsConcatFilter4...");

	private_node_handle_.param<bool>("use_intensity_render", use_intensity_render_, false);

	private_node_handle_.param<bool>("use_yaml_config", use_yaml_config_, false);

	private_node_handle_.param("lidar_child_1_yaml_path", lidar_child_1_yaml_path_, std::string(""));
	ROS_INFO("lidar_child_1_yaml_path: %s", lidar_child_1_yaml_path_.c_str());
	private_node_handle_.param("lidar_child_2_yaml_path", lidar_child_2_yaml_path_, std::string(""));
	ROS_INFO("lidar_child_2_yaml_path: %s", lidar_child_2_yaml_path_.c_str());
	private_node_handle_.param("lidar_child_3_yaml_path", lidar_child_3_yaml_path_, std::string(""));
	ROS_INFO("lidar_child_3_yaml_path: %s", lidar_child_3_yaml_path_.c_str());

	if(use_yaml_config_){
		YAML::Node lidar_child_1_yaml_config = YAML::LoadFile(lidar_child_1_yaml_path_);
		transform_child1_2parent_x_ = lidar_child_1_yaml_config["x"].as<double>();
		transform_child1_2parent_y_ = lidar_child_1_yaml_config["y"].as<double>();
		transform_child1_2parent_z_ = lidar_child_1_yaml_config["z"].as<double>();
		transform_child1_2parent_roll_ = lidar_child_1_yaml_config["roll"].as<double>() * PI / 180;
		transform_child1_2parent_pitch_ = lidar_child_1_yaml_config["pitch"].as<double>() * PI / 180;
		transform_child1_2parent_yaw_ = lidar_child_1_yaml_config["yaw"].as<double>() * PI / 180;

		YAML::Node lidar_child_2_yaml_config = YAML::LoadFile(lidar_child_2_yaml_path_);
		transform_child2_2parent_x_ = lidar_child_2_yaml_config["x"].as<double>();
		transform_child2_2parent_y_ = lidar_child_2_yaml_config["y"].as<double>();
		transform_child2_2parent_z_ = lidar_child_2_yaml_config["z"].as<double>();
		transform_child2_2parent_roll_ = lidar_child_2_yaml_config["roll"].as<double>() * PI / 180;
		transform_child2_2parent_pitch_ = lidar_child_2_yaml_config["pitch"].as<double>() * PI / 180;
		transform_child2_2parent_yaw_ = lidar_child_2_yaml_config["yaw"].as<double>() * PI / 180;

		YAML::Node lidar_child_3_yaml_config = YAML::LoadFile(lidar_child_3_yaml_path_);
		transform_child3_2parent_x_ = lidar_child_3_yaml_config["x"].as<double>();
		transform_child3_2parent_y_ = lidar_child_3_yaml_config["y"].as<double>();
		transform_child3_2parent_z_ = lidar_child_3_yaml_config["z"].as<double>();
		transform_child3_2parent_roll_ = lidar_child_3_yaml_config["roll"].as<double>() * PI / 180;
		transform_child3_2parent_pitch_ = lidar_child_3_yaml_config["pitch"].as<double>() * PI / 180;
		transform_child3_2parent_yaw_ = lidar_child_3_yaml_config["yaw"].as<double>() * PI / 180;

	}else{
		private_node_handle_.param("transform_child1_2parent_x", transform_child1_2parent_x_, 0.0);
		private_node_handle_.param("transform_child1_2parent_y", transform_child1_2parent_y_, 0.0);
		private_node_handle_.param("transform_child1_2parent_z", transform_child1_2parent_z_, 0.0);
		private_node_handle_.param("transform_child1_2parent_roll", transform_child1_2parent_roll_, 0.0);
		private_node_handle_.param("transform_child1_2parent_pitch", transform_child1_2parent_pitch_, 0.0);
		private_node_handle_.param("transform_child1_2parent_yaw", transform_child1_2parent_yaw_, 0.0);

		private_node_handle_.param("transform_child2_2parent_x", transform_child2_2parent_x_, 0.0);
		private_node_handle_.param("transform_child2_2parent_y", transform_child2_2parent_y_, 0.0);
		private_node_handle_.param("transform_child2_2parent_z", transform_child2_2parent_z_, 0.0);
		private_node_handle_.param("transform_child2_2parent_roll", transform_child2_2parent_roll_, 0.0);
		private_node_handle_.param("transform_child2_2parent_pitch", transform_child2_2parent_pitch_, 0.0);
		private_node_handle_.param("transform_child2_2parent_yaw", transform_child2_2parent_yaw_, 0.0);

		private_node_handle_.param("transform_child3_2parent_x", transform_child3_2parent_x_, 0.0);
		private_node_handle_.param("transform_child3_2parent_y", transform_child3_2parent_y_, 0.0);
		private_node_handle_.param("transform_child3_2parent_z", transform_child3_2parent_z_, 0.0);
		private_node_handle_.param("transform_child3_2parent_roll", transform_child3_2parent_roll_, 0.0);
		private_node_handle_.param("transform_child3_2parent_pitch", transform_child3_2parent_pitch_, 0.0);
		private_node_handle_.param("transform_child3_2parent_yaw", transform_child3_2parent_yaw_, 0.0);
	}

	// output the transformation between the child to the parent
	ROS_INFO("transform_child1_2parent_x: %f", transform_child1_2parent_x_);
	ROS_INFO("transform_child1_2parent_y: %f", transform_child1_2parent_y_);
	ROS_INFO("transform_child1_2parent_z: %f", transform_child1_2parent_z_);
	ROS_INFO("transform_child1_2parent_roll: %f", transform_child1_2parent_roll_);
	ROS_INFO("transform_child1_2parent_pitch: %f", transform_child1_2parent_pitch_);
	ROS_INFO("transform_child1_2parent_yaw: %f", transform_child1_2parent_yaw_);

	ROS_INFO("transform_child2_2parent_x: %f", transform_child2_2parent_x_);
	ROS_INFO("transform_child2_2parent_y: %f", transform_child2_2parent_y_);
	ROS_INFO("transform_child2_2parent_z: %f", transform_child2_2parent_z_);
	ROS_INFO("transform_child2_2parent_roll: %f", transform_child2_2parent_roll_);
	ROS_INFO("transform_child2_2parent_pitch: %f", transform_child2_2parent_pitch_);
	ROS_INFO("transform_child2_2parent_yaw: %f", transform_child2_2parent_yaw_);

	ROS_INFO("transform_child3_2parent_x: %f", transform_child3_2parent_x_);
	ROS_INFO("transform_child3_2parent_y: %f", transform_child3_2parent_y_);
	ROS_INFO("transform_child3_2parent_z: %f", transform_child3_2parent_z_);
	ROS_INFO("transform_child3_2parent_roll: %f", transform_child3_2parent_roll_);
	ROS_INFO("transform_child3_2parent_pitch: %f", transform_child3_2parent_pitch_);
	ROS_INFO("transform_child3_2parent_yaw: %f", transform_child3_2parent_yaw_);

  	private_node_handle_.param("output_frame", output_frame_, std::string("ls"));
	ROS_INFO("output frame: %s", output_frame_.c_str());
	
	private_node_handle_.param("lidar_parent_topic", lidar_parent_topic_, std::string("/lidar2/points_raw"));
	ROS_INFO("lidar parent topic: %s", lidar_parent_topic_.c_str());
	
	private_node_handle_.param("lidar_child1_topic", lidar_child_1_topic_, std::string("/lidar1/points_raw"));
	ROS_INFO("lidar child1 topic: %s", lidar_child_1_topic_.c_str());
	private_node_handle_.param("lidar_child2_topic", lidar_child_2_topic_, std::string("/lidar3/points_raw"));
	ROS_INFO("lidar child2 topic: %s", lidar_child_2_topic_.c_str());
	private_node_handle_.param("lidar_child3_topic", lidar_child_3_topic_, std::string("/lidar4/points_raw"));
	ROS_INFO("lidar child3 topic: %s", lidar_child_3_topic_.c_str());

	private_node_handle_.param("concat_points_topic", concat_points_topic_, std::string("/points_concat"));
	ROS_INFO("concat points topic: %s", concat_points_topic_.c_str());
	
	cloud_subscriber_parent_ = new message_filters::Subscriber<PointCloudMsgT>(node_handle_, lidar_parent_topic_, 10);
	cloud_subscriber_child_1_ = new message_filters::Subscriber<PointCloudMsgT>(node_handle_, lidar_child_1_topic_, 10);
	cloud_subscriber_child_2_ = new message_filters::Subscriber<PointCloudMsgT>(node_handle_, lidar_child_2_topic_, 10);
	cloud_subscriber_child_3_ = new message_filters::Subscriber<PointCloudMsgT>(node_handle_, lidar_child_3_topic_, 10);
	
	cloud_synchronizer_ =
		new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100), *cloud_subscriber_parent_, *cloud_subscriber_child_1_, *cloud_subscriber_child_2_, *cloud_subscriber_child_3_);
  	cloud_synchronizer_->registerCallback(boost::bind(&PointsConcatFilter4::pointcloud_callback, this, _1, _2, _3, _4));
  	concat_cloud_publisher_ = node_handle_.advertise<PointCloudMsgT>(concat_points_topic_, 1);

	Eigen::Translation3f init_translation_1(transform_child1_2parent_x_, transform_child1_2parent_y_, transform_child1_2parent_z_);
	Eigen::AngleAxisf init_rotation_x_1(transform_child1_2parent_roll_, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y_1(transform_child1_2parent_pitch_, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z_1(transform_child1_2parent_yaw_, Eigen::Vector3f::UnitZ());
	transform_child1_to_parent_ = (init_translation_1 * init_rotation_z_1 * init_rotation_y_1 * init_rotation_x_1).matrix();

	Eigen::Translation3f init_translation_2(transform_child2_2parent_x_, transform_child2_2parent_y_, transform_child2_2parent_z_);
	Eigen::AngleAxisf init_rotation_x_2(transform_child2_2parent_roll_, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y_2(transform_child2_2parent_pitch_, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z_2(transform_child2_2parent_yaw_, Eigen::Vector3f::UnitZ());
	transform_child2_to_parent_ = (init_translation_2 * init_rotation_z_2 * init_rotation_y_2 * init_rotation_x_2).matrix();

	Eigen::Translation3f init_translation_3(transform_child3_2parent_x_, transform_child3_2parent_y_, transform_child3_2parent_z_);
	Eigen::AngleAxisf init_rotation_x_3(transform_child3_2parent_roll_, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y_3(transform_child3_2parent_pitch_, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z_3(transform_child3_2parent_yaw_, Eigen::Vector3f::UnitZ());
	transform_child3_to_parent_ = (init_translation_3 * init_rotation_z_3 * init_rotation_y_3 * init_rotation_x_3).matrix();
}


void PointsConcatFilter4::pointcloud_callback(const PointCloudMsgT::ConstPtr& cloud_msg_parent,
                                             const PointCloudMsgT::ConstPtr& cloud_msg_child_1, 
											 const PointCloudMsgT::ConstPtr& cloud_msg_child_2, 
											 const PointCloudMsgT::ConstPtr& cloud_msg_child_3)
{
	PointCloudT::Ptr cloud_source_parent(new PointCloudT);

	PointCloudT::Ptr cloud_source_child_1(new PointCloudT);
	PointCloudT::Ptr cloud_source_child_1_transformed(new PointCloudT);
	PointCloudT::Ptr cloud_source_child_2(new PointCloudT);
	PointCloudT::Ptr cloud_source_child_2_transformed(new PointCloudT);
	PointCloudT::Ptr cloud_source_child_3(new PointCloudT);
	PointCloudT::Ptr cloud_source_child_3_transformed(new PointCloudT);

	pcl::fromROSMsg(*cloud_msg_parent, *cloud_source_parent);
	pcl::fromROSMsg(*cloud_msg_child_1, *cloud_source_child_1);
	pcl::fromROSMsg(*cloud_msg_child_2, *cloud_source_child_2);
	pcl::fromROSMsg(*cloud_msg_child_3, *cloud_source_child_3);

	pcl::transformPointCloud (*cloud_source_child_1, *cloud_source_child_1_transformed, transform_child1_to_parent_);
	pcl::transformPointCloud (*cloud_source_child_2, *cloud_source_child_2_transformed, transform_child2_to_parent_);
	pcl::transformPointCloud (*cloud_source_child_3, *cloud_source_child_3_transformed, transform_child3_to_parent_);

	
	// merge points
	if(use_intensity_render_){
		pcl::PointCloud<pcl::PointXYZI> cloud_concatenated_intensity_color;
		pcl::PointXYZI p_i;
		for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = cloud_source_parent->begin(); item != cloud_source_parent->end(); item++)
		{
			p_i.x = (double)item->x;
			p_i.y = (double)item->y;
			p_i.z = (double)item->z;
			p_i.intensity = 0;
			cloud_concatenated_intensity_color.push_back(p_i);
		}
		for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = cloud_source_child_1_transformed->begin(); item != cloud_source_child_1_transformed->end(); item++)
		{
			p_i.x = (double)item->x;
			p_i.y = (double)item->y;
			p_i.z = (double)item->z;
			p_i.intensity = 64;
			cloud_concatenated_intensity_color.push_back(p_i);
		}
		for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = cloud_source_child_2_transformed->begin(); item != cloud_source_child_2_transformed->end(); item++)
		{
			p_i.x = (double)item->x;
			p_i.y = (double)item->y;
			p_i.z = (double)item->z;
			p_i.intensity = 128;
			cloud_concatenated_intensity_color.push_back(p_i);
		}
		for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = cloud_source_child_3_transformed->begin(); item != cloud_source_child_3_transformed->end(); item++)
		{
			p_i.x = (double)item->x;
			p_i.y = (double)item->y;
			p_i.z = (double)item->z;
			p_i.intensity = 192;
			cloud_concatenated_intensity_color.push_back(p_i);
		}
		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(cloud_concatenated_intensity_color, cloud_msg);
		cloud_msg.header = cloud_msg_parent->header;
		concat_cloud_publisher_.publish(cloud_msg);
	}else{
		PointCloudT::Ptr cloud_concatenated(new PointCloudT);
		*cloud_concatenated += *cloud_source_parent;
		*cloud_concatenated += *cloud_source_child_1_transformed;
		*cloud_concatenated += *cloud_source_child_2_transformed;
		*cloud_concatenated += *cloud_source_child_3_transformed;

		sensor_msgs::PointCloud2 cloud_msg;
		pcl::toROSMsg(*cloud_concatenated, cloud_msg);
		cloud_msg.header = cloud_msg_parent->header;
		concat_cloud_publisher_.publish(cloud_msg);
	}
	
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "PointsConcatFilter4");
  PointsConcatFilter4 node;
  ros::spin();
  return 0;
}
