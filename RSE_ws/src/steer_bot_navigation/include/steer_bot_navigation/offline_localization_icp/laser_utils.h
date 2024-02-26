#ifndef LASER_UTILS_H
#define LASER_UTILS_H

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <queue>

class GetGoalCluster {
 public:
  GetGoalCluster();
  static pcl::PointCloud<pcl::PointXYZ>::Ptr getGoalCluster(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_cloud_array,std::string parameter);

};

class LaserScanPclConversion {
 public:
	LaserScanPclConversion();
  static pcl::PointCloud<pcl::PointXYZ>::Ptr laserScanToPclPtr(const sensor_msgs::LaserScan::ConstPtr& scan);
  static bool pclToLaserScan(sensor_msgs::LaserScan original_scan, sensor_msgs::PointCloud2 ros_pointcloud, sensor_msgs::LaserScan& output);
  static tf::Transform eigenMatrix4dToTransform(Eigen::Matrix4d mat);
  static void print4x4dMatrix(const Eigen::Matrix4d &matrix);
  static geometry_msgs::PoseStamped generatePoseFromTF(tf::Transform tf,
  		const sensor_msgs::LaserScan::ConstPtr &msg);
  static geometry_msgs::PoseStamped generatePoseFromMatrix(const Eigen::Matrix4d &matrix, const
		sensor_msgs::LaserScan::ConstPtr &msg);

  static void makeCloudsBijective(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1,
  		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2);
};

#endif