#include <steer_bot_navigation/offline_localization_icp/laser_utils.h>
#include <laser_geometry/laser_geometry.h>

LaserScanPclConversion::LaserScanPclConversion() {
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LaserScanPclConversion::laserScanToPclPtr(
		const sensor_msgs::LaserScan::ConstPtr& scan) {
	laser_geometry::LaserProjection laser_projector;
	tf::TransformListener tf_listener;
	sensor_msgs::PointCloud2 ros_point_cloud;
	laser_projector.transformLaserScanToPointCloud(scan->header.frame_id, *scan,
			ros_point_cloud, tf_listener);

	//convert to pcl point cloud
	pcl::PCLPointCloud2 pcl_point_cloud2;
	pcl_conversions::toPCL(ros_point_cloud, pcl_point_cloud2);

	//convert to PointCloud Ptr
	pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud_ptr(
			new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_point_cloud2, *final_cloud_ptr);

	return final_cloud_ptr;

}

bool LaserScanPclConversion::pclToLaserScan(sensor_msgs::LaserScan original_scan,
		sensor_msgs::PointCloud2 ros_pointcloud,
		sensor_msgs::LaserScan& output) {

	output.header = original_scan.header;
	output.angle_min = original_scan.angle_min;
	output.angle_max = original_scan.angle_max;
	output.angle_increment = original_scan.angle_increment;
	output.time_increment = 0.0;
	output.scan_time = original_scan.scan_time;
	output.range_min = original_scan.range_min;
	output.range_max = original_scan.range_max;

	if (output.angle_increment == 0.0)
		return false;

	uint32_t ranges_size = std::ceil(
			(output.angle_max - output.angle_min) / output.angle_increment);
	output.ranges.assign(ranges_size, output.range_max + 1.0);
	for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(ros_pointcloud,
			"x"), iter_y(ros_pointcloud, "y"), iter_z(ros_pointcloud, "z");
			iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {

		if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
			continue;
		}

		double range = hypot(*iter_x, *iter_y);
		if (range < output.range_min) {
			continue;
		}

		double angle = atan2(*iter_y, *iter_x);
		if (angle < output.angle_min || angle > output.angle_max) {
			continue;
		}

		//overwrite range at laserscan ray if new range is smaller
		int index = (angle - output.angle_min) / output.angle_increment;
		if (range < output.ranges[index]) {
			output.ranges[index] = range;
		}
	}
}

/**
 * Converts Eigen::Matrix4d to tr::Transform, adapted from:
 * https://answers.ros.org/question/103411/4x4-transformation-matrix-to-tf-transform/
 * Note: can convert Eigen::Matrix4f to Eigen::Matrix4d with .cast<double>()
 */
tf::Transform LaserScanPclConversion::eigenMatrix4dToTransform(Eigen::Matrix4d mat) {
  tf::Vector3 origin;
  origin.setValue((mat(0,3)), (mat(1,3)), (mat(2,3)));
  tf::Matrix3x3 tf3d;
  tf3d.setValue((mat(0,0)), (mat(0,1)), (mat(0,2)), (mat(1,0)), (mat(1,1)), (mat(1,2)), (mat(2,0)),
                (mat(2,1)), (mat(2,2)));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  tfqt.normalize();
  tf::Transform result;
  result.setOrigin(origin);
  result.setRotation(tfqt);
  return result;
}

/**
 * generate a pose from tf
 */
geometry_msgs::PoseStamped LaserScanPclConversion::generatePoseFromTF(tf::Transform tf, const
    sensor_msgs::LaserScan::ConstPtr &msg) {
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  tf::Vector3 o = tf.getOrigin();
  pose.pose.position.x = o.getX();
  pose.pose.position.y = o.getY();
  pose.pose.position.z = o.getZ();
  tf::quaternionTFToMsg(tf.getRotation(), pose.pose.orientation);
  return pose;
}

geometry_msgs::PoseStamped LaserScanPclConversion::generatePoseFromMatrix(const Eigen::Matrix4d
		&matrix, const sensor_msgs::LaserScan::ConstPtr &msg) {
	geometry_msgs::PoseStamped pose;
	pose.header = msg->header;
	pose.pose.position.x = matrix(0,3);
	pose.pose.position.y = matrix(1,3);
	pose.pose.position.z = matrix(2,3);

	tf::Matrix3x3 tf3d;
	tf3d.setValue((matrix(0,0)), (matrix(0,1)), (matrix(0,2)), (matrix(1,0)), (matrix(1,1)),
			(matrix(1,2)), (matrix(2,0)), (matrix(2,1)), (matrix(2,2)));
	tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);
	tfqt.normalize();
	tf::quaternionTFToMsg(tfqt, pose.pose.orientation);
	return pose;
}

void LaserScanPclConversion::print4x4dMatrix(const Eigen::Matrix4d &matrix) {
  printf("Rotation matrix :\n");
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

/**
 * Makes two points clouds the same size by selecting points uniformly at random to remove from
 * the larger cloud.
 * @require cloud_1.size() > cloud_2.size()
 * @ensure cloud_1.size() == cloud_2.size()
 */
void LaserScanPclConversion::makeCloudsBijective(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2) {
  size_t diff = cloud_1->size() - cloud_2->size();
  for (size_t i = 0; i < diff; i++) {
    size_t index = rand() % cloud_1->size();
    auto it = cloud_1->points.begin() + index;
    cloud_1->points.erase(it);
  }
}
