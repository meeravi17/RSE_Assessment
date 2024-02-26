#ifndef OFFLINE_LOCALIZATION_ICP
#define OFFLINE_LOCALIZATION_ICP

#include <steer_bot_navigation/offline_localization_icp/laser_utils.h>
#include <steer_bot_navigation/offline_localization_icp/matrix_utils.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
#include<std_msgs/Float32.h>
#include <tf2/convert.h>
#include<nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// class icpExposed : public pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> {
//   public:
//     icpExposed() : pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>() {}
//     void setCorrespondences(const pcl::CorrespondencesPtr& correspondences){
//       correspondences_ = correspondences;
//     }

//     pcl::CorrespondencesPtr getCorrespondencesPtr() {
//       if (this->correspondences_->size() == 0) {
//         std::cerr << "Correspondences size is zero!" << std::endl;
//         // Handle the case when correspondences are empty
//         return this->correspondences_;
//       }
//         for (uint32_t i = 0; i < this->correspondences_->size(); i++) {
//             pcl::Correspondence currentCorrespondence = (*this->correspondences_)[i];
//             std::cout << "Index of the source point: " << currentCorrespondence.index_query << std::endl;
//             std::cout << "Index of the matching target point: " << currentCorrespondence.index_match << std::endl;
//             std::cout << "Distance between the corresponding points: " << currentCorrespondence.distance << std::endl;
//             std::cout << "Weight of the confidence in the correspondence: " << currentCorrespondence.weight << std::endl;
//         }
//         return this->correspondences_;
//     }

//     pcl::CorrespondencesPtr getCorrespondences()const {
//       return correspondences_ ;
//     }
//   private:
//     pcl::CorrespondencesPtr correspondences_;

// };

class offlineLocalization //: public icpExposed 
{
public:
  offlineLocalization();
  void createModelPointCloud(std::string);
  pcl::PointCloud<pcl::PointXYZ>::Ptr generateModelPointCloud(std::string);
  bool runICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
              const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
              int iterations, double threshold,
              Eigen::Matrix4d &result, bool store_required);
  void sourceLaserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
  bool loadImageFromFile(std::string complete_file_name,
                         cv_bridge::CvImage &cv_image, double &orgin_x,
                         double &orgin_y);

  void setInitialPoseFromParameterServer(const ros::NodeHandle &nh,
                                         const std::string &prefix);

  double degreesToRadians(double degrees);

  void applyTransformation(const Eigen::Matrix4d &transformation,
                           const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud);
  void setInitialPose(const Eigen::Vector3d &p, const Eigen::Quaterniond &q);
  void initialPoseReceived(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void processOccupancyGrid(const nav_msgs::OccupancyGrid& msg);
  pcl::KdTree<pcl::PointXYZ>::Ptr createKdTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudb);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz;
  bool validateMapFrame();
  bool transformLaserScanToMapFrame(const sensor_msgs::LaserScan::ConstPtr &msg, sensor_msgs::PointCloud2 &transformed_scan);
  void performICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &tmp_input_cloud,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
                                     std::vector<Eigen::Matrix4d> &further_interpretation_transformation_matrix,
                                     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &further_interpretation_point_cloud);

  void applyFinalTransformation(const std::vector<Eigen::Matrix4d> &further_interpretation_transformation_matrix, Eigen::Matrix4d &final_transformation_matrix);
  void publishResult(const pcl::PointCloud<pcl::PointXYZ>::Ptr &tmp_input_cloud, const std::string &frame_id, Eigen::Matrix4d &final_transformation_matrix);
  bool runNDT(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
                                 int iterations, double threshold,
                                 Eigen::Matrix4d &result, bool store_required);
  bool performNDT(const pcl::PointCloud<pcl::PointXYZ>::Ptr &tmp_input_cloud,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
                                     std::vector<Eigen::Matrix4d> &further_interpretation_transformation_matrix,
                                     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &further_interpretation_point_cloud);
 void publishTransform(const Eigen::Matrix4d& transformation_matrix);
//  pcl::CorrespondencesPtr getCorrespondences(); 
 
private:
  ros::Subscriber input_sub, map_sub, initial_pose_sub, odom_sub, target_sub, on_tranformed_pattern_rviz_sub;
  ros::Publisher pose_stamped_pub, transformed_pattern_rviz_pub, laser_output_pub;
  ros::Publisher pub_output_, laserOutput, score_pub;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;

  double max_distance, further_interpretation_threshold,
      default_fitness_threshold, fitness_threshold, default_correspondence_distance, default_interpretation_threshold;
  bool input_set, map_received, to_publish_tf, pose_sent, retry_done, input_cloud_set;
  int max_iterations, further_interpretation_count, max_retry_count;
  int rotation_row, rotation_column;
  int translation_row, translation_column;
  int retry_count, false_count;
  double feature_posx, feature_posy;
  double default_further_interpretation_threshold;
  std::vector<Eigen::Matrix4d> stored_transformation_matrix;
  std::vector<double> stored_fitness_score;
  geometry_msgs::Pose final_pose;
  Eigen::Matrix4f prev_guess, init_guess;  
  std::string localizationMethodStr;
  pcl::CorrespondencesPtr correspondencesicp_;

  boost::mutex m_lock;
  int searched;
  MatrixUtils mat;
  YAML::Node doc;
  std::string feature_folder, map_folder, global_frame_id, odom_topic;
  geometry_msgs::PoseWithCovarianceStamped initialPose;
  geometry_msgs::Pose robot_pose;
  double x, y, z, roll, pitch, yaw;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud_xyz, generated_model_cloud, input_cloud;
  pcl::KdTree<pcl::PointXYZ>::Ptr mapTree;
  boost::shared_ptr< sensor_msgs::PointCloud2> output_map_cloud = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());
  tf::TransformListener listener;
  laser_geometry::LaserProjection laser_projector_;

};



#endif
