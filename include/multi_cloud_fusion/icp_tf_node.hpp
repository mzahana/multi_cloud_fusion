#ifndef MULTI_CLOUD_FUSION__ICP_TF_NODE_HPP_
#define MULTI_CLOUD_FUSION__ICP_TF_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace multi_cloud_fusion
{

class IcpTfNode : public rclcpp::Node
{
public:
  IcpTfNode();

private:
  using PointT = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2>;

  // Subscribers and synchronizer
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> source_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> target_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Current estimate of the transform (target_frame <- source_frame)
  Eigen::Matrix4f current_transform_;

  // Parameters
  std::string source_topic_;
  std::string target_topic_;

  int icp_max_iterations_;
  double icp_max_correspondence_distance_;
  double icp_transformation_epsilon_;
  double icp_euclidean_fitness_epsilon_;
  double voxel_leaf_size_;
  double fitness_score_threshold_;
  bool use_previous_transform_as_initial_guess_;
  bool publish_tf_;
  double icp_rate_hz_;

  // New parameter: YAML file with initial transform
  std::string initial_tf_yaml_path_;

  // Rate limiting
  rclcpp::Time last_icp_time_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  void logCurrentParameters();

  rcl_interfaces::msg::SetParametersResult
  onParametersChanged(const std::vector<rclcpp::Parameter> & params);

  bool shouldRunIcp(const rclcpp::Time & now);

  void pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & source_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & target_msg);

  bool loadInitialTransformFromYaml(const std::string & path);
};

}  // namespace multi_cloud_fusion

#endif  // MULTI_CLOUD_FUSION__ICP_TF_NODE_HPP_
