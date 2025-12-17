#pragma once

#include <memory>
#include <string>
#include <mutex>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <chrono>

class CloudFusionNode : public rclcpp::Node
{
public:
  explicit CloudFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CloudMsg   = sensor_msgs::msg::PointCloud2;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<CloudMsg, CloudMsg>;

  // Main synchronized callback
  void callback(
    const CloudMsg::ConstSharedPtr & cloud1,
    const CloudMsg::ConstSharedPtr & cloud2);

  // Timer callback for low-frequency TF lookup
  void tfTimerCallback();

  // Runtime parameter callback
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params);

  // ------------- Runtime parameters (protected by mutex) -------------
  std::string target_frame_;
  bool        use_voxel_grid_;
  double      voxel_leaf_size_;

  //extra 3D filters
  bool   use_z_crop_;
  double z_min_;
  double z_max_;

  bool   use_outlier_removal_;
  double outlier_radius_;
  int    outlier_min_neighbors_;

  // ------------- TF state (protected by mutex) -------------
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr                tf_timer_;

  bool          have_tf1_;
  bool          have_tf2_;
  std::string   frame1_;
  std::string   frame2_;
  Eigen::Matrix4f tf1_matrix_;  // target_frame_ <- frame1_
  Eigen::Matrix4f tf2_matrix_;  // target_frame_ <- frame2_

  // ------------- Subscribers & sync (not protected; setup at startup) -------------
  message_filters::Subscriber<CloudMsg> cloud1_sub_;
  message_filters::Subscriber<CloudMsg> cloud2_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // ------------- Publisher -------------
  rclcpp::Publisher<CloudMsg>::SharedPtr fused_pub_;

  // ------------- Parameter callback -------------
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ------------- Mutex for shared state -------------
  std::mutex state_mutex_;

  // To track syncing timing
  rclcpp::Time last_callback_time_;
};
