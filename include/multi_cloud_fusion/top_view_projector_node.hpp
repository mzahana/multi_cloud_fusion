#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <cstdint>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

class TopViewProjectorNode : public rclcpp::Node
{
public:
  explicit TopViewProjectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using CloudMsg = sensor_msgs::msg::PointCloud2;
  using ImageMsg = sensor_msgs::msg::Image;

  void cloudCallback(const CloudMsg::ConstSharedPtr & cloud);

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params);

  // ---------- Runtime parameters (protected by mutex) ----------
  int    image_width_;
  int    image_height_;
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  bool   use_max_height_;
  uint8_t bg_r_;
  uint8_t bg_g_;
  uint8_t bg_b_;

  // image-level filters
  bool use_hole_filling_;
  int  hole_filling_iterations_;

  bool use_smoothing_;
  int  smoothing_kernel_size_;

  // ---------- ROS interfaces ----------
  rclcpp::Subscription<CloudMsg>::SharedPtr cloud_sub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr rgb_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr depth_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ---------- Mutex for shared state ----------
  std::mutex state_mutex_;
};
