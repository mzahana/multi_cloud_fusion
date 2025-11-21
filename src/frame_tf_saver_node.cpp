// @file frame_tf_saver_node.cpp
#include <memory>
#include <string>
#include <fstream>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

class FrameTfSaverNode : public rclcpp::Node
{
public:
  FrameTfSaverNode()
  : Node("frame_tf_saver_node"),
    buffer_(this->get_clock()),
    tf_listener_(buffer_),
    start_time_(this->get_clock()->now())
  {
    source_frame_ = this->declare_parameter<std::string>("source_frame", "camera1_link");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "camera2_link");
    output_yaml_path_ = this->declare_parameter<std::string>("output_yaml_path", "icp_initial_tf.yaml");
    lookup_timeout_sec_ = this->declare_parameter<double>("lookup_timeout_sec", 5.0);

    RCLCPP_INFO(get_logger(), "frame_tf_saver_node started.");
    RCLCPP_INFO(get_logger(), "  source_frame: %s", source_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  target_frame: %s", target_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  output_yaml_path: %s", output_yaml_path_.c_str());
    RCLCPP_INFO(get_logger(), "  lookup_timeout_sec: %.2f", lookup_timeout_sec_);

    // Timer to periodically try and get the transform
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&FrameTfSaverNode::timerCallback, this));
  }

private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string source_frame_;
  std::string target_frame_;
  std::string output_yaml_path_;
  double lookup_timeout_sec_;

  rclcpp::Time start_time_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timerCallback()
  {
    auto now = this->get_clock()->now();
    double elapsed = (now - start_time_).seconds();
    if (elapsed > lookup_timeout_sec_) {
      RCLCPP_ERROR(
        get_logger(),
        "Timeout (%.2f s) while waiting for transform %s -> %s.",
        lookup_timeout_sec_, target_frame_.c_str(), source_frame_.c_str());
      rclcpp::shutdown();
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = buffer_.lookupTransform(
        target_frame_,  // target
        source_frame_,  // source
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Transform not available yet (%s -> %s): %s",
        target_frame_.c_str(), source_frame_.c_str(), ex.what());
      return;
    }

    // If we get here, we found the transform
    RCLCPP_INFO(get_logger(), "Found transform %s -> %s.",
                target_frame_.c_str(), source_frame_.c_str());

    printTransform(tf_msg);

    if (!saveTransformToYaml(tf_msg, output_yaml_path_)) {
      RCLCPP_ERROR(get_logger(), "Failed to save transform to YAML. Exiting with error.");
    } else {
      RCLCPP_INFO(get_logger(), "Transform saved to '%s'.", output_yaml_path_.c_str());
    }

    // We are done; shut down the node.
    rclcpp::shutdown();
  }

  void printTransform(const geometry_msgs::msg::TransformStamped & tf_msg)
  {
    const auto & t = tf_msg.transform.translation;
    const auto & q = tf_msg.transform.rotation;

    std::cout << "Transform " << tf_msg.header.frame_id
              << " -> " << tf_msg.child_frame_id << ":\n";
    std::cout << "  Translation: ["
              << t.x << ", " << t.y << ", " << t.z << "]\n";
    std::cout << "  Rotation (x,y,z,w): ["
              << q.x << ", " << q.y << ", " << q.z << ", " << q.w << "]\n";
    std::cout.flush();
  }

  bool saveTransformToYaml(
    const geometry_msgs::msg::TransformStamped & tf_msg,
    const std::string & path)
  {
    std::ofstream out(path);
    if (!out.is_open()) {
      RCLCPP_ERROR(get_logger(), "Could not open '%s' for writing.", path.c_str());
      return false;
    }

    const auto & t = tf_msg.transform.translation;
    const auto & q = tf_msg.transform.rotation;

    // Simple YAML structure
    out << "parent_frame: " << tf_msg.header.frame_id << "\n";
    out << "child_frame: " << tf_msg.child_frame_id << "\n";
    out << "translation: [" << t.x << ", " << t.y << ", " << t.z << "]\n";
    out << "rotation: [" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << "]\n";

    out.close();
    return true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrameTfSaverNode>();
  rclcpp::spin(node);
  // rclcpp::shutdown() is called from inside the node after saving TF.
  return 0;
}
