#include "rclcpp/rclcpp.hpp"
#include "multi_cloud_fusion/cloud_fusion_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CloudFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
