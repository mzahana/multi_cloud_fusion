#include "rclcpp/rclcpp.hpp"
#include "multi_cloud_fusion/top_view_projector_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopViewProjectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
