#include "multi_cloud_fusion/icp_tf_node.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/common/common.h>       // for pcl::isFinite

#include "rmw/qos_profiles.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>

namespace multi_cloud_fusion
{

IcpTfNode::IcpTfNode()
: Node("icp_tf_node"),
  current_transform_(Eigen::Matrix4f::Identity()),
  last_icp_time_(0, 0, get_clock()->get_clock_type()),
  icp_rate_hz_(5.0)  // default 5 Hz
{
  RCLCPP_INFO(get_logger(), "Starting ICP TF node (multi_cloud_fusion)");

  // Declare parameters
  source_topic_ = this->declare_parameter<std::string>(
    "source_topic", "/camera1/depth/points");
  target_topic_ = this->declare_parameter<std::string>(
    "target_topic", "/camera2/depth/points");

  icp_max_iterations_ = this->declare_parameter<int>(
    "icp_max_iterations", 50);
  icp_max_correspondence_distance_ = this->declare_parameter<double>(
    "icp_max_correspondence_distance", 0.05);  // meters
  icp_transformation_epsilon_ = this->declare_parameter<double>(
    "icp_transformation_epsilon", 1e-8);
  icp_euclidean_fitness_epsilon_ = this->declare_parameter<double>(
    "icp_euclidean_fitness_epsilon", 1e-6);

  voxel_leaf_size_ = this->declare_parameter<double>(
    "voxel_leaf_size", 0.01);  // 1 cm; <= 0 disables

  fitness_score_threshold_ = this->declare_parameter<double>(
    "fitness_score_threshold", 0.5);

  use_previous_transform_as_initial_guess_ = this->declare_parameter<bool>(
    "use_previous_transform_as_initial_guess", true);

  publish_tf_ = this->declare_parameter<bool>(
    "publish_tf", true);

  icp_rate_hz_ = this->declare_parameter<double>(
    "icp_rate_hz", 5.0);  // max ICP updates per second

  initial_tf_yaml_path_ = this->declare_parameter<std::string>(
    "initial_tf_yaml_path", "");

  // Try to load initial transform from YAML if given
  if (!initial_tf_yaml_path_.empty()) {
    if (loadInitialTransformFromYaml(initial_tf_yaml_path_)) {
      RCLCPP_INFO(
        get_logger(),
        "Loaded initial transform from '%s' and set as ICP starting guess.",
        initial_tf_yaml_path_.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Failed to load initial transform from '%s'. Using identity.",
        initial_tf_yaml_path_.c_str());
    }
  }

  logCurrentParameters();

  // TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Subscribers with ApproximateTime synchronization and sensor QoS
  source_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
    this, source_topic_, rmw_qos_profile_sensor_data);
  target_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
    this, target_topic_, rmw_qos_profile_sensor_data);

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), *source_sub_, *target_sub_);
  sync_->registerCallback(
    std::bind(&IcpTfNode::pointcloudCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Dynamic parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&IcpTfNode::onParametersChanged, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
              "ICP TF node initialized. Source topic: '%s', Target topic: '%s'",
              source_topic_.c_str(), target_topic_.c_str());
}

void IcpTfNode::logCurrentParameters()
{
  RCLCPP_INFO(get_logger(), "ICP parameters:");
  RCLCPP_INFO(get_logger(), "  source_topic: %s", source_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  target_topic: %s", target_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  icp_max_iterations: %d", icp_max_iterations_);
  RCLCPP_INFO(get_logger(), "  icp_max_correspondence_distance: %.4f",
              icp_max_correspondence_distance_);
  RCLCPP_INFO(get_logger(), "  icp_transformation_epsilon: %e",
              icp_transformation_epsilon_);
  RCLCPP_INFO(get_logger(), "  icp_euclidean_fitness_epsilon: %e",
              icp_euclidean_fitness_epsilon_);
  RCLCPP_INFO(get_logger(), "  voxel_leaf_size: %.4f", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "  fitness_score_threshold: %.4f",
              fitness_score_threshold_);
  RCLCPP_INFO(get_logger(), "  use_previous_transform_as_initial_guess: %s",
              use_previous_transform_as_initial_guess_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  publish_tf: %s",
              publish_tf_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "  icp_rate_hz: %.2f", icp_rate_hz_);
  RCLCPP_INFO(get_logger(), "  initial_tf_yaml_path: %s",
              initial_tf_yaml_path_.empty() ? "(none)" : initial_tf_yaml_path_.c_str());
}

rcl_interfaces::msg::SetParametersResult
IcpTfNode::onParametersChanged(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : params) {
    const auto & name = param.get_name();

    if (name == "icp_max_iterations") {
      icp_max_iterations_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Updated icp_max_iterations: %d", icp_max_iterations_);
    } else if (name == "icp_max_correspondence_distance") {
      icp_max_correspondence_distance_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated icp_max_correspondence_distance: %f",
                  icp_max_correspondence_distance_);
    } else if (name == "icp_transformation_epsilon") {
      icp_transformation_epsilon_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated icp_transformation_epsilon: %e",
                  icp_transformation_epsilon_);
    } else if (name == "icp_euclidean_fitness_epsilon") {
      icp_euclidean_fitness_epsilon_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated icp_euclidean_fitness_epsilon: %e",
                  icp_euclidean_fitness_epsilon_);
    } else if (name == "voxel_leaf_size") {
      voxel_leaf_size_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated voxel_leaf_size: %f", voxel_leaf_size_);
    } else if (name == "fitness_score_threshold") {
      fitness_score_threshold_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated fitness_score_threshold: %f",
                  fitness_score_threshold_);
    } else if (name == "use_previous_transform_as_initial_guess") {
      use_previous_transform_as_initial_guess_ = param.as_bool();
      RCLCPP_INFO(this->get_logger(),
                  "Updated use_previous_transform_as_initial_guess: %s",
                  use_previous_transform_as_initial_guess_ ? "true" : "false");
    } else if (name == "publish_tf") {
      publish_tf_ = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Updated publish_tf: %s",
                  publish_tf_ ? "true" : "false");
    } else if (name == "icp_rate_hz") {
      icp_rate_hz_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated icp_rate_hz: %.2f", icp_rate_hz_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Parameter '%s' is not dynamically adjustable or is ignored in this node.",
                  name.c_str());
    }
  }

  return result;
}

bool IcpTfNode::shouldRunIcp(const rclcpp::Time & now)
{
  if (icp_rate_hz_ <= 0.0) {
    // Non-positive means "run every sync"
    return true;
  }

  if (last_icp_time_.nanoseconds() == 0) {
    // First ICP run is always allowed
    return true;
  }

  const double min_dt = 1.0 / icp_rate_hz_;
  const double dt = (now - last_icp_time_).seconds();
  if (dt < min_dt) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Skipping ICP due to rate limit (dt=%.3f < %.3f)", dt, min_dt);
    return false;
  }
  return true;
}

void IcpTfNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & source_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & target_msg)
{
  const auto now = this->get_clock()->now();
  if (!shouldRunIcp(now)) {
    return;
  }
  last_icp_time_ = now;

  RCLCPP_DEBUG(this->get_logger(),
               "Received synchronized clouds: source frame '%s', target frame '%s'",
               source_msg->header.frame_id.c_str(),
               target_msg->header.frame_id.c_str());

  // Convert ROS PointCloud2 to PCL point clouds
  PointCloudT::Ptr source_raw(new PointCloudT);
  PointCloudT::Ptr target_raw(new PointCloudT);

  pcl::fromROSMsg(*source_msg, *source_raw);
  pcl::fromROSMsg(*target_msg, *target_raw);

  if (source_raw->empty() || target_raw->empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Received empty pointcloud(s), skipping ICP.");
    return;
  }

  // Helper to filter only finite points (no NaN, no Inf)
  auto filterFinite = [](const PointCloudT::Ptr & in_cloud,
                         const rclcpp::Logger & logger,
                         const char * label) -> PointCloudT::Ptr
  {
    PointCloudT::Ptr out_cloud(new PointCloudT);
    out_cloud->reserve(in_cloud->size());

    std::size_t removed = 0;
    for (const auto & p : *in_cloud) {
      if (pcl::isFinite(p)) {
        out_cloud->push_back(p);
      } else {
        ++removed;
      }
    }

    if (removed > 0) {
      RCLCPP_DEBUG(
        logger,
        "Removed %zu non-finite points from %s cloud. %zu -> %zu",
        removed, label, in_cloud->size(), out_cloud->size());
    }
    return out_cloud;
  };

  // Filter finite points from the raw clouds
  PointCloudT::Ptr source_cloud = filterFinite(source_raw, this->get_logger(), "source");
  PointCloudT::Ptr target_cloud = filterFinite(target_raw, this->get_logger(), "target");

  if (source_cloud->empty() || target_cloud->empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "After removing non-finite points, cloud(s) became empty. Skipping ICP.");
    return;
  }

  // Optional voxel-grid downsampling to accelerate and stabilize ICP
  PointCloudT::Ptr source_filtered(new PointCloudT);
  PointCloudT::Ptr target_filtered(new PointCloudT);

  if (voxel_leaf_size_ > 0.0) {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_));

    voxel.setInputCloud(source_cloud);
    voxel.filter(*source_filtered);

    voxel.setInputCloud(target_cloud);
    voxel.filter(*target_filtered);

    RCLCPP_DEBUG(
      this->get_logger(), "Downsampled source: %zu -> %zu, target: %zu -> %zu points",
      source_cloud->size(), source_filtered->size(),
      target_cloud->size(), target_filtered->size());

    // Filter non-finite points that may have been introduced by voxel grid
    source_filtered = filterFinite(source_filtered, this->get_logger(), "source(voxel)");
    target_filtered = filterFinite(target_filtered, this->get_logger(), "target(voxel)");
  } else {
    // No downsampling, use finite-filtered clouds directly
    source_filtered = source_cloud;
    target_filtered = target_cloud;
  }

  // Final check for empty clouds after all filtering
  if (source_filtered->empty() || target_filtered->empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "After filtering, cloud(s) became empty. Skipping ICP.");
    return;
  }

  // One more hard check for safety: if any point is non-finite, abort frame
  for (const auto & p : *source_filtered) {
    if (!pcl::isFinite(p)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Non-finite point detected in source_filtered just before ICP. Skipping frame.");
      return;
    }
  }
  for (const auto & p : *target_filtered) {
    if (!pcl::isFinite(p)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Non-finite point detected in target_filtered just before ICP. Skipping frame.");
      return;
    }
  }

  // Setup ICP
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(icp_max_iterations_);
  icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
  icp.setTransformationEpsilon(icp_transformation_epsilon_);
  icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_);

  icp.setInputSource(source_filtered);
  icp.setInputTarget(target_filtered);

  PointCloudT aligned_cloud;
  if (use_previous_transform_as_initial_guess_) {
    icp.align(aligned_cloud, current_transform_);
  } else {
    icp.align(aligned_cloud);
  }

  if (!icp.hasConverged()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "ICP did not converge.");
    return;
  }

  double fitness = icp.getFitnessScore();
  if (fitness > fitness_score_threshold_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "ICP fitness too high (%.6f > %.6f), rejecting transform update.",
      fitness, fitness_score_threshold_);
    return;
  }

  // Update current transform estimate
  current_transform_ = icp.getFinalTransformation();

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 3000,
    "ICP converged. Fitness: %.6f", fitness);

  if (!publish_tf_) {
    RCLCPP_DEBUG(
      this->get_logger(), "TF publishing disabled (publish_tf == false).");
    return;
  }

  // Publish TF: target_frame <- source_frame
  // ICP transforms source_cloud into target_cloud
  const std::string & source_frame = source_msg->header.frame_id;
  const std::string & target_frame = target_msg->header.frame_id;

  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = source_msg->header.stamp;
  transform_msg.header.frame_id = target_frame;
  transform_msg.child_frame_id = source_frame;

  Eigen::Matrix3f rotation = current_transform_.block<3, 3>(0, 0);
  Eigen::Vector3f translation = current_transform_.block<3, 1>(0, 3);

  Eigen::Quaternionf q(rotation);

  transform_msg.transform.translation.x = translation.x();
  transform_msg.transform.translation.y = translation.y();
  transform_msg.transform.translation.z = translation.z();

  transform_msg.transform.rotation.x = q.x();
  transform_msg.transform.rotation.y = q.y();
  transform_msg.transform.rotation.z = q.z();
  transform_msg.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(transform_msg);

  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "Published TF %s -> %s: t = [%.3f, %.3f, %.3f]",
    target_frame.c_str(), source_frame.c_str(),
    translation.x(), translation.y(), translation.z());
}

bool IcpTfNode::loadInitialTransformFromYaml(const std::string & path)
{
  std::ifstream in(path);
  if (!in.is_open()) {
    RCLCPP_ERROR(get_logger(), "Could not open initial TF YAML file '%s'.", path.c_str());
    return false;
  }

  auto trim = [](std::string s) -> std::string {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
      [](int ch) { return !std::isspace(ch); }));
    s.erase(std::find_if(s.rbegin(), s.rend(),
      [](int ch) { return !std::isspace(ch); }).base(), s.end());
    return s;
  };

  float tx = 0.0f, ty = 0.0f, tz = 0.0f;
  float qx = 0.0f, qy = 0.0f, qz = 0.0f, qw = 1.0f;
  bool translation_set = false;
  bool rotation_set = false;

  std::string line;
  while (std::getline(in, line)) {
    std::string tline = trim(line);
    if (tline.rfind("translation:", 0) == 0) {
      size_t lb = tline.find('[');
      size_t rb = tline.find(']');
      if (lb != std::string::npos && rb != std::string::npos && rb > lb) {
        std::string vals = tline.substr(lb + 1, rb - lb - 1);
        std::replace(vals.begin(), vals.end(), ',', ' ');
        std::stringstream ss(vals);
        ss >> tx >> ty >> tz;
        translation_set = true;
      }
    } else if (tline.rfind("rotation:", 0) == 0) {
      size_t lb = tline.find('[');
      size_t rb = tline.find(']');
      if (lb != std::string::npos && rb != std::string::npos && rb > lb) {
        std::string vals = tline.substr(lb + 1, rb - lb - 1);
        std::replace(vals.begin(), vals.end(), ',', ' ');
        std::stringstream ss(vals);
        ss >> qx >> qy >> qz >> qw;
        rotation_set = true;
      }
    }
  }

  if (!translation_set || !rotation_set) {
    RCLCPP_ERROR(
      get_logger(),
      "YAML file '%s' does not contain valid 'translation' and 'rotation' entries.",
      path.c_str());
    return false;
  }

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3) = tx;
  T(1, 3) = ty;
  T(2, 3) = tz;

  // Eigen::Quaternionf(w, x, y, z)
  Eigen::Quaternionf q(qw, qx, qy, qz);
  q.normalize();  // extra safety
  T.block<3, 3>(0, 0) = q.toRotationMatrix();

  current_transform_ = T;

  RCLCPP_INFO(
    get_logger(),
    "Initial transform from YAML:\n  t = [%.4f, %.4f, %.4f]\n  q(x,y,z,w) = [%.4f, %.4f, %.4f, %.4f]",
    tx, ty, tz, qx, qy, qz, qw);

  return true;
}

}  // namespace multi_cloud_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<multi_cloud_fusion::IcpTfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
