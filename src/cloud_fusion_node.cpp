#include "multi_cloud_fusion/cloud_fusion_node.hpp"

#include <functional>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/common/transforms.h"
#include "tf2/exceptions.h"

using std::placeholders::_1;
using std::placeholders::_2;

// ---------------- Helper: geometry_msgs::Transform -> Eigen::Matrix4f ----------------
static Eigen::Matrix4f transformMsgToEigen(const geometry_msgs::msg::Transform & t)
{
  Eigen::Quaternionf q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  Eigen::Vector3f trans(t.translation.x, t.translation.y, t.translation.z);

  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat.block<3,3>(0,0) = q.toRotationMatrix();
  mat.block<3,1>(0,3) = trans;

  return mat;
}

// ============================================================================
// Constructor
// ============================================================================
CloudFusionNode::CloudFusionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("cloud_fusion_node", options),
  cloud1_sub_(),
  cloud2_sub_(),
  have_tf1_(false),
  have_tf2_(false)
{
  // -------- Declare runtime parameters --------
  target_frame_    = this->declare_parameter<std::string>("target_frame", "world");
  use_voxel_grid_  = this->declare_parameter<bool>("use_voxel_grid", false);
  voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.005);

  // NEW: 3D filters
  use_z_crop_      = this->declare_parameter<bool>("use_z_crop", false);
  z_min_           = this->declare_parameter<double>("z_min", -1.0);
  z_max_           = this->declare_parameter<double>("z_max",  2.0);

  use_outlier_removal_   = this->declare_parameter<bool>("use_outlier_removal", false);
  outlier_radius_        = this->declare_parameter<double>("outlier_radius", 0.01);
  outlier_min_neighbors_ = this->declare_parameter<int>("outlier_min_neighbors", 5);

  RCLCPP_INFO(
    this->get_logger(),
    "CloudFusionNode started:\n"
    "  target_frame: %s\n"
    "  use_voxel_grid: %s (leaf=%.4f)\n"
    "  use_z_crop: %s (z_min=%.3f, z_max=%.3f)\n"
    "  use_outlier_removal: %s (radius=%.3f, min_neighbors=%d)",
    target_frame_.c_str(),
    use_voxel_grid_ ? "true" : "false", voxel_leaf_size_,
    use_z_crop_ ? "true" : "false", z_min_, z_max_,
    use_outlier_removal_ ? "true" : "false", outlier_radius_, outlier_min_neighbors_);

  // -------- TF setup --------
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // TF timer: low-frequency static transform lookup
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&CloudFusionNode::tfTimerCallback, this));

  // -------- Publisher (output topic can be remapped) --------
  fused_pub_ = this->create_publisher<CloudMsg>("fused_cloud", rclcpp::SensorDataQoS());

  // -------- Subscribers --------
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  cloud1_sub_.subscribe(this, "cloud1", qos.get_rmw_qos_profile());
  cloud2_sub_.subscribe(this, "cloud2", qos.get_rmw_qos_profile());

  // -------- Synchronizer (ApproximateTime, fixed queue size) --------
  const uint32_t queue_size = 10;
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(queue_size), cloud1_sub_, cloud2_sub_);

  sync_->registerCallback(
    std::bind(&CloudFusionNode::callback, this, _1, _2));

  // -------- Parameter callback --------
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CloudFusionNode::onParameterChange, this, _1));
}

// ============================================================================
// TF timer: periodically cache static transforms (thread-safe)
// ============================================================================
void CloudFusionNode::tfTimerCallback()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (frame1_.empty() || frame2_.empty()) {
    // We don't even know the frames yet (no synchronized messages seen)
    return;
  }

  // If we already have both transforms, nothing to do
  if (have_tf1_ && have_tf2_) {
    return;
  }

  // Try to resolve TF for frame1_
  if (!have_tf1_) {
    try {
      auto tf1_stamped = tf_buffer_->lookupTransform(
        target_frame_, frame1_, tf2::TimePointZero);

      tf1_matrix_ = transformMsgToEigen(tf1_stamped.transform);
      have_tf1_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "TF timer: cached static transform %s -> %s",
        frame1_.c_str(), target_frame_.c_str());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "TF timer: failed to lookup %s -> %s: %s",
        frame1_.c_str(), target_frame_.c_str(), ex.what());
    }
  }

  // Try to resolve TF for frame2_
  if (!have_tf2_) {
    try {
      auto tf2_stamped = tf_buffer_->lookupTransform(
        target_frame_, frame2_, tf2::TimePointZero);

      tf2_matrix_ = transformMsgToEigen(tf2_stamped.transform);
      have_tf2_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "TF timer: cached static transform %s -> %s",
        frame2_.c_str(), target_frame_.c_str());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "TF timer: failed to lookup %s -> %s: %s",
        frame2_.c_str(), target_frame_.c_str(), ex.what());
    }
  }
}

// ============================================================================
// Synchronized fusion callback (thread-safe via mutex on shared state)
// ============================================================================
void CloudFusionNode::callback(
  const CloudMsg::ConstSharedPtr & cloud1,
  const CloudMsg::ConstSharedPtr & cloud2)
{
  if (!fused_pub_) {
    return;
  }

  // Convert incoming ROS messages to PCL first (no need to lock for this)
  pcl::PointCloud<pcl::PointXYZRGB> pcl1;
  pcl::PointCloud<pcl::PointXYZRGB> pcl2;
  pcl::fromROSMsg(*cloud1, pcl1);
  pcl::fromROSMsg(*cloud2, pcl2);

  // Local copies of shared state so we can unlock during heavy processing
  Eigen::Matrix4f tf1, tf2;
  std::string target_frame;
  bool use_voxel_grid;
  double voxel_leaf_size;
  bool local_have_tf1, local_have_tf2;
  bool use_z_crop;
  double z_min, z_max;
  bool use_outlier_removal;
  double outlier_radius;
  int outlier_min_neighbors;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (frame1_.empty()) {
      frame1_ = cloud1->header.frame_id;
      RCLCPP_INFO(
        this->get_logger(),
        "Detected cloud1 frame_id: '%s'", frame1_.c_str());
    }

    if (frame2_.empty()) {
      frame2_ = cloud2->header.frame_id;
      RCLCPP_INFO(
        this->get_logger(),
        "Detected cloud2 frame_id: '%s'", frame2_.c_str());
    }

    local_have_tf1 = have_tf1_;
    local_have_tf2 = have_tf2_;

    if (!local_have_tf1 || !local_have_tf2) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Transforms to target_frame '%s' not ready; skipping fusion.",
        target_frame_.c_str());
      return;
    }

    tf1 = tf1_matrix_;
    tf2 = tf2_matrix_;
    target_frame    = target_frame_;
    use_voxel_grid  = use_voxel_grid_;
    voxel_leaf_size = voxel_leaf_size_;

    use_z_crop           = use_z_crop_;
    z_min                = z_min_;
    z_max                = z_max_;
    use_outlier_removal  = use_outlier_removal_;
    outlier_radius       = outlier_radius_;
    outlier_min_neighbors = outlier_min_neighbors_;
  }

  // Now we do the heavy work *outside* the lock, using local copies
  pcl::PointCloud<pcl::PointXYZRGB> pcl1_tf, pcl2_tf;
  pcl::transformPointCloud(pcl1, pcl1_tf, tf1);
  pcl::transformPointCloud(pcl2, pcl2_tf, tf2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current(new pcl::PointCloud<pcl::PointXYZRGB>);
  *current = pcl1_tf;
  *current += pcl2_tf;

  // 1) Optional voxel grid downsampling
  if (use_voxel_grid && voxel_leaf_size > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(current);
    float leaf = static_cast<float>(voxel_leaf_size);
    voxel.setLeafSize(leaf, leaf, leaf);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel.filter(*tmp);
    current = tmp;
  }

  // 2) Optional z-crop (keep only points z in [z_min, z_max])
  if (use_z_crop && z_max > z_min) {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(current);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(static_cast<float>(z_min), static_cast<float>(z_max));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pass.filter(*tmp);
    current = tmp;
  }

  // 3) Optional radius outlier removal
  if (use_outlier_removal && outlier_radius > 0.0 && outlier_min_neighbors > 0) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
    ror.setInputCloud(current);
    ror.setRadiusSearch(static_cast<float>(outlier_radius));
    ror.setMinNeighborsInRadius(outlier_min_neighbors);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    ror.filter(*tmp);
    current = tmp;
  }

  CloudMsg out;
  pcl::toROSMsg(*current, out);
  out.header.stamp    = cloud1->header.stamp;   // or combine with cloud2's stamp
  out.header.frame_id = target_frame;

  fused_pub_->publish(out);

  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Fusion stats: cloud1=%zu cloud2=%zu merged=%zu output=%zu",
    pcl1.size(), pcl2.size(), pcl1_tf.size() + pcl2_tf.size(), current->size());
}

// ============================================================================
// Runtime parameter callback (thread-safe)
// ============================================================================
rcl_interfaces::msg::SetParametersResult CloudFusionNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  std::lock_guard<std::mutex> lock(state_mutex_);

  for (const auto & p : params) {
    const std::string & name = p.get_name();

    if (name == "target_frame") {
      target_frame_ = p.as_string();
      have_tf1_ = false;
      have_tf2_ = false;
      RCLCPP_INFO(
        this->get_logger(),
        "Parameter update: target_frame=%s (TFs invalidated)",
        target_frame_.c_str());
    }
    else if (name == "use_voxel_grid") {
      use_voxel_grid_ = p.as_bool();
      RCLCPP_INFO(
        this->get_logger(),
        "Parameter update: use_voxel_grid=%s",
        use_voxel_grid_ ? "true" : "false");
    }
    else if (name == "voxel_leaf_size") {
      voxel_leaf_size_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "Parameter update: voxel_leaf_size=%.4f",
        voxel_leaf_size_);
      if (voxel_leaf_size_ <= 0.0 && use_voxel_grid_) {
        RCLCPP_WARN(
          this->get_logger(),
          "voxel_leaf_size <= 0 while use_voxel_grid=true; no downsampling will occur.");
      }
    }
    else if (name == "use_z_crop") {
      use_z_crop_ = p.as_bool();
      RCLCPP_INFO(
        this->get_logger(),
        "Parameter update: use_z_crop=%s",
        use_z_crop_ ? "true" : "false");
    }
    else if (name == "z_min") {
      z_min_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "Parameter update: z_min=%.3f", z_min_);
    }
    else if (name == "z_max") {
      z_max_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "Parameter update: z_max=%.3f", z_max_);
    }
    else if (name == "use_outlier_removal") {
      use_outlier_removal_ = p.as_bool();
      RCLCPP_INFO(
        this->get_logger(),
        "Parameter update: use_outlier_removal=%s",
        use_outlier_removal_ ? "true" : "false");
    }
    else if (name == "outlier_radius") {
      outlier_radius_ = p.as_double();
      RCLCPP_INFO(
        this->get_logger(),
        "Parameter update: outlier_radius=%.3f", outlier_radius_);
    }
    else if (name == "outlier_min_neighbors") {
      outlier_min_neighbors_ = p.as_int();
      RCLCPP_INFO(
        this->get_logger(),
        "Parameter update: outlier_min_neighbors=%d", outlier_min_neighbors_);
    }
  }

  return result;
}
