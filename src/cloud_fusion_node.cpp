// @file cloud_fusion_node.cpp
#include "multi_cloud_fusion/cloud_fusion_node.hpp"

#include <functional>
#include <chrono>
#include <omp.h>
#include <vector>
#include <cstring> // for memcpy

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
// No PCL headers needed for raw merge!

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
// KERNEL: Raw Buffer Transform (No Checks, No Filters, Just Math)
// ============================================================================
void transformRaw(
    const sensor_msgs::msg::PointCloud2& input,
    sensor_msgs::msg::PointCloud2& output,
    size_t out_offset_bytes,
    const Eigen::Matrix4f& mat)
{
    // 1. Validate Fields (Find XYZ offsets once)
    int x_off = -1, y_off = -1, z_off = -1;
    for (const auto& f : input.fields) {
        if (f.name == "x") x_off = f.offset;
        else if (f.name == "y") y_off = f.offset;
        else if (f.name == "z") z_off = f.offset;
    }

    if (x_off < 0 || y_off < 0 || z_off < 0) return; // Should not happen with standard cameras

    const size_t point_step = input.point_step;
    const size_t num_points = input.width * input.height;
    const uint8_t* src_ptr = input.data.data();
    uint8_t* dst_ptr_base  = output.data.data() + out_offset_bytes;

    // 2. Pre-compute matrix vars for registers
    float m00 = mat(0,0), m01 = mat(0,1), m02 = mat(0,2), m03 = mat(0,3);
    float m10 = mat(1,0), m11 = mat(1,1), m12 = mat(1,2), m13 = mat(1,3);
    float m20 = mat(2,0), m21 = mat(2,1), m22 = mat(2,2), m23 = mat(2,3);

    // 3. Parallel Loop
    #pragma omp parallel for schedule(static)
    for (size_t i = 0; i < num_points; ++i) {
        const uint8_t* p_in = src_ptr + i * point_step;
        uint8_t* p_out      = dst_ptr_base + i * point_step;

        // A. Bulk Copy (Copies RGB, Intensity, Rings, etc.)
        memcpy(p_out, p_in, point_step);

        // B. Read XYZ
        float x = *reinterpret_cast<const float*>(p_in + x_off);
        float y = *reinterpret_cast<const float*>(p_in + y_off);
        float z = *reinterpret_cast<const float*>(p_in + z_off);

        // C. Transform & Write
        *reinterpret_cast<float*>(p_out + x_off) = m00*x + m01*y + m02*z + m03;
        *reinterpret_cast<float*>(p_out + y_off) = m10*x + m11*y + m12*z + m13;
        *reinterpret_cast<float*>(p_out + z_off) = m20*x + m21*y + m22*z + m23;
    }
}

// ============================================================================
// Constructor
// ============================================================================
CloudFusionNode::CloudFusionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("cloud_fusion_node", options),
  cloud1_sub_(), cloud2_sub_(), have_tf1_(false), have_tf2_(false)
{
  target_frame_ = this->declare_parameter<std::string>("target_frame", "world");

  // Note: All other parameters (voxel, crop, outlier) REMOVED for speed focus.
  
  RCLCPP_INFO(this->get_logger(), "CloudFusionNode: ULTRA FAST MODE (No Filters).");

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), 
    std::bind(&CloudFusionNode::tfTimerCallback, this));

  fused_pub_ = this->create_publisher<CloudMsg>("fused_cloud", rclcpp::SensorDataQoS());

  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  cloud1_sub_.subscribe(this, "cloud1", qos.get_rmw_qos_profile());
  cloud2_sub_.subscribe(this, "cloud2", qos.get_rmw_qos_profile());

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), cloud1_sub_, cloud2_sub_);

  sync_->registerCallback(std::bind(&CloudFusionNode::callback, this, _1, _2));

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CloudFusionNode::onParameterChange, this, _1));
}

// ============================================================================
// TF Timer
// ============================================================================
void CloudFusionNode::tfTimerCallback()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (frame1_.empty() || frame2_.empty()) return;
  if (have_tf1_ && have_tf2_) return;

  auto lookup = [&](const std::string& source, Eigen::Matrix4f& target_mat, bool& ready_flag) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(target_frame_, source, tf2::TimePointZero);
      target_mat = transformMsgToEigen(tf_stamped.transform);
      ready_flag = true;
      RCLCPP_INFO(this->get_logger(), "TF ready: %s -> %s", source.c_str(), target_frame_.c_str());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for TF %s", source.c_str());
    }
  };

  if (!have_tf1_) lookup(frame1_, tf1_matrix_, have_tf1_);
  if (!have_tf2_) lookup(frame2_, tf2_matrix_, have_tf2_);
}

// ============================================================================
// CALLBACK: The "Bare Metal" Merge
// ============================================================================
void CloudFusionNode::callback(
  const CloudMsg::ConstSharedPtr & cloud1,
  const CloudMsg::ConstSharedPtr & cloud2)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  if (!fused_pub_) return;

  // 1. Get Transform State
  Eigen::Matrix4f tf1, tf2;
  std::string target_frame;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (frame1_.empty()) frame1_ = cloud1->header.frame_id;
    if (frame2_.empty()) frame2_ = cloud2->header.frame_id;
    if (!have_tf1_ || !have_tf2_) return;
    tf1 = tf1_matrix_;
    tf2 = tf2_matrix_;
    target_frame = target_frame_;
  }

  // 2. Allocation (Reuse Header from Cloud1)
  CloudMsg fused_msg;
  fused_msg.header       = cloud1->header;
  fused_msg.header.frame_id = target_frame;
  fused_msg.fields       = cloud1->fields;
  fused_msg.point_step   = cloud1->point_step;
  fused_msg.is_bigendian = cloud1->is_bigendian;
  fused_msg.is_dense     = cloud1->is_dense && cloud2->is_dense;

  // Resize vector once (Fastest allocation)
  size_t size1 = cloud1->data.size();
  size_t size2 = cloud2->data.size();
  fused_msg.data.resize(size1 + size2);
  
  // Update width/height to Unorganized (1D)
  fused_msg.width  = cloud1->width * cloud1->height + cloud2->width * cloud2->height;
  fused_msg.height = 1;

  auto t_alloc = std::chrono::high_resolution_clock::now();

  // 3. Parallel Processing: Transform directly into output buffer
  #pragma omp parallel sections
  {
    #pragma omp section
    {
       // Write Cloud1 to beginning of buffer
       transformRaw(*cloud1, fused_msg, 0, tf1);
    }
    #pragma omp section
    {
       // Write Cloud2 to offset position of buffer
       transformRaw(*cloud2, fused_msg, size1, tf2);
    }
  }

  auto t_process = std::chrono::high_resolution_clock::now();

  // 4. Publish
  fused_pub_->publish(fused_msg);

  auto t_end = std::chrono::high_resolution_clock::now();

  // --- Profiling ---
  double ms_alloc = std::chrono::duration<double, std::milli>(t_alloc - t_start).count();
  double ms_proc  = std::chrono::duration<double, std::milli>(t_process - t_alloc).count();
  double ms_total = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "FastMerge: %.2f ms (Alloc: %.2f, Math: %.2f) | Hz: %.1f", 
    ms_total, ms_alloc, ms_proc, 1000.0/ms_total);
}

// ============================================================================
// Parameter Callback
// ============================================================================
rcl_interfaces::msg::SetParametersResult CloudFusionNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  std::lock_guard<std::mutex> lock(state_mutex_);

  for (const auto & p : params) {
    if (p.get_name() == "target_frame") {
      target_frame_ = p.as_string();
      have_tf1_ = false;
      have_tf2_ = false;
      RCLCPP_INFO(this->get_logger(), "Target Frame Changed: %s", target_frame_.c_str());
    }
  }
  return result;
}