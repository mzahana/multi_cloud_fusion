#include "multi_cloud_fusion/top_view_projector_node.hpp"

#include <limits>
#include <cmath>
#include <algorithm>

#include "pcl_conversions/pcl_conversions.h"

TopViewProjectorNode::TopViewProjectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("top_view_projector_node", options)
{
  // -------- Declare parameters --------
  image_width_  = this->declare_parameter<int>("image_width", 640);
  image_height_ = this->declare_parameter<int>("image_height", 480);

  x_min_ = this->declare_parameter<double>("x_min", -1.0);
  x_max_ = this->declare_parameter<double>("x_max",  1.0);
  y_min_ = this->declare_parameter<double>("y_min", -1.0);
  y_max_ = this->declare_parameter<double>("y_max",  1.0);

  use_max_height_ = this->declare_parameter<bool>("use_max_height", true);

  bg_r_ = static_cast<uint8_t>(this->declare_parameter<int>("bg_r", 0));
  bg_g_ = static_cast<uint8_t>(this->declare_parameter<int>("bg_g", 0));
  bg_b_ = static_cast<uint8_t>(this->declare_parameter<int>("bg_b", 0));

  // NEW: image-level filters
  use_hole_filling_        = this->declare_parameter<bool>("use_hole_filling", false);
  hole_filling_iterations_ = this->declare_parameter<int>("hole_filling_iterations", 1);

  use_smoothing_           = this->declare_parameter<bool>("use_smoothing", false);
  smoothing_kernel_size_   = this->declare_parameter<int>("smoothing_kernel_size", 3);

  RCLCPP_INFO(
    this->get_logger(),
    "TopViewProjectorNode started with:\n"
    "  image_width=%d image_height=%d\n"
    "  ROI: x=[%.3f, %.3f], y=[%.3f, %.3f]\n"
    "  use_max_height=%s\n"
    "  bg_color=(%u,%u,%u)\n"
    "  use_hole_filling=%s (iterations=%d)\n"
    "  use_smoothing=%s (kernel_size=%d)",
    image_width_, image_height_,
    x_min_, x_max_, y_min_, y_max_,
    use_max_height_ ? "true" : "false",
    bg_r_, bg_g_, bg_b_,
    use_hole_filling_ ? "true" : "false", hole_filling_iterations_,
    use_smoothing_ ? "true" : "false", smoothing_kernel_size_);

  // -------- Publishers (topics can be remapped) --------
  rgb_pub_ = this->create_publisher<ImageMsg>("top_view/rgb", rclcpp::SensorDataQoS());
  depth_pub_ = this->create_publisher<ImageMsg>("top_view/depth", rclcpp::SensorDataQoS());

  // -------- Subscriber (input cloud) --------
  cloud_sub_ = this->create_subscription<CloudMsg>(
    "cloud",
    rclcpp::SensorDataQoS(),
    std::bind(&TopViewProjectorNode::cloudCallback, this, std::placeholders::_1));

  // -------- Parameter callback --------
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TopViewProjectorNode::onParameterChange, this, std::placeholders::_1));
}

// ============================================================================
// Runtime parameter callback
// ============================================================================
rcl_interfaces::msg::SetParametersResult TopViewProjectorNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  std::lock_guard<std::mutex> lock(state_mutex_);

  for (const auto & p : params) {
    const auto & name = p.get_name();

    if (name == "image_width") {
      int v = p.as_int();
      if (v <= 0) {
        result.successful = false;
        result.reason = "image_width must be > 0";
        return result;
      }
      image_width_ = v;
      RCLCPP_INFO(this->get_logger(), "Param: image_width=%d", image_width_);
    }
    else if (name == "image_height") {
      int v = p.as_int();
      if (v <= 0) {
        result.successful = false;
        result.reason = "image_height must be > 0";
        return result;
      }
      image_height_ = v;
      RCLCPP_INFO(this->get_logger(), "Param: image_height=%d", image_height_);
    }
    else if (name == "x_min") {
      x_min_ = p.as_double();
      RCLCPP_INFO(this->get_logger(), "Param: x_min=%.3f", x_min_);
    }
    else if (name == "x_max") {
      x_max_ = p.as_double();
      RCLCPP_INFO(this->get_logger(), "Param: x_max=%.3f", x_max_);
    }
    else if (name == "y_min") {
      y_min_ = p.as_double();
      RCLCPP_INFO(this->get_logger(), "Param: y_min=%.3f", y_min_);
    }
    else if (name == "y_max") {
      y_max_ = p.as_double();
      RCLCPP_INFO(this->get_logger(), "Param: y_max=%.3f", y_max_);
    }
    else if (name == "use_max_height") {
      use_max_height_ = p.as_bool();
      RCLCPP_INFO(this->get_logger(), "Param: use_max_height=%s",
                  use_max_height_ ? "true" : "false");
    }
    else if (name == "bg_r") {
      int v = p.as_int();
      bg_r_ = static_cast<uint8_t>(std::clamp(v, 0, 255));
      RCLCPP_INFO(this->get_logger(), "Param: bg_r=%u", bg_r_);
    }
    else if (name == "bg_g") {
      int v = p.as_int();
      bg_g_ = static_cast<uint8_t>(std::clamp(v, 0, 255));
      RCLCPP_INFO(this->get_logger(), "Param: bg_g=%u", bg_g_);
    }
    else if (name == "bg_b") {
      int v = p.as_int();
      bg_b_ = static_cast<uint8_t>(std::clamp(v, 0, 255));
      RCLCPP_INFO(this->get_logger(), "Param: bg_b=%u", bg_b_);
    }
    else if (name == "use_hole_filling") {
      use_hole_filling_ = p.as_bool();
      RCLCPP_INFO(this->get_logger(), "Param: use_hole_filling=%s",
                  use_hole_filling_ ? "true" : "false");
    }
    else if (name == "hole_filling_iterations") {
      int v = p.as_int();
      if (v < 1) v = 1;
      hole_filling_iterations_ = v;
      RCLCPP_INFO(this->get_logger(), "Param: hole_filling_iterations=%d",
                  hole_filling_iterations_);
    }
    else if (name == "use_smoothing") {
      use_smoothing_ = p.as_bool();
      RCLCPP_INFO(this->get_logger(), "Param: use_smoothing=%s",
                  use_smoothing_ ? "true" : "false");
    }
    else if (name == "smoothing_kernel_size") {
      int k = p.as_int();
      if (k < 3) k = 3;
      if (k % 2 == 0) k += 1;  // enforce odd
      smoothing_kernel_size_ = k;
      RCLCPP_INFO(this->get_logger(),
                  "Param: smoothing_kernel_size=%d (odd, >=3 enforced)",
                  smoothing_kernel_size_);
    }
  }

  return result;
}

// ============================================================================
// Cloud callback: project fused cloud to top-view RGB + depth
// ============================================================================
void TopViewProjectorNode::cloudCallback(const CloudMsg::ConstSharedPtr & cloud_msg)
{
  // Early exit if nobody is listening
  if (rgb_pub_->get_subscription_count() == 0 &&
      depth_pub_->get_subscription_count() == 0) {
    return;
  }

  // Copy parameters under lock
  int image_width, image_height;
  double x_min, x_max, y_min, y_max;
  bool use_max_height;
  uint8_t bg_r, bg_g, bg_b;
  bool use_hole_filling;
  int hole_filling_iterations;
  bool use_smoothing;
  int smoothing_kernel_size;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    image_width   = image_width_;
    image_height  = image_height_;
    x_min         = x_min_;
    x_max         = x_max_;
    y_min         = y_min_;
    y_max         = y_max_;
    use_max_height = use_max_height_;
    bg_r          = bg_r_;
    bg_g          = bg_g_;
    bg_b          = bg_b_;

    use_hole_filling      = use_hole_filling_;
    hole_filling_iterations = hole_filling_iterations_;
    use_smoothing         = use_smoothing_;
    smoothing_kernel_size = smoothing_kernel_size_;
  }

  if (x_max <= x_min || y_max <= y_min) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Invalid ROI: x_max<=x_min or y_max<=y_min, skipping projection.");
    return;
  }

  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);

  // Prepare RGB image
  ImageMsg rgb_img;
  rgb_img.header = cloud_msg->header;
  rgb_img.height = static_cast<uint32_t>(image_height);
  rgb_img.width  = static_cast<uint32_t>(image_width);
  rgb_img.encoding = "rgb8";
  rgb_img.is_bigendian = false;
  rgb_img.step = image_width * 3;
  rgb_img.data.resize(rgb_img.height * rgb_img.step, 0);

  // Fill background color
  for (int v = 0; v < image_height; ++v) {
    for (int u = 0; u < image_width; ++u) {
      size_t idx = static_cast<size_t>(v * rgb_img.step + u * 3);
      rgb_img.data[idx + 0] = bg_r;
      rgb_img.data[idx + 1] = bg_g;
      rgb_img.data[idx + 2] = bg_b;
    }
  }

  // Prepare depth buffer (float) and depth image message
  const size_t n_pixels = static_cast<size_t>(image_width * image_height);
  std::vector<float> depth_buffer(n_pixels);

  float init_val = use_max_height
    ? -std::numeric_limits<float>::infinity()
    :  std::numeric_limits<float>::infinity();

  std::fill(depth_buffer.begin(), depth_buffer.end(), init_val);

  // Project points
  const double x_range = x_max - x_min;
  const double y_range = y_max - y_min;

  for (const auto & pt : cloud.points) {
    double x = pt.x;
    double y = pt.y;
    float  z = pt.z;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    if (x < x_min || x > x_max || y < y_min || y > y_max) {
      continue;
    }

    // Map to pixel indices
    double u_f = (x - x_min) / x_range * (image_width - 1);
    double v_f = (y_max - y) / y_range * (image_height - 1);  // flip y for image coordinates

    int u = static_cast<int>(std::round(u_f));
    int v = static_cast<int>(std::round(v_f));

    if (u < 0 || u >= image_width || v < 0 || v >= image_height) {
      continue;
    }

    size_t idx = static_cast<size_t>(v * image_width + u);

    // Decide whether to update this pixel
    bool take_point = false;
    if (use_max_height) {
      if (z > depth_buffer[idx]) {
        take_point = true;
      }
    } else {
      if (z < depth_buffer[idx]) {
        take_point = true;
      }
    }

    if (!take_point) {
      continue;
    }

    depth_buffer[idx] = z;

    // Update RGB image pixel
    size_t rgb_idx = static_cast<size_t>(v * rgb_img.step + u * 3);
    rgb_img.data[rgb_idx + 0] = pt.r;
    rgb_img.data[rgb_idx + 1] = pt.g;
    rgb_img.data[rgb_idx + 2] = pt.b;
  }

  // ---------------- Optional 2D hole filling ----------------
  if (use_hole_filling && hole_filling_iterations > 0) {
    std::vector<float> depth_filled = depth_buffer;
    std::vector<uint8_t> rgb_filled = rgb_img.data;

    auto is_hole = [use_max_height](float val) {
      return std::isinf(val);  // we used +/-inf to mark empty pixels
    };

    for (int it = 0; it < hole_filling_iterations; ++it) {
      std::vector<float> depth_next = depth_filled;
      std::vector<uint8_t> rgb_next = rgb_filled;

      for (int v = 0; v < image_height; ++v) {
        for (int u = 0; u < image_width; ++u) {
          size_t idx = static_cast<size_t>(v * image_width + u);
          if (!is_hole(depth_filled[idx])) {
            continue;  // already has data
          }

          float sum_depth = 0.0f;
          int   count = 0;
          int   sum_r = 0, sum_g = 0, sum_b = 0;

          // 4-neighborhood (up, down, left, right)
          const int du[4] = { 1, -1, 0, 0 };
          const int dv[4] = { 0, 0, 1, -1 };

          for (int k = 0; k < 4; ++k) {
            int uu = u + du[k];
            int vv = v + dv[k];
            if (uu < 0 || uu >= image_width || vv < 0 || vv >= image_height) {
              continue;
            }
            size_t idx2 = static_cast<size_t>(vv * image_width + uu);
            float d2 = depth_filled[idx2];
            if (is_hole(d2) || !std::isfinite(d2)) {
              continue;
            }
            sum_depth += d2;
            ++count;

            size_t rgb_idx2 = static_cast<size_t>(vv * rgb_img.step + uu * 3);
            sum_r += rgb_filled[rgb_idx2 + 0];
            sum_g += rgb_filled[rgb_idx2 + 1];
            sum_b += rgb_filled[rgb_idx2 + 2];
          }

          if (count > 0) {
            depth_next[idx] = sum_depth / static_cast<float>(count);
            size_t rgb_idx = static_cast<size_t>(v * rgb_img.step + u * 3);
            rgb_next[rgb_idx + 0] = static_cast<uint8_t>(sum_r / count);
            rgb_next[rgb_idx + 1] = static_cast<uint8_t>(sum_g / count);
            rgb_next[rgb_idx + 2] = static_cast<uint8_t>(sum_b / count);
          }
        }
      }

      depth_filled.swap(depth_next);
      rgb_filled.swap(rgb_next);
    }

    depth_buffer.swap(depth_filled);
    rgb_img.data.swap(rgb_filled);
  }

  // ---------------- Optional 2D smoothing (box filter) ----------------
  if (use_smoothing && smoothing_kernel_size > 1) {
    int radius = smoothing_kernel_size / 2;

    std::vector<float> depth_smoothed = depth_buffer;
    std::vector<uint8_t> rgb_smoothed = rgb_img.data;

    for (int v = 0; v < image_height; ++v) {
      for (int u = 0; u < image_width; ++u) {
        size_t idx = static_cast<size_t>(v * image_width + u);
        float center = depth_buffer[idx];
        if (!std::isfinite(center)) {
          continue;  // keep holes/NaNs as they are
        }

        float sum_depth = 0.0f;
        int   count = 0;
        int   sum_r = 0, sum_g = 0, sum_b = 0;

        for (int dv = -radius; dv <= radius; ++dv) {
          int vv = v + dv;
          if (vv < 0 || vv >= image_height) continue;
          for (int du = -radius; du <= radius; ++du) {
            int uu = u + du;
            if (uu < 0 || uu >= image_width) continue;

            size_t idx2 = static_cast<size_t>(vv * image_width + uu);
            float d2 = depth_buffer[idx2];
            if (!std::isfinite(d2)) continue;

            sum_depth += d2;
            ++count;

            size_t rgb_idx2 = static_cast<size_t>(vv * rgb_img.step + uu * 3);
            sum_r += rgb_img.data[rgb_idx2 + 0];
            sum_g += rgb_img.data[rgb_idx2 + 1];
            sum_b += rgb_img.data[rgb_idx2 + 2];
          }
        }

        if (count > 0) {
          depth_smoothed[idx] = sum_depth / static_cast<float>(count);
          size_t rgb_idx = static_cast<size_t>(v * rgb_img.step + u * 3);
          rgb_smoothed[rgb_idx + 0] = static_cast<uint8_t>(sum_r / count);
          rgb_smoothed[rgb_idx + 1] = static_cast<uint8_t>(sum_g / count);
          rgb_smoothed[rgb_idx + 2] = static_cast<uint8_t>(sum_b / count);
        }
      }
    }

    depth_buffer.swap(depth_smoothed);
    rgb_img.data.swap(rgb_smoothed);
  }

  // Prepare depth image msg from depth_buffer
  ImageMsg depth_img;
  depth_img.header = cloud_msg->header;
  depth_img.height = static_cast<uint32_t>(image_height);
  depth_img.width  = static_cast<uint32_t>(image_width);
  depth_img.encoding = "32FC1";
  depth_img.is_bigendian = false;
  depth_img.step = image_width * sizeof(float);
  depth_img.data.resize(depth_img.height * depth_img.step);

  float * depth_ptr = reinterpret_cast<float*>(depth_img.data.data());

  for (size_t idx = 0; idx < n_pixels; ++idx) {
    float val = depth_buffer[idx];
    if (!std::isfinite(val)) {
      depth_ptr[idx] = std::numeric_limits<float>::quiet_NaN();
    } else {
      depth_ptr[idx] = val;
    }
  }

  rgb_pub_->publish(rgb_img);
  depth_pub_->publish(depth_img);

  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "Projected cloud with %zu points to image %dx%d",
    cloud.points.size(), image_width, image_height);
}
