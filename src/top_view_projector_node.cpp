#include "multi_cloud_fusion/top_view_projector_node.hpp"

#include <limits>
#include <cmath>
#include <algorithm>
#include <vector>
#include <cstring>
#include <atomic>
#include <omp.h>

// ============================================================================
// Atomic Helper: Lock-free update of (Depth + Index)
// ============================================================================
union DepthIndex {
    uint64_t val;
    struct {
        int32_t index; // Low 32
        int32_t depth; // High 32 (Scaled Integer)
    } parts;
};

inline void atomic_update_max(std::atomic<uint64_t>& current, uint64_t candidate_val) {
    uint64_t prev_val = current.load(std::memory_order_relaxed);
    while (true) {
        DepthIndex prev, cand;
        prev.val = prev_val;
        cand.val = candidate_val;
        if (cand.parts.depth <= prev.parts.depth) break;
        if (current.compare_exchange_weak(prev_val, candidate_val, 
                                          std::memory_order_release, 
                                          std::memory_order_relaxed)) {
            break;
        }
    }
}

inline void atomic_update_min(std::atomic<uint64_t>& current, uint64_t candidate_val) {
    uint64_t prev_val = current.load(std::memory_order_relaxed);
    while (true) {
        DepthIndex prev, cand;
        prev.val = prev_val;
        cand.val = candidate_val;
        if (prev.parts.index != -1 && cand.parts.depth >= prev.parts.depth) break;
        if (current.compare_exchange_weak(prev_val, candidate_val, 
                                          std::memory_order_release, 
                                          std::memory_order_relaxed)) {
            break;
        }
    }
}

TopViewProjectorNode::TopViewProjectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("top_view_projector_node", options)
{
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

  use_hole_filling_        = this->declare_parameter<bool>("use_hole_filling", false);
  hole_filling_iterations_ = this->declare_parameter<int>("hole_filling_iterations", 1);
  use_smoothing_           = this->declare_parameter<bool>("use_smoothing", false);
  smoothing_kernel_size_   = this->declare_parameter<int>("smoothing_kernel_size", 3);

  RCLCPP_INFO(this->get_logger(), "TopViewProjectorNode started (Atomic + BGR Fix).");

  rgb_pub_   = this->create_publisher<ImageMsg>("top_view/rgb",   rclcpp::SensorDataQoS());
  depth_pub_ = this->create_publisher<ImageMsg>("top_view/depth", rclcpp::SensorDataQoS());
  index_pub_ = this->create_publisher<ImageMsg>("top_view/index_map", rclcpp::SensorDataQoS());

  cloud_sub_ = this->create_subscription<CloudMsg>(
    "cloud",
    rclcpp::SensorDataQoS(),
    std::bind(&TopViewProjectorNode::cloudCallback, this, std::placeholders::_1));

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TopViewProjectorNode::onParameterChange, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult TopViewProjectorNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true; result.reason = "success";
  std::lock_guard<std::mutex> lock(state_mutex_);

  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (name == "image_width") image_width_ = std::max(1, static_cast<int>(p.as_int()));
    else if (name == "image_height") image_height_ = std::max(1, static_cast<int>(p.as_int()));
    else if (name == "x_min") x_min_ = p.as_double();
    else if (name == "x_max") x_max_ = p.as_double();
    else if (name == "y_min") y_min_ = p.as_double();
    else if (name == "y_max") y_max_ = p.as_double();
    else if (name == "use_max_height") use_max_height_ = p.as_bool();
    else if (name == "bg_r") bg_r_ = static_cast<uint8_t>(std::clamp(static_cast<int>(p.as_int()), 0, 255));
    else if (name == "bg_g") bg_g_ = static_cast<uint8_t>(std::clamp(static_cast<int>(p.as_int()), 0, 255));
    else if (name == "bg_b") bg_b_ = static_cast<uint8_t>(std::clamp(static_cast<int>(p.as_int()), 0, 255));
    else if (name == "use_hole_filling") use_hole_filling_ = p.as_bool();
    else if (name == "hole_filling_iterations") hole_filling_iterations_ = std::max(1, static_cast<int>(p.as_int()));
    else if (name == "use_smoothing") use_smoothing_ = p.as_bool();
    else if (name == "smoothing_kernel_size") {
      int k = static_cast<int>(p.as_int());
      smoothing_kernel_size_ = (k < 3) ? 3 : (k % 2 == 0 ? k + 1 : k);
    }
  }
  return result;
}

void TopViewProjectorNode::cloudCallback(const CloudMsg::ConstSharedPtr & cloud_msg)
{
  if (rgb_pub_->get_subscription_count() == 0 &&
      depth_pub_->get_subscription_count() == 0 &&
      index_pub_->get_subscription_count() == 0) return;

  // 1. Snapshot Params
  int width, height;
  double x_min, x_max, y_min, y_max;
  bool use_max;
  uint8_t bg_r, bg_g, bg_b;
  bool do_hole, do_smooth;
  int hole_iters, smooth_k;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    width = image_width_; height = image_height_;
    x_min = x_min_; x_max = x_max_;
    y_min = y_min_; y_max = y_max_;
    use_max = use_max_height_;
    bg_r = bg_r_; bg_g = bg_g_; bg_b = bg_b_;
    do_hole = use_hole_filling_; hole_iters = hole_filling_iterations_;
    do_smooth = use_smoothing_; smooth_k = smoothing_kernel_size_;
  }

  if (x_max <= x_min || y_max <= y_min) return;

  // 2. Setup Atomic Grid
  size_t n_pixels = width * height;
  
  DepthIndex init_val;
  init_val.parts.index = -1;
  init_val.parts.depth = use_max ? std::numeric_limits<int32_t>::min() : std::numeric_limits<int32_t>::max();
  
  std::vector<std::atomic<uint64_t>> atomic_grid(n_pixels);
  
  #pragma omp parallel for schedule(static)
  for (size_t i = 0; i < n_pixels; ++i) {
      atomic_grid[i].store(init_val.val, std::memory_order_relaxed);
  }

  // 3. Raw Data Access
  const size_t point_step = cloud_msg->point_step;
  const size_t num_points = cloud_msg->width * cloud_msg->height;
  const uint8_t* raw_data = cloud_msg->data.data();

  int x_off = -1, y_off = -1, z_off = -1, rgb_off = -1;
  for (const auto& f : cloud_msg->fields) {
    if (f.name == "x") x_off = f.offset;
    else if (f.name == "y") y_off = f.offset;
    else if (f.name == "z") z_off = f.offset;
    else if (f.name == "rgb") rgb_off = f.offset;
  }
  if (x_off < 0 || y_off < 0 || z_off < 0) return;

  double x_scale = (width - 1) / (x_max - x_min);
  double y_scale = (height - 1) / (y_max - y_min);

  // 4. Parallel Projection (ATOMIC)
  #pragma omp parallel for schedule(static)
  for (size_t i = 0; i < num_points; ++i) {
      const uint8_t* pt_ptr = raw_data + i * point_step;
      
      float x = *reinterpret_cast<const float*>(pt_ptr + x_off);
      float y = *reinterpret_cast<const float*>(pt_ptr + y_off);
      float z = *reinterpret_cast<const float*>(pt_ptr + z_off);

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      if (x < x_min || x > x_max || y < y_min || y > y_max) continue;

      int u = static_cast<int>((x - x_min) * x_scale + 0.5f);
      int v = static_cast<int>((y_max - y) * y_scale + 0.5f);

      if (u < 0 || u >= width || v < 0 || v >= height) continue;

      int32_t z_int = static_cast<int32_t>(z * 10000.0f); 

      DepthIndex cand;
      cand.parts.index = static_cast<int32_t>(i);
      cand.parts.depth = z_int;

      size_t idx = v * width + u;
      
      if (use_max) atomic_update_max(atomic_grid[idx], cand.val);
      else atomic_update_min(atomic_grid[idx], cand.val);
  }

  // 5. Generate Output Buffers (Decode + Color Swap)
  std::vector<float> final_depth(n_pixels);
  std::vector<uint8_t> final_rgb(n_pixels * 3);
  std::vector<int32_t> final_index(n_pixels);

  #pragma omp parallel for schedule(static)
  for (size_t i = 0; i < n_pixels; ++i) {
      uint64_t val = atomic_grid[i].load(std::memory_order_relaxed);
      DepthIndex di;
      di.val = val;

      if (di.parts.index != -1) {
          final_index[i] = di.parts.index;
          final_depth[i] = static_cast<float>(di.parts.depth) / 10000.0f;
          
          if (rgb_off >= 0) {
              const uint8_t* pt_color = raw_data + di.parts.index * point_step + rgb_off;
              // === FIX: SWAP RED AND BLUE ===
              // Memory is BGR, Image is RGB
              final_rgb[i*3 + 0] = pt_color[2]; // R
              final_rgb[i*3 + 1] = pt_color[1]; // G
              final_rgb[i*3 + 2] = pt_color[0]; // B
          } else {
              final_rgb[i*3+0] = 255; final_rgb[i*3+1] = 255; final_rgb[i*3+2] = 255;
          }
      } else {
          final_index[i] = -1;
          final_depth[i] = std::numeric_limits<float>::quiet_NaN();
          final_rgb[i*3 + 0] = bg_r;
          final_rgb[i*3 + 1] = bg_g;
          final_rgb[i*3 + 2] = bg_b;
      }
  }

  // 6. FAST FILTERS
  if (do_hole && hole_iters > 0) {
    std::vector<float> d_buf = final_depth; 
    std::vector<uint8_t> c_buf = final_rgb;

    float* d_src = final_depth.data();
    float* d_dst = d_buf.data();
    uint8_t* c_src = final_rgb.data();
    uint8_t* c_dst = c_buf.data();

    for (int iter = 0; iter < hole_iters; ++iter) {
        #pragma omp parallel for collapse(2) schedule(static)
        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                size_t idx = v * width + u;
                if (std::isfinite(d_src[idx])) {
                    d_dst[idx] = d_src[idx];
                    size_t c = idx*3;
                    c_dst[c] = c_src[c]; c_dst[c+1] = c_src[c+1]; c_dst[c+2] = c_src[c+2];
                    continue;
                }
                float sum_d = 0; int sum_r=0, sum_g=0, sum_b=0, count=0;
                const int du[] = {0, 0, -1, 1};
                const int dv[] = {-1, 1, 0, 0};
                for(int k=0; k<4; ++k) {
                    int nu = u+du[k], nv = v+dv[k];
                    if (nu >= 0 && nu < width && nv >= 0 && nv < height) {
                        size_t nidx = nv * width + nu;
                        float val = d_src[nidx];
                        if (std::isfinite(val)) {
                            sum_d += val;
                            size_t nc = nidx*3;
                            sum_r += c_src[nc]; sum_g += c_src[nc+1]; sum_b += c_src[nc+2];
                            count++;
                        }
                    }
                }
                if (count > 0) {
                    d_dst[idx] = sum_d / count;
                    size_t c = idx*3;
                    c_dst[c] = (uint8_t)(sum_r/count); 
                    c_dst[c+1] = (uint8_t)(sum_g/count); 
                    c_dst[c+2] = (uint8_t)(sum_b/count);
                } else {
                    d_dst[idx] = std::numeric_limits<float>::quiet_NaN();
                }
            }
        }
        std::swap(d_src, d_dst);
        std::swap(c_src, c_dst);
    }
    if (d_src != final_depth.data()) {
        final_depth = d_buf; 
        final_rgb = c_buf;
    }
  }

  if (do_smooth && smooth_k >= 3) {
    int rad = smooth_k / 2;
    std::vector<float> d_out(n_pixels);
    std::vector<uint8_t> c_out(n_pixels * 3);
    const float* d_in = final_depth.data();
    const uint8_t* c_in = final_rgb.data();

    #pragma omp parallel for collapse(2) schedule(static)
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            size_t idx = v * width + u;
            if (!std::isfinite(d_in[idx])) {
                d_out[idx] = d_in[idx];
                size_t c = idx*3;
                c_out[c] = c_in[c]; c_out[c+1] = c_in[c+1]; c_out[c+2] = c_in[c+2];
                continue;
            }
            double sum_d = 0; int sum_r=0, sum_g=0, sum_b=0, count=0;
            for (int r = -rad; r <= rad; ++r) {
                int nv = v + r;
                if (nv < 0 || nv >= height) continue;
                for (int c = -rad; c <= rad; ++c) {
                    int nu = u + c;
                    if (nu < 0 || nu >= width) continue;
                    size_t nidx = nv * width + nu;
                    float val = d_in[nidx];
                    if (std::isfinite(val)) {
                        sum_d += val;
                        size_t nc = nidx * 3;
                        sum_r += c_in[nc]; sum_g += c_in[nc+1]; sum_b += c_in[nc+2];
                        count++;
                    }
                }
            }
            if (count > 0) {
                d_out[idx] = (float)(sum_d / count);
                size_t c = idx*3;
                c_out[c] = (uint8_t)(sum_r/count);
                c_out[c+1] = (uint8_t)(sum_g/count);
                c_out[c+2] = (uint8_t)(sum_b/count);
            } else {
                d_out[idx] = d_in[idx];
            }
        }
    }
    final_depth = d_out;
    final_rgb = c_out;
  }

  auto header = cloud_msg->header;
  
  ImageMsg rgb_msg;
  rgb_msg.header = header;
  rgb_msg.height = height; rgb_msg.width = width;
  rgb_msg.encoding = "rgb8"; rgb_msg.step = width * 3;
  rgb_msg.data = final_rgb;
  rgb_pub_->publish(rgb_msg);

  ImageMsg depth_msg;
  depth_msg.header = header;
  depth_msg.height = height; depth_msg.width = width;
  depth_msg.encoding = "32FC1"; depth_msg.step = width * 4;
  depth_msg.data.resize(n_pixels * 4);
  memcpy(depth_msg.data.data(), final_depth.data(), n_pixels * 4);
  depth_pub_->publish(depth_msg);

  ImageMsg idx_msg;
  idx_msg.header = header;
  idx_msg.height = height; idx_msg.width = width;
  idx_msg.encoding = "32SC1"; idx_msg.step = width * 4;
  idx_msg.data.resize(n_pixels * 4);
  memcpy(idx_msg.data.data(), final_index.data(), n_pixels * 4);
  index_pub_->publish(idx_msg);
}