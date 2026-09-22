#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace hospital_dynamic_layer
{
class PredictionLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    auto node = node_.lock();
    if (!node) {throw std::runtime_error("Costmap node expired");}
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("timeout", rclcpp::ParameterValue(0.6));
    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".timeout", timeout_);
    current_ = false;
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud>(
      "/hospital/predicted_obstacles", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud::SharedPtr msg) {
        if (msg->header.frame_id != layered_costmap_->getGlobalFrameID() ||
          msg->channels.size() != 1 || msg->channels[0].name != "radius" ||
          msg->channels[0].values.size() != msg->points.size()) {return;}
        std::lock_guard<std::mutex> guard(mutex_);
        cloud_ = msg;
      }, options);
  }

  void reset() override
  {
    std::lock_guard<std::mutex> guard(mutex_);
    cloud_.reset();
    current_ = false;
  }
  bool isClearable() override {return false;}

  void updateBounds(double, double, double, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    // Invalidate the whole small rolling window, including the previous envelope.
    // LayeredCostmap rebuilds it from the static/scan layers before this layer runs.
    auto * grid = layered_costmap_->getCostmap();
    *min_x = std::min(*min_x, grid->getOriginX());
    *min_y = std::min(*min_y, grid->getOriginY());
    *max_x = std::max(*max_x, grid->getOriginX() + grid->getSizeInMetersX());
    *max_y = std::max(*max_y, grid->getOriginY() + grid->getSizeInMetersY());
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & grid, int min_i, int min_j,
    int max_i, int max_j) override
  {
    if (!enabled_) {current_ = true; return;}
    sensor_msgs::msg::PointCloud::SharedPtr msg;
    {std::lock_guard<std::mutex> guard(mutex_); msg = cloud_;}
    current_ = false;
    if (!msg) {return;}
    const double age = (clock_->now() - rclcpp::Time(msg->header.stamp)).seconds();
    if (age < -0.05 || age > timeout_) {return;}
    current_ = true;
    const double resolution = grid.getResolution();
    for (size_t i = 0; i < msg->points.size(); ++i) {
      const auto & p = msg->points[i];
      const double radius = msg->channels[0].values[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        !std::isfinite(radius) || radius <= 0 || radius > 1.0) {continue;}
      int x0, y0, x1, y1;
      // Future occupancy is a soft preference, never a present-time collision.
      // A hard future envelope can surround the robot at t=0 and make every
      // MPPI sample invalid, stopping it directly in front of an approaching person.
      // Real, current scan obstacles remain lethal in the preceding scan layer.
      const double support = radius + 1.2;
      grid.worldToMapEnforceBounds(p.x - support, p.y - support, x0, y0);
      grid.worldToMapEnforceBounds(p.x + support, p.y + support, x1, y1);
      for (int y = std::max(y0, min_j); y <= std::min(y1, max_j - 1); ++y) {
        for (int x = std::max(x0, min_i); x <= std::min(x1, max_i - 1); ++x) {
          double wx, wy;
          grid.mapToWorld(x, y, wx, wy);
          const double distance = std::hypot(wx - p.x, wy - p.y);
          if (distance <= support + resolution * 0.71) {
            const auto cost = static_cast<unsigned char>(
              240.0 * std::exp(-2.0 * std::max(0.0, distance - radius)));
            const auto existing = grid.getCost(x, y);
            if (existing != nav2_costmap_2d::NO_INFORMATION) {
              grid.setCost(x, y, std::max(existing, cost));
            }
          }
        }
      }
    }
  }

private:
  double timeout_{0.6};
  std::mutex mutex_;
  sensor_msgs::msg::PointCloud::SharedPtr cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subscription_;
};
}  // namespace hospital_dynamic_layer
PLUGINLIB_EXPORT_CLASS(hospital_dynamic_layer::PredictionLayer, nav2_costmap_2d::Layer)
