// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "control_filters/threshold_filter.hpp"

// #include "geometry_msgs/msg/vector3_stamped.hpp"
// #include "geometry_msgs/msg/wrench_stamped.hpp"
// #include "tf2/LinearMath/Vector3.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace control_filters
{
template <>
bool ThresholdFilter<geometry_msgs::msg::WrenchStamped>::update(
  const geometry_msgs::msg::WrenchStamped & data_in, geometry_msgs::msg::WrenchStamped & data_out)
{
  if (!this->configured_)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE((*logger_), *clock_, 2000, "Filter is not configured");
    return false;
  }

  // Update internal parameters if required
  if (parameter_handler_->is_old(parameters_))
  {
    parameters_ = parameter_handler_->get_params();
    if (!compute_internal_params())
    {
      RCLCPP_ERROR_THROTTLE((*logger_), *clock_, 2000, "Failed to compute internal parameters");
      return false;
    }
  }

  auto apply_threshold = [&](double value, double threshold) -> double
  {
    if (fabs(value) > threshold)
    {
      double sign = (value > 0) ? 1.0 : -1.0;
      return value - sign * threshold;
    }
    return 0.0;
  };

  // apply thresholds
  data_out = data_in;
  data_out.wrench.force.x =
    apply_threshold(data_in.wrench.force.x, wrench_threshold_.wrench.force.x);
  data_out.wrench.force.y =
    apply_threshold(data_in.wrench.force.y, wrench_threshold_.wrench.force.y);
  data_out.wrench.force.z =
    apply_threshold(data_in.wrench.force.z, wrench_threshold_.wrench.force.z);
  data_out.wrench.torque.x =
    apply_threshold(data_in.wrench.torque.x, wrench_threshold_.wrench.torque.x);
  data_out.wrench.torque.y =
    apply_threshold(data_in.wrench.torque.y, wrench_threshold_.wrench.torque.y);
  data_out.wrench.torque.z =
    apply_threshold(data_in.wrench.torque.z, wrench_threshold_.wrench.torque.z);

  return true;
}

}  // namespace control_filters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  control_filters::ThresholdFilter<geometry_msgs::msg::WrenchStamped>,
  filters::FilterBase<geometry_msgs::msg::WrenchStamped>)
