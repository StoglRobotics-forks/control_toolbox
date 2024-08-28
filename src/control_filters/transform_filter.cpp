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

#include "control_filters/transform_filter.hpp"

// #include "geometry_msgs/msg/vector3_stamped.hpp"
// #include "geometry_msgs/msg/wrench_stamped.hpp"
// #include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace control_filters
{
template <>
bool TransformFilter<geometry_msgs::msg::WrenchStamped>::update(
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

  try
  {
    transform_ = p_tf_Buffer_->lookupTransform(target_frame_id_, sensor_frame_id_, rclcpp::Time());
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(
      (*logger_), *clock_, 5000, "Could not transform data from '%s' to '%s': %s",
      data_in.header.frame_id.c_str(), target_frame_id_.c_str(), ex.what());
    return false;  // if cannot transform, result of subsequent computations is invalid
  }

  if (
    data_in.wrench.torque.x == 0.0 && data_in.wrench.torque.y == 0.0 &&
    data_in.wrench.torque.z == 0.0)
  {
    // HACK: only for case when all torques are zero, transform only force
    data_out.header.stamp = transform_.header.stamp;
    data_out.header.frame_id = transform_.header.frame_id;
    data_out.wrench.torque = data_in.wrench.torque;
    tf2::doTransform(data_in.wrench.force, data_out.wrench.force, transform_);
    return true;
  }
  else
  {
    tf2::doTransform(data_in, data_out, transform_);
  }

  return true;
}

}  // namespace control_filters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  control_filters::TransformFilter<geometry_msgs::msg::WrenchStamped>,
  filters::FilterBase<geometry_msgs::msg::WrenchStamped>)
