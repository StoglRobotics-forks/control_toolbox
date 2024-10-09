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

#include "control_filters/virtual_forces_filter.hpp"

// #include "geometry_msgs/msg/vector3_stamped.hpp"
// #include "geometry_msgs/msg/wrench_stamped.hpp"
// #include "tf2/LinearMath/Vector3.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace control_filters
{
template <>
bool VirtualForcesFilter<geometry_msgs::msg::WrenchStamped>::update(
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

  // apply virtual forces
  data_out = data_in;
  data_out.wrench.force.x += virtual_forces_.wrench.force.x;
  data_out.wrench.force.y += virtual_forces_.wrench.force.y;
  data_out.wrench.force.z += virtual_forces_.wrench.force.z;
  data_out.wrench.torque.x += virtual_forces_.wrench.torque.x;
  data_out.wrench.torque.y += virtual_forces_.wrench.torque.y;
  data_out.wrench.torque.z += virtual_forces_.wrench.torque.z;

  return true;
}

}  // namespace control_filters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  control_filters::VirtualForcesFilter<geometry_msgs::msg::WrenchStamped>,
  filters::FilterBase<geometry_msgs::msg::WrenchStamped>)
