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

#ifndef CONTROL_FILTERS__THRESHOLD_FILTER_HPP_
#define CONTROL_FILTERS__THRESHOLD_FILTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "filters/filter_base.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
// #include "tf2_ros/buffer.h"
// #include "tf2_ros/transform_listener.h"
#include "threshold_filter_parameters.hpp"

namespace control_filters
{

template <typename T>
class ThresholdFilter : public filters::FilterBase<T>
{
public:
  /** \brief Constructor */
  ThresholdFilter();

  /** \brief Destructor */
  ~ThresholdFilter();

  /** @brief Configure filter parameters  */
  bool configure() override;

  /** \brief Update the filter and return the data separately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  bool update(const T & data_in, T & data_out) override;

protected:
  bool compute_internal_params()
  {
    wrench_threshold_.wrench.force.x = parameters_.wrench.force[0];
    wrench_threshold_.wrench.force.y = parameters_.wrench.force[1];
    wrench_threshold_.wrench.force.z = parameters_.wrench.force[2];
    wrench_threshold_.wrench.torque.x = parameters_.wrench.torque[0];
    wrench_threshold_.wrench.torque.y = parameters_.wrench.torque[1];
    wrench_threshold_.wrench.torque.z = parameters_.wrench.torque[2];

    auto check_threshold = [&](double threshold) -> bool
    {
      if (threshold < 0.0)
      {
        RCLCPP_ERROR((*logger_), "Threshold cannot be negative");
        return false;
      }
      return true;
    };

    if (
      !check_threshold(wrench_threshold_.wrench.force.x) ||
      !check_threshold(wrench_threshold_.wrench.force.y) ||
      !check_threshold(wrench_threshold_.wrench.force.z) ||
      !check_threshold(wrench_threshold_.wrench.torque.x) ||
      !check_threshold(wrench_threshold_.wrench.torque.y) ||
      !check_threshold(wrench_threshold_.wrench.torque.z))
    {
      RCLCPP_ERROR((*logger_), "Threshold filter cannot be configured");
      return false;
    }

    return true;
  };

private:
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<rclcpp::Logger> logger_;
  std::shared_ptr<threshold_filter::ParamListener> parameter_handler_;
  threshold_filter::Params parameters_;

  // Frames for Transformation of Gravity / CoG Vector
  // std::string world_frame_;   // frame in which gravity is given
  // std::string sensor_frame_;  // frame in which Cog is given and compution occur
  // Storage for Calibration Values
  // geometry_msgs::msg::Vector3Stamped cog_;            // Center of Gravity Vector (wrt sensor
  // frame) geometry_msgs::msg::Vector3Stamped cst_ext_force_;  // Gravity Force Vector (wrt world
  // frame)
  geometry_msgs::msg::WrenchStamped wrench_threshold_;

  // Filter objects
  // std::unique_ptr<tf2_ros::Buffer> p_tf_Buffer_;
  // std::unique_ptr<tf2_ros::TransformListener> p_tf_Listener_;
  // geometry_msgs::msg::TransformStamped transform_sensor_datain_, transform_world_dataout_,
  //   transform_data_out_sensor_, transform_sensor_world_;
};

template <typename T>
ThresholdFilter<T>::ThresholdFilter()
{
}

template <typename T>
ThresholdFilter<T>::~ThresholdFilter()
{
}

template <typename T>
bool ThresholdFilter<T>::configure()
{
  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  // p_tf_Buffer_.reset(new tf2_ros::Buffer(clock_));
  // p_tf_Listener_.reset(new tf2_ros::TransformListener(*p_tf_Buffer_.get(), true));

  logger_.reset(
    new rclcpp::Logger(this->logging_interface_->get_logger().get_child(this->filter_name_)));

  // Initialize the parameter_listener once
  if (!parameter_handler_)
  {
    try
    {
      parameter_handler_ = std::make_shared<threshold_filter::ParamListener>(
        this->params_interface_, this->param_prefix_);
    }
    catch (rclcpp::exceptions::ParameterUninitializedException & ex)
    {
      RCLCPP_ERROR((*logger_), "Threshold filter cannot be configured: %s", ex.what());
      parameter_handler_.reset();
      return false;
    }
    catch (rclcpp::exceptions::InvalidParameterValueException & ex)
    {
      RCLCPP_ERROR((*logger_), "Threshold filter cannot be configured: %s", ex.what());
      parameter_handler_.reset();
      return false;
    }
  }

  parameters_ = parameter_handler_->get_params();
  if (!compute_internal_params())
  {
    return false;
  }

  return true;
}

}  // namespace control_filters

#endif  // CONTROL_FILTERS__THRESHOLD_FILTER_HPP_
