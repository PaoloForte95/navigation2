// Copyright (c) 2023 Paolo Forte
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

#include "nav2_weather_detection/weather_detector_node.hpp"

#include <functional>
#include <utility>

#include "nav2_util/node_utils.hpp"

namespace nav2_weather_detection
{

WeatherDetector::WeatherDetector(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("weather_detector", "", options),
  process_active_(false),
  limit_active_(false),
  slowdown_ratio_(1.0),
  percentage_(true)
{
}

WeatherDetector::~WeatherDetector()
{
}

nav2_util::CallbackReturn
WeatherDetector::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();
  logger_ = node->get_logger();
  RCLCPP_INFO(logger_, "Configuring weather detector");

  nav2_util::declare_parameter_if_not_declared(
    node, "topic", rclcpp::ParameterValue("weather_condition"));
  nav2_util::declare_parameter_if_not_declared(
    node, "speed_limit_topic", rclcpp::ParameterValue("/speed_limit"));
  nav2_util::declare_parameter_if_not_declared(
    node, "slowdown_ratio", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "percentage", rclcpp::ParameterValue(true));

  std::string topic = node->get_parameter("topic").as_string();
  std::string speed_limit_topic = node->get_parameter("speed_limit_topic").as_string();
  slowdown_ratio_ = node->get_parameter("slowdown_ratio").as_double();
  percentage_ = node->get_parameter("percentage").as_bool();

  data_sub_ = node->create_subscription<std_msgs::msg::UInt8>(
    topic, rclcpp::QoS(10),
    std::bind(&WeatherDetector::dataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(logger_, "Subscribing to topic: %s", topic.c_str());

  speed_limit_pub_ = node->create_publisher<nav2_msgs::msg::SpeedLimit>(
    speed_limit_topic, rclcpp::QoS(10));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WeatherDetector::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating weather detector");

  speed_limit_pub_->on_activate();
  limit_active_ = false;
  process_active_ = true;

  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WeatherDetector::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating weather detector");

  process_active_ = false;

  if (limit_active_) {
    publishSpeedLimit(false);
  }

  speed_limit_pub_->on_deactivate();

  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WeatherDetector::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up weather detector");

  speed_limit_pub_.reset();
  data_sub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WeatherDetector::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}

void WeatherDetector::dataCallback(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
  if (!process_active_) {
    return;
  }

  bool limit = msg->data != 0;

  if (limit == limit_active_) {
    return;
  }

  publishSpeedLimit(limit);
}

void WeatherDetector::publishSpeedLimit(bool limit)
{
  auto speed_limit_msg = std::make_unique<nav2_msgs::msg::SpeedLimit>();
  speed_limit_msg->header.stamp = now();
  speed_limit_msg->percentage = percentage_;
  speed_limit_msg->speed_limit = limit ? slowdown_ratio_ : 0.0;
  speed_limit_pub_->publish(std::move(speed_limit_msg));

  limit_active_ = limit;

  if (limit) {
    if (percentage_) {
      RCLCPP_INFO(logger_, "Bad weather detected, limiting speed to %.2f%% of maximum", slowdown_ratio_);
    } else {
      RCLCPP_INFO(logger_, "Bad weather detected, limiting speed to %.2f m/s", slowdown_ratio_);
    }
  } else {
    RCLCPP_INFO(logger_, "Clear weather, speed limit removed");
  }
}

}  // namespace nav2_weather_detection
