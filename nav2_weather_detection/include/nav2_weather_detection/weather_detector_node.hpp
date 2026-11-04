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

#ifndef NAV2_WEATHER_DETECTION__WEATHER_DETECTOR_NODE_HPP_
#define NAV2_WEATHER_DETECTION__WEATHER_DETECTOR_NODE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "nav2_util/lifecycle_node.hpp"

#include "std_msgs/msg/u_int8.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

namespace nav2_weather_detection
{

class WeatherDetector : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for the nav2_weather_detection::WeatherDetector
   * @param options Additional options to control creation of the node.
   */
  explicit WeatherDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for the nav2_weather_detection::WeatherDetector
   */
  ~WeatherDetector() override;

protected:
  /**
   * @brief Reads parameters, creates subscriber and publisher
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates publisher and main processor, creates bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Removes any active speed limit, deactivates publisher, destroys bond connection
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets subscriber and publisher
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called in shutdown state
   * @param state Lifecycle Node's state
   * @return Success or Failure
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Weather condition callback
   * @param msg Weather condition: 0 CLEAR, 1 FOG, 2 RAIN, 3 SNOW
   */
  void dataCallback(const std_msgs::msg::UInt8::ConstSharedPtr msg);

  /**
   * @brief Publishes the speed limit, or removes it
   * @param limit True to apply the limit, false to remove it
   */
  void publishSpeedLimit(bool limit);

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr data_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;

  bool process_active_;
  bool limit_active_;
  double slowdown_ratio_;
  bool percentage_;

  rclcpp::Logger logger_{rclcpp::get_logger("WeatherDetector")};
};

}  // namespace nav2_weather_detection

#endif  // NAV2_WEATHER_DETECTION__WEATHER_DETECTOR_NODE_HPP_
