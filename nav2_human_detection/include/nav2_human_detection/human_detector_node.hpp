#ifndef NAV2_HUMAN_DETECTION__HUMAN_DETECTOR_NODE_HPP_
#define NAV2_HUMAN_DETECTION__HUMAN_DETECTOR_NODE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "nav2_util/lifecycle_node.hpp"

#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

namespace nav2_human_detection
{

class HumanDetector : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor for the nav2_human_detection::HumanDetector
   * @param options Additional options to control creation of the node.
   */
  explicit HumanDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief Destructor for the nav2_human_detection::HumanDetector
   */
  ~HumanDetector() override;

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
   * @brief Human detection callback
   * @param msg True when at least one human is detected
   */
  void dataCallback(const std_msgs::msg::Bool::ConstSharedPtr msg);

  /**
   * @brief Publishes the speed limit, or removes it
   * @param limit True to apply the limit, false to remove it
   */
  void publishSpeedLimit(bool limit);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr data_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;

  bool process_active_;
  bool limit_active_;
  double slowdown_ratio_;
  bool percentage_;

  rclcpp::Logger logger_{rclcpp::get_logger("HumanDetector")};
};

}  // namespace nav2_human_detection

#endif  // NAV2_HUMAN_DETECTION__HUMAN_DETECTOR_NODE_HPP_