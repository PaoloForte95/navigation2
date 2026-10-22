#ifndef NAV2_SELECTOR__SELECTOR_SERVER_HPP_
#define NAV2_SELECTOR__SELECTOR_SERVER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_loader.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/selector.hpp"

namespace nav2_selector
{
/**
 * @class nav2_selector::SelectorServer
 * @brief A lifecycle node that hosts selector plugins and periodically publishes the best
 * planner and controller on the topics read by the PlannerSelector and ControllerSelector
 * behavior tree nodes.
 */
class SelectorServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_selector::SelectorServer
   * @param options Additional options to control creation of the node.
   */
  explicit SelectorServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief A destructor for nav2_selector::SelectorServer
   */
  ~SelectorServer();

  using SelectorMap = std::unordered_map<std::string, nav2_core::Selector::Ptr>;

  /**
   * @brief Select the best planner with the active planner selector plugin
   * @param robot_pose Current robot pose
   * @return std::string The best planner, or an empty string if none
   */
  std::string getPlanner(const geometry_msgs::msg::PoseStamped & robot_pose);

  /**
   * @brief Select the best controller with the active controller selector plugin
   * @param robot_pose Current robot pose
   * @return std::string The best controller, or an empty string if none
   */
  std::string getController(const geometry_msgs::msg::PoseStamped & robot_pose);

protected:
  /**
   * @brief Configure member variables and load the selector plugins
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activate member variables and start the selection timer
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivate member variables and stop the selection timer
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Load and configure a list of selector plugins
   * @param ids Plugin names to load
   * @param types Filled with the plugin type of each name
   * @param selectors Map filled with the loaded plugins
   * @param ids_concat Filled with all plugin names, for logging
   * @return true if all plugins were loaded
   */
  bool loadSelectors(
    const std::vector<std::string> & ids,
    std::vector<std::string> & types,
    SelectorMap & selectors,
    std::string & ids_concat);

  /**
   * @brief Timer callback that runs the selectors and publishes any new selection
   */
  void selectorCallback();

  /**
   * @brief Publish the selected planner
   * @param planner Name of the planner
   */
  void publishPlanner(const std::string & planner);

  /**
   * @brief Publish the selected controller
   * @param controller Name of the controller
   */
  void publishController(const std::string & controller);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters List of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // Selectors
  SelectorMap planner_selectors_;
  SelectorMap controller_selectors_;
  pluginlib::ClassLoader<nav2_core::Selector> selector_loader_;
  std::vector<std::string> default_planner_ids_;
  std::vector<std::string> default_planner_types_;
  std::vector<std::string> default_controller_ids_;
  std::vector<std::string> default_controller_types_;
  std::vector<std::string> planner_selector_ids_;
  std::vector<std::string> planner_selector_types_;
  std::vector<std::string> controller_selector_ids_;
  std::vector<std::string> controller_selector_types_;
  std::string planner_selector_ids_concat_;
  std::string controller_selector_ids_concat_;
  std::string planner_selector_id_;
  std::string controller_selector_id_;

  // Last published selection
  std::string last_planner_;
  std::string last_controller_;

  // Local costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  // Publishers read by the PlannerSelector and ControllerSelector BT nodes
  std::string planner_selector_topic_;
  std::string controller_selector_topic_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr planner_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr controller_publisher_;

  // Selection timer
  rclcpp::TimerBase::SharedPtr selector_timer_;
  double selector_frequency_;
};

}  // namespace nav2_selector

#endif  // NAV2_SELECTOR__SELECTOR_SERVER_HPP_