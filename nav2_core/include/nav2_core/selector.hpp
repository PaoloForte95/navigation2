#ifndef NAV2_CORE__SELECTOR_HPP_
#define NAV2_CORE__SELECTOR_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_core
{

/**
 * @class Selector
 * @brief Abstract interface for planner and controller selectors to adhere to with pluginlib
 */
class Selector
{
public:
  using Ptr = std::shared_ptr<Selector>;

  /**
   * @brief Virtual destructor
   */
  virtual ~Selector() {}

  /**
   * @brief Configure the plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  /**
   * @brief Cleanup the plugin
   */
  virtual void cleanup() = 0;

  /**
   * @brief Activate the plugin
   */
  virtual void activate() = 0;

  /**
   * @brief Deactivate the plugin
   */
  virtual void deactivate() = 0;

  /**
   * @brief Select the best planner for the current situation
   * @param robot_pose Current robot pose
   * @return std::string The selected planner, or an empty string if none
   */
  virtual std::string selectPlanner(const geometry_msgs::msg::PoseStamped & robot_pose) = 0;

  /**
   * @brief Select the best controller for the current situation
   * @param robot_pose Current robot pose
   * @return std::string The selected controller, or an empty string if none
   */
  virtual std::string selectController(const geometry_msgs::msg::PoseStamped & robot_pose) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__SELECTOR_HPP_