// Copyright (c) 2022, Paolo Forte
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
// limitations under the License. Reserved.

#ifndef NAV2_SELECTOR__SELECTORS__COSTMAP_SELECTOR_HPP_
#define NAV2_SELECTOR__SELECTORS__COSTMAP_SELECTOR_HPP_

#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include "nav2_core/selector.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav2_collision_monitor/types.hpp"
#include "nav2_collision_monitor/source.hpp"
#include "nav2_collision_monitor/scan.hpp"
#include "nav2_collision_monitor/pointcloud.hpp"
#include "nav2_collision_monitor/image.hpp"

namespace nav2_selector
{

class CostmapSelector : public nav2_core::Selector
{
public:
  /**
   * @brief constructor
   */
  CostmapSelector();

  /**
   * @brief destructor
   */
  ~CostmapSelector();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Select the best global planner for the area around the robot
   * @param robot_pose Current robot pose
   * @return std::string The best global planner
   */
  std::string selectPlanner(const geometry_msgs::msg::PoseStamped & robot_pose) override;

  /**
   * @brief Select the best controller for the area around the robot
   * @param robot_pose Current robot pose
   * @return std::string The best controller
   */
  std::string selectController(const geometry_msgs::msg::PoseStamped & robot_pose) override;

  /**
   * @brief Publish the polygon used to detect obstacles
   */
  void publishPolygon() const;

  /**
   * @brief Gets number of points inside the left and right halves of the polygon
   * @param points Input array of points to be checked
   * @return Number of points inside the left and the right polygon. If there are no points,
   * returns zero values.
   */
  virtual std::vector<int> getPointsInside(
    const std::vector<nav2_collision_monitor::Point> & points) const;

  /**
   * @brief Checks if point is inside polygon
   * @param point Given point to check
   * @param poly_points The points of the polygon
   * @return True if given point is inside polygon, otherwise false
   */
  bool isPointInside(
    const nav2_collision_monitor::Point & point,
    const std::vector<geometry_msgs::msg::Point> & poly_points) const;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters List of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

protected:
  /**
   * @brief Classify the area around the robot and pick the matching approach
   * @param robot_pose Current robot pose
   * @return std::string The selected approach
   */
  std::string selectApproach(const geometry_msgs::msg::PoseStamped & robot_pose);

  bool _allow_unknown;
  bool _base_shift_correction;
  double _distance_threshold;
  int _max_points;
  std::string polygon_pub_topic;
  std::string _points;
  std::vector<geometry_msgs::msg::Point> _poly_points;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Logger _logger{rclcpp::get_logger("CostmapSelector")};
  std::string _name, _odom_frame_id, _base_frame_id;
  std::string open_space_approach_, corridor_approach_, obstacle_right_approach_,
    obstacle_left_approach_;
  std::string _default_approach;
  std::vector<std::string> _allowed_ids;
  std::mutex _mutex;
  rclcpp_lifecycle::LifecycleNode::WeakPtr _node;
  std::vector<std::shared_ptr<nav2_collision_monitor::Source>> _sources;
  std::vector<std::shared_ptr<nav2_collision_monitor::Image>> _image_sources;
  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
};

}  // namespace nav2_selector

#endif  // NAV2_SELECTOR__SELECTORS__COSTMAP_SELECTOR_HPP_