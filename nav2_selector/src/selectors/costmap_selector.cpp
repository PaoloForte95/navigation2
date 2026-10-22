// Copyright (c) 2022, Paolo Forte
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
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <stdexcept>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "nav2_selector/selectors/costmap_selector.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace nav2_selector
{
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

CostmapSelector::CostmapSelector()
{
}

CostmapSelector::~CostmapSelector()
{
  RCLCPP_INFO(
    _logger, "Destroying plugin %s of type CostmapSelector",
    _name.c_str());
}

void CostmapSelector::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>/*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  _node = parent;
  auto node = parent.lock();
  _logger = node->get_logger();
  _clock = node->get_clock();
  _name = name;

  RCLCPP_INFO(_logger, "Configuring %s of type CostmapSelector", name.c_str());

  auto cm_node = std::dynamic_pointer_cast<nav2_util::LifecycleNode>(node);
  if (!cm_node) {
    throw std::runtime_error{"CostmapSelector needs a nav2_util::LifecycleNode parent"};
  }

  // General selector params
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter<bool>(name + ".allow_unknown", _allow_unknown);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".distance_threshold", rclcpp::ParameterValue(3.0));
  node->get_parameter<double>(name + ".distance_threshold", _distance_threshold);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".points", rclcpp::ParameterValue(std::string("[]")));
  node->get_parameter<std::string>(name + ".points", _points);
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".max_points", rclcpp::ParameterValue(3));
  _max_points = node->get_parameter(name + ".max_points").as_int();
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".polygon_pub_topic", rclcpp::ParameterValue("polygon_en"));
  polygon_pub_topic = node->get_parameter(name + ".polygon_pub_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".base_frame_id", rclcpp::ParameterValue("base_link"));
  _base_frame_id = node->get_parameter(name + ".base_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".odom_frame_id", rclcpp::ParameterValue("odom"));
  _odom_frame_id = node->get_parameter(name + ".odom_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".base_shift_correction", rclcpp::ParameterValue(true));
  _base_shift_correction = node->get_parameter(name + ".base_shift_correction").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".open_space_approach", rclcpp::ParameterValue("GridBased"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".corridor_approach", rclcpp::ParameterValue("GridBased"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".obstacle_right_approach", rclcpp::ParameterValue("GridBased"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".obstacle_left_approach", rclcpp::ParameterValue("GridBased"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".default_approach", rclcpp::ParameterValue("GridBased"));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".allowed_ids", rclcpp::ParameterValue(std::vector<std::string>{}));

  open_space_approach_ = node->get_parameter(name + ".open_space_approach").as_string();
  corridor_approach_ = node->get_parameter(name + ".corridor_approach").as_string();
  obstacle_right_approach_ = node->get_parameter(name + ".obstacle_right_approach").as_string();
  obstacle_left_approach_ = node->get_parameter(name + ".obstacle_left_approach").as_string();
  _default_approach = node->get_parameter(name + ".default_approach").as_string();
  _allowed_ids = node->get_parameter(name + ".allowed_ids").as_string_array();

  nav2_costmap_2d::makeFootprintFromString(_points, _poly_points);
  // Check for points format correctness
  if (_poly_points.size() <= 3 || _poly_points.size() % 2 != 0) {
    RCLCPP_ERROR(
      _logger,
      "Polygon has incorrect points description");
  }

  polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>(
    polygon_pub_topic, rclcpp::SystemDefaultsQoS());

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".observation_sources", rclcpp::PARAMETER_STRING_ARRAY);
  std::vector<std::string> source_names =
    node->get_parameter(name + ".observation_sources").as_string_array();
  tf2::Duration transform_tolerance = tf2::durationFromSec(0.5);
  rclcpp::Duration source_timeout = rclcpp::Duration::from_seconds(6.0);

  for (const std::string & source_name : source_names) {
    nav2_util::declare_parameter_if_not_declared(
      node, name + "." + source_name + ".type", rclcpp::ParameterValue(""));
    const std::string source_type =
      node->get_parameter(name + "." + source_name + ".type").as_string();
    RCLCPP_INFO(_logger, "Adding source of type: %s", source_type.c_str());

    if (source_type == "scan") {
      auto s = std::make_shared<nav2_collision_monitor::Scan>(
        cm_node, name + "." + source_name, costmap_ros->getTfBuffer(),
        _base_frame_id, _odom_frame_id, transform_tolerance, source_timeout,
        _base_shift_correction);
      s->configure();
      _sources.push_back(s);
    } else if (source_type == "pointcloud") {
      auto p = std::make_shared<nav2_collision_monitor::PointCloud>(
        cm_node, name + "." + source_name, costmap_ros->getTfBuffer(),
        _base_frame_id, _odom_frame_id, transform_tolerance, source_timeout,
        _base_shift_correction);
      p->configure();
      _sources.push_back(p);
    } else if (source_type == "image") {
      auto i = std::make_shared<nav2_collision_monitor::Image>(
        cm_node, name + "." + source_name, costmap_ros->getTfBuffer(),
        _base_frame_id, _odom_frame_id, transform_tolerance, source_timeout,
        _base_shift_correction);
      i->configure();
      _sources.push_back(i);
      _image_sources.push_back(i);
    } else {
      RCLCPP_ERROR(
        _logger, "[%s]: Unknown source type: %s",
        source_name.c_str(), source_type.c_str());
    }
  }

  RCLCPP_INFO(
    _logger, "Configured plugin %s of type CostmapSelector with "
    "distance_threshold %.2f, and %s.",
    _name.c_str(), _distance_threshold,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal");
}

void CostmapSelector::activate()
{
  RCLCPP_INFO(
    _logger, "Activating plugin %s of type CostmapSelector",
    _name.c_str());
  polygon_pub_->on_activate();
  auto node = _node.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(
    std::bind(&CostmapSelector::dynamicParametersCallback, this, _1));
}

void CostmapSelector::deactivate()
{
  RCLCPP_INFO(
    _logger, "Deactivating plugin %s of type CostmapSelector",
    _name.c_str());
  polygon_pub_->on_deactivate();
  _dyn_params_handler.reset();
}

void CostmapSelector::cleanup()
{
  RCLCPP_INFO(
    _logger, "Cleaning up plugin %s of type CostmapSelector",
    _name.c_str());
  polygon_pub_.reset();
  _image_sources.clear();
  _sources.clear();
}

std::string CostmapSelector::selectPlanner(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  std::string selected_planner = selectApproach(robot_pose);
  RCLCPP_DEBUG(_logger, "Best global planner is: %s", selected_planner.c_str());
  return selected_planner;
}

std::string CostmapSelector::selectController(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  std::string selected_controller = selectApproach(robot_pose);
  RCLCPP_DEBUG(_logger, "Best controller is: %s", selected_controller.c_str());
  return selected_controller;
}

std::string CostmapSelector::selectApproach(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  std::lock_guard<std::mutex> lock_reinit(_mutex);

  RCLCPP_DEBUG(
    _logger, "Robot current position: (%f, %f)",
    robot_pose.pose.position.x, robot_pose.pose.position.y);

  // Fill collision_points array from different data sources
  auto node = _node.lock();
  rclcpp::Time curr_time = node->get_clock()->now();

  // Scan, PointCloud and Image
  std::vector<nav2_collision_monitor::Point> collision_points;
  for (const auto & source : _sources) {
    if (source->getEnabled()) {
      source->getData(curr_time, collision_points);
    }
  }

  // Image values
  std::vector<double> colors;
  for (const auto & image : _image_sources) {
    if (image->getEnabled()) {
      image->getData(curr_time, colors);
    }
  }

  RCLCPP_DEBUG(_logger, "Collision Points %zu", collision_points.size());
  std::vector<int> inside_points = getPointsInside(collision_points);
  RCLCPP_DEBUG(
    _logger, "Point inside left polygon: %d and right polygon: %d",
    inside_points[0], inside_points[1]);

  std::string selected;
  if (inside_points[0] >= _max_points && inside_points[1] >= _max_points) {
    RCLCPP_DEBUG(_logger, "Obstacles on both sides! Entering a corridor!");
    selected = corridor_approach_;
  } else if (inside_points[0] > _max_points) {
    RCLCPP_DEBUG(_logger, "Obstacles on left side!");
    selected = obstacle_left_approach_;
  } else if (inside_points[1] > _max_points) {
    RCLCPP_DEBUG(_logger, "Obstacles on right side!");
    selected = obstacle_right_approach_;
  } else {
    RCLCPP_DEBUG(_logger, "No obstacles detected in the polygons. Open space");
    selected = open_space_approach_;
  }

  // An empty list means that every approach is allowed
  if (!_allowed_ids.empty() &&
    std::find(_allowed_ids.begin(), _allowed_ids.end(), selected) == _allowed_ids.end())
  {
    RCLCPP_DEBUG(
      _logger, "Selected approach %s is not allowed. Changing to %s",
      selected.c_str(), _default_approach.c_str());
    selected = _default_approach;
  }

  publishPolygon();
  return selected;
}

void CostmapSelector::publishPolygon() const
{
  geometry_msgs::msg::Polygon polygon;
  auto node = _node.lock();

  for (const geometry_msgs::msg::Point & p : _poly_points) {
    geometry_msgs::msg::Point32 p_s;
    p_s.x = p.x;
    p_s.y = p.y;
    polygon.points.push_back(p_s);
  }
  auto poly_s = std::make_unique<geometry_msgs::msg::PolygonStamped>();

  poly_s->header.stamp = node->get_clock()->now();
  poly_s->header.frame_id = _base_frame_id;
  poly_s->polygon = polygon;
  polygon_pub_->publish(std::move(poly_s));
}

std::vector<int> CostmapSelector::getPointsInside(
  const std::vector<nav2_collision_monitor::Point> & points) const
{
  if (_poly_points.size() < 4) {
    return {0, 0};
  }

  int num_l = 0, num_r = 0;
  std::vector<geometry_msgs::msg::Point> poly_left_points, poly_right_points;
  geometry_msgs::msg::Point middle1, middle2;
  middle1.y = (_poly_points[1].y + _poly_points[0].y) / 2;
  middle1.x = _poly_points[0].x;
  middle2.y = (_poly_points[2].y + _poly_points[3].y) / 2;
  middle2.x = _poly_points[3].x;

  // Add the points to left and right polygon
  poly_left_points.push_back(middle1);
  poly_left_points.push_back(_poly_points[1]);
  poly_left_points.push_back(_poly_points[2]);
  poly_left_points.push_back(middle2);

  poly_right_points.push_back(_poly_points[0]);
  poly_right_points.push_back(middle1);
  poly_right_points.push_back(middle2);
  poly_right_points.push_back(_poly_points[3]);

  for (const nav2_collision_monitor::Point & point : points) {
    if (isPointInside(point, poly_left_points)) {
      num_l++;
    }
    if (isPointInside(point, poly_right_points)) {
      num_r++;
    }
  }

  return {num_l, num_r};
}

bool CostmapSelector::isPointInside(
  const nav2_collision_monitor::Point & point,
  const std::vector<geometry_msgs::msg::Point> & poly_points) const
{
  // Adaptation of Shimrat, Moshe. "Algorithm 112: position of point relative to polygon."
  // Communications of the ACM 5.8 (1962): 434.
  // Implementation of ray crossings algorithm for point in polygon task solving.
  // Y coordinate is fixed. Moving the ray on X+ axis starting from given point.
  // Odd number of intersections with polygon boundaries means the point is inside polygon.
  const int poly_size = poly_points.size();
  int i, j;  // Polygon vertex iterators
  bool res = false;  // Final result, initialized with already inverted value

  // Starting from the edge where the last point of polygon is connected to the first
  i = poly_size - 1;
  for (j = 0; j < poly_size; j++) {
    // Checking the edge only if given point is between edge boundaries by Y coordinates.
    // One of the condition should contain equality in order to exclude the edges
    // parallel to X+ ray.
    if ((point.y <= poly_points[i].y) == (point.y > poly_points[j].y)) {
      // Calculating the intersection coordinate of X+ ray
      const double x_inter = poly_points[i].x +
        (point.y - poly_points[i].y) * (poly_points[j].x - poly_points[i].x) /
        (poly_points[j].y - poly_points[i].y);
      // If intersection with checked edge is greater than point.x coordinate, inverting the result
      if (x_inter > point.x) {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}

rcl_interfaces::msg::SetParametersResult
CostmapSelector::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(_mutex);

  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == _name + ".distance_threshold") {
        _distance_threshold = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == _name + ".allow_unknown") {
        _allow_unknown = parameter.as_bool();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == _name + ".max_points") {
        _max_points = parameter.as_int();
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == _name + ".points") {
        _points = parameter.as_string();
        _poly_points.clear();
        nav2_costmap_2d::makeFootprintFromString(_points, _poly_points);
      } else if (name == _name + ".polygon_pub_topic") {
        polygon_pub_topic = parameter.as_string();
      } else if (name == _name + ".base_frame_id") {
        _base_frame_id = parameter.as_string();
      } else if (name == _name + ".odom_frame_id") {
        _odom_frame_id = parameter.as_string();
      } else if (name == _name + ".open_space_approach") {
        open_space_approach_ = parameter.as_string();
      } else if (name == _name + ".corridor_approach") {
        corridor_approach_ = parameter.as_string();
      } else if (name == _name + ".obstacle_right_approach") {
        obstacle_right_approach_ = parameter.as_string();
      } else if (name == _name + ".obstacle_left_approach") {
        obstacle_left_approach_ = parameter.as_string();
      } else if (name == _name + ".default_approach") {
        _default_approach = parameter.as_string();
      }
    } else if (type == ParameterType::PARAMETER_STRING_ARRAY) {
      if (name == _name + ".allowed_ids") {
        _allowed_ids = parameter.as_string_array();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_selector

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_selector::CostmapSelector, nav2_core::Selector)