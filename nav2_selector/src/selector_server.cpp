#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

#include "nav2_selector/selector_server.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_selector
{

SelectorServer::SelectorServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("selector_server", "", options),
  selector_loader_("nav2_core", "nav2_core::Selector"),
  default_planner_ids_{"CostmapPlanner"},
  default_planner_types_{"nav2_selector::CostmapSelector"},
  default_controller_ids_{"CostmapController"},
  default_controller_types_{"nav2_selector::CostmapSelector"},
  selector_frequency_(20.0)
{
  RCLCPP_INFO(get_logger(), "Creating selector server");

  // Declare this node's parameters
  declare_parameter("selector_frequency", 20.0);
  declare_parameter("planner_plugins", default_planner_ids_);
  declare_parameter("controller_plugins", default_controller_ids_);
  declare_parameter("planner_selector_id", std::string(""));
  declare_parameter("controller_selector_id", std::string(""));
  declare_parameter("planner_selector_topic", std::string("planner_selector"));
  declare_parameter("controller_selector_topic", std::string("controller_selector"));

  // Setup the local costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", std::string{get_namespace()}, "local_costmap",
    get_parameter("use_sim_time").as_bool());

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
}

SelectorServer::~SelectorServer()
{
  selector_timer_.reset();
  planner_selectors_.clear();
  controller_selectors_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
SelectorServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();
  RCLCPP_INFO(get_logger(), "Configuring selector interface");

  get_parameter("planner_plugins", planner_selector_ids_);
  if (planner_selector_ids_ == default_planner_ids_) {
    for (size_t i = 0; i < default_planner_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_planner_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_planner_types_[i]));
    }
  }

  get_parameter("controller_plugins", controller_selector_ids_);
  if (controller_selector_ids_ == default_controller_ids_) {
    for (size_t i = 0; i < default_controller_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        node, default_controller_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_controller_types_[i]));
    }
  }

  get_parameter("planner_selector_id", planner_selector_id_);
  get_parameter("controller_selector_id", controller_selector_id_);
  get_parameter("planner_selector_topic", planner_selector_topic_);
  get_parameter("controller_selector_topic", controller_selector_topic_);

  get_parameter("selector_frequency", selector_frequency_);
  if (selector_frequency_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "selector_frequency must be greater than zero");
    return nav2_util::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "Selector frequency set to %.4fHz", selector_frequency_);

  costmap_ros_->configure();

  if (!loadSelectors(
      planner_selector_ids_, planner_selector_types_,
      planner_selectors_, planner_selector_ids_concat_))
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (!loadSelectors(
      controller_selector_ids_, controller_selector_types_,
      controller_selectors_, controller_selector_ids_concat_))
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    get_logger(), "Selector Server has planner selectors: %s",
    planner_selector_ids_concat_.c_str());
  RCLCPP_INFO(
    get_logger(), "Selector Server has controller selectors: %s",
    controller_selector_ids_concat_.c_str());

  // Same QoS as the PlannerSelector and ControllerSelector BT nodes
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  planner_publisher_ = create_publisher<std_msgs::msg::String>(planner_selector_topic_, qos);
  controller_publisher_ = create_publisher<std_msgs::msg::String>(
    controller_selector_topic_, qos);

  RCLCPP_INFO(
    get_logger(), "Publishing planner on %s and controller on %s",
    planner_selector_topic_.c_str(), controller_selector_topic_.c_str());

  return nav2_util::CallbackReturn::SUCCESS;
}

bool
SelectorServer::loadSelectors(
  const std::vector<std::string> & ids,
  std::vector<std::string> & types,
  SelectorMap & selectors,
  std::string & ids_concat)
{
  auto node = shared_from_this();
  types.resize(ids.size());
  ids_concat.clear();

  for (size_t i = 0; i != ids.size(); i++) {
    try {
      types[i] = nav2_util::get_plugin_type_param(node, ids[i]);
      nav2_core::Selector::Ptr selector = selector_loader_.createUniqueInstance(types[i]);
      RCLCPP_INFO(
        get_logger(), "Created selector plugin %s of type %s",
        ids[i].c_str(), types[i].c_str());
      selector->configure(node, ids[i], costmap_ros_->getTfBuffer(), costmap_ros_);
      selectors.insert({ids[i], selector});
      ids_concat += ids[i] + std::string(" ");
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create selector %s. Exception: %s",
        ids[i].c_str(), ex.what());
      return false;
    }
  }

  return true;
}

nav2_util::CallbackReturn
SelectorServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  planner_publisher_->on_activate();
  controller_publisher_->on_activate();
  costmap_ros_->activate();

  for (auto & selector : planner_selectors_) {
    selector.second->activate();
  }
  for (auto & selector : controller_selectors_) {
    selector.second->activate();
  }

  last_planner_.clear();
  last_controller_.clear();

  auto node = shared_from_this();

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SelectorServer::dynamicParametersCallback, this, _1));

  selector_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / selector_frequency_)),
    std::bind(&SelectorServer::selectorCallback, this));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SelectorServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (selector_timer_) {
    selector_timer_->cancel();
    selector_timer_.reset();
  }

  planner_publisher_->on_deactivate();
  controller_publisher_->on_deactivate();
  costmap_ros_->deactivate();

  for (auto & selector : planner_selectors_) {
    selector.second->deactivate();
  }
  for (auto & selector : controller_selectors_) {
    selector.second->deactivate();
  }

  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SelectorServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  planner_publisher_.reset();
  controller_publisher_.reset();
  costmap_ros_->cleanup();

  for (auto & selector : planner_selectors_) {
    selector.second->cleanup();
  }
  for (auto & selector : controller_selectors_) {
    selector.second->cleanup();
  }
  planner_selectors_.clear();
  controller_selectors_.clear();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
SelectorServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
SelectorServer::selectorCallback()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  // Don't select until costmap is valid (after clear costmap)
  if (!costmap_ros_->isCurrent()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Costmap is not current. Skipping selection.");
    return;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!costmap_ros_->getRobotPose(robot_pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Could not get the robot pose. Skipping selection.");
    return;
  }

  try {
    const std::string planner = getPlanner(robot_pose);
    if (!planner.empty() && planner != last_planner_) {
      RCLCPP_INFO(get_logger(), "Selected planner: %s", planner.c_str());
      last_planner_ = planner;
      publishPlanner(planner);
    }

    const std::string controller = getController(robot_pose);
    if (!controller.empty() && controller != last_controller_) {
      RCLCPP_INFO(get_logger(), "Selected controller: %s", controller.c_str());
      last_controller_ = controller;
      publishController(controller);
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Selection failed: \"%s\"", ex.what());
  }
}

std::string
SelectorServer::getPlanner(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (planner_selectors_.empty()) {
    return std::string();
  }

  auto it = planner_selectors_.find(planner_selector_id_);
  if (it != planner_selectors_.end()) {
    return it->second->selectPlanner(robot_pose);
  }

  if (planner_selectors_.size() == 1 && planner_selector_id_.empty()) {
    return planner_selectors_.begin()->second->selectPlanner(robot_pose);
  }

  RCLCPP_ERROR_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "planner_selector_id %s is not valid. Planner selector names are: %s",
    planner_selector_id_.c_str(), planner_selector_ids_concat_.c_str());
  return std::string();
}

std::string
SelectorServer::getController(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  if (controller_selectors_.empty()) {
    return std::string();
  }

  auto it = controller_selectors_.find(controller_selector_id_);
  if (it != controller_selectors_.end()) {
    return it->second->selectController(robot_pose);
  }

  if (controller_selectors_.size() == 1 && controller_selector_id_.empty()) {
    return controller_selectors_.begin()->second->selectController(robot_pose);
  }

  RCLCPP_ERROR_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "controller_selector_id %s is not valid. Controller selector names are: %s",
    controller_selector_id_.c_str(), controller_selector_ids_concat_.c_str());
  return std::string();
}

void
SelectorServer::publishPlanner(const std::string & planner)
{
  if (!planner_publisher_ || !planner_publisher_->is_activated()) {
    return;
  }
  auto message = std::make_unique<std_msgs::msg::String>();
  message->data = planner;
  planner_publisher_->publish(std::move(message));
}

void
SelectorServer::publishController(const std::string & controller)
{
  if (!controller_publisher_ || !controller_publisher_->is_activated()) {
    return;
  }
  auto message = std::make_unique<std_msgs::msg::String>();
  message->data = controller;
  controller_publisher_->publish(std::move(message));
}

rcl_interfaces::msg::SetParametersResult
SelectorServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;

  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_STRING) {
      if (name == "planner_selector_id") {
        planner_selector_id_ = parameter.as_string();
        last_planner_.clear();
      } else if (name == "controller_selector_id") {
        controller_selector_id_ = parameter.as_string();
        last_controller_.clear();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace nav2_selector

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_selector::SelectorServer)