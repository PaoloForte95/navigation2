#include "nav2_collision_monitor/image.hpp"

#include <functional>
#include <stdexcept>

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

Image::Image(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout, base_shift_correction),
  data_(nullptr),
  max_distance_range_(5.0),
  min_distance_range_(0.1)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Image", source_name_.c_str());
}

Image::~Image()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Image", source_name_.c_str());
  data_sub_.reset();
}

void Image::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  Source::configure();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".topic",
    rclcpp::ParameterValue("/intel_realsense_r200_depth/image_raw"));

  std::string source_topic;
  getCommonParameters(source_topic);

  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".max_distance_range", rclcpp::ParameterValue(5.0));
  nav2_util::declare_parameter_if_not_declared(
    node, source_name_ + ".min_distance_range", rclcpp::ParameterValue(0.1));

  max_distance_range_ = node->get_parameter(source_name_ + ".max_distance_range").as_double();
  min_distance_range_ = node->get_parameter(source_name_ + ".min_distance_range").as_double();

  rclcpp::QoS image_qos = rclcpp::SensorDataQoS();

  data_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    source_topic, image_qos,
    std::bind(&Image::dataCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_, "[%s]: Camera subscribing to topic: %s",
    source_name_.c_str(), source_topic.c_str());
}

bool Image::getData(
  const rclcpp::Time & /*curr_time*/,
  std::vector<Point> & /*data*/)
{
  // TODO
  return true;
}

bool Image::getData(
  const rclcpp::Time & /*curr_time*/,
  std::vector<double> & /*data*/)
{
  // TODO
  return true;
}

void Image::dataCallback(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  data_ = msg;
}

}  // namespace nav2_collision_monitor