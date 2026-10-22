#ifndef NAV2_COLLISION_MONITOR__IMAGE_HPP_
#define NAV2_COLLISION_MONITOR__IMAGE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs/msg/image.hpp"

#include "nav2_collision_monitor/source.hpp"

namespace nav2_collision_monitor
{

/**
 * @brief Implementation for image sensor source
 */
class Image : public Source
{
public:
  /**
   * @brief Image constructor
   * @param node Node pointer
   * @param source_name Name of data source
   * @param tf_buffer Shared pointer to a TF buffer
   * @param base_frame_id Robot base frame ID. The output data will be transformed into this frame.
   * @param global_frame_id Global frame ID for correct transform calculation
   * @param transform_tolerance Transform tolerance
   * @param source_timeout Maximum time interval in which data is considered valid
   * @param base_shift_correction Whether to correct source data towards to base frame movement
   */
  Image(
    const nav2_util::LifecycleNode::WeakPtr & node,
    const std::string & source_name,
    const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string & base_frame_id,
    const std::string & global_frame_id,
    const tf2::Duration & transform_tolerance,
    const rclcpp::Duration & source_timeout,
    const bool base_shift_correction);

  /**
   * @brief Image destructor
   */
  ~Image();

  /**
   * @brief Data source configuration routine. Obtains ROS-parameters
   * and creates image subscriber.
   */
  void configure();

  /**
   * @brief Adds latest obstacle points from the image to the data array.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * Added data is transformed to base_frame_id_ coordinate system at curr_time.
   * @return false if an invalid source should block the robot
   */
  bool getData(
    const rclcpp::Time & curr_time,
    std::vector<Point> & data) override;

  /**
   * @brief Adds latest values from the image to the data array.
   * @param curr_time Current node time for data interpolation
   * @param data Array where the data from source to be added.
   * @return false if an invalid source should block the robot
   */
  bool getData(
    const rclcpp::Time & curr_time,
    std::vector<double> & data);

protected:
  /**
   * @brief Image data callback
   * @param msg Shared pointer to Image message
   */
  void dataCallback(sensor_msgs::msg::Image::ConstSharedPtr msg);

  // ----- Variables -----

  /// @brief Image data subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr data_sub_;

  /// @brief Latest data obtained from the image
  sensor_msgs::msg::Image::ConstSharedPtr data_;

  /// @brief Distance range for obstacles
  double max_distance_range_;
  double min_distance_range_;
};  // class Image

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__IMAGE_HPP_