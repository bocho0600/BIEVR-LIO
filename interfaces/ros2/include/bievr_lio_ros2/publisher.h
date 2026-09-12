#ifndef BIEVR_LIO_ROS2_PUBLISHER_H_
#define BIEVR_LIO_ROS2_PUBLISHER_H_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <vector>

// publisher_base.h holds the shared publish logic and transitively includes
// bievr_ros_common/conversions.h (which pulls in the ROS message headers).
#include "bievr_ros_common/publisher_base.h"

namespace bievr {

// Type-erased wrapper around a typed rclcpp::Publisher. ROS2 publishers are
// strongly typed (rclcpp::Publisher<T>), so we keep the base handle and cast
// back to the concrete type on publish (the type is recorded on advertise()).
class TypedPublisher {
 public:
  TypedPublisher() : type_(std::type_index(typeid(void))) {}

  template <typename T>
  void advertise(rclcpp::Node::SharedPtr node, const std::string& topic) {
    // Publishers are created lazily on the first publish() to a topic, so the
    // first message would otherwise be sent before any subscriber has finished
    // the discovery handshake and would be dropped. transient_local makes the
    // publisher retain the last message and deliver it to subscribers as soon
    // as they connect, so the first message is never lost.
    pub_ = node->create_publisher<T>(topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
    type_ = std::type_index(typeid(T));
  }

  template <typename T>
  void publish(const T& msg) const {
    std::static_pointer_cast<rclcpp::Publisher<T>>(pub_)->publish(msg);
  }

  bool isSameType(const std::type_info& type) const { return type_ == std::type_index(type); }

 private:
  rclcpp::PublisherBase::SharedPtr pub_;
  std::type_index type_;
};

// Supplies the ROS2-specific pieces to PublisherBase.
struct Ros2Backend {
  using Handle = rclcpp::Node::SharedPtr;
  using TypedPublisher = bievr::TypedPublisher;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Odometry = nav_msgs::msg::Odometry;
  using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;
  using TransformStamped = geometry_msgs::msg::TransformStamped;

  explicit Ros2Backend(Handle node)
      : node_(node), tf_(std::make_shared<tf2_ros::TransformBroadcaster>(node)) {}

  template <typename M>
  void advertise(TypedPublisher& pub, const std::string& topic) {
    pub.template advertise<M>(node_, topic);
  }

  void sendTransform(const TransformStamped& transform) { tf_->sendTransform(transform); }

  Handle node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_;
};

using Publisher = PublisherBase<Ros2Backend>;

/*** Declares the odometry-covariance knobs as ordinary ROS2 parameters on `node` -- settable
     live with `ros2 param set`, not just Pipeline::Config's own custom YAML file -- and keeps
     `pub` in sync with them. Seeded from `config` (the values the custom YAML file already
     resolved), so the file remains the source of the startup default; declare_parameter's own
     override mechanism (a launch file's `parameters=[...]`, or `--ros-args -p name:=value`)
     still applies on top of that, exactly as for any other ROS parameter.

     Call once, after both `node` and `pub` exist, and keep the returned handle alive for as
     long as the node runs -- rclcpp holds it only by weak_ptr, specifically so the caller
     controls the callback's lifetime by holding the shared_ptr; let the return value go out
     of scope (or discard it) and the callback is silently removed; `ros2 param set` would
     then keep reporting success while doing nothing.

     Every later `ros2 param set` on any of these five re-reads all five and pushes them to
     `pub` in one call; that is more work than strictly necessary per change, but the five
     change together rarely enough (interactive tuning, not a hot path) that reading them all
     back is simpler than tracking which one fired. ***/
inline rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
declareOdomCovarianceParams(const rclcpp::Node::SharedPtr& node, const Pipeline::Config& config,
                           const std::shared_ptr<Publisher>& pub) {
  node->declare_parameter("enable_odom_covariance", config.enable_odom_covariance);
  node->declare_parameter("odom_position_variance", config.odom_position_variance);
  node->declare_parameter("odom_orientation_variance", config.odom_orientation_variance);
  node->declare_parameter("odom_linear_velocity_variance", config.odom_linear_velocity_variance);
  node->declare_parameter("odom_angular_velocity_variance",
                          config.odom_angular_velocity_variance);

  const auto apply = [node, pub]() {
    pub->setOdomCovarianceParams(node->get_parameter("enable_odom_covariance").as_bool(),
                                 node->get_parameter("odom_position_variance").as_double(),
                                 node->get_parameter("odom_orientation_variance").as_double(),
                                 node->get_parameter("odom_linear_velocity_variance").as_double(),
                                 node->get_parameter("odom_angular_velocity_variance").as_double());
  };
  apply();  // seed from whatever declare_parameter resolved (the config default, or an override)
  return node->add_on_set_parameters_callback([apply](const std::vector<rclcpp::Parameter>&) {
    apply();
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  });
}

}  // namespace bievr
#endif  // BIEVR_LIO_ROS2_PUBLISHER_H_
