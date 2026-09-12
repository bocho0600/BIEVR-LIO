#ifndef BIEVR_ROS_COMMON_PUBLISHER_BASE_H_
#define BIEVR_ROS_COMMON_PUBLISHER_BASE_H_

// ROS-version-agnostic publisher logic shared by the ROS1 and ROS2 wrappers.
// Everything that genuinely differs between the two (the node handle, the
// type-erased publisher mechanism, the concrete message types and the TF
// broadcaster) is supplied by a `Backend` policy; the dispatch, lazy
// advertising, namespace handling and pipeline registration live here once.
//
// The message conversions (headerToMsg / pointCloudToMsg / transformToMsg /
// vecToMsg) are the version-agnostic templates from conversions.h, included
// below so they resolve by ordinary lookup at this template's definition. They
// build PointCloud2 messages via sensor_msgs::PointCloud2Modifier, so the TU
// that instantiates a Publisher must have included the matching
// sensor_msgs/point_cloud2_iterator header (the wrappers' conversions.h, pulled
// in by publisher.h, does this).

#include <functional>
#include <memory>
#include <string>
#include <typeinfo>
#include <unordered_map>

#include "bievr_lio/common.h"
#include "bievr_lio/log++.h"
#include "bievr_lio/pipeline.h"
#include "bievr_ros_common/conversions.h"

namespace bievr {

template <typename Backend>
class PublisherBase {
 public:
  using Handle = typename Backend::Handle;

  // If ns is non-empty, every topic published through this object is advertised
  // under that namespace (e.g. ns="bievr_lio" turns "odometry" into
  // "/bievr_lio/odometry"). Absolute topics (leading '/') are left untouched.
  PublisherBase(Handle handle, std::shared_ptr<Pipeline> pipeline, const std::string& ns = "")
      : backend_(std::move(handle)),
        ns_(ns),
        publish_tf_(pipeline->config().publish_tf),
        odom_covariance_enabled_(pipeline->config().enable_odom_covariance),
        odom_position_variance_(pipeline->config().odom_position_variance),
        odom_orientation_variance_(pipeline->config().odom_orientation_variance),
        odom_linear_velocity_variance_(pipeline->config().odom_linear_velocity_variance),
        odom_angular_velocity_variance_(pipeline->config().odom_angular_velocity_variance) {
    registerTypes<Pointcloud, IntensityPointcloud, Odometry, V3>(pipeline);
  }
  virtual ~PublisherBase() = default;

  template <typename T>
  bool publish(const T& data, const Header& header, const std::string& topic,
               const std::string& child_frame = "") {
    return publishImpl(data, header, topic, child_frame);
  }

  // Live update of the covariance knobs, so a ROS2 wrapper can expose them as ordinary ROS
  // parameters (settable with `ros2 param set`, not just the Pipeline's own config file) and
  // push changes here whenever they change, rather than only reading them once at
  // construction. See bievr_lio_ros2/publisher.h's declareOdomCovarianceParams.
  void setOdomCovarianceParams(bool enabled, double position_variance, double orientation_variance,
                               double linear_velocity_variance, double angular_velocity_variance) {
    odom_covariance_enabled_ = enabled;
    odom_position_variance_ = position_variance;
    odom_orientation_variance_ = orientation_variance;
    odom_linear_velocity_variance_ = linear_velocity_variance;
    odom_angular_velocity_variance_ = angular_velocity_variance;
  }

 private:
  // Pointcloud and IntensityPointcloud both go out as PointCloud2.
  bool publishImpl(const Pointcloud& cloud, const Header& header, const std::string& topic,
                   const std::string& /*child_frame*/) {
    typename Backend::PointCloud2 msg;
    if (!getOrAdvertise<typename Backend::PointCloud2>(topic)) return false;
    headerToMsg(header, msg.header);
    pointCloudToMsg(cloud, msg);
    publishers_[topic].publish(msg);
    return true;
  }

  bool publishImpl(const IntensityPointcloud& cloud, const Header& header, const std::string& topic,
                   const std::string& /*child_frame*/) {
    typename Backend::PointCloud2 msg;
    if (!getOrAdvertise<typename Backend::PointCloud2>(topic)) return false;
    headerToMsg(header, msg.header);
    pointCloudToMsg(cloud, msg);
    publishers_[topic].publish(msg);
    return true;
  }

  bool publishImpl(const Odometry& odometry, const Header& header, const std::string& topic,
                   const std::string& child_frame) {
    typename Backend::Odometry odom_msg;
    if (!getOrAdvertise<typename Backend::Odometry>(topic)) return false;
    headerToMsg(header, odom_msg.header);
    odom_msg.child_frame_id = child_frame;
    transformToMsg(odometry.pose, odom_msg.pose.pose);
    // Twist is expressed in the child (body) frame.
    vecToMsg(odometry.linear_velocity, odom_msg.twist.twist.linear);
    vecToMsg(odometry.angular_velocity, odom_msg.twist.twist.angular);
    // Diagonal only: see Pipeline::Config::enable_odom_covariance for why this is here at
    // all, and off entirely leaves the message at its default zero -- e.g. for A/B testing
    // against that original (broken) behaviour without a rebuild. Row-major 6x6, so the
    // diagonal is index i*6+i for i in [0,6) -- x, y, z, roll, pitch, yaw for pose; vx, vy,
    // vz, vroll, vpitch, vyaw for twist. Pose and twist get separate variances: they are
    // different physical quantities (position vs. velocity), so the same number would be
    // dimensionally wrong for both.
    //
    // Pose position/orientation come from odometry.pose_*_variance when non-zero -- a real
    // per-scan estimate from the registration itself, see
    // LsqRegistration::poseCovarianceDiagonal -- and fall back to the configured constant
    // otherwise (zero is that struct's sentinel for "no estimate this scan", never a value
    // to publish literally). Twist has no such per-scan estimate, so it is always the
    // configured constant.
    if (odom_covariance_enabled_) {
      for (int i = 0; i < 3; ++i) {
        const double position_variance = odometry.pose_position_variance(i);
        odom_msg.pose.covariance[i * 6 + i] =
            position_variance > 0.0 ? position_variance : odom_position_variance_;
        odom_msg.twist.covariance[i * 6 + i] = odom_linear_velocity_variance_;
      }
      for (int i = 3; i < 6; ++i) {
        const double orientation_variance = odometry.pose_orientation_variance(i - 3);
        odom_msg.pose.covariance[i * 6 + i] =
            orientation_variance > 0.0 ? orientation_variance : odom_orientation_variance_;
        odom_msg.twist.covariance[i * 6 + i] = odom_angular_velocity_variance_;
      }
    }
    publishers_[topic].publish(odom_msg);

    /*** Mirror the pose as a TF transform, unless another node owns that edge. Two
         publishers on one TF edge make lookups depend on message arrival order, which is
         the usual arrangement when a filter downstream (e.g. robot_localization) fuses this
         odometry and broadcasts the result itself. The child frame follows the odometry
         message, so with Pipeline::Config::odom_in_base set this is map_frame ->
         base_frame rather than map_frame -> body_frame. ***/
    if (publish_tf_) {
      typename Backend::TransformStamped transform_msg;
      transform_msg.header = odom_msg.header;
      transform_msg.child_frame_id = child_frame;
      transformToMsg(odometry.pose, transform_msg.transform);
      backend_.sendTransform(transform_msg);
    }
    return true;
  }

  bool publishImpl(const V3& vec, const Header& header, const std::string& topic,
                   const std::string& /*child_frame*/) {
    typename Backend::Vector3Stamped msg;
    if (!getOrAdvertise<typename Backend::Vector3Stamped>(topic)) return false;
    headerToMsg(header, msg.header);
    vecToMsg(vec, msg.vector);
    publishers_[topic].publish(msg);
    return true;
  }

  // Look up (or lazily advertise) the publisher for `topic`, verifying that its
  // message type matches any previously advertised one. Returns false on a
  // type mismatch (the message is then dropped by the caller).
  template <typename MsgT>
  bool getOrAdvertise(const std::string& topic) {
    auto it = publishers_.find(topic);
    if (it == publishers_.end()) {
      backend_.template advertise<MsgT>(publishers_[topic], resolveTopic(topic));
    } else if (!it->second.isSameType(typeid(MsgT))) {
      LOG(W, "Publisher type mismatch for topic " << topic);
      return false;
    }
    return true;
  }

  template <typename T>
  void registerWithPipeline(std::shared_ptr<Pipeline> pipeline) {
    pipeline->registerPublisher<T>(std::bind_front(&PublisherBase::publish<T>, this));
  }

  template <typename... Ts>
  void registerTypes(std::shared_ptr<Pipeline> pipeline) {
    (registerWithPipeline<Ts>(pipeline), ...);  // fold expression
  }

  // Prepends the namespace to a relative topic. Absolute topics (leading '/')
  // and the empty-namespace case are returned unchanged.
  std::string resolveTopic(const std::string& topic) const {
    if (ns_.empty() || topic.empty() || topic.front() == '/') {
      return topic;
    }
    return "/" + ns_ + "/" + topic;
  }

  Backend backend_;
  std::string ns_;
  bool publish_tf_ = true;
  bool odom_covariance_enabled_ = true;
  double odom_position_variance_ = 4e-4;
  double odom_orientation_variance_ = 3e-4;
  double odom_linear_velocity_variance_ = 1e-2;
  double odom_angular_velocity_variance_ = 1e-4;
  std::unordered_map<std::string, typename Backend::TypedPublisher> publishers_;
};

}  // namespace bievr

#endif  // BIEVR_ROS_COMMON_PUBLISHER_BASE_H_
