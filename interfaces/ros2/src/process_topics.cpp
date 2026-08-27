#include <bievr_lio/common.h>
#include <bievr_lio/log++.h>
#include <bievr_lio/synchronizer.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "bievr_lio/config_loader.h"
#include "bievr_lio_ros2/publisher.h"
#include "bievr_ros_common/conversions.h"
#ifdef BIEVR_WITH_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  srand(1);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bievr_lio_topic_node");

  bievr::Config config;
  // rclcpp::init does not strip ROS arguments from argv; remove_ros_arguments
  // returns just the application arguments (our config-file flags).
  if (!bievr::loadConfigFromArgs(rclcpp::remove_ros_arguments(argc, argv), config)) {
    LOG(E, "Failed to load config.");
    return -1;
  }

  // Cap TBB parallelism for the whole process (0 = TBB default, i.e. all cores).
  const int n_threads =
      config.max_num_threads > 0 ? config.max_num_threads : tbb::this_task_arena::max_concurrency();
  tbb::global_control tbb_control(tbb::global_control::max_allowed_parallelism, n_threads);
  LOG(I, config.max_num_threads > 0, "TBB parallelism limited to " << n_threads << " threads.");

  auto pipeline = std::make_shared<bievr::Pipeline>(config.pipeline_config);
  auto synchronizer = std::make_shared<bievr::Synchronizer>(pipeline);
  auto lio_pub = std::make_shared<bievr::Publisher>(node, pipeline, "bievr_lio");

  // ROS2 has no ShapeShifter: discover the pointcloud topic's type from the
  // graph, then use a generic (serialized) subscription to handle whichever of
  // the supported message types is being published on that single topic.
  const std::string& pc_topic = config.topic_config.pointcloud_topic;
  std::string pc_type;
  LOG(I, "Waiting to discover type of pointcloud topic '" << pc_topic << "'...");
  while (rclcpp::ok() && pc_type.empty()) {
    const auto names_types = node->get_topic_names_and_types();
    const auto it = names_types.find(pc_topic);
    if (it != names_types.end() && !it->second.empty()) {
      pc_type = it->second.front();
    } else {
      rclcpp::spin_some(node);
      rclcpp::sleep_for(100ms);
    }
  }
  if (!rclcpp::ok()) {
    rclcpp::shutdown();
    return 0;
  }
  LOG(I, "Pointcloud topic type: " << pc_type);

  // QoS must be compatible with the publisher. SensorDataQoS (best-effort)
  // matches most LiDAR drivers (e.g. Ouster); some drivers (e.g. Livox) publish
  // reliable — adjust here if you see no messages arriving.
  const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();

  auto pc_sub = node->create_generic_subscription(
      pc_topic, pc_type, sensor_qos,
      [&, pc_type](std::shared_ptr<rclcpp::SerializedMessage> serialized) {
        if (pc_type == "sensor_msgs/msg/PointCloud2") {
          sensor_msgs::msg::PointCloud2 msg;
          rclcpp::Serialization<sensor_msgs::msg::PointCloud2>().deserialize_message(
              serialized.get(), &msg);
          bievr::StampedIntensityPointcloud pointcloud;
          if (bievr::msgToPointcloud(msg, pointcloud)) {
            synchronizer->addPointcloud(pointcloud);
          }
        }
#ifdef BIEVR_WITH_LIVOX
        else if (pc_type == "livox_ros_driver2/msg/CustomMsg") {
          livox_ros_driver2::msg::CustomMsg msg;
          rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg>().deserialize_message(
              serialized.get(), &msg);
          bievr::StampedIntensityPointcloud pointcloud;
          if (bievr::msgToPointcloud(msg, pointcloud)) {
            synchronizer->addPointcloud(pointcloud);
          }
        }
#endif
        else {
          LOG_FIRST(W, 1, "Received unsupported pointcloud message type: " << pc_type);
        }
      });

  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
      config.topic_config.imu_topic, sensor_qos, [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
        bievr::ImuMeasurement imu;
        if (!bievr::msgToImuMeasurement(*msg, imu)) {
          return;
        }
        synchronizer->addImu(imu);
      });

  /*** Static body -> lidar transform, straight from the LiDAR-IMU calibration. Off by
       default because a robot description normally already publishes that edge. ***/
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;
  if (config.pipeline_config.publish_tf_lidar) {
    static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    geometry_msgs::msg::TransformStamped static_tf;
    static_tf.header.stamp = node->now();
    static_tf.header.frame_id = config.pipeline_config.body_frame;
    static_tf.child_frame_id = config.pipeline_config.lidar_frame;
    bievr::transformToMsg(config.pipeline_config.T_I_L, static_tf.transform);
    static_tf_broadcaster->sendTransform(static_tf);
  }

  /*** The base_frame switches need to know where the robot base sits relative to the
       sensor, and only the robot description knows that. Read lidar_frame -> base_frame
       from TF once and hand the pipeline T_I_B = T_I_L * T_L_B.
       Polled on a timer rather than waited for inline, because the description may come up
       after this node does and blocking here would also block the subscriptions. The
       pipeline decides what to do in the meantime: origin_at_base and heading_at_base hold
       initialisation back (the world frame they define cannot be applied retroactively),
       odom_in_base only withholds the odometry message. ***/
  rclcpp::TimerBase::SharedPtr base_extrinsic_timer;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  if (pipeline->needsBaseExtrinsic()) {
    tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);
    base_extrinsic_timer = node->create_wall_timer(200ms, [&]() {
      if (pipeline->baseExtrinsicValid()) {
        base_extrinsic_timer->cancel();
        return;
      }
      geometry_msgs::msg::TransformStamped tf_lidar_base;
      try {
        tf_lidar_base = tf_buffer->lookupTransform(config.pipeline_config.lidar_frame,
                                                   config.pipeline_config.base_frame,
                                                   tf2::TimePointZero);
      } catch (const tf2::TransformException& e) {
        LOG_TIMED(W, 5.0,
                  "Waiting for " << config.pipeline_config.lidar_frame << " -> "
                                 << config.pipeline_config.base_frame << ": " << e.what());
        return;
      }
      bievr::Transform T_L_B;
      bievr::msgToTransformStamped(tf_lidar_base, T_L_B);
      const bievr::Transform T_I_B(
          Eigen::Isometry3d(config.pipeline_config.T_I_L * T_L_B));
      pipeline->setBaseExtrinsic(T_I_B);
      LOG(I, "Resolved " << config.pipeline_config.base_frame << " in IMU coordinates: t = ["
                         << T_I_B.translation().transpose() << "].");
      base_extrinsic_timer->cancel();
    });
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
