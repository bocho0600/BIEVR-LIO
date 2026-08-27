#ifndef BIEVR_LIO_PIPELINE_H_
#define BIEVR_LIO_PIPELINE_H_

#include <typeindex>

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/imu_integrator.h"
#include "bievr_lio/log++.h"
#include "bievr_lio/ls_optimizer.h"
#include "bievr_lio/preprocess.h"
#include "bievr_lio/utils.h"

namespace bievr {

class Pipeline {
 public:
  struct Config {
    Config() = default;
    PreprocessConfig preprocess;
    ImuConfig imu;
    RegistrationConfig registration;
    BIEVRMap::Config map;
    bool print_timing = false;
    bool publish_all_clouds = false;
    bool print_debug = false;      // when true, lower the log level to show DEBUG messages
    bool print_dashboard = false;  // when true, print the fancy live status dashboard
    // Path to the ASCII art shown at the top of the dashboard (e.g. bievr_ascii.txt).
    std::string dashboard_ascii_path = "";
    Transform T_I_L = Transform::Identity();  // Transform from LiDAR to IMU frame
    /*** Read T_I_L from TF (imu_frame -> lidar_frame) at startup instead of taking it from
         the config. The offset between the two is a property of the sensor, so a robot
         description that already models both links is a better home for it than a second
         copy here that nothing checks against the first. Off by default: a bag replay has
         no live TF tree, and the explicit vectors stay mandatory when it is off. ***/
    bool calibration_from_tf = false;
    /*** TF frame names. map_frame is the world-fixed parent of the published odometry and
         clouds; body_frame is the IMU, which is what the filter actually estimates.
         lidar_frame and base_frame name links this node never estimates but has to be able
         to talk about: lidar_frame is the source of the lidar_frame -> base_frame lookup the
         wrapper reads from TF, and base_frame is the robot base the three switches below
         express poses relative to. ***/
    std::string map_frame = "map";
    std::string body_frame = "body";
    std::string lidar_frame = "lidar";
    std::string base_frame = "base_link";
    /*** Broadcast the odometry pose on TF as well as publishing it. Turn this off when
         another node (e.g. a robot_localization EKF) already owns the map_frame -> body
         edge: two publishers on one TF edge make lookups depend on message arrival order. ***/
    bool publish_tf = true;
    /*** Also broadcast a static body_frame -> lidar_frame transform, taken from the
         LiDAR-IMU calibration. Off by default: a robot description usually already
         publishes that edge, and two publishers on it would fight. ***/
    bool publish_tf_lidar = false;
    /*** Report base_frame rather than body_frame on the odometry topic, with the pose
         composed as T_W_I * T_I_B. Use it when a downstream filter wants the base pose with
         no transform of its own between the message's child frame and its base frame. ***/
    bool odom_in_base = false;
    /*** Put the world origin on base_frame's initial pose instead of the IMU's (position
         only), and put the world heading on base_frame too. See initializeBias for why the
         heading is a separate switch from the origin. ***/
    bool origin_at_base = false;
    bool heading_at_base = false;
    std::string log_path = "";

    size_t min_points_for_map_init = 100;
    size_t map_size_running_threshold = 5;
    size_t informed_sample_count = 300;
    size_t min_map_size_for_imu_opt = 100;
    size_t min_imu_integrators_for_opt = 5;
    double gravity_prior_weight = 5.0;
  };

  explicit Pipeline(const Config& config);
  virtual ~Pipeline() = default;

  void processFrame(const std::vector<ImuMeasurement>& imu_data,
                    const StampedIntensityPointcloud& pointcloud);

  // Read-only view of the resolved configuration, so the ROS wrappers can honour the
  // frame names and publishing switches without a second copy of them.
  const Config& config() const { return config_; }

  // Where base_frame sits in IMU coordinates: T_I_B = T_I_L * T_L_B, with T_L_B read from
  // TF by the wrapper. Only the wrapper can know it, so it is pushed in rather than
  // configured. Needed by odom_in_base, origin_at_base and heading_at_base; until it
  // arrives the first two hold the pipeline back entirely (see processFrame) and the third
  // withholds the odometry message.
  void setBaseExtrinsic(const Transform& T_I_B);
  bool baseExtrinsicValid() const { return base_extrinsic_valid_; }

  // The LiDAR-IMU extrinsic, when calibration_from_tf asked for it to come from TF rather
  // than from the config. Overwrites Config::T_I_L, which is the identity until it lands --
  // hence the gate in processFrame: every scan is transformed by it, so no frame may be
  // processed before it arrives.
  void setLidarExtrinsic(const Transform& T_I_L);
  bool lidarExtrinsicValid() const { return !config_.calibration_from_tf || lidar_extrinsic_valid_; }
  // Whether anything that is enabled actually needs the extrinsic above.
  bool needsBaseExtrinsic() const {
    return config_.odom_in_base || config_.origin_at_base || config_.heading_at_base;
  }

  template <typename T>
  void registerPublisher(std::function<void(const T&, const Header&, const std::string& topic,
                                            const std::string& child_frame)>
                             func) {
    auto wrapper = [func](const void* val, const Header& header, const std::string& topic,
                          const std::string& child_frame) {
      func(*static_cast<const T*>(val), header, topic, child_frame);
    };
    publishers_[typeid(T)] = wrapper;
  }

 private:
  enum class Phase { NeedBias, NeedMap, Running };

  // Pipeline helpers
  bool initializeBias(const std::vector<ImuMeasurement>& imu_data, const Pointcloud& pointcloud);
  void tryInitMap(uint64_t stamp, const State& x_j_pred, const Transform& T_W_I_init,
                  const Pointcloud& undistorted, const IntensityView& intensities,
                  std::vector<double>& ranges, const Header& header);
  void sampleSource(const Pointcloud& undistorted, const Transform& T_W_I_init,
                    Pointcloud& filtered, Pointcloud& coarse, Pointcloud& fine) const;

  // State and optimization management
  bool addState(const uint64_t time, const Quaternion& quat, const V3& p, const V3& v);
  bool addImuIntegrator(ImuIntegratorPtr imu_integrator);
  bool optimizeInertialWindow();

  // Publishing
  template <typename T>
  void publish(const T& value, const Header& header, const std::string& topic,
               const std::string& child_frame = "") const {
    auto it = publishers_.find(typeid(T));
    if (it != publishers_.end()) {
      it->second(static_cast<const void*>(&value), header, topic, child_frame);
    } else {
      LOG(E, "No publisher registered in pipeline for type.");
    }
  }

  void publishFrame(const Header& header, const Transform& T_W_I, const Pointcloud& full_registered,
                    const Pointcloud& source_filtered, const Pointcloud& source_coarse,
                    const Pointcloud& source_fine, const Pointcloud& undistorted,
                    const IntensityView& intensities);
  void publishLatestState(const Header& header);
  void publishDebugClouds(const Pointcloud& source_filtered, const Pointcloud& source_coarse,
                          const Pointcloud& source_fine, const Pointcloud& undistorted_cloud,
                          const IntensityView& intensities, const Transform& T_W_I,
                          const Header& header);

  // Logging
  void logTUM(double timestamp, const Transform& pose);

  Config config_;
  std::map<uint64_t, State> states_;
  std::map<uint64_t, ImuIntegratorPtr> imu_integrators_;

  size_t seq_counter_ = 0;
  Phase phase_ = Phase::NeedBias;
  std::unique_ptr<BiasInitializer> bias_initializer_;
  std::shared_ptr<BIEVRMap> map_;
  V3 acc_bias_ = V3::Zero();
  V3 gyro_bias_ = V3::Zero();
  V3 gravity_dir_ = V3(0, 0, 1);
  // base_frame expressed in IMU coordinates, see setBaseExtrinsic.
  Transform T_I_B_ = Transform::Identity();
  bool base_extrinsic_valid_ = false;
  bool lidar_extrinsic_valid_ = false;
  // Latest gyro reading, used to report the angular velocity in the odometry twist.
  V3 latest_gyro_ = V3::Zero();
  // Accelerometer scale resolved during bias estimation (1 if raw, g if the IMU
  // reports gravity-normalized accelerations). Applied to all incoming IMU data.
  double imu_acc_scale_ = 1.0;

  using PublishFunction =
      std::function<void(const void*, const Header&, const std::string&, const std::string&)>;
  std::unordered_map<std::type_index, PublishFunction> publishers_;
  std::shared_ptr<std::ofstream> tum_log_;

  // Accumulated state for the live status dashboard (printDashboard in utils).
  DashboardState dashboard_;
};
}  // namespace bievr

#endif  // BIEVR_LIO_PIPELINE_H_
