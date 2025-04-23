/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/TrajectoryReference.h>

#include <std_srvs/Trigger.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <mrs_msgs/DynamicsConstraints.h>
#include <mrs_msgs/Path.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/GetPathSrv.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/MpcPredictionFullState.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/UavState.h>

#include <eth_trajectory_generation/impl/polynomial_optimization_nonlinear_impl.h>
#include <eth_trajectory_generation/trajectory.h>
#include <eth_trajectory_generation/trajectory_sampling.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_uav_trajectory_generation/drsConfig.h>

#include <future>

//}

/* using //{ */

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;
using mat3_t = Eigen::Matrix3Xd;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

/* defines //{ */

#define FUTURIZATION_EXEC_TIME_FACTOR 0.5        // [0, 1]
#define FUTURIZATION_FIRST_WAYPOINT_FACTOR 0.50  // [0, 1]
#define OVERTIME_SAFETY_FACTOR 0.95              // [0, 1]
#define OVERTIME_SAFETY_OFFSET 0.01              // [s]
#define NLOPT_EXEC_TIME_FACTOR 0.95              // [0, 1]

typedef struct
{
  Eigen::Vector4d coords;
  bool            stop_at;
} Waypoint_t;

//}

namespace mrs_uav_trajectory_generation
{

/* class MrsTrajectoryGeneration //{ */
class MrsTrajectoryGeneration : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;

  // | ----------------------- parameters ----------------------- |

  double _sampling_dt_;

  double _max_trajectory_len_factor_;
  double _min_trajectory_len_factor_;

  int _n_attempts_;

  double _min_waypoint_distance_;

  bool   _fallback_sampling_enabled_;
  double _fallback_sampling_speed_factor_;
  double _fallback_sampling_accel_factor_;
  double _fallback_sampling_stopping_time_;
  bool   _fallback_sampling_first_waypoint_additional_stop_;

  double _takeoff_height_;

  std::string _uav_name_;

  bool   _trajectory_max_segment_deviation_enabled_;
  double trajectory_max_segment_deviation_;
  int    _trajectory_max_segment_deviation_max_iterations_;

  bool   _path_straightener_enabled_;
  double _path_straightener_max_deviation_;
  double _path_straightener_max_hdg_deviation_;

  bool _override_heading_atan2_;

  // | -------- variable parameters (come with the path) -------- |

  std::string frame_id_;
  bool        fly_now_                              = false;
  bool        use_heading_                          = false;
  bool        stop_at_waypoints_                    = false;
  bool        override_constraints_                 = false;
  bool        loop_                                 = false;
  double      override_max_velocity_horizontal_     = 0.0;
  double      override_max_velocity_vertical_       = 0.0;
  double      override_max_acceleration_horizontal_ = 0.0;
  double      override_max_acceleration_vertical_   = 0.0;
  double      override_max_jerk_horizontal_         = 0.0;
  double      override_max_jerk_vertical_           = 0.0;

  // | -------------- variable parameters (deduced) ------------- |

  double     max_execution_time_ = 0;
  std::mutex mutex_max_execution_time_;

  bool max_deviation_first_segment_;

  std::atomic<bool> dont_prepend_initial_condition_ = false;

  // | -------------------- the transformer  -------------------- |

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  // | ------------------- scope timer logger ------------------- |

  bool                                       scope_timer_enabled_ = false;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;

  // service client for input
  bool               callbackPathSrv(mrs_msgs::PathSrv::Request& req, mrs_msgs::PathSrv::Response& res);
  ros::ServiceServer service_server_path_;

  // service client for returning result to the user
  bool               callbackGetPathSrv(mrs_msgs::GetPathSrv::Request& req, mrs_msgs::GetPathSrv::Response& res);
  ros::ServiceServer service_server_get_path_;

  void                                      callbackPath(const mrs_msgs::Path::ConstPtr msg);
  mrs_lib::SubscribeHandler<mrs_msgs::Path> sh_path_;

  mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>      sh_tracker_cmd_;
  mrs_lib::SubscribeHandler<mrs_msgs::DynamicsConstraints> sh_constraints_;

  void                                          callbackUavState(const mrs_msgs::UavState::ConstPtr msg);
  mrs_lib::SubscribeHandler<mrs_msgs::UavState> sh_uav_state_;

  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

  // service client for publishing trajectory out
  ros::ServiceClient service_client_trajectory_reference_;

  // solve the whole problem
  std::tuple<bool, std::string, mrs_msgs::TrajectoryReference, bool> optimize(const std::vector<Waypoint_t>& waypoints_in,
                                                                              const std_msgs::Header& waypoints_stamp, bool fallback_sampling,
                                                                              const bool relax_heading);

  std::tuple<std::optional<mrs_msgs::TrackerCommand>, bool, int> prepareInitialCondition(const ros::Time path_time);

  // batch vizualizer
  mrs_lib::BatchVisualizer bw_original_;
  mrs_lib::BatchVisualizer bw_final_;

  // transforming TrackerCommand
  std::optional<mrs_msgs::Path> transformPath(const mrs_msgs::Path& path, const std::string& target_frame);

  // | ------------------ trajectory validation ----------------- |

  /**
   * @brief validates samples of a trajectory agains a path of waypoints
   *
   * @param trajectory
   * @param segments
   *
   * @return <success, traj_fail_idx, path_fail_segment>
   */
  std::tuple<bool, int, std::vector<bool>, double> validateTrajectorySpatial(const eth_mav_msgs::EigenTrajectoryPoint::Vector& trajectory,
                                                                             const std::vector<Waypoint_t>&                    waypoints);

  std::vector<int> getWaypointInTrajectoryIdxs(const mrs_msgs::TrajectoryReference& trajectory, const std::vector<Waypoint_t>& waypoints);

  std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector> findTrajectory(const std::vector<Waypoint_t>&                 waypoints,
                                                                           const std::optional<mrs_msgs::TrackerCommand>& initial_state,
                                                                           const double& sampling_dt, const bool& relax_heading);

  std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector> findTrajectoryFallback(const std::vector<Waypoint_t>& waypoints, const double& sampling_dt,
                                                                                   const bool& relax_heading);

  std::future<std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector>> future_trajectory_result_;
  std::atomic<bool>                                                      running_async_planning_ = false;

  std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector> findTrajectoryAsync(const std::vector<Waypoint_t>&                 waypoints,
                                                                                const std::optional<mrs_msgs::TrackerCommand>& initial_state,
                                                                                const double& sampling_dt, const bool& relax_heading);

  std::vector<Waypoint_t> preprocessPath(const std::vector<Waypoint_t>& waypoints_in);

  mrs_msgs::TrajectoryReference getTrajectoryReference(const eth_mav_msgs::EigenTrajectoryPoint::Vector& trajectory,
                                                       const std::optional<mrs_msgs::TrackerCommand>& initial_condition, const double& sampling_dt);

  Waypoint_t interpolatePoint(const Waypoint_t& a, const Waypoint_t& b, const double& coeff);

  bool checkNaN(const Waypoint_t& a);

  double distFromSegment(const vec3_t& point, const vec3_t& seg1, const vec3_t& seg2);

  bool trajectorySrv(const mrs_msgs::TrajectoryReference& msg);

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                           mutex_drs_;
  typedef mrs_uav_trajectory_generation::drsConfig DrsParams_t;
  typedef dynamic_reconfigure::Server<DrsParams_t> Drs_t;
  boost::shared_ptr<Drs_t>                         drs_;
  void                                             callbackDrs(mrs_uav_trajectory_generation::drsConfig& params, uint32_t level);
  DrsParams_t                                      params_;
  std::mutex                                       mutex_params_;

  // | ------------ Republisher for the desired path ------------ |

  mrs_lib::PublisherHandler<mrs_msgs::Path> ph_original_path_;

  // | ------------- measuring the time of execution ------------ |

  ros::Time  start_time_total_;
  std::mutex mutex_start_time_total_;
  bool       overtime(void);
  double     timeLeft(void);
};

//}

/* onInit() //{ */

void MrsTrajectoryGeneration::onInit() {

  /* obtain node handle */
  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ----------------------- publishers ----------------------- |

  ph_original_path_ = mrs_lib::PublisherHandler<mrs_msgs::Path>(nh_, "original_path_out", 1);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "TrajectoryGeneration";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_constraints_          = mrs_lib::SubscribeHandler<mrs_msgs::DynamicsConstraints>(shopts, "constraints_in");
  sh_tracker_cmd_          = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in");
  sh_uav_state_            = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in", &MrsTrajectoryGeneration::callbackUavState, this);
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in");

  sh_path_ = mrs_lib::SubscribeHandler<mrs_msgs::Path>(shopts, "path_in", &MrsTrajectoryGeneration::callbackPath, this);

  // | --------------------- service servers -------------------- |

  service_server_path_ = nh_.advertiseService("path_in", &MrsTrajectoryGeneration::callbackPathSrv, this);

  service_server_get_path_ = nh_.advertiseService("get_path_in", &MrsTrajectoryGeneration::callbackGetPathSrv, this);

  service_client_trajectory_reference_ = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("trajectory_reference_out");

  // | ----------------------- parameters ----------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "MrsTrajectoryGeneration");

  std::string custom_config_path;
  std::string platform_config_path;
  std::string uav_manager_config_path;

  param_loader.loadParam("custom_config", custom_config_path);
  param_loader.loadParam("platform_config", platform_config_path);
  param_loader.loadParam("uav_manager_config", uav_manager_config_path);

  if (uav_manager_config_path == "") {
    ROS_ERROR("[MrsTrajectoryGeneration]: uav_manager_config param is empty");
    ros::shutdown();
  }

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  if (platform_config_path != "") {
    param_loader.addYamlFile(platform_config_path);
  }

  param_loader.addYamlFile(uav_manager_config_path);

  param_loader.addYamlFileFromParam("private_config");
  param_loader.addYamlFileFromParam("public_config");

  const std::string yaml_prefix = "mrs_uav_trajectory_generation/";

  param_loader.loadParam("uav_name", _uav_name_);

  param_loader.loadParam(yaml_prefix + "sampling_dt", _sampling_dt_);

  param_loader.loadParam(yaml_prefix + "enforce_fallback_solver", params_.enforce_fallback_solver);

  param_loader.loadParam(yaml_prefix + "max_trajectory_len_factor", _max_trajectory_len_factor_);
  param_loader.loadParam(yaml_prefix + "min_trajectory_len_factor", _min_trajectory_len_factor_);

  param_loader.loadParam(yaml_prefix + "n_attempts", _n_attempts_);
  param_loader.loadParam(yaml_prefix + "fallback_sampling/enabled", _fallback_sampling_enabled_);
  param_loader.loadParam(yaml_prefix + "fallback_sampling/speed_factor", _fallback_sampling_speed_factor_);
  param_loader.loadParam(yaml_prefix + "fallback_sampling/accel_factor", _fallback_sampling_accel_factor_);
  param_loader.loadParam(yaml_prefix + "fallback_sampling/stopping_time", _fallback_sampling_stopping_time_);
  param_loader.loadParam(yaml_prefix + "fallback_sampling/first_waypoint_additional_stop", _fallback_sampling_first_waypoint_additional_stop_);

  param_loader.loadParam(yaml_prefix + "check_trajectory_deviation/enabled", _trajectory_max_segment_deviation_enabled_);
  param_loader.loadParam(yaml_prefix + "check_trajectory_deviation/max_deviation", params_.max_deviation);
  param_loader.loadParam(yaml_prefix + "check_trajectory_deviation/max_iterations", _trajectory_max_segment_deviation_max_iterations_);

  param_loader.loadParam(yaml_prefix + "path_straightener/enabled", _path_straightener_enabled_);
  param_loader.loadParam(yaml_prefix + "path_straightener/max_deviation", _path_straightener_max_deviation_);
  param_loader.loadParam(yaml_prefix + "path_straightener/max_hdg_deviation", _path_straightener_max_hdg_deviation_);

  param_loader.loadParam(yaml_prefix + "override_heading_atan2", _override_heading_atan2_);

  param_loader.loadParam(yaml_prefix + "min_waypoint_distance", _min_waypoint_distance_);

  param_loader.loadParam("mrs_uav_managers/uav_manager/takeoff/takeoff_height", _takeoff_height_);

  // | --------------------- tf transformer --------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(nh_, "TrajectoryGeneration");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ------------------- scope timer logger ------------------- |

  param_loader.loadParam(yaml_prefix + "scope_timer/enabled", scope_timer_enabled_);
  const std::string scope_timer_log_filename = param_loader.loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(scope_timer_log_filename, scope_timer_enabled_);

  // | --------------------- service clients -------------------- |

  param_loader.loadParam(yaml_prefix + "time_penalty", params_.time_penalty);
  param_loader.loadParam(yaml_prefix + "soft_constraints_enabled", params_.soft_constraints_enabled);
  param_loader.loadParam(yaml_prefix + "soft_constraints_weight", params_.soft_constraints_weight);
  param_loader.loadParam(yaml_prefix + "time_allocation", params_.time_allocation);
  param_loader.loadParam(yaml_prefix + "equality_constraint_tolerance", params_.equality_constraint_tolerance);
  param_loader.loadParam(yaml_prefix + "inequality_constraint_tolerance", params_.inequality_constraint_tolerance);
  param_loader.loadParam(yaml_prefix + "max_iterations", params_.max_iterations);
  param_loader.loadParam(yaml_prefix + "derivative_to_optimize", params_.derivative_to_optimize);

  param_loader.loadParam(yaml_prefix + "max_time", params_.max_execution_time);

  max_execution_time_ = params_.max_execution_time;

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[TrajectoryGeneration]: could not load all parameters!");
    ros::shutdown();
  }

  // | -------------------- batch visualizer -------------------- |

  bw_original_ = mrs_lib::BatchVisualizer(nh_, "markers/original", "");

  bw_original_.clearBuffers();
  bw_original_.clearVisuals();

  bw_final_ = mrs_lib::BatchVisualizer(nh_, "markers/final", "");

  bw_final_.clearBuffers();
  bw_final_.clearVisuals();

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(params_);
  Drs_t::CallbackType f = boost::bind(&MrsTrajectoryGeneration::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | --------------------- finish the init -------------------- |

  ROS_INFO_ONCE("[TrajectoryGeneration]: initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- main routines --------------------- |

/*
 * 1. preprocessPath(): preprocessing the incoming path
 *    - throughs away too close waypoints
 *    - straightness path by neglecting waypoints close to segments
 * 2. optimize(): solves the whole problem including
 *    - subdivision for satisfying max deviation
 * 3. findTrajectory(): solves single instance by the ETH tool
 * 4. findTrajectoryFallback(): Baca's sampling for backup solution
 * 5. validateTrajectorySpatial(): checks for the spatial soundness of a trajectory vs. the original path
 */

/* preprocessPath() //{ */

std::vector<Waypoint_t> MrsTrajectoryGeneration::preprocessPath(const std::vector<Waypoint_t>& waypoints_in) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("MrsTrajectoryGeneration::preprocessPath", scope_timer_logger_, scope_timer_enabled_);

  std::vector<Waypoint_t> waypoints;

  size_t last_added_idx = 0;  // in "waypoints_in"

  for (size_t i = 0; i < waypoints_in.size(); i++) {

    double x       = waypoints_in.at(i).coords(0);
    double y       = waypoints_in.at(i).coords(1);
    double z       = waypoints_in.at(i).coords(2);
    double heading = waypoints_in.at(i).coords(3);

    bw_original_.addPoint(vec3_t(x, y, z), 1.0, 0.0, 0.0, 1.0);

    if (_path_straightener_enabled_ && waypoints_in.size() >= 3 && i > 0 && i < (waypoints_in.size() - 1)) {

      vec3_t first(waypoints_in.at(last_added_idx).coords(0), waypoints_in.at(last_added_idx).coords(1), waypoints_in.at(last_added_idx).coords(2));
      vec3_t last(waypoints_in.at(i + 1).coords(0), waypoints_in.at(i + 1).coords(1), waypoints_in.at(i + 1).coords(2));

      double first_hdg = waypoints_in.at(last_added_idx).coords(3);
      double last_hdg  = waypoints_in.at(i + 1).coords(3);

      size_t next_point = last_added_idx + 1;

      bool segment_is_ok = true;

      for (size_t j = next_point; j < i + 1; j++) {

        vec3_t mid(waypoints_in.at(j).coords(0), waypoints_in.at(j).coords(1), waypoints_in.at(j).coords(2));
        double mid_hdg = waypoints_in.at(j).coords(3);

        double dist_from_segment = distFromSegment(mid, first, last);

        if (dist_from_segment > _path_straightener_max_deviation_ || fabs(radians::diff(first_hdg, mid_hdg) > _path_straightener_max_hdg_deviation_) ||
            fabs(radians::diff(last_hdg, mid_hdg) > _path_straightener_max_hdg_deviation_)) {
          segment_is_ok = false;
          break;
        }
      }

      if (segment_is_ok) {
        continue;
      }
    }

    if (i > 0 && i < (waypoints_in.size() - 1)) {

      vec3_t first(waypoints_in.at(last_added_idx).coords(0), waypoints_in.at(last_added_idx).coords(1), waypoints_in.at(last_added_idx).coords(2));
      vec3_t last(waypoints_in.at(i).coords(0), waypoints_in.at(i).coords(1), waypoints_in.at(i).coords(2));

      if (mrs_lib::geometry::dist(first, last) < _min_waypoint_distance_) {
        ROS_INFO("[TrajectoryGeneration]: waypoint #%d too close (< %.3f m) to the previous one (#%d), throwing it away", int(i), _min_waypoint_distance_,
                 int(last_added_idx));
        continue;
      }
    }

    Waypoint_t wp;
    wp.coords  = Eigen::Vector4d(x, y, z, heading);
    wp.stop_at = waypoints_in.at(i).stop_at;
    waypoints.push_back(wp);

    last_added_idx = i;
  }

  return waypoints;
}

//}

/* prepareInitialCondition() //{ */

std::tuple<std::optional<mrs_msgs::TrackerCommand>, bool, int> MrsTrajectoryGeneration::prepareInitialCondition(const ros::Time path_time) {

  if (dont_prepend_initial_condition_) {
    return {{}, false, 0};
  }

  auto tracker_cmd = sh_tracker_cmd_.getMsg();

  // | ------------- prepare the initial conditions ------------- |

  mrs_msgs::TrackerCommand initial_condition;

  if (!sh_tracker_cmd_.hasMsg() || ((ros::Time::now() - sh_tracker_cmd_.lastMsgTime())).toSec() > 1.0) {

    auto uav_state = sh_uav_state_.getMsg();

    initial_condition.position = uav_state->pose.position;

    try {
      initial_condition.heading = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();
    }
    catch (...) {
      ROS_WARN_THROTTLE(1.0, "[MrsTrajectoryGeneration]: could not obtain heading from the UAV State");
    }

    initial_condition.position.z += _takeoff_height_;

    initial_condition.header = uav_state->header;

    return {initial_condition, false, 0};
  }

  bool path_from_future = false;

  // positive = in the future
  double path_time_offset = 0;

  if (path_time != ros::Time(0)) {
    path_time_offset = (path_time - ros::Time::now()).toSec();
  }

  int path_sample_offset = 0;

  // if the desired path starts in the future, more than one MPC step ahead
  if (path_time_offset > 0.2) {

    ROS_INFO("[TrajectoryGeneration]: desired path is from the future by %.2f s", path_time_offset);

    // calculate the offset in samples in the predicted trajectory
    // 0.01 is subtracted for the first sample, which is smaller
    // +1 is added due to the first sample, which was subtarcted
    path_sample_offset = int(ceil((path_time_offset * FUTURIZATION_FIRST_WAYPOINT_FACTOR - 0.01) / 0.2)) + 1;

    if (path_sample_offset > (int(tracker_cmd->full_state_prediction.position.size()) - 1)) {

      ROS_ERROR("[TrajectoryGeneration]: can not extrapolate into the waypoints, using tracker_cmd instead");
      initial_condition = *tracker_cmd;

    } else {

      // copy the sample from the current prediction into TrackerCommand, so that we can easily transform it
      mrs_msgs::TrackerCommand full_state;

      full_state.header = tracker_cmd->full_state_prediction.header;

      full_state.position     = tracker_cmd->full_state_prediction.position.at(path_sample_offset);
      full_state.velocity     = tracker_cmd->full_state_prediction.velocity.at(path_sample_offset);
      full_state.acceleration = tracker_cmd->full_state_prediction.acceleration.at(path_sample_offset);
      full_state.jerk         = tracker_cmd->full_state_prediction.jerk.at(path_sample_offset);

      full_state.heading              = tracker_cmd->full_state_prediction.heading.at(path_sample_offset);
      full_state.heading_rate         = tracker_cmd->full_state_prediction.heading_rate.at(path_sample_offset);
      full_state.heading_acceleration = tracker_cmd->full_state_prediction.heading_acceleration.at(path_sample_offset);
      full_state.heading_jerk         = tracker_cmd->full_state_prediction.heading_jerk.at(path_sample_offset);

      ROS_INFO("[TrajectoryGeneration]: getting initial condition from the %d-th sample of the MPC prediction", path_sample_offset);

      initial_condition.header = full_state.header;

      initial_condition.position     = full_state.position;
      initial_condition.velocity     = full_state.velocity;
      initial_condition.acceleration = full_state.acceleration;
      initial_condition.jerk         = full_state.jerk;

      initial_condition.heading              = full_state.heading;
      initial_condition.heading_rate         = full_state.heading_rate;
      initial_condition.heading_acceleration = full_state.heading_acceleration;
      initial_condition.heading_jerk         = full_state.heading_jerk;

      path_from_future = true;
    }

  } else {

    ROS_INFO("[TrajectoryGeneration]: desired path is NOT from the future, using tracker_cmd as the initial condition");

    initial_condition = *tracker_cmd;
  }

  auto control_manager_diag = sh_control_manager_diag_.getMsg();

  if (path_time == ros::Time(0)) {
    if (!control_manager_diag->tracker_status.have_goal) {
      initial_condition.header.stamp = ros::Time(0);
    }
  }

  return {{initial_condition}, path_from_future, path_sample_offset};
}

//}

/* optimize() //{ */

std::tuple<bool, std::string, mrs_msgs::TrajectoryReference, bool> MrsTrajectoryGeneration::optimize(const std::vector<Waypoint_t>& waypoints_in,
                                                                                                     const std_msgs::Header&        waypoints_header,
                                                                                                     const bool fallback_sampling, const bool relax_heading) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("MrsTrajectoryGeneration::optimize", scope_timer_logger_, scope_timer_enabled_);

  ros::Time optimize_time_start = ros::Time::now();

  // | ---------------- reset the visual markers ---------------- |

  bw_original_.clearBuffers();
  bw_original_.clearVisuals();
  bw_final_.clearBuffers();
  bw_final_.clearVisuals();

  bw_original_.setParentFrame(transformer_->resolveFrame(frame_id_));
  bw_final_.setParentFrame(transformer_->resolveFrame(frame_id_));

  bw_original_.setPointsScale(0.4);
  bw_final_.setPointsScale(0.35);

  // empty path is invalid
  if (waypoints_in.size() == 0) {
    std::stringstream ss;
    ss << "the path is empty (before postprocessing)";
    ROS_ERROR_STREAM("[TrajectoryGeneration]: " << ss.str());
    return std::tuple(false, ss.str(), mrs_msgs::TrajectoryReference(), false);
  }

  std::vector<Waypoint_t> waypoints_in_with_init = waypoints_in;

  double path_time_offset = (waypoints_header.stamp - ros::Time::now()).toSec();

  if (path_time_offset > 0.2 && waypoints_in_with_init.size() >= 2) {
    waypoints_in_with_init.erase(waypoints_in_with_init.begin());
  }

  auto [initial_condition, path_from_future, path_sample_offset] = prepareInitialCondition(waypoints_header.stamp);

  // prepend the initial condition
  if (initial_condition) {

    Waypoint_t initial_waypoint;
    initial_waypoint.coords =
        Eigen::Vector4d(initial_condition->position.x, initial_condition->position.y, initial_condition->position.z, initial_condition->heading);
    initial_waypoint.stop_at = false;
    waypoints_in_with_init.insert(waypoints_in_with_init.begin(), initial_waypoint);

  } else {
    if (!dont_prepend_initial_condition_) {
      fly_now_ = false;
    }
  }

  std::vector<Waypoint_t> waypoints = preprocessPath(waypoints_in_with_init);

  if (waypoints.size() <= 1) {
    std::stringstream ss;
    ss << "the path is empty (after postprocessing)";
    ROS_ERROR_STREAM("[TrajectoryGeneration]: " << ss.str());
    return std::tuple(false, ss.str(), mrs_msgs::TrajectoryReference(), false);
  }

  bool              safe = false;
  int               traj_idx;
  std::vector<bool> segment_safeness;
  double            max_deviation = 0;

  eth_mav_msgs::EigenTrajectoryPoint::Vector trajectory;

  double sampling_dt = 0;

  if (path_from_future) {
    ROS_INFO("[TrajectoryGeneration]: changing dt = 0.2, cause the path is from the future");
    sampling_dt = 0.2;
  } else {
    sampling_dt = _sampling_dt_;
  }

  std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector> result;

  auto params = mrs_lib::get_mutexed(mutex_params_, params_);

  if (params.enforce_fallback_solver) {
    ROS_WARN("[TrajectoryGeneration]: fallback sampling enforced");
    result = findTrajectoryFallback(waypoints, sampling_dt, relax_heading);
  } else if (fallback_sampling) {
    ROS_WARN("[TrajectoryGeneration]: executing fallback sampling");
    result = findTrajectoryFallback(waypoints, sampling_dt, relax_heading);
  } else if (running_async_planning_) {
    ROS_WARN("[TrajectoryGeneration]: executing fallback sampling, the previous async task is still running");
    result = findTrajectoryFallback(waypoints, sampling_dt, relax_heading);
  } else if (overtime()) {
    ROS_WARN("[TrajectoryGeneration]: executing fallback sampling, we are running over time");
    result = findTrajectoryFallback(waypoints, sampling_dt, relax_heading);
  } else {

    result = findTrajectoryAsync(waypoints, initial_condition, sampling_dt, relax_heading);
  }

  if (result) {
    trajectory = result.value();
  } else {
    std::stringstream ss;
    ss << "failed to find trajectory";
    ROS_ERROR_STREAM("[TrajectoryGeneration]: " << ss.str());
    return std::tuple(false, ss.str(), mrs_msgs::TrajectoryReference(), false);
  }

  for (int k = 0; k < _trajectory_max_segment_deviation_max_iterations_; k++) {

    ROS_DEBUG("[TrajectoryGeneration]: revalidation cycle #%d", k);

    std::tie(safe, traj_idx, segment_safeness, max_deviation) = validateTrajectorySpatial(trajectory, waypoints);

    if (_trajectory_max_segment_deviation_enabled_ && !safe) {

      ROS_DEBUG("[TrajectoryGeneration]: trajectory is not safe, max deviation %.3f m", max_deviation);

      std::vector<Waypoint_t>::iterator waypoint = waypoints.begin();
      std::vector<bool>::iterator       safeness = segment_safeness.begin();

      for (; waypoint < waypoints.end() - 1; waypoint++) {

        if (!(*safeness)) {

          if (waypoint > waypoints.begin() || max_deviation_first_segment_ || int(waypoints.size()) <= 2) {
            Waypoint_t midpoint2 = interpolatePoint(*waypoint, *(waypoint + 1), 0.5);
            waypoint             = waypoints.insert(waypoint + 1, midpoint2);
          }
        }

        safeness++;
      }

      if (params.enforce_fallback_solver) {
        ROS_WARN("[TrajectoryGeneration]: fallback sampling enforced");
        result = findTrajectoryFallback(waypoints, sampling_dt, relax_heading);
      } else if (fallback_sampling) {
        ROS_WARN("[TrajectoryGeneration]: executing fallback sampling");
        result = findTrajectoryFallback(waypoints, sampling_dt, relax_heading);
      } else if (running_async_planning_) {
        ROS_WARN("[TrajectoryGeneration]: executing fallback sampling, the previous async task is still running");
        result = findTrajectoryFallback(waypoints, sampling_dt, relax_heading);
      } else if (overtime()) {
        ROS_WARN("[TrajectoryGeneration]: executing fallback sampling, we are running over time");
        result = findTrajectoryFallback(waypoints, sampling_dt, relax_heading);
      } else {
        result = findTrajectoryAsync(waypoints, initial_condition, sampling_dt, relax_heading);
      }

      if (result) {
        trajectory = result.value();
      } else {
        std::stringstream ss;
        ss << "failed to find trajectory";
        ROS_WARN_STREAM("[TrajectoryGeneration]: " << ss.str());
        return std::tuple(false, ss.str(), mrs_msgs::TrajectoryReference(), false);
      }

    } else {
      ROS_DEBUG("[TrajectoryGeneration]: trajectory is safe (%.2f)", max_deviation);
      safe = true;
      break;
    }
  }

  ROS_INFO("[TrajectoryGeneration]: final max trajectory-path deviation: %.2f m, total trajectory time: %.2fs ", max_deviation,
           trajectory.size() * sampling_dt);

  // prepare rviz markers
  for (int i = 0; i < int(waypoints.size()); i++) {
    bw_final_.addPoint(vec3_t(waypoints.at(i).coords(0), waypoints.at(i).coords(1), waypoints.at(i).coords(2)), 0.0, 1.0, 0.0, 1.0);
  }

  mrs_msgs::TrajectoryReference mrs_trajectory;

  // convert the optimized trajectory to mrs_msgs::TrajectoryReference
  mrs_trajectory = getTrajectoryReference(trajectory, initial_condition, sampling_dt);

  // insert part of the MPC prediction in the front of the generated trajectory to compensate for the future
  if (path_from_future) {

    auto current_prediction = sh_tracker_cmd_.getMsg()->full_state_prediction;

    // calculate the starting idx that we will use from the current_prediction
    double path_time_offset_2   = (ros::Time::now() - current_prediction.header.stamp).toSec();  // = how long did it take to optimize
    int    path_sample_offset_2 = int(floor((path_time_offset_2 - 0.01) / 0.2)) + 1;

    // if there is anything to insert
    if (path_sample_offset > path_sample_offset_2) {

      ROS_INFO("[TrajectoryGeneration]: inserting pre-trajectory from the prediction, idxs %d to %d", path_sample_offset_2, path_sample_offset);

      for (int i = path_sample_offset - 1; i >= 0; i--) {

        ROS_DEBUG("[TrajectoryGeneration]: inserting idx %d", i);

        mrs_msgs::ReferenceStamped reference;

        reference.header = current_prediction.header;

        reference.reference.heading = current_prediction.heading.at(i);

        reference.reference.position = current_prediction.position.at(i);

        auto res = transformer_->transformSingle(reference, waypoints_header.frame_id);

        if (res) {
          reference = res.value();
        } else {
          std::stringstream ss;
          ss << "could not transform reference to the path frame";
          ROS_ERROR_STREAM("[TrajectoryGeneration]: " << ss.str());
          return std::tuple(false, ss.str(), mrs_msgs::TrajectoryReference(), false);
        }

        mrs_trajectory.points.insert(mrs_trajectory.points.begin(), reference.reference);
      }
    }
  }

  bw_original_.publish();
  bw_final_.publish();

  std::stringstream ss;
  ss << "trajectory generated";

  ROS_DEBUG("[TrajectoryGeneration]: trajectory generated, took %.3f s", (ros::Time::now() - optimize_time_start).toSec());

  return std::tuple(true, ss.str(), mrs_trajectory, initial_condition.has_value());
}

//}

/* findTrajectory() //{ */

std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector> MrsTrajectoryGeneration::findTrajectory(const std::vector<Waypoint_t>&                 waypoints,
                                                                                                  const std::optional<mrs_msgs::TrackerCommand>& initial_state,
                                                                                                  const double& sampling_dt, const bool& relax_heading) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("MrsTrajectoryGeneration::findTrajectory", scope_timer_logger_, scope_timer_enabled_);

  mrs_lib::AtomicScopeFlag unset_running(running_async_planning_);

  ROS_DEBUG("[TrajectoryGeneration]: findTrajectory() started");

  ros::Time find_trajectory_time_start = ros::Time::now();

  auto params      = mrs_lib::get_mutexed(mutex_params_, params_);
  auto constraints = sh_constraints_.getMsg();

  auto control_manager_diag = sh_control_manager_diag_.getMsg();

  if (initial_state && (initial_state->header.stamp - ros::Time::now()).toSec() < 0.2 && control_manager_diag->tracker_status.have_goal) {
    max_deviation_first_segment_ = false;
  } else {
    max_deviation_first_segment_ = true;
  }

  // optimizer

  eth_trajectory_generation::NonlinearOptimizationParameters parameters;

  parameters.f_rel                  = 0.05;
  parameters.x_rel                  = 0.1;
  parameters.time_penalty           = params.time_penalty;
  parameters.use_soft_constraints   = params.soft_constraints_enabled;
  parameters.soft_constraint_weight = params.soft_constraints_weight;
  parameters.time_alloc_method      = static_cast<eth_trajectory_generation::NonlinearOptimizationParameters::TimeAllocMethod>(params.time_allocation);
  if (params.time_allocation == 2) {
    parameters.algorithm = nlopt::LD_LBFGS;
  }
  parameters.initial_stepsize_rel            = 0.1;
  parameters.inequality_constraint_tolerance = params.inequality_constraint_tolerance;
  parameters.equality_constraint_tolerance   = params.equality_constraint_tolerance;
  parameters.max_iterations                  = params.max_iterations;

  // let's tell the solver tha it is more time then it thinks, to not stop prematurely
  parameters.max_time = 2 * (NLOPT_EXEC_TIME_FACTOR * timeLeft());

  eth_trajectory_generation::Vertex::Vector vertices;
  const int                                 dimension = 4;

  int derivative_to_optimize = eth_trajectory_generation::derivative_order::ACCELERATION;

  switch (params.derivative_to_optimize) {
    case 0: {
      derivative_to_optimize = eth_trajectory_generation::derivative_order::ACCELERATION;
      break;
    }
    case 1: {
      derivative_to_optimize = eth_trajectory_generation::derivative_order::JERK;
      break;
    }
    case 2: {
      derivative_to_optimize = eth_trajectory_generation::derivative_order::SNAP;
      break;
    }
  }

  // | --------------- add constraints to vertices -------------- |

  double last_heading;

  if (initial_state) {
    last_heading = initial_state->heading;
  } else {
    last_heading = waypoints.at(0).coords(3);
  }

  for (size_t i = 0; i < waypoints.size(); i++) {
    double x       = waypoints.at(i).coords(0);
    double y       = waypoints.at(i).coords(1);
    double z       = waypoints.at(i).coords(2);
    double heading = sradians::unwrap(waypoints.at(i).coords(3), last_heading);
    last_heading   = heading;

    eth_trajectory_generation::Vertex vertex(dimension);

    if (i == 0) {

      vertex.makeStartOrEnd(Eigen::Vector4d(x, y, z, heading), derivative_to_optimize);

      vertex.addConstraint(eth_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));

      if (initial_state) {

        vertex.addConstraint(eth_trajectory_generation::derivative_order::VELOCITY,
                             Eigen::Vector4d(initial_state->velocity.x, initial_state->velocity.y, initial_state->velocity.z, initial_state->heading_rate));

        vertex.addConstraint(
            eth_trajectory_generation::derivative_order::ACCELERATION,
            Eigen::Vector4d(initial_state->acceleration.x, initial_state->acceleration.y, initial_state->acceleration.z, initial_state->heading_acceleration));

        vertex.addConstraint(eth_trajectory_generation::derivative_order::JERK,
                             Eigen::Vector4d(initial_state->jerk.x, initial_state->jerk.y, initial_state->jerk.z, initial_state->heading_jerk));
      }

    } else if (i == (waypoints.size() - 1)) {  // the last point

      vertex.makeStartOrEnd(Eigen::Vector4d(x, y, z, heading), derivative_to_optimize);

      vertex.addConstraint(eth_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));

    } else {  // mid points

      vertex.addConstraint(eth_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));

      if (waypoints.at(i).stop_at) {
        vertex.addConstraint(eth_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector4d(0, 0, 0, 0));
        vertex.addConstraint(eth_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector4d(0, 0, 0, 0));
        vertex.addConstraint(eth_trajectory_generation::derivative_order::JERK, Eigen::Vector4d(0, 0, 0, 0));
      }
    }

    vertices.push_back(vertex);
  }

  // | ---------------- compute the segment times --------------- |

  double v_max_horizontal, a_max_horizontal, j_max_horizontal;
  double v_max_vertical, a_max_vertical, j_max_vertical;

  // use the small of the ascending/descending values
  double vertical_speed_lim        = std::min(constraints->vertical_ascending_speed, constraints->vertical_descending_speed);
  double vertical_acceleration_lim = std::min(constraints->vertical_ascending_acceleration, constraints->vertical_descending_acceleration);

  v_max_horizontal = constraints->horizontal_speed;
  a_max_horizontal = constraints->horizontal_acceleration;

  v_max_vertical = vertical_speed_lim;
  a_max_vertical = vertical_acceleration_lim;

  j_max_horizontal = constraints->horizontal_jerk;
  j_max_vertical   = std::min(constraints->vertical_ascending_jerk, constraints->vertical_descending_jerk);

  if (override_constraints_) {

    bool can_change = true;


    if (initial_state) {
      can_change = (hypot(initial_state->velocity.x, initial_state->velocity.y) < override_max_velocity_horizontal_) &&
                   (hypot(initial_state->acceleration.x, initial_state->acceleration.y) < override_max_acceleration_horizontal_) &&
                   (hypot(initial_state->jerk.x, initial_state->jerk.y) < override_max_jerk_horizontal_) &&
                   (fabs(initial_state->velocity.z) < override_max_velocity_vertical_) &&
                   (fabs(initial_state->acceleration.z) < override_max_acceleration_vertical_) && (fabs(initial_state->jerk.z) < override_max_jerk_vertical_);
    }

    if (can_change) {

      v_max_horizontal = override_max_velocity_horizontal_;
      a_max_horizontal = override_max_acceleration_horizontal_;
      j_max_horizontal = override_max_jerk_horizontal_;

      v_max_vertical = override_max_velocity_vertical_;
      a_max_vertical = override_max_acceleration_vertical_;
      j_max_vertical = override_max_jerk_vertical_;

      ROS_DEBUG("[TrajectoryGeneration]: overriding constraints by a user");

    } else {

      ROS_WARN("[TrajectoryGeneration]: overrifing constraints refused due to possible infeasibility");
    }
  }

  double v_max_heading, a_max_heading, j_max_heading;

  if (relax_heading) {
    v_max_heading = std::numeric_limits<float>::max();
    a_max_heading = std::numeric_limits<float>::max();
    j_max_heading = std::numeric_limits<float>::max();
  } else {
    v_max_heading = constraints->heading_speed;
    a_max_heading = constraints->heading_acceleration;
    j_max_heading = constraints->heading_jerk;
  }

  ROS_DEBUG("[TrajectoryGeneration]: using constraints:");
  ROS_DEBUG("[TrajectoryGeneration]: horizontal: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_horizontal, a_max_horizontal, j_max_horizontal);
  ROS_DEBUG("[TrajectoryGeneration]: vertical: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_vertical, a_max_vertical, j_max_vertical);
  ROS_DEBUG("[TrajectoryGeneration]: heading: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_heading, a_max_heading, j_max_heading);

  std::vector<double> segment_times, segment_times_baca;
  segment_times      = estimateSegmentTimes(vertices, v_max_horizontal, v_max_vertical, a_max_horizontal, a_max_vertical, j_max_horizontal, j_max_vertical,
                                       v_max_heading, a_max_heading);
  segment_times_baca = estimateSegmentTimesBaca(vertices, v_max_horizontal, v_max_vertical, a_max_horizontal, a_max_vertical, j_max_horizontal, j_max_vertical,
                                                v_max_heading, a_max_heading);

  double initial_total_time      = 0;
  double initial_total_time_baca = 0;
  for (int i = 0; i < int(segment_times_baca.size()); i++) {
    initial_total_time += segment_times.at(i);
    initial_total_time_baca += segment_times_baca.at(i);
  }

  ROS_DEBUG("[TrajectoryGeneration]: initial total time (Euclidean): %.2f", initial_total_time);
  ROS_DEBUG("[TrajectoryGeneration]: initial total time (Baca): %.2f", initial_total_time_baca);

  // | --------- create an optimizer object and solve it -------- |

  const int                                                     N = 10;
  eth_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  opt.addMaximumMagnitudeConstraint(0, eth_trajectory_generation::derivative_order::VELOCITY, v_max_horizontal);
  opt.addMaximumMagnitudeConstraint(0, eth_trajectory_generation::derivative_order::ACCELERATION, a_max_horizontal);
  opt.addMaximumMagnitudeConstraint(0, eth_trajectory_generation::derivative_order::JERK, j_max_horizontal);

  opt.addMaximumMagnitudeConstraint(1, eth_trajectory_generation::derivative_order::VELOCITY, v_max_horizontal);
  opt.addMaximumMagnitudeConstraint(1, eth_trajectory_generation::derivative_order::ACCELERATION, a_max_horizontal);
  opt.addMaximumMagnitudeConstraint(1, eth_trajectory_generation::derivative_order::JERK, j_max_horizontal);

  opt.addMaximumMagnitudeConstraint(2, eth_trajectory_generation::derivative_order::VELOCITY, v_max_vertical);
  opt.addMaximumMagnitudeConstraint(2, eth_trajectory_generation::derivative_order::ACCELERATION, a_max_vertical);
  opt.addMaximumMagnitudeConstraint(2, eth_trajectory_generation::derivative_order::JERK, j_max_vertical);

  opt.addMaximumMagnitudeConstraint(3, eth_trajectory_generation::derivative_order::VELOCITY, v_max_heading);
  opt.addMaximumMagnitudeConstraint(3, eth_trajectory_generation::derivative_order::ACCELERATION, a_max_heading);
  opt.addMaximumMagnitudeConstraint(3, eth_trajectory_generation::derivative_order::JERK, j_max_heading);

  opt.optimize();

  if (overtime()) {
    return {};
  }

  std::string result_str;

  switch (opt.getOptimizationInfo().stopping_reason) {
    case nlopt::FAILURE: {
      result_str = "generic failure";
      break;
    }
    case nlopt::INVALID_ARGS: {
      result_str = "invalid args";
      break;
    }
    case nlopt::OUT_OF_MEMORY: {
      result_str = "out of memory";
      break;
    }
    case nlopt::ROUNDOFF_LIMITED: {
      result_str = "roundoff limited";
      break;
    }
    case nlopt::FORCED_STOP: {
      result_str = "forced stop";
      break;
    }
    case nlopt::STOPVAL_REACHED: {
      result_str = "stopval reached";
      break;
    }
    case nlopt::FTOL_REACHED: {
      result_str = "ftol reached";
      break;
    }
    case nlopt::XTOL_REACHED: {
      result_str = "xtol reached";
      break;
    }
    case nlopt::MAXEVAL_REACHED: {
      result_str = "maxeval reached";
      break;
    }
    case nlopt::MAXTIME_REACHED: {
      result_str = "maxtime reached";
      break;
    }
    default: {
      result_str = "UNKNOWN FAILURE CODE";
      break;
    }
  }

  if (opt.getOptimizationInfo().stopping_reason >= 1 && opt.getOptimizationInfo().stopping_reason != 6) {
    ROS_DEBUG("[TrajectoryGeneration]: optimization finished successfully with code %d, '%s'", opt.getOptimizationInfo().stopping_reason, result_str.c_str());

  } else if (opt.getOptimizationInfo().stopping_reason == -1) {
    ROS_DEBUG("[TrajectoryGeneration]: optimization finished with a generic error code %d, '%s'", opt.getOptimizationInfo().stopping_reason,
              result_str.c_str());

  } else {
    ROS_WARN("[TrajectoryGeneration]: optimization failed with code %d, '%s', took %.3f s", opt.getOptimizationInfo().stopping_reason, result_str.c_str(),
             (ros::Time::now() - find_trajectory_time_start).toSec());
    return {};
  }

  // | ------------- obtain the polynomial segments ------------- |

  eth_trajectory_generation::Segment::Vector segments;
  opt.getPolynomialOptimizationRef().getSegments(&segments);

  if (overtime()) {
    return {};
  }

  // | --------------- create the trajectory class -------------- |

  eth_trajectory_generation::Trajectory trajectory;
  opt.getTrajectory(&trajectory);

  eth_mav_msgs::EigenTrajectoryPoint::Vector states;

  ROS_DEBUG("[TrajectoryGeneration]: starting eth sampling with dt = %.2f s ", sampling_dt);

  bool success = eth_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_dt, &states);

  if (overtime()) {
    return {};
  }

  // validate the temporal sampling of the trajectory

  // only check this if the trajectory is > 1.0 sec, this check does not make much sense for the short ones
  if ((states.size() * sampling_dt) > 1.0 && (states.size() * sampling_dt) > (_max_trajectory_len_factor_ * initial_total_time_baca)) {
    ROS_ERROR("[TrajectoryGeneration]: the final trajectory sampling is too long = %.2f, initial 'baca' estimate = %.2f, allowed factor %.2f, aborting",
              (states.size() * sampling_dt), initial_total_time_baca, _max_trajectory_len_factor_);

    std::stringstream ss;
    ss << "trajectory sampling failed";
    ROS_ERROR_STREAM("[TrajectoryGeneration]: " << ss.str());
    return {};

  } else if ((states.size() * sampling_dt) > 1.0 && (states.size() * sampling_dt) < (_min_trajectory_len_factor_ * initial_total_time_baca)) {
    ROS_ERROR("[TrajectoryGeneration]: the final trajectory sampling is too short = %.2f, initial 'baca' estimate = %.2f, allowed factor %.2f, aborting",
              (states.size() * sampling_dt), initial_total_time_baca, _min_trajectory_len_factor_);

    std::stringstream ss;
    ss << "trajectory sampling failed";
    ROS_ERROR_STREAM("[TrajectoryGeneration]: " << ss.str());
    return {};

  } else {
    ROS_DEBUG("[TrajectoryGeneration]: estimated/final trajectory length ratio (final/estimated) %.2f",
              (states.size() * sampling_dt) / initial_total_time_baca);
  }

  if (success) {
    ROS_DEBUG("[TrajectoryGeneration]: eth sampling finished, took %.3f s", (ros::Time::now() - find_trajectory_time_start).toSec());
    return std::optional(states);

  } else {
    ROS_ERROR("[TrajectoryGeneration]: eth could not sample the trajectory, took %.3f s", (ros::Time::now() - find_trajectory_time_start).toSec());
    return {};
  }
}

//}

/* findTrajectoryFallback() //{ */

std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector> MrsTrajectoryGeneration::findTrajectoryFallback(const std::vector<Waypoint_t>& waypoints,
                                                                                                          const double&                  sampling_dt,
                                                                                                          const bool&                    relax_heading) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("MrsTrajectoryGeneration::findTrajectoryFallback", scope_timer_logger_, scope_timer_enabled_);

  ros::Time time_start = ros::Time::now();

  ROS_WARN("[TrajectoryGeneration]: fallback sampling started");

  auto params      = mrs_lib::get_mutexed(mutex_params_, params_);
  auto constraints = sh_constraints_.getMsg();

  eth_trajectory_generation::Vertex::Vector vertices;
  const int                                 dimension = 4;

  // | --------------- add constraints to vertices -------------- |

  double last_heading = waypoints.at(0).coords(3);

  for (size_t i = 0; i < waypoints.size(); i++) {

    double x       = waypoints.at(i).coords(0);
    double y       = waypoints.at(i).coords(1);
    double z       = waypoints.at(i).coords(2);
    double heading = sradians::unwrap(waypoints.at(i).coords(3), last_heading);
    last_heading   = heading;

    eth_trajectory_generation::Vertex vertex(dimension);

    vertex.addConstraint(eth_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));

    vertices.push_back(vertex);
  }

  // | ---------------- compute the segment times --------------- |

  double v_max_horizontal, a_max_horizontal, j_max_horizontal;
  double v_max_vertical, a_max_vertical, j_max_vertical;

  // use the small of the ascending/descending values
  double vertical_speed_lim        = std::min(constraints->vertical_ascending_speed, constraints->vertical_descending_speed);
  double vertical_acceleration_lim = std::min(constraints->vertical_ascending_acceleration, constraints->vertical_descending_acceleration);

  if (override_constraints_) {

    v_max_horizontal = override_max_velocity_horizontal_;
    a_max_horizontal = override_max_acceleration_horizontal_;
    j_max_horizontal = override_max_jerk_horizontal_;

    v_max_vertical = override_max_velocity_vertical_;
    a_max_vertical = override_max_acceleration_vertical_;
    j_max_vertical = override_max_jerk_vertical_;

    ROS_DEBUG("[TrajectoryGeneration]: overriding constraints by a user");
  } else {

    v_max_horizontal = constraints->horizontal_speed;
    a_max_horizontal = constraints->horizontal_acceleration;

    v_max_vertical = vertical_speed_lim;
    a_max_vertical = vertical_acceleration_lim;

    j_max_horizontal = constraints->horizontal_jerk;
    j_max_vertical   = std::min(constraints->vertical_ascending_jerk, constraints->vertical_descending_jerk);
  }


  double v_max_heading, a_max_heading, j_max_heading;

  if (relax_heading) {
    v_max_heading = std::numeric_limits<float>::max();
    a_max_heading = std::numeric_limits<float>::max();
    j_max_heading = std::numeric_limits<float>::max();
  } else {
    v_max_heading = constraints->heading_speed;
    a_max_heading = constraints->heading_acceleration;
    j_max_heading = constraints->heading_jerk;
  }

  v_max_horizontal *= _fallback_sampling_speed_factor_;
  v_max_vertical *= _fallback_sampling_speed_factor_;

  a_max_horizontal *= _fallback_sampling_accel_factor_;
  a_max_vertical *= _fallback_sampling_accel_factor_;

  ROS_DEBUG("[TrajectoryGeneration]: using constraints:");
  ROS_DEBUG("[TrajectoryGeneration]: horizontal: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_horizontal, a_max_horizontal, j_max_horizontal);
  ROS_DEBUG("[TrajectoryGeneration]: vertical: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_vertical, a_max_vertical, j_max_vertical);
  ROS_DEBUG("[TrajectoryGeneration]: heading: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_heading, a_max_heading, j_max_heading);

  std::vector<double> segment_times, segment_times_baca;
  segment_times      = estimateSegmentTimes(vertices, v_max_horizontal, v_max_vertical, a_max_horizontal, a_max_vertical, j_max_horizontal, j_max_vertical,
                                       v_max_heading, a_max_heading);
  segment_times_baca = estimateSegmentTimesBaca(vertices, v_max_horizontal, v_max_vertical, a_max_horizontal, a_max_vertical, j_max_horizontal, j_max_vertical,
                                                v_max_heading, a_max_heading);

  double initial_total_time      = 0;
  double initial_total_time_baca = 0;
  for (int i = 0; i < int(segment_times_baca.size()); i++) {
    initial_total_time += segment_times.at(i);
    initial_total_time_baca += segment_times_baca.at(i);

    ROS_DEBUG("[TrajectoryGeneration]: segment time [%d] = %.2f", i, segment_times_baca.at(i));
  }

  ROS_WARN("[TrajectoryGeneration]: fallback: initial total time (Euclidean): %.2f", initial_total_time);
  ROS_WARN("[TrajectoryGeneration]: fallback: initial total time (Baca): %.2f", initial_total_time_baca);

  eth_mav_msgs::EigenTrajectoryPoint::Vector states;

  // interpolate each segment
  for (size_t i = 0; i < waypoints.size() - 1; i++) {

    Eigen::VectorXd start, end;

    const double segment_time = segment_times_baca.at(i);

    int    n_samples;
    double interp_step;

    if (segment_time > 1e-1) {

      n_samples = ceil(segment_time / sampling_dt);

      // important
      if (n_samples > 0) {
        interp_step = 1.0 / double(n_samples);
      } else {
        interp_step = 0.5;
      }

    } else {
      n_samples   = 0;
      interp_step = 0;
    }

    ROS_DEBUG("[TrajectoryGeneration]: segment n_samples [%lu] = %d", i, n_samples);

    // for the last segment, hit the last waypoint completely
    // otherwise, it is hit as the first sample of the following segment
    if (n_samples > 0 && i == waypoints.size() - 2) {
      n_samples++;
    }

    for (int j = 0; j < n_samples; j++) {

      Waypoint_t point = interpolatePoint(waypoints.at(i), waypoints.at(i + 1), j * interp_step);

      eth_mav_msgs::EigenTrajectoryPoint eth_point;
      eth_point.position_W(0) = point.coords(0);
      eth_point.position_W(1) = point.coords(1);
      eth_point.position_W(2) = point.coords(2);
      eth_point.setFromYaw(point.coords(3));

      states.push_back(eth_point);

      if (j == 0 && i > 0 && waypoints.at(i).stop_at) {

        int insert_samples = int(round(_fallback_sampling_stopping_time_ / sampling_dt));

        for (int k = 0; k < insert_samples; k++) {
          states.push_back(eth_point);
        }
      }
    }
  }

  bool success = true;

  ROS_WARN("[TrajectoryGeneration]: fallback: sampling finished, took %.3f s", (ros::Time::now() - time_start).toSec());

  // | --------------- create the trajectory class -------------- |

  if (success) {
    return std::optional(states);
  } else {
    ROS_ERROR("[TrajectoryGeneration]: fallback: sampling failed");
    return {};
  }
}

//}

/* validateTrajectorySpatial() //{ */

std::tuple<bool, int, std::vector<bool>, double> MrsTrajectoryGeneration::validateTrajectorySpatial(
    const eth_mav_msgs::EigenTrajectoryPoint::Vector& trajectory, const std::vector<Waypoint_t>& waypoints) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("MrsTrajectoryGeneration::validateTrajectorySpatial", scope_timer_logger_, scope_timer_enabled_);

  // prepare the output

  std::vector<bool> segments;
  for (size_t i = 0; i < waypoints.size() - 1; i++) {
    segments.push_back(true);
  }

  int waypoint_idx = 0;

  bool   is_safe       = true;
  double max_deviation = 0;

  for (size_t i = 0; i < trajectory.size() - 1; i++) {

    // the trajectory sample
    const vec3_t sample = vec3_t(trajectory.at(i).position_W(0), trajectory.at(i).position_W(1), trajectory.at(i).position_W(2));

    // next sample
    const vec3_t next_sample = vec3_t(trajectory.at(i + 1).position_W(0), trajectory.at(i + 1).position_W(1), trajectory.at(i + 1).position_W(2));

    // segment start
    const vec3_t segment_start = vec3_t(waypoints.at(waypoint_idx).coords(0), waypoints.at(waypoint_idx).coords(1), waypoints.at(waypoint_idx).coords(2));

    // segment end
    const vec3_t segment_end =
        vec3_t(waypoints.at(waypoint_idx + 1).coords(0), waypoints.at(waypoint_idx + 1).coords(1), waypoints.at(waypoint_idx + 1).coords(2));

    const double distance_from_segment = distFromSegment(sample, segment_start, segment_end);

    const double segment_end_dist = distFromSegment(segment_end, sample, next_sample);

    if (waypoint_idx > 0 || max_deviation_first_segment_ || int(waypoints.size()) <= 2) {

      if (distance_from_segment > max_deviation) {
        max_deviation = distance_from_segment;
      }

      if (distance_from_segment > trajectory_max_segment_deviation_) {
        segments.at(waypoint_idx) = false;
        is_safe                   = false;
      }
    }

    if (segment_end_dist < 0.05 && waypoint_idx < (int(waypoints.size()) - 2)) {
      waypoint_idx++;
    }
  }

  return std::tuple(is_safe, trajectory.size(), segments, max_deviation);
}

//}

/* getWaypointInTrajectoryIdxs() //{ */

std::vector<int> MrsTrajectoryGeneration::getWaypointInTrajectoryIdxs(const mrs_msgs::TrajectoryReference& trajectory, const std::vector<Waypoint_t>& waypoints) {

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("MrsTrajectoryGeneration::validateTrajectorySpatial", scope_timer_logger_, scope_timer_enabled_);

  // prepare the output

  std::vector<int> idxs;

  int waypoint_idx = 0;

  for (size_t i = 0; i < trajectory.points.size() - 1; i++) {

    // the trajectory sample
    const vec3_t sample = vec3_t(trajectory.points.at(i).position.x, trajectory.points.at(i).position.y, trajectory.points.at(i).position.z);

    // next sample
    const vec3_t next_sample = vec3_t(trajectory.points.at(i + 1).position.x, trajectory.points.at(i + 1).position.y, trajectory.points.at(i + 1).position.z);

    // segment end
    const vec3_t waypoint =
        vec3_t(waypoints.at(waypoint_idx).coords(0), waypoints.at(waypoint_idx).coords(1), waypoints.at(waypoint_idx).coords(2));

    const double waypoint_traj_seg_dist = distFromSegment(waypoint, sample, next_sample);

    ROS_DEBUG("[MrsTrajectoryGeneration]: distance %.3f", waypoint_traj_seg_dist);

    if (waypoint_traj_seg_dist < 0.1) {
      ROS_DEBUG("[MrsTrajectoryGeneration]: waypoint_idx=%d, trajectory_idx=%d/%d", waypoint_idx, int(i), int(trajectory.points.size()));
      idxs.push_back(i);
      waypoint_idx++;
    }

    if (waypoint_idx == int(waypoints.size())) {
      break;
    }
  }

  return idxs;
}

//}

// | --------------------- minor routines --------------------- |

/* findTrajectoryAsync() //{ */

std::optional<eth_mav_msgs::EigenTrajectoryPoint::Vector> MrsTrajectoryGeneration::findTrajectoryAsync(
    const std::vector<Waypoint_t>& waypoints, const std::optional<mrs_msgs::TrackerCommand>& initial_state, const double& sampling_dt,
    const bool& relax_heading) {

  ROS_DEBUG("[TrajectoryGeneration]: starting the async planning task");

  future_trajectory_result_ =
      std::async(std::launch::async, &MrsTrajectoryGeneration::findTrajectory, this, waypoints, initial_state, sampling_dt, relax_heading);

  while (ros::ok() && future_trajectory_result_.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {

    if (overtime()) {
      ROS_WARN("[TrajectoryGeneration]: async task planning timeout, breaking");
      return {};
    }
  }

  ROS_DEBUG("[TrajectoryGeneration]: async planning task finished successfully");

  return future_trajectory_result_.get();
}

//}

/* distFromSegment() //{ */

double MrsTrajectoryGeneration::distFromSegment(const vec3_t& point, const vec3_t& seg1, const vec3_t& seg2) {

  vec3_t segment_vector = seg2 - seg1;
  double segment_len    = segment_vector.norm();

  vec3_t segment_vector_norm = segment_vector;
  segment_vector_norm.normalize();

  double point_coordinate = segment_vector_norm.dot(point - seg1);

  if (point_coordinate < 0) {
    return (point - seg1).norm();
  } else if (point_coordinate > segment_len) {
    return (point - seg2).norm();
  } else {

    mat3_t segment_projector = segment_vector_norm * segment_vector_norm.transpose();
    vec3_t projection        = seg1 + segment_projector * (point - seg1);

    return (point - projection).norm();
  }
}

//}

/* getTrajectoryReference() //{ */

mrs_msgs::TrajectoryReference MrsTrajectoryGeneration::getTrajectoryReference(const eth_mav_msgs::EigenTrajectoryPoint::Vector& trajectory,
                                                                              const std::optional<mrs_msgs::TrackerCommand>&    initial_condition,
                                                                              const double&                                     sampling_dt) {

  mrs_msgs::TrajectoryReference msg;

  if (initial_condition) {
    msg.header.stamp = initial_condition->header.stamp;
  } else {
    msg.header.stamp = ros::Time::now();
  }

  msg.header.frame_id = frame_id_;
  msg.fly_now         = fly_now_;
  msg.loop            = loop_;
  msg.use_heading     = use_heading_;
  msg.dt              = sampling_dt;

  for (size_t it = 0; it < trajectory.size(); it++) {

    mrs_msgs::Reference point;

    point.position.x = trajectory.at(it).position_W(0);
    point.position.y = trajectory.at(it).position_W(1);
    point.position.z = trajectory.at(it).position_W(2);

    if (_override_heading_atan2_ && it < (trajectory.size() - 1)) {

      const double points_dist = std::hypot(trajectory.at(it + 1).position_W(1) - point.position.y, trajectory.at(it + 1).position_W(0) - point.position.x);

      if (points_dist < 0.05 && it > 0) {

        point.heading = msg.points.at(it - 1).heading;

      } else {
        point.heading = atan2(trajectory.at(it + 1).position_W(1) - point.position.y, trajectory.at(it + 1).position_W(0) - point.position.x);
      }

    } else {
      point.heading = trajectory.at(it).getYaw();
    }

    msg.points.push_back(point);
  }

  return msg;
}

//}

/* interpolatePoint() //{ */

Waypoint_t MrsTrajectoryGeneration::interpolatePoint(const Waypoint_t& a, const Waypoint_t& b, const double& coeff) {

  Waypoint_t      out;
  Eigen::Vector4d diff = b.coords - a.coords;

  out.coords(0) = a.coords(0) + coeff * diff(0);
  out.coords(1) = a.coords(1) + coeff * diff(1);
  out.coords(2) = a.coords(2) + coeff * diff(2);
  out.coords(3) = radians::interp(a.coords(3), b.coords(3), coeff);

  out.stop_at = false;

  return out;
}

//}

/* checkNaN() //{ */

bool MrsTrajectoryGeneration::checkNaN(const Waypoint_t& a) {

  if (!std::isfinite(a.coords(0))) {
    ROS_ERROR("NaN detected in variable \"a.coords(0)\"!!!");
    return false;
  }

  if (!std::isfinite(a.coords(1))) {
    ROS_ERROR("NaN detected in variable \"a.coords(1)\"!!!");
    return false;
  }

  if (!std::isfinite(a.coords(2))) {
    ROS_ERROR("NaN detected in variable \"a.coords(2)\"!!!");
    return false;
  }

  if (!std::isfinite(a.coords(3))) {
    ROS_ERROR("NaN detected in variable \"a.coords(3)\"!!!");
    return false;
  }

  return true;
}

//}

/* trajectorySrv() //{ */

bool MrsTrajectoryGeneration::trajectorySrv(const mrs_msgs::TrajectoryReference& msg) {

  mrs_msgs::TrajectoryReferenceSrv srv;
  srv.request.trajectory = msg;

  bool res = service_client_trajectory_reference_.call(srv);

  if (res) {

    if (!srv.response.success) {
      ROS_WARN("[TrajectoryGeneration]: service call for trajectory_reference returned: '%s'", srv.response.message.c_str());
    }

    return srv.response.success;

  } else {

    ROS_ERROR("[TrajectoryGeneration]: service call for trajectory_reference failed!");

    return false;
  }
}

//}

/* transformTrackerCmd() //{ */

std::optional<mrs_msgs::Path> MrsTrajectoryGeneration::transformPath(const mrs_msgs::Path& path_in, const std::string& target_frame) {

  // if we transform to the current control frame, which is in fact the same frame as the tracker_cmd is in
  if (target_frame == path_in.header.frame_id) {
    return path_in;
  }

  // find the transformation
  auto tf = transformer_->getTransform(path_in.header.frame_id, target_frame, path_in.header.stamp);

  if (!tf) {
    ROS_ERROR("[TrajectoryGeneration]: could not find transform from '%s' to '%s' in time %f", path_in.header.frame_id.c_str(), target_frame.c_str(),
              path_in.header.stamp.toSec());
    return {};
  }

  mrs_msgs::Path path_out = path_in;

  path_out.header.stamp    = tf.value().header.stamp;
  path_out.header.frame_id = transformer_->frame_to(tf.value());

  for (size_t i = 0; i < path_in.points.size(); i++) {

    mrs_msgs::ReferenceStamped waypoint;

    waypoint.header    = path_in.header;
    waypoint.reference = path_in.points.at(i);

    if (auto ret = transformer_->transform(waypoint, tf.value())) {

      path_out.points.at(i) = ret.value().reference;

    } else {
      return {};
    }
  }

  return path_out;
}

//}

/* overtime() //{ */

bool MrsTrajectoryGeneration::overtime(void) {

  auto start_time_total   = mrs_lib::get_mutexed(mutex_start_time_total_, start_time_total_);
  auto max_execution_time = mrs_lib::get_mutexed(mutex_max_execution_time_, max_execution_time_);

  double overtime = (ros::Time::now() - start_time_total).toSec();

  if (overtime > (OVERTIME_SAFETY_FACTOR * max_execution_time - OVERTIME_SAFETY_OFFSET)) {
    return true;
  }

  return false;
}

//}

/* timeLeft() //{ */

double MrsTrajectoryGeneration::timeLeft(void) {

  auto start_time_total   = mrs_lib::get_mutexed(mutex_start_time_total_, start_time_total_);
  auto max_execution_time = mrs_lib::get_mutexed(mutex_max_execution_time_, max_execution_time_);

  double current_execution_time = (ros::Time::now() - start_time_total).toSec();

  if (current_execution_time >= max_execution_time) {
    return 0;
  } else {
    return max_execution_time - current_execution_time;
  }
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackPath() //{ */

void MrsTrajectoryGeneration::callbackPath(const mrs_msgs::Path::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  /* preconditions //{ */

  if (!sh_constraints_.hasMsg()) {
    std::stringstream ss;
    ss << "missing constraints";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());
    return;
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    std::stringstream ss;
    ss << "missing control manager diagnostics";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());
    return;
  }

  if (!sh_uav_state_.hasMsg()) {
    std::stringstream ss;
    ss << "missing UAV state";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());
    return;
  }

  //}

  {
    std::scoped_lock lock(mutex_start_time_total_);

    start_time_total_ = ros::Time::now();
  }

  double path_time_offset = 0;

  if (msg->header.stamp != ros::Time(0)) {
    path_time_offset = (msg->header.stamp - ros::Time::now()).toSec();
  }

  if (path_time_offset > 1e-3) {

    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = std::min(FUTURIZATION_EXEC_TIME_FACTOR * path_time_offset, params_.max_execution_time);

    ROS_INFO("[TrajectoryGeneration]: setting the max execution time to %.3f s = %.1f * %.3f", max_execution_time_, FUTURIZATION_EXEC_TIME_FACTOR,
             path_time_offset);
  } else {

    std::scoped_lock lock(mutex_max_execution_time_, mutex_params_);

    max_execution_time_ = params_.max_execution_time;
  }

  ROS_INFO("[TrajectoryGeneration]: got path from message");

  ph_original_path_.publish(msg);

  if (msg->points.empty()) {
    std::stringstream ss;
    ss << "received an empty message";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());
    return;
  }

  auto transformed_path = transformPath(*msg, "");

  if (!transformed_path) {
    std::stringstream ss;
    ss << "could not transform the path to the current control frame";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());
    return;
  }

  fly_now_                              = transformed_path->fly_now;
  use_heading_                          = transformed_path->use_heading;
  frame_id_                             = transformed_path->header.frame_id;
  override_constraints_                 = transformed_path->override_constraints;
  loop_                                 = transformed_path->loop;
  override_max_velocity_horizontal_     = transformed_path->override_max_velocity_horizontal;
  override_max_velocity_vertical_       = transformed_path->override_max_velocity_vertical;
  override_max_acceleration_horizontal_ = transformed_path->override_max_acceleration_horizontal;
  override_max_acceleration_vertical_   = transformed_path->override_max_acceleration_vertical;
  override_max_jerk_horizontal_         = transformed_path->override_max_jerk_horizontal;
  override_max_jerk_vertical_           = transformed_path->override_max_jerk_horizontal;
  stop_at_waypoints_                    = transformed_path->stop_at_waypoints;

  auto params = mrs_lib::get_mutexed(mutex_params_, params_);

  if (transformed_path->max_execution_time > 0) {

    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = transformed_path->max_execution_time;

  } else {

    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = params.max_execution_time;
  }

  if (transformed_path->max_deviation_from_path > 0) {
    trajectory_max_segment_deviation_ = transformed_path->max_deviation_from_path;
  } else {
    trajectory_max_segment_deviation_ = params.max_deviation;
  }

  dont_prepend_initial_condition_ = transformed_path->dont_prepend_current_state;

  std::vector<Waypoint_t> waypoints;

  for (size_t i = 0; i < transformed_path->points.size(); i++) {

    double x       = transformed_path->points.at(i).position.x;
    double y       = transformed_path->points.at(i).position.y;
    double z       = transformed_path->points.at(i).position.z;
    double heading = transformed_path->points.at(i).heading;

    Waypoint_t wp;
    wp.coords  = Eigen::Vector4d(x, y, z, heading);
    wp.stop_at = stop_at_waypoints_;

    if (!checkNaN(wp)) {
      ROS_ERROR("[TrajectoryGeneration]: NaN detected in waypoint #%d", int(i));
      return;
    }

    waypoints.push_back(wp);
  }

  if (loop_) {
    waypoints.push_back(waypoints.at(0));
  }

  bool                          success = false;
  std::string                   message;
  mrs_msgs::TrajectoryReference trajectory;
  bool                          initial_condition = false;

  for (int i = 0; i < _n_attempts_; i++) {

    // the last iteration and the fallback sampling is enabled
    bool fallback_sampling = (_n_attempts_ > 1) && (i == (_n_attempts_ - 1)) && _fallback_sampling_enabled_;

    std::tie(success, message, trajectory, initial_condition) =
        optimize(waypoints, transformed_path->header, fallback_sampling, transformed_path->relax_heading);

    if (success) {
      break;
    } else {
      if (i < _n_attempts_) {
        ROS_WARN("[TrajectoryGeneration]: failed to calculate a feasible trajectory, trying again with different initial conditions!");
      } else {
        ROS_WARN("[TrajectoryGeneration]: failed to calculate a feasible trajectory");
      }
    }
  }

  double total_time = (ros::Time::now() - start_time_total_).toSec();

  auto max_execution_time = mrs_lib::get_mutexed(mutex_max_execution_time_, max_execution_time_);

  if (total_time > max_execution_time) {
    ROS_ERROR("[TrajectoryGeneration]: trajectory ready, took %.3f s in total (exceeding maxtime %.3f s by %.3f s)", total_time, max_execution_time,
              total_time - max_execution_time);
  } else {
    ROS_INFO("[TrajectoryGeneration]: trajectory ready, took %.3f s in total (out of %.3f)", total_time, max_execution_time);
  }

  trajectory.input_id = transformed_path->input_id;

  if (success) {

    bool published = trajectorySrv(trajectory);

    if (published) {

      ROS_INFO("[TrajectoryGeneration]: trajectory successfully published");

    } else {

      ROS_ERROR("[TrajectoryGeneration]: could not publish the trajectory");
    }

  } else {

    ROS_ERROR("[TrajectoryGeneration]: failed to calculate a feasible trajectory, no publishing a result");
  }
}

//}

/* callbackPathSrv() //{ */

bool MrsTrajectoryGeneration::callbackPathSrv(mrs_msgs::PathSrv::Request& req, mrs_msgs::PathSrv::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  /* preconditions //{ */

  if (!sh_constraints_.hasMsg()) {
    std::stringstream ss;
    ss << "missing constraints";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    std::stringstream ss;
    ss << "missing control manager diagnostics";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!sh_uav_state_.hasMsg()) {
    std::stringstream ss;
    ss << "missing UAV state";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  //}

  {
    std::scoped_lock lock(mutex_start_time_total_);

    start_time_total_ = ros::Time::now();
  }

  double path_time_offset = 0;

  if (req.path.header.stamp != ros::Time(0)) {
    path_time_offset = (req.path.header.stamp - ros::Time::now()).toSec();
  }

  if (path_time_offset > 1e-3) {

    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = std::min(FUTURIZATION_EXEC_TIME_FACTOR * path_time_offset, params_.max_execution_time);

    ROS_INFO("[TrajectoryGeneration]: setting the max execution time to %.3f s = %.1f * %.3f", max_execution_time_, FUTURIZATION_EXEC_TIME_FACTOR,
             path_time_offset);
  } else {

    std::scoped_lock lock(mutex_max_execution_time_, mutex_params_);

    max_execution_time_ = params_.max_execution_time;
  }

  ROS_INFO("[TrajectoryGeneration]: got path from service");

  ph_original_path_.publish(req.path);

  if (req.path.points.empty()) {
    std::stringstream ss;
    ss << "received an empty message";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  auto transformed_path = transformPath(req.path, "");

  if (!transformed_path) {
    std::stringstream ss;
    ss << "could not transform the path to the current control frame";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  fly_now_                              = transformed_path->fly_now;
  use_heading_                          = transformed_path->use_heading;
  frame_id_                             = transformed_path->header.frame_id;
  override_constraints_                 = transformed_path->override_constraints;
  loop_                                 = transformed_path->loop;
  override_max_velocity_horizontal_     = transformed_path->override_max_velocity_horizontal;
  override_max_velocity_vertical_       = transformed_path->override_max_velocity_vertical;
  override_max_acceleration_horizontal_ = transformed_path->override_max_acceleration_horizontal;
  override_max_acceleration_vertical_   = transformed_path->override_max_acceleration_vertical;
  override_max_jerk_horizontal_         = transformed_path->override_max_jerk_horizontal;
  override_max_jerk_vertical_           = transformed_path->override_max_jerk_horizontal;
  stop_at_waypoints_                    = transformed_path->stop_at_waypoints;

  auto params = mrs_lib::get_mutexed(mutex_params_, params_);

  if (transformed_path->max_execution_time > 0) {

    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = transformed_path->max_execution_time;

  } else {

    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = params.max_execution_time;
  }

  if (transformed_path->max_deviation_from_path > 0) {
    trajectory_max_segment_deviation_ = transformed_path->max_deviation_from_path;
  } else {
    trajectory_max_segment_deviation_ = params.max_deviation;
  }

  dont_prepend_initial_condition_ = transformed_path->dont_prepend_current_state;

  std::vector<Waypoint_t> waypoints;

  for (size_t i = 0; i < req.path.points.size(); i++) {

    double x       = transformed_path->points.at(i).position.x;
    double y       = transformed_path->points.at(i).position.y;
    double z       = transformed_path->points.at(i).position.z;
    double heading = transformed_path->points.at(i).heading;

    Waypoint_t wp;
    wp.coords  = Eigen::Vector4d(x, y, z, heading);
    wp.stop_at = stop_at_waypoints_;

    if (!checkNaN(wp)) {
      ROS_ERROR("[TrajectoryGeneration]: NaN detected in waypoint #%d", int(i));
      res.success = false;
      res.message = "invalid path";
      return true;
    }

    waypoints.push_back(wp);
  }

  if (loop_) {
    waypoints.push_back(waypoints.at(0));
  }

  bool                          success = false;
  std::string                   message;
  mrs_msgs::TrajectoryReference trajectory;
  bool                          initial_condition = false;

  for (int i = 0; i < _n_attempts_; i++) {

    // the last iteration and the fallback sampling is enabled
    bool fallback_sampling = (_n_attempts_ > 1) && (i == (_n_attempts_ - 1)) && _fallback_sampling_enabled_;

    std::tie(success, message, trajectory, initial_condition) =
        optimize(waypoints, transformed_path->header, fallback_sampling, transformed_path->relax_heading);

    if (success) {
      break;
    } else {
      if (i < _n_attempts_) {
        ROS_WARN("[TrajectoryGeneration]: failed to calculate a feasible trajectory, trying again with different initial conditions!");
      } else {
        ROS_WARN("[TrajectoryGeneration]: failed to calculate a feasible trajectory");
      }
    }
  }

  double total_time = (ros::Time::now() - start_time_total_).toSec();

  auto max_execution_time = mrs_lib::get_mutexed(mutex_max_execution_time_, max_execution_time_);

  if (total_time > max_execution_time) {
    ROS_ERROR("[TrajectoryGeneration]: trajectory ready, took %.3f s in total (exceeding maxtime %.3f s by %.3f s)", total_time, max_execution_time,
              total_time - max_execution_time);
  } else {
    ROS_INFO("[TrajectoryGeneration]: trajectory ready, took %.3f s in total (out of %.3f)", total_time, max_execution_time);
  }

  trajectory.input_id = transformed_path->input_id;

  if (success) {

    bool published = trajectorySrv(trajectory);

    if (published) {

      res.success = success;
      res.message = message;

    } else {

      std::stringstream ss;
      ss << "could not publish the trajectory";

      res.success = false;
      res.message = ss.str();

      ROS_ERROR_STREAM("[TrajectoryGeneration]: " << ss.str());
    }

  } else {

    ROS_ERROR("[TrajectoryGeneration]: failed to calculate a feasible trajectory, not publishing a result");

    res.success = success;
    res.message = message;
  }

  return true;
}

//}

/* callbackGetPathSrv() //{ */

bool MrsTrajectoryGeneration::callbackGetPathSrv(mrs_msgs::GetPathSrv::Request& req, mrs_msgs::GetPathSrv::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  /* preconditions //{ */

  if (!sh_constraints_.hasMsg()) {
    std::stringstream ss;
    ss << "missing constraints";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    std::stringstream ss;
    ss << "missing control manager diagnostics";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  if (!sh_uav_state_.hasMsg()) {
    std::stringstream ss;
    ss << "missing UAV state";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  //}

  {
    std::scoped_lock lock(mutex_start_time_total_);

    start_time_total_ = ros::Time::now();
  }

  double path_time_offset = 0;

  if (req.path.header.stamp != ros::Time(0)) {
    path_time_offset = (req.path.header.stamp - ros::Time::now()).toSec();
  }

  if (path_time_offset > 1e-3) {

    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = FUTURIZATION_EXEC_TIME_FACTOR * path_time_offset;

    ROS_INFO("[TrajectoryGeneration]: setting the max execution time to %.3f s = %.1f * %.3f", max_execution_time_, FUTURIZATION_EXEC_TIME_FACTOR,
             path_time_offset);
  } else {

    std::scoped_lock lock(mutex_max_execution_time_, mutex_params_);

    max_execution_time_ = params_.max_execution_time;
  }

  ROS_INFO("[TrajectoryGeneration]: got path from service");

  ph_original_path_.publish(req.path);

  if (req.path.points.empty()) {
    std::stringstream ss;
    ss << "received an empty message";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  // if the path frame_id is latlon_origin, set UTM zone by calling setLatLon for mrs_lib::transformer using the first point in the trajectory
  if (req.path.header.frame_id == "latlon_origin") {
    transformer_->setLatLon(req.path.points.front().position.x, req.path.points.front().position.y);
  }

  auto transformed_path = transformPath(req.path, "");

  if (!transformed_path) {
    std::stringstream ss;
    ss << "could not transform the path to the current control frame";
    ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());

    res.message = ss.str();
    res.success = false;
    return true;
  }

  fly_now_                              = transformed_path->fly_now;
  use_heading_                          = transformed_path->use_heading;
  frame_id_                             = transformed_path->header.frame_id;
  override_constraints_                 = transformed_path->override_constraints;
  loop_                                 = transformed_path->loop;
  override_max_velocity_horizontal_     = transformed_path->override_max_velocity_horizontal;
  override_max_velocity_vertical_       = transformed_path->override_max_velocity_vertical;
  override_max_acceleration_horizontal_ = transformed_path->override_max_acceleration_horizontal;
  override_max_acceleration_vertical_   = transformed_path->override_max_acceleration_vertical;
  override_max_jerk_horizontal_         = transformed_path->override_max_jerk_horizontal;
  override_max_jerk_vertical_           = transformed_path->override_max_jerk_horizontal;
  stop_at_waypoints_                    = transformed_path->stop_at_waypoints;

  auto params = mrs_lib::get_mutexed(mutex_params_, params_);

  if (transformed_path->max_execution_time > 0) {

    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = transformed_path->max_execution_time;

  } else {

    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = params.max_execution_time;
  }

  if (transformed_path->max_deviation_from_path > 0) {
    trajectory_max_segment_deviation_ = transformed_path->max_deviation_from_path;
  } else {
    trajectory_max_segment_deviation_ = params.max_deviation;
  }

  dont_prepend_initial_condition_ = transformed_path->dont_prepend_current_state;

  std::vector<Waypoint_t> waypoints;

  for (size_t i = 0; i < transformed_path->points.size(); i++) {

    double x       = transformed_path->points.at(i).position.x;
    double y       = transformed_path->points.at(i).position.y;
    double z       = transformed_path->points.at(i).position.z;
    double heading = transformed_path->points.at(i).heading;

    Waypoint_t wp;
    wp.coords  = Eigen::Vector4d(x, y, z, heading);
    wp.stop_at = stop_at_waypoints_;

    if (!checkNaN(wp)) {
      ROS_ERROR("[TrajectoryGeneration]: NaN detected in waypoint #%d", int(i));
      res.success = false;
      res.message = "invalid path";
      return true;
    }

    waypoints.push_back(wp);
  }

  if (loop_) {
    waypoints.push_back(waypoints.at(0));
  }

  bool                          success = false;
  std::string                   message;
  mrs_msgs::TrajectoryReference trajectory;
  bool                          initial_condition = false;

  for (int i = 0; i < _n_attempts_; i++) {

    // the last iteration and the fallback sampling is enabled
    bool fallback_sampling = (_n_attempts_ > 1) && (i == (_n_attempts_ - 1)) && _fallback_sampling_enabled_;

    std::tie(success, message, trajectory, initial_condition) =
        optimize(waypoints, transformed_path->header, fallback_sampling, transformed_path->relax_heading);

    if (success) {
      break;
    } else {
      if (i < _n_attempts_) {
        ROS_WARN("[TrajectoryGeneration]: failed to calculate a feasible trajectory, trying again with different initial conditions!");
      } else {
        ROS_WARN("[TrajectoryGeneration]: failed to calculate a feasible trajectory");
      }
    }
  }

  double total_time = (ros::Time::now() - start_time_total_).toSec();

  auto max_execution_time = mrs_lib::get_mutexed(mutex_max_execution_time_, max_execution_time_);

  if (total_time > max_execution_time) {
    ROS_ERROR("[TrajectoryGeneration]: trajectory ready, took %.3f s in total (exceeding maxtime %.3f s by %.3f s)", total_time, max_execution_time,
              total_time - max_execution_time);
  } else {
    ROS_INFO("[TrajectoryGeneration]: trajectory ready, took %.3f s in total (out of %.3f)", total_time, max_execution_time);
  }

  // locate the waypoint idxs
  auto waypoint_idxs = getWaypointInTrajectoryIdxs(trajectory, waypoints);

  if (success) {

    std::optional<geometry_msgs::TransformStamped> tf_traj_state = transformer_->getTransform("", req.path.header.frame_id, ros::Time::now());

    std::stringstream ss;

    if (!tf_traj_state) {
      ss << "could not create TF transformer for the trajectory to the requested frame: \"" << req.path.header.frame_id << "\"";
      ROS_WARN_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());
      res.success = false;
      res.message = ss.str();
      return true;
    }

    trajectory.header.frame_id = transformer_->frame_to(*tf_traj_state);

    for (unsigned long i = 0; i < trajectory.points.size(); i++) {

      mrs_msgs::ReferenceStamped trajectory_point;
      trajectory_point.header    = trajectory.header;
      trajectory_point.reference = trajectory.points.at(i);

      auto ret = transformer_->transform(trajectory_point, *tf_traj_state);

      if (!ret) {

        ss << "trajectory cannot be transformed to the requested frame: \"" << req.path.header.frame_id << "\"";
        ROS_WARN_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());
        res.success = false;
        res.message = ss.str();
        return true;

      } else {

        // transform the points in the trajectory to the current frame
        trajectory.points.at(i) = ret.value().reference;
      }
    }

    for (size_t it = 0; it < waypoint_idxs.size(); it++) {
      res.waypoint_trajectory_idxs.push_back(waypoint_idxs[it]);
    }

    res.trajectory = trajectory;
    res.success    = success;
    res.message    = message;

  } else {

    ROS_ERROR("[TrajectoryGeneration]: failed to calculate a feasible trajectory");

    res.success = success;
    res.message = message;
  }

  return true;
}

//}

/* callbackUavState() //{ */

void MrsTrajectoryGeneration::callbackUavState(const mrs_msgs::UavState::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[TrajectoryGeneration]: getting uav state");

  transformer_->setDefaultFrame(msg->header.frame_id);
}

//}

/* //{ callbackDrs() */

void MrsTrajectoryGeneration::callbackDrs(mrs_uav_trajectory_generation::drsConfig& params, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_params_, params, params_);

  {
    std::scoped_lock lock(mutex_max_execution_time_);

    max_execution_time_ = params.max_execution_time;
  }

  ROS_INFO("[TrajectoryGeneration]: DRS updated");
}

//}

}  // namespace mrs_uav_trajectory_generation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_trajectory_generation::MrsTrajectoryGeneration, nodelet::Nodelet);
