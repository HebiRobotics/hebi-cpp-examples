#include <eth_trajectory_generation/impl/polynomial_optimization_nonlinear_impl.h>
#include <eth_trajectory_generation/trajectory.h>

typedef struct
{
  Eigen::Vector4d coords;
  bool stop_at;
} Waypoint_t;

namespace mrs_uav_trajectory_generation
{

  bool optimize(const std::vector<Waypoint_t> &waypoints_in, const std::optional<mrs_msgs::TrackerCommand> &initial_state)
  {
    // empty path is invalid
    if (waypoints_in.size() == 0)
    {
      std::stringstream ss;
      ss << "the path is empty (before postprocessing)";
      ROS_ERROR_STREAM("[TrajectoryGeneration]: " << ss.str());
      return false;
    }

    eth_mav_msgs::EigenTrajectoryPoint::Vector trajectory;

    ROS_DEBUG("[TrajectoryGeneration]: optimize() started");

    ros::Time optimize_time_start = ros::Time::now();

    // optimizer

    eth_trajectory_generation::NonlinearOptimizationParameters parameters;

    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 100.0;
    parameters.use_soft_constraints = true;
    parameters.soft_constraint_weight = 1.5;
    parameters.time_alloc_method = static_cast<eth_trajectory_generation::NonlinearOptimizationParameters::TimeAllocMethod>(2); // Mellinger
    if (params.time_allocation == 2)
    {
      parameters.algorithm = nlopt::LD_LBFGS;
    }
    parameters.initial_stepsize_rel = 0.1;
    parameters.inequality_constraint_tolerance = 0.1;
    parameters.equality_constraint_tolerance = 1.0e-3;
    parameters.max_iterations = 10;
    parameters.max_time = 1.0;

    eth_trajectory_generation::Vertex::Vector vertices;
    const int dimension = 4;

    int derivative_to_optimize = eth_trajectory_generation::derivative_order::JERK;

    // | --------------- add constraints to vertices -------------- |

    for (size_t i = 0; i < waypoints_in.size(); i++)
    {
      double x = waypoints_in.at(i).coords(0);
      double y = waypoints_in.at(i).coords(1);
      double z = waypoints_in.at(i).coords(2);
      double heading = waypoints_in.at(0).coords(3);

      eth_trajectory_generation::Vertex vertex(dimension);

      if (i == 0)
      {

        vertex.makeStartOrEnd(Eigen::Vector4d(x, y, z, heading), derivative_to_optimize);

        vertex.addConstraint(eth_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));

        if (initial_state)
        {

          vertex.addConstraint(eth_trajectory_generation::derivative_order::VELOCITY,
                               Eigen::Vector4d(initial_state->velocity.x, initial_state->velocity.y, initial_state->velocity.z, initial_state->heading_rate));

          vertex.addConstraint(
              eth_trajectory_generation::derivative_order::ACCELERATION,
              Eigen::Vector4d(initial_state->acceleration.x, initial_state->acceleration.y, initial_state->acceleration.z, initial_state->heading_acceleration));

          vertex.addConstraint(eth_trajectory_generation::derivative_order::JERK,
                               Eigen::Vector4d(initial_state->jerk.x, initial_state->jerk.y, initial_state->jerk.z, initial_state->heading_jerk));
        }
      }
      else if (i == (waypoints_in.size() - 1))
      { // the last point

        vertex.makeStartOrEnd(Eigen::Vector4d(x, y, z, heading), derivative_to_optimize);

        vertex.addConstraint(eth_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));
      }
      else
      { // mid points

        vertex.addConstraint(eth_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x, y, z, heading));

        if (waypoints_in.at(i).stop_at)
        {
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
    double vertical_speed_lim = std::min(constraints->vertical_ascending_speed, constraints->vertical_descending_speed);
    double vertical_acceleration_lim = std::min(constraints->vertical_ascending_acceleration, constraints->vertical_descending_acceleration);

    v_max_horizontal = constraints->horizontal_speed;
    a_max_horizontal = constraints->horizontal_acceleration;

    v_max_vertical = vertical_speed_lim;
    a_max_vertical = vertical_acceleration_lim;

    j_max_horizontal = constraints->horizontal_jerk;
    j_max_vertical = std::min(constraints->vertical_ascending_jerk, constraints->vertical_descending_jerk);

    double v_max_heading, a_max_heading, j_max_heading;
    v_max_heading = constraints->heading_speed;
    a_max_heading = constraints->heading_acceleration;
    j_max_heading = constraints->heading_jerk;

    ROS_DEBUG("[TrajectoryGeneration]: using constraints:");
    ROS_DEBUG("[TrajectoryGeneration]: horizontal: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_horizontal, a_max_horizontal, j_max_horizontal);
    ROS_DEBUG("[TrajectoryGeneration]: vertical: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_vertical, a_max_vertical, j_max_vertical);
    ROS_DEBUG("[TrajectoryGeneration]: heading: vel = %.2f, acc = %.2f, jerk = %.2f", v_max_heading, a_max_heading, j_max_heading);

    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, v_max_horizontal, v_max_vertical, a_max_horizontal, a_max_vertical, j_max_horizontal, j_max_vertical,
                                         v_max_heading, a_max_heading);

    double initial_total_time = 0;
    for (int i = 0; i < int(segment_times.size()); i++)
    {
      initial_total_time += segment_times.at(i);
    }

    ROS_DEBUG("[TrajectoryGeneration]: initial total time (Euclidean): %.2f", initial_total_time);

    // | --------- create an optimizer object and solve it -------- |

    const int N = 6;
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

    std::string result_str;

    switch (opt.getOptimizationInfo().stopping_reason)
    {
    case nlopt::FAILURE:
    {
      result_str = "generic failure";
      break;
    }
    case nlopt::INVALID_ARGS:
    {
      result_str = "invalid args";
      break;
    }
    case nlopt::OUT_OF_MEMORY:
    {
      result_str = "out of memory";
      break;
    }
    case nlopt::ROUNDOFF_LIMITED:
    {
      result_str = "roundoff limited";
      break;
    }
    case nlopt::FORCED_STOP:
    {
      result_str = "forced stop";
      break;
    }
    case nlopt::STOPVAL_REACHED:
    {
      result_str = "stopval reached";
      break;
    }
    case nlopt::FTOL_REACHED:
    {
      result_str = "ftol reached";
      break;
    }
    case nlopt::XTOL_REACHED:
    {
      result_str = "xtol reached";
      break;
    }
    case nlopt::MAXEVAL_REACHED:
    {
      result_str = "maxeval reached";
      break;
    }
    case nlopt::MAXTIME_REACHED:
    {
      result_str = "maxtime reached";
      break;
    }
    default:
    {
      result_str = "UNKNOWN FAILURE CODE";
      break;
    }
    }

    if (opt.getOptimizationInfo().stopping_reason >= 1 && opt.getOptimizationInfo().stopping_reason != 6)
    {
      ROS_DEBUG("[TrajectoryGeneration]: optimization finished successfully with code %d, '%s'", opt.getOptimizationInfo().stopping_reason, result_str.c_str());
    }
    else if (opt.getOptimizationInfo().stopping_reason == -1)
    {
      ROS_DEBUG("[TrajectoryGeneration]: optimization finished with a generic error code %d, '%s'", opt.getOptimizationInfo().stopping_reason,
                result_str.c_str());
    }
    else
    {
      ROS_WARN("[TrajectoryGeneration]: optimization failed with code %d, '%s', took %.3f s", opt.getOptimizationInfo().stopping_reason, result_str.c_str(),
               (ros::Time::now() - optimize_time_start).toSec());
      return false;
    }

    // | --------------- create the trajectory class -------------- |
    eth_trajectory_generation::Trajectory trajectory_obj;
    opt.getTrajectory(&trajectory_obj);

    // Convert to EigenTrajectoryPoint::Vector
    // Note: You would need to implement a conversion method here
    // trajectory = convertToEigenTrajectoryPoints(trajectory_obj);

    ROS_INFO("[TrajectoryGeneration]: total trajectory time: %.2fs ", trajectory.back().time_from_start.toSec());

    ROS_DEBUG("[TrajectoryGeneration]: trajectory generated, took %.3f s", (ros::Time::now() - optimize_time_start).toSec());

    return true;
  }

} // namespace mrs_uav_trajectory_generation
