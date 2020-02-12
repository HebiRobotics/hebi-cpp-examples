#pragma once

#include <Eigen/Dense>

namespace hebi {
namespace experimental {
namespace arm {

// A class that specifies a goal position of one or more waypoint and/or
// auxilary states.
// Static create methods are provided for various cases; in the case that
// velocities or accelerations are omitted, the default behavior is to leave
// these unconstrained except for a final "0" state.  For aux states, this is
// left as unchanged. For times, this is left to the Arm object to fill in a
// heuristic.
class Goal {

public:
  //////////////////////////////////////////////////////////////////////////////
  // Single waypoint static create functions
  //////////////////////////////////////////////////////////////////////////////

  // Single waypoint, default vel/accel, no time
  static Goal createFromPosition(const Eigen::VectorXd& positions) {
    return Goal(Eigen::VectorXd(0),
                toMatrix(positions),
                nanWithZeroRight(positions.size(), 1),
                nanWithZeroRight(positions.size(), 1),
                Eigen::MatrixXd(0, 0));
  }

  // Single waypoint, default vel/accel
  static Goal createFromPosition(double time, const Eigen::VectorXd& positions) {
    return Goal(toVector(time),
                toMatrix(positions),
                nanWithZeroRight(positions.size(), 1),
                nanWithZeroRight(positions.size(), 1),
                Eigen::MatrixXd(0, 0));
  }

  // Single waypoint, no time
  static Goal createFromWaypoint(const Eigen::VectorXd& positions,
                                 const Eigen::VectorXd& velocities,
                                 const Eigen::VectorXd& accelerations) {
    return Goal(Eigen::VectorXd(0),
                toMatrix(positions),
                toMatrix(velocities),
                toMatrix(accelerations),
                Eigen::MatrixXd(0, 0));
  }

  // Single waypoint
  static Goal createFromWaypoint(double time,
                                 const Eigen::VectorXd& positions,
                                 const Eigen::VectorXd& velocities,
                                 const Eigen::VectorXd& accelerations) {
    return Goal(toVector(time),
                toMatrix(positions),
                toMatrix(velocities),
                toMatrix(accelerations),
                Eigen::MatrixXd(0, 0));
  }

  // Single waypoints + aux state, no time
  static Goal createFromWaypointWithAux(const Eigen::VectorXd& positions,
                                        const Eigen::VectorXd& velocities,
                                        const Eigen::VectorXd& accelerations,
                                        const Eigen::VectorXd& aux) {
    return Goal(Eigen::VectorXd(0),
                toMatrix(positions),
                toMatrix(velocities),
                toMatrix(accelerations),
                toMatrix(aux));
  }

  // Single waypoints + aux state
  static Goal createFromWaypointWithAux(double time,
                                        const Eigen::VectorXd& positions,
                                        const Eigen::VectorXd& velocities,
                                        const Eigen::VectorXd& accelerations,
                                        const Eigen::VectorXd& aux) {
    return Goal(toVector(time),
                toMatrix(positions),
                toMatrix(velocities),
                toMatrix(accelerations),
                toMatrix(aux));
  }

  //////////////////////////////////////////////////////////////////////////////
  // Multiple waypoint static create functions
  //////////////////////////////////////////////////////////////////////////////

  // Multiple waypoints, default vel/accel, no time
  static Goal createFromPositions(const Eigen::MatrixXd& positions) {
    return Goal(Eigen::VectorXd(0),
                positions,
                nanWithZeroRight(positions.rows(), positions.cols()),
                nanWithZeroRight(positions.rows(), positions.cols()),
                Eigen::MatrixXd(0, 0));
  }

  // Multiple waypoints, default vel/accel
  static Goal createFromPositions(const Eigen::VectorXd& times, 
                                  const Eigen::MatrixXd& positions) {
    return Goal(times,
                positions,
                nanWithZeroRight(positions.rows(), positions.cols()),
                nanWithZeroRight(positions.rows(), positions.cols()),
                Eigen::MatrixXd(0, 0));
  }

  // Multiple waypoints, no time
  static Goal createFromWaypoints(const Eigen::MatrixXd& positions,
                                  const Eigen::MatrixXd& velocities,
                                  const Eigen::MatrixXd& accelerations) {
    return Goal(Eigen::VectorXd(0),
                positions,
                velocities,
                accelerations,
                Eigen::MatrixXd(0, 0));
  }

  // Multiple waypoints
  static Goal createFromWaypoints(const Eigen::VectorXd& times,
                                  const Eigen::MatrixXd& positions,
                                  const Eigen::MatrixXd& velocities,
                                  const Eigen::MatrixXd& accelerations) {
    return Goal(times,
                positions,
                velocities,
                accelerations,
                Eigen::MatrixXd(0, 0));
  }

  // Multiple waypoints + aux state, no time
  static Goal createFromWaypointsWithAux(const Eigen::MatrixXd& positions,
                                         const Eigen::MatrixXd& velocities,
                                         const Eigen::MatrixXd& accelerations,
                                         const Eigen::MatrixXd& aux) {
    return Goal(Eigen::VectorXd(0),
                positions,
                velocities,
                accelerations,
                aux);
  }

  // Multiple waypoints + aux state
  static Goal createFromWaypointsWithAux(const Eigen::VectorXd& times,
                                         const Eigen::MatrixXd& positions,
                                         const Eigen::MatrixXd& velocities,
                                         const Eigen::MatrixXd& accelerations,
                                         const Eigen::MatrixXd& aux) {
    return Goal(times,
                positions,
                velocities,
                accelerations,
                aux);
  }

  const Eigen::VectorXd& times() const { return times_; }
  const Eigen::MatrixXd& positions() const { return positions_; }
  const Eigen::MatrixXd& velocities() const { return velocities_; }
  const Eigen::MatrixXd& accelerations() const { return accelerations_; }
  const Eigen::MatrixXd& aux() const { return aux_; }

private:

  Goal(const Eigen::VectorXd& times,
       const Eigen::MatrixXd& positions,
       const Eigen::MatrixXd& velocities,
       const Eigen::MatrixXd& accelerations,
       const Eigen::MatrixXd& aux)
    : times_(times),
      positions_(positions),
      velocities_(velocities),
      accelerations_(accelerations),
      aux_(aux) {}

  // Helper function to create unconstrained points along a motion, with nan at the right side.
  static Eigen::MatrixXd nanWithZeroRight(size_t num_joints, size_t num_waypoints) {
    double nan = std::numeric_limits<double>::quiet_NaN();
    Eigen::MatrixXd matrix(num_joints, num_waypoints);
    matrix.setConstant(nan);
    matrix.rightCols<1>().setZero();
    return matrix;
  }

  static Eigen::VectorXd toVector(double scalar) {
    Eigen::VectorXd vector(1);
    vector[0] = scalar;
    return vector;
  }

  static Eigen::MatrixXd toMatrix(const Eigen::VectorXd& vector) {
    Eigen::MatrixXd matrix(vector.size(), 1);
    matrix.col(0) = vector;
    return matrix;
  }

  const Eigen::VectorXd times_{0};
  const Eigen::MatrixXd positions_{0, 0};
  const Eigen::MatrixXd velocities_{0, 0};
  const Eigen::MatrixXd accelerations_{0, 0};
  const Eigen::MatrixXd aux_{0, 0};
};

} // namespace arm
} // namespace experimental
} // namespace hebi
