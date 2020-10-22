#include "step.hpp"
#include "leg.hpp"

namespace hebi {

Step::Step(double start_time, const Leg& leg)
  : leg_(leg), start_time_(start_time)
{
  for (int i = 0; i < num_phase_pts_; ++i)
    time_.push_back(phase_[i] * period_);

//  update(start_time, leg); // NOTE: I probably don't actually need to call this here, because it gets called from main before accessing the legs.  TODO: try removing?
  computeKeyframes();
}

void Step::computeKeyframes()
{
  // make space for 3 keyframes
  keyframes_.emplace_back();
  keyframes_.emplace_back();
  keyframes_.emplace_back();

  lift_off_vel_ = leg_.getStanceVelXYZ();

  // Liftoff
  keyframes_[0].p = leg_.getCmdStanceXYZ();
  keyframes_[0].v = lift_off_vel_;
  keyframes_[0].a.setZero();

  // Set touch down point to overshoot the stance error; only do this at the beginning to prevent
  // large changes in foot touch down location and corresponding weird trajectories.  TODO: fix to
  // allow small incremental updates to touch down location!
  // Touchdown
  Eigen::Vector3d touchdown_offset = overshoot_ * period_ * lift_off_vel_;
  keyframes_[2].p = leg_.getHomeStanceXYZ() - touchdown_offset;
  keyframes_[2].v = leg_.getStanceVelXYZ();
  keyframes_[2].a.setZero();

  // Midpoint
  keyframes_[1].p = (keyframes_[0].p + keyframes_[2].p) / 2.0;
  keyframes_[1].p(2) += height_;
  keyframes_[1].v.setConstant(std::numeric_limits<double>::quiet_NaN());
  keyframes_[1].a.setConstant(std::numeric_limits<double>::quiet_NaN());
}

void Step::recomputeKeyframes()
{
  // Midpoint
  keyframes_[1].p = (keyframes_[0].p + keyframes_[2].p) / 2.0;
  keyframes_[1].p(2) += height_;
  keyframes_[1].v.setConstant(std::numeric_limits<double>::quiet_NaN());
  keyframes_[1].a.setConstant(std::numeric_limits<double>::quiet_NaN());
}

// Note: returns 'true' if complete
bool Step::update(double t)
{
  double elapsed_time = t - start_time_;

  // We are done with this step!
  if (elapsed_time > period_)
    return true;
  // Close enough to the end; don't replan
  else if ((period_ - elapsed_time) < ignore_waypoint_threshold_)
    return false;

  // Replan based on current waypoints; create new trajectory objects
  // TODO: make this all more modular! (put waypoints in a vector so we can refer to them more easily here?)
  int num_joints = Leg::getNumJoints();
  int max_num_waypoints = time_.size(); // max number of waypoints
  const robot_model::RobotModel& kin = leg_.getKinematics();
 
  VectorXd leg_times(max_num_waypoints); 
  MatrixXd leg_waypoints(num_joints, max_num_waypoints);
  MatrixXd leg_waypoint_vels(num_joints, max_num_waypoints);
  MatrixXd leg_waypoint_accels(num_joints, max_num_waypoints);
  MatrixXd jacobian_ee;
  VectorXd ik_output;

  int next_pt = 0;
  if (trajectory_) {
    //std::cout << "Adding current position to trajectory" << std::endl;
    Eigen::VectorXd p(num_joints), v(num_joints), a(num_joints);
    trajectory_->getState(elapsed_time, &p, &v, &a);

    // TODO: maybe use current position and velocity instead of trajectory ones?
    leg_waypoints.col(next_pt) = p;
    leg_waypoint_vels.col(next_pt) = v;
    leg_waypoint_accels.col(next_pt) = a;

    leg_times[next_pt] = elapsed_time;
    next_pt++;
  }

  //recomputeKeyframes();

  for (int idx = 0; idx < keyframes_.size(); ++idx)
  {
    auto keyframe_deadline = time_[idx] - ignore_waypoint_threshold_;
    // handle liftoff keyframe
    if(keyframe_deadline < 0.0)
      keyframe_deadline = 0.0;

    if (elapsed_time > keyframe_deadline)
      continue;

    auto keyframe = keyframes_[idx];

    kin.solveIK(
      leg_.getSeedAngles(),
      ik_output,
      robot_model::EndEffectorPositionObjective(keyframe.p));
    if (ik_output.size() == 0)
      assert(false);

    // J(1:3, :) \ lift_off_vel;
    kin.getJEndEffector(ik_output, jacobian_ee);
    MatrixXd jacobian_part = jacobian_ee.topLeftCorner(3,jacobian_ee.cols());

    leg_waypoints.col(next_pt) = ik_output;

    // is there a better way to do this?
    if (keyframe.v.array().isNaN().any()) {
      leg_waypoint_vels.col(next_pt).setConstant(std::numeric_limits<double>::quiet_NaN());
    } else if (keyframe.v.array().isZero()) {
      leg_waypoint_vels.col(next_pt).setConstant(0);
   } else {
      leg_waypoint_vels.col(next_pt) = jacobian_part.colPivHouseholderQr().solve(keyframe.v).eval().cast<double>();
    }

    if (keyframe.a.array().isNaN().any()) {
      leg_waypoint_accels.col(next_pt).setConstant(std::numeric_limits<double>::quiet_NaN());
    } else if (keyframe.a.array().isZero()) {
      leg_waypoint_accels.col(next_pt).setConstant(0);
    } else {
      leg_waypoint_accels.col(next_pt) = jacobian_part.colPivHouseholderQr().solve(keyframe.a).eval().cast<double>();
    }
    
    leg_times[next_pt] = time_[idx];
    next_pt++;
  }

  int num_pts = next_pt;
  // TODO: don't resize, but just be smarter about passing in partial vectors
  // or creating the right size in the first place!
  leg_waypoint_vels.conservativeResize(num_joints, num_pts);
  leg_waypoint_accels.conservativeResize(num_joints, num_pts);

  trajectory_ = trajectory::Trajectory::createUnconstrainedQp(
    leg_times.head(num_pts),
    leg_waypoints.topLeftCorner(num_joints, num_pts),
    &leg_waypoint_vels,
    &leg_waypoint_accels);

  assert(trajectory_);
  return false; // Not done with the step
}
  
void Step::computeState(double t, Eigen::VectorXd& angles, Eigen::VectorXd& vels, Eigen::VectorXd& accels) const
{
  assert(trajectory_);
  if (!trajectory_)
    return;
  int num_joints = Leg::getNumJoints();
  Eigen::VectorXd p(num_joints), v(num_joints), a(num_joints);
  bool success = trajectory_->getState(t - start_time_, &angles, &v, &a);
  assert(success);
  if (success)
    vels = v;
//    vels = v.cast<float>();
}

const Eigen::Vector3d& Step::getTouchDown() const
{
  return keyframes_.back().p;
}

} // namespace hebi
