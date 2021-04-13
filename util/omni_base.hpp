#include "base.hpp"

namespace hebi {
namespace experimental {
namespace mobile {

class OmniBase : public MobileBase {
public:
  // Parameters for creating a base
  struct Params: public MobileBase::Params {
    Params() : wheel_radius(0.0762), base_radius(0.220) {}
    double wheel_radius; // m
    double base_radius; // m (center of omni to origin of base) 
  };

  OmniBase(Params p) : MobileBase(p),
                       wheel_radius_(p.wheel_radius),
                       base_radius_(p.base_radius)
  {}

  virtual Eigen::VectorXd SE2ToWheelVel(Pose pos, Vel vel) const override;
  virtual Vel getMaxVelocity() const override;

protected:
  // Updates local velocity based on wheel change in position since last time
  virtual void updateOdometry(const Eigen::VectorXd& wheel_vel, double dt) override;

private:

  Eigen::Matrix3d createJacobian() {
    double a = sqrt(3)/(2 * wheel_radius_);
    double b = 1.0 / wheel_radius_;
    double c = -base_radius_ / wheel_radius_;
    Eigen::Matrix3d j;
    j << -a, -b / 2.0, c,
         a, -b / 2.0, c,
         0.0, b, c;
    return j;
  }

  Eigen::Matrix3d createJacobianInv() {
    Eigen::Matrix3d j_inv;
    double a = wheel_radius_ / sqrt(3);
    double b = wheel_radius_ / 3.0;
    double c = -b / base_radius_;
    j_inv << -a, a, 0,
             -b, -b, 2.0 * b,
             c, c, c;
    return j_inv;
  }

  // TODO: think about limitations on (x,y) vs. (x,y,theta)
  // trajectories
  virtual std::optional<Goal> buildTrajectory(const CartesianGoal& g) override;

  // Helper function to create unconstrained points along a motion.
  static MatrixXd nan(size_t num_joints, size_t num_waypoints) {
    double nan = std::numeric_limits<double>::quiet_NaN();
    MatrixXd matrix(num_joints, num_waypoints);
    matrix.setConstant(nan);
    return matrix;
  }

  // Helper function to create unconstrained points along a motion, with nan at the right side.
  static MatrixXd nanWithZeroRight(size_t num_joints, size_t num_waypoints) {
    double nan = std::numeric_limits<double>::quiet_NaN();
    MatrixXd matrix(num_joints, num_waypoints);
    matrix.setConstant(nan);
    matrix.rightCols<1>().setZero();
    return matrix;
  }

   /* Declare main kinematic variables */
  double wheel_radius_; // m
  double base_radius_; // m (center of omni to origin of base) 

  const Eigen::Matrix3d jacobian_ = createJacobian();

  // Wheel velocities to local (x,y,theta)
  const Eigen::Matrix3d jacobian_inv_ = createJacobianInv();
};

}
}
}