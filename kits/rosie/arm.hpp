#include "robot_model.hpp"
#include "trajectory.hpp"

using namespace hebi;

class GripperArm{
  public:
  std::unique_ptr<robot_model::RobotModel> model;
  std::vector<std::string> arm_module_names_;
  std::vector<std::string> gripper_module_name_;
  double shoulder_joint_comp_;
  Eigen::Matrix<double,6,1> effort_offset_;
  double gripper_open_effort_;
  double gripper_close_effort_;
  Eigen::Matrix<double,6,1> ik_seed_pos_;
  double min_traj_duration_;
  double default_speed_factor_;
  GripperArm():
	  arm_module_names_({"Base","Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"}),
          gripper_module_name_({"Spool"}),
	  shoulder_joint_comp_(0),
	  effort_offset_((Eigen::VectorXd(6) << 0, shoulder_joint_comp_, 0,0,0,0).finished()),
	  gripper_open_effort_(1),
	  gripper_close_effort_(-5),
	  ik_seed_pos_((Eigen::VectorXd(6) << 0,1,2.5,1.5,-1.5,1).finished()),
	  min_traj_duration_(.50),
	  default_speed_factor_(.9)
	{
    model = robot_model::RobotModel::loadHRDF("/home/hebi/hebi-cpp-examples/build/hrdf/6-dof_arm_w_gripper.hrdf");
  }
};
