
#define PI 3.1415926535

class Base{
  public:
  double wheel_radius_;
  double wheel_base_;
  double max_lin_speed_;
  double max_rot_speed_;
  Eigen::Matrix<double, 4, 4> chassis_com_;
  double chassis_mass_;
  double chassis_inertia_zz_;
  std::vector<std::string> wheel_module_names_;
  uint32_t num_wheels_;
  char* gains_filename_;
  std::vector<Eigen::Matrix<double,4,4>> wheel_base_frames_;  
  Eigen::Matrix<double, 3, 3> wheel_velocity_matrix_;
  Eigen::Matrix<double, 3, 3> wheel_effort_matrix_;
  double ramp_time_;
  Base():
	 wheel_radius_(.150/2.0),
        wheel_base_(.470),
        max_lin_speed_(.6),
	max_rot_speed_(max_lin_speed_*(wheel_base_/2.0)),
	chassis_com_((Eigen::MatrixXd(4,4) << 1,0,0,0, 0,1,0,0, 0,0,1, wheel_radius_+.005, 0,0,0,1).finished()),
	chassis_mass_(12.0),
	chassis_inertia_zz_(.5*chassis_mass_*wheel_base_*wheel_base_*.25),
	wheel_module_names_({"_Wheel1","_Wheel2","_Wheel3"}),
	num_wheels_(3),
	gains_filename_("/home/hebi/hebi-cpp-examples/build/gains/omni-drive-wheel-gains.xml"),
	wheel_base_frames_({(Eigen::MatrixXd(4,4) << 1,0,0,wheel_base_/2.0*cos(-60*PI/180.0),
			     0,1,0,wheel_base_/2.0*sin(-60*PI/180.0),
			     0,0,1,wheel_radius_,
			     0,0,0,1).finished(),

			    (Eigen::MatrixXd(4,4) << 1,0,0,wheel_base_/2.0*cos(60*PI/180.0),
			     0,1,0,wheel_base_/2.0*sin(60*PI/180.0),
			     0,0,1,wheel_radius_,
			     0,0,0,1).finished(),

			    (Eigen::MatrixXd(4,4) << 1,0,0,wheel_base_/2.0*cos(180*PI/180.0),
			     0,1,0,wheel_base_/2.0*cos(180*PI/180.0),
			     0,0,1,wheel_radius_,
			     0,0,0,1).finished()}),
	ramp_time_(.33){}

};

class OmniBase : public Base {
	public:
  OmniBase(){       
    double a1 = -60*PI/180;
    double a2 = 60*PI/180;
    double a3 = 180*PI/180;

    Eigen::Matrix<double, 3, 3> wheel_transform_;
    wheel_transform_ << sin(a1), -cos(a1), wheel_base_/2,
	                sin(a2), -cos(a2), wheel_base_/2,
	                sin(a3), -cos(a3), wheel_base_/2;
    for(size_t i = 0; i < 9; ++i){
      wheel_velocity_matrix_(i/3, i%3) = wheel_transform_(i/3, i%3) / wheel_radius_;
      wheel_effort_matrix_(i/3, i%3) = wheel_transform_(i/3, i%3) * wheel_radius_;
    }  
  }

};
