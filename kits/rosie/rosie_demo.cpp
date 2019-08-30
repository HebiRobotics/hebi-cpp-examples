#define PI 3.141592653592

#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

#include "util/grav_comp.hpp"
#include "util/vector_utils.h"

#include "lookup.hpp"
#include "group_feedback.hpp"
#include "group_command.hpp"
#include "robot_model.hpp"
#include "trajectory.hpp"

using namespace hebi;

void getTrajectoryCommand(std::shared_ptr<hebi::trajectory::Trajectory> traj,GroupCommand& cmd,
                          std::vector<uint32_t> indices, double t, Eigen::VectorXd* pos=nullptr,
                          Eigen::VectorXd* vel=nullptr, Eigen::VectorXd* acc=nullptr) {
  auto len = indices.size();
  Eigen::VectorXd position(len);
  Eigen::VectorXd velocity(len);
  Eigen::VectorXd acceleration(len);
  traj->getState(t, &position, &velocity, &acceleration);
  hebi::util::setPositionScattered(cmd, indices, position);
  hebi::util::setVelocityScattered(cmd, indices, velocity);
  if(pos!=nullptr)
    *pos = position;
  if(vel!=nullptr)
    *vel = velocity;
  if(acc!=nullptr)
    *acc = acceleration;
}

int main() {
  const std::unique_ptr<robot_model::RobotModel> arm_model = robot_model::RobotModel::loadHRDF("hrdf/6-dof_arm_w_gripper.hrdf");
  const std::vector<std::string> arm_module_names = {"Base","Shoulder", "Elbow", "Wrist1", "Wrist2", "Wrist3"};
  const std::vector<std::string> arm_gripper_module_name = {"Spool"};
  const double arm_shoulder_joint_comp = 0;
  const Eigen::VectorXd arm_effort_offset = (Eigen::VectorXd(6) << 0, arm_shoulder_joint_comp, 0,0,0,0).finished();
  const double arm_gripper_open_effort = 1;
  const double arm_gripper_close_effort = -5;
  const Eigen::VectorXd arm_ik_seed_pos = (Eigen::VectorXd(6) << 0,1,2.5,1.5,-1.5,1).finished();
  const double arm_min_traj_duration = .5;

  const double base_wheel_radius = .15/2.0;
  const double base_wheel_base = 0.470;
  const double base_max_lin_speed = 0.6;
  const double base_max_rot_speed = base_max_lin_speed*(base_wheel_base/2.0);
  const Eigen::Matrix4d base_chassis_com = (Eigen::Matrix4d() << 1,0,0,0, 0,1,0,0, 0,0,1, base_wheel_radius +.005, 0,0,0,1).finished();
  const double base_chassis_mass = 12.0;
  const double base_chassis_inertia_zz = .5*base_chassis_mass*base_wheel_base*base_wheel_base*.25;
  const std::vector<std::string> base_wheel_module_names = {"_Wheel1","_Wheel2","_Wheel3"};
  const uint32_t base_num_wheels  = 3;
  const std::vector<Eigen::Matrix4d> base_wheel_base_frames = 
            {(Eigen::Matrix4d() << 
            1,0,0,base_wheel_base/2.0*cos(-60*PI/180.0),
            0,1,0,base_wheel_base/2.0*sin(-60*PI/180.0),
            0,0,1,base_wheel_radius,
            0,0,0,1).finished(),

            (Eigen::Matrix4d() << 
            1,0,0,base_wheel_base/2.0*cos(60*PI/180.0),
            0,1,0,base_wheel_base/2.0*sin(60*PI/180.0),
            0,0,1,base_wheel_radius,
            0,0,0,1).finished(),

            (Eigen::Matrix4d() <<
            1,0,0,base_wheel_base/2.0*cos(180*PI/180.0),
            0,1,0,base_wheel_base/2.0*cos(180*PI/180.0),
            0,0,1,base_wheel_radius,
            0,0,0,1).finished()};  

  double a1 = -60*PI/180;
  double a2 = 60*PI/180;
  double a3 = 180*PI/180;

  const Eigen::Matrix3d base_wheel_transform = 
                    (Eigen::MatrixXd(3,3) <<
                    sin(a1), -cos(a1), 2/base_wheel_base,
                    sin(a2), -cos(a2), 2/base_wheel_base/2,
                    sin(a3), -cos(a3), 2/base_wheel_base/2).finished();

  const Eigen::Matrix3d base_wheel_velocity_matrix = base_wheel_transform / base_wheel_radius;
  const Eigen::Matrix3d base_wheel_effort_matrix = base_wheel_transform * base_wheel_radius;
  const double base_ramp_time = .33;

  Lookup lookup;
  
  //    % Optional step to limit the lookup to a set of interfaces or modules
  //    % HebiLookup.setLookupAddresses('10.10.10.255');
  bool enable_logging = true;
  //
  //    %%
  //    %%%%%%%%%%%%%%%%%%%%%
  //    % Setup Mobile Base %
  //   %%%%%%%%%%%%%%%%%%%%%
  // 
  //    % Maps XYZ chassis velocities to wheel velocities
  Eigen::MatrixXd chassis_mass_matrix(3,3);
  chassis_mass_matrix << base_chassis_mass,0,0, 0,base_chassis_mass,0, 0,0,base_chassis_inertia_zz;

  //    %%
  //    %%%%%%%%%%%%%
  //    % Setup Arm %
  //    %%%%%%%%%%%%%

  //    % Assume that gravity points up
  Eigen::Vector3d gravity_vec(0, 0, -1);

  //    %%
  //    %%%%%%%%%%%%%%%
  //    % Setup Robot %
  //    %%%%%%%%%%%%%%%
  std::vector<std::string> robot_family = {"Rosie"};
  std::vector<std::string> robot_names = base_wheel_module_names;
  robot_names.insert(robot_names.end(), arm_module_names.begin(), arm_module_names.end());
  robot_names.insert(robot_names.end(), arm_gripper_module_name.begin(), arm_gripper_module_name.end());
  
  auto num_modules_ = robot_names.size();

  std::vector<uint32_t> wheel_dofs = {0,1,2};
  std::vector<uint32_t> arm_dofs = {3,4,5,6,7,8};
  std::vector<uint32_t> gripper_dof = {9};

  auto robot_group = lookup.getGroupFromNames(robot_family, robot_names);

  robot_group->setFeedbackFrequencyHz(100);    
  //    
  //    %%
  //    %%%%%%%%%%%%%%%%%
  //    % Set the Gains %
  //    %%%%%%%%%%%%%%%%%
     
  auto wheel_group = lookup.getGroupFromNames(robot_family, base_wheel_module_names);
  GroupCommand base_gains_command(wheel_group->size());
  if (!base_gains_command.readGains("gains/omni-drive-wheel-gains.xml")) {
    printf("Could not read omni base gains");
    return 1;
  }
  if (!wheel_group->sendCommandWithAcknowledgement(base_gains_command)) {
    printf("Could not send omni base gains");
    return 1;
  }
  
  auto arm_group = lookup.getGroupFromNames(robot_family, arm_module_names);
  GroupCommand arm_gains_command(arm_group->size());
  if (!arm_gains_command.readGains("gains/6-dof_arm_gains.xml")) {
    printf("Could not read 6 dof arm gains");
    return 1;
  }
  if (!arm_group->sendCommandWithAcknowledgement(arm_gains_command)) {
    printf("Could not send 6 dof arm gains");
    return 1;
  }
  
  auto gripper_group = lookup.getGroupFromNames(robot_family, arm_gripper_module_name);
  GroupCommand gripper_gains_command(gripper_group->size());
  if (!gripper_gains_command.readGains("gains/gripper_gains.xml")) {
    printf("Could not read gripper gains");
    return 1;
  }
  if (!gripper_group->sendCommandWithAcknowledgement(gripper_gains_command)) {
    printf("Could not send gripper gains");
    return 1;
  }

  //    %%
  //    %%%%%%%%%%%%%%%%%%%%%%%%%%
  //    % Setup Mobile I/O Group %
  //    %%%%%%%%%%%%%%%%%%%%%%%%%%

  std::vector<std::string> phone_family = {"HEBI"};
  std::vector<std::string> phone_name = {"Mobile IO"};

  std::printf("Searching for phone Controller...\n");
  std::shared_ptr<Group> phone_group;
  while(true) {
    if(phone_group = lookup.getGroupFromNames(phone_family, phone_name)) {
      break;
    }
    std::printf("Searching for phone Controller...\n");
  }
  std::printf("Phone Found. Starting up\n");
  //    % Get the initial feedback objects that we'll reuse later
  GroupFeedback latest_phone_fbk(phone_group->size());// = phone_fbk;
  phone_group->getNextFeedback(latest_phone_fbk);
  
  //    %%
  //    %%%%%%%%%%%%%%%%%%%%%%%
  //    % Begin the demo loop %
  //    %%%%%%%%%%%%%%%%%%%%%%%
  //    
  //    % Start background logging
  if(enable_logging) {
    robot_group->startLog("logs");
    phone_group->startLog("logs");
  }	
  //    % This outer loop is what we fall back to anytime we 're-home' the arm
  while(true) { 
    GroupFeedback fbk(robot_group->size());
    GroupCommand cmd(robot_group->size());
    robot_group->getNextFeedback(fbk);

    //        % Exaggerate Z-Axis by 2x, X-Y are 1-to-1.
    Eigen::Vector3d xyz_scale;
    xyz_scale << -1, -1, -2; 
    //
    //        % Move to current coordinates
    Eigen::Vector3d xyz_target_init;
    xyz_target_init << 0.3, 0.0, 0.3;
    //        % Gripper down
    Eigen::MatrixXd rot_mat_target_init(3,3);
    rot_mat_target_init << -1,0,0, 0,1,0, 0,0,-1; 

    Eigen::VectorXd arm_fbk_pos(arm_dofs.size());
    for(size_t i = 0; i < arm_fbk_pos.size(); ++i){
      arm_fbk_pos[i] = fbk.getPosition()[arm_dofs[i]];
    }
    Eigen::VectorXd ik_result_joint_angles(arm_dofs.size());
  
    arm_model->solveIK(arm_ik_seed_pos, ik_result_joint_angles, robot_model::EndEffectorPositionObjective(xyz_target_init), robot_model::EndEffectorSO3Objective(rot_mat_target_init));
    const size_t len = arm_dofs.size();  
    Eigen::MatrixXd position(len,2);
    position.col(0) = arm_fbk_pos;
    position.col(1) = ik_result_joint_angles;
    Eigen::VectorXd time(2);
    time << 0, arm_min_traj_duration;
  
    auto arm_traj = hebi::trajectory::Trajectory::createUnconstrainedQp(time, position);

    auto t0 = fbk.getTime();
    auto t = 0.0; 
  
    while(t < arm_min_traj_duration) {
      //
      if(!robot_group->getNextFeedback(fbk)) printf("no feedback recieved\n");
      auto temp_t = fbk.getTime();
      t = (fbk.getTime() - t0) > arm_min_traj_duration ? arm_min_traj_duration : fbk.getTime() - t0;

      getTrajectoryCommand(arm_traj, cmd, arm_dofs, t);
  
      //         TODO:   dynamicsComp
      Eigen::VectorXd masses;
      arm_model->getMasses(masses);
      Eigen::VectorXd effort(arm_dofs.size());
      auto base_accel = fbk[base_num_wheels].imu().accelerometer().get();
      Eigen::Vector3d gravity(-base_accel.getX(),
                     -base_accel.getY(),
                     -base_accel.getZ());
      Eigen::VectorXd arm_positions(arm_dofs.size());
      for(size_t i = 0; i < arm_positions.size(); ++i){
        arm_positions[i] = fbk.getPosition()[arm_dofs[i]];
      }
      effort = hebi::util::getGravityCompensationEfforts(*arm_model, masses, arm_positions, gravity);
      /*TODO add dynamic_comp*/
      // Apply the effort offset as well
      effort += arm_effort_offset;
      hebi::util::setEffortScattered(cmd, arm_dofs, effort);
      robot_group->sendCommand(cmd);
    }
    //        % Grab initial pose
    GroupFeedback latest_phone_mobile(phone_group->size());
    while(!phone_group->getNextFeedback(latest_phone_mobile)){
      printf("feedback not recieved\n\n");
    }
  
    auto orient = latest_phone_fbk[0].mobile().arOrientation().get();
    Eigen::Quaterniond q;
    q.w() = orient.getW();
    q.x() = orient.getX();
    q.y() = orient.getY();
    q.z() = orient.getZ();

    auto r_init = q.toRotationMatrix(); 

    auto ar_pos = latest_phone_fbk[0].mobile().arPosition().get();
    Eigen::Vector3d xyz_init;
    xyz_init << ar_pos.getX(), ar_pos.getY(), ar_pos.getZ();
    auto xyz_phone_new = xyz_init;

    Eigen::VectorXd end_velocities = Eigen::VectorXd::Zero(arm_dofs.size()); 
    Eigen::VectorXd end_accels = Eigen::VectorXd::Zero(arm_dofs.size()); 

    t0 = fbk.getTime();

    auto time_last = t0;

    auto chassis_traj_start_time = t0;  

    GroupCommand wheel_cmd(wheel_group->size());\
    wheel_cmd.setPosition(hebi::util::vectorGather(wheel_cmd.getPosition(),wheel_dofs, fbk.getPosition()));
    //
    //        % Replan Trajectory for the mobile base
    Eigen::Vector2d omni_base_traj_time;
    omni_base_traj_time << 0, base_ramp_time;
    //
    //        % Initialize trajectory for Omnibase
    Eigen::MatrixXd positions(3,2); 
    Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(3,2); 
    Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(3,2); 
    Eigen::MatrixXd jerks = Eigen::MatrixXd::Zero(3,2); 
    
    auto omni_base_traj = hebi::trajectory::Trajectory::createUnconstrainedQp(time, velocities, &accelerations, &jerks);
    bool first_run = true;
    
    while(true) {
      //            %%%%%%%%%%%%%%%%%%%
      //            % Gather Feedback %
      //            %%%%%%%%%%%%%%%%%%%

      if(!robot_group->getNextFeedback(fbk)) {
        printf("Could not get robot feedback!");
        break;
      }
  
      //
      //            % Get feedback with a timeout of 0, which means that they return
      //            % instantly, but if there was no new feedback, they return empty.
      //            % This is because the mobile device is on wireless and might drop
      //            % out or be really delayed, in which case we would rather keep
      //            % running with an old data instead of waiting here for new data.
      bool tmp_fbk = phone_group->getNextFeedback(latest_phone_fbk);

      auto time_now = fbk.getTime();
      auto dt = time_now - time_last;
      time_last = fbk.getTime();
      //
      //            % Reset the Command Struct
      cmd.clear();
  
      //
      //            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
      //            % Read/Map Joystick Inputs %
      //            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
      //
      //            % Check for restart command
      double b_1 = latest_phone_fbk[0].io().b().hasInt(1) ? latest_phone_fbk[0].io().b().getInt(1) : latest_phone_fbk[0].io().b().getFloat(1);
      if(b_1 == 1) break;
      
      //
      //            % Parameter to limit XYZ Translation of the arm if a slider is
      //            % pulled down.
      double phone_control_scale = (latest_phone_fbk[0].io().a().hasInt(3) ? latest_phone_fbk[0].io().a().getInt(3) : latest_phone_fbk[0].io().a().getFloat(3) + 1)/2;
      if(phone_control_scale < .1){
        xyz_init = xyz_phone_new;
      }
      //
      //            % Joystick Input for Omnibase Control        
      double x_vel = base_max_lin_speed * (latest_phone_fbk[0].io().a().hasInt(8) ? latest_phone_fbk[0].io().a().getInt(8) : latest_phone_fbk[0].io().a().getFloat(8)); 
      double y_vel = -1*base_max_lin_speed * (latest_phone_fbk[0].io().a().hasInt(7) ? latest_phone_fbk[0].io().a().getInt(7) : latest_phone_fbk[0].io().a().getFloat(7));
      double rot_vel = base_max_rot_speed * (latest_phone_fbk[0].io().a().hasInt(1) ? latest_phone_fbk[0].io().a().getInt(1) : latest_phone_fbk[0].io().a().getFloat(1)); 
  
      //
      //            % Pose Information for Arm Control
      xyz_init << latest_phone_fbk[0].mobile().arPosition().get().getX(), latest_phone_fbk[0].mobile().arPosition().get().getY(), latest_phone_fbk[0].mobile().arPosition().get().getZ();
      auto xyz_target = xyz_target_init + (phone_control_scale * xyz_scale.array() * (r_init.transpose() * (xyz_phone_new - xyz_init)).array()).matrix();
    
      q.w() = latest_phone_fbk[0].mobile().arOrientation().get().getW();
      q.x() = latest_phone_fbk[0].mobile().arOrientation().get().getX();
      q.y() = latest_phone_fbk[0].mobile().arOrientation().get().getY();
      q.z() = latest_phone_fbk[0].mobile().arOrientation().get().getZ();

      auto rot_mat_target = r_init.transpose() * q.toRotationMatrix() * rot_mat_target_init;
      //
      //            %%%%%%%%%%%%%%%
      //            % Arm Control %
      //            %%%%%%%%%%%%%%%
      //            % Get state of current trajectory
      Eigen::VectorXd pos(arm_dofs.size());
      Eigen::VectorXd vel(arm_dofs.size());
      Eigen::VectorXd acc(arm_dofs.size());
      if(first_run) {
        pos = hebi::util::vectorGather(pos,arm_dofs,fbk.getPositionCommand());
        vel = end_velocities;
        acc = end_accels;
        hebi::util::setPositionScattered(cmd, arm_dofs, pos);
        hebi::util::setVelocityScattered(cmd, arm_dofs, vel);
        first_run = false;
      } else {
        getTrajectoryCommand(arm_traj,cmd,arm_dofs,t,&pos,&vel,&acc);
      }
      //
      //            TODO:dynamicsComp
      Eigen::VectorXd masses;
      arm_model->getMasses(masses);
      auto base_accel = fbk[base_num_wheels].imu().accelerometer().get();
      Eigen::Vector3d gravity(-base_accel.getX(),
                     -base_accel.getY(),
                     -base_accel.getZ());
      Eigen::VectorXd arm_positions(arm_dofs.size());
      for(size_t i = 0; i < arm_positions.size(); ++i) {
        arm_positions[i] = fbk.getPosition()[arm_dofs[i]];
      }
      Eigen::VectorXd grav_comp = hebi::util::getGravityCompensationEfforts(*arm_model, masses, arm_positions, gravity);
      // Apply the effort offset as well
      grav_comp += arm_effort_offset;
      hebi::util::setEffortScattered(cmd, arm_dofs, grav_comp);
      //            % Force elbow up config
      auto seed_pos_ik = pos;
      seed_pos_ik[2] = abs(seed_pos_ik[2]);
      //            % Find target using inverse kinematics
      Eigen::VectorXd ik_position(arm_dofs.size());
  
      arm_model->solveIK(seed_pos_ik, ik_position, robot_model::EndEffectorPositionObjective(xyz_target), robot_model::EndEffectorSO3Objective(rot_mat_target));
  
      //            % Start new trajectory at the current state
      Eigen::MatrixXd traj_pos(arm_dofs.size(),2);
      traj_pos.col(0) = pos;
      traj_pos.col(1) = ik_position;
      Eigen::MatrixXd traj_vel(arm_dofs.size(),2);
      traj_vel.col(0) = vel;
      traj_vel.col(1) = end_velocities;
      Eigen::MatrixXd traj_acc(arm_dofs.size(),2);
      traj_acc.col(0) = acc;
      traj_acc.col(1) = end_accels;
  
      arm_traj = hebi::trajectory::Trajectory::createUnconstrainedQp(time, traj_pos, &traj_vel, &traj_acc);
      //
      //            %%%%%%%%%%%%%%%%%%%
      //            % Gripper Control %
      //            %%%%%%%%%%%%%%%%%%%
      auto effort_tmp = cmd.getEffort();
      effort_tmp[gripper_dof[0]] = arm_gripper_close_effort * (latest_phone_fbk[0].io().a().hasInt(6) ? latest_phone_fbk[0].io().a().getInt(6) : latest_phone_fbk[0].io().a().getFloat(6)); 
      cmd.setEffort(effort_tmp);
      //
      //            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      //            % Evaluate Trajectory State %
      //            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      //            % Chassis (linear velocity)
      if(time_now - chassis_traj_start_time < omni_base_traj->getDuration()) {
        t = time_now - chassis_traj_start_time;
      } else {
        t = omni_base_traj->getDuration();
      }
      Eigen::VectorXd chassis_cmd_vel(base_num_wheels);
      Eigen::VectorXd chassis_cmd_acc(base_num_wheels);
      Eigen::VectorXd chassis_cmd_jerk(base_num_wheels);
      omni_base_traj->getState(t, &chassis_cmd_vel, &chassis_cmd_acc, &chassis_cmd_jerk);
      //
      //            % Chassis (convert linear to joint velocities)
      wheel_cmd.setVelocity(base_wheel_velocity_matrix * chassis_cmd_vel);
      wheel_cmd.setPosition(wheel_cmd.getPosition() + wheel_cmd.getVelocity() * dt);
      wheel_cmd.setEffort(base_wheel_effort_matrix * (chassis_mass_matrix * chassis_cmd_acc));

      hebi::util::setPositionScattered(cmd, wheel_dofs, wheel_cmd.getPosition());
      hebi::util::setVelocityScattered(cmd, wheel_dofs, wheel_cmd.getVelocity());
      hebi::util::setEffortScattered(cmd, wheel_dofs, wheel_cmd.getEffort());
      
      //            % Hold down button 8 to put the arm in a compliant grav-comp mode
      double b_8 = (latest_phone_fbk[0].io().b().hasInt(8) ? latest_phone_fbk[0].io().b().getInt(8) : latest_phone_fbk[0].io().b().getFloat(8));
      if(b_8 == 1){
        effort_tmp = cmd.getEffort();
        cmd.clear();
        cmd.setEffort(effort_tmp);
      }
      //
      //            % Send to robot
      robot_group->sendCommand(cmd);
      //
      //            % Chassis (linear velocity)
      Eigen::Vector3d chassis_desired_vel;
      chassis_desired_vel << x_vel, y_vel, rot_vel;
      
      Eigen::MatrixXd velocities(base_num_wheels, 2);
      velocities.col(0) = chassis_cmd_vel;
      velocities.col(1) = chassis_desired_vel;
      
      Eigen::MatrixXd accelerations(base_num_wheels, 2);
      accelerations.col(0) = chassis_cmd_acc;
      accelerations.col(1) = Eigen::Vector3d::Zero();
  
      Eigen::MatrixXd jerks(base_num_wheels, 2);
      jerks.col(0) = chassis_cmd_jerk;
      jerks.col(1) = Eigen::Vector3d::Zero();

      omni_base_traj = hebi::trajectory::Trajectory::createUnconstrainedQp(time, velocities, &accelerations , &jerks);
      chassis_traj_start_time = time_now;
    }
  }
}
