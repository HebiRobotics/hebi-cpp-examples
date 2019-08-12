#define EIGEN_INITIALIZE_MATRICES_BY_NAN

#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <util/grav_comp.hpp>
#include "lookup.hpp"
#include "group_feedback.hpp"
#include "group_command.hpp"
#include "robot_model.hpp"
#include "base.hpp"
#include "arm.hpp"

using namespace hebi;

int main(){
  Lookup lookup;
  
  //    % Optional step to limit the lookup to a set of interfaces or modules
  //    % HebiLookup.setLookupAddresses('10.10.10.255');
  bool enable_logging = true;
  //
  //    %%
  //    %%%%%%%%%%%%%%%%%%%%%
  //    % Setup Mobile Base %
  //   %%%%%%%%%%%%%%%%%%%%%
  
  OmniBase base;
  // 
  //    % Maps XYZ chassis velocities to wheel velocities
  Eigen::MatrixXd chassis_mass_matrix(3,3);
  chassis_mass_matrix << base.chassis_mass_,0,0, 0,base.chassis_mass_,0, 0,0,base.chassis_inertia_zz_;

  //    %%
  //    %%%%%%%%%%%%%
  //    % Setup Arm %
  //    %%%%%%%%%%%%%
  GripperArm arm;

  //    % Assume that gravity points up
  Eigen::Vector3d gravity_vec(0, 0, -1);

  //    %%
  //    %%%%%%%%%%%%%%%
  //    % Setup Robot %
  //    %%%%%%%%%%%%%%%
  std::vector<std::string> robot_family = {"Rosie"};
  std::vector<std::string> robot_names = base.wheel_module_names_;
  robot_names.insert(robot_names.end(), arm.arm_module_names_.begin(), arm.arm_module_names_.end());
  robot_names.insert(robot_names.end(), arm.gripper_module_name_.begin(), arm.gripper_module_name_.end());
  
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
     
  auto wheel_group = lookup.getGroupFromNames(robot_family, base.wheel_module_names_);
  GroupCommand base_gains_command(wheel_group->size());
  if (!base_gains_command.readGains("/home/hebi/hebi-cpp-examples/build/gains/omni-drive-wheel-gains.xml")){
    printf("Could not read omni base gains");
    return -1;
  }
  if (!wheel_group->sendCommandWithAcknowledgement(base_gains_command)){
    printf("Could not send omni base gains");
    return -1;
  }
  
  auto arm_group = lookup.getGroupFromNames(robot_family, arm.arm_module_names_);
  GroupCommand arm_gains_command(arm_group->size());
  if (!arm_gains_command.readGains("/home/hebi/hebi-cpp-examples/build/gains/6-dof_arm_gains.xml")){
    printf("Could not read 6 dof arm gains");
    return -1;
  }
  if (!arm_group->sendCommandWithAcknowledgement(arm_gains_command)){
    printf("Could not send 6 dof arm gains");
    return -1;
  }
  
  auto gripper_group = lookup.getGroupFromNames(robot_family, arm.gripper_module_name_);
  GroupCommand gripper_gains_command(gripper_group->size());
  if (!gripper_gains_command.readGains("/home/hebi/hebi-cpp-examples/build/gains/gripper_gains.xml")){
    printf("Could not read gripper gains");
    return -1;
  }
  if (!gripper_group->sendCommandWithAcknowledgement(gripper_gains_command)){
    printf("Could not send gripper gains");
    return -1;
  }

  //    %%
  //    %%%%%%%%%%%%%%%%%%%%%%%%%%
  //    % Setup Mobile I/O Group %
  //    %%%%%%%%%%%%%%%%%%%%%%%%%%

  std::vector<std::string> phone_family = {"HEBI"};
  std::vector<std::string> phone_name = {"Mobile IO"};

  std::printf("Searching for phone Controller...\n");
  std::shared_ptr<Group> phone_group;
  while(true){
    if(phone_group = lookup.getGroupFromNames(phone_family, phone_name)){
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
  if(enable_logging){
    robot_group->startLog("logs");
    phone_group->startLog("logs");
  }	
  //    % This outer loop is what we fall back to anytime we 're-home' the arm
  while(true){ 
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
  
    arm.model->solveIK(arm.ik_seed_pos_, ik_result_joint_angles, robot_model::EndEffectorPositionObjective(xyz_target_init), robot_model::EndEffectorSO3Objective(rot_mat_target_init));
    const size_t len = arm_dofs.size();  
    Eigen::MatrixXd position(len,2);
    position.col(0) = arm_fbk_pos;
    position.col(1) = ik_result_joint_angles;
    Eigen::VectorXd time(2);
    time << 0, arm.min_traj_duration_;
  
    auto arm_traj = hebi::trajectory::Trajectory::createUnconstrainedQp(time, position);

    auto t0 = fbk.getTime();
    auto t = 0.0; 
  
    while(t < arm.min_traj_duration_){
      //
      if(!robot_group->getNextFeedback(fbk)) printf("no feedback recieved\n");
      auto temp_t = fbk.getTime();
      t = (fbk.getTime() - t0) > arm.min_traj_duration_ ? arm.min_traj_duration_ : fbk.getTime() - t0;

      Eigen::VectorXd pos(len);
      Eigen::VectorXd vel(len);
      Eigen::VectorXd accel(len);
      arm_traj->getState(t, &pos, &vel, &accel);
      Eigen::VectorXd pos_tmp = cmd.getPosition();
      Eigen::VectorXd vel_tmp = cmd.getVelocity();
      for(size_t i = 0; i < arm_dofs.size(); i++){
          pos_tmp[arm_dofs[i]] = pos[i];
          vel_tmp[arm_dofs[i]] = vel[i];
      }
      cmd.setPosition(pos_tmp);
      cmd.setVelocity(vel_tmp);
  
      //         TODO:   dynamicsComp
      Eigen::VectorXd masses;
      arm.model->getMasses(masses);
      Eigen::VectorXd effort(arm_dofs.size());
      auto base_accel = fbk[base.num_wheels_ + 0].imu().accelerometer().get();
      Eigen::Vector3d gravity(-base_accel.getX(),
                     -base_accel.getY(),
                     -base_accel.getZ());
      Eigen::VectorXd arm_positions(arm_dofs.size());
      for(size_t i = 0; i < arm_positions.size(); ++i){
        arm_positions[i] = fbk.getPosition()[arm_dofs[i]];
      }
      effort = hebi::util::GravityCompensation::getEfforts(*arm.model, masses, arm_positions, gravity);
      /*TODO add dynamic_comp*/
      auto effort_tmp = cmd.getEffort();
      for(size_t i = 0; i < arm_dofs.size(); i++){
          effort_tmp[arm_dofs[i]] = effort[i] + arm.effort_offset_[i];
      }
      cmd.setEffort(effort_tmp);
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

    GroupCommand wheel_cmd(wheel_group->size());
    //
    //        % Replan Trajectory for the mobile base
    Eigen::Vector2d omni_base_traj_time;
    omni_base_traj_time << 0, base.ramp_time_;
    //
    //        % Initialize trajectory for Omnibase
    Eigen::MatrixXd positions(3,2); 
    Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(3,2); 
    Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(3,2); 
    Eigen::MatrixXd jerks = Eigen::MatrixXd::Zero(3,2); 
    
    auto omni_base_traj = hebi::trajectory::Trajectory::createUnconstrainedQp(time, velocities, &accelerations, &jerks);
    bool first_run = true;
    
    while(true){
      //            %%%%%%%%%%%%%%%%%%%
      //            % Gather Feedback %
      //            %%%%%%%%%%%%%%%%%%%

      if(!robot_group->getNextFeedback(fbk)){
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
      double x_vel = base.max_lin_speed_ * (latest_phone_fbk[0].io().a().hasInt(8) ? latest_phone_fbk[0].io().a().getInt(8) : latest_phone_fbk[0].io().a().getFloat(8)); 
      double y_vel = -1*base.max_lin_speed_ * (latest_phone_fbk[0].io().a().hasInt(7) ? latest_phone_fbk[0].io().a().getInt(7) : latest_phone_fbk[0].io().a().getFloat(7));
      double rot_vel = base.max_rot_speed_ * (latest_phone_fbk[0].io().a().hasInt(1) ? latest_phone_fbk[0].io().a().getInt(1) : latest_phone_fbk[0].io().a().getFloat(1)); 
  
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
      if(first_run){
        for(size_t i = 0; i < arm_dofs.size(); i++){
          pos[i] = fbk.getPositionCommand()[arm_dofs[i]];
        }
        vel = end_velocities;
        acc = end_accels;
        first_run = false;
      } else {
        arm_traj->getState(t, &pos, &vel, &acc);
      }

      Eigen::VectorXd pos_tmp = cmd.getPosition();
      Eigen::VectorXd vel_tmp = cmd.getVelocity();
      for(size_t i = 0; i < arm_dofs.size(); i++){
          pos_tmp[arm_dofs[i]] = pos[i];
          vel_tmp[arm_dofs[i]] = vel[i];
      }
      cmd.setPosition(pos_tmp);
      cmd.setVelocity(vel_tmp);
      //
      //            TODO:dynamicsComp
      Eigen::VectorXd masses;
      arm.model->getMasses(masses);
      auto base_accel = fbk[base.num_wheels_ + 0].imu().accelerometer().get();
      Eigen::Vector3d gravity(-base_accel.getX(),
                     -base_accel.getY(),
                     -base_accel.getZ());
      Eigen::VectorXd arm_positions(arm_dofs.size());
      for(size_t i = 0; i < arm_positions.size(); ++i){
        arm_positions[i] = fbk.getPosition()[arm_dofs[i]];
      }
      Eigen::VectorXd grav_comp = hebi::util::GravityCompensation::getEfforts(*arm.model, masses, arm_positions, gravity);

      Eigen::VectorXd effort_tmp = cmd.getEffort();
      for(size_t i = 0; i < arm_dofs.size(); i++){
          effort_tmp[arm_dofs[i]] = grav_comp[i] + arm.effort_offset_[i];
      }
      cmd.setEffort(effort_tmp);
      //            % Force elbow up config
      auto seed_pos_ik = pos;
      seed_pos_ik[2] = abs(seed_pos_ik[2]);
      //            % Find target using inverse kinematics
      Eigen::VectorXd ik_position(arm_dofs.size());
  
      arm.model->solveIK(seed_pos_ik, ik_position, robot_model::EndEffectorPositionObjective(xyz_target), robot_model::EndEffectorSO3Objective(rot_mat_target));
  
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
      effort_tmp = cmd.getEffort();
      effort_tmp[gripper_dof[0]] = arm.gripper_close_effort_ * (latest_phone_fbk[0].io().a().hasInt(6) ? latest_phone_fbk[0].io().a().getInt(6) : latest_phone_fbk[0].io().a().getFloat(6)); 
      cmd.setEffort(effort_tmp);
      //
      //            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      //            % Evaluate Trajectory State %
      //            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      //            % Chassis (linear velocity)
      if(time_now - chassis_traj_start_time < omni_base_traj->getDuration()){
        t = time_now - chassis_traj_start_time;
      } else {
        t = omni_base_traj->getDuration();
      }
      Eigen::VectorXd chassis_cmd_vel(base.num_wheels_);
      Eigen::VectorXd chassis_cmd_acc(base.num_wheels_);
      Eigen::VectorXd chassis_cmd_jerk(base.num_wheels_);
      omni_base_traj->getState(t, &chassis_cmd_vel, &chassis_cmd_acc, &chassis_cmd_jerk);
      //
      //            % Chassis (convert linear to joint velocities)
      GroupCommand wheel_cmd(wheel_dofs.size());
      wheel_cmd.setVelocity(base.wheel_velocity_matrix_ * chassis_cmd_vel);
      wheel_cmd.setPosition(wheel_cmd.getPosition() + wheel_cmd.getVelocity() * dt);  
      wheel_cmd.setEffort(base.wheel_effort_matrix_ * (chassis_mass_matrix * chassis_cmd_acc));

      pos_tmp = cmd.getPosition();
      vel_tmp = cmd.getVelocity();
      effort_tmp = cmd.getEffort();   
      for(size_t i = 0; i < wheel_dofs.size(); i++){
        pos_tmp[wheel_dofs[i]] = wheel_cmd.getPosition()[i];
        vel_tmp[wheel_dofs[i]] = wheel_cmd.getVelocity()[i];
        effort_tmp[wheel_dofs[i]] = wheel_cmd.getEffort()[i];
      }
      cmd.setPosition(pos_tmp);
      cmd.setVelocity(vel_tmp);
      cmd.setEffort(effort_tmp);
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
      
      Eigen::MatrixXd velocities(base.num_wheels_, 2);
      velocities.col(0) = chassis_cmd_vel;
      velocities.col(1) = chassis_desired_vel;
      
      Eigen::MatrixXd accelerations(base.num_wheels_, 2);
      accelerations.col(0) = chassis_cmd_acc;
      accelerations.col(1) = Eigen::Vector3d::Zero();
  
      Eigen::MatrixXd jerks(base.num_wheels_, 2);
      jerks.col(0) = chassis_cmd_jerk;
      jerks.col(1) = Eigen::Vector3d::Zero();

      omni_base_traj = hebi::trajectory::Trajectory::createUnconstrainedQp(time, velocities, &accelerations , &jerks);
      chassis_traj_start_time = time_now;
    }
  }
}
