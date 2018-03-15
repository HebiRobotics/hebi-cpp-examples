#pragma once

/**
 * This file is a structure that holds related data describing a robot arm,
 * such as the physical modules which represent it and the kinematic and
 * dynamic properties of that arm.
 */

#include "lookup.hpp"
#include "group.hpp"
#include "robot_model.hpp"
#include "Eigen/Dense"

using ActuatorType = hebi::robot_model::RobotModel::ActuatorType;
using BracketType = hebi::robot_model::RobotModel::BracketType;
using LinkType = hebi::robot_model::RobotModel::LinkType;

namespace hebi {

  /**
   * This class holds related data describing a robot arm -- such as the
   * physical modules which represent it (the hebi::Group object) and the
   * kinematic and dynamic properties of that arm (the hebi::RobotModel object).
   */
  class ArmContainer
  {
  public:
    ~ArmContainer() = default;

    Group& getGroup() { return *group_; }
    robot_model::RobotModel& getRobotModel() { return *robot_model_; }
    Eigen::VectorXd& getMasses() { return masses_; }

    /*
     * This static factory method creates an ArmContainer describing a 3 DOF
     * robot arm.
     */
    static std::unique_ptr<ArmContainer> create3Dof()
    {
      // Look on the network for the requested modules
      Lookup lookup;
      std::shared_ptr<Group> arm = lookup.getGroupFromNames(
        {"HEBI"},
        {"base", "shoulder", "elbow"});

      if (!arm)
      {
        std::cout << "Could not find arm group - check names!" << std::endl;
        return std::unique_ptr<ArmContainer>();
      }

      // Create a simple kinematic description of the arm 
      std::unique_ptr<robot_model::RobotModel> model(
        new robot_model::RobotModel());
      model->addActuator(ActuatorType::X5_4);
      model->addBracket(BracketType::X5LightRight);
      model->addActuator(ActuatorType::X5_4);
      model->addLink(LinkType::X5, 0.18, M_PI);
      model->addActuator(ActuatorType::X5_4);
      model->addLink(LinkType::X5, 0.15, 0);

      // The degrees of freedom on the arm should match the kinematic
      // description!
      assert(arm->size() == static_cast<int>(model->getDoFCount()));

      return std::unique_ptr<ArmContainer>(
        new ArmContainer(arm, std::move(model)));
    }

    /**
     * Create an ArmContainer from your own group and robot model object.
     * Note -- this takes ownership of the RobotModel pointer.  See example
     * "create" functions above for example usage.
     */
    ArmContainer(std::shared_ptr<Group> group, std::unique_ptr<robot_model::RobotModel> robot_model)
      : group_(group), robot_model_(std::move(robot_model))
    {
      // Retrieve masses from robot model for efficient/convenient access later.
      masses_.resize(robot_model_->getFrameCount(HebiFrameTypeCenterOfMass));
      robot_model_->getMasses(masses_);
    }

  private:

    std::shared_ptr<Group> group_;
    std::unique_ptr<robot_model::RobotModel> robot_model_;
    Eigen::VectorXd masses_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace hebi
