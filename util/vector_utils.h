#pragma once

#include <Eigen/Dense>

#include "group_command.hpp"

namespace hebi {
namespace util {

Eigen::VectorXd stdToEigenXd(const std::vector<double>& in) {
  // Wrap in a Eigen::Map, and then copy on return so we aren't holding onto the memory.
  return Eigen::Map<const Eigen::VectorXd>(in.data(), in.size());
}

Eigen::Vector3d stdToEigen3d(const std::vector<double>& in) {
  if (in.size() != 3) {
    std::cerr << "Error -- vector size mismatch!\n";
    return Eigen::Vector3d::Zero();
  }
  return Eigen::Vector3d(in.data());
}

template<typename ScalarType, typename IndexType, int RowCount=1, int ColCount=Eigen::Dynamic>
Eigen::Matrix<ScalarType, RowCount, ColCount> vectorScatter(Eigen::Matrix<ScalarType, RowCount, ColCount> vect, const std::vector<IndexType>& indices, const Eigen::Matrix<ScalarType, RowCount, ColCount>& values) {
  for(size_t i = 0; i < indices.size(); i++) {
    vect[indices[i]] = values[i];
  }
  return vect;
}

template<typename ScalarType, typename IndexType, int RowCount=1, int ColCount=Eigen::Dynamic>
Eigen::Matrix<ScalarType, RowCount, ColCount> vectorGather(Eigen::Matrix<ScalarType, RowCount, ColCount> vect, const std::vector<IndexType>& indices, const Eigen::Matrix<ScalarType, RowCount, ColCount>& values) {
  for(size_t i = 0; i < indices.size(); i++) {
    vect[i] = values[indices[i]];
  }
  return vect;
}

template<typename ScalarType, typename IndexType, int RowCount=1, int ColCount=Eigen::Dynamic>
void setPositionScattered(GroupCommand& cmd, const std::vector<IndexType>& indices, const Eigen::Matrix<ScalarType, RowCount, ColCount>& values) {
  Eigen::VectorXd pos_tmp = cmd.getPosition();
  for(size_t i = 0; i < indices.size(); i++) {
    pos_tmp[indices[i]] = values[i];
  }
  cmd.setPosition(pos_tmp);
}

template<typename ScalarType, typename IndexType, int RowCount=1, int ColCount=Eigen::Dynamic>
void setVelocityScattered(GroupCommand& cmd, const std::vector<IndexType>& indices, const Eigen::Matrix<ScalarType, RowCount, ColCount>& values) {
  Eigen::VectorXd vel_tmp = cmd.getVelocity();
  for(size_t i = 0; i < indices.size(); i++) {
    vel_tmp[indices[i]] = values[i];
  }
  cmd.setVelocity(vel_tmp);
}

template<typename ScalarType, typename IndexType, int RowCount=1, int ColCount=Eigen::Dynamic>
void setEffortScattered(GroupCommand& cmd, const std::vector<IndexType>& indices, const Eigen::Matrix<ScalarType, RowCount, ColCount>& values) {
  Eigen::VectorXd effort_tmp = cmd.getEffort();
  for(size_t i = 0; i < indices.size(); i++) {
    effort_tmp[indices[i]] = values[i];
  }
  cmd.setEffort(effort_tmp);
}

}
}
