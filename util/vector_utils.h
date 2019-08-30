#pragma once

#include <Eigen/Dense>

#include "group_command.hpp"

namespace hebi {
namespace util {

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
