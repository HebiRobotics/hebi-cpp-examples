#pragma once

#include "hebi.h"

#include <vector>

#include "Eigen/Eigen"
#include "command.hpp"

namespace hebi {

/**
 * \brief A list of Command objects appropriate for sending to a Group of
 * modules; the size() must match the number of modules in the group.
 */
class GroupCommand final {
public:
#ifndef DOXYGEN_OMIT_INTERNAL
  /**
   * C-style group command object.
   * NOTE: this should not be used except by library functions!
   */
  HebiGroupCommandPtr internal_;
#endif // DOXYGEN_OMIT_INTERNAL

private:
  /**
   * The number of modules in this group command.
   */
  const size_t number_of_modules_;
  /**
   * The list of Command subobjects
   */
  std::vector<Command> commands_;

public:
  /**
   * \brief Create a group command with the specified number of modules.
   */
  GroupCommand(size_t number_of_modules);

  /**
   * \brief Destructor cleans up group command object as necessary.
   */
  ~GroupCommand() noexcept; /* annotating specified destructor as noexcept is best-practice */

  /**
   * \brief Returns the number of module commands in this group command.
   */
  size_t size() const;

  /**
   * \brief Access the command for an individual module.
   */
  Command& operator[](size_t index);

  /**
   * \brief Access the command for an individual module.
   */
  const Command& operator[](size_t index) const;

  /**
   * \brief Clears all data in this GroupCommand object; this returns to the
   * state the GroupCommand was at time of creation.
   */
  void clear();

  /**
   * \brief Import the gains from a file into this GroupCommand object.
   * \param file The filename (or path + filename) to the file to read from.
   */
  bool readGains(const std::string& file);

  /**
   * \brief Export the gains from this GroupCommand object into a file, creating it as necessary.
   * \param file The filename (or path + filename) to the file to write to.
   */
  bool writeGains(const std::string& file) const;

  /**
   * \brief Convenience function for setting position commands from Eigen
   * vectors.
   *
   * Note that if the vector is not the correct link, no action is taken.
   */
  void setPosition(const Eigen::VectorXd& position);
  /**
   * \brief Convenience function for setting velocity commands from Eigen
   * vectors.
   *
   * Note that if the vector is not the correct link, no action is taken.
   */
  void setVelocity(const Eigen::VectorXd& velocity);
  /**
   * \brief Convenience function for setting effort commands from Eigen
   * vectors.
   *
   * Note that if the vector is not the correct link, no action is taken.
   */
  void setEffort(const Eigen::VectorXd& effort);
  /**
   * \brief Convenience function for setting spring constant commands from Eigen
   * vectors.
   *
   * Note that if the vector is not the correct link, no action is taken.
   */
  void setSpringConstant(const Eigen::VectorXd& springConstant);

  /**
   * \brief Convenience function for returning commanded position values.
   */
  Eigen::VectorXd getPosition() const;
  /**
   * \brief Convenience function for returning commanded velocity values.
   */
  Eigen::VectorXd getVelocity() const;
  /**
   * \brief Convenience function for returning commanded effort values.
   */
  Eigen::VectorXd getEffort() const;
  /**
   * \brief Convenience function for returning commanded spring constant values.
   */
  Eigen::VectorXd getSpringConstant() const;

  /**
   * \brief Convenience function for returning commanded position values.
   */
  void getPosition(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning commanded velocity values.
   */
  void getVelocity(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning commanded effort values.
   */
  void getEffort(Eigen::VectorXd& out) const;
  /**
   * \brief Convenience function for returning commanded spring constant values.
   */
  void getSpringConstant(Eigen::VectorXd& out) const;
};

} // namespace hebi
