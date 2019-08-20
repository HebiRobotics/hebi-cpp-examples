#pragma once

namespace hebi {

/**
 * \brief A simple structure to hold a standard semantic versioning version
 * number.
 */
class VersionNumber {
public:
  VersionNumber(int major, int minor, int revision) : major_(major), minor_(minor), revision_(revision) {}
  int getMajor() { return major_; }
  int getMinor() { return minor_; }
  int getRevision() { return revision_; }

private:
  int major_;
  int minor_;
  int revision_;
};

/**
 * \brief Returns the version numbers for the HEBI C API that is wrapped by this
 * HEBI C++ API.
 */
VersionNumber getCVersion();

/**
 * \brief Returns the version numbers for the HEBI C++ API.
 */
VersionNumber getCppVersion();

} // namespace hebi
