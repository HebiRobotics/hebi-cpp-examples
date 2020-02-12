#include "version.hpp"

#include "hebi.h"
#include <cstdio>
#include <exception>

struct VersionChecker {
  VersionChecker() {
    auto version = hebi::getCVersion();
    if (version.getMajor() != 2) {
      fprintf(stderr,
        "ERROR: Loaded an incompatible C API version (%d.%d.%d)\n",
        version.getMajor(), version.getMinor(), version.getRevision());
      std::terminate();
    }
  }
};

static VersionChecker check;

namespace hebi {

VersionNumber getCVersion() {
  int32_t maj, min, rev;
  hebiGetLibraryVersion(&maj, &min, &rev);
  return VersionNumber(maj, min, rev);
}

VersionNumber getCppVersion() { return VersionNumber(3, 1, 1); }

} // namespace hebi
