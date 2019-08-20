#include "version.hpp"

#include "hebi.h"

namespace hebi {

VersionNumber getCVersion() {
  int32_t maj, min, rev;
  hebiGetLibraryVersion(&maj, &min, &rev);
  return VersionNumber(maj, min, rev);
}

VersionNumber getCppVersion() { return VersionNumber(2, 0, 1); }

} // namespace hebi
