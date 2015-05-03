#include <heightmap/utils.hpp>

namespace heightmap {

  const uint8_t* toByteArrayUnsafe(const float& f) {
     return reinterpret_cast<const uint8_t*>(&f);
  }

}
