#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>

namespace wire::core {

using ObjectId = std::uint64_t;
constexpr ObjectId kInvalidObjectId = 0;

class IdGenerator {
 public:
  explicit IdGenerator(ObjectId next_id = 1) : next_id_(next_id) {}

  [[nodiscard]] ObjectId next() { return next_id_++; }

  [[nodiscard]] ObjectId peek() const { return next_id_; }

  void reset(ObjectId next_id = 1) { next_id_ = next_id; }

 private:
  ObjectId next_id_ = 1;
};

inline std::string make_display_id(std::string_view prefix, ObjectId id, int pad_width = 6) {
  std::ostringstream oss;
  oss << prefix << "-" << std::setw(pad_width) << std::setfill('0') << id;
  return oss.str();
}

}  // namespace wire::core
