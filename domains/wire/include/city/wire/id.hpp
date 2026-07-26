#pragma once

#include <array>
#include <charconv>
#include <cstdint>
#include <string>
#include <string_view>

namespace city::wire {

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
  std::array<char, 32> digits{};
  const auto [end, ec] = std::to_chars(digits.data(), digits.data() + digits.size(), id);
  const std::size_t digit_count = (ec == std::errc{}) ? static_cast<std::size_t>(end - digits.data()) : 1u;
  const std::size_t zero_pad =
      (pad_width > static_cast<int>(digit_count)) ? static_cast<std::size_t>(pad_width - static_cast<int>(digit_count)) : 0u;

  std::string out{};
  out.reserve(prefix.size() + 1u + zero_pad + digit_count);
  out.append(prefix);
  out.push_back('-');
  out.append(zero_pad, '0');
  if (ec == std::errc{}) {
    out.append(digits.data(), digit_count);
  } else {
    out.push_back('0');
  }
  return out;
}

} // namespace city::wire
