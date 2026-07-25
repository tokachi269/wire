#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "wire/core/id.hpp"

namespace wire::core {

struct ChangeSet {
  std::vector<ObjectId> created_ids;
  std::vector<ObjectId> updated_ids;
  std::vector<ObjectId> deleted_ids;
};

enum class EditErrorKind : std::uint8_t {
  kNone = 0,
  kValidation = 1,
  kUnsupported = 2,
  kInternal = 3,
};

[[nodiscard]] inline bool starts_with(std::string_view value, std::string_view prefix) {
  return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
}

[[nodiscard]] inline EditErrorKind ClassifyEditError(std::string_view error) {
  if (error.empty()) {
    return EditErrorKind::kNone;
  }
  if (starts_with(error, "backbone invalid input:")) {
    return EditErrorKind::kValidation;
  }
  if (starts_with(error, "backbone internal:")) {
    return EditErrorKind::kInternal;
  }
  return EditErrorKind::kUnsupported;
}

template <typename TValue> struct EditResult {
  bool ok = false;
  TValue value{};
  std::string error{};
  EditErrorKind error_kind = EditErrorKind::kNone;
  ChangeSet change_set{};

  [[nodiscard]] EditErrorKind effective_error_kind() const {
    if (ok) {
      return EditErrorKind::kNone;
    }
    return error_kind == EditErrorKind::kNone ? ClassifyEditError(error) : error_kind;
  }

  void classify_error() {
    error_kind = effective_error_kind();
  }
};

} // namespace wire::core
