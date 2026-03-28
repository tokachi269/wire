#pragma once

#include <string>
#include <vector>

#include "wire/core/id.hpp"

namespace wire::core {

struct ChangeSet {
  std::vector<ObjectId> created_ids;
  std::vector<ObjectId> updated_ids;
  std::vector<ObjectId> deleted_ids;
  std::vector<ObjectId> dirty_span_ids;
};

template <typename TValue> struct EditResult {
  bool ok = false;
  TValue value{};
  std::string error{};
  ChangeSet change_set{};
};

} // namespace wire::core
