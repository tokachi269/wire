#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "city/wire/id.hpp"

namespace city::wire {

struct ChangeSet {
  std::vector<ObjectId> created_ids;
  std::vector<ObjectId> updated_ids;
  std::vector<ObjectId> deleted_ids;
};

enum class CommitFailureCategory : std::uint8_t {
  kNone = 0,
  kRequirementConstraint = 1,
  kInvalidInput = 2,
  kNotImplemented = 3,
  kStateConflict = 4,
  kInternalError = 5,
};

[[nodiscard]] inline const char* DefaultReasonCode(CommitFailureCategory category) {
  switch (category) {
    case CommitFailureCategory::kNone: return "";
    case CommitFailureCategory::kRequirementConstraint: return "requirement_constraint";
    case CommitFailureCategory::kInvalidInput: return "invalid_input";
    case CommitFailureCategory::kNotImplemented: return "not_implemented";
    case CommitFailureCategory::kStateConflict: return "state_conflict";
    case CommitFailureCategory::kInternalError: return "internal_error";
  }
  return "internal_error";
}

[[nodiscard]] inline bool starts_with(std::string_view value, std::string_view prefix) {
  return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
}

[[nodiscard]] inline CommitFailureCategory ClassifyCommitFailure(std::string_view error) {
  if (error.empty()) {
    return CommitFailureCategory::kNone;
  }
  if (starts_with(error, "backbone invalid input:") ||
      starts_with(error, "core invalid input:") ||
      starts_with(error, "authoritative invalid input:") ||
      starts_with(error, "cable curve invalid input:") ||
      starts_with(error, "model assembly invalid input:") ||
      starts_with(error, "cable population invalid input:")) {
    return CommitFailureCategory::kInvalidInput;
  }
  if (starts_with(error, "backbone requirement constraint:") ||
      starts_with(error, "core requirement constraint:")) {
    return CommitFailureCategory::kRequirementConstraint;
  }
  if (starts_with(error, "backbone state conflict:") ||
      starts_with(error, "core state conflict:")) {
    return CommitFailureCategory::kStateConflict;
  }
  if (starts_with(error, "backbone unsupported:") ||
      starts_with(error, "core unsupported:") ||
      starts_with(error, "authoritative unsupported:") ||
      starts_with(error, "cable curve unsupported:") ||
      starts_with(error, "model assembly unsupported:") ||
      starts_with(error, "model mount graph unsupported:") ||
      starts_with(error, "cable population unsupported:")) {
    return CommitFailureCategory::kNotImplemented;
  }
  if (starts_with(error, "backbone internal:") ||
      starts_with(error, "core internal:") ||
      starts_with(error, "authoritative internal:") ||
      starts_with(error, "cable curve internal:") ||
      starts_with(error, "model assembly internal:") ||
      starts_with(error, "model mount graph internal:") ||
      starts_with(error, "cable population internal:")) {
    return CommitFailureCategory::kInternalError;
  }
  return CommitFailureCategory::kInternalError;
}

[[nodiscard]] inline const char* CommitFailureReasonCode(std::string_view error,
                                                         CommitFailureCategory category) {
  if (starts_with(error, "backbone state conflict: unknown node reference")) {
    return "stale_anchor_reference";
  }
  if (starts_with(error, "backbone requirement constraint: no selected bundle template allows midair branch")) {
    return "midair_branch_disabled";
  }
  return DefaultReasonCode(category);
}

template <typename TValue> struct EditResult {
  bool ok = false;
  TValue value{};
  std::string error{};
  CommitFailureCategory failure_category = CommitFailureCategory::kNone;
  std::string reason_code{};
  ChangeSet change_set{};

  [[nodiscard]] CommitFailureCategory effective_failure_category() const {
    if (ok) {
      return CommitFailureCategory::kNone;
    }
    return failure_category == CommitFailureCategory::kNone ? ClassifyCommitFailure(error) : failure_category;
  }

  void classify_error() {
    failure_category = effective_failure_category();
    if (!ok && reason_code.empty()) {
      reason_code = CommitFailureReasonCode(error, failure_category);
    }
  }
};

} // namespace city::wire
