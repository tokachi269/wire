#pragma once

#include <cstdint>
#include <vector>

#include "wire/core/core_edit_types.hpp"
#include "wire/core/core_runtime_types.hpp"
#include "wire/core/entities.hpp"
#include "wire/core/id.hpp"
#include "wire/core/workflow_types.hpp"

namespace wire::core {

enum class ValidationSeverity : std::uint8_t {
  kError = 0,
  kWarning = 1,
};

struct ValidationIssue {
  ValidationSeverity severity = ValidationSeverity::kError;
  std::string code{};
  std::string message{};
  ObjectId object_id = kInvalidObjectId;
};

struct ValidationResult {
  std::vector<ValidationIssue> issues;

  [[nodiscard]] bool has_errors() const;
  [[nodiscard]] bool ok() const { return !has_errors(); }
};

struct CommitOptions {
  bool run_recalc = true;
  bool run_validate_fast = true;
  bool run_validate = false;
};

struct CommitResult {
  RecalcStats recalc_stats{};
  ValidationResult validation{};
};

struct AddConnectionByPoleOptions {
  SpanKind span_kind = SpanKind::kDistribution;
  SpanLayer span_layer = SpanLayer::kUnknown;
  ObjectId bundle_id = kInvalidObjectId;
  BundleKind bundle_template_id = BundleKind::kLowVoltage;
  bool use_bundle_template = false;
  bool auto_create_bundle = true;
  bool allow_generate_port = true;
  ConnectionContext connection_context = ConnectionContext::kTrunkContinue;
  PoleContextKind pole_context_a = PoleContextKind::kStraight;
  PoleContextKind pole_context_b = PoleContextKind::kStraight;
  double corner_angle_deg_a = 0.0;
  double corner_angle_deg_b = 0.0;
  double corner_turn_sign_a = 0.0;
  double corner_turn_sign_b = 0.0;
  ObjectId reference_span_id = kInvalidObjectId;
  std::uint32_t branch_index = 0;
  ObjectId preferred_port_a_id = kInvalidObjectId;
  ObjectId preferred_port_b_id = kInvalidObjectId;
};

struct ResolveBranchPickOptions {
  std::vector<BundleKind> selected_bundle_template_ids{};
  double snap_radius_world = 0.6;
  bool create_midair_node = true;
  bool enforce_midair_template_policy = true;
};

struct AddConnectionByPoleResult {
  ObjectId span_id = kInvalidObjectId;
  ObjectId port_a_id = kInvalidObjectId;
  ObjectId port_b_id = kInvalidObjectId;
};

struct AddDropResult {
  ObjectId span_id = kInvalidObjectId;
  ObjectId source_port_id = kInvalidObjectId;
  ObjectId target_port_id = kInvalidObjectId;
  ObjectId split_port_id = kInvalidObjectId;
};

struct GenerateSimpleLineResult {
  std::vector<ObjectId> pole_ids{};
  std::vector<ObjectId> span_ids{};
  std::uint64_t generation_session_id = 0;
};

struct GenerateBundleFromPathResult {
  ObjectId bundle_id = kInvalidObjectId;
  std::vector<ObjectId> bundle_ids{};
  std::vector<ObjectId> generated_span_ids{};
  std::vector<ObjectId> generated_pole_ids{};
};

struct PoleDetailInfo {
  const Pole* pole = nullptr;
  const PoleTypeDefinition* pole_type = nullptr;
  std::vector<const Port*> owned_ports{};
  std::vector<const Anchor*> owned_anchors{};
};

struct SplitSpanResult {
  ObjectId old_span_id = kInvalidObjectId;
  ObjectId new_port_id = kInvalidObjectId;
  ObjectId new_span_a_id = kInvalidObjectId;
  ObjectId new_span_b_id = kInvalidObjectId;
};

enum class PickBranchResolutionKind : std::uint8_t {
  kNode = 0,
  kMidair = 1,
};

struct ResolveBranchPickResult {
  PickBranchResolutionKind resolution = PickBranchResolutionKind::kNode;
  ObjectId resolved_node_id = kInvalidObjectId;
  SupportKind support_kind = SupportKind::kPole;
  Vec3d position{};
  bool snapped_from_segment_endpoint = false;
};

} // namespace wire::core
