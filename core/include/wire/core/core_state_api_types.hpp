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

struct ResolveBranchPickOptions {
  std::vector<BundleTemplateId> selected_bundle_template_ids{};
  double snap_radius_world = 0.6;
  bool create_midair_node = true;
  bool create_midair_node_set = false;
  bool enforce_midair_template_policy = true;
};

struct GenerationTiming {
  double prepare_ms = 0.0;
  double check_ms = 0.0;
  double pairs_ms = 0.0;
  double preflight_ms = 0.0;
  double intent_ms = 0.0;
  double support_groups_ms = 0.0;
  double emit_ms = 0.0;
  double save_graph_ms = 0.0;
  double rules_ms = 0.0;
  double layout_ms = 0.0;
  double geom_ms = 0.0;
  double draw_ms = 0.0;
  double total_ms = 0.0;
};

struct GenerateBundleFromPathResult {
  ObjectId bundle_id = kInvalidObjectId;
  std::vector<ObjectId> bundle_ids{};
  std::vector<ObjectId> generated_span_ids{};
  std::vector<ObjectId> generated_pole_ids{};
  GenerationTiming timing{};
};

struct BackboneFrontier {
  ObjectId pole_id = kInvalidObjectId;
  ObjectId span_id = kInvalidObjectId;
  ObjectId node_id = kInvalidObjectId;
  ObjectId edge_id = kInvalidObjectId;
  ObjectId edge_bundle_id = kInvalidObjectId;
  std::vector<ObjectId> node_ids{};
  std::vector<ObjectId> edge_ids{};
  std::vector<ObjectId> edge_bundle_ids{};
  std::vector<ObjectId> bundle_ids{};
  std::vector<ObjectId> span_ids{};
  std::vector<ObjectId> pole_ids{};
};

struct PoleDetailInfo {
  const Pole* pole = nullptr;
  const PoleTypeDefinition* pole_type = nullptr;
  std::vector<const Port*> owned_ports{};
  std::vector<const Anchor*> owned_anchors{};
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
