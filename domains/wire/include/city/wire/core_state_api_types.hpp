#pragma once

#include <cstdint>
#include <vector>

#include "city/wire/core_edit_types.hpp"
#include "city/wire/core_runtime_types.hpp"
#include "city/wire/entities.hpp"
#include "city/wire/id.hpp"
#include "city/wire/workflow_types.hpp"

namespace city::wire {

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
  double state_copy_ms = 0.0;
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
  std::vector<ObjectId> generated_node_ids{};
  GenerationTiming timing{};
};

struct GenerateBackboneBundleVariationResult {
  ObjectId variation_id = kInvalidObjectId;
  GenerateBundleFromPathResult generation{};
};

struct DefaultBundlePlacementResult {
  double height_m = 0.0;
  double lateral_m = 0.0;
  double spacing_m = 0.0;
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

} // namespace city::wire
