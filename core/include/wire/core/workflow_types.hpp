#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "wire/core/entities.hpp"

namespace wire::core {

using RoadId = std::uint64_t;

// Workflow input polyline for generation commands (not an entity).
struct RoadSegment {
  RoadId id = 0;
  std::vector<Vec3d> polyline{};
};

// Workflow input path spec (DrawPath/road adapters share this shape).
struct BackboneInputSpec {
  std::vector<Vec3d> polyline{};
  // Optional per-path-point support-node metadata. Missing indices are treated as Pole.
  struct NodeSpec {
    std::size_t point_index = std::numeric_limits<std::size_t>::max();
    SupportKind support_kind = SupportKind::kPole;
    bool has_tangent_hint = false;
    Vec3d tangent_hint{};
  };
  std::vector<NodeSpec> node_specs{};
};

struct BackboneGenerationConstraints {
  std::vector<Vec3d> avoid_points{};
  double avoid_radius_m = 0.0;
  double lateral_offset_m = 0.0;
};

struct BackbonePolePlacementOptions {
  // Do not force manual poles from guide by default; users pin explicitly.
  bool pin_endpoints = false;
  bool pin_vertices = false;
  // Session-scoped regeneration can restrict pole reuse to the target session.
  bool restrict_reuse_to_session = false;
  std::uint64_t reuse_session_id = 0;
};

enum class BundleCountRuleKind : std::uint8_t {
  kFixed = 0,
  kRange = 1,
};

struct BundleTemplate {
  BundleKind id = BundleKind::kLowVoltage;
  std::string name{};
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  SpanLayer default_layer = SpanLayer::kLowVoltage;
  bool is_electric = false;
  // If true, conductor identity/order is treated as strict by higher-level workflow policy.
  bool preserve_conductor_identity = false;
  BundleCountRuleKind count_rule = BundleCountRuleKind::kFixed;
  int fixed_count = 1;
  int min_count = 1;
  int max_count = 1;
  int default_count = 1;
  double default_spacing_m = 0.2;
  bool allow_mirror = true;
  bool allow_midair_node = true;
  bool allow_midair_branch = true;
};

enum class BundleNodeMode : std::uint8_t {
  kNotPresent = 0,
  kPassThrough = 1,
  kBranch = 2,
  kTerminate = 3,
};

struct SupportNodeBundleMode {
  BundleKind bundle_template_id = BundleKind::kLowVoltage;
  BundleNodeMode mode = BundleNodeMode::kNotPresent;
};

struct SupportNode {
  ObjectId node_id = kInvalidObjectId;
  SupportKind support_kind = SupportKind::kPole;
  Vec3d position{};
  ObjectId pole_id = kInvalidObjectId;
  int path_point_index = -1;
  bool has_tangent_hint = false;
  Vec3d tangent_hint{};
  std::vector<SupportNodeBundleMode> bundle_modes{};
};

struct BackboneBundleSpec {
  BundleKind bundle_template_id = BundleKind::kLowVoltage;
  SpanLayer layer = SpanLayer::kUnknown;
  // Used only for variable-count templates. Fixed templates reject count input.
  int count = 0;
};

struct BackboneSpec {
  BackboneInputSpec path{};
  double interval_m = 0.0;
  PoleTypeId pole_type_id = kInvalidPoleTypeId;
  std::vector<BackboneBundleSpec> bundles{};
  // Optional per-node per-bundle connection mode hints.
  struct NodeBundleModeSpec {
    std::size_t point_index = std::numeric_limits<std::size_t>::max();
    BundleKind bundle_template_id = BundleKind::kLowVoltage;
    BundleNodeMode mode = BundleNodeMode::kNotPresent;
  };
  std::vector<NodeBundleModeSpec> node_bundle_modes{};
  BackboneGenerationConstraints constraints{};
  BackbonePolePlacementOptions pole_placement{};
  PathDirectionMode direction_mode = PathDirectionMode::kAuto;
};

using GuidePath = BackboneInputSpec;
using GenerationConstraints = BackboneGenerationConstraints;
using GuidePolePlacementOptions = BackbonePolePlacementOptions;

// Legacy grouped-generation input. New call sites should use BackboneSpec.
struct ConductorGroupSpec {
  ConnectionCategory category = ConnectionCategory::kHighVoltage;
  ConductorGroupKind group_kind = ConductorGroupKind::kSingle;
  int conductor_count = 1;
  double lane_spacing_m = 0.3;
  bool maintain_lane_order = true;
  bool allow_lane_mirror = true;
};

struct ConductorLaneId {
  int lane_index = 0;
};

struct ConductorGroupState {
  ObjectId bundle_id = kInvalidObjectId;
  ConductorGroupSpec spec{};
  std::vector<int> canonical_lane_order{};
};

struct JunctionIncident {
  ObjectId neighbor_node_id = kInvalidObjectId;
  int order = -1;
  bool primary = false;
  std::uint64_t source_session_id = 0;
};

struct JunctionInfo {
  ObjectId node_id = kInvalidObjectId;
  std::uint64_t prioritized_session_id = 0;
  bool used_neighbor_continuity = false;
  std::vector<JunctionIncident> incidents{};
};

enum class LaneOrientation : std::uint8_t {
  kNormal = 0,
  kReversed = 1,
};

enum class LaneFlipReason : std::uint8_t {
  kNone = 0,
  kAcuteTurn = 1,
};

struct BackboneEdgeOrientation {
  ObjectId node_a_id = kInvalidObjectId;
  ObjectId node_b_id = kInvalidObjectId;
  BundleKind bundle_template_id = BundleKind::kLowVoltage;
  LaneOrientation orientation = LaneOrientation::kNormal;
  bool flipped_from_previous = false;
  LaneFlipReason flip_reason = LaneFlipReason::kNone;
  double turn_angle_deg = 0.0;
};

// Derived backbone output (no generation-input policy mixed in this type).
struct BackboneResult {
  std::vector<SupportNode> nodes{};
  std::vector<BackboneEdge> edges{};
  std::vector<JunctionInfo> junctions{};
  std::vector<BackboneEdgeOrientation> edge_orientations{};
};

} // namespace wire::core
