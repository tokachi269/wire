#pragma once

#include <cstdint>
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

struct BackboneSpec {
  BackboneInputSpec path{};
  double interval_m = 0.0;
  PoleTypeId pole_type_id = kInvalidPoleTypeId;
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  BackboneGenerationConstraints constraints{};
  BackbonePolePlacementOptions pole_placement{};
  PathDirectionMode direction_mode = PathDirectionMode::kAuto;
  int requested_lane_count = 0;
};

// Compatibility aliases. Prefer Backbone* names for new code.
using GuidePath = BackboneInputSpec;
using GenerationConstraints = BackboneGenerationConstraints;
using GuidePolePlacementOptions = BackbonePolePlacementOptions;
using GenerationRequest = BackboneSpec;

// Workflow-layer grouped generation input.
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

// Derived backbone output (no generation-input policy mixed in this type).
struct BackboneResult {
  std::vector<BackboneEdge> edges{};
};

} // namespace wire::core
