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

// Workflow input guide path (DrawPath/road adapters share this shape).
struct GuidePath {
  std::vector<Vec3d> polyline{};
};

struct GenerationConstraints {
  std::vector<Vec3d> avoid_points{};
  double avoid_radius_m = 0.0;
  double lateral_offset_m = 0.0;
};

struct GuidePolePlacementOptions {
  // Do not force manual poles from guide by default; users pin explicitly.
  bool pin_endpoints = false;
  bool pin_vertices = false;
  // Session-scoped regeneration can restrict pole reuse to the target session.
  bool restrict_reuse_to_session = false;
  std::uint64_t reuse_session_id = 0;
};

struct GenerationRequest {
  GuidePath path{};
  double interval_m = 0.0;
  PoleTypeId pole_type_id = kInvalidPoleTypeId;
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  GenerationConstraints constraints{};
  GuidePolePlacementOptions pole_placement{};
  PathDirectionMode direction_mode = PathDirectionMode::kAuto;
  int requested_lane_count = 0;
};

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

} // namespace wire::core

