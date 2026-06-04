#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "wire/core/core_authoritative_types.hpp"
#include "wire/core/core_runtime_types.hpp"
#include "wire/core/core_state_internal_types.hpp"
#include "wire/core/debug_types.hpp"
#include "wire/core/id.hpp"

namespace wire::core {

struct CoreStateIdentityStorage {
  IdGenerator id_generator{};
  std::uint64_t next_data_version = 1;
  std::uint64_t next_generation_session_id = 1;
  std::unordered_map<std::string, std::uint64_t> display_id_counters{};
};

struct CoreStateAuthoritativeStorage {
  EditState edit_state{};
  SavedBackboneGraph backbone{};
  std::unordered_map<PoleTypeId, PoleTypeDefinition> pole_types{};
  std::unordered_map<CableTemplateId, CableTemplate> cable_templates{};
  std::unordered_map<BundleKind, BundleTemplate> bundle_templates{};
  std::unordered_map<AttachmentTemplateId, AttachmentTemplate> attachment_templates{};
  ContextProfile context_profile{};
  TemplateDependencyState template_dependency_state{};
  OverrideState override_state{};
  LayoutSettings layout_settings{};
};

struct CoreStateRuntimeStorage {
  ConnectionIndex connection_index{};
  RelationIndex relation_index{};
  BackboneIndex backbone_index{};
  std::unordered_map<ObjectId, SpanRuntimeState> span_runtime_states{};
  DirtyQueue dirty_queue{};
  RecalcStats last_recalc_stats{};
  CacheState cache_state{};
};

struct CoreStateDebugStorage {
  PathDirectionEvaluationDebug last_path_direction_debug{};
  std::vector<PathDirectionEvaluationDebug> path_direction_debug_records{};
  std::unordered_map<ObjectId, PoleOrientationDebugRecord> pole_orientation_debug_records{};
  std::vector<SupportNode> last_generation_support_nodes{};
  std::vector<SegmentLaneAssignment> last_generation_lane_assignments{};
  std::vector<BackboneEdgeOrientation> last_generation_edge_orientations{};
  std::unordered_map<ObjectId, JunctionRelation> last_generation_junction_relations{};
  ObjectId next_virtual_support_node_id = 0x9000000000000000ull;
  std::vector<PortResolutionDebugRecord> port_resolution_debug_records{};
};

} // namespace wire::core
