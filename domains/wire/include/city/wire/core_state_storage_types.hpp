#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "city/wire/core_authoritative_types.hpp"
#include "city/wire/core_state_api_types.hpp"
#include "city/wire/core_runtime_types.hpp"
#include "city/wire/core_state_internal_types.hpp"
#include "city/wire/debug_types.hpp"
#include "city/wire/id.hpp"

namespace city::wire {

// CoreState storage layout. Exposed by the current header shape, but not a stable consumer API.
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
  std::unordered_map<BundleTemplateId, BundleTemplate> bundle_templates{};
  std::unordered_map<AttachmentTemplateId, AttachmentTemplate> attachment_templates{};
  std::unordered_map<ModelAssemblyTemplateId, ModelAssemblyTemplate> model_assembly_templates{};
  ContextProfile context_profile{};
  OverrideState override_state{};
  LayoutSettings layout_settings{};
  GeometrySettings geometry_settings{};
  VisualSettings visual_settings{};
  VariationSettings variation_settings{};
};

struct CoreStateRuntimeStorage {
  ConnectionIndex connection_index{};
  RelationIndex relation_index{};
  BackboneIndex backbone_index{};
  std::unordered_map<ObjectId, SpanRuntimeState> span_runtime_states{};
  CacheState cache_state{};
};

struct CoreStateSessionStorage {
  std::vector<SupportNode> pending_support_nodes{};
  ObjectId next_virtual_support_node_id = 0x9000000000000000ull;
};

struct CoreStateDebugStorage {
  PathDirectionEvaluationDebug last_path_direction_debug{};
  std::vector<PathDirectionEvaluationDebug> path_direction_debug_records{};
  std::unordered_map<ObjectId, PoleOrientationDebugRecord> pole_orientation_debug_records{};
  std::vector<BackboneEdgeOrientation> last_generation_edge_orientations{};
  GenerationTiming last_generation_timing{};
  std::vector<PortResolutionDebugRecord> port_resolution_debug_records{};
  UpdateTiming last_update_timing{};
};

} // namespace city::wire
