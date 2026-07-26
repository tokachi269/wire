#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "city/wire/debug_types.hpp"
#include "city/wire/detail_curve.hpp"
#include "city/wire/entities.hpp"
#include "city/wire/id.hpp"
#include "city/wire/span_layout_types.hpp"
#include "city/wire/variation.hpp"

namespace city::wire {

// Runtime/cache types used by CoreState and CoreView. These are derived state, not source input.
using CableRunId = std::uint64_t;

struct AttachmentDebugLineEffect {
  ObjectId attachment_id = kInvalidObjectId;
  AttachmentTemplateId template_id = kInvalidAttachmentTemplateId;
  AttachmentLineInteractionMode interaction_mode = AttachmentLineInteractionMode::kPassThrough;
  double center_length_m = 0.0;
  CurveLengthInterval affected_interval{};
  bool uses_internal_path = false;
};

struct GeometrySettings {
  // Persisted generation settings. Derived caches are rebuilt from these.
  int curve_samples = 24;
  bool sag_enabled = true;
  double sag_factor = 0.03;
  // Pole skin clearance used to avoid center-line overlap with pole body.
  double pole_clearance_m = 0.05;
};

struct CurveCacheEntry {
  DetailCurve detail{};
  std::vector<Vec3d> points{};
  std::uint64_t source_version = 0;
};

struct CurveCache {
  std::unordered_map<ObjectId, CurveCacheEntry> by_span{};
};

struct BoundsCacheEntry {
  AABBd whole{};
  std::vector<AABBd> segments{};
  std::uint64_t source_version = 0;
};

struct BoundsCache {
  std::unordered_map<ObjectId, BoundsCacheEntry> by_span{};
};

enum class VisualPartKind : std::uint8_t {
  kInsulator = 0,
  kFitting = 1,
};

struct VisualPart {
  VisualPartKind kind = VisualPartKind::kInsulator;
  Vec3d a{};
  Vec3d b{};
  double radius_m = 0.02;
};

struct SpanVisualCacheEntry {
  std::vector<VisualPart> parts{};
  std::uint64_t source_version = 0;
};

struct VisualCache {
  std::unordered_map<ObjectId, SpanVisualCacheEntry> by_span{};
};

struct SpanRenderCacheEntry {
  // Render-only appearance data derived from templates + current span geometry.
  double wire_radius_m = 0.015;
  std::uint32_t color_rgba = 0xFFFFFFFFu;
  CableMaterialStyleKind material_style = CableMaterialStyleKind::kGeneric;
  std::vector<float> arc_length_m_by_point{};
  std::vector<float> arc_length_normalized_by_point{};
  std::vector<float> segment_length_m{};
  std::uint64_t source_version = 0;
};

struct RenderCache {
  std::unordered_map<ObjectId, SpanRenderCacheEntry> by_span{};
};

enum class VisualCurvePartKind : std::uint8_t {
  kEdgeBody = 0,
  kNodePatch = 1,
  kLead = 2,
  kJumper = 3,
  kSupplemental = 4,
};

enum class VisualSupplementalKind : std::uint8_t {
  kNone = 0,
  kSupportPath = 1,
  kHelix = 2,
};

enum class NodePatchClassification : std::uint8_t {
  kNone = 0,
  kTerminal = 1,
  kSimpleContinuous = 2,
  kMultiIncidentUnsupported = 3,
  kFixtureBoundaryUnsupported = 4,
};

enum class VisualCurveSagMethod : std::uint8_t {
  kNone = 0,
  kParabolic = 1,
};

struct CableSectionKey {
  ObjectId logical_span_id = kInvalidObjectId;
  ObjectId edge_bundle_id = kInvalidObjectId;
  std::uint64_t rule_owner_id = 0;
  CableSectionRuleId rule_id = 0;
  std::size_t instance_index = 0;

  [[nodiscard]] bool is_base() const { return rule_owner_id == 0 && rule_id == 0 && instance_index == 0; }
};

struct CableSectionLayout {
  CableSectionKey key{};
  Vec3d endpoint_a{};
  Vec3d endpoint_b{};
  PoleTypeId endpoint_a_pole_type_id = kInvalidPoleTypeId;
  PoleTypeId endpoint_b_pole_type_id = kInvalidPoleTypeId;
  int endpoint_a_band_id = 0;
  int endpoint_b_band_id = 0;
};

struct CablePopulationDiagnostic {
  ObjectId logical_span_id = kInvalidObjectId;
  ObjectId edge_bundle_id = kInvalidObjectId;
  CableSectionRuleId rule_id = 0;
  int extra_count_requested = 0;
  int extra_count_accepted = 0;
  int omitted_count = 0;
  std::string reason{};
};

struct VisualCurveDiagnostic {
  ObjectId source_node_id = kInvalidObjectId;
  ObjectId source_span_id = kInvalidObjectId;
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  std::size_t lane_index = 0;
  std::string reason{};
};

struct VisualCurvePart {
  VisualCurvePartKind kind = VisualCurvePartKind::kEdgeBody;
  VisualSupplementalKind supplemental_kind = VisualSupplementalKind::kNone;
  NodePatchClassification node_patch_classification = NodePatchClassification::kNone;
  ObjectId source_node_id = kInvalidObjectId;
  ObjectId source_edge_id = kInvalidObjectId;
  ObjectId source_span_id = kInvalidObjectId;
  ObjectId source_bundle_id = kInvalidObjectId;
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  std::size_t lane_index = 0;
  std::vector<ObjectId> incident_edge_ids{};
  Vec3d boundary_a{};
  Vec3d boundary_b{};
  Vec3d tangent_a{};
  Vec3d tangent_b{};
  Vec3d attachment_point{};
  bool has_attachment_point = false;
  bool passes_attachment_point = false;
  bool has_explicit_attachment_orientation = false;
  std::size_t section_count = 1;
  VisualCurveSagMethod sag_method = VisualCurveSagMethod::kNone;
  double sag_m = 0.0;
  bool has_section_key = false;
  CableSectionKey section_key{};
  CableRunId cable_run_id = 0;
  PoleTypeId endpoint_a_pole_type_id = kInvalidPoleTypeId;
  PoleTypeId endpoint_b_pole_type_id = kInvalidPoleTypeId;
  int endpoint_a_band_id = 0;
  int endpoint_b_band_id = 0;
  double wire_radius_m = 0.015;
  std::uint32_t color_rgba = 0xFFFFFFFFu;
  CableMaterialStyleKind material_style = CableMaterialStyleKind::kGeneric;
  std::vector<Vec3d> bezier_control_points{};
  std::vector<Vec3d> samples{};
  AABBd bounds{};
  std::uint64_t source_version = 0;
};

struct VisualCurvePartStats {
  // Derived-cache diagnostics only; not authoritative topology or geometry state.
  std::size_t curve_builds = 0;
  std::size_t sections = 0;
};

struct VisualCurvePartCache {
  std::vector<VisualCurvePart> parts{};
  std::vector<VisualCurveDiagnostic> diagnostics{};
  std::vector<CablePopulationDiagnostic> population_diagnostics{};
  VisualCurvePartStats stats{};
};

struct VisualModelInstance {
  std::string stable_key{};
  std::string model_key{};
  std::uint64_t content_version = 0;
  Transformd world_transform{};

  bool operator==(const VisualModelInstance& other) const {
    return stable_key == other.stable_key && model_key == other.model_key &&
           content_version == other.content_version &&
           world_transform.position.x == other.world_transform.position.x &&
           world_transform.position.y == other.world_transform.position.y &&
           world_transform.position.z == other.world_transform.position.z &&
           world_transform.rotation_euler_deg.x == other.world_transform.rotation_euler_deg.x &&
           world_transform.rotation_euler_deg.y == other.world_transform.rotation_euler_deg.y &&
           world_transform.rotation_euler_deg.z == other.world_transform.rotation_euler_deg.z &&
           world_transform.scale.x == other.world_transform.scale.x &&
           world_transform.scale.y == other.world_transform.scale.y &&
           world_transform.scale.z == other.world_transform.scale.z;
  }
};

struct VisualModelInstanceCache {
  std::vector<VisualModelInstance> instances{};
};

struct VisualSettings {
  bool enable_insulators = true;
  double insulator_radius_m = 0.07;
  double insulator_length_m = 0.16;
};

struct CacheState {
  // Derived cache layer. Not treated as authoritative topology state.
  CurveCache curve_cache{};
  BoundsCache bounds_cache{};
  SpanLayoutCache span_layout_cache{};
  VisualCache visual_cache{};
  RenderCache render_cache{};
  VisualCurvePartCache visual_curve_part_cache{};
  VisualModelInstanceCache visual_model_instance_cache{};
};

inline constexpr double kDefaultCornerThresholdDeg = 70.0;
inline constexpr double kMaxCornerSideScale = 1.7;

struct LayoutSettings {
  bool angle_correction_enabled = true;
  double corner_threshold_deg = kDefaultCornerThresholdDeg;
  double min_side_scale = 1.0;
  double max_side_scale = kMaxCornerSideScale;
};

struct ConnectionIndex {
  std::unordered_map<ObjectId, std::vector<ObjectId>> spans_by_port;
  std::unordered_map<ObjectId, std::vector<ObjectId>> spans_by_anchor;
};

struct RelationIndex {
  std::unordered_map<ObjectId, std::vector<ObjectId>> ports_by_pole;
  std::unordered_map<ObjectId, std::vector<ObjectId>> anchors_by_pole;
  std::unordered_map<ObjectId, std::vector<ObjectId>> spans_by_bundle;
  std::unordered_map<ObjectId, std::vector<ObjectId>> attachments_by_span;
};

struct BackboneEdgeKey {
  ObjectId a = kInvalidObjectId;
  ObjectId b = kInvalidObjectId;
  bool operator==(const BackboneEdgeKey& other) const { return a == other.a && b == other.b; }
};

struct BackboneEdgeKeyHash {
  std::size_t operator()(const BackboneEdgeKey& key) const {
    const std::size_t h1 = std::hash<ObjectId>{}(key.a);
    const std::size_t h2 = std::hash<ObjectId>{}(key.b);
    return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
  }
};

struct BackboneEdgeBundleKey {
  ObjectId edge_id = kInvalidObjectId;
  ObjectId bundle_id = kInvalidObjectId;
  bool operator==(const BackboneEdgeBundleKey& other) const {
    return edge_id == other.edge_id && bundle_id == other.bundle_id;
  }
};

struct BackboneEdgeBundleKeyHash {
  std::size_t operator()(const BackboneEdgeBundleKey& key) const {
    const std::size_t h1 = std::hash<ObjectId>{}(key.edge_id);
    const std::size_t h2 = std::hash<ObjectId>{}(key.bundle_id);
    return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
  }
};

struct BackboneIndex {
  std::unordered_map<ObjectId, std::vector<ObjectId>> node_edges;
  std::unordered_map<BackboneEdgeKey, ObjectId, BackboneEdgeKeyHash> edge_by_nodes;
  std::unordered_map<ObjectId, std::vector<ObjectId>> edge_bundles;
  std::unordered_map<BackboneEdgeBundleKey, ObjectId, BackboneEdgeBundleKeyHash>
      edge_bundle_by_edge_and_bundle;
  std::unordered_map<ObjectId, std::size_t> edge_bundle_positions;
  std::unordered_map<ObjectId, std::vector<ObjectId>> bundle_edge;
  std::unordered_map<ObjectId, std::vector<ObjectId>> edge_bundle_spans;
  std::unordered_map<ObjectId, ObjectId> span_edge_bundle;
  std::unordered_map<ObjectId, std::vector<std::size_t>> edge_bundle_span_bindings;
  std::unordered_map<ObjectId, std::vector<std::size_t>> span_bindings_by_span;
  std::unordered_map<ObjectId, std::vector<std::size_t>> edge_bundle_ports;
  std::unordered_map<ObjectId, std::vector<std::size_t>> port_bindings_by_port;
  std::unordered_map<ObjectId, ObjectId> pole_node;
};

enum class UpdateKind : std::uint8_t {
  kRegenerate = 0,
  kReposition = 1,
  kReshape = 2,
  kRedraw = 3,
};

enum class UpdateTargetKind : std::uint8_t {
  kUnknown = 0,
  kPole = 1,
  kPort = 2,
  kSpan = 3,
  kAllSpans = 4,
};

struct UpdateRequest {
  UpdateKind kind = UpdateKind::kRegenerate;
  UpdateTargetKind target_kind = UpdateTargetKind::kUnknown;
  ObjectId target_id = kInvalidObjectId;
};

struct AffectedSet {
  std::vector<ObjectId> poles{};
  std::vector<ObjectId> ports{};
  std::vector<ObjectId> spans{};
  std::vector<ObjectId> edges{};
};

struct UpdatePlan {
  UpdateKind kind = UpdateKind::kRegenerate;
  AffectedSet affected{};
  double plan_ms = 0.0;
};

struct UpdateTiming {
  UpdateKind kind = UpdateKind::kRegenerate;
  std::size_t affected_span_count = 0;
  double plan_ms = 0.0;
  double derive_ms = 0.0;
  double total_ms = 0.0;
};

struct SpanRuntimeState {
  // Derived runtime/version state for incremental recomputation.
  ObjectId span_id = kInvalidObjectId;
  std::uint64_t data_version = 0;
  std::uint64_t geometry_version = 0;
  std::uint64_t bounds_version = 0;
  std::uint64_t render_version = 0;
  std::uint64_t raycast_version = 0;
  std::uint64_t variation_flow_key = 0;
};

} // namespace city::wire
