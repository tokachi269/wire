#pragma once

#include <cstdint>
#include <functional>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "wire/core/debug_types.hpp"
#include "wire/core/detail_curve.hpp"
#include "wire/core/entities.hpp"
#include "wire/core/id.hpp"
#include "wire/core/support_layout_types.hpp"
#include "wire/core/variation.hpp"

namespace wire::core {

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
  int curve_samples = 8;
  bool sag_enabled = false;
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
  kSupportArm = 0,
  kInsulator = 1,
  kFitting = 2,
};

struct VisualPart {
  VisualPartKind kind = VisualPartKind::kSupportArm;
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

struct VisualSettings {
  bool enable_support_structures = true;
  bool enable_insulators = true;
  double support_center_threshold_m = 0.03;
  double support_arm_extra_m = 0.20;
  double insulator_radius_m = 0.07;
  double insulator_length_m = 0.16;
};

struct CacheState {
  // Derived cache layer. Not treated as authoritative topology state.
  GeometrySettings geometry_settings{};
  CurveCache curve_cache{};
  BoundsCache bounds_cache{};
  VisualSettings visual_settings{};
  VariationSettings variation_settings{};
  SpanLayoutCache span_layout_cache{};
  VisualCache visual_cache{};
  RenderCache render_cache{};
};

struct TemplateDependencyState {
  // Topology-affecting template edits must not be applied as render-only cache updates.
  std::vector<ObjectId> bundles_requiring_regeneration{};
  std::vector<std::uint64_t> sessions_requiring_regeneration{};
};

inline constexpr double kDefaultCornerThresholdDeg = 70.0;

struct LayoutSettings {
  bool angle_correction_enabled = true;
  double corner_threshold_deg = kDefaultCornerThresholdDeg;
  double min_side_scale = 1.0;
  double max_side_scale = 1.8;
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

struct BackboneIndex {
  std::unordered_map<ObjectId, std::vector<ObjectId>> node_edges;
  std::unordered_map<BackboneEdgeKey, ObjectId, BackboneEdgeKeyHash> edge_by_nodes;
  std::unordered_map<ObjectId, std::vector<ObjectId>> edge_bundles;
  std::unordered_map<ObjectId, std::vector<ObjectId>> bundle_edge;
  std::unordered_map<ObjectId, std::vector<ObjectId>> edge_bundle_spans;
  std::unordered_map<ObjectId, ObjectId> span_edge_bundle;
  std::unordered_map<ObjectId, std::vector<std::size_t>> edge_bundle_ports;
  std::unordered_map<ObjectId, std::vector<std::size_t>> port_bindings_by_port;
  std::unordered_map<ObjectId, ObjectId> pole_node;
};

enum class DirtyBits : std::uint32_t {
  kNone = 0,
  kTopology = 1u << 0,
  kDecision = 1u << 1,
  kGeometryRefresh = 1u << 2,
  kBounds = 1u << 3,
  kRenderRefresh = 1u << 4,
  kRaycast = 1u << 5,
  kGeometry = (1u << 1) | (1u << 2),
  kRender = 1u << 4,
};

inline DirtyBits operator|(DirtyBits a, DirtyBits b) {
  using T = std::underlying_type_t<DirtyBits>;
  return static_cast<DirtyBits>(static_cast<T>(a) | static_cast<T>(b));
}

inline DirtyBits operator&(DirtyBits a, DirtyBits b) {
  using T = std::underlying_type_t<DirtyBits>;
  return static_cast<DirtyBits>(static_cast<T>(a) & static_cast<T>(b));
}

inline DirtyBits operator~(DirtyBits a) {
  using T = std::underlying_type_t<DirtyBits>;
  return static_cast<DirtyBits>(~static_cast<T>(a));
}

inline DirtyBits& operator|=(DirtyBits& a, DirtyBits b) {
  a = a | b;
  return a;
}

inline bool any(DirtyBits bits, DirtyBits flag) { return (bits & flag) != DirtyBits::kNone; }

struct SpanRuntimeState {
  // Derived runtime/version state for incremental recomputation.
  ObjectId span_id = kInvalidObjectId;
  std::uint64_t data_version = 0;
  std::uint64_t geometry_version = 0;
  std::uint64_t bounds_version = 0;
  std::uint64_t render_version = 0;
  std::uint64_t raycast_version = 0;
  std::uint64_t variation_flow_key = 0;
  DirtyBits dirty_bits = DirtyBits::kNone;
};

struct DirtyQueue {
  // Derived work queues. Content is rebuildable from edit operations.
  std::vector<ObjectId> topology_dirty_span_ids;
  std::vector<ObjectId> decision_dirty_span_ids;
  std::vector<ObjectId> geometry_dirty_span_ids;
  std::vector<ObjectId> bounds_dirty_span_ids;
  std::vector<ObjectId> render_dirty_span_ids;
  std::vector<ObjectId> raycast_dirty_span_ids;
};

struct RecalcStats {
  std::size_t topology_processed = 0;
  std::size_t decision_processed = 0;
  std::size_t geometry_processed = 0;
  std::size_t bounds_processed = 0;
  std::size_t render_processed = 0;
  std::size_t raycast_processed = 0;

  [[nodiscard]] std::size_t total_processed() const {
    return topology_processed + decision_processed + geometry_processed + bounds_processed + render_processed +
           raycast_processed;
  }
};

} // namespace wire::core
