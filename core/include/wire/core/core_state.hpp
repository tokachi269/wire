#pragma once

#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "wire/core/entities.hpp"
#include "wire/core/id.hpp"
#include "wire/core/object_store.hpp"
#include "wire/core/types.hpp"

namespace wire::core {

struct EditState {
  // Entity-layer authoritative stores.
  ObjectStore<Pole> poles;
  ObjectStore<Port> ports;
  ObjectStore<Anchor> anchors;
  ObjectStore<Bundle> bundles;
  ObjectStore<WireGroup> wire_groups;
  ObjectStore<WireLane> wire_lanes;
  ObjectStore<Span> spans;
  ObjectStore<Attachment> attachments;
};

struct GeometrySettings {
  // Persisted generation settings. Derived caches are rebuilt from these.
  int curve_samples = 8;
  bool sag_enabled = false;
  double sag_factor = 0.03;
};

struct CurveCacheEntry {
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

struct CacheState {
  // Derived cache layer. Not treated as authoritative topology state.
  GeometrySettings geometry_settings{};
  CurveCache curve_cache{};
  BoundsCache bounds_cache{};
};

struct LayoutSettings {
  bool angle_correction_enabled = true;
  double corner_threshold_deg = 12.0;
  double min_side_scale = 1.0;
  double max_side_scale = 1.8;
};

struct PathDirectionCostWeights {
  int estimated_cross_penalty = 100;
  int side_flip_penalty = 30;
  int layer_jump_penalty = 20;
  int corner_compression_penalty = 25;
  int branch_conflict_penalty = 15;
};

struct PathDirectionCostBreakdown {
  int estimated_cross_penalty = 0;
  int side_flip_penalty = 0;
  int layer_jump_penalty = 0;
  int corner_compression_penalty = 0;
  int branch_conflict_penalty = 0;
  int total = 0;
};

struct PathDirectionEvaluationDebug {
  RoadId road_id = 0;
  PathDirectionMode requested_mode = PathDirectionMode::kAuto;
  PathDirectionChosen chosen = PathDirectionChosen::kForward;
  PathDirectionCostBreakdown forward_cost{};
  PathDirectionCostBreakdown reverse_cost{};
  std::string reason{};
};

struct SegmentLaneAssignment {
  std::size_t segment_index = 0;
  ObjectId pole_a_id = kInvalidObjectId;
  ObjectId pole_b_id = kInvalidObjectId;
  ObjectId bundle_id = kInvalidObjectId;
  std::vector<int> slot_ids_a{};
  std::vector<int> slot_ids_b{};
  std::vector<ObjectId> port_ids_a{};
  std::vector<ObjectId> port_ids_b{};
  bool mirrored = false;
};

struct ConnectionIndex {
  std::unordered_map<ObjectId, std::vector<ObjectId>> spans_by_port;
  std::unordered_map<ObjectId, std::vector<ObjectId>> spans_by_anchor;
};

enum class DirtyBits : std::uint32_t {
  kNone = 0,
  kTopology = 1u << 0,
  kGeometry = 1u << 1,
  kBounds = 1u << 2,
  kRender = 1u << 3,
  kRaycast = 1u << 4,
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

inline bool any(DirtyBits bits, DirtyBits flag) {
  return (bits & flag) != DirtyBits::kNone;
}

struct SpanRuntimeState {
  // Derived runtime/version state for incremental recomputation.
  ObjectId span_id = kInvalidObjectId;
  std::uint64_t data_version = 0;
  std::uint64_t geometry_version = 0;
  std::uint64_t bounds_version = 0;
  std::uint64_t render_version = 0;
  std::uint64_t raycast_version = 0;
  DirtyBits dirty_bits = DirtyBits::kNone;
};

struct DirtyQueue {
  // Derived work queues. Content is rebuildable from edit operations.
  std::vector<ObjectId> topology_dirty_span_ids;
  std::vector<ObjectId> geometry_dirty_span_ids;
  std::vector<ObjectId> bounds_dirty_span_ids;
  std::vector<ObjectId> render_dirty_span_ids;
  std::vector<ObjectId> raycast_dirty_span_ids;
};

struct RecalcStats {
  std::size_t topology_processed = 0;
  std::size_t geometry_processed = 0;
  std::size_t bounds_processed = 0;
  std::size_t render_processed = 0;
  std::size_t raycast_processed = 0;

  [[nodiscard]] std::size_t total_processed() const {
    return topology_processed + geometry_processed + bounds_processed + render_processed + raycast_processed;
  }
};

struct ChangeSet {
  std::vector<ObjectId> created_ids;
  std::vector<ObjectId> updated_ids;
  std::vector<ObjectId> deleted_ids;
  std::vector<ObjectId> dirty_span_ids;
};

template <typename TValue>
struct EditResult {
  bool ok = false;
  TValue value{};
  std::string error{};
  ChangeSet change_set{};
};

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

struct SlotCandidateDebug {
  int slot_id = -1;
  bool eligible = false;
  int total_score = 0;
  int category_score = 0;
  int context_score = 0;
  int layer_score = 0;
  int side_score = 0;
  int role_score = 0;
  int priority_score = 0;
  int usage_score = 0;
  int congestion_score = 0;
  int tie_breaker = 0;
  std::size_t usage_count = 0;
  std::size_t congestion_count = 0;
  std::string reason{};
};

struct SlotSelectionDebugRecord {
  // Session diagnostics for slot selection; not part of entity model.
  ObjectId pole_id = kInvalidObjectId;
  ObjectId peer_pole_id = kInvalidObjectId;
  ObjectId reference_span_id = kInvalidObjectId;
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  ConnectionContext connection_context = ConnectionContext::kTrunkContinue;
  PoleContextKind pole_context = PoleContextKind::kStraight;
  double corner_angle_deg = 0.0;
  double corner_turn_sign = 0.0;
  double side_scale = 1.0;
  int selected_slot_id = -1;
  std::string result{};
  std::vector<SlotCandidateDebug> candidates{};
};

class CoreState {
 public:
  CoreState();

  struct AddConnectionByPoleOptions {
    SpanKind span_kind = SpanKind::kDistribution;
    SpanLayer span_layer = SpanLayer::kLowVoltage;
    ObjectId bundle_id = kInvalidObjectId;
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

  struct AddConnectionByPoleResult {
    ObjectId span_id = kInvalidObjectId;
    ObjectId port_a_id = kInvalidObjectId;
    ObjectId port_b_id = kInvalidObjectId;
    int slot_a_id = -1;
    int slot_b_id = -1;
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

  struct GenerateGroupedLineOptions {
    RoadSegment road{};
    double interval = 30.0;
    PoleTypeId pole_type_id = kInvalidPoleTypeId;
    ConductorGroupSpec group_spec{};
    PathDirectionMode direction_mode = PathDirectionMode::kAuto;
  };

  struct GenerateGroupedLineResult {
    std::vector<ObjectId> pole_ids{};
    std::vector<ObjectId> span_ids{};
    ObjectId bundle_id = kInvalidObjectId;
    ObjectId wire_group_id = kInvalidObjectId;
    std::vector<ObjectId> wire_lane_ids{};
    std::vector<SegmentLaneAssignment> lane_assignments{};
    PathDirectionEvaluationDebug direction_debug{};
    std::uint64_t generation_session_id = 0;
  };

  struct GenerateWireGroupFromPathInput {
    std::vector<Vec3d> polyline{};
    double interval_m = 0.0;
    PoleTypeId pole_type_id = kInvalidPoleTypeId;
    ConnectionCategory category = ConnectionCategory::kLowVoltage;
    PathDirectionMode direction_mode = PathDirectionMode::kAuto;
    int requested_lane_count = 0;  // 0: use category standard lanes.
  };

  struct GenerateWireGroupFromPathResult {
    ObjectId wire_group_id = kInvalidObjectId;
    std::vector<ObjectId> wire_lane_ids{};
    std::vector<ObjectId> generated_span_ids{};
    std::vector<ObjectId> generated_pole_ids{};
  };

  struct PoleDetailInfo {
    const Pole* pole = nullptr;
    const PoleTypeDefinition* pole_type = nullptr;
    std::vector<const Port*> owned_ports{};
    std::vector<const Anchor*> owned_anchors{};
  };

  EditResult<ObjectId> AddPole(
      const Transformd& world_transform,
      double height_m = 10.0,
      std::string_view name = {},
      PoleKind kind = PoleKind::kGeneric);
  EditResult<ObjectId> AddPort(
      ObjectId owner_pole_id,
      const Vec3d& world_position,
      PortKind kind = PortKind::kGeneric,
      PortLayer layer = PortLayer::kUnknown,
      const Frame3d& direction = {});
  EditResult<ObjectId> AddAnchor(
      ObjectId owner_pole_id,
      const Vec3d& world_position,
      AnchorSupportKind support_kind = AnchorSupportKind::kGeneric,
      double support_strength = 1.0);
  EditResult<ObjectId> AddBundle(
      int conductor_count,
      double phase_spacing_m,
      BundleKind kind = BundleKind::kLowVoltage);
  EditResult<ObjectId> AddWireGroup(
      WireGroupKind kind = WireGroupKind::kUnknown,
      std::string_view network_tag = {},
      std::string_view feeder_tag = {});
  EditResult<ObjectId> AddWireLane(
      ObjectId wire_group_id,
      int lane_index,
      WireLaneRole role = WireLaneRole::kUnknown);
  EditResult<ObjectId> AddSpan(
      ObjectId port_a_id,
      ObjectId port_b_id,
      SpanKind kind = SpanKind::kGeneric,
      SpanLayer layer = SpanLayer::kUnknown,
      ObjectId bundle_id = kInvalidObjectId,
      ObjectId anchor_a_id = kInvalidObjectId,
      ObjectId anchor_b_id = kInvalidObjectId);
  EditResult<ObjectId> AddAttachment(
      ObjectId span_id,
      double t,
      AttachmentKind kind = AttachmentKind::kGeneric,
      double offset_m = 0.0);
  EditResult<ObjectId> AssignSpanToWireLane(
      ObjectId span_id,
      ObjectId wire_group_id,
      ObjectId wire_lane_id);
  EditResult<ObjectId> MovePole(ObjectId pole_id, const Transformd& new_world_transform);
  EditResult<ObjectId> MovePort(ObjectId port_id, const Vec3d& new_world_position);
  EditResult<ObjectId> SetPortWorldPositionManual(ObjectId port_id, const Vec3d& new_world_position);
  EditResult<ObjectId> ResetPortPositionToAuto(ObjectId port_id);
  EditResult<ObjectId> MoveAnchor(ObjectId anchor_id, const Vec3d& new_world_position);
  EditResult<ObjectId> DeleteSpan(ObjectId span_id);

  struct SplitSpanResult {
    ObjectId old_span_id = kInvalidObjectId;
    ObjectId new_port_id = kInvalidObjectId;
    ObjectId new_span_a_id = kInvalidObjectId;
    ObjectId new_span_b_id = kInvalidObjectId;
  };
  EditResult<SplitSpanResult> SplitSpan(ObjectId span_id, double t);
  EditResult<ObjectId> ApplyPoleType(ObjectId pole_id, PoleTypeId pole_type_id);
  EditResult<AddConnectionByPoleResult> AddConnectionByPole(
      ObjectId pole_a_id,
      ObjectId pole_b_id,
      ConnectionCategory category,
      const AddConnectionByPoleOptions& options = {});
  EditResult<AddDropResult> AddDropFromPole(
      ObjectId source_pole_id,
      const Vec3d& target_world_position,
      ConnectionCategory category = ConnectionCategory::kDrop);
  EditResult<AddDropResult> AddDropFromSpan(
      ObjectId source_span_id,
      double t,
      const Vec3d& target_world_position,
      ConnectionCategory category = ConnectionCategory::kDrop);
  EditResult<std::vector<ObjectId>> GeneratePolesAlongRoad(
      const RoadSegment& road,
      double interval,
      PoleTypeId pole_type_id);
  EditResult<std::vector<ObjectId>> GenerateSpansBetweenPoles(
      const std::vector<ObjectId>& poles,
      ConnectionCategory category);
  EditResult<GenerateSimpleLineResult> GenerateSimpleLine(
      const RoadSegment& road,
      double interval,
      PoleTypeId pole_type_id,
      ConnectionCategory category);
  EditResult<GenerateSimpleLineResult> GenerateSimpleLineFromPoints(
      const RoadSegment& road,
      PoleTypeId pole_type_id,
      ConnectionCategory category);
  EditResult<GenerateGroupedLineResult> GenerateGroupedLine(
      const GenerateGroupedLineOptions& options);
  EditResult<GenerateWireGroupFromPathResult> GenerateWireGroupFromPath(
      const GenerateWireGroupFromPathInput& input);
  EditResult<ObjectId> SetPoleFlip180(ObjectId pole_id, bool flip_180);
  [[nodiscard]] PoleDetailInfo GetPoleDetail(ObjectId pole_id) const;
  [[nodiscard]] const WireGroup* GetWireGroup(ObjectId wire_group_id) const;
  [[nodiscard]] const WireLane* GetWireLane(ObjectId wire_lane_id) const;
  [[nodiscard]] std::vector<ObjectId> GetSpansByWireGroup(ObjectId wire_group_id) const;
  [[nodiscard]] std::vector<ObjectId> GetWireLanesByGroup(ObjectId wire_group_id) const;
  EditResult<bool> UpdateGeometrySettings(const GeometrySettings& settings, bool mark_all_spans_dirty = true);
  EditResult<bool> UpdateLayoutSettings(const LayoutSettings& settings);
  [[nodiscard]] const CurveCacheEntry* find_curve_cache(ObjectId span_id) const;
  [[nodiscard]] const BoundsCacheEntry* find_bounds_cache(ObjectId span_id) const;

  RecalcStats ProcessDirtyQueues();

  [[nodiscard]] ValidationResult Validate() const;

  [[nodiscard]] ObjectId next_id() const { return id_generator_.peek(); }

  [[nodiscard]] const EditState& edit_state() const { return edit_state_; }
  [[nodiscard]] EditState& edit_state() { return edit_state_; }

  [[nodiscard]] const ConnectionIndex& connection_index() const { return connection_index_; }
  [[nodiscard]] const std::unordered_map<ObjectId, SpanRuntimeState>& span_runtime_states() const { return span_runtime_states_; }
  [[nodiscard]] const SpanRuntimeState* find_span_runtime_state(ObjectId span_id) const;
  [[nodiscard]] const DirtyQueue& dirty_queue() const { return dirty_queue_; }
  [[nodiscard]] const RecalcStats& last_recalc_stats() const { return last_recalc_stats_; }
  [[nodiscard]] const std::unordered_map<PoleTypeId, PoleTypeDefinition>& pole_types() const { return pole_types_; }
  [[nodiscard]] const GeometrySettings& geometry_settings() const { return cache_state_.geometry_settings; }
  [[nodiscard]] const LayoutSettings& layout_settings() const { return layout_settings_; }
  [[nodiscard]] const PathDirectionCostWeights& path_direction_cost_weights() const { return path_direction_cost_weights_; }
  [[nodiscard]] const PathDirectionEvaluationDebug& last_path_direction_debug() const { return last_path_direction_debug_; }
  [[nodiscard]] const std::vector<PathDirectionEvaluationDebug>& path_direction_debug_records() const {
    return path_direction_debug_records_;
  }
  [[nodiscard]] const std::vector<SegmentLaneAssignment>& last_lane_assignments() const {
    return last_lane_assignments_;
  }
  void clear_path_direction_debug_records() { path_direction_debug_records_.clear(); }
  [[nodiscard]] const std::vector<SlotSelectionDebugRecord>& slot_selection_debug_records() const {
    return slot_selection_debug_records_;
  }
  void clear_slot_selection_debug_records() { slot_selection_debug_records_.clear(); }

  [[nodiscard]] const CacheState& cache_state() const { return cache_state_; }
  [[nodiscard]] CacheState& cache_state() { return cache_state_; }

  [[nodiscard]] const IdGenerator& id_generator() const { return id_generator_; }
  [[nodiscard]] IdGenerator& id_generator() { return id_generator_; }

 private:
  void remove_span_from_indexes(const Span& span);
  void add_span_to_index(const Span& span);
  void initialize_span_runtime_state(ObjectId span_id);
  void mark_span_dirty(ObjectId span_id, DirtyBits dirty_bits, bool bump_data_version);
  void add_dirty_queue(ObjectId span_id, DirtyBits dirty_bits);
  void mark_connected_spans_dirty_from_port(ObjectId port_id, DirtyBits dirty_bits, ChangeSet* change_set);
  void mark_connected_spans_dirty_from_anchor(ObjectId anchor_id, DirtyBits dirty_bits, ChangeSet* change_set);
  [[nodiscard]] bool rebuild_span_curve(ObjectId span_id, std::string* error_message);
  [[nodiscard]] bool rebuild_span_bounds(ObjectId span_id, std::string* error_message);
  [[nodiscard]] std::vector<Vec3d> generate_span_points(const Span& span, std::string* error_message) const;
  [[nodiscard]] static AABBd build_aabb_from_points(const std::vector<Vec3d>& points);
  [[nodiscard]] static AABBd build_aabb_from_two_points(const Vec3d& a, const Vec3d& b);
  void remove_span_from_caches(ObjectId span_id);
  [[nodiscard]] static std::vector<Vec3d> sample_polyline_points(const std::vector<Vec3d>& polyline, double interval);
  EditResult<std::vector<ObjectId>> generate_poles_from_points(
      const RoadSegment& road,
      PoleTypeId pole_type_id,
      const std::vector<Vec3d>& points);
  EditResult<std::vector<ObjectId>> generate_grouped_spans_between_poles(
      const std::vector<ObjectId>& poles,
      ObjectId bundle_id,
      const ConductorGroupSpec& group_spec,
      std::vector<SegmentLaneAssignment>* out_lane_assignments);
  [[nodiscard]] PathDirectionCostBreakdown evaluate_path_direction_cost(
      const std::vector<Vec3d>& points,
      const ConductorGroupSpec& group_spec) const;
  [[nodiscard]] static std::uint64_t hash_path_points(const std::vector<Vec3d>& points);
  [[nodiscard]] PathDirectionChosen choose_path_direction(
      const GenerateGroupedLineOptions& options,
      const std::vector<Vec3d>& sampled_points,
      PathDirectionEvaluationDebug* out_debug) const;
  [[nodiscard]] static double effective_pole_yaw_deg(const Pole& pole);
  [[nodiscard]] static Vec3d to_local_on_pole(const Pole& pole, const Vec3d& world);
  [[nodiscard]] static SlotSide preferred_side_from_geometry(
      const Pole& pole,
      const Pole* peer,
      double eps);
  [[nodiscard]] static double polyline_length(const std::vector<Vec3d>& polyline);
  [[nodiscard]] static PortLayer category_to_port_layer(ConnectionCategory category);
  [[nodiscard]] static SpanLayer category_to_span_layer(ConnectionCategory category);
  [[nodiscard]] static BundleKind category_to_bundle_kind(ConnectionCategory category);
  [[nodiscard]] static PortKind category_to_port_kind(ConnectionCategory category);
  [[nodiscard]] static int default_lane_count_for_category(ConnectionCategory category);
  [[nodiscard]] static bool is_supported_category(ConnectionCategory category);
  void register_default_pole_types();
  [[nodiscard]] const PoleTypeDefinition* find_pole_type(PoleTypeId pole_type_id) const;
  [[nodiscard]] std::vector<PortSlotTemplate> sorted_port_slots(const PoleTypeDefinition& pole_type, ConnectionCategory category) const;
  [[nodiscard]] bool is_port_slot_used(ObjectId pole_id, int slot_id) const;

  struct SlotSelectionRequest {
    ObjectId pole_id = kInvalidObjectId;
    ObjectId peer_pole_id = kInvalidObjectId;
    ObjectId reference_span_id = kInvalidObjectId;
    ConnectionCategory category = ConnectionCategory::kLowVoltage;
    ConnectionContext connection_context = ConnectionContext::kTrunkContinue;
    PoleContextKind pole_context = PoleContextKind::kStraight;
    double corner_angle_deg = 0.0;
    double corner_turn_sign = 0.0;
    bool allow_generate_port = true;
    int preferred_slot_id = -1;
    std::uint32_t branch_index = 0;
  };

  struct SlotSelectionResolved {
    ObjectId port_id = kInvalidObjectId;
    int slot_id = -1;
    ChangeSet change_set{};
  };

  EditResult<ObjectId> ensure_pole_slot_port(
      const SlotSelectionRequest& request,
      int* out_slot_id);
  [[nodiscard]] static std::uint8_t deterministic_tiebreak_0_255(
      ObjectId pole_id,
      int slot_id,
      ConnectionCategory category,
      ConnectionContext context,
      ObjectId peer_pole_id,
      ObjectId reference_span_id,
      std::uint32_t branch_index);
  [[nodiscard]] static bool is_valid_slot_side(SlotSide side);
  [[nodiscard]] static bool is_valid_slot_role(SlotRole role);
  [[nodiscard]] static int inversion_count(const std::vector<double>& values);
  [[nodiscard]] double compute_side_scale(PoleContextKind context, double corner_angle_deg) const;
  [[nodiscard]] static double compute_corner_angle_deg(const Vec3d& prev, const Vec3d& curr, const Vec3d& next);
  [[nodiscard]] static double compute_corner_turn_sign_xy(const Vec3d& prev, const Vec3d& curr, const Vec3d& next);
  [[nodiscard]] PoleContextInfo classify_pole_context_from_path(
      const std::vector<Vec3d>& points,
      std::size_t index,
      std::size_t pending_degree) const;
  EditResult<ObjectId> ensure_bundle_for_category(ConnectionCategory category, const AddConnectionByPoleOptions& options);
  static void add_unique_id(std::vector<ObjectId>& ids, ObjectId id);
  static std::string dirty_bits_to_string(DirtyBits bits);
  std::string next_display_id(std::string_view prefix);

  [[nodiscard]] static bool has_zero_length(const Port& a, const Port& b);
  [[nodiscard]] static std::unordered_map<ObjectId, std::vector<ObjectId>> make_expected_port_index(const EditState& edit_state);
  [[nodiscard]] static std::unordered_map<ObjectId, std::vector<ObjectId>> make_expected_anchor_index(const EditState& edit_state);

  IdGenerator id_generator_{};
  std::uint64_t next_data_version_ = 1;
  std::uint64_t next_generation_session_id_ = 1;
  std::unordered_map<std::string, std::uint64_t> display_id_counters_{};
  // PersistCore entity layer.
  EditState edit_state_{};
  ConnectionIndex connection_index_{};
  std::unordered_map<PoleTypeId, PoleTypeDefinition> pole_types_{};
  // Derived cache/runtime layer.
  std::unordered_map<ObjectId, SpanRuntimeState> span_runtime_states_{};
  DirtyQueue dirty_queue_{};
  RecalcStats last_recalc_stats_{};
  CacheState cache_state_{};
  // Persisted/authoritative generation policy layer.
  LayoutSettings layout_settings_{};
  PathDirectionCostWeights path_direction_cost_weights_{};
  // Session debug layer (non-authoritative, non-persist by policy).
  PathDirectionEvaluationDebug last_path_direction_debug_{};
  std::vector<PathDirectionEvaluationDebug> path_direction_debug_records_{};
  std::vector<SegmentLaneAssignment> last_lane_assignments_{};
  std::vector<SlotSelectionDebugRecord> slot_selection_debug_records_{};
};

CoreState make_demo_state();

}  // namespace wire::core
