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
  ObjectStore<Pole> poles;
  ObjectStore<Port> ports;
  ObjectStore<Anchor> anchors;
  ObjectStore<Bundle> bundles;
  ObjectStore<Span> spans;
  ObjectStore<Attachment> attachments;
};

struct CacheState {};

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
  ObjectId span_id = kInvalidObjectId;
  std::uint64_t data_version = 0;
  std::uint64_t geometry_version = 0;
  std::uint64_t bounds_version = 0;
  std::uint64_t render_version = 0;
  std::uint64_t raycast_version = 0;
  DirtyBits dirty_bits = DirtyBits::kNone;
};

struct DirtyQueue {
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

class CoreState {
 public:
  CoreState();

  struct AddConnectionByPoleOptions {
    SpanKind span_kind = SpanKind::kDistribution;
    SpanLayer span_layer = SpanLayer::kLowVoltage;
    ObjectId bundle_id = kInvalidObjectId;
    bool auto_create_bundle = true;
    bool allow_generate_port = true;
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
  EditResult<ObjectId> MovePole(ObjectId pole_id, const Transformd& new_world_transform);
  EditResult<ObjectId> MovePort(ObjectId port_id, const Vec3d& new_world_position);
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
  [[nodiscard]] PoleDetailInfo GetPoleDetail(ObjectId pole_id) const;

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
  [[nodiscard]] static PortLayer category_to_port_layer(ConnectionCategory category);
  [[nodiscard]] static SpanLayer category_to_span_layer(ConnectionCategory category);
  [[nodiscard]] static BundleKind category_to_bundle_kind(ConnectionCategory category);
  [[nodiscard]] static PortKind category_to_port_kind(ConnectionCategory category);
  void register_default_pole_types();
  [[nodiscard]] const PoleTypeDefinition* find_pole_type(PoleTypeId pole_type_id) const;
  [[nodiscard]] std::vector<PortSlotTemplate> sorted_port_slots(const PoleTypeDefinition& pole_type, ConnectionCategory category) const;
  [[nodiscard]] bool is_port_slot_used(ObjectId pole_id, int slot_id) const;
  EditResult<ObjectId> ensure_pole_slot_port(
      ObjectId pole_id,
      ConnectionCategory category,
      bool allow_generate_port,
      int* out_slot_id);
  EditResult<ObjectId> ensure_bundle_for_category(ConnectionCategory category, const AddConnectionByPoleOptions& options);
  static void add_unique_id(std::vector<ObjectId>& ids, ObjectId id);
  static std::string dirty_bits_to_string(DirtyBits bits);

  [[nodiscard]] static bool has_zero_length(const Port& a, const Port& b);
  [[nodiscard]] static std::unordered_map<ObjectId, std::vector<ObjectId>> make_expected_port_index(const EditState& edit_state);
  [[nodiscard]] static std::unordered_map<ObjectId, std::vector<ObjectId>> make_expected_anchor_index(const EditState& edit_state);

  IdGenerator id_generator_{};
  std::uint64_t next_data_version_ = 1;
  EditState edit_state_{};
  ConnectionIndex connection_index_{};
  std::unordered_map<PoleTypeId, PoleTypeDefinition> pole_types_{};
  std::unordered_map<ObjectId, SpanRuntimeState> span_runtime_states_{};
  DirtyQueue dirty_queue_{};
  RecalcStats last_recalc_stats_{};
  CacheState cache_state_{};
};

CoreState make_demo_state();

}  // namespace wire::core
