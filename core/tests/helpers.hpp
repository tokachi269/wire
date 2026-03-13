#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_state.hpp"

namespace helpers {

using wire::core::ConnectionCategory;
using wire::core::CoreState;
using wire::core::DirtyBits;
using wire::core::ObjectId;
using wire::core::PoleTypeId;

struct CoreCounts {
  std::size_t poles = 0;
  std::size_t ports = 0;
  std::size_t anchors = 0;
  std::size_t bundles = 0;
  std::size_t spans = 0;
  std::size_t attachments = 0;
};

struct BackbonePathGenerateOptions {
  wire::core::RoadSegment road{};
  double interval = 30.0;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  wire::core::BundleKind bundle_template_id = wire::core::BundleKind::kLowVoltage;
  int bundle_count = 0;
  bool override_allow_mirror = false;
  bool allow_mirror = true;
  wire::core::PathDirectionMode direction_mode = wire::core::PathDirectionMode::kAuto;
};

struct BackbonePathGenerateResult {
  std::vector<ObjectId> pole_ids{};
  std::vector<ObjectId> span_ids{};
  ObjectId bundle_id = wire::core::kInvalidObjectId;
  std::vector<wire::core::SegmentLaneAssignment> lane_assignments{};
  wire::core::PathDirectionEvaluationDebug direction_debug{};
  std::uint64_t generation_session_id = 0;
};

struct LaneOrderMetrics {
  int y_inversions = 0;
  int z_inversions = 0;
  int layer_jumps = 0;

  [[nodiscard]] int weighted_score() const { return y_inversions * 1000 + z_inversions * 600 + layer_jumps * 30; }
};

struct AxisRelationMetrics {
  wire::core::Vec3d row_axis{};
  wire::core::Vec3d support_forward_axis{};
  wire::core::Vec3d span_chord_axis{};
  double angle_row_vs_span_deg = 0.0;
  double angle_forward_vs_span_deg = 0.0;
  bool valid = false;
};

struct VisualSeparationMetrics {
  int port_count = 0;
  double min_port_spacing_m = 0.0;
  double min_endpoint_spacing_m = 0.0;
  double min_wire_spacing_near_start_m = 0.0;
  double min_wire_spacing_near_end_m = 0.0;
  double projected_min_spacing_topview_m = 0.0;
  double projected_mean_spacing_topview_m = 0.0;
  double visual_separation_score = 0.0;
  bool topology_distinct = false;
  bool visual_distinct = false;
};

struct BranchRunoutMetrics {
  double max_lateral_runout_m = 0.0;
  double lateral_runout_ratio = 0.0;
  double departure_lateral_offset_m = 0.0;
  double midspan_lateral_offset_m = 0.0;
  double support_departure_length_m = 0.0;
  double chord_length_m = 0.0;
  bool local_departure_dominates = false;
};

CoreCounts snapshot_counts(const CoreState& state);
bool same_counts(const CoreCounts& a, const CoreCounts& b);
bool regex_contains(const std::string& text, const std::string& pattern);
bool has_dirty(const wire::core::SpanRuntimeState* state, DirtyBits bits);
bool contains_id(const std::vector<ObjectId>& ids, ObjectId id);
bool almost_equal(double a, double b, double eps = 1e-9);
bool almost_equal(const wire::core::Vec3d& a, const wire::core::Vec3d& b, double eps = 1e-9);
wire::core::Vec3d normalize_xy_safe(const wire::core::Vec3d& v);
double dot_xy(const wire::core::Vec3d& a, const wire::core::Vec3d& b);
wire::core::Vec3d local_side_axis_from_yaw(double yaw_deg);
double angle_diff_abs_deg(double a, double b);
wire::core::Vec3d rotate_xy_by_yaw_test(const wire::core::Vec3d& local, double yaw_deg);
double effective_pole_yaw_deg_test(const wire::core::Pole& pole);
wire::core::Vec3d to_local_on_pole_test(const wire::core::Pole& pole, const wire::core::Vec3d& world);
bool aabb_valid(const wire::core::AABBd& aabb);
bool starts_with(const std::string& value, const std::string& prefix);
wire::core::ValidationResult validate_now(CoreState& state);
std::vector<PoleTypeId> sorted_pole_type_ids(const CoreState& state);
wire::core::EditResult<BackbonePathGenerateResult> generate_from_backbone_options(
    CoreState& state, const BackbonePathGenerateOptions& options);
LaneOrderMetrics compute_lane_order_metrics(const CoreState& state,
                                            const std::vector<wire::core::SegmentLaneAssignment>& assignments);
void dump_lane_assignment_debug(const CoreState& state,
                                const std::vector<wire::core::SegmentLaneAssignment>& assignments, const char* tag);
int count_lane_segment_xy_intersections(const CoreState& state,
                                        const std::vector<wire::core::SegmentLaneAssignment>& assignments);
int count_bundle_lane_polyline_xy_intersections(const CoreState& state,
                                                const std::vector<wire::core::SegmentLaneAssignment>& assignments);
int count_bundle_lane_adjacent_order_discontinuities(const CoreState& state,
                                                     const std::vector<wire::core::SegmentLaneAssignment>& assignments);
int count_mirrored_assignments(const std::vector<wire::core::SegmentLaneAssignment>& assignments);
const wire::core::JunctionInfo* find_junction(const wire::core::BackboneResult& backbone, ObjectId node_id);
const wire::core::SupportNode* find_support_node_by_point_index(const wire::core::BackboneResult& backbone,
                                                                int point_index);
ObjectId find_pole_id_by_position(const CoreState& state, const wire::core::Vec3d& pos, double eps = 1e-6);
bool is_monotonic(const std::vector<double>& values);
void add_backbone_bundle(wire::core::BackboneSpec& req, wire::core::BundleKind template_id,
                         wire::core::SpanLayer layer = wire::core::SpanLayer::kUnknown, int count = 0);
wire::core::BundleKind bundle_template_for_category_test(wire::core::ConnectionCategory category);
AxisRelationMetrics measure_pole_axis_relation_metrics(const CoreState& state, ObjectId pole_id,
                                                       wire::core::PortLayer layer,
                                                       const wire::core::Vec3d& span_axis);
VisualSeparationMetrics measure_lane_visual_separation_metrics(const CoreState& state,
                                                               const wire::core::SegmentLaneAssignment& assignment,
                                                               double sample_length_m = 0.35);
BranchRunoutMetrics measure_branch_runout_metrics(const CoreState& state, ObjectId span_id);
std::string describe_axis_relation_metrics(const AxisRelationMetrics& metrics);
std::string describe_visual_separation_metrics(const VisualSeparationMetrics& metrics);
std::string describe_branch_runout_metrics(const BranchRunoutMetrics& metrics);
wire::core::EditResult<wire::core::CoreState::AddConnectionByPoleResult>
add_connection_by_category(wire::core::CoreState& state, wire::core::ObjectId pole_a_id, wire::core::ObjectId pole_b_id,
                           wire::core::ConnectionCategory category,
                           wire::core::AddConnectionByPoleOptions options = {});
bool has_selected_slot_in_candidates(const wire::core::SlotSelectionDebugRecord& record);

template <typename T> std::vector<ObjectId> collect_sorted_ids(const std::vector<T>& items) {
  std::vector<ObjectId> ids;
  ids.reserve(items.size());
  for (const auto& item : items) {
    ids.push_back(item.id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

} // namespace helpers


