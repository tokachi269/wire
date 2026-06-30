#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "wire/core/core_state.hpp"
#include "wire/core/core_state_api_types.hpp"
#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_view.hpp"

namespace helpers {

using wire::core::ConnectionCategory;
using wire::core::CoreState;
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

struct AxisRelationMetrics {
  wire::core::Vec3d row_axis{};
  wire::core::Vec3d support_forward_axis{};
  wire::core::Vec3d span_chord_axis{};
  double angle_row_vs_span_deg = 0.0;
  double angle_forward_vs_span_deg = 0.0;
  bool valid = false;
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
BranchRunoutMetrics measure_branch_runout_metrics(const CoreState& state, ObjectId span_id);
std::string describe_axis_relation_metrics(const AxisRelationMetrics& metrics);
std::string describe_branch_runout_metrics(const BranchRunoutMetrics& metrics);
wire::core::EditResult<wire::core::AddConnectionByPoleResult>
add_connection_by_category(wire::core::CoreState& state, wire::core::ObjectId pole_a_id, wire::core::ObjectId pole_b_id,
                           wire::core::ConnectionCategory category,
                           wire::core::AddConnectionByPoleOptions options = {});
bool has_selected_port_in_candidates(const wire::core::PortResolutionDebugRecord& record);
bool restore_capture_request_scene(const std::filesystem::path& capture_path, CoreState& state,
                                   wire::core::BackboneSpec* remapped_spec, std::string* error = nullptr);

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
