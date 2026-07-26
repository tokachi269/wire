#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "city/wire/core_state.hpp"
#include "city/wire/core_state_api_types.hpp"
#include "city/wire/core_test_hook.hpp"
#include "city/wire/core_view.hpp"

namespace helpers {

using city::wire::ConnectionCategory;
using city::wire::CoreState;
using city::wire::ObjectId;
using city::wire::PoleTypeId;

struct CoreCounts {
  std::size_t poles = 0;
  std::size_t ports = 0;
  std::size_t anchors = 0;
  std::size_t bundles = 0;
  std::size_t spans = 0;
  std::size_t attachments = 0;
};

struct BackboneFixture {
  city::wire::GenerateBundleFromPathResult generation{};
  std::vector<ObjectId> poles{};
  std::vector<ObjectId> nodes{};
  std::vector<ObjectId> spans{};
  std::vector<ObjectId> bundles{};
};

struct AxisRelationMetrics {
  city::wire::Vec3d row_axis{};
  city::wire::Vec3d support_forward_axis{};
  city::wire::Vec3d span_chord_axis{};
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
bool almost_equal(const city::wire::Vec3d& a, const city::wire::Vec3d& b, double eps = 1e-9);
city::wire::EditResult<BackboneFixture>
make_backbone_fixture(CoreState& state, const std::vector<city::wire::Vec3d>& points,
                 const std::vector<city::wire::BundleKind>& bundles = {city::wire::BundleKind::kLowVoltage});
city::wire::Vec3d normalize_xy_safe(const city::wire::Vec3d& v);
double dot_xy(const city::wire::Vec3d& a, const city::wire::Vec3d& b);
city::wire::Vec3d local_side_axis_from_yaw(double yaw_deg);
double angle_diff_abs_deg(double a, double b);
city::wire::Vec3d rotate_xy_by_yaw_test(const city::wire::Vec3d& local, double yaw_deg);
double effective_pole_yaw_deg_test(const city::wire::Pole& pole);
city::wire::Vec3d to_local_on_pole_test(const city::wire::Pole& pole, const city::wire::Vec3d& world);
bool aabb_valid(const city::wire::AABBd& aabb);
bool starts_with(const std::string& value, const std::string& prefix);
city::wire::ValidationResult validate_now(CoreState& state);
std::vector<PoleTypeId> sorted_pole_type_ids(const CoreState& state);
const city::wire::JunctionInfo* find_junction(const city::wire::BackboneResult& backbone, ObjectId node_id);
const city::wire::SupportNode* find_support_node_by_point_index(const city::wire::BackboneResult& backbone,
                                                                int point_index);
ObjectId find_pole_id_by_position(const CoreState& state, const city::wire::Vec3d& pos, double eps = 1e-6);
bool is_monotonic(const std::vector<double>& values);
void add_backbone_bundle(city::wire::BackboneSpec& req, city::wire::BundleKind template_id,
                         city::wire::SpanLayer layer = city::wire::SpanLayer::kUnknown, int count = 0);
city::wire::BundleTemplateId bundle_template_for_category_test(city::wire::ConnectionCategory category);
AxisRelationMetrics measure_pole_axis_relation_metrics(const CoreState& state, ObjectId pole_id,
                                                       city::wire::PortLayer layer,
                                                       const city::wire::Vec3d& span_axis);
BranchRunoutMetrics measure_branch_runout_metrics(const CoreState& state, ObjectId span_id);
std::string describe_axis_relation_metrics(const AxisRelationMetrics& metrics);
std::string describe_branch_runout_metrics(const BranchRunoutMetrics& metrics);
struct FixtureConnectionOptions {
  city::wire::BundleTemplateId bundle_template_id = city::wire::kDefaultLowVoltageBundleTemplateId;
  bool use_bundle_template = false;
  city::wire::ConnectionContext connection_context = city::wire::ConnectionContext::kTrunkContinue;
  std::uint32_t branch_index = 0;
};

struct FixtureConnectionResult {
  ObjectId span_id = city::wire::kInvalidObjectId;
  ObjectId port_a_id = city::wire::kInvalidObjectId;
  ObjectId port_b_id = city::wire::kInvalidObjectId;
};

city::wire::EditResult<FixtureConnectionResult>
add_connection_by_category(city::wire::CoreState& state, city::wire::ObjectId pole_a_id, city::wire::ObjectId pole_b_id,
                           city::wire::ConnectionCategory category,
                           FixtureConnectionOptions options = {});
bool has_selected_port_in_candidates(const city::wire::PortResolutionDebugRecord& record);
bool restore_capture_request_scene(const std::filesystem::path& capture_path, CoreState& state,
                                   city::wire::BackboneSpec* remapped_spec, std::string* error = nullptr);

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
