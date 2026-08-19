#pragma once

#include "../helpers.hpp"

#include "city/wire/core_state.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace backbone_tests {

struct CurveSnapshot {
  std::vector<city::wire::Vec3d> points{};
  std::vector<double> lengths{};
};

struct BoundsSnapshot {
  std::vector<city::wire::Vec3d> pts{};
};

struct SpanOutputSnapshot {
  city::wire::ObjectId span_id = city::wire::kInvalidObjectId;
  std::uint64_t data_version = 0;
  std::uint64_t layout_source_version = 0;
  city::wire::Vec3d layout_start_support{};
  city::wire::Vec3d layout_start_endpoint{};
  city::wire::Vec3d layout_end_support{};
  city::wire::Vec3d layout_end_endpoint{};
  std::uint64_t curve_source_version = 0;
  std::vector<city::wire::Vec3d> curve_samples{};
  std::uint64_t bounds_source_version = 0;
  city::wire::AABBd bounds{};
};

city::wire::BackboneSpec line_req(city::wire::CoreState& state);
city::wire::BackboneSpec poly3_req(city::wire::CoreState& state);
city::wire::ObjectId span_for_bundle(const city::wire::CoreState& state,
                                     const std::vector<city::wire::ObjectId>& span_ids,
                                     city::wire::BundleKind bundle_template_id);
int bundle_count(const city::wire::CoreState& state, city::wire::BundleKind id);
int req_bundle_count(const city::wire::CoreState& state, const city::wire::BackboneSpec& req);
int layer_rank(city::wire::SpanLayer layer);
double band_height(const city::wire::CoreState& state, city::wire::PoleTypeId pole_type_id,
                   city::wire::BundleKind bundle_kind);
bool span_ports_match_z(const city::wire::CoreState& state, city::wire::ObjectId span_id, double expected_z);
bool contains_text(const std::string& haystack, const std::string& needle);
std::filesystem::path repo_root();
bool file_text(const std::filesystem::path& path, std::string* out);
bool function_body(const std::string& source, const std::string& signature, std::string* out);
std::vector<city::wire::Vec3d> backbone_layout_points(city::wire::CoreState& state);
CurveSnapshot backbone_curve_points(city::wire::CoreState& state);
void push_box(const city::wire::AABBd& box, BoundsSnapshot* out);
BoundsSnapshot backbone_bounds_points(city::wire::CoreState& state);
std::vector<city::wire::Vec3d> poly3_points(city::wire::CoreState& state);
std::vector<city::wire::Vec3d> span_curve_points(city::wire::CoreState& state,
                                                const std::vector<city::wire::ObjectId>& spans);
std::vector<city::wire::Vec3d> existing_sequence_points(city::wire::CoreState& state);
std::vector<city::wire::Vec3d> pole_positions_for(city::wire::CoreState& state,
                                                 const city::wire::BackboneSpec& req);
std::vector<city::wire::Vec3d> offset_curve_points(double offset);
std::vector<city::wire::Vec3d> offset_points(
    double offset,
    city::wire::PathDirectionMode direction_mode =
        city::wire::PathDirectionMode::kForward);
std::vector<city::wire::Vec3d> node_mode_points(bool with_mode);
city::wire::BackboneInputSpec::NodeSpec pole_spec(std::size_t point_index, city::wire::ObjectId pole_id);
double dist2(const city::wire::Vec3d& a, const city::wire::Vec3d& b);
std::vector<city::wire::Vec3d> pole_port_positions(const city::wire::CoreState& state,
                                                  city::wire::ObjectId pole_id);
std::vector<city::wire::Vec3d> generated_ports_on_pole(const city::wire::CoreState& state,
                                                      const std::vector<city::wire::ObjectId>& spans,
                                                      city::wire::ObjectId pole_id);
bool separated_from(const std::vector<city::wire::Vec3d>& existing,
                    const std::vector<city::wire::Vec3d>& placed);
std::vector<city::wire::Vec3d> branch_separation_points();
city::wire::BackboneSpec pass_branch_req(city::wire::CoreState& state, city::wire::ObjectId pole_id,
                                         const city::wire::Vec3d& pole_pos);
city::wire::BackboneSpec hv_poly3_req(city::wire::CoreState& state);
city::wire::BackboneSpec hv_branch_req(city::wire::CoreState& state, city::wire::ObjectId pole_id,
                                       const city::wire::Vec3d& pole_pos);
std::vector<city::wire::ObjectId> lowering_branch_spans(city::wire::CoreState& state);
city::wire::BackboneSpec pass_poly3_req(city::wire::CoreState& state);
std::vector<city::wire::Vec3d> pass_intent_points();
bool span_has_lowered_endpoint(const city::wire::CoreState& state, city::wire::ObjectId span_id);
std::vector<city::wire::Vec3d> junction_v1_points();
std::vector<SpanOutputSnapshot> snapshot_span_outputs(const city::wire::CoreState& state,
                                                       const std::vector<city::wire::ObjectId>& span_ids);
bool same_span_output_snapshots(const std::vector<SpanOutputSnapshot>& before,
                                const city::wire::CoreState& state);
bool hv_edge_body_xy_intersections_absent(const city::wire::CoreState& state,
                                          std::string* reason);
bool backbone_common_invariants_pass(const city::wire::CoreState& state, std::string* reason);

} // namespace backbone_tests
