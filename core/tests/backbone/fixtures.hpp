#pragma once

#include "../helpers.hpp"

#include "wire/core/core_state.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace backbone_tests {

struct CurveSnapshot {
  std::vector<wire::core::Vec3d> points{};
  std::vector<double> lengths{};
};

struct BoundsSnapshot {
  std::vector<wire::core::Vec3d> pts{};
};

struct SpanOutputSnapshot {
  wire::core::ObjectId span_id = wire::core::kInvalidObjectId;
  std::uint64_t data_version = 0;
  std::uint64_t layout_source_version = 0;
  wire::core::Vec3d layout_start_support{};
  wire::core::Vec3d layout_start_endpoint{};
  wire::core::Vec3d layout_end_support{};
  wire::core::Vec3d layout_end_endpoint{};
  std::uint64_t curve_source_version = 0;
  std::vector<wire::core::Vec3d> curve_samples{};
  std::uint64_t bounds_source_version = 0;
  wire::core::AABBd bounds{};
};

wire::core::BackboneSpec line_req(wire::core::CoreState& state);
wire::core::BackboneSpec poly3_req(wire::core::CoreState& state);
wire::core::ObjectId span_for_bundle(const wire::core::CoreState& state,
                                     const std::vector<wire::core::ObjectId>& span_ids,
                                     wire::core::BundleKind bundle_template_id);
int bundle_count(const wire::core::CoreState& state, wire::core::BundleKind id);
int req_bundle_count(const wire::core::CoreState& state, const wire::core::BackboneSpec& req);
int layer_rank(wire::core::SpanLayer layer);
double band_height(const wire::core::CoreState& state, wire::core::PoleTypeId pole_type_id,
                   wire::core::BundleKind bundle_kind);
bool span_ports_match_z(const wire::core::CoreState& state, wire::core::ObjectId span_id, double expected_z);
bool contains_text(const std::string& haystack, const std::string& needle);
std::filesystem::path repo_root();
bool file_text(const std::filesystem::path& path, std::string* out);
bool function_body(const std::string& source, const std::string& signature, std::string* out);
std::vector<wire::core::Vec3d> backbone_layout_points(wire::core::CoreState& state);
CurveSnapshot backbone_curve_points(wire::core::CoreState& state);
void push_box(const wire::core::AABBd& box, BoundsSnapshot* out);
BoundsSnapshot backbone_bounds_points(wire::core::CoreState& state);
std::vector<wire::core::Vec3d> poly3_points(wire::core::CoreState& state);
std::vector<wire::core::Vec3d> span_curve_points(wire::core::CoreState& state,
                                                const std::vector<wire::core::ObjectId>& spans);
std::vector<wire::core::Vec3d> existing_sequence_points(wire::core::CoreState& state);
std::vector<wire::core::Vec3d> pole_positions_for(wire::core::CoreState& state,
                                                 const wire::core::BackboneSpec& req);
std::vector<wire::core::Vec3d> offset_curve_points(double offset);
std::vector<wire::core::Vec3d> offset_points(double offset);
std::vector<wire::core::Vec3d> node_mode_points(bool with_mode);
wire::core::BackboneInputSpec::NodeSpec pole_spec(std::size_t point_index, wire::core::ObjectId pole_id);
double dist2(const wire::core::Vec3d& a, const wire::core::Vec3d& b);
std::vector<wire::core::Vec3d> pole_port_positions(const wire::core::CoreState& state,
                                                  wire::core::ObjectId pole_id);
std::vector<wire::core::Vec3d> generated_ports_on_pole(const wire::core::CoreState& state,
                                                      const std::vector<wire::core::ObjectId>& spans,
                                                      wire::core::ObjectId pole_id);
bool separated_from(const std::vector<wire::core::Vec3d>& existing,
                    const std::vector<wire::core::Vec3d>& placed);
std::vector<wire::core::Vec3d> branch_separation_points();
wire::core::BackboneSpec pass_branch_req(wire::core::CoreState& state, wire::core::ObjectId pole_id,
                                         const wire::core::Vec3d& pole_pos);
wire::core::BackboneSpec hv_poly3_req(wire::core::CoreState& state);
wire::core::BackboneSpec hv_branch_req(wire::core::CoreState& state, wire::core::ObjectId pole_id,
                                       const wire::core::Vec3d& pole_pos);
std::vector<wire::core::ObjectId> lowering_branch_spans(wire::core::CoreState& state);
wire::core::BackboneSpec pass_poly3_req(wire::core::CoreState& state);
std::vector<wire::core::Vec3d> pass_intent_points();
bool span_has_lowered_endpoint(const wire::core::CoreState& state, wire::core::ObjectId span_id);
std::vector<wire::core::Vec3d> junction_v1_points();
std::vector<SpanOutputSnapshot> snapshot_span_outputs(const wire::core::CoreState& state,
                                                       const std::vector<wire::core::ObjectId>& span_ids);
bool same_span_output_snapshots(const std::vector<SpanOutputSnapshot>& before,
                                const wire::core::CoreState& state);
bool backbone_common_invariants_pass(const wire::core::CoreState& state, std::string* reason);

} // namespace backbone_tests
