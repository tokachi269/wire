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

wire::core::BackboneSpec line_req(wire::core::CoreState& state);
wire::core::BackboneSpec poly3_req(wire::core::CoreState& state);
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
wire::core::BackboneSpec pass_poly3_req(wire::core::CoreState& state);
std::vector<wire::core::Vec3d> pass_intent_points();
bool span_has_lowered_endpoint(const wire::core::CoreState& state, wire::core::ObjectId span_id);
std::vector<wire::core::Vec3d> junction_v1_points();

} // namespace backbone_tests
