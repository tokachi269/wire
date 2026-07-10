#include "section_behavior.hpp"

#include "../../geometry/detail_curve_postprocess.hpp"
#include "out.hpp"

#include <algorithm>
#include <cmath>

namespace wire::core::generation::backbone {
namespace {

bool finite_point(const Vec3d& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

AABBd bounds_for(const std::vector<Vec3d>& points) {
  AABBd out{};
  if (points.empty()) {
    return out;
  }
  out.min = points.front();
  out.max = points.front();
  for (const Vec3d& point : points) {
    out.min.x = std::min(out.min.x, point.x);
    out.min.y = std::min(out.min.y, point.y);
    out.min.z = std::min(out.min.z, point.z);
    out.max.x = std::max(out.max.x, point.x);
    out.max.y = std::max(out.max.y, point.y);
    out.max.z = std::max(out.max.z, point.z);
  }
  return out;
}

} // namespace

std::string validate_population_rule_behavior(const CablePopulationRule& rule) {
  switch (rule.profile) {
  case CableSectionProfile::kWrap:
    if (!std::isfinite(rule.wrap_radius_m) || rule.wrap_radius_m <= 1e-6 ||
        !std::isfinite(rule.wrap_turns_per_meter) || rule.wrap_turns_per_meter <= 1e-6 ||
        !std::isfinite(rule.wrap_phase) || (rule.wrap_direction != 1 && rule.wrap_direction != -1) ||
        !std::isfinite(rule.end_trim_m) || rule.end_trim_m < 0.0) {
      return "cable population: wrap rule requires positive radius and turns-per-meter, direction +1/-1, and non-negative end trim";
    }
    return {};
  case CableSectionProfile::kFree:
    if (std::abs(rule.wrap_radius_m) > 1e-12 || std::abs(rule.wrap_turns_per_meter) > 1e-12 ||
        std::abs(rule.wrap_phase) > 1e-12 || std::abs(rule.end_trim_m) > 1e-12) {
      return "cable population: only wrap rules may set wrap parameters";
    }
    return {};
  }
  return "cable population: unknown section profile";
}

std::vector<CableSectionLayout> make_behavior_sections(const CablePopulationInput& input, int requested_count) {
  std::vector<CableSectionLayout> out{};
  if (input.rule.profile != CableSectionProfile::kWrap) {
    return out;
  }
  constexpr double kTwoPi = 6.283185307179586;
  for (int index = 0; index < requested_count; ++index) {
    CableSectionLayout section{};
    section.key = input.key;
    section.key.instance_index = static_cast<std::size_t>(index) + 1;
    section.endpoint_a = input.endpoint_a_world;
    section.endpoint_b = input.endpoint_b_world;
    section.profile = input.rule.profile;
    section.wrap_radius_m = input.rule.wrap_radius_m;
    section.wrap_turns_per_meter = input.rule.wrap_turns_per_meter;
    section.wrap_phase = input.rule.wrap_phase + kTwoPi * static_cast<double>(index) /
                                                     static_cast<double>(std::max(1, requested_count));
    section.wrap_direction = input.rule.wrap_direction;
    section.end_trim_m = input.rule.end_trim_m;
    out.push_back(std::move(section));
  }
  return out;
}

bool participates_in_node_patch(const CableSectionLayout& section) {
  return section.profile != CableSectionProfile::kWrap;
}

EditResult<VisualCurvePart> build_behavior_curve_part(const behavior_curve_part_input& input) {
  EditResult<VisualCurvePart> out{};
  if (input.section.profile != CableSectionProfile::kWrap) {
    out.error = "section behavior: no behavior curve part";
    return out;
  }
  const double trim_m = std::max(0.0, input.section.end_trim_m);
  const std::vector<Vec3d> samples = sample_wrap_helix_points(
      input.carrier, trim_m, input.carrier.Length() - trim_m, input.section.wrap_radius_m,
      input.section.wrap_turns_per_meter, input.section.wrap_phase, static_cast<double>(input.section.wrap_direction));
  if (samples.size() < 2 || !finite_point(samples.front()) || !finite_point(samples.back())) {
    out.error = "wrap trim leaves no visible section";
    return out;
  }
  VisualCurvePart body{};
  body.kind = VisualCurvePartKind::kEdgeBody;
  body.source_edge_id = input.edge_id;
  body.source_span_id = input.section.key.logical_span_id;
  body.source_bundle_id = input.bundle_id;
  body.bundle_template_id = input.bundle_template_id;
  body.lane_index = input.lane_index;
  body.has_section_key = true;
  body.section_key = input.section.key;
  body.boundary_a = samples.front();
  body.boundary_b = samples.back();
  body.tangent_a = input.carrier.EvaluateTangent(input.carrier.LengthToU(trim_m));
  body.tangent_b = ScaleVec(input.carrier.EvaluateTangent(input.carrier.LengthToU(input.carrier.Length() - trim_m)), -1.0);
  body.sag_method = VisualCurveSagMethod::kNone;
  body.cable_run_id = input.cable_run_id;
  const SpanRenderCacheEntry appearance = render(input.state, input.section.key.logical_span_id, {});
  body.wire_radius_m = appearance.wire_radius_m;
  body.color_rgba = appearance.color_rgba;
  body.material_style = appearance.material_style;
  body.samples = samples;
  body.bounds = bounds_for(body.samples);
  body.source_version = input.source_version;
  out.value = std::move(body);
  out.ok = true;
  return out;
}

} // namespace wire::core::generation::backbone
