#include "out.hpp"

#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "../../geometry/curve/curve.hpp"
#include "../../geometry/detail_curve_postprocess.hpp"

#include <algorithm>

namespace wire::core::generation::backbone {
namespace {

AABBd box(const Vec3d& a, const Vec3d& b) {
  AABBd out{};
  out.min = {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
  out.max = {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
  return out;
}

AABBd box(const std::vector<Vec3d>& pts) {
  AABBd out{};
  if (pts.empty()) {
    return out;
  }
  out.min = pts.front();
  out.max = pts.front();
  for (const Vec3d& p : pts) {
    out.min.x = std::min(out.min.x, p.x);
    out.min.y = std::min(out.min.y, p.y);
    out.min.z = std::min(out.min.z, p.z);
    out.max.x = std::max(out.max.x, p.x);
    out.max.y = std::max(out.max.y, p.y);
    out.max.z = std::max(out.max.z, p.z);
  }
  return out;
}

struct curve_input_data {
  geometry::curve::CableCurveInput input{};
  CableContinuityPolicyHint continuity_policy = CableContinuityPolicyHint::kAuto;
  double bend_stiffness_hint = 1.0;
  double min_bend_radius_hint_m = 0.0;
  double chord_length = 0.0;
};

curve_input_data make_curve_input_data(const CoreState& state, ObjectId span_id, const Vec3d& start,
                                       const Vec3d& end, const Vec3d* start_tangent_hint,
                                       const Vec3d* end_tangent_hint) {
  const GeometrySettings& settings = state.view().geometry_settings();

  double sag_ratio = settings.sag_factor;
  double radius_m = 0.0;
  curve_input_data data{};
  const Span* span = state.view().spans().find(span_id);
  if (span != nullptr) {
    const Bundle* bundle = state.view().bundles().find(span->bundle_id);
    if (bundle != nullptr) {
      const auto bundle_template = state.view().bundle_templates().find(bundle->bundle_template_id);
      if (bundle_template != state.view().bundle_templates().end()) {
        const auto cable = state.view().cable_templates().find(bundle_template->second.cable_template_id);
        if (cable != state.view().cable_templates().end()) {
          sag_ratio = cable->second.sag_factor + cable->second.slack_factor;
          radius_m = std::max(0.0, cable->second.outer_diameter_m * 0.5);
          data.continuity_policy = cable->second.continuity_policy;
          data.bend_stiffness_hint = cable->second.bend_stiffness;
          data.min_bend_radius_hint_m = cable->second.min_bend_radius_m;
        }
      }
    }
  }
  sag_ratio = std::max(0.0, sag_ratio);

  const Vec3d chord = end - start;
  data.chord_length = Length(chord);
  const Vec3d tangent = data.chord_length > 1e-9 ? ScaleVec(chord, 1.0 / data.chord_length) : Vec3d{1.0, 0.0, 0.0};
  geometry::curve::CableCurveInput& input = data.input;
  input.start = start;
  input.end = end;
  input.start_tangent_hint = start_tangent_hint == nullptr ? tangent : *start_tangent_hint;
  input.end_tangent_hint = end_tangent_hint == nullptr ? tangent : *end_tangent_hint;
  input.has_start_tangent_hint = start_tangent_hint != nullptr;
  input.has_end_tangent_hint = end_tangent_hint != nullptr;
  input.gravity_dir = {0.0, 0.0, -1.0};
  input.canonical_dir = tangent;
  const BackboneFrontier frontier = state.view().span_frontier(span_id);
  if (const SavedBackboneEdge* edge = state.view().backbone_edge(frontier.edge_id);
      edge != nullptr && Length(edge->dir) > 1e-9) {
    input.canonical_dir = edge->dir;
  }
  input.sag_m = settings.sag_enabled ? sag_ratio * data.chord_length : 0.0;
  input.radius_m = radius_m;
  input.family = geometry::curve::CurveFamily::kMainSpan;
  input.method = geometry::curve::CurveMethod::kParabolicSag;
  input.tessellation.min_segments =
      std::max(input.tessellation.min_segments, static_cast<std::size_t>(std::max(2, settings.curve_samples) - 1));
  return data;
}

CurveConstraint detail_curve_constraint(const Vec3d& point, const Vec3d& tangent, const GeometrySettings& settings,
                                        double sag_ratio, CableContinuityPolicyHint continuity_policy,
                                        double bend_stiffness_hint, double min_bend_radius_hint_m,
                                        double chord_length) {
  CurveConstraint constraint{};
  constraint.point = point;
  constraint.tangent_dir = tangent;
  constraint.tangent_length_hint_m = std::max(0.0, chord_length / 3.0);
  constraint.sag_hint = settings.sag_enabled ? sag_ratio : 0.0;
  constraint.continuity_preference = continuity_policy;
  constraint.bend_stiffness_hint = bend_stiffness_hint;
  constraint.min_bend_radius_hint_m = min_bend_radius_hint_m;
  return constraint;
}

} // namespace

const SavedBackboneSpanBinding* source_span_binding_for(const CoreState& state,
                                                        const SourceEdgeProjectionRef& ref) {
  const CoreView& view = state.view();
  const SavedBackboneEdge* edge = view.backbone_edge(ref.source_edge_id);
  if (edge == nullptr || (ref.from_node_id != edge->node_a && ref.from_node_id != edge->node_b)) {
    return nullptr;
  }
  const SavedBackboneEdgeBundle* matched = view.backbone_edge_bundle(ref.source_edge_bundle_id);
  if (matched == nullptr || matched->edge_id != ref.source_edge_id) {
    return nullptr;
  }

  const SavedBackbonePortBinding* binding_a = nullptr;
  const SavedBackbonePortBinding* binding_b = nullptr;
  for (const SavedBackbonePortBinding* binding : view.backbone_port_bindings_for_edge_bundle(matched->edge_bundle_id)) {
    if (binding == nullptr || binding->lane_index != ref.lane_index) {
      continue;
    }
    const bool at_a = binding->row_key.node_id == edge->node_a;
    const bool at_b = binding->row_key.node_id == edge->node_b;
    if ((!at_a && !at_b) || (at_a && binding_a != nullptr) || (at_b && binding_b != nullptr)) {
      return nullptr;
    }
    (at_a ? binding_a : binding_b) = binding;
  }
  if (binding_a == nullptr || binding_b == nullptr) {
    return nullptr;
  }

  const auto span_bindings_it = view.backbone_index().edge_bundle_span_bindings.find(matched->edge_bundle_id);
  if (span_bindings_it == view.backbone_index().edge_bundle_span_bindings.end()) {
    return nullptr;
  }
  const SavedBackboneSpanBinding* span_binding = nullptr;
  for (std::size_t index : span_bindings_it->second) {
    if (index >= view.backbone().span_bindings.size()) {
      return nullptr;
    }
    const SavedBackboneSpanBinding& candidate = view.backbone().span_bindings[index];
    if (candidate.lane_index != ref.lane_index) {
      continue;
    }
    if (span_binding != nullptr) {
      return nullptr;
    }
    span_binding = &candidate;
  }
  return span_binding;
}

namespace {

EditResult<DetailCurve> make_primary_curve_between_impl(const CoreState& state, ObjectId span_id, const Vec3d& start,
                                                        const Vec3d& end, const Vec3d* start_tangent_hint,
                                                        const Vec3d* end_tangent_hint) {
  EditResult<DetailCurve> result{};
  const GeometrySettings& settings = state.view().geometry_settings();
  const curve_input_data data = make_curve_input_data(state, span_id, start, end, start_tangent_hint, end_tangent_hint);
  if (data.continuity_policy != CableContinuityPolicyHint::kAuto) {
    double sag_ratio = settings.sag_factor;
    const Span* span = state.view().spans().find(span_id);
    if (span != nullptr) {
      const Bundle* bundle = state.view().bundles().find(span->bundle_id);
      if (bundle != nullptr) {
        const auto bundle_template = state.view().bundle_templates().find(bundle->bundle_template_id);
        if (bundle_template != state.view().bundle_templates().end()) {
          const auto cable = state.view().cable_templates().find(bundle_template->second.cable_template_id);
          if (cable != state.view().cable_templates().end()) {
            sag_ratio = cable->second.sag_factor + cable->second.slack_factor;
          }
        }
      }
    }
    sag_ratio = std::max(0.0, sag_ratio);
    const CurveConstraint start_constraint =
        detail_curve_constraint(start, data.input.start_tangent_hint, settings, sag_ratio, data.continuity_policy,
                                data.bend_stiffness_hint, data.min_bend_radius_hint_m, data.chord_length);
    CurveConstraint end_constraint =
        detail_curve_constraint(end, data.input.end_tangent_hint, settings, sag_ratio, data.continuity_policy,
                                data.bend_stiffness_hint, data.min_bend_radius_hint_m, data.chord_length);
    result.value = BuildDetailCurve(start_constraint, end_constraint, std::max(2, settings.curve_samples));
  } else {
    const EditResult<geometry::curve::CableCurveOutput> built = geometry::curve::BuildCableCurve(data.input);
    if (!built.ok) {
      result.error = built.error;
      return result;
    }
    result.value = geometry::curve::ToDetailCurve(data.input, built.value);
  }
  result.ok = true;
  return result;
}

} // namespace

EditResult<DetailCurve> make_curve_between(const CoreState& state, ObjectId span_id, const Vec3d& start,
                                          const Vec3d& end) {
  EditResult<DetailCurve> result = make_primary_curve_between_impl(state, span_id, start, end, nullptr, nullptr);
  if (result.ok) {
    apply_attachment_line_effects_to_curve(state, span_id, &result.value);
  }
  return result;
}

EditResult<DetailCurve> make_primary_curve_between(const CoreState& state, ObjectId span_id, const Vec3d& start,
                                                    const Vec3d& end, const Vec3d* start_tangent,
                                                    const Vec3d* end_tangent) {
  return make_primary_curve_between_impl(state, span_id, start, end, start_tangent, end_tangent);
}

EditResult<DetailCurve> make_curve(const CoreState& state, ObjectId span_id, const SpanLayoutEntry& layout) {
  return make_curve_between(state, span_id, layout.start.endpoint_world, layout.end.endpoint_world);
}

std::optional<Vec3d> source_edge_projection_world(const CoreState& state, const SourceEdgeProjectionRef& ref) {
  if (!ref.valid()) {
    return std::nullopt;
  }
  const SavedBackboneEdge* edge = state.view().backbone_edge(ref.source_edge_id);
  const SavedBackboneSpanBinding* binding = source_span_binding_for(state, ref);
  if (edge == nullptr || binding == nullptr) {
    return std::nullopt;
  }
  if (const CurveCacheEntry* cached = state.find_curve_cache(binding->span_id);
      cached != nullptr && cached->detail.sample_points.size() >= 2) {
    const double t = std::clamp(ref.t, 0.0, 1.0);
    const double u = ref.from_node_id == edge->node_a ? t : 1.0 - t;
    return cached->detail.EvaluatePosition(u);
  }
  const SpanLayoutView layout = state.span_layout(binding->span_id);
  if (!layout.has_layout()) {
    return std::nullopt;
  }
  EditResult<DetailCurve> curve = make_curve(state, binding->span_id, *layout.entry);
  if (!curve.ok || curve.value.sample_points.size() < 2) {
    return std::nullopt;
  }

  const double t = std::clamp(ref.t, 0.0, 1.0);
  const double u = ref.from_node_id == edge->node_a ? t : 1.0 - t;
  return curve.value.EvaluatePosition(u);
}

BoundsCacheEntry bounds(const DetailCurve& curve, std::uint64_t source_version) {
  BoundsCacheEntry out{};
  out.whole = box(curve.sample_points);
  if (curve.sample_points.size() >= 2) {
    out.segments.reserve(curve.sample_points.size() - 1);
    for (std::size_t i = 0; i + 1 < curve.sample_points.size(); ++i) {
      out.segments.push_back(box(curve.sample_points[i], curve.sample_points[i + 1]));
    }
  }
  out.source_version = source_version;
  return out;
}

SpanRenderCacheEntry render(const CoreState& state, ObjectId span_id, const DetailCurve& detail) {
  SpanRenderCacheEntry out{};
  if (const Span* span = state.view().spans().find(span_id); span != nullptr) {
    if (const Bundle* bundle = state.view().bundles().find(span->bundle_id); bundle != nullptr) {
      const auto bundle_template_it = state.view().bundle_templates().find(bundle->bundle_template_id);
      if (bundle_template_it != state.view().bundle_templates().end()) {
        const auto cable_it = state.view().cable_templates().find(bundle_template_it->second.cable_template_id);
        if (cable_it != state.view().cable_templates().end()) {
          out.wire_radius_m = std::max(0.0005, cable_it->second.outer_diameter_m * 0.5);
          out.color_rgba = cable_it->second.color_rgba;
          out.material_style = cable_it->second.material_style;
        }
      }
    }
  }
  out.arc_length_m_by_point.reserve(detail.sample_points.size());
  out.arc_length_normalized_by_point.reserve(detail.sample_points.size());
  out.segment_length_m.reserve(detail.sample_points.size() > 0 ? detail.sample_points.size() - 1 : 0);
  double total = 0.0;
  for (std::size_t i = 0; i < detail.sample_points.size(); ++i) {
    if (i > 0) {
      const double segment = Length(detail.sample_points[i] - detail.sample_points[i - 1]);
      total += segment;
      out.segment_length_m.push_back(static_cast<float>(segment));
    }
    out.arc_length_m_by_point.push_back(static_cast<float>(total));
  }
  const double denom = total > 1e-9 ? total : 1.0;
  for (float value : out.arc_length_m_by_point) {
    out.arc_length_normalized_by_point.push_back(static_cast<float>(static_cast<double>(value) / denom));
  }
  return out;
}

SpanVisualCacheEntry visual(const VisualSettings&, const SpanLayoutEntry&) {
  SpanVisualCacheEntry out{};
  return out;
}

} // namespace wire::core::generation::backbone
