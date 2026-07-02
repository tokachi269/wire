#include "curve_parts.hpp"

#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "out.hpp"
#include "population.hpp"

#include <algorithm>
#include <cmath>

namespace wire::core::generation::backbone {
namespace {

constexpr double kNodePatchHorizontalLengthM = 0.35;
constexpr double kNodePatchMaxSpanFraction = 0.25;
constexpr double kCurveEps = 1e-9;
constexpr double kPatchMetersPerSegment = 0.08;
constexpr double kPatchRadiansPerSegment = 0.17453292519943295;

struct curve_endpoint_ref {
  ObjectId span_id = kInvalidObjectId;
  ObjectId node_id = kInvalidObjectId;
  ObjectId edge_id = kInvalidObjectId;
  ObjectId edge_bundle_id = kInvalidObjectId;
  ObjectId bundle_id = kInvalidObjectId;
  BundleKind bundle_template_id = BundleKind::kLowVoltage;
  std::size_t lane_index = 0;
  Vec3d point{};
  Vec3d away_from_node{};
  double span_length_m = 0.0;
  bool has_curve_tangent = false;
  bool is_start = true;
};

struct curve_patch_key {
  ObjectId node_id = kInvalidObjectId;
  BundleKind bundle_template_id = BundleKind::kLowVoltage;
  std::size_t lane_index = 0;
};

struct curve_boundary {
  ObjectId span_id = kInvalidObjectId;
  bool is_start = true;
  Vec3d attachment_point{};
  double horizontal_length_m = 0.0;
  Vec3d point{};
  Vec3d tangent{};
};

struct curve_patch_spec {
  curve_patch_key key{};
  curve_endpoint_ref a{};
  curve_endpoint_ref b{};
  Vec3d attachment_point{};
};

bool same_key(const curve_patch_key& a, const curve_patch_key& b) {
  return a.node_id == b.node_id && a.bundle_template_id == b.bundle_template_id && a.lane_index == b.lane_index;
}

AABBd curve_part_bounds(const std::vector<Vec3d>& pts) {
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

bool finite_point(const Vec3d& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

Vec3d safe_unit(const Vec3d& value, const Vec3d& fallback) {
  const double len = Length(value);
  if (len <= kCurveEps) {
    return fallback;
  }
  return ScaleVec(value, 1.0 / len);
}

const SavedBackboneSpanBinding* span_binding_for(const CoreState& state, ObjectId span_id) {
  const auto it = state.view().backbone_index().span_bindings_by_span.find(span_id);
  if (it == state.view().backbone_index().span_bindings_by_span.end() || it->second.empty()) {
    return nullptr;
  }
  const std::size_t index = it->second.front();
  const SavedBackboneGraph& graph = state.view().backbone();
  if (index >= graph.span_bindings.size()) {
    return nullptr;
  }
  return &graph.span_bindings[index];
}

bool boundary_for(const std::vector<curve_boundary>& boundaries, ObjectId span_id, bool is_start,
                  curve_boundary* out) {
  for (const curve_boundary& boundary : boundaries) {
    if (boundary.span_id == span_id && boundary.is_start == is_start) {
      if (out != nullptr) {
        *out = boundary;
      }
      return true;
    }
  }
  return false;
}

curve_boundary* mutable_boundary_for(std::vector<curve_boundary>* boundaries, ObjectId span_id, bool is_start) {
  if (boundaries == nullptr) {
    return nullptr;
  }
  for (curve_boundary& boundary : *boundaries) {
    if (boundary.span_id == span_id && boundary.is_start == is_start) {
      return &boundary;
    }
  }
  return nullptr;
}

Vec3d boundary_from_tangent(const Vec3d& attachment, const Vec3d& outward_tangent, double horizontal_length_m) {
  const double horizontal = std::hypot(outward_tangent.x, outward_tangent.y);
  if (horizontal <= kCurveEps) {
    return attachment;
  }
  return attachment + ScaleVec(outward_tangent, horizontal_length_m / horizontal);
}

ObjectId saved_node_id_for_endpoint(const CoreState& state, ObjectId endpoint_node_id) {
  if (const SavedBackboneNode* node = state.view().backbone_node(endpoint_node_id); node != nullptr) {
    return node->node_id;
  }
  if (const SavedBackboneNode* node = state.view().backbone_node_for_pole(endpoint_node_id); node != nullptr) {
    return node->node_id;
  }
  return endpoint_node_id;
}

void add_unique_incident(ObjectId edge_id, std::vector<ObjectId>* ids) {
  if (edge_id == kInvalidObjectId || ids == nullptr) {
    return;
  }
  if (std::find(ids->begin(), ids->end(), edge_id) == ids->end()) {
    ids->push_back(edge_id);
  }
}

Vec3d cubic_hermite(const Vec3d& p0, const Vec3d& m0, const Vec3d& p1, const Vec3d& m1, double t) {
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
  const double h10 = t3 - 2.0 * t2 + t;
  const double h01 = -2.0 * t3 + 3.0 * t2;
  const double h11 = t3 - t2;
  return {
      p0.x * h00 + m0.x * h10 + p1.x * h01 + m1.x * h11,
      p0.y * h00 + m0.y * h10 + p1.y * h01 + m1.y * h11,
      p0.z * h00 + m0.z * h10 + p1.z * h01 + m1.z * h11,
  };
}

double cubic_control_distance(const Vec3d& a, const Vec3d& start_direction, const Vec3d& b,
                              const Vec3d& end_direction) {
  const double chord_len = Length(b - a);
  if (chord_len <= kCurveEps) {
    return 0.0;
  }
  const Vec3d start_unit = safe_unit(start_direction, {1.0, 0.0, 0.0});
  const Vec3d end_unit = safe_unit(end_direction, start_unit);
  const double dot =
      std::clamp(start_unit.x * end_unit.x + start_unit.y * end_unit.y + start_unit.z * end_unit.z,
                 -1.0, 1.0);
  const double turn_angle = std::acos(dot);
  // Cubic circular-arc approximation expressed from chord length and tangent turn angle.
  const double cos_quarter = std::cos(turn_angle * 0.25);
  const double denominator = 3.0 * cos_quarter * cos_quarter;
  return denominator <= kCurveEps ? chord_len / 3.0 : chord_len / denominator;
}

std::size_t patch_segment_count(const Vec3d& a, const Vec3d& start_direction, const Vec3d& b,
                                const Vec3d& end_direction) {
  const double length = Length(b - a);
  const Vec3d start_unit = safe_unit(start_direction, {1.0, 0.0, 0.0});
  const Vec3d end_unit = safe_unit(end_direction, start_unit);
  const double dot =
      std::clamp(start_unit.x * end_unit.x + start_unit.y * end_unit.y + start_unit.z * end_unit.z,
                 -1.0, 1.0);
  const double angle = std::acos(dot);
  const std::size_t length_segments =
      static_cast<std::size_t>(std::ceil(length / kPatchMetersPerSegment));
  const std::size_t angle_segments =
      static_cast<std::size_t>(std::ceil(angle / kPatchRadiansPerSegment));
  return std::clamp(std::max<std::size_t>({2, length_segments, angle_segments}), std::size_t{2},
                    std::size_t{32});
}

void append_patch_section(const Vec3d& a, const Vec3d& start_direction, const Vec3d& b,
                          const Vec3d& end_direction, bool include_start, std::vector<Vec3d>* controls,
                          std::vector<Vec3d>* samples) {
  const double control_distance = cubic_control_distance(a, start_direction, b, end_direction);
  const Vec3d start_tangent = ScaleVec(safe_unit(start_direction, b - a), control_distance * 3.0);
  const Vec3d end_tangent = ScaleVec(safe_unit(end_direction, b - a), control_distance * 3.0);
  const std::vector<Vec3d> section_controls{
      a,
      a + ScaleVec(start_tangent, 1.0 / 3.0),
      b - ScaleVec(end_tangent, 1.0 / 3.0),
      b,
  };
  controls->insert(controls->end(), section_controls.begin() + (include_start ? 0 : 1), section_controls.end());
  const std::size_t segments = patch_segment_count(a, start_direction, b, end_direction);
  for (std::size_t index = include_start ? 0 : 1; index <= segments; ++index) {
    const double t = static_cast<double>(index) / static_cast<double>(segments);
    samples->push_back(cubic_hermite(a, start_tangent, b, end_tangent, t));
  }
}

void copy_span_appearance(const CoreState& state, ObjectId span_id, VisualCurvePart* part) {
  if (part == nullptr) {
    return;
  }
  const SpanRenderCacheEntry appearance = render(state, span_id, {});
  part->wire_radius_m = appearance.wire_radius_m;
  part->color_rgba = appearance.color_rgba;
  part->material_style = appearance.material_style;
}

layout merged_visual_curve_layouts(const CoreState& state, const layout& made) {
  layout merged{};
  merged.entries = made.entries;
  state.view().cache_state().span_layout_cache.for_each_layout_record(
      [&](ObjectId, const SpanLayoutCacheRecord&, const SpanLayoutEntry& existing) {
        const bool replaced = std::any_of(made.entries.begin(), made.entries.end(), [&](const SpanLayoutEntry& entry) {
          return entry.span_id == existing.span_id;
        });
        if (!replaced) {
          merged.entries.push_back(existing);
        }
      });
  return merged;
}

} // namespace

VisualCurvePartCache make_visual_curve_parts(const CoreState& state, const layout& made) {
  const layout placed = merged_visual_curve_layouts(state, made);
  VisualCurvePartCache out{};
  std::vector<curve_endpoint_ref> endpoints{};
  endpoints.reserve(placed.entries.size() * 2);
  const Vec3d fallback_dir{1.0, 0.0, 0.0};

  for (const SpanLayoutEntry& entry : placed.entries) {
    const Span* span = state.view().spans().find(entry.span_id);
    const SavedBackboneSpanBinding* binding = span_binding_for(state, entry.span_id);
    const SavedBackboneEdgeBundle* edge_bundle =
        binding == nullptr ? nullptr : state.view().backbone_edge_bundle(binding->edge_bundle_id);
    if (span == nullptr || binding == nullptr || edge_bundle == nullptr) {
      continue;
    }
    const Bundle* bundle = state.view().bundles().find(span->bundle_id);
    const BundleKind template_id = bundle == nullptr ? BundleKind::kLowVoltage : bundle->bundle_template_id;
    const Vec3d chord = entry.end.endpoint_world - entry.start.endpoint_world;
    const double span_length = Length(chord);
    const EditResult<DetailCurve> full_curve =
        make_curve_between(state, entry.span_id, entry.start.endpoint_world, entry.end.endpoint_world);
    const bool has_curve_tangent = full_curve.ok && full_curve.value.sample_points.size() >= 2;
    const Vec3d start_away =
        has_curve_tangent ? full_curve.value.start_constraint.tangent_dir : safe_unit(chord, fallback_dir);
    const Vec3d end_away = has_curve_tangent
                              ? ScaleVec(full_curve.value.end_constraint.tangent_dir, -1.0)
                              : safe_unit(ScaleVec(chord, -1.0), ScaleVec(fallback_dir, -1.0));

    curve_endpoint_ref start{};
    start.span_id = entry.span_id;
    start.node_id = saved_node_id_for_endpoint(state, entry.start.endpoint_node_id);
    start.edge_id = edge_bundle->edge_id;
    start.edge_bundle_id = edge_bundle->edge_bundle_id;
    start.bundle_id = span->bundle_id;
    start.bundle_template_id = template_id;
    start.lane_index = binding->lane_index;
    start.point = entry.start.endpoint_world;
    start.away_from_node = start_away;
    start.span_length_m = span_length;
    start.has_curve_tangent = has_curve_tangent;
    start.is_start = true;
    endpoints.push_back(start);

    curve_endpoint_ref end = start;
    end.node_id = saved_node_id_for_endpoint(state, entry.end.endpoint_node_id);
    end.point = entry.end.endpoint_world;
    end.away_from_node = end_away;
    end.is_start = false;
    endpoints.push_back(end);
  }

  std::vector<curve_boundary> boundaries{};
  std::vector<curve_patch_spec> patch_specs{};
  std::vector<curve_patch_key> processed_patch_keys{};
  for (const curve_endpoint_ref& first : endpoints) {
    const curve_patch_key key{first.node_id, first.bundle_template_id, first.lane_index};
    if (std::find_if(processed_patch_keys.begin(), processed_patch_keys.end(),
                     [&](const curve_patch_key& processed) { return same_key(processed, key); }) !=
        processed_patch_keys.end()) {
      continue;
    }
    processed_patch_keys.push_back(key);
    std::vector<curve_endpoint_ref> group{};
    for (const curve_endpoint_ref& endpoint : endpoints) {
      if (same_key(key, {endpoint.node_id, endpoint.bundle_template_id, endpoint.lane_index})) {
        group.push_back(endpoint);
      }
    }
    if (group.size() != 2 || group[0].edge_id == group[1].edge_id ||
        !group[0].has_curve_tangent || !group[1].has_curve_tangent) {
      continue;
    }
    if (Length(group[1].point - group[0].point) > 1e-6) {
      continue;
    }

    const double a_len =
        std::min(kNodePatchHorizontalLengthM, group[0].span_length_m * kNodePatchMaxSpanFraction);
    const double b_len =
        std::min(kNodePatchHorizontalLengthM, group[1].span_length_m * kNodePatchMaxSpanFraction);
    if (a_len <= kCurveEps || b_len <= kCurveEps) {
      continue;
    }

    curve_boundary a_boundary{};
    a_boundary.span_id = group[0].span_id;
    a_boundary.is_start = group[0].is_start;
    a_boundary.attachment_point = group[0].point;
    a_boundary.horizontal_length_m = a_len;
    a_boundary.point = boundary_from_tangent(group[0].point, group[0].away_from_node, a_len);
    a_boundary.tangent = group[0].away_from_node;
    if (!boundary_for(boundaries, a_boundary.span_id, a_boundary.is_start, nullptr)) {
      boundaries.push_back(a_boundary);
    }

    curve_boundary b_boundary{};
    b_boundary.span_id = group[1].span_id;
    b_boundary.is_start = group[1].is_start;
    b_boundary.attachment_point = group[1].point;
    b_boundary.horizontal_length_m = b_len;
    b_boundary.point = boundary_from_tangent(group[1].point, group[1].away_from_node, b_len);
    b_boundary.tangent = group[1].away_from_node;
    if (!boundary_for(boundaries, b_boundary.span_id, b_boundary.is_start, nullptr)) {
      boundaries.push_back(b_boundary);
    }
    patch_specs.push_back({key, group[0], group[1], group[0].point});
  }

  for (int iteration = 0; iteration < 2; ++iteration) {
    for (const SpanLayoutEntry& entry : placed.entries) {
      curve_boundary start_boundary{};
      curve_boundary end_boundary{};
      const bool has_start_patch = boundary_for(boundaries, entry.span_id, true, &start_boundary);
      const bool has_end_patch = boundary_for(boundaries, entry.span_id, false, &end_boundary);
      if (!has_start_patch && !has_end_patch) {
        continue;
      }
      const Vec3d start = has_start_patch ? start_boundary.point : entry.start.endpoint_world;
      const Vec3d end = has_end_patch ? end_boundary.point : entry.end.endpoint_world;
      const EditResult<DetailCurve> curve = make_curve_between(state, entry.span_id, start, end);
      if (!curve.ok || curve.value.sample_points.size() < 2) {
        continue;
      }
      if (curve_boundary* boundary = mutable_boundary_for(&boundaries, entry.span_id, true); boundary != nullptr) {
        boundary->tangent = curve.value.start_constraint.tangent_dir;
        boundary->point =
            boundary_from_tangent(boundary->attachment_point, boundary->tangent, boundary->horizontal_length_m);
      }
      if (curve_boundary* boundary = mutable_boundary_for(&boundaries, entry.span_id, false); boundary != nullptr) {
        boundary->tangent = ScaleVec(curve.value.end_constraint.tangent_dir, -1.0);
        boundary->point =
            boundary_from_tangent(boundary->attachment_point, boundary->tangent, boundary->horizontal_length_m);
      }
    }
  }

  for (const SpanLayoutEntry& entry : placed.entries) {
    const Span* span = state.view().spans().find(entry.span_id);
    const SavedBackboneSpanBinding* binding = span_binding_for(state, entry.span_id);
    const SavedBackboneEdgeBundle* edge_bundle =
        binding == nullptr ? nullptr : state.view().backbone_edge_bundle(binding->edge_bundle_id);
    if (span == nullptr || binding == nullptr || edge_bundle == nullptr) {
      continue;
    }
    const Bundle* bundle = state.view().bundles().find(span->bundle_id);
    const BundleKind template_id = bundle == nullptr ? BundleKind::kLowVoltage : bundle->bundle_template_id;
    curve_boundary start_boundary{};
    curve_boundary end_boundary{};
    const bool has_start_patch = boundary_for(boundaries, entry.span_id, true, &start_boundary);
    const bool has_end_patch = boundary_for(boundaries, entry.span_id, false, &end_boundary);
    const Vec3d start = has_start_patch ? start_boundary.point : entry.start.endpoint_world;
    const Vec3d end = has_end_patch ? end_boundary.point : entry.end.endpoint_world;
    const EditResult<DetailCurve> curve = make_curve_between(state, entry.span_id, start, end);
    if (!curve.ok || curve.value.sample_points.size() < 2) {
      continue;
    }
    const Vec3d start_param_tangent = curve.value.start_constraint.tangent_dir;
    const Vec3d end_param_tangent = curve.value.end_constraint.tangent_dir;
    const Vec3d start_outward_tangent = start_param_tangent;
    const Vec3d end_outward_tangent = ScaleVec(end_param_tangent, -1.0);
    if (curve_boundary* boundary = mutable_boundary_for(&boundaries, entry.span_id, true); boundary != nullptr) {
      boundary->tangent = start_outward_tangent;
    }
    if (curve_boundary* boundary = mutable_boundary_for(&boundaries, entry.span_id, false); boundary != nullptr) {
      boundary->tangent = end_outward_tangent;
    }

    VisualCurvePart body{};
    body.kind = VisualCurvePartKind::kEdgeBody;
    body.source_edge_id = edge_bundle->edge_id;
    body.source_span_id = entry.span_id;
    body.source_bundle_id = span->bundle_id;
    body.bundle_template_id = template_id;
    body.lane_index = binding->lane_index;
    body.boundary_a = start;
    body.boundary_b = end;
    body.tangent_a = start_outward_tangent;
    body.tangent_b = end_outward_tangent;
    body.sag_method = VisualCurveSagMethod::kParabolic;
    body.sag_m = curve.value.sag_amplitude_m;
    copy_span_appearance(state, entry.span_id, &body);
    body.samples = curve.value.sample_points;
    body.bounds = curve_part_bounds(body.samples);
    if (!body.samples.empty() && finite_point(body.samples.front()) && finite_point(body.samples.back())) {
      if (const SpanRuntimeState* runtime = state.view().find_span_runtime_state(entry.span_id); runtime != nullptr) {
        body.source_version = runtime->data_version;
      }
      out.parts.push_back(std::move(body));
    }
  }

  for (const curve_patch_spec& spec : patch_specs) {
    curve_boundary a_boundary{};
    curve_boundary b_boundary{};
    if (!boundary_for(boundaries, spec.a.span_id, spec.a.is_start, &a_boundary) ||
        !boundary_for(boundaries, spec.b.span_id, spec.b.is_start, &b_boundary)) {
      continue;
    }
    const Vec3d incoming = ScaleVec(a_boundary.tangent, -1.0);
    const Vec3d outgoing = b_boundary.tangent;

    VisualCurvePart patch{};
    patch.kind = VisualCurvePartKind::kNodePatch;
    patch.node_patch_classification = NodePatchClassification::kSimpleContinuous;
    patch.source_node_id = spec.key.node_id;
    patch.source_bundle_id = spec.a.bundle_id;
    patch.bundle_template_id = spec.key.bundle_template_id;
    patch.lane_index = spec.key.lane_index;
    add_unique_incident(spec.a.edge_id, &patch.incident_edge_ids);
    add_unique_incident(spec.b.edge_id, &patch.incident_edge_ids);
    std::sort(patch.incident_edge_ids.begin(), patch.incident_edge_ids.end());
    patch.boundary_a = a_boundary.point;
    patch.boundary_b = b_boundary.point;
    patch.tangent_a = a_boundary.tangent;
    patch.tangent_b = b_boundary.tangent;
    patch.attachment_point = spec.attachment_point;
    patch.has_attachment_point = true;
    patch.passes_attachment_point = false;
    patch.section_count = 1;
    copy_span_appearance(state, spec.a.span_id, &patch);
    append_patch_section(a_boundary.point, incoming, b_boundary.point, outgoing, true,
                         &patch.bezier_control_points, &patch.samples);
    patch.bounds = curve_part_bounds(patch.samples);
    out.parts.push_back(std::move(patch));
  }
  append_experimental_physical_lines(state, placed.entries, &out);
  return out;
}

} // namespace wire::core::generation::backbone
