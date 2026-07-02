#include "curve_parts.hpp"

#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>

namespace wire::core::generation::backbone {
namespace {

constexpr double kNodePatchLengthM = 0.35;
constexpr double kCurveEps = 1e-9;
constexpr int kNodePatchSampleCount = 9;

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
  Vec3d point{};
  Vec3d tangent{};
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

std::vector<Vec3d> edge_body_samples(const Vec3d& a, const Vec3d& b, const GeometrySettings& settings) {
  const int count = std::max(3, settings.curve_samples);
  const Vec3d chord = b - a;
  const double len = Length(chord);
  const double sag = settings.sag_enabled ? std::max(0.0, settings.sag_factor) * len : 0.0;
  std::vector<Vec3d> samples{};
  samples.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double t = count <= 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(count - 1);
    const double sag_weight = 16.0 * t * t * (1.0 - t) * (1.0 - t);
    samples.push_back({a.x + chord.x * t, a.y + chord.y * t, a.z + chord.z * t - sag * sag_weight});
  }
  return samples;
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

std::vector<Vec3d> node_patch_samples(const curve_boundary& a, const curve_boundary& b) {
  const double chord_len = Length(b.point - a.point);
  const double handle = std::max(0.0, chord_len * 0.5);
  const Vec3d start_tangent = ScaleVec(a.tangent, -handle);
  const Vec3d end_tangent = ScaleVec(b.tangent, handle);
  std::vector<Vec3d> samples{};
  samples.reserve(kNodePatchSampleCount);
  for (int i = 0; i < kNodePatchSampleCount; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(kNodePatchSampleCount - 1);
    samples.push_back(cubic_hermite(a.point, start_tangent, b.point, end_tangent, t));
  }
  return samples;
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
    const Vec3d start_away = safe_unit(chord, fallback_dir);
    const Vec3d end_away = safe_unit(ScaleVec(chord, -1.0), ScaleVec(fallback_dir, -1.0));

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
    if (group.size() != 2 || group[0].edge_id == group[1].edge_id) {
      continue;
    }

    const double a_len = std::min(kNodePatchLengthM, group[0].span_length_m * 0.25);
    const double b_len = std::min(kNodePatchLengthM, group[1].span_length_m * 0.25);
    if (a_len <= kCurveEps || b_len <= kCurveEps) {
      continue;
    }

    curve_boundary a_boundary{};
    a_boundary.span_id = group[0].span_id;
    a_boundary.is_start = group[0].is_start;
    a_boundary.point = group[0].point + ScaleVec(group[0].away_from_node, a_len);
    a_boundary.tangent = group[0].away_from_node;
    if (!boundary_for(boundaries, a_boundary.span_id, a_boundary.is_start, nullptr)) {
      boundaries.push_back(a_boundary);
    }

    curve_boundary b_boundary{};
    b_boundary.span_id = group[1].span_id;
    b_boundary.is_start = group[1].is_start;
    b_boundary.point = group[1].point + ScaleVec(group[1].away_from_node, b_len);
    b_boundary.tangent = group[1].away_from_node;
    if (!boundary_for(boundaries, b_boundary.span_id, b_boundary.is_start, nullptr)) {
      boundaries.push_back(b_boundary);
    }

    VisualCurvePart patch{};
    patch.kind = VisualCurvePartKind::kNodePatch;
    patch.node_patch_classification = NodePatchClassification::kSimpleContinuous;
    patch.source_node_id = key.node_id;
    patch.source_bundle_id = group[0].bundle_id;
    patch.bundle_template_id = key.bundle_template_id;
    patch.lane_index = key.lane_index;
    add_unique_incident(group[0].edge_id, &patch.incident_edge_ids);
    add_unique_incident(group[1].edge_id, &patch.incident_edge_ids);
    std::sort(patch.incident_edge_ids.begin(), patch.incident_edge_ids.end());
    patch.boundary_a = a_boundary.point;
    patch.boundary_b = b_boundary.point;
    patch.tangent_a = a_boundary.tangent;
    patch.tangent_b = b_boundary.tangent;
    patch.samples = node_patch_samples(a_boundary, b_boundary);
    patch.bounds = curve_part_bounds(patch.samples);
    out.parts.push_back(std::move(patch));
  }

  const GeometrySettings& settings = state.view().geometry_settings();
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
    const Vec3d chord = entry.end.endpoint_world - entry.start.endpoint_world;
    const Vec3d start_tangent = has_start_patch ? start_boundary.tangent : safe_unit(chord, fallback_dir);
    const Vec3d end_tangent = has_end_patch ? end_boundary.tangent : safe_unit(ScaleVec(chord, -1.0), fallback_dir);
    const Vec3d start = has_start_patch ? start_boundary.point : entry.start.endpoint_world;
    const Vec3d end = has_end_patch ? end_boundary.point : entry.end.endpoint_world;

    VisualCurvePart body{};
    body.kind = VisualCurvePartKind::kEdgeBody;
    body.source_edge_id = edge_bundle->edge_id;
    body.source_span_id = entry.span_id;
    body.source_bundle_id = span->bundle_id;
    body.bundle_template_id = template_id;
    body.lane_index = binding->lane_index;
    body.boundary_a = start;
    body.boundary_b = end;
    body.tangent_a = start_tangent;
    body.tangent_b = end_tangent;
    body.samples = edge_body_samples(start, end, settings);
    body.bounds = curve_part_bounds(body.samples);
    if (!body.samples.empty() && finite_point(body.samples.front()) && finite_point(body.samples.back())) {
      if (const SpanRuntimeState* runtime = state.view().find_span_runtime_state(entry.span_id); runtime != nullptr) {
        body.source_version = runtime->data_version;
      }
      out.parts.push_back(std::move(body));
    }
  }
  return out;
}

} // namespace wire::core::generation::backbone
