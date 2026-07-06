#include "curve_parts.hpp"

#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "out.hpp"
#include "population.hpp"
#include "../../geometry/detail_curve_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace wire::core::generation::backbone {
namespace {

constexpr double kNodePatchHorizontalLengthM = 0.35;
constexpr double kNodePatchMaxSpanFraction = 0.25;
constexpr double kCurveEps = 1e-9;
constexpr double kPatchMetersPerSegment = 0.08;
constexpr double kPatchRadiansPerSegment = 0.17453292519943295;

struct curve_endpoint_ref {
  CableInstanceKey cable_instance_key{};
  ObjectId node_id = kInvalidObjectId;
  ObjectId edge_id = kInvalidObjectId;
  ObjectId edge_bundle_id = kInvalidObjectId;
  ObjectId bundle_id = kInvalidObjectId;
  ObjectId port_id = kInvalidObjectId;
  ObjectId jumper_peer_port_id = kInvalidObjectId;
  BundleKind bundle_template_id = BundleKind::kLowVoltage;
  std::size_t lane_index = 0;
  PoleTypeId pole_type_id = kInvalidPoleTypeId;
  int band_id = 0;
  SavedBackboneRowKey row_key{};
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
  PoleTypeId pole_type_id = kInvalidPoleTypeId;
  int band_id = 0;
  std::uint64_t rule_owner_id = 0;
  CableInstanceRuleId rule_id = 0;
  std::size_t instance_index = 0;
};

struct curve_boundary {
  CableInstanceKey cable_instance_key{};
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

struct visual_cable_section {
  CableSectionLayout layout{};
  ObjectId start_node_id = kInvalidObjectId;
  ObjectId end_node_id = kInvalidObjectId;
  ObjectId start_port_id = kInvalidObjectId;
  ObjectId end_port_id = kInvalidObjectId;
  ObjectId start_jumper_peer_port_id = kInvalidObjectId;
  ObjectId end_jumper_peer_port_id = kInvalidObjectId;
  SavedBackboneRowKey start_row_key{};
  SavedBackboneRowKey end_row_key{};
};

bool same_cable_instance(const CableInstanceKey& a, const CableInstanceKey& b) {
  return a.logical_span_id == b.logical_span_id && a.edge_bundle_id == b.edge_bundle_id &&
         a.rule_owner_id == b.rule_owner_id && a.rule_id == b.rule_id &&
         a.instance_index == b.instance_index;
}

bool same_key(const curve_patch_key& a, const curve_patch_key& b) {
  return a.node_id == b.node_id && a.bundle_template_id == b.bundle_template_id &&
         a.lane_index == b.lane_index && a.pole_type_id == b.pole_type_id &&
         a.band_id == b.band_id && a.rule_owner_id == b.rule_owner_id &&
         a.rule_id == b.rule_id && a.instance_index == b.instance_index;
}

bool row_pairs_edges(const SavedBackboneRowKey& row_key, ObjectId edge_a, ObjectId edge_b) {
  if (row_key.source_is_open || row_key.source_edge_a == kInvalidObjectId ||
      row_key.source_edge_b == kInvalidObjectId) {
    return false;
  }
  return std::min(row_key.source_edge_a, row_key.source_edge_b) == std::min(edge_a, edge_b) &&
         std::max(row_key.source_edge_a, row_key.source_edge_b) == std::max(edge_a, edge_b);
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

bool boundary_for(const std::vector<curve_boundary>& boundaries, const CableInstanceKey& cable_instance_key, bool is_start,
                  curve_boundary* out) {
  for (const curve_boundary& boundary : boundaries) {
    if (same_cable_instance(boundary.cable_instance_key, cable_instance_key) && boundary.is_start == is_start) {
      if (out != nullptr) {
        *out = boundary;
      }
      return true;
    }
  }
  return false;
}

curve_boundary* mutable_boundary_for(std::vector<curve_boundary>* boundaries,
                                     const CableInstanceKey& cable_instance_key, bool is_start) {
  if (boundaries == nullptr) {
    return nullptr;
  }
  for (curve_boundary& boundary : *boundaries) {
    if (same_cable_instance(boundary.cable_instance_key, cable_instance_key) && boundary.is_start == is_start) {
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

PoleTypeId pole_type_for_port(const CoreState& state, ObjectId port_id) {
  const Port* port = state.view().ports().find(port_id);
  if (port == nullptr || port->owner_pole_id == kInvalidObjectId) {
    return kInvalidPoleTypeId;
  }
  const Pole* pole = state.view().poles().find(port->owner_pole_id);
  return pole == nullptr ? kInvalidPoleTypeId : pole->pole_type_id;
}

const SavedBackbonePortBinding* port_binding_for(const CoreState& state, ObjectId edge_bundle_id,
                                                 std::size_t lane_index, ObjectId port_id) {
  const std::vector<const SavedBackbonePortBinding*> bindings =
      state.view().backbone_port_bindings_for_edge_bundle(edge_bundle_id);
  const SavedBackbonePortBinding* found = nullptr;
  for (const SavedBackbonePortBinding* binding : bindings) {
    if (binding == nullptr || binding->lane_index != lane_index || binding->port_id != port_id) {
      continue;
    }
    if (found != nullptr) {
      return nullptr;
    }
    found = binding;
  }
  return found;
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

void append_jumper_section(const Vec3d& a, const Vec3d& b, std::vector<Vec3d>* controls,
                           std::vector<Vec3d>* samples) {
  const Vec3d chord = b - a;
  const double length = Length(chord);
  if (length <= kCurveEps || controls == nullptr || samples == nullptr) {
    return;
  }
  const Vec3d along = ScaleVec(chord, 1.0 / length);
  const Vec3d down{0.0, 0.0, -1.0};
  const double handle = std::clamp(length * 0.45, 0.08, 0.35);
  const Vec3d start_direction = safe_unit(along + down, along);
  const Vec3d end_direction = safe_unit(along - down, along);
  const Vec3d start_tangent = ScaleVec(start_direction, handle * 3.0);
  const Vec3d end_tangent = ScaleVec(end_direction, handle * 3.0);
  controls->assign({
      a,
      a + ScaleVec(start_tangent, 1.0 / 3.0),
      b - ScaleVec(end_tangent, 1.0 / 3.0),
      b,
  });
  const std::size_t segments = std::clamp<std::size_t>(
      static_cast<std::size_t>(std::ceil(length / kPatchMetersPerSegment)), 4, 24);
  samples->reserve(segments + 1);
  for (std::size_t index = 0; index <= segments; ++index) {
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
  std::vector<visual_cable_section> sections{};
  sections.reserve(placed.entries.size());
  for (const SpanLayoutEntry& entry : placed.entries) {
    const SavedBackboneSpanBinding* binding = span_binding_for(state, entry.span_id);
    if (binding == nullptr) {
      out.diagnostics.push_back(
          {kInvalidObjectId, entry.span_id, BundleKind::kLowVoltage, 0, "span binding missing"});
      continue;
    }
    const Span* span = state.view().spans().find(entry.span_id);
    const Bundle* bundle = span == nullptr ? nullptr : state.view().bundles().find(span->bundle_id);
    if (span == nullptr || bundle == nullptr) {
      out.diagnostics.push_back(
          {kInvalidObjectId, entry.span_id, BundleKind::kLowVoltage, binding->lane_index, "span bundle missing"});
      continue;
    }
    const SavedBackbonePortBinding* start_binding =
        port_binding_for(state, binding->edge_bundle_id, binding->lane_index, entry.start.port_id);
    const SavedBackbonePortBinding* end_binding =
        port_binding_for(state, binding->edge_bundle_id, binding->lane_index, entry.end.port_id);
    if (start_binding == nullptr || end_binding == nullptr) {
      out.diagnostics.push_back(
          {kInvalidObjectId, entry.span_id, bundle->bundle_template_id, binding->lane_index,
           "endpoint port binding missing or ambiguous"});
      continue;
    }
    visual_cable_section section{};
    section.layout.key.logical_span_id = entry.span_id;
    section.layout.key.edge_bundle_id = binding->edge_bundle_id;
    section.layout.endpoint_a = entry.start.endpoint_world;
    section.layout.endpoint_b = entry.end.endpoint_world;
    section.layout.endpoint_a_pole_type_id = pole_type_for_port(state, entry.start.port_id);
    section.layout.endpoint_b_pole_type_id = pole_type_for_port(state, entry.end.port_id);
    section.layout.endpoint_a_band_id = start_binding->placement_band_id;
    section.layout.endpoint_b_band_id = end_binding->placement_band_id;
    section.start_node_id = entry.start.endpoint_node_id;
    section.end_node_id = entry.end.endpoint_node_id;
    section.start_port_id = entry.start.port_id;
    section.end_port_id = entry.end.port_id;
    section.start_jumper_peer_port_id = entry.start.jumper_peer_port_id;
    section.end_jumper_peer_port_id = entry.end.jumper_peer_port_id;
    section.start_row_key = start_binding->row_key;
    section.end_row_key = end_binding->row_key;
    sections.push_back(section);
  }
  CablePopulation population = make_cable_population(state, placed.entries);
  out.population_diagnostics = std::move(population.diagnostics);
  for (CableSectionLayout& extra : population.sections) {
    const auto source = std::find_if(placed.entries.begin(), placed.entries.end(), [&](const SpanLayoutEntry& entry) {
      return entry.span_id == extra.key.logical_span_id;
    });
    if (source == placed.entries.end()) {
      continue;
    }
    visual_cable_section section{};
    section.layout = std::move(extra);
    section.start_node_id = source->start.endpoint_node_id;
    section.end_node_id = source->end.endpoint_node_id;
    const auto base_section =
        std::find_if(sections.begin(), sections.end(), [&](const visual_cable_section& candidate) {
          return candidate.layout.key.logical_span_id == section.layout.key.logical_span_id &&
                 candidate.layout.key.is_base();
        });
    if (base_section != sections.end()) {
      section.start_row_key = base_section->start_row_key;
      section.end_row_key = base_section->end_row_key;
    }
    sections.push_back(std::move(section));
  }

  std::vector<curve_endpoint_ref> endpoints{};
  endpoints.reserve(sections.size() * 2);
  const Vec3d fallback_dir{1.0, 0.0, 0.0};

  for (const visual_cable_section& section : sections) {
    const CableSectionLayout& entry = section.layout;
    if (entry.profile == CableSectionProfile::kWrap) {
      continue;
    }
    const Span* span = state.view().spans().find(entry.key.logical_span_id);
    const SavedBackboneSpanBinding* binding = span_binding_for(state, entry.key.logical_span_id);
    const SavedBackboneEdgeBundle* edge_bundle =
        binding == nullptr ? nullptr : state.view().backbone_edge_bundle(binding->edge_bundle_id);
    if (span == nullptr || binding == nullptr || edge_bundle == nullptr) {
      out.diagnostics.push_back(
          {kInvalidObjectId, entry.key.logical_span_id, BundleKind::kLowVoltage, 0,
           "curve source identity missing"});
      continue;
    }
    const Bundle* bundle = state.view().bundles().find(span->bundle_id);
    if (bundle == nullptr) {
      out.diagnostics.push_back(
          {kInvalidObjectId, entry.key.logical_span_id, BundleKind::kLowVoltage, binding->lane_index,
           "curve source bundle missing"});
      continue;
    }
    const BundleKind template_id = bundle->bundle_template_id;
    const Vec3d chord = entry.endpoint_b - entry.endpoint_a;
    const double span_length = Length(chord);
    const EditResult<DetailCurve> full_curve =
        make_curve_between(state, entry.key.logical_span_id, entry.endpoint_a, entry.endpoint_b);
    const bool has_curve_tangent = full_curve.ok && full_curve.value.sample_points.size() >= 2;
    const Vec3d start_away =
        has_curve_tangent ? full_curve.value.start_constraint.tangent_dir : safe_unit(chord, fallback_dir);
    const Vec3d end_away = has_curve_tangent
                              ? ScaleVec(full_curve.value.end_constraint.tangent_dir, -1.0)
                              : safe_unit(ScaleVec(chord, -1.0), ScaleVec(fallback_dir, -1.0));

    curve_endpoint_ref start{};
    start.cable_instance_key = entry.key;
    start.node_id = saved_node_id_for_endpoint(state, section.start_node_id);
    start.edge_id = edge_bundle->edge_id;
    start.edge_bundle_id = edge_bundle->edge_bundle_id;
    start.bundle_id = span->bundle_id;
    start.port_id = section.start_port_id;
    start.jumper_peer_port_id = section.start_jumper_peer_port_id;
    start.bundle_template_id = template_id;
    start.lane_index = binding->lane_index;
    start.pole_type_id = entry.endpoint_a_pole_type_id;
    start.band_id = entry.endpoint_a_band_id;
    start.row_key = section.start_row_key;
    start.point = entry.endpoint_a;
    start.away_from_node = start_away;
    start.span_length_m = span_length;
    start.has_curve_tangent = has_curve_tangent;
    start.is_start = true;
    endpoints.push_back(start);

    curve_endpoint_ref end = start;
    end.node_id = saved_node_id_for_endpoint(state, section.end_node_id);
    end.port_id = section.end_port_id;
    end.jumper_peer_port_id = section.end_jumper_peer_port_id;
    end.pole_type_id = entry.endpoint_b_pole_type_id;
    end.band_id = entry.endpoint_b_band_id;
    end.row_key = section.end_row_key;
    end.point = entry.endpoint_b;
    end.away_from_node = end_away;
    end.is_start = false;
    endpoints.push_back(end);
  }

  std::vector<curve_boundary> boundaries{};
  std::vector<curve_patch_spec> patch_specs{};
  std::vector<curve_patch_key> processed_patch_keys{};
  for (const curve_endpoint_ref& first : endpoints) {
    const curve_patch_key key{first.node_id, first.bundle_template_id, first.lane_index,
                              first.pole_type_id, first.band_id,
                              first.cable_instance_key.rule_owner_id, first.cable_instance_key.rule_id,
                              first.cable_instance_key.instance_index};
    if (std::find_if(processed_patch_keys.begin(), processed_patch_keys.end(),
                     [&](const curve_patch_key& processed) { return same_key(processed, key); }) !=
        processed_patch_keys.end()) {
      continue;
    }
    processed_patch_keys.push_back(key);
    std::vector<curve_endpoint_ref> group{};
    for (const curve_endpoint_ref& endpoint : endpoints) {
      if (same_key(key, {endpoint.node_id, endpoint.bundle_template_id, endpoint.lane_index,
                         endpoint.pole_type_id, endpoint.band_id,
                         endpoint.cable_instance_key.rule_owner_id, endpoint.cable_instance_key.rule_id,
                         endpoint.cable_instance_key.instance_index})) {
        group.push_back(endpoint);
      }
    }
    std::vector<std::pair<std::size_t, std::size_t>> candidates{};
    for (std::size_t a = 0; a < group.size(); ++a) {
      for (std::size_t b = a + 1; b < group.size(); ++b) {
        if (group[a].edge_id == group[b].edge_id) {
          continue;
        }
        const bool shared_port =
            group[a].port_id != kInvalidObjectId && group[a].port_id == group[b].port_id;
        const bool explicit_pair = row_pairs_edges(group[a].row_key, group[a].edge_id, group[b].edge_id) ||
                                   row_pairs_edges(group[b].row_key, group[a].edge_id, group[b].edge_id);
        if (shared_port || explicit_pair) {
          candidates.push_back({a, b});
        }
      }
    }
    if (candidates.size() != 1) {
      out.diagnostics.push_back(
          {key.node_id, kInvalidObjectId, key.bundle_template_id, key.lane_index,
           candidates.empty() ? (group.size() < 2 ? "terminal node has no patch peer"
                                                  : "no connectivity-owned patch pair")
                              : "multiple connectivity-owned patch pairs"});
      continue;
    }
    const curve_endpoint_ref& patch_a = group[candidates.front().first];
    const curve_endpoint_ref& patch_b = group[candidates.front().second];
    if (!patch_a.has_curve_tangent || !patch_b.has_curve_tangent) {
      out.diagnostics.push_back(
          {key.node_id, kInvalidObjectId, key.bundle_template_id, key.lane_index,
           "patch endpoint tangent missing"});
      continue;
    }
    if (patch_a.jumper_peer_port_id != kInvalidObjectId ||
        patch_b.jumper_peer_port_id != kInvalidObjectId) {
      out.diagnostics.push_back(
          {key.node_id, kInvalidObjectId, key.bundle_template_id, key.lane_index,
           "explicit jumper owns this connection"});
      continue;
    }
    const double a_len =
        std::min(kNodePatchHorizontalLengthM, patch_a.span_length_m * kNodePatchMaxSpanFraction);
    const double b_len =
        std::min(kNodePatchHorizontalLengthM, patch_b.span_length_m * kNodePatchMaxSpanFraction);
    if (a_len <= kCurveEps || b_len <= kCurveEps) {
      out.diagnostics.push_back(
          {key.node_id, kInvalidObjectId, key.bundle_template_id, key.lane_index,
           "patch boundary length is zero"});
      continue;
    }

    curve_boundary a_boundary{};
    a_boundary.cable_instance_key = patch_a.cable_instance_key;
    a_boundary.is_start = patch_a.is_start;
    a_boundary.attachment_point = patch_a.point;
    a_boundary.horizontal_length_m = a_len;
    a_boundary.point = boundary_from_tangent(patch_a.point, patch_a.away_from_node, a_len);
    a_boundary.tangent = patch_a.away_from_node;
    if (!boundary_for(boundaries, a_boundary.cable_instance_key, a_boundary.is_start, nullptr)) {
      boundaries.push_back(a_boundary);
    }

    curve_boundary b_boundary{};
    b_boundary.cable_instance_key = patch_b.cable_instance_key;
    b_boundary.is_start = patch_b.is_start;
    b_boundary.attachment_point = patch_b.point;
    b_boundary.horizontal_length_m = b_len;
    b_boundary.point = boundary_from_tangent(patch_b.point, patch_b.away_from_node, b_len);
    b_boundary.tangent = patch_b.away_from_node;
    if (!boundary_for(boundaries, b_boundary.cable_instance_key, b_boundary.is_start, nullptr)) {
      boundaries.push_back(b_boundary);
    }
    patch_specs.push_back({key, patch_a, patch_b, ScaleVec(patch_a.point + patch_b.point, 0.5)});
  }

  for (int iteration = 0; iteration < 2; ++iteration) {
    for (const visual_cable_section& section : sections) {
      const CableSectionLayout& entry = section.layout;
      curve_boundary start_boundary{};
      curve_boundary end_boundary{};
      const bool has_start_patch = boundary_for(boundaries, entry.key, true, &start_boundary);
      const bool has_end_patch = boundary_for(boundaries, entry.key, false, &end_boundary);
      if (!has_start_patch && !has_end_patch) {
        continue;
      }
      const Vec3d start = has_start_patch ? start_boundary.point : entry.endpoint_a;
      const Vec3d end = has_end_patch ? end_boundary.point : entry.endpoint_b;
      const Vec3d end_param_tangent = ScaleVec(end_boundary.tangent, -1.0);
      const EditResult<DetailCurve> curve =
          make_curve_between_with_tangent_hints(state, entry.key.logical_span_id, start, end,
                                                has_start_patch ? &start_boundary.tangent : nullptr,
                                                has_end_patch ? &end_param_tangent : nullptr);
      if (!curve.ok || curve.value.sample_points.size() < 2) {
        continue;
      }
      if (curve_boundary* boundary = mutable_boundary_for(&boundaries, entry.key, true); boundary != nullptr) {
        boundary->tangent = curve.value.start_constraint.tangent_dir;
        boundary->point =
            boundary_from_tangent(boundary->attachment_point, boundary->tangent, boundary->horizontal_length_m);
      }
      if (curve_boundary* boundary = mutable_boundary_for(&boundaries, entry.key, false); boundary != nullptr) {
        boundary->tangent = ScaleVec(curve.value.end_constraint.tangent_dir, -1.0);
        boundary->point =
            boundary_from_tangent(boundary->attachment_point, boundary->tangent, boundary->horizontal_length_m);
      }
    }
  }

  // Base bodies land before population extras in `sections`, so a wrap section can always read
  // its carrier's final (boundary-trimmed) curve from this map.
  std::unordered_map<ObjectId, DetailCurve> base_final_curves{};
  for (const visual_cable_section& section : sections) {
    const CableSectionLayout& entry = section.layout;
    const Span* span = state.view().spans().find(entry.key.logical_span_id);
    const SavedBackboneSpanBinding* binding = span_binding_for(state, entry.key.logical_span_id);
    const SavedBackboneEdgeBundle* edge_bundle =
        binding == nullptr ? nullptr : state.view().backbone_edge_bundle(binding->edge_bundle_id);
    if (span == nullptr || binding == nullptr || edge_bundle == nullptr) {
      continue;
    }
    const Bundle* bundle = state.view().bundles().find(span->bundle_id);
    if (bundle == nullptr) {
      continue;
    }
    const BundleKind template_id = bundle->bundle_template_id;
    if (entry.profile == CableSectionProfile::kWrap) {
      const auto carrier_it = base_final_curves.find(entry.key.logical_span_id);
      if (carrier_it == base_final_curves.end()) {
        out.diagnostics.push_back(
            {kInvalidObjectId, entry.key.logical_span_id, template_id, binding->lane_index,
             "wrap carrier curve missing"});
        continue;
      }
      const DetailCurve& carrier = carrier_it->second;
      const double trim_m = std::max(0.0, entry.end_trim_m);
      const std::vector<Vec3d> helix = sample_wrap_helix_points(
          carrier, trim_m, carrier.Length() - trim_m, entry.wrap_radius_m,
          entry.wrap_turns_per_meter, entry.wrap_phase, static_cast<double>(entry.wrap_direction));
      if (helix.size() < 2 || !finite_point(helix.front()) || !finite_point(helix.back())) {
        out.diagnostics.push_back(
            {kInvalidObjectId, entry.key.logical_span_id, template_id, binding->lane_index,
             "wrap trim leaves no visible section"});
        continue;
      }
      VisualCurvePart body{};
      body.kind = VisualCurvePartKind::kEdgeBody;
      body.source_edge_id = edge_bundle->edge_id;
      body.source_span_id = entry.key.logical_span_id;
      body.source_bundle_id = span->bundle_id;
      body.bundle_template_id = template_id;
      body.lane_index = binding->lane_index;
      body.has_cable_instance_key = true;
      body.cable_instance_key = entry.key;
      body.boundary_a = helix.front();
      body.boundary_b = helix.back();
      body.tangent_a = carrier.EvaluateTangent(carrier.LengthToU(trim_m));
      body.tangent_b =
          ScaleVec(carrier.EvaluateTangent(carrier.LengthToU(carrier.Length() - trim_m)), -1.0);
      body.sag_method = VisualCurveSagMethod::kNone;
      body.sag_m = 0.0;
      copy_span_appearance(state, entry.key.logical_span_id, &body);
      body.samples = helix;
      body.bounds = curve_part_bounds(body.samples);
      if (const SpanRuntimeState* runtime =
              state.view().find_span_runtime_state(entry.key.logical_span_id);
          runtime != nullptr) {
        body.source_version = runtime->data_version;
      }
      out.parts.push_back(std::move(body));
      continue;
    }
    curve_boundary start_boundary{};
    curve_boundary end_boundary{};
    const bool has_start_patch = boundary_for(boundaries, entry.key, true, &start_boundary);
    const bool has_end_patch = boundary_for(boundaries, entry.key, false, &end_boundary);
    const Vec3d start = has_start_patch ? start_boundary.point : entry.endpoint_a;
    const Vec3d end = has_end_patch ? end_boundary.point : entry.endpoint_b;
    const Vec3d end_boundary_param_tangent = ScaleVec(end_boundary.tangent, -1.0);
    const EditResult<DetailCurve> curve =
        make_curve_between_with_tangent_hints(state, entry.key.logical_span_id, start, end,
                                              has_start_patch ? &start_boundary.tangent : nullptr,
                                              has_end_patch ? &end_boundary_param_tangent : nullptr);
    if (!curve.ok || curve.value.sample_points.size() < 2) {
      continue;
    }
    if (entry.key.is_base()) {
      base_final_curves[entry.key.logical_span_id] = curve.value;
    }
    const Vec3d start_param_tangent = curve.value.start_constraint.tangent_dir;
    const Vec3d end_param_tangent = curve.value.end_constraint.tangent_dir;
    const Vec3d start_outward_tangent = start_param_tangent;
    const Vec3d end_outward_tangent = ScaleVec(end_param_tangent, -1.0);
    if (curve_boundary* boundary = mutable_boundary_for(&boundaries, entry.key, true); boundary != nullptr) {
      boundary->tangent = start_outward_tangent;
    }
    if (curve_boundary* boundary = mutable_boundary_for(&boundaries, entry.key, false); boundary != nullptr) {
      boundary->tangent = end_outward_tangent;
    }

    VisualCurvePart body{};
    body.kind = VisualCurvePartKind::kEdgeBody;
    body.source_edge_id = edge_bundle->edge_id;
    body.source_span_id = entry.key.logical_span_id;
    body.source_bundle_id = span->bundle_id;
    body.bundle_template_id = template_id;
    body.lane_index = binding->lane_index;
    body.has_cable_instance_key = true;
    body.cable_instance_key = entry.key;
    body.endpoint_a_pole_type_id = entry.endpoint_a_pole_type_id;
    body.endpoint_b_pole_type_id = entry.endpoint_b_pole_type_id;
    body.endpoint_a_band_id = entry.endpoint_a_band_id;
    body.endpoint_b_band_id = entry.endpoint_b_band_id;
    body.boundary_a = start;
    body.boundary_b = end;
    body.tangent_a = start_outward_tangent;
    body.tangent_b = end_outward_tangent;
    body.sag_method = VisualCurveSagMethod::kParabolic;
    body.sag_m = curve.value.sag_amplitude_m;
    copy_span_appearance(state, entry.key.logical_span_id, &body);
    body.samples = curve.value.sample_points;
    body.bounds = curve_part_bounds(body.samples);
    if (!body.samples.empty() && finite_point(body.samples.front()) && finite_point(body.samples.back())) {
      if (const SpanRuntimeState* runtime =
              state.view().find_span_runtime_state(entry.key.logical_span_id);
          runtime != nullptr) {
        body.source_version = runtime->data_version;
      }
      out.parts.push_back(std::move(body));
    }
  }

  for (const curve_patch_spec& spec : patch_specs) {
    curve_boundary a_boundary{};
    curve_boundary b_boundary{};
    if (!boundary_for(boundaries, spec.a.cable_instance_key, spec.a.is_start, &a_boundary) ||
        !boundary_for(boundaries, spec.b.cable_instance_key, spec.b.is_start, &b_boundary)) {
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
    copy_span_appearance(state, spec.a.cable_instance_key.logical_span_id, &patch);
    append_patch_section(a_boundary.point, incoming, b_boundary.point, outgoing, true,
                         &patch.bezier_control_points, &patch.samples);
    patch.bounds = curve_part_bounds(patch.samples);
    out.parts.push_back(std::move(patch));
  }
  std::vector<ObjectId> emitted_jumper_ports{};
  for (const curve_endpoint_ref& endpoint : endpoints) {
    if (endpoint.port_id == kInvalidObjectId || endpoint.jumper_peer_port_id == kInvalidObjectId ||
        std::find(emitted_jumper_ports.begin(), emitted_jumper_ports.end(), endpoint.port_id) !=
            emitted_jumper_ports.end()) {
      continue;
    }
    const auto peer = std::find_if(endpoints.begin(), endpoints.end(), [&](const curve_endpoint_ref& candidate) {
      return candidate.port_id == endpoint.jumper_peer_port_id &&
             candidate.jumper_peer_port_id == endpoint.port_id && candidate.node_id == endpoint.node_id &&
             candidate.bundle_template_id == endpoint.bundle_template_id &&
             candidate.lane_index == endpoint.lane_index;
    });
    if (peer == endpoints.end() || Length(peer->point - endpoint.point) <= kCurveEps) {
      continue;
    }

    VisualCurvePart jumper_part{};
    jumper_part.kind = VisualCurvePartKind::kJumper;
    jumper_part.source_node_id = endpoint.node_id;
    jumper_part.source_bundle_id = endpoint.bundle_id;
    jumper_part.bundle_template_id = endpoint.bundle_template_id;
    jumper_part.lane_index = endpoint.lane_index;
    add_unique_incident(endpoint.edge_id, &jumper_part.incident_edge_ids);
    add_unique_incident(peer->edge_id, &jumper_part.incident_edge_ids);
    std::sort(jumper_part.incident_edge_ids.begin(), jumper_part.incident_edge_ids.end());
    jumper_part.boundary_a = endpoint.point;
    jumper_part.boundary_b = peer->point;
    jumper_part.tangent_a = safe_unit(peer->point - endpoint.point, {1.0, 0.0, 0.0});
    jumper_part.tangent_b = jumper_part.tangent_a;
    jumper_part.section_count = 1;
    copy_span_appearance(state, endpoint.cable_instance_key.logical_span_id, &jumper_part);
    append_jumper_section(jumper_part.boundary_a, jumper_part.boundary_b,
                          &jumper_part.bezier_control_points, &jumper_part.samples);
    jumper_part.bounds = curve_part_bounds(jumper_part.samples);
    if (jumper_part.samples.size() >= 2 && finite_point(jumper_part.samples.front()) &&
        finite_point(jumper_part.samples.back())) {
      out.parts.push_back(std::move(jumper_part));
      emitted_jumper_ports.push_back(endpoint.port_id);
      emitted_jumper_ports.push_back(peer->port_id);
    }
  }
  return out;
}

} // namespace wire::core::generation::backbone
