#include "curve_parts.hpp"

#include "wire/core/core_view.hpp"
#include "wire/core/numeric_tolerances.hpp"
#include "wire/core/coord_utils.hpp"

#include "../../support/hash_mix.hpp"
#include "out.hpp"
#include "population.hpp"
#include "row_representation.hpp"
#include "span_visual_assembly.hpp"
#include "../../geometry/detail_curve_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace wire::core::generation::backbone {
namespace {

constexpr double kNodePatchHorizontalLengthM = 0.35;
constexpr double kNodePatchMaxSpanFraction = 0.25;
constexpr double kCurveEps = kLengthToleranceM;
constexpr double kPatchMetersPerSegment = 0.08;
constexpr double kPatchRadiansPerSegment = 0.17453292519943295;

struct curve_endpoint_ref {
  CableSectionKey section_key{};
  ObjectId node_id = kInvalidObjectId;
  ObjectId edge_id = kInvalidObjectId;
  ObjectId edge_bundle_id = kInvalidObjectId;
  ObjectId bundle_id = kInvalidObjectId;
  ObjectId port_id = kInvalidObjectId;
  ObjectId jumper_peer_port_id = kInvalidObjectId;
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  std::size_t lane_index = 0;
  PoleTypeId pole_type_id = kInvalidPoleTypeId;
  int band_id = 0;
  SavedBackboneRowKey row_key{};
  Vec3d point{};
  Vec3d away_from_node{};
  double span_length_m = 0.0;
  std::size_t source_curve_index = std::numeric_limits<std::size_t>::max();
  bool has_curve_tangent = false;
  bool is_start = true;
};

struct curve_patch_key {
  ObjectId node_id = kInvalidObjectId;
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  std::size_t lane_index = 0;
  PoleTypeId pole_type_id = kInvalidPoleTypeId;
  int band_id = 0;
  std::uint64_t rule_owner_id = 0;
  CableSectionRuleId rule_id = 0;
  std::size_t instance_index = 0;
};

struct curve_boundary_key {
  CableSectionKey section_key{};
  bool is_start = true;
};

struct curve_boundary {
  CableSectionKey section_key{};
  bool is_start = true;
  Vec3d attachment_point{};
  double horizontal_length_m = 0.0;
  double source_u = 0.0;
  Vec3d point{};
  Vec3d tangent{};
};

struct curve_patch_spec {
  curve_patch_key key{};
  curve_endpoint_ref a{};
  curve_endpoint_ref b{};
  Vec3d attachment_point{};
};

struct continuity_endpoint_key {
  ObjectId node_id = kInvalidObjectId;
  ObjectId edge_bundle_id = kInvalidObjectId;
  std::size_t lane_index = 0;
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
  std::size_t source_curve_index = std::numeric_limits<std::size_t>::max();
};

bool same_cable_section(const CableSectionKey& a, const CableSectionKey& b) {
  return a.logical_span_id == b.logical_span_id && a.edge_bundle_id == b.edge_bundle_id &&
         a.rule_owner_id == b.rule_owner_id && a.rule_id == b.rule_id &&
         a.instance_index == b.instance_index;
}

bool cable_section_less_for_run(const CableSectionKey& a, const CableSectionKey& b) {
  if (a.edge_bundle_id != b.edge_bundle_id) {
    return a.edge_bundle_id < b.edge_bundle_id;
  }
  if (a.logical_span_id != b.logical_span_id) {
    return a.logical_span_id < b.logical_span_id;
  }
  if (a.rule_owner_id != b.rule_owner_id) {
    return a.rule_owner_id < b.rule_owner_id;
  }
  if (a.rule_id != b.rule_id) {
    return a.rule_id < b.rule_id;
  }
  return a.instance_index < b.instance_index;
}

using support::hash_combine;

CableRunId run_id_from_canonical_section(const CableSectionKey& key) {
  std::uint64_t hash = hash_combine(0, static_cast<std::uint64_t>(key.edge_bundle_id));
  hash = hash_combine(hash, static_cast<std::uint64_t>(key.logical_span_id));
  hash = hash_combine(hash, key.rule_owner_id);
  hash = hash_combine(hash, static_cast<std::uint64_t>(key.rule_id));
  hash = hash_combine(hash, static_cast<std::uint64_t>(key.instance_index));
  return hash == 0 ? 1 : hash;
}

struct cable_run_assignment {
  CableSectionKey section_key{};
  std::size_t parent = 0;
  CableSectionKey canonical_key{};
  CableRunId run_id = 0;
};

struct cable_section_key_hash {
  std::size_t operator()(const CableSectionKey& key) const {
    std::uint64_t hash = hash_combine(0, static_cast<std::uint64_t>(key.edge_bundle_id));
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.logical_span_id));
    hash = hash_combine(hash, key.rule_owner_id);
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.rule_id));
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.instance_index));
    return static_cast<std::size_t>(hash);
  }
};

struct cable_section_key_equal {
  bool operator()(const CableSectionKey& a, const CableSectionKey& b) const {
    return same_cable_section(a, b);
  }
};

struct cable_run_assignments {
  std::vector<cable_run_assignment> items{};
  std::unordered_map<CableSectionKey, std::size_t, cable_section_key_hash, cable_section_key_equal>
      index_by_section{};
};

std::size_t find_run_root(cable_run_assignments* assignments, std::size_t index) {
  std::size_t parent = assignments->items[index].parent;
  if (parent != index) {
    parent = find_run_root(assignments, parent);
    assignments->items[index].parent = parent;
  }
  return parent;
}

std::size_t ensure_run_section(cable_run_assignments* assignments, const CableSectionKey& key) {
  if (const auto found = assignments->index_by_section.find(key);
      found != assignments->index_by_section.end()) {
    return found->second;
  }
  const std::size_t index = assignments->items.size();
  assignments->items.push_back({key, index, key, 0});
  assignments->index_by_section.emplace(key, index);
  return index;
}

void union_run_sections(cable_run_assignments* assignments, const CableSectionKey& a,
                        const CableSectionKey& b) {
  const std::size_t a_index = ensure_run_section(assignments, a);
  const std::size_t b_index = ensure_run_section(assignments, b);
  const std::size_t a_root = find_run_root(assignments, a_index);
  const std::size_t b_root = find_run_root(assignments, b_index);
  if (a_root == b_root) {
    return;
  }
  const CableSectionKey canonical =
      cable_section_less_for_run(assignments->items[a_root].canonical_key,
                                 assignments->items[b_root].canonical_key)
          ? assignments->items[a_root].canonical_key
          : assignments->items[b_root].canonical_key;
  assignments->items[b_root].parent = a_root;
  assignments->items[a_root].canonical_key = canonical;
}

cable_run_assignments derive_cable_run_ids(const std::vector<visual_cable_section>& sections,
                                           const std::vector<curve_patch_spec>& patch_specs) {
  cable_run_assignments assignments{};
  assignments.items.reserve(sections.size());
  assignments.index_by_section.reserve(sections.size());
  for (const visual_cable_section& section : sections) {
    ensure_run_section(&assignments, section.layout.key);
  }
  for (const curve_patch_spec& spec : patch_specs) {
    union_run_sections(&assignments, spec.a.section_key, spec.b.section_key);
  }
  for (std::size_t index = 0; index < assignments.items.size(); ++index) {
    cable_run_assignment& assignment = assignments.items[index];
    const std::size_t root = find_run_root(&assignments, index);
    const CableSectionKey& canonical = assignments.items[root].canonical_key;
    assignment.run_id = run_id_from_canonical_section(canonical);
  }
  return assignments;
}

CableRunId run_id_for_section(const cable_run_assignments& assignments, const CableSectionKey& key) {
  const auto found = assignments.index_by_section.find(key);
  return found == assignments.index_by_section.end() ? 0 : assignments.items[found->second].run_id;
}

bool same_key(const curve_patch_key& a, const curve_patch_key& b) {
  return a.node_id == b.node_id && a.bundle_template_id == b.bundle_template_id &&
         a.lane_index == b.lane_index && a.pole_type_id == b.pole_type_id &&
         a.band_id == b.band_id && a.rule_owner_id == b.rule_owner_id &&
         a.rule_id == b.rule_id && a.instance_index == b.instance_index;
}

curve_patch_key patch_key_for(const curve_endpoint_ref& endpoint) {
  return {endpoint.node_id, endpoint.bundle_template_id, endpoint.lane_index,
          endpoint.pole_type_id, endpoint.band_id, endpoint.section_key.rule_owner_id,
          endpoint.section_key.rule_id, endpoint.section_key.instance_index};
}

struct curve_patch_key_hash {
  std::size_t operator()(const curve_patch_key& key) const {
    std::uint64_t hash = hash_combine(0, static_cast<std::uint64_t>(key.node_id));
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.bundle_template_id));
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.lane_index));
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.pole_type_id));
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.band_id));
    hash = hash_combine(hash, key.rule_owner_id);
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.rule_id));
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.instance_index));
    return static_cast<std::size_t>(hash);
  }
};

struct curve_patch_key_equal {
  bool operator()(const curve_patch_key& a, const curve_patch_key& b) const {
    return same_key(a, b);
  }
};

bool same_boundary_key(const curve_boundary_key& a, const curve_boundary_key& b) {
  return same_cable_section(a.section_key, b.section_key) && a.is_start == b.is_start;
}

struct curve_boundary_key_hash {
  std::size_t operator()(const curve_boundary_key& key) const {
    std::uint64_t hash = hash_combine(0, static_cast<std::uint64_t>(key.section_key.edge_bundle_id));
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.section_key.logical_span_id));
    hash = hash_combine(hash, key.section_key.rule_owner_id);
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.section_key.rule_id));
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.section_key.instance_index));
    hash = hash_combine(hash, key.is_start ? 1u : 0u);
    return static_cast<std::size_t>(hash);
  }
};

struct curve_boundary_key_equal {
  bool operator()(const curve_boundary_key& a, const curve_boundary_key& b) const {
    return same_boundary_key(a, b);
  }
};

struct continuity_endpoint_key_hash {
  std::size_t operator()(const continuity_endpoint_key& key) const {
    std::uint64_t hash = hash_combine(0, static_cast<std::uint64_t>(key.node_id));
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.edge_bundle_id));
    hash = hash_combine(hash, static_cast<std::uint64_t>(key.lane_index));
    return static_cast<std::size_t>(hash);
  }
};

struct continuity_endpoint_key_equal {
  bool operator()(const continuity_endpoint_key& a, const continuity_endpoint_key& b) const {
    return a.node_id == b.node_id && a.edge_bundle_id == b.edge_bundle_id &&
           a.lane_index == b.lane_index;
  }
};

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

using boundary_index_map = std::unordered_map<curve_boundary_key, std::size_t, curve_boundary_key_hash,
                                              curve_boundary_key_equal>;

bool boundary_for(const std::vector<curve_boundary>& boundaries, const boundary_index_map& index,
                  const CableSectionKey& section_key, bool is_start, curve_boundary* out) {
  const auto it = index.find({section_key, is_start});
  if (it == index.end() || it->second >= boundaries.size()) {
    return false;
  }
  if (out != nullptr) {
    *out = boundaries[it->second];
  }
  return true;
}

curve_boundary* mutable_boundary_for(std::vector<curve_boundary>* boundaries, const boundary_index_map& index,
                                     const CableSectionKey& section_key, bool is_start) {
  if (boundaries == nullptr) {
    return nullptr;
  }
  const auto it = index.find({section_key, is_start});
  if (it == index.end() || it->second >= boundaries->size()) {
    return nullptr;
  }
  return &(*boundaries)[it->second];
}

void insert_boundary_once(std::vector<curve_boundary>* boundaries, boundary_index_map* index,
                          const curve_boundary& boundary) {
  if (boundaries == nullptr || index == nullptr) {
    return;
  }
  const curve_boundary_key key{boundary.section_key, boundary.is_start};
  if (index->find(key) != index->end()) {
    return;
  }
  (*index)[key] = boundaries->size();
  boundaries->push_back(boundary);
}

curve_boundary boundary_from_source_curve(const curve_endpoint_ref& endpoint,
                                          const DetailCurve& curve,
                                          double horizontal_length_m) {
  curve_boundary boundary{};
  boundary.section_key = endpoint.section_key;
  boundary.is_start = endpoint.is_start;
  boundary.attachment_point = endpoint.point;
  boundary.horizontal_length_m = horizontal_length_m;
  const double length_from_start = endpoint.is_start
                                       ? horizontal_length_m
                                       : std::max(0.0, curve.Length() - horizontal_length_m);
  const double u = curve.LengthToU(length_from_start);
  boundary.source_u = u;
  boundary.point = curve.EvaluatePosition(u);
  boundary.tangent = curve.EvaluateTangent(u);
  if (!endpoint.is_start) {
    boundary.tangent = ScaleVec(boundary.tangent, -1.0);
  }
  return boundary;
}

std::vector<Vec3d> trimmed_source_curve_samples(const DetailCurve& curve, double start_u, double end_u) {
  std::vector<Vec3d> samples{};
  start_u = std::clamp(start_u, 0.0, 1.0);
  end_u = std::clamp(end_u, start_u, 1.0);
  samples.reserve(curve.sample_points.size() + 2);
  samples.push_back(curve.EvaluatePosition(start_u));
  if (curve.sample_points.size() >= 2) {
    const double denominator = static_cast<double>(curve.sample_points.size() - 1);
    for (std::size_t index = 1; index + 1 < curve.sample_points.size(); ++index) {
      const double u = static_cast<double>(index) / denominator;
      if (u > start_u + kCurveEps && u < end_u - kCurveEps) {
        samples.push_back(curve.sample_points[index]);
      }
    }
  }
  const Vec3d end = curve.EvaluatePosition(end_u);
  if (samples.empty() || Length(end - samples.back()) > kCurveEps) {
    samples.push_back(end);
  }
  return samples;
}

ObjectId saved_node_id_for_endpoint(const CoreState& state, ObjectId endpoint_node_id) {
  if (const SavedBackboneNode* node = state.view().backbone_node_for_pole(endpoint_node_id); node != nullptr) {
    return node->node_id;
  }
  if (const SavedBackboneNode* node = state.view().backbone_node(endpoint_node_id); node != nullptr) {
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

EditResult<ObjectId> derived_jumper_peer_port(
    const CoreState& state, const SavedBackbonePortBinding& binding) {
  EditResult<ObjectId> out{};
  out.ok = true;
  const EditResult<EndpointRowRepresentation> representation =
      DeriveEndpointRowRepresentation(state, binding);
  if (!representation.ok) {
    out.ok = false;
    out.error = representation.error;
    return out;
  }
  if (!representation.value.sharp) {
    return out;
  }
  for (const SavedBackbonePortBinding* candidate :
       state.view().backbone_port_bindings_for_edge_bundle(
           representation.value.peer_edge_bundle_id)) {
    if (candidate == nullptr ||
        candidate->lane_index != representation.value.peer_lane_index ||
        candidate->row_key.node_id != binding.row_key.node_id) {
      continue;
    }
    if (out.value != kInvalidObjectId &&
        out.value != candidate->port_id) {
      out.ok = false;
      out.error =
          "backbone internal: sharp endpoint peer port is ambiguous";
      return out;
    }
    out.value = candidate->port_id;
  }
  if (out.value == kInvalidObjectId) {
    out.ok = false;
    out.error = "backbone internal: sharp endpoint peer port is missing";
  }
  return out;
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

std::uint64_t span_source_version(const CoreState& state, ObjectId span_id) {
  const SpanRuntimeState* runtime = state.view().find_span_runtime_state(span_id);
  return runtime == nullptr ? 0 : runtime->data_version;
}

std::uint64_t quantized_source_value(double value) {
  return static_cast<std::uint64_t>(std::llround(value * 1'000'000'000.0));
}

std::uint64_t body_source_version(std::uint64_t span_version, double start_u, double end_u) {
  std::uint64_t version = hash_combine(0, span_version);
  version = hash_combine(version, quantized_source_value(start_u));
  version = hash_combine(version, quantized_source_value(end_u));
  return version == 0 ? 1 : version;
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

std::unordered_set<ObjectId> span_set_from(const std::vector<ObjectId>& span_ids) {
  std::unordered_set<ObjectId> out{};
  out.reserve(span_ids.size());
  for (ObjectId span_id : span_ids) {
    if (span_id != kInvalidObjectId) {
      out.insert(span_id);
    }
  }
  return out;
}

std::unordered_set<ObjectId> saved_nodes_for_spans(const CoreState& state, const std::unordered_set<ObjectId>& span_ids) {
  std::unordered_set<ObjectId> nodes{};
  for (ObjectId span_id : span_ids) {
    const SavedBackboneSpanBinding* binding = span_binding_for(state, span_id);
    const SavedBackboneEdgeBundle* edge_bundle =
        binding == nullptr ? nullptr : state.view().backbone_edge_bundle(binding->edge_bundle_id);
    const SavedBackboneEdge* edge = edge_bundle == nullptr ? nullptr : state.view().backbone_edge(edge_bundle->edge_id);
    if (edge == nullptr) {
      continue;
    }
    nodes.insert(edge->node_a);
    nodes.insert(edge->node_b);
  }
  return nodes;
}

std::unordered_set<ObjectId> scoped_visual_spans(const CoreState& state, const std::vector<ObjectId>& scope_span_ids) {
  std::unordered_set<ObjectId> spans = span_set_from(scope_span_ids);
  if (spans.empty()) {
    return spans;
  }
  const std::unordered_set<ObjectId> nodes = saved_nodes_for_spans(state, spans);
  if (nodes.empty()) {
    return spans;
  }
  const SavedBackboneGraph& graph = state.view().backbone();
  for (const SavedBackboneSpanBinding& binding : graph.span_bindings) {
    const SavedBackboneEdgeBundle* edge_bundle = state.view().backbone_edge_bundle(binding.edge_bundle_id);
    const SavedBackboneEdge* edge = edge_bundle == nullptr ? nullptr : state.view().backbone_edge(edge_bundle->edge_id);
    if (edge == nullptr) {
      continue;
    }
    if (nodes.contains(edge->node_a) || nodes.contains(edge->node_b)) {
      spans.insert(binding.span_id);
    }
  }
  return spans;
}

layout filter_layouts_to_spans(const layout& placed, const std::unordered_set<ObjectId>& span_ids) {
  if (span_ids.empty()) {
    return placed;
  }
  layout filtered{};
  for (const SpanLayoutEntry& entry : placed.entries) {
    if (span_ids.contains(entry.span_id)) {
      filtered.entries.push_back(entry);
    }
  }
  return filtered;
}

int visual_part_kind_order(VisualCurvePartKind kind) {
  switch (kind) {
  case VisualCurvePartKind::kEdgeBody:
    return 0;
  case VisualCurvePartKind::kNodePatch:
    return 1;
  case VisualCurvePartKind::kLead:
    return 2;
  case VisualCurvePartKind::kJumper:
    return 3;
  case VisualCurvePartKind::kSupplemental:
    return 4;
  }
  return 5;
}

bool section_key_less(const CableSectionKey& a, const CableSectionKey& b) {
  if (a.logical_span_id != b.logical_span_id) {
    return a.logical_span_id < b.logical_span_id;
  }
  if (a.edge_bundle_id != b.edge_bundle_id) {
    return a.edge_bundle_id < b.edge_bundle_id;
  }
  if (a.rule_owner_id != b.rule_owner_id) {
    return a.rule_owner_id < b.rule_owner_id;
  }
  if (a.rule_id != b.rule_id) {
    return a.rule_id < b.rule_id;
  }
  return a.instance_index < b.instance_index;
}

bool visual_part_less(const VisualCurvePart& a, const VisualCurvePart& b) {
  const int kind_a = visual_part_kind_order(a.kind);
  const int kind_b = visual_part_kind_order(b.kind);
  if (kind_a != kind_b) {
    return kind_a < kind_b;
  }
  if (a.source_span_id != b.source_span_id) {
    return a.source_span_id < b.source_span_id;
  }
  if (a.source_node_id != b.source_node_id) {
    return a.source_node_id < b.source_node_id;
  }
  if (a.bundle_template_id != b.bundle_template_id) {
    return a.bundle_template_id < b.bundle_template_id;
  }
  if (a.lane_index != b.lane_index) {
    return a.lane_index < b.lane_index;
  }
  if (a.has_section_key != b.has_section_key) {
    return a.has_section_key;
  }
  if (a.has_section_key && !same_cable_section(a.section_key, b.section_key)) {
    return section_key_less(a.section_key, b.section_key);
  }
  return a.cable_run_id < b.cable_run_id;
}

void sort_visual_parts(VisualCurvePartCache* cache) {
  if (cache == nullptr) {
    return;
  }
  std::sort(cache->parts.begin(), cache->parts.end(), visual_part_less);
}

VisualCurvePartCache merge_scoped_visual_curve_parts(const CoreState& state, VisualCurvePartCache rebuilt,
                                                     const std::unordered_set<ObjectId>& rebuilt_spans) {
  if (rebuilt_spans.empty()) {
    sort_visual_parts(&rebuilt);
    return rebuilt;
  }
  const std::unordered_set<ObjectId> affected_nodes = saved_nodes_for_spans(state, rebuilt_spans);
  VisualCurvePartCache merged = state.view().visual_curve_parts();
  merged.parts.erase(std::remove_if(merged.parts.begin(), merged.parts.end(), [&](const VisualCurvePart& part) {
                       if (part.source_span_id != kInvalidObjectId && rebuilt_spans.contains(part.source_span_id)) {
                         return true;
                       }
                       if (part.has_section_key && rebuilt_spans.contains(part.section_key.logical_span_id)) {
                         return true;
                       }
                       return part.source_node_id != kInvalidObjectId && affected_nodes.contains(part.source_node_id);
                     }),
                     merged.parts.end());
  merged.parts.insert(merged.parts.end(), std::make_move_iterator(rebuilt.parts.begin()),
                      std::make_move_iterator(rebuilt.parts.end()));
  merged.diagnostics = std::move(rebuilt.diagnostics);
  merged.population_diagnostics = std::move(rebuilt.population_diagnostics);
  merged.stats = rebuilt.stats;
  sort_visual_parts(&merged);
  return merged;
}

} // namespace

EditResult<VisualCurvePartCache> make_visual_curve_parts(const CoreState& state, const layout& made,
                                                         const std::vector<ObjectId>& scope_span_ids,
                                                         const curve* built_curves) {
  EditResult<VisualCurvePartCache> result{};
  const layout merged_layout = merged_visual_curve_layouts(state, made);
  const std::unordered_set<ObjectId> scoped_spans = scoped_visual_spans(state, scope_span_ids);
  const std::unordered_set<ObjectId> scoped_nodes = saved_nodes_for_spans(state, scoped_spans);
  const layout placed = filter_layouts_to_spans(merged_layout, scoped_spans);
  std::unordered_map<ObjectId, const DetailCurve*> built_curve_by_span{};
  if (built_curves != nullptr) {
    built_curve_by_span.reserve(built_curves->data.size());
    for (const auto& [span_id, detail] : built_curves->data) {
      built_curve_by_span.emplace(span_id, &detail);
    }
  }
  VisualCurvePartCache out{};
  auto fail = [&](std::string message) {
    result.error = std::move(message);
  };
  std::vector<visual_cable_section> sections{};
  sections.reserve(placed.entries.size());
  for (const SpanLayoutEntry& entry : placed.entries) {
    const SavedBackboneSpanBinding* binding = span_binding_for(state, entry.span_id);
    if (binding == nullptr) {
      out.diagnostics.push_back(
          {kInvalidObjectId, entry.span_id, kInvalidBundleTemplateId, 0, "span binding missing"});
      continue;
    }
    const Span* span = state.view().spans().find(entry.span_id);
    const Bundle* bundle = span == nullptr ? nullptr : state.view().bundles().find(span->bundle_id);
    if (span == nullptr || bundle == nullptr) {
      out.diagnostics.push_back(
          {kInvalidObjectId, entry.span_id, kInvalidBundleTemplateId, binding->lane_index, "span bundle missing"});
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
    const EditResult<ObjectId> start_peer =
        derived_jumper_peer_port(state, *start_binding);
    const EditResult<ObjectId> end_peer =
        derived_jumper_peer_port(state, *end_binding);
    if (!start_peer.ok || !end_peer.ok) {
      fail(start_peer.ok ? end_peer.error : start_peer.error);
      return result;
    }
    section.start_jumper_peer_port_id = start_peer.value;
    section.end_jumper_peer_port_id = end_peer.value;
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
  out.stats.sections = sections.size();

  std::vector<curve_endpoint_ref> endpoints{};
  std::vector<DetailCurve> source_curves{};
  endpoints.reserve(sections.size() * 2);
  source_curves.reserve(sections.size());
  const Vec3d fallback_dir{1.0, 0.0, 0.0};

  for (visual_cable_section& section : sections) {
    const CableSectionLayout& entry = section.layout;
    const Span* span = state.view().spans().find(entry.key.logical_span_id);
    const SavedBackboneSpanBinding* binding = span_binding_for(state, entry.key.logical_span_id);
    const SavedBackboneEdgeBundle* edge_bundle =
        binding == nullptr ? nullptr : state.view().backbone_edge_bundle(binding->edge_bundle_id);
    if (span == nullptr || binding == nullptr || edge_bundle == nullptr) {
      out.diagnostics.push_back(
          {kInvalidObjectId, entry.key.logical_span_id, kInvalidBundleTemplateId, 0,
           "curve source identity missing"});
      continue;
    }
    const Bundle* bundle = state.view().bundles().find(span->bundle_id);
    if (bundle == nullptr) {
      out.diagnostics.push_back(
          {kInvalidObjectId, entry.key.logical_span_id, kInvalidBundleTemplateId, binding->lane_index,
           "curve source bundle missing"});
      continue;
    }
    const BundleTemplateId template_id = bundle->bundle_template_id;
    const Vec3d chord = entry.endpoint_b - entry.endpoint_a;
    const double span_length = Length(chord);
    DetailCurve source_curve{};
    bool has_curve_tangent = false;
    if (entry.key.is_base()) {
      const auto built = built_curve_by_span.find(entry.key.logical_span_id);
      if (built != built_curve_by_span.end() && built->second->sample_points.size() >= 2) {
        source_curve = *built->second;
        has_curve_tangent = true;
      } else if (const CurveCacheEntry* cached = state.find_curve_cache(entry.key.logical_span_id);
                 cached != nullptr && cached->detail.sample_points.size() >= 2) {
        source_curve = cached->detail;
        has_curve_tangent = true;
      }
    }
    if (!has_curve_tangent) {
      const EditResult<DetailCurve> built =
          make_curve_between(state, entry.key.logical_span_id, entry.endpoint_a, entry.endpoint_b);
      if (built.ok && built.value.sample_points.size() >= 2) {
        source_curve = built.value;
        has_curve_tangent = true;
      }
    }
    const std::size_t source_curve_index = source_curves.size();
    if (has_curve_tangent) {
      source_curves.push_back(std::move(source_curve));
      section.source_curve_index = source_curve_index;
    }
    const Vec3d start_away =
        has_curve_tangent ? source_curves[source_curve_index].start_constraint.tangent_dir
                          : safe_unit(chord, fallback_dir);
    const Vec3d end_away = has_curve_tangent
                              ? ScaleVec(source_curves[source_curve_index].end_constraint.tangent_dir, -1.0)
                              : safe_unit(ScaleVec(chord, -1.0), ScaleVec(fallback_dir, -1.0));

    curve_endpoint_ref start{};
    start.section_key = entry.key;
    start.node_id = section.start_row_key.node_id != kInvalidObjectId
        ? section.start_row_key.node_id
        : saved_node_id_for_endpoint(state, section.start_node_id);
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
    start.source_curve_index = source_curve_index;
    start.has_curve_tangent = has_curve_tangent;
    start.is_start = true;
    endpoints.push_back(start);

    curve_endpoint_ref end = start;
    end.node_id = section.end_row_key.node_id != kInvalidObjectId
        ? section.end_row_key.node_id
        : saved_node_id_for_endpoint(state, section.end_node_id);
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
  boundary_index_map boundary_index{};
  std::vector<curve_patch_spec> patch_specs{};

  std::vector<curve_patch_key> patch_key_order{};
  std::unordered_set<curve_patch_key, curve_patch_key_hash, curve_patch_key_equal> seen_patch_keys{};
  patch_key_order.reserve(endpoints.size());
  for (std::size_t endpoint_index = 0; endpoint_index < endpoints.size(); ++endpoint_index) {
    const curve_patch_key key = patch_key_for(endpoints[endpoint_index]);
    if (seen_patch_keys.insert(key).second) {
      patch_key_order.push_back(key);
    }
  }

  std::unordered_map<continuity_endpoint_key, std::vector<std::size_t>,
                     continuity_endpoint_key_hash, continuity_endpoint_key_equal>
      endpoints_by_continuity_key{};
  endpoints_by_continuity_key.reserve(endpoints.size());
  for (std::size_t endpoint_index = 0; endpoint_index < endpoints.size(); ++endpoint_index) {
    const curve_endpoint_ref& endpoint = endpoints[endpoint_index];
    endpoints_by_continuity_key[continuity_endpoint_key{
        endpoint.node_id, endpoint.edge_bundle_id, endpoint.lane_index}].push_back(endpoint_index);
  }
  auto has_any_endpoint = [&](ObjectId edge_bundle_id, std::size_t lane) {
    return std::any_of(endpoints_by_continuity_key.begin(), endpoints_by_continuity_key.end(), [&](const auto& item) {
      return item.first.edge_bundle_id == edge_bundle_id && item.first.lane_index == lane;
    });
  };
  auto continuity_endpoint_outside_scope = [&](ObjectId edge_bundle_id, std::size_t lane) {
    if (scoped_spans.empty()) {
      return false;
    }
    const SavedBackboneEdgeBundle* edge_bundle = state.view().backbone_edge_bundle(edge_bundle_id);
    if (edge_bundle == nullptr) {
      return false;
    }
    return !has_any_endpoint(edge_bundle_id, lane);
  };

  std::unordered_map<curve_patch_key, std::vector<std::pair<std::size_t, std::size_t>>,
                     curve_patch_key_hash, curve_patch_key_equal>
      continuity_pairs_by_patch_key{};
  continuity_pairs_by_patch_key.reserve(state.view().backbone().row_continuities.size());
  for (const SavedBackboneRowContinuity& continuity : state.view().backbone().row_continuities) {
    if (!scoped_spans.empty() && !scoped_nodes.contains(continuity.node_id)) {
      continue;
    }
    const auto a_it = endpoints_by_continuity_key.find(
        continuity_endpoint_key{continuity.node_id, continuity.a.edge_bundle_id, continuity.a.lane_index});
    const auto b_it = endpoints_by_continuity_key.find(
        continuity_endpoint_key{continuity.node_id, continuity.b.edge_bundle_id, continuity.b.lane_index});
    if (a_it == endpoints_by_continuity_key.end() || b_it == endpoints_by_continuity_key.end()) {
      if (!has_any_endpoint(continuity.a.edge_bundle_id,
                            continuity.a.lane_index) ||
          !has_any_endpoint(continuity.b.edge_bundle_id,
                            continuity.b.lane_index)) {
        continue;
      }
      if (!scoped_spans.empty()) {
        continue;
      }
      const bool a_outside = a_it == endpoints_by_continuity_key.end() &&
                             continuity_endpoint_outside_scope(continuity.a.edge_bundle_id,
                                                               continuity.a.lane_index);
      const bool b_outside = b_it == endpoints_by_continuity_key.end() &&
                             continuity_endpoint_outside_scope(continuity.b.edge_bundle_id,
                                                               continuity.b.lane_index);
      if (a_outside || b_outside) {
        continue;
      }
      const SavedBackboneEdgeBundle* a_bundle = state.view().backbone_edge_bundle(continuity.a.edge_bundle_id);
      const SavedBackboneEdgeBundle* b_bundle = state.view().backbone_edge_bundle(continuity.b.edge_bundle_id);
      std::string endpoint_nodes{};
      for (const auto& item : endpoints_by_continuity_key) {
        if ((item.first.edge_bundle_id == continuity.a.edge_bundle_id &&
             item.first.lane_index == continuity.a.lane_index) ||
            (item.first.edge_bundle_id == continuity.b.edge_bundle_id &&
             item.first.lane_index == continuity.b.lane_index)) {
          endpoint_nodes += " key=" + std::to_string(item.first.node_id) + "/" +
                            std::to_string(item.first.edge_bundle_id) + "/" +
                            std::to_string(item.first.lane_index);
        }
      }
      fail("backbone internal: row continuity endpoint is missing node=" +
           std::to_string(continuity.node_id) + " a=" + std::to_string(continuity.a.edge_bundle_id) +
           "/" + std::to_string(continuity.a.lane_index) + "/spans=" +
           std::to_string(a_bundle == nullptr ? 0 : a_bundle->span_ids.size()) + " b=" +
           std::to_string(continuity.b.edge_bundle_id) + "/" + std::to_string(continuity.b.lane_index) +
           "/spans=" + std::to_string(b_bundle == nullptr ? 0 : b_bundle->span_ids.size()) +
           endpoint_nodes);
      return result;
    }
    for (std::size_t endpoint_a_index : a_it->second) {
      for (std::size_t endpoint_b_index : b_it->second) {
        const curve_endpoint_ref& endpoint_a = endpoints[endpoint_a_index];
        const curve_endpoint_ref& endpoint_b = endpoints[endpoint_b_index];
        if (endpoint_a.edge_id == endpoint_b.edge_id) {
          continue;
        }
        const curve_patch_key key = patch_key_for(endpoint_a);
        if (!same_key(key, patch_key_for(endpoint_b))) {
          continue;
        }
        continuity_pairs_by_patch_key[key].push_back({endpoint_a_index, endpoint_b_index});
      }
    }
  }

  for (const curve_patch_key& key : patch_key_order) {
    const auto candidates_it = continuity_pairs_by_patch_key.find(key);
    if (candidates_it == continuity_pairs_by_patch_key.end() || candidates_it->second.empty()) {
      continue;
    }
    const std::vector<std::pair<std::size_t, std::size_t>>& candidates = candidates_it->second;
    std::unordered_set<std::size_t> used_group_endpoints{};
    bool ambiguous = false;
    for (const auto& candidate : candidates) {
      if (!used_group_endpoints.insert(candidate.first).second ||
          !used_group_endpoints.insert(candidate.second).second) {
        ambiguous = true;
        break;
      }
    }
    if (ambiguous) {
      fail("backbone internal: multiple overlapping connectivity-owned patch pairs");
      return result;
    }

    for (const auto& candidate : candidates) {
      const curve_endpoint_ref& raw_patch_a = endpoints[candidate.first];
      const curve_endpoint_ref& raw_patch_b = endpoints[candidate.second];
      const curve_endpoint_ref& patch_a =
          section_key_less(raw_patch_a.section_key, raw_patch_b.section_key) ? raw_patch_a : raw_patch_b;
      const curve_endpoint_ref& patch_b =
          section_key_less(raw_patch_a.section_key, raw_patch_b.section_key) ? raw_patch_b : raw_patch_a;
      if (!patch_a.has_curve_tangent || !patch_b.has_curve_tangent) {
        fail("backbone internal: patch endpoint tangent missing");
        return result;
      }
      if (patch_a.jumper_peer_port_id != kInvalidObjectId ||
          patch_b.jumper_peer_port_id != kInvalidObjectId) {
        continue;
      }
      const double a_len =
          std::min(kNodePatchHorizontalLengthM, patch_a.span_length_m * kNodePatchMaxSpanFraction);
      const double b_len =
          std::min(kNodePatchHorizontalLengthM, patch_b.span_length_m * kNodePatchMaxSpanFraction);
      if (a_len <= kCurveEps || b_len <= kCurveEps) {
        fail("backbone internal: patch boundary length is zero");
        return result;
      }

      curve_boundary a_boundary = boundary_from_source_curve(
          patch_a, source_curves[patch_a.source_curve_index], a_len);
      insert_boundary_once(&boundaries, &boundary_index, a_boundary);

      curve_boundary b_boundary = boundary_from_source_curve(
          patch_b, source_curves[patch_b.source_curve_index], b_len);
      insert_boundary_once(&boundaries, &boundary_index, b_boundary);
      patch_specs.push_back({key, patch_a, patch_b, ScaleVec(patch_a.point + patch_b.point, 0.5)});
    }
  }

  const cable_run_assignments cable_runs = derive_cable_run_ids(sections, patch_specs);

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
    const BundleTemplateId template_id = bundle->bundle_template_id;
    curve_boundary start_boundary{};
    curve_boundary end_boundary{};
    const bool has_start_patch = boundary_for(boundaries, boundary_index, entry.key, true, &start_boundary);
    const bool has_end_patch = boundary_for(boundaries, boundary_index, entry.key, false, &end_boundary);
    const Vec3d start = has_start_patch ? start_boundary.point : entry.endpoint_a;
    const Vec3d end = has_end_patch ? end_boundary.point : entry.endpoint_b;
    ++out.stats.curve_builds;
    if (section.source_curve_index >= source_curves.size()) {
      continue;
    }
    const DetailCurve& source_curve = source_curves[section.source_curve_index];
    const double start_u = has_start_patch ? start_boundary.source_u : 0.0;
    const double end_u = has_end_patch ? end_boundary.source_u : 1.0;
    std::vector<Vec3d> body_samples = trimmed_source_curve_samples(source_curve, start_u, end_u);
    if (body_samples.size() < 2) {
      continue;
    }
    const Vec3d start_param_tangent = source_curve.EvaluateTangent(start_u);
    const Vec3d end_param_tangent = source_curve.EvaluateTangent(end_u);
    const Vec3d start_outward_tangent = start_param_tangent;
    const Vec3d end_outward_tangent = ScaleVec(end_param_tangent, -1.0);
    if (curve_boundary* boundary = mutable_boundary_for(&boundaries, boundary_index, entry.key, true);
        boundary != nullptr) {
      boundary->tangent = start_outward_tangent;
    }
    if (curve_boundary* boundary = mutable_boundary_for(&boundaries, boundary_index, entry.key, false);
        boundary != nullptr) {
      boundary->tangent = end_outward_tangent;
    }

    VisualCurvePart body{};
    body.kind = VisualCurvePartKind::kEdgeBody;
    body.source_edge_id = edge_bundle->edge_id;
    body.source_span_id = entry.key.logical_span_id;
    body.source_bundle_id = span->bundle_id;
    body.bundle_template_id = template_id;
    body.lane_index = binding->lane_index;
    body.has_section_key = true;
    body.section_key = entry.key;
    body.cable_run_id = run_id_for_section(cable_runs, entry.key);
    body.endpoint_a_pole_type_id = entry.endpoint_a_pole_type_id;
    body.endpoint_b_pole_type_id = entry.endpoint_b_pole_type_id;
    body.endpoint_a_band_id = entry.endpoint_a_band_id;
    body.endpoint_b_band_id = entry.endpoint_b_band_id;
    body.boundary_a = start;
    body.boundary_b = end;
    body.tangent_a = start_outward_tangent;
    body.tangent_b = end_outward_tangent;
    body.sag_method = VisualCurveSagMethod::kParabolic;
    body.sag_m = source_curve.sag_amplitude_m;
    copy_span_appearance(state, entry.key.logical_span_id, &body);
    body.samples = std::move(body_samples);
    body.bounds = curve_part_bounds(body.samples);
    if (!body.samples.empty() && finite_point(body.samples.front()) && finite_point(body.samples.back())) {
      if (const SpanRuntimeState* runtime =
              state.view().find_span_runtime_state(entry.key.logical_span_id);
          runtime != nullptr) {
        body.source_version = body_source_version(runtime->data_version, start_u, end_u);
      }
      out.parts.push_back(std::move(body));
    }
  }

  SpanVisualAssemblyEndpointMap assembly_endpoints{};
  assembly_endpoints.reserve(placed.entries.size());
  for (const SpanLayoutEntry& entry : placed.entries) {
    assembly_endpoints.emplace(
        entry.span_id,
        SpanVisualAssemblyEndpoints{entry.start.endpoint_world, entry.end.endpoint_world});
  }
  apply_span_visual_assemblies(state, assembly_endpoints, &out);

  for (const curve_patch_spec& spec : patch_specs) {
    curve_boundary a_boundary{};
    curve_boundary b_boundary{};
    if (!boundary_for(boundaries, boundary_index, spec.a.section_key, spec.a.is_start, &a_boundary) ||
        !boundary_for(boundaries, boundary_index, spec.b.section_key, spec.b.is_start, &b_boundary)) {
      fail("backbone internal: patch boundary is missing");
      return result;
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
    const CableRunId a_run_id = run_id_for_section(cable_runs, spec.a.section_key);
    const CableRunId b_run_id = run_id_for_section(cable_runs, spec.b.section_key);
    patch.cable_run_id = a_run_id == b_run_id ? a_run_id : 0;
    if (patch.cable_run_id == 0) {
      out.diagnostics.push_back({spec.key.node_id, spec.a.section_key.logical_span_id,
                                 spec.key.bundle_template_id, spec.key.lane_index,
                                 "node patch cable run mismatch"});
    }
    copy_span_appearance(state, spec.a.section_key.logical_span_id, &patch);
    patch.source_version = std::max(span_source_version(state, spec.a.section_key.logical_span_id),
                                    span_source_version(state, spec.b.section_key.logical_span_id));
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
    const Vec3d start_direction = ScaleVec(endpoint.away_from_node, -1.0);
    const Vec3d end_direction = peer->away_from_node;
    jumper_part.tangent_a = start_direction;
    jumper_part.tangent_b = ScaleVec(end_direction, -1.0);
    jumper_part.section_count = 1;
    copy_span_appearance(state, endpoint.section_key.logical_span_id, &jumper_part);
    jumper_part.source_version = std::max(span_source_version(state, endpoint.section_key.logical_span_id),
                                          span_source_version(state, peer->section_key.logical_span_id));
    append_patch_section(jumper_part.boundary_a, start_direction, jumper_part.boundary_b,
                         end_direction, true, &jumper_part.bezier_control_points,
                         &jumper_part.samples);
    jumper_part.bounds = curve_part_bounds(jumper_part.samples);
    if (jumper_part.samples.size() >= 2 && finite_point(jumper_part.samples.front()) &&
        finite_point(jumper_part.samples.back())) {
      out.parts.push_back(std::move(jumper_part));
      emitted_jumper_ports.push_back(endpoint.port_id);
      emitted_jumper_ports.push_back(peer->port_id);
    }
  }
  if (!scoped_spans.empty()) {
    result.value = merge_scoped_visual_curve_parts(state, std::move(out), scoped_spans);
    result.ok = true;
    return result;
  }
  sort_visual_parts(&out);
  result.value = std::move(out);
  result.ok = true;
  return result;
}

} // namespace wire::core::generation::backbone
