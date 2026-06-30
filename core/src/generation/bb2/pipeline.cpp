#include "pipeline.hpp"

#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "out.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace wire::core::generation::bb2 {
namespace {

constexpr double kRowSeparationM = 0.35;
constexpr double kLowerOffsetM = 0.6;

void add(ChangeSet& dst, const ChangeSet& src) {
  auto append = [](std::vector<ObjectId>& a, const std::vector<ObjectId>& b) {
    for (ObjectId id : b) {
      if (std::find(a.begin(), a.end(), id) == a.end()) {
        a.push_back(id);
      }
    }
  };
  append(dst.created_ids, src.created_ids);
  append(dst.updated_ids, src.updated_ids);
  append(dst.deleted_ids, src.deleted_ids);
  append(dst.dirty_span_ids, src.dirty_span_ids);
}

Vec3d norm(Vec3d v) {
  v.z = 0.0;
  return Normalize(&v) ? v : Vec3d{1.0, 0.0, 0.0};
}

bool norm_strict(Vec3d* v) {
  if (v == nullptr) {
    return false;
  }
  v->z = 0.0;
  return Normalize(v);
}

Vec3d side(Vec3d v) {
  v = norm(v);
  return Vec3d{-v.y, v.x, 0.0};
}

ObjectId saved_node_id_for(const CoreState& state, ObjectId node_or_pole_id) {
  if (node_or_pole_id == kInvalidObjectId) {
    return kInvalidObjectId;
  }
  if (const SavedBackboneNode* saved = state.view().backbone_node(node_or_pole_id); saved != nullptr) {
    return saved->node_id;
  }
  if (const SavedBackboneNode* saved = state.view().backbone_node_for_pole(node_or_pole_id); saved != nullptr) {
    return saved->node_id;
  }
  return kInvalidObjectId;
}

const SavedBackboneNode* saved_source_node_for(const CoreState& state, const SupportNode& node) {
  if (!node.has_source_edge) {
    return nullptr;
  }
  const ObjectId source_a = saved_node_id_for(state, node.source_edge_node_a_id);
  const ObjectId source_b = saved_node_id_for(state, node.source_edge_node_b_id);
  if (source_a == kInvalidObjectId || source_b == kInvalidObjectId || source_a == source_b) {
    return nullptr;
  }
  const ObjectId lo = std::min(source_a, source_b);
  const ObjectId hi = std::max(source_a, source_b);
  for (const SavedBackboneNode& saved : state.view().backbone().nodes) {
    if (saved.pole_id != kInvalidObjectId || saved.support_kind != node.support_kind || !saved.has_source_edge) {
      continue;
    }
    if (std::min(saved.source_edge_node_a, saved.source_edge_node_b) != lo ||
        std::max(saved.source_edge_node_a, saved.source_edge_node_b) != hi) {
      continue;
    }
    if (std::abs(saved.source_edge_t - node.source_edge_t) <= 1e-9) {
      return &saved;
    }
  }
  return nullptr;
}

PortKind port_kind(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kCommunication:
  case ConnectionCategory::kOptical:
    return PortKind::kCommunication;
  case ConnectionCategory::kHighVoltage:
  case ConnectionCategory::kLowVoltage:
  case ConnectionCategory::kDrop:
  default:
    return PortKind::kPower;
  }
}

PortLayer port_layer(SpanLayer layer) {
  switch (layer) {
  case SpanLayer::kHighVoltage:
    return PortLayer::kHighVoltage;
  case SpanLayer::kLowVoltage:
    return PortLayer::kLowVoltage;
  case SpanLayer::kCommunication:
    return PortLayer::kCommunication;
  case SpanLayer::kOptical:
    return PortLayer::kOptical;
  case SpanLayer::kDrop:
    return PortLayer::kDrop;
  case SpanLayer::kUnknown:
  default:
    return PortLayer::kUnknown;
  }
}

SpanKind span_kind(ConnectionCategory category) {
  return (category == ConnectionCategory::kDrop) ? SpanKind::kService : SpanKind::kDistribution;
}

int rank(SpanLayer layer) {
  switch (layer) {
  case SpanLayer::kHighVoltage:
    return 2;
  case SpanLayer::kDrop:
    return 0;
  case SpanLayer::kLowVoltage:
  case SpanLayer::kCommunication:
  case SpanLayer::kOptical:
  case SpanLayer::kUnknown:
  default:
    return 1;
  }
}

struct spec_view {
  const BackboneBundleSpec* spec = nullptr;
  const BundleTemplate* tmpl = nullptr;
  int count = 0;
  SpanLayer layer = SpanLayer::kUnknown;
};

struct port_scope {
  BundleKind bundle = BundleKind::kLowVoltage;
  PortKind kind = PortKind::kGeneric;
  PortLayer layer = PortLayer::kUnknown;
};

EditResult<spec_view> view_for(const CoreState& state, const BackboneBundleSpec& spec) {
  EditResult<spec_view> out{};
  const auto tmpl_it = state.view().bundle_templates().find(spec.bundle_template_id);
  if (tmpl_it == state.view().bundle_templates().end()) {
    out.error = "bb2 unsupported: bundle template not found";
    return out;
  }
  const BundleTemplate& tmpl = tmpl_it->second;
  const int count = (tmpl.count_rule == BundleCountRuleKind::kFixed)
                        ? tmpl.fixed_count
                        : ((spec.count > 0) ? spec.count : tmpl.default_count);
  if (count <= 0) {
    out.error = "bb2 unsupported: bundle count resolved to zero";
    return out;
  }
  if (tmpl.count_rule == BundleCountRuleKind::kFixed && spec.count > 0 && spec.count != tmpl.fixed_count) {
    out.error = "bb2 unsupported: fixed bundle count override";
    return out;
  }
  if (tmpl.count_rule == BundleCountRuleKind::kRange && (count < tmpl.min_count || count > tmpl.max_count)) {
    out.error = "bb2 unsupported: bundle count is out of range";
    return out;
  }
  const SpanLayer layer = (spec.layer == SpanLayer::kUnknown) ? tmpl.default_layer : spec.layer;
  if (layer == SpanLayer::kUnknown) {
    out.error = "bb2 unsupported: bundle layer is unknown";
    return out;
  }
  out.value = spec_view{&spec, &tmpl, count, layer};
  out.ok = true;
  return out;
}

EditResult<PoleTypeId> pole_type_for(const CoreState& state, const BackboneSpec& spec) {
  EditResult<PoleTypeId> out{};
  if (state.view().pole_types().find(spec.pole_type_id) != state.view().pole_types().end()) {
    out.value = spec.pole_type_id;
    out.ok = true;
    return out;
  }
  if (spec.pole_type_id != kInvalidPoleTypeId) {
    out.error = "bb2 unsupported: pole type not found";
    return out;
  }
  PoleTypeId resolved = kInvalidPoleTypeId;
  for (const BackboneBundleSpec& bundle : spec.bundles) {
    const auto tmpl_it = state.view().bundle_templates().find(bundle.bundle_template_id);
    if (tmpl_it == state.view().bundle_templates().end()) {
      out.error = "bb2 unsupported: bundle template not found";
      return out;
    }
    const PoleTypeId candidate = tmpl_it->second.related_pole_type_id;
    if (candidate == kInvalidPoleTypeId || state.view().pole_types().find(candidate) == state.view().pole_types().end()) {
      out.error = "bb2 unsupported: pole type missing";
      return out;
    }
    if (resolved == kInvalidPoleTypeId) {
      resolved = candidate;
      continue;
    }
    if (resolved != candidate) {
      out.error = "bb2 unsupported: pole type is ambiguous";
      return out;
    }
  }
  if (resolved == kInvalidPoleTypeId) {
    out.error = "bb2 unsupported: pole type not found";
    return out;
  }
  out.value = resolved;
  out.ok = true;
  return out;
}

EditResult<PortPlacementBand> band_for(const CoreState& state, ObjectId pole_id, const spec_view& view) {
  EditResult<PortPlacementBand> out{};
  const Pole* pole = state.view().poles().find(pole_id);
  if (pole == nullptr || pole->pole_type_id == kInvalidPoleTypeId) {
    out.error = "bb2 unsupported: pole type missing";
    return out;
  }
  const auto type_it = state.view().pole_types().find(pole->pole_type_id);
  if (type_it == state.view().pole_types().end()) {
    out.error = "bb2 unsupported: pole type missing";
    return out;
  }
  const PoleTypeDefinition& type = type_it->second;
  const int target_rank = rank(view.layer);
  const PortPlacementBand* best = nullptr;
  for (const PortPlacementBand& band : type.port_bands) {
    if (!band.enabled || band.category != view.tmpl->category || band.layer != target_rank) {
      continue;
    }
    if (best == nullptr || band.priority > best->priority ||
        (band.priority == best->priority && band.band_id < best->band_id)) {
      best = &band;
    }
  }
  if (best == nullptr) {
    out.error = "bb2 unsupported: port band missing";
    return out;
  }
  out.value = *best;
  out.ok = true;
  return out;
}

ObjectId saved_edge_for(const CoreState& state, const graph& made, const link& edge) {
  if (edge.a >= made.nodes.size() || edge.b >= made.nodes.size()) {
    return kInvalidObjectId;
  }
  const ObjectId a = made.nodes[edge.a].saved;
  const ObjectId b = made.nodes[edge.b].saved;
  if (a == kInvalidObjectId || b == kInvalidObjectId || a == b) {
    return kInvalidObjectId;
  }
  const BackboneEdgeKey key{std::min(a, b), std::max(a, b)};
  const auto it = state.view().backbone_index().edge_by_nodes.find(key);
  return it == state.view().backbone_index().edge_by_nodes.end() ? kInvalidObjectId : it->second;
}

ObjectId resolve_existing_bundle(const CoreState& state, const graph& made, const BackboneBundleSpec& spec) {
  ObjectId resolved = kInvalidObjectId;
  bool saw_existing_edge = false;
  for (const link& edge : made.links) {
    if (!edge.is_new) {
      continue;
    }
    const ObjectId edge_id = saved_edge_for(state, made, edge);
    if (edge_id == kInvalidObjectId) {
      return kInvalidObjectId;
    }
    saw_existing_edge = true;
    ObjectId edge_bundle_match = kInvalidObjectId;
    const auto bundles_it = state.view().backbone_index().edge_bundles.find(edge_id);
    if (bundles_it == state.view().backbone_index().edge_bundles.end()) {
      return kInvalidObjectId;
    }
    for (ObjectId edge_bundle_id : bundles_it->second) {
      const SavedBackboneEdgeBundle* edge_bundle = state.view().backbone_edge_bundle(edge_bundle_id);
      if (edge_bundle == nullptr) {
        continue;
      }
      const Bundle* bundle = state.view().bundles().find(edge_bundle->bundle_id);
      if (bundle == nullptr || bundle->bundle_template_id != spec.bundle_template_id) {
        continue;
      }
      if (edge_bundle_match != kInvalidObjectId && edge_bundle_match != bundle->id) {
        return kInvalidObjectId;
      }
      edge_bundle_match = bundle->id;
    }
    if (edge_bundle_match == kInvalidObjectId) {
      return kInvalidObjectId;
    }
    if (resolved != kInvalidObjectId && resolved != edge_bundle_match) {
      return kInvalidObjectId;
    }
    resolved = edge_bundle_match;
  }
  return saw_existing_edge ? resolved : kInvalidObjectId;
}

SavedBackboneRowKey key_for(const pairs& ps, const trow& row, const std::vector<ObjectId>& node_id_by_local,
                            const std::vector<SavedBackboneEdgeRef>& edge_by_link) {
  SavedBackboneRowKey key{};
  if (row.node >= node_id_by_local.size() || row.source.id == bad) {
    return key;
  }
  key.node_id = node_id_by_local[row.node];
  key.source_is_open = row.source.is_open;
  auto edge_id = [&](std::size_t link_id) -> ObjectId {
    return link_id < edge_by_link.size() ? edge_by_link[link_id].edge_id : kInvalidObjectId;
  };
  if (row.source.is_open) {
    if (row.source.id >= ps.opens.size()) {
      key.node_id = kInvalidObjectId;
      return key;
    }
    key.source_edge_a = edge_id(ps.opens[row.source.id].link);
    return key;
  }
  if (row.source.id >= ps.joins.size()) {
    key.node_id = kInvalidObjectId;
    return key;
  }
  const ObjectId a = edge_id(ps.joins[row.source.id].left);
  const ObjectId b = edge_id(ps.joins[row.source.id].right);
  key.source_edge_a = std::min(a, b);
  key.source_edge_b = std::max(a, b);
  return key;
}

SavedBackboneEdgeRef ref_for_existing_edge(const CoreState& state, const graph& made, const link& edge) {
  SavedBackboneEdgeRef out{};
  (void)made;
  if (edge.saved == kInvalidObjectId) {
    return out;
  }
  const SavedBackboneEdge* saved = state.view().backbone_edge(edge.saved);
  if (saved == nullptr) {
    return out;
  }
  out.edge_id = saved->edge_id;
  out.node_a = saved->node_a;
  out.node_b = saved->node_b;
  return out;
}

bool same_scope(const SavedBackbonePortBinding& binding, port_scope scope) {
  return binding.bundle_template_id == scope.bundle && binding.port_kind == scope.kind &&
         binding.port_layer == scope.layer;
}

bool makes_pole(const node& item) {
  return item.support == SupportKind::kPole && item.is_new;
}

bool needs_pole_type(const graph& made) {
  return std::any_of(made.nodes.begin(), made.nodes.end(), makes_pole);
}

bool has_band(const PoleTypeDefinition& pole_type, const spec_view& spec) {
  const int target_rank = rank(spec.layer);
  return std::any_of(pole_type.port_bands.begin(), pole_type.port_bands.end(),
                     [&](const PortPlacementBand& band) {
                       return band.enabled && band.category == spec.tmpl->category && band.layer == target_rank;
                     });
}

EditResult<bool> check_port_bands(const CoreState& state, const graph& made, const BackboneSpec& spec,
                                  const std::vector<std::size_t>& active_bundle_indices) {
  for (const node& item : made.nodes) {
    if (!item.on_route) {
      continue;
    }
    if (item.support == SupportKind::kMidair || item.support == SupportKind::kExternal ||
        item.support == SupportKind::kGround) {
      continue;
    }
    PoleTypeId pole_type_id = kInvalidPoleTypeId;
    if (item.is_new) {
      EditResult<PoleTypeId> resolved_type = pole_type_for(state, spec);
      if (!resolved_type.ok) {
        EditResult<bool> failed{};
        failed.error = resolved_type.error;
        return failed;
      }
      pole_type_id = resolved_type.value;
    } else {
      const ObjectId pole_id = item.pole;
      if (pole_id == kInvalidObjectId) {
        EditResult<bool> failed{};
        failed.error = "bb2 unsupported: existing pole is missing";
        return failed;
      }
      const Pole* pole = state.view().poles().find(pole_id);
      if (pole == nullptr) {
        EditResult<bool> failed{};
        failed.error = "bb2 unsupported: existing pole is missing";
        return failed;
      }
      pole_type_id = pole->pole_type_id;
    }
    const auto type_it = state.view().pole_types().find(pole_type_id);
    if (type_it == state.view().pole_types().end()) {
      EditResult<bool> failed{};
      failed.error = "bb2 unsupported: pole type missing";
      return failed;
    }
    for (const std::size_t bundle_index : active_bundle_indices) {
      if (bundle_index >= spec.bundles.size()) {
        EditResult<bool> failed{};
        failed.error = "bb2 unsupported: active bundle index is invalid";
        return failed;
      }
      const BackboneBundleSpec& bundle = spec.bundles[bundle_index];
      EditResult<spec_view> checked = view_for(state, bundle);
      if (!checked.ok) {
        EditResult<bool> failed{};
        failed.error = checked.error;
        return failed;
      }
      if (!has_band(type_it->second, checked.value)) {
        EditResult<bool> failed{};
        failed.error = "bb2 unsupported: port band missing";
        return failed;
      }
    }
  }
  EditResult<bool> out{};
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<ObjectId> resolve_port_binding(const CoreState& state, ObjectId pole_id, const SavedBackboneRowKey& row_key,
                                          std::size_t lane_index, port_scope scope) {
  EditResult<ObjectId> out{};
  out.value = kInvalidObjectId;
  out.ok = true;
  if (pole_id == kInvalidObjectId || row_key.node_id == kInvalidObjectId ||
      row_key.source_edge_a == kInvalidObjectId) {
    return out;
  }
  ObjectId found = kInvalidObjectId;
  for (const SavedBackbonePortBinding* binding : state.view().backbone_port_bindings_for_row(row_key, lane_index)) {
    if (binding == nullptr) {
      continue;
    }
    if (!same_scope(*binding, scope)) {
      continue;
    }
    const Port* port = state.view().ports().find(binding->port_id);
    if (port == nullptr || port->owner_pole_id != pole_id || port->kind != scope.kind || port->layer != scope.layer) {
      continue;
    }
    if (found != kInvalidObjectId && found != port->id) {
      out.error = "bb2 unsupported: ambiguous backbone port binding";
      out.ok = false;
      return out;
    }
    found = port->id;
  }
  out.value = found;
  return out;
}

std::vector<Vec3d> row_shifts(const pairs& ps) {
  std::vector<Vec3d> shifts(ps.rows.size(), Vec3d{});
  std::unordered_map<std::size_t, std::vector<std::size_t>> rows_by_node{};
  for (const row& r : ps.rows) {
    rows_by_node[r.node].push_back(r.id);
  }
  for (auto& item : rows_by_node) {
    std::vector<std::size_t>& rows = item.second;
    std::sort(rows.begin(), rows.end());
    const double center = (static_cast<double>(rows.size()) - 1.0) * 0.5;
    for (std::size_t order = 0; order < rows.size(); ++order) {
      const std::size_t row_id = rows[order];
      if (row_id >= ps.rows.size()) {
        continue;
      }
      const Vec3d sep = side(ps.rows[row_id].axis);
      const double amount = (static_cast<double>(order) - center) * kRowSeparationM;
      shifts[row_id] = Vec3d{sep.x * amount, sep.y * amount, 0.0};
    }
  }
  return shifts;
}

double yaw(Vec3d forward) {
  forward = norm(forward);
  return std::atan2(forward.y, forward.x) * (180.0 / 3.14159265358979323846);
}

double len(Vec3d v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

double dist2_to_segment(const Vec3d& p, const Vec3d& a, const Vec3d& b) {
  const Vec3d ab = b - a;
  const double len2 = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
  if (len2 <= 1e-12) {
    const Vec3d d = p - a;
    return d.x * d.x + d.y * d.y + d.z * d.z;
  }
  const double t = std::clamp(((p.x - a.x) * ab.x + (p.y - a.y) * ab.y + (p.z - a.z) * ab.z) / len2, 0.0, 1.0);
  const Vec3d closest = {a.x + ab.x * t, a.y + ab.y * t, a.z + ab.z * t};
  const Vec3d d = p - closest;
  return d.x * d.x + d.y * d.y + d.z * d.z;
}

bool route_clear_of_avoid_points(const graph& made, const std::vector<Vec3d>& points, double radius) {
  if (points.empty() || radius <= 0.0) {
    return true;
  }
  const double radius2 = radius * radius;
  const auto same_point = [](const Vec3d& lhs, const Vec3d& rhs) {
    const Vec3d d = lhs - rhs;
    return d.x * d.x + d.y * d.y + d.z * d.z <= 1e-18;
  };
  for (const Vec3d& point : points) {
    if (!made.nodes.empty() && (same_point(point, made.nodes.front().pos) || same_point(point, made.nodes.back().pos))) {
      continue;
    }
    const auto node_it = std::find_if(made.nodes.begin(), made.nodes.end(), [&](const node& n) {
      return (!n.is_new || n.explicit_support) && same_point(point, n.pos);
    });
    if (node_it != made.nodes.end()) {
      continue;
    }
    for (const link& edge : made.links) {
      if (!edge.is_new || edge.a >= made.nodes.size() || edge.b >= made.nodes.size()) {
        continue;
      }
      if (dist2_to_segment(point, made.nodes[edge.a].pos, made.nodes[edge.b].pos) <= radius2) {
        return false;
      }
    }
  }
  return true;
}

struct segment_insert {
  double t = 0.0;
  Vec3d p{};
  SupportKind support = SupportKind::kPole;
  bool detour = false;
};

bool ownerless_support(SupportKind kind) {
  return kind == SupportKind::kMidair || kind == SupportKind::kExternal || kind == SupportKind::kGround;
}

EditResult<std::size_t> avoid_detours_for_segment(const Vec3d& a, const Vec3d& b, const std::vector<Vec3d>& points,
                                                  double radius, SupportKind inserted_support,
                                                  std::vector<segment_insert>* inserts) {
  EditResult<std::size_t> out{};
  out.value = 0;
  if (points.empty() || radius <= 0.0) {
    out.ok = true;
    return out;
  }
  Vec3d ab = b - a;
  const double len2 = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
  if (len2 <= 1e-12) {
    out.error = "bb2 unsupported: avoid routing source segment is zero length";
    return out;
  }
  Vec3d dir = ab;
  if (!norm_strict(&dir)) {
    out.error = "bb2 unsupported: avoid routing source segment is zero length";
    return out;
  }
  const Vec3d detour_axis = side(dir);
  constexpr double kAvoidClearanceM = 0.5;
  for (const Vec3d& p : points) {
    const double t = ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y + (p.z - a.z) * ab.z) / len2;
    if (t <= 1e-6 || t >= 1.0 - 1e-6 || dist2_to_segment(p, a, b) > radius * radius) {
      continue;
    }
    const Vec3d closest = {a.x + ab.x * t, a.y + ab.y * t, a.z + ab.z * t};
    const double len_m = std::sqrt(len2);
    const double before_m = t * len_m;
    const double after_m = (1.0 - t) * len_m;
    if (radius >= before_m || radius >= after_m) {
      out.error = "bb2 unsupported: avoid radius reaches a fixed route endpoint";
      return out;
    }
    double clearance = radius + kAvoidClearanceM;
    clearance = std::min(clearance, std::min(before_m, after_m) - 1e-6);
    auto required_detour = [&](double along_m) {
      const double denom2 = along_m * along_m - clearance * clearance;
      return denom2 > 1e-12 ? (clearance * along_m) / std::sqrt(denom2) : along_m + radius + kAvoidClearanceM;
    };
    const double detour = std::max(required_detour(before_m), required_detour(after_m));
    if (inserts != nullptr) {
      inserts->push_back(
          {t, closest + Vec3d{detour_axis.x * detour, detour_axis.y * detour, 0.0}, inserted_support, true});
    }
    ++out.value;
  }
  out.ok = true;
  return out;
}

bool detour_internal_vertex(Vec3d prev, Vec3d current, Vec3d next, const std::vector<Vec3d>& points, double radius,
                            Vec3d* out) {
  if (points.empty() || radius <= 0.0) {
    return false;
  }
  const double radius2 = radius * radius;
  bool needs_detour = false;
  for (const Vec3d& point : points) {
    const Vec3d d = point - current;
    if (d.x * d.x + d.y * d.y + d.z * d.z <= radius2) {
      needs_detour = true;
      break;
    }
  }
  if (!needs_detour) {
    return false;
  }
  Vec3d chord = next - prev;
  if (!norm_strict(&chord)) {
    chord = current - prev;
    if (!norm_strict(&chord)) {
      chord = next - current;
      if (!norm_strict(&chord)) {
        return false;
      }
    }
  }
  constexpr double kAvoidClearanceM = 0.5;
  const Vec3d axis = side(chord);
  *out = current + Vec3d{axis.x * (radius + kAvoidClearanceM), axis.y * (radius + kAvoidClearanceM), 0.0};
  return true;
}

EditResult<bool> unsupported(std::string reason) {
  EditResult<bool> out{};
  out.error = "bb2 unsupported: " + std::move(reason);
  return out;
}

EditResult<pairs> unsupported_pairs(std::string reason) {
  EditResult<pairs> out{};
  out.error = "bb2 unsupported: " + std::move(reason);
  return out;
}

bool has_current_route_pair_at(const graph& made, std::size_t node_id) {
  const link* incoming = nullptr;
  const link* outgoing = nullptr;
  for (const link& edge : made.links) {
    if (!edge.is_new) {
      continue;
    }
    if (edge.b == node_id) {
      if (incoming != nullptr) {
        return false;
      }
      incoming = &edge;
    }
    if (edge.a == node_id) {
      if (outgoing != nullptr) {
        return false;
      }
      outgoing = &edge;
    }
  }
  return incoming != nullptr && outgoing != nullptr && incoming->route == outgoing->route &&
         incoming->order + 1 == outgoing->order;
}

std::size_t current_route_incident_count_at(const graph& made, std::size_t node_id) {
  std::size_t count = 0;
  for (const link& edge : made.links) {
    if (!edge.is_new) {
      continue;
    }
    if (edge.a == node_id || edge.b == node_id) {
      ++count;
    }
  }
  return count;
}

std::size_t context_incident_count_at(const graph& made, std::size_t node_id) {
  std::size_t count = 0;
  for (const link& edge : made.links) {
    if (edge.is_new) {
      continue;
    }
    if (edge.a == node_id || edge.b == node_id) {
      ++count;
    }
  }
  return count;
}

std::size_t add_open(pairs* out, std::size_t node_id, std::size_t link_id, const Vec3d& axis) {
  const std::size_t open_id = out->opens.size();
  out->opens.push_back(open{open_id, node_id, link_id, axis});
  const std::size_t row_id = out->rows.size();
  out->rows.push_back(row{row_id, node_id, src{true, open_id}, axis});
  return row_id;
}

std::size_t add_pair(pairs* out, std::size_t node_id, std::size_t left, std::size_t right, const Vec3d& axis) {
  const std::size_t pair_id = out->joins.size();
  out->joins.push_back(pair{pair_id, node_id, left, right, axis});
  const std::size_t row_id = out->rows.size();
  out->rows.push_back(row{row_id, node_id, src{false, pair_id}, axis});
  return row_id;
}

} // namespace

std::size_t pipeline::local(std::size_t input_point) const {
  return input_point < local_by_input_.size() ? local_by_input_[input_point] : bad;
}

EditResult<bool> pipeline::prepare() {
  EditResult<bool> out{};
  g_ = {};
  active_bundle_indices_.clear();
  local_by_input_.clear();
  const std::vector<Vec3d>& input = spec_.path.polyline;
  std::vector<Vec3d> guide = input;
  std::unordered_map<std::size_t, const BackboneInputSpec::NodeSpec*> spec_by_point{};
  spec_by_point.reserve(spec_.path.node_specs.size());
  for (const BackboneInputSpec::NodeSpec& spec : spec_.path.node_specs) {
    if (spec.point_index >= input.size()) {
      out.error = "bb2 unsupported: node spec point index is out of range";
      return out;
    }
    if (spec.support_kind != SupportKind::kPole && spec.support_kind != SupportKind::kMidair &&
        spec.support_kind != SupportKind::kExternal && spec.support_kind != SupportKind::kGround) {
      out.error = "bb2 unsupported: node specs require pole, midair, external, or ground support";
      return out;
    }
    if (spec.has_tangent_hint) {
      Vec3d tangent = spec.tangent_hint;
      if (!norm_strict(&tangent)) {
        out.error = "bb2 unsupported: node spec tangent hint is zero";
        return out;
      }
    }
    if (!spec_by_point.emplace(spec.point_index, &spec).second) {
      out.error = "bb2 unsupported: duplicate node spec";
      return out;
    }
  }
  if (spec_.direction_mode == PathDirectionMode::kReverse) {
    std::reverse(guide.begin(), guide.end());
    std::unordered_map<std::size_t, const BackboneInputSpec::NodeSpec*> reversed{};
    reversed.reserve(spec_by_point.size());
    for (const auto& item : spec_by_point) {
      reversed.emplace(guide.size() - 1 - item.first, item.second);
    }
    spec_by_point.swap(reversed);
  }
  std::vector<Vec3d> pts{};
  std::vector<std::size_t> guide_by_local{};
  std::vector<SupportKind> support_by_local{};
  pts.reserve(guide.size());
  guide_by_local.reserve(guide.size());
  support_by_local.reserve(guide.size());
  auto push_point = [&](const Vec3d& p, std::size_t guide_index, SupportKind support = SupportKind::kPole) {
    pts.push_back(p);
    guide_by_local.push_back(guide_index);
    support_by_local.push_back(support);
  };
  auto inserted_support_for_segment = [&](std::size_t a, std::size_t b) {
    const auto spec_a = spec_by_point.find(a);
    const auto spec_b = spec_by_point.find(b);
    if (spec_a == spec_by_point.end() || spec_b == spec_by_point.end()) {
      return SupportKind::kPole;
    }
    const SupportKind support = spec_a->second->support_kind;
    return support == spec_b->second->support_kind && ownerless_support(support) ? support : SupportKind::kPole;
  };
  if (!guide.empty()) {
    push_point(guide.front(), 0);
    for (std::size_t i = 0; i + 1 < guide.size(); ++i) {
      const Vec3d a = guide[i];
      Vec3d b = guide[i + 1];
      if (i + 2 < guide.size() && spec_by_point.find(i + 1) == spec_by_point.end()) {
        Vec3d detoured{};
        if (detour_internal_vertex(guide[i], guide[i + 1], guide[i + 2], spec_.constraints.avoid_points,
                                   spec_.constraints.avoid_radius_m, &detoured)) {
          b = detoured;
          guide[i + 1] = detoured;
        }
      }
      const SupportKind inserted_support = inserted_support_for_segment(i, i + 1);
      const Vec3d seg = b - a;
      constexpr double kIntervalEps = 1e-9;
      const double len = std::sqrt(seg.x * seg.x + seg.y * seg.y + seg.z * seg.z);
      std::vector<segment_insert> inserts{};
      if (spec_.interval_m > 0.0 && len > kIntervalEps) {
        for (double dist = spec_.interval_m; dist < len - kIntervalEps; dist += spec_.interval_m) {
          const double t = std::clamp(dist / len, 0.0, 1.0);
          inserts.push_back({t, {a.x + seg.x * t, a.y + seg.y * t, a.z + seg.z * t}, inserted_support, false});
        }
      }
      EditResult<std::size_t> avoid =
          avoid_detours_for_segment(a, b, spec_.constraints.avoid_points, spec_.constraints.avoid_radius_m,
                                    inserted_support, &inserts);
      if (!avoid.ok) {
        out.error = avoid.error;
        return out;
      }
      std::sort(inserts.begin(), inserts.end(), [](const segment_insert& lhs, const segment_insert& rhs) {
        if (std::abs(lhs.t - rhs.t) > 1e-9) {
          return lhs.t < rhs.t;
        }
        const auto len2 = [](const Vec3d& p) { return p.x * p.x + p.y * p.y + p.z * p.z; };
        return len2(lhs.p) < len2(rhs.p);
      });
      std::vector<segment_insert> unique_inserts{};
      unique_inserts.reserve(inserts.size());
      const auto same_point = [](const Vec3d& lhs, const Vec3d& rhs) {
        const Vec3d d = lhs - rhs;
        return d.x * d.x + d.y * d.y + d.z * d.z <= 1e-18;
      };
      for (const segment_insert& item : inserts) {
        if (!unique_inserts.empty() && std::abs(unique_inserts.back().t - item.t) <= 1e-9 &&
            unique_inserts.back().support == item.support) {
          if (item.detour && !unique_inserts.back().detour) {
            unique_inserts.back() = item;
          }
          continue;
        }
        const bool duplicate = !unique_inserts.empty() && unique_inserts.back().support == item.support &&
                               same_point(unique_inserts.back().p, item.p);
        if (duplicate) {
          continue;
        }
        unique_inserts.push_back(item);
      }
      for (const segment_insert& item : unique_inserts) {
        push_point(item.p, bad, item.support);
      }
      push_point(b, i + 1);
    }
  }
  if (pts.size() > 2 && !spec_.pole_placement.pin_vertices) {
    constexpr double kAutoCollapseDistanceM = 1.5;
    constexpr double kAutoCollapseDistanceSq = kAutoCollapseDistanceM * kAutoCollapseDistanceM;
    auto is_auto_generic = [&](std::size_t index) {
      if (index >= guide_by_local.size() || index >= support_by_local.size()) {
        return false;
      }
      const std::size_t guide_index = guide_by_local[index];
      return guide_index != bad && support_by_local[index] == SupportKind::kPole &&
             spec_by_point.find(guide_index) == spec_by_point.end();
    };
    auto is_endpoint = [&](std::size_t index) {
      const std::size_t guide_index = guide_by_local[index];
      return guide_index == 0 || guide_index + 1 == guide.size();
    };
    std::vector<bool> drop(pts.size(), false);
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
      if (drop[i] || !is_auto_generic(i) || !is_auto_generic(i + 1)) {
        continue;
      }
      const Vec3d d = pts[i + 1] - pts[i];
      const double dist_sq = d.x * d.x + d.y * d.y + d.z * d.z;
      if (dist_sq <= 1e-18 || dist_sq >= kAutoCollapseDistanceSq) {
        continue;
      }
      const bool a_endpoint = is_endpoint(i);
      const bool b_endpoint = is_endpoint(i + 1);
      if (a_endpoint && b_endpoint) {
        continue;
      }
      drop[a_endpoint ? i + 1 : i] = true;
    }
    if (std::any_of(drop.begin(), drop.end(), [](bool item) { return item; })) {
      std::vector<Vec3d> kept_pts{};
      std::vector<std::size_t> kept_guide{};
      std::vector<SupportKind> kept_support{};
      kept_pts.reserve(pts.size());
      kept_guide.reserve(guide_by_local.size());
      kept_support.reserve(support_by_local.size());
      for (std::size_t i = 0; i < pts.size(); ++i) {
        if (drop[i]) {
          continue;
        }
        kept_pts.push_back(pts[i]);
        kept_guide.push_back(guide_by_local[i]);
        kept_support.push_back(support_by_local[i]);
      }
      pts = std::move(kept_pts);
      guide_by_local = std::move(kept_guide);
      support_by_local = std::move(kept_support);
    }
  }
  std::vector<std::size_t> local_by_guide(guide.size(), bad);
  for (std::size_t i = 0; i < guide_by_local.size(); ++i) {
    if (guide_by_local[i] != bad && guide_by_local[i] < local_by_guide.size()) {
      local_by_guide[guide_by_local[i]] = i;
    }
  }
  local_by_input_.assign(input.size(), bad);
  for (std::size_t i = 0; i < input.size(); ++i) {
    const std::size_t guide_index = (spec_.direction_mode == PathDirectionMode::kReverse) ? input.size() - 1 - i : i;
    local_by_input_[i] = guide_index < local_by_guide.size() ? local_by_guide[guide_index] : bad;
  }
  g_.nodes.reserve(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    node n{};
    n.id = i;
    n.pos = pts[i];
    n.support = support_by_local[i];
    n.on_route = true;
    const std::size_t guide_index = guide_by_local[i];
    if (guide_index != bad) {
      const bool endpoint = guide_index == 0 || guide_index + 1 == guide.size();
      n.pinned = spec_.pole_placement.pin_vertices || (spec_.pole_placement.pin_endpoints && endpoint);
    }
    if (guide_index != bad) {
      const auto spec_it = spec_by_point.find(guide_index);
      if (spec_it == spec_by_point.end()) {
        g_.nodes.push_back(n);
        continue;
      }
      if (spec_it->second->has_tangent_hint) {
        Vec3d tangent = spec_it->second->tangent_hint;
        (void)norm_strict(&tangent);
        n.has_tangent = true;
        n.tangent = tangent;
      }
      n.explicit_support = true;
      n.support = spec_it->second->support_kind;
      if (n.support == SupportKind::kMidair || n.support == SupportKind::kExternal ||
          n.support == SupportKind::kGround) {
        if (spec_it->second->node_id == kInvalidObjectId) {
          g_.nodes.push_back(n);
          continue;
        }
        const SavedBackboneNode* saved = state_.view().backbone_node(spec_it->second->node_id);
        if (saved != nullptr && saved->pole_id == kInvalidObjectId && saved->support_kind == n.support) {
          n.saved = saved->node_id;
          n.pos = saved->position;
          n.is_new = false;
          n.has_source_edge = saved->has_source_edge;
          n.source_edge_node_a = saved->source_edge_node_a;
          n.source_edge_node_b = saved->source_edge_node_b;
          n.source_edge_t = saved->source_edge_t;
          n.bundle_modes = saved->bundle_modes;
          g_.nodes.push_back(n);
          continue;
        }
        const SupportNode* pending = state_.view().pending_support_node(spec_it->second->node_id);
        if (pending != nullptr && pending->support_kind == n.support) {
          n.pos = pending->position;
          n.bundle_modes = pending->bundle_modes;
          n.has_source_edge = pending->has_source_edge;
          n.source_edge_node_a = pending->source_edge_node_a_id;
          n.source_edge_node_b = pending->source_edge_node_b_id;
          n.source_edge_t = pending->source_edge_t;
          const SavedBackboneNode* resolved = nullptr;
          if (pending->saved_backbone_node_id != kInvalidObjectId) {
            resolved = state_.view().backbone_node(pending->saved_backbone_node_id);
          }
          if (resolved == nullptr) {
            resolved = saved_source_node_for(state_, *pending);
          }
          if (resolved != nullptr && resolved->pole_id == kInvalidObjectId && resolved->support_kind == n.support) {
            n.saved = resolved->node_id;
            n.pos = resolved->position;
            n.is_new = false;
            n.has_source_edge = resolved->has_source_edge;
            n.source_edge_node_a = resolved->source_edge_node_a;
            n.source_edge_node_b = resolved->source_edge_node_b;
            n.source_edge_t = resolved->source_edge_t;
          }
          if (pending->has_tangent_hint) {
            Vec3d tangent = pending->tangent_hint;
            if (norm_strict(&tangent)) {
              n.has_tangent = true;
              n.tangent = tangent;
            }
          }
          g_.nodes.push_back(n);
          continue;
        }
        if (saved == nullptr || saved->pole_id != kInvalidObjectId || saved->support_kind != n.support) {
          out.error = (n.support == SupportKind::kMidair) ? "bb2 unsupported: saved midair node not found"
                      : (n.support == SupportKind::kExternal)
                          ? "bb2 unsupported: saved external node not found"
                          : "bb2 unsupported: saved ground node not found";
          return out;
        }
      }
      if (spec_it->second->node_id == kInvalidObjectId) {
        g_.nodes.push_back(n);
        continue;
      }
      const SupportNode* pending_pole = state_.view().pending_support_node(spec_it->second->node_id);
      ObjectId pole_id = spec_it->second->node_id;
      if (pending_pole != nullptr && pending_pole->support_kind == SupportKind::kPole &&
          pending_pole->pole_id != kInvalidObjectId) {
        pole_id = pending_pole->pole_id;
        n.bundle_modes = pending_pole->bundle_modes;
      }
      const Pole* pole = state_.view().poles().find(pole_id);
      if (pole == nullptr) {
        out.error = "bb2 unsupported: node spec pole not found";
        return out;
      }
      n.pole = pole->id;
      n.pos = pole->world_transform.position;
      if (pending_pole != nullptr && pending_pole->saved_backbone_node_id != kInvalidObjectId) {
        n.saved = pending_pole->saved_backbone_node_id;
      } else if (const SavedBackboneNode* saved = state_.view().backbone_node_for_pole(pole->id); saved != nullptr) {
        n.saved = saved->node_id;
      } else {
        out.error = "bb2 unsupported: saved backbone graph missing for existing pole";
        return out;
      }
      n.is_new = false;
    }
    g_.nodes.push_back(n);
  }
  g_.links.reserve(pts.size() > 0 ? pts.size() - 1 : 0);
  for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
    link edge{};
    edge.id = i;
    edge.a = i;
    edge.b = i + 1;
    edge.route = 0;
    edge.order = i;
    edge.dir = g_.nodes[i + 1].pos - g_.nodes[i].pos;
    edge.is_new = true;
    g_.links.push_back(edge);
  }
  std::unordered_map<ObjectId, std::size_t> local_by_saved{};
  for (const node& n : g_.nodes) {
    if (n.saved != kInvalidObjectId) {
      local_by_saved[n.saved] = n.id;
    }
  }
  auto has_requested_saved_pair = [&](ObjectId a, ObjectId b) {
    if (a == kInvalidObjectId || b == kInvalidObjectId) {
      return false;
    }
    const ObjectId lo = std::min(a, b);
    const ObjectId hi = std::max(a, b);
    for (const link& edge : g_.links) {
      if (!edge.is_new || edge.a >= g_.nodes.size() || edge.b >= g_.nodes.size()) {
        continue;
      }
      const ObjectId ea = g_.nodes[edge.a].saved;
      const ObjectId eb = g_.nodes[edge.b].saved;
      if (ea == kInvalidObjectId || eb == kInvalidObjectId) {
        continue;
      }
      if (std::min(ea, eb) == lo && std::max(ea, eb) == hi) {
        return true;
      }
    }
    return false;
  };
  auto local_for_saved = [&](ObjectId saved_node_id) -> std::size_t {
    if (const auto it = local_by_saved.find(saved_node_id); it != local_by_saved.end()) {
      return it->second;
    }
    const SavedBackboneNode* saved = state_.view().backbone_node(saved_node_id);
    if (saved == nullptr) {
      return bad;
    }
    node n{};
    n.id = g_.nodes.size();
    n.pos = saved->position;
    n.support = saved->support_kind;
    n.pole = saved->pole_id;
    n.saved = saved->node_id;
    n.is_new = false;
    n.has_source_edge = saved->has_source_edge;
    n.source_edge_node_a = saved->source_edge_node_a;
    n.source_edge_node_b = saved->source_edge_node_b;
    n.source_edge_t = saved->source_edge_t;
    n.bundle_modes = saved->bundle_modes;
    g_.nodes.push_back(n);
    local_by_saved[n.saved] = n.id;
    return n.id;
  };
  std::unordered_set<ObjectId> context_edges{};
  const std::size_t route_node_count = pts.size();
  for (std::size_t i = 0; i < route_node_count && i < g_.nodes.size(); ++i) {
    const node& n = g_.nodes[i];
    if (n.saved == kInvalidObjectId) {
      continue;
    }
    const auto incident_it = state_.view().backbone_index().node_edges.find(n.saved);
    if (incident_it == state_.view().backbone_index().node_edges.end()) {
      continue;
    }
    for (ObjectId edge_id : incident_it->second) {
      if (context_edges.contains(edge_id)) {
        continue;
      }
      const SavedBackboneEdge* saved = state_.view().backbone_edge(edge_id);
      if (saved == nullptr) {
        continue;
      }
      if (has_requested_saved_pair(saved->node_a, saved->node_b)) {
        continue;
      }
      const std::size_t a = local_for_saved(saved->node_a);
      const std::size_t b = local_for_saved(saved->node_b);
      if (a == bad || b == bad || a == b) {
        continue;
      }
      link edge{};
      edge.id = g_.links.size();
      edge.a = a;
      edge.b = b;
      edge.route = saved->route + 1;
      edge.order = saved->order;
      edge.dir = saved->dir;
      edge.saved = saved->edge_id;
      edge.is_new = false;
      g_.links.push_back(edge);
      context_edges.insert(edge_id);
    }
  }
  const std::size_t source_context_node_count = g_.nodes.size();
  for (std::size_t node_index = 0; node_index < source_context_node_count; ++node_index) {
    const node n = g_.nodes[node_index];
    const ObjectId source_a = saved_node_id_for(state_, n.source_edge_node_a);
    const ObjectId source_b = saved_node_id_for(state_, n.source_edge_node_b);
    if (!n.has_source_edge || source_a == kInvalidObjectId || source_b == kInvalidObjectId ||
        source_a == source_b) {
      continue;
    }
    const std::size_t a = local_for_saved(source_a);
    const std::size_t b = local_for_saved(source_b);
    if (a == bad || b == bad || n.id >= g_.nodes.size() || a == n.id || b == n.id) {
      out.error = "bb2 unsupported: source edge context node is missing";
      return out;
    }
    const BackboneEdgeKey key{std::min(source_a, source_b), std::max(source_a, source_b)};
    const auto edge_it = state_.view().backbone_index().edge_by_nodes.find(key);
    if (edge_it == state_.view().backbone_index().edge_by_nodes.end()) {
      out.error = "bb2 unsupported: source edge context is missing";
      return out;
    }
    const ObjectId edge_id = edge_it->second;
    const SavedBackboneEdge* saved = state_.view().backbone_edge(edge_id);
    if (saved == nullptr) {
      out.error = "bb2 unsupported: source edge context is missing";
      return out;
    }
    const std::size_t source_route = g_.links.size() + 1;
    auto add_source_context = [&](std::size_t from, std::size_t to, std::size_t order) {
      link edge{};
      edge.id = g_.links.size();
      edge.a = from;
      edge.b = to;
      edge.route = source_route;
      edge.order = order;
      edge.dir = g_.nodes[to].pos - g_.nodes[from].pos;
      edge.saved = edge_id;
      edge.is_new = false;
      g_.links.push_back(edge);
    };
    add_source_context(a, n.id, 0);
    add_source_context(n.id, b, 1);
  }
  active_bundle_indices_.reserve(spec_.bundles.size());
  for (std::size_t bundle_index = 0; bundle_index < spec_.bundles.size(); ++bundle_index) {
    bool allowed = true;
    const BundleKind bundle_template_id = spec_.bundles[bundle_index].bundle_template_id;
    for (const node& n : g_.nodes) {
      if (!n.on_route) {
        continue;
      }
      const auto mode_it = std::find_if(n.bundle_modes.begin(), n.bundle_modes.end(),
                                        [&](const SupportNodeBundleMode& mode) {
                                          return mode.bundle_template_id == bundle_template_id;
                                        });
      if (!n.bundle_modes.empty() && mode_it == n.bundle_modes.end()) {
        allowed = false;
        break;
      }
      if (mode_it != n.bundle_modes.end() && mode_it->mode == BundleNodeMode::kNotPresent) {
        allowed = false;
        break;
      }
    }
    if (allowed) {
      active_bundle_indices_.push_back(bundle_index);
    }
  }
  ready_ = true;
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::check() const {
  if (!ready_) {
    return unsupported("pipeline is not prepared");
  }
  if (g_.nodes.size() < 2 || g_.links.empty()) {
    return unsupported("path needs at least two points");
  }
  if (spec_.bundles.empty()) {
    return unsupported("at least one bundle is required");
  }
  for (const BackboneSpec::NodeBundleModeSpec& mode : spec_.node_bundle_modes) {
    if (mode.point_index >= spec_.path.polyline.size()) {
      return unsupported("node bundle mode point index is out of range");
    }
    const auto bundle_it = std::find_if(spec_.bundles.begin(), spec_.bundles.end(), [&](const BackboneBundleSpec& bundle) {
      return bundle.bundle_template_id == mode.bundle_template_id;
    });
    if (bundle_it == spec_.bundles.end()) {
      return unsupported("node bundle mode references missing bundle");
    }
    if (mode.mode == BundleNodeMode::kPassThrough) {
      const std::size_t local = this->local(mode.point_index);
      if (local == bad || local >= g_.nodes.size()) {
        return unsupported("pass-through node mode point is missing");
      }
      if (has_current_route_pair_at(g_, local)) {
        continue;
      }
      if (g_.nodes[local].saved == kInvalidObjectId) {
        const std::size_t source_context_incidents =
            g_.nodes[local].has_source_edge ? context_incident_count_at(g_, local) : 0U;
        if (source_context_incidents + current_route_incident_count_at(g_, local) < 2) {
          return unsupported("pass-through node mode requires current route pair or saved source context");
        }
        continue;
      }
      const auto incident_it = state_.view().backbone_index().node_edges.find(g_.nodes[local].saved);
      const std::size_t saved_incidents =
          (incident_it == state_.view().backbone_index().node_edges.end()) ? 0U : incident_it->second.size();
      if (saved_incidents + current_route_incident_count_at(g_, local) < 2) {
        return unsupported("pass-through node mode requires current route pair or saved junction context");
      }
      continue;
    }
    if (mode.mode != BundleNodeMode::kNotPresent) {
      return unsupported("node bundle mode is not supported");
    }
  }
  if (!route_clear_of_avoid_points(g_, spec_.constraints.avoid_points, spec_.constraints.avoid_radius_m)) {
    return unsupported("avoid routing cannot satisfy the requested constraints");
  }
  if (needs_pole_type(g_)) {
    EditResult<PoleTypeId> resolved_type = pole_type_for(state_, spec_);
    if (!resolved_type.ok) {
      EditResult<bool> failed{};
      failed.error = resolved_type.error;
      return failed;
    }
  }
  for (const BackboneBundleSpec& spec : spec_.bundles) {
    EditResult<spec_view> checked = view_for(state_, spec);
    if (!checked.ok) {
      EditResult<bool> failed{};
      failed.error = checked.error;
      return failed;
    }
  }
  EditResult<bool> bands = check_port_bands(state_, g_, spec_, active_bundle_indices_);
  if (!bands.ok) {
    return bands;
  }
  EditResult<bool> out{};
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<pairs> pipeline::make(const graph& made) const {
  if (made.nodes.size() < 2 || made.links.empty()) {
    return unsupported_pairs("path needs at least two points");
  }

  EditResult<pairs> out{};
  out.value.links = made.links;
  std::vector<std::vector<std::size_t>> in(made.nodes.size());
  std::vector<std::vector<std::size_t>> out_links(made.nodes.size());
  for (link& edge : out.value.links) {
    if (edge.a >= made.nodes.size() || edge.b >= made.nodes.size()) {
      return unsupported_pairs("link endpoint is out of range");
    }
    if (!norm_strict(&edge.dir)) {
      return unsupported_pairs("zero length link");
    }
    out_links[edge.a].push_back(edge.id);
    in[edge.b].push_back(edge.id);
  }

  std::vector<bool> aused(out.value.links.size(), false);
  std::vector<bool> bused(out.value.links.size(), false);
  for (const node& n : made.nodes) {
    std::vector<std::size_t> incoming = in[n.id];
    std::vector<std::size_t> outgoing = out_links[n.id];
    auto link_less = [&](std::size_t lhs, std::size_t rhs) {
      const link& a = out.value.links[lhs];
      const link& b = out.value.links[rhs];
      if (a.route != b.route) {
        return a.route < b.route;
      }
      if (a.order != b.order) {
        return a.order < b.order;
      }
      return a.id < b.id;
    };
    std::sort(incoming.begin(), incoming.end(), link_less);
    std::sort(outgoing.begin(), outgoing.end(), link_less);

    for (std::size_t right : outgoing) {
      int candidates = 0;
      for (std::size_t left : incoming) {
        const link& a = out.value.links[left];
        const link& b = out.value.links[right];
        if (a.route == b.route && a.order + 1 == b.order) {
          ++candidates;
        }
      }
      if (candidates > 1) {
        return unsupported_pairs("route continuity is ambiguous");
      }
    }

    for (std::size_t left : incoming) {
      if (bused[left]) {
        continue;
      }
      std::size_t matched = bad;
      for (std::size_t right : outgoing) {
        if (aused[right]) {
          continue;
        }
        const link& a = out.value.links[left];
        const link& b = out.value.links[right];
        if (a.route != b.route || a.order + 1 != b.order) {
          continue;
        }
        if (matched != bad) {
          return unsupported_pairs("route continuity is ambiguous");
        }
        matched = right;
      }
      if (matched == bad) {
        continue;
      }
      const link& a = out.value.links[left];
      const link& b = out.value.links[matched];
      Vec3d chord = made.nodes[b.b].pos - made.nodes[a.a].pos;
      if (!norm_strict(&chord)) {
        return unsupported_pairs("zero length pair chord");
      }
      bused[left] = true;
      aused[matched] = true;
      if (out.value.links[left].brow != bad || out.value.links[matched].arow != bad) {
        return unsupported_pairs("incident already used");
      }
      const std::size_t row_id = add_pair(&out.value, n.id, left, matched, side(chord));
      out.value.links[left].brow = row_id;
      out.value.links[matched].arow = row_id;
    }

    for (std::size_t link_id : outgoing) {
      if (aused[link_id]) {
        continue;
      }
      aused[link_id] = true;
      if (out.value.links[link_id].arow != bad) {
        return unsupported_pairs("incident already used");
      }
      out.value.links[link_id].arow = add_open(&out.value, n.id, link_id, side(out.value.links[link_id].dir));
    }
    for (std::size_t link_id : incoming) {
      if (bused[link_id]) {
        continue;
      }
      bused[link_id] = true;
      if (out.value.links[link_id].brow != bad) {
        return unsupported_pairs("incident already used");
      }
      out.value.links[link_id].brow = add_open(&out.value, n.id, link_id, side(out.value.links[link_id].dir));
    }
  }

  for (const link& edge : out.value.links) {
    if (edge.arow == bad || edge.brow == bad) {
      return unsupported_pairs("link row is unresolved");
    }
  }
  out.ok = true;
  return out;
}

EditResult<intent> pipeline::make(const pairs& ps) const {
  EditResult<intent> out{};
  std::vector<bool> high_voltage_bundle(active_bundle_indices_.size(), false);
  for (std::size_t bundle = 0; bundle < active_bundle_indices_.size(); ++bundle) {
    const std::size_t spec_index = active_bundle_indices_[bundle];
    if (spec_index >= spec_.bundles.size()) {
      out.error = "bb2 unsupported: active bundle index is invalid";
      return out;
    }
    const EditResult<spec_view> v = view_for(state_, spec_.bundles[spec_index]);
    if (!v.ok || v.value.tmpl == nullptr) {
      out.error = v.ok ? "bb2 unsupported: bundle template not found" : v.error;
      return out;
    }
    high_voltage_bundle[bundle] = v.value.tmpl->category == ConnectionCategory::kHighVoltage;
  }
  auto incident_dir = [&](const link& edge, std::size_t node_id) {
    Vec3d dir = edge.dir;
    if (edge.b == node_id) {
      dir = Vec3d{-dir.x, -dir.y, -dir.z};
    }
    return norm(dir);
  };
  auto pair_is_bent = [&](const pair& item) {
    if (item.left >= ps.links.size() || item.right >= ps.links.size()) {
      return false;
    }
    const Vec3d left = incident_dir(ps.links[item.left], item.node);
    const Vec3d right = incident_dir(ps.links[item.right], item.node);
    return Dot(left, right) > -0.5;
  };
  auto add_row_intent = [&](std::size_t row_id, std::size_t bundle, intent_reason reason) {
    for (row_intent& item : out.value.rows) {
      if (item.row == row_id && item.bundle == bundle) {
        item.lower_required = true;
        if (item.reason == intent_reason::none) {
          item.reason = reason;
        }
        return;
      }
    }
    out.value.rows.push_back(row_intent{row_id, bundle, CurvePassMode::kPassThrough, true, reason});
  };
  for (const BackboneSpec::NodeBundleModeSpec& mode : spec_.node_bundle_modes) {
    if (mode.mode != BundleNodeMode::kPassThrough) {
      continue;
    }
    const std::size_t local = this->local(mode.point_index);
    if (local == bad || local >= g_.nodes.size()) {
      out.error = "bb2 unsupported: pass-through node mode point is missing";
      return out;
    }
    const auto bundle_it = std::find_if(spec_.bundles.begin(), spec_.bundles.end(), [&](const BackboneBundleSpec& spec) {
      return spec.bundle_template_id == mode.bundle_template_id;
    });
    if (bundle_it == spec_.bundles.end()) {
      out.error = "bb2 unsupported: pass-through bundle is missing";
      return out;
    }
    const std::size_t spec_index = static_cast<std::size_t>(std::distance(spec_.bundles.begin(), bundle_it));
    const auto active_it = std::find(active_bundle_indices_.begin(), active_bundle_indices_.end(), spec_index);
    if (active_it == active_bundle_indices_.end()) {
      out.error = "bb2 unsupported: pass-through bundle is inactive";
      return out;
    }
    const std::size_t bundle = static_cast<std::size_t>(std::distance(active_bundle_indices_.begin(), active_it));
    std::vector<std::size_t> matches{};
    for (const link& edge : ps.links) {
      if (!edge.is_new) {
        continue;
      }
      if (edge.a == local && edge.arow != bad) {
        matches.push_back(edge.arow);
      }
      if (edge.b == local && edge.brow != bad) {
        matches.push_back(edge.brow);
      }
    }
    std::sort(matches.begin(), matches.end());
    matches.erase(std::unique(matches.begin(), matches.end()), matches.end());
    if (matches.size() != 1) {
      out.error = "bb2 unsupported: pass-through target row is ambiguous";
      return out;
    }
    add_row_intent(matches.front(), bundle, intent_reason::node_mode_pass_through);
  }

  std::unordered_map<std::size_t, std::vector<std::size_t>> rows_by_node{};
  for (const row& r : ps.rows) {
    rows_by_node[r.node].push_back(r.id);
  }
  std::vector<bool> active(ps.rows.size(), false);
  for (const link& edge : ps.links) {
    if (!edge.is_new) {
      continue;
    }
    if (edge.arow < active.size()) {
      active[edge.arow] = true;
    }
    if (edge.brow < active.size()) {
      active[edge.brow] = true;
    }
  }
  for (const auto& item : rows_by_node) {
    if (item.second.size() < 2) {
      continue;
    }
    for (std::size_t row_id : item.second) {
      if (row_id >= active.size() || !active[row_id]) {
        continue;
      }
      for (std::size_t bundle = 0; bundle < active_bundle_indices_.size(); ++bundle) {
        add_row_intent(row_id, bundle, intent_reason::conflicting_rows);
      }
    }
  }
  for (const pair& item : ps.joins) {
    if (!pair_is_bent(item)) {
      continue;
    }
    for (std::size_t bundle = 0; bundle < active_bundle_indices_.size(); ++bundle) {
      if (high_voltage_bundle[bundle]) {
        add_row_intent(item.id, bundle, intent_reason::bent_pair);
      }
    }
  }
  out.ok = true;
  return out;
}

EditResult<groups> pipeline::make(const pairs& ps, const intent& intents) const {
  EditResult<groups> out{};
  std::unordered_map<std::size_t, std::vector<std::size_t>> rows_by_node{};
  for (const row& r : ps.rows) {
    rows_by_node[r.node].push_back(r.id);
  }
  std::unordered_map<std::size_t, int> order_by_row{};
  for (auto& item : rows_by_node) {
    std::vector<std::size_t>& rows = item.second;
    std::sort(rows.begin(), rows.end());
    for (std::size_t order = 0; order < rows.size(); ++order) {
      order_by_row[rows[order]] = static_cast<int>(order);
    }
  }
  for (const row_intent& item : intents.rows) {
    if (!item.lower_required) {
      continue;
    }
    if (item.row >= ps.rows.size() || item.bundle >= active_bundle_indices_.size()) {
      out.error = "bb2 unsupported: support group row is invalid";
      return out;
    }
    group placed{};
    placed.id = out.value.items.size();
    placed.node = ps.rows[item.row].node;
    placed.row_members.push_back(group_member{item.row, item.bundle});
    placed.group_axis = ps.rows[item.row].axis;
    placed.vertical_order = order_by_row.contains(item.row) ? order_by_row[item.row] : 0;
    placed.lower_offset_m = kLowerOffsetM;
    out.value.items.push_back(std::move(placed));
  }
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::emit_poles(topo* made, ChangeSet* changes) {
  EditResult<bool> out{};
  if (made == nullptr || changes == nullptr) {
    out.error = "bb2 topology: output missing";
    return out;
  }
  made->poles.reserve(g_.nodes.size());
  PoleTypeId pole_type = kInvalidPoleTypeId;
  if (needs_pole_type(g_)) {
    EditResult<PoleTypeId> resolved_type = pole_type_for(state_, spec_);
    if (!resolved_type.ok) {
      out.error = resolved_type.error;
      return out;
    }
    pole_type = resolved_type.value;
  }
  for (std::size_t i = 0; i < g_.nodes.size(); ++i) {
    if (g_.nodes[i].support == SupportKind::kMidair || g_.nodes[i].support == SupportKind::kExternal ||
        g_.nodes[i].support == SupportKind::kGround) {
      made->poles.push_back(kInvalidObjectId);
      continue;
    }
    if (!g_.nodes[i].is_new) {
      made->poles.push_back(g_.nodes[i].pole);
      continue;
    }
    Transformd tf{};
    tf.position = g_.nodes[i].pos;
    std::size_t prev_index = bad;
    std::size_t next_index = bad;
    for (const link& edge : g_.links) {
      if (!edge.is_new) {
        continue;
      }
      if (edge.b == i) {
        prev_index = edge.a;
      }
      if (edge.a == i) {
        next_index = edge.b;
      }
    }
    const Vec3d next = (next_index != bad && next_index < g_.nodes.size()) ? g_.nodes[next_index].pos : g_.nodes[i].pos;
    const Vec3d prev = (prev_index != bad && prev_index < g_.nodes.size()) ? g_.nodes[prev_index].pos : g_.nodes[i].pos;
    const Vec3d dir = g_.nodes[i].has_tangent ? g_.nodes[i].tangent
                                               : ((next_index != bad) ? (next - g_.nodes[i].pos)
                                                                      : (g_.nodes[i].pos - prev));
    tf.rotation_euler_deg.z = yaw(dir);
    EditResult<ObjectId> pole = state_.AddPole(tf, 10.0, "bb2-pole", PoleKind::kConcrete,
                                               g_.nodes[i].pinned ? PlacementMode::kManual : PlacementMode::kAuto);
    if (!pole.ok) {
      out.error = pole.error;
      return out;
    }
    add(*changes, pole.change_set);
    EditResult<ObjectId> typed = state_.ApplyPoleType(pole.value, pole_type);
    if (!typed.ok) {
      out.error = typed.error;
      return out;
    }
    add(*changes, typed.change_set);
    made->poles.push_back(pole.value);
    made->new_poles.push_back(pole.value);
  }

  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::check(const pairs& ps) const {
  for (const link& edge : ps.links) {
    if (!edge.is_new) {
      continue;
    }
    const ObjectId edge_id = saved_edge_for(state_, g_, edge);
    if (edge_id == kInvalidObjectId) {
      continue;
    }
    const auto bundles_it = state_.view().backbone_index().edge_bundles.find(edge_id);
    if (bundles_it == state_.view().backbone_index().edge_bundles.end()) {
      continue;
    }
    for (std::size_t spec_index : active_bundle_indices_) {
      const BackboneBundleSpec& spec = spec_.bundles[spec_index];
      for (ObjectId edge_bundle_id : bundles_it->second) {
        const SavedBackboneEdgeBundle* edge_bundle = state_.view().backbone_edge_bundle(edge_bundle_id);
        if (edge_bundle == nullptr) {
          continue;
        }
        const Bundle* bundle = state_.view().bundles().find(edge_bundle->bundle_id);
        if (bundle != nullptr && bundle->bundle_template_id == spec.bundle_template_id) {
          const auto spans_it = state_.view().backbone_index().edge_bundle_span_bindings.find(edge_bundle_id);
          if (spans_it != state_.view().backbone_index().edge_bundle_span_bindings.end()) {
            const SavedBackboneGraph& graph = state_.view().backbone();
            EditResult<spec_view> v = view_for(state_, spec);
            if (!v.ok) {
              EditResult<bool> error{};
              error.error = v.error;
              return error;
            }
            for (std::size_t index : spans_it->second) {
              if (index >= graph.span_bindings.size()) {
                continue;
              }
              const SavedBackboneSpanBinding& binding = graph.span_bindings[index];
              if (binding.lane_index < static_cast<std::size_t>(v.value.count)) {
                return unsupported("duplicate saved span binding");
              }
            }
          }
          return unsupported("duplicate saved edge bundle");
        }
      }
    }
  }
  EditResult<bool> out{};
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::emit_bundles(topo* made, ChangeSet* changes) {
  EditResult<bool> out{};
  if (made == nullptr || changes == nullptr) {
    out.error = "bb2 topology: output missing";
    return out;
  }
  made->bundles.reserve(active_bundle_indices_.size());
  made->bundle_specs.reserve(active_bundle_indices_.size());
  for (std::size_t spec_index : active_bundle_indices_) {
    const BackboneBundleSpec& spec = spec_.bundles[spec_index];
    EditResult<spec_view> v = view_for(state_, spec);
    if (!v.ok) {
      out.error = v.error;
      return out;
    }
    const ObjectId resolved = resolve_existing_bundle(state_, g_, spec);
    if (resolved != kInvalidObjectId) {
      made->bundles.push_back(resolved);
      made->bundle_specs.push_back(spec_index);
      continue;
    }
    EditResult<ObjectId> bundle = state_.AddBundle(v.value.count, v.value.tmpl->default_spacing_m, v.value.tmpl->id);
    if (!bundle.ok) {
      out.error = bundle.error;
      return out;
    }
    add(*changes, bundle.change_set);
    made->bundles.push_back(bundle.value);
    made->bundle_specs.push_back(spec_index);
  }

  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::emit_ports(topo* made, const pairs& ps, ChangeSet* changes) {
  EditResult<bool> out{};
  if (made == nullptr || changes == nullptr) {
    out.error = "bb2 topology: output missing";
    return out;
  }

  std::vector<ObjectId> node_id_by_local(g_.nodes.size(), kInvalidObjectId);
  for (std::size_t i = 0; i < g_.nodes.size(); ++i) {
    node_id_by_local[i] = g_.nodes[i].saved;
  }
  std::vector<SavedBackboneEdgeRef> edge_by_link(ps.links.size());
  for (const link& edge : ps.links) {
    if (edge.id < edge_by_link.size()) {
      edge_by_link[edge.id] = ref_for_existing_edge(state_, g_, edge);
    }
  }
  std::vector<bool> active_rows(ps.rows.size(), false);
  for (const link& edge : ps.links) {
    if (!edge.is_new) {
      continue;
    }
    if (edge.arow < active_rows.size()) {
      active_rows[edge.arow] = true;
    }
    if (edge.brow < active_rows.size()) {
      active_rows[edge.brow] = true;
    }
  }
  const std::vector<Vec3d> shifts = row_shifts(ps);
  made->rows.resize(ps.rows.size());
  for (const row& r : ps.rows) {
    if (r.node >= made->poles.size() || r.node >= g_.nodes.size()) {
      out.error = "bb2 topology: row node missing";
      return out;
    }
    trow& tr = made->rows[r.id];
    tr.row = r.id;
    tr.node = r.node;
    tr.source = r.source;
    tr.axis = r.axis;
    tr.pole = made->poles[r.node];
    if (r.id >= active_rows.size() || !active_rows[r.id]) {
      continue;
    }
    const bool ownerless = g_.nodes[r.node].support == SupportKind::kMidair ||
                           g_.nodes[r.node].support == SupportKind::kExternal ||
                           g_.nodes[r.node].support == SupportKind::kGround;
    if (!ownerless && tr.pole == kInvalidObjectId) {
      out.error = "bb2 topology: active row pole missing";
      return out;
    }
    tr.ports.resize(made->bundles.size());
    for (std::size_t bundle_index = 0; bundle_index < made->bundles.size(); ++bundle_index) {
      if (bundle_index >= made->bundle_specs.size()) {
        out.error = "bb2 topology: bundle spec missing";
        return out;
      }
      const std::size_t spec_index = made->bundle_specs[bundle_index];
      EditResult<spec_view> v = view_for(state_, spec_.bundles[spec_index]);
      if (!v.ok) {
        out.error = v.error;
        return out;
      }
      const double group_offset =
          (static_cast<double>(bundle_index) - (static_cast<double>(made->bundles.size() - 1) * 0.5)) *
          (v.value.tmpl->default_spacing_m * static_cast<double>(v.value.count + 1));
      tr.ports[bundle_index].reserve(static_cast<std::size_t>(v.value.count));
      PortPlacementBand band{};
      if (!ownerless) {
        EditResult<PortPlacementBand> resolved_band = band_for(state_, tr.pole, v.value);
        if (!resolved_band.ok) {
          out.error = resolved_band.error;
          return out;
        }
        band = resolved_band.value;
      }
      const port_scope scope{spec_.bundles[spec_index].bundle_template_id, port_kind(v.value.tmpl->category),
                             port_layer(v.value.layer)};
      for (int lane = 0; lane < v.value.count; ++lane) {
        const double lane_offset =
            (static_cast<double>(lane) - (static_cast<double>(v.value.count - 1) * 0.5)) * v.value.tmpl->default_spacing_m;
        const double offset = group_offset + lane_offset + spec_.constraints.lateral_offset_m;
        const Vec3d shift = (r.id < shifts.size()) ? shifts[r.id] : Vec3d{};
        const double height = ownerless ? 0.0 : band.height_center_m;
        Vec3d p = g_.nodes[r.node].pos +
                  Vec3d{r.axis.x * offset + shift.x, r.axis.y * offset + shift.y, height};
        const SavedBackboneRowKey row_key = key_for(ps, tr, node_id_by_local, edge_by_link);
        EditResult<ObjectId> resolved =
            resolve_port_binding(state_, tr.pole, row_key, static_cast<std::size_t>(lane), scope);
        if (!resolved.ok) {
          out.error = resolved.error;
          return out;
        }
        if (resolved.value != kInvalidObjectId) {
          tr.ports[bundle_index].push_back(resolved.value);
          continue;
        }
        EditResult<ObjectId> port =
            state_.AddPort(ownerless ? kInvalidObjectId : made->poles[r.node], p, scope.kind, scope.layer);
        if (!port.ok) {
          out.error = port.error;
          return out;
        }
        add(*changes, port.change_set);
        tr.ports[bundle_index].push_back(port.value);
      }
    }
  }

  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> pipeline::emit_spans(topo* made, const pairs& ps, ChangeSet* changes) {
  EditResult<bool> out{};
  if (made == nullptr || changes == nullptr) {
    out.error = "bb2 topology: output missing";
    return out;
  }
  for (const link& edge : ps.links) {
    if (!edge.is_new) {
      continue;
    }
    if (edge.arow >= made->rows.size() || edge.brow >= made->rows.size()) {
      out.error = "bb2 topology: span row missing";
      return out;
    }
    for (std::size_t bundle_index = 0; bundle_index < made->bundles.size(); ++bundle_index) {
      if (bundle_index >= made->bundle_specs.size()) {
        out.error = "bb2 topology: bundle spec missing";
        return out;
      }
      EditResult<spec_view> v = view_for(state_, spec_.bundles[made->bundle_specs[bundle_index]]);
      if (!v.ok) {
        out.error = v.error;
        return out;
      }
      for (int lane = 0; lane < v.value.count; ++lane) {
        if (made->rows[edge.arow].ports.size() <= bundle_index ||
            made->rows[edge.brow].ports.size() <= bundle_index ||
            made->rows[edge.arow].ports[bundle_index].size() <= static_cast<std::size_t>(lane) ||
            made->rows[edge.brow].ports[bundle_index].size() <= static_cast<std::size_t>(lane)) {
          out.error = "bb2 topology: span port missing";
          return out;
        }
        EditResult<ObjectId> span = state_.AddSpan(
            made->rows[edge.arow].ports[bundle_index][static_cast<std::size_t>(lane)],
            made->rows[edge.brow].ports[bundle_index][static_cast<std::size_t>(lane)], span_kind(v.value.tmpl->category),
            v.value.layer, made->bundles[bundle_index]);
        if (!span.ok) {
          out.error = span.error;
          return out;
        }
        EditResult<bool> endpoints = state_.set_span_endpoint_nodes(span.value, made->rows[edge.arow].pole,
                                                                     made->rows[edge.brow].pole);
        if (!endpoints.ok) {
          out.error = endpoints.error;
          return out;
        }
        add(*changes, span.change_set);
        made->spans.push_back(
            tspan{span.value, edge.id, bundle_index, static_cast<std::size_t>(lane), edge.arow, edge.brow});
      }
    }
  }

  out.value = true;
  out.ok = true;
  return out;
}

EditResult<topo> pipeline::emit(const pairs& ps) {
  EditResult<topo> out{};
  topo made{};
  auto step = [&](EditResult<bool> result) -> bool {
    if (!result.ok) {
      out.error = result.error;
      return false;
    }
    return true;
  };
  if (!step(emit_poles(&made, &out.change_set)) || !step(emit_bundles(&made, &out.change_set)) ||
      !step(emit_ports(&made, ps, &out.change_set)) || !step(emit_spans(&made, ps, &out.change_set))) {
    return out;
  }
  out.value = std::move(made);
  out.ok = true;
  return out;
}

rules pipeline::make(const topo& made, const groups& placement) const {
  rules out{};
  auto group_for = [&](std::size_t row, std::size_t bundle) -> const group* {
    for (const group& item : placement.items) {
      for (const group_member& member : item.row_members) {
        if (member.row == row && member.bundle == bundle) {
          return &item;
        }
      }
    }
    return nullptr;
  };
  for (const tspan& span : made.spans) {
    const trow& arow = made.rows[span.arow];
    const trow& brow = made.rows[span.brow];
    const group* start_group = group_for(span.arow, span.bundle);
    const group* end_group = group_for(span.brow, span.bundle);
    SpanLayoutRule rule{};
    rule.span_id = span.id;
    rule.flow_kind = BackboneFlowKind::kMain;
    rule.pass_mode = CurvePassMode::kPassThrough;
    if (start_group != nullptr || end_group != nullptr) {
      rule.flow_kind = BackboneFlowKind::kBranch;
      rule.pass_mode = CurvePassMode::kBranch;
      rule.lowering_kind = BackboneLoweringKind::kBranchSupport;
    }
    auto endpoint = [&](ObjectId pole_id, ObjectId port_id) {
      EndpointLayoutRule e{};
      e.endpoint_node_id = pole_id;
      e.port_id = port_id;
      e.semantic.owner_pole_id = pole_id;
      e.flow_kind = BackboneFlowKind::kMain;
      e.origin = LayoutOriginKind::kMainSupport;
      e.endpoint_source = LayoutEndpointSourceKind::kPlainSupport;
      e.port_source = PortPlacementSourceKind::kGenerated;
      e.side = SlotSide::kCenter;
      e.endpoint_mode = CurveEndpointMode::kDirectThrough;
      e.same_level_feasible = true;
      return e;
    };
    rule.start = endpoint(arow.pole, arow.ports[span.bundle][span.lane]);
    rule.end = endpoint(brow.pole, brow.ports[span.bundle][span.lane]);
    auto apply_group = [](const group* source, EndpointLayoutRule* endpoint) {
      if (source == nullptr || endpoint == nullptr) {
        return;
      }
      endpoint->flow_kind = BackboneFlowKind::kBranch;
      endpoint->origin = LayoutOriginKind::kBranchSupport;
      endpoint->default_lower_required = true;
      endpoint->same_level_feasible = false;
      endpoint->same_level_reason = SameLevelFeasibilityReason::kBundleRule;
      endpoint->semantic.lower_required = true;
      endpoint->semantic.lowering_blocked_by_policy = false;
      endpoint->semantic.support_group_id = static_cast<int>(source->id);
      endpoint->branch_down_offset_m = source->lower_offset_m;
      endpoint->automatic_branch_down_offset_m = source->lower_offset_m;
    };
    apply_group(start_group, &rule.start);
    apply_group(end_group, &rule.end);
    auto append_group_rule = [&](const EndpointLayoutRule& endpoint) {
      if (!UsesAuthoritativeGroupedLoweredSupport(endpoint.semantic)) {
        return;
      }
      const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint.semantic);
      SupportGroupDecision& group = rule.support_group_rules[key];
      static_cast<LayoutSemantic&>(group) = endpoint.semantic;
      group.side = endpoint.side;
      group.origin = endpoint.origin;
      group.order_decision_policy = endpoint.order_decision_policy;
      group.order_decision_choice = endpoint.order_decision_choice;
      group.order_decision_choice_reason = endpoint.order_decision_choice_reason;
      group.chosen_side = endpoint.chosen_side;
      group.used_junction_pair_side_assignment = endpoint.used_junction_pair_side_assignment;
    };
    append_group_rule(rule.start);
    append_group_rule(rule.end);
    out.data.spans.push_back(std::move(rule));
  }
  return out;
}

EditResult<layout> pipeline::make(const rules& made) const {
  EditResult<layout> out{};
  const EditState& edit = state_.view().edit_state();
  auto endpoint = [&](const EndpointLayoutRule& rule, LayoutEndpoint* target) -> bool {
    const Port* port = edit.ports.find(rule.port_id);
    if (port == nullptr || target == nullptr) {
      return false;
    }
    static_cast<LayoutSemantic&>(*target) = rule.semantic;
    target->endpoint_node_id = rule.endpoint_node_id;
    target->port_id = rule.port_id;
    target->flow_kind = rule.flow_kind;
    target->origin = rule.origin;
    target->endpoint_source = rule.endpoint_source;
    target->port_source = rule.port_source;
    target->side = rule.side;
    target->endpoint_mode = rule.endpoint_mode;
    target->automatic_branch_down_offset_m = rule.automatic_branch_down_offset_m;
    target->branch_down_offset_m = rule.branch_down_offset_m;
    target->default_lower_required = rule.default_lower_required;
    target->same_level_feasible = rule.same_level_feasible;
    target->unresolved_same_level_conflict = rule.unresolved_same_level_conflict;
    target->same_level_reason = rule.same_level_reason;
    target->projected_spacing_topview_m = rule.projected_spacing_topview_m;
    target->required_clearance_m = rule.required_clearance_m;
    target->solver_used_same_level_constraint = rule.solver_used_same_level_constraint;
    target->used_special_case_ports = rule.used_special_case_ports;
    target->order_decision_policy = rule.order_decision_policy;
    target->order_decision_choice = rule.order_decision_choice;
    target->order_decision_choice_reason = rule.order_decision_choice_reason;
    target->chosen_side = rule.chosen_side;
    target->used_junction_pair_side_assignment = rule.used_junction_pair_side_assignment;
    target->down_offset_variation = rule.down_offset_variation;
    target->support_world = port->world_position;
    target->endpoint_world = port->world_position;
    if (rule.default_lower_required || rule.semantic.lower_required) {
      const double lower_offset = rule.branch_down_offset_m > 0.0 ? rule.branch_down_offset_m : rule.automatic_branch_down_offset_m;
      target->endpoint_world.z -= lower_offset;
      target->branch_down_offset_m = lower_offset;
      target->automatic_branch_down_offset_m = lower_offset;
    }
    target->departure_dir = WorldForward();
    return true;
  };
  for (const SpanLayoutRule& rule : made.data.spans) {
    const Span* span = edit.spans.find(rule.span_id);
    if (span == nullptr) {
      out.error = "bb2 layout: span not found";
      return out;
    }
    SpanLayoutEntry entry{};
    entry.span_id = rule.span_id;
    entry.flow_kind = rule.flow_kind;
    entry.pass_mode = rule.pass_mode;
    entry.variation_flow_key = rule.variation_flow_key;
    entry.lowering_kind = rule.lowering_kind;
    if (!endpoint(rule.start, &entry.start) || !endpoint(rule.end, &entry.end)) {
      out.error = "bb2 layout: endpoint port not found";
      return out;
    }
    auto append_group_key = [&](const LayoutEndpoint& endpoint) {
      if (!UsesAuthoritativeGroupedLoweredSupport(endpoint)) {
        return;
      }
      const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint);
      if (std::find(entry.lowered_support_group_keys.begin(), entry.lowered_support_group_keys.end(), key) ==
          entry.lowered_support_group_keys.end()) {
        entry.lowered_support_group_keys.push_back(key);
      }
    };
    append_group_key(entry.start);
    append_group_key(entry.end);
    const Vec3d chord = entry.end.endpoint_world - entry.start.endpoint_world;
    entry.basis_length_m = std::sqrt(chord.x * chord.x + chord.y * chord.y + chord.z * chord.z);
    entry.effective_sag_ratio = 0.0;
    entry.continuity_preference = CableContinuityPolicyHint::kAuto;
    entry.bend_stiffness_hint = 1.0;
    out.value.entries.push_back(std::move(entry));
  }
  out.ok = true;
  return out;
}

geom pipeline::make(const layout& made) const {
  geom out{};
  out.curves.data.reserve(made.entries.size());
  out.boxes.data.reserve(made.entries.size());
  for (const SpanLayoutEntry& entry : made.entries) {
    DetailCurve detail = make_curve(state_, entry.span_id, entry);
    BoundsCacheEntry cached = bounds(detail);
    out.boxes.data.push_back({entry.span_id, std::move(cached)});
    out.curves.data.push_back({entry.span_id, std::move(detail)});
  }
  return out;
}

draw pipeline::make(const layout& placed, const geom& shaped) const {
  draw out{};
  out.visuals.reserve(placed.entries.size());
  out.renders.reserve(shaped.curves.data.size());
  const VisualSettings& visual_settings = state_.view().visual_settings();
  for (const auto& item : shaped.curves.data) {
    out.renders.push_back({item.first, render(state_, item.first, item.second)});
  }
  for (const SpanLayoutEntry& entry : placed.entries) {
    out.visuals.push_back({entry.span_id, visual(visual_settings, entry)});
  }
  return out;
}

void pipeline::save(const rules& made) {
  state_.cache_span_rules(made.data);
}

void pipeline::save(const layout& made) {
  struct cached_group {
    LoweredSupportGroupKey key{};
    SupportGroupDecision decision{};
    LoweredSupportGroupPlacement placement{};
  };
  std::vector<cached_group> groups{};
  auto group_for = [&](const LoweredSupportGroupKey& key) -> cached_group& {
    for (cached_group& item : groups) {
      if (item.key == key) {
        return item;
      }
    }
    cached_group item{};
    item.key = key;
    groups.push_back(std::move(item));
    return groups.back();
  };
  auto collect_group = [&](const LayoutEndpoint& endpoint) {
    if (!UsesAuthoritativeGroupedLoweredSupport(endpoint)) {
      return;
    }
    const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint);
    cached_group& item = group_for(key);
    static_cast<LayoutSemantic&>(item.decision) = endpoint;
    item.decision.side = endpoint.side;
    item.decision.origin = endpoint.origin;
    item.decision.order_decision_policy = endpoint.order_decision_policy;
    item.decision.order_decision_choice = endpoint.order_decision_choice;
    item.decision.order_decision_choice_reason = endpoint.order_decision_choice_reason;
    item.decision.chosen_side = endpoint.chosen_side;
    item.decision.used_junction_pair_side_assignment = endpoint.used_junction_pair_side_assignment;
    item.placement.grouped_port_count += 1;
    item.placement.down_offset_m = std::max(item.placement.down_offset_m, endpoint.branch_down_offset_m);
    if (item.placement.attachment_worlds.empty()) {
      item.placement.mount_world = endpoint.support_world;
      item.placement.tip_world = endpoint.endpoint_world;
    }
    item.placement.attachment_worlds.push_back(endpoint.endpoint_world);
  };
  for (const SpanLayoutEntry& entry : made.entries) {
    collect_group(entry.start);
    collect_group(entry.end);
  }
  for (cached_group& item : groups) {
    state_.cache_support_group(std::move(item.decision), std::move(item.placement));
  }
  for (SpanLayoutEntry entry : made.entries) {
    state_.cache_span_layout(std::move(entry));
  }
}

void pipeline::save(geom made) {
  for (auto& item : made.curves.data) {
    state_.cache_span_curve(item.first, std::move(item.second));
  }
  for (auto& item : made.boxes.data) {
    state_.cache_span_bounds(item.first, std::move(item.second));
  }
}

void pipeline::save(draw made) {
  for (auto& item : made.visuals) {
    state_.cache_span_visual(item.first, std::move(item.second));
  }
  for (auto& item : made.renders) {
    state_.cache_span_render(item.first, std::move(item.second));
  }
}

EditResult<bool> pipeline::save_graph(const topo& made, const pairs& ps) {
  EditResult<bool> out{};
  std::vector<int> path_index_by_local(g_.nodes.size(), -1);
  for (std::size_t input_index = 0; input_index < local_by_input_.size(); ++input_index) {
    const std::size_t local = local_by_input_[input_index];
    if (local < path_index_by_local.size()) {
      path_index_by_local[local] = static_cast<int>(input_index);
    }
  }

  std::vector<ObjectId> node_id_by_local(g_.nodes.size(), kInvalidObjectId);
  for (std::size_t i = 0; i < g_.nodes.size() && i < made.poles.size(); ++i) {
    const ObjectId source_a = saved_node_id_for(state_, g_.nodes[i].source_edge_node_a);
    const ObjectId source_b = saved_node_id_for(state_, g_.nodes[i].source_edge_node_b);
    if (g_.nodes[i].saved != kInvalidObjectId) {
      node_id_by_local[i] = g_.nodes[i].saved;
      if (g_.nodes[i].on_route && i < path_index_by_local.size()) {
        EditResult<bool> indexed =
            state_.bind_backbone_node_path_point_index(node_id_by_local[i], path_index_by_local[i]);
        if (!indexed.ok) {
          out.error = indexed.error;
          return out;
        }
      }
      if (g_.nodes[i].pole == kInvalidObjectId) {
        EditResult<bool> bound = state_.bind_backbone_node_bundle_modes(g_.nodes[i].saved, g_.nodes[i].bundle_modes);
        if (!bound.ok) {
          out.error = bound.error;
          return out;
        }
      }
      continue;
    }
    node_id_by_local[i] = state_.save_backbone_node(made.poles[i], g_.nodes[i].pos, g_.nodes[i].support, source_a,
                                                    source_b, g_.nodes[i].source_edge_t, g_.nodes[i].bundle_modes);
    if (g_.nodes[i].on_route && i < path_index_by_local.size()) {
      EditResult<bool> indexed =
          state_.bind_backbone_node_path_point_index(node_id_by_local[i], path_index_by_local[i]);
      if (!indexed.ok) {
        out.error = indexed.error;
        return out;
      }
    }
  }

  std::vector<SavedBackboneEdgeRef> edge_by_link(ps.links.size());
  for (const link& edge : ps.links) {
    if (edge.a >= node_id_by_local.size() || edge.b >= node_id_by_local.size() || edge.id >= edge_by_link.size()) {
      continue;
    }
    if (edge.is_new) {
      edge_by_link[edge.id] =
          state_.save_backbone_edge(node_id_by_local[edge.a], node_id_by_local[edge.b], edge.route, edge.order, edge.dir);
      continue;
    }
    edge_by_link[edge.id] = ref_for_existing_edge(state_, g_, edge);
    if (edge_by_link[edge.id].edge_id == kInvalidObjectId) {
      out.error = "bb2 graph: context link saved edge missing";
      return out;
    }
  }


  for (const tspan& span : made.spans) {
    if (span.link >= edge_by_link.size() || span.link >= ps.links.size() || span.bundle >= made.bundles.size()) {
      continue;
    }
    const link& edge = ps.links[span.link];
    const SavedBackboneEdgeRef& stored = edge_by_link[span.link];
    if (stored.edge_id == kInvalidObjectId || edge.a >= node_id_by_local.size() || edge.b >= node_id_by_local.size()) {
      continue;
    }
    const bool edge_forward =
        stored.node_a == node_id_by_local[edge.a] && stored.node_b == node_id_by_local[edge.b];
    const ObjectId edge_bundle_id =
        state_.bind_backbone_bundle(stored.edge_id, made.bundles[span.bundle], edge_forward, edge.route, edge.order, edge.dir);
    EditResult<bool> span_bound = state_.bind_backbone_span(edge_bundle_id, span.lane, span.id);
    if (!span_bound.ok) {
      out.error = span_bound.error;
      return out;
    }
    auto bind_port = [&](std::size_t row_index) -> bool {
      if (row_index >= made.rows.size() || span.bundle >= made.rows[row_index].ports.size() ||
          span.lane >= made.rows[row_index].ports[span.bundle].size()) {
        out.error = "bb2 graph: port binding row missing";
        return false;
      }
      const SavedBackboneRowKey row_key = key_for(ps, made.rows[row_index], node_id_by_local, edge_by_link);
      const Bundle* bundle = state_.view().bundles().find(made.bundles[span.bundle]);
      if (bundle == nullptr) {
        out.error = "bb2 graph: bundle missing for port binding";
        return false;
      }
      const Port* port = state_.view().ports().find(made.rows[row_index].ports[span.bundle][span.lane]);
      if (port == nullptr) {
        out.error = "bb2 graph: port missing for binding";
        return false;
      }
      EditResult<bool> bound =
          state_.bind_backbone_port(edge_bundle_id, row_key, span.lane, bundle->bundle_template_id, port->kind,
                                    port->layer, port->id);
      if (!bound.ok) {
        out.error = bound.error;
        return false;
      }
      return true;
    };
    if (!bind_port(span.arow) || !bind_port(span.brow)) {
      return out;
    }
  }

  out.value = true;
  out.ok = true;
  return out;
}

EditResult<GenerateBundleFromPathResult> pipeline::build() {
  EditResult<GenerateBundleFromPathResult> out{};
  auto elapsed_ms = [](const auto& started) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  };
  auto started = std::chrono::steady_clock::now();
  EditResult<pairs> ps = make(g_);
  out.value.timing.pairs_ms = elapsed_ms(started);
  if (!ps.ok) {
    out.error = ps.error;
    return out;
  }
  started = std::chrono::steady_clock::now();
  EditResult<bool> duplicates = check(ps.value);
  out.value.timing.preflight_ms = elapsed_ms(started);
  if (!duplicates.ok) {
    out.error = duplicates.error;
    return out;
  }
  started = std::chrono::steady_clock::now();
  EditResult<intent> intents = make(ps.value);
  out.value.timing.intent_ms = elapsed_ms(started);
  if (!intents.ok) {
    out.error = intents.error;
    return out;
  }
  if (active_bundle_indices_.empty()) {
    out.ok = true;
    return out;
  }
  started = std::chrono::steady_clock::now();
  EditResult<groups> placement = make(ps.value, intents.value);
  out.value.timing.support_groups_ms = elapsed_ms(started);
  if (!placement.ok) {
    out.error = placement.error;
    return out;
  }
  started = std::chrono::steady_clock::now();
  EditResult<topo> made = emit(ps.value);
  out.value.timing.emit_ms = elapsed_ms(started);
  if (!made.ok) {
    out.error = made.error;
    return out;
  }
  started = std::chrono::steady_clock::now();
  EditResult<bool> graph_saved = save_graph(made.value, ps.value);
  out.value.timing.save_graph_ms = elapsed_ms(started);
  if (!graph_saved.ok) {
    out.error = graph_saved.error;
    return out;
  }
  started = std::chrono::steady_clock::now();
  rules saved = make(made.value, placement.value);
  save(saved);
  out.value.timing.rules_ms = elapsed_ms(started);
  started = std::chrono::steady_clock::now();
  EditResult<layout> placed = make(saved);
  if (!placed.ok) {
    out.error = placed.error;
    return out;
  }
  save(placed.value);
  out.value.timing.layout_ms = elapsed_ms(started);
  started = std::chrono::steady_clock::now();
  geom shaped = make(placed.value);
  out.value.timing.geom_ms = elapsed_ms(started);
  started = std::chrono::steady_clock::now();
  draw drawn = make(placed.value, shaped);
  out.value.timing.draw_ms = elapsed_ms(started);
  save(std::move(shaped));
  save(std::move(drawn));
  out.change_set = std::move(made.change_set);
  out.value.generated_pole_ids = made.value.new_poles;
  out.value.bundle_ids = made.value.bundles;
  out.value.bundle_id = made.value.bundles.empty() ? kInvalidObjectId : made.value.bundles.front();
  out.value.generated_span_ids.reserve(made.value.spans.size());
  for (const tspan& span : made.value.spans) {
    out.value.generated_span_ids.push_back(span.id);
  }
  out.ok = true;
  return out;
}

} // namespace wire::core::generation::bb2
