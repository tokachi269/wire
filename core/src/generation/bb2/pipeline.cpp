#include "pipeline.hpp"

#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <utility>

namespace wire::core::generation::bb2 {
namespace {

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
  if (tmpl.count_rule == BundleCountRuleKind::kFixed && spec.count > 0) {
    out.error = "bb2 unsupported: fixed bundle count override";
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

double yaw(Vec3d forward) {
  forward = norm(forward);
  return std::atan2(forward.y, forward.x) * (180.0 / 3.14159265358979323846);
}

double len(Vec3d v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3d mul(Vec3d v, double k) {
  return Vec3d{v.x * k, v.y * k, v.z * k};
}

DetailCurve line(const Vec3d& a, const Vec3d& b) {
  const Vec3d d = b - a;
  const double l = len(d);
  DetailCurve out{};
  out.start_constraint.point = a;
  out.end_constraint.point = b;
  out.control_points = {a, a + mul(d, 1.0 / 3.0), a + mul(d, 2.0 / 3.0), b};
  DetailCurveSegment segment{};
  segment.control_points = out.control_points;
  segment.u_start = 0.0;
  segment.u_end = 1.0;
  out.segments.push_back(segment);
  out.sample_points = {a, b};
  out.arc_length_table = {{0.0, 0.0}, {1.0, l}};
  out.visible_intervals = {{0.0, l}};
  out.total_length_m = l;
  return out;
}

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

EditResult<bool> pipeline::prepare() {
  EditResult<bool> out{};
  g_ = {};
  std::vector<Vec3d> pts = spec_.path.polyline;
  std::unordered_map<std::size_t, const BackboneInputSpec::NodeSpec*> spec_by_point{};
  spec_by_point.reserve(spec_.path.node_specs.size());
  for (const BackboneInputSpec::NodeSpec& spec : spec_.path.node_specs) {
    if (spec.point_index >= pts.size()) {
      out.error = "bb2 unsupported: node spec point index is out of range";
      return out;
    }
    if (spec.support_kind != SupportKind::kPole || spec.node_id == kInvalidObjectId) {
      out.error = "bb2 unsupported: node specs require an existing pole";
      return out;
    }
    if (!spec_by_point.emplace(spec.point_index, &spec).second) {
      out.error = "bb2 unsupported: duplicate node spec";
      return out;
    }
  }
  if (spec_.direction_mode == PathDirectionMode::kReverse) {
    std::reverse(pts.begin(), pts.end());
    std::unordered_map<std::size_t, const BackboneInputSpec::NodeSpec*> reversed{};
    reversed.reserve(spec_by_point.size());
    for (const auto& item : spec_by_point) {
      reversed.emplace(pts.size() - 1 - item.first, item.second);
    }
    spec_by_point.swap(reversed);
  }
  g_.nodes.reserve(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    node n{};
    n.id = i;
    n.pos = pts[i];
    if (const auto spec_it = spec_by_point.find(i); spec_it != spec_by_point.end()) {
      const Pole* pole = state_.view().poles().find(spec_it->second->node_id);
      if (pole == nullptr) {
        out.error = "bb2 unsupported: node spec pole not found";
        return out;
      }
      n.pole = pole->id;
      n.pos = pole->world_transform.position;
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
    edge.dir = pts[i + 1] - pts[i];
    g_.links.push_back(edge);
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
    if (mode.mode != BundleNodeMode::kNotPresent) {
      return unsupported("node bundle mode is not in milestone 7");
    }
  }
  if (!spec_.constraints.avoid_points.empty() || spec_.constraints.avoid_radius_m != 0.0) {
    return unsupported("constraints are not in milestone 1");
  }
  if (state_.view().pole_types().find(spec_.pole_type_id) == state_.view().pole_types().end()) {
    return unsupported("pole type not found");
  }
  for (const BackboneBundleSpec& spec : spec_.bundles) {
    EditResult<spec_view> checked = view_for(state_, spec);
    if (!checked.ok) {
      EditResult<bool> failed{};
      failed.error = checked.error;
      return failed;
    }
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
    edge.dir = made.nodes[edge.b].pos - made.nodes[edge.a].pos;
    if (!norm_strict(&edge.dir)) {
      return unsupported_pairs("zero length link");
    }
    out_links[edge.a].push_back(edge.id);
    in[edge.b].push_back(edge.id);
  }

  std::vector<bool> aused(out.value.links.size(), false);
  std::vector<bool> bused(out.value.links.size(), false);
  for (const node& n : made.nodes) {
    if (in[n.id].empty() && out_links[n.id].size() == 1) {
      const std::size_t link_id = out_links[n.id].front();
      if (aused[link_id]) {
        return unsupported_pairs("incident reused");
      }
      aused[link_id] = true;
      out.value.links[link_id].arow = add_open(&out.value, n.id, link_id, side(out.value.links[link_id].dir));
      continue;
    }
    if (in[n.id].size() == 1 && out_links[n.id].empty()) {
      const std::size_t link_id = in[n.id].front();
      if (bused[link_id]) {
        return unsupported_pairs("incident reused");
      }
      bused[link_id] = true;
      out.value.links[link_id].brow = add_open(&out.value, n.id, link_id, side(out.value.links[link_id].dir));
      continue;
    }
    if (in[n.id].size() == 1 && out_links[n.id].size() == 1) {
      const std::size_t left = in[n.id].front();
      const std::size_t right = out_links[n.id].front();
      if (bused[left] || aused[right]) {
        return unsupported_pairs("incident reused");
      }
      const link& a = out.value.links[left];
      const link& b = out.value.links[right];
      if (a.route != b.route || a.order + 1 != b.order) {
        return unsupported_pairs("route continuity is ambiguous");
      }
      Vec3d chord = made.nodes[b.b].pos - made.nodes[a.a].pos;
      if (!norm_strict(&chord)) {
        return unsupported_pairs("zero length pair chord");
      }
      bused[left] = true;
      aused[right] = true;
      const std::size_t row_id = add_pair(&out.value, n.id, left, right, side(chord));
      out.value.links[left].brow = row_id;
      out.value.links[right].arow = row_id;
      continue;
    }
    return unsupported_pairs("node incidence is ambiguous");
  }

  for (const link& edge : out.value.links) {
    if (edge.arow == bad || edge.brow == bad) {
      return unsupported_pairs("link row is unresolved");
    }
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
  for (std::size_t i = 0; i < g_.nodes.size(); ++i) {
    if (!g_.nodes[i].is_new) {
      made->poles.push_back(g_.nodes[i].pole);
      continue;
    }
    Transformd tf{};
    tf.position = g_.nodes[i].pos;
    const Vec3d next = (i + 1 < g_.nodes.size()) ? g_.nodes[i + 1].pos : g_.nodes[i].pos;
    const Vec3d prev = (i > 0) ? g_.nodes[i - 1].pos : g_.nodes[i].pos;
    tf.rotation_euler_deg.z = yaw((i + 1 < g_.nodes.size()) ? (next - g_.nodes[i].pos) : (g_.nodes[i].pos - prev));
    EditResult<ObjectId> pole = state_.AddPole(tf, 10.0, "bb2-pole", PoleKind::kConcrete);
    if (!pole.ok) {
      out.error = pole.error;
      return out;
    }
    add(*changes, pole.change_set);
    EditResult<ObjectId> typed = state_.ApplyPoleType(pole.value, spec_.pole_type_id);
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

EditResult<bool> pipeline::emit_bundles(topo* made, ChangeSet* changes) {
  EditResult<bool> out{};
  if (made == nullptr || changes == nullptr) {
    out.error = "bb2 topology: output missing";
    return out;
  }
  made->bundles.reserve(spec_.bundles.size());
  for (const BackboneBundleSpec& spec : spec_.bundles) {
    EditResult<spec_view> v = view_for(state_, spec);
    if (!v.ok) {
      out.error = v.error;
      return out;
    }
    EditResult<ObjectId> bundle = state_.AddBundle(v.value.count, v.value.tmpl->default_spacing_m, v.value.tmpl->id);
    if (!bundle.ok) {
      out.error = bundle.error;
      return out;
    }
    add(*changes, bundle.change_set);
    made->bundles.push_back(bundle.value);
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
    tr.ports.resize(spec_.bundles.size());
    for (std::size_t bundle_index = 0; bundle_index < spec_.bundles.size(); ++bundle_index) {
      EditResult<spec_view> v = view_for(state_, spec_.bundles[bundle_index]);
      if (!v.ok) {
        out.error = v.error;
        return out;
      }
      const double group_offset =
          (static_cast<double>(bundle_index) - (static_cast<double>(spec_.bundles.size() - 1) * 0.5)) *
          (v.value.tmpl->default_spacing_m * static_cast<double>(v.value.count + 1));
      tr.ports[bundle_index].reserve(static_cast<std::size_t>(v.value.count));
      EditResult<PortPlacementBand> band = band_for(state_, tr.pole, v.value);
      if (!band.ok) {
        out.error = band.error;
        return out;
      }
      for (int lane = 0; lane < v.value.count; ++lane) {
        const double lane_offset =
            (static_cast<double>(lane) - (static_cast<double>(v.value.count - 1) * 0.5)) * v.value.tmpl->default_spacing_m;
        const double offset = group_offset + lane_offset + spec_.constraints.lateral_offset_m;
        Vec3d p = g_.nodes[r.node].pos + Vec3d{r.axis.x * offset, r.axis.y * offset, band.value.height_center_m};
        EditResult<ObjectId> port =
            state_.AddPort(made->poles[r.node], p, port_kind(v.value.tmpl->category), port_layer(v.value.layer));
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
    if (edge.arow >= made->rows.size() || edge.brow >= made->rows.size()) {
      out.error = "bb2 topology: span row missing";
      return out;
    }
    for (std::size_t bundle_index = 0; bundle_index < made->bundles.size(); ++bundle_index) {
      EditResult<spec_view> v = view_for(state_, spec_.bundles[bundle_index]);
      if (!v.ok) {
        out.error = v.error;
        return out;
      }
      for (int lane = 0; lane < v.value.count; ++lane) {
        EditResult<ObjectId> span = state_.AddSpan(
            made->rows[edge.arow].ports[bundle_index][static_cast<std::size_t>(lane)],
            made->rows[edge.brow].ports[bundle_index][static_cast<std::size_t>(lane)], span_kind(v.value.tmpl->category),
            v.value.layer, made->bundles[bundle_index]);
        if (!span.ok) {
          out.error = span.error;
          return out;
        }
        add(*changes, span.change_set);
        made->spans.push_back(
            tspan{span.value, edge.id, bundle_index, static_cast<std::size_t>(lane)});
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

rules pipeline::make(const topo& made, const pairs& ps) const {
  rules out{};
  for (const tspan& span : made.spans) {
    const link& edge = ps.links[span.link];
    const trow& arow = made.rows[edge.arow];
    const trow& brow = made.rows[edge.brow];
    SpanLayoutRule rule{};
    rule.span_id = span.id;
    rule.flow_kind = BackboneFlowKind::kMain;
    rule.pass_mode = CurvePassMode::kPassThrough;
    auto endpoint = [&](ObjectId pole_id, ObjectId port_id) {
      EndpointLayoutRule e{};
      e.endpoint_node_id = pole_id;
      e.port_id = port_id;
      e.semantic.owner_pole_id = pole_id;
      e.flow_kind = BackboneFlowKind::kMain;
      e.origin = SupportLayoutOriginKind::kMainSupport;
      e.endpoint_source = SupportLayoutEndpointSourceKind::kPlainSupport;
      e.port_source = PortPlacementSourceKind::kGenerated;
      e.side = SlotSide::kCenter;
      e.endpoint_mode = CurveEndpointMode::kDirectThrough;
      e.same_level_feasible = true;
      return e;
    };
    rule.start = endpoint(arow.pole, arow.ports[span.bundle][span.lane]);
    rule.end = endpoint(brow.pole, brow.ports[span.bundle][span.lane]);
    out.data.spans.push_back(std::move(rule));
  }
  return out;
}

EditResult<layout> pipeline::make(const rules& made) const {
  EditResult<layout> out{};
  const EditState& edit = state_.view().edit_state();
  auto endpoint = [&](const EndpointLayoutRule& rule, SupportLayoutEndpoint* target) -> bool {
    const Port* port = edit.ports.find(rule.port_id);
    if (port == nullptr || target == nullptr) {
      return false;
    }
    static_cast<SupportLayoutSemanticDecision&>(*target) = rule.semantic;
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
    target->departure_dir = WorldForward();
    return true;
  };
  for (const SpanLayoutRule& rule : made.data.spans) {
    const Span* span = edit.spans.find(rule.span_id);
    if (span == nullptr) {
      out.error = "bb2 layout: span not found";
      return out;
    }
    SpanSupportLayoutEntry entry{};
    entry.span_id = rule.span_id;
    entry.flow_kind = rule.flow_kind;
    entry.pass_mode = rule.pass_mode;
    entry.variation_flow_key = rule.variation_flow_key;
    entry.lowering_kind = rule.lowering_kind;
    if (!endpoint(rule.start, &entry.start) || !endpoint(rule.end, &entry.end)) {
      out.error = "bb2 layout: endpoint port not found";
      return out;
    }
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
  for (const SpanSupportLayoutEntry& entry : made.entries) {
    DetailCurve detail = line(entry.start.endpoint_world, entry.end.endpoint_world);
    const std::vector<Vec3d>& pts = detail.sample_points;
    BoundsCacheEntry cached{};
    cached.whole = box(pts);
    if (pts.size() >= 2) {
      cached.segments.reserve(pts.size() - 1);
      for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        cached.segments.push_back(box(pts[i], pts[i + 1]));
      }
    }
    out.boxes.data.push_back({entry.span_id, std::move(cached)});
    out.curves.data.push_back({entry.span_id, std::move(detail)});
  }
  return out;
}

void pipeline::save(const rules& made) {
  state_.cache_span_rules(made.data);
}

void pipeline::save(const layout& made) {
  for (SpanSupportLayoutEntry entry : made.entries) {
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

EditResult<GenerateBundleFromPathResult> pipeline::build() {
  EditResult<GenerateBundleFromPathResult> out{};
  EditResult<pairs> ps = make(g_);
  if (!ps.ok) {
    out.error = ps.error;
    return out;
  }
  EditResult<topo> made = emit(ps.value);
  if (!made.ok) {
    out.error = made.error;
    return out;
  }
  rules saved = make(made.value, ps.value);
  save(saved);
  EditResult<layout> placed = make(saved);
  if (!placed.ok) {
    out.error = placed.error;
    return out;
  }
  save(placed.value);
  geom shaped = make(placed.value);
  save(std::move(shaped));
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
