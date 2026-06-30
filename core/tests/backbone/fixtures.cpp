#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_view.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace helpers;

namespace backbone_tests {

wire::core::BackboneSpec line_req(wire::core::CoreState& state) {
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 0.0;
  const std::vector<wire::core::PoleTypeId> types = sorted_pole_type_ids(state);
  req.pole_type_id = types.empty() ? wire::core::kInvalidPoleTypeId : types.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  return req;
}

wire::core::BackboneSpec poly3_req(wire::core::CoreState& state) {
  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {12.0, 8.0, 0.0}};
  return req;
}

int bundle_count(const wire::core::CoreState& state, wire::core::BundleKind id) {
  const auto it = state.view().bundle_templates().find(id);
  if (it == state.view().bundle_templates().end()) {
    return 0;
  }
  const wire::core::BundleTemplate& t = it->second;
  return (t.count_rule == wire::core::BundleCountRuleKind::kFixed) ? t.fixed_count : t.default_count;
}

int req_bundle_count(const wire::core::CoreState& state, const wire::core::BackboneSpec& req) {
  int total = 0;
  for (const wire::core::BackboneBundleSpec& bundle : req.bundles) {
    total += bundle_count(state, bundle.bundle_template_id);
  }
  return total;
}

int layer_rank(wire::core::SpanLayer layer) {
  switch (layer) {
  case wire::core::SpanLayer::kHighVoltage:
    return 2;
  case wire::core::SpanLayer::kDrop:
    return 0;
  case wire::core::SpanLayer::kLowVoltage:
  case wire::core::SpanLayer::kCommunication:
  case wire::core::SpanLayer::kOptical:
  case wire::core::SpanLayer::kUnknown:
  default:
    return 1;
  }
}

double band_height(const wire::core::CoreState& state, wire::core::PoleTypeId pole_type_id,
                   wire::core::BundleKind bundle_kind) {
  const auto type_it = state.view().pole_types().find(pole_type_id);
  const auto bundle_it = state.view().bundle_templates().find(bundle_kind);
  if (type_it == state.view().pole_types().end() || bundle_it == state.view().bundle_templates().end()) {
    return -1.0;
  }
  const wire::core::BundleTemplate& tmpl = bundle_it->second;
  const int target_layer = layer_rank(tmpl.default_layer);
  const wire::core::PortPlacementBand* best = nullptr;
  for (const wire::core::PortPlacementBand& band : type_it->second.port_bands) {
    if (!band.enabled || band.category != tmpl.category || band.layer != target_layer) {
      continue;
    }
    if (best == nullptr || band.priority > best->priority ||
        (band.priority == best->priority && band.band_id < best->band_id)) {
      best = &band;
    }
  }
  return best == nullptr ? -1.0 : best->height_center_m;
}

bool span_ports_match_z(const wire::core::CoreState& state, wire::core::ObjectId span_id, double expected_z) {
  const auto* span = state.view().spans().find(span_id);
  if (span == nullptr) {
    return false;
  }
  const auto* a = state.view().ports().find(span->port_a_id);
  const auto* b = state.view().ports().find(span->port_b_id);
  return a != nullptr && b != nullptr && almost_equal(a->world_position.z, expected_z, 1e-9) &&
         almost_equal(b->world_position.z, expected_z, 1e-9);
}

bool contains_text(const std::string& haystack, const std::string& needle) {
  return haystack.find(needle) != std::string::npos;
}

std::filesystem::path repo_root() {
  std::filesystem::path ledger = WIRE_TEST_SPEC_LEDGER_PATH;
  return ledger.parent_path().parent_path().parent_path();
}

bool file_text(const std::filesystem::path& path, std::string* out) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return false;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  *out = ss.str();
  return true;
}

std::vector<wire::core::Vec3d> backbone_layout_points(wire::core::CoreState& state) {
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  std::vector<wire::core::Vec3d> points{};
  if (!out.ok) {
    return points;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* layout = state.span_layout(span_id).entry;
    if (layout == nullptr) {
      return {};
    }
    points.push_back(layout->start.support_world);
    points.push_back(layout->start.endpoint_world);
    points.push_back(layout->end.support_world);
    points.push_back(layout->end.endpoint_world);
  }
  return points;
}

CurveSnapshot backbone_curve_points(wire::core::CoreState& state) {
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  CurveSnapshot snapshot{};
  if (!out.ok) {
    return snapshot;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* curve = state.find_curve_cache(span_id);
    if (curve == nullptr) {
      return {};
    }
    snapshot.lengths.push_back(curve->detail.total_length_m);
    for (const wire::core::Vec3d& p : curve->detail.sample_points) {
      snapshot.points.push_back(p);
    }
  }
  return snapshot;
}

void push_box(const wire::core::AABBd& box, BoundsSnapshot* out) {
  out->pts.push_back(box.min);
  out->pts.push_back(box.max);
}

BoundsSnapshot backbone_bounds_points(wire::core::CoreState& state) {
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  BoundsSnapshot snapshot{};
  if (!out.ok) {
    return snapshot;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* bounds = state.find_bounds_cache(span_id);
    if (bounds == nullptr) {
      return {};
    }
    push_box(bounds->whole, &snapshot);
    for (const wire::core::AABBd& segment : bounds->segments) {
      push_box(segment, &snapshot);
    }
  }
  return snapshot;
}

std::vector<wire::core::Vec3d> poly3_points(wire::core::CoreState& state) {
  wire::core::BackboneSpec req = poly3_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  std::vector<wire::core::Vec3d> points{};
  if (!out.ok) {
    return points;
  }
  for (wire::core::ObjectId pole_id : out.value.generated_pole_ids) {
    const auto* pole = state.view().poles().find(pole_id);
    if (pole == nullptr) {
      return {};
    }
    points.push_back(pole->world_transform.position);
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* curve = state.find_curve_cache(span_id);
    if (curve == nullptr) {
      return {};
    }
    for (const wire::core::Vec3d& p : curve->detail.sample_points) {
      points.push_back(p);
    }
  }
  return points;
}

std::vector<wire::core::Vec3d> span_curve_points(wire::core::CoreState& state,
                                                  const std::vector<wire::core::ObjectId>& spans) {
  std::vector<wire::core::Vec3d> out{};
  for (wire::core::ObjectId span_id : spans) {
    const auto* curve = state.find_curve_cache(span_id);
    if (curve == nullptr) {
      return {};
    }
    for (const wire::core::Vec3d& p : curve->detail.sample_points) {
      out.push_back(p);
    }
  }
  return out;
}

std::vector<wire::core::Vec3d> existing_sequence_points(wire::core::CoreState& state) {
  wire::core::BackboneSpec first = line_req(state);
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok || first_out.value.generated_pole_ids.empty()) {
    return {};
  }
  const wire::core::ObjectId existing = first_out.value.generated_pole_ids.front();
  const auto* pole = state.view().poles().find(existing);
  if (pole == nullptr) {
    return {};
  }
  wire::core::BackboneSpec second = line_req(state);
  second.path.polyline = {pole->world_transform.position, {0.0, 10.0, 0.0}, {4.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = existing;
  second.path.node_specs.push_back(node);
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (!second_out.ok) {
    return {};
  }
  std::vector<wire::core::Vec3d> out{};
  for (wire::core::ObjectId pole_id : second_out.value.generated_pole_ids) {
    const auto* generated = state.view().poles().find(pole_id);
    if (generated == nullptr) {
      return {};
    }
    out.push_back(generated->world_transform.position);
  }
  const std::vector<wire::core::Vec3d> curves = span_curve_points(state, second_out.value.generated_span_ids);
  out.insert(out.end(), curves.begin(), curves.end());
  return out;
}

std::vector<wire::core::Vec3d> pole_positions_for(wire::core::CoreState& state, const wire::core::BackboneSpec& req) {
  const auto out = state.GenerateFromBackboneSpec(req);
  std::vector<wire::core::Vec3d> pts{};
  if (!out.ok) {
    return pts;
  }
  for (wire::core::ObjectId pole_id : out.value.generated_pole_ids) {
    const auto* pole = state.view().poles().find(pole_id);
    if (pole == nullptr) {
      return {};
    }
    pts.push_back(pole->world_transform.position);
  }
  return pts;
}

std::vector<wire::core::Vec3d> offset_curve_points(double offset) {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.constraints.lateral_offset_m = offset;
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok) {
    return {};
  }
  return span_curve_points(state, out.value.generated_span_ids);
}

std::vector<wire::core::Vec3d> offset_points(double offset) {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.constraints.lateral_offset_m = offset;
  const auto out = state.GenerateFromBackboneSpec(req);
  std::vector<wire::core::Vec3d> pts{};
  if (!out.ok) {
    return pts;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return {};
    }
    const auto* a = state.view().ports().find(span->port_a_id);
    const auto* b = state.view().ports().find(span->port_b_id);
    const auto* curve = state.find_curve_cache(span_id);
    const auto* bounds = state.find_bounds_cache(span_id);
    if (a == nullptr || b == nullptr || curve == nullptr || bounds == nullptr) {
      return {};
    }
    pts.push_back(a->world_position);
    pts.push_back(b->world_position);
    for (const wire::core::Vec3d& p : curve->detail.sample_points) {
      pts.push_back(p);
    }
    pts.push_back(bounds->whole.min);
    pts.push_back(bounds->whole.max);
  }
  return pts;
}

std::vector<wire::core::Vec3d> node_mode_points(bool with_mode) {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  if (with_mode) {
    wire::core::BackboneSpec::NodeBundleModeSpec mode{};
    mode.point_index = 0;
    mode.bundle_template_id = wire::core::BundleKind::kLowVoltage;
    mode.mode = wire::core::BundleNodeMode::kNotPresent;
    req.node_bundle_modes.push_back(mode);
  }
  const auto out = state.GenerateFromBackboneSpec(req);
  std::vector<wire::core::Vec3d> pts{};
  if (!out.ok) {
    return pts;
  }
  pts.push_back({static_cast<double>(out.value.generated_span_ids.size()), 0.0, 0.0});
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    const auto* curve = state.find_curve_cache(span_id);
    const auto* bounds = state.find_bounds_cache(span_id);
    if (span == nullptr || curve == nullptr || bounds == nullptr) {
      return {};
    }
    const auto* a = state.view().ports().find(span->port_a_id);
    const auto* b = state.view().ports().find(span->port_b_id);
    if (a == nullptr || b == nullptr) {
      return {};
    }
    pts.push_back(a->world_position);
    pts.push_back(b->world_position);
    for (const wire::core::Vec3d& p : curve->detail.sample_points) {
      pts.push_back(p);
    }
    pts.push_back(bounds->whole.min);
    pts.push_back(bounds->whole.max);
  }
  return pts;
}

wire::core::BackboneInputSpec::NodeSpec pole_spec(std::size_t point_index, wire::core::ObjectId pole_id) {
  wire::core::BackboneInputSpec::NodeSpec spec{};
  spec.point_index = point_index;
  spec.support_kind = wire::core::SupportKind::kPole;
  spec.node_id = pole_id;
  return spec;
}

double dist2(const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  const wire::core::Vec3d d = a - b;
  return d.x * d.x + d.y * d.y + d.z * d.z;
}

std::vector<wire::core::Vec3d> pole_port_positions(const wire::core::CoreState& state, wire::core::ObjectId pole_id) {
  std::vector<wire::core::Vec3d> out{};
  for (const wire::core::Port& port : state.view().ports().items()) {
    if (port.owner_pole_id == pole_id) {
      out.push_back(port.world_position);
    }
  }
  std::sort(out.begin(), out.end(), [](const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
    if (!almost_equal(a.x, b.x, 1e-9)) {
      return a.x < b.x;
    }
    if (!almost_equal(a.y, b.y, 1e-9)) {
      return a.y < b.y;
    }
    return a.z < b.z;
  });
  return out;
}

std::vector<wire::core::Vec3d> generated_ports_on_pole(const wire::core::CoreState& state,
                                                       const std::vector<wire::core::ObjectId>& spans,
                                                       wire::core::ObjectId pole_id) {
  std::vector<wire::core::ObjectId> ids{};
  for (wire::core::ObjectId span_id : spans) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      continue;
    }
    for (wire::core::ObjectId port_id : {span->port_a_id, span->port_b_id}) {
      const wire::core::Port* port = state.view().ports().find(port_id);
      if (port != nullptr && port->owner_pole_id == pole_id && !contains_id(ids, port->id)) {
        ids.push_back(port->id);
      }
    }
  }
  std::vector<wire::core::Vec3d> out{};
  for (wire::core::ObjectId id : ids) {
    if (const wire::core::Port* port = state.view().ports().find(id)) {
      out.push_back(port->world_position);
    }
  }
  std::sort(out.begin(), out.end(), [](const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
    if (!almost_equal(a.x, b.x, 1e-9)) {
      return a.x < b.x;
    }
    if (!almost_equal(a.y, b.y, 1e-9)) {
      return a.y < b.y;
    }
    return a.z < b.z;
  });
  return out;
}

bool separated_from(const std::vector<wire::core::Vec3d>& existing, const std::vector<wire::core::Vec3d>& placed) {
  if (existing.empty() || placed.empty()) {
    return false;
  }
  for (const wire::core::Vec3d& p : placed) {
    bool saw_separation = false;
    for (const wire::core::Vec3d& q : existing) {
      if (dist2(p, q) > 1e-4) {
        saw_separation = true;
      } else {
        return false;
      }
    }
    if (!saw_separation) {
      return false;
    }
  }
  return true;
}

std::vector<wire::core::Vec3d> branch_separation_points() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return {};
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return {};
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok) {
    return {};
  }
  std::vector<wire::core::Vec3d> out = generated_ports_on_pole(state, second.value.generated_span_ids, b);
  const std::vector<wire::core::Vec3d> curves = span_curve_points(state, second.value.generated_span_ids);
  out.insert(out.end(), curves.begin(), curves.end());
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::BoundsCacheEntry* bounds = state.find_bounds_cache(span_id);
    if (bounds != nullptr) {
      out.push_back(bounds->whole.min);
      out.push_back(bounds->whole.max);
    }
  }
  return out;
}

wire::core::BackboneSpec pass_branch_req(wire::core::CoreState& state, wire::core::ObjectId pole_id,
                                         const wire::core::Vec3d& pole_pos) {
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_pos, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, pole_id)};
  wire::core::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = 0;
  mode.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  mode.mode = wire::core::BundleNodeMode::kPassThrough;
  branch.node_bundle_modes = {mode};
  return branch;
}

wire::core::BackboneSpec pass_poly3_req(wire::core::CoreState& state) {
  wire::core::BackboneSpec req = poly3_req(state);
  wire::core::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = 1;
  mode.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  mode.mode = wire::core::BundleNodeMode::kPassThrough;
  req.node_bundle_modes = {mode};
  return req;
}

std::vector<wire::core::Vec3d> pass_intent_points() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return {};
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return {};
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok) {
    return {};
  }
  std::vector<wire::core::Vec3d> out = generated_ports_on_pole(state, second.value.generated_span_ids, b);
  const std::vector<wire::core::Vec3d> curves = span_curve_points(state, second.value.generated_span_ids);
  out.insert(out.end(), curves.begin(), curves.end());
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::SpanLayoutRulesView rules = state.span_layout_rules(span_id);
    if (rules.has_rule()) {
      out.push_back({rules.rule->start.default_lower_required ? 1.0 : 0.0,
                     rules.rule->end.default_lower_required ? 1.0 : 0.0,
                     static_cast<double>(static_cast<int>(rules.rule->lowering_kind))});
    }
    const wire::core::BoundsCacheEntry* bounds = state.find_bounds_cache(span_id);
    if (bounds != nullptr) {
      out.push_back(bounds->whole.min);
      out.push_back(bounds->whole.max);
    }
  }
  return out;
}

bool span_has_lowered_endpoint(const wire::core::CoreState& state, wire::core::ObjectId span_id) {
  const wire::core::Span* span = state.view().spans().find(span_id);
  const wire::core::SpanLayoutView layout = state.span_layout(span_id);
  const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  const wire::core::BoundsCacheEntry* bounds = state.find_bounds_cache(span_id);
  if (span == nullptr || !layout.has_layout() || curve == nullptr || bounds == nullptr) {
    return false;
  }
  const wire::core::Port* a = state.view().ports().find(span->port_a_id);
  const wire::core::Port* b = state.view().ports().find(span->port_b_id);
  if (a == nullptr || b == nullptr) {
    return false;
  }
  bool lowered = false;
  if (layout.entry->start.default_lower_required) {
    lowered = lowered || layout.entry->start.endpoint_world.z < a->world_position.z - 0.1;
  }
  if (layout.entry->end.default_lower_required) {
    lowered = lowered || layout.entry->end.endpoint_world.z < b->world_position.z - 0.1;
  }
  if (!lowered || curve->detail.sample_points.empty()) {
    return false;
  }
  double min_sample_z = curve->detail.sample_points.front().z;
  for (const wire::core::Vec3d& point : curve->detail.sample_points) {
    min_sample_z = std::min(min_sample_z, point.z);
  }
  return min_sample_z < std::min(a->world_position.z, b->world_position.z) - 0.1 &&
         bounds->whole.min.z <= min_sample_z + 1e-9;
}

std::vector<wire::core::Vec3d> junction_v1_points() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return {};
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return {};
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok) {
    return {};
  }
  std::vector<wire::core::Vec3d> out = pass_intent_points();
  const wire::core::BackboneFrontier frontier = state.view().pole_frontier(b);
  out.push_back({static_cast<double>(frontier.edge_ids.size()), static_cast<double>(frontier.edge_bundle_ids.size()),
                 static_cast<double>(frontier.span_ids.size())});
  for (const wire::core::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    out.push_back({static_cast<double>(static_cast<int>(binding.bundle_template_id)),
                   static_cast<double>(static_cast<int>(binding.port_kind)),
                   static_cast<double>(static_cast<int>(binding.port_layer))});
  }
  return out;
}

} // namespace backbone_tests
