#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "city/wire/core_test_hook.hpp"
#include "city/wire/core_view.hpp"
#include "city/wire/coord_utils.hpp"

#include "../../src/generation/backbone/emit_shared.hpp"
#include "../../src/generation/backbone/model_assembly.hpp"
#include "../../src/generation/backbone/mount_graph.hpp"
#include "../../src/generation/backbone/row_representation.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace helpers;

namespace backbone_tests {

city::wire::BackboneSpec line_req(city::wire::CoreState& state) {
  city::wire::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 0.0;
  const std::vector<city::wire::PoleTypeId> types = sorted_pole_type_ids(state);
  req.pole_type_id = types.empty() ? city::wire::kInvalidPoleTypeId : types.front();
  add_backbone_bundle(req, city::wire::BundleKind::kLowVoltage);
  return req;
}

city::wire::BackboneSpec poly3_req(city::wire::CoreState& state) {
  city::wire::BackboneSpec req = line_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {12.0, 8.0, 0.0}};
  return req;
}

city::wire::ObjectId span_for_bundle(const city::wire::CoreState& state,
                                     const std::vector<city::wire::ObjectId>& span_ids,
                                     city::wire::BundleKind bundle_template_id) {
  const city::wire::BundleTemplateId template_id = city::wire::DefaultBundleTemplateId(bundle_template_id);
  for (city::wire::ObjectId span_id : span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      continue;
    }
    const auto* bundle = state.view().bundles().find(span->bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == template_id) {
      return span_id;
    }
  }
  return city::wire::kInvalidObjectId;
}

int bundle_count(const city::wire::CoreState& state, city::wire::BundleKind id) {
  const auto it = state.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(id));
  if (it == state.view().bundle_templates().end()) {
    return 0;
  }
  const city::wire::BundleTemplate& t = it->second;
  return (t.count_rule == city::wire::BundleCountRuleKind::kFixed) ? t.fixed_count : t.default_count;
}

int req_bundle_count(const city::wire::CoreState& state, const city::wire::BackboneSpec& req) {
  int total = 0;
  for (const city::wire::BackboneBundleSpec& bundle : req.bundles) {
    const auto it = state.view().bundle_templates().find(bundle.bundle_template_id);
    if (it != state.view().bundle_templates().end()) {
      const city::wire::BundleTemplate& t = it->second;
      total += (t.count_rule == city::wire::BundleCountRuleKind::kFixed) ? t.fixed_count : t.default_count;
    }
  }
  return total;
}

int layer_rank(city::wire::SpanLayer layer) {
  switch (layer) {
  case city::wire::SpanLayer::kHighVoltage:
    return 2;
  case city::wire::SpanLayer::kDrop:
    return 0;
  case city::wire::SpanLayer::kLowVoltage:
  case city::wire::SpanLayer::kCommunication:
  case city::wire::SpanLayer::kOptical:
  case city::wire::SpanLayer::kUnknown:
  default:
    return 1;
  }
}

double band_height(const city::wire::CoreState& state, city::wire::PoleTypeId pole_type_id,
                   city::wire::BundleKind bundle_kind) {
  const auto type_it = state.view().pole_types().find(pole_type_id);
  const auto bundle_it = state.view().bundle_templates().find(city::wire::DefaultBundleTemplateId(bundle_kind));
  if (type_it == state.view().pole_types().end() || bundle_it == state.view().bundle_templates().end()) {
    return -1.0;
  }
  const city::wire::BundleTemplate& tmpl = bundle_it->second;
  const int target_layer = layer_rank(tmpl.default_layer);
  const city::wire::PortPlacementBand* best = nullptr;
  for (const city::wire::PortPlacementBand& band : type_it->second.port_bands) {
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

bool span_ports_match_z(const city::wire::CoreState& state, city::wire::ObjectId span_id, double expected_z) {
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
  return ledger.parent_path().parent_path().parent_path().parent_path();
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

bool function_body(const std::string& source, const std::string& signature, std::string* out) {
  if (out == nullptr) {
    return false;
  }
  const std::size_t signature_pos = source.find(signature);
  const std::size_t body_begin = source.find('{', signature_pos);
  if (signature_pos == std::string::npos || body_begin == std::string::npos) {
    return false;
  }
  std::size_t depth = 0;
  for (std::size_t i = body_begin; i < source.size(); ++i) {
    if (source[i] == '{') {
      ++depth;
    } else if (source[i] == '}') {
      if (--depth == 0) {
        *out = source.substr(signature_pos, i - signature_pos + 1);
        return true;
      }
    }
  }
  return false;
}

std::vector<city::wire::Vec3d> backbone_layout_points(city::wire::CoreState& state) {
  city::wire::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  std::vector<city::wire::Vec3d> points{};
  if (!out.ok) {
    return points;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
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

CurveSnapshot backbone_curve_points(city::wire::CoreState& state) {
  city::wire::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  CurveSnapshot snapshot{};
  if (!out.ok) {
    return snapshot;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    const auto* curve = state.find_curve_cache(span_id);
    if (curve == nullptr) {
      return {};
    }
    snapshot.lengths.push_back(curve->detail.total_length_m);
    for (const city::wire::Vec3d& p : curve->detail.sample_points) {
      snapshot.points.push_back(p);
    }
  }
  return snapshot;
}

void push_box(const city::wire::AABBd& box, BoundsSnapshot* out) {
  out->pts.push_back(box.min);
  out->pts.push_back(box.max);
}

BoundsSnapshot backbone_bounds_points(city::wire::CoreState& state) {
  city::wire::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  BoundsSnapshot snapshot{};
  if (!out.ok) {
    return snapshot;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    const auto* bounds = state.find_bounds_cache(span_id);
    if (bounds == nullptr) {
      return {};
    }
    push_box(bounds->whole, &snapshot);
    for (const city::wire::AABBd& segment : bounds->segments) {
      push_box(segment, &snapshot);
    }
  }
  return snapshot;
}

std::vector<city::wire::Vec3d> poly3_points(city::wire::CoreState& state) {
  city::wire::BackboneSpec req = poly3_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  std::vector<city::wire::Vec3d> points{};
  if (!out.ok) {
    return points;
  }
  for (city::wire::ObjectId pole_id : out.value.generated_pole_ids) {
    const auto* pole = state.view().poles().find(pole_id);
    if (pole == nullptr) {
      return {};
    }
    points.push_back(pole->world_transform.position);
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    const auto* curve = state.find_curve_cache(span_id);
    if (curve == nullptr) {
      return {};
    }
    for (const city::wire::Vec3d& p : curve->detail.sample_points) {
      points.push_back(p);
    }
  }
  return points;
}

std::vector<city::wire::Vec3d> span_curve_points(city::wire::CoreState& state,
                                                  const std::vector<city::wire::ObjectId>& spans) {
  std::vector<city::wire::Vec3d> out{};
  for (city::wire::ObjectId span_id : spans) {
    const auto* curve = state.find_curve_cache(span_id);
    if (curve == nullptr) {
      return {};
    }
    for (const city::wire::Vec3d& p : curve->detail.sample_points) {
      out.push_back(p);
    }
  }
  return out;
}

std::vector<city::wire::Vec3d> existing_sequence_points(city::wire::CoreState& state) {
  city::wire::BackboneSpec first = line_req(state);
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok || first_out.value.generated_pole_ids.empty()) {
    return {};
  }
  const city::wire::ObjectId existing = first_out.value.generated_pole_ids.front();
  const auto* pole = state.view().poles().find(existing);
  if (pole == nullptr) {
    return {};
  }
  city::wire::BackboneSpec second = line_req(state);
  second.path.polyline = {pole->world_transform.position, {0.0, 10.0, 0.0}, {4.0, 10.0, 0.0}};
  city::wire::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = city::wire::SupportKind::kPole;
  node.node_id = existing;
  second.path.node_specs.push_back(node);
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (!second_out.ok) {
    return {};
  }
  std::vector<city::wire::Vec3d> out{};
  for (city::wire::ObjectId pole_id : second_out.value.generated_pole_ids) {
    const auto* generated = state.view().poles().find(pole_id);
    if (generated == nullptr) {
      return {};
    }
    out.push_back(generated->world_transform.position);
  }
  const std::vector<city::wire::Vec3d> curves = span_curve_points(state, second_out.value.generated_span_ids);
  out.insert(out.end(), curves.begin(), curves.end());
  return out;
}

std::vector<city::wire::Vec3d> pole_positions_for(city::wire::CoreState& state, const city::wire::BackboneSpec& req) {
  const auto out = state.GenerateFromBackboneSpec(req);
  std::vector<city::wire::Vec3d> pts{};
  if (!out.ok) {
    return pts;
  }
  for (city::wire::ObjectId pole_id : out.value.generated_pole_ids) {
    const auto* pole = state.view().poles().find(pole_id);
    if (pole == nullptr) {
      return {};
    }
    pts.push_back(pole->world_transform.position);
  }
  return pts;
}

std::vector<city::wire::Vec3d> offset_curve_points(double offset) {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  req.constraints.lateral_offset_m = offset;
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok) {
    return {};
  }
  return span_curve_points(state, out.value.generated_span_ids);
}

std::vector<city::wire::Vec3d> offset_points(double offset) {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  req.constraints.lateral_offset_m = offset;
  const auto out = state.GenerateFromBackboneSpec(req);
  std::vector<city::wire::Vec3d> pts{};
  if (!out.ok) {
    return pts;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
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
    for (const city::wire::Vec3d& p : curve->detail.sample_points) {
      pts.push_back(p);
    }
    pts.push_back(bounds->whole.min);
    pts.push_back(bounds->whole.max);
  }
  return pts;
}

std::vector<city::wire::Vec3d> node_mode_points(bool with_mode) {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  if (with_mode) {
    city::wire::BackboneSpec::NodeBundleModeSpec mode{};
    mode.point_index = 0;
    mode.bundle_template_id = city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage);
    mode.mode = city::wire::BundleNodeMode::kNotPresent;
    req.node_bundle_modes.push_back(mode);
  }
  const auto out = state.GenerateFromBackboneSpec(req);
  std::vector<city::wire::Vec3d> pts{};
  if (!out.ok) {
    return pts;
  }
  pts.push_back({static_cast<double>(out.value.generated_span_ids.size()), 0.0, 0.0});
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
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
    for (const city::wire::Vec3d& p : curve->detail.sample_points) {
      pts.push_back(p);
    }
    pts.push_back(bounds->whole.min);
    pts.push_back(bounds->whole.max);
  }
  return pts;
}

city::wire::BackboneInputSpec::NodeSpec pole_spec(std::size_t point_index, city::wire::ObjectId pole_id) {
  city::wire::BackboneInputSpec::NodeSpec spec{};
  spec.point_index = point_index;
  spec.support_kind = city::wire::SupportKind::kPole;
  spec.node_id = pole_id;
  return spec;
}

double dist2(const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
  const city::wire::Vec3d d = a - b;
  return d.x * d.x + d.y * d.y + d.z * d.z;
}

std::vector<city::wire::Vec3d> pole_port_positions(const city::wire::CoreState& state, city::wire::ObjectId pole_id) {
  std::vector<city::wire::Vec3d> out{};
  for (const city::wire::Port& port : state.view().ports().items()) {
    if (port.owner_pole_id == pole_id) {
      out.push_back(port.world_position);
    }
  }
  std::sort(out.begin(), out.end(), [](const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
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

std::vector<city::wire::Vec3d> generated_ports_on_pole(const city::wire::CoreState& state,
                                                       const std::vector<city::wire::ObjectId>& spans,
                                                       city::wire::ObjectId pole_id) {
  std::vector<city::wire::ObjectId> ids{};
  for (city::wire::ObjectId span_id : spans) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      continue;
    }
    for (city::wire::ObjectId port_id : {span->port_a_id, span->port_b_id}) {
      const city::wire::Port* port = state.view().ports().find(port_id);
      if (port != nullptr && port->owner_pole_id == pole_id && !contains_id(ids, port->id)) {
        ids.push_back(port->id);
      }
    }
  }
  std::vector<city::wire::Vec3d> out{};
  for (city::wire::ObjectId id : ids) {
    if (const city::wire::Port* port = state.view().ports().find(id)) {
      out.push_back(port->world_position);
    }
  }
  std::sort(out.begin(), out.end(), [](const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
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

bool separated_from(const std::vector<city::wire::Vec3d>& existing, const std::vector<city::wire::Vec3d>& placed) {
  if (existing.empty() || placed.empty()) {
    return false;
  }
  for (const city::wire::Vec3d& p : placed) {
    bool saw_separation = false;
    for (const city::wire::Vec3d& q : existing) {
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

std::vector<city::wire::Vec3d> branch_separation_points() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return {};
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return {};
  }
  city::wire::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok) {
    return {};
  }
  std::vector<city::wire::Vec3d> out = generated_ports_on_pole(state, second.value.generated_span_ids, b);
  const std::vector<city::wire::Vec3d> curves = span_curve_points(state, second.value.generated_span_ids);
  out.insert(out.end(), curves.begin(), curves.end());
  for (city::wire::ObjectId span_id : second.value.generated_span_ids) {
    const city::wire::BoundsCacheEntry* bounds = state.find_bounds_cache(span_id);
    if (bounds != nullptr) {
      out.push_back(bounds->whole.min);
      out.push_back(bounds->whole.max);
    }
  }
  return out;
}

city::wire::BackboneSpec pass_branch_req(city::wire::CoreState& state, city::wire::ObjectId pole_id,
                                         const city::wire::Vec3d& pole_pos) {
  city::wire::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_pos, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, pole_id)};
  city::wire::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = 0;
  mode.bundle_template_id = city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage);
  mode.mode = city::wire::BundleNodeMode::kPassThrough;
  branch.node_bundle_modes = {mode};
  return branch;
}

city::wire::BackboneSpec hv_poly3_req(city::wire::CoreState& state) {
  city::wire::BackboneSpec req = poly3_req(state);
  req.bundles.clear();
  add_backbone_bundle(req, city::wire::BundleKind::kHighVoltage);
  return req;
}

city::wire::BackboneSpec hv_branch_req(city::wire::CoreState& state, city::wire::ObjectId pole_id,
                                       const city::wire::Vec3d& pole_pos) {
  city::wire::BackboneSpec branch = pass_branch_req(state, pole_id, pole_pos);
  branch.bundles.clear();
  add_backbone_bundle(branch, city::wire::BundleKind::kHighVoltage);
  branch.node_bundle_modes.front().bundle_template_id = city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage);
  return branch;
}

std::vector<city::wire::ObjectId> lowering_branch_spans(city::wire::CoreState& state) {
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return {};
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return {};
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
  return second.ok ? second.value.generated_span_ids : std::vector<city::wire::ObjectId>{};
}

city::wire::BackboneSpec pass_poly3_req(city::wire::CoreState& state) {
  city::wire::BackboneSpec req = poly3_req(state);
  city::wire::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = 1;
  mode.bundle_template_id = city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage);
  mode.mode = city::wire::BundleNodeMode::kPassThrough;
  req.node_bundle_modes = {mode};
  return req;
}

std::vector<city::wire::Vec3d> pass_intent_points() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return {};
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return {};
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok) {
    return {};
  }
  std::vector<city::wire::Vec3d> out = generated_ports_on_pole(state, second.value.generated_span_ids, b);
  const std::vector<city::wire::Vec3d> curves = span_curve_points(state, second.value.generated_span_ids);
  out.insert(out.end(), curves.begin(), curves.end());
  for (city::wire::ObjectId span_id : second.value.generated_span_ids) {
    const city::wire::SpanLayoutRulesView rules = state.span_layout_rules(span_id);
    if (rules.has_rule()) {
      out.push_back({rules.rule->start.default_lower_required ? 1.0 : 0.0,
                     rules.rule->end.default_lower_required ? 1.0 : 0.0,
                     static_cast<double>(static_cast<int>(rules.rule->lowering_kind))});
    }
    const city::wire::BoundsCacheEntry* bounds = state.find_bounds_cache(span_id);
    if (bounds != nullptr) {
      out.push_back(bounds->whole.min);
      out.push_back(bounds->whole.max);
    }
  }
  return out;
}

bool span_has_lowered_endpoint(const city::wire::CoreState& state, city::wire::ObjectId span_id) {
  const city::wire::Span* span = state.view().spans().find(span_id);
  const city::wire::SpanLayoutView layout = state.span_layout(span_id);
  const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  const city::wire::BoundsCacheEntry* bounds = state.find_bounds_cache(span_id);
  if (span == nullptr || !layout.has_layout() || curve == nullptr || bounds == nullptr) {
    return false;
  }
  const city::wire::Port* a = state.view().ports().find(span->port_a_id);
  const city::wire::Port* b = state.view().ports().find(span->port_b_id);
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
  for (const city::wire::Vec3d& point : curve->detail.sample_points) {
    min_sample_z = std::min(min_sample_z, point.z);
  }
  return min_sample_z < std::min(a->world_position.z, b->world_position.z) - 0.1 &&
         bounds->whole.min.z <= min_sample_z + 1e-9;
}

std::vector<city::wire::Vec3d> junction_v1_points() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return {};
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return {};
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok) {
    return {};
  }
  std::vector<city::wire::Vec3d> out = pass_intent_points();
  const city::wire::BackboneFrontier frontier = state.view().pole_frontier(b);
  out.push_back({static_cast<double>(frontier.edge_ids.size()), static_cast<double>(frontier.edge_bundle_ids.size()),
                 static_cast<double>(frontier.span_ids.size())});
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    out.push_back({static_cast<double>(static_cast<int>(binding.bundle_template_id)),
                   static_cast<double>(static_cast<int>(binding.port_kind)),
                   static_cast<double>(static_cast<int>(binding.port_layer))});
  }
  return out;
}

std::vector<SpanOutputSnapshot> snapshot_span_outputs(const city::wire::CoreState& state,
                                                       const std::vector<city::wire::ObjectId>& span_ids) {
  std::vector<SpanOutputSnapshot> out{};
  out.reserve(span_ids.size());
  for (city::wire::ObjectId span_id : span_ids) {
    const city::wire::SpanLayoutView layout = state.span_layout(span_id);
    const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span_id);
    const city::wire::BoundsCacheEntry* bounds = state.find_bounds_cache(span_id);
    const city::wire::SpanRuntimeState* runtime = state.view().find_span_runtime_state(span_id);
    if (!layout.has_layout() || curve == nullptr || bounds == nullptr || runtime == nullptr) {
      return {};
    }
    out.push_back({span_id,
                   runtime->data_version,
                   layout.entry->source_version,
                   layout.entry->start.support_world,
                   layout.entry->start.endpoint_world,
                   layout.entry->end.support_world,
                   layout.entry->end.endpoint_world,
                   curve->source_version,
                   curve->detail.sample_points,
                   bounds->source_version,
                   bounds->whole});
  }
  return out;
}

bool same_span_output_snapshots(const std::vector<SpanOutputSnapshot>& before,
                                const city::wire::CoreState& state) {
  for (const SpanOutputSnapshot& expected : before) {
    const city::wire::SpanLayoutView layout = state.span_layout(expected.span_id);
    const city::wire::CurveCacheEntry* curve = state.find_curve_cache(expected.span_id);
    const city::wire::BoundsCacheEntry* bounds = state.find_bounds_cache(expected.span_id);
    const city::wire::SpanRuntimeState* runtime = state.view().find_span_runtime_state(expected.span_id);
    if (!layout.has_layout() || curve == nullptr || bounds == nullptr || runtime == nullptr ||
        runtime->data_version != expected.data_version || layout.entry->source_version != expected.layout_source_version ||
        !almost_equal(layout.entry->start.support_world, expected.layout_start_support, 1e-12) ||
        !almost_equal(layout.entry->start.endpoint_world, expected.layout_start_endpoint, 1e-12) ||
        !almost_equal(layout.entry->end.support_world, expected.layout_end_support, 1e-12) ||
        !almost_equal(layout.entry->end.endpoint_world, expected.layout_end_endpoint, 1e-12) ||
        curve->source_version != expected.curve_source_version || bounds->source_version != expected.bounds_source_version ||
        !almost_equal(bounds->whole.min, expected.bounds.min, 1e-12) ||
        !almost_equal(bounds->whole.max, expected.bounds.max, 1e-12) ||
        curve->detail.sample_points.size() != expected.curve_samples.size()) {
      return false;
    }
    for (std::size_t i = 0; i < expected.curve_samples.size(); ++i) {
      if (!almost_equal(curve->detail.sample_points[i], expected.curve_samples[i], 1e-12)) {
        return false;
      }
    }
  }
  return true;
}

bool backbone_common_invariants_pass(const city::wire::CoreState& state, std::string* reason) {
  const auto fail = [&](const std::string& message) {
    if (reason != nullptr) {
      *reason = message;
    }
    return false;
  };
  const auto finite_vec = [](const city::wire::Vec3d& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
  };
  const auto vec_text = [](const city::wire::Vec3d& value) {
    return "(" + std::to_string(value.x) + "," + std::to_string(value.y) +
           "," + std::to_string(value.z) + ")";
  };
  const auto transform_equal = [](const city::wire::Transformd& a,
                                  const city::wire::Transformd& b) {
    return a.position.x == b.position.x &&
           a.position.y == b.position.y &&
           a.position.z == b.position.z &&
           a.rotation_euler_deg.x == b.rotation_euler_deg.x &&
           a.rotation_euler_deg.y == b.rotation_euler_deg.y &&
           a.rotation_euler_deg.z == b.rotation_euler_deg.z &&
           a.scale.x == b.scale.x && a.scale.y == b.scale.y &&
           a.scale.z == b.scale.z;
  };
  const auto transform_close = [&](const city::wire::Transformd& a,
                                   const city::wire::Transformd& b) {
    return almost_equal(a.position, b.position, 1e-9) &&
           almost_equal(a.rotation_euler_deg, b.rotation_euler_deg, 1e-9) &&
           almost_equal(a.scale, b.scale, 1e-9);
  };
  const auto transform_text = [](const city::wire::Transformd& value) {
    std::ostringstream out;
    out << std::hexfloat << "p(" << value.position.x << ","
        << value.position.y << "," << value.position.z << ") r("
        << value.rotation_euler_deg.x << ","
        << value.rotation_euler_deg.y << ","
        << value.rotation_euler_deg.z << ") s(" << value.scale.x << ","
        << value.scale.y << "," << value.scale.z << ")";
    return out.str();
  };
  const city::wire::SavedBackboneGraph& graph = state.view().backbone();
  const city::wire::CoreView view = state.view();
  const auto saved_node_exists = [&](city::wire::ObjectId node_id) {
    return std::any_of(graph.nodes.begin(), graph.nodes.end(), [&](const city::wire::SavedBackboneNode& node) {
      return node.node_id == node_id;
    });
  };
  const auto saved_edge_exists = [&](city::wire::ObjectId edge_id) {
    return std::any_of(graph.edges.begin(), graph.edges.end(), [&](const city::wire::SavedBackboneEdge& edge) {
      return edge.edge_id == edge_id;
    });
  };
  const auto edge_bundle_exists = [&](city::wire::ObjectId edge_bundle_id) {
    return std::any_of(graph.edge_bundles.begin(), graph.edge_bundles.end(),
                       [&](const city::wire::SavedBackboneEdgeBundle& edge_bundle) {
                         return edge_bundle.edge_bundle_id == edge_bundle_id;
                       });
  };
  const auto edge_bundle_has_lane = [&](city::wire::ObjectId edge_bundle_id, std::size_t lane_index) {
    return std::any_of(graph.span_bindings.begin(), graph.span_bindings.end(),
                       [&](const city::wire::SavedBackboneSpanBinding& binding) {
                         return binding.edge_bundle_id == edge_bundle_id && binding.lane_index == lane_index;
                       });
  };

  const auto placement_band = [&](const city::wire::Pole& pole,
                                  int placement_band_id) -> const city::wire::PortPlacementBand* {
    const auto type_it = view.pole_types().find(pole.pole_type_id);
    if (type_it == view.pole_types().end()) return nullptr;
    const auto band_it = std::find_if(
        type_it->second.port_bands.begin(), type_it->second.port_bands.end(),
        [&](const city::wire::PortPlacementBand& band) {
          return band.band_id == placement_band_id;
        });
    return band_it == type_it->second.port_bands.end() ? nullptr : &*band_it;
  };

  const auto resolved_fixture_plan =
      city::wire::generation::backbone::fixture_placement_plan_from_cache(state);
  if (!resolved_fixture_plan.ok) {
    return fail("row frame coherence fixture plan failed: " +
                resolved_fixture_plan.error);
  }
  const auto& fixture_plan = resolved_fixture_plan.value;

  for (const city::wire::SavedBackbonePortBinding& binding : graph.port_bindings) {
    const city::wire::SavedBackboneEdgeBundle* edge_bundle =
        view.backbone_edge_bundle(binding.edge_bundle_id);
    const city::wire::Bundle* bundle =
        edge_bundle == nullptr ? nullptr : view.bundles().find(edge_bundle->bundle_id);
    const city::wire::Port* port = view.ports().find(binding.port_id);
    const city::wire::Pole* pole =
        port == nullptr ? nullptr : view.poles().find(port->owner_pole_id);
    if (edge_bundle == nullptr || bundle == nullptr || port == nullptr) {
      return fail("row frame coherence binding " + std::to_string(binding.edge_bundle_id) +
                  " lane " + std::to_string(binding.lane_index) +
                  " is missing its edge bundle, bundle, or port " +
                  std::to_string(binding.port_id));
    }
    if (pole == nullptr) {
      continue;
    }

    const auto representation =
        city::wire::generation::backbone::DeriveEndpointRowRepresentation(state, binding);
    if (!representation.ok) {
      return fail("row frame coherence binding " + std::to_string(binding.edge_bundle_id) +
                  " port " + std::to_string(binding.port_id) +
                  " has no derived row representation: " + representation.error);
    }
    const double yaw_delta = city::wire::NormalizeYawDeg(
        binding.layout_yaw_deg - representation.value.layout_yaw_deg);
    if (representation.value.connected && !representation.value.sharp &&
        std::abs(yaw_delta) > 1e-9) {
      const city::wire::SavedBackboneEdgeBundle* peer_edge_bundle =
          view.backbone_edge_bundle(
              representation.value.peer_edge_bundle_id);
      return fail("row frame coherence binding " + std::to_string(binding.edge_bundle_id) +
                  " port " + std::to_string(binding.port_id) +
                  " pole " + std::to_string(pole->id) + " node " +
                  std::to_string(binding.row_key.node_id) + " edge " +
                  std::to_string(binding.row_key.edge_id) + " pair=" +
                  std::to_string(representation.value.row_key.edge_a) + ":" +
                  std::to_string(representation.value.row_key.edge_b) +
                  " peer_edge_bundle=" +
                  std::to_string(representation.value.peer_edge_bundle_id) +
                  " peer_edge=" +
                  std::to_string(peer_edge_bundle == nullptr
                                     ? city::wire::kInvalidObjectId
                                     : peer_edge_bundle->edge_id) +
                  " row_axis=" + vec_text(representation.value.row_axis) +
                  " yaw expected=" + std::to_string(representation.value.layout_yaw_deg) +
                  " actual=" + std::to_string(binding.layout_yaw_deg));
    }

    const city::wire::PortPlacementBand* band = placement_band(*pole, binding.placement_band_id);
    if (band == nullptr) {
      return fail("row frame coherence binding " + std::to_string(binding.edge_bundle_id) +
                  " port " + std::to_string(binding.port_id) +
                  " references missing placement band " +
                  std::to_string(binding.placement_band_id));
    }
    std::unordered_set<int> lane_band_ids{};
    for (const city::wire::SavedBackbonePortBinding& candidate : graph.port_bindings) {
      if (candidate.edge_bundle_id == binding.edge_bundle_id &&
          candidate.row_key.node_id == binding.row_key.node_id) {
        lane_band_ids.insert(candidate.placement_band_id);
      }
    }
    const bool uses_lane_bands = bundle->conductor_count > 1 &&
                                 lane_band_ids.size() ==
                                     static_cast<std::size_t>(bundle->conductor_count);
    const double expected_lateral =
        bundle->placement_explicit
            ? bundle->lateral_m +
                  city::wire::generation::backbone::LaneOffset(
                      binding.lane_index, bundle->conductor_count,
                      bundle->phase_spacing_m)
            : band->lateral_center_m +
                  (uses_lane_bands
                       ? 0.0
                       : city::wire::generation::backbone::LaneOffset(
                             binding.lane_index, bundle->conductor_count,
                             bundle->phase_spacing_m));
    const city::wire::PoleFrame frame =
        city::wire::BuildPoleFrame(pole->world_transform, binding.layout_yaw_deg);
    const city::wire::Vec3d local =
        city::wire::WorldPointToLocal(frame, port->world_position);
    if (std::abs(local.y - expected_lateral) > 1e-9) {
      const std::size_t port_binding_count = static_cast<std::size_t>(
          std::count_if(graph.port_bindings.begin(), graph.port_bindings.end(),
                        [&](const city::wire::SavedBackbonePortBinding& candidate) {
                          return candidate.port_id == binding.port_id;
                        }));
      return fail("row frame coherence binding " + std::to_string(binding.edge_bundle_id) +
                  " port " + std::to_string(binding.port_id) +
                  " pole " + std::to_string(pole->id) + " node " +
                  std::to_string(binding.row_key.node_id) + " edge " +
                  std::to_string(binding.row_key.edge_id) + " band " +
                  std::to_string(binding.placement_band_id) + " yaw=" +
                  std::to_string(binding.layout_yaw_deg) + " pole_origin=" +
                  vec_text(pole->world_transform.position) + " world=" +
                  vec_text(port->world_position) + " local=" + vec_text(local) +
                  " bindings=" +
                  std::to_string(port_binding_count) +
                  " lateral expected=" + std::to_string(expected_lateral) +
                  " actual=" + std::to_string(local.y));
    }
    const auto template_it = view.bundle_templates().find(binding.bundle_template_id);
    const bool height_is_direct_anchor =
        bundle->placement_explicit || template_it == view.bundle_templates().end() ||
        !template_it->second.enable_branch_down_offset;
    const double expected_height =
        bundle->placement_explicit ? bundle->height_m : band->height_center_m;
    if (height_is_direct_anchor && std::abs(local.z - expected_height) > 1e-9) {
      return fail("row frame coherence binding " + std::to_string(binding.edge_bundle_id) +
                  " port " + std::to_string(binding.port_id) +
                  " height expected=" + std::to_string(expected_height) +
                  " actual=" + std::to_string(local.z));
    }

    const bool binding_has_fixture =
        template_it != view.bundle_templates().end() &&
        (template_it->second.row_fixture_assembly_id !=
             city::wire::kInvalidModelAssemblyTemplateId ||
         template_it->second.endpoint_fixture_assembly_id !=
             city::wire::kInvalidModelAssemblyTemplateId);
    if (binding_has_fixture) {
      const auto plan_it = fixture_plan.find(binding.port_id);
      if (plan_it == fixture_plan.end()) {
        std::string cache_spans{};
        view.cache_state().span_layout_cache.for_each_layout_record(
            [&](city::wire::ObjectId span_id,
                const city::wire::SpanLayoutCacheRecord& record,
                const city::wire::SpanLayoutEntry&) {
              const city::wire::SpanLayoutRule* rule =
                  record.span_layout_rule();
              cache_spans += (cache_spans.empty() ? "" : ",") +
                             std::to_string(span_id);
              if (rule != nullptr) {
                cache_spans += "(" + std::to_string(rule->start.port_id) +
                               ":" + std::to_string(rule->end.port_id) + ")";
              }
            });
        return fail("row frame coherence binding " +
                    std::to_string(binding.edge_bundle_id) + " port " +
                    std::to_string(binding.port_id) +
                    " has no endpoint fixture plan; cache spans=" +
                    cache_spans);
      }
      const city::wire::Vec3d fixture_local = city::wire::WorldPointToLocal(
          frame, plan_it->second.endpoint_fixture_root.position);
      if (std::abs(fixture_local.y - expected_lateral) > 1e-9) {
        return fail("row frame coherence binding " +
                    std::to_string(binding.edge_bundle_id) + " port " +
                    std::to_string(binding.port_id) +
                    " endpoint fixture lateral expected=" +
                    std::to_string(expected_lateral) + " actual=" +
                    std::to_string(fixture_local.y));
      }
      if (plan_it->second.row_fixture.available) {
        const double row_yaw_delta = city::wire::NormalizeYawDeg(
            plan_it->second.row_fixture.root.rotation_euler_deg.z -
            binding.layout_yaw_deg);
        if (std::abs(row_yaw_delta) > 1e-9) {
          return fail("row frame coherence binding " +
                      std::to_string(binding.edge_bundle_id) + " port " +
                      std::to_string(binding.port_id) +
                      " row fixture yaw expected=" +
                      std::to_string(binding.layout_yaw_deg) + " actual=" +
                      std::to_string(
                          plan_it->second.row_fixture.root.rotation_euler_deg.z));
        }
      }
    }
  }

  for (const city::wire::Span& span : state.view().spans().items()) {
    const auto endpoint_exists = [&](city::wire::ObjectId port_id,
                                     city::wire::ObjectId anchor_id,
                                     const char* endpoint) {
      const bool has_port = port_id != city::wire::kInvalidObjectId &&
                            state.view().ports().find(port_id) != nullptr;
      const bool has_anchor = anchor_id != city::wire::kInvalidObjectId &&
                              state.view().anchors().find(anchor_id) != nullptr;
      if (has_port == has_anchor) {
        std::string binding_scope;
        for (const city::wire::SavedBackbonePortBinding& binding : graph.port_bindings) {
          if (binding.port_id == port_id) {
            binding_scope += " edge_bundle=" + std::to_string(binding.edge_bundle_id) +
                             " node=" + std::to_string(binding.row_key.node_id);
          }
        }
        return fail("span " + std::to_string(span.id) + " endpoint " + endpoint +
                    " must reference exactly one existing port or anchor (port=" +
                    std::to_string(port_id) + ", anchor=" + std::to_string(anchor_id) +
                    binding_scope + ")");
      }
      return true;
    };
    if (!endpoint_exists(span.port_a_id, span.anchor_a_id, "a") ||
        !endpoint_exists(span.port_b_id, span.anchor_b_id, "b")) {
      return false;
    }
    if (state.view().bundles().find(span.bundle_id) == nullptr) {
      return fail("span " + std::to_string(span.id) + " has missing bundle");
    }
    const city::wire::SpanLayoutEntry* layout = state.span_layout(span.id).entry;
    const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span.id);
    if (layout == nullptr || curve == nullptr || curve->detail.sample_points.size() < 2) {
      return fail("span " + std::to_string(span.id) +
                  " has no complete layout and curve for row frame coherence");
    }
    const auto check_endpoint = [&](city::wire::ObjectId port_id,
                                    const city::wire::LayoutEndpoint& endpoint,
                                    const city::wire::Vec3d& curve_point,
                                    const char* endpoint_name) {
      if (port_id == city::wire::kInvalidObjectId) return true;
      const city::wire::Port* port = view.ports().find(port_id);
      if (port == nullptr) {
        return fail("span " + std::to_string(span.id) + " endpoint " +
                    endpoint_name + " port " + std::to_string(port_id) +
                    " is missing");
      }
      const auto plan_it = fixture_plan.find(port_id);
      const city::wire::Vec3d expected_endpoint =
          plan_it == fixture_plan.end() ? port->world_position
                                        : plan_it->second.wire_endpoint;
      if (!endpoint.source_projection.valid() &&
          (!almost_equal(endpoint.support_world, expected_endpoint, 1e-9) ||
           !almost_equal(endpoint.endpoint_world, expected_endpoint, 1e-9))) {
        return fail("span " + std::to_string(span.id) + " endpoint " +
                    endpoint_name + " port " + std::to_string(port_id) +
                    " resolved anchor expected=" +
                    vec_text(expected_endpoint) + " support=" +
                    vec_text(endpoint.support_world) + " endpoint=" +
                    vec_text(endpoint.endpoint_world));
      }
      if (!almost_equal(curve_point, endpoint.endpoint_world, 1e-9)) {
        return fail("span " + std::to_string(span.id) + " endpoint " +
                    endpoint_name + " port " + std::to_string(port_id) +
                    " curve endpoint differs from layout endpoint");
      }
      return true;
    };
    if (!check_endpoint(span.port_a_id, layout->start,
                        curve->detail.sample_points.front(), "a") ||
        !check_endpoint(span.port_b_id, layout->end,
                        curve->detail.sample_points.back(), "b")) {
      return false;
    }
  }

  const auto rematerialized =
      city::wire::generation::backbone::materialize_model_assemblies(
          state, fixture_plan);
  if (!rematerialized.ok) {
    return fail("row frame coherence model materialization failed: " +
                rematerialized.error);
  }
  const auto& expected = rematerialized.value.instances;
  const auto& actual = view.visual_model_instances().instances;
  const bool cache_matches =
      expected.size() == actual.size() &&
      std::equal(expected.begin(), expected.end(), actual.begin(),
                 [&](const city::wire::VisualModelInstance& expected_instance,
                     const city::wire::VisualModelInstance& actual_instance) {
                   return expected_instance.stable_key ==
                              actual_instance.stable_key &&
                          expected_instance.model_key ==
                              actual_instance.model_key &&
                          transform_close(expected_instance.world_transform,
                                          actual_instance.world_transform);
                 });
  if (!cache_matches) {
    std::string detail = " expected_count=" + std::to_string(expected.size()) +
                         " actual_count=" + std::to_string(actual.size());
    const std::size_t common = std::min(expected.size(), actual.size());
    for (std::size_t i = 0; i < common; ++i) {
      if (expected[i].stable_key != actual[i].stable_key ||
          expected[i].model_key != actual[i].model_key ||
          !transform_close(expected[i].world_transform,
                           actual[i].world_transform)) {
        detail += " first_difference=" + std::to_string(i) +
                  " expected_key=" + expected[i].stable_key +
                  " actual_key=" + actual[i].stable_key +
                  " expected_position=" +
                  vec_text(expected[i].world_transform.position) +
                  " actual_position=" +
                  vec_text(actual[i].world_transform.position) +
                  " expected_version=" +
                  std::to_string(expected[i].content_version) +
                  " actual_version=" +
                  std::to_string(actual[i].content_version) +
                  " expected_yaw=" +
                  std::to_string(
                      expected[i].world_transform.rotation_euler_deg.z) +
                  " actual_yaw=" +
                  std::to_string(
                      actual[i].world_transform.rotation_euler_deg.z);
        if (expected[i].model_key != actual[i].model_key) {
          detail += " model_key";
        }
        if (!transform_equal(expected[i].world_transform,
                             actual[i].world_transform)) {
          detail += " transform expected_transform=" +
                    transform_text(expected[i].world_transform) +
                    " actual_transform=" +
                    transform_text(actual[i].world_transform);
        }
        if (expected[i].content_version != actual[i].content_version) {
          detail += " content_version";
        }
        break;
      }
    }
    return fail("row frame coherence visual model cache differs from the resolved fixture plan" +
                detail);
  }
  for (const city::wire::SavedBackboneEdge& edge : graph.edges) {
    if (!saved_node_exists(edge.node_a) || !saved_node_exists(edge.node_b)) {
      return fail("saved edge " + std::to_string(edge.edge_id) + " references missing node");
    }
  }
  for (const city::wire::SavedBackboneEdgeBundle& edge_bundle : graph.edge_bundles) {
    if (!saved_edge_exists(edge_bundle.edge_id)) {
      return fail("edge bundle " + std::to_string(edge_bundle.edge_bundle_id) + " references missing edge");
    }
    if (state.view().bundles().find(edge_bundle.bundle_id) == nullptr) {
      return fail("edge bundle " + std::to_string(edge_bundle.edge_bundle_id) + " references missing bundle");
    }
    for (city::wire::ObjectId span_id : edge_bundle.span_ids) {
      if (state.view().spans().find(span_id) == nullptr) {
        return fail("edge bundle " + std::to_string(edge_bundle.edge_bundle_id) + " references missing span");
      }
    }
  }
  for (const city::wire::SavedBackboneSpanBinding& binding : graph.span_bindings) {
    if (!edge_bundle_exists(binding.edge_bundle_id)) {
      return fail("span binding references missing edge bundle " + std::to_string(binding.edge_bundle_id));
    }
    if (state.view().spans().find(binding.span_id) == nullptr) {
      return fail("span binding references missing span " + std::to_string(binding.span_id));
    }
  }
  for (const city::wire::SavedBackbonePortBinding& binding : graph.port_bindings) {
    if (!edge_bundle_exists(binding.edge_bundle_id)) {
      return fail("port binding references missing edge bundle " + std::to_string(binding.edge_bundle_id));
    }
    if (state.view().ports().find(binding.port_id) == nullptr) {
      return fail("port binding references missing port " + std::to_string(binding.port_id));
    }
    if (!saved_node_exists(binding.row_key.node_id)) {
      return fail("port binding references missing row node " + std::to_string(binding.row_key.node_id));
    }
    if (!saved_edge_exists(binding.row_key.edge_id)) {
      return fail("port binding references missing row edge " + std::to_string(binding.row_key.edge_id));
    }
  }
  for (const city::wire::SavedBackboneRowContinuity& continuity : graph.row_continuities) {
    if (!saved_node_exists(continuity.node_id)) {
      return fail("row continuity references missing node " + std::to_string(continuity.node_id));
    }
    if (!edge_bundle_exists(continuity.a.edge_bundle_id) ||
        !edge_bundle_has_lane(continuity.a.edge_bundle_id, continuity.a.lane_index) ||
        !edge_bundle_exists(continuity.b.edge_bundle_id) ||
        !edge_bundle_has_lane(continuity.b.edge_bundle_id, continuity.b.lane_index)) {
      return fail("row continuity references missing edge bundle lane");
    }
  }

  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    for (const city::wire::Vec3d& sample : part.samples) {
      if (!finite_vec(sample)) {
        return fail("visual curve part has non-finite sample");
      }
    }
    if (!finite_vec(part.bounds.min) || !finite_vec(part.bounds.max)) {
      return fail("visual curve part has non-finite bounds");
    }
  }
  if (reason != nullptr) {
    reason->clear();
  }
  return true;
}

} // namespace backbone_tests
