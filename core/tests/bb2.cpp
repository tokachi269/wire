#include "registry.hpp"
#include "helpers.hpp"

#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace helpers;

namespace {

wire::core::BackboneSpec line_req(wire::core::CoreState& state) {
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1.0;
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

bool C368_bb2_smoke_line() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const int count = bundle_count(state, wire::core::BundleKind::kLowVoltage);
  const auto out = state.GenerateFromBackboneSpec(req);
  return out.ok && out.value.generated_pole_ids.size() == 2 && out.value.bundle_ids.size() == 1 &&
         out.value.generated_span_ids.size() == static_cast<std::size_t>(count) &&
         state.view().poles().size() >= 2 && state.view().bundles().size() >= 1 &&
         state.view().spans().size() >= static_cast<std::size_t>(count);
}

bool C369_bb2_rules_saved() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
  }
  return true;
}

bool C370_bb2_no_v1_deps() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "bb2";
  const std::vector<std::string> banned = {
      "backbone_pipeline",
      "bundle_spans",
      "build_backbone",
      "support_layout_",
      "BackbonePipeline",
      "JunctionRelationKind",
      "SpanSupportLayoutDecisionSeed",
      "Authority",
      "Projection",
      "Materialization",
      "generate_grouped_spans_between_support_nodes",
      "remap_backbone_build_to_real_nodes",
      "cache_span_layout_rules",
      "generate_span_curve",
      "cache_rebuilt_span_geometry",
      "rebuild_span_bounds",
      "rebuild_span_visual",
      "build_aabb_from_points",
      "build_aabb_from_two_points",
      "Commit",
      "recalc",
      "materialization",
      "authority",
      "projection",
      "seed",
  };
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    for (const std::string& token : banned) {
      if (contains_text(text, token)) {
        return false;
      }
    }
  }
  return true;
}

bool C371_bb2_rejects_unsupported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec empty = line_req(state);
  empty.bundles.clear();
  const auto empty_out = state.GenerateFromBackboneSpec(empty);
  if (empty_out.ok || !contains_text(empty_out.error, "unsupported")) {
    return false;
  }

  wire::core::BackboneSpec midair = line_req(state);
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kMidair;
  midair.path.node_specs.push_back(node);
  const auto midair_out = state.GenerateFromBackboneSpec(midair);
  return !midair_out.ok && contains_text(midair_out.error, "unsupported");
}

bool C372_bb2_rules_do_not_seed() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
    if (state.span_layout_state(span_id).input_required) {
      return false;
    }
  }
  return true;
}

bool C373_bb2_layout_saved_without_recalc() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
    if (!state.span_layout(span_id).has_layout()) {
      return false;
    }
    if (state.span_layout_state(span_id).input_required) {
      return false;
    }
  }
  return true;
}

std::vector<wire::core::Vec3d> bb2_layout_points(wire::core::CoreState& state) {
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

bool C374_bb2_layout_is_deterministic() {
  wire::core::CoreState a;
  wire::core::CoreState b;
  const std::vector<wire::core::Vec3d> pa = bb2_layout_points(a);
  const std::vector<wire::core::Vec3d> pb = bb2_layout_points(b);
  if (pa.empty() || pa.size() != pb.size()) {
    return false;
  }
  for (std::size_t i = 0; i < pa.size(); ++i) {
    if (!almost_equal(pa[i], pb[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C375_bb2_curve_saved_without_recalc() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
    if (!state.span_layout(span_id).has_layout()) {
      return false;
    }
    const auto* curve = state.find_curve_cache(span_id);
    if (curve == nullptr || curve->detail.sample_points.size() != 2 || curve->points.size() != 2) {
      return false;
    }
    if (state.span_layout_state(span_id).input_required) {
      return false;
    }
  }
  return true;
}

struct CurveSnapshot {
  std::vector<wire::core::Vec3d> points{};
  std::vector<double> lengths{};
};

CurveSnapshot bb2_curve_points(wire::core::CoreState& state) {
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

bool C376_bb2_curve_is_deterministic() {
  wire::core::CoreState a;
  wire::core::CoreState b;
  const CurveSnapshot ca = bb2_curve_points(a);
  const CurveSnapshot cb = bb2_curve_points(b);
  if (ca.points.empty() || ca.points.size() != cb.points.size() || ca.lengths.size() != cb.lengths.size()) {
    return false;
  }
  for (std::size_t i = 0; i < ca.points.size(); ++i) {
    if (!almost_equal(ca.points[i], cb.points[i], 1e-9)) {
      return false;
    }
  }
  for (std::size_t i = 0; i < ca.lengths.size(); ++i) {
    if (!almost_equal(ca.lengths[i], cb.lengths[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C377_bb2_bounds_saved_without_recalc() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
    if (!state.span_layout(span_id).has_layout()) {
      return false;
    }
    if (state.find_curve_cache(span_id) == nullptr) {
      return false;
    }
    const auto* bounds = state.find_bounds_cache(span_id);
    if (bounds == nullptr || bounds->segments.empty()) {
      return false;
    }
    if (state.span_layout_state(span_id).input_required) {
      return false;
    }
  }
  return true;
}

struct BoundsSnapshot {
  std::vector<wire::core::Vec3d> pts{};
};

void push_box(const wire::core::AABBd& box, BoundsSnapshot* out) {
  out->pts.push_back(box.min);
  out->pts.push_back(box.max);
}

BoundsSnapshot bb2_bounds_points(wire::core::CoreState& state) {
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

bool C378_bb2_bounds_is_deterministic() {
  wire::core::CoreState a;
  wire::core::CoreState b;
  const BoundsSnapshot ba = bb2_bounds_points(a);
  const BoundsSnapshot bb = bb2_bounds_points(b);
  if (ba.pts.empty() || ba.pts.size() != bb.pts.size()) {
    return false;
  }
  for (std::size_t i = 0; i < ba.pts.size(); ++i) {
    if (!almost_equal(ba.pts[i], bb.pts[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C379_bb2_m1_required_outputs() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 2 || out.value.bundle_ids.size() != 1 ||
      out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
    if (!state.span_layout(span_id).has_layout()) {
      return false;
    }
    if (state.find_curve_cache(span_id) == nullptr) {
      return false;
    }
    if (state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
  }
  return true;
}

bool C380_bb2_m1_draw_outputs_not_required() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (state.find_span_visual_cache(span_id) != nullptr) {
      return false;
    }
    if (state.view().find_span_render_cache(span_id) != nullptr) {
      return false;
    }
  }
  return true;
}

bool C381_bb2_m1_no_recalc_contract() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (state.span_layout_state(span_id).input_required) {
      return false;
    }
  }
  return C370_bb2_no_v1_deps();
}

bool C382_bb2_geom_is_single_pipeline_layer() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "bb2";
  bool has_geom = false;
  const std::vector<std::string> banned = {
      "make(const curve&",
      "save(curve",
      "save(bounds",
  };
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    has_geom = has_geom || contains_text(text, "struct geom");
    for (const std::string& token : banned) {
      if (contains_text(text, token)) {
        return false;
      }
    }
  }
  return has_geom;
}

bool C383_bb2_draw_is_declared_but_not_built() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "bb2";
  bool has_draw = false;
  const std::vector<std::string> banned = {
      "make(draw",
      "save(draw",
      "visual_cache",
      "render_cache",
  };
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    has_draw = has_draw || contains_text(text, "struct draw");
    for (const std::string& token : banned) {
      if (contains_text(text, token)) {
        return false;
      }
    }
  }
  return has_draw;
}

bool C384_bb2_topo_is_single_output_layer() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "bb2";
  bool has_topo = false;
  bool has_graph_poles = false;
  const std::vector<std::string> banned = {
      "struct spans",
  };
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    has_topo = has_topo || contains_text(text, "struct topo");
    has_graph_poles = has_graph_poles || contains_text(text, "struct graph {\n  route r{};\n  std::vector<ObjectId> poles{};");
    for (const std::string& token : banned) {
      if (contains_text(text, token)) {
        return false;
      }
    }
  }
  return has_topo && !has_graph_poles;
}

bool C385_bb2_emit_is_split_by_topology_parts() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "bb2";
  bool has_poles = false;
  bool has_bundles = false;
  bool has_ports = false;
  bool has_spans = false;
  bool emit_body_has_add = false;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    has_poles = has_poles || contains_text(text, "emit_poles");
    has_bundles = has_bundles || contains_text(text, "emit_bundles");
    has_ports = has_ports || contains_text(text, "emit_ports");
    has_spans = has_spans || contains_text(text, "emit_spans");
    const std::string marker = "EditResult<topo> pipeline::emit()";
    const std::size_t start = text.find(marker);
    if (start != std::string::npos) {
      const std::size_t end = text.find("rules pipeline::make", start);
      const std::string body = text.substr(start, end == std::string::npos ? std::string::npos : end - start);
      emit_body_has_add = contains_text(body, "AddPole") || contains_text(body, "AddBundle") ||
                          contains_text(body, "AddPort") || contains_text(body, "AddSpan");
    }
  }
  return has_poles && has_bundles && has_ports && has_spans && !emit_body_has_add;
}

bool C386_bb2_link_pair_row_are_separate() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "bb2";
  bool has_link = false;
  bool has_pair = false;
  bool has_open = false;
  bool has_row = false;
  bool row_has_source = false;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    has_link = has_link || contains_text(text, "struct link");
    has_pair = has_pair || contains_text(text, "struct pair");
    has_open = has_open || contains_text(text, "struct open");
    has_row = has_row || contains_text(text, "struct row");
    row_has_source = row_has_source || contains_text(text, "src source");
  }
  return has_link && has_pair && has_open && has_row && row_has_source;
}

bool C387_bb2_pairs_are_single_source() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text)) {
    return false;
  }
  const std::string signature = "EditResult<pairs> pipeline::make(const graph& made) const";
  const std::string call = "EditResult<pairs> ps = make(g_)";
  const std::size_t first = text.find(call);
  return contains_text(text, signature) && first != std::string::npos && text.find(call, first + call.size()) == std::string::npos;
}

bool C388_bb2_polyline3_pair_model() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text)) {
    return false;
  }
  const bool builds_links = contains_text(text, "g_.links.push_back(edge)");
  const bool builds_pair = contains_text(text, "add_pair(&out.value");
  const bool builds_open = contains_text(text, "add_open(&out.value");
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  const int count = bundle_count(state, wire::core::BundleKind::kLowVoltage);
  const auto out = state.GenerateFromBackboneSpec(req);
  return builds_links && builds_pair && builds_open && out.ok && out.value.generated_pole_ids.size() == 3 &&
         out.value.generated_span_ids.size() == static_cast<std::size_t>(count * 2);
}

bool C389_bb2_row_axis_owned_by_pairs() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text)) {
    return false;
  }
  const std::string ports_marker = "EditResult<bool> pipeline::emit_ports";
  const std::size_t ports_start = text.find(ports_marker);
  if (ports_start == std::string::npos) {
    return false;
  }
  const std::size_t ports_end = text.find("EditResult<bool> pipeline::emit_spans", ports_start);
  const std::string ports_body = text.substr(ports_start, ports_end == std::string::npos ? std::string::npos
                                                                                          : ports_end - ports_start);
  return contains_text(ports_body, "r.axis") && !contains_text(ports_body, "side(") &&
         !contains_text(ports_body, "norm(");
}

bool C390_bb2_rejects_reused_incident() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text) || !contains_text(text, "incident reused")) {
    return false;
  }
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {4.0, 0.0, 0.0}};
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C391_bb2_no_kind_label() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "bb2";
  const std::vector<std::string> banned = {
      "ThroughMain",
      "SideBranch",
      "CrossUnderpass",
      "CornerContinuation",
      "enum class junction",
      "enum class role",
  };
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    for (const std::string& token : banned) {
      if (contains_text(text, token)) {
        return false;
      }
    }
  }
  return true;
}

bool C392_bb2_polyline3_outputs() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
    if (!state.span_layout(span_id).has_layout()) {
      return false;
    }
    if (state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
  }
  return true;
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

bool C393_bb2_polyline3_is_deterministic() {
  wire::core::CoreState a;
  wire::core::CoreState b;
  const std::vector<wire::core::Vec3d> pa = poly3_points(a);
  const std::vector<wire::core::Vec3d> pb = poly3_points(b);
  if (pa.empty() || pa.size() != pb.size()) {
    return false;
  }
  for (std::size_t i = 0; i < pa.size(); ++i) {
    if (!almost_equal(pa[i], pb[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C394_bb2_existing_pole_node_is_not_recreated() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = line_req(state);
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok || first_out.value.generated_pole_ids.empty()) {
    return false;
  }
  const std::size_t pole_count_before = state.view().poles().size();
  wire::core::BackboneSpec second = line_req(state);
  const wire::core::ObjectId existing = first_out.value.generated_pole_ids.front();
  const auto* existing_pole = state.view().poles().find(existing);
  if (existing_pole == nullptr) {
    return false;
  }
  second.path.polyline = {existing_pole->world_transform.position, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = existing;
  second.path.node_specs.push_back(node);
  const auto second_out = state.GenerateFromBackboneSpec(second);
  return second_out.ok && second_out.value.generated_pole_ids.size() == 1 &&
         state.view().poles().size() == pole_count_before + 1;
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

bool C395_bb2_is_new_does_not_affect_pairs() {
  wire::core::CoreState all_new;
  wire::core::BackboneSpec req = line_req(all_new);
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  const auto all_new_out = all_new.GenerateFromBackboneSpec(req);
  if (!all_new_out.ok || all_new_out.value.generated_pole_ids.empty()) {
    return false;
  }
  const std::vector<wire::core::Vec3d> a = span_curve_points(all_new, all_new_out.value.generated_span_ids);

  wire::core::CoreState existing;
  wire::core::BackboneSpec seed = line_req(existing);
  seed.path.polyline = {{0.0, 0.0, 0.0}, {4.0, 4.0, 0.0}};
  const auto seed_out = existing.GenerateFromBackboneSpec(seed);
  if (!seed_out.ok || seed_out.value.generated_pole_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec next = line_req(existing);
  const wire::core::ObjectId existing_id = seed_out.value.generated_pole_ids.front();
  next.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = existing_id;
  next.path.node_specs.push_back(node);
  const auto existing_out = existing.GenerateFromBackboneSpec(next);
  if (!existing_out.ok) {
    return false;
  }
  const std::vector<wire::core::Vec3d> b = span_curve_points(existing, existing_out.value.generated_span_ids);
  if (a.empty() || a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!almost_equal(a[i], b[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C396_bb2_existing_pole_does_not_read_existing_spans() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = line_req(state);
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok || first_out.value.generated_pole_ids.empty() || first_out.value.generated_span_ids.empty()) {
    return false;
  }
  std::vector<const wire::core::SpanLayoutEntry*> before{};
  for (wire::core::ObjectId span_id : first_out.value.generated_span_ids) {
    before.push_back(state.span_layout(span_id).entry);
    if (before.back() == nullptr) {
      return false;
    }
  }
  const wire::core::ObjectId existing = first_out.value.generated_pole_ids.front();
  const auto* existing_pole = state.view().poles().find(existing);
  if (existing_pole == nullptr) {
    return false;
  }
  wire::core::BackboneSpec second = line_req(state);
  second.path.polyline = {existing_pole->world_transform.position, {0.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = existing;
  second.path.node_specs.push_back(node);
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (!second_out.ok || second_out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : second_out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
  }
  for (std::size_t i = 0; i < first_out.value.generated_span_ids.size(); ++i) {
    if (state.span_layout(first_out.value.generated_span_ids[i]).entry != before[i]) {
      return false;
    }
  }
  return true;
}

bool C397_bb2_rejects_non_pole_node_spec() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kMidair;
  node.node_id = 1;
  req.path.node_specs.push_back(node);
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C398_bb2_rejects_missing_existing_pole() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = 999999;
  req.path.node_specs.push_back(node);
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
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

bool C399_bb2_existing_pole_sequence_is_deterministic() {
  wire::core::CoreState a;
  wire::core::CoreState b;
  const std::vector<wire::core::Vec3d> pa = existing_sequence_points(a);
  const std::vector<wire::core::Vec3d> pb = existing_sequence_points(b);
  if (pa.empty() || pa.size() != pb.size()) {
    return false;
  }
  for (std::size_t i = 0; i < pa.size(); ++i) {
    if (!almost_equal(pa[i], pb[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C400_bb2_multiple_bundles_smoke() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  const int count = req_bundle_count(state, req);
  const auto out = state.GenerateFromBackboneSpec(req);
  return out.ok && out.value.bundle_ids.size() == 2 &&
         out.value.generated_span_ids.size() == static_cast<std::size_t>(count);
}

bool C401_bb2_multiple_bundles_polyline3_outputs() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  const int count = req_bundle_count(state, req);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.bundle_ids.size() != 2 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(count * 2)) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
  }
  return true;
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

bool C402_bb2_bundle_spec_does_not_affect_pairs() {
  wire::core::CoreState single;
  wire::core::BackboneSpec a = poly3_req(single);
  const std::vector<wire::core::Vec3d> pa = pole_positions_for(single, a);

  wire::core::CoreState multi;
  wire::core::BackboneSpec b = poly3_req(multi);
  add_backbone_bundle(b, wire::core::BundleKind::kCommunication);
  const std::vector<wire::core::Vec3d> pb = pole_positions_for(multi, b);
  if (pa.empty() || pa.size() != pb.size()) {
    return false;
  }
  for (std::size_t i = 0; i < pa.size(); ++i) {
    if (!almost_equal(pa[i], pb[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C403_bb2_existing_pole_with_multiple_bundles() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = line_req(state);
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok || first_out.value.generated_pole_ids.empty()) {
    return false;
  }
  const std::size_t pole_count_before = state.view().poles().size();
  const wire::core::ObjectId existing = first_out.value.generated_pole_ids.front();
  const auto* existing_pole = state.view().poles().find(existing);
  if (existing_pole == nullptr) {
    return false;
  }
  wire::core::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  req.path.polyline = {existing_pole->world_transform.position, {0.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = existing;
  req.path.node_specs.push_back(node);
  const int count = req_bundle_count(state, req);
  const auto out = state.GenerateFromBackboneSpec(req);
  return out.ok && out.value.bundle_ids.size() == 2 && out.value.generated_pole_ids.size() == 1 &&
         state.view().poles().size() == pole_count_before + 1 &&
         out.value.generated_span_ids.size() == static_cast<std::size_t>(count);
}

bool C404_bb2_rejects_empty_bundles() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.bundles.clear();
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C405_bb2_no_bundle_pair_branching() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text)) {
    return false;
  }
  const std::string marker = "EditResult<pairs> pipeline::make(const graph& made) const";
  const std::size_t start = text.find(marker);
  if (start == std::string::npos) {
    return false;
  }
  const std::size_t end = text.find("EditResult<bool> pipeline::emit_poles", start);
  const std::string body = text.substr(start, end == std::string::npos ? std::string::npos : end - start);
  return !contains_text(body, "spec_.bundles") && !contains_text(body, "BundleTemplate") &&
         !contains_text(body, "bundle_template");
}

bool C406_bb2_port_height_uses_pole_band() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const double expected = band_height(state, req.pole_type_id, wire::core::BundleKind::kLowVoltage);
  if (expected < 0.0 || almost_equal(expected, 9.32, 1e-9)) {
    return false;
  }
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!span_ports_match_z(state, span_id, expected)) {
      return false;
    }
  }
  return true;
}

bool C407_bb2_multiple_bundle_heights_are_band_based() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  const double lv_z = band_height(state, req.pole_type_id, wire::core::BundleKind::kLowVoltage);
  const double comm_z = band_height(state, req.pole_type_id, wire::core::BundleKind::kCommunication);
  if (lv_z < 0.0 || comm_z < 0.0 || almost_equal(lv_z, comm_z, 1e-9)) {
    return false;
  }
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  bool saw_lv = false;
  bool saw_comm = false;
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    const auto* bundle = state.view().bundles().find(span->bundle_id);
    if (bundle == nullptr) {
      return false;
    }
    if (bundle->bundle_template_id == wire::core::BundleKind::kLowVoltage) {
      saw_lv = true;
      if (!span_ports_match_z(state, span_id, lv_z)) {
        return false;
      }
    }
    if (bundle->bundle_template_id == wire::core::BundleKind::kCommunication) {
      saw_comm = true;
      if (!span_ports_match_z(state, span_id, comm_z)) {
        return false;
      }
    }
  }
  return saw_lv && saw_comm;
}

bool C408_bb2_existing_pole_uses_actual_pole_type_height() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const double request_z = band_height(state, req.pole_type_id, wire::core::BundleKind::kLowVoltage);
  wire::core::PoleTypeId existing_type = wire::core::kInvalidPoleTypeId;
  double existing_z = -1.0;
  for (const auto& item : state.view().pole_types()) {
    const double z = band_height(state, item.first, wire::core::BundleKind::kLowVoltage);
    if (z >= 0.0 && !almost_equal(z, request_z, 1e-9)) {
      existing_type = item.first;
      existing_z = z;
      break;
    }
  }
  if (existing_type == wire::core::kInvalidPoleTypeId || existing_z < 0.0) {
    return false;
  }
  wire::core::Transformd tf{};
  tf.position = {0.0, 0.0, 0.0};
  const auto pole = state.AddPole(tf, 10.0, "existing", wire::core::PoleKind::kConcrete);
  if (!pole.ok || !state.ApplyPoleType(pole.value, existing_type).ok) {
    return false;
  }
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = pole.value;
  req.path.node_specs.push_back(node);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  bool saw_existing = false;
  bool saw_new = false;
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    const auto* a = state.view().ports().find(span->port_a_id);
    const auto* b = state.view().ports().find(span->port_b_id);
    if (a == nullptr || b == nullptr) {
      return false;
    }
    for (const wire::core::Port* port : {a, b}) {
      if (port->owner_pole_id == pole.value) {
        saw_existing = true;
        if (!almost_equal(port->world_position.z, existing_z, 1e-9)) {
          return false;
        }
      } else {
        saw_new = true;
        if (!almost_equal(port->world_position.z, request_z, 1e-9)) {
          return false;
        }
      }
    }
  }
  return saw_existing && saw_new;
}

bool C409_bb2_rejects_missing_port_band() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  auto it = state.view().pole_types().find(req.pole_type_id);
  if (it == state.view().pole_types().end()) {
    return false;
  }
  wire::core::PoleTypeDefinition type = it->second;
  type.port_bands.erase(std::remove_if(type.port_bands.begin(), type.port_bands.end(),
                                       [](const wire::core::PortPlacementBand& band) {
                                         return band.category == wire::core::ConnectionCategory::kLowVoltage &&
                                                band.layer == 1;
                                       }),
                        type.port_bands.end());
  if (!state.UpdatePoleTypeDefinition(type).ok) {
    return false;
  }
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C410_bb2_height_does_not_affect_pairs() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text)) {
    return false;
  }
  const std::string marker = "EditResult<pairs> pipeline::make(const graph& made) const";
  const std::size_t start = text.find(marker);
  if (start == std::string::npos) {
    return false;
  }
  const std::size_t end = text.find("EditResult<bool> pipeline::emit_poles", start);
  const std::string body = text.substr(start, end == std::string::npos ? std::string::npos : end - start);
  return !contains_text(body, "pole_type") && !contains_text(body, "PortPlacementBand") &&
         !contains_text(body, "height") && !contains_text(body, "BundleTemplate") &&
         !contains_text(body, "spec_.bundles");
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

bool C411_bb2_lateral_offset_moves_ports_along_row_axis() {
  const std::vector<wire::core::Vec3d> zero = offset_curve_points(0.0);
  const std::vector<wire::core::Vec3d> plus = offset_curve_points(1.0);
  const std::vector<wire::core::Vec3d> minus = offset_curve_points(-1.0);
  if (zero.empty() || zero.size() != plus.size() || zero.size() != minus.size()) {
    return false;
  }
  for (std::size_t i = 0; i < zero.size(); ++i) {
    const wire::core::Vec3d expected_plus{zero[i].x, zero[i].y + 1.0, zero[i].z};
    const wire::core::Vec3d expected_minus{zero[i].x, zero[i].y - 1.0, zero[i].z};
    if (!almost_equal(plus[i], expected_plus, 1e-9) || !almost_equal(minus[i], expected_minus, 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C412_bb2_lateral_offset_sign_is_deterministic() {
  const std::vector<wire::core::Vec3d> a = offset_points(1.0);
  const std::vector<wire::core::Vec3d> b = offset_points(1.0);
  if (a.empty() || a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!almost_equal(a[i], b[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C413_bb2_lateral_offset_does_not_affect_pairs() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text)) {
    return false;
  }
  const std::string marker = "EditResult<pairs> pipeline::make(const graph& made) const";
  const std::size_t start = text.find(marker);
  if (start == std::string::npos) {
    return false;
  }
  const std::size_t end = text.find("EditResult<bool> pipeline::emit_poles", start);
  const std::string body = text.substr(start, end == std::string::npos ? std::string::npos : end - start);
  return !contains_text(body, "constraints") && !contains_text(body, "lateral_offset_m");
}

bool C414_bb2_still_rejects_avoid_constraints() {
  wire::core::CoreState state;
  wire::core::BackboneSpec avoid_point = line_req(state);
  avoid_point.constraints.avoid_points.push_back({6.0, 0.0, 0.0});
  const auto point_out = state.GenerateFromBackboneSpec(avoid_point);
  if (point_out.ok || !contains_text(point_out.error, "unsupported")) {
    return false;
  }
  wire::core::BackboneSpec avoid_radius = line_req(state);
  avoid_radius.constraints.avoid_radius_m = 1.0;
  const auto radius_out = state.GenerateFromBackboneSpec(avoid_radius);
  return !radius_out.ok && contains_text(radius_out.error, "unsupported");
}

bool C415_bb2_has_no_empty_levels_layer() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "bb2";
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    if (contains_text(text, "struct levels") || contains_text(text, "levels ")) {
      return false;
    }
  }
  return true;
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

bool C416_bb2_node_mode_not_present_is_noop() {
  const std::vector<wire::core::Vec3d> plain = node_mode_points(false);
  const std::vector<wire::core::Vec3d> no_op = node_mode_points(true);
  if (plain.empty() || plain.size() != no_op.size()) {
    return false;
  }
  for (std::size_t i = 0; i < plain.size(); ++i) {
    if (!almost_equal(plain[i], no_op[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C417_bb2_node_mode_pass_through_rejected() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  wire::core::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = 0;
  mode.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  mode.mode = wire::core::BundleNodeMode::kPassThrough;
  req.node_bundle_modes.push_back(mode);
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C418_bb2_node_mode_unknown_bundle_rejected() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  wire::core::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = 0;
  mode.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  mode.mode = wire::core::BundleNodeMode::kNotPresent;
  req.node_bundle_modes.push_back(mode);
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C419_bb2_node_mode_point_index_rejected() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  wire::core::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = req.path.polyline.size();
  mode.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  mode.mode = wire::core::BundleNodeMode::kNotPresent;
  req.node_bundle_modes.push_back(mode);
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C420_bb2_node_mode_does_not_affect_pairs() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text)) {
    return false;
  }
  const std::string marker = "EditResult<pairs> pipeline::make(const graph& made) const";
  const std::size_t start = text.find(marker);
  if (start == std::string::npos) {
    return false;
  }
  const std::size_t end = text.find("EditResult<bool> pipeline::emit_poles", start);
  const std::string body = text.substr(start, end == std::string::npos ? std::string::npos : end - start);
  return !contains_text(body, "node_bundle_modes") && !contains_text(body, "BundleNodeMode");
}

bool C421_bb2_topo_row_carries_source() {
  const std::filesystem::path header = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.hpp";
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string h;
  std::string cpp;
  if (!file_text(header, &h) || !file_text(source, &cpp)) {
    return false;
  }
  const std::size_t trow_pos = h.find("struct trow");
  const std::size_t tspan_pos = h.find("struct tspan", trow_pos);
  if (trow_pos == std::string::npos || tspan_pos == std::string::npos) {
    return false;
  }
  const std::string trow_body = h.substr(trow_pos, tspan_pos - trow_pos);
  if (!contains_text(trow_body, "std::size_t row") || !contains_text(trow_body, "std::size_t node") ||
      !contains_text(trow_body, "src source") || !contains_text(trow_body, "Vec3d axis")) {
    return false;
  }
  const std::size_t ports_pos = cpp.find("EditResult<bool> pipeline::emit_ports");
  const std::size_t spans_pos = cpp.find("EditResult<bool> pipeline::emit_spans", ports_pos);
  if (ports_pos == std::string::npos || spans_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(ports_pos, spans_pos - ports_pos);
  return contains_text(body, "tr.row = r.id") && contains_text(body, "tr.node = r.node") &&
         contains_text(body, "tr.source = r.source") && contains_text(body, "tr.axis = r.axis");
}

bool C422_bb2_rules_consume_topo_only() {
  const std::filesystem::path header = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.hpp";
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string h;
  std::string cpp;
  if (!file_text(header, &h) || !file_text(source, &cpp)) {
    return false;
  }
  if (!contains_text(h, "rules make(const topo& made) const") ||
      contains_text(h, "rules make(const topo& made, const pairs& ps) const")) {
    return false;
  }
  const std::size_t rules_pos = cpp.find("rules pipeline::make(const topo& made) const");
  const std::size_t layout_pos = cpp.find("EditResult<layout> pipeline::make", rules_pos);
  if (rules_pos == std::string::npos || layout_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(rules_pos, layout_pos - rules_pos);
  return !contains_text(body, "ps.links");
}

bool C423_bb2_tspan_carries_endpoint_rows() {
  const std::filesystem::path header = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.hpp";
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string h;
  std::string cpp;
  if (!file_text(header, &h) || !file_text(source, &cpp)) {
    return false;
  }
  const std::size_t tspan_pos = h.find("struct tspan");
  const std::size_t topo_pos = h.find("struct topo", tspan_pos);
  if (tspan_pos == std::string::npos || topo_pos == std::string::npos) {
    return false;
  }
  const std::string tspan_body = h.substr(tspan_pos, topo_pos - tspan_pos);
  if (!contains_text(tspan_body, "std::size_t arow") || !contains_text(tspan_body, "std::size_t brow")) {
    return false;
  }
  const std::size_t spans_pos = cpp.find("EditResult<bool> pipeline::emit_spans");
  const std::size_t emit_pos = cpp.find("EditResult<topo> pipeline::emit", spans_pos);
  if (spans_pos == std::string::npos || emit_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(spans_pos, emit_pos - spans_pos);
  return contains_text(body, "edge.arow") && contains_text(body, "edge.brow") && contains_text(body, "tspan{");
}

bool C424_bb2_saves_backbone_graph_nodes_edges() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok) {
    return false;
  }
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.nodes.size() != 2 || graph.edges.size() != 1) {
    return false;
  }
  const wire::core::SavedBackboneEdge& edge = graph.edges.front();
  const auto node_has_pole = [&](wire::core::ObjectId node_id) {
    const auto it = std::find_if(graph.nodes.begin(), graph.nodes.end(), [&](const wire::core::SavedBackboneNode& node) {
      return node.node_id == node_id && node.pole_id != wire::core::kInvalidObjectId;
    });
    return it != graph.nodes.end();
  };
  return edge.node_a != wire::core::kInvalidObjectId && edge.node_b != wire::core::kInvalidObjectId &&
         node_has_pole(edge.node_a) && node_has_pole(edge.node_b);
}

bool C425_bb2_edge_carries_multiple_spans() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.size() < 2) {
    return false;
  }
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.edges.size() != 1) {
    return false;
  }
  std::size_t span_count = 0;
  const auto edge_bundle_it = state.view().backbone_index().edge_bundles.find(graph.edges.front().edge_id);
  if (edge_bundle_it == state.view().backbone_index().edge_bundles.end()) {
    return false;
  }
  for (wire::core::ObjectId edge_bundle_id : edge_bundle_it->second) {
    const auto spans_it = state.view().backbone_index().edge_bundle_spans.find(edge_bundle_id);
    if (spans_it != state.view().backbone_index().edge_bundle_spans.end()) {
      span_count += spans_it->second.size();
    }
  }
  return span_count == out.value.generated_span_ids.size();
}

bool C426_bb2_existing_pole_reuses_graph_node() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = line_req(state);
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok || first_out.value.generated_pole_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId existing = first_out.value.generated_pole_ids.front();
  const auto* existing_pole = state.view().poles().find(existing);
  if (existing_pole == nullptr) {
    return false;
  }
  wire::core::BackboneSpec second = line_req(state);
  second.path.polyline = {existing_pole->world_transform.position, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = existing;
  second.path.node_specs.push_back(node);
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (!second_out.ok) {
    return false;
  }
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  const auto pole_node_it = state.view().backbone_index().pole_node.find(existing);
  if (pole_node_it == state.view().backbone_index().pole_node.end()) {
    return false;
  }
  int matching_nodes = 0;
  for (const wire::core::SavedBackboneNode& saved : graph.nodes) {
    if (saved.pole_id == existing) {
      ++matching_nodes;
    }
  }
  return graph.nodes.size() == 3 && graph.edges.size() == 2 && matching_nodes == 1;
}

bool C427_bb2_graph_index_links_outputs() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok) {
    return false;
  }
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  const wire::core::BackboneIndex& index = state.view().backbone_index();
  if (graph.nodes.size() != 2 || graph.edges.size() != 1) {
    return false;
  }
  const wire::core::ObjectId edge_id = graph.edges.front().edge_id;
  const auto edge_bundles = index.edge_bundles.find(edge_id);
  if (edge_bundles == index.edge_bundles.end() || edge_bundles->second.empty()) {
    return false;
  }
  for (const wire::core::SavedBackboneNode& node : graph.nodes) {
    if (node.pole_id == wire::core::kInvalidObjectId || index.pole_node.find(node.pole_id) == index.pole_node.end()) {
      return false;
    }
    const auto node_edges = index.node_edges.find(node.node_id);
    if (node_edges == index.node_edges.end() || node_edges->second.empty()) {
      return false;
    }
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto span_edge_bundle = index.span_edge_bundle.find(span_id);
    if (span_edge_bundle == index.span_edge_bundle.end() ||
        !contains_id(edge_bundles->second, span_edge_bundle->second)) {
      return false;
    }
    const auto edge_bundle_spans = index.edge_bundle_spans.find(span_edge_bundle->second);
    if (edge_bundle_spans == index.edge_bundle_spans.end() ||
        !contains_id(edge_bundle_spans->second, span_id)) {
      return false;
    }
  }
  return true;
}

bool C428_bb2_pole_frontier_collects_incident_graph() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = poly3_req(state);
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok || first_out.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first_out.value.generated_pole_ids[1];
  const auto* b_pole = state.view().poles().find(b);
  if (b_pole == nullptr) {
    return false;
  }

  wire::core::BackboneSpec second = line_req(state);
  second.path.polyline = {b_pole->world_transform.position, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = b;
  second.path.node_specs.push_back(node);
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (!second_out.ok) {
    return false;
  }

  const wire::core::BackboneFrontier frontier = state.view().pole_frontier(b);
  return frontier.pole_id == b && frontier.node_id != wire::core::kInvalidObjectId && frontier.edge_ids.size() == 3 &&
         frontier.pole_ids.size() == 4 &&
         frontier.span_ids.size() == first_out.value.generated_span_ids.size() + second_out.value.generated_span_ids.size();
}

bool C429_bb2_span_frontier_collects_edge_bundle_spans() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::BackboneFrontier frontier = state.view().span_frontier(out.value.generated_span_ids.front());
  return frontier.span_id == out.value.generated_span_ids.front() &&
         frontier.edge_id != wire::core::kInvalidObjectId && frontier.edge_ids.size() == 1 &&
         frontier.node_ids.size() == 2 && frontier.pole_ids.size() == 2 &&
         frontier.span_ids.size() == out.value.generated_span_ids.size();
}

bool C430_bb2_frontier_uses_saved_graph_index() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "state" / "core_view.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t pole_pos = cpp.find("BackboneFrontier CoreView::pole_frontier");
  const std::size_t span_pos = cpp.find("BackboneFrontier CoreView::span_frontier", pole_pos);
  const std::size_t attach_pos = cpp.find("const AttachmentTemplate* CoreView::find_attachment_template", span_pos);
  if (pole_pos == std::string::npos || span_pos == std::string::npos || attach_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(pole_pos, attach_pos - pole_pos);
  return contains_text(body, "backbone_index") && contains_text(body, "authoritative_.backbone") &&
         !contains_text(body, "BuildBackboneEdges") && !contains_text(body, "spans_by_port");
}

bool C431_bb2_edge_bundle_is_saved_backbone_unit() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.bundle_ids.size() != 1 || out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.edges.size() != 1 || graph.edge_bundles.size() != 1) {
    return false;
  }
  const wire::core::SavedBackboneEdgeBundle& item = graph.edge_bundles.front();
  if (item.edge_id != graph.edges.front().edge_id || item.bundle_id != out.value.bundle_ids.front() ||
      item.span_ids.size() != out.value.generated_span_ids.size()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!contains_id(item.span_ids, span_id)) {
      return false;
    }
  }
  return true;
}

bool C432_bb2_multiple_bundles_create_multiple_edge_bundles() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.bundle_ids.size() != 2) {
    return false;
  }
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.edges.size() != 1 || graph.edge_bundles.size() != out.value.bundle_ids.size()) {
    return false;
  }
  for (wire::core::ObjectId bundle_id : out.value.bundle_ids) {
    const auto it = std::find_if(graph.edge_bundles.begin(), graph.edge_bundles.end(),
                                 [&](const wire::core::SavedBackboneEdgeBundle& item) {
                                   return item.edge_id == graph.edges.front().edge_id && item.bundle_id == bundle_id &&
                                          !item.span_ids.empty();
                                 });
    if (it == graph.edge_bundles.end()) {
      return false;
    }
  }
  return true;
}

bool C433_bb2_reuses_edge_for_same_poles() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const wire::core::ObjectId a = first.value.generated_pole_ids[0];
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  wire::core::BackboneSpec second = line_req(state);
  second.bundles.clear();
  add_backbone_bundle(second, wire::core::BundleKind::kCommunication);
  second.path.polyline = {pa->world_transform.position, pb->world_transform.position};
  wire::core::BackboneInputSpec::NodeSpec na{};
  na.point_index = 0;
  na.support_kind = wire::core::SupportKind::kPole;
  na.node_id = a;
  wire::core::BackboneInputSpec::NodeSpec nb{};
  nb.point_index = 1;
  nb.support_kind = wire::core::SupportKind::kPole;
  nb.node_id = b;
  second.path.node_specs = {na, nb};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  return second_out.ok && state.view().backbone().nodes.size() == 2 && state.view().backbone().edges.size() == 1 &&
         state.view().backbone().edge_bundles.size() == 2;
}

bool C434_bb2_reverse_generation_reuses_edge_bundle() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const wire::core::ObjectId a = first.value.generated_pole_ids[0];
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  wire::core::BackboneSpec second = line_req(state);
  second.path.polyline = {pb->world_transform.position, pa->world_transform.position};
  wire::core::BackboneInputSpec::NodeSpec nb{};
  nb.point_index = 0;
  nb.support_kind = wire::core::SupportKind::kPole;
  nb.node_id = b;
  wire::core::BackboneInputSpec::NodeSpec na{};
  na.point_index = 1;
  na.support_kind = wire::core::SupportKind::kPole;
  na.node_id = a;
  second.path.node_specs = {nb, na};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (!second_out.ok || state.view().backbone().edges.size() != 1 || state.view().backbone().edge_bundles.size() != 1) {
    return false;
  }
  const wire::core::SavedBackboneEdgeBundle& item = state.view().backbone().edge_bundles.front();
  return item.span_ids.size() == first.value.generated_span_ids.size() + second_out.value.generated_span_ids.size();
}

bool C435_bb2_edge_metadata_is_not_overwritten_on_reuse() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2 || state.view().backbone().edges.size() != 1) {
    return false;
  }
  const wire::core::SavedBackboneEdge before = state.view().backbone().edges.front();
  const wire::core::ObjectId a = first.value.generated_pole_ids[0];
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  wire::core::BackboneSpec second = line_req(state);
  second.path.polyline = {pb->world_transform.position, pa->world_transform.position};
  wire::core::BackboneInputSpec::NodeSpec nb{};
  nb.point_index = 0;
  nb.support_kind = wire::core::SupportKind::kPole;
  nb.node_id = b;
  wire::core::BackboneInputSpec::NodeSpec na{};
  na.point_index = 1;
  na.support_kind = wire::core::SupportKind::kPole;
  na.node_id = a;
  second.path.node_specs = {nb, na};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (!second_out.ok || state.view().backbone().edges.size() != 1) {
    return false;
  }
  const wire::core::SavedBackboneEdge& after = state.view().backbone().edges.front();
  return after.edge_id == before.edge_id && after.node_a == before.node_a && after.node_b == before.node_b &&
         after.route == before.route && after.order == before.order && almost_equal(after.dir.x, before.dir.x, 1e-9) &&
         almost_equal(after.dir.y, before.dir.y, 1e-9) && almost_equal(after.dir.z, before.dir.z, 1e-9);
}

bool C436_bb2_frontier_reads_edge_bundles() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::BackboneFrontier pole_frontier = state.view().pole_frontier(out.value.generated_pole_ids.front());
  const wire::core::BackboneFrontier span_frontier = state.view().span_frontier(out.value.generated_span_ids.front());
  return pole_frontier.edge_ids.size() == 1 && pole_frontier.edge_bundle_ids.size() == out.value.bundle_ids.size() &&
         pole_frontier.bundle_ids.size() == out.value.bundle_ids.size() &&
         pole_frontier.span_ids.size() == out.value.generated_span_ids.size() &&
         span_frontier.edge_bundle_id != wire::core::kInvalidObjectId && span_frontier.edge_bundle_ids.size() == 2 &&
         span_frontier.bundle_ids.size() == 2 && span_frontier.span_ids.size() == out.value.generated_span_ids.size();
}

bool C437_bb2_layout_save_is_direct() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "recalc" / "recalc_pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("void CoreState::cache_span_layout(SpanLayoutEntry layout)");
  const std::size_t next_pos = cpp.find("void CoreState::cache_span_curve", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "span_layout_cache.store_layout") &&
         !contains_text(body, "cache_span_support_layout") &&
         !contains_text(body, "rebuild_lowered_support_groups_for_keys");
}

bool C438_bb2_layout_save_keeps_no_authority_contract() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout()) {
      return false;
    }
    if (state.span_layout_state(span_id).input_required) {
      return false;
    }
  }
  return true;
}

bool C439_bb2_source_still_avoids_support_layout_entrypoint() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "bb2";
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    if (contains_text(text, "cache_span_support_layout") || contains_text(text, "support_layout_")) {
      return false;
    }
  }
  return true;
}

bool C440_bb2_does_not_read_authoritative_backbone_directly() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "bb2";
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    if (contains_text(text, "authoritative_.backbone")) {
      return false;
    }
  }
  return true;
}

bool C441_bb2_save_backbone_edge_returns_saved_ref() {
  const std::filesystem::path header = repo_root() / "core" / "include" / "wire" / "core" / "core_state.hpp";
  const std::filesystem::path types = repo_root() / "core" / "include" / "wire" / "core" / "core_authoritative_types.hpp";
  std::string h;
  std::string t;
  if (!file_text(header, &h) || !file_text(types, &t)) {
    return false;
  }
  return contains_text(t, "struct SavedBackboneEdgeRef") &&
         contains_text(h, "SavedBackboneEdgeRef save_backbone_edge");
}

bool C442_bb2_edge_forward_uses_saved_ref() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("void pipeline::save_graph");
  const std::size_t build_pos = cpp.find("EditResult<GenerateBundleFromPathResult> pipeline::build", fn_pos);
  if (fn_pos == std::string::npos || build_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, build_pos - fn_pos);
  return contains_text(body, "std::vector<SavedBackboneEdgeRef>") && contains_text(body, "stored.node_a") &&
         contains_text(body, "stored.node_b") && !contains_text(body, "authoritative_.backbone") &&
         !contains_text(body, "saved_edge") && !contains_text(body, "find_if");
}

bool C443_bb2_edge_reuse_behavior_unchanged() {
  return C434_bb2_reverse_generation_reuses_edge_bundle();
}

bool C444_bb2_layout_uses_neutral_types() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "bb2";
  const std::vector<std::string> banned = {"SpanSupportLayoutEntry", "SupportLayoutEndpoint",
                                           "SupportLayoutSemanticDecision", "SupportLayoutOriginKind",
                                           "SupportLayoutEndpointSourceKind"};
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    for (const std::string& token : banned) {
      if (contains_text(text, token)) {
        return false;
      }
    }
  }
  return true;
}

bool C445_bb2_cache_span_layout_accepts_neutral_entry() {
  const std::filesystem::path header = repo_root() / "core" / "include" / "wire" / "core" / "core_state.hpp";
  const std::filesystem::path source = repo_root() / "core" / "src" / "recalc" / "recalc_pipeline.cpp";
  std::string h;
  std::string cpp;
  if (!file_text(header, &h) || !file_text(source, &cpp)) {
    return false;
  }
  if (!contains_text(h, "void cache_span_layout(SpanLayoutEntry layout)")) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("void CoreState::cache_span_layout(SpanLayoutEntry layout)");
  const std::size_t next_pos = cpp.find("void CoreState::cache_span_curve", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "span_layout_cache.store_layout") &&
         !contains_text(body, "cache_span_support_layout") &&
         !contains_text(body, "rebuild_lowered_support_groups_for_keys");
}

bool C446_bb2_layout_boundary_behavior_unchanged() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
    if (state.span_layout_state(span_id).input_required) {
      return false;
    }
  }
  return true;
}

bool C447_bb2_span_layout_view_reads_neutral_layout() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const wire::core::SpanLayoutView view = state.span_layout(span_id);
    const wire::core::SpanLayoutEntry* entry = view.entry;
    if (!view.has_layout() || entry == nullptr || entry->span_id != span_id) {
      return false;
    }
  }
  return true;
}

bool C448_bb2_tests_use_neutral_layout_read() {
  const std::filesystem::path source = repo_root() / "core" / "tests" / "bb2.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::string old_read = std::string("support_layout_") + "projection(";
  return contains_text(cpp, "span_layout(") && !contains_text(cpp, old_read);
}

bool C449_bb2_layout_read_does_not_expose_authority() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "recalc" / "recalc_pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("SpanLayoutView CoreState::span_layout");
  const std::size_t next_pos = cpp.find("SpanLayoutRulesView CoreState::span_layout_rules", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "span_layout_cache.layout_view") && !contains_text(body, "authority_view") &&
         !contains_text(body, "contract_view") && !contains_text(body, "seed");
}

bool C450_bb2_span_layout_state_is_neutral() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const wire::core::SpanLayoutState state_view = state.span_layout_state(span_id);
    if (!state_view.has_rules || !state_view.has_layout || state_view.input_required) {
      return false;
    }
  }
  return true;
}

bool C451_bb2_tests_do_not_read_old_contract() {
  const std::filesystem::path source = repo_root() / "core" / "tests" / "bb2.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::string old_read = std::string("support_layout_") + "contract(";
  return !contains_text(cpp, old_read);
}

bool C452_bb2_layout_state_does_not_expose_old_contract_names() {
  const std::filesystem::path header = repo_root() / "core" / "include" / "wire" / "core" / "support_layout_types.hpp";
  std::string h;
  if (!file_text(header, &h)) {
    return false;
  }
  const std::size_t type_pos = h.find("struct SpanLayoutState");
  const std::size_t next_pos = h.find("struct LoweredSupportGroupPlacement", type_pos);
  if (type_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = h.substr(type_pos, next_pos - type_pos);
  return !contains_text(body, "authority") && !contains_text(body, "seed") && !contains_text(body, "contract");
}

bool C453_bb2_layout_state_reads_existing_cache_without_seed_path() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "recalc" / "recalc_pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("SpanLayoutState CoreState::span_layout_state");
  const std::size_t next_pos = cpp.find("SpanLayoutRulesView CoreState::span_layout_rules", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "span_layout_cache.layout_state") && !contains_text(body, "store_seed") &&
         !contains_text(body, "cache_span_support_layout_seed") && !contains_text(body, "contract_view");
}

bool C454_bb2_cache_state_uses_span_layout_cache() {
  const std::filesystem::path header = repo_root() / "core" / "include" / "wire" / "core" / "core_runtime_types.hpp";
  std::string h;
  if (!file_text(header, &h)) {
    return false;
  }
  const std::size_t type_pos = h.find("struct CacheState");
  const std::size_t next_pos = h.find("struct TemplateDependencyState", type_pos);
  if (type_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = h.substr(type_pos, next_pos - type_pos);
  return contains_text(body, "SpanLayoutCache span_layout_cache") &&
         !contains_text(body, "SupportLayoutCache support_layout_cache") &&
         !contains_text(body, "support_layout_cache");
}

bool C455_bb2_neutral_layout_api_uses_span_layout_cache() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "recalc" / "recalc_pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t read_pos = cpp.find("SpanLayoutView CoreState::span_layout");
  const std::size_t read_end = cpp.find("bool CoreState::cache_rebuilt_span_geometry", read_pos);
  const std::size_t save_pos = cpp.find("void CoreState::cache_span_layout(SpanLayoutEntry layout)");
  const std::size_t curve_pos = cpp.find("void CoreState::cache_span_curve", save_pos);
  const std::size_t rules_pos = cpp.find("void CoreState::cache_span_rules");
  const std::size_t rules_end = cpp.find("void CoreState::erase_cached_span_support_layout_seed", rules_pos);
  if (read_pos == std::string::npos || read_end == std::string::npos || save_pos == std::string::npos ||
      curve_pos == std::string::npos || rules_pos == std::string::npos || rules_end == std::string::npos) {
    return false;
  }
  const std::string read_body = cpp.substr(read_pos, read_end - read_pos);
  const std::string save_body = cpp.substr(save_pos, curve_pos - save_pos);
  const std::string rules_body = cpp.substr(rules_pos, rules_end - rules_pos);
  return contains_text(read_body, "span_layout_cache.layout_view") &&
         contains_text(read_body, "span_layout_cache.layout_state") &&
         contains_text(read_body, "span_layout_cache.rules_view") &&
         contains_text(save_body, "span_layout_cache.store_layout") &&
         contains_text(rules_body, "span_layout_cache.store_rules") &&
         !contains_text(read_body, "support_layout_cache") && !contains_text(save_body, "support_layout_cache") &&
         !contains_text(rules_body, "support_layout_cache");
}

bool C456_bb2_source_avoids_old_layout_cache_names() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "bb2";
  const std::vector<std::string> banned = {"support_layout_cache", "support_layout_projection",
                                           "support_layout_contract"};
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    for (const std::string& token : banned) {
      if (contains_text(text, token)) {
        return false;
      }
    }
  }
  return true;
}

bool C457_bb2_layout_cache_boundary_behavior_unchanged() {
  return C446_bb2_layout_boundary_behavior_unchanged();
}

wire::core::BackboneInputSpec::NodeSpec pole_spec(std::size_t point_index, wire::core::ObjectId pole_id) {
  wire::core::BackboneInputSpec::NodeSpec spec{};
  spec.point_index = point_index;
  spec.support_kind = wire::core::SupportKind::kPole;
  spec.node_id = pole_id;
  return spec;
}

bool C458_bb2_existing_branch_BD_on_ABC() {
  wire::core::CoreState state;
  wire::core::BackboneSpec abc = poly3_req(state);
  const int count = req_bundle_count(state, abc);
  const auto first = state.GenerateFromBackboneSpec(abc);
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  const wire::core::BackboneFrontier frontier = state.view().pole_frontier(b);
  return second.ok && second.value.generated_pole_ids.size() == 1 &&
         second.value.generated_span_ids.size() == static_cast<std::size_t>(count) &&
         frontier.edge_ids.size() == 3 && state.view().backbone().edges.size() == 3;
}

bool C459_bb2_existing_cross_DBE_on_ABC() {
  wire::core::CoreState state;
  wire::core::BackboneSpec abc = poly3_req(state);
  const int count = req_bundle_count(state, abc);
  const auto first = state.GenerateFromBackboneSpec(abc);
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }

  wire::core::BackboneSpec cross = line_req(state);
  cross.path.polyline = {{12.0, -8.0, 0.0}, pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  cross.path.node_specs = {pole_spec(1, b)};
  const auto second = state.GenerateFromBackboneSpec(cross);
  const wire::core::BackboneFrontier frontier = state.view().pole_frontier(b);
  return second.ok && second.value.generated_pole_ids.size() == 2 &&
         second.value.generated_span_ids.size() == static_cast<std::size_t>(count * 2) &&
         frontier.edge_ids.size() == 4 && state.view().backbone().edges.size() == 4;
}

bool C460_bb2_context_links_are_not_emitted() {
  wire::core::CoreState state;
  wire::core::BackboneSpec abc = poly3_req(state);
  const int count = req_bundle_count(state, abc);
  const auto first = state.GenerateFromBackboneSpec(abc);
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const std::size_t span_count_before = state.view().spans().size();

  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  return second.ok && state.view().spans().size() == span_count_before + second.value.generated_span_ids.size() &&
         second.value.generated_span_ids.size() == static_cast<std::size_t>(count);
}

bool C461_bb2_same_edge_request_skips_duplicate_context() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first_req = line_req(state);
  const auto first = state.GenerateFromBackboneSpec(first_req);
  if (!first.ok || first.value.generated_pole_ids.size() != 2 || state.view().backbone().edges.size() != 1) {
    return false;
  }
  const wire::core::ObjectId a = first.value.generated_pole_ids[0];
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_a = state.view().poles().find(a);
  const auto* pole_b = state.view().poles().find(b);
  if (pole_a == nullptr || pole_b == nullptr) {
    return false;
  }

  wire::core::BackboneSpec second_req = line_req(state);
  second_req.bundles.clear();
  add_backbone_bundle(second_req, wire::core::BundleKind::kCommunication);
  second_req.path.polyline = {pole_a->world_transform.position, pole_b->world_transform.position};
  second_req.path.node_specs = {pole_spec(0, a), pole_spec(1, b)};
  const auto second = state.GenerateFromBackboneSpec(second_req);
  return second.ok && state.view().backbone().edges.size() == 1 && state.view().backbone().edge_bundles.size() == 2 &&
         !second.value.generated_span_ids.empty();
}

bool C462_bb2_no_junction_kind_after_existing_context() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  return C391_bb2_no_kind_label() && contains_text(cpp, "has_requested_saved_pair") &&
         contains_text(cpp, "if (!edge.is_new)") && contains_text(cpp, "EditResult<pairs> pipeline::make");
}

bool C463_bb2_same_edge_bundle_reuses_edge_bundle() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2 || state.view().backbone().edge_bundles.size() != 1) {
    return false;
  }
  const wire::core::ObjectId edge_bundle_id = state.view().backbone().edge_bundles.front().edge_bundle_id;
  const wire::core::ObjectId a = first.value.generated_pole_ids[0];
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  wire::core::BackboneSpec second = line_req(state);
  second.path.polyline = {pa->world_transform.position, pb->world_transform.position};
  second.path.node_specs = {pole_spec(0, a), pole_spec(1, b)};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (!second_out.ok || state.view().backbone().edges.size() != 1 || state.view().backbone().edge_bundles.size() != 1) {
    return false;
  }
  const wire::core::SavedBackboneEdgeBundle& item = state.view().backbone().edge_bundles.front();
  return item.edge_bundle_id == edge_bundle_id &&
         item.span_ids.size() == first.value.generated_span_ids.size() + second_out.value.generated_span_ids.size();
}

bool C464_bb2_reverse_same_bundle_reuses_edge_bundle_metadata() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2 || state.view().backbone().edge_bundles.size() != 1) {
    return false;
  }
  const wire::core::SavedBackboneEdgeBundle before = state.view().backbone().edge_bundles.front();
  const wire::core::ObjectId a = first.value.generated_pole_ids[0];
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  wire::core::BackboneSpec second = line_req(state);
  second.path.polyline = {pb->world_transform.position, pa->world_transform.position};
  second.path.node_specs = {pole_spec(0, b), pole_spec(1, a)};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (!second_out.ok || state.view().backbone().edge_bundles.size() != 1) {
    return false;
  }
  const wire::core::SavedBackboneEdgeBundle& after = state.view().backbone().edge_bundles.front();
  return after.edge_bundle_id == before.edge_bundle_id && after.edge_forward == before.edge_forward &&
         after.route == before.route && after.order == before.order && almost_equal(after.dir.x, before.dir.x, 1e-9) &&
         almost_equal(after.dir.y, before.dir.y, 1e-9) && almost_equal(after.dir.z, before.dir.z, 1e-9) &&
         after.span_ids.size() == before.span_ids.size() + second_out.value.generated_span_ids.size();
}

bool C465_bb2_different_bundle_still_creates_edge_bundle() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2 || state.view().backbone().edge_bundles.size() != 1) {
    return false;
  }
  const wire::core::ObjectId a = first.value.generated_pole_ids[0];
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  wire::core::BackboneSpec second = line_req(state);
  second.bundles.clear();
  add_backbone_bundle(second, wire::core::BundleKind::kCommunication);
  second.path.polyline = {pa->world_transform.position, pb->world_transform.position};
  second.path.node_specs = {pole_spec(0, a), pole_spec(1, b)};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  return second_out.ok && state.view().backbone().edges.size() == 1 && state.view().backbone().edge_bundles.size() == 2;
}

void register_bb2_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C368_bb2_smoke_line", "bb2 generates the milestone-1 line slice", "Invariant", false,
                         C368_bb2_smoke_line);
  test_registry::AddTest(tests, "C369_bb2_rules_saved", "bb2 saves SpanLayoutRules for each generated span", "Invariant",
                         false, C369_bb2_rules_saved);
  test_registry::AddTest(tests, "C370_bb2_no_v1_deps", "bb2 source does not depend on v1 generation internals",
                         "Boundary", false, C370_bb2_no_v1_deps);
  test_registry::AddTest(tests, "C371_bb2_rejects_unsupported", "bb2 rejects unsupported milestone-1 inputs",
                         "Boundary", true, C371_bb2_rejects_unsupported);
  test_registry::AddTest(tests, "C372_bb2_rules_do_not_seed", "bb2 saves rules without creating authority seed",
                         "Boundary", false, C372_bb2_rules_do_not_seed);
  test_registry::AddTest(tests, "C373_bb2_layout_saved_without_recalc", "bb2 saves layout immediately from rules",
                         "Boundary", false, C373_bb2_layout_saved_without_recalc);
  test_registry::AddTest(tests, "C374_bb2_layout_is_deterministic", "bb2 derives stable layout from identical input",
                         "Invariant", false, C374_bb2_layout_is_deterministic);
  test_registry::AddTest(tests, "C375_bb2_curve_saved_without_recalc", "bb2 saves curve immediately from layout",
                         "Boundary", false, C375_bb2_curve_saved_without_recalc);
  test_registry::AddTest(tests, "C376_bb2_curve_is_deterministic", "bb2 derives stable curve from identical input",
                         "Invariant", false, C376_bb2_curve_is_deterministic);
  test_registry::AddTest(tests, "C377_bb2_bounds_saved_without_recalc", "bb2 saves bounds immediately from curve",
                         "Boundary", false, C377_bb2_bounds_saved_without_recalc);
  test_registry::AddTest(tests, "C378_bb2_bounds_is_deterministic", "bb2 derives stable bounds from identical input",
                         "Invariant", false, C378_bb2_bounds_is_deterministic);
  test_registry::AddTest(tests, "C379_bb2_m1_required_outputs", "bb2 milestone 1 required outputs are fixed",
                         "Boundary", false, C379_bb2_m1_required_outputs);
  test_registry::AddTest(tests, "C380_bb2_m1_draw_outputs_not_required", "bb2 milestone 1 does not require draw caches",
                         "Boundary", false, C380_bb2_m1_draw_outputs_not_required);
  test_registry::AddTest(tests, "C381_bb2_m1_no_recalc_contract", "bb2 milestone 1 has no recalc contract",
                         "Boundary", false, C381_bb2_m1_no_recalc_contract);
  test_registry::AddTest(tests, "C382_bb2_geom_is_single_pipeline_layer", "bb2 keeps geom as one pipeline layer",
                         "Boundary", false, C382_bb2_geom_is_single_pipeline_layer);
  test_registry::AddTest(tests, "C383_bb2_draw_is_declared_but_not_built", "bb2 declares draw without building it",
                         "Boundary", false, C383_bb2_draw_is_declared_but_not_built);
  test_registry::AddTest(tests, "C384_bb2_topo_is_single_output_layer", "bb2 keeps topology as one output layer",
                         "Boundary", false, C384_bb2_topo_is_single_output_layer);
  test_registry::AddTest(tests, "C385_bb2_emit_is_split_by_topology_parts", "bb2 splits topology emission by part",
                         "Boundary", false, C385_bb2_emit_is_split_by_topology_parts);
  test_registry::AddTest(tests, "C386_bb2_link_pair_row_are_separate", "bb2 separates links, pairs, opens, and rows",
                         "Boundary", false, C386_bb2_link_pair_row_are_separate);
  test_registry::AddTest(tests, "C387_bb2_pairs_are_single_source", "bb2 creates pairs from graph in one place",
                         "Boundary", false, C387_bb2_pairs_are_single_source);
  test_registry::AddTest(tests, "C388_bb2_polyline3_pair_model", "bb2 represents a three-point route as links, pair, and opens",
                         "Invariant", false, C388_bb2_polyline3_pair_model);
  test_registry::AddTest(tests, "C389_bb2_row_axis_owned_by_pairs", "bb2 port rows consume pair-owned axes",
                         "Boundary", false, C389_bb2_row_axis_owned_by_pairs);
  test_registry::AddTest(tests, "C390_bb2_rejects_reused_incident", "bb2 rejects invalid pair incident ownership",
                         "Boundary", true, C390_bb2_rejects_reused_incident);
  test_registry::AddTest(tests, "C391_bb2_no_kind_label", "bb2 does not add junction kind labels",
                         "Boundary", false, C391_bb2_no_kind_label);
  test_registry::AddTest(tests, "C392_bb2_polyline3_outputs", "bb2 saves required outputs for three-point routes",
                         "Invariant", false, C392_bb2_polyline3_outputs);
  test_registry::AddTest(tests, "C393_bb2_polyline3_is_deterministic", "bb2 derives deterministic three-point output",
                         "Invariant", false, C393_bb2_polyline3_is_deterministic);
  test_registry::AddTest(tests, "C394_bb2_existing_pole_node_is_not_recreated",
                         "bb2 uses existing pole nodes without recreating them", "Boundary", false,
                         C394_bb2_existing_pole_node_is_not_recreated);
  test_registry::AddTest(tests, "C395_bb2_is_new_does_not_affect_pairs",
                         "bb2 pair output does not change because a node is existing", "Invariant", false,
                         C395_bb2_is_new_does_not_affect_pairs);
  test_registry::AddTest(tests, "C396_bb2_existing_pole_does_not_read_existing_spans",
                         "bb2 existing pole nodes do not use existing spans for new meaning", "Boundary", false,
                         C396_bb2_existing_pole_does_not_read_existing_spans);
  test_registry::AddTest(tests, "C397_bb2_rejects_non_pole_node_spec", "bb2 rejects non-pole node specs", "Boundary",
                         true, C397_bb2_rejects_non_pole_node_spec);
  test_registry::AddTest(tests, "C398_bb2_rejects_missing_existing_pole", "bb2 rejects missing existing pole ids",
                         "Boundary", true, C398_bb2_rejects_missing_existing_pole);
  test_registry::AddTest(tests, "C399_bb2_existing_pole_sequence_is_deterministic",
                         "bb2 derives deterministic output with existing pole nodes", "Invariant", false,
                         C399_bb2_existing_pole_sequence_is_deterministic);
  test_registry::AddTest(tests, "C400_bb2_multiple_bundles_smoke", "bb2 generates multiple bundles on one pairs graph",
                         "Invariant", false, C400_bb2_multiple_bundles_smoke);
  test_registry::AddTest(tests, "C401_bb2_multiple_bundles_polyline3_outputs",
                         "bb2 saves required outputs for multiple bundles on a three-point route", "Invariant", false,
                         C401_bb2_multiple_bundles_polyline3_outputs);
  test_registry::AddTest(tests, "C402_bb2_bundle_spec_does_not_affect_pairs",
                         "bb2 bundle specs do not alter graph pair output", "Boundary", false,
                         C402_bb2_bundle_spec_does_not_affect_pairs);
  test_registry::AddTest(tests, "C403_bb2_existing_pole_with_multiple_bundles",
                         "bb2 combines existing pole nodes with multiple bundles", "Invariant", false,
                         C403_bb2_existing_pole_with_multiple_bundles);
  test_registry::AddTest(tests, "C404_bb2_rejects_empty_bundles", "bb2 rejects empty bundle requests", "Boundary", true,
                         C404_bb2_rejects_empty_bundles);
  test_registry::AddTest(tests, "C405_bb2_no_bundle_pair_branching", "bb2 pair building does not inspect bundle specs",
                         "Boundary", false, C405_bb2_no_bundle_pair_branching);
  test_registry::AddTest(tests, "C406_bb2_port_height_uses_pole_band", "bb2 port height comes from pole bands",
                         "Invariant", false, C406_bb2_port_height_uses_pole_band);
  test_registry::AddTest(tests, "C407_bb2_multiple_bundle_heights_are_band_based",
                         "bb2 multiple bundle heights come from pole bands", "Invariant", false,
                         C407_bb2_multiple_bundle_heights_are_band_based);
  test_registry::AddTest(tests, "C408_bb2_existing_pole_uses_actual_pole_type_height",
                         "bb2 existing pole height uses the actual pole type", "Invariant", false,
                         C408_bb2_existing_pole_uses_actual_pole_type_height);
  test_registry::AddTest(tests, "C409_bb2_rejects_missing_port_band", "bb2 rejects missing port bands", "Boundary",
                         true, C409_bb2_rejects_missing_port_band);
  test_registry::AddTest(tests, "C410_bb2_height_does_not_affect_pairs",
                         "bb2 height selection does not affect pairs", "Boundary", false,
                         C410_bb2_height_does_not_affect_pairs);
  test_registry::AddTest(tests, "C411_bb2_lateral_offset_moves_ports_along_row_axis",
                         "bb2 lateral offset moves generated output along row axis", "Invariant", false,
                         C411_bb2_lateral_offset_moves_ports_along_row_axis);
  test_registry::AddTest(tests, "C412_bb2_lateral_offset_sign_is_deterministic",
                         "bb2 lateral offset sign is deterministic", "Invariant", false,
                         C412_bb2_lateral_offset_sign_is_deterministic);
  test_registry::AddTest(tests, "C413_bb2_lateral_offset_does_not_affect_pairs",
                         "bb2 lateral offset does not affect pairs", "Boundary", false,
                         C413_bb2_lateral_offset_does_not_affect_pairs);
  test_registry::AddTest(tests, "C414_bb2_still_rejects_avoid_constraints",
                         "bb2 still rejects avoid constraints", "Boundary", true,
                         C414_bb2_still_rejects_avoid_constraints);
  test_registry::AddTest(tests, "C415_bb2_has_no_empty_levels_layer",
                         "bb2 has no empty levels layer", "Boundary", false,
                         C415_bb2_has_no_empty_levels_layer);
  test_registry::AddTest(tests, "C416_bb2_node_mode_not_present_is_noop",
                         "bb2 accepts kNotPresent node modes as no-op", "Invariant", false,
                         C416_bb2_node_mode_not_present_is_noop);
  test_registry::AddTest(tests, "C417_bb2_node_mode_pass_through_rejected",
                         "bb2 rejects pass-through node modes", "Boundary", true,
                         C417_bb2_node_mode_pass_through_rejected);
  test_registry::AddTest(tests, "C418_bb2_node_mode_unknown_bundle_rejected",
                         "bb2 rejects node modes for bundles absent from the request", "Boundary", true,
                         C418_bb2_node_mode_unknown_bundle_rejected);
  test_registry::AddTest(tests, "C419_bb2_node_mode_point_index_rejected",
                         "bb2 rejects out-of-range node mode point indices", "Boundary", true,
                         C419_bb2_node_mode_point_index_rejected);
  test_registry::AddTest(tests, "C420_bb2_node_mode_does_not_affect_pairs",
                         "bb2 node modes do not affect pairs", "Boundary", false,
                         C420_bb2_node_mode_does_not_affect_pairs);
  test_registry::AddTest(tests, "C421_bb2_topo_row_carries_source",
                         "bb2 topology rows carry pair row source", "Boundary", false,
                         C421_bb2_topo_row_carries_source);
  test_registry::AddTest(tests, "C422_bb2_rules_consume_topo_only",
                         "bb2 rules consume topology without pairs", "Boundary", false,
                         C422_bb2_rules_consume_topo_only);
  test_registry::AddTest(tests, "C423_bb2_tspan_carries_endpoint_rows",
                         "bb2 topology spans carry endpoint row indices", "Boundary", false,
                         C423_bb2_tspan_carries_endpoint_rows);
  test_registry::AddTest(tests, "C424_bb2_saves_backbone_graph_nodes_edges",
                         "bb2 saves backbone graph nodes and edges", "Boundary", false,
                         C424_bb2_saves_backbone_graph_nodes_edges);
  test_registry::AddTest(tests, "C425_bb2_edge_carries_multiple_spans",
                         "bb2 backbone edge carries generated spans", "Boundary", false,
                         C425_bb2_edge_carries_multiple_spans);
  test_registry::AddTest(tests, "C426_bb2_existing_pole_reuses_graph_node",
                         "bb2 reuses saved graph node for existing poles", "Boundary", false,
                         C426_bb2_existing_pole_reuses_graph_node);
  test_registry::AddTest(tests, "C427_bb2_graph_index_links_outputs",
                         "bb2 backbone index links graph to outputs", "Boundary", false,
                         C427_bb2_graph_index_links_outputs);
  test_registry::AddTest(tests, "C428_bb2_pole_frontier_collects_incident_graph",
                         "bb2 pole frontier collects saved graph incidents", "Boundary", false,
                         C428_bb2_pole_frontier_collects_incident_graph);
  test_registry::AddTest(tests, "C429_bb2_span_frontier_collects_edge_bundle_spans",
                         "bb2 span frontier collects spans from its backbone edge bundle", "Boundary", false,
                         C429_bb2_span_frontier_collects_edge_bundle_spans);
  test_registry::AddTest(tests, "C430_bb2_frontier_uses_saved_graph_index",
                         "bb2 frontier reads saved graph indexes", "Boundary", false,
                         C430_bb2_frontier_uses_saved_graph_index);
  test_registry::AddTest(tests, "C431_bb2_edge_bundle_is_saved_backbone_unit",
                         "bb2 stores edge bundle as the saved backbone unit", "Boundary", false,
                         C431_bb2_edge_bundle_is_saved_backbone_unit);
  test_registry::AddTest(tests, "C432_bb2_multiple_bundles_create_multiple_edge_bundles",
                         "bb2 multiple bundles create multiple saved edge bundles", "Boundary", false,
                         C432_bb2_multiple_bundles_create_multiple_edge_bundles);
  test_registry::AddTest(tests, "C433_bb2_reuses_edge_for_same_poles",
                         "bb2 reuses saved edge for the same pole pair", "Boundary", false,
                         C433_bb2_reuses_edge_for_same_poles);
  test_registry::AddTest(tests, "C434_bb2_reverse_generation_reuses_edge_bundle",
                         "bb2 reverse generation reuses the same edge bundle", "Boundary", false,
                         C434_bb2_reverse_generation_reuses_edge_bundle);
  test_registry::AddTest(tests, "C435_bb2_edge_metadata_is_not_overwritten_on_reuse",
                         "bb2 saved edge metadata is not overwritten on reuse", "Boundary", false,
                         C435_bb2_edge_metadata_is_not_overwritten_on_reuse);
  test_registry::AddTest(tests, "C436_bb2_frontier_reads_edge_bundles",
                         "bb2 frontier reads edge bundles", "Boundary", false,
                         C436_bb2_frontier_reads_edge_bundles);
  test_registry::AddTest(tests, "C437_bb2_layout_save_is_direct",
                         "bb2 layout save uses direct layout storage", "Boundary", false,
                         C437_bb2_layout_save_is_direct);
  test_registry::AddTest(tests, "C438_bb2_layout_save_keeps_no_authority_contract",
                         "bb2 direct layout save keeps authority contract empty", "Boundary", false,
                         C438_bb2_layout_save_keeps_no_authority_contract);
  test_registry::AddTest(tests, "C439_bb2_source_still_avoids_support_layout_entrypoint",
                         "bb2 source avoids support layout entrypoints", "Boundary", false,
                         C439_bb2_source_still_avoids_support_layout_entrypoint);
  test_registry::AddTest(tests, "C440_bb2_does_not_read_authoritative_backbone_directly",
                         "bb2 does not read authoritative backbone directly", "Boundary", false,
                         C440_bb2_does_not_read_authoritative_backbone_directly);
  test_registry::AddTest(tests, "C441_bb2_save_backbone_edge_returns_saved_ref",
                         "bb2 backbone edge save returns saved edge ref", "Boundary", false,
                         C441_bb2_save_backbone_edge_returns_saved_ref);
  test_registry::AddTest(tests, "C442_bb2_edge_forward_uses_saved_ref",
                         "bb2 edge forward uses saved edge ref", "Boundary", false,
                         C442_bb2_edge_forward_uses_saved_ref);
  test_registry::AddTest(tests, "C443_bb2_edge_reuse_behavior_unchanged",
                         "bb2 edge reuse behavior remains unchanged", "Boundary", false,
                         C443_bb2_edge_reuse_behavior_unchanged);
  test_registry::AddTest(tests, "C444_bb2_layout_uses_neutral_types",
                         "bb2 layout source uses neutral layout types", "Boundary", false,
                         C444_bb2_layout_uses_neutral_types);
  test_registry::AddTest(tests, "C445_bb2_cache_span_layout_accepts_neutral_entry",
                         "bb2 layout save accepts neutral layout entries", "Boundary", false,
                         C445_bb2_cache_span_layout_accepts_neutral_entry);
  test_registry::AddTest(tests, "C446_bb2_layout_boundary_behavior_unchanged",
                         "bb2 neutral layout boundary keeps generated outputs", "Boundary", false,
                         C446_bb2_layout_boundary_behavior_unchanged);
  test_registry::AddTest(tests, "C447_bb2_span_layout_view_reads_neutral_layout",
                         "bb2 reads generated layout through neutral view", "Boundary", false,
                         C447_bb2_span_layout_view_reads_neutral_layout);
  test_registry::AddTest(tests, "C448_bb2_tests_use_neutral_layout_read",
                         "bb2 tests use neutral layout reads", "Boundary", false,
                         C448_bb2_tests_use_neutral_layout_read);
  test_registry::AddTest(tests, "C449_bb2_layout_read_does_not_expose_authority",
                         "bb2 neutral layout read does not expose authority", "Boundary", false,
                         C449_bb2_layout_read_does_not_expose_authority);
  test_registry::AddTest(tests, "C450_bb2_span_layout_state_is_neutral",
                         "bb2 layout state observes outputs without old contract", "Boundary", false,
                         C450_bb2_span_layout_state_is_neutral);
  test_registry::AddTest(tests, "C451_bb2_tests_do_not_read_support_layout_contract",
                         "bb2 tests do not read support layout contract", "Boundary", false,
                         C451_bb2_tests_do_not_read_old_contract);
  test_registry::AddTest(tests, "C452_bb2_layout_state_does_not_expose_old_contract_names",
                         "bb2 layout state does not expose old contract names", "Boundary", false,
                         C452_bb2_layout_state_does_not_expose_old_contract_names);
  test_registry::AddTest(tests, "C453_bb2_layout_state_reads_existing_cache_without_seed_path",
                         "bb2 layout state read avoids seed path", "Boundary", false,
                         C453_bb2_layout_state_reads_existing_cache_without_seed_path);
  test_registry::AddTest(tests, "C454_bb2_cache_state_uses_span_layout_cache",
                         "bb2 cache state uses span layout cache owner name", "Boundary", false,
                         C454_bb2_cache_state_uses_span_layout_cache);
  test_registry::AddTest(tests, "C455_bb2_neutral_layout_api_uses_span_layout_cache",
                         "bb2 neutral layout API uses span layout cache", "Boundary", false,
                         C455_bb2_neutral_layout_api_uses_span_layout_cache);
  test_registry::AddTest(tests, "C456_bb2_source_avoids_old_layout_cache_names",
                         "bb2 source avoids old layout cache names", "Boundary", false,
                         C456_bb2_source_avoids_old_layout_cache_names);
  test_registry::AddTest(tests, "C457_bb2_layout_cache_boundary_behavior_unchanged",
                         "bb2 layout cache boundary keeps generated outputs", "Boundary", false,
                         C457_bb2_layout_cache_boundary_behavior_unchanged);
  test_registry::AddTest(tests, "C458_bb2_existing_branch_BD_on_ABC",
                         "bb2 adds an existing-pole branch using saved graph context", "Boundary", false,
                         C458_bb2_existing_branch_BD_on_ABC);
  test_registry::AddTest(tests, "C459_bb2_existing_cross_DBE_on_ABC",
                         "bb2 adds an existing-pole crossing route without kind labels", "Boundary", false,
                         C459_bb2_existing_cross_DBE_on_ABC);
  test_registry::AddTest(tests, "C460_bb2_context_links_are_not_emitted",
                         "bb2 saved graph context links are not emitted as new topology", "Boundary", false,
                         C460_bb2_context_links_are_not_emitted);
  test_registry::AddTest(tests, "C461_bb2_same_edge_request_skips_duplicate_context",
                         "bb2 skips duplicate saved context for same-edge requests", "Boundary", false,
                         C461_bb2_same_edge_request_skips_duplicate_context);
  test_registry::AddTest(tests, "C462_bb2_no_junction_kind_after_existing_context",
                         "bb2 existing graph context does not add junction kind labels", "Boundary", false,
                         C462_bb2_no_junction_kind_after_existing_context);
  test_registry::AddTest(tests, "C463_bb2_same_edge_bundle_reuses_edge_bundle",
                         "bb2 same edge and bundle reuses saved edge bundle", "Boundary", false,
                         C463_bb2_same_edge_bundle_reuses_edge_bundle);
  test_registry::AddTest(tests, "C464_bb2_reverse_same_bundle_reuses_edge_bundle_metadata",
                         "bb2 reverse same-bundle generation keeps edge bundle metadata", "Boundary", false,
                         C464_bb2_reverse_same_bundle_reuses_edge_bundle_metadata);
  test_registry::AddTest(tests, "C465_bb2_different_bundle_still_creates_edge_bundle",
                         "bb2 different bundles still create distinct edge bundles", "Boundary", false,
                         C465_bb2_different_bundle_still_creates_edge_bundle);
}

WIRE_REGISTER_TEST_SUITE(register_bb2_tests);

} // namespace
