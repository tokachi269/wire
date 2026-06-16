#include "registry.hpp"
#include "helpers.hpp"

#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
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

bool C380_bb2_m1_draw_outputs_saved() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (state.find_span_visual_cache(span_id) == nullptr) {
      return false;
    }
    if (state.view().find_span_render_cache(span_id) == nullptr) {
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

bool C383_bb2_draw_is_pipeline_layer() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "bb2";
  bool has_draw = false;
  bool has_make = false;
  bool has_save = false;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    has_draw = has_draw || contains_text(text, "struct draw");
    has_make = has_make || contains_text(text, "draw make(const layout& placed, const geom& shaped) const");
    has_save = has_save || contains_text(text, "void save(draw made)");
    if (contains_text(text, "visual_cache") || contains_text(text, "render_cache")) {
      return false;
    }
  }
  return has_draw && has_make && has_save;
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

bool C390_bb2_rejects_already_used_incident() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text) || !contains_text(text, "incident already used")) {
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
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t make_pos = cpp.find("EditResult<pairs> pipeline::make");
  const std::size_t check_pos = cpp.find("EditResult<intent> pipeline::make", make_pos);
  if (make_pos == std::string::npos || check_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(make_pos, check_pos - make_pos);
  return !contains_text(body, ".is_new");
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
  const std::size_t end = text.find("EditResult<intent> pipeline::make", start);
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
  wire::core::BackboneSpec first = line_req(state);
  first.pole_type_id = existing_type;
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok || first_out.value.generated_pole_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId pole = first_out.value.generated_pole_ids.front();
  const auto* existing_pole = state.view().poles().find(pole);
  if (existing_pole == nullptr) {
    return false;
  }
  req.path.polyline = {existing_pole->world_transform.position, {0.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = pole;
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
      if (port->owner_pole_id == pole) {
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
  const std::size_t end = text.find("EditResult<intent> pipeline::make", start);
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
  const std::size_t end = text.find("EditResult<intent> pipeline::make", start);
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
  const std::size_t end = text.find("EditResult<intent> pipeline::make", start);
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

bool C422_bb2_rules_consume_topo_and_groups() {
  const std::filesystem::path header = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.hpp";
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string h;
  std::string cpp;
  if (!file_text(header, &h) || !file_text(source, &cpp)) {
    return false;
  }
  if (!contains_text(h, "rules make(const topo& made, const groups& placement) const") ||
      contains_text(h, "rules make(const topo& made, const pairs& ps) const")) {
    return false;
  }
  const std::size_t rules_pos = cpp.find("rules pipeline::make(const topo& made, const groups& placement) const");
  const std::size_t layout_pos = cpp.find("EditResult<layout> pipeline::make", rules_pos);
  if (rules_pos == std::string::npos || layout_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(rules_pos, layout_pos - rules_pos);
  return contains_text(body, "group_for") && !contains_text(body, "ps.links");
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

bool C426_bb2_existing_pole_resolves_graph_node() {
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

bool C433_bb2_resolves_edge_for_same_poles() {
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

bool C434_bb2_reverse_duplicate_same_bundle_rejected() {
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
  return !second_out.ok && contains_text(second_out.error, "duplicate saved span binding") &&
         state.view().backbone().edges.size() == 1 && state.view().backbone().edge_bundles.size() == 1 &&
         state.view().backbone().edge_bundles.front().span_ids.size() == first.value.generated_span_ids.size();
}

bool C435_bb2_edge_metadata_is_not_overwritten_on_duplicate_reject() {
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
  if (second_out.ok || !contains_text(second_out.error, "duplicate saved span binding") ||
      state.view().backbone().edges.size() != 1) {
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
  const std::size_t fn_pos = cpp.find("EditResult<bool> pipeline::save_graph");
  const std::size_t build_pos = cpp.find("EditResult<GenerateBundleFromPathResult> pipeline::build", fn_pos);
  if (fn_pos == std::string::npos || build_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, build_pos - fn_pos);
  return contains_text(body, "std::vector<SavedBackboneEdgeRef>") && contains_text(body, "stored.node_a") &&
         contains_text(body, "stored.node_b") && !contains_text(body, "authoritative_.backbone") &&
         !contains_text(body, "saved_edge") && !contains_text(body, "find_if");
}

bool C443_bb2_edge_resolution_behavior_unchanged() {
  return C434_bb2_reverse_duplicate_same_bundle_rejected();
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

bool C463_bb2_duplicate_same_edge_bundle_rejected() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2 || state.view().backbone().edge_bundles.size() != 1) {
    return false;
  }
  const std::size_t edge_count = state.view().backbone().edges.size();
  const std::size_t edge_bundle_count = state.view().backbone().edge_bundles.size();
  const std::size_t span_count = state.view().spans().size();
  const std::size_t saved_span_count = state.view().backbone().edge_bundles.front().span_ids.size();
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
  return !second_out.ok && contains_text(second_out.error, "duplicate saved span binding") &&
         state.view().backbone().edges.size() == edge_count &&
         state.view().backbone().edge_bundles.size() == edge_bundle_count && state.view().spans().size() == span_count &&
         state.view().backbone().edge_bundles.front().span_ids.size() == saved_span_count;
}

bool C464_bb2_different_bundle_on_same_edge_allowed() {
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

bool C465_bb2_duplicate_policy_does_not_read_existing_spans() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("EditResult<bool> pipeline::check(const pairs& ps) const");
  const std::size_t next_pos = cpp.find("EditResult<bool> pipeline::emit_bundles", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "backbone_index().edge_bundles") && contains_text(body, "backbone_edge_bundle") &&
         !contains_text(body, ".spans") && !contains_text(body, "span_layout") && !contains_text(body, "seed") &&
         !contains_text(body, "layout");
}

bool C466_bb2_duplicate_reject_keeps_state_unchanged() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2 || state.view().backbone().edge_bundles.size() != 1) {
    return false;
  }
  const std::size_t pole_count = state.view().poles().size();
  const std::size_t port_count = state.view().ports().size();
  const std::size_t bundle_count_before = state.view().bundles().size();
  const std::size_t span_count = state.view().spans().size();
  const std::size_t edge_count = state.view().backbone().edges.size();
  const std::size_t edge_bundle_count = state.view().backbone().edge_bundles.size();
  const std::size_t saved_span_count = state.view().backbone().edge_bundles.front().span_ids.size();
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
  const wire::core::BackboneFrontier frontier = state.view().pole_frontier(a);
  return !second_out.ok && contains_text(second_out.error, "duplicate saved span binding") &&
         state.view().poles().size() == pole_count && state.view().ports().size() == port_count &&
         state.view().bundles().size() == bundle_count_before && state.view().spans().size() == span_count &&
         state.view().backbone().edges.size() == edge_count &&
         state.view().backbone().edge_bundles.size() == edge_bundle_count &&
         state.view().backbone().edge_bundles.front().span_ids.size() == saved_span_count &&
         frontier.edge_ids.size() == 1 && frontier.edge_bundle_ids.size() == 1 &&
         frontier.span_ids.size() == saved_span_count;
}

bool C467_bb2_saves_row_port_bindings() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  if (!out.ok || graph.edge_bundles.size() != 1 ||
      graph.port_bindings.size() != out.value.generated_span_ids.size() * 2) {
    return false;
  }
  const wire::core::ObjectId edge_bundle_id = graph.edge_bundles.front().edge_bundle_id;
  const std::vector<const wire::core::SavedBackbonePortBinding*> by_edge_bundle =
      state.view().backbone_port_bindings_for_edge_bundle(edge_bundle_id);
  if (by_edge_bundle.size() != graph.port_bindings.size()) {
    return false;
  }
  for (const wire::core::SavedBackbonePortBinding& binding : graph.port_bindings) {
    const wire::core::SavedBackbonePortBinding* by_port = state.view().backbone_port_binding_for_port(binding.port_id);
    if (binding.edge_bundle_id != edge_bundle_id || binding.row_key.node_id == wire::core::kInvalidObjectId ||
        binding.row_key.source_edge_a == wire::core::kInvalidObjectId ||
        state.view().ports().find(binding.port_id) == nullptr || by_port == nullptr ||
        by_port->port_id != binding.port_id) {
      return false;
    }
  }
  return true;
}

bool C468_bb2_row_port_binding_is_stable_for_existing_context() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const std::size_t before = state.view().backbone().port_bindings.size();
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  return second.ok && state.view().backbone().port_bindings.size() == before + second.value.generated_span_ids.size() * 2;
}

bool C469_bb2_row_port_binding_rejects_duplicate_without_resolution() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const std::size_t before = state.view().backbone().port_bindings.size();
  const wire::core::ObjectId a = first.value.generated_pole_ids[0];
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  wire::core::BackboneSpec duplicate = line_req(state);
  duplicate.path.polyline = {pa->world_transform.position, pb->world_transform.position};
  duplicate.path.node_specs = {pole_spec(0, a), pole_spec(1, b)};
  const auto second = state.GenerateFromBackboneSpec(duplicate);
  return !second.ok && contains_text(second.error, "duplicate saved span binding") &&
         state.view().backbone().port_bindings.size() == before;
}

bool C470_bb2_row_port_identity_does_not_use_position_match() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "state" / "backbone.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("EditResult<bool> CoreState::bind_backbone_port");
  const std::size_t next_pos = cpp.find("PoleDetailInfo CoreState::GetPoleDetail", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "edge_bundle_id") && contains_text(body, "row_key") &&
         !contains_text(body, "world_position") && !contains_text(body, "span_layout") &&
         !contains_text(body, "seed") && !contains_text(body, "layout") && !contains_text(body, "position");
}

bool C471_bb2_resolves_existing_port_by_binding() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  std::vector<wire::core::ObjectId> middle_ports{};
  for (wire::core::ObjectId span_id : first.value.generated_span_ids) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    const wire::core::Port* a = state.view().ports().find(span->port_a_id);
    const wire::core::Port* c = state.view().ports().find(span->port_b_id);
    if (a != nullptr && a->owner_pole_id == b) {
      middle_ports.push_back(a->id);
    }
    if (c != nullptr && c->owner_pole_id == b) {
      middle_ports.push_back(c->id);
    }
  }
  for (wire::core::ObjectId port_id : middle_ports) {
    if (std::count(middle_ports.begin(), middle_ports.end(), port_id) < 2) {
      continue;
    }
    const wire::core::Port* port = state.view().ports().find(port_id);
    const std::vector<const wire::core::SavedBackbonePortBinding*> bindings =
        state.view().backbone_port_bindings_for_port(port_id);
    if (port == nullptr || bindings.size() < 2) {
      return false;
    }
    for (const wire::core::SavedBackbonePortBinding* binding : bindings) {
      if (binding == nullptr || binding->bundle_template_id != bindings.front()->bundle_template_id ||
          binding->port_kind != port->kind || binding->port_layer != port->layer ||
          binding->port_kind != bindings.front()->port_kind || binding->port_layer != bindings.front()->port_layer) {
        return false;
      }
    }
    return true;
  }
  return false;
}

bool C487_bb2_port_resolution_requires_bundle_compatible_scope() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  std::vector<wire::core::ObjectId> first_ports{};
  for (wire::core::ObjectId span_id : first.value.generated_span_ids) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    first_ports.push_back(span->port_a_id);
    first_ports.push_back(span->port_b_id);
  }
  const std::size_t port_count = state.view().ports().size();
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
  const auto out = state.GenerateFromBackboneSpec(second);
  if (!out.ok || out.value.generated_span_ids.empty() || state.view().ports().size() <= port_count) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    if (span == nullptr || contains_id(first_ports, span->port_a_id) || contains_id(first_ports, span->port_b_id)) {
      return false;
    }
  }
  return true;
}

bool C472_bb2_port_resolution_requires_saved_binding() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto manual_a =
      state.AddPort(b, pole_b->world_transform.position + wire::core::Vec3d{0.0, 0.0, 9.2},
                    wire::core::PortKind::kPower, wire::core::PortLayer::kLowVoltage);
  if (!manual_a.ok) {
    return false;
  }
  const std::size_t before = state.view().ports().size();
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || state.view().ports().size() <= before) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    if (span == nullptr || span->port_a_id == manual_a.value || span->port_b_id == manual_a.value) {
      return false;
    }
  }
  return true;
}

bool C473_bb2_resolved_port_used_by_new_span_endpoint() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  std::vector<wire::core::ObjectId> middle_ports{};
  for (wire::core::ObjectId span_id : first.value.generated_span_ids) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    const wire::core::Port* a = state.view().ports().find(span->port_a_id);
    const wire::core::Port* c = state.view().ports().find(span->port_b_id);
    if (a != nullptr && a->owner_pole_id == b) {
      middle_ports.push_back(a->id);
    }
    if (c != nullptr && c->owner_pole_id == b) {
      middle_ports.push_back(c->id);
    }
  }
  for (wire::core::ObjectId port_id : middle_ports) {
    if (std::count(middle_ports.begin(), middle_ports.end(), port_id) >= 2) {
      return true;
    }
  }
  return false;
}

bool C474_bb2_port_resolution_rejects_ambiguous_binding() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("EditResult<ObjectId> resolve_port_binding");
  const std::size_t next_pos = cpp.find("double yaw", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "ambiguous backbone port binding") && contains_text(body, "found != kInvalidObjectId") &&
         contains_text(body, "found != port->id");
}

bool C475_bb2_port_resolution_does_not_read_existing_layout() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("EditResult<ObjectId> resolve_port_binding");
  const std::size_t next_pos = cpp.find("double yaw", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "backbone_port_bindings_for_row") && !contains_text(body, "span_layout") &&
         !contains_text(body, "support_layout") && !contains_text(body, "seed") &&
         !contains_text(body, "world_position") && !contains_text(body, "position");
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

bool C476_bb2_branch_rows_are_separated_without_branch_kind() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const std::vector<wire::core::Vec3d> before = pole_port_positions(state, b);
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  const std::vector<wire::core::Vec3d> placed = generated_ports_on_pole(state, second.value.generated_span_ids, b);
  return second.ok && separated_from(before, placed) && C391_bb2_no_kind_label();
}

bool C477_bb2_cross_rows_are_separated_without_cross_kind() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const std::vector<wire::core::Vec3d> before = pole_port_positions(state, b);
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  wire::core::BackboneSpec cross = line_req(state);
  cross.path.polyline = {{12.0, -8.0, 0.0}, pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  cross.path.node_specs = {pole_spec(1, b)};
  const auto second = state.GenerateFromBackboneSpec(cross);
  const std::vector<wire::core::Vec3d> placed = generated_ports_on_pole(state, second.value.generated_span_ids, b);
  return second.ok && separated_from(before, placed) && C391_bb2_no_kind_label();
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

bool C478_bb2_row_separation_is_deterministic() {
  const std::vector<wire::core::Vec3d> a = branch_separation_points();
  const std::vector<wire::core::Vec3d> b = branch_separation_points();
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

bool C479_bb2_row_separation_does_not_change_pairs() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t make_pos = cpp.find("EditResult<pairs> pipeline::make");
  const std::size_t check_pos = cpp.find("EditResult<bool> pipeline::emit_poles", make_pos);
  if (make_pos == std::string::npos || check_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(make_pos, check_pos - make_pos);
  return !contains_text(body, "row_shifts") && !contains_text(body, "kRowSeparationM");
}

bool C480_bb2_context_rows_affect_order_but_are_not_emitted() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
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
  if (!second.ok) {
    return false;
  }
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  return !second.value.generated_span_ids.empty() && contains_text(cpp, "row_shifts(ps)") &&
         contains_text(cpp, "if (r.id >= active_rows.size() || !active_rows[r.id])");
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

bool C481_bb2_pass_through_mode_is_accepted_in_limited_scope() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto ok = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));

  wire::core::BackboneSpec bad_index = pass_branch_req(state, b, pole_b->world_transform.position);
  bad_index.node_bundle_modes.front().point_index = 9;
  const auto bad_index_out = state.GenerateFromBackboneSpec(bad_index);

  wire::core::BackboneSpec bad_bundle = pass_branch_req(state, b, pole_b->world_transform.position);
  bad_bundle.node_bundle_modes.front().bundle_template_id = static_cast<wire::core::BundleKind>(999);
  const auto bad_bundle_out = state.GenerateFromBackboneSpec(bad_bundle);
  return ok.ok && !bad_index_out.ok && !bad_bundle_out.ok;
}

bool C482_bb2_pass_through_creates_explicit_intent() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  bool saw_intent = false;
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::SpanLayoutRulesView rules = state.span_layout_rules(span_id);
    const wire::core::SpanLayoutView layout = state.span_layout(span_id);
    if (!rules.has_rule() || !layout.has_layout()) {
      return false;
    }
    saw_intent = saw_intent || rules.rule->lowering_kind == wire::core::BackboneLoweringKind::kBranchSupport ||
                 rules.rule->start.default_lower_required || rules.rule->end.default_lower_required ||
                 layout.entry->start.default_lower_required || layout.entry->end.default_lower_required;
  }
  return saw_intent && C479_bb2_row_separation_does_not_change_pairs();
}

bool C483_bb2_pass_through_ambiguous_target_rejected() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("EditResult<intent> pipeline::make(const pairs& ps) const");
  const std::size_t next_pos = cpp.find("EditResult<bool> pipeline::emit_poles", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "matches.size() != 1") &&
         contains_text(body, "pass-through target row is ambiguous");
}

bool C484_bb2_lowering_draw_uses_layout_only() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  bool saw_lowered_visual = false;
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    const wire::core::SpanRenderCacheEntry* render = state.find_span_render_cache(span_id);
    if (visual == nullptr || render == nullptr) {
      return false;
    }
    saw_lowered_visual = saw_lowered_visual || !visual->parts.empty();
  }
  return saw_lowered_visual;
}

bool C485_bb2_lowering_intent_does_not_read_existing_spans() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("EditResult<intent> pipeline::make(const pairs& ps) const");
  const std::size_t next_pos = cpp.find("EditResult<bool> pipeline::emit_poles", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "node_bundle_modes") && !contains_text(body, ".spans") &&
         !contains_text(body, "span_layout") && !contains_text(body, "seed") && !contains_text(body, "layout");
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

bool C486_bb2_pass_through_is_deterministic() {
  const std::vector<wire::core::Vec3d> a = pass_intent_points();
  const std::vector<wire::core::Vec3d> b = pass_intent_points();
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

bool C488_bb2_port_resolution_accepts_same_compatible_binding() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok || out.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = out.value.generated_pole_ids[1];
  for (const wire::core::Port& port : state.view().ports().items()) {
    if (port.owner_pole_id != b) {
      continue;
    }
    const std::vector<const wire::core::SavedBackbonePortBinding*> bindings =
        state.view().backbone_port_bindings_for_port(port.id);
    if (bindings.size() < 2) {
      continue;
    }
    const wire::core::SavedBackbonePortBinding* first = bindings.front();
    for (const wire::core::SavedBackbonePortBinding* binding : bindings) {
      if (binding == nullptr || binding->bundle_template_id != first->bundle_template_id ||
          binding->port_kind != first->port_kind || binding->port_layer != first->port_layer ||
          binding->port_kind != port.kind || binding->port_layer != port.layer) {
        return false;
      }
    }
    return true;
  }
  return false;
}

bool C489_bb2_port_binding_index_invariant() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok) {
    return false;
  }
  bool saw_multiple = false;
  for (const wire::core::Port& port : state.view().ports().items()) {
    const std::vector<const wire::core::SavedBackbonePortBinding*> bindings =
        state.view().backbone_port_bindings_for_port(port.id);
    if (bindings.size() < 2) {
      continue;
    }
    saw_multiple = true;
    const wire::core::SavedBackbonePortBinding* first = bindings.front();
    for (const wire::core::SavedBackbonePortBinding* binding : bindings) {
      if (binding == nullptr || binding->bundle_template_id != first->bundle_template_id ||
          binding->port_kind != first->port_kind || binding->port_layer != first->port_layer ||
          binding->port_kind != port.kind || binding->port_layer != port.layer) {
        return false;
      }
    }
  }
  return saw_multiple;
}

bool C490_bb2_duplicate_same_edge_bundle_lane_rejected() {
  return C463_bb2_duplicate_same_edge_bundle_rejected() && C466_bb2_duplicate_reject_keeps_state_unchanged();
}

bool C502_bb2_span_bindings_save_lane() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  if (!out.ok || graph.edge_bundles.size() != 1 || graph.span_bindings.size() != out.value.generated_span_ids.size()) {
    return false;
  }
  const wire::core::ObjectId edge_bundle_id = graph.edge_bundles.front().edge_bundle_id;
  const auto index_it = state.view().backbone_index().edge_bundle_span_bindings.find(edge_bundle_id);
  if (index_it == state.view().backbone_index().edge_bundle_span_bindings.end() ||
      index_it->second.size() != graph.span_bindings.size()) {
    return false;
  }
  std::unordered_set<std::size_t> lanes{};
  for (const wire::core::SavedBackboneSpanBinding& binding : graph.span_bindings) {
    if (binding.edge_bundle_id != edge_bundle_id || state.view().spans().find(binding.span_id) == nullptr) {
      return false;
    }
    if (!lanes.insert(binding.lane_index).second) {
      return false;
    }
    const auto by_span = state.view().backbone_index().span_bindings_by_span.find(binding.span_id);
    if (by_span == state.view().backbone_index().span_bindings_by_span.end() || by_span->second.empty()) {
      return false;
    }
  }
  return true;
}

bool C503_bb2_duplicate_span_binding_rejected_by_lane() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "state" / "backbone.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("EditResult<bool> CoreState::bind_backbone_span");
  const std::size_t next_pos = cpp.find("EditResult<bool> CoreState::bind_backbone_port", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "binding.lane_index == lane_index") &&
         contains_text(body, "duplicate backbone span binding") &&
         contains_text(body, "span_bindings_by_span");
}

bool C504_bb2_span_resolution_does_not_read_geometry_or_layout() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "state" / "backbone.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("EditResult<bool> CoreState::bind_backbone_span");
  const std::size_t next_pos = cpp.find("EditResult<bool> CoreState::bind_backbone_port", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "edge_bundle_span_bindings") && contains_text(body, "lane_index") &&
         !contains_text(body, "span_layout") && !contains_text(body, "seed") &&
         !contains_text(body, "find_curve_cache") && !contains_text(body, "world_position");
}

bool C505_bb2_save_graph_propagates_span_binding_failure() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("EditResult<bool> pipeline::save_graph");
  const std::size_t next_pos = cpp.find("EditResult<GenerateBundleFromPathResult> pipeline::build", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "bind_backbone_span(edge_bundle_id, span.lane, span.id)") &&
         contains_text(body, "span_bound.error");
}

bool C506_bb2_support_group_is_placement_layer() {
  const std::filesystem::path header = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.hpp";
  std::string h;
  if (!file_text(header, &h)) {
    return false;
  }
  return contains_text(h, "struct group_member") && contains_text(h, "struct group") &&
         contains_text(h, "std::vector<group_member> row_members") && contains_text(h, "Vec3d group_axis") &&
         contains_text(h, "int vertical_order") && contains_text(h, "double lower_offset_m");
}

bool C507_bb2_support_group_built_after_intent() {
  const std::filesystem::path header = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.hpp";
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string h;
  std::string cpp;
  if (!file_text(header, &h) || !file_text(source, &cpp)) {
    return false;
  }
  return contains_text(h, "EditResult<groups> make(const pairs& ps, const intent& intents) const") &&
         contains_text(h, "rules make(const topo& made, const groups& placement) const") &&
         !contains_text(h, "rules make(const topo& made, const intent& intents) const") &&
         contains_text(cpp, "EditResult<groups> placement = make(ps.value, intents.value)");
}

bool C508_bb2_support_group_drives_lowered_rules() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::SpanLayoutRulesView rules = state.span_layout_rules(span_id);
    if (!rules.has_rule()) {
      return false;
    }
    const auto has_lowered_group = [](const wire::core::EndpointLayoutRule& endpoint) {
      return endpoint.default_lower_required && endpoint.semantic.lower_required &&
             endpoint.semantic.support_group_id >= 0 && endpoint.branch_down_offset_m > 0.0;
    };
    if (has_lowered_group(rules.rule->start) || has_lowered_group(rules.rule->end)) {
      return true;
    }
  }
  return false;
}

bool C509_bb2_support_group_avoids_visual_terms() {
  const std::filesystem::path header = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.hpp";
  std::string h;
  if (!file_text(header, &h)) {
    return false;
  }
  const std::size_t group_pos = h.find("struct group {");
  const std::size_t next_pos = h.find("struct groups", group_pos);
  if (group_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = h.substr(group_pos, next_pos - group_pos);
  return !contains_text(body, "mount") && !contains_text(body, "tip") && !contains_text(body, "arm") &&
         !contains_text(body, "insulator") && !contains_text(body, "attachment");
}

bool C510_bb2_layout_consumes_group_offset() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("EditResult<layout> pipeline::make(const rules& made) const");
  const std::size_t next_pos = cpp.find("geom pipeline::make", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "rule.branch_down_offset_m") && contains_text(body, "target->endpoint_world.z -= lower_offset") &&
         !contains_text(body, "kLowerOffsetM");
}

bool C511_bb2_draw_saved_from_geom() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span_id);
    const wire::core::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    const wire::core::SpanRenderCacheEntry* render = state.find_span_render_cache(span_id);
    if (curve == nullptr || visual == nullptr || render == nullptr) {
      return false;
    }
    if (render->arc_length_m_by_point.size() != curve->detail.sample_points.size() ||
        render->arc_length_normalized_by_point.size() != curve->detail.sample_points.size()) {
      return false;
    }
    if (!curve->detail.sample_points.empty() && render->arc_length_m_by_point.empty()) {
      return false;
    }
    if (curve->detail.sample_points.size() >= 2 &&
        render->segment_length_m.size() + 1 != curve->detail.sample_points.size()) {
      return false;
    }
  }
  return true;
}

bool C512_bb2_draw_does_not_read_topology() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("draw pipeline::make(const layout& placed, const geom& shaped) const");
  const std::size_t next_pos = cpp.find("void pipeline::save(const rules& made)", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return !contains_text(body, "pairs") && !contains_text(body, "ps.") && !contains_text(body, "backbone") &&
         !contains_text(body, "node_bundle_modes") && !contains_text(body, "save_backbone") &&
         !contains_text(body, "bind_backbone");
}

bool C513_bb2_support_visual_placeholder_from_layout() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    const wire::core::SpanLayoutView layout = state.span_layout(span_id);
    if (visual == nullptr || !layout.has_layout()) {
      return false;
    }
    if (layout.entry->start.default_lower_required || layout.entry->end.default_lower_required) {
      return !visual->parts.empty();
    }
  }
  return false;
}

bool C514_bb2_draw_save_is_direct() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "recalc" / "recalc_pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t visual_pos = cpp.find("void CoreState::cache_span_visual");
  const std::size_t render_pos = cpp.find("void CoreState::cache_span_render");
  const std::size_t next_pos = cpp.find("void CoreState::cache_span_support_layout_seed", render_pos);
  if (visual_pos == std::string::npos || render_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(visual_pos, next_pos - visual_pos);
  return contains_text(body, "visual_cache.by_span") && contains_text(body, "render_cache.by_span") &&
         !contains_text(body, "rebuild_span_visual") && !contains_text(body, "rebuild_span_bounds") &&
         !contains_text(body, "cache_rebuilt_span_geometry") && !contains_text(body, "dirty");
}

bool C515_bb2_rejects_existing_pole_without_saved_graph() {
  wire::core::CoreState state;
  wire::core::Transformd tf{};
  tf.position = {0.0, 0.0, 0.0};
  const auto pole = state.AddPole(tf);
  if (!pole.ok) {
    return false;
  }
  const std::size_t pole_count = state.view().poles().size();
  const std::size_t span_count = state.view().spans().size();
  const std::size_t graph_nodes = state.view().backbone().nodes.size();
  const std::size_t graph_edges = state.view().backbone().edges.size();
  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {tf.position, {12.0, 0.0, 0.0}};
  req.path.node_specs = {pole_spec(0, pole.value)};
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "saved backbone graph missing") &&
         state.view().poles().size() == pole_count && state.view().spans().size() == span_count &&
         state.view().backbone().nodes.size() == graph_nodes && state.view().backbone().edges.size() == graph_edges;
}

bool C516_bb2_generated_pole_with_saved_graph_still_connects() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr || state.view().backbone_node_for_pole(b) == nullptr) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  return second.ok && !second.value.generated_span_ids.empty() && state.view().pole_frontier(b).edge_ids.size() == 3;
}

bool C517_bb2_migration_gate_does_not_infer_from_outputs() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("EditResult<bool> pipeline::prepare()");
  const std::size_t next_pos = cpp.find("EditResult<bool> pipeline::check() const", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "backbone_node_for_pole") &&
         contains_text(body, "saved backbone graph missing for existing pole") &&
         !contains_text(body, ".spans") && !contains_text(body, "span_layout") &&
         !contains_text(body, "find_curve_cache") && !contains_text(body, "find_bounds_cache") &&
         !contains_text(body, "save_backbone_node");
}

bool C518_bb2_lowered_layout_keeps_support_world_at_port_height() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::SpanLayoutView layout = state.span_layout(span_id);
    if (!layout.has_layout()) {
      return false;
    }
    const auto endpoint_ok = [&](const wire::core::LayoutEndpoint& endpoint) {
      if (!endpoint.default_lower_required && !endpoint.lower_required) {
        return false;
      }
      const wire::core::Port* port = state.view().ports().find(endpoint.port_id);
      if (port == nullptr) {
        return false;
      }
      const double lower_offset =
          endpoint.branch_down_offset_m > 0.0 ? endpoint.branch_down_offset_m : endpoint.automatic_branch_down_offset_m;
      return lower_offset > 0.0 && almost_equal(endpoint.support_world.z, port->world_position.z, 1e-9) &&
             almost_equal(endpoint.endpoint_world.z, port->world_position.z - lower_offset, 1e-9);
    };
    if (endpoint_ok(layout.entry->start) || endpoint_ok(layout.entry->end)) {
      return true;
    }
  }
  return false;
}

bool C519_bb2_draw_placeholder_uses_layout_points() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("draw pipeline::make(const layout& placed, const geom& shaped) const");
  const std::size_t next_pos = cpp.find("void pipeline::save(const rules& made)", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  if (contains_text(body, "branch_down_offset_m")) {
    return false;
  }

  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::SpanLayoutView layout = state.span_layout(span_id);
    const wire::core::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    if (!layout.has_layout() || visual == nullptr) {
      return false;
    }
    auto part_matches = [&](const wire::core::LayoutEndpoint& endpoint) {
      if (!endpoint.default_lower_required && !endpoint.lower_required) {
        return false;
      }
      for (const wire::core::VisualPart& part : visual->parts) {
        if (almost_equal(part.a, endpoint.support_world, 1e-9) &&
            almost_equal(part.b, endpoint.endpoint_world, 1e-9)) {
          return true;
        }
      }
      return false;
    };
    if (part_matches(layout.entry->start) || part_matches(layout.entry->end)) {
      return true;
    }
  }
  return false;
}

bool C520_bb2_duplicate_span_binding_preflight_before_emit() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t build_pos = cpp.find("EditResult<GenerateBundleFromPathResult> pipeline::build()");
  const std::size_t check_call = cpp.find("EditResult<bool> duplicates = check(ps.value)", build_pos);
  const std::size_t emit_call = cpp.find("EditResult<topo> made = emit(ps.value)", build_pos);
  if (build_pos == std::string::npos || check_call == std::string::npos || emit_call == std::string::npos ||
      check_call > emit_call) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("EditResult<bool> pipeline::check(const pairs& ps) const");
  const std::size_t next_pos = cpp.find("EditResult<bool> pipeline::emit_bundles", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "edge_bundle_span_bindings") && contains_text(body, "span_bindings") &&
         contains_text(body, "lane_index") && !contains_text(body, "AddPort") && !contains_text(body, "AddSpan") &&
         !contains_text(body, "AddBundle");
}

bool C521_bb2_context_link_preserves_saved_dir() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t prepare_pos = cpp.find("EditResult<bool> pipeline::prepare()");
  const std::size_t check_pos = cpp.find("EditResult<bool> pipeline::check() const", prepare_pos);
  const std::size_t make_pos = cpp.find("EditResult<pairs> pipeline::make(const graph& made) const");
  const std::size_t intent_pos = cpp.find("EditResult<intent> pipeline::make(const pairs& ps) const", make_pos);
  if (prepare_pos == std::string::npos || check_pos == std::string::npos || make_pos == std::string::npos ||
      intent_pos == std::string::npos) {
    return false;
  }
  const std::string prepare_body = cpp.substr(prepare_pos, check_pos - prepare_pos);
  const std::string make_body = cpp.substr(make_pos, intent_pos - make_pos);
  return contains_text(prepare_body, "edge.dir = g_.nodes[i + 1].pos - g_.nodes[i].pos") &&
         contains_text(prepare_body, "edge.dir = saved->dir") &&
         !contains_text(make_body, "edge.dir = made.nodes");
}

bool C522_bb2_supported_scope_is_documented() {
  const std::filesystem::path doc = repo_root() / "redesign.md";
  std::string text;
  if (!file_text(doc, &text)) {
    return false;
  }
  return contains_text(text, "## bb2 supported generation scope") && contains_text(text, "Supported:") &&
         contains_text(text, "Unsupported:") && contains_text(text, "`SavedBackboneGraph` is topology authority") &&
         contains_text(text, "`pairs make(graph)`") && contains_text(text, "duplicate same edge_bundle + lane") &&
         contains_text(text, "support_world") && contains_text(text, "endpoint_world") &&
         contains_text(text, "does not fall back to v1");
}

bool C523_bb2_scope_gate_matches_entrypoint() {
  const std::filesystem::path entry = repo_root() / "core" / "src" / "generation" / "generate_from_backbone.cpp";
  const std::filesystem::path bb2 = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string entry_text;
  std::string bb2_text;
  if (!file_text(entry, &entry_text) || !file_text(bb2, &bb2_text)) {
    return false;
  }
  const bool entry_uses_bb2 = contains_text(entry_text, "generation::bb2::pipeline") &&
                              contains_text(entry_text, "pipeline.prepare()") &&
                              contains_text(entry_text, "pipeline.check()") &&
                              contains_text(entry_text, "return pipeline.build();");
  const bool entry_avoids_v1 = !contains_text(entry_text, "BackbonePipeline") &&
                               !contains_text(entry_text, "backbone_pipeline") &&
                               !contains_text(entry_text, "generate_grouped_spans_between_support_nodes");
  const std::size_t build_pos = bb2_text.find("EditResult<GenerateBundleFromPathResult> pipeline::build()");
  const std::size_t check_call = bb2_text.find("EditResult<bool> duplicates = check(ps.value)", build_pos);
  const std::size_t intent_call = bb2_text.find("EditResult<intent> intents = make(ps.value)", build_pos);
  const std::size_t emit_call = bb2_text.find("EditResult<topo> made = emit(ps.value)", build_pos);
  const bool preflight_before_emit = build_pos != std::string::npos && check_call != std::string::npos &&
                                     intent_call != std::string::npos && emit_call != std::string::npos &&
                                     check_call < intent_call && intent_call < emit_call;
  return entry_uses_bb2 && entry_avoids_v1 && preflight_before_emit;
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

bool C491_bb2_branch_lowering_v1_affects_geom() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    if (span_has_lowered_endpoint(state, span_id)) {
      return true;
    }
  }
  return false;
}

bool C492_bb2_cross_lowering_v1_affects_only_new_links() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const std::size_t span_count = state.view().spans().size();
  const std::size_t port_count = state.view().ports().size();
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  wire::core::BackboneSpec cross = line_req(state);
  cross.path.polyline = {{12.0, -8.0, 0.0}, pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  cross.path.node_specs = {pole_spec(1, b)};
  cross.node_bundle_modes.push_back({1, cross.bundles.front().bundle_template_id,
                                     wire::core::BundleNodeMode::kPassThrough});
  const auto second = state.GenerateFromBackboneSpec(cross);
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  bool saw_lowered = false;
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    saw_lowered = saw_lowered || span_has_lowered_endpoint(state, span_id);
  }
  return saw_lowered && state.view().spans().size() == span_count + second.value.generated_span_ids.size() &&
         state.view().ports().size() > port_count;
}

bool C493_bb2_pass_through_does_not_change_pair_open() {
  return C420_bb2_node_mode_does_not_affect_pairs() && C479_bb2_row_separation_does_not_change_pairs();
}

bool C494_bb2_lowering_v1_draw_does_not_redecide() {
  return C484_bb2_lowering_draw_uses_layout_only();
}

bool C495_bb2_lowering_v1_does_not_read_existing_spans() {
  return C485_bb2_lowering_intent_does_not_read_existing_spans();
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

bool C496_bb2_junction_v1_deterministic() {
  const std::vector<wire::core::Vec3d> a = junction_v1_points();
  const std::vector<wire::core::Vec3d> b = junction_v1_points();
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

bool C497_bb2_context_rows_order_but_do_not_materialize() {
  return C480_bb2_context_rows_affect_order_but_are_not_emitted();
}

bool C498_bb2_saved_graph_remains_topology_authority() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok) {
    return false;
  }
  const wire::core::BackboneFrontier frontier = state.view().pole_frontier(b);
  return frontier.edge_ids.size() == 3 && frontier.edge_bundle_ids.size() == 3 &&
         frontier.span_ids.size() == first.value.generated_span_ids.size() + second.value.generated_span_ids.size();
}

bool C499_bb2_context_link_is_not_saved() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3 || state.view().backbone().edges.size() != 2) {
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
  if (!second.ok) {
    return false;
  }
  const wire::core::BackboneFrontier frontier = state.view().pole_frontier(b);
  return state.view().backbone().edges.size() == 3 && frontier.edge_ids.size() == 3 &&
         state.view().backbone().edge_bundles.size() == 3;
}

bool C500_bb2_context_link_requires_saved_edge_ref() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t ref_pos = cpp.find("SavedBackboneEdgeRef ref_for_existing_edge");
  const std::size_t next_pos = cpp.find("bool same_scope", ref_pos);
  const std::size_t save_pos = cpp.find("EditResult<bool> pipeline::save_graph");
  const std::size_t build_pos = cpp.find("EditResult<GenerateBundleFromPathResult> pipeline::build", save_pos);
  if (ref_pos == std::string::npos || next_pos == std::string::npos || save_pos == std::string::npos ||
      build_pos == std::string::npos) {
    return false;
  }
  const std::string ref_body = cpp.substr(ref_pos, next_pos - ref_pos);
  const std::string save_body = cpp.substr(save_pos, build_pos - save_pos);
  return contains_text(ref_body, "edge.saved == kInvalidObjectId") && !contains_text(ref_body, "saved_edge_for") &&
         contains_text(save_body, "context link saved edge missing");
}

bool C501_bb2_gate3_contract_passes() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t save_pos = cpp.find("EditResult<bool> pipeline::save_graph");
  const std::size_t build_pos = cpp.find("EditResult<GenerateBundleFromPathResult> pipeline::build", save_pos);
  if (save_pos == std::string::npos || build_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(save_pos, build_pos - save_pos);
  const std::size_t new_gate = body.find("if (edge.is_new)");
  const std::size_t save_call = body.find("state_.save_backbone_edge", new_gate);
  const std::size_t context_ref = body.find("ref_for_existing_edge", save_call);
  return new_gate != std::string::npos && save_call != std::string::npos && context_ref != std::string::npos &&
         save_call < context_ref;
}

bool C524_bb2_scenario_simple_line_mainline() {
  return C379_bb2_m1_required_outputs() && C511_bb2_draw_saved_from_geom() &&
         C424_bb2_saves_backbone_graph_nodes_edges() && C370_bb2_no_v1_deps();
}

bool C525_bb2_scenario_polyline3_connectivity_once() {
  return C388_bb2_polyline3_pair_model() && C392_bb2_polyline3_outputs() &&
         C422_bb2_rules_consume_topo_and_groups();
}

bool C526_bb2_scenario_multiple_bundles_share_connectivity() {
  return C400_bb2_multiple_bundles_smoke() && C401_bb2_multiple_bundles_polyline3_outputs() &&
         C402_bb2_bundle_spec_does_not_affect_pairs();
}

bool C527_bb2_scenario_existing_pole_continuation_uses_saved_graph() {
  return C516_bb2_generated_pole_with_saved_graph_still_connects() &&
         C515_bb2_rejects_existing_pole_without_saved_graph() &&
         C396_bb2_existing_pole_does_not_read_existing_spans();
}

bool C528_bb2_scenario_branch_emits_new_link_only() {
  return C458_bb2_existing_branch_BD_on_ABC() && C460_bb2_context_links_are_not_emitted() &&
         C499_bb2_context_link_is_not_saved();
}

bool C529_bb2_scenario_cross_without_kind_label() {
  return C459_bb2_existing_cross_DBE_on_ABC() && C462_bb2_no_junction_kind_after_existing_context() &&
         C477_bb2_cross_rows_are_separated_without_cross_kind();
}

bool C530_bb2_scenario_same_edge_different_bundle() {
  return C464_bb2_different_bundle_on_same_edge_allowed() &&
         C432_bb2_multiple_bundles_create_multiple_edge_bundles() &&
         C487_bb2_port_resolution_requires_bundle_compatible_scope();
}

bool C531_bb2_scenario_duplicate_reject_unchanged() {
  return C490_bb2_duplicate_same_edge_bundle_lane_rejected() &&
         C466_bb2_duplicate_reject_keeps_state_unchanged() &&
         C520_bb2_duplicate_span_binding_preflight_before_emit();
}

bool C532_bb2_scenario_pass_through_lowering_consumer_chain() {
  return C491_bb2_branch_lowering_v1_affects_geom() && C493_bb2_pass_through_does_not_change_pair_open() &&
         C519_bb2_draw_placeholder_uses_layout_points();
}

bool C533_bb2_build_mutation_order_is_fixed() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "bb2" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t build_pos = cpp.find("EditResult<GenerateBundleFromPathResult> pipeline::build()");
  const std::size_t pairs_pos = cpp.find("EditResult<pairs> ps = make(g_)", build_pos);
  const std::size_t check_pos = cpp.find("EditResult<bool> duplicates = check(ps.value)", build_pos);
  const std::size_t intent_pos = cpp.find("EditResult<intent> intents = make(ps.value)", build_pos);
  const std::size_t group_pos = cpp.find("EditResult<groups> placement = make(ps.value, intents.value)", build_pos);
  const std::size_t emit_pos = cpp.find("EditResult<topo> made = emit(ps.value)", build_pos);
  const std::size_t graph_pos = cpp.find("EditResult<bool> graph_saved = save_graph(made.value, ps.value)", build_pos);
  const std::size_t rules_pos = cpp.find("rules saved = make(made.value, placement.value)", build_pos);
  const std::size_t layout_pos = cpp.find("EditResult<layout> placed = make(saved)", build_pos);
  const std::size_t geom_pos = cpp.find("geom shaped = make(placed.value)", build_pos);
  const std::size_t draw_pos = cpp.find("draw drawn = make(placed.value, shaped)", build_pos);
  if (build_pos == std::string::npos || pairs_pos == std::string::npos || check_pos == std::string::npos ||
      intent_pos == std::string::npos || group_pos == std::string::npos || emit_pos == std::string::npos ||
      graph_pos == std::string::npos || rules_pos == std::string::npos || layout_pos == std::string::npos ||
      geom_pos == std::string::npos || draw_pos == std::string::npos) {
    return false;
  }
  return pairs_pos < check_pos && check_pos < intent_pos && intent_pos < group_pos &&
         group_pos < emit_pos && emit_pos < graph_pos && graph_pos < rules_pos && rules_pos < layout_pos &&
         layout_pos < geom_pos && geom_pos < draw_pos;
}

bool C534_bb2_invalid_inputs_stop_before_emit() {
  return C409_bb2_rejects_missing_port_band() && C414_bb2_still_rejects_avoid_constraints() &&
         C515_bb2_rejects_existing_pole_without_saved_graph();
}

bool C535_bb2_duplicate_preflight_is_mutation_boundary() {
  return C466_bb2_duplicate_reject_keeps_state_unchanged() &&
         C520_bb2_duplicate_span_binding_preflight_before_emit();
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
  test_registry::AddTest(tests, "C380_bb2_m1_draw_outputs_saved", "bb2 milestone 1 saves draw caches",
                         "Boundary", false, C380_bb2_m1_draw_outputs_saved);
  test_registry::AddTest(tests, "C381_bb2_m1_no_recalc_contract", "bb2 milestone 1 has no recalc contract",
                         "Boundary", false, C381_bb2_m1_no_recalc_contract);
  test_registry::AddTest(tests, "C382_bb2_geom_is_single_pipeline_layer", "bb2 keeps geom as one pipeline layer",
                         "Boundary", false, C382_bb2_geom_is_single_pipeline_layer);
  test_registry::AddTest(tests, "C383_bb2_draw_is_pipeline_layer", "bb2 draw is a pipeline layer",
                         "Boundary", false, C383_bb2_draw_is_pipeline_layer);
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
  test_registry::AddTest(tests, "C390_bb2_rejects_already_used_incident",
                         "bb2 rejects invalid pair incident ownership", "Boundary", true,
                         C390_bb2_rejects_already_used_incident);
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
  test_registry::AddTest(tests, "C422_bb2_rules_consume_topo_and_groups",
                         "bb2 rules consume topology and groups without pairs", "Boundary", false,
                         C422_bb2_rules_consume_topo_and_groups);
  test_registry::AddTest(tests, "C423_bb2_tspan_carries_endpoint_rows",
                         "bb2 topology spans carry endpoint row indices", "Boundary", false,
                         C423_bb2_tspan_carries_endpoint_rows);
  test_registry::AddTest(tests, "C424_bb2_saves_backbone_graph_nodes_edges",
                         "bb2 saves backbone graph nodes and edges", "Boundary", false,
                         C424_bb2_saves_backbone_graph_nodes_edges);
  test_registry::AddTest(tests, "C425_bb2_edge_carries_multiple_spans",
                         "bb2 backbone edge carries generated spans", "Boundary", false,
                         C425_bb2_edge_carries_multiple_spans);
  test_registry::AddTest(tests, "C426_bb2_existing_pole_resolves_graph_node",
                         "bb2 resolves saved graph node for existing poles", "Boundary", false,
                         C426_bb2_existing_pole_resolves_graph_node);
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
  test_registry::AddTest(tests, "C433_bb2_resolves_edge_for_same_poles",
                         "bb2 resolves saved edge for the same pole pair", "Boundary", false,
                         C433_bb2_resolves_edge_for_same_poles);
  test_registry::AddTest(tests, "C434_bb2_reverse_duplicate_same_bundle_rejected",
                         "bb2 reverse duplicate same-bundle generation is rejected", "Boundary", false,
                         C434_bb2_reverse_duplicate_same_bundle_rejected);
  test_registry::AddTest(tests, "C435_bb2_edge_metadata_is_not_overwritten_on_duplicate_reject",
                         "bb2 saved edge metadata is not overwritten on duplicate reject", "Boundary", false,
                         C435_bb2_edge_metadata_is_not_overwritten_on_duplicate_reject);
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
  test_registry::AddTest(tests, "C443_bb2_edge_resolution_behavior_unchanged",
                         "bb2 edge resolution behavior remains unchanged after duplicate reject", "Boundary", false,
                         C443_bb2_edge_resolution_behavior_unchanged);
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
  test_registry::AddTest(tests, "C463_bb2_duplicate_same_edge_bundle_rejected",
                         "bb2 duplicate same edge and bundle requests are rejected", "Boundary", false,
                         C463_bb2_duplicate_same_edge_bundle_rejected);
  test_registry::AddTest(tests, "C464_bb2_different_bundle_on_same_edge_allowed",
                         "bb2 different bundles on the same edge are allowed", "Boundary", false,
                         C464_bb2_different_bundle_on_same_edge_allowed);
  test_registry::AddTest(tests, "C465_bb2_duplicate_policy_does_not_read_existing_spans",
                         "bb2 duplicate policy reads saved edge bundles, not existing spans", "Boundary", false,
                         C465_bb2_duplicate_policy_does_not_read_existing_spans);
  test_registry::AddTest(tests, "C466_bb2_duplicate_reject_keeps_state_unchanged",
                         "bb2 duplicate reject keeps state unchanged", "Boundary", false,
                         C466_bb2_duplicate_reject_keeps_state_unchanged);
  test_registry::AddTest(tests, "C467_bb2_saves_row_port_bindings",
                         "bb2 saves row to materialized port bindings", "Boundary", false,
                         C467_bb2_saves_row_port_bindings);
  test_registry::AddTest(tests, "C468_bb2_row_port_binding_is_stable_for_existing_context",
                         "bb2 saves new row-port bindings without emitting context rows", "Boundary", false,
                         C468_bb2_row_port_binding_is_stable_for_existing_context);
  test_registry::AddTest(tests, "C469_bb2_row_port_binding_rejects_duplicate_without_resolution",
                         "bb2 duplicate row-port binding is rejected without port resolution", "Boundary", false,
                         C469_bb2_row_port_binding_rejects_duplicate_without_resolution);
  test_registry::AddTest(tests, "C470_bb2_row_port_identity_does_not_use_position_match",
                         "bb2 row-port identity does not use position matching", "Boundary", false,
                         C470_bb2_row_port_identity_does_not_use_position_match);
  test_registry::AddTest(tests, "C471_bb2_resolves_existing_port_by_binding",
                         "bb2 resolves existing ports by saved row-port binding", "Boundary", false,
                         C471_bb2_resolves_existing_port_by_binding);
  test_registry::AddTest(tests, "C472_bb2_port_resolution_requires_saved_binding",
                         "bb2 does not resolve ports without saved binding", "Boundary", false,
                         C472_bb2_port_resolution_requires_saved_binding);
  test_registry::AddTest(tests, "C473_bb2_resolved_port_used_by_new_span_endpoint",
                         "bb2 new spans use resolved port endpoints", "Boundary", false,
                         C473_bb2_resolved_port_used_by_new_span_endpoint);
  test_registry::AddTest(tests, "C474_bb2_port_resolution_rejects_ambiguous_binding",
                         "bb2 port resolution rejects ambiguous row-port bindings", "Boundary", false,
                         C474_bb2_port_resolution_rejects_ambiguous_binding);
  test_registry::AddTest(tests, "C475_bb2_port_resolution_does_not_read_existing_layout",
                         "bb2 port resolution does not read existing layout", "Boundary", false,
                         C475_bb2_port_resolution_does_not_read_existing_layout);
  test_registry::AddTest(tests, "C476_bb2_branch_rows_are_separated_without_branch_kind",
                         "bb2 branch rows are separated without branch kind", "Boundary", false,
                         C476_bb2_branch_rows_are_separated_without_branch_kind);
  test_registry::AddTest(tests, "C477_bb2_cross_rows_are_separated_without_cross_kind",
                         "bb2 cross rows are separated without cross kind", "Boundary", false,
                         C477_bb2_cross_rows_are_separated_without_cross_kind);
  test_registry::AddTest(tests, "C478_bb2_row_separation_is_deterministic",
                         "bb2 row separation is deterministic", "Invariant", false,
                         C478_bb2_row_separation_is_deterministic);
  test_registry::AddTest(tests, "C479_bb2_row_separation_does_not_change_pairs",
                         "bb2 row separation does not change pair source", "Boundary", false,
                         C479_bb2_row_separation_does_not_change_pairs);
  test_registry::AddTest(tests, "C480_bb2_context_rows_affect_order_but_are_not_emitted",
                         "bb2 context rows affect separation order but are not emitted", "Boundary", false,
                         C480_bb2_context_rows_affect_order_but_are_not_emitted);
  test_registry::AddTest(tests, "C481_bb2_pass_through_mode_is_accepted_in_limited_scope",
                         "bb2 accepts pass-through mode for saved junction nodes", "Boundary", false,
                         C481_bb2_pass_through_mode_is_accepted_in_limited_scope);
  test_registry::AddTest(tests, "C482_bb2_pass_through_creates_explicit_intent",
                         "bb2 pass-through node mode creates explicit layout intent", "Boundary", false,
                         C482_bb2_pass_through_creates_explicit_intent);
  test_registry::AddTest(tests, "C483_bb2_pass_through_ambiguous_target_rejected",
                         "bb2 pass-through ambiguous row target is rejected", "Boundary", false,
                         C483_bb2_pass_through_ambiguous_target_rejected);
  test_registry::AddTest(tests, "C484_bb2_lowering_draw_uses_layout_only",
                         "bb2 lowering draw uses layout only", "Boundary", false,
                         C484_bb2_lowering_draw_uses_layout_only);
  test_registry::AddTest(tests, "C485_bb2_lowering_intent_does_not_read_existing_spans",
                         "bb2 lowering intent does not read existing spans", "Boundary", false,
                         C485_bb2_lowering_intent_does_not_read_existing_spans);
  test_registry::AddTest(tests, "C486_bb2_pass_through_is_deterministic",
                         "bb2 pass-through intent is deterministic", "Invariant", false,
                         C486_bb2_pass_through_is_deterministic);
  test_registry::AddTest(tests, "C487_bb2_port_resolution_requires_bundle_compatible_scope",
                         "bb2 port resolution requires bundle-compatible scope", "Boundary", false,
                         C487_bb2_port_resolution_requires_bundle_compatible_scope);
  test_registry::AddTest(tests, "C488_bb2_port_resolution_accepts_same_compatible_binding",
                         "bb2 accepts multiple same-compatible port bindings", "Boundary", false,
                         C488_bb2_port_resolution_accepts_same_compatible_binding);
  test_registry::AddTest(tests, "C489_bb2_port_binding_index_invariant",
                         "bb2 port binding index exposes all compatible bindings", "Boundary", false,
                         C489_bb2_port_binding_index_invariant);
  test_registry::AddTest(tests, "C490_bb2_duplicate_same_edge_bundle_lane_rejected",
                         "bb2 duplicate same edge bundle lane requests do not duplicate", "Boundary", false,
                         C490_bb2_duplicate_same_edge_bundle_lane_rejected);
  test_registry::AddTest(tests, "C491_bb2_branch_lowering_v1_affects_geom",
                         "bb2 branch lowering v1 affects layout geometry", "Invariant", false,
                         C491_bb2_branch_lowering_v1_affects_geom);
  test_registry::AddTest(tests, "C492_bb2_cross_lowering_v1_affects_only_new_links",
                         "bb2 cross lowering v1 affects generated links only", "Invariant", false,
                         C492_bb2_cross_lowering_v1_affects_only_new_links);
  test_registry::AddTest(tests, "C493_bb2_pass_through_does_not_change_pair_open",
                         "bb2 pass-through does not change pair open authority", "Boundary", false,
                         C493_bb2_pass_through_does_not_change_pair_open);
  test_registry::AddTest(tests, "C494_bb2_lowering_v1_draw_does_not_redecide",
                         "bb2 lowering v1 draw does not redecide", "Boundary", false,
                         C494_bb2_lowering_v1_draw_does_not_redecide);
  test_registry::AddTest(tests, "C495_bb2_lowering_v1_does_not_read_existing_spans",
                         "bb2 lowering v1 does not read existing spans", "Boundary", false,
                         C495_bb2_lowering_v1_does_not_read_existing_spans);
  test_registry::AddTest(tests, "C496_bb2_junction_v1_deterministic",
                         "bb2 junction v1 output is deterministic", "Invariant", false,
                         C496_bb2_junction_v1_deterministic);
  test_registry::AddTest(tests, "C497_bb2_context_rows_order_but_do_not_materialize",
                         "bb2 context rows order placement without materializing", "Boundary", false,
                         C497_bb2_context_rows_order_but_do_not_materialize);
  test_registry::AddTest(tests, "C498_bb2_saved_graph_remains_topology_authority",
                         "bb2 saved graph remains topology authority", "Boundary", false,
                         C498_bb2_saved_graph_remains_topology_authority);
  test_registry::AddTest(tests, "C499_bb2_context_link_is_not_saved",
                         "bb2 context links are not saved as new edges", "Boundary", false,
                         C499_bb2_context_link_is_not_saved);
  test_registry::AddTest(tests, "C500_bb2_context_link_requires_saved_edge_ref",
                         "bb2 context links require saved edge refs", "Boundary", false,
                         C500_bb2_context_link_requires_saved_edge_ref);
  test_registry::AddTest(tests, "C501_bb2_gate3_contract_passes",
                         "bb2 save graph keeps context links out of save targets", "Boundary", false,
                         C501_bb2_gate3_contract_passes);
  test_registry::AddTest(tests, "C502_bb2_span_bindings_save_lane",
                         "bb2 saved span bindings carry lane identity", "Boundary", false,
                         C502_bb2_span_bindings_save_lane);
  test_registry::AddTest(tests, "C503_bb2_duplicate_span_binding_rejected_by_lane",
                         "bb2 rejects duplicate saved span binding lanes", "Boundary", false,
                         C503_bb2_duplicate_span_binding_rejected_by_lane);
  test_registry::AddTest(tests, "C504_bb2_span_resolution_does_not_read_geometry_or_layout",
                         "bb2 span resolution reads saved bindings only", "Boundary", false,
                         C504_bb2_span_resolution_does_not_read_geometry_or_layout);
  test_registry::AddTest(tests, "C505_bb2_save_graph_propagates_span_binding_failure",
                         "bb2 save graph propagates span binding failures", "Boundary", false,
                         C505_bb2_save_graph_propagates_span_binding_failure);
  test_registry::AddTest(tests, "C506_bb2_support_group_is_placement_layer",
                         "bb2 support group is a placement layer", "Boundary", false,
                         C506_bb2_support_group_is_placement_layer);
  test_registry::AddTest(tests, "C507_bb2_support_group_built_after_intent",
                         "bb2 support group is built after intent", "Boundary", false,
                         C507_bb2_support_group_built_after_intent);
  test_registry::AddTest(tests, "C508_bb2_support_group_drives_lowered_rules",
                         "bb2 support group drives lowered rules", "Boundary", false,
                         C508_bb2_support_group_drives_lowered_rules);
  test_registry::AddTest(tests, "C509_bb2_support_group_avoids_visual_terms",
                         "bb2 support group avoids visual terms", "Boundary", false,
                         C509_bb2_support_group_avoids_visual_terms);
  test_registry::AddTest(tests, "C510_bb2_layout_consumes_group_offset",
                         "bb2 layout consumes support group offset", "Boundary", false,
                         C510_bb2_layout_consumes_group_offset);
  test_registry::AddTest(tests, "C511_bb2_draw_saved_from_geom",
                         "bb2 draw is saved from geom", "Boundary", false,
                         C511_bb2_draw_saved_from_geom);
  test_registry::AddTest(tests, "C512_bb2_draw_does_not_read_topology",
                         "bb2 draw does not read topology", "Boundary", false,
                         C512_bb2_draw_does_not_read_topology);
  test_registry::AddTest(tests, "C513_bb2_support_visual_placeholder_from_layout",
                         "bb2 support visual placeholder comes from layout", "Boundary", false,
                         C513_bb2_support_visual_placeholder_from_layout);
  test_registry::AddTest(tests, "C514_bb2_draw_save_is_direct",
                         "bb2 draw save is direct", "Boundary", false,
                         C514_bb2_draw_save_is_direct);
  test_registry::AddTest(tests, "C515_bb2_rejects_existing_pole_without_saved_graph",
                         "bb2 rejects existing poles without saved graph", "Boundary", true,
                         C515_bb2_rejects_existing_pole_without_saved_graph);
  test_registry::AddTest(tests, "C516_bb2_generated_pole_with_saved_graph_still_connects",
                         "bb2 generated poles with saved graph still connect", "Boundary", false,
                         C516_bb2_generated_pole_with_saved_graph_still_connects);
  test_registry::AddTest(tests, "C517_bb2_migration_gate_does_not_infer_from_outputs",
                         "bb2 migration gate does not infer from outputs", "Boundary", false,
                         C517_bb2_migration_gate_does_not_infer_from_outputs);
  test_registry::AddTest(tests, "C518_bb2_lowered_layout_keeps_support_world_at_port_height",
                         "bb2 lowered layout keeps support world at port height", "Boundary", false,
                         C518_bb2_lowered_layout_keeps_support_world_at_port_height);
  test_registry::AddTest(tests, "C519_bb2_draw_placeholder_uses_layout_points",
                         "bb2 draw placeholder uses layout points", "Boundary", false,
                         C519_bb2_draw_placeholder_uses_layout_points);
  test_registry::AddTest(tests, "C520_bb2_duplicate_span_binding_preflight_before_emit",
                         "bb2 duplicate span binding preflight runs before emit", "Boundary", false,
                         C520_bb2_duplicate_span_binding_preflight_before_emit);
  test_registry::AddTest(tests, "C521_bb2_context_link_preserves_saved_dir",
                         "bb2 context link preserves saved direction", "Boundary", false,
                         C521_bb2_context_link_preserves_saved_dir);
  test_registry::AddTest(tests, "C522_bb2_supported_scope_is_documented",
                         "bb2 supported generation scope is documented", "Boundary", false,
                         C522_bb2_supported_scope_is_documented);
  test_registry::AddTest(tests, "C523_bb2_scope_gate_matches_entrypoint",
                         "bb2 scope gate matches the entrypoint", "Boundary", false,
                         C523_bb2_scope_gate_matches_entrypoint);
  test_registry::AddTest(tests, "C524_bb2_scenario_simple_line_mainline",
                         "bb2 simple line scenario covers outputs and authority", "Boundary", false,
                         C524_bb2_scenario_simple_line_mainline);
  test_registry::AddTest(tests, "C525_bb2_scenario_polyline3_connectivity_once",
                         "bb2 polyline scenario keeps connectivity authority single", "Boundary", false,
                         C525_bb2_scenario_polyline3_connectivity_once);
  test_registry::AddTest(tests, "C526_bb2_scenario_multiple_bundles_share_connectivity",
                         "bb2 multiple-bundle scenario shares connectivity", "Boundary", false,
                         C526_bb2_scenario_multiple_bundles_share_connectivity);
  test_registry::AddTest(tests, "C527_bb2_scenario_existing_pole_continuation_uses_saved_graph",
                         "bb2 existing-pole continuation scenario uses saved graph", "Boundary", false,
                         C527_bb2_scenario_existing_pole_continuation_uses_saved_graph);
  test_registry::AddTest(tests, "C528_bb2_scenario_branch_emits_new_link_only",
                         "bb2 branch scenario emits only the new link", "Boundary", false,
                         C528_bb2_scenario_branch_emits_new_link_only);
  test_registry::AddTest(tests, "C529_bb2_scenario_cross_without_kind_label",
                         "bb2 cross scenario runs without kind labels", "Boundary", false,
                         C529_bb2_scenario_cross_without_kind_label);
  test_registry::AddTest(tests, "C530_bb2_scenario_same_edge_different_bundle",
                         "bb2 same-edge different-bundle scenario shares saved edge", "Boundary", false,
                         C530_bb2_scenario_same_edge_different_bundle);
  test_registry::AddTest(tests, "C531_bb2_scenario_duplicate_reject_unchanged",
                         "bb2 duplicate scenario rejects without mutation", "Boundary", false,
                         C531_bb2_scenario_duplicate_reject_unchanged);
  test_registry::AddTest(tests, "C532_bb2_scenario_pass_through_lowering_consumer_chain",
                         "bb2 pass-through lowering scenario preserves consumer chain", "Boundary", false,
                         C532_bb2_scenario_pass_through_lowering_consumer_chain);
  test_registry::AddTest(tests, "C533_bb2_build_mutation_order_is_fixed",
                         "bb2 build mutation order is fixed", "Boundary", false,
                         C533_bb2_build_mutation_order_is_fixed);
  test_registry::AddTest(tests, "C534_bb2_invalid_inputs_stop_before_emit",
                         "bb2 invalid inputs stop before emit", "Boundary", true,
                         C534_bb2_invalid_inputs_stop_before_emit);
  test_registry::AddTest(tests, "C535_bb2_duplicate_preflight_is_mutation_boundary",
                         "bb2 duplicate preflight is the mutation boundary", "Boundary", false,
                         C535_bb2_duplicate_preflight_is_mutation_boundary);
}

WIRE_REGISTER_TEST_SUITE(register_bb2_tests);

} // namespace
