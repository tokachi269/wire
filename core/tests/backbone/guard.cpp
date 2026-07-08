#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_view.hpp"

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

bool C370_backbone_no_v1_deps() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "backbone";
  const std::vector<std::string> banned = {
      "support_layout_",
      "JunctionRelationKind",
      "SpanSupportLayoutDecisionSeed",
      "Authority",
      "Materialization",
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

bool C387_backbone_pairs_are_single_source() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text)) {
    return false;
  }
  const std::string signature = "EditResult<pairs> pipeline::make(const graph& made) const";
  const std::size_t first = text.find(signature);
  if (first == std::string::npos || text.find(signature, first + signature.size()) != std::string::npos) {
    return false;
  }
  return !contains_text(text, "make_pairs") && !contains_text(text, "build_pairs") &&
         !contains_text(text, "regenerate_pairs");
}

bool C391_backbone_no_kind_label() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "backbone";
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

bool C395_backbone_is_new_does_not_affect_pairs() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  std::string body;
  if (!function_body(cpp, "EditResult<pairs> pipeline::make(const graph& made) const", &body)) {
    return false;
  }
  return !contains_text(body, ".is_new");
}

bool C396_backbone_existing_pole_does_not_read_existing_spans() {
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

bool C402_backbone_bundle_spec_does_not_affect_pairs() {
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

bool C410_placement_height_does_not_affect_pairs() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text)) {
    return false;
  }
  std::string body;
  if (!function_body(text, "EditResult<pairs> pipeline::make(const graph& made) const", &body)) {
    return false;
  }
  return !contains_text(body, "pole_type") && !contains_text(body, "PortPlacementBand") &&
         !contains_text(body, "height") && !contains_text(body, "BundleTemplate") &&
         !contains_text(body, "spec_.bundles");
}

bool C413_backbone_lateral_offset_does_not_affect_pairs() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
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

bool C420_backbone_node_mode_does_not_affect_pairs() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
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

bool C421_backbone_topo_row_carries_source() {
  const std::filesystem::path header = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.hpp";
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
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

bool C439_backbone_source_still_avoids_support_layout_entrypoint() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "backbone";
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

bool C440_backbone_does_not_read_authoritative_backbone_directly() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "backbone";
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

bool C446_backbone_layout_boundary_behavior_unchanged() {
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
  }
  return true;
}

bool C449_backbone_layout_read_does_not_expose_authority() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "state" / "span_runtime.cpp";
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

bool C452_backbone_layout_state_does_not_expose_old_contract_names() {
  const std::filesystem::path header = repo_root() / "core" / "include" / "wire" / "core" / "span_layout_types.hpp";
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

bool C456_backbone_source_avoids_old_layout_cache_names() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "backbone";
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

bool C457_backbone_layout_cache_boundary_behavior_unchanged() {
  return C446_backbone_layout_boundary_behavior_unchanged();
}

bool C465_backbone_duplicate_policy_does_not_read_existing_spans() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
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

bool C470_backbone_row_port_identity_does_not_use_position_match() {
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
         !contains_text(body, "seed") && !contains_text(body, "position");
}

bool C475_backbone_port_resolution_does_not_read_existing_layout() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("EditResult<ObjectId> resolve_port_binding");
  const std::size_t next_pos = cpp.find("bool route_clear_of_avoid_points", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "backbone_port_bindings_for_row") && !contains_text(body, "span_layout") &&
         !contains_text(body, "support_layout") && !contains_text(body, "seed") &&
         !contains_text(body, "world_position") && !contains_text(body, "position");
}

bool C479_backbone_row_separation_does_not_change_pairs() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
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

bool C485_backbone_lowering_intent_does_not_read_existing_spans() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
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

bool C504_backbone_span_resolution_does_not_read_geometry_or_layout() {
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

bool C505_backbone_save_graph_propagates_span_binding_failure() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  std::string body;
  if (!function_body(cpp, "EditResult<bool> pipeline::save_graph(const topo& made, const pairs& ps)", &body)) {
    return false;
  }
  return contains_text(body, "bind_backbone_span(edge_bundle_id, span.lane, span.id)") &&
         contains_text(body, "span_bound.error");
}

bool C512_backbone_draw_does_not_read_topology() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
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

bool C517_backbone_migration_gate_does_not_infer_from_outputs() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
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

bool C520_backbone_duplicate_span_binding_preflight_before_emit() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t route_pos = cpp.find("EditResult<pipeline::route> pipeline::emit_route");
  const std::size_t check_call = cpp.find("return check(ps.value)", route_pos);
  const std::size_t emit_call = cpp.find("return emit(ps.value)", route_pos);
  if (route_pos == std::string::npos || check_call == std::string::npos || emit_call == std::string::npos ||
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

bool C522_backbone_supported_scope_is_documented() {
  const std::filesystem::path doc = repo_root() / "docs" / "architecture.md";
  std::string text;
  if (!file_text(doc, &text)) {
    return false;
  }
  return contains_text(text, "## backbone generation") && contains_text(text, "`GenerateFromBackboneSpec()`") &&
         contains_text(text, "`unsupported`") && contains_text(text, "`SavedBackboneGraph`") &&
         contains_text(text, "`pairs make(graph)`") && contains_text(text, "duplicate edge bundle/span binding") &&
         contains_text(text, "`support_world`") && contains_text(text, "`endpoint_world`") &&
         contains_text(text, "v1") && contains_text(text, "fallback");
}

bool C523_backbone_scope_gate_matches_entrypoint() {
  const std::filesystem::path entry = repo_root() / "core" / "src" / "generation" / "generate_from_backbone.cpp";
  const std::filesystem::path backbone = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string entry_text;
  std::string backbone_text;
  if (!file_text(entry, &entry_text) || !file_text(backbone, &backbone_text)) {
    return false;
  }
  const bool entry_uses_backbone = contains_text(entry_text, "generation::backbone::pipeline") &&
                              contains_text(entry_text, "pipeline.prepare()") &&
                              contains_text(entry_text, "pipeline.check()") &&
                              contains_text(entry_text, "pipeline.build()");
  const std::size_t route_pos = backbone_text.find("EditResult<pipeline::route> pipeline::emit_route");
  const std::size_t check_call = backbone_text.find("return check(ps.value)", route_pos);
  const std::size_t intent_call = backbone_text.find("return make(ps.value)", route_pos);
  const std::size_t emit_call = backbone_text.find("return emit(ps.value)", route_pos);
  const bool preflight_before_emit = route_pos != std::string::npos && check_call != std::string::npos &&
                                     intent_call != std::string::npos && emit_call != std::string::npos &&
                                     check_call < intent_call && intent_call < emit_call;
  return entry_uses_backbone && preflight_before_emit;
}

bool C493_backbone_pass_through_does_not_change_pair_open() {
  return C420_backbone_node_mode_does_not_affect_pairs() && C479_backbone_row_separation_does_not_change_pairs();
}

bool C494_backbone_lowering_v1_draw_does_not_redecide() {
  return C484_backbone_lowering_draw_uses_layout_only();
}

bool C495_backbone_lowering_v1_does_not_read_existing_spans() {
  return C485_backbone_lowering_intent_does_not_read_existing_spans();
}

bool C501_backbone_gate3_contract_passes() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  std::string body;
  if (!function_body(cpp, "EditResult<bool> pipeline::save_graph(const topo& made, const pairs& ps)", &body)) {
    return false;
  }
  const std::size_t new_gate = body.find("if (edge.is_new)");
  const std::size_t save_call = body.find("state_.save_backbone_edge", new_gate);
  const std::size_t context_ref = body.find("ref_for_existing_edge", save_call);
  return new_gate != std::string::npos && save_call != std::string::npos && context_ref != std::string::npos &&
         save_call < context_ref;
}

bool C533_backbone_build_mutation_order_is_fixed() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t route_pos = cpp.find("EditResult<pipeline::route> pipeline::emit_route");
  const std::size_t derived_pos = cpp.find("EditResult<bool> pipeline::save_derived", route_pos);
  const std::size_t pairs_pos = cpp.find("return make(g_)", route_pos);
  const std::size_t check_pos = cpp.find("return check(ps.value)", route_pos);
  const std::size_t intent_pos = cpp.find("return make(ps.value)", route_pos);
  const std::size_t group_pos = cpp.find("return make(ps.value, intents.value)", route_pos);
  const std::size_t emit_pos = cpp.find("return emit(ps.value)", route_pos);
  const std::size_t graph_pos = cpp.find("return save_graph(made.value, ps.value)", route_pos);
  const std::size_t rules_pos = cpp.find("rules next = make(route.made, route.ps, route.placement)", derived_pos);
  const std::size_t layout_pos = cpp.find("return make(saved)", derived_pos);
  const std::size_t geom_pos = cpp.find("return make(placed.value)", derived_pos);
  const std::size_t draw_pos = cpp.find("return make(placed.value, shaped.value)", derived_pos);
  if (route_pos == std::string::npos || derived_pos == std::string::npos || pairs_pos == std::string::npos ||
      check_pos == std::string::npos ||
      intent_pos == std::string::npos || group_pos == std::string::npos || emit_pos == std::string::npos ||
      graph_pos == std::string::npos || rules_pos == std::string::npos || layout_pos == std::string::npos ||
      geom_pos == std::string::npos || draw_pos == std::string::npos) {
    return false;
  }
  return pairs_pos < check_pos && check_pos < intent_pos && intent_pos < group_pos &&
         group_pos < emit_pos && emit_pos < graph_pos && graph_pos < rules_pos && rules_pos < layout_pos &&
         layout_pos < geom_pos && geom_pos < draw_pos;
}

bool C535_backbone_duplicate_preflight_is_mutation_boundary() {
  return C466_backbone_duplicate_reject_keeps_state_unchanged() &&
         C520_backbone_duplicate_span_binding_preflight_before_emit();
}

bool C537_backbone_draw_source_has_no_decision_inputs() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
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
  return !contains_text(body, "SavedBackbone") && !contains_text(body, "pairs") && !contains_text(body, "edge_bundle") &&
         !contains_text(body, "support_group") && !contains_text(body, "branch_down_offset_m");
}

bool C538_backbone_viewer_deps_are_not_core_draw_gate() {
  const std::filesystem::path doc = repo_root() / "docs" / "architecture.md";
  std::string text;
  if (!file_text(doc, &text)) {
    return false;
  }
  return contains_text(text, "viewer") && contains_text(text, "visual/render cache") &&
         contains_text(text, "topology") && contains_text(text, "pair") &&
         contains_text(text, "row") && contains_text(text, "lowering");
}

bool C540_backbone_unsupported_request_does_not_create_v1_outputs() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.bundles.clear();
  const std::size_t poles = state.view().poles().size();
  const std::size_t ports = state.view().ports().size();
  const std::size_t bundles = state.view().bundles().size();
  const std::size_t spans = state.view().spans().size();
  const std::size_t nodes = state.view().backbone().nodes.size();
  const std::size_t edges = state.view().backbone().edges.size();
  const std::size_t edge_bundles = state.view().backbone().edge_bundles.size();
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && state.view().poles().size() == poles && state.view().ports().size() == ports &&
         state.view().bundles().size() == bundles && state.view().spans().size() == spans &&
         state.view().backbone().nodes.size() == nodes && state.view().backbone().edges.size() == edges &&
         state.view().backbone().edge_bundles.size() == edge_bundles;
}

bool C541_backbone_manual_existing_pole_without_graph_is_gate_rejected() {
  return C515_backbone_rejects_existing_pole_without_saved_graph() &&
         C517_backbone_migration_gate_does_not_infer_from_outputs();
}

bool C542_backbone_usable_mainline_architecture_audit_passes() {
  const std::filesystem::path doc = repo_root() / "docs" / "architecture.md";
  std::string text;
  if (!file_text(doc, &text)) {
    return false;
  }
  const bool doc_ok = contains_text(text, "`SavedBackboneGraph`") &&
                      contains_text(text, "| topology | `SavedBackboneGraph`") &&
                      contains_text(text, "| connectivity | `pairs make(graph)`") &&
                      contains_text(text, "| placement | support group / row placement") &&
                      contains_text(text, "| draw | visual / render cache");
  return doc_ok && C498_backbone_saved_graph_remains_topology_authority() && C387_backbone_pairs_are_single_source() &&
         C506_backbone_support_group_is_placement_layer() && C535_backbone_duplicate_preflight_is_mutation_boundary() &&
         C537_backbone_draw_source_has_no_decision_inputs() && C523_backbone_scope_gate_matches_entrypoint();
}

bool C601_backbone_context_only_bundle_policy_does_not_filter_new_route() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = line_req(state);
  add_backbone_bundle(base, wire::core::BundleKind::kCommunication);
  const auto base_out = state.GenerateFromBackboneSpec(base);
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId source_span_id =
      span_for_bundle(state, base_out.value.generated_span_ids, wire::core::BundleKind::kCommunication);
  const auto* source_span = state.view().spans().find(source_span_id);
  if (source_span == nullptr) {
    return false;
  }
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span->id;
  pick.hit_pos_world = {6.0, 0.0, 4.0};
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kCommunication};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.bundles.clear();
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 0;
  midair.support_kind = resolved.value.support_kind;
  midair.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {midair};
  const auto branch_out = state.GenerateFromBackboneSpec(branch);
  if (!branch_out.ok || branch_out.value.generated_pole_ids.size() != 1 || branch_out.value.bundle_ids.size() != 1) {
    return false;
  }
  const auto* branch_bundle = state.view().bundles().find(branch_out.value.bundle_ids.front());
  if (branch_bundle == nullptr || branch_bundle->bundle_template_id != wire::core::BundleKind::kCommunication) {
    return false;
  }

  const wire::core::ObjectId e = branch_out.value.generated_pole_ids.front();
  const auto* pole = state.view().poles().find(e);
  if (pole == nullptr) {
    return false;
  }
  wire::core::BackboneSpec cont = line_req(state);
  cont.bundles.clear();
  add_backbone_bundle(cont, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(cont, wire::core::BundleKind::kCommunication);
  cont.path.polyline = {pole->world_transform.position, {6.0, 16.0, 0.0}};
  cont.path.node_specs = {pole_spec(0, e)};
  const auto out = state.GenerateFromBackboneSpec(cont);
  if (!out.ok || out.value.bundle_ids.size() != 2) {
    return false;
  }
  std::unordered_set<wire::core::BundleKind> generated{};
  for (wire::core::ObjectId bundle_id : out.value.bundle_ids) {
    const auto* bundle = state.view().bundles().find(bundle_id);
    if (bundle != nullptr) {
      generated.insert(bundle->bundle_template_id);
    }
  }
  return generated.contains(wire::core::BundleKind::kLowVoltage) &&
         generated.contains(wire::core::BundleKind::kCommunication);
}

bool C602_backbone_context_only_pole_band_does_not_filter_new_route() {
  wire::core::CoreState state;
  const auto base = state.GenerateFromBackboneSpec(line_req(state));
  if (!base.ok || base.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const wire::core::ObjectId a = base.value.generated_pole_ids[0];
  const wire::core::ObjectId b = base.value.generated_pole_ids[1];
  wire::core::PoleTypeId comm_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& item : state.view().pole_types()) {
    if (item.second.name == "CommunicationPole") {
      comm_type_id = item.first;
      break;
    }
  }
  if (comm_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }
  auto type_it = state.view().pole_types().find(comm_type_id);
  if (type_it == state.view().pole_types().end()) {
    return false;
  }
  wire::core::PoleTypeDefinition comm_type = type_it->second;
  comm_type.port_bands.erase(std::remove_if(comm_type.port_bands.begin(), comm_type.port_bands.end(),
                                            [](const wire::core::PortPlacementBand& band) {
                                              return band.category == wire::core::ConnectionCategory::kLowVoltage &&
                                                     band.layer == 1;
                                            }),
                             comm_type.port_bands.end());
  if (!state.UpdatePoleTypeDefinition(comm_type).ok) {
    return false;
  }
  const auto rejected = state.ApplyPoleType(b, comm_type_id);
  const auto* unchanged_b = state.view().poles().find(b);
  if (rejected.ok || unchanged_b == nullptr || unchanged_b->pole_type_id == comm_type_id) {
    return false;
  }
  const auto* pole_a = state.view().poles().find(a);
  if (pole_a == nullptr) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_a->world_transform.position, {0.0, 8.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, a)};
  const auto out = state.GenerateFromBackboneSpec(branch);
  return out.ok && out.value.generated_span_ids.size() ==
                       static_cast<std::size_t>(bundle_count(state, wire::core::BundleKind::kLowVoltage));
}

bool C603_backbone_context_node_does_not_affect_generated_endpoint_yaw() {
  wire::core::CoreState state;
  const auto base = state.GenerateFromBackboneSpec(line_req(state));
  if (!base.ok || base.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const wire::core::ObjectId a = base.value.generated_pole_ids[0];
  const auto* pole_a = state.view().poles().find(a);
  if (pole_a == nullptr) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_a->world_transform.position, {0.0, 8.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, a)};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.generated_pole_ids.size() != 1) {
    return false;
  }
  const auto* endpoint = state.view().poles().find(out.value.generated_pole_ids.front());
  return endpoint != nullptr && almost_equal(endpoint->world_transform.rotation_euler_deg.z, 90.0, 1e-6);
}

bool C608_backbone_saved_backbone_result_does_not_duplicate_saved_pole_nodes() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || state.view().backbone().nodes.empty()) {
    return false;
  }
  const wire::core::BackboneResult result = state.SavedBackboneResult();
  if (result.nodes.size() != state.view().backbone().nodes.size()) {
    return false;
  }
  std::unordered_set<wire::core::ObjectId> node_ids{};
  for (const wire::core::SupportNode& node : result.nodes) {
    if (node.node_id == wire::core::kInvalidObjectId || !node_ids.insert(node.node_id).second) {
      return false;
    }
    const auto* saved = state.view().backbone_node(node.node_id);
    if (saved == nullptr || saved->pole_id != node.pole_id || saved->support_kind != node.support_kind) {
      return false;
    }
  }
  return true;
}

bool C567_backbone_segment_pick_midair_uses_source_span_height() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = line_req(state);
  const auto base_out = state.GenerateFromBackboneSpec(base);
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) {
    return false;
  }
  const auto* source_span = state.view().spans().find(base_out.value.generated_span_ids.front());
  if (source_span == nullptr) {
    return false;
  }
  const auto* source_a = state.view().ports().find(source_span->port_a_id);
  const auto* source_b = state.view().ports().find(source_span->port_b_id);
  if (source_a == nullptr || source_b == nullptr) {
    return false;
  }
  const double expected_z = 0.5 * (source_a->world_position.z + source_b->world_position.z);

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span->id;
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  pick.has_segment_endpoints = false;
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kLowVoltage};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId ||
      !almost_equal(resolved.value.position.z, expected_z, 1e-9)) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.generated_span_ids.size() != static_cast<std::size_t>(req_bundle_count(state, branch))) {
    return false;
  }
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
    if ((a->owner_pole_id == wire::core::kInvalidObjectId && almost_equal(a->world_position.z, expected_z, 1e-9)) ||
        (b->owner_pole_id == wire::core::kInvalidObjectId && almost_equal(b->world_position.z, expected_z, 1e-9))) {
      return true;
    }
  }
  return false;
}

bool C568_backbone_source_edge_midair_branch_uses_source_curve_projection() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = line_req(state);
  const auto base_out = state.GenerateFromBackboneSpec(base);
  if (!base_out.ok || base_out.value.generated_span_ids.empty() || state.view().backbone().edges.size() != 1) {
    return false;
  }
  const wire::core::SavedBackboneEdge& source_edge = state.view().backbone().edges.front();
  const std::optional<wire::core::Vec3d> expected = state.view().backbone_attachment_world(
      source_edge.edge_id, source_edge.node_a, wire::core::BundleKind::kLowVoltage, 0, 0.5);
  if (!expected.has_value() || expected->z <= 0.0) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = wire::core::kInvalidObjectId;
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = source_edge.node_a;
  pick.segment_node_b_id = source_edge.node_b;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {12.0, 0.0, 0.0};
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kLowVoltage};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId ||
      !almost_equal(resolved.value.position, *expected, 1e-9)) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.edges.size() != 2) {
    return false;
  }
  const auto saved_midair = std::find_if(graph.nodes.begin(), graph.nodes.end(),
                                         [](const wire::core::SavedBackboneNode& saved) {
                                           return saved.pole_id == wire::core::kInvalidObjectId &&
                                                  saved.has_source_edge;
                                         });
  if (saved_midair == graph.nodes.end()) {
    return false;
  }

  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    const wire::core::SpanLayoutView layout = state.span_layout(span_id);
    if (span == nullptr || !layout.has_layout()) {
      return false;
    }
    const auto* a = state.view().ports().find(span->port_a_id);
    const auto* b = state.view().ports().find(span->port_b_id);
    if (a == nullptr || b == nullptr) {
      return false;
    }
    const bool start_ownerless = a->owner_pole_id == wire::core::kInvalidObjectId;
    const bool end_ownerless = b->owner_pole_id == wire::core::kInvalidObjectId;
    const wire::core::LayoutEndpoint& endpoint = start_ownerless ? layout.entry->start : layout.entry->end;
    const wire::core::Port* ownerless_port = start_ownerless ? a : b;
    if ((start_ownerless || end_ownerless) && !endpoint.default_lower_required && !endpoint.lower_required &&
        almost_equal(ownerless_port->world_position, *expected, 1e-9) &&
        almost_equal(endpoint.endpoint_world, *expected, 1e-9)) {
      return true;
    }
  }
  return false;
}

bool C718_viewer_hit_world_height_is_not_source_edge_branch_authority() {
  wire::core::CoreState state;
  const auto base_out = state.GenerateFromBackboneSpec(line_req(state));
  if (!base_out.ok || base_out.value.generated_span_ids.empty() || state.view().backbone().edges.size() != 1) {
    return false;
  }
  const wire::core::SavedBackboneEdge& source_edge = state.view().backbone().edges.front();
  const auto expected = state.view().backbone_attachment_world(
      source_edge.edge_id, source_edge.node_a, wire::core::BundleKind::kLowVoltage, 0, 0.5);
  if (!expected.has_value()) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = wire::core::kInvalidObjectId;
  pick.hit_pos_world = {6.0, 0.0, 123.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = source_edge.node_a;
  pick.segment_node_b_id = source_edge.node_b;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {12.0, 0.0, 0.0};
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kLowVoltage};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || almost_equal(resolved.value.position.z, pick.hit_pos_world.z, 1e-9) ||
      !almost_equal(resolved.value.position, *expected, 1e-9)) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  return std::any_of(graph.nodes.begin(), graph.nodes.end(), [](const wire::core::SavedBackboneNode& saved) {
    return saved.pole_id == wire::core::kInvalidObjectId && saved.has_source_edge;
  });
}

bool C719_source_edge_branch_endpoint_follows_current_curve_projection() {
  wire::core::CoreState state;
  const auto base_out = state.GenerateFromBackboneSpec(line_req(state));
  if (!base_out.ok || base_out.value.generated_span_ids.size() != 1 || state.view().backbone().edges.size() != 1) {
    return false;
  }
  const wire::core::ObjectId source_span_id = base_out.value.generated_span_ids.front();
  const wire::core::SavedBackboneEdge& source_edge = state.view().backbone().edges.front();
  const wire::core::ObjectId source_edge_id = source_edge.edge_id;
  const wire::core::ObjectId source_node_a = source_edge.node_a;
  const wire::core::ObjectId source_node_b = source_edge.node_b;
  const auto initial_projection = state.view().backbone_attachment_world(
      source_edge_id, source_node_a, wire::core::BundleKind::kLowVoltage, 0, 0.5);
  if (!initial_projection.has_value()) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = wire::core::kInvalidObjectId;
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = source_node_a;
  pick.segment_node_b_id = source_node_b;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {12.0, 0.0, 0.0};
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kLowVoltage};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto branch_out = state.GenerateFromBackboneSpec(branch);
  if (!branch_out.ok || branch_out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId branch_span_id = branch_out.value.generated_span_ids.front();

  wire::core::GeometrySettings geometry = state.view().geometry_settings();
  geometry.sag_enabled = true;
  geometry.sag_factor = std::max(geometry.sag_factor, 0.08);
  const auto updated = state.UpdateGeometrySettings(geometry);
  if (!updated.ok) {
    return false;
  }
  const wire::core::CurveCacheEntry* source_curve = state.find_curve_cache(source_span_id);
  const auto current_projection = state.view().backbone_attachment_world(
      source_edge_id, source_node_a, wire::core::BundleKind::kLowVoltage, 0, 0.5);
  const wire::core::SpanLayoutView branch_layout = state.span_layout(branch_span_id);
  const wire::core::Span* branch_span = state.view().spans().find(branch_span_id);
  if (source_curve == nullptr || !current_projection.has_value() || !branch_layout.has_layout() ||
      branch_span == nullptr) {
    return false;
  }
  const wire::core::Port* start_port = state.view().ports().find(branch_layout.entry->start.port_id);
  const bool start_ownerless = start_port != nullptr && start_port->owner_pole_id == wire::core::kInvalidObjectId;
  const wire::core::LayoutEndpoint& endpoint = start_ownerless ? branch_layout.entry->start : branch_layout.entry->end;
  const bool projection_changed = !almost_equal(*initial_projection, *current_projection, 1e-4);
  const bool has_ref = endpoint.source_projection.valid();
  const bool endpoint_matches = almost_equal(endpoint.endpoint_world, *current_projection, 1e-9);
  return projection_changed && has_ref && endpoint_matches;
}

bool C720_source_edge_pipeline_front_half_does_not_read_curve_projection() {
  std::string text;
  if (!file_text(repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp", &text)) {
    return false;
  }
  std::string check_body;
  std::string emit_ports_body;
  if (!function_body(text, "EditResult<bool> pipeline::check(const pairs& ps) const", &check_body) ||
      !function_body(text, "EditResult<bool> pipeline::emit_ports(topo* made, const pairs& ps, ChangeSet* changes)",
                     &emit_ports_body)) {
    return false;
  }
  const std::vector<std::string> banned = {"backbone_attachment_world", "find_curve_cache", "CurveCacheEntry",
                                           "EvaluatePosition"};
  for (const std::string& token : banned) {
    if (contains_text(check_body, token) || contains_text(emit_ports_body, token)) {
      return false;
    }
  }
  return contains_text(check_body, "source_projection_binding_exists") &&
         contains_text(emit_ports_body, "source_projection_binding_exists");
}

bool C612_backbone_direct_derive_does_not_call_recalc_paths() {
  std::string text;
  if (!file_text(repo_root() / "core/src/generation/backbone/derive.cpp", &text)) {
    return false;
  }
  const std::vector<std::string> banned = {"Commit(", "ProcessDirtyQueues", "rebuild_span_geometry",
                                           "rebuild_span_visual", "cache_span_support_layout",
                                           "support_layout_contract", "support_layout_projection",
                                           "materialization", "generate_span_curve"};
  for (const std::string& word : banned) {
    if (contains_text(text, word)) {
      return false;
    }
  }
  return contains_text(text, "DeriveGeneratedSpanOutputs") && contains_text(text, "cache_span_layout") &&
         contains_text(text, "cache_span_curve") && contains_text(text, "cache_span_bounds") &&
         contains_text(text, "cache_span_visual") && contains_text(text, "cache_span_render");
}

bool C617_backbone_reshape_does_not_rewrite_layout() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId span_id = out.value.generated_span_ids.front();
  const wire::core::SpanLayoutView before = state.span_layout(span_id);
  if (!before.has_layout()) {
    return false;
  }
  const wire::core::SpanLayoutEntry before_layout = *before.entry;
  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = !settings.sag_enabled;
  settings.sag_factor += 0.01;
  const auto updated = state.UpdateGeometrySettings(settings);
  const wire::core::SpanLayoutView after = state.span_layout(span_id);
  return updated.ok && after.has_layout() &&
         almost_equal(before_layout.start.endpoint_world.x, after.entry->start.endpoint_world.x, 1e-9) &&
         almost_equal(before_layout.start.endpoint_world.y, after.entry->start.endpoint_world.y, 1e-9) &&
         almost_equal(before_layout.start.endpoint_world.z, after.entry->start.endpoint_world.z, 1e-9) &&
         almost_equal(before_layout.end.endpoint_world.x, after.entry->end.endpoint_world.x, 1e-9) &&
         almost_equal(before_layout.end.endpoint_world.y, after.entry->end.endpoint_world.y, 1e-9) &&
         almost_equal(before_layout.end.endpoint_world.z, after.entry->end.endpoint_world.z, 1e-9) &&
         before_layout.source_version == after.entry->source_version;
}

bool C618_backbone_redraw_does_not_rewrite_layout_or_geom() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId span_id = out.value.generated_span_ids.front();
  const wire::core::SpanLayoutView before_layout_view = state.span_layout(span_id);
  const wire::core::CurveCacheEntry* before_curve = state.find_curve_cache(span_id);
  const wire::core::BoundsCacheEntry* before_bounds = state.find_bounds_cache(span_id);
  if (!before_layout_view.has_layout() || before_curve == nullptr || before_bounds == nullptr) {
    return false;
  }
  const wire::core::SpanLayoutEntry before_layout = *before_layout_view.entry;
  const std::vector<wire::core::Vec3d> before_samples = before_curve->detail.sample_points;
  const wire::core::AABBd before_whole = before_bounds->whole;
  wire::core::VisualSettings settings = state.view().visual_settings();
  settings.support_arm_radius_m += 0.01;
  const auto updated = state.UpdateVisualSettings(settings);
  const wire::core::SpanLayoutView after_layout = state.span_layout(span_id);
  const wire::core::CurveCacheEntry* after_curve = state.find_curve_cache(span_id);
  const wire::core::BoundsCacheEntry* after_bounds = state.find_bounds_cache(span_id);
  if (!updated.ok || !after_layout.has_layout() || after_curve == nullptr || after_bounds == nullptr ||
      before_layout.source_version != after_layout.entry->source_version ||
      before_samples.size() != after_curve->detail.sample_points.size()) {
    return false;
  }
  for (std::size_t i = 0; i < before_samples.size(); ++i) {
    if (!almost_equal(before_samples[i].x, after_curve->detail.sample_points[i].x, 1e-9) ||
        !almost_equal(before_samples[i].y, after_curve->detail.sample_points[i].y, 1e-9) ||
        !almost_equal(before_samples[i].z, after_curve->detail.sample_points[i].z, 1e-9)) {
      return false;
    }
  }
  return almost_equal(before_whole.min.x, after_bounds->whole.min.x, 1e-9) &&
         almost_equal(before_whole.min.y, after_bounds->whole.min.y, 1e-9) &&
         almost_equal(before_whole.min.z, after_bounds->whole.min.z, 1e-9) &&
         almost_equal(before_whole.max.x, after_bounds->whole.max.x, 1e-9) &&
         almost_equal(before_whole.max.y, after_bounds->whole.max.y, 1e-9) &&
         almost_equal(before_whole.max.z, after_bounds->whole.max.z, 1e-9);
}

bool C620_backbone_update_boundary_has_no_operation_specific_kinds() {
  std::string h;
  std::string cpp;
  if (!file_text(repo_root() / "core/include/wire/core/core_runtime_types.hpp", &h) ||
      !file_text(repo_root() / "core/src/generation/backbone/derive.cpp", &cpp)) {
    return false;
  }
  const std::vector<std::string> required = {"kRegenerate", "kReposition", "kReshape", "kRedraw"};
  for (const std::string& word : required) {
    if (!contains_text(h, word)) {
      return false;
    }
  }
  const std::vector<std::string> banned = {"PoleTilt", "PoleYaw", "PortHeight", "SagUpdate", "VisualStyleUpdate",
                                           "support_layout_contract", "support_layout_projection", "materialization",
                                           "ProcessDirtyQueues", "Commit("};
  for (const std::string& word : banned) {
    if (contains_text(h, word) || contains_text(cpp, word)) {
      return false;
    }
  }
  return true;
}

bool C674_backbone_port_band_selection_has_one_owner() {
  const auto occurrences = [](const std::string& text, const std::string& needle) {
    std::size_t count = 0;
    for (std::size_t pos = text.find(needle); pos != std::string::npos;
         pos = text.find(needle, pos + needle.size())) {
      ++count;
    }
    return count;
  };
  std::string state{};
  std::string endpoint{};
  std::string population{};
  std::string shared{};
  std::string emit{};
  if (!file_text(repo_root() / "core/src/state/state.cpp", &state) ||
      !file_text(repo_root() / "core/src/state/endpoint_refresh_service.cpp", &endpoint) ||
      !file_text(repo_root() / "core/src/generation/backbone/population.cpp", &population) ||
      !file_text(repo_root() / "core/src/state/port_placement.cpp", &shared) ||
      !file_text(repo_root() / "core/src/generation/backbone/emit_shared.cpp", &emit)) {
    return false;
  }
  const std::string priority_decision = "band.priority >";
  const std::size_t decision_count =
      occurrences(state, priority_decision) + occurrences(endpoint, priority_decision) +
      occurrences(population, priority_decision) + occurrences(shared, priority_decision) +
      occurrences(emit, priority_decision);
  return decision_count <= 2 && !contains_text(population, "identity_score") &&
         contains_text(population, "backbone_port_binding_for_port") &&
         contains_text(shared, "placement_band_id");
}

bool C675_backbone_layout_yaw_does_not_read_debug_records() {
  std::string state{};
  std::string body{};
  if (!file_text(repo_root() / "core/src/state/state.cpp", &state) ||
      !function_body(state, "double CoreState::effective_port_layout_yaw_deg", &body)) {
    return false;
  }
  return !contains_text(state, "pole_orientation_debug_records") &&
         contains_text(body, "backbone_port_binding_for_port") &&
         contains_text(body, "layout_yaw_deg");
}

bool C677_backbone_corner_scale_has_one_definition() {
  std::size_t scale_definitions = 0;
  std::size_t side_definitions = 0;
  const std::filesystem::path root = repo_root() / "core/src";
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".cpp") {
      continue;
    }
    std::string source{};
    if (!file_text(entry.path(), &source)) {
      return false;
    }
    for (std::size_t pos = source.find("double apply_corner_side_scale(");
         pos != std::string::npos;
         pos = source.find("double apply_corner_side_scale(", pos + 1)) {
      ++scale_definitions;
    }
    for (std::size_t pos = source.find("SlotSide inner_side_for_turn(");
         pos != std::string::npos;
         pos = source.find("SlotSide inner_side_for_turn(", pos + 1)) {
      ++side_definitions;
    }
  }
  return scale_definitions == 1 && side_definitions == 1;
}

} // namespace backbone_tests
