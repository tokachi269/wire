#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "city/wire/core_test_hook.hpp"
#include "city/wire/core_view.hpp"
#include "city/wire/coord_utils.hpp"

#include "../../src/generation/backbone/mount_graph.hpp"
#include "../../src/generation/backbone/model_placement_rules.hpp"
#include "../../src/generation/backbone/row_representation.hpp"
#include "../../src/support/instrumentation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace helpers;

namespace backbone_tests {

bool C422_backbone_rules_consume_topo_and_groups() {
  const std::filesystem::path header = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.hpp";
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string h;
  std::string cpp;
  if (!file_text(header, &h) || !file_text(source, &cpp)) {
    return false;
  }
  if (!contains_text(h, "rules make(const topo& made, const pairs& ps, const groups& placement) const")) {
    return false;
  }
  const std::size_t rules_pos =
      cpp.find("rules pipeline::make(const topo& made, const pairs& ps, const groups& placement) const");
  const std::size_t layout_pos = cpp.find("EditResult<layout> pipeline::make", rules_pos);
  if (rules_pos == std::string::npos || layout_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(rules_pos, layout_pos - rules_pos);
  return contains_text(body, "group_for") && !contains_text(body, "ps.jumpers") &&
         !contains_text(body, "ps.links");
}

bool C423_backbone_tspan_carries_endpoint_rows() {
  const std::filesystem::path header = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.hpp";
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.cpp";
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
  std::string body;
  if (!function_body(cpp, "EditResult<bool> pipeline::emit_spans(topo* made, const pairs& ps, ChangeSet* changes)",
                     &body)) {
    return false;
  }
  return contains_text(body, "edge.arow") && contains_text(body, "edge.brow") && contains_text(body, "tspan{");
}

bool C424_backbone_saves_backbone_graph_nodes_edges() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok) {
    return false;
  }
  const city::wire::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.nodes.size() != 2 || graph.edges.size() != 1) {
    return false;
  }
  const city::wire::SavedBackboneEdge& edge = graph.edges.front();
  const auto node_has_pole = [&](city::wire::ObjectId node_id) {
    const auto it = std::find_if(graph.nodes.begin(), graph.nodes.end(), [&](const city::wire::SavedBackboneNode& node) {
      return node.node_id == node_id && node.pole_id != city::wire::kInvalidObjectId;
    });
    return it != graph.nodes.end();
  };
  return edge.node_a != city::wire::kInvalidObjectId && edge.node_b != city::wire::kInvalidObjectId &&
         node_has_pole(edge.node_a) && node_has_pole(edge.node_b);
}

bool C425_backbone_edge_carries_multiple_spans() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, city::wire::BundleKind::kCommunication);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.size() < 2) {
    return false;
  }
  const city::wire::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.edges.size() != 1) {
    return false;
  }
  std::size_t span_count = 0;
  const auto edge_bundle_it = state.view().backbone_index().edge_bundles.find(graph.edges.front().edge_id);
  if (edge_bundle_it == state.view().backbone_index().edge_bundles.end()) {
    return false;
  }
  for (city::wire::ObjectId edge_bundle_id : edge_bundle_it->second) {
    const auto spans_it = state.view().backbone_index().edge_bundle_spans.find(edge_bundle_id);
    if (spans_it != state.view().backbone_index().edge_bundle_spans.end()) {
      span_count += spans_it->second.size();
    }
  }
  return span_count == out.value.generated_span_ids.size();
}

bool C426_backbone_existing_pole_resolves_graph_node() {
  city::wire::CoreState state;
  city::wire::BackboneSpec first = line_req(state);
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok || first_out.value.generated_pole_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId existing = first_out.value.generated_pole_ids.front();
  const auto* existing_pole = state.view().poles().find(existing);
  if (existing_pole == nullptr) {
    return false;
  }
  city::wire::BackboneSpec second = line_req(state);
  second.path.polyline = {existing_pole->world_transform.position, {20.0, 0.0, 0.0}};
  city::wire::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = city::wire::SupportKind::kPole;
  node.node_id = existing;
  second.path.node_specs.push_back(node);
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (!second_out.ok) {
    return false;
  }
  const city::wire::SavedBackboneGraph& graph = state.view().backbone();
  const auto pole_node_it = state.view().backbone_index().pole_node.find(existing);
  if (pole_node_it == state.view().backbone_index().pole_node.end()) {
    return false;
  }
  int matching_nodes = 0;
  for (const city::wire::SavedBackboneNode& saved : graph.nodes) {
    if (saved.pole_id == existing) {
      ++matching_nodes;
    }
  }
  return graph.nodes.size() == 3 && graph.edges.size() == 2 && matching_nodes == 1;
}

bool C427_backbone_graph_index_links_outputs() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok) {
    return false;
  }
  const city::wire::SavedBackboneGraph& graph = state.view().backbone();
  const city::wire::BackboneIndex& index = state.view().backbone_index();
  if (graph.nodes.size() != 2 || graph.edges.size() != 1) {
    return false;
  }
  const city::wire::ObjectId edge_id = graph.edges.front().edge_id;
  const auto edge_bundles = index.edge_bundles.find(edge_id);
  if (edge_bundles == index.edge_bundles.end() || edge_bundles->second.empty()) {
    return false;
  }
  for (const city::wire::SavedBackboneNode& node : graph.nodes) {
    if (node.pole_id == city::wire::kInvalidObjectId || index.pole_node.find(node.pole_id) == index.pole_node.end()) {
      return false;
    }
    const auto node_edges = index.node_edges.find(node.node_id);
    if (node_edges == index.node_edges.end() || node_edges->second.empty()) {
      return false;
    }
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
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

bool C428_backbone_pole_frontier_collects_incident_graph() {
  city::wire::CoreState state;
  city::wire::BackboneSpec first = poly3_req(state);
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok || first_out.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first_out.value.generated_pole_ids[1];
  const auto* b_pole = state.view().poles().find(b);
  if (b_pole == nullptr) {
    return false;
  }

  city::wire::BackboneSpec second = line_req(state);
  second.path.polyline = {b_pole->world_transform.position, {20.0, 0.0, 0.0}};
  city::wire::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = city::wire::SupportKind::kPole;
  node.node_id = b;
  second.path.node_specs.push_back(node);
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (!second_out.ok) {
    return false;
  }

  const city::wire::BackboneFrontier frontier = state.view().pole_frontier(b);
  return frontier.pole_id == b && frontier.node_id != city::wire::kInvalidObjectId && frontier.edge_ids.size() == 3 &&
         frontier.pole_ids.size() == 4 &&
         frontier.span_ids.size() == first_out.value.generated_span_ids.size() + second_out.value.generated_span_ids.size();
}

bool C429_backbone_span_frontier_collects_edge_bundle_spans() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, city::wire::BundleKind::kCommunication);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::BackboneFrontier frontier = state.view().span_frontier(out.value.generated_span_ids.front());
  return frontier.span_id == out.value.generated_span_ids.front() &&
         frontier.edge_id != city::wire::kInvalidObjectId && frontier.edge_ids.size() == 1 &&
         frontier.node_ids.size() == 2 && frontier.pole_ids.size() == 2 &&
         frontier.span_ids.size() == out.value.generated_span_ids.size();
}

bool C430_backbone_frontier_uses_saved_graph_index() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "state" / "core_view.cpp";
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
         !contains_text(body, "SavedBackboneEdges") && !contains_text(body, "spans_by_port");
}

bool C431_backbone_edge_bundle_is_saved_backbone_unit() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.bundle_ids.size() != 1 || out.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.edges.size() != 1 || graph.edge_bundles.size() != 1) {
    return false;
  }
  const city::wire::SavedBackboneEdgeBundle& item = graph.edge_bundles.front();
  if (item.edge_id != graph.edges.front().edge_id || item.bundle_id != out.value.bundle_ids.front() ||
      item.span_ids.size() != out.value.generated_span_ids.size()) {
    return false;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    if (!contains_id(item.span_ids, span_id)) {
      return false;
    }
  }
  return true;
}

bool C432_backbone_multiple_bundles_create_multiple_edge_bundles() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, city::wire::BundleKind::kCommunication);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.bundle_ids.size() != 2) {
    return false;
  }
  const city::wire::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.edges.size() != 1 || graph.edge_bundles.size() != out.value.bundle_ids.size()) {
    return false;
  }
  for (city::wire::ObjectId bundle_id : out.value.bundle_ids) {
    const auto it = std::find_if(graph.edge_bundles.begin(), graph.edge_bundles.end(),
                                 [&](const city::wire::SavedBackboneEdgeBundle& item) {
                                   return item.edge_id == graph.edges.front().edge_id && item.bundle_id == bundle_id &&
                                          !item.span_ids.empty();
                                 });
    if (it == graph.edge_bundles.end()) {
      return false;
    }
  }
  return true;
}

bool C433_backbone_resolves_edge_for_same_poles() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const city::wire::ObjectId a = first.value.generated_pole_ids[0];
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  city::wire::BackboneSpec second = line_req(state);
  second.bundles.clear();
  add_backbone_bundle(second, city::wire::BundleKind::kCommunication);
  second.path.polyline = {pa->world_transform.position, pb->world_transform.position};
  city::wire::BackboneInputSpec::NodeSpec na{};
  na.point_index = 0;
  na.support_kind = city::wire::SupportKind::kPole;
  na.node_id = a;
  city::wire::BackboneInputSpec::NodeSpec nb{};
  nb.point_index = 1;
  nb.support_kind = city::wire::SupportKind::kPole;
  nb.node_id = b;
  second.path.node_specs = {na, nb};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  return second_out.ok && state.view().backbone().nodes.size() == 2 && state.view().backbone().edges.size() == 1 &&
         state.view().backbone().edge_bundles.size() == 2;
}

bool C434_backbone_reverse_duplicate_same_bundle_rejected() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const city::wire::ObjectId a = first.value.generated_pole_ids[0];
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  city::wire::BackboneSpec second = line_req(state);
  second.path.polyline = {pb->world_transform.position, pa->world_transform.position};
  city::wire::BackboneInputSpec::NodeSpec nb{};
  nb.point_index = 0;
  nb.support_kind = city::wire::SupportKind::kPole;
  nb.node_id = b;
  city::wire::BackboneInputSpec::NodeSpec na{};
  na.point_index = 1;
  na.support_kind = city::wire::SupportKind::kPole;
  na.node_id = a;
  second.path.node_specs = {nb, na};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  return !second_out.ok && contains_text(second_out.error, "duplicate saved span binding") &&
         state.view().backbone().edges.size() == 1 && state.view().backbone().edge_bundles.size() == 1 &&
         state.view().backbone().edge_bundles.front().span_ids.size() == first.value.generated_span_ids.size();
}

bool C435_backbone_edge_metadata_is_not_overwritten_on_duplicate_reject() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2 || state.view().backbone().edges.size() != 1) {
    return false;
  }
  const city::wire::SavedBackboneEdge before = state.view().backbone().edges.front();
  const city::wire::ObjectId a = first.value.generated_pole_ids[0];
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  city::wire::BackboneSpec second = line_req(state);
  second.path.polyline = {pb->world_transform.position, pa->world_transform.position};
  city::wire::BackboneInputSpec::NodeSpec nb{};
  nb.point_index = 0;
  nb.support_kind = city::wire::SupportKind::kPole;
  nb.node_id = b;
  city::wire::BackboneInputSpec::NodeSpec na{};
  na.point_index = 1;
  na.support_kind = city::wire::SupportKind::kPole;
  na.node_id = a;
  second.path.node_specs = {nb, na};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (second_out.ok || !contains_text(second_out.error, "duplicate saved span binding") ||
      state.view().backbone().edges.size() != 1) {
    return false;
  }
  const city::wire::SavedBackboneEdge& after = state.view().backbone().edges.front();
  return after.edge_id == before.edge_id && after.node_a == before.node_a && after.node_b == before.node_b &&
         after.route == before.route && after.order == before.order && almost_equal(after.dir.x, before.dir.x, 1e-9) &&
         almost_equal(after.dir.y, before.dir.y, 1e-9) && almost_equal(after.dir.z, before.dir.z, 1e-9);
}

bool C436_backbone_frontier_reads_edge_bundles() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, city::wire::BundleKind::kCommunication);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::BackboneFrontier pole_frontier = state.view().pole_frontier(out.value.generated_pole_ids.front());
  const city::wire::BackboneFrontier span_frontier = state.view().span_frontier(out.value.generated_span_ids.front());
  return pole_frontier.edge_ids.size() == 1 && pole_frontier.edge_bundle_ids.size() == out.value.bundle_ids.size() &&
         pole_frontier.bundle_ids.size() == out.value.bundle_ids.size() &&
         pole_frontier.span_ids.size() == out.value.generated_span_ids.size() &&
         span_frontier.edge_bundle_id != city::wire::kInvalidObjectId && span_frontier.edge_bundle_ids.size() == 2 &&
         span_frontier.bundle_ids.size() == 2 && span_frontier.span_ids.size() == out.value.generated_span_ids.size();
}

bool C437_backbone_layout_save_is_direct() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "state" / "span_runtime.cpp";
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

bool C438_backbone_layout_save_keeps_no_authority_contract() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout()) {
      return false;
    }
  }
  return true;
}

bool C441_backbone_save_backbone_edge_returns_saved_ref() {
  const std::filesystem::path header = repo_root() / "domains" / "wire" / "include" / "city" / "wire" / "core_state.hpp";
  const std::filesystem::path types = repo_root() / "domains" / "wire" / "include" / "city" / "wire" / "core_authoritative_types.hpp";
  std::string h;
  std::string t;
  if (!file_text(header, &h) || !file_text(types, &t)) {
    return false;
  }
  return contains_text(t, "struct SavedBackboneEdgeRef") &&
         contains_text(h, "SavedBackboneEdgeRef save_backbone_edge");
}

bool C442_backbone_edge_forward_uses_saved_ref() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  std::string body;
  if (!function_body(cpp, "EditResult<bool> pipeline::save_graph(const topo& made, const pairs& ps,", &body)) {
    return false;
  }
  return contains_text(body, "std::vector<SavedBackboneEdgeRef>") && contains_text(body, "stored.node_a") &&
         contains_text(body, "stored.node_b") && !contains_text(body, "authoritative_.backbone") &&
         !contains_text(body, "const SavedBackboneEdge*") && !contains_text(body, "find_if");
}

bool C443_backbone_edge_resolution_behavior_unchanged() {
  return C434_backbone_reverse_duplicate_same_bundle_rejected();
}

bool C444_backbone_layout_uses_neutral_types() {
  const std::filesystem::path dir = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone";
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

bool C445_backbone_cache_span_layout_accepts_neutral_entry() {
  const std::filesystem::path header = repo_root() / "domains" / "wire" / "include" / "city" / "wire" / "core_state.hpp";
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "state" / "span_runtime.cpp";
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

bool C447_backbone_span_layout_view_reads_neutral_layout() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    const city::wire::SpanLayoutView view = state.span_layout(span_id);
    const city::wire::SpanLayoutEntry* entry = view.entry;
    if (!view.has_layout() || entry == nullptr || entry->span_id != span_id) {
      return false;
    }
  }
  return true;
}

bool C448_backbone_tests_use_neutral_layout_read() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "tests" / "backbone";
  std::string cpp{};
  for (const auto& entry : std::filesystem::directory_iterator(source)) {
    std::string part{};
    if (entry.path().extension() == ".cpp" && file_text(entry.path(), &part)) {
      cpp += part;
    }
  }
  const std::string old_read = std::string("support_layout_") + "projection(";
  return contains_text(cpp, "span_layout(") && !contains_text(cpp, old_read);
}

bool C450_backbone_span_layout_state_is_neutral() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    const city::wire::SpanLayoutState state_view = state.span_layout_state(span_id);
    if (!state_view.has_rules || !state_view.has_layout) {
      return false;
    }
  }
  return true;
}

bool C451_backbone_tests_do_not_read_old_contract() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "tests" / "backbone";
  std::string cpp{};
  for (const auto& entry : std::filesystem::directory_iterator(source)) {
    std::string part{};
    if (entry.path().extension() == ".cpp" && file_text(entry.path(), &part)) {
      cpp += part;
    }
  }
  const std::string old_read = std::string("support_layout_") + "contract(";
  return !contains_text(cpp, old_read);
}

bool C453_backbone_layout_state_reads_existing_cache_without_seed_path() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "state" / "span_runtime.cpp";
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

bool C454_backbone_cache_state_uses_span_layout_cache() {
  const std::filesystem::path header = repo_root() / "domains" / "wire" / "include" / "city" / "wire" / "core_runtime_types.hpp";
  std::string h;
  if (!file_text(header, &h)) {
    return false;
  }
  const std::size_t type_pos = h.find("struct CacheState");
  const std::size_t next_pos = h.find("inline constexpr double kDefaultCornerThresholdDeg", type_pos);
  if (type_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = h.substr(type_pos, next_pos - type_pos);
  return contains_text(body, "SpanLayoutCache span_layout_cache") &&
         !contains_text(body, "SupportLayoutCache support_layout_cache") &&
         !contains_text(body, "support_layout_cache");
}

bool C455_backbone_neutral_layout_api_uses_span_layout_cache() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "state" / "span_runtime.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t read_pos = cpp.find("SpanLayoutView CoreState::span_layout");
  const std::size_t read_end = cpp.find("void CoreState::cache_span_layout", read_pos);
  const std::size_t save_pos = cpp.find("void CoreState::cache_span_layout(SpanLayoutEntry layout)");
  const std::size_t curve_pos = cpp.find("void CoreState::cache_span_curve", save_pos);
  const std::size_t rules_pos = cpp.find("void CoreState::cache_span_rules");
  const std::size_t rules_end = cpp.find("void CoreState::remove_span_from_caches", rules_pos);
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

bool C458_backbone_existing_branch_BD_on_ABC() {
  city::wire::CoreState state;
  city::wire::BackboneSpec abc = poly3_req(state);
  const int count = req_bundle_count(state, abc);
  const auto first = state.GenerateFromBackboneSpec(abc);
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }

  city::wire::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  const city::wire::BackboneFrontier frontier = state.view().pole_frontier(b);
  return second.ok && second.value.generated_pole_ids.size() == 1 &&
         second.value.generated_span_ids.size() == static_cast<std::size_t>(count) &&
         frontier.edge_ids.size() == 3 && state.view().backbone().edges.size() == 3;
}

bool C459_backbone_existing_cross_DBE_on_ABC() {
  city::wire::CoreState state;
  city::wire::BackboneSpec abc = poly3_req(state);
  const int count = req_bundle_count(state, abc);
  const auto first = state.GenerateFromBackboneSpec(abc);
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }

  city::wire::BackboneSpec cross = line_req(state);
  cross.path.polyline = {{12.0, -8.0, 0.0}, pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  cross.path.node_specs = {pole_spec(1, b)};
  const auto second = state.GenerateFromBackboneSpec(cross);
  const city::wire::BackboneFrontier frontier = state.view().pole_frontier(b);
  return second.ok && second.value.generated_pole_ids.size() == 2 &&
         second.value.generated_span_ids.size() == static_cast<std::size_t>(count * 2) &&
         frontier.edge_ids.size() == 4 && state.view().backbone().edges.size() == 4;
}

namespace {

struct JunctionRowSnapshot {
  std::vector<std::tuple<bool, city::wire::ObjectId, city::wire::ObjectId>> row_keys{};
  std::vector<city::wire::Vec3d> port_positions{};
  std::vector<std::pair<city::wire::ObjectId, city::wire::ObjectId>> span_ports{};
  std::vector<std::vector<city::wire::ObjectId>> node_patch_edges{};
  std::size_t pair_rows = 0;
  std::size_t open_rows = 0;
  std::size_t support_levels = 0;
  std::size_t row_fixture_instances = 0;
  std::size_t endpoint_fixture_instances = 0;
};

city::wire::ObjectId edge_between(const city::wire::CoreState& state,
                                  city::wire::ObjectId a,
                                  city::wire::ObjectId b) {
  const city::wire::BackboneEdgeKey key{std::min(a, b), std::max(a, b)};
  const auto it = state.view().backbone_index().edge_by_nodes.find(key);
  return it == state.view().backbone_index().edge_by_nodes.end() ? city::wire::kInvalidObjectId : it->second;
}

city::wire::ObjectId edge_bundle_for_edge_and_lane(const city::wire::CoreState& state,
                                                   city::wire::ObjectId edge_id,
                                                   std::size_t lane_index) {
  const auto edge_bundles_it = state.view().backbone_index().edge_bundles.find(edge_id);
  if (edge_bundles_it == state.view().backbone_index().edge_bundles.end()) {
    return city::wire::kInvalidObjectId;
  }
  city::wire::ObjectId matched = city::wire::kInvalidObjectId;
  for (city::wire::ObjectId edge_bundle_id : edge_bundles_it->second) {
    const auto span_bindings_it = state.view().backbone_index().edge_bundle_span_bindings.find(edge_bundle_id);
    if (span_bindings_it == state.view().backbone_index().edge_bundle_span_bindings.end()) {
      continue;
    }
    for (std::size_t index : span_bindings_it->second) {
      if (index >= state.view().backbone().span_bindings.size()) {
        continue;
      }
      const city::wire::SavedBackboneSpanBinding& binding = state.view().backbone().span_bindings[index];
      if (binding.lane_index != lane_index) {
        continue;
      }
      if (matched != city::wire::kInvalidObjectId && matched != edge_bundle_id) {
        return city::wire::kInvalidObjectId;
      }
      matched = edge_bundle_id;
    }
  }
  return matched;
}

bool has_row_continuity(const city::wire::CoreState& state,
                        city::wire::ObjectId node_id,
                        city::wire::ObjectId edge_bundle_a,
                        std::size_t lane_a,
                        city::wire::ObjectId edge_bundle_b,
                        std::size_t lane_b) {
  for (const city::wire::SavedBackboneRowContinuity* continuity :
       state.view().backbone_row_continuities_for_node(node_id)) {
    if (continuity == nullptr) {
      continue;
    }
    const bool forward = continuity->a.edge_bundle_id == edge_bundle_a &&
                         continuity->a.lane_index == lane_a &&
                         continuity->b.edge_bundle_id == edge_bundle_b &&
                         continuity->b.lane_index == lane_b;
    const bool reverse = continuity->a.edge_bundle_id == edge_bundle_b &&
                         continuity->a.lane_index == lane_b &&
                         continuity->b.edge_bundle_id == edge_bundle_a &&
                         continuity->b.lane_index == lane_a;
    if (forward || reverse) {
      return true;
    }
  }
  return false;
}

bool edge_bundle_lane_exists(const city::wire::CoreState& state,
                             city::wire::ObjectId edge_bundle_id,
                             std::size_t lane_index) {
  if (state.view().backbone_edge_bundle(edge_bundle_id) == nullptr) {
    return false;
  }
  const auto spans_it = state.view().backbone_index().edge_bundle_span_bindings.find(edge_bundle_id);
  if (spans_it == state.view().backbone_index().edge_bundle_span_bindings.end()) {
    return false;
  }
  for (std::size_t index : spans_it->second) {
    if (index >= state.view().backbone().span_bindings.size()) {
      continue;
    }
    if (state.view().backbone().span_bindings[index].lane_index == lane_index) {
      return true;
    }
  }
  return false;
}

bool row_continuity_graph_lint_passes(const city::wire::CoreState& state) {
  std::vector<std::tuple<city::wire::ObjectId, city::wire::ObjectId, std::size_t>> endpoints{};
  for (const city::wire::SavedBackboneRowContinuity& continuity : state.view().backbone().row_continuities) {
    if (state.view().backbone_node(continuity.node_id) == nullptr ||
        !edge_bundle_lane_exists(state, continuity.a.edge_bundle_id, continuity.a.lane_index) ||
        !edge_bundle_lane_exists(state, continuity.b.edge_bundle_id, continuity.b.lane_index) ||
        continuity.a.edge_bundle_id == continuity.b.edge_bundle_id) {
      return false;
    }
    for (const auto& endpoint : {
             std::make_tuple(continuity.node_id, continuity.a.edge_bundle_id, continuity.a.lane_index),
             std::make_tuple(continuity.node_id, continuity.b.edge_bundle_id, continuity.b.lane_index)}) {
      if (std::find(endpoints.begin(), endpoints.end(), endpoint) != endpoints.end()) {
        return false;
      }
      endpoints.push_back(endpoint);
    }
  }
  return true;
}

bool has_row_key(const std::vector<std::tuple<bool, city::wire::ObjectId, city::wire::ObjectId>>& rows,
                 bool is_open,
                 city::wire::ObjectId a,
                 city::wire::ObjectId b) {
  const city::wire::ObjectId lo = is_open ? a : std::min(a, b);
  const city::wire::ObjectId hi = is_open ? b : std::max(a, b);
  return std::find(rows.begin(), rows.end(), std::make_tuple(is_open, lo, hi)) != rows.end();
}

std::vector<city::wire::ObjectId> unique_generated_ports_on_pole(
    const city::wire::CoreState& state,
    const std::vector<city::wire::ObjectId>& spans,
    city::wire::ObjectId pole_id) {
  std::vector<city::wire::ObjectId> out{};
  for (city::wire::ObjectId span_id : spans) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) continue;
    for (city::wire::ObjectId port_id : {span->port_a_id, span->port_b_id}) {
      const city::wire::Port* port = state.view().ports().find(port_id);
      if (port != nullptr && port->owner_pole_id == pole_id &&
          std::find(out.begin(), out.end(), port_id) == out.end()) {
        out.push_back(port_id);
      }
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

city::wire::ObjectId edge_bundle_for_template(const city::wire::CoreState& state,
                                              city::wire::ObjectId edge_id,
                                              city::wire::BundleKind kind) {
  const city::wire::BundleTemplateId template_id = city::wire::DefaultBundleTemplateId(kind);
  const auto bundles_it = state.view().backbone_index().edge_bundles.find(edge_id);
  if (bundles_it == state.view().backbone_index().edge_bundles.end()) {
    return city::wire::kInvalidObjectId;
  }
  for (city::wire::ObjectId edge_bundle_id : bundles_it->second) {
    const city::wire::SavedBackboneEdgeBundle* edge_bundle =
        state.view().backbone_edge_bundle(edge_bundle_id);
    const city::wire::Bundle* bundle =
        edge_bundle == nullptr ? nullptr : state.view().bundles().find(edge_bundle->bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == template_id) {
      return edge_bundle_id;
    }
  }
  return city::wire::kInvalidObjectId;
}

struct PortBindingSnapshot {
  std::size_t lane = 0;
  city::wire::ObjectId port_id = city::wire::kInvalidObjectId;
  city::wire::Vec3d position{};
  double layout_yaw_deg = 0.0;
};

std::vector<PortBindingSnapshot> port_binding_snapshot(const city::wire::CoreState& state,
                                                       city::wire::ObjectId edge_bundle_id,
                                                       city::wire::ObjectId node_id) {
  std::vector<PortBindingSnapshot> out{};
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.edge_bundle_id != edge_bundle_id || binding.row_key.node_id != node_id) {
      continue;
    }
    const city::wire::Port* port = state.view().ports().find(binding.port_id);
    if (port == nullptr) {
      continue;
    }
    out.push_back({binding.lane_index, binding.port_id, port->world_position, binding.layout_yaw_deg});
  }
  std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
    return std::tie(a.lane, a.port_id) < std::tie(b.lane, b.port_id);
  });
  return out;
}

bool same_port_binding_identity(const std::vector<PortBindingSnapshot>& a,
                                const std::vector<PortBindingSnapshot>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].lane != b[i].lane || a[i].port_id != b[i].port_id) {
      return false;
    }
  }
  return true;
}

std::optional<city::wire::Transformd> endpoint_fixture_transform(
    const city::wire::CoreState& state, city::wire::ObjectId port_id);

bool port_positions_match_explicit_lane_anchors(
    const city::wire::CoreState& state,
    const std::vector<PortBindingSnapshot>& snapshots,
    const std::vector<double>& layout_yaws_deg,
    double lateral_m,
    double spacing_m) {
  if (snapshots.size() != layout_yaws_deg.size()) {
    return false;
  }
  for (std::size_t i = 0; i < snapshots.size(); ++i) {
    const PortBindingSnapshot& snapshot = snapshots[i];
    const city::wire::Port* port = state.view().ports().find(snapshot.port_id);
    const city::wire::Pole* pole =
        port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
    if (port == nullptr || pole == nullptr) {
      return false;
    }
    const city::wire::PoleFrame frame =
        city::wire::BuildPoleFrame(pole->world_transform, layout_yaws_deg[i]);
    const city::wire::Vec3d local =
        city::wire::WorldPointToLocal(frame, port->world_position);
    const double expected_lateral =
        lateral_m +
        (static_cast<double>(snapshot.lane) -
         (static_cast<double>(snapshots.size() - 1) * 0.5)) *
            spacing_m;
    if (!almost_equal(local.y, expected_lateral, 1e-9)) {
      return false;
    }
  }
  return true;
}

bool endpoint_fixtures_match_explicit_lane_anchors(
    const city::wire::CoreState& state,
    const std::vector<PortBindingSnapshot>& snapshots,
    const std::vector<double>& layout_yaws_deg,
    double lateral_m,
    double spacing_m) {
  if (snapshots.size() != layout_yaws_deg.size()) {
    return false;
  }
  for (std::size_t i = 0; i < snapshots.size(); ++i) {
    const PortBindingSnapshot& snapshot = snapshots[i];
    const city::wire::Port* port = state.view().ports().find(snapshot.port_id);
    const city::wire::Pole* pole =
        port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
    const auto fixture = endpoint_fixture_transform(state, snapshot.port_id);
    if (port == nullptr || pole == nullptr || !fixture.has_value()) {
      return false;
    }
    const city::wire::PoleFrame frame =
        city::wire::BuildPoleFrame(pole->world_transform, layout_yaws_deg[i]);
    const city::wire::Vec3d local =
        city::wire::WorldPointToLocal(frame, fixture->position);
    const double expected_lateral =
        lateral_m +
        (static_cast<double>(snapshot.lane) -
         (static_cast<double>(snapshots.size() - 1) * 0.5)) *
            spacing_m;
    if (!almost_equal(local.y, expected_lateral, 1e-9)) {
      return false;
    }
  }
  return true;
}

std::optional<std::vector<double>> derived_row_yaws_for_ports(
    const city::wire::CoreState& state,
    const std::vector<PortBindingSnapshot>& snapshots) {
  std::vector<double> yaws{};
  yaws.reserve(snapshots.size());
  for (const PortBindingSnapshot& snapshot : snapshots) {
    const city::wire::SavedBackbonePortBinding* binding =
        state.view().backbone_port_binding_for_port(snapshot.port_id);
    if (binding == nullptr) {
      return std::nullopt;
    }
    const auto representation =
        city::wire::generation::backbone::DeriveEndpointRowRepresentation(
            state, *binding);
    if (!representation.ok) {
      return std::nullopt;
    }
    yaws.push_back(representation.value.layout_yaw_deg);
  }
  return yaws;
}

bool same_port_binding_geometry(const std::vector<PortBindingSnapshot>& a,
                                const std::vector<PortBindingSnapshot>& b,
                                bool require_distinct_ids) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].lane != b[i].lane ||
        !almost_equal(a[i].position, b[i].position, 1e-9) ||
        std::abs(city::wire::NormalizeYawDeg(
            a[i].layout_yaw_deg - b[i].layout_yaw_deg)) > 1e-9 ||
        (require_distinct_ids && a[i].port_id == b[i].port_id)) {
      return false;
    }
  }
  return true;
}

bool same_transform(const city::wire::Transformd& a, const city::wire::Transformd& b) {
  return almost_equal(a.position, b.position, 1e-9) &&
         almost_equal(a.rotation_euler_deg, b.rotation_euler_deg, 1e-9) &&
         almost_equal(a.scale, b.scale, 1e-9);
}

bool register_hv_fixture_models(city::wire::CoreState* state,
                                city::wire::ModelAssemblyTemplateId row_id,
                                city::wire::ModelAssemblyTemplateId endpoint_id) {
  if (state == nullptr) {
    return false;
  }
  city::wire::ModelAssemblyTemplate row_assembly{};
  row_assembly.id = row_id;
  city::wire::ModelAssemblyPart row_part{};
  row_part.part_id = 1;
  row_part.model_key = "hv_crossarm";
  row_part.sockets.push_back({"endpoint_mount", {0.0, 0.0, 0.04}, {0.0, 0.0, 1.0}});
  row_assembly.parts.push_back(row_part);
  row_assembly.endpoint_mount_socket = city::wire::AssemblySocketRef{1, "endpoint_mount"};

  city::wire::ModelAssemblyTemplate endpoint_assembly{};
  endpoint_assembly.id = endpoint_id;
  city::wire::ModelAssemblyPart endpoint_part{};
  endpoint_part.part_id = 1;
  endpoint_part.model_key = "hv_insulator";
  endpoint_part.sockets.push_back({"wire", {0.0, 0.0, 0.20}, {1.0, 0.0, 0.0}});
  endpoint_assembly.parts.push_back(endpoint_part);
  endpoint_assembly.wire_socket = city::wire::AssemblySocketRef{1, "wire"};

  if (!state->RegisterModelAssemblyTemplate(row_assembly).ok ||
      !state->RegisterModelAssemblyTemplate(endpoint_assembly).ok) {
    return false;
  }
  const city::wire::BundleTemplateId hv_template_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage);
  city::wire::BundleTemplate hv = state->view().bundle_templates().at(hv_template_id);
  hv.row_fixture_assembly_id = row_id;
  hv.endpoint_fixture_assembly_id = endpoint_id;
  return state->UpdateBundleTemplate(hv).ok;
}

city::wire::Vec3d hv_visible_socket(const city::wire::Transformd& fixture);
std::optional<city::wire::Vec3d> layout_endpoint_for_port(
    const city::wire::CoreState& state, city::wire::ObjectId port_id);

std::optional<city::wire::Transformd> endpoint_fixture_transform(
    const city::wire::CoreState& state, city::wire::ObjectId port_id) {
  const auto endpoint = layout_endpoint_for_port(state, port_id);
  if (!endpoint.has_value()) return std::nullopt;
  for (const city::wire::VisualModelInstance& instance :
       state.view().visual_model_instances().instances) {
    if (instance.model_key == "hv_insulator" &&
        almost_equal(hv_visible_socket(instance.world_transform), *endpoint, 1e-9)) {
      return instance.world_transform;
    }
  }
  return std::nullopt;
}

city::wire::Vec3d hv_visible_socket(const city::wire::Transformd& fixture) {
  const city::wire::Vec3d local_socket{0.0, 0.0, 0.20};
  return fixture.position +
         city::wire::RotateEulerXYZDeg({local_socket.x * fixture.scale.x,
                                        local_socket.y * fixture.scale.y,
                                        local_socket.z * fixture.scale.z},
                                       fixture.rotation_euler_deg);
}

std::optional<city::wire::Vec3d> layout_endpoint_for_port(const city::wire::CoreState& state,
                                                          city::wire::ObjectId port_id) {
  for (const city::wire::Span& span : state.view().spans().items()) {
    if (span.port_a_id != port_id && span.port_b_id != port_id) {
      continue;
    }
    const city::wire::SpanLayoutEntry* layout = state.span_layout(span.id).entry;
    if (layout == nullptr) {
      return std::nullopt;
    }
    if (layout->start.port_id == port_id) {
      return layout->start.endpoint_world;
    }
    if (layout->end.port_id == port_id) {
      return layout->end.endpoint_world;
    }
  }
  return std::nullopt;
}

std::vector<city::wire::Transformd> row_fixture_transforms_on_pole(
    const city::wire::CoreState& state, city::wire::ObjectId pole_id) {
  std::vector<city::wire::Transformd> out{};
  const std::string prefix = "row:" + std::to_string(pole_id) + ":";
  for (const city::wire::VisualModelInstance& instance :
       state.view().visual_model_instances().instances) {
    if (instance.model_key == "hv_crossarm" &&
        instance.stable_key.rfind(prefix, 0) == 0) {
      out.push_back(instance.world_transform);
    }
  }
  std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
    return std::tie(a.position.x, a.position.y, a.position.z,
                    a.rotation_euler_deg.x, a.rotation_euler_deg.y,
                    a.rotation_euler_deg.z) <
           std::tie(b.position.x, b.position.y, b.position.z,
                    b.rotation_euler_deg.x, b.rotation_euler_deg.y,
                    b.rotation_euler_deg.z);
  });
  return out;
}

bool contains_matching_transform(const std::vector<city::wire::Transformd>& transforms,
                                 const city::wire::Transformd& expected) {
  return std::any_of(transforms.begin(), transforms.end(), [&](const auto& candidate) {
    return same_transform(candidate, expected);
  });
}

bool curve_endpoints_match_layout(const city::wire::CoreState& state) {
  for (const city::wire::Span& span : state.view().spans().items()) {
    const city::wire::SpanLayoutEntry* layout = state.span_layout(span.id).entry;
    const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span.id);
    if (layout == nullptr || curve == nullptr || curve->detail.sample_points.size() < 2) {
      return false;
    }
    if (!almost_equal(curve->detail.sample_points.front(), layout->start.endpoint_world, 1e-9) ||
        !almost_equal(curve->detail.sample_points.back(), layout->end.endpoint_world, 1e-9)) {
      return false;
    }
  }
  return true;
}

JunctionRowSnapshot junction_snapshot(const city::wire::CoreState& state,
                                      city::wire::ObjectId pole_id) {
  JunctionRowSnapshot out{};
  const city::wire::SavedBackboneNode* node = state.view().backbone_node_for_pole(pole_id);
  if (node == nullptr) {
    return out;
  }
  std::vector<double> support_levels{};
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != node->node_id) {
      continue;
    }
    const city::wire::SavedBackboneEdgeBundle* own_edge_bundle =
        state.view().backbone_edge_bundle(binding.edge_bundle_id);
    if (own_edge_bundle == nullptr ||
        binding.row_key.edge_id != own_edge_bundle->edge_id) {
      continue;
    }
    city::wire::ObjectId peer_edge = city::wire::kInvalidObjectId;
    for (const city::wire::SavedBackboneRowContinuity& continuity :
         state.view().backbone().row_continuities) {
      if (continuity.node_id != binding.row_key.node_id) continue;
      const bool is_a =
          continuity.a.edge_bundle_id == binding.edge_bundle_id &&
          continuity.a.lane_index == binding.lane_index;
      const bool is_b =
          continuity.b.edge_bundle_id == binding.edge_bundle_id &&
          continuity.b.lane_index == binding.lane_index;
      if (!is_a && !is_b) continue;
      const city::wire::ObjectId peer_edge_bundle_id =
          is_a ? continuity.b.edge_bundle_id : continuity.a.edge_bundle_id;
      const city::wire::SavedBackboneEdgeBundle* peer_edge_bundle =
          state.view().backbone_edge_bundle(peer_edge_bundle_id);
      if (peer_edge != city::wire::kInvalidObjectId ||
          peer_edge_bundle == nullptr) {
        return {};
      }
      peer_edge = peer_edge_bundle->edge_id;
    }
    const bool is_open = peer_edge == city::wire::kInvalidObjectId;
    const city::wire::ObjectId a =
        is_open ? binding.row_key.edge_id
                : std::min(binding.row_key.edge_id, peer_edge);
    const city::wire::ObjectId b =
        is_open ? city::wire::kInvalidObjectId
                : std::max(binding.row_key.edge_id, peer_edge);
    const auto key = std::make_tuple(is_open, a, b);
    if (std::find(out.row_keys.begin(), out.row_keys.end(), key) == out.row_keys.end()) {
      out.row_keys.push_back(key);
      if (is_open) {
        ++out.open_rows;
      } else {
        ++out.pair_rows;
      }
    }
    const city::wire::Port* port = state.view().ports().find(binding.port_id);
    if (port != nullptr && port->owner_pole_id == pole_id) {
      out.port_positions.push_back(port->world_position);
      if (std::none_of(support_levels.begin(), support_levels.end(),
                       [&](double z) { return std::abs(z - port->world_position.z) <= 1e-9; })) {
        support_levels.push_back(port->world_position.z);
      }
    }
  }
  std::sort(out.row_keys.begin(), out.row_keys.end());
  std::sort(out.port_positions.begin(), out.port_positions.end(), [](const auto& a, const auto& b) {
    return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
  });
  out.support_levels = support_levels.size();
  for (const city::wire::Span& span : state.view().spans().items()) {
    const city::wire::Port* a = state.view().ports().find(span.port_a_id);
    const city::wire::Port* b = state.view().ports().find(span.port_b_id);
    if ((a != nullptr && a->owner_pole_id == pole_id) || (b != nullptr && b->owner_pole_id == pole_id)) {
      out.span_ports.push_back({span.port_a_id, span.port_b_id});
    }
  }
  std::sort(out.span_ports.begin(), out.span_ports.end());
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == city::wire::VisualCurvePartKind::kNodePatch && part.source_node_id == node->node_id) {
      out.node_patch_edges.push_back(part.incident_edge_ids);
    }
  }
  std::sort(out.node_patch_edges.begin(), out.node_patch_edges.end());
  for (const city::wire::VisualModelInstance& instance : state.view().visual_model_instances().instances) {
    out.row_fixture_instances += instance.stable_key.rfind("row:", 0) == 0 ? 1U : 0U;
    out.endpoint_fixture_instances += instance.stable_key.rfind("port:", 0) == 0 ? 1U : 0U;
  }
  return out;
}

bool snapshots_match(const JunctionRowSnapshot& a, const JunctionRowSnapshot& b) {
  if (a.row_keys.size() != b.row_keys.size() || a.pair_rows != b.pair_rows || a.open_rows != b.open_rows ||
      a.support_levels != b.support_levels || a.node_patch_edges.size() != b.node_patch_edges.size() ||
      a.row_fixture_instances != b.row_fixture_instances ||
      a.endpoint_fixture_instances != b.endpoint_fixture_instances ||
      a.port_positions.size() != b.port_positions.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.node_patch_edges.size(); ++i) {
    if (a.node_patch_edges[i].size() != b.node_patch_edges[i].size()) {
      return false;
    }
  }
  for (std::size_t i = 0; i < a.port_positions.size(); ++i) {
    if (!almost_equal(a.port_positions[i], b.port_positions[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

std::string junction_snapshot_counts(const JunctionRowSnapshot& value) {
  return "rows=" + std::to_string(value.row_keys.size()) +
         " pairs=" + std::to_string(value.pair_rows) +
         " opens=" + std::to_string(value.open_rows) +
         " levels=" + std::to_string(value.support_levels) +
         " patches=" + std::to_string(value.node_patch_edges.size()) +
         " row_fixtures=" + std::to_string(value.row_fixture_instances) +
         " endpoint_fixtures=" +
         std::to_string(value.endpoint_fixture_instances) +
         " ports=" + std::to_string(value.port_positions.size());
}

struct IncrementalCrossFixture {
  city::wire::CoreState state{};
  city::wire::ObjectId pole_b = city::wire::kInvalidObjectId;
  city::wire::ObjectId pole_d = city::wire::kInvalidObjectId;
  city::wire::ObjectId pole_e = city::wire::kInvalidObjectId;
  std::vector<city::wire::ObjectId> bd_spans{};
  std::vector<city::wire::ObjectId> bd_ports{};
  city::wire::EditResult<city::wire::GenerateBundleFromPathResult> completion{};
};

bool make_incremental_cross(IncrementalCrossFixture* out, bool reverse_completion = true,
                            city::wire::BundleKind completion_kind = city::wire::BundleKind::kLowVoltage,
                            city::wire::Vec3d d = {12.0, -8.0, 0.0},
                            city::wire::Vec3d e = {20.0, 0.0, 0.0}) {
  if (out == nullptr) {
    return false;
  }
  out->state = city::wire::CoreState{};
  const auto abc = out->state.GenerateFromBackboneSpec(poly3_req(out->state));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  out->pole_b = abc.value.generated_pole_ids[1];
  const city::wire::Pole* pole_b = out->state.view().poles().find(out->pole_b);
  if (pole_b == nullptr) return false;

  city::wire::BackboneSpec bd = line_req(out->state);
  bd.path.polyline = {pole_b->world_transform.position, d};
  bd.path.node_specs = {pole_spec(0, out->pole_b)};
  const auto bd_out = out->state.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_pole_ids.size() != 1 || bd_out.value.generated_span_ids.empty()) {
    return false;
  }
  out->pole_d = bd_out.value.generated_pole_ids.front();
  out->bd_spans = bd_out.value.generated_span_ids;
  out->bd_ports = unique_generated_ports_on_pole(out->state, out->bd_spans, out->pole_b);

  city::wire::BackboneSpec completion = line_req(out->state);
  if (completion_kind != city::wire::BundleKind::kLowVoltage) {
    completion.bundles.clear();
    add_backbone_bundle(completion, completion_kind);
  }
  completion.path.polyline = reverse_completion
                                 ? std::vector<city::wire::Vec3d>{e, pole_b->world_transform.position}
                                 : std::vector<city::wire::Vec3d>{pole_b->world_transform.position, e};
  completion.path.node_specs = {pole_spec(reverse_completion ? 1 : 0, out->pole_b)};
  out->completion = out->state.GenerateFromBackboneSpec(completion);
  if (!out->completion.ok || out->completion.value.generated_pole_ids.size() != 1) {
    return out->completion.ok;
  }
  out->pole_e = out->completion.value.generated_pole_ids.front();
  return true;
}

bool canonical_cross_at_b(const IncrementalCrossFixture& fixture) {
  const city::wire::SavedBackboneNode* node_b = fixture.state.view().backbone_node_for_pole(fixture.pole_b);
  const city::wire::SavedBackboneNode* node_d = fixture.state.view().backbone_node_for_pole(fixture.pole_d);
  const city::wire::SavedBackboneNode* node_e = fixture.state.view().backbone_node_for_pole(fixture.pole_e);
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr) return false;
  const city::wire::ObjectId bd = edge_between(fixture.state, node_b->node_id, node_d->node_id);
  const city::wire::ObjectId be = edge_between(fixture.state, node_b->node_id, node_e->node_id);
  const JunctionRowSnapshot snapshot = junction_snapshot(fixture.state, fixture.pole_b);
  return bd != city::wire::kInvalidObjectId && be != city::wire::kInvalidObjectId &&
         snapshot.pair_rows == 2 && snapshot.open_rows == 0 &&
         has_row_key(snapshot.row_keys, false, bd, be);
}

std::vector<double> hv_row_down_offsets_at_pole(const city::wire::CoreState& state,
                                                 city::wire::ObjectId pole_id) {
  std::vector<std::tuple<city::wire::ObjectId, city::wire::ObjectId, double>>
      rows{};
  const city::wire::SavedBackboneNode* node = state.view().backbone_node_for_pole(pole_id);
  if (node == nullptr) {
    return {};
  }
  const city::wire::BundleTemplateId hv_template_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage);
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != node->node_id ||
        binding.bundle_template_id != hv_template_id ||
        binding.lane_index != 0) {
      continue;
    }
    city::wire::ObjectId row_a = binding.edge_bundle_id;
    city::wire::ObjectId row_b = city::wire::kInvalidObjectId;
    for (const city::wire::SavedBackboneRowContinuity& continuity :
         state.view().backbone().row_continuities) {
      if (continuity.node_id != node->node_id) continue;
      const bool is_a =
          continuity.a.edge_bundle_id == binding.edge_bundle_id &&
          continuity.a.lane_index == binding.lane_index;
      const bool is_b =
          continuity.b.edge_bundle_id == binding.edge_bundle_id &&
          continuity.b.lane_index == binding.lane_index;
      if (!is_a && !is_b) continue;
      const city::wire::ObjectId peer =
          is_a ? continuity.b.edge_bundle_id : continuity.a.edge_bundle_id;
      row_a = std::min(binding.edge_bundle_id, peer);
      row_b = std::max(binding.edge_bundle_id, peer);
    }
    const auto existing =
        std::find_if(rows.begin(), rows.end(), [&](const auto& item) {
          return std::get<0>(item) == row_a && std::get<1>(item) == row_b;
        });
    if (existing != rows.end()) {
      continue;
    }
    bool found = false;
    double down_offset_m = 0.0;
    for (const city::wire::Span& span : state.view().spans().items()) {
      const city::wire::SpanLayoutView layout = state.span_layout(span.id);
      if (!layout.has_layout()) {
        continue;
      }
      const city::wire::LayoutEndpoint* endpoint = nullptr;
      if (layout.entry->start.port_id == binding.port_id) {
        endpoint = &layout.entry->start;
      } else if (layout.entry->end.port_id == binding.port_id) {
        endpoint = &layout.entry->end;
      }
      if (endpoint == nullptr) {
        continue;
      }
      if (found && !almost_equal(down_offset_m, endpoint->branch_down_offset_m, 1e-9)) {
        return {};
      }
      found = true;
      down_offset_m = endpoint->branch_down_offset_m;
    }
    if (!found) {
      return {};
    }
    rows.push_back({row_a, row_b, down_offset_m});
  }
  std::vector<double> offsets{};
  offsets.reserve(rows.size());
  for (const auto& [row_a, row_b, offset] : rows) {
    static_cast<void>(row_a);
    static_cast<void>(row_b);
    offsets.push_back(offset);
  }
  std::sort(offsets.begin(), offsets.end());
  return offsets;
}

std::vector<std::pair<int, double>> hv_endpoint_levels_at_pole(
    const city::wire::CoreState& state, city::wire::ObjectId pole_id) {
  std::vector<std::pair<int, double>> out{};
  const city::wire::SavedBackboneNode* node =
      state.view().backbone_node_for_pole(pole_id);
  if (node == nullptr) {
    return out;
  }
  const city::wire::BundleTemplateId hv_template_id =
      city::wire::DefaultBundleTemplateId(
          city::wire::BundleKind::kHighVoltage);
  for (const city::wire::SavedBackbonePortBinding& binding :
       state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != node->node_id ||
        binding.bundle_template_id != hv_template_id ||
        binding.lane_index != 0) {
      continue;
    }
    const city::wire::LayoutEndpoint* matched = nullptr;
    for (const city::wire::Span& span : state.view().spans().items()) {
      const city::wire::SpanLayoutView layout = state.span_layout(span.id);
      if (!layout.has_layout()) {
        continue;
      }
      const city::wire::LayoutEndpoint* candidate = nullptr;
      if (layout.entry->start.port_id == binding.port_id) {
        candidate = &layout.entry->start;
      } else if (layout.entry->end.port_id == binding.port_id) {
        candidate = &layout.entry->end;
      }
      if (candidate == nullptr) {
        continue;
      }
      if (matched != nullptr &&
          !almost_equal(matched->branch_down_offset_m,
                        candidate->branch_down_offset_m, 1e-9)) {
        return {};
      }
      matched = candidate;
    }
    if (matched == nullptr) {
      return {};
    }
    out.push_back({binding.support_level,
                   matched->branch_down_offset_m});
  }
  std::sort(out.begin(), out.end());
  return out;
}

int hv_lane0_support_level_for_edge_at_pole(
    const city::wire::CoreState& state, city::wire::ObjectId pole_id,
    city::wire::ObjectId edge_id) {
  const city::wire::SavedBackboneNode* node =
      state.view().backbone_node_for_pole(pole_id);
  if (node == nullptr) {
    return -1;
  }
  const city::wire::BundleTemplateId hv_template_id =
      city::wire::DefaultBundleTemplateId(
          city::wire::BundleKind::kHighVoltage);
  int level = -1;
  for (const city::wire::SavedBackbonePortBinding& binding :
       state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != node->node_id ||
        binding.row_key.edge_id != edge_id ||
        binding.bundle_template_id != hv_template_id ||
        binding.lane_index != 0) {
      continue;
    }
    if (level >= 0 && level != binding.support_level) {
      return -1;
    }
    level = binding.support_level;
  }
  return level;
}

const city::wire::SavedBackbonePortBinding* hv_lane0_binding_for_edge_at_pole(
    const city::wire::CoreState& state, city::wire::ObjectId pole_id,
    city::wire::ObjectId edge_id) {
  const city::wire::SavedBackboneNode* node =
      state.view().backbone_node_for_pole(pole_id);
  if (node == nullptr) {
    return nullptr;
  }
  const city::wire::BundleTemplateId hv_template_id =
      city::wire::DefaultBundleTemplateId(
          city::wire::BundleKind::kHighVoltage);
  const city::wire::SavedBackbonePortBinding* found = nullptr;
  for (const city::wire::SavedBackbonePortBinding& binding :
       state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != node->node_id ||
        binding.row_key.edge_id != edge_id ||
        binding.bundle_template_id != hv_template_id ||
        binding.lane_index != 0) {
      continue;
    }
    if (found != nullptr) {
      return nullptr;
    }
    found = &binding;
  }
  return found;
}

std::optional<double> local_port_height_for_binding(
    const city::wire::CoreState& state,
    const city::wire::SavedBackbonePortBinding& binding) {
  const city::wire::Port* port = state.view().ports().find(binding.port_id);
  const city::wire::Pole* pole =
      port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
  if (port == nullptr || pole == nullptr) {
    return std::nullopt;
  }
  const city::wire::PoleFrame frame =
      city::wire::BuildPoleFrame(pole->world_transform, binding.layout_yaw_deg);
  return city::wire::WorldPointToLocal(frame, port->world_position).z;
}

std::optional<double> nominal_band_height_for_binding(
    const city::wire::CoreState& state,
    const city::wire::SavedBackbonePortBinding& binding) {
  const city::wire::Port* port = state.view().ports().find(binding.port_id);
  const city::wire::Pole* pole =
      port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
  if (pole == nullptr) {
    return std::nullopt;
  }
  const auto pole_type_it = state.view().pole_types().find(pole->pole_type_id);
  if (pole_type_it == state.view().pole_types().end()) {
    return std::nullopt;
  }
  const auto band_it = std::find_if(
      pole_type_it->second.port_bands.begin(),
      pole_type_it->second.port_bands.end(),
      [&](const city::wire::PortPlacementBand& band) {
        return band.band_id == binding.placement_band_id;
      });
  if (band_it == pole_type_it->second.port_bands.end()) {
    return std::nullopt;
  }
  return band_it->height_center_m;
}

} // namespace

bool C771_backbone_incremental_cross_completion_matches_one_shot_rows() {
  city::wire::CoreState one_shot;
  const auto one_abc = one_shot.GenerateFromBackboneSpec(poly3_req(one_shot));
  if (!one_abc.ok || one_abc.value.generated_pole_ids.size() != 3) return false;
  const city::wire::ObjectId one_b = one_abc.value.generated_pole_ids[1];
  const city::wire::Pole* one_pole_b = one_shot.view().poles().find(one_b);
  if (one_pole_b == nullptr || one_shot.view().backbone_node_for_pole(one_b) == nullptr) return false;
  city::wire::BackboneSpec one_cross = line_req(one_shot);
  one_cross.path.polyline = {{12.0, -8.0, 0.0}, one_pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  one_cross.path.node_specs = {pole_spec(1, one_b)};
  const auto one_out = one_shot.GenerateFromBackboneSpec(one_cross);
  if (!one_out.ok) return false;
  const city::wire::ObjectId one_d = one_out.value.generated_pole_ids.front();
  const city::wire::ObjectId one_e = one_out.value.generated_pole_ids.back();
  const city::wire::SavedBackboneNode* one_node_b = one_shot.view().backbone_node_for_pole(one_b);
  const city::wire::SavedBackboneNode* one_node_d = one_shot.view().backbone_node_for_pole(one_d);
  const city::wire::SavedBackboneNode* one_node_e = one_shot.view().backbone_node_for_pole(one_e);
  if (one_node_b == nullptr || one_node_d == nullptr || one_node_e == nullptr) return false;
  const city::wire::ObjectId one_bd = edge_between(one_shot, one_node_b->node_id, one_node_d->node_id);
  const city::wire::ObjectId one_be = edge_between(one_shot, one_node_b->node_id, one_node_e->node_id);
  const JunctionRowSnapshot expected = junction_snapshot(one_shot, one_b);
  if (one_bd == city::wire::kInvalidObjectId || one_be == city::wire::kInvalidObjectId ||
      expected.pair_rows != 2 || expected.open_rows != 0 ||
      !has_row_key(expected.row_keys, false, one_bd, one_be) || !curve_endpoints_match_layout(one_shot)) {
    return false;
  }

  city::wire::CoreState incremental;
  const auto inc_abc = incremental.GenerateFromBackboneSpec(poly3_req(incremental));
  if (!inc_abc.ok || inc_abc.value.generated_pole_ids.size() != 3) return false;
  const city::wire::ObjectId inc_b = inc_abc.value.generated_pole_ids[1];
  const city::wire::Pole* inc_pole_b = incremental.view().poles().find(inc_b);
  if (inc_pole_b == nullptr || incremental.view().backbone_node_for_pole(inc_b) == nullptr) return false;

  city::wire::BackboneSpec bd = line_req(incremental);
  bd.path.polyline = {inc_pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd.path.node_specs = {pole_spec(0, inc_b)};
  const auto bd_out = incremental.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_pole_ids.size() != 1 ||
      bd_out.value.generated_span_ids.empty()) return false;
  const city::wire::ObjectId inc_d = bd_out.value.generated_pole_ids.front();
  const std::vector<city::wire::ObjectId> bd_ports_before =
      unique_generated_ports_on_pole(incremental, bd_out.value.generated_span_ids, inc_b);
  const std::vector<city::wire::ObjectId> bd_spans_before = bd_out.value.generated_span_ids;

  city::wire::BackboneSpec eb = line_req(incremental);
  eb.path.polyline = {{20.0, 0.0, 0.0}, inc_pole_b->world_transform.position};
  eb.path.node_specs = {pole_spec(1, inc_b)};
  const auto eb_out = incremental.GenerateFromBackboneSpec(eb);
  if (!eb_out.ok || eb_out.value.generated_pole_ids.size() != 1 ||
      eb_out.value.generated_span_ids.empty()) return false;
  const city::wire::ObjectId inc_e = eb_out.value.generated_pole_ids.front();
  const city::wire::SavedBackboneNode* inc_node_b = incremental.view().backbone_node_for_pole(inc_b);
  const city::wire::SavedBackboneNode* inc_node_d = incremental.view().backbone_node_for_pole(inc_d);
  const city::wire::SavedBackboneNode* inc_node_e = incremental.view().backbone_node_for_pole(inc_e);
  if (inc_node_b == nullptr || inc_node_d == nullptr || inc_node_e == nullptr) return false;
  const city::wire::ObjectId inc_bd = edge_between(incremental, inc_node_b->node_id, inc_node_d->node_id);
  const city::wire::ObjectId inc_be = edge_between(incremental, inc_node_b->node_id, inc_node_e->node_id);
  const JunctionRowSnapshot actual = junction_snapshot(incremental, inc_b);
  const std::vector<city::wire::ObjectId> bd_ports_after =
      unique_generated_ports_on_pole(incremental, bd_spans_before, inc_b);

  return inc_bd != city::wire::kInvalidObjectId && inc_be != city::wire::kInvalidObjectId &&
         actual.pair_rows == 2 && actual.open_rows == 0 &&
         has_row_key(actual.row_keys, false, inc_bd, inc_be) &&
         bd_ports_before == bd_ports_after &&
         std::all_of(bd_spans_before.begin(), bd_spans_before.end(), [&](city::wire::ObjectId span_id) {
           return incremental.view().spans().find(span_id) != nullptr;
         }) &&
         eb_out.value.generated_span_ids.size() == bd_spans_before.size() &&
         curve_endpoints_match_layout(incremental);
}

bool C772_backbone_incremental_pair_promotion_leaves_ambiguous_candidates_open() {
  city::wire::CoreState state;
  const auto abc = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const city::wire::ObjectId b = abc.value.generated_pole_ids[1];
  const city::wire::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  auto add_open = [&](city::wire::Vec3d end) {
    city::wire::BackboneSpec branch = line_req(state);
    branch.path.polyline = {end, pole_b->world_transform.position};
    branch.path.node_specs = {pole_spec(1, b)};
    return state.GenerateFromBackboneSpec(branch);
  };
  const auto bd = add_open({12.0, -8.0, 0.0});
  const auto bf = add_open({8.0, -8.0, 0.0});
  if (!bd.ok || !bf.ok) return false;
  const std::size_t pole_count = state.view().poles().size();
  const std::size_t span_count = state.view().spans().size();
  const std::size_t edge_count = state.view().backbone().edges.size();
  const std::size_t binding_count = state.view().backbone().port_bindings.size();
  const std::size_t continuity_count =
      state.view().backbone().row_continuities.size();

  city::wire::BackboneSpec completion = line_req(state);
  completion.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  completion.path.node_specs = {pole_spec(0, b)};
  const auto out = state.GenerateFromBackboneSpec(completion);
  const JunctionRowSnapshot snapshot = junction_snapshot(state, b);
  return out.ok && state.view().poles().size() == pole_count + 1 &&
         state.view().spans().size() == span_count + 1 &&
         state.view().backbone().edges.size() == edge_count + 1 &&
         state.view().backbone().port_bindings.size() > binding_count &&
         state.view().backbone().row_continuities.size() ==
             continuity_count &&
         snapshot.open_rows >= 1;
}

bool C773_backbone_incremental_sharp_completion_derives_jumper_from_continuity() {
  IncrementalCrossFixture fixture{};
  WIRE_TEST_EXPECT(make_incremental_cross(&fixture, true, city::wire::BundleKind::kLowVoltage,
                                          {12.0, -8.0, 0.0}, {13.0, -10.0, 0.0}),
                   "incremental sharp fixture generation failed");
  WIRE_TEST_EXPECT(fixture.completion.ok, fixture.completion.error);
  const city::wire::SavedBackboneNode* node_b = fixture.state.view().backbone_node_for_pole(fixture.pole_b);
  const city::wire::SavedBackboneNode* node_d = fixture.state.view().backbone_node_for_pole(fixture.pole_d);
  const city::wire::SavedBackboneNode* node_e = fixture.state.view().backbone_node_for_pole(fixture.pole_e);
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr) return false;
  const city::wire::ObjectId bd = edge_between(fixture.state, node_b->node_id, node_d->node_id);
  const city::wire::ObjectId be = edge_between(fixture.state, node_b->node_id, node_e->node_id);
  const JunctionRowSnapshot snapshot = junction_snapshot(fixture.state, fixture.pole_b);
  WIRE_TEST_EXPECT(bd != city::wire::kInvalidObjectId && be != city::wire::kInvalidObjectId,
                   "sharp completion edge lookup failed");
  WIRE_TEST_EXPECT(snapshot.pair_rows == 2 && snapshot.open_rows == 0,
                   "sharp completion row counts are wrong");
  WIRE_TEST_EXPECT(has_row_key(snapshot.row_keys, false, bd, be),
                   "sharp completion pair row key is missing");
  WIRE_TEST_EXPECT(fixture.bd_ports == unique_generated_ports_on_pole(fixture.state, fixture.bd_spans, fixture.pole_b),
                   "sharp completion changed existing BD ports");
  WIRE_TEST_EXPECT(std::count_if(
                       fixture.state.visual_curve_parts().parts.begin(),
                       fixture.state.visual_curve_parts().parts.end(),
                       [](const city::wire::VisualCurvePart& part) {
                         return part.kind ==
                                city::wire::VisualCurvePartKind::kJumper;
                       }) >= 1,
                   "sharp completion did not derive a jumper");
  WIRE_TEST_EXPECT(curve_endpoints_match_layout(fixture.state), "curve endpoints do not match layout");
  std::string invariant_error{};
  WIRE_TEST_EXPECT(backbone_common_invariants_pass(fixture.state, &invariant_error), invariant_error);
  return true;
}

bool C782_backbone_incremental_sharp_extension_adds_open_when_sharp_candidates_are_ambiguous() {
  city::wire::CoreState state;
  const auto abc = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const city::wire::ObjectId b = abc.value.generated_pole_ids[1];
  const city::wire::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  city::wire::BackboneSpec bd_req = line_req(state);
  bd_req.path.polyline = {pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd_req.path.node_specs = {pole_spec(0, b)};
  const auto bd = state.GenerateFromBackboneSpec(bd_req);
  if (!bd.ok || bd.value.generated_pole_ids.size() != 1) return false;
  const std::vector<city::wire::ObjectId> bd_ports =
      unique_generated_ports_on_pole(state, bd.value.generated_span_ids, b);

  city::wire::BackboneSpec eb_req = line_req(state);
  eb_req.path.polyline = {{13.0, -10.0, 0.0}, pole_b->world_transform.position};
  eb_req.path.node_specs = {pole_spec(1, b)};
  const auto eb = state.GenerateFromBackboneSpec(eb_req);
  if (!eb.ok || eb.value.generated_pole_ids.size() != 1 || eb.value.generated_span_ids.empty()) {
    return false;
  }

  city::wire::BackboneSpec bg_req = line_req(state);
  bg_req.path.polyline = {pole_b->world_transform.position, {14.0, -12.0, 0.0}};
  bg_req.path.node_specs = {pole_spec(0, b)};
  const auto bg = state.GenerateFromBackboneSpec(bg_req);
  if (!bg.ok || bg.value.generated_pole_ids.size() != 1 || bg.value.generated_span_ids.empty()) {
    return false;
  }

  const city::wire::SavedBackboneNode* node_b = state.view().backbone_node_for_pole(b);
  const city::wire::SavedBackboneNode* node_d =
      state.view().backbone_node_for_pole(bd.value.generated_pole_ids.front());
  const city::wire::SavedBackboneNode* node_e =
      state.view().backbone_node_for_pole(eb.value.generated_pole_ids.front());
  const city::wire::SavedBackboneNode* node_g =
      state.view().backbone_node_for_pole(bg.value.generated_pole_ids.front());
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr || node_g == nullptr) return false;
  const city::wire::ObjectId bd_edge = edge_between(state, node_b->node_id, node_d->node_id);
  const city::wire::ObjectId be_edge = edge_between(state, node_b->node_id, node_e->node_id);
  const city::wire::ObjectId bg_edge = edge_between(state, node_b->node_id, node_g->node_id);
  const JunctionRowSnapshot snapshot = junction_snapshot(state, b);
  return bd_edge != city::wire::kInvalidObjectId && be_edge != city::wire::kInvalidObjectId &&
         bg_edge != city::wire::kInvalidObjectId &&
         snapshot.pair_rows == 2 && snapshot.open_rows == 1 &&
         has_row_key(snapshot.row_keys, false, bd_edge, be_edge) &&
         has_row_key(snapshot.row_keys, true, bg_edge, city::wire::kInvalidObjectId) &&
         bd_ports == unique_generated_ports_on_pole(state, bd.value.generated_span_ids, b) &&
         curve_endpoints_match_layout(state);
}

bool C774_backbone_incremental_scope_mismatch_does_not_share_ports() {
  IncrementalCrossFixture fixture{};
  if (!make_incremental_cross(&fixture, true, city::wire::BundleKind::kHighVoltage) ||
      !fixture.completion.ok || fixture.completion.value.generated_span_ids.empty()) {
    return false;
  }
  const std::vector<city::wire::ObjectId> be_ports =
      unique_generated_ports_on_pole(fixture.state, fixture.completion.value.generated_span_ids, fixture.pole_b);
  if (be_ports.empty()) return false;
  for (city::wire::ObjectId port_id : be_ports) {
    if (std::find(fixture.bd_ports.begin(), fixture.bd_ports.end(), port_id) != fixture.bd_ports.end()) {
      return false;
    }
  }
  const JunctionRowSnapshot snapshot = junction_snapshot(fixture.state, fixture.pole_b);
  return snapshot.open_rows >= 1 && fixture.bd_ports == unique_generated_ports_on_pole(fixture.state, fixture.bd_spans, fixture.pole_b);
}

bool C837_backbone_source_bundle_endpoint_completion_ignores_other_bundle_open() {
  city::wire::CoreState state;
  city::wire::BackboneSpec left = line_req(state);
  left.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  const auto left_out = state.GenerateFromBackboneSpec(left);
  WIRE_TEST_EXPECT(left_out.ok, left_out.error);
  WIRE_TEST_EXPECT(left_out.value.bundle_ids.size() == 1 && left_out.value.generated_pole_ids.size() == 2,
                   "left fixture did not create one bundle and two poles");

  city::wire::BackboneSpec right = line_req(state);
  right.path.polyline = {{20.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
  const auto right_out = state.GenerateFromBackboneSpec(right);
  WIRE_TEST_EXPECT(right_out.ok, right_out.error);
  WIRE_TEST_EXPECT(right_out.value.bundle_ids.size() == 1 && right_out.value.generated_pole_ids.size() == 2,
                   "right fixture did not create one bundle and two poles");
  WIRE_TEST_EXPECT(left_out.value.bundle_ids.front() != right_out.value.bundle_ids.front(),
                   "fixtures unexpectedly share a bundle");

  const city::wire::ObjectId left_end_pole = left_out.value.generated_pole_ids[1];
  const city::wire::ObjectId right_start_pole = right_out.value.generated_pole_ids[0];
  const city::wire::Pole* left_end = state.view().poles().find(left_end_pole);
  const city::wire::Pole* right_start = state.view().poles().find(right_start_pole);
  WIRE_TEST_EXPECT(left_end != nullptr && right_start != nullptr, "fixture endpoint poles are missing");

  city::wire::BackboneSpec completion = line_req(state);
  completion.path.polyline = {left_end->world_transform.position, right_start->world_transform.position};
  completion.path.node_specs = {pole_spec(0, left_end_pole), pole_spec(1, right_start_pole)};
  completion.bundles.front().source_bundle_id = left_out.value.bundle_ids.front();

  const auto completed = state.GenerateFromBackboneSpec(completion);
  WIRE_TEST_EXPECT(completed.ok, completed.error);
  WIRE_TEST_EXPECT(completed.value.bundle_ids.size() == 1 &&
                       completed.value.bundle_ids.front() == left_out.value.bundle_ids.front(),
                   "completion did not continue the source bundle");
  WIRE_TEST_EXPECT(!completed.value.generated_span_ids.empty(), "completion did not create a span");
  for (city::wire::ObjectId span_id : completed.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    WIRE_TEST_EXPECT(span != nullptr && span->bundle_id == left_out.value.bundle_ids.front(),
                     "completion span is not owned by the source bundle");
  }
  WIRE_TEST_EXPECT(curve_endpoints_match_layout(state), "curve endpoints do not match layout");
  std::string invariant_error{};
  WIRE_TEST_EXPECT(backbone_common_invariants_pass(state, &invariant_error), invariant_error);
  return true;
}

bool C775_backbone_incremental_canonical_pair_survives_save_load() {
  IncrementalCrossFixture source{};
  WIRE_TEST_EXPECT(make_incremental_cross(&source), "incremental cross fixture generation failed");
  WIRE_TEST_EXPECT(source.completion.ok, source.completion.error);
  WIRE_TEST_EXPECT(canonical_cross_at_b(source), "incremental cross was not canonical at B before save");
  std::string source_invariant_error{};
  WIRE_TEST_EXPECT_ANCHOR(
      backbone_common_invariants_pass(source.state, &source_invariant_error),
      "before save: " + source_invariant_error);
  const JunctionRowSnapshot before = junction_snapshot(source.state, source.pole_b);
  std::string saved{};
  const auto serialized = source.state.SerializeAuthoritative(&saved);
  WIRE_TEST_EXPECT(serialized.ok, serialized.error);
  city::wire::CoreState loaded;
  const auto deserialized = loaded.DeserializeAuthoritative(saved);
  WIRE_TEST_EXPECT(deserialized.ok, deserialized.error);
  const JunctionRowSnapshot after = junction_snapshot(loaded, source.pole_b);
  std::string visual_diagnostics{};
  std::string patch_nodes{};
  for (const city::wire::VisualCurveDiagnostic& diagnostic :
       loaded.view().visual_curve_parts().diagnostics) {
    visual_diagnostics +=
        (visual_diagnostics.empty() ? "" : "; ") + diagnostic.reason;
  }
  for (const city::wire::VisualCurvePart& part :
       loaded.view().visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kNodePatch) continue;
    patch_nodes += (patch_nodes.empty() ? "" : ",") +
                   std::to_string(part.source_node_id);
  }
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      snapshots_match(before, after),
      "junction row snapshot changed after save/load: before " +
          junction_snapshot_counts(before) + " after " +
          junction_snapshot_counts(after) + " diagnostics=" +
          visual_diagnostics + " patch_nodes=" + patch_nodes);
  WIRE_TEST_EXPECT_ANCHOR(
      curve_endpoints_match_layout(loaded),
      "curve endpoints do not match layout after save/load");
  std::string invariant_error{};
  WIRE_TEST_EXPECT_ANCHOR(
      backbone_common_invariants_pass(loaded, &invariant_error),
      invariant_error);
  return true;
}

bool C805_backbone_generation_scoped_route_order_does_not_break_t_branch_restore() {
  city::wire::CoreState source;
  city::wire::BackboneSpec base = poly3_req(source);
  base.bundles.clear();
  add_backbone_bundle(base, city::wire::BundleKind::kHighVoltage);
  add_backbone_bundle(base, city::wire::BundleKind::kLowVoltage);
  const auto abc = source.GenerateFromBackboneSpec(base);
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3 ||
      abc.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId pole_b = abc.value.generated_pole_ids[1];
  const city::wire::Pole* b = source.view().poles().find(pole_b);
  const city::wire::SavedBackboneNode* node_b = source.view().backbone_node_for_pole(pole_b);
  if (b == nullptr || node_b == nullptr) {
    return false;
  }
  city::wire::BackboneSpec branch = base;
  branch.path.polyline = {{12.0, -8.0, 0.0}, b->world_transform.position};
  branch.path.node_specs = {pole_spec(1, pole_b)};
  const auto bd = source.GenerateFromBackboneSpec(branch);
  if (!bd.ok || bd.value.generated_pole_ids.size() != 1 ||
      bd.value.generated_span_ids.size() != 4) {
    return false;
  }
  const JunctionRowSnapshot before = junction_snapshot(source, pole_b);
  if (before.pair_rows != 1 || before.open_rows != 1 ||
      !row_continuity_graph_lint_passes(source) ||
      !curve_endpoints_match_layout(source)) {
    return false;
  }
  const std::size_t before_span_count = source.view().spans().size();
  const std::size_t before_part_count = source.view().visual_curve_parts().parts.size();
  std::string saved{};
  const auto serialized = source.SerializeAuthoritative(&saved);
  if (!serialized.ok) {
    return false;
  }

  city::wire::CoreState loaded;
  const auto deserialized = loaded.DeserializeAuthoritative(saved);
  if (!deserialized.ok) {
    return false;
  }
  const JunctionRowSnapshot after = junction_snapshot(loaded, pole_b);
  return loaded.view().spans().size() == before_span_count &&
         loaded.view().visual_curve_parts().parts.size() == before_part_count &&
         snapshots_match(before, after) &&
         row_continuity_graph_lint_passes(loaded) &&
         curve_endpoints_match_layout(loaded);
}

bool C776_backbone_incremental_canonical_pair_survives_regenerate() {
  IncrementalCrossFixture fixture{};
  if (!make_incremental_cross(&fixture) || !fixture.completion.ok || !canonical_cross_at_b(fixture)) {
    return false;
  }
  if (fixture.completion.value.bundle_ids.empty()) return false;
  const city::wire::ObjectId bundle_id = fixture.completion.value.bundle_ids.front();
  const city::wire::Bundle* bundle = fixture.state.view().bundles().find(bundle_id);
  if (bundle == nullptr) return false;
  const auto updated = fixture.state.UpdateBackboneBundlePlacement(
      bundle_id, true, bundle->height_m + 0.05, bundle->lateral_m, bundle->phase_spacing_m);
  const bool canonical = canonical_cross_at_b(fixture);
  const bool curves = curve_endpoints_match_layout(fixture.state);
  return updated.ok && canonical && curves;
}

bool C777_backbone_incremental_reverse_completion_uses_same_pair_key() {
  IncrementalCrossFixture reverse{};
  IncrementalCrossFixture forward{};
  if (!make_incremental_cross(&reverse, true) || !reverse.completion.ok ||
      !make_incremental_cross(&forward, false) || !forward.completion.ok ||
      !canonical_cross_at_b(reverse) || !canonical_cross_at_b(forward)) {
    return false;
  }
  const JunctionRowSnapshot a = junction_snapshot(reverse.state, reverse.pole_b);
  const JunctionRowSnapshot b = junction_snapshot(forward.state, forward.pole_b);
  return snapshots_match(a, b) && curve_endpoints_match_layout(reverse.state) &&
         curve_endpoints_match_layout(forward.state);
}

bool C778_backbone_incremental_multi_bundle_completion_promotes_each_scope_once() {
  auto multi_bundle_req = [](city::wire::CoreState& state) {
    city::wire::BackboneSpec req = poly3_req(state);
    req.bundles.clear();
    add_backbone_bundle(req, city::wire::BundleKind::kLowVoltage);
    add_backbone_bundle(req, city::wire::BundleKind::kHighVoltage);
    add_backbone_bundle(req, city::wire::BundleKind::kCommunication);
    add_backbone_bundle(req, city::wire::BundleKind::kOptical);
    return req;
  };

  city::wire::CoreState state;
  city::wire::BackboneSpec base = multi_bundle_req(state);
  const auto abc = state.GenerateFromBackboneSpec(base);
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const city::wire::ObjectId b = abc.value.generated_pole_ids[1];
  const city::wire::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  city::wire::BackboneSpec bd = multi_bundle_req(state);
  bd.path.polyline = {pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd.path.node_specs = {pole_spec(0, b)};
  const auto bd_out = state.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_pole_ids.size() != 1 || bd_out.value.generated_span_ids.empty()) {
    return false;
  }
  const std::vector<city::wire::ObjectId> bd_ports = unique_generated_ports_on_pole(state, bd_out.value.generated_span_ids, b);
  const std::vector<city::wire::ObjectId> bd_spans = bd_out.value.generated_span_ids;

  city::wire::BackboneSpec eb = multi_bundle_req(state);
  eb.path.polyline = {{20.0, 0.0, 0.0}, pole_b->world_transform.position};
  eb.path.node_specs = {pole_spec(1, b)};
  const auto eb_out = state.GenerateFromBackboneSpec(eb);
  if (!eb_out.ok || eb_out.value.generated_pole_ids.size() != 1 || eb_out.value.generated_span_ids.empty()) {
    return false;
  }

  const city::wire::ObjectId d = bd_out.value.generated_pole_ids.front();
  const city::wire::ObjectId e = eb_out.value.generated_pole_ids.front();
  const city::wire::SavedBackboneNode* node_b = state.view().backbone_node_for_pole(b);
  const city::wire::SavedBackboneNode* node_d = state.view().backbone_node_for_pole(d);
  const city::wire::SavedBackboneNode* node_e = state.view().backbone_node_for_pole(e);
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr) return false;
  const city::wire::ObjectId bd_edge = edge_between(state, node_b->node_id, node_d->node_id);
  const city::wire::ObjectId be_edge = edge_between(state, node_b->node_id, node_e->node_id);
  const JunctionRowSnapshot snapshot = junction_snapshot(state, b);

  return bd_edge != city::wire::kInvalidObjectId && be_edge != city::wire::kInvalidObjectId &&
         snapshot.pair_rows == 2 && snapshot.open_rows == 0 &&
         has_row_key(snapshot.row_keys, false, bd_edge, be_edge) &&
         bd_ports == unique_generated_ports_on_pole(state, bd_spans, b) &&
         eb_out.value.generated_span_ids.size() == bd_spans.size() &&
         curve_endpoints_match_layout(state);
}

bool C779_backbone_incremental_same_template_multi_placement_uses_placement_key() {
  auto web_default_req = [](city::wire::CoreState& state) {
    city::wire::BackboneSpec req = poly3_req(state);
    req.bundles.clear();
    auto add = [&](city::wire::BundleKind kind, std::uint64_t placement_key, int count,
                   double height_m) {
      const city::wire::BundleTemplateId template_id = city::wire::DefaultBundleTemplateId(kind);
      const auto template_it = state.view().bundle_templates().find(template_id);
      if (template_it == state.view().bundle_templates().end()) {
        return;
      }
      city::wire::BackboneBundleSpec bundle{};
      bundle.bundle_template_id = template_id;
      bundle.placement_key = placement_key;
      bundle.layer = template_it->second.default_layer;
      bundle.count = count;
      bundle.placement_explicit = true;
      bundle.height_m = height_m;
      bundle.lateral_m = kind == city::wire::BundleKind::kHighVoltage ? -0.2 : 0.0;
      bundle.spacing_m = template_it->second.default_spacing_m;
      req.bundles.push_back(bundle);
    };
    add(city::wire::BundleKind::kHighVoltage, 1, 3, 9.2);
    add(city::wire::BundleKind::kLowVoltage, 2, 1, 7.7);
    add(city::wire::BundleKind::kLowVoltage, 3, 1, 7.35);
    add(city::wire::BundleKind::kLowVoltage, 4, 1, 7.0);
    add(city::wire::BundleKind::kCommunication, 5, 1, 5.5);
    add(city::wire::BundleKind::kOptical, 6, 1, 5.3);
    return req;
  };
  auto edge_bundle_for_key = [](const city::wire::CoreState& state, city::wire::ObjectId edge_id,
                                std::uint64_t placement_key) {
    const auto bundles_it = state.view().backbone_index().edge_bundles.find(edge_id);
    if (bundles_it == state.view().backbone_index().edge_bundles.end()) {
      return city::wire::kInvalidObjectId;
    }
    for (city::wire::ObjectId edge_bundle_id : bundles_it->second) {
      const city::wire::SavedBackboneEdgeBundle* edge_bundle =
          state.view().backbone_edge_bundle(edge_bundle_id);
      const city::wire::Bundle* bundle =
          edge_bundle == nullptr ? nullptr : state.view().bundles().find(edge_bundle->bundle_id);
      if (bundle != nullptr && bundle->placement_key == placement_key) {
        return edge_bundle_id;
      }
    }
    return city::wire::kInvalidObjectId;
  };
  auto port_for_edge_bundle_at_node = [](const city::wire::CoreState& state, city::wire::ObjectId edge_bundle_id,
                                         city::wire::ObjectId node_id) {
    std::vector<city::wire::ObjectId> found{};
    for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
      if (binding.edge_bundle_id != edge_bundle_id || binding.row_key.node_id != node_id) continue;
      if (std::find(found.begin(), found.end(), binding.port_id) == found.end()) {
        found.push_back(binding.port_id);
      }
    }
    std::sort(found.begin(), found.end());
    return found;
  };

  city::wire::CoreState state;
  const auto abc = state.GenerateFromBackboneSpec(web_default_req(state));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const city::wire::ObjectId b = abc.value.generated_pole_ids[1];
  const city::wire::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  city::wire::BackboneSpec bd = web_default_req(state);
  bd.path.polyline = {pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd.path.node_specs = {pole_spec(0, b)};
  const auto bd_out = state.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_pole_ids.size() != 1 ||
      bd_out.value.generated_span_ids.size() != 8) {
    return false;
  }
  const city::wire::ObjectId d = bd_out.value.generated_pole_ids.front();
  const city::wire::SavedBackboneNode* node_b_before = state.view().backbone_node_for_pole(b);
  const city::wire::SavedBackboneNode* node_d_before = state.view().backbone_node_for_pole(d);
  if (node_b_before == nullptr || node_d_before == nullptr) return false;
  const city::wire::ObjectId bd_edge_before = edge_between(state, node_b_before->node_id, node_d_before->node_id);
  if (bd_edge_before == city::wire::kInvalidObjectId) return false;

  std::vector<std::pair<std::uint64_t, city::wire::ObjectId>> bd_bundle_ids{};
  std::vector<std::pair<std::uint64_t, std::vector<city::wire::ObjectId>>> bd_ports{};
  for (std::uint64_t key : {1ULL, 2ULL, 3ULL, 4ULL, 5ULL, 6ULL}) {
    const city::wire::ObjectId edge_bundle_id = edge_bundle_for_key(state, bd_edge_before, key);
    const city::wire::SavedBackboneEdgeBundle* edge_bundle = state.view().backbone_edge_bundle(edge_bundle_id);
    if (edge_bundle == nullptr) return false;
    const std::vector<city::wire::ObjectId> port_ids =
        port_for_edge_bundle_at_node(state, edge_bundle_id, node_b_before->node_id);
    if (port_ids.size() != (key == 1ULL ? 3U : 1U)) return false;
    bd_bundle_ids.push_back({key, edge_bundle->bundle_id});
    bd_ports.push_back({key, port_ids});
  }

  city::wire::BackboneSpec eb = web_default_req(state);
  eb.path.polyline = {{20.0, 0.0, 0.0}, pole_b->world_transform.position};
  eb.path.node_specs = {pole_spec(1, b)};
  const auto eb_out = state.GenerateFromBackboneSpec(eb);
  if (!eb_out.ok || eb_out.value.generated_pole_ids.size() != 1 ||
      eb_out.value.generated_span_ids.size() != 8) {
    return false;
  }

  const city::wire::ObjectId e = eb_out.value.generated_pole_ids.front();
  const city::wire::SavedBackboneNode* node_b = state.view().backbone_node_for_pole(b);
  const city::wire::SavedBackboneNode* node_d = state.view().backbone_node_for_pole(d);
  const city::wire::SavedBackboneNode* node_e = state.view().backbone_node_for_pole(e);
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr) return false;
  const city::wire::ObjectId bd_edge = edge_between(state, node_b->node_id, node_d->node_id);
  const city::wire::ObjectId be_edge = edge_between(state, node_b->node_id, node_e->node_id);
  if (bd_edge == city::wire::kInvalidObjectId || be_edge == city::wire::kInvalidObjectId) return false;

  for (const auto& [key, bd_bundle_id] : bd_bundle_ids) {
    const city::wire::ObjectId be_edge_bundle_id = edge_bundle_for_key(state, be_edge, key);
    const city::wire::SavedBackboneEdgeBundle* be_edge_bundle = state.view().backbone_edge_bundle(be_edge_bundle_id);
    if (be_edge_bundle == nullptr || be_edge_bundle->bundle_id != bd_bundle_id) {
      return false;
    }
    const auto bd_port_it = std::find_if(bd_ports.begin(), bd_ports.end(), [&](const auto& item) {
      return item.first == key;
    });
    const std::vector<city::wire::ObjectId> be_ports =
        port_for_edge_bundle_at_node(state, be_edge_bundle_id, node_b->node_id);
    if (bd_port_it == bd_ports.end() || be_ports.size() != bd_port_it->second.size()) {
      return false;
    }
    for (std::size_t lane = 0; lane < be_ports.size(); ++lane) {
      const city::wire::Port* old_port =
          state.view().ports().find(bd_port_it->second[lane]);
      const city::wire::Port* new_port = state.view().ports().find(be_ports[lane]);
      if (old_port == nullptr || new_port == nullptr ||
          old_port->id == new_port->id ||
          !almost_equal(old_port->world_position, new_port->world_position, 1e-9)) {
        return false;
      }
    }
  }

  const JunctionRowSnapshot snapshot = junction_snapshot(state, b);
  return snapshot.pair_rows == 2 && snapshot.open_rows == 0 &&
         has_row_key(snapshot.row_keys, false, bd_edge, be_edge) &&
         eb_out.value.generated_span_ids.size() == bd_out.value.generated_span_ids.size() &&
         std::all_of(bd_out.value.generated_span_ids.begin(), bd_out.value.generated_span_ids.end(),
                     [&](city::wire::ObjectId span_id) {
                       return state.view().spans().find(span_id) != nullptr;
                     }) &&
         curve_endpoints_match_layout(state);
}

bool C785_backbone_incremental_hv_promotion_reframes_existing_ports() {
  city::wire::CoreState state;
  const auto abc = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const city::wire::ObjectId b = abc.value.generated_pole_ids[1];
  const city::wire::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  city::wire::BackboneSpec bd = line_req(state);
  bd.bundles.clear();
  add_backbone_bundle(bd, city::wire::BundleKind::kHighVoltage);
  bd.path.polyline = {pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd.path.node_specs = {pole_spec(0, b)};
  const auto bd_out = state.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_pole_ids.size() != 1 || bd_out.value.generated_span_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId d = bd_out.value.generated_pole_ids.front();
  const city::wire::SavedBackboneNode* node_b_before = state.view().backbone_node_for_pole(b);
  const city::wire::SavedBackboneNode* node_d_before = state.view().backbone_node_for_pole(d);
  if (node_b_before == nullptr || node_d_before == nullptr) return false;
  const city::wire::ObjectId bd_edge_before = edge_between(state, node_b_before->node_id, node_d_before->node_id);
  const city::wire::ObjectId bd_edge_bundle_before =
      edge_bundle_for_template(state, bd_edge_before, city::wire::BundleKind::kHighVoltage);
  if (bd_edge_bundle_before == city::wire::kInvalidObjectId) return false;
  const std::vector<PortBindingSnapshot> before =
      port_binding_snapshot(state, bd_edge_bundle_before, node_b_before->node_id);
  if (before.size() != 3 || before[0].lane != 0 || before[1].lane != 1 || before[2].lane != 2) {
    return false;
  }

  city::wire::BackboneSpec eb = line_req(state);
  eb.bundles.clear();
  add_backbone_bundle(eb, city::wire::BundleKind::kHighVoltage);
  eb.path.polyline = {{20.0, 0.0, 0.0}, pole_b->world_transform.position};
  eb.path.node_specs = {pole_spec(1, b)};
  const auto eb_out = state.GenerateFromBackboneSpec(eb);
  if (!eb_out.ok || eb_out.value.generated_pole_ids.size() != 1 || eb_out.value.generated_span_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId e = eb_out.value.generated_pole_ids.front();
  const city::wire::SavedBackboneNode* node_b = state.view().backbone_node_for_pole(b);
  const city::wire::SavedBackboneNode* node_d = state.view().backbone_node_for_pole(d);
  const city::wire::SavedBackboneNode* node_e = state.view().backbone_node_for_pole(e);
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr) return false;
  const city::wire::ObjectId bd_edge = edge_between(state, node_b->node_id, node_d->node_id);
  const city::wire::ObjectId be_edge = edge_between(state, node_b->node_id, node_e->node_id);
  const city::wire::ObjectId bd_edge_bundle =
      edge_bundle_for_template(state, bd_edge, city::wire::BundleKind::kHighVoltage);
  const city::wire::ObjectId be_edge_bundle =
      edge_bundle_for_template(state, be_edge, city::wire::BundleKind::kHighVoltage);
  const city::wire::SavedBackboneEdgeBundle* bd_bundle = state.view().backbone_edge_bundle(bd_edge_bundle);
  const city::wire::SavedBackboneEdgeBundle* be_bundle = state.view().backbone_edge_bundle(be_edge_bundle);
  if (bd_bundle == nullptr || be_bundle == nullptr || bd_bundle->bundle_id != be_bundle->bundle_id) {
    return false;
  }
  const std::vector<PortBindingSnapshot> bd_after =
      port_binding_snapshot(state, bd_edge_bundle, node_b->node_id);
  const std::vector<PortBindingSnapshot> be_after =
      port_binding_snapshot(state, be_edge_bundle, node_b->node_id);
  const JunctionRowSnapshot snapshot = junction_snapshot(state, b);
  const bool frame_changed = std::any_of(
      before.begin(), before.end(), [&](const PortBindingSnapshot& old_binding) {
        const auto after = std::find_if(
            bd_after.begin(), bd_after.end(), [&](const PortBindingSnapshot& new_binding) {
              return new_binding.port_id == old_binding.port_id;
            });
        return after != bd_after.end() &&
               std::abs(city::wire::NormalizeYawDeg(
                   after->layout_yaw_deg - old_binding.layout_yaw_deg)) > 1e-9;
      });
  return snapshot.pair_rows == 2 && snapshot.open_rows == 0 &&
         has_row_key(snapshot.row_keys, false, bd_edge, be_edge) &&
         same_port_binding_identity(before, bd_after) && frame_changed &&
         same_port_binding_geometry(bd_after, be_after, true) &&
         curve_endpoints_match_layout(state);
}

bool C795_backbone_incremental_hv_promotion_preserves_model_fixture_geometry() {
  constexpr city::wire::ModelAssemblyTemplateId kRowAssembly = 9195;
  constexpr city::wire::ModelAssemblyTemplateId kEndpointAssembly = 9196;
  city::wire::CoreState state;
  if (!register_hv_fixture_models(&state, kRowAssembly, kEndpointAssembly)) {
    return false;
  }

  auto make_hv_request = [](city::wire::BackboneSpec req) {
    for (city::wire::BackboneBundleSpec& bundle : req.bundles) {
      bundle.placement_explicit = true;
      bundle.count = 3;
      bundle.height_m = 9.2;
      bundle.lateral_m = -0.2;
      bundle.spacing_m = 0.45;
    }
    return req;
  };

  const auto abc = state.GenerateFromBackboneSpec(make_hv_request(hv_poly3_req(state)));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const city::wire::ObjectId b = abc.value.generated_pole_ids[1];
  const city::wire::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  city::wire::BackboneSpec bd = line_req(state);
  bd.bundles.clear();
  add_backbone_bundle(bd, city::wire::BundleKind::kHighVoltage);
  bd = make_hv_request(bd);
  bd.path.polyline = {pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd.path.node_specs = {pole_spec(0, b)};
  const auto bd_out = state.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_span_ids.size() != 3) return false;

  const city::wire::ObjectId d = bd_out.value.generated_pole_ids.front();
  const city::wire::SavedBackboneNode* node_b_before = state.view().backbone_node_for_pole(b);
  const city::wire::SavedBackboneNode* node_d_before = state.view().backbone_node_for_pole(d);
  if (node_b_before == nullptr || node_d_before == nullptr) return false;
  const city::wire::ObjectId bd_edge_before =
      edge_between(state, node_b_before->node_id, node_d_before->node_id);
  const city::wire::ObjectId bd_edge_bundle_before =
      edge_bundle_for_template(state, bd_edge_before, city::wire::BundleKind::kHighVoltage);
  const std::vector<PortBindingSnapshot> before =
      port_binding_snapshot(state, bd_edge_bundle_before, node_b_before->node_id);
  if (before.size() != 3) return false;

  const std::vector<city::wire::Transformd> row_transforms_before =
      row_fixture_transforms_on_pole(state, b);
  if (row_transforms_before.empty()) return false;

  for (const PortBindingSnapshot& snapshot : before) {
    const auto fixture = endpoint_fixture_transform(state, snapshot.port_id);
    const auto endpoint = layout_endpoint_for_port(state, snapshot.port_id);
    if (!fixture.has_value() || !endpoint.has_value()) return false;
    const city::wire::Vec3d visible_socket = hv_visible_socket(*fixture);
    if (!almost_equal(visible_socket, *endpoint, 1e-9)) return false;
  }

  city::wire::BackboneSpec eb = line_req(state);
  eb.bundles.clear();
  add_backbone_bundle(eb, city::wire::BundleKind::kHighVoltage);
  eb = make_hv_request(eb);
  eb.path.polyline = {{20.0, 0.0, 0.0}, pole_b->world_transform.position};
  eb.path.node_specs = {pole_spec(1, b)};
  const auto eb_out = state.GenerateFromBackboneSpec(eb);
  if (!eb_out.ok || eb_out.value.generated_span_ids.size() != 3) return false;

  const city::wire::ObjectId e = eb_out.value.generated_pole_ids.front();
  const city::wire::SavedBackboneNode* node_b = state.view().backbone_node_for_pole(b);
  const city::wire::SavedBackboneNode* node_d = state.view().backbone_node_for_pole(d);
  const city::wire::SavedBackboneNode* node_e = state.view().backbone_node_for_pole(e);
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr) return false;
  const city::wire::ObjectId bd_edge = edge_between(state, node_b->node_id, node_d->node_id);
  const city::wire::ObjectId be_edge = edge_between(state, node_b->node_id, node_e->node_id);
  const city::wire::ObjectId bd_edge_bundle =
      edge_bundle_for_template(state, bd_edge, city::wire::BundleKind::kHighVoltage);
  const city::wire::ObjectId be_edge_bundle =
      edge_bundle_for_template(state, be_edge, city::wire::BundleKind::kHighVoltage);
  const std::vector<PortBindingSnapshot> bd_after =
      port_binding_snapshot(state, bd_edge_bundle, node_b->node_id);
  const std::vector<PortBindingSnapshot> be_after =
      port_binding_snapshot(state, be_edge_bundle, node_b->node_id);
  if (!same_port_binding_identity(before, bd_after) ||
      !same_port_binding_geometry(bd_after, be_after, true)) {
    return false;
  }
  const auto derived_yaws = derived_row_yaws_for_ports(state, bd_after);
  WIRE_TEST_EXPECT(derived_yaws.has_value(),
                   "promoted pair row representation is missing");
  const bool row_frame_changed = std::any_of(
      before.begin(), before.end(), [&](const PortBindingSnapshot& old_binding) {
        const auto after = std::find_if(
            bd_after.begin(), bd_after.end(), [&](const PortBindingSnapshot& new_binding) {
              return new_binding.port_id == old_binding.port_id;
            });
        if (after == bd_after.end()) {
          return false;
        }
        const std::size_t index =
            static_cast<std::size_t>(std::distance(bd_after.begin(), after));
        return std::abs(city::wire::NormalizeYawDeg(
                   (*derived_yaws)[index] - old_binding.layout_yaw_deg)) > 1e-9;
      });
  WIRE_TEST_EXPECT(
      row_frame_changed,
      "promotion fixture does not exercise a changed derived pair row frame");
  WIRE_TEST_EXPECT(
      port_positions_match_explicit_lane_anchors(
          state, bd_after, *derived_yaws, -0.2, 0.45),
      "promoted existing HV ports do not follow the resolved pair row frame");
  const auto new_derived_yaws = derived_row_yaws_for_ports(state, be_after);
  WIRE_TEST_EXPECT(new_derived_yaws.has_value(),
                   "new pair row representation is missing");
  WIRE_TEST_EXPECT(
      port_positions_match_explicit_lane_anchors(
          state, be_after, *new_derived_yaws, -0.2, 0.45),
      "new HV ports do not follow the resolved pair row frame");

  const std::vector<city::wire::Transformd> row_transforms_after =
      row_fixture_transforms_on_pole(state, b);
  if (row_transforms_after.size() != row_transforms_before.size()) {
    return false;
  }
  WIRE_TEST_EXPECT(
      std::any_of(row_transforms_after.begin(), row_transforms_after.end(),
                  [&](const city::wire::Transformd& transform) {
                    return std::abs(city::wire::NormalizeYawDeg(
                               transform.rotation_euler_deg.z -
                               derived_yaws->front())) <= 1e-9;
                  }),
      "promoted HV row fixture does not use the resolved pair row frame");
  WIRE_TEST_EXPECT(
      endpoint_fixtures_match_explicit_lane_anchors(
          state, bd_after, *derived_yaws, -0.2, 0.45),
      "promoted existing HV endpoint fixtures do not follow the row fixture frame");
  WIRE_TEST_EXPECT(
      endpoint_fixtures_match_explicit_lane_anchors(
          state, be_after, *new_derived_yaws, -0.2, 0.45),
      "new HV endpoint fixtures do not follow the row fixture frame");
  for (const PortBindingSnapshot& snapshot : before) {
    const auto fixture = endpoint_fixture_transform(state, snapshot.port_id);
    const auto endpoint = layout_endpoint_for_port(state, snapshot.port_id);
    if (!fixture.has_value() || !endpoint.has_value()) return false;
    const city::wire::Vec3d visible_socket = hv_visible_socket(*fixture);
    if (!almost_equal(visible_socket, *endpoint, 1e-9)) {
      return false;
    }
  }

  std::array<const city::wire::VisualCurvePart*, 3> patches{nullptr, nullptr, nullptr};
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kNodePatch ||
        part.source_node_id != node_b->node_id ||
        part.bundle_template_id != city::wire::kDefaultHighVoltageBundleTemplateId ||
        part.lane_index >= patches.size()) {
      continue;
    }
    const bool incident_matches =
        std::find(part.incident_edge_ids.begin(), part.incident_edge_ids.end(), bd_edge) !=
            part.incident_edge_ids.end() &&
        std::find(part.incident_edge_ids.begin(), part.incident_edge_ids.end(), be_edge) !=
            part.incident_edge_ids.end();
    if (incident_matches) {
      patches[part.lane_index] = &part;
    }
  }
  if (patches[0] == nullptr || patches[1] == nullptr || patches[2] == nullptr) {
    return false;
  }
  const std::size_t sample_count = std::min({patches[0]->samples.size(),
                                             patches[1]->samples.size(),
                                             patches[2]->samples.size()});
  if (sample_count < 2) {
    return false;
  }
  for (std::size_t i = 0; i < sample_count; ++i) {
    const double d01 = std::sqrt(dist2(patches[0]->samples[i], patches[1]->samples[i]));
    const double d12 = std::sqrt(dist2(patches[1]->samples[i], patches[2]->samples[i]));
    const double d02 = std::sqrt(dist2(patches[0]->samples[i], patches[2]->samples[i]));
    if (d01 < 0.02 || d12 < 0.02 || d02 < 0.04) {
      return false;
    }
  }
  return curve_endpoints_match_layout(state);
}

bool C803_model_mount_graph_resolves_depth_four_chain() {
  using city::wire::Transformd;
  using city::wire::Vec3d;
  using city::wire::generation::backbone::MountGraphNode;
  using city::wire::generation::backbone::MountGraphSocket;
  using city::wire::generation::backbone::MountRefKind;
  using city::wire::generation::backbone::resolve_mount_node;
  using city::wire::generation::backbone::resolve_mount_socket;

  auto node = [](std::size_t id, Vec3d local_position, std::string socket_name,
                 Vec3d socket_position) {
    MountGraphNode out{};
    out.node = id;
    out.local_transform.position = local_position;
    MountGraphSocket socket{};
    socket.name = std::move(socket_name);
    socket.local_transform.position = socket_position;
    out.sockets.push_back(std::move(socket));
    return out;
  };

  std::vector<MountGraphNode> nodes{};
  nodes.push_back(node(0, {1.0, 0.0, 0.0}, "out", {0.0, 2.0, 0.0}));
  nodes[0].parent.kind = MountRefKind::kPoleFrame;
  nodes[0].parent.anchor_transform.position = {10.0, 0.0, 1.0};
  nodes.push_back(node(1, {0.0, 3.0, 0.0}, "out", {0.0, 0.0, 4.0}));
  nodes[1].parent.kind = MountRefKind::kInstanceSocket;
  nodes[1].parent.parent_node = 0;
  nodes[1].parent.socket_name = "out";
  nodes.push_back(node(2, {5.0, 0.0, 0.0}, "out", {0.0, 6.0, 0.0}));
  nodes[2].parent.kind = MountRefKind::kInstanceSocket;
  nodes[2].parent.parent_node = 1;
  nodes[2].parent.socket_name = "out";
  nodes.push_back(node(3, {0.0, 0.0, 7.0}, "tip", {8.0, 0.0, 0.0}));
  nodes[3].parent.kind = MountRefKind::kInstanceSocket;
  nodes[3].parent.parent_node = 2;
  nodes[3].parent.socket_name = "out";

  const auto resolved_node = resolve_mount_node(nodes, 3);
  const auto resolved_socket = resolve_mount_socket(nodes, 3, "tip");
  if (!resolved_node.ok || !resolved_socket.ok ||
      !almost_equal(resolved_node.value.position, Vec3d{16.0, 11.0, 12.0}, 1e-12) ||
      !almost_equal(resolved_socket.value.position, Vec3d{24.0, 11.0, 12.0}, 1e-12)) {
    return false;
  }

  MountGraphNode span_anchor = node(4, {0.25, 0.0, 0.0}, "wire", {0.0, 0.5, 0.0});
  span_anchor.parent.kind = MountRefKind::kSpanAnchor;
  span_anchor.parent.anchor_transform.position = {20.0, 30.0, 40.0};
  nodes.push_back(span_anchor);
  const auto resolved_span_anchor = resolve_mount_socket(nodes, 4, "wire");
  if (!resolved_span_anchor.ok ||
      !almost_equal(resolved_span_anchor.value.position, Vec3d{20.25, 30.5, 40.0}, 1e-12)) {
    return false;
  }

  if (resolve_mount_socket(nodes, 2, "missing").ok) {
    return false;
  }
  nodes[0].parent.kind = MountRefKind::kInstanceSocket;
  nodes[0].parent.parent_node = 3;
  nodes[0].parent.socket_name = "tip";
  return !resolve_mount_node(nodes, 0).ok;
}

bool C804_model_placement_rules_adapt_legacy_fields_and_interval_anchors() {
  using city::wire::BundleTemplate;
  using city::wire::Vec3d;
  using city::wire::generation::backbone::ModelPlacementOrientationPolicy;
  using city::wire::generation::backbone::ModelPlacementRuleKind;
  using city::wire::generation::backbone::interval_anchor_frames;
  using city::wire::generation::backbone::placement_rules_from_bundle_template;

  BundleTemplate bundle{};
  bundle.row_fixture_assembly_id = 901;
  bundle.endpoint_fixture_assembly_id = 902;
  const auto rules = placement_rules_from_bundle_template(bundle);
  if (rules.size() != 2 ||
      rules[0].kind != ModelPlacementRuleKind::kAtRow ||
      rules[0].assembly_id != 901 ||
      rules[1].kind != ModelPlacementRuleKind::kAtEndpoint ||
      rules[1].assembly_id != 902) {
    return false;
  }

  const std::vector<city::wire::generation::backbone::SpanAnchorFrame> anchors =
      interval_anchor_frames({0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, 3.0, 1.0,
                             ModelPlacementOrientationPolicy::kAlignTangent);
  if (anchors.size() != 4 ||
      !almost_equal(anchors[0].transform.position, Vec3d{1.0, 0.0, 0.0}, 1e-12) ||
      !almost_equal(anchors[1].transform.position, Vec3d{4.0, 0.0, 0.0}, 1e-12) ||
      !almost_equal(anchors[2].transform.position, Vec3d{7.0, 0.0, 0.0}, 1e-12) ||
      !almost_equal(anchors[3].transform.position, Vec3d{10.0, 0.0, 0.0}, 1e-12) ||
      std::abs(anchors[0].transform.rotation_euler_deg.z) > 1e-12) {
    return false;
  }

  const std::filesystem::path model_assembly =
      repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "model_assembly.cpp";
  const std::filesystem::path mount_header =
      repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "mount_graph.hpp";
  const std::filesystem::path rule_source =
      repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "model_placement_rules.cpp";
  const std::filesystem::path models_doc = repo_root() / "docs" / "wire" / "models.md";
  std::string model_text{};
  std::string mount_text{};
  std::string rule_text{};
  std::string doc_text{};
  if (!file_text(model_assembly, &model_text) ||
      !file_text(mount_header, &mount_text) ||
      !file_text(rule_source, &rule_text) ||
      !file_text(models_doc, &doc_text)) {
    return false;
  }
  return !contains_text(model_text, "row_fixture_assembly_id") &&
         !contains_text(model_text, "endpoint_fixture_assembly_id") &&
         contains_text(mount_text, "kPoleFrame") &&
         contains_text(mount_text, "kSpanAnchor") &&
         contains_text(mount_text, "kInstanceSocket") &&
         contains_text(rule_text, "placement_rules_from_bundle_template") &&
         contains_text(rule_text, "interval_anchor_frames") &&
         contains_text(doc_text, "MountRef::pole_frame") &&
         contains_text(doc_text, "MountRef::span_anchor") &&
         contains_text(doc_text, "MountRef::instance_socket") &&
         contains_text(doc_text, "PlacementRule") &&
         contains_text(doc_text, "bit flag") &&
         !contains_text(doc_text, "two-stage hardcode");
}

bool C780_backbone_incremental_duplicate_values_are_order_independent_by_placement_key() {
  auto duplicate_req = [](city::wire::CoreState& state, bool reversed) {
    city::wire::BackboneSpec req = poly3_req(state);
    req.bundles.clear();
    const city::wire::BundleTemplateId template_id =
        city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage);
    const auto template_it = state.view().bundle_templates().find(template_id);
    if (template_it == state.view().bundle_templates().end()) {
      return req;
    }
    auto make = [&](std::uint64_t key) {
      city::wire::BackboneBundleSpec bundle{};
      bundle.bundle_template_id = template_id;
      bundle.placement_key = key;
      bundle.layer = template_it->second.default_layer;
      bundle.count = 1;
      bundle.placement_explicit = true;
      bundle.height_m = 7.35;
      bundle.lateral_m = 0.0;
      bundle.spacing_m = template_it->second.default_spacing_m;
      return bundle;
    };
    if (reversed) {
      req.bundles = {make(12), make(11)};
    } else {
      req.bundles = {make(11), make(12)};
    }
    return req;
  };
  auto bundle_id_for_key_on_edge = [](const city::wire::CoreState& state, city::wire::ObjectId edge_id,
                                      std::uint64_t key) {
    const auto bundles_it = state.view().backbone_index().edge_bundles.find(edge_id);
    if (bundles_it == state.view().backbone_index().edge_bundles.end()) {
      return city::wire::kInvalidObjectId;
    }
    for (city::wire::ObjectId edge_bundle_id : bundles_it->second) {
      const city::wire::SavedBackboneEdgeBundle* edge_bundle =
          state.view().backbone_edge_bundle(edge_bundle_id);
      const city::wire::Bundle* bundle =
          edge_bundle == nullptr ? nullptr : state.view().bundles().find(edge_bundle->bundle_id);
      if (bundle != nullptr && bundle->placement_key == key) {
        return bundle->id;
      }
    }
    return city::wire::kInvalidObjectId;
  };

  city::wire::CoreState state;
  const auto abc = state.GenerateFromBackboneSpec(duplicate_req(state, false));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const city::wire::ObjectId b = abc.value.generated_pole_ids[1];
  const city::wire::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  city::wire::BackboneSpec bd = duplicate_req(state, false);
  bd.path.polyline = {pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd.path.node_specs = {pole_spec(0, b)};
  const auto bd_out = state.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_pole_ids.size() != 1) return false;
  const city::wire::ObjectId d = bd_out.value.generated_pole_ids.front();
  const city::wire::SavedBackboneNode* node_b_before = state.view().backbone_node_for_pole(b);
  const city::wire::SavedBackboneNode* node_d_before = state.view().backbone_node_for_pole(d);
  if (node_b_before == nullptr || node_d_before == nullptr) return false;
  const city::wire::ObjectId bd_edge_before = edge_between(state, node_b_before->node_id, node_d_before->node_id);
  if (bd_edge_before == city::wire::kInvalidObjectId) return false;
  const city::wire::ObjectId key_11_bundle = bundle_id_for_key_on_edge(state, bd_edge_before, 11);
  const city::wire::ObjectId key_12_bundle = bundle_id_for_key_on_edge(state, bd_edge_before, 12);
  if (key_11_bundle == city::wire::kInvalidObjectId || key_12_bundle == city::wire::kInvalidObjectId ||
      key_11_bundle == key_12_bundle) {
    return false;
  }

  city::wire::BackboneSpec eb = duplicate_req(state, true);
  eb.path.polyline = {{20.0, 0.0, 0.0}, pole_b->world_transform.position};
  eb.path.node_specs = {pole_spec(1, b)};
  const auto eb_out = state.GenerateFromBackboneSpec(eb);
  if (!eb_out.ok || eb_out.value.generated_pole_ids.size() != 1) return false;
  const city::wire::ObjectId e = eb_out.value.generated_pole_ids.front();
  const city::wire::SavedBackboneNode* node_b = state.view().backbone_node_for_pole(b);
  const city::wire::SavedBackboneNode* node_e = state.view().backbone_node_for_pole(e);
  if (node_b == nullptr || node_e == nullptr) return false;
  const city::wire::ObjectId be_edge = edge_between(state, node_b->node_id, node_e->node_id);
  if (be_edge == city::wire::kInvalidObjectId) return false;
  const JunctionRowSnapshot snapshot = junction_snapshot(state, b);
  return bundle_id_for_key_on_edge(state, be_edge, 11) == key_11_bundle &&
         bundle_id_for_key_on_edge(state, be_edge, 12) == key_12_bundle &&
         snapshot.pair_rows == 2 && snapshot.open_rows == 0 &&
         std::all_of(bd_out.value.generated_span_ids.begin(), bd_out.value.generated_span_ids.end(),
                     [&](city::wire::ObjectId span_id) {
                       return state.view().spans().find(span_id) != nullptr;
                     }) &&
         curve_endpoints_match_layout(state);
}

bool C781_backbone_incremental_cross_extension_preserves_existing_spans() {
  IncrementalCrossFixture fixture{};
  if (!make_incremental_cross(&fixture) || !fixture.completion.ok || !canonical_cross_at_b(fixture)) {
    return false;
  }
  const std::vector<city::wire::ObjectId> original_spans = fixture.bd_spans;
  std::vector<city::wire::ObjectId> cross_spans = fixture.completion.value.generated_span_ids;
  cross_spans.insert(cross_spans.end(), original_spans.begin(), original_spans.end());
  for (city::wire::ObjectId span_id : cross_spans) {
    if (fixture.state.view().spans().find(span_id) == nullptr) return false;
  }

  const city::wire::Pole* pole_b = fixture.state.view().poles().find(fixture.pole_b);
  if (pole_b == nullptr) return false;
  city::wire::BackboneSpec bf = line_req(fixture.state);
  bf.path.polyline = {pole_b->world_transform.position, {4.0, -8.0, 0.0}};
  bf.path.node_specs = {pole_spec(0, fixture.pole_b)};
  const auto out = fixture.state.GenerateFromBackboneSpec(bf);
  if (!out.ok || out.value.generated_pole_ids.size() != 1 || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (city::wire::ObjectId span_id : cross_spans) {
    if (fixture.state.view().spans().find(span_id) == nullptr) return false;
  }
  const JunctionRowSnapshot snapshot = junction_snapshot(fixture.state, fixture.pole_b);
  return snapshot.pair_rows >= 2 && snapshot.open_rows >= 1 &&
         fixture.bd_ports == unique_generated_ports_on_pole(fixture.state, fixture.bd_spans, fixture.pole_b) &&
         curve_endpoints_match_layout(fixture.state);
}

bool C460_backbone_context_links_are_not_emitted() {
  city::wire::CoreState state;
  city::wire::BackboneSpec abc = poly3_req(state);
  const int count = req_bundle_count(state, abc);
  const auto first = state.GenerateFromBackboneSpec(abc);
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const std::size_t span_count_before = state.view().spans().size();

  city::wire::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  return second.ok && state.view().spans().size() == span_count_before + second.value.generated_span_ids.size() &&
         second.value.generated_span_ids.size() == static_cast<std::size_t>(count);
}

bool C461_backbone_same_edge_request_skips_duplicate_context() {
  city::wire::CoreState state;
  city::wire::BackboneSpec first_req = line_req(state);
  const auto first = state.GenerateFromBackboneSpec(first_req);
  if (!first.ok || first.value.generated_pole_ids.size() != 2 || state.view().backbone().edges.size() != 1) {
    return false;
  }
  const city::wire::ObjectId a = first.value.generated_pole_ids[0];
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_a = state.view().poles().find(a);
  const auto* pole_b = state.view().poles().find(b);
  if (pole_a == nullptr || pole_b == nullptr) {
    return false;
  }

  city::wire::BackboneSpec second_req = line_req(state);
  second_req.bundles.clear();
  add_backbone_bundle(second_req, city::wire::BundleKind::kCommunication);
  second_req.path.polyline = {pole_a->world_transform.position, pole_b->world_transform.position};
  second_req.path.node_specs = {pole_spec(0, a), pole_spec(1, b)};
  const auto second = state.GenerateFromBackboneSpec(second_req);
  return second.ok && state.view().backbone().edges.size() == 1 && state.view().backbone().edge_bundles.size() == 2 &&
         !second.value.generated_span_ids.empty();
}

bool C462_backbone_no_junction_kind_after_existing_context() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  return C391_backbone_no_kind_label() && contains_text(cpp, "has_requested_saved_pair") &&
         contains_text(cpp, "if (!edge.is_new)") && contains_text(cpp, "EditResult<pairs> pipeline::make");
}

bool C463_backbone_duplicate_same_edge_bundle_rejected() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2 || state.view().backbone().edge_bundles.size() != 1) {
    return false;
  }
  const std::size_t edge_count = state.view().backbone().edges.size();
  const std::size_t edge_bundle_count = state.view().backbone().edge_bundles.size();
  const std::size_t span_count = state.view().spans().size();
  const std::size_t saved_span_count = state.view().backbone().edge_bundles.front().span_ids.size();
  const city::wire::ObjectId a = first.value.generated_pole_ids[0];
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  city::wire::BackboneSpec second = line_req(state);
  second.path.polyline = {pa->world_transform.position, pb->world_transform.position};
  second.path.node_specs = {pole_spec(0, a), pole_spec(1, b)};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  return !second_out.ok && contains_text(second_out.error, "duplicate saved span binding") &&
         state.view().backbone().edges.size() == edge_count &&
         state.view().backbone().edge_bundles.size() == edge_bundle_count && state.view().spans().size() == span_count &&
         state.view().backbone().edge_bundles.front().span_ids.size() == saved_span_count;
}

bool C464_backbone_different_bundle_on_same_edge_allowed() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2 || state.view().backbone().edge_bundles.size() != 1) {
    return false;
  }
  const city::wire::ObjectId a = first.value.generated_pole_ids[0];
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  city::wire::BackboneSpec second = line_req(state);
  second.bundles.clear();
  add_backbone_bundle(second, city::wire::BundleKind::kCommunication);
  second.path.polyline = {pa->world_transform.position, pb->world_transform.position};
  second.path.node_specs = {pole_spec(0, a), pole_spec(1, b)};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  return second_out.ok && state.view().backbone().edges.size() == 1 && state.view().backbone().edge_bundles.size() == 2;
}

bool C466_backbone_duplicate_reject_keeps_state_unchanged() {
  city::wire::CoreState state;
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
  const city::wire::ObjectId a = first.value.generated_pole_ids[0];
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  city::wire::BackboneSpec second = line_req(state);
  second.path.polyline = {pa->world_transform.position, pb->world_transform.position};
  second.path.node_specs = {pole_spec(0, a), pole_spec(1, b)};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  const city::wire::BackboneFrontier frontier = state.view().pole_frontier(a);
  return !second_out.ok && contains_text(second_out.error, "duplicate saved span binding") &&
         state.view().poles().size() == pole_count && state.view().ports().size() == port_count &&
         state.view().bundles().size() == bundle_count_before && state.view().spans().size() == span_count &&
         state.view().backbone().edges.size() == edge_count &&
         state.view().backbone().edge_bundles.size() == edge_bundle_count &&
         state.view().backbone().edge_bundles.front().span_ids.size() == saved_span_count &&
         frontier.edge_ids.size() == 1 && frontier.edge_bundle_ids.size() == 1 &&
         frontier.span_ids.size() == saved_span_count;
}

bool C467_backbone_saves_row_port_bindings() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  const city::wire::SavedBackboneGraph& graph = state.view().backbone();
  if (!out.ok || graph.edge_bundles.size() != 1 ||
      graph.port_bindings.size() != out.value.generated_span_ids.size() * 2) {
    return false;
  }
  const city::wire::ObjectId edge_bundle_id = graph.edge_bundles.front().edge_bundle_id;
  const std::vector<const city::wire::SavedBackbonePortBinding*> by_edge_bundle =
      state.view().backbone_port_bindings_for_edge_bundle(edge_bundle_id);
  if (by_edge_bundle.size() != graph.port_bindings.size()) {
    return false;
  }
  for (const city::wire::SavedBackbonePortBinding& binding : graph.port_bindings) {
    const city::wire::SavedBackbonePortBinding* by_port = state.view().backbone_port_binding_for_port(binding.port_id);
    if (binding.edge_bundle_id != edge_bundle_id || binding.row_key.node_id == city::wire::kInvalidObjectId ||
        binding.row_key.edge_id == city::wire::kInvalidObjectId ||
        state.view().ports().find(binding.port_id) == nullptr || by_port == nullptr ||
        by_port->port_id != binding.port_id) {
      return false;
    }
  }
  return true;
}

bool C468_backbone_row_port_binding_is_stable_for_existing_context() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const std::size_t before = state.view().backbone().port_bindings.size();
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  city::wire::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  return second.ok && state.view().backbone().port_bindings.size() == before + second.value.generated_span_ids.size() * 2;
}

bool C469_backbone_row_port_binding_rejects_duplicate_without_resolution() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const std::size_t before = state.view().backbone().port_bindings.size();
  const city::wire::ObjectId a = first.value.generated_pole_ids[0];
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  city::wire::BackboneSpec duplicate = line_req(state);
  duplicate.path.polyline = {pa->world_transform.position, pb->world_transform.position};
  duplicate.path.node_specs = {pole_spec(0, a), pole_spec(1, b)};
  const auto second = state.GenerateFromBackboneSpec(duplicate);
  return !second.ok && contains_text(second.error, "duplicate saved span binding") &&
         state.view().backbone().port_bindings.size() == before;
}

bool C471_backbone_resolves_existing_port_by_binding() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  std::vector<city::wire::ObjectId> middle_ports{};
  for (city::wire::ObjectId span_id : first.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    const city::wire::Port* a = state.view().ports().find(span->port_a_id);
    const city::wire::Port* c = state.view().ports().find(span->port_b_id);
    if (a != nullptr && a->owner_pole_id == b) {
      middle_ports.push_back(a->id);
    }
    if (c != nullptr && c->owner_pole_id == b) {
      middle_ports.push_back(c->id);
    }
  }
  if (middle_ports.size() != 2 || middle_ports[0] == middle_ports[1]) return false;
  const city::wire::Port* a = state.view().ports().find(middle_ports[0]);
  const city::wire::Port* b_port = state.view().ports().find(middle_ports[1]);
  const auto a_bindings = state.view().backbone_port_bindings_for_port(middle_ports[0]);
  const auto b_bindings = state.view().backbone_port_bindings_for_port(middle_ports[1]);
  return a != nullptr && b_port != nullptr &&
         a->world_position.x == b_port->world_position.x &&
         a->world_position.y == b_port->world_position.y &&
         a->world_position.z == b_port->world_position.z &&
         a_bindings.size() == 1 && b_bindings.size() == 1 &&
         a_bindings.front()->bundle_template_id == b_bindings.front()->bundle_template_id &&
         a_bindings.front()->port_kind == b_bindings.front()->port_kind &&
         a_bindings.front()->port_layer == b_bindings.front()->port_layer;
}

bool C487_backbone_port_resolution_requires_bundle_compatible_scope() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  std::vector<city::wire::ObjectId> first_ports{};
  for (city::wire::ObjectId span_id : first.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    first_ports.push_back(span->port_a_id);
    first_ports.push_back(span->port_b_id);
  }
  const std::size_t port_count = state.view().ports().size();
  const city::wire::ObjectId a = first.value.generated_pole_ids[0];
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  city::wire::BackboneSpec second = line_req(state);
  second.bundles.clear();
  add_backbone_bundle(second, city::wire::BundleKind::kCommunication);
  second.path.polyline = {pa->world_transform.position, pb->world_transform.position};
  second.path.node_specs = {pole_spec(0, a), pole_spec(1, b)};
  const auto out = state.GenerateFromBackboneSpec(second);
  if (!out.ok || out.value.generated_span_ids.empty() || state.view().ports().size() <= port_count) {
    return false;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr || contains_id(first_ports, span->port_a_id) || contains_id(first_ports, span->port_b_id)) {
      return false;
    }
  }
  return true;
}

bool C472_backbone_port_resolution_requires_saved_binding() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto manual_a =
      state.AddPort(b, pole_b->world_transform.position + city::wire::Vec3d{0.0, 0.0, 9.2},
                    city::wire::PortKind::kPower, city::wire::PortLayer::kLowVoltage);
  if (!manual_a.ok) {
    return false;
  }
  const std::size_t before = state.view().ports().size();
  city::wire::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || state.view().ports().size() <= before) {
    return false;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr || span->port_a_id == manual_a.value || span->port_b_id == manual_a.value) {
      return false;
    }
  }
  return true;
}

bool C473_backbone_resolved_port_used_by_new_span_endpoint() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  std::vector<city::wire::ObjectId> middle_ports{};
  for (city::wire::ObjectId span_id : first.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    const city::wire::Port* a = state.view().ports().find(span->port_a_id);
    const city::wire::Port* c = state.view().ports().find(span->port_b_id);
    if (a != nullptr && a->owner_pole_id == b) {
      middle_ports.push_back(a->id);
    }
    if (c != nullptr && c->owner_pole_id == b) {
      middle_ports.push_back(c->id);
    }
  }
  if (middle_ports.size() != 2 || middle_ports[0] == middle_ports[1]) return false;
  const city::wire::Port* a = state.view().ports().find(middle_ports[0]);
  const city::wire::Port* b_port = state.view().ports().find(middle_ports[1]);
  return a != nullptr && b_port != nullptr &&
         a->world_position.x == b_port->world_position.x &&
         a->world_position.y == b_port->world_position.y &&
         a->world_position.z == b_port->world_position.z;
}

bool C474_backbone_port_resolution_rejects_ambiguous_binding() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.cpp";
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
  return contains_text(body, "ambiguous backbone port binding") && contains_text(body, "found != kInvalidObjectId") &&
         contains_text(body, "found != port->id");
}

bool C476_backbone_branch_rows_are_separated_without_branch_kind() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const std::vector<city::wire::Vec3d> before = pole_port_positions(state, b);
  const std::vector<city::wire::Vec3d> main_row_ports =
      generated_ports_on_pole(state, first.value.generated_span_ids, b);
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr || main_row_ports.empty()) {
    return false;
  }
  const city::wire::Vec3d pole_b_position = pole_b->world_transform.position;
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b_position));
  const std::vector<city::wire::Vec3d> placed = generated_ports_on_pole(state, second.value.generated_span_ids, b);
  if (!second.ok || !separated_from(before, placed) || placed.empty()) {
    return false;
  }
  city::wire::Vec3d center{};
  for (const city::wire::Vec3d& point : placed) {
    center.x += point.x;
    center.y += point.y;
  }
  center.x /= static_cast<double>(placed.size());
  center.y /= static_cast<double>(placed.size());
  double main_height = 0.0;
  double branch_height = 0.0;
  for (const city::wire::Vec3d& point : main_row_ports) {
    main_height += point.z;
  }
  for (const city::wire::Vec3d& point : placed) {
    branch_height += point.z;
  }
  main_height /= static_cast<double>(main_row_ports.size());
  branch_height /= static_cast<double>(placed.size());
  return almost_equal(center.x, pole_b_position.x, 1e-9) && almost_equal(center.y, pole_b_position.y, 1e-9) &&
         std::abs(branch_height - main_height) > 0.1 && C391_backbone_no_kind_label();
}

bool C477_backbone_cross_rows_are_separated_without_cross_kind() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  city::wire::BackboneSpec cross = line_req(state);
  cross.path.polyline = {{12.0, -8.0, 0.0}, pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  cross.path.node_specs = {pole_spec(1, b)};
  const auto second = state.GenerateFromBackboneSpec(cross);
  const JunctionRowSnapshot snapshot = junction_snapshot(state, b);
  return second.ok && second.value.generated_pole_ids.size() == 2 &&
         !second.value.generated_span_ids.empty() &&
         snapshot.pair_rows == 2 && snapshot.open_rows == 0 &&
         curve_endpoints_match_layout(state) &&
         C391_backbone_no_kind_label();
}

bool C478_backbone_row_separation_is_deterministic() {
  const std::vector<city::wire::Vec3d> a = branch_separation_points();
  const std::vector<city::wire::Vec3d> b = branch_separation_points();
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

bool C789_backbone_multi_route_same_band_rows_keep_spacing() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId junction = first.value.generated_pole_ids[1];
  const city::wire::Pole* pole = state.view().poles().find(junction);
  const auto pole_type_it =
      pole == nullptr ? state.view().pole_types().end() : state.view().pole_types().find(pole->pole_type_id);
  if (pole == nullptr || pole_type_it == state.view().pole_types().end()) {
    return false;
  }

  std::vector<std::pair<city::wire::ObjectId, city::wire::Vec3d>> before_ports{};
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    const city::wire::Port* port = state.view().ports().find(binding.port_id);
    if (port != nullptr && port->owner_pole_id == junction &&
        binding.bundle_template_id == city::wire::kDefaultHighVoltageBundleTemplateId) {
      before_ports.push_back({port->id, port->world_position});
    }
  }
  if (before_ports.empty()) {
    return false;
  }

  city::wire::BackboneSpec branch = hv_branch_req(state, junction, pole->world_transform.position);
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok || second.value.generated_span_ids.size() != 3) {
    return false;
  }

  double min_same_band_distance = std::numeric_limits<double>::infinity();
  for (city::wire::ObjectId span_id : second.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    for (city::wire::ObjectId port_id : {span->port_a_id, span->port_b_id}) {
      const city::wire::Port* new_port = state.view().ports().find(port_id);
      if (new_port == nullptr || new_port->owner_pole_id != junction) {
        continue;
      }
      for (const auto& existing_port : before_ports) {
        if (existing_port.first == new_port->id) {
          continue;
        }
        min_same_band_distance = std::min(min_same_band_distance,
                                          std::sqrt(dist2(existing_port.second, new_port->world_position)));
      }
    }
  }

  return std::isfinite(min_same_band_distance) && min_same_band_distance + 1e-9 >= 0.35;
}

bool C790_backbone_duplicate_support_point_requires_node_reference() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const city::wire::ObjectId next_id_before = state.next_id();
  const CoreCounts counts_before = snapshot_counts(state);
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const city::wire::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }

  city::wire::BackboneSpec duplicate = line_req(state);
  duplicate.path.polyline = {pole_b->world_transform.position, {12.0, 8.0, 0.0}};
  duplicate.path.node_specs.clear();
  city::wire::instrumentation::reset();
  const auto rejected = state.GenerateFromBackboneSpec(duplicate);
  const city::wire::instrumentation::Counters counters =
      city::wire::instrumentation::snapshot();

  return !rejected.ok &&
         contains_text(rejected.error, "existing support requires an explicit node reference") &&
         state.next_id() == next_id_before &&
         same_counts(snapshot_counts(state), counts_before) &&
         counters.pair_build_count == 0 &&
         counters.stable_row_slot_lookup_count == 0 &&
         counters.fixture_rule_merge_count == 0 &&
         counters.last_fixture_rule_count == 0 &&
         counters.fixture_plan_build_count == 0 &&
         counters.model_materialization_count == 0 &&
         counters.support_group_rebuild_count == 0;
}

bool C791_backbone_large_route_add_has_bounded_fixture_pipeline_counters() {
  city::wire::CoreState state;
  for (int route = 0; route < 12; ++route) {
    city::wire::BackboneSpec req = line_req(state);
    const double y = static_cast<double>(route) * 4.0;
    req.path.polyline = {{0.0, y, 0.0}, {12.0, y, 0.0}};
    const auto out = state.GenerateFromBackboneSpec(req);
    if (!out.ok) {
      return false;
    }
  }

  city::wire::instrumentation::reset();
  city::wire::BackboneSpec add = line_req(state);
  add.path.polyline = {{0.0, 60.0, 0.0}, {12.0, 60.0, 0.0}};
  const auto out = state.GenerateFromBackboneSpec(add);
  const city::wire::instrumentation::Counters counters =
      city::wire::instrumentation::snapshot();

  return out.ok &&
         counters.fixture_rule_merge_count == 1 &&
         counters.last_fixture_rule_count == 13 &&
         counters.fixture_plan_build_count == 1 &&
         counters.model_materialization_count == 1 &&
         counters.endpoint_placement_fallback_count == 0 &&
         counters.row_fixture_fallback_count == 0 &&
         counters.affected_span_derive_count == 0 &&
         counters.support_group_rebuild_count == 1;
}

bool C792_backbone_incremental_new_row_uses_empty_stable_slot() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId junction = first.value.generated_pole_ids[1];
  const city::wire::Pole* pole = state.view().poles().find(junction);
  if (pole == nullptr) {
    return false;
  }

  std::vector<std::pair<city::wire::ObjectId, city::wire::Vec3d>> existing_ports{};
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    const city::wire::Port* port = state.view().ports().find(binding.port_id);
    if (port != nullptr && port->owner_pole_id == junction &&
        binding.bundle_template_id == city::wire::kDefaultHighVoltageBundleTemplateId &&
        std::none_of(existing_ports.begin(), existing_ports.end(), [&](const auto& item) {
          return item.first == port->id;
        })) {
      existing_ports.push_back({port->id, port->world_position});
    }
  }
  if (existing_ports.size() != 6) {
    return false;
  }

  city::wire::BackboneSpec branch = hv_branch_req(state, junction, pole->world_transform.position);
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok || second.value.generated_span_ids.size() != 3) {
    return false;
  }

  double existing_height = 0.0;
  for (const auto& item : existing_ports) {
    const city::wire::Port* port = state.view().ports().find(item.first);
    if (port == nullptr || !almost_equal(port->world_position, item.second, 1e-9)) {
      return false;
    }
    existing_height += port->world_position.z;
  }
  existing_height /= static_cast<double>(existing_ports.size());

  std::vector<city::wire::ObjectId> new_junction_ports{};
  for (city::wire::ObjectId span_id : second.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    for (city::wire::ObjectId port_id : {span->port_a_id, span->port_b_id}) {
      const city::wire::Port* port = state.view().ports().find(port_id);
      if (port != nullptr && port->owner_pole_id == junction &&
          std::none_of(existing_ports.begin(), existing_ports.end(), [&](const auto& item) {
            return item.first == port_id;
          }) &&
          std::find(new_junction_ports.begin(), new_junction_ports.end(), port_id) == new_junction_ports.end()) {
        new_junction_ports.push_back(port_id);
      }
    }
  }
  if (new_junction_ports.size() != 3) {
    return false;
  }
  double new_height = 0.0;
  for (city::wire::ObjectId port_id : new_junction_ports) {
    const city::wire::Port* port = state.view().ports().find(port_id);
    if (port == nullptr) {
      return false;
    }
    new_height += port->world_position.z;
  }
  new_height /= static_cast<double>(new_junction_ports.size());

  return std::abs(new_height - existing_height) + 1e-9 >= 0.5;
}

bool C796_backbone_incremental_explicit_placement_height_is_not_row_reflowed() {
  auto explicit_bundle = [](const city::wire::CoreState& state,
                            city::wire::BundleKind kind,
                            std::uint64_t placement_key,
                            int count,
                            double height_m,
                            double lateral_m,
                            double spacing_m) {
    const city::wire::BundleTemplateId template_id =
        city::wire::DefaultBundleTemplateId(kind);
    const city::wire::BundleTemplate& tmpl = state.view().bundle_templates().at(template_id);
    city::wire::BackboneBundleSpec bundle{};
    bundle.bundle_template_id = template_id;
    bundle.placement_key = placement_key;
    bundle.layer = tmpl.default_layer;
    bundle.count = count;
    bundle.placement_explicit = true;
    bundle.height_m = height_m;
    bundle.lateral_m = lateral_m;
    bundle.spacing_m = spacing_m;
    return bundle;
  };
  auto default_mixed_req = [&](city::wire::CoreState& state) {
    city::wire::BackboneSpec req = poly3_req(state);
    req.bundles.clear();
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kHighVoltage, 3101, 3, 9.2, -0.2, 0.45));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kLowVoltage, 3201, 1, 7.35, 0.0, 0.30));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kLowVoltage, 3202, 1, 7.35, 0.0, 0.30));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kCommunication, 3401, 1, 6.4, 0.0, 0.30));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kOptical, 3501, 1, 6.0, 0.0, 0.30));
    return req;
  };
  auto branch_from = [&](city::wire::CoreState& state, city::wire::ObjectId pole_id,
                         const city::wire::Vec3d& pole_pos, const city::wire::Vec3d& end) {
    city::wire::BackboneSpec req = default_mixed_req(state);
    req.path.polyline = {pole_pos, end};
    req.path.node_specs = {pole_spec(0, pole_id)};
    req.node_bundle_modes.clear();
    for (const city::wire::BackboneBundleSpec& bundle : req.bundles) {
      city::wire::BackboneSpec::NodeBundleModeSpec mode{};
      mode.point_index = 0;
      mode.bundle_template_id = bundle.bundle_template_id;
      mode.mode = city::wire::BundleNodeMode::kPassThrough;
      req.node_bundle_modes.push_back(mode);
    }
    return req;
  };
  auto completion_to = [&](city::wire::CoreState& state, city::wire::ObjectId pole_id,
                           const city::wire::Vec3d& pole_pos, const city::wire::Vec3d& start) {
    city::wire::BackboneSpec req = default_mixed_req(state);
    req.path.polyline = {start, pole_pos};
    req.path.node_specs = {pole_spec(1, pole_id)};
    req.node_bundle_modes.clear();
    for (const city::wire::BackboneBundleSpec& bundle : req.bundles) {
      city::wire::BackboneSpec::NodeBundleModeSpec mode{};
      mode.point_index = 1;
      mode.bundle_template_id = bundle.bundle_template_id;
      mode.mode = city::wire::BundleNodeMode::kPassThrough;
      req.node_bundle_modes.push_back(mode);
    }
    return req;
  };
  const auto bundle_for_binding = [](const city::wire::CoreState& state,
                                     const city::wire::SavedBackbonePortBinding& binding)
      -> const city::wire::Bundle* {
    const city::wire::SavedBackboneEdgeBundle* edge_bundle =
        state.view().backbone_edge_bundle(binding.edge_bundle_id);
    return edge_bundle == nullptr ? nullptr : state.view().bundles().find(edge_bundle->bundle_id);
  };
  const auto row_key_equal = [](const city::wire::SavedBackboneRowKey& a,
                                const city::wire::SavedBackboneRowKey& b) {
    return a == b;
  };
  const auto local_height_for_binding = [](const city::wire::CoreState& state,
                                           const city::wire::SavedBackbonePortBinding& binding)
      -> std::optional<double> {
    const city::wire::Port* port = state.view().ports().find(binding.port_id);
    const city::wire::Pole* pole =
        port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
    if (port == nullptr || pole == nullptr) {
      return std::nullopt;
    }
    const city::wire::PoleFrame frame =
        city::wire::BuildPoleFrame(pole->world_transform, binding.layout_yaw_deg);
    return city::wire::WorldPointToLocal(frame, port->world_position).z;
  };

  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(default_mixed_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) return false;
  const city::wire::ObjectId junction = first.value.generated_pole_ids[1];
  const city::wire::Pole* pole = state.view().poles().find(junction);
  const city::wire::SavedBackboneNode* node = state.view().backbone_node_for_pole(junction);
  if (pole == nullptr || node == nullptr) return false;

  std::unordered_map<city::wire::ObjectId, city::wire::Vec3d> existing_positions{};
  std::unordered_map<std::uint64_t, std::vector<city::wire::ObjectId>> existing_by_key{};
  std::optional<double> existing_row_delta{};
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != node->node_id) continue;
    const city::wire::Port* port = state.view().ports().find(binding.port_id);
    const city::wire::Bundle* bundle = bundle_for_binding(state, binding);
    const auto local_height = local_height_for_binding(state, binding);
    if (port == nullptr || bundle == nullptr || !local_height.has_value()) return false;
    existing_positions.emplace(port->id, port->world_position);
    std::vector<city::wire::ObjectId>& key_ports = existing_by_key[bundle->placement_key];
    if (std::find(key_ports.begin(), key_ports.end(), port->id) == key_ports.end()) {
      key_ports.push_back(port->id);
    }
    const double delta = *local_height - bundle->height_m;
    if (!almost_equal(delta, 0.0, 1e-9)) {
      return false;
    }
    if (!existing_row_delta.has_value()) {
      existing_row_delta = delta;
    } else if (!almost_equal(*existing_row_delta, delta, 1e-9)) {
      return false;
    }
  }
  if (!existing_row_delta.has_value() || existing_by_key[3101].size() != 6 ||
      existing_by_key[3201].size() != 2 || existing_by_key[3202].size() != 2) {
    return false;
  }

  city::wire::BackboneSpec hv_only = line_req(state);
  hv_only.bundles.clear();
  hv_only.bundles.push_back(
      explicit_bundle(state, city::wire::BundleKind::kHighVoltage, 3101, 3, 9.2, -0.2, 0.45));
  hv_only.path.polyline = {pole->world_transform.position, {12.0, -6.0, 0.0}};
  hv_only.path.node_specs = {pole_spec(0, junction)};
  const auto hv_only_out = state.GenerateFromBackboneSpec(hv_only);
  if (!hv_only_out.ok) return false;
  for (const auto& [port_id, position] : existing_positions) {
    const city::wire::Port* port = state.view().ports().find(port_id);
    if (port == nullptr || !almost_equal(port->world_position, position, 1e-9)) {
      return false;
    }
  }

  city::wire::CoreState pair_state;
  const auto pair_first = pair_state.GenerateFromBackboneSpec(default_mixed_req(pair_state));
  if (!pair_first.ok || pair_first.value.generated_pole_ids.size() != 3) return false;
  const city::wire::ObjectId pair_junction = pair_first.value.generated_pole_ids[1];
  const city::wire::Pole* pair_pole = pair_state.view().poles().find(pair_junction);
  const city::wire::SavedBackboneNode* pair_node =
      pair_state.view().backbone_node_for_pole(pair_junction);
  if (pair_pole == nullptr || pair_node == nullptr) return false;

  std::unordered_set<city::wire::ObjectId> original_pair_ports{};
  for (const city::wire::SavedBackbonePortBinding& binding : pair_state.view().backbone().port_bindings) {
    if (binding.row_key.node_id == pair_node->node_id) {
      original_pair_ports.insert(binding.port_id);
    }
  }

  const auto bd_out =
      pair_state.GenerateFromBackboneSpec(branch_from(pair_state, pair_junction,
                                                      pair_pole->world_transform.position,
                                                      {12.0, -8.0, 0.0}));
  if (!bd_out.ok || bd_out.value.generated_span_ids.size() != 7) return false;
  const city::wire::SavedBackboneNode* pair_node_after_bd =
      pair_state.view().backbone_node_for_pole(pair_junction);
  if (pair_node_after_bd == nullptr) return false;

  std::optional<city::wire::SavedBackboneRowKey> new_open_row{};
  std::optional<double> new_row_delta{};
  std::unordered_map<city::wire::ObjectId, double> bd_local_height_by_port{};
  std::unordered_map<std::uint64_t, std::vector<city::wire::ObjectId>> new_ports_by_key{};
  for (const city::wire::SavedBackbonePortBinding& binding : pair_state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != pair_node_after_bd->node_id ||
        original_pair_ports.find(binding.port_id) != original_pair_ports.end()) {
      continue;
    }
    const city::wire::Bundle* bundle = bundle_for_binding(pair_state, binding);
    const auto local_height = local_height_for_binding(pair_state, binding);
    if (bundle == nullptr || !local_height.has_value()) return false;
    if (!new_open_row.has_value()) {
      new_open_row = binding.row_key;
    } else if (!row_key_equal(*new_open_row, binding.row_key)) {
      return false;
    }
    const double delta = *local_height - bundle->height_m;
    if (!almost_equal(delta, 0.0, 1e-9)) {
      return false;
    }
    if (!new_row_delta.has_value()) {
      new_row_delta = delta;
    } else if (!almost_equal(*new_row_delta, delta, 1e-9)) {
      return false;
    }
    bd_local_height_by_port.emplace(binding.port_id, *local_height);
    new_ports_by_key[bundle->placement_key].push_back(binding.port_id);
  }
  if (!new_open_row.has_value() || !new_row_delta.has_value() ||
      !almost_equal(*new_row_delta, *existing_row_delta, 1e-9) ||
      new_ports_by_key[3101].size() != 3 ||
      new_ports_by_key[3201].size() != 1 ||
      new_ports_by_key[3202].size() != 1 ||
      new_ports_by_key[3201].front() == new_ports_by_key[3202].front()) {
    return false;
  }

  const auto eb_out =
      pair_state.GenerateFromBackboneSpec(completion_to(pair_state, pair_junction,
                                                        pair_pole->world_transform.position,
                                                        {20.0, 0.0, 0.0}));
  if (!eb_out.ok || eb_out.value.generated_span_ids.size() != 7) return false;
  const city::wire::ObjectId e = eb_out.value.generated_pole_ids.front();
  const city::wire::SavedBackboneNode* pair_node_after =
      pair_state.view().backbone_node_for_pole(pair_junction);
  const city::wire::SavedBackboneNode* e_node = pair_state.view().backbone_node_for_pole(e);
  if (pair_node_after == nullptr || e_node == nullptr) return false;
  std::size_t continued_bindings = 0;
  for (const city::wire::SavedBackbonePortBinding& binding : pair_state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != pair_node_after->node_id) continue;
    const auto before_height = bd_local_height_by_port.find(binding.port_id);
    if (before_height == bd_local_height_by_port.end()) continue;
    const auto after_height = local_height_for_binding(pair_state, binding);
    if (!after_height.has_value() || !almost_equal(*after_height, before_height->second, 1e-9)) {
      return false;
    }
    ++continued_bindings;
  }
  return continued_bindings >= bd_local_height_by_port.size() &&
         curve_endpoints_match_layout(pair_state);
}

bool C480_backbone_context_rows_affect_order_but_are_not_emitted() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  std::unordered_set<city::wire::ObjectId> b_ports_before{};
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    const city::wire::Port* port = state.view().ports().find(binding.port_id);
    if (port != nullptr && port->owner_pole_id == b) {
      b_ports_before.insert(port->id);
    }
  }
  city::wire::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok) {
    return false;
  }
  std::unordered_set<city::wire::ObjectId> b_ports_after{};
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    const city::wire::Port* port = state.view().ports().find(binding.port_id);
    if (port == nullptr || port->owner_pole_id != b) {
      continue;
    }
    b_ports_after.insert(port->id);
  }
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  std::string emit_ports_body;
  if (!file_text(source, &cpp) ||
      !function_body(cpp, "EditResult<bool> pipeline::emit_ports(topo* made, const pairs& ps, ChangeSet* changes)",
                     &emit_ports_body)) {
    return false;
  }
  const JunctionRowSnapshot snapshot = junction_snapshot(state, b);
  return !second.value.generated_span_ids.empty() &&
         b_ports_after.size() == b_ports_before.size() + second.value.generated_span_ids.size() &&
         snapshot.pair_rows == 1 && snapshot.open_rows == 1 &&
         contains_text(emit_ports_body, "row_height_offsets(ps)") &&
         contains_text(emit_ports_body, "AllowsBranchHeightOffset") &&
         contains_text(emit_ports_body, "canonical row reflow requires moving manual ports") &&
         !contains_text(emit_ports_body, "if (r.id >= active_rows.size() || !active_rows[r.id])");
}

bool C481_backbone_pass_through_mode_is_accepted_in_limited_scope() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const city::wire::Vec3d pole_b_position = pole_b->world_transform.position;
  const auto ok = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b_position));

  city::wire::BackboneSpec bad_index = pass_branch_req(state, b, pole_b_position);
  bad_index.node_bundle_modes.front().point_index = 9;
  const auto bad_index_out = state.GenerateFromBackboneSpec(bad_index);

  city::wire::BackboneSpec bad_bundle = pass_branch_req(state, b, pole_b_position);
  bad_bundle.node_bundle_modes.front().bundle_template_id = 999;
  const auto bad_bundle_out = state.GenerateFromBackboneSpec(bad_bundle);
  return ok.ok && !bad_index_out.ok && !bad_bundle_out.ok;
}

bool C482_backbone_pass_through_creates_explicit_intent() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  bool saw_intent = false;
  for (city::wire::ObjectId span_id : second.value.generated_span_ids) {
    const city::wire::SpanLayoutRulesView rules = state.span_layout_rules(span_id);
    const city::wire::SpanLayoutView layout = state.span_layout(span_id);
    if (!rules.has_rule() || !layout.has_layout()) {
      return false;
    }
    saw_intent = saw_intent || rules.rule->lowering_kind == city::wire::BackboneLoweringKind::kBranchSupport ||
                 rules.rule->start.default_lower_required || rules.rule->end.default_lower_required ||
                 layout.entry->start.default_lower_required || layout.entry->end.default_lower_required;
  }
  return saw_intent && C479_backbone_row_separation_does_not_change_pairs();
}

bool C483_backbone_pass_through_ambiguous_target_rejected() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.cpp";
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

bool C484_backbone_lowering_draw_uses_layout_only() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  bool saw_lowered_layout = false;
  for (city::wire::ObjectId span_id : second.value.generated_span_ids) {
    const city::wire::SpanLayoutView layout = state.span_layout(span_id);
    const city::wire::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    const city::wire::SpanRenderCacheEntry* render = state.find_span_render_cache(span_id);
    if (!layout.has_layout() || visual == nullptr || render == nullptr) {
      return false;
    }
    const auto lowered = [](const city::wire::LayoutEndpoint& endpoint) {
      return (endpoint.default_lower_required || endpoint.lower_required) &&
             endpoint.branch_down_offset_m > 1e-9 &&
             almost_equal(endpoint.support_world, endpoint.endpoint_world, 1e-9);
    };
    saw_lowered_layout = saw_lowered_layout || lowered(layout.entry->start) || lowered(layout.entry->end);
    if (!visual->parts.empty()) {
      return false;
    }
  }
  return saw_lowered_layout;
}

bool C486_backbone_pass_through_is_deterministic() {
  const std::vector<city::wire::Vec3d> a = pass_intent_points();
  const std::vector<city::wire::Vec3d> b = pass_intent_points();
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

bool C488_backbone_port_resolution_accepts_same_compatible_binding() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok || out.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = out.value.generated_pole_ids[1];
  std::vector<const city::wire::SavedBackbonePortBinding*> middle{};
  for (const auto& binding : state.view().backbone().port_bindings) {
    const city::wire::Port* port = state.view().ports().find(binding.port_id);
    if (port != nullptr && port->owner_pole_id == b) middle.push_back(&binding);
  }
  if (middle.size() != 2 || middle[0]->port_id == middle[1]->port_id) return false;
  const city::wire::Port* a = state.view().ports().find(middle[0]->port_id);
  const city::wire::Port* c = state.view().ports().find(middle[1]->port_id);
  return a != nullptr && c != nullptr &&
         state.view().backbone_port_bindings_for_port(a->id).size() == 1 &&
         state.view().backbone_port_bindings_for_port(c->id).size() == 1 &&
         middle[0]->bundle_template_id == middle[1]->bundle_template_id &&
         middle[0]->port_kind == middle[1]->port_kind &&
         middle[0]->port_layer == middle[1]->port_layer &&
         a->world_position.x == c->world_position.x &&
         a->world_position.y == c->world_position.y &&
         a->world_position.z == c->world_position.z;
}

bool C489_backbone_port_binding_index_invariant() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok) {
    return false;
  }
  bool saw_binding = false;
  for (const city::wire::Port& port : state.view().ports().items()) {
    const std::vector<const city::wire::SavedBackbonePortBinding*> bindings =
        state.view().backbone_port_bindings_for_port(port.id);
    if (bindings.empty()) continue;
    saw_binding = true;
    if (bindings.size() != 1 || bindings.front() == nullptr ||
        bindings.front()->port_kind != port.kind ||
        bindings.front()->port_layer != port.layer) return false;
  }
  return saw_binding;
}

bool C490_backbone_duplicate_same_edge_bundle_lane_rejected() {
  return C463_backbone_duplicate_same_edge_bundle_rejected() && C466_backbone_duplicate_reject_keeps_state_unchanged();
}

bool C502_backbone_span_bindings_save_lane() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  const city::wire::SavedBackboneGraph& graph = state.view().backbone();
  if (!out.ok || graph.edge_bundles.size() != 1 || graph.span_bindings.size() != out.value.generated_span_ids.size()) {
    return false;
  }
  const city::wire::ObjectId edge_bundle_id = graph.edge_bundles.front().edge_bundle_id;
  const city::wire::SavedBackboneEdgeBundle& edge_bundle = graph.edge_bundles.front();
  const city::wire::BackboneIndex& backbone_index = state.view().backbone_index();
  const auto position = backbone_index.edge_bundle_positions.find(edge_bundle_id);
  const auto by_pair = backbone_index.edge_bundle_by_edge_and_bundle.find(
      {edge_bundle.edge_id, edge_bundle.bundle_id});
  if (position == backbone_index.edge_bundle_positions.end() || position->second != 0 ||
      by_pair == backbone_index.edge_bundle_by_edge_and_bundle.end() || by_pair->second != edge_bundle_id ||
      state.view().backbone_edge_bundle(edge_bundle_id) != &graph.edge_bundles.front()) {
    return false;
  }
  const auto index_it = state.view().backbone_index().edge_bundle_span_bindings.find(edge_bundle_id);
  if (index_it == state.view().backbone_index().edge_bundle_span_bindings.end() ||
      index_it->second.size() != graph.span_bindings.size()) {
    return false;
  }
  std::unordered_set<std::size_t> lanes{};
  for (const city::wire::SavedBackboneSpanBinding& binding : graph.span_bindings) {
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

bool C503_backbone_duplicate_span_binding_rejected_by_lane() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "state" / "backbone.cpp";
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

bool C491_backbone_branch_lowering_v1_affects_geom() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (city::wire::ObjectId span_id : second.value.generated_span_ids) {
    if (span_has_lowered_endpoint(state, span_id)) {
      return true;
    }
  }
  return false;
}

bool C492_backbone_cross_lowering_v1_affects_only_new_links() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const std::size_t span_count = state.view().spans().size();
  const std::size_t port_count = state.view().ports().size();
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  city::wire::BackboneSpec cross = line_req(state);
  cross.bundles.clear();
  add_backbone_bundle(cross, city::wire::BundleKind::kHighVoltage);
  cross.path.polyline = {{12.0, -8.0, 0.0}, pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  cross.path.node_specs = {pole_spec(1, b)};
  cross.node_bundle_modes.push_back({1, cross.bundles.front().bundle_template_id,
                                     city::wire::BundleNodeMode::kPassThrough});
  const auto second = state.GenerateFromBackboneSpec(cross);
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  bool saw_lowered = false;
  for (city::wire::ObjectId span_id : second.value.generated_span_ids) {
    saw_lowered = saw_lowered || span_has_lowered_endpoint(state, span_id);
  }
  return saw_lowered && state.view().spans().size() == span_count + second.value.generated_span_ids.size() &&
         state.view().ports().size() > port_count;
}

bool C496_backbone_junction_v1_deterministic() {
  const std::vector<city::wire::Vec3d> a = junction_v1_points();
  const std::vector<city::wire::Vec3d> b = junction_v1_points();
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

bool C497_backbone_context_rows_order_but_do_not_materialize() {
  return C480_backbone_context_rows_affect_order_but_are_not_emitted();
}

bool C498_backbone_saved_graph_remains_topology_authority() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok) {
    return false;
  }
  const city::wire::BackboneFrontier frontier = state.view().pole_frontier(b);
  return frontier.edge_ids.size() == 3 && frontier.edge_bundle_ids.size() == 3 &&
         frontier.span_ids.size() == first.value.generated_span_ids.size() + second.value.generated_span_ids.size();
}

bool C499_backbone_context_link_is_not_saved() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3 || state.view().backbone().edges.size() != 2) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  city::wire::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok) {
    return false;
  }
  const city::wire::BackboneFrontier frontier = state.view().pole_frontier(b);
  return state.view().backbone().edges.size() == 3 && frontier.edge_ids.size() == 3 &&
         state.view().backbone().edge_bundles.size() == 3;
}

bool C500_backbone_context_link_requires_saved_edge_ref() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  std::string ref_body;
  std::string save_body;
  if (!function_body(cpp, "SavedBackboneEdgeRef ref_for_existing_edge", &ref_body) ||
      !function_body(cpp, "EditResult<bool> pipeline::save_graph(const topo& made, const pairs& ps,", &save_body)) {
    return false;
  }
  return contains_text(ref_body, "edge.saved == kInvalidObjectId") && !contains_text(ref_body, "saved_edge_for") &&
         contains_text(save_body, "context link saved edge missing");
}

bool C797_backbone_row_continuity_records_route_and_promotion_lanes() {
  auto hv_req = [](city::wire::CoreState& state, bool polyline3) {
    city::wire::BackboneSpec req = polyline3 ? poly3_req(state) : line_req(state);
    req.bundles.clear();
    add_backbone_bundle(req, city::wire::BundleKind::kHighVoltage);
    return req;
  };
  auto expect_hv_lane_continuity = [](const city::wire::CoreState& state,
                                      city::wire::ObjectId node_id,
                                      city::wire::ObjectId edge_a,
                                      city::wire::ObjectId edge_b) {
    for (std::size_t lane = 0; lane < 3; ++lane) {
      const city::wire::ObjectId edge_bundle_a = edge_bundle_for_edge_and_lane(state, edge_a, lane);
      const city::wire::ObjectId edge_bundle_b = edge_bundle_for_edge_and_lane(state, edge_b, lane);
      if (edge_bundle_a == city::wire::kInvalidObjectId ||
          edge_bundle_b == city::wire::kInvalidObjectId ||
          !has_row_continuity(state, node_id, edge_bundle_a, lane, edge_bundle_b, lane)) {
        return false;
      }
    }
    return true;
  };

  city::wire::CoreState straight;
  const auto straight_out = straight.GenerateFromBackboneSpec(hv_req(straight, false));
  if (!straight_out.ok || straight_out.value.generated_pole_ids.size() != 2) {
    return false;
  }
  for (city::wire::ObjectId pole_id : straight_out.value.generated_pole_ids) {
    const city::wire::SavedBackboneNode* node = straight.view().backbone_node_for_pole(pole_id);
    if (node == nullptr || !straight.view().backbone_row_continuities_for_node(node->node_id).empty()) {
      return false;
    }
  }

  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(hv_req(state, true));
  if (!out.ok || out.value.generated_pole_ids.size() != 3) return false;
  const city::wire::SavedBackboneNode* node_a =
      state.view().backbone_node_for_pole(out.value.generated_pole_ids[0]);
  const city::wire::SavedBackboneNode* node_b =
      state.view().backbone_node_for_pole(out.value.generated_pole_ids[1]);
  const city::wire::SavedBackboneNode* node_c =
      state.view().backbone_node_for_pole(out.value.generated_pole_ids[2]);
  if (node_a == nullptr || node_b == nullptr || node_c == nullptr) {
    return false;
  }
  const city::wire::ObjectId ab_edge = edge_between(state, node_a->node_id, node_b->node_id);
  const city::wire::ObjectId bc_edge = edge_between(state, node_b->node_id, node_c->node_id);
  const city::wire::ObjectId node_b_id = node_b->node_id;
  if (ab_edge == city::wire::kInvalidObjectId || bc_edge == city::wire::kInvalidObjectId) {
    return false;
  }
  if (!state.view().backbone_row_continuities_for_node(node_a->node_id).empty() ||
      !state.view().backbone_row_continuities_for_node(node_c->node_id).empty()) {
    return false;
  }
  if (!expect_hv_lane_continuity(state, node_b_id, ab_edge, bc_edge)) {
    return false;
  }

  const city::wire::Pole* pole_b = state.view().poles().find(out.value.generated_pole_ids[1]);
  if (pole_b == nullptr) return false;
  city::wire::BackboneSpec branch = hv_req(state, false);
  branch.path.polyline = {pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, out.value.generated_pole_ids[1])};
  const auto branch_out = state.GenerateFromBackboneSpec(branch);
  if (!branch_out.ok || branch_out.value.generated_pole_ids.size() != 1 ||
      !expect_hv_lane_continuity(state, node_b_id, ab_edge, bc_edge)) {
    return false;
  }

  IncrementalCrossFixture cross{};
  if (!make_incremental_cross(&cross) ||
      !cross.completion.ok || !canonical_cross_at_b(cross)) {
    return false;
  }
  const city::wire::SavedBackboneNode* cross_b = cross.state.view().backbone_node_for_pole(cross.pole_b);
  const city::wire::SavedBackboneNode* cross_d = cross.state.view().backbone_node_for_pole(cross.pole_d);
  const city::wire::SavedBackboneNode* cross_e = cross.state.view().backbone_node_for_pole(cross.pole_e);
  if (cross_b == nullptr || cross_d == nullptr || cross_e == nullptr) {
    return false;
  }
  const city::wire::ObjectId cross_bd = edge_between(cross.state, cross_b->node_id, cross_d->node_id);
  const city::wire::ObjectId cross_be = edge_between(cross.state, cross_b->node_id, cross_e->node_id);
  const city::wire::ObjectId bd_bundle = edge_bundle_for_edge_and_lane(cross.state, cross_bd, 0);
  const city::wire::ObjectId be_bundle = edge_bundle_for_edge_and_lane(cross.state, cross_be, 0);
  return bd_bundle != city::wire::kInvalidObjectId &&
         be_bundle != city::wire::kInvalidObjectId &&
         has_row_continuity(cross.state, cross_b->node_id, bd_bundle, 0, be_bundle, 0);
}

bool viewer_default_t_branch_keeps_hv_and_only_flagged_lowering(bool anchor_at_end) {
  auto explicit_bundle = [](const city::wire::CoreState& state,
                            city::wire::BundleKind kind,
                            std::uint64_t placement_key,
                            int count,
                            double height_m,
                            double lateral_m,
                            double spacing_m) {
    const city::wire::BundleTemplateId template_id =
        city::wire::DefaultBundleTemplateId(kind);
    const city::wire::BundleTemplate& tmpl = state.view().bundle_templates().at(template_id);
    city::wire::BackboneBundleSpec bundle{};
    bundle.bundle_template_id = template_id;
    bundle.placement_key = placement_key;
    bundle.layer = tmpl.default_layer;
    bundle.count = count;
    bundle.placement_explicit = true;
    bundle.height_m = height_m;
    bundle.lateral_m = lateral_m;
    bundle.spacing_m = spacing_m;
    return bundle;
  };
  auto viewer_default_req = [&](city::wire::CoreState& state) {
    city::wire::BackboneSpec req = poly3_req(state);
    req.pole_type_id = 2;
    req.pole_placement.enable_tilt = true;
    req.pole_placement.max_tilt_deg = 12.0;
    req.bundles.clear();
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kHighVoltage, 1, 3, 9.2, -0.2, 0.45));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kLowVoltage, 2, 1, 7.7, 0.0, 0.2));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kLowVoltage, 3, 1, 7.35, 0.0, 0.2));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kLowVoltage, 4, 1, 7.0, 0.0, 0.2));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kCommunication, 5, 1, 5.5, 0.0, 0.2));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kOptical, 6, 1, 5.3, 0.0, 0.2));
    return req;
  };
  auto span_template = [](const city::wire::CoreState& state, city::wire::ObjectId span_id) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    const city::wire::Bundle* bundle = span == nullptr ? nullptr : state.view().bundles().find(span->bundle_id);
    return bundle == nullptr ? city::wire::kInvalidBundleTemplateId : bundle->bundle_template_id;
  };
  auto endpoint_stays_at_port = [](const city::wire::CoreState& state,
                                   const city::wire::LayoutEndpoint& endpoint) {
    const city::wire::Port* port = state.view().ports().find(endpoint.port_id);
    return port != nullptr &&
           !endpoint.default_lower_required &&
           !endpoint.lower_required &&
           endpoint.branch_down_offset_m <= 1e-9 &&
           almost_equal(endpoint.support_world.z, port->world_position.z, 1e-9) &&
           almost_equal(endpoint.endpoint_world.z, port->world_position.z, 1e-9);
  };
  auto endpoint_follows_template_policy = [&](const city::wire::CoreState& state,
                                              const city::wire::LayoutEndpoint& endpoint,
                                              bool expect_lowered) {
    const city::wire::Port* port = state.view().ports().find(endpoint.port_id);
    if (port == nullptr) return false;
    if (!expect_lowered) {
      return endpoint_stays_at_port(state, endpoint);
    }
    const city::wire::Pole* pole = state.view().poles().find(port->owner_pole_id);
    if (pole == nullptr) return false;
    const double layout_yaw = state.effective_port_layout_yaw_deg(*pole, port->id, port->category);
    const city::wire::PoleFrame frame = city::wire::BuildPoleFrame(pole->world_transform, layout_yaw);
    const city::wire::Vec3d port_local = city::wire::WorldPointToLocal(frame, port->world_position);
    const city::wire::Vec3d endpoint_local = city::wire::WorldPointToLocal(frame, endpoint.endpoint_world);
    return endpoint.default_lower_required &&
           endpoint.lower_required &&
           endpoint.branch_down_offset_m > 1e-9 &&
           almost_equal(endpoint_local.z, port_local.z - endpoint.branch_down_offset_m, 1e-9) &&
           almost_equal(endpoint.endpoint_world, endpoint.support_world, 1e-9);
  };

  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(viewer_default_req(state));
  WIRE_TEST_EXPECT(first.ok, first.error);
  WIRE_TEST_EXPECT(first.value.generated_pole_ids.size() == 3, "viewer default base did not generate 3 poles");
  WIRE_TEST_EXPECT(first.value.generated_span_ids.size() == 16, "viewer default base did not generate 16 spans");
  const city::wire::ObjectId junction = first.value.generated_pole_ids[1];
  std::unordered_map<city::wire::ObjectId, city::wire::Vec3d> port_positions_before{};
  for (const city::wire::Port& port : state.view().ports().items()) {
    port_positions_before.emplace(port.id, port.world_position);
  }
  const city::wire::Pole* pole = state.view().poles().find(junction);
  WIRE_TEST_EXPECT(pole != nullptr, "junction pole is missing");
  const city::wire::SavedBackboneNode* junction_node = state.view().backbone_node_for_pole(junction);
  WIRE_TEST_EXPECT(junction_node != nullptr, "junction backbone node is missing");
  const city::wire::SavedBackboneEdge* incident = nullptr;
  for (const city::wire::SavedBackboneEdge& edge : state.view().backbone().edges) {
    if (edge.node_a == junction_node->node_id || edge.node_b == junction_node->node_id) {
      incident = &edge;
      break;
    }
  }
  WIRE_TEST_EXPECT(incident != nullptr, "junction has no incident saved backbone edge");
  const city::wire::ObjectId peer_node_id =
      incident->node_a == junction_node->node_id ? incident->node_b : incident->node_a;
  const city::wire::SavedBackboneNode* peer_node = state.view().backbone_node(peer_node_id);
  WIRE_TEST_EXPECT(peer_node != nullptr, "peer backbone node is missing");

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_id = city::wire::kInvalidObjectId;
  pick.hit_pos_world = {pole->world_transform.position.x + 0.12,
                        pole->world_transform.position.y - 0.10,
                        9.2};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = junction_node->node_id;
  pick.segment_node_b_id = peer_node->node_id;
  pick.segment_endpoint_a_world = junction_node->position;
  pick.segment_endpoint_b_world = peer_node->position;
  city::wire::ResolveBranchPickOptions pick_options{};
  pick_options.selected_bundle_template_ids = {
      city::wire::kDefaultHighVoltageBundleTemplateId,
      city::wire::kDefaultLowVoltageBundleTemplateId,
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kCommunication),
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kOptical),
  };
  const auto resolved = state.ResolveBranchPick(pick, pick_options);
  WIRE_TEST_EXPECT(resolved.ok, resolved.error);
  WIRE_TEST_EXPECT(resolved.value.support_kind == city::wire::SupportKind::kPole,
                   "segment endpoint snap did not resolve to pole");
  WIRE_TEST_EXPECT(resolved.value.resolved_node_id == junction_node->node_id,
                   "segment endpoint snap resolved to the wrong backbone node");
  WIRE_TEST_EXPECT(resolved.value.snapped_from_segment_endpoint,
                   "segment endpoint snap flag was not set");

  city::wire::BackboneSpec branch = viewer_default_req(state);
  const city::wire::Vec3d new_point{12.0, -8.0, 0.0};
  branch.path.polyline = anchor_at_end
      ? std::vector<city::wire::Vec3d>{new_point, resolved.value.position}
      : std::vector<city::wire::Vec3d>{resolved.value.position, new_point};
  branch.path.node_specs = {pole_spec(anchor_at_end ? 1 : 0, resolved.value.resolved_node_id)};
  const auto out = state.GenerateFromBackboneSpec(branch);
  WIRE_TEST_EXPECT(out.ok, out.error);
  WIRE_TEST_EXPECT(out.value.generated_pole_ids.size() == 1, "viewer default branch did not generate one pole");
  WIRE_TEST_EXPECT(out.value.generated_span_ids.size() == 8, "viewer default branch did not generate 8 spans");

  std::unordered_map<city::wire::BundleTemplateId, std::size_t> generated_by_template{};
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    ++generated_by_template[span_template(state, span_id)];
  }
  if (generated_by_template[city::wire::kDefaultHighVoltageBundleTemplateId] != 3 ||
      generated_by_template[city::wire::kDefaultLowVoltageBundleTemplateId] != 3 ||
      generated_by_template[city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kCommunication)] != 1 ||
      generated_by_template[city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kOptical)] != 1) {
    test_registry::SetFailureReason("viewer default branch generated wrong template span counts");
    return false;
  }

  const std::unordered_set<city::wire::ObjectId> branch_span_ids(
      out.value.generated_span_ids.begin(), out.value.generated_span_ids.end());
  for (const city::wire::Span& span : state.view().spans().items()) {
    const city::wire::Bundle* bundle = state.view().bundles().find(span.bundle_id);
    const auto template_it = bundle == nullptr
        ? state.view().bundle_templates().end()
        : state.view().bundle_templates().find(bundle->bundle_template_id);
    WIRE_TEST_EXPECT(bundle != nullptr && template_it != state.view().bundle_templates().end(),
                     "span bundle/template is missing");
    const city::wire::SpanLayoutView layout = state.span_layout(span.id);
    WIRE_TEST_EXPECT(layout.has_layout(), "span layout is missing");
    const bool flagged = branch_span_ids.contains(span.id) &&
                         template_it->second.enable_branch_down_offset &&
                         template_it->second.branch_endpoint_offset_m < -1e-9;
    const city::wire::Port* start_port = state.view().ports().find(layout.entry->start.port_id);
    const city::wire::Port* end_port = state.view().ports().find(layout.entry->end.port_id);
    const bool start_at_junction = start_port != nullptr && start_port->owner_pole_id == junction;
    const bool end_at_junction = end_port != nullptr && end_port->owner_pole_id == junction;
    if (!endpoint_follows_template_policy(state, layout.entry->start, flagged && start_at_junction) ||
        !endpoint_follows_template_policy(state, layout.entry->end, flagged && end_at_junction)) {
      test_registry::SetFailureReason("branch endpoint lowering did not follow template policy");
      return false;
    }
  }
  for (const auto& [port_id, before_position] : port_positions_before) {
    const city::wire::Port* after_port = state.view().ports().find(port_id);
    if (after_port == nullptr || !almost_equal(after_port->world_position, before_position, 1e-9)) {
      test_registry::SetFailureReason("existing port position changed during T branch generation");
      return false;
    }
  }

  WIRE_TEST_EXPECT(curve_endpoints_match_layout(state), "curve endpoints do not match layout");
  std::string invariant_error{};
  WIRE_TEST_EXPECT(backbone_common_invariants_pass(state, &invariant_error), invariant_error);
  return true;
}

bool C798_backbone_viewer_default_t_branch_keeps_hv_and_only_flagged_lowering() {
  return viewer_default_t_branch_keeps_hv_and_only_flagged_lowering(false);
}

bool C802_backbone_viewer_default_reverse_t_branch_keeps_hv_and_only_flagged_lowering() {
  return viewer_default_t_branch_keeps_hv_and_only_flagged_lowering(true);
}

bool C808_backbone_branch_lowering_uses_template_flag_not_hv_category() {
  city::wire::CoreState state;
  const city::wire::BundleTemplateId hv_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage);
  const city::wire::BundleTemplateId lv_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage);
  city::wire::BundleTemplate hv = state.view().bundle_templates().at(hv_id);
  hv.enable_branch_down_offset = false;
  hv.branch_endpoint_offset_m = -0.275;
  city::wire::BundleTemplate lv = state.view().bundle_templates().at(lv_id);
  lv.enable_branch_down_offset = true;
  lv.branch_endpoint_offset_m = -0.325;
  if (!state.UpdateBundleTemplate(hv).ok || !state.UpdateBundleTemplate(lv).ok) {
    return false;
  }

  city::wire::BackboneSpec base = poly3_req(state);
  add_backbone_bundle(base, city::wire::BundleKind::kHighVoltage);
  const auto first = state.GenerateFromBackboneSpec(base);
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId junction = first.value.generated_pole_ids[1];
  const city::wire::Pole* pole = state.view().poles().find(junction);
  const city::wire::SavedBackboneNode* node = state.view().backbone_node_for_pole(junction);
  if (pole == nullptr || node == nullptr) {
    return false;
  }

  city::wire::BackboneSpec branch = line_req(state);
  add_backbone_bundle(branch, city::wire::BundleKind::kHighVoltage);
  branch.path.polyline = {pole->world_transform.position, {20.0, -8.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, node->node_id)};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.generated_span_ids.size() != 4) {
    return false;
  }

  std::size_t hv_spans = 0;
  std::size_t lv_spans = 0;
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    const city::wire::Bundle* bundle = span == nullptr ? nullptr : state.view().bundles().find(span->bundle_id);
    if (span == nullptr || bundle == nullptr) {
      return false;
    }
    const bool is_hv = bundle->bundle_template_id == hv_id;
    const bool is_lv = bundle->bundle_template_id == lv_id;
    if (!is_hv && !is_lv) {
      return false;
    }
    hv_spans += is_hv ? 1U : 0U;
    lv_spans += is_lv ? 1U : 0U;
    const city::wire::SpanLayoutView layout = state.span_layout(span_id);
    if (!layout.has_layout()) {
      return false;
    }
    const city::wire::Port* start_port = state.view().ports().find(layout.entry->start.port_id);
    const city::wire::Port* end_port = state.view().ports().find(layout.entry->end.port_id);
    const bool start_at_junction = start_port != nullptr && start_port->owner_pole_id == junction;
    const bool end_at_junction = end_port != nullptr && end_port->owner_pole_id == junction;
    const city::wire::LayoutEndpoint& endpoint =
        start_at_junction ? layout.entry->start : layout.entry->end;
    const city::wire::SavedBackbonePortBinding* junction_binding =
        state.view().backbone_port_binding_for_port(endpoint.port_id);
    const std::optional<double> junction_port_height =
        junction_binding == nullptr ? std::nullopt : local_port_height_for_binding(state, *junction_binding);
    const std::optional<double> junction_nominal_height =
        junction_binding == nullptr ? std::nullopt : nominal_band_height_for_binding(state, *junction_binding);
    if ((!start_at_junction && !end_at_junction) || (is_hv && endpoint.default_lower_required) ||
        (is_lv && (!endpoint.default_lower_required || endpoint.branch_down_offset_m <= 0.1))) {
      return false;
    }
    WIRE_TEST_EXPECT(junction_binding != nullptr && junction_port_height.has_value() &&
                         junction_nominal_height.has_value(),
                     "junction port binding or nominal band height is missing");
    if (is_hv) {
      WIRE_TEST_EXPECT(almost_equal(*junction_port_height, *junction_nominal_height, 1e-9),
                       "non-flagged HV junction port height moved away from its placement band");
    }
  }
  return hv_spans == 3 && lv_spans == 1 && curve_endpoints_match_layout(state);
}

bool C809_backbone_incremental_rows_use_one_support_level_per_pair() {
  city::wire::CoreState state;
  const city::wire::BundleTemplate& hv =
      state.view().bundle_templates().at(
          city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage));
  const double step = std::max(0.0, -hv.branch_endpoint_offset_m);
  WIRE_TEST_EXPECT(hv.enable_branch_down_offset, "default HV branch-down offset flag is disabled");
  WIRE_TEST_EXPECT(step > 0.0, "default HV branch-down offset step is not positive");
  const auto expected = [&](const city::wire::CoreState& current,
                            city::wire::ObjectId pole_id,
                            std::initializer_list<double> levels) {
    const std::vector<double> actual = hv_row_down_offsets_at_pole(current, pole_id);
    if (actual.size() != levels.size()) {
      return false;
    }
    std::size_t index = 0;
    for (double level : levels) {
      if (!almost_equal(actual[index], level * step, 1e-9)) {
        return false;
      }
      ++index;
    }
    return true;
  };
  const auto use_explicit_hv_placement = [](city::wire::BackboneSpec* request) {
    if (request == nullptr || request->bundles.size() != 1) {
      return false;
    }
    city::wire::BackboneBundleSpec& bundle = request->bundles.front();
    bundle.placement_key = 1;
    bundle.placement_explicit = true;
    bundle.count = 3;
    bundle.height_m = 9.2;
    bundle.lateral_m = -0.2;
    bundle.spacing_m = 0.45;
    return true;
  };
  const auto add_hv_edge = [&](city::wire::ObjectId junction,
                               const city::wire::Vec3d& point,
                               bool junction_at_end) {
    const city::wire::Pole* pole = state.view().poles().find(junction);
    if (pole == nullptr) {
      return false;
    }
    city::wire::BackboneSpec request = line_req(state);
    request.bundles.clear();
    add_backbone_bundle(request, city::wire::BundleKind::kHighVoltage);
    if (!use_explicit_hv_placement(&request)) {
      return false;
    }
    request.path.polyline = junction_at_end
                                ? std::vector<city::wire::Vec3d>{point, pole->world_transform.position}
                                : std::vector<city::wire::Vec3d>{pole->world_transform.position, point};
    request.path.node_specs = {pole_spec(junction_at_end ? 1 : 0, junction)};
    const auto result = state.GenerateFromBackboneSpec(request);
    return result.ok && result.value.generated_pole_ids.size() == 1;
  };

  city::wire::BackboneSpec base_request = hv_poly3_req(state);
  WIRE_TEST_EXPECT(use_explicit_hv_placement(&base_request), "failed to apply explicit HV placement to base request");
  const auto base = state.GenerateFromBackboneSpec(base_request);
  WIRE_TEST_EXPECT(base.ok, base.error);
  WIRE_TEST_EXPECT(base.value.generated_pole_ids.size() == 3, "base HV polyline did not generate 3 poles");
  const city::wire::ObjectId junction = base.value.generated_pole_ids[1];
  WIRE_TEST_EXPECT(expected(state, junction, {0.0}), "base junction levels are not {0}");
  WIRE_TEST_EXPECT(add_hv_edge(junction, {20.0, -8.0, 0.0}, false), "failed to add first HV branch edge");
  WIRE_TEST_EXPECT(expected(state, junction, {0.0, 1.0}), "degree 3 junction levels are not {0,1}");
  WIRE_TEST_EXPECT(add_hv_edge(junction, {4.0, 8.0, 0.0}, true), "failed to add opposite HV branch edge");
  WIRE_TEST_EXPECT(expected(state, junction, {0.0, 1.0}), "degree 4 junction levels changed unexpectedly");
  WIRE_TEST_EXPECT(add_hv_edge(junction, {20.0, 8.0, 0.0}, false), "failed to add third HV branch edge");
  WIRE_TEST_EXPECT(expected(state, junction, {0.0, 1.0, 2.0}), "degree 5 junction levels are not {0,1,2}");
  WIRE_TEST_EXPECT(add_hv_edge(junction, {4.0, -8.0, 0.0}, true), "failed to add fourth HV branch edge");
  WIRE_TEST_EXPECT(expected(state, junction, {0.0, 1.0, 2.0}), "degree 6 junction levels changed unexpectedly");

  std::string saved{};
  const auto saved_out = state.SerializeAuthoritative(&saved);
  WIRE_TEST_EXPECT(saved_out.ok, saved_out.error);
  city::wire::CoreState loaded;
  const auto loaded_result = loaded.DeserializeAuthoritative(saved);
  WIRE_TEST_EXPECT(loaded_result.ok, loaded_result.error);
  WIRE_TEST_EXPECT(expected(loaded, junction, {0.0, 1.0, 2.0}), "loaded junction levels are not {0,1,2}");
  std::string legacy{};
  for (std::size_t line_begin = 0; line_begin < saved.size();) {
    const std::size_t line_end = saved.find('\n', line_begin);
    if (line_end == std::string::npos) {
      return false;
    }
    const std::string_view line(saved.data() + line_begin,
                                line_end - line_begin);
    if (line.find(".support_level=") == std::string_view::npos &&
        line.find(".support_group_id=") == std::string_view::npos) {
      legacy.append(line);
      legacy.push_back('\n');
    }
    line_begin = line_end + 1;
  }
  city::wire::CoreState legacy_loaded;
  const auto legacy_loaded_result = legacy_loaded.DeserializeAuthoritative(legacy);
  WIRE_TEST_EXPECT(legacy_loaded_result.ok, legacy_loaded_result.error);
  WIRE_TEST_EXPECT(expected(legacy_loaded, junction, {0.0, 1.0, 2.0}),
                   "legacy loaded junction levels are not {0,1,2}");
  std::string invariant_error{};
  WIRE_TEST_EXPECT(backbone_common_invariants_pass(state, &invariant_error), invariant_error);
  WIRE_TEST_EXPECT(backbone_common_invariants_pass(loaded, &invariant_error), invariant_error);
  WIRE_TEST_EXPECT(backbone_common_invariants_pass(legacy_loaded, &invariant_error), invariant_error);
  return true;
}

bool C800_backbone_row_continuity_graph_lint_covers_route_branch_and_cross() {
  {
    city::wire::CoreState state;
    const auto out = state.GenerateFromBackboneSpec(line_req(state));
    if (!out.ok || !row_continuity_graph_lint_passes(state)) return false;
    std::string invariant_error{};
    WIRE_TEST_EXPECT(backbone_common_invariants_pass(state, &invariant_error), invariant_error);
  }
  {
    city::wire::CoreState state;
    const auto out = state.GenerateFromBackboneSpec(hv_poly3_req(state));
    if (!out.ok || !row_continuity_graph_lint_passes(state)) return false;
    std::string invariant_error{};
    WIRE_TEST_EXPECT(backbone_common_invariants_pass(state, &invariant_error), invariant_error);
  }
  {
    city::wire::CoreState state;
    const auto spans = lowering_branch_spans(state);
    if (spans.empty() || !row_continuity_graph_lint_passes(state)) return false;
    std::string invariant_error{};
    WIRE_TEST_EXPECT(backbone_common_invariants_pass(state, &invariant_error), invariant_error);
  }
  {
    IncrementalCrossFixture cross{};
    if (!make_incremental_cross(&cross) || !cross.completion.ok ||
        !row_continuity_graph_lint_passes(cross.state)) {
      return false;
    }
    std::string invariant_error{};
    WIRE_TEST_EXPECT(backbone_common_invariants_pass(cross.state, &invariant_error), invariant_error);
  }
  return true;
}

bool C807_backbone_pipeline_does_not_infer_continuity_from_route_order() {
  std::string cpp{};
  if (!file_text(repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.cpp", &cpp)) {
    return false;
  }
  std::string regenerate{};
  std::string state{};
  std::string update{};
  if (!file_text(repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "regenerate.cpp", &regenerate) ||
      !file_text(repo_root() / "domains" / "wire" / "src" / "state" / "state.cpp", &state) ||
      !file_text(repo_root() / "domains" / "wire" / "src" / "state" / "template" / "update.cpp", &update)) {
    return false;
  }
  return !contains_text(cpp, "participates_in_route_order_continuity") &&
         !contains_text(cpp, "is_route_order_continuation") &&
         !contains_text(cpp, "route_continuation =") &&
         !contains_text(cpp, "left_link.route == right_link.route") &&
         !contains_text(cpp, "incoming->route == outgoing->route") &&
         !contains_text(cpp, "incoming->order + 1 == outgoing->order") &&
         !contains_text(cpp, "a.route == b.route") &&
         !contains_text(cpp, "a.route != 0 && b.route != 0") &&
         !contains_text(regenerate, "candidate.route != route_id") &&
         !contains_text(regenerate, "candidate.order == anchor.order + 1") &&
         !contains_text(regenerate, "edge->route != route_id") &&
         !contains_text(state, "candidate.route != seed.route") &&
         !contains_text(state, "candidate.order == anchor.order + 1") &&
         !contains_text(update, "scope.route == edge->route");
}

bool C814_backbone_same_operation_does_not_prefer_new_pair_over_existing_open() {
  city::wire::CoreState state;
  city::wire::BackboneSpec existing = line_req(state);
  existing.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}};
  const auto bd = state.GenerateFromBackboneSpec(existing);
  if (!bd.ok || bd.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const city::wire::ObjectId junction = bd.value.generated_pole_ids.front();
  const city::wire::Pole* pole = state.view().poles().find(junction);
  if (pole == nullptr) {
    return false;
  }

  city::wire::BackboneSpec two_edges = poly3_req(state);
  two_edges.path.polyline = {
      {-10.0, 0.0, 0.0}, pole->world_transform.position, {10.0, 0.0, 0.0}};
  two_edges.path.node_specs = {pole_spec(1, junction)};
  const auto added = state.GenerateFromBackboneSpec(two_edges);
  if (!added.ok || added.value.generated_pole_ids.size() != 2) {
    return false;
  }

  const JunctionRowSnapshot snapshot = junction_snapshot(state, junction);
  return snapshot.pair_rows == 0 && snapshot.open_rows == 3 &&
         curve_endpoints_match_layout(state);
}

bool C815_backbone_sharp_pair_height_is_operation_order_independent() {
  const city::wire::Vec3d d{12.0, -8.0, 0.0};
  const city::wire::Vec3d e{13.0, -10.0, 0.0};
  const auto hv_request = [](city::wire::CoreState& state) {
    city::wire::BackboneSpec request = line_req(state);
    request.bundles.clear();
    add_backbone_bundle(request, city::wire::BundleKind::kHighVoltage);
    return request;
  };

  city::wire::CoreState one_shot;
  const auto one_base =
      one_shot.GenerateFromBackboneSpec(hv_poly3_req(one_shot));
  WIRE_TEST_EXPECT(one_base.ok, one_base.error);
  WIRE_TEST_EXPECT(one_base.value.generated_pole_ids.size() == 3, "one-shot base did not generate 3 poles");
  const city::wire::ObjectId one_b = one_base.value.generated_pole_ids[1];
  const city::wire::Pole* one_pole = one_shot.view().poles().find(one_b);
  WIRE_TEST_EXPECT(one_pole != nullptr, "one-shot junction pole is missing");
  city::wire::BackboneSpec one_pair = hv_request(one_shot);
  one_pair.path.polyline = {d, one_pole->world_transform.position, e};
  one_pair.path.node_specs = {pole_spec(1, one_b)};
  const auto one_added = one_shot.GenerateFromBackboneSpec(one_pair);
  WIRE_TEST_EXPECT(one_added.ok, one_added.error);
  WIRE_TEST_EXPECT(one_added.value.generated_pole_ids.size() == 2, "one-shot sharp pair did not generate 2 poles");

  city::wire::CoreState incremental;
  const auto incremental_base =
      incremental.GenerateFromBackboneSpec(hv_poly3_req(incremental));
  WIRE_TEST_EXPECT(incremental_base.ok, incremental_base.error);
  WIRE_TEST_EXPECT(incremental_base.value.generated_pole_ids.size() == 3,
                   "incremental base did not generate 3 poles");
  const city::wire::ObjectId incremental_b =
      incremental_base.value.generated_pole_ids[1];
  const city::wire::Pole* incremental_pole =
      incremental.view().poles().find(incremental_b);
  WIRE_TEST_EXPECT(incremental_pole != nullptr, "incremental junction pole is missing");
  city::wire::BackboneSpec first = hv_request(incremental);
  first.path.polyline = {incremental_pole->world_transform.position, d};
  first.path.node_specs = {pole_spec(0, incremental_b)};
  const auto first_added = incremental.GenerateFromBackboneSpec(first);
  WIRE_TEST_EXPECT(first_added.ok, first_added.error);
  WIRE_TEST_EXPECT(first_added.value.generated_pole_ids.size() == 1,
                   "incremental first sharp edge did not generate 1 pole");
  city::wire::BackboneSpec second = hv_request(incremental);
  second.path.polyline = {e, incremental_pole->world_transform.position};
  second.path.node_specs = {pole_spec(1, incremental_b)};
  const auto second_added = incremental.GenerateFromBackboneSpec(second);
  WIRE_TEST_EXPECT(second_added.ok, second_added.error);
  WIRE_TEST_EXPECT(second_added.value.generated_pole_ids.size() == 1,
                   "incremental second sharp edge did not generate 1 pole");

  const auto one_levels = hv_endpoint_levels_at_pole(one_shot, one_b);
  const auto incremental_levels =
      hv_endpoint_levels_at_pole(incremental, incremental_b);
  const double step = std::max(
      0.0, -one_shot.view()
                .bundle_templates()
                .at(city::wire::DefaultBundleTemplateId(
                    city::wire::BundleKind::kHighVoltage))
                .branch_endpoint_offset_m);
  const std::vector<std::pair<int, double>> expected{
      {0, 0.0}, {0, 0.0}, {1, step}, {2, step * 2.0}};
  WIRE_TEST_EXPECT(step > 0.0, "HV branch-down step is not positive");
  WIRE_TEST_EXPECT(one_levels == expected, "one-shot sharp endpoint levels do not match expected");
  WIRE_TEST_EXPECT(incremental_levels == expected, "incremental sharp endpoint levels do not match expected");
  WIRE_TEST_EXPECT(one_levels == incremental_levels, "one-shot and incremental sharp endpoint levels differ");
  WIRE_TEST_EXPECT(curve_endpoints_match_layout(one_shot), "one-shot curve endpoints do not match layout");
  WIRE_TEST_EXPECT(curve_endpoints_match_layout(incremental), "incremental curve endpoints do not match layout");
  std::string invariant_error{};
  WIRE_TEST_EXPECT(backbone_common_invariants_pass(one_shot, &invariant_error), invariant_error);
  WIRE_TEST_EXPECT(backbone_common_invariants_pass(incremental, &invariant_error), invariant_error);
  return true;
}

bool C816_backbone_incremental_normal_pair_keeps_allocated_support_level() {
  constexpr city::wire::ModelAssemblyTemplateId kRowAssembly = 9816;
  city::wire::CoreState state;
  city::wire::ModelAssemblyTemplate row_assembly{};
  row_assembly.id = kRowAssembly;
  city::wire::ModelAssemblyPart row_part{};
  row_part.part_id = 1;
  row_part.model_key = "c816_crossarm";
  row_part.descriptor_version = 1;
  row_part.fit_mode = city::wire::ModelFitMode::kRigid;
  row_assembly.parts.push_back(row_part);
  if (!state.RegisterModelAssemblyTemplate(row_assembly).ok) {
    return false;
  }
  const city::wire::BundleTemplate& hv =
      state.view().bundle_templates().at(
          city::wire::DefaultBundleTemplateId(
              city::wire::BundleKind::kHighVoltage));
  if (!hv.enable_branch_down_offset || hv.branch_endpoint_offset_m == 0.0) {
    return false;
  }
  city::wire::BundleTemplate hv_with_model = hv;
  hv_with_model.row_fixture_assembly_id = kRowAssembly;
  if (!state.UpdateBundleTemplate(hv_with_model).ok) {
    return false;
  }
  const auto use_explicit_hv_placement =
      [](city::wire::BackboneSpec* request) {
        if (request == nullptr || request->bundles.size() != 1) {
          return false;
        }
        city::wire::BackboneBundleSpec& bundle = request->bundles.front();
        bundle.placement_key = 1;
        bundle.placement_explicit = true;
        bundle.count = 3;
        bundle.height_m = 9.2;
        bundle.lateral_m = -0.2;
        bundle.spacing_m = 0.45;
        return true;
      };
  const auto add_hv_edge =
      [&](city::wire::ObjectId junction, const city::wire::Vec3d& point,
          bool junction_at_end) -> city::wire::ObjectId {
    const city::wire::Pole* pole = state.view().poles().find(junction);
    if (pole == nullptr) {
      return city::wire::kInvalidObjectId;
    }
    city::wire::BackboneSpec request = line_req(state);
    request.bundles.clear();
    add_backbone_bundle(request, city::wire::BundleKind::kHighVoltage);
    if (!use_explicit_hv_placement(&request)) {
      return city::wire::kInvalidObjectId;
    }
    request.path.polyline =
        junction_at_end
            ? std::vector<city::wire::Vec3d>{point,
                                             pole->world_transform.position}
            : std::vector<city::wire::Vec3d>{pole->world_transform.position,
                                             point};
    request.path.node_specs = {pole_spec(junction_at_end ? 1 : 0, junction)};
    const auto result = state.GenerateFromBackboneSpec(request);
    return result.ok && result.value.generated_pole_ids.size() == 1
               ? result.value.generated_pole_ids.front()
               : city::wire::kInvalidObjectId;
  };

  city::wire::BackboneSpec base_request = hv_poly3_req(state);
  if (!use_explicit_hv_placement(&base_request)) {
    return false;
  }
  const auto base = state.GenerateFromBackboneSpec(base_request);
  if (!base.ok || base.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId junction = base.value.generated_pole_ids[1];
  const city::wire::SavedBackboneNode* node_b =
      state.view().backbone_node_for_pole(junction);
  if (node_b == nullptr) {
    return false;
  }
  const city::wire::ObjectId pole_d =
      add_hv_edge(junction, {20.0, -8.0, 0.0}, false);
  const city::wire::ObjectId pole_e =
      add_hv_edge(junction, {4.0, 8.0, 0.0}, true);
  const city::wire::ObjectId pole_f =
      add_hv_edge(junction, {20.0, 8.0, 0.0}, false);
  const city::wire::ObjectId pole_g =
      add_hv_edge(junction, {4.0, -8.0, 0.0}, true);
  if (pole_d == city::wire::kInvalidObjectId ||
      pole_e == city::wire::kInvalidObjectId ||
      pole_f == city::wire::kInvalidObjectId ||
      pole_g == city::wire::kInvalidObjectId) {
    return false;
  }
  const auto edge_to = [&](city::wire::ObjectId pole) {
    const city::wire::SavedBackboneNode* current_b =
        state.view().backbone_node_for_pole(junction);
    const city::wire::SavedBackboneNode* node =
        state.view().backbone_node_for_pole(pole);
    return current_b == nullptr || node == nullptr
               ? city::wire::kInvalidObjectId
               : edge_between(state, current_b->node_id, node->node_id);
  };
  const city::wire::ObjectId bd = edge_to(pole_d);
  const city::wire::ObjectId be = edge_to(pole_e);
  const city::wire::ObjectId bf = edge_to(pole_f);
  const city::wire::ObjectId bg = edge_to(pole_g);
  if (bd == city::wire::kInvalidObjectId ||
      be == city::wire::kInvalidObjectId ||
      bf == city::wire::kInvalidObjectId ||
      bg == city::wire::kInvalidObjectId) {
    return false;
  }
  const city::wire::SavedBackbonePortBinding* bd_binding =
      hv_lane0_binding_for_edge_at_pole(state, junction, bd);
  const city::wire::SavedBackbonePortBinding* be_binding =
      hv_lane0_binding_for_edge_at_pole(state, junction, be);
  const city::wire::SavedBackbonePortBinding* bf_binding =
      hv_lane0_binding_for_edge_at_pole(state, junction, bf);
  if (bd_binding == nullptr || be_binding == nullptr ||
      bf_binding == nullptr) {
    return false;
  }
  const city::wire::SavedBackboneEdgeBundle* bd_edge_bundle =
      state.view().backbone_edge_bundle(bd_binding->edge_bundle_id);
  const city::wire::Bundle* bd_bundle =
      bd_edge_bundle == nullptr
          ? nullptr
          : state.view().bundles().find(bd_edge_bundle->bundle_id);
  const std::optional<double> bd_height_before =
      local_port_height_for_binding(state, *bd_binding);
  const std::optional<double> be_height_before =
      local_port_height_for_binding(state, *be_binding);
  const std::optional<double> bf_height_before =
      local_port_height_for_binding(state, *bf_binding);
  if (bd_bundle == nullptr || !bd_height_before.has_value() ||
      !be_height_before.has_value() || !bf_height_before.has_value()) {
    return false;
  }
  const std::vector<double> offsets =
      hv_row_down_offsets_at_pole(state, junction);
  std::vector<double> row_model_z{};
  for (const city::wire::VisualModelInstance& instance :
       state.view().visual_model_instances().instances) {
    if (instance.model_key == "c816_crossarm" &&
        instance.stable_key.find("row:" + std::to_string(junction) + ":") ==
            0) {
      row_model_z.push_back(instance.world_transform.position.z);
    }
  }
  std::sort(row_model_z.begin(), row_model_z.end());
  const bool levels_ok =
      offsets.size() == 3 && row_model_z.size() == 3 &&
      !almost_equal(row_model_z[0], row_model_z[1], 1e-9) &&
      !almost_equal(row_model_z[1], row_model_z[2], 1e-9) &&
      hv_lane0_support_level_for_edge_at_pole(state, junction, bd) == 1 &&
      hv_lane0_support_level_for_edge_at_pole(state, junction, be) == 1 &&
      hv_lane0_support_level_for_edge_at_pole(state, junction, bf) == 2 &&
      hv_lane0_support_level_for_edge_at_pole(state, junction, bg) == 2 &&
      curve_endpoints_match_layout(state);
  if (!levels_ok) {
    return false;
  }
  const double delta = 0.4;
  const city::wire::ObjectId updated_bundle_id = bd_bundle->id;
  const auto updated = state.UpdateBackboneBundlePlacement(
      updated_bundle_id, true, bd_bundle->height_m + delta,
      bd_bundle->lateral_m, bd_bundle->phase_spacing_m);
  if (!updated.ok) {
    return false;
  }
  bd_binding = hv_lane0_binding_for_edge_at_pole(state, junction, bd);
  be_binding = hv_lane0_binding_for_edge_at_pole(state, junction, be);
  bf_binding = hv_lane0_binding_for_edge_at_pole(state, junction, bf);
  if (bd_binding == nullptr || be_binding == nullptr ||
      bf_binding == nullptr) {
    return false;
  }
  const std::optional<double> bd_height_after =
      local_port_height_for_binding(state, *bd_binding);
  const std::optional<double> be_height_after =
      local_port_height_for_binding(state, *be_binding);
  const std::optional<double> bf_height_after =
      local_port_height_for_binding(state, *bf_binding);
  return bd_height_after.has_value() && be_height_after.has_value() &&
         bf_height_after.has_value() &&
         almost_equal(*bd_height_after, *bd_height_before + delta, 1e-9) &&
         almost_equal(*be_height_after, *be_height_before + delta, 1e-9) &&
         almost_equal(*bf_height_after, *bf_height_before, 1e-9) &&
         curve_endpoints_match_layout(state);
}

bool C835_backbone_sharp_route_corner_uses_branch_down_level() {
  auto explicit_bundle = [](const city::wire::CoreState& state,
                            city::wire::BundleKind kind,
                            std::uint64_t placement_key,
                            int count,
                            double height_m,
                            double lateral_m,
                            double spacing_m) {
    const city::wire::BundleTemplateId template_id =
        city::wire::DefaultBundleTemplateId(kind);
    const city::wire::BundleTemplate& tmpl =
        state.view().bundle_templates().at(template_id);
    city::wire::BackboneBundleSpec bundle{};
    bundle.bundle_template_id = template_id;
    bundle.placement_key = placement_key;
    bundle.layer = tmpl.default_layer;
    bundle.count = count;
    bundle.placement_explicit = true;
    bundle.height_m = height_m;
    bundle.lateral_m = lateral_m;
    bundle.spacing_m = spacing_m;
    return bundle;
  };
  auto viewer_default_repro_req = [&](city::wire::CoreState& state,
                                      bool polyline3) {
    city::wire::BackboneSpec req = polyline3 ? poly3_req(state) : line_req(state);
    req.pole_type_id = 2;
    req.pole_placement.enable_tilt = true;
    req.pole_placement.max_tilt_deg = 12.0;
    if (polyline3) {
      req.path.polyline = {{21.390, 15.503, 0.0},
                           {-1.406, 9.899, 0.0},
                           {25.824, 3.610, 0.0}};
    }
    req.bundles.clear();
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kHighVoltage, 1, 3, 9.2, -0.2, 0.45));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kLowVoltage, 2, 1, 7.7, 0.0, 0.2));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kLowVoltage, 3, 1, 7.35, 0.0, 0.2));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kLowVoltage, 4, 1, 7.0, 0.0, 0.2));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kCommunication, 5, 1, 5.5, 0.0, 0.2));
    req.bundles.push_back(
        explicit_bundle(state, city::wire::BundleKind::kOptical, 6, 1, 5.3, 0.0, 0.2));
    return req;
  };
  auto repro_points = []() {
    return std::array<city::wire::Vec3d, 3>{
        city::wire::Vec3d{21.390, 15.503, 0.0},
        city::wire::Vec3d{-1.406, 9.899, 0.0},
        city::wire::Vec3d{25.824, 3.610, 0.0}};
  };
  struct SharpRowEndpoint {
    city::wire::ObjectId edge_id = city::wire::kInvalidObjectId;
    std::size_t lane = 0;
    int support_level = -1;
    int support_group_id = -2;
    double down_offset_m = 0.0;
    city::wire::BackboneFlowKind flow_kind = city::wire::BackboneFlowKind::kBranch;
    city::wire::Vec3d endpoint_world{};
  };
  auto sharp_row_endpoints =
      [&](const city::wire::CoreState& state,
          city::wire::ObjectId junction,
          city::wire::BundleTemplateId template_id) {
    std::vector<SharpRowEndpoint> rows{};
    const city::wire::SavedBackboneNode* node =
        state.view().backbone_node_for_pole(junction);
    if (node == nullptr) {
      return rows;
    }
    for (const city::wire::SavedBackbonePortBinding& binding :
         state.view().backbone().port_bindings) {
      if (binding.row_key.node_id != node->node_id ||
          binding.bundle_template_id != template_id) {
        continue;
      }
      const city::wire::SavedBackboneEdgeBundle* edge_bundle =
          state.view().backbone_edge_bundle(binding.edge_bundle_id);
      if (edge_bundle == nullptr) {
        continue;
      }
      const city::wire::LayoutEndpoint* matched = nullptr;
      const city::wire::SpanLayoutEntry* matched_layout = nullptr;
      for (const city::wire::Span& span : state.view().spans().items()) {
        const city::wire::SpanLayoutView layout = state.span_layout(span.id);
        if (!layout.has_layout()) {
          continue;
        }
        if (layout.entry->start.port_id == binding.port_id) {
          matched = &layout.entry->start;
          matched_layout = layout.entry;
        } else if (layout.entry->end.port_id == binding.port_id) {
          matched = &layout.entry->end;
          matched_layout = layout.entry;
        }
      }
      if (matched == nullptr || matched_layout == nullptr) {
        continue;
      }
      rows.push_back({edge_bundle->edge_id, binding.lane_index,
                      binding.support_level, binding.support_group_id,
                      matched->branch_down_offset_m,
                      matched_layout->flow_kind,
                      matched->endpoint_world});
    }
    std::sort(rows.begin(), rows.end(), [](const SharpRowEndpoint& a,
                                           const SharpRowEndpoint& b) {
      return std::tie(a.edge_id, a.lane) < std::tie(b.edge_id, b.lane);
    });
    return rows;
  };
  auto expect_sharp_hv_stagger =
      [&](const city::wire::CoreState& state,
          city::wire::ObjectId junction,
          const char* label) {
    const double hv_step = std::max(
        0.0, -state.view()
                  .bundle_templates()
                  .at(city::wire::DefaultBundleTemplateId(
                      city::wire::BundleKind::kHighVoltage))
                  .branch_endpoint_offset_m);
    WIRE_TEST_EXPECT(hv_step > 0.0,
                     std::string(label) + ": HV branch-down step is not positive");
    const auto hv_rows = sharp_row_endpoints(
        state, junction,
        city::wire::DefaultBundleTemplateId(
            city::wire::BundleKind::kHighVoltage));
    WIRE_TEST_EXPECT(hv_rows.size() == 6,
                     std::string(label) + ": expected 6 HV row endpoints");
    std::map<city::wire::ObjectId, int> edge_level{};
    std::map<city::wire::ObjectId, int> edge_group{};
    std::map<city::wire::ObjectId, double> edge_down{};
    for (const SharpRowEndpoint& row : hv_rows) {
      WIRE_TEST_EXPECT(row.flow_kind == city::wire::BackboneFlowKind::kMain,
                       std::string(label) + ": sharp route row was classified as branch flow");
      const auto [level_it, level_inserted] =
          edge_level.emplace(row.edge_id, row.support_level);
      WIRE_TEST_EXPECT(level_inserted || level_it->second == row.support_level,
                       std::string(label) + ": HV lanes disagree on edge support level");
      const auto [group_it, group_inserted] =
          edge_group.emplace(row.edge_id, row.support_group_id);
      WIRE_TEST_EXPECT(group_inserted || group_it->second == row.support_group_id,
                       std::string(label) + ": HV lanes disagree on edge support group");
      const auto [down_it, down_inserted] =
          edge_down.emplace(row.edge_id, row.down_offset_m);
      WIRE_TEST_EXPECT(down_inserted ||
                           almost_equal(down_it->second, row.down_offset_m,
                                        1e-9),
                       std::string(label) + ": HV lanes disagree on edge down offset");
    }
    WIRE_TEST_EXPECT(edge_level.size() == 2,
                     std::string(label) + ": sharp pair did not produce two physical rows");
    std::vector<int> levels{};
    std::vector<int> groups{};
    std::vector<double> downs{};
    for (const auto& [edge_id, level] : edge_level) {
      levels.push_back(level);
      groups.push_back(edge_group[edge_id]);
      downs.push_back(edge_down[edge_id]);
    }
    std::sort(levels.begin(), levels.end());
    std::sort(groups.begin(), groups.end());
    std::sort(downs.begin(), downs.end());
    WIRE_TEST_EXPECT(levels == std::vector<int>({0, 1}),
                     std::string(label) + ": isolated sharp pair levels are not {0,1}");
    WIRE_TEST_EXPECT(groups.size() == 2 && groups[0] >= 0 &&
                         groups[1] >= 0 && groups[0] != groups[1],
                     std::string(label) + ": sharp physical rows do not have distinct support groups");
    WIRE_TEST_EXPECT(almost_equal(downs[0], 0.0, 1e-9) &&
                         almost_equal(downs[1], hv_step, 1e-9),
                     std::string(label) + ": sharp pair down offsets are not {0, step}");
    std::size_t non_hv_lowered = 0;
    for (const city::wire::Span& span : state.view().spans().items()) {
      const city::wire::Bundle* bundle =
          state.view().bundles().find(span.bundle_id);
      const city::wire::SpanLayoutEntry* layout = state.span_layout(span.id).entry;
      WIRE_TEST_EXPECT(bundle != nullptr && layout != nullptr,
                       std::string(label) + ": span bundle or layout is missing");
      for (const city::wire::LayoutEndpoint* endpoint :
           {&layout->start, &layout->end}) {
        const city::wire::Port* port =
            state.view().ports().find(endpoint->port_id);
        if (port == nullptr || port->owner_pole_id != junction) {
          continue;
        }
        if (bundle->bundle_template_id !=
                city::wire::DefaultBundleTemplateId(
                    city::wire::BundleKind::kHighVoltage) &&
            (endpoint->default_lower_required ||
                   endpoint->lower_required ||
                   endpoint->branch_down_offset_m > 1e-9)) {
          ++non_hv_lowered;
        }
      }
    }
    WIRE_TEST_EXPECT(non_hv_lowered == 0,
                     std::string(label) + ": non-HV sharp route endpoints were lowered");
    WIRE_TEST_EXPECT(curve_endpoints_match_layout(state),
                     std::string(label) + ": curve endpoints do not match layout");
    return true;
  };

  city::wire::CoreState one_shot;
  const auto one_points = repro_points();
  const auto one_generated =
      one_shot.GenerateFromBackboneSpec(viewer_default_repro_req(one_shot, true));
  WIRE_TEST_EXPECT(one_generated.ok, one_generated.error);
  WIRE_TEST_EXPECT(one_generated.value.generated_pole_ids.size() == 3,
                   "viewer repro sharp route did not generate 3 poles");
  WIRE_TEST_EXPECT(one_generated.value.generated_span_ids.size() == 16,
                   "viewer repro sharp route did not generate 16 spans");
  if (!expect_sharp_hv_stagger(one_shot, one_generated.value.generated_pole_ids[1], "one-shot")) {
    return false;
  }

  city::wire::CoreState incremental;
  const auto inc_points = repro_points();
  city::wire::BackboneSpec first = viewer_default_repro_req(incremental, false);
  first.path.polyline = {inc_points[0], inc_points[1]};
  const auto first_out = incremental.GenerateFromBackboneSpec(first);
  WIRE_TEST_EXPECT(first_out.ok, first_out.error);
  WIRE_TEST_EXPECT(first_out.value.generated_pole_ids.size() == 2,
                   "first sharp edge did not generate 2 poles");
  const city::wire::ObjectId junction = first_out.value.generated_pole_ids[1];
  const city::wire::Pole* junction_pole = incremental.view().poles().find(junction);
  WIRE_TEST_EXPECT(junction_pole != nullptr, "incremental junction pole is missing");
  city::wire::BackboneSpec second = viewer_default_repro_req(incremental, false);
  second.path.polyline = {junction_pole->world_transform.position, inc_points[2]};
  second.path.node_specs = {pole_spec(0, junction)};
  const auto second_out = incremental.GenerateFromBackboneSpec(second);
  WIRE_TEST_EXPECT(second_out.ok, second_out.error);
  WIRE_TEST_EXPECT(second_out.value.generated_pole_ids.size() == 1,
                   "second sharp edge did not generate one pole");
  if (!expect_sharp_hv_stagger(incremental, junction, "incremental")) {
    return false;
  }
  return true;
}

} // namespace backbone_tests
