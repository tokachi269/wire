#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_view.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

using namespace helpers;

namespace backbone_tests {

bool C422_backbone_rules_consume_topo_and_groups() {
  const std::filesystem::path header = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.hpp";
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
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
  return contains_text(body, "group_for") && contains_text(body, "ps.jumpers") &&
         !contains_text(body, "ps.links");
}

bool C423_backbone_tspan_carries_endpoint_rows() {
  const std::filesystem::path header = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.hpp";
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
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

bool C425_backbone_edge_carries_multiple_spans() {
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

bool C426_backbone_existing_pole_resolves_graph_node() {
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

bool C427_backbone_graph_index_links_outputs() {
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

bool C428_backbone_pole_frontier_collects_incident_graph() {
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

bool C429_backbone_span_frontier_collects_edge_bundle_spans() {
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

bool C430_backbone_frontier_uses_saved_graph_index() {
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
         !contains_text(body, "SavedBackboneEdges") && !contains_text(body, "spans_by_port");
}

bool C431_backbone_edge_bundle_is_saved_backbone_unit() {
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

bool C432_backbone_multiple_bundles_create_multiple_edge_bundles() {
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

bool C433_backbone_resolves_edge_for_same_poles() {
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

bool C434_backbone_reverse_duplicate_same_bundle_rejected() {
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

bool C435_backbone_edge_metadata_is_not_overwritten_on_duplicate_reject() {
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

bool C436_backbone_frontier_reads_edge_bundles() {
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

bool C437_backbone_layout_save_is_direct() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "state" / "span_runtime.cpp";
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
  }
  return true;
}

bool C441_backbone_save_backbone_edge_returns_saved_ref() {
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

bool C442_backbone_edge_forward_uses_saved_ref() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  std::string body;
  if (!function_body(cpp, "EditResult<bool> pipeline::save_graph(const topo& made, const pairs& ps)", &body)) {
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
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "backbone";
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
  const std::filesystem::path header = repo_root() / "core" / "include" / "wire" / "core" / "core_state.hpp";
  const std::filesystem::path source = repo_root() / "core" / "src" / "state" / "span_runtime.cpp";
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

bool C448_backbone_tests_use_neutral_layout_read() {
  const std::filesystem::path source = repo_root() / "core" / "tests" / "backbone";
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
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const wire::core::SpanLayoutState state_view = state.span_layout_state(span_id);
    if (!state_view.has_rules || !state_view.has_layout) {
      return false;
    }
  }
  return true;
}

bool C451_backbone_tests_do_not_read_old_contract() {
  const std::filesystem::path source = repo_root() / "core" / "tests" / "backbone";
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
  const std::filesystem::path source = repo_root() / "core" / "src" / "state" / "span_runtime.cpp";
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
  const std::filesystem::path header = repo_root() / "core" / "include" / "wire" / "core" / "core_runtime_types.hpp";
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
  const std::filesystem::path source = repo_root() / "core" / "src" / "state" / "span_runtime.cpp";
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

bool C459_backbone_existing_cross_DBE_on_ABC() {
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

namespace {

struct JunctionRowSnapshot {
  std::vector<std::tuple<bool, wire::core::ObjectId, wire::core::ObjectId>> row_keys{};
  std::vector<wire::core::Vec3d> port_positions{};
  std::vector<std::pair<wire::core::ObjectId, wire::core::ObjectId>> span_ports{};
  std::vector<std::vector<wire::core::ObjectId>> node_patch_edges{};
  std::size_t pair_rows = 0;
  std::size_t open_rows = 0;
  std::size_t support_levels = 0;
  std::size_t row_fixture_instances = 0;
  std::size_t endpoint_fixture_instances = 0;
};

wire::core::ObjectId edge_between(const wire::core::CoreState& state,
                                  wire::core::ObjectId a,
                                  wire::core::ObjectId b) {
  const wire::core::BackboneEdgeKey key{std::min(a, b), std::max(a, b)};
  const auto it = state.view().backbone_index().edge_by_nodes.find(key);
  return it == state.view().backbone_index().edge_by_nodes.end() ? wire::core::kInvalidObjectId : it->second;
}

bool has_row_key(const std::vector<std::tuple<bool, wire::core::ObjectId, wire::core::ObjectId>>& rows,
                 bool is_open,
                 wire::core::ObjectId a,
                 wire::core::ObjectId b) {
  const wire::core::ObjectId lo = is_open ? a : std::min(a, b);
  const wire::core::ObjectId hi = is_open ? b : std::max(a, b);
  return std::find(rows.begin(), rows.end(), std::make_tuple(is_open, lo, hi)) != rows.end();
}

std::vector<wire::core::ObjectId> unique_generated_ports_on_pole(
    const wire::core::CoreState& state,
    const std::vector<wire::core::ObjectId>& spans,
    wire::core::ObjectId pole_id) {
  std::vector<wire::core::ObjectId> out{};
  for (wire::core::ObjectId span_id : spans) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) continue;
    for (wire::core::ObjectId port_id : {span->port_a_id, span->port_b_id}) {
      const wire::core::Port* port = state.view().ports().find(port_id);
      if (port != nullptr && port->owner_pole_id == pole_id &&
          std::find(out.begin(), out.end(), port_id) == out.end()) {
        out.push_back(port_id);
      }
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

bool curve_endpoints_match_layout(const wire::core::CoreState& state) {
  for (const wire::core::Span& span : state.view().spans().items()) {
    const wire::core::SpanLayoutEntry* layout = state.span_layout(span.id).entry;
    const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span.id);
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

JunctionRowSnapshot junction_snapshot(const wire::core::CoreState& state,
                                      wire::core::ObjectId pole_id) {
  JunctionRowSnapshot out{};
  const wire::core::SavedBackboneNode* node = state.view().backbone_node_for_pole(pole_id);
  if (node == nullptr) {
    return out;
  }
  std::vector<double> support_levels{};
  for (const wire::core::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != node->node_id) {
      continue;
    }
    const wire::core::ObjectId a = binding.row_key.source_is_open
                                       ? binding.row_key.source_edge_a
                                       : std::min(binding.row_key.source_edge_a, binding.row_key.source_edge_b);
    const wire::core::ObjectId b = binding.row_key.source_is_open
                                       ? binding.row_key.source_edge_b
                                       : std::max(binding.row_key.source_edge_a, binding.row_key.source_edge_b);
    const auto key = std::make_tuple(binding.row_key.source_is_open, a, b);
    if (std::find(out.row_keys.begin(), out.row_keys.end(), key) == out.row_keys.end()) {
      out.row_keys.push_back(key);
      if (binding.row_key.source_is_open) {
        ++out.open_rows;
      } else {
        ++out.pair_rows;
      }
    }
    const wire::core::Port* port = state.view().ports().find(binding.port_id);
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
  for (const wire::core::Span& span : state.view().spans().items()) {
    const wire::core::Port* a = state.view().ports().find(span.port_a_id);
    const wire::core::Port* b = state.view().ports().find(span.port_b_id);
    if ((a != nullptr && a->owner_pole_id == pole_id) || (b != nullptr && b->owner_pole_id == pole_id)) {
      out.span_ports.push_back({span.port_a_id, span.port_b_id});
    }
  }
  std::sort(out.span_ports.begin(), out.span_ports.end());
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == wire::core::VisualCurvePartKind::kNodePatch && part.source_node_id == node->node_id) {
      out.node_patch_edges.push_back(part.incident_edge_ids);
    }
  }
  std::sort(out.node_patch_edges.begin(), out.node_patch_edges.end());
  for (const wire::core::VisualModelInstance& instance : state.view().visual_model_instances().instances) {
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

struct IncrementalCrossFixture {
  wire::core::CoreState state{};
  wire::core::ObjectId pole_b = wire::core::kInvalidObjectId;
  wire::core::ObjectId pole_d = wire::core::kInvalidObjectId;
  wire::core::ObjectId pole_e = wire::core::kInvalidObjectId;
  std::vector<wire::core::ObjectId> bd_spans{};
  std::vector<wire::core::ObjectId> bd_ports{};
  wire::core::EditResult<wire::core::GenerateBundleFromPathResult> completion{};
};

bool make_incremental_cross(IncrementalCrossFixture* out, bool reverse_completion = true,
                            wire::core::BundleKind completion_kind = wire::core::BundleKind::kLowVoltage,
                            wire::core::Vec3d d = {12.0, -8.0, 0.0},
                            wire::core::Vec3d e = {20.0, 0.0, 0.0}) {
  if (out == nullptr) {
    return false;
  }
  out->state = wire::core::CoreState{};
  const auto abc = out->state.GenerateFromBackboneSpec(poly3_req(out->state));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  out->pole_b = abc.value.generated_pole_ids[1];
  const wire::core::Pole* pole_b = out->state.view().poles().find(out->pole_b);
  if (pole_b == nullptr) return false;

  wire::core::BackboneSpec bd = line_req(out->state);
  bd.path.polyline = {pole_b->world_transform.position, d};
  bd.path.node_specs = {pole_spec(0, out->pole_b)};
  const auto bd_out = out->state.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_pole_ids.size() != 1 || bd_out.value.generated_span_ids.empty()) {
    return false;
  }
  out->pole_d = bd_out.value.generated_pole_ids.front();
  out->bd_spans = bd_out.value.generated_span_ids;
  out->bd_ports = unique_generated_ports_on_pole(out->state, out->bd_spans, out->pole_b);

  wire::core::BackboneSpec completion = line_req(out->state);
  if (completion_kind != wire::core::BundleKind::kLowVoltage) {
    completion.bundles.clear();
    add_backbone_bundle(completion, completion_kind);
  }
  completion.path.polyline = reverse_completion
                                 ? std::vector<wire::core::Vec3d>{e, pole_b->world_transform.position}
                                 : std::vector<wire::core::Vec3d>{pole_b->world_transform.position, e};
  completion.path.node_specs = {pole_spec(reverse_completion ? 1 : 0, out->pole_b)};
  out->completion = out->state.GenerateFromBackboneSpec(completion);
  if (!out->completion.ok || out->completion.value.generated_pole_ids.size() != 1) {
    return out->completion.ok;
  }
  out->pole_e = out->completion.value.generated_pole_ids.front();
  return true;
}

bool canonical_cross_at_b(const IncrementalCrossFixture& fixture) {
  const wire::core::SavedBackboneNode* node_b = fixture.state.view().backbone_node_for_pole(fixture.pole_b);
  const wire::core::SavedBackboneNode* node_d = fixture.state.view().backbone_node_for_pole(fixture.pole_d);
  const wire::core::SavedBackboneNode* node_e = fixture.state.view().backbone_node_for_pole(fixture.pole_e);
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr) return false;
  const wire::core::ObjectId bd = edge_between(fixture.state, node_b->node_id, node_d->node_id);
  const wire::core::ObjectId be = edge_between(fixture.state, node_b->node_id, node_e->node_id);
  const JunctionRowSnapshot snapshot = junction_snapshot(fixture.state, fixture.pole_b);
  return bd != wire::core::kInvalidObjectId && be != wire::core::kInvalidObjectId &&
         snapshot.pair_rows == 2 && snapshot.open_rows == 0 &&
         has_row_key(snapshot.row_keys, false, bd, be);
}

} // namespace

bool C771_backbone_incremental_cross_completion_matches_one_shot_rows() {
  wire::core::CoreState one_shot;
  const auto one_abc = one_shot.GenerateFromBackboneSpec(poly3_req(one_shot));
  if (!one_abc.ok || one_abc.value.generated_pole_ids.size() != 3) return false;
  const wire::core::ObjectId one_b = one_abc.value.generated_pole_ids[1];
  const wire::core::Pole* one_pole_b = one_shot.view().poles().find(one_b);
  if (one_pole_b == nullptr || one_shot.view().backbone_node_for_pole(one_b) == nullptr) return false;
  wire::core::BackboneSpec one_cross = line_req(one_shot);
  one_cross.path.polyline = {{12.0, -8.0, 0.0}, one_pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  one_cross.path.node_specs = {pole_spec(1, one_b)};
  const auto one_out = one_shot.GenerateFromBackboneSpec(one_cross);
  if (!one_out.ok) return false;
  const wire::core::ObjectId one_d = one_out.value.generated_pole_ids.front();
  const wire::core::ObjectId one_e = one_out.value.generated_pole_ids.back();
  const wire::core::SavedBackboneNode* one_node_b = one_shot.view().backbone_node_for_pole(one_b);
  const wire::core::SavedBackboneNode* one_node_d = one_shot.view().backbone_node_for_pole(one_d);
  const wire::core::SavedBackboneNode* one_node_e = one_shot.view().backbone_node_for_pole(one_e);
  if (one_node_b == nullptr || one_node_d == nullptr || one_node_e == nullptr) return false;
  const wire::core::ObjectId one_bd = edge_between(one_shot, one_node_b->node_id, one_node_d->node_id);
  const wire::core::ObjectId one_be = edge_between(one_shot, one_node_b->node_id, one_node_e->node_id);
  const JunctionRowSnapshot expected = junction_snapshot(one_shot, one_b);
  if (one_bd == wire::core::kInvalidObjectId || one_be == wire::core::kInvalidObjectId ||
      expected.pair_rows != 2 || expected.open_rows != 0 || expected.support_levels != 2 ||
      !has_row_key(expected.row_keys, false, one_bd, one_be) || !curve_endpoints_match_layout(one_shot)) {
    return false;
  }

  wire::core::CoreState incremental;
  const auto inc_abc = incremental.GenerateFromBackboneSpec(poly3_req(incremental));
  if (!inc_abc.ok || inc_abc.value.generated_pole_ids.size() != 3) return false;
  const wire::core::ObjectId inc_b = inc_abc.value.generated_pole_ids[1];
  const wire::core::Pole* inc_pole_b = incremental.view().poles().find(inc_b);
  if (inc_pole_b == nullptr || incremental.view().backbone_node_for_pole(inc_b) == nullptr) return false;

  wire::core::BackboneSpec bd = line_req(incremental);
  bd.path.polyline = {inc_pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd.path.node_specs = {pole_spec(0, inc_b)};
  const auto bd_out = incremental.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_pole_ids.size() != 1 ||
      bd_out.value.generated_span_ids.empty()) return false;
  const wire::core::ObjectId inc_d = bd_out.value.generated_pole_ids.front();
  const std::vector<wire::core::ObjectId> bd_ports_before =
      unique_generated_ports_on_pole(incremental, bd_out.value.generated_span_ids, inc_b);
  const std::vector<wire::core::ObjectId> bd_spans_before = bd_out.value.generated_span_ids;

  wire::core::BackboneSpec eb = line_req(incremental);
  eb.path.polyline = {{20.0, 0.0, 0.0}, inc_pole_b->world_transform.position};
  eb.path.node_specs = {pole_spec(1, inc_b)};
  const auto eb_out = incremental.GenerateFromBackboneSpec(eb);
  if (!eb_out.ok || eb_out.value.generated_pole_ids.size() != 1 ||
      eb_out.value.generated_span_ids.empty()) return false;
  const wire::core::ObjectId inc_e = eb_out.value.generated_pole_ids.front();
  const wire::core::SavedBackboneNode* inc_node_b = incremental.view().backbone_node_for_pole(inc_b);
  const wire::core::SavedBackboneNode* inc_node_d = incremental.view().backbone_node_for_pole(inc_d);
  const wire::core::SavedBackboneNode* inc_node_e = incremental.view().backbone_node_for_pole(inc_e);
  if (inc_node_b == nullptr || inc_node_d == nullptr || inc_node_e == nullptr) return false;
  const wire::core::ObjectId inc_bd = edge_between(incremental, inc_node_b->node_id, inc_node_d->node_id);
  const wire::core::ObjectId inc_be = edge_between(incremental, inc_node_b->node_id, inc_node_e->node_id);
  const JunctionRowSnapshot actual = junction_snapshot(incremental, inc_b);
  const std::vector<wire::core::ObjectId> bd_ports_after =
      unique_generated_ports_on_pole(incremental, bd_spans_before, inc_b);

  return inc_bd != wire::core::kInvalidObjectId && inc_be != wire::core::kInvalidObjectId &&
         actual.pair_rows == 2 && actual.open_rows == 0 && actual.support_levels == 2 &&
         has_row_key(actual.row_keys, false, inc_bd, inc_be) &&
         bd_ports_before == bd_ports_after &&
         std::all_of(bd_spans_before.begin(), bd_spans_before.end(), [&](wire::core::ObjectId span_id) {
           return incremental.view().spans().find(span_id) != nullptr;
         }) &&
         eb_out.value.generated_span_ids.size() == bd_spans_before.size() &&
         snapshots_match(expected, actual) && curve_endpoints_match_layout(incremental);
}

bool C772_backbone_incremental_pair_promotion_rejects_ambiguous_open_candidates() {
  wire::core::CoreState state;
  const auto abc = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const wire::core::ObjectId b = abc.value.generated_pole_ids[1];
  const wire::core::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  auto add_open = [&](wire::core::Vec3d end) {
    wire::core::BackboneSpec branch = line_req(state);
    branch.path.polyline = {pole_b->world_transform.position, end};
    branch.path.node_specs = {pole_spec(0, b)};
    return state.GenerateFromBackboneSpec(branch);
  };
  const auto bd = add_open({12.0, -8.0, 0.0});
  const auto bf = add_open({16.0, -8.0, 0.0});
  if (!bd.ok || !bf.ok) return false;
  const std::size_t pole_count = state.view().poles().size();
  const std::size_t span_count = state.view().spans().size();
  const std::size_t edge_count = state.view().backbone().edges.size();
  const std::size_t binding_count = state.view().backbone().port_bindings.size();

  wire::core::BackboneSpec completion = line_req(state);
  completion.path.polyline = {{20.0, 0.0, 0.0}, pole_b->world_transform.position};
  completion.path.node_specs = {pole_spec(1, b)};
  const auto out = state.GenerateFromBackboneSpec(completion);
  return !out.ok && contains_text(out.error, "ambiguous promoted open edge") &&
         state.view().poles().size() == pole_count &&
         state.view().spans().size() == span_count &&
         state.view().backbone().edges.size() == edge_count &&
         state.view().backbone().port_bindings.size() == binding_count;
}

bool C773_backbone_incremental_sharp_completion_keeps_open_jumper_rows() {
  IncrementalCrossFixture fixture{};
  if (!make_incremental_cross(&fixture, true, wire::core::BundleKind::kLowVoltage,
                              {12.0, -8.0, 0.0}, {13.0, -10.0, 0.0}) ||
      !fixture.completion.ok) {
    return false;
  }
  const wire::core::SavedBackboneNode* node_b = fixture.state.view().backbone_node_for_pole(fixture.pole_b);
  const wire::core::SavedBackboneNode* node_d = fixture.state.view().backbone_node_for_pole(fixture.pole_d);
  const wire::core::SavedBackboneNode* node_e = fixture.state.view().backbone_node_for_pole(fixture.pole_e);
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr) return false;
  const wire::core::ObjectId bd = edge_between(fixture.state, node_b->node_id, node_d->node_id);
  const wire::core::ObjectId be = edge_between(fixture.state, node_b->node_id, node_e->node_id);
  const JunctionRowSnapshot snapshot = junction_snapshot(fixture.state, fixture.pole_b);
  return bd != wire::core::kInvalidObjectId && be != wire::core::kInvalidObjectId &&
         snapshot.pair_rows == 1 && snapshot.open_rows == 2 &&
         has_row_key(snapshot.row_keys, true, bd, wire::core::kInvalidObjectId) &&
         has_row_key(snapshot.row_keys, true, be, wire::core::kInvalidObjectId) &&
         fixture.bd_ports == unique_generated_ports_on_pole(fixture.state, fixture.bd_spans, fixture.pole_b) &&
         curve_endpoints_match_layout(fixture.state);
}

bool C774_backbone_incremental_scope_mismatch_does_not_share_ports() {
  IncrementalCrossFixture fixture{};
  if (!make_incremental_cross(&fixture, true, wire::core::BundleKind::kHighVoltage) ||
      !fixture.completion.ok || fixture.completion.value.generated_span_ids.empty()) {
    return false;
  }
  const std::vector<wire::core::ObjectId> be_ports =
      unique_generated_ports_on_pole(fixture.state, fixture.completion.value.generated_span_ids, fixture.pole_b);
  if (be_ports.empty()) return false;
  for (wire::core::ObjectId port_id : be_ports) {
    if (std::find(fixture.bd_ports.begin(), fixture.bd_ports.end(), port_id) != fixture.bd_ports.end()) {
      return false;
    }
  }
  const JunctionRowSnapshot snapshot = junction_snapshot(fixture.state, fixture.pole_b);
  return snapshot.open_rows >= 1 && fixture.bd_ports == unique_generated_ports_on_pole(fixture.state, fixture.bd_spans, fixture.pole_b);
}

bool C775_backbone_incremental_canonical_pair_survives_save_load() {
  IncrementalCrossFixture source{};
  if (!make_incremental_cross(&source) || !source.completion.ok || !canonical_cross_at_b(source)) {
    return false;
  }
  const JunctionRowSnapshot before = junction_snapshot(source.state, source.pole_b);
  std::string saved{};
  const auto serialized = source.state.SerializeAuthoritative(&saved);
  if (!serialized.ok) {
    return false;
  }
  wire::core::CoreState loaded;
  const auto deserialized = loaded.DeserializeAuthoritative(saved);
  if (!deserialized.ok) {
    return false;
  }
  const JunctionRowSnapshot after = junction_snapshot(loaded, source.pole_b);
  return snapshots_match(before, after) && curve_endpoints_match_layout(loaded);
}

bool C776_backbone_incremental_canonical_pair_survives_regenerate() {
  IncrementalCrossFixture fixture{};
  if (!make_incremental_cross(&fixture) || !fixture.completion.ok || !canonical_cross_at_b(fixture)) {
    return false;
  }
  if (fixture.completion.value.bundle_ids.empty()) return false;
  const wire::core::ObjectId bundle_id = fixture.completion.value.bundle_ids.front();
  const wire::core::Bundle* bundle = fixture.state.view().bundles().find(bundle_id);
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
  auto multi_bundle_req = [](wire::core::CoreState& state) {
    wire::core::BackboneSpec req = poly3_req(state);
    req.bundles.clear();
    add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
    add_backbone_bundle(req, wire::core::BundleKind::kOptical);
    return req;
  };

  wire::core::CoreState state;
  wire::core::BackboneSpec base = multi_bundle_req(state);
  const auto abc = state.GenerateFromBackboneSpec(base);
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const wire::core::ObjectId b = abc.value.generated_pole_ids[1];
  const wire::core::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  wire::core::BackboneSpec bd = multi_bundle_req(state);
  bd.path.polyline = {pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd.path.node_specs = {pole_spec(0, b)};
  const auto bd_out = state.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_pole_ids.size() != 1 || bd_out.value.generated_span_ids.empty()) {
    return false;
  }
  const std::vector<wire::core::ObjectId> bd_ports = unique_generated_ports_on_pole(state, bd_out.value.generated_span_ids, b);
  const std::vector<wire::core::ObjectId> bd_spans = bd_out.value.generated_span_ids;

  wire::core::BackboneSpec eb = multi_bundle_req(state);
  eb.path.polyline = {{20.0, 0.0, 0.0}, pole_b->world_transform.position};
  eb.path.node_specs = {pole_spec(1, b)};
  const auto eb_out = state.GenerateFromBackboneSpec(eb);
  if (!eb_out.ok || eb_out.value.generated_pole_ids.size() != 1 || eb_out.value.generated_span_ids.empty()) {
    return false;
  }

  const wire::core::ObjectId d = bd_out.value.generated_pole_ids.front();
  const wire::core::ObjectId e = eb_out.value.generated_pole_ids.front();
  const wire::core::SavedBackboneNode* node_b = state.view().backbone_node_for_pole(b);
  const wire::core::SavedBackboneNode* node_d = state.view().backbone_node_for_pole(d);
  const wire::core::SavedBackboneNode* node_e = state.view().backbone_node_for_pole(e);
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr) return false;
  const wire::core::ObjectId bd_edge = edge_between(state, node_b->node_id, node_d->node_id);
  const wire::core::ObjectId be_edge = edge_between(state, node_b->node_id, node_e->node_id);
  const JunctionRowSnapshot snapshot = junction_snapshot(state, b);

  return bd_edge != wire::core::kInvalidObjectId && be_edge != wire::core::kInvalidObjectId &&
         snapshot.pair_rows == 2 && snapshot.open_rows == 0 &&
         has_row_key(snapshot.row_keys, false, bd_edge, be_edge) &&
         bd_ports == unique_generated_ports_on_pole(state, bd_spans, b) &&
         eb_out.value.generated_span_ids.size() == bd_spans.size() &&
         curve_endpoints_match_layout(state);
}

bool C779_backbone_incremental_same_template_multi_placement_uses_placement_key() {
  auto web_default_req = [](wire::core::CoreState& state) {
    wire::core::BackboneSpec req = poly3_req(state);
    req.bundles.clear();
    auto add = [&](wire::core::BundleKind kind, std::uint64_t placement_key, int count,
                   double height_m) {
      const wire::core::BundleTemplateId template_id = wire::core::DefaultBundleTemplateId(kind);
      const auto template_it = state.view().bundle_templates().find(template_id);
      if (template_it == state.view().bundle_templates().end()) {
        return;
      }
      wire::core::BackboneBundleSpec bundle{};
      bundle.bundle_template_id = template_id;
      bundle.placement_key = placement_key;
      bundle.layer = template_it->second.default_layer;
      bundle.count = count;
      bundle.placement_explicit = true;
      bundle.height_m = height_m;
      bundle.lateral_m = kind == wire::core::BundleKind::kHighVoltage ? -0.2 : 0.0;
      bundle.spacing_m = template_it->second.default_spacing_m;
      req.bundles.push_back(bundle);
    };
    add(wire::core::BundleKind::kHighVoltage, 1, 3, 9.2);
    add(wire::core::BundleKind::kLowVoltage, 2, 1, 7.7);
    add(wire::core::BundleKind::kLowVoltage, 3, 1, 7.35);
    add(wire::core::BundleKind::kLowVoltage, 4, 1, 7.0);
    add(wire::core::BundleKind::kCommunication, 5, 1, 5.5);
    add(wire::core::BundleKind::kOptical, 6, 1, 5.3);
    return req;
  };
  auto edge_bundle_for_key = [](const wire::core::CoreState& state, wire::core::ObjectId edge_id,
                                std::uint64_t placement_key) {
    const auto bundles_it = state.view().backbone_index().edge_bundles.find(edge_id);
    if (bundles_it == state.view().backbone_index().edge_bundles.end()) {
      return wire::core::kInvalidObjectId;
    }
    for (wire::core::ObjectId edge_bundle_id : bundles_it->second) {
      const wire::core::SavedBackboneEdgeBundle* edge_bundle =
          state.view().backbone_edge_bundle(edge_bundle_id);
      const wire::core::Bundle* bundle =
          edge_bundle == nullptr ? nullptr : state.view().bundles().find(edge_bundle->bundle_id);
      if (bundle != nullptr && bundle->placement_key == placement_key) {
        return edge_bundle_id;
      }
    }
    return wire::core::kInvalidObjectId;
  };
  auto port_for_edge_bundle_at_node = [](const wire::core::CoreState& state, wire::core::ObjectId edge_bundle_id,
                                         wire::core::ObjectId node_id) {
    wire::core::ObjectId found = wire::core::kInvalidObjectId;
    for (const wire::core::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
      if (binding.edge_bundle_id != edge_bundle_id || binding.row_key.node_id != node_id) continue;
      if (found != wire::core::kInvalidObjectId && found != binding.port_id) {
        return wire::core::kInvalidObjectId;
      }
      found = binding.port_id;
    }
    return found;
  };

  wire::core::CoreState state;
  const auto abc = state.GenerateFromBackboneSpec(web_default_req(state));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const wire::core::ObjectId b = abc.value.generated_pole_ids[1];
  const wire::core::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  wire::core::BackboneSpec bd = web_default_req(state);
  bd.path.polyline = {pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd.path.node_specs = {pole_spec(0, b)};
  const auto bd_out = state.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_pole_ids.size() != 1 || bd_out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId d = bd_out.value.generated_pole_ids.front();
  const wire::core::SavedBackboneNode* node_b_before = state.view().backbone_node_for_pole(b);
  const wire::core::SavedBackboneNode* node_d_before = state.view().backbone_node_for_pole(d);
  if (node_b_before == nullptr || node_d_before == nullptr) return false;
  const wire::core::ObjectId bd_edge_before = edge_between(state, node_b_before->node_id, node_d_before->node_id);
  if (bd_edge_before == wire::core::kInvalidObjectId) return false;

  std::vector<std::pair<std::uint64_t, wire::core::ObjectId>> bd_bundle_ids{};
  std::vector<std::pair<std::uint64_t, wire::core::ObjectId>> bd_ports{};
  for (std::uint64_t key : {2ULL, 3ULL, 4ULL}) {
    const wire::core::ObjectId edge_bundle_id = edge_bundle_for_key(state, bd_edge_before, key);
    const wire::core::SavedBackboneEdgeBundle* edge_bundle = state.view().backbone_edge_bundle(edge_bundle_id);
    if (edge_bundle == nullptr) return false;
    const wire::core::ObjectId port_id =
        port_for_edge_bundle_at_node(state, edge_bundle_id, node_b_before->node_id);
    if (port_id == wire::core::kInvalidObjectId) return false;
    bd_bundle_ids.push_back({key, edge_bundle->bundle_id});
    bd_ports.push_back({key, port_id});
  }

  wire::core::BackboneSpec eb = web_default_req(state);
  eb.path.polyline = {{20.0, 0.0, 0.0}, pole_b->world_transform.position};
  eb.path.node_specs = {pole_spec(1, b)};
  const auto eb_out = state.GenerateFromBackboneSpec(eb);
  if (!eb_out.ok || eb_out.value.generated_pole_ids.size() != 1 || eb_out.value.generated_span_ids.empty()) {
    return false;
  }

  const wire::core::ObjectId e = eb_out.value.generated_pole_ids.front();
  const wire::core::SavedBackboneNode* node_b = state.view().backbone_node_for_pole(b);
  const wire::core::SavedBackboneNode* node_d = state.view().backbone_node_for_pole(d);
  const wire::core::SavedBackboneNode* node_e = state.view().backbone_node_for_pole(e);
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr) return false;
  const wire::core::ObjectId bd_edge = edge_between(state, node_b->node_id, node_d->node_id);
  const wire::core::ObjectId be_edge = edge_between(state, node_b->node_id, node_e->node_id);
  if (bd_edge == wire::core::kInvalidObjectId || be_edge == wire::core::kInvalidObjectId) return false;

  for (const auto& [key, bd_bundle_id] : bd_bundle_ids) {
    const wire::core::ObjectId be_edge_bundle_id = edge_bundle_for_key(state, be_edge, key);
    const wire::core::SavedBackboneEdgeBundle* be_edge_bundle = state.view().backbone_edge_bundle(be_edge_bundle_id);
    if (be_edge_bundle == nullptr || be_edge_bundle->bundle_id != bd_bundle_id) {
      return false;
    }
    const auto bd_port_it = std::find_if(bd_ports.begin(), bd_ports.end(), [&](const auto& item) {
      return item.first == key;
    });
    if (bd_port_it == bd_ports.end() ||
        port_for_edge_bundle_at_node(state, be_edge_bundle_id, node_b->node_id) != bd_port_it->second) {
      return false;
    }
  }

  const JunctionRowSnapshot snapshot = junction_snapshot(state, b);
  wire::core::CoreState one_shot;
  const auto one_abc = one_shot.GenerateFromBackboneSpec(web_default_req(one_shot));
  if (!one_abc.ok || one_abc.value.generated_pole_ids.size() != 3) return false;
  const wire::core::ObjectId one_b = one_abc.value.generated_pole_ids[1];
  const wire::core::Pole* one_pole_b = one_shot.view().poles().find(one_b);
  if (one_pole_b == nullptr) return false;
  wire::core::BackboneSpec one_cross = web_default_req(one_shot);
  one_cross.path.polyline = {{12.0, -8.0, 0.0}, one_pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  one_cross.path.node_specs = {pole_spec(1, one_b)};
  const auto one_out = one_shot.GenerateFromBackboneSpec(one_cross);
  if (!one_out.ok) return false;
  const JunctionRowSnapshot expected = junction_snapshot(one_shot, one_b);

  return snapshot.pair_rows == 2 && snapshot.open_rows == 0 &&
         has_row_key(snapshot.row_keys, false, bd_edge, be_edge) &&
         eb_out.value.generated_span_ids.size() == bd_out.value.generated_span_ids.size() &&
         std::all_of(bd_out.value.generated_span_ids.begin(), bd_out.value.generated_span_ids.end(),
                     [&](wire::core::ObjectId span_id) {
                       return state.view().spans().find(span_id) != nullptr;
                     }) &&
         snapshots_match(expected, snapshot) &&
         curve_endpoints_match_layout(state);
}

bool C780_backbone_incremental_duplicate_values_are_order_independent_by_placement_key() {
  auto duplicate_req = [](wire::core::CoreState& state, bool reversed) {
    wire::core::BackboneSpec req = poly3_req(state);
    req.bundles.clear();
    const wire::core::BundleTemplateId template_id =
        wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage);
    const auto template_it = state.view().bundle_templates().find(template_id);
    if (template_it == state.view().bundle_templates().end()) {
      return req;
    }
    auto make = [&](std::uint64_t key) {
      wire::core::BackboneBundleSpec bundle{};
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
  auto bundle_id_for_key_on_edge = [](const wire::core::CoreState& state, wire::core::ObjectId edge_id,
                                      std::uint64_t key) {
    const auto bundles_it = state.view().backbone_index().edge_bundles.find(edge_id);
    if (bundles_it == state.view().backbone_index().edge_bundles.end()) {
      return wire::core::kInvalidObjectId;
    }
    for (wire::core::ObjectId edge_bundle_id : bundles_it->second) {
      const wire::core::SavedBackboneEdgeBundle* edge_bundle =
          state.view().backbone_edge_bundle(edge_bundle_id);
      const wire::core::Bundle* bundle =
          edge_bundle == nullptr ? nullptr : state.view().bundles().find(edge_bundle->bundle_id);
      if (bundle != nullptr && bundle->placement_key == key) {
        return bundle->id;
      }
    }
    return wire::core::kInvalidObjectId;
  };

  wire::core::CoreState state;
  const auto abc = state.GenerateFromBackboneSpec(duplicate_req(state, false));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const wire::core::ObjectId b = abc.value.generated_pole_ids[1];
  const wire::core::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  wire::core::BackboneSpec bd = duplicate_req(state, false);
  bd.path.polyline = {pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd.path.node_specs = {pole_spec(0, b)};
  const auto bd_out = state.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_pole_ids.size() != 1) return false;
  const wire::core::ObjectId d = bd_out.value.generated_pole_ids.front();
  const wire::core::SavedBackboneNode* node_b_before = state.view().backbone_node_for_pole(b);
  const wire::core::SavedBackboneNode* node_d_before = state.view().backbone_node_for_pole(d);
  if (node_b_before == nullptr || node_d_before == nullptr) return false;
  const wire::core::ObjectId bd_edge_before = edge_between(state, node_b_before->node_id, node_d_before->node_id);
  if (bd_edge_before == wire::core::kInvalidObjectId) return false;
  const wire::core::ObjectId key_11_bundle = bundle_id_for_key_on_edge(state, bd_edge_before, 11);
  const wire::core::ObjectId key_12_bundle = bundle_id_for_key_on_edge(state, bd_edge_before, 12);
  if (key_11_bundle == wire::core::kInvalidObjectId || key_12_bundle == wire::core::kInvalidObjectId ||
      key_11_bundle == key_12_bundle) {
    return false;
  }

  wire::core::BackboneSpec eb = duplicate_req(state, true);
  eb.path.polyline = {{20.0, 0.0, 0.0}, pole_b->world_transform.position};
  eb.path.node_specs = {pole_spec(1, b)};
  const auto eb_out = state.GenerateFromBackboneSpec(eb);
  if (!eb_out.ok || eb_out.value.generated_pole_ids.size() != 1) return false;
  const wire::core::ObjectId e = eb_out.value.generated_pole_ids.front();
  const wire::core::SavedBackboneNode* node_b = state.view().backbone_node_for_pole(b);
  const wire::core::SavedBackboneNode* node_e = state.view().backbone_node_for_pole(e);
  if (node_b == nullptr || node_e == nullptr) return false;
  const wire::core::ObjectId be_edge = edge_between(state, node_b->node_id, node_e->node_id);
  if (be_edge == wire::core::kInvalidObjectId) return false;
  const JunctionRowSnapshot snapshot = junction_snapshot(state, b);
  return bundle_id_for_key_on_edge(state, be_edge, 11) == key_11_bundle &&
         bundle_id_for_key_on_edge(state, be_edge, 12) == key_12_bundle &&
         snapshot.pair_rows == 2 && snapshot.open_rows == 0 &&
         std::all_of(bd_out.value.generated_span_ids.begin(), bd_out.value.generated_span_ids.end(),
                     [&](wire::core::ObjectId span_id) {
                       return state.view().spans().find(span_id) != nullptr;
                     }) &&
         curve_endpoints_match_layout(state);
}

bool C781_backbone_incremental_cross_extension_preserves_existing_spans() {
  IncrementalCrossFixture fixture{};
  if (!make_incremental_cross(&fixture) || !fixture.completion.ok || !canonical_cross_at_b(fixture)) {
    return false;
  }
  const std::vector<wire::core::ObjectId> original_spans = fixture.bd_spans;
  std::vector<wire::core::ObjectId> cross_spans = fixture.completion.value.generated_span_ids;
  cross_spans.insert(cross_spans.end(), original_spans.begin(), original_spans.end());
  for (wire::core::ObjectId span_id : cross_spans) {
    if (fixture.state.view().spans().find(span_id) == nullptr) return false;
  }

  const wire::core::Pole* pole_b = fixture.state.view().poles().find(fixture.pole_b);
  if (pole_b == nullptr) return false;
  wire::core::BackboneSpec bf = line_req(fixture.state);
  bf.path.polyline = {pole_b->world_transform.position, {4.0, -8.0, 0.0}};
  bf.path.node_specs = {pole_spec(0, fixture.pole_b)};
  const auto out = fixture.state.GenerateFromBackboneSpec(bf);
  if (!out.ok || out.value.generated_pole_ids.size() != 1 || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : cross_spans) {
    if (fixture.state.view().spans().find(span_id) == nullptr) return false;
  }
  const JunctionRowSnapshot snapshot = junction_snapshot(fixture.state, fixture.pole_b);
  return snapshot.pair_rows >= 2 && snapshot.open_rows >= 1 &&
         fixture.bd_ports == unique_generated_ports_on_pole(fixture.state, fixture.bd_spans, fixture.pole_b) &&
         curve_endpoints_match_layout(fixture.state);
}

bool C460_backbone_context_links_are_not_emitted() {
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

bool C461_backbone_same_edge_request_skips_duplicate_context() {
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

bool C462_backbone_no_junction_kind_after_existing_context() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  return C391_backbone_no_kind_label() && contains_text(cpp, "has_requested_saved_pair") &&
         contains_text(cpp, "if (!edge.is_new)") && contains_text(cpp, "EditResult<pairs> pipeline::make");
}

bool C463_backbone_duplicate_same_edge_bundle_rejected() {
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

bool C464_backbone_different_bundle_on_same_edge_allowed() {
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

bool C466_backbone_duplicate_reject_keeps_state_unchanged() {
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

bool C467_backbone_saves_row_port_bindings() {
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

bool C468_backbone_row_port_binding_is_stable_for_existing_context() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
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

bool C469_backbone_row_port_binding_rejects_duplicate_without_resolution() {
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

bool C471_backbone_resolves_existing_port_by_binding() {
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

bool C487_backbone_port_resolution_requires_bundle_compatible_scope() {
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

bool C472_backbone_port_resolution_requires_saved_binding() {
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

bool C473_backbone_resolved_port_used_by_new_span_endpoint() {
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

bool C474_backbone_port_resolution_rejects_ambiguous_binding() {
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
  return contains_text(body, "ambiguous backbone port binding") && contains_text(body, "found != kInvalidObjectId") &&
         contains_text(body, "found != port->id");
}

bool C476_backbone_branch_rows_are_separated_without_branch_kind() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const std::vector<wire::core::Vec3d> before = pole_port_positions(state, b);
  const std::vector<wire::core::Vec3d> main_row_ports =
      generated_ports_on_pole(state, first.value.generated_span_ids, b);
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr || main_row_ports.empty()) {
    return false;
  }
  const wire::core::Vec3d pole_b_position = pole_b->world_transform.position;
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b_position));
  const std::vector<wire::core::Vec3d> placed = generated_ports_on_pole(state, second.value.generated_span_ids, b);
  if (!second.ok || !separated_from(before, placed) || placed.empty()) {
    return false;
  }
  wire::core::Vec3d center{};
  for (const wire::core::Vec3d& point : placed) {
    center.x += point.x;
    center.y += point.y;
  }
  center.x /= static_cast<double>(placed.size());
  center.y /= static_cast<double>(placed.size());
  double main_height = 0.0;
  double branch_height = 0.0;
  for (const wire::core::Vec3d& point : main_row_ports) {
    main_height += point.z;
  }
  for (const wire::core::Vec3d& point : placed) {
    branch_height += point.z;
  }
  main_height /= static_cast<double>(main_row_ports.size());
  branch_height /= static_cast<double>(placed.size());
  return almost_equal(center.x, pole_b_position.x, 1e-9) && almost_equal(center.y, pole_b_position.y, 1e-9) &&
         std::abs(branch_height - main_height) > 0.1 && C391_backbone_no_kind_label();
}

bool C477_backbone_cross_rows_are_separated_without_cross_kind() {
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
  return second.ok && separated_from(before, placed) && C391_backbone_no_kind_label();
}

bool C478_backbone_row_separation_is_deterministic() {
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

bool C480_backbone_context_rows_affect_order_but_are_not_emitted() {
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
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  return !second.value.generated_span_ids.empty() && contains_text(cpp, "row_height_offsets(ps)") &&
         contains_text(cpp, "if (r.id >= active_rows.size() || !active_rows[r.id])");
}

bool C481_backbone_pass_through_mode_is_accepted_in_limited_scope() {
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
  const wire::core::Vec3d pole_b_position = pole_b->world_transform.position;
  const auto ok = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b_position));

  wire::core::BackboneSpec bad_index = pass_branch_req(state, b, pole_b_position);
  bad_index.node_bundle_modes.front().point_index = 9;
  const auto bad_index_out = state.GenerateFromBackboneSpec(bad_index);

  wire::core::BackboneSpec bad_bundle = pass_branch_req(state, b, pole_b_position);
  bad_bundle.node_bundle_modes.front().bundle_template_id = 999;
  const auto bad_bundle_out = state.GenerateFromBackboneSpec(bad_bundle);
  return ok.ok && !bad_index_out.ok && !bad_bundle_out.ok;
}

bool C482_backbone_pass_through_creates_explicit_intent() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
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
  return saw_intent && C479_backbone_row_separation_does_not_change_pairs();
}

bool C483_backbone_pass_through_ambiguous_target_rejected() {
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
  return contains_text(body, "matches.size() != 1") &&
         contains_text(body, "pass-through target row is ambiguous");
}

bool C484_backbone_lowering_draw_uses_layout_only() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  bool saw_lowered_layout = false;
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::SpanLayoutView layout = state.span_layout(span_id);
    const wire::core::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    const wire::core::SpanRenderCacheEntry* render = state.find_span_render_cache(span_id);
    if (!layout.has_layout() || visual == nullptr || render == nullptr) {
      return false;
    }
    const auto lowered = [](const wire::core::LayoutEndpoint& endpoint) {
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

bool C488_backbone_port_resolution_accepts_same_compatible_binding() {
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

bool C489_backbone_port_binding_index_invariant() {
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

bool C490_backbone_duplicate_same_edge_bundle_lane_rejected() {
  return C463_backbone_duplicate_same_edge_bundle_rejected() && C466_backbone_duplicate_reject_keeps_state_unchanged();
}

bool C502_backbone_span_bindings_save_lane() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  if (!out.ok || graph.edge_bundles.size() != 1 || graph.span_bindings.size() != out.value.generated_span_ids.size()) {
    return false;
  }
  const wire::core::ObjectId edge_bundle_id = graph.edge_bundles.front().edge_bundle_id;
  const wire::core::SavedBackboneEdgeBundle& edge_bundle = graph.edge_bundles.front();
  const wire::core::BackboneIndex& backbone_index = state.view().backbone_index();
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

bool C503_backbone_duplicate_span_binding_rejected_by_lane() {
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

bool C491_backbone_branch_lowering_v1_affects_geom() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
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

bool C492_backbone_cross_lowering_v1_affects_only_new_links() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
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
  cross.bundles.clear();
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
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

bool C496_backbone_junction_v1_deterministic() {
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

bool C497_backbone_context_rows_order_but_do_not_materialize() {
  return C480_backbone_context_rows_affect_order_but_are_not_emitted();
}

bool C498_backbone_saved_graph_remains_topology_authority() {
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

bool C499_backbone_context_link_is_not_saved() {
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

bool C500_backbone_context_link_requires_saved_edge_ref() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  std::string ref_body;
  std::string save_body;
  if (!function_body(cpp, "SavedBackboneEdgeRef ref_for_existing_edge", &ref_body) ||
      !function_body(cpp, "EditResult<bool> pipeline::save_graph(const topo& made, const pairs& ps)", &save_body)) {
    return false;
  }
  return contains_text(ref_body, "edge.saved == kInvalidObjectId") && !contains_text(ref_body, "saved_edge_for") &&
         contains_text(save_body, "context link saved edge missing");
}

} // namespace backbone_tests
