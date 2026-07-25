#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "../../src/generation/backbone/mount_graph.hpp"
#include "../../src/generation/backbone/model_placement_rules.hpp"
#include "../../src/support/instrumentation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
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
  return contains_text(body, "group_for") && !contains_text(body, "ps.jumpers") &&
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

wire::core::ObjectId edge_bundle_for_edge_and_lane(const wire::core::CoreState& state,
                                                   wire::core::ObjectId edge_id,
                                                   std::size_t lane_index) {
  const auto edge_bundles_it = state.view().backbone_index().edge_bundles.find(edge_id);
  if (edge_bundles_it == state.view().backbone_index().edge_bundles.end()) {
    return wire::core::kInvalidObjectId;
  }
  wire::core::ObjectId matched = wire::core::kInvalidObjectId;
  for (wire::core::ObjectId edge_bundle_id : edge_bundles_it->second) {
    const auto span_bindings_it = state.view().backbone_index().edge_bundle_span_bindings.find(edge_bundle_id);
    if (span_bindings_it == state.view().backbone_index().edge_bundle_span_bindings.end()) {
      continue;
    }
    for (std::size_t index : span_bindings_it->second) {
      if (index >= state.view().backbone().span_bindings.size()) {
        continue;
      }
      const wire::core::SavedBackboneSpanBinding& binding = state.view().backbone().span_bindings[index];
      if (binding.lane_index != lane_index) {
        continue;
      }
      if (matched != wire::core::kInvalidObjectId && matched != edge_bundle_id) {
        return wire::core::kInvalidObjectId;
      }
      matched = edge_bundle_id;
    }
  }
  return matched;
}

bool has_row_continuity(const wire::core::CoreState& state,
                        wire::core::ObjectId node_id,
                        wire::core::ObjectId edge_bundle_a,
                        std::size_t lane_a,
                        wire::core::ObjectId edge_bundle_b,
                        std::size_t lane_b) {
  for (const wire::core::SavedBackboneRowContinuity* continuity :
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

bool edge_bundle_lane_exists(const wire::core::CoreState& state,
                             wire::core::ObjectId edge_bundle_id,
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

bool row_continuity_graph_lint_passes(const wire::core::CoreState& state) {
  std::vector<std::tuple<wire::core::ObjectId, wire::core::ObjectId, std::size_t>> endpoints{};
  for (const wire::core::SavedBackboneRowContinuity& continuity : state.view().backbone().row_continuities) {
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

wire::core::ObjectId edge_bundle_for_template(const wire::core::CoreState& state,
                                              wire::core::ObjectId edge_id,
                                              wire::core::BundleKind kind) {
  const wire::core::BundleTemplateId template_id = wire::core::DefaultBundleTemplateId(kind);
  const auto bundles_it = state.view().backbone_index().edge_bundles.find(edge_id);
  if (bundles_it == state.view().backbone_index().edge_bundles.end()) {
    return wire::core::kInvalidObjectId;
  }
  for (wire::core::ObjectId edge_bundle_id : bundles_it->second) {
    const wire::core::SavedBackboneEdgeBundle* edge_bundle =
        state.view().backbone_edge_bundle(edge_bundle_id);
    const wire::core::Bundle* bundle =
        edge_bundle == nullptr ? nullptr : state.view().bundles().find(edge_bundle->bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == template_id) {
      return edge_bundle_id;
    }
  }
  return wire::core::kInvalidObjectId;
}

struct PortBindingSnapshot {
  std::size_t lane = 0;
  wire::core::ObjectId port_id = wire::core::kInvalidObjectId;
  wire::core::Vec3d position{};
  double layout_yaw_deg = 0.0;
};

std::vector<PortBindingSnapshot> port_binding_snapshot(const wire::core::CoreState& state,
                                                       wire::core::ObjectId edge_bundle_id,
                                                       wire::core::ObjectId node_id) {
  std::vector<PortBindingSnapshot> out{};
  for (const wire::core::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.edge_bundle_id != edge_bundle_id || binding.row_key.node_id != node_id) {
      continue;
    }
    const wire::core::Port* port = state.view().ports().find(binding.port_id);
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

bool same_port_binding_snapshot(const std::vector<PortBindingSnapshot>& a,
                                const std::vector<PortBindingSnapshot>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].lane != b[i].lane || a[i].port_id != b[i].port_id ||
        !almost_equal(a[i].position, b[i].position, 1e-9) ||
        std::abs(wire::core::NormalizeYawDeg(a[i].layout_yaw_deg - b[i].layout_yaw_deg)) > 1e-9) {
      return false;
    }
  }
  return true;
}

bool same_port_binding_geometry(const std::vector<PortBindingSnapshot>& a,
                                const std::vector<PortBindingSnapshot>& b,
                                bool require_distinct_ids) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].lane != b[i].lane ||
        !almost_equal(a[i].position, b[i].position, 1e-9) ||
        std::abs(wire::core::NormalizeYawDeg(
            a[i].layout_yaw_deg - b[i].layout_yaw_deg)) > 1e-9 ||
        (require_distinct_ids && a[i].port_id == b[i].port_id)) {
      return false;
    }
  }
  return true;
}

bool same_transform(const wire::core::Transformd& a, const wire::core::Transformd& b) {
  return almost_equal(a.position, b.position, 1e-9) &&
         almost_equal(a.rotation_euler_deg, b.rotation_euler_deg, 1e-9) &&
         almost_equal(a.scale, b.scale, 1e-9);
}

bool register_hv_fixture_models(wire::core::CoreState* state,
                                wire::core::ModelAssemblyTemplateId row_id,
                                wire::core::ModelAssemblyTemplateId endpoint_id) {
  if (state == nullptr) {
    return false;
  }
  wire::core::ModelAssemblyTemplate row_assembly{};
  row_assembly.id = row_id;
  wire::core::ModelAssemblyPart row_part{};
  row_part.part_id = 1;
  row_part.model_key = "hv_crossarm";
  row_part.sockets.push_back({"endpoint_mount", {0.0, 0.0, 0.04}, {0.0, 0.0, 1.0}});
  row_assembly.parts.push_back(row_part);
  row_assembly.endpoint_mount_socket = wire::core::AssemblySocketRef{1, "endpoint_mount"};

  wire::core::ModelAssemblyTemplate endpoint_assembly{};
  endpoint_assembly.id = endpoint_id;
  wire::core::ModelAssemblyPart endpoint_part{};
  endpoint_part.part_id = 1;
  endpoint_part.model_key = "hv_insulator";
  endpoint_part.sockets.push_back({"wire", {0.0, 0.0, 0.20}, {1.0, 0.0, 0.0}});
  endpoint_assembly.parts.push_back(endpoint_part);
  endpoint_assembly.wire_socket = wire::core::AssemblySocketRef{1, "wire"};

  if (!state->RegisterModelAssemblyTemplate(row_assembly).ok ||
      !state->RegisterModelAssemblyTemplate(endpoint_assembly).ok) {
    return false;
  }
  const wire::core::BundleTemplateId hv_template_id =
      wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kHighVoltage);
  wire::core::BundleTemplate hv = state->view().bundle_templates().at(hv_template_id);
  hv.row_fixture_assembly_id = row_id;
  hv.endpoint_fixture_assembly_id = endpoint_id;
  return state->UpdateBundleTemplate(hv).ok;
}

wire::core::Vec3d hv_visible_socket(const wire::core::Transformd& fixture);
std::optional<wire::core::Vec3d> layout_endpoint_for_port(
    const wire::core::CoreState& state, wire::core::ObjectId port_id);

std::optional<wire::core::Transformd> endpoint_fixture_transform(
    const wire::core::CoreState& state, wire::core::ObjectId port_id) {
  const auto endpoint = layout_endpoint_for_port(state, port_id);
  if (!endpoint.has_value()) return std::nullopt;
  for (const wire::core::VisualModelInstance& instance :
       state.view().visual_model_instances().instances) {
    if (instance.model_key == "hv_insulator" &&
        almost_equal(hv_visible_socket(instance.world_transform), *endpoint, 1e-9)) {
      return instance.world_transform;
    }
  }
  return std::nullopt;
}

wire::core::Vec3d hv_visible_socket(const wire::core::Transformd& fixture) {
  const wire::core::Vec3d local_socket{0.0, 0.0, 0.20};
  return fixture.position +
         wire::core::RotateEulerXYZDeg({local_socket.x * fixture.scale.x,
                                        local_socket.y * fixture.scale.y,
                                        local_socket.z * fixture.scale.z},
                                       fixture.rotation_euler_deg);
}

std::optional<wire::core::Vec3d> layout_endpoint_for_port(const wire::core::CoreState& state,
                                                          wire::core::ObjectId port_id) {
  for (const wire::core::Span& span : state.view().spans().items()) {
    if (span.port_a_id != port_id && span.port_b_id != port_id) {
      continue;
    }
    const wire::core::SpanLayoutEntry* layout = state.span_layout(span.id).entry;
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

std::vector<wire::core::Transformd> row_fixture_transforms_on_pole(
    const wire::core::CoreState& state, wire::core::ObjectId pole_id) {
  std::vector<wire::core::Transformd> out{};
  const std::string prefix = "row:" + std::to_string(pole_id) + ":";
  for (const wire::core::VisualModelInstance& instance :
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

bool contains_matching_transform(const std::vector<wire::core::Transformd>& transforms,
                                 const wire::core::Transformd& expected) {
  return std::any_of(transforms.begin(), transforms.end(), [&](const auto& candidate) {
    return same_transform(candidate, expected);
  });
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
    const wire::core::SavedBackboneEdgeBundle* own_edge_bundle =
        state.view().backbone_edge_bundle(binding.edge_bundle_id);
    if (own_edge_bundle == nullptr ||
        binding.row_key.edge_id != own_edge_bundle->edge_id) {
      continue;
    }
    wire::core::ObjectId peer_edge = wire::core::kInvalidObjectId;
    for (const wire::core::SavedBackboneRowContinuity& continuity :
         state.view().backbone().row_continuities) {
      if (continuity.node_id != binding.row_key.node_id) continue;
      const bool is_a =
          continuity.a.edge_bundle_id == binding.edge_bundle_id &&
          continuity.a.lane_index == binding.lane_index;
      const bool is_b =
          continuity.b.edge_bundle_id == binding.edge_bundle_id &&
          continuity.b.lane_index == binding.lane_index;
      if (!is_a && !is_b) continue;
      const wire::core::ObjectId peer_edge_bundle_id =
          is_a ? continuity.b.edge_bundle_id : continuity.a.edge_bundle_id;
      const wire::core::SavedBackboneEdgeBundle* peer_edge_bundle =
          state.view().backbone_edge_bundle(peer_edge_bundle_id);
      if (peer_edge != wire::core::kInvalidObjectId ||
          peer_edge_bundle == nullptr) {
        return {};
      }
      peer_edge = peer_edge_bundle->edge_id;
    }
    const bool is_open = peer_edge == wire::core::kInvalidObjectId;
    const wire::core::ObjectId a =
        is_open ? binding.row_key.edge_id
                : std::min(binding.row_key.edge_id, peer_edge);
    const wire::core::ObjectId b =
        is_open ? wire::core::kInvalidObjectId
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

std::vector<double> hv_row_down_offsets_at_pole(const wire::core::CoreState& state,
                                                 wire::core::ObjectId pole_id) {
  std::vector<std::tuple<wire::core::ObjectId, wire::core::ObjectId, double>>
      rows{};
  const wire::core::SavedBackboneNode* node = state.view().backbone_node_for_pole(pole_id);
  if (node == nullptr) {
    return {};
  }
  const wire::core::BundleTemplateId hv_template_id =
      wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kHighVoltage);
  for (const wire::core::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != node->node_id ||
        binding.bundle_template_id != hv_template_id ||
        binding.lane_index != 0) {
      continue;
    }
    wire::core::ObjectId row_a = binding.edge_bundle_id;
    wire::core::ObjectId row_b = wire::core::kInvalidObjectId;
    for (const wire::core::SavedBackboneRowContinuity& continuity :
         state.view().backbone().row_continuities) {
      if (continuity.node_id != node->node_id) continue;
      const bool is_a =
          continuity.a.edge_bundle_id == binding.edge_bundle_id &&
          continuity.a.lane_index == binding.lane_index;
      const bool is_b =
          continuity.b.edge_bundle_id == binding.edge_bundle_id &&
          continuity.b.lane_index == binding.lane_index;
      if (!is_a && !is_b) continue;
      const wire::core::ObjectId peer =
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
    for (const wire::core::Span& span : state.view().spans().items()) {
      const wire::core::SpanLayoutView layout = state.span_layout(span.id);
      if (!layout.has_layout()) {
        continue;
      }
      const wire::core::LayoutEndpoint* endpoint = nullptr;
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
    const wire::core::CoreState& state, wire::core::ObjectId pole_id) {
  std::vector<std::pair<int, double>> out{};
  const wire::core::SavedBackboneNode* node =
      state.view().backbone_node_for_pole(pole_id);
  if (node == nullptr) {
    return out;
  }
  const wire::core::BundleTemplateId hv_template_id =
      wire::core::DefaultBundleTemplateId(
          wire::core::BundleKind::kHighVoltage);
  for (const wire::core::SavedBackbonePortBinding& binding :
       state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != node->node_id ||
        binding.bundle_template_id != hv_template_id ||
        binding.lane_index != 0) {
      continue;
    }
    const wire::core::LayoutEndpoint* matched = nullptr;
    for (const wire::core::Span& span : state.view().spans().items()) {
      const wire::core::SpanLayoutView layout = state.span_layout(span.id);
      if (!layout.has_layout()) {
        continue;
      }
      const wire::core::LayoutEndpoint* candidate = nullptr;
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
    const wire::core::CoreState& state, wire::core::ObjectId pole_id,
    wire::core::ObjectId edge_id) {
  const wire::core::SavedBackboneNode* node =
      state.view().backbone_node_for_pole(pole_id);
  if (node == nullptr) {
    return -1;
  }
  const wire::core::BundleTemplateId hv_template_id =
      wire::core::DefaultBundleTemplateId(
          wire::core::BundleKind::kHighVoltage);
  int level = -1;
  for (const wire::core::SavedBackbonePortBinding& binding :
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

const wire::core::SavedBackbonePortBinding* hv_lane0_binding_for_edge_at_pole(
    const wire::core::CoreState& state, wire::core::ObjectId pole_id,
    wire::core::ObjectId edge_id) {
  const wire::core::SavedBackboneNode* node =
      state.view().backbone_node_for_pole(pole_id);
  if (node == nullptr) {
    return nullptr;
  }
  const wire::core::BundleTemplateId hv_template_id =
      wire::core::DefaultBundleTemplateId(
          wire::core::BundleKind::kHighVoltage);
  const wire::core::SavedBackbonePortBinding* found = nullptr;
  for (const wire::core::SavedBackbonePortBinding& binding :
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
    const wire::core::CoreState& state,
    const wire::core::SavedBackbonePortBinding& binding) {
  const wire::core::Port* port = state.view().ports().find(binding.port_id);
  const wire::core::Pole* pole =
      port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
  if (port == nullptr || pole == nullptr) {
    return std::nullopt;
  }
  const wire::core::PoleFrame frame =
      wire::core::BuildPoleFrame(pole->world_transform, binding.layout_yaw_deg);
  return wire::core::WorldPointToLocal(frame, port->world_position).z;
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
         curve_endpoints_match_layout(incremental);
}

bool C772_backbone_incremental_pair_promotion_leaves_ambiguous_candidates_open() {
  wire::core::CoreState state;
  const auto abc = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const wire::core::ObjectId b = abc.value.generated_pole_ids[1];
  const wire::core::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  auto add_open = [&](wire::core::Vec3d end) {
    wire::core::BackboneSpec branch = line_req(state);
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

  wire::core::BackboneSpec completion = line_req(state);
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
  WIRE_TEST_EXPECT(make_incremental_cross(&fixture, true, wire::core::BundleKind::kLowVoltage,
                                          {12.0, -8.0, 0.0}, {13.0, -10.0, 0.0}),
                   "incremental sharp fixture generation failed");
  WIRE_TEST_EXPECT(fixture.completion.ok, fixture.completion.error);
  const wire::core::SavedBackboneNode* node_b = fixture.state.view().backbone_node_for_pole(fixture.pole_b);
  const wire::core::SavedBackboneNode* node_d = fixture.state.view().backbone_node_for_pole(fixture.pole_d);
  const wire::core::SavedBackboneNode* node_e = fixture.state.view().backbone_node_for_pole(fixture.pole_e);
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr) return false;
  const wire::core::ObjectId bd = edge_between(fixture.state, node_b->node_id, node_d->node_id);
  const wire::core::ObjectId be = edge_between(fixture.state, node_b->node_id, node_e->node_id);
  const JunctionRowSnapshot snapshot = junction_snapshot(fixture.state, fixture.pole_b);
  WIRE_TEST_EXPECT(bd != wire::core::kInvalidObjectId && be != wire::core::kInvalidObjectId,
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
                       [](const wire::core::VisualCurvePart& part) {
                         return part.kind ==
                                wire::core::VisualCurvePartKind::kJumper;
                       }) >= 1,
                   "sharp completion did not derive a jumper");
  WIRE_TEST_EXPECT(curve_endpoints_match_layout(fixture.state), "curve endpoints do not match layout");
  std::string invariant_error{};
  WIRE_TEST_EXPECT(backbone_common_invariants_pass(fixture.state, &invariant_error), invariant_error);
  return true;
}

bool C782_backbone_incremental_sharp_extension_adds_open_when_sharp_candidates_are_ambiguous() {
  wire::core::CoreState state;
  const auto abc = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const wire::core::ObjectId b = abc.value.generated_pole_ids[1];
  const wire::core::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  wire::core::BackboneSpec bd_req = line_req(state);
  bd_req.path.polyline = {pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd_req.path.node_specs = {pole_spec(0, b)};
  const auto bd = state.GenerateFromBackboneSpec(bd_req);
  if (!bd.ok || bd.value.generated_pole_ids.size() != 1) return false;
  const std::vector<wire::core::ObjectId> bd_ports =
      unique_generated_ports_on_pole(state, bd.value.generated_span_ids, b);

  wire::core::BackboneSpec eb_req = line_req(state);
  eb_req.path.polyline = {{13.0, -10.0, 0.0}, pole_b->world_transform.position};
  eb_req.path.node_specs = {pole_spec(1, b)};
  const auto eb = state.GenerateFromBackboneSpec(eb_req);
  if (!eb.ok || eb.value.generated_pole_ids.size() != 1 || eb.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec bg_req = line_req(state);
  bg_req.path.polyline = {pole_b->world_transform.position, {14.0, -12.0, 0.0}};
  bg_req.path.node_specs = {pole_spec(0, b)};
  const auto bg = state.GenerateFromBackboneSpec(bg_req);
  if (!bg.ok || bg.value.generated_pole_ids.size() != 1 || bg.value.generated_span_ids.empty()) {
    return false;
  }

  const wire::core::SavedBackboneNode* node_b = state.view().backbone_node_for_pole(b);
  const wire::core::SavedBackboneNode* node_d =
      state.view().backbone_node_for_pole(bd.value.generated_pole_ids.front());
  const wire::core::SavedBackboneNode* node_e =
      state.view().backbone_node_for_pole(eb.value.generated_pole_ids.front());
  const wire::core::SavedBackboneNode* node_g =
      state.view().backbone_node_for_pole(bg.value.generated_pole_ids.front());
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr || node_g == nullptr) return false;
  const wire::core::ObjectId bd_edge = edge_between(state, node_b->node_id, node_d->node_id);
  const wire::core::ObjectId be_edge = edge_between(state, node_b->node_id, node_e->node_id);
  const wire::core::ObjectId bg_edge = edge_between(state, node_b->node_id, node_g->node_id);
  const JunctionRowSnapshot snapshot = junction_snapshot(state, b);
  return bd_edge != wire::core::kInvalidObjectId && be_edge != wire::core::kInvalidObjectId &&
         bg_edge != wire::core::kInvalidObjectId &&
         snapshot.pair_rows == 2 && snapshot.open_rows == 1 &&
         has_row_key(snapshot.row_keys, false, bd_edge, be_edge) &&
         has_row_key(snapshot.row_keys, true, bg_edge, wire::core::kInvalidObjectId) &&
         bd_ports == unique_generated_ports_on_pole(state, bd.value.generated_span_ids, b) &&
         curve_endpoints_match_layout(state);
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
  WIRE_TEST_EXPECT(make_incremental_cross(&source), "incremental cross fixture generation failed");
  WIRE_TEST_EXPECT(source.completion.ok, source.completion.error);
  WIRE_TEST_EXPECT(canonical_cross_at_b(source), "incremental cross was not canonical at B before save");
  const JunctionRowSnapshot before = junction_snapshot(source.state, source.pole_b);
  std::string saved{};
  const auto serialized = source.state.SerializeAuthoritative(&saved);
  WIRE_TEST_EXPECT(serialized.ok, serialized.error);
  wire::core::CoreState loaded;
  const auto deserialized = loaded.DeserializeAuthoritative(saved);
  WIRE_TEST_EXPECT(deserialized.ok, deserialized.error);
  const JunctionRowSnapshot after = junction_snapshot(loaded, source.pole_b);
  WIRE_TEST_EXPECT(snapshots_match(before, after), "junction row snapshot changed after save/load");
  WIRE_TEST_EXPECT(curve_endpoints_match_layout(loaded), "curve endpoints do not match layout after save/load");
  std::string invariant_error{};
  WIRE_TEST_EXPECT(backbone_common_invariants_pass(loaded, &invariant_error), invariant_error);
  return true;
}

bool C805_backbone_generation_scoped_route_order_does_not_break_t_branch_restore() {
  wire::core::CoreState source;
  wire::core::BackboneSpec base = poly3_req(source);
  base.bundles.clear();
  add_backbone_bundle(base, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(base, wire::core::BundleKind::kLowVoltage);
  const auto abc = source.GenerateFromBackboneSpec(base);
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3 ||
      abc.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId pole_b = abc.value.generated_pole_ids[1];
  const wire::core::Pole* b = source.view().poles().find(pole_b);
  const wire::core::SavedBackboneNode* node_b = source.view().backbone_node_for_pole(pole_b);
  if (b == nullptr || node_b == nullptr) {
    return false;
  }
  wire::core::BackboneSpec branch = base;
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

  wire::core::CoreState loaded;
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
    std::vector<wire::core::ObjectId> found{};
    for (const wire::core::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
      if (binding.edge_bundle_id != edge_bundle_id || binding.row_key.node_id != node_id) continue;
      if (std::find(found.begin(), found.end(), binding.port_id) == found.end()) {
        found.push_back(binding.port_id);
      }
    }
    std::sort(found.begin(), found.end());
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
  if (!bd_out.ok || bd_out.value.generated_pole_ids.size() != 1 ||
      bd_out.value.generated_span_ids.size() != 8) {
    return false;
  }
  const wire::core::ObjectId d = bd_out.value.generated_pole_ids.front();
  const wire::core::SavedBackboneNode* node_b_before = state.view().backbone_node_for_pole(b);
  const wire::core::SavedBackboneNode* node_d_before = state.view().backbone_node_for_pole(d);
  if (node_b_before == nullptr || node_d_before == nullptr) return false;
  const wire::core::ObjectId bd_edge_before = edge_between(state, node_b_before->node_id, node_d_before->node_id);
  if (bd_edge_before == wire::core::kInvalidObjectId) return false;

  std::vector<std::pair<std::uint64_t, wire::core::ObjectId>> bd_bundle_ids{};
  std::vector<std::pair<std::uint64_t, std::vector<wire::core::ObjectId>>> bd_ports{};
  for (std::uint64_t key : {1ULL, 2ULL, 3ULL, 4ULL, 5ULL, 6ULL}) {
    const wire::core::ObjectId edge_bundle_id = edge_bundle_for_key(state, bd_edge_before, key);
    const wire::core::SavedBackboneEdgeBundle* edge_bundle = state.view().backbone_edge_bundle(edge_bundle_id);
    if (edge_bundle == nullptr) return false;
    const std::vector<wire::core::ObjectId> port_ids =
        port_for_edge_bundle_at_node(state, edge_bundle_id, node_b_before->node_id);
    if (port_ids.size() != (key == 1ULL ? 3U : 1U)) return false;
    bd_bundle_ids.push_back({key, edge_bundle->bundle_id});
    bd_ports.push_back({key, port_ids});
  }

  wire::core::BackboneSpec eb = web_default_req(state);
  eb.path.polyline = {{20.0, 0.0, 0.0}, pole_b->world_transform.position};
  eb.path.node_specs = {pole_spec(1, b)};
  const auto eb_out = state.GenerateFromBackboneSpec(eb);
  if (!eb_out.ok || eb_out.value.generated_pole_ids.size() != 1 ||
      eb_out.value.generated_span_ids.size() != 8) {
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
    const std::vector<wire::core::ObjectId> be_ports =
        port_for_edge_bundle_at_node(state, be_edge_bundle_id, node_b->node_id);
    if (bd_port_it == bd_ports.end() || be_ports.size() != bd_port_it->second.size()) {
      return false;
    }
    for (std::size_t lane = 0; lane < be_ports.size(); ++lane) {
      const wire::core::Port* old_port =
          state.view().ports().find(bd_port_it->second[lane]);
      const wire::core::Port* new_port = state.view().ports().find(be_ports[lane]);
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
                     [&](wire::core::ObjectId span_id) {
                       return state.view().spans().find(span_id) != nullptr;
                     }) &&
         curve_endpoints_match_layout(state);
}

bool C785_backbone_incremental_hv_promotion_preserves_existing_row_frame() {
  wire::core::CoreState state;
  const auto abc = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!abc.ok || abc.value.generated_pole_ids.size() != 3) return false;
  const wire::core::ObjectId b = abc.value.generated_pole_ids[1];
  const wire::core::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  wire::core::BackboneSpec bd = line_req(state);
  bd.bundles.clear();
  add_backbone_bundle(bd, wire::core::BundleKind::kHighVoltage);
  bd.path.polyline = {pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd.path.node_specs = {pole_spec(0, b)};
  const auto bd_out = state.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_pole_ids.size() != 1 || bd_out.value.generated_span_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId d = bd_out.value.generated_pole_ids.front();
  const wire::core::SavedBackboneNode* node_b_before = state.view().backbone_node_for_pole(b);
  const wire::core::SavedBackboneNode* node_d_before = state.view().backbone_node_for_pole(d);
  if (node_b_before == nullptr || node_d_before == nullptr) return false;
  const wire::core::ObjectId bd_edge_before = edge_between(state, node_b_before->node_id, node_d_before->node_id);
  const wire::core::ObjectId bd_edge_bundle_before =
      edge_bundle_for_template(state, bd_edge_before, wire::core::BundleKind::kHighVoltage);
  if (bd_edge_bundle_before == wire::core::kInvalidObjectId) return false;
  const std::vector<PortBindingSnapshot> before =
      port_binding_snapshot(state, bd_edge_bundle_before, node_b_before->node_id);
  if (before.size() != 3 || before[0].lane != 0 || before[1].lane != 1 || before[2].lane != 2) {
    return false;
  }

  wire::core::BackboneSpec eb = line_req(state);
  eb.bundles.clear();
  add_backbone_bundle(eb, wire::core::BundleKind::kHighVoltage);
  eb.path.polyline = {{20.0, 0.0, 0.0}, pole_b->world_transform.position};
  eb.path.node_specs = {pole_spec(1, b)};
  const auto eb_out = state.GenerateFromBackboneSpec(eb);
  if (!eb_out.ok || eb_out.value.generated_pole_ids.size() != 1 || eb_out.value.generated_span_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId e = eb_out.value.generated_pole_ids.front();
  const wire::core::SavedBackboneNode* node_b = state.view().backbone_node_for_pole(b);
  const wire::core::SavedBackboneNode* node_d = state.view().backbone_node_for_pole(d);
  const wire::core::SavedBackboneNode* node_e = state.view().backbone_node_for_pole(e);
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr) return false;
  const wire::core::ObjectId bd_edge = edge_between(state, node_b->node_id, node_d->node_id);
  const wire::core::ObjectId be_edge = edge_between(state, node_b->node_id, node_e->node_id);
  const wire::core::ObjectId bd_edge_bundle =
      edge_bundle_for_template(state, bd_edge, wire::core::BundleKind::kHighVoltage);
  const wire::core::ObjectId be_edge_bundle =
      edge_bundle_for_template(state, be_edge, wire::core::BundleKind::kHighVoltage);
  const wire::core::SavedBackboneEdgeBundle* bd_bundle = state.view().backbone_edge_bundle(bd_edge_bundle);
  const wire::core::SavedBackboneEdgeBundle* be_bundle = state.view().backbone_edge_bundle(be_edge_bundle);
  if (bd_bundle == nullptr || be_bundle == nullptr || bd_bundle->bundle_id != be_bundle->bundle_id) {
    return false;
  }
  const std::vector<PortBindingSnapshot> bd_after =
      port_binding_snapshot(state, bd_edge_bundle, node_b->node_id);
  const std::vector<PortBindingSnapshot> be_after =
      port_binding_snapshot(state, be_edge_bundle, node_b->node_id);
  const JunctionRowSnapshot snapshot = junction_snapshot(state, b);
  return snapshot.pair_rows == 2 && snapshot.open_rows == 0 &&
         has_row_key(snapshot.row_keys, false, bd_edge, be_edge) &&
         same_port_binding_snapshot(before, bd_after) &&
         same_port_binding_geometry(before, be_after, true) &&
         curve_endpoints_match_layout(state);
}

bool C795_backbone_incremental_hv_promotion_preserves_model_fixture_geometry() {
  constexpr wire::core::ModelAssemblyTemplateId kRowAssembly = 9195;
  constexpr wire::core::ModelAssemblyTemplateId kEndpointAssembly = 9196;
  wire::core::CoreState state;
  if (!register_hv_fixture_models(&state, kRowAssembly, kEndpointAssembly)) {
    return false;
  }

  auto make_hv_request = [](wire::core::BackboneSpec req) {
    for (wire::core::BackboneBundleSpec& bundle : req.bundles) {
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
  const wire::core::ObjectId b = abc.value.generated_pole_ids[1];
  const wire::core::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) return false;

  wire::core::BackboneSpec bd = line_req(state);
  bd.bundles.clear();
  add_backbone_bundle(bd, wire::core::BundleKind::kHighVoltage);
  bd = make_hv_request(bd);
  bd.path.polyline = {pole_b->world_transform.position, {12.0, -8.0, 0.0}};
  bd.path.node_specs = {pole_spec(0, b)};
  const auto bd_out = state.GenerateFromBackboneSpec(bd);
  if (!bd_out.ok || bd_out.value.generated_span_ids.size() != 3) return false;

  const wire::core::ObjectId d = bd_out.value.generated_pole_ids.front();
  const wire::core::SavedBackboneNode* node_b_before = state.view().backbone_node_for_pole(b);
  const wire::core::SavedBackboneNode* node_d_before = state.view().backbone_node_for_pole(d);
  if (node_b_before == nullptr || node_d_before == nullptr) return false;
  const wire::core::ObjectId bd_edge_before =
      edge_between(state, node_b_before->node_id, node_d_before->node_id);
  const wire::core::ObjectId bd_edge_bundle_before =
      edge_bundle_for_template(state, bd_edge_before, wire::core::BundleKind::kHighVoltage);
  const std::vector<PortBindingSnapshot> before =
      port_binding_snapshot(state, bd_edge_bundle_before, node_b_before->node_id);
  if (before.size() != 3) return false;

  const std::vector<wire::core::Transformd> row_transforms_before =
      row_fixture_transforms_on_pole(state, b);
  if (row_transforms_before.empty()) return false;

  for (const PortBindingSnapshot& snapshot : before) {
    const auto fixture = endpoint_fixture_transform(state, snapshot.port_id);
    const auto endpoint = layout_endpoint_for_port(state, snapshot.port_id);
    if (!fixture.has_value() || !endpoint.has_value()) return false;
    const wire::core::Vec3d visible_socket = hv_visible_socket(*fixture);
    if (!almost_equal(visible_socket, *endpoint, 1e-9)) return false;
  }

  wire::core::BackboneSpec eb = line_req(state);
  eb.bundles.clear();
  add_backbone_bundle(eb, wire::core::BundleKind::kHighVoltage);
  eb = make_hv_request(eb);
  eb.path.polyline = {{20.0, 0.0, 0.0}, pole_b->world_transform.position};
  eb.path.node_specs = {pole_spec(1, b)};
  const auto eb_out = state.GenerateFromBackboneSpec(eb);
  if (!eb_out.ok || eb_out.value.generated_span_ids.size() != 3) return false;

  const wire::core::ObjectId e = eb_out.value.generated_pole_ids.front();
  const wire::core::SavedBackboneNode* node_b = state.view().backbone_node_for_pole(b);
  const wire::core::SavedBackboneNode* node_d = state.view().backbone_node_for_pole(d);
  const wire::core::SavedBackboneNode* node_e = state.view().backbone_node_for_pole(e);
  if (node_b == nullptr || node_d == nullptr || node_e == nullptr) return false;
  const wire::core::ObjectId bd_edge = edge_between(state, node_b->node_id, node_d->node_id);
  const wire::core::ObjectId be_edge = edge_between(state, node_b->node_id, node_e->node_id);
  const wire::core::ObjectId bd_edge_bundle =
      edge_bundle_for_template(state, bd_edge, wire::core::BundleKind::kHighVoltage);
  const wire::core::ObjectId be_edge_bundle =
      edge_bundle_for_template(state, be_edge, wire::core::BundleKind::kHighVoltage);
  const std::vector<PortBindingSnapshot> bd_after =
      port_binding_snapshot(state, bd_edge_bundle, node_b->node_id);
  const std::vector<PortBindingSnapshot> be_after =
      port_binding_snapshot(state, be_edge_bundle, node_b->node_id);
  if (!same_port_binding_snapshot(before, bd_after) ||
      !same_port_binding_geometry(before, be_after, true)) {
    return false;
  }

  const std::vector<wire::core::Transformd> row_transforms_after =
      row_fixture_transforms_on_pole(state, b);
  if (row_transforms_after.size() != row_transforms_before.size()) {
    return false;
  }
  for (const PortBindingSnapshot& snapshot : before) {
    const auto fixture = endpoint_fixture_transform(state, snapshot.port_id);
    const auto endpoint = layout_endpoint_for_port(state, snapshot.port_id);
    if (!fixture.has_value() || !endpoint.has_value()) return false;
    const wire::core::Vec3d visible_socket = hv_visible_socket(*fixture);
    if (!almost_equal(visible_socket, *endpoint, 1e-9)) {
      return false;
    }
  }

  std::array<const wire::core::VisualCurvePart*, 3> patches{nullptr, nullptr, nullptr};
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != wire::core::VisualCurvePartKind::kNodePatch ||
        part.source_node_id != node_b->node_id ||
        part.bundle_template_id != wire::core::kDefaultHighVoltageBundleTemplateId ||
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
  using wire::core::Transformd;
  using wire::core::Vec3d;
  using wire::core::generation::backbone::MountGraphNode;
  using wire::core::generation::backbone::MountGraphSocket;
  using wire::core::generation::backbone::MountRefKind;
  using wire::core::generation::backbone::resolve_mount_node;
  using wire::core::generation::backbone::resolve_mount_socket;

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
  using wire::core::BundleTemplate;
  using wire::core::Vec3d;
  using wire::core::generation::backbone::ModelPlacementOrientationPolicy;
  using wire::core::generation::backbone::ModelPlacementRuleKind;
  using wire::core::generation::backbone::interval_anchor_frames;
  using wire::core::generation::backbone::placement_rules_from_bundle_template;

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

  const std::vector<wire::core::generation::backbone::SpanAnchorFrame> anchors =
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
      repo_root() / "core" / "src" / "generation" / "backbone" / "model_assembly.cpp";
  const std::filesystem::path mount_header =
      repo_root() / "core" / "src" / "generation" / "backbone" / "mount_graph.hpp";
  const std::filesystem::path rule_source =
      repo_root() / "core" / "src" / "generation" / "backbone" / "model_placement_rules.cpp";
  const std::filesystem::path models_doc = repo_root() / "docs" / "models.md";
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
        binding.row_key.edge_id == wire::core::kInvalidObjectId ||
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
  if (middle_ports.size() != 2 || middle_ports[0] == middle_ports[1]) return false;
  const wire::core::Port* a = state.view().ports().find(middle_ports[0]);
  const wire::core::Port* b_port = state.view().ports().find(middle_ports[1]);
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
  if (middle_ports.size() != 2 || middle_ports[0] == middle_ports[1]) return false;
  const wire::core::Port* a = state.view().ports().find(middle_ports[0]);
  const wire::core::Port* b_port = state.view().ports().find(middle_ports[1]);
  return a != nullptr && b_port != nullptr &&
         a->world_position.x == b_port->world_position.x &&
         a->world_position.y == b_port->world_position.y &&
         a->world_position.z == b_port->world_position.z;
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

bool C789_backbone_multi_route_same_band_rows_keep_spacing() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId junction = first.value.generated_pole_ids[1];
  const wire::core::Pole* pole = state.view().poles().find(junction);
  const auto pole_type_it =
      pole == nullptr ? state.view().pole_types().end() : state.view().pole_types().find(pole->pole_type_id);
  if (pole == nullptr || pole_type_it == state.view().pole_types().end()) {
    return false;
  }

  std::vector<std::pair<wire::core::ObjectId, wire::core::Vec3d>> before_ports{};
  for (const wire::core::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    const wire::core::Port* port = state.view().ports().find(binding.port_id);
    if (port != nullptr && port->owner_pole_id == junction &&
        binding.bundle_template_id == wire::core::kDefaultHighVoltageBundleTemplateId) {
      before_ports.push_back({port->id, port->world_position});
    }
  }
  if (before_ports.empty()) {
    return false;
  }

  wire::core::BackboneSpec branch = hv_branch_req(state, junction, pole->world_transform.position);
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok || second.value.generated_span_ids.size() != 3) {
    return false;
  }

  double min_same_band_distance = std::numeric_limits<double>::infinity();
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    for (wire::core::ObjectId port_id : {span->port_a_id, span->port_b_id}) {
      const wire::core::Port* new_port = state.view().ports().find(port_id);
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
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const wire::core::ObjectId next_id_before = state.next_id();
  const CoreCounts counts_before = snapshot_counts(state);
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const wire::core::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }

  wire::core::BackboneSpec duplicate = line_req(state);
  duplicate.path.polyline = {pole_b->world_transform.position, {12.0, 8.0, 0.0}};
  duplicate.path.node_specs.clear();
  wire::core::instrumentation::reset();
  const auto rejected = state.GenerateFromBackboneSpec(duplicate);
  const wire::core::instrumentation::Counters counters =
      wire::core::instrumentation::snapshot();

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
  wire::core::CoreState state;
  for (int route = 0; route < 12; ++route) {
    wire::core::BackboneSpec req = line_req(state);
    const double y = static_cast<double>(route) * 4.0;
    req.path.polyline = {{0.0, y, 0.0}, {12.0, y, 0.0}};
    const auto out = state.GenerateFromBackboneSpec(req);
    if (!out.ok) {
      return false;
    }
  }

  wire::core::instrumentation::reset();
  wire::core::BackboneSpec add = line_req(state);
  add.path.polyline = {{0.0, 60.0, 0.0}, {12.0, 60.0, 0.0}};
  const auto out = state.GenerateFromBackboneSpec(add);
  const wire::core::instrumentation::Counters counters =
      wire::core::instrumentation::snapshot();

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
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId junction = first.value.generated_pole_ids[1];
  const wire::core::Pole* pole = state.view().poles().find(junction);
  if (pole == nullptr) {
    return false;
  }

  std::vector<std::pair<wire::core::ObjectId, wire::core::Vec3d>> existing_ports{};
  for (const wire::core::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    const wire::core::Port* port = state.view().ports().find(binding.port_id);
    if (port != nullptr && port->owner_pole_id == junction &&
        binding.bundle_template_id == wire::core::kDefaultHighVoltageBundleTemplateId &&
        std::none_of(existing_ports.begin(), existing_ports.end(), [&](const auto& item) {
          return item.first == port->id;
        })) {
      existing_ports.push_back({port->id, port->world_position});
    }
  }
  if (existing_ports.size() != 6) {
    return false;
  }

  wire::core::BackboneSpec branch = hv_branch_req(state, junction, pole->world_transform.position);
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok || second.value.generated_span_ids.size() != 3) {
    return false;
  }

  double existing_height = 0.0;
  for (const auto& item : existing_ports) {
    const wire::core::Port* port = state.view().ports().find(item.first);
    if (port == nullptr || !almost_equal(port->world_position, item.second, 1e-9)) {
      return false;
    }
    existing_height += port->world_position.z;
  }
  existing_height /= static_cast<double>(existing_ports.size());

  std::vector<wire::core::ObjectId> new_junction_ports{};
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    for (wire::core::ObjectId port_id : {span->port_a_id, span->port_b_id}) {
      const wire::core::Port* port = state.view().ports().find(port_id);
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
  for (wire::core::ObjectId port_id : new_junction_ports) {
    const wire::core::Port* port = state.view().ports().find(port_id);
    if (port == nullptr) {
      return false;
    }
    new_height += port->world_position.z;
  }
  new_height /= static_cast<double>(new_junction_ports.size());

  return std::abs(new_height - existing_height) + 1e-9 >= 0.5;
}

bool C796_backbone_incremental_explicit_placement_height_is_not_row_reflowed() {
  auto explicit_bundle = [](const wire::core::CoreState& state,
                            wire::core::BundleKind kind,
                            std::uint64_t placement_key,
                            int count,
                            double height_m,
                            double lateral_m,
                            double spacing_m) {
    const wire::core::BundleTemplateId template_id =
        wire::core::DefaultBundleTemplateId(kind);
    const wire::core::BundleTemplate& tmpl = state.view().bundle_templates().at(template_id);
    wire::core::BackboneBundleSpec bundle{};
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
  auto default_mixed_req = [&](wire::core::CoreState& state) {
    wire::core::BackboneSpec req = poly3_req(state);
    req.bundles.clear();
    req.bundles.push_back(
        explicit_bundle(state, wire::core::BundleKind::kHighVoltage, 3101, 3, 9.2, -0.2, 0.45));
    req.bundles.push_back(
        explicit_bundle(state, wire::core::BundleKind::kLowVoltage, 3201, 1, 7.35, 0.0, 0.30));
    req.bundles.push_back(
        explicit_bundle(state, wire::core::BundleKind::kLowVoltage, 3202, 1, 7.35, 0.0, 0.30));
    req.bundles.push_back(
        explicit_bundle(state, wire::core::BundleKind::kCommunication, 3401, 1, 6.4, 0.0, 0.30));
    req.bundles.push_back(
        explicit_bundle(state, wire::core::BundleKind::kOptical, 3501, 1, 6.0, 0.0, 0.30));
    return req;
  };
  auto branch_from = [&](wire::core::CoreState& state, wire::core::ObjectId pole_id,
                         const wire::core::Vec3d& pole_pos, const wire::core::Vec3d& end) {
    wire::core::BackboneSpec req = default_mixed_req(state);
    req.path.polyline = {pole_pos, end};
    req.path.node_specs = {pole_spec(0, pole_id)};
    req.node_bundle_modes.clear();
    for (const wire::core::BackboneBundleSpec& bundle : req.bundles) {
      wire::core::BackboneSpec::NodeBundleModeSpec mode{};
      mode.point_index = 0;
      mode.bundle_template_id = bundle.bundle_template_id;
      mode.mode = wire::core::BundleNodeMode::kPassThrough;
      req.node_bundle_modes.push_back(mode);
    }
    return req;
  };
  auto completion_to = [&](wire::core::CoreState& state, wire::core::ObjectId pole_id,
                           const wire::core::Vec3d& pole_pos, const wire::core::Vec3d& start) {
    wire::core::BackboneSpec req = default_mixed_req(state);
    req.path.polyline = {start, pole_pos};
    req.path.node_specs = {pole_spec(1, pole_id)};
    req.node_bundle_modes.clear();
    for (const wire::core::BackboneBundleSpec& bundle : req.bundles) {
      wire::core::BackboneSpec::NodeBundleModeSpec mode{};
      mode.point_index = 1;
      mode.bundle_template_id = bundle.bundle_template_id;
      mode.mode = wire::core::BundleNodeMode::kPassThrough;
      req.node_bundle_modes.push_back(mode);
    }
    return req;
  };
  const auto bundle_for_binding = [](const wire::core::CoreState& state,
                                     const wire::core::SavedBackbonePortBinding& binding)
      -> const wire::core::Bundle* {
    const wire::core::SavedBackboneEdgeBundle* edge_bundle =
        state.view().backbone_edge_bundle(binding.edge_bundle_id);
    return edge_bundle == nullptr ? nullptr : state.view().bundles().find(edge_bundle->bundle_id);
  };
  const auto row_key_equal = [](const wire::core::SavedBackboneRowKey& a,
                                const wire::core::SavedBackboneRowKey& b) {
    return a == b;
  };
  const auto local_height_for_binding = [](const wire::core::CoreState& state,
                                           const wire::core::SavedBackbonePortBinding& binding)
      -> std::optional<double> {
    const wire::core::Port* port = state.view().ports().find(binding.port_id);
    const wire::core::Pole* pole =
        port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
    if (port == nullptr || pole == nullptr) {
      return std::nullopt;
    }
    const wire::core::PoleFrame frame =
        wire::core::BuildPoleFrame(pole->world_transform, binding.layout_yaw_deg);
    return wire::core::WorldPointToLocal(frame, port->world_position).z;
  };

  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(default_mixed_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) return false;
  const wire::core::ObjectId junction = first.value.generated_pole_ids[1];
  const wire::core::Pole* pole = state.view().poles().find(junction);
  const wire::core::SavedBackboneNode* node = state.view().backbone_node_for_pole(junction);
  if (pole == nullptr || node == nullptr) return false;

  std::unordered_map<wire::core::ObjectId, wire::core::Vec3d> existing_positions{};
  std::unordered_map<std::uint64_t, std::vector<wire::core::ObjectId>> existing_by_key{};
  std::optional<double> existing_row_delta{};
  for (const wire::core::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != node->node_id) continue;
    const wire::core::Port* port = state.view().ports().find(binding.port_id);
    const wire::core::Bundle* bundle = bundle_for_binding(state, binding);
    const auto local_height = local_height_for_binding(state, binding);
    if (port == nullptr || bundle == nullptr || !local_height.has_value()) return false;
    existing_positions.emplace(port->id, port->world_position);
    std::vector<wire::core::ObjectId>& key_ports = existing_by_key[bundle->placement_key];
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

  wire::core::BackboneSpec hv_only = line_req(state);
  hv_only.bundles.clear();
  hv_only.bundles.push_back(
      explicit_bundle(state, wire::core::BundleKind::kHighVoltage, 3101, 3, 9.2, -0.2, 0.45));
  hv_only.path.polyline = {pole->world_transform.position, {12.0, -6.0, 0.0}};
  hv_only.path.node_specs = {pole_spec(0, junction)};
  const auto hv_only_out = state.GenerateFromBackboneSpec(hv_only);
  if (!hv_only_out.ok) return false;
  for (const auto& [port_id, position] : existing_positions) {
    const wire::core::Port* port = state.view().ports().find(port_id);
    if (port == nullptr || !almost_equal(port->world_position, position, 1e-9)) {
      return false;
    }
  }

  wire::core::CoreState pair_state;
  const auto pair_first = pair_state.GenerateFromBackboneSpec(default_mixed_req(pair_state));
  if (!pair_first.ok || pair_first.value.generated_pole_ids.size() != 3) return false;
  const wire::core::ObjectId pair_junction = pair_first.value.generated_pole_ids[1];
  const wire::core::Pole* pair_pole = pair_state.view().poles().find(pair_junction);
  const wire::core::SavedBackboneNode* pair_node =
      pair_state.view().backbone_node_for_pole(pair_junction);
  if (pair_pole == nullptr || pair_node == nullptr) return false;

  std::unordered_set<wire::core::ObjectId> original_pair_ports{};
  for (const wire::core::SavedBackbonePortBinding& binding : pair_state.view().backbone().port_bindings) {
    if (binding.row_key.node_id == pair_node->node_id) {
      original_pair_ports.insert(binding.port_id);
    }
  }

  const auto bd_out =
      pair_state.GenerateFromBackboneSpec(branch_from(pair_state, pair_junction,
                                                      pair_pole->world_transform.position,
                                                      {12.0, -8.0, 0.0}));
  if (!bd_out.ok || bd_out.value.generated_span_ids.size() != 7) return false;
  const wire::core::SavedBackboneNode* pair_node_after_bd =
      pair_state.view().backbone_node_for_pole(pair_junction);
  if (pair_node_after_bd == nullptr) return false;

  std::optional<wire::core::SavedBackboneRowKey> new_open_row{};
  std::optional<double> new_row_delta{};
  std::unordered_map<wire::core::ObjectId, double> bd_local_height_by_port{};
  std::unordered_map<std::uint64_t, std::vector<wire::core::ObjectId>> new_ports_by_key{};
  for (const wire::core::SavedBackbonePortBinding& binding : pair_state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != pair_node_after_bd->node_id ||
        original_pair_ports.find(binding.port_id) != original_pair_ports.end()) {
      continue;
    }
    const wire::core::Bundle* bundle = bundle_for_binding(pair_state, binding);
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
  const wire::core::ObjectId e = eb_out.value.generated_pole_ids.front();
  const wire::core::SavedBackboneNode* pair_node_after =
      pair_state.view().backbone_node_for_pole(pair_junction);
  const wire::core::SavedBackboneNode* e_node = pair_state.view().backbone_node_for_pole(e);
  if (pair_node_after == nullptr || e_node == nullptr) return false;
  std::size_t continued_bindings = 0;
  for (const wire::core::SavedBackbonePortBinding& binding : pair_state.view().backbone().port_bindings) {
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
  std::unordered_set<wire::core::ObjectId> b_ports_before{};
  for (const wire::core::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    const wire::core::Port* port = state.view().ports().find(binding.port_id);
    if (port != nullptr && port->owner_pole_id == b) {
      b_ports_before.insert(port->id);
    }
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok) {
    return false;
  }
  std::unordered_set<wire::core::ObjectId> b_ports_after{};
  std::vector<double> levels{};
  for (const wire::core::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    const wire::core::Port* port = state.view().ports().find(binding.port_id);
    if (port == nullptr || port->owner_pole_id != b) {
      continue;
    }
    b_ports_after.insert(port->id);
    if (std::none_of(levels.begin(), levels.end(),
                     [&](double z) { return std::abs(z - port->world_position.z) <= 1e-9; })) {
      levels.push_back(port->world_position.z);
    }
  }
  std::sort(levels.begin(), levels.end());
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  std::string emit_ports_body;
  if (!file_text(source, &cpp) ||
      !function_body(cpp, "EditResult<bool> pipeline::emit_ports(topo* made, const pairs& ps, ChangeSet* changes)",
                     &emit_ports_body)) {
    return false;
  }
  return !second.value.generated_span_ids.empty() &&
         b_ports_after.size() == b_ports_before.size() + second.value.generated_span_ids.size() &&
         levels.size() == 2 &&
         std::abs((levels[1] - levels[0]) - 0.5) <= 1e-9 &&
         contains_text(emit_ports_body, "row_height_offsets(ps)") &&
         contains_text(emit_ports_body, "canonical row reflow requires moving manual ports") &&
         !contains_text(emit_ports_body, "if (r.id >= active_rows.size() || !active_rows[r.id])");
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
  std::vector<const wire::core::SavedBackbonePortBinding*> middle{};
  for (const auto& binding : state.view().backbone().port_bindings) {
    const wire::core::Port* port = state.view().ports().find(binding.port_id);
    if (port != nullptr && port->owner_pole_id == b) middle.push_back(&binding);
  }
  if (middle.size() != 2 || middle[0]->port_id == middle[1]->port_id) return false;
  const wire::core::Port* a = state.view().ports().find(middle[0]->port_id);
  const wire::core::Port* c = state.view().ports().find(middle[1]->port_id);
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
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok) {
    return false;
  }
  bool saw_binding = false;
  for (const wire::core::Port& port : state.view().ports().items()) {
    const std::vector<const wire::core::SavedBackbonePortBinding*> bindings =
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
      !function_body(cpp, "EditResult<bool> pipeline::save_graph(const topo& made, const pairs& ps,", &save_body)) {
    return false;
  }
  return contains_text(ref_body, "edge.saved == kInvalidObjectId") && !contains_text(ref_body, "saved_edge_for") &&
         contains_text(save_body, "context link saved edge missing");
}

bool C797_backbone_row_continuity_records_route_and_promotion_lanes() {
  auto hv_req = [](wire::core::CoreState& state, bool polyline3) {
    wire::core::BackboneSpec req = polyline3 ? poly3_req(state) : line_req(state);
    req.bundles.clear();
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    return req;
  };
  auto expect_hv_lane_continuity = [](const wire::core::CoreState& state,
                                      wire::core::ObjectId node_id,
                                      wire::core::ObjectId edge_a,
                                      wire::core::ObjectId edge_b) {
    for (std::size_t lane = 0; lane < 3; ++lane) {
      const wire::core::ObjectId edge_bundle_a = edge_bundle_for_edge_and_lane(state, edge_a, lane);
      const wire::core::ObjectId edge_bundle_b = edge_bundle_for_edge_and_lane(state, edge_b, lane);
      if (edge_bundle_a == wire::core::kInvalidObjectId ||
          edge_bundle_b == wire::core::kInvalidObjectId ||
          !has_row_continuity(state, node_id, edge_bundle_a, lane, edge_bundle_b, lane)) {
        return false;
      }
    }
    return true;
  };

  wire::core::CoreState straight;
  const auto straight_out = straight.GenerateFromBackboneSpec(hv_req(straight, false));
  if (!straight_out.ok || straight_out.value.generated_pole_ids.size() != 2) {
    return false;
  }
  for (wire::core::ObjectId pole_id : straight_out.value.generated_pole_ids) {
    const wire::core::SavedBackboneNode* node = straight.view().backbone_node_for_pole(pole_id);
    if (node == nullptr || !straight.view().backbone_row_continuities_for_node(node->node_id).empty()) {
      return false;
    }
  }

  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(hv_req(state, true));
  if (!out.ok || out.value.generated_pole_ids.size() != 3) return false;
  const wire::core::SavedBackboneNode* node_a =
      state.view().backbone_node_for_pole(out.value.generated_pole_ids[0]);
  const wire::core::SavedBackboneNode* node_b =
      state.view().backbone_node_for_pole(out.value.generated_pole_ids[1]);
  const wire::core::SavedBackboneNode* node_c =
      state.view().backbone_node_for_pole(out.value.generated_pole_ids[2]);
  if (node_a == nullptr || node_b == nullptr || node_c == nullptr) {
    return false;
  }
  const wire::core::ObjectId ab_edge = edge_between(state, node_a->node_id, node_b->node_id);
  const wire::core::ObjectId bc_edge = edge_between(state, node_b->node_id, node_c->node_id);
  const wire::core::ObjectId node_b_id = node_b->node_id;
  if (ab_edge == wire::core::kInvalidObjectId || bc_edge == wire::core::kInvalidObjectId) {
    return false;
  }
  if (!state.view().backbone_row_continuities_for_node(node_a->node_id).empty() ||
      !state.view().backbone_row_continuities_for_node(node_c->node_id).empty()) {
    return false;
  }
  if (!expect_hv_lane_continuity(state, node_b_id, ab_edge, bc_edge)) {
    return false;
  }

  const wire::core::Pole* pole_b = state.view().poles().find(out.value.generated_pole_ids[1]);
  if (pole_b == nullptr) return false;
  wire::core::BackboneSpec branch = hv_req(state, false);
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
  const wire::core::SavedBackboneNode* cross_b = cross.state.view().backbone_node_for_pole(cross.pole_b);
  const wire::core::SavedBackboneNode* cross_d = cross.state.view().backbone_node_for_pole(cross.pole_d);
  const wire::core::SavedBackboneNode* cross_e = cross.state.view().backbone_node_for_pole(cross.pole_e);
  if (cross_b == nullptr || cross_d == nullptr || cross_e == nullptr) {
    return false;
  }
  const wire::core::ObjectId cross_bd = edge_between(cross.state, cross_b->node_id, cross_d->node_id);
  const wire::core::ObjectId cross_be = edge_between(cross.state, cross_b->node_id, cross_e->node_id);
  const wire::core::ObjectId bd_bundle = edge_bundle_for_edge_and_lane(cross.state, cross_bd, 0);
  const wire::core::ObjectId be_bundle = edge_bundle_for_edge_and_lane(cross.state, cross_be, 0);
  return bd_bundle != wire::core::kInvalidObjectId &&
         be_bundle != wire::core::kInvalidObjectId &&
         has_row_continuity(cross.state, cross_b->node_id, bd_bundle, 0, be_bundle, 0);
}

bool viewer_default_t_branch_keeps_hv_and_only_flagged_lowering(bool anchor_at_end) {
  auto explicit_bundle = [](const wire::core::CoreState& state,
                            wire::core::BundleKind kind,
                            std::uint64_t placement_key,
                            int count,
                            double height_m,
                            double lateral_m,
                            double spacing_m) {
    const wire::core::BundleTemplateId template_id =
        wire::core::DefaultBundleTemplateId(kind);
    const wire::core::BundleTemplate& tmpl = state.view().bundle_templates().at(template_id);
    wire::core::BackboneBundleSpec bundle{};
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
  auto viewer_default_req = [&](wire::core::CoreState& state) {
    wire::core::BackboneSpec req = poly3_req(state);
    req.pole_type_id = 2;
    req.pole_placement.enable_tilt = true;
    req.pole_placement.max_tilt_deg = 12.0;
    req.bundles.clear();
    req.bundles.push_back(
        explicit_bundle(state, wire::core::BundleKind::kHighVoltage, 1, 3, 9.2, -0.2, 0.45));
    req.bundles.push_back(
        explicit_bundle(state, wire::core::BundleKind::kLowVoltage, 2, 1, 7.7, 0.0, 0.2));
    req.bundles.push_back(
        explicit_bundle(state, wire::core::BundleKind::kLowVoltage, 3, 1, 7.35, 0.0, 0.2));
    req.bundles.push_back(
        explicit_bundle(state, wire::core::BundleKind::kLowVoltage, 4, 1, 7.0, 0.0, 0.2));
    req.bundles.push_back(
        explicit_bundle(state, wire::core::BundleKind::kCommunication, 5, 1, 5.5, 0.0, 0.2));
    req.bundles.push_back(
        explicit_bundle(state, wire::core::BundleKind::kOptical, 6, 1, 5.3, 0.0, 0.2));
    return req;
  };
  auto span_template = [](const wire::core::CoreState& state, wire::core::ObjectId span_id) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    const wire::core::Bundle* bundle = span == nullptr ? nullptr : state.view().bundles().find(span->bundle_id);
    return bundle == nullptr ? wire::core::kInvalidBundleTemplateId : bundle->bundle_template_id;
  };
  auto endpoint_stays_at_port = [](const wire::core::CoreState& state,
                                   const wire::core::LayoutEndpoint& endpoint) {
    const wire::core::Port* port = state.view().ports().find(endpoint.port_id);
    return port != nullptr &&
           !endpoint.default_lower_required &&
           !endpoint.lower_required &&
           endpoint.branch_down_offset_m <= 1e-9 &&
           almost_equal(endpoint.support_world.z, port->world_position.z, 1e-9) &&
           almost_equal(endpoint.endpoint_world.z, port->world_position.z, 1e-9);
  };
  auto endpoint_follows_template_policy = [&](const wire::core::CoreState& state,
                                              const wire::core::LayoutEndpoint& endpoint,
                                              bool expect_lowered) {
    const wire::core::Port* port = state.view().ports().find(endpoint.port_id);
    if (port == nullptr) return false;
    if (!expect_lowered) {
      return endpoint_stays_at_port(state, endpoint);
    }
    const wire::core::Pole* pole = state.view().poles().find(port->owner_pole_id);
    if (pole == nullptr) return false;
    const double layout_yaw = state.effective_port_layout_yaw_deg(*pole, port->id, port->category);
    const wire::core::PoleFrame frame = wire::core::BuildPoleFrame(pole->world_transform, layout_yaw);
    const wire::core::Vec3d port_local = wire::core::WorldPointToLocal(frame, port->world_position);
    const wire::core::Vec3d endpoint_local = wire::core::WorldPointToLocal(frame, endpoint.endpoint_world);
    return endpoint.default_lower_required &&
           endpoint.lower_required &&
           endpoint.branch_down_offset_m > 1e-9 &&
           almost_equal(endpoint_local.z, port_local.z - endpoint.branch_down_offset_m, 1e-9) &&
           almost_equal(endpoint.endpoint_world, endpoint.support_world, 1e-9);
  };

  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(viewer_default_req(state));
  WIRE_TEST_EXPECT(first.ok, first.error);
  WIRE_TEST_EXPECT(first.value.generated_pole_ids.size() == 3, "viewer default base did not generate 3 poles");
  WIRE_TEST_EXPECT(first.value.generated_span_ids.size() == 16, "viewer default base did not generate 16 spans");
  const wire::core::ObjectId junction = first.value.generated_pole_ids[1];
  std::unordered_map<wire::core::ObjectId, wire::core::Vec3d> port_positions_before{};
  for (const wire::core::Port& port : state.view().ports().items()) {
    port_positions_before.emplace(port.id, port.world_position);
  }
  const wire::core::Pole* pole = state.view().poles().find(junction);
  WIRE_TEST_EXPECT(pole != nullptr, "junction pole is missing");
  const wire::core::SavedBackboneNode* junction_node = state.view().backbone_node_for_pole(junction);
  WIRE_TEST_EXPECT(junction_node != nullptr, "junction backbone node is missing");
  const wire::core::SavedBackboneEdge* incident = nullptr;
  for (const wire::core::SavedBackboneEdge& edge : state.view().backbone().edges) {
    if (edge.node_a == junction_node->node_id || edge.node_b == junction_node->node_id) {
      incident = &edge;
      break;
    }
  }
  WIRE_TEST_EXPECT(incident != nullptr, "junction has no incident saved backbone edge");
  const wire::core::ObjectId peer_node_id =
      incident->node_a == junction_node->node_id ? incident->node_b : incident->node_a;
  const wire::core::SavedBackboneNode* peer_node = state.view().backbone_node(peer_node_id);
  WIRE_TEST_EXPECT(peer_node != nullptr, "peer backbone node is missing");

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = wire::core::kInvalidObjectId;
  pick.hit_pos_world = {pole->world_transform.position.x + 0.12,
                        pole->world_transform.position.y - 0.10,
                        9.2};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = junction_node->node_id;
  pick.segment_node_b_id = peer_node->node_id;
  pick.segment_endpoint_a_world = junction_node->position;
  pick.segment_endpoint_b_world = peer_node->position;
  wire::core::ResolveBranchPickOptions pick_options{};
  pick_options.selected_bundle_template_ids = {
      wire::core::kDefaultHighVoltageBundleTemplateId,
      wire::core::kDefaultLowVoltageBundleTemplateId,
      wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication),
      wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kOptical),
  };
  const auto resolved = state.ResolveBranchPick(pick, pick_options);
  WIRE_TEST_EXPECT(resolved.ok, resolved.error);
  WIRE_TEST_EXPECT(resolved.value.support_kind == wire::core::SupportKind::kPole,
                   "segment endpoint snap did not resolve to pole");
  WIRE_TEST_EXPECT(resolved.value.resolved_node_id == junction_node->node_id,
                   "segment endpoint snap resolved to the wrong backbone node");
  WIRE_TEST_EXPECT(resolved.value.snapped_from_segment_endpoint,
                   "segment endpoint snap flag was not set");

  wire::core::BackboneSpec branch = viewer_default_req(state);
  const wire::core::Vec3d new_point{12.0, -8.0, 0.0};
  branch.path.polyline = anchor_at_end
      ? std::vector<wire::core::Vec3d>{new_point, resolved.value.position}
      : std::vector<wire::core::Vec3d>{resolved.value.position, new_point};
  branch.path.node_specs = {pole_spec(anchor_at_end ? 1 : 0, resolved.value.resolved_node_id)};
  const auto out = state.GenerateFromBackboneSpec(branch);
  WIRE_TEST_EXPECT(out.ok, out.error);
  WIRE_TEST_EXPECT(out.value.generated_pole_ids.size() == 1, "viewer default branch did not generate one pole");
  WIRE_TEST_EXPECT(out.value.generated_span_ids.size() == 8, "viewer default branch did not generate 8 spans");

  std::unordered_map<wire::core::BundleTemplateId, std::size_t> generated_by_template{};
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    ++generated_by_template[span_template(state, span_id)];
  }
  if (generated_by_template[wire::core::kDefaultHighVoltageBundleTemplateId] != 3 ||
      generated_by_template[wire::core::kDefaultLowVoltageBundleTemplateId] != 3 ||
      generated_by_template[wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)] != 1 ||
      generated_by_template[wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kOptical)] != 1) {
    test_registry::SetFailureReason("viewer default branch generated wrong template span counts");
    return false;
  }

  const std::unordered_set<wire::core::ObjectId> branch_span_ids(
      out.value.generated_span_ids.begin(), out.value.generated_span_ids.end());
  for (const wire::core::Span& span : state.view().spans().items()) {
    const wire::core::Bundle* bundle = state.view().bundles().find(span.bundle_id);
    const auto template_it = bundle == nullptr
        ? state.view().bundle_templates().end()
        : state.view().bundle_templates().find(bundle->bundle_template_id);
    WIRE_TEST_EXPECT(bundle != nullptr && template_it != state.view().bundle_templates().end(),
                     "span bundle/template is missing");
    const wire::core::SpanLayoutView layout = state.span_layout(span.id);
    WIRE_TEST_EXPECT(layout.has_layout(), "span layout is missing");
    const bool flagged = branch_span_ids.contains(span.id) &&
                         template_it->second.enable_branch_down_offset &&
                         template_it->second.branch_endpoint_offset_m < -1e-9;
    const wire::core::Port* start_port = state.view().ports().find(layout.entry->start.port_id);
    const wire::core::Port* end_port = state.view().ports().find(layout.entry->end.port_id);
    const bool start_at_junction = start_port != nullptr && start_port->owner_pole_id == junction;
    const bool end_at_junction = end_port != nullptr && end_port->owner_pole_id == junction;
    if (!endpoint_follows_template_policy(state, layout.entry->start, flagged && start_at_junction) ||
        !endpoint_follows_template_policy(state, layout.entry->end, flagged && end_at_junction)) {
      test_registry::SetFailureReason("branch endpoint lowering did not follow template policy");
      return false;
    }
  }
  for (const auto& [port_id, before_position] : port_positions_before) {
    const wire::core::Port* after_port = state.view().ports().find(port_id);
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
  wire::core::CoreState state;
  const wire::core::BundleTemplateId hv_id =
      wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kHighVoltage);
  const wire::core::BundleTemplateId lv_id =
      wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage);
  wire::core::BundleTemplate hv = state.view().bundle_templates().at(hv_id);
  hv.enable_branch_down_offset = false;
  hv.branch_endpoint_offset_m = -0.275;
  wire::core::BundleTemplate lv = state.view().bundle_templates().at(lv_id);
  lv.enable_branch_down_offset = true;
  lv.branch_endpoint_offset_m = -0.325;
  if (!state.UpdateBundleTemplate(hv).ok || !state.UpdateBundleTemplate(lv).ok) {
    return false;
  }

  wire::core::BackboneSpec base = poly3_req(state);
  add_backbone_bundle(base, wire::core::BundleKind::kHighVoltage);
  const auto first = state.GenerateFromBackboneSpec(base);
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId junction = first.value.generated_pole_ids[1];
  const wire::core::Pole* pole = state.view().poles().find(junction);
  const wire::core::SavedBackboneNode* node = state.view().backbone_node_for_pole(junction);
  if (pole == nullptr || node == nullptr) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  branch.path.polyline = {pole->world_transform.position, {20.0, -8.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, node->node_id)};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.generated_span_ids.size() != 4) {
    return false;
  }

  std::size_t hv_spans = 0;
  std::size_t lv_spans = 0;
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    const wire::core::Bundle* bundle = span == nullptr ? nullptr : state.view().bundles().find(span->bundle_id);
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
    const wire::core::SpanLayoutView layout = state.span_layout(span_id);
    if (!layout.has_layout()) {
      return false;
    }
    const wire::core::Port* start_port = state.view().ports().find(layout.entry->start.port_id);
    const wire::core::Port* end_port = state.view().ports().find(layout.entry->end.port_id);
    const bool start_at_junction = start_port != nullptr && start_port->owner_pole_id == junction;
    const bool end_at_junction = end_port != nullptr && end_port->owner_pole_id == junction;
    const wire::core::LayoutEndpoint& endpoint =
        start_at_junction ? layout.entry->start : layout.entry->end;
    if ((!start_at_junction && !end_at_junction) || (is_hv && endpoint.default_lower_required) ||
        (is_lv && (!endpoint.default_lower_required || endpoint.branch_down_offset_m <= 0.1))) {
      return false;
    }
  }
  return hv_spans == 3 && lv_spans == 1 && curve_endpoints_match_layout(state);
}

bool C809_backbone_incremental_rows_use_one_support_level_per_pair() {
  wire::core::CoreState state;
  const wire::core::BundleTemplate& hv =
      state.view().bundle_templates().at(
          wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kHighVoltage));
  const double step = std::max(0.0, -hv.branch_endpoint_offset_m);
  WIRE_TEST_EXPECT(hv.enable_branch_down_offset, "default HV branch-down offset flag is disabled");
  WIRE_TEST_EXPECT(step > 0.0, "default HV branch-down offset step is not positive");
  const auto expected = [&](const wire::core::CoreState& current,
                            wire::core::ObjectId pole_id,
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
  const auto use_explicit_hv_placement = [](wire::core::BackboneSpec* request) {
    if (request == nullptr || request->bundles.size() != 1) {
      return false;
    }
    wire::core::BackboneBundleSpec& bundle = request->bundles.front();
    bundle.placement_key = 1;
    bundle.placement_explicit = true;
    bundle.count = 3;
    bundle.height_m = 9.2;
    bundle.lateral_m = -0.2;
    bundle.spacing_m = 0.45;
    return true;
  };
  const auto add_hv_edge = [&](wire::core::ObjectId junction,
                               const wire::core::Vec3d& point,
                               bool junction_at_end) {
    const wire::core::Pole* pole = state.view().poles().find(junction);
    if (pole == nullptr) {
      return false;
    }
    wire::core::BackboneSpec request = line_req(state);
    request.bundles.clear();
    add_backbone_bundle(request, wire::core::BundleKind::kHighVoltage);
    if (!use_explicit_hv_placement(&request)) {
      return false;
    }
    request.path.polyline = junction_at_end
                                ? std::vector<wire::core::Vec3d>{point, pole->world_transform.position}
                                : std::vector<wire::core::Vec3d>{pole->world_transform.position, point};
    request.path.node_specs = {pole_spec(junction_at_end ? 1 : 0, junction)};
    const auto result = state.GenerateFromBackboneSpec(request);
    return result.ok && result.value.generated_pole_ids.size() == 1;
  };

  wire::core::BackboneSpec base_request = hv_poly3_req(state);
  WIRE_TEST_EXPECT(use_explicit_hv_placement(&base_request), "failed to apply explicit HV placement to base request");
  const auto base = state.GenerateFromBackboneSpec(base_request);
  WIRE_TEST_EXPECT(base.ok, base.error);
  WIRE_TEST_EXPECT(base.value.generated_pole_ids.size() == 3, "base HV polyline did not generate 3 poles");
  const wire::core::ObjectId junction = base.value.generated_pole_ids[1];
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
  wire::core::CoreState loaded;
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
  wire::core::CoreState legacy_loaded;
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
    wire::core::CoreState state;
    const auto out = state.GenerateFromBackboneSpec(line_req(state));
    if (!out.ok || !row_continuity_graph_lint_passes(state)) return false;
    std::string invariant_error{};
    WIRE_TEST_EXPECT(backbone_common_invariants_pass(state, &invariant_error), invariant_error);
  }
  {
    wire::core::CoreState state;
    const auto out = state.GenerateFromBackboneSpec(hv_poly3_req(state));
    if (!out.ok || !row_continuity_graph_lint_passes(state)) return false;
    std::string invariant_error{};
    WIRE_TEST_EXPECT(backbone_common_invariants_pass(state, &invariant_error), invariant_error);
  }
  {
    wire::core::CoreState state;
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
  if (!file_text(repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp", &cpp)) {
    return false;
  }
  std::string regenerate{};
  std::string state{};
  std::string update{};
  if (!file_text(repo_root() / "core" / "src" / "generation" / "backbone" / "regenerate.cpp", &regenerate) ||
      !file_text(repo_root() / "core" / "src" / "state" / "state.cpp", &state) ||
      !file_text(repo_root() / "core" / "src" / "state" / "template" / "update.cpp", &update)) {
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
  wire::core::CoreState state;
  wire::core::BackboneSpec existing = line_req(state);
  existing.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}};
  const auto bd = state.GenerateFromBackboneSpec(existing);
  if (!bd.ok || bd.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const wire::core::ObjectId junction = bd.value.generated_pole_ids.front();
  const wire::core::Pole* pole = state.view().poles().find(junction);
  if (pole == nullptr) {
    return false;
  }

  wire::core::BackboneSpec two_edges = poly3_req(state);
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
  const wire::core::Vec3d d{12.0, -8.0, 0.0};
  const wire::core::Vec3d e{13.0, -10.0, 0.0};
  const auto hv_request = [](wire::core::CoreState& state) {
    wire::core::BackboneSpec request = line_req(state);
    request.bundles.clear();
    add_backbone_bundle(request, wire::core::BundleKind::kHighVoltage);
    return request;
  };

  wire::core::CoreState one_shot;
  const auto one_base =
      one_shot.GenerateFromBackboneSpec(hv_poly3_req(one_shot));
  WIRE_TEST_EXPECT(one_base.ok, one_base.error);
  WIRE_TEST_EXPECT(one_base.value.generated_pole_ids.size() == 3, "one-shot base did not generate 3 poles");
  const wire::core::ObjectId one_b = one_base.value.generated_pole_ids[1];
  const wire::core::Pole* one_pole = one_shot.view().poles().find(one_b);
  WIRE_TEST_EXPECT(one_pole != nullptr, "one-shot junction pole is missing");
  wire::core::BackboneSpec one_pair = hv_request(one_shot);
  one_pair.path.polyline = {d, one_pole->world_transform.position, e};
  one_pair.path.node_specs = {pole_spec(1, one_b)};
  const auto one_added = one_shot.GenerateFromBackboneSpec(one_pair);
  WIRE_TEST_EXPECT(one_added.ok, one_added.error);
  WIRE_TEST_EXPECT(one_added.value.generated_pole_ids.size() == 2, "one-shot sharp pair did not generate 2 poles");

  wire::core::CoreState incremental;
  const auto incremental_base =
      incremental.GenerateFromBackboneSpec(hv_poly3_req(incremental));
  WIRE_TEST_EXPECT(incremental_base.ok, incremental_base.error);
  WIRE_TEST_EXPECT(incremental_base.value.generated_pole_ids.size() == 3,
                   "incremental base did not generate 3 poles");
  const wire::core::ObjectId incremental_b =
      incremental_base.value.generated_pole_ids[1];
  const wire::core::Pole* incremental_pole =
      incremental.view().poles().find(incremental_b);
  WIRE_TEST_EXPECT(incremental_pole != nullptr, "incremental junction pole is missing");
  wire::core::BackboneSpec first = hv_request(incremental);
  first.path.polyline = {incremental_pole->world_transform.position, d};
  first.path.node_specs = {pole_spec(0, incremental_b)};
  const auto first_added = incremental.GenerateFromBackboneSpec(first);
  WIRE_TEST_EXPECT(first_added.ok, first_added.error);
  WIRE_TEST_EXPECT(first_added.value.generated_pole_ids.size() == 1,
                   "incremental first sharp edge did not generate 1 pole");
  wire::core::BackboneSpec second = hv_request(incremental);
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
                .at(wire::core::DefaultBundleTemplateId(
                    wire::core::BundleKind::kHighVoltage))
                .branch_endpoint_offset_m);
  const std::vector<std::pair<int, double>> expected{
      {0, 0.0}, {0, 0.0}, {1, step}, {1, step}};
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
  constexpr wire::core::ModelAssemblyTemplateId kRowAssembly = 9816;
  wire::core::CoreState state;
  wire::core::ModelAssemblyTemplate row_assembly{};
  row_assembly.id = kRowAssembly;
  wire::core::ModelAssemblyPart row_part{};
  row_part.part_id = 1;
  row_part.model_key = "c816_crossarm";
  row_part.descriptor_version = 1;
  row_part.fit_mode = wire::core::ModelFitMode::kRigid;
  row_assembly.parts.push_back(row_part);
  if (!state.RegisterModelAssemblyTemplate(row_assembly).ok) {
    return false;
  }
  const wire::core::BundleTemplate& hv =
      state.view().bundle_templates().at(
          wire::core::DefaultBundleTemplateId(
              wire::core::BundleKind::kHighVoltage));
  if (!hv.enable_branch_down_offset || hv.branch_endpoint_offset_m == 0.0) {
    return false;
  }
  wire::core::BundleTemplate hv_with_model = hv;
  hv_with_model.row_fixture_assembly_id = kRowAssembly;
  if (!state.UpdateBundleTemplate(hv_with_model).ok) {
    return false;
  }
  const auto use_explicit_hv_placement =
      [](wire::core::BackboneSpec* request) {
        if (request == nullptr || request->bundles.size() != 1) {
          return false;
        }
        wire::core::BackboneBundleSpec& bundle = request->bundles.front();
        bundle.placement_key = 1;
        bundle.placement_explicit = true;
        bundle.count = 3;
        bundle.height_m = 9.2;
        bundle.lateral_m = -0.2;
        bundle.spacing_m = 0.45;
        return true;
      };
  const auto add_hv_edge =
      [&](wire::core::ObjectId junction, const wire::core::Vec3d& point,
          bool junction_at_end) -> wire::core::ObjectId {
    const wire::core::Pole* pole = state.view().poles().find(junction);
    if (pole == nullptr) {
      return wire::core::kInvalidObjectId;
    }
    wire::core::BackboneSpec request = line_req(state);
    request.bundles.clear();
    add_backbone_bundle(request, wire::core::BundleKind::kHighVoltage);
    if (!use_explicit_hv_placement(&request)) {
      return wire::core::kInvalidObjectId;
    }
    request.path.polyline =
        junction_at_end
            ? std::vector<wire::core::Vec3d>{point,
                                             pole->world_transform.position}
            : std::vector<wire::core::Vec3d>{pole->world_transform.position,
                                             point};
    request.path.node_specs = {pole_spec(junction_at_end ? 1 : 0, junction)};
    const auto result = state.GenerateFromBackboneSpec(request);
    return result.ok && result.value.generated_pole_ids.size() == 1
               ? result.value.generated_pole_ids.front()
               : wire::core::kInvalidObjectId;
  };

  wire::core::BackboneSpec base_request = hv_poly3_req(state);
  if (!use_explicit_hv_placement(&base_request)) {
    return false;
  }
  const auto base = state.GenerateFromBackboneSpec(base_request);
  if (!base.ok || base.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId junction = base.value.generated_pole_ids[1];
  const wire::core::SavedBackboneNode* node_b =
      state.view().backbone_node_for_pole(junction);
  if (node_b == nullptr) {
    return false;
  }
  const wire::core::ObjectId pole_d =
      add_hv_edge(junction, {20.0, -8.0, 0.0}, false);
  const wire::core::ObjectId pole_e =
      add_hv_edge(junction, {4.0, 8.0, 0.0}, true);
  const wire::core::ObjectId pole_f =
      add_hv_edge(junction, {20.0, 8.0, 0.0}, false);
  const wire::core::ObjectId pole_g =
      add_hv_edge(junction, {4.0, -8.0, 0.0}, true);
  if (pole_d == wire::core::kInvalidObjectId ||
      pole_e == wire::core::kInvalidObjectId ||
      pole_f == wire::core::kInvalidObjectId ||
      pole_g == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto edge_to = [&](wire::core::ObjectId pole) {
    const wire::core::SavedBackboneNode* current_b =
        state.view().backbone_node_for_pole(junction);
    const wire::core::SavedBackboneNode* node =
        state.view().backbone_node_for_pole(pole);
    return current_b == nullptr || node == nullptr
               ? wire::core::kInvalidObjectId
               : edge_between(state, current_b->node_id, node->node_id);
  };
  const wire::core::ObjectId bd = edge_to(pole_d);
  const wire::core::ObjectId be = edge_to(pole_e);
  const wire::core::ObjectId bf = edge_to(pole_f);
  const wire::core::ObjectId bg = edge_to(pole_g);
  if (bd == wire::core::kInvalidObjectId ||
      be == wire::core::kInvalidObjectId ||
      bf == wire::core::kInvalidObjectId ||
      bg == wire::core::kInvalidObjectId) {
    return false;
  }
  const wire::core::SavedBackbonePortBinding* bd_binding =
      hv_lane0_binding_for_edge_at_pole(state, junction, bd);
  const wire::core::SavedBackbonePortBinding* be_binding =
      hv_lane0_binding_for_edge_at_pole(state, junction, be);
  const wire::core::SavedBackbonePortBinding* bf_binding =
      hv_lane0_binding_for_edge_at_pole(state, junction, bf);
  if (bd_binding == nullptr || be_binding == nullptr ||
      bf_binding == nullptr) {
    return false;
  }
  const wire::core::SavedBackboneEdgeBundle* bd_edge_bundle =
      state.view().backbone_edge_bundle(bd_binding->edge_bundle_id);
  const wire::core::Bundle* bd_bundle =
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
  for (const wire::core::VisualModelInstance& instance :
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
  const wire::core::ObjectId updated_bundle_id = bd_bundle->id;
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

} // namespace backbone_tests
