#include "semantics_coverage.hpp"

#include "city/wire/core_authoritative_types.hpp"
#include "city/wire/core_runtime_types.hpp"
#include "city/wire/core_test_hook.hpp"
#include "city/wire/core_view.hpp"

#include <algorithm>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#ifndef WIRE_TEST_BACKBONE_SEMANTICS_PATH
#define WIRE_TEST_BACKBONE_SEMANTICS_PATH "docs/wire/backbone_operation_semantics.md"
#endif

namespace backbone_tests::semantics_coverage {
namespace {

enum class State { kS0, kS1, kS2, kS3, kS4, kS5, kSM };

struct Endpoint {
  city::wire::ObjectId edge_bundle_id = city::wire::kInvalidObjectId;
  city::wire::ObjectId edge_id = city::wire::kInvalidObjectId;
  bool operator<(const Endpoint& other) const {
    return std::tie(edge_bundle_id, edge_id) < std::tie(other.edge_bundle_id, other.edge_id);
  }
};

struct Observation {
  std::string case_id{};
  Operation operation = Operation::kAddOneEdge;
  State state = State::kS0;
  Entry entry = Entry::kCoreApi;
  test_registry::TestFamily family = test_registry::TestFamily::kBehavior;
};

std::vector<Observation>& observations() {
  static std::vector<Observation> value;
  return value;
}

const char* operation_name(Operation operation) {
  switch (operation) {
  case Operation::kAddOneEdge: return "add_one_edge";
  case Operation::kAddTwoEdges: return "add_two_edges";
  case Operation::kMovePoleAngle: return "move_pole_angle";
  case Operation::kUpdatePlacement: return "update_placement";
  case Operation::kSaveLoad: return "save_load";
  case Operation::kRegenerate: return "regenerate";
  }
  return "unknown";
}

const char* state_name(State state) {
  switch (state) {
  case State::kS0: return "S0";
  case State::kS1: return "S1";
  case State::kS2: return "S2";
  case State::kS3: return "S3";
  case State::kS4: return "S4";
  case State::kS5: return "S5";
  case State::kSM: return "SM";
  }
  return "unknown";
}

const char* entry_name(Entry entry) {
  switch (entry) {
  case Entry::kCoreApi: return "core_api";
  case Entry::kWasmAdapter: return "wasm_adapter";
  case Entry::kViewerAction: return "viewer_action";
  }
  return "unknown";
}

std::string cell_id(Operation operation, State state) {
  return std::string("BOS:") + operation_name(operation) + ":" + state_name(state);
}

std::vector<std::string> markdown_cells(const std::string& line) {
  std::vector<std::string> cells;
  std::size_t begin = 0;
  while (begin < line.size()) {
    const std::size_t separator = line.find('|', begin);
    const std::size_t end = separator == std::string::npos ? line.size() : separator;
    std::string cell = line.substr(begin, end - begin);
    const std::size_t first = cell.find_first_not_of(" \t`\r\n");
    const std::size_t last = cell.find_last_not_of(" \t`\r\n");
    if (first != std::string::npos) cells.push_back(cell.substr(first, last - first + 1));
    if (separator == std::string::npos) break;
    begin = separator + 1;
  }
  return cells;
}

std::set<std::string> required_cells(std::string* error) {
  std::ifstream input(WIRE_TEST_BACKBONE_SEMANTICS_PATH);
  if (!input.is_open()) {
    if (error != nullptr) *error = std::string("unable to open ") + WIRE_TEST_BACKBONE_SEMANTICS_PATH;
    return {};
  }
  bool in_matrix = false;
  std::set<std::string> required;
  const std::vector<std::string> states{"S0", "S1", "S2", "S3", "S4", "S5", "SM"};
  std::string line;
  while (std::getline(input, line)) {
    if (line.find("`S0`") != std::string::npos && line.find("`SM`") != std::string::npos) {
      in_matrix = true;
      continue;
    }
    if (in_matrix && line.rfind("## ", 0) == 0) break;
    if (!in_matrix || line.empty() || line.rfind("|---", 0) == 0) continue;
    const std::vector<std::string> cells = markdown_cells(line);
    if (cells.size() != 8) continue;
    const std::size_t operation_end = cells[0].find(' ');
    std::string operation = cells[0].substr(0, operation_end);
    operation.erase(std::remove(operation.begin(), operation.end(), char(96)), operation.end());
    for (std::size_t i = 1; i < cells.size(); ++i) {
      const std::string& value = cells[i];
      const auto has_code = [&](char code) {
        return value == std::string(1, code) ||
               value.find(std::string("`") + code + "`") != std::string::npos;
      };
      const bool covered = has_code('C') || has_code('O') || has_code('K') || has_code('U');
      if (covered && value.find("D2") == std::string::npos &&
          value.find("D3") == std::string::npos) {
        required.insert("BOS:" + operation + ":" + states[i - 1]);
      }
    }
  }
  if (required.empty() && error != nullptr) *error = "operation semantics matrix has no required cells";
  return required;
}

const city::wire::SavedBackboneEdgeBundle* edge_bundle(const city::wire::CoreState& state,
                                                        city::wire::ObjectId id) {
  return state.view().backbone_edge_bundle(id);
}

const city::wire::Bundle* bundle_for(const city::wire::CoreState& state,
                                     const city::wire::SavedBackboneEdgeBundle* value) {
  return value == nullptr ? nullptr : state.view().bundles().find(value->bundle_id);
}

bool classify(const city::wire::CoreState& state, city::wire::ObjectId pole_id,
              city::wire::ObjectId reference_edge_bundle_id, std::size_t lane_index,
              State* result, std::string* error) {
  const city::wire::SavedBackboneNode* node = state.view().backbone_node_for_pole(pole_id);
  const city::wire::SavedBackboneEdgeBundle* reference = edge_bundle(state, reference_edge_bundle_id);
  const city::wire::Bundle* reference_bundle = bundle_for(state, reference);
  if (node == nullptr || reference == nullptr || reference_bundle == nullptr) {
    if (error != nullptr) *error = "semantics observation requires an existing pole node and edge bundle";
    return false;
  }

  std::map<city::wire::ObjectId, Endpoint> endpoints;
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.row_key.node_id != node->node_id || binding.lane_index != lane_index ||
        binding.bundle_template_id != reference_bundle->bundle_template_id) continue;
    const city::wire::SavedBackboneEdgeBundle* candidate = edge_bundle(state, binding.edge_bundle_id);
    const city::wire::Bundle* candidate_bundle = bundle_for(state, candidate);
    if (candidate == nullptr || candidate_bundle == nullptr ||
        candidate_bundle->placement_key != reference_bundle->placement_key) continue;
    if (binding.row_key.edge_id != candidate->edge_id) {
      if (error != nullptr) *error = "semantics observation found an invalid endpoint binding";
      return false;
    }
    endpoints.emplace(binding.edge_bundle_id, Endpoint{binding.edge_bundle_id, candidate->edge_id});
  }
  if (!endpoints.contains(reference_edge_bundle_id)) {
    if (error != nullptr) *error = "semantics observation reference scope has no endpoint binding";
    return false;
  }

  std::set<Endpoint> connected;
  std::set<std::pair<Endpoint, Endpoint>> pairs;
  for (const city::wire::SavedBackboneRowContinuity* continuity :
       state.view().backbone_row_continuities_for_node(node->node_id)) {
    if (continuity == nullptr || continuity->a.lane_index != lane_index ||
        continuity->b.lane_index != lane_index) continue;
    const auto a_it = endpoints.find(continuity->a.edge_bundle_id);
    const auto b_it = endpoints.find(continuity->b.edge_bundle_id);
    if (a_it == endpoints.end() || b_it == endpoints.end()) continue;
    Endpoint a = a_it->second;
    Endpoint b = b_it->second;
    if (b < a) std::swap(a, b);
    if (!pairs.emplace(a, b).second || !connected.insert(a).second || !connected.insert(b).second) {
      if (error != nullptr) *error = "semantics observation found non-unique continuity";
      return false;
    }
  }

  const std::size_t open_count = endpoints.size() - connected.size();
  if (pairs.empty()) {
    if (open_count == 1) {
      *result = State::kS1;
      return true;
    }
    if (error != nullptr) *error = "state without a pair must contain exactly one open endpoint";
    return false;
  }
  if (pairs.size() != 1) {
    if (error != nullptr) *error = "state classifier supports exactly one connected pair";
    return false;
  }
  if (open_count == 1) {
    *result = State::kS4;
    return true;
  }
  if (open_count >= 2) {
    *result = State::kS5;
    return true;
  }

  const std::set<city::wire::ObjectId> pair_edges{pairs.begin()->first.edge_id, pairs.begin()->second.edge_id};
  const bool has_jumper = std::any_of(
      state.view().visual_curve_parts().parts.begin(), state.view().visual_curve_parts().parts.end(),
      [&](const city::wire::VisualCurvePart& part) {
        if (part.kind != city::wire::VisualCurvePartKind::kJumper || part.source_node_id != node->node_id) return false;
        return std::all_of(pair_edges.begin(), pair_edges.end(), [&](city::wire::ObjectId id) {
          return std::find(part.incident_edge_ids.begin(), part.incident_edge_ids.end(), id) !=
                 part.incident_edge_ids.end();
        });
      });
  *result = has_jumper ? State::kS3 : State::kS2;
  return true;
}

void add_observation(Operation operation, State state, Entry entry) {
  observations().push_back({test_registry::CurrentTestCaseId(), operation, state, entry,
                            test_registry::CurrentTestFamily()});
}

} // namespace

bool Observe(Operation operation, Entry entry, const city::wire::CoreState& state,
             city::wire::ObjectId pole_id, city::wire::ObjectId reference_edge_bundle_id,
             std::size_t lane_index, std::string* error) {
  State classified = State::kS0;
  if (!classify(state, pole_id, reference_edge_bundle_id, lane_index, &classified, error)) return false;
  add_observation(operation, classified, entry);
  return true;
}

bool ObserveEmpty(Operation operation, Entry entry, const city::wire::CoreState& state, std::string* error) {
  if (!state.view().backbone().port_bindings.empty()) {
    if (error != nullptr) *error = "S0 observation requires no existing endpoint bindings";
    return false;
  }
  add_observation(operation, State::kS0, entry);
  return true;
}

bool ObserveMidspan(Operation operation, Entry entry, city::wire::CoreState& state,
                    city::wire::ObjectId source_edge_id, std::string* error) {
  const city::wire::SavedBackboneEdge* source_edge = state.view().backbone_edge(source_edge_id);
  if (source_edge == nullptr) {
    if (error != nullptr) *error = "midspan observation requires an existing source edge";
    return false;
  }
  const auto matches_source = [&](city::wire::ObjectId node_a, city::wire::ObjectId node_b) {
    return (node_a == source_edge->node_a && node_b == source_edge->node_b) ||
           (node_a == source_edge->node_b && node_b == source_edge->node_a);
  };
  const bool saved_source = std::any_of(
      state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
      [&](const city::wire::SavedBackboneNode& node) {
        return node.has_source_edge &&
               matches_source(node.source_edge_node_a, node.source_edge_node_b);
      });
  const auto& pending = city::wire::CoreStateTestHook::pending_support_nodes(state);
  const bool pending_source = std::any_of(
      pending.begin(), pending.end(), [&](const city::wire::SupportNode& node) {
        return node.has_source_edge &&
               matches_source(node.source_edge_node_a_id, node.source_edge_node_b_id);
      });
  if (!saved_source && !pending_source) {
    if (error != nullptr) *error = "SM observation requires a matching pending or saved source node";
    return false;
  }
  add_observation(operation, State::kSM, entry);
  return true;
}

bool ValidateRuntimeCoverage(std::string* error) {
  std::string parse_error;
  const std::set<std::string> required = required_cells(&parse_error);
  if (!parse_error.empty()) {
    if (error != nullptr) *error = parse_error;
    return false;
  }
  std::set<std::string> covered;
  std::set<std::string> weak;
  for (const Observation& observation : observations()) {
    const std::string cell = cell_id(observation.operation, observation.state);
    if (observation.family == test_registry::TestFamily::kSourceGuard) {
      weak.insert(cell + " case:" + observation.case_id + " is a SourceGuard");
      continue;
    }
    if (observation.entry == Entry::kCoreApi) covered.insert(cell);
    if (!test_registry::TestCaseHasIndependentAssertion(observation.case_id)) {
      weak.insert(cell + " case:" + observation.case_id + " entry:" + entry_name(observation.entry));
    }
  }
  std::vector<std::string> missing;
  std::set_difference(required.begin(), required.end(), covered.begin(), covered.end(), std::back_inserter(missing));
  if (missing.empty() && weak.empty()) return true;
  if (error != nullptr) {
    std::ostringstream out;
    if (!missing.empty()) {
      out << "unreached cells: ";
      for (std::size_t i = 0; i < missing.size(); ++i) {
        if (i > 0) out << ", ";
        out << missing[i];
      }
    }
    if (!weak.empty()) {
      if (!missing.empty()) out << "; ";
      out << "observations without independent assertion: ";
      std::size_t i = 0;
      for (const std::string& item : weak) {
        if (i++ > 0) out << ", ";
        out << item;
      }
    }
    *error = out.str();
  }
  return false;
}

void ResetRuntimeCoverage() {
  observations().clear();
}

} // namespace backbone_tests::semantics_coverage
