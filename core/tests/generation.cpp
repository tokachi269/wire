#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "registry.hpp"
#include "helpers.hpp"
#include "wire/core/coord_utils.hpp"
#include "../src/generation/bundle_spans/build_endpoint_heights.hpp"
#include "../src/generation/backbone_pipeline/backbone_pipeline.hpp"
#include "../src/generation/support_policy.hpp"

using namespace helpers;

std::optional<wire::core::SupportLayoutEndpointView> layout_endpoint_for_owner(
    const wire::core::SupportLayoutInspectionView& layout_view, wire::core::ObjectId owner_pole_id);
std::optional<wire::core::LoweredSupportGroupInspectionView> lowered_support_group_for_owner(
  const wire::core::SupportLayoutInspectionView& layout_view, wire::core::ObjectId owner_pole_id);
std::optional<wire::core::SegmentLaneAssignment> find_assignment_for_span(const wire::core::CoreState& state,
                                                                          wire::core::ObjectId span_id);
std::optional<wire::core::VisualPart> support_arm_part_for_owner(const wire::core::CoreState& state,
                                                                 wire::core::ObjectId span_id,
                                                                 wire::core::ObjectId owner_pole_id,
                                                                 double z_tolerance = 1e-6);
bool build_replayed_latest_cross_capture_scene(CoreState& state, std::vector<wire::core::ObjectId>* out_span_ids);
bool build_replayed_latest_support_orientation_scene(CoreState& state,
                                                     std::vector<wire::core::ObjectId>* out_generated_span_ids);
bool build_minimal_latest_pair_cross_capture_scene(CoreState& state, wire::core::ObjectId* out_center_id,
                                                   std::unordered_map<wire::core::ObjectId, wire::core::ObjectId>* out_remapped_ids,
                                                   std::vector<wire::core::ObjectId>* out_span_ids);
bool endpoint_uses_pair_authority_without_local_fallback(const wire::core::SupportLayoutEndpointView& endpoint,
                                                         bool require_side_axis = true);
bool pair_authority_axis_family_is_consistent(const wire::core::SupportLayoutEndpointView& endpoint);

namespace {

std::uint64_t mix_u64_test(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}

double unit_from_u64_test(std::uint64_t x) {
  constexpr double kInv = 1.0 / static_cast<double>(1ull << 53);
  return static_cast<double>((mix_u64_test(x) >> 11) & ((1ull << 53) - 1ull)) * kInv;
}

std::vector<wire::core::Vec3d> make_latest_capture_twist_variant_polyline(std::uint64_t seed) {
  std::vector<wire::core::Vec3d> polyline = {
      {2.38374, -21.7773, 0.0},
      {-12.8223, -15.1594, 0.0},
      {-17.5767, -14.0377, 0.0},
      {-16.1859, -5.45037, 0.0},
      {-15.6915, -4.78471, 0.0},
      {-13.6725, -5.22158, 0.0},
      {-12.7205, -9.02353, 0.0},
  };
  for (std::size_t i = 1; i + 1 < polyline.size(); ++i) {
    const std::uint64_t sx = mix_u64_test(seed ^ (0xA511E9B3ull + static_cast<std::uint64_t>(i) * 17ull));
    const std::uint64_t sy = mix_u64_test(seed ^ (0xC3D2E1F0ull + static_cast<std::uint64_t>(i) * 29ull));
    polyline[i].x += (unit_from_u64_test(sx) * 2.0 - 1.0) * 1.2;
    polyline[i].y += (unit_from_u64_test(sy) * 2.0 - 1.0) * 1.2;
  }
  return polyline;
}

struct HvEndpointSnapshot {
  int segment_index = -1;
  wire::core::ObjectId bundle_id = wire::core::kInvalidObjectId;
  wire::core::ObjectId pole_id = wire::core::kInvalidObjectId;
  std::vector<wire::core::ObjectId> port_ids{};
  std::vector<double> local_y{};
  std::vector<wire::core::Vec3d> world_positions{};
  wire::core::RowLayoutAxisMode row_layout_axis_mode = wire::core::RowLayoutAxisMode::kPoleYaw;
  double layout_yaw_deg = 0.0;
  double final_yaw_deg = 0.0;
  wire::core::Vec3d support_axis_dir{};
};

struct BackboneCandidateObservation {
  std::vector<wire::core::SupportNode> nodes{};
  std::vector<wire::core::ObjectId> generated_pole_ids{};
};

std::string hv_endpoint_snapshot_key(const HvEndpointSnapshot& snapshot) {
  std::ostringstream oss;
  oss << snapshot.bundle_id << ":" << snapshot.segment_index << ":" << snapshot.pole_id;
  return oss.str();
}

std::optional<BackboneCandidateObservation> observe_backbone_candidates(
    const std::vector<wire::core::Vec3d>& polyline, double interval_m, bool pin_vertices, bool pin_endpoints,
    const std::vector<wire::core::BackboneInputSpec::NodeSpec>& node_specs = {}, std::string* error = nullptr) {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    if (error != nullptr) {
      *error = "missing pole types";
    }
    return std::nullopt;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = polyline;
  req.path.node_specs = node_specs;
  req.interval_m = interval_m;
  req.pole_type_id = type_ids.front();
  req.pole_placement.pin_vertices = pin_vertices;
  req.pole_placement.pin_endpoints = pin_endpoints;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);

  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    if (error != nullptr) {
      *error = generated.error;
    }
    return std::nullopt;
  }

  BackboneCandidateObservation observation{};
  observation.nodes = state.BuildBackboneResult().nodes;
  observation.generated_pole_ids = generated.value.generated_pole_ids;
  return observation;
}

const wire::core::SupportNode* find_backbone_node_near_world(const BackboneCandidateObservation& observation,
                                                             const wire::core::Vec3d& world, double eps = 1e-6) {
  for (const auto& node : observation.nodes) {
    if (almost_equal(node.position, world, eps)) {
      return &node;
    }
  }
  return nullptr;
}

std::vector<wire::core::ObjectId> undo_order_choice(const std::vector<wire::core::ObjectId>& ordered,
                                                    wire::core::OrderDecisionChoiceKind choice) {
  std::vector<wire::core::ObjectId> base = ordered;
  if (choice == wire::core::OrderDecisionChoiceKind::kReversed) {
    std::reverse(base.begin(), base.end());
  }
  return base;
}

std::vector<wire::core::ObjectId> apply_order_choice(const std::vector<wire::core::ObjectId>& base,
                                                     wire::core::OrderDecisionChoiceKind choice) {
  std::vector<wire::core::ObjectId> ordered = base;
  if (choice == wire::core::OrderDecisionChoiceKind::kReversed) {
    std::reverse(ordered.begin(), ordered.end());
  }
  return ordered;
}

std::vector<wire::core::SegmentLaneAssignment> collect_template_assignments(const CoreState& state,
                                                                            wire::core::BundleKind bundle_kind) {
  std::vector<wire::core::SegmentLaneAssignment> result{};
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const auto* bundle = state.view().edit_state().bundles.find(assignment.bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != bundle_kind) {
      continue;
    }
    result.push_back(assignment);
  }
  std::sort(result.begin(), result.end(),
            [](const wire::core::SegmentLaneAssignment& a, const wire::core::SegmentLaneAssignment& b) {
              return a.segment_index < b.segment_index;
            });
  return result;
}

bool reconstruct_assignment_node_bases(
    const std::vector<wire::core::SegmentLaneAssignment>& assignments,
    std::vector<std::vector<wire::core::ObjectId>>* base_ports_by_node, std::vector<int>* chosen_parity,
    std::vector<wire::core::ObjectId>* node_ids, std::string* error) {
  if (base_ports_by_node == nullptr || chosen_parity == nullptr || node_ids == nullptr) {
    return false;
  }
  if (assignments.empty()) {
    if (error != nullptr) {
      *error = "no assignments";
    }
    return false;
  }

  const std::size_t node_count = assignments.size() + 1;
  base_ports_by_node->assign(node_count, {});
  chosen_parity->assign(node_count, 0);
  node_ids->assign(node_count, wire::core::kInvalidObjectId);

  (*node_ids)[0] = assignments.front().pole_a_id;
  (*chosen_parity)[0] =
      (assignments.front().order_decision_choice_a == wire::core::OrderDecisionChoiceKind::kReversed) ? 1 : 0;
  (*base_ports_by_node)[0] =
      undo_order_choice(assignments.front().port_ids_a, assignments.front().order_decision_choice_a);

  for (std::size_t seg = 0; seg < assignments.size(); ++seg) {
    const auto& assignment = assignments[seg];
    const std::size_t node_index = seg + 1;
    (*node_ids)[node_index] = assignment.pole_b_id;
    (*chosen_parity)[node_index] =
        (assignment.order_decision_choice_b == wire::core::OrderDecisionChoiceKind::kReversed) ? 1 : 0;
    const auto base_from_left = undo_order_choice(assignment.port_ids_b, assignment.order_decision_choice_b);
    if ((*base_ports_by_node)[node_index].empty()) {
      (*base_ports_by_node)[node_index] = base_from_left;
    } else if ((*base_ports_by_node)[node_index] != base_from_left) {
      if (error != nullptr) {
        std::ostringstream oss;
        oss << "base_mismatch nodeIndex=" << node_index;
        *error = oss.str();
      }
      return false;
    }

    if (seg + 1 < assignments.size()) {
      const auto& next = assignments[seg + 1];
      if (assignment.pole_b_id != next.pole_a_id) {
        if (error != nullptr) {
          std::ostringstream oss;
          oss << "route_break seg=" << seg << " node=" << assignment.pole_b_id << " next=" << next.pole_a_id;
          *error = oss.str();
        }
        return false;
      }
      const int next_parity =
          (next.order_decision_choice_a == wire::core::OrderDecisionChoiceKind::kReversed) ? 1 : 0;
      if (next_parity != (*chosen_parity)[node_index]) {
        if (error != nullptr) {
          std::ostringstream oss;
          oss << "parity_break nodeIndex=" << node_index << " left=" << (*chosen_parity)[node_index]
              << " right=" << next_parity;
          *error = oss.str();
        }
        return false;
      }
      const auto base_from_right = undo_order_choice(next.port_ids_a, next.order_decision_choice_a);
      if ((*base_ports_by_node)[node_index] != base_from_right) {
        if (error != nullptr) {
          std::ostringstream oss;
          oss << "peer_base_mismatch nodeIndex=" << node_index;
          *error = oss.str();
        }
        return false;
      }
    }
  }
  return true;
}

void dump_assignment_parity_bruteforce(const CoreState& state,
                                       const std::vector<wire::core::SegmentLaneAssignment>& assignments,
                                       const char* tag) {
  std::vector<std::vector<wire::core::ObjectId>> base_ports_by_node{};
  std::vector<int> chosen_parity{};
  std::vector<wire::core::ObjectId> node_ids{};
  std::string error{};
  if (!reconstruct_assignment_node_bases(assignments, &base_ports_by_node, &chosen_parity, &node_ids, &error)) {
    std::cerr << "[DBG] " << tag << " bruteforce_unavailable reason=" << error << "\n";
    return;
  }

  const std::size_t node_count = base_ports_by_node.size();
  if (node_count == 0 || node_count > 20) {
    std::cerr << "[DBG] " << tag << " bruteforce_unavailable nodeCount=" << node_count << "\n";
    return;
  }

  auto build_assignments_for_mask = [&](std::uint64_t mask) {
    std::vector<wire::core::SegmentLaneAssignment> candidate = assignments;
    for (std::size_t seg = 0; seg < assignments.size(); ++seg) {
      const bool parity_a = ((mask >> seg) & 1ull) != 0ull;
      const bool parity_b = ((mask >> (seg + 1)) & 1ull) != 0ull;
      candidate[seg].order_decision_choice_a =
          parity_a ? wire::core::OrderDecisionChoiceKind::kReversed : wire::core::OrderDecisionChoiceKind::kNormal;
      candidate[seg].order_decision_choice_b =
          parity_b ? wire::core::OrderDecisionChoiceKind::kReversed : wire::core::OrderDecisionChoiceKind::kNormal;
      candidate[seg].port_ids_a = apply_order_choice(base_ports_by_node[seg], candidate[seg].order_decision_choice_a);
      candidate[seg].port_ids_b =
          apply_order_choice(base_ports_by_node[seg + 1], candidate[seg].order_decision_choice_b);
    }
    return candidate;
  };

  std::uint64_t chosen_mask = 0ull;
  for (std::size_t i = 0; i < chosen_parity.size(); ++i) {
    if (chosen_parity[i] != 0) {
      chosen_mask |= (1ull << i);
    }
  }
  const int chosen_intersections = count_bundle_lane_polyline_xy_intersections(state, assignments);

  int best_intersections = std::numeric_limits<int>::max();
  std::uint64_t best_mask = 0ull;
  for (std::uint64_t mask = 0ull; mask < (1ull << node_count); ++mask) {
    const auto candidate = build_assignments_for_mask(mask);
    const int intersections = count_bundle_lane_polyline_xy_intersections(state, candidate);
    if (intersections < best_intersections) {
      best_intersections = intersections;
      best_mask = mask;
    }
  }

  std::cerr << "[DBG] " << tag << " chosenMask=" << chosen_mask << " chosenIntersections=" << chosen_intersections
            << " bestMask=" << best_mask << " bestIntersections=" << best_intersections << " nodeParities=";
  for (std::size_t i = 0; i < chosen_parity.size(); ++i) {
    std::cerr << chosen_parity[i] << ",";
  }
  std::cerr << "\n";
}

std::vector<HvEndpointSnapshot> capture_hv_endpoint_snapshots(const CoreState& state) {
  std::vector<HvEndpointSnapshot> snapshots{};
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const auto* bundle = state.view().edit_state().bundles.find(assignment.bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }

    const auto capture_side = [&](wire::core::ObjectId pole_id, const std::vector<wire::core::ObjectId>& port_ids) {
      const auto* pole = state.view().edit_state().poles.find(pole_id);
      if (pole == nullptr) {
        return;
      }
      const auto pole_view = state.view().inspect_pole(pole_id);
      HvEndpointSnapshot snapshot{};
      snapshot.segment_index = assignment.segment_index;
      snapshot.bundle_id = assignment.bundle_id;
      snapshot.pole_id = pole_id;
      snapshot.port_ids = port_ids;
      if (pole_view.has_value()) {
        snapshot.row_layout_axis_mode = pole_view->row_layout_axis_mode;
        snapshot.layout_yaw_deg = pole_view->layout_yaw_deg;
        snapshot.final_yaw_deg = pole_view->final_yaw_deg;
        snapshot.support_axis_dir = pole_view->support_axis_dir;
      }
      for (wire::core::ObjectId port_id : port_ids) {
        const auto* port = state.view().edit_state().ports.find(port_id);
        if (port == nullptr) {
          snapshot.local_y.push_back(0.0);
          snapshot.world_positions.push_back({});
          continue;
        }
        const double layout_yaw_deg = state.effective_port_layout_yaw_deg(*pole, port->category);
        const wire::core::Vec3d local = wire::core::WorldPointToLocal(
            wire::core::BuildPoleFrame(pole->world_transform, layout_yaw_deg), port->world_position);
        snapshot.local_y.push_back(local.y);
        snapshot.world_positions.push_back(port->world_position);
      }
      snapshots.push_back(std::move(snapshot));
    };

    capture_side(assignment.pole_a_id, assignment.port_ids_a);
    capture_side(assignment.pole_b_id, assignment.port_ids_b);
  }
  std::sort(snapshots.begin(), snapshots.end(), [](const HvEndpointSnapshot& a, const HvEndpointSnapshot& b) {
    return hv_endpoint_snapshot_key(a) < hv_endpoint_snapshot_key(b);
  });
  return snapshots;
}

bool test_backbone_pipeline_build_direction_initializes_from_input_direction() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec request{};
  request.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  request.interval_m = 8.0;
  request.pole_type_id = type_ids.front();
  request.direction_mode = wire::core::PathDirectionMode::kReverse;
  add_backbone_bundle(request, wire::core::BundleKind::kLowVoltage);

  wire::core::generation::detail::BackboneBuilder builder(state);
  const auto built = builder.build(request);
  if (!built.ok) {
    return false;
  }
  if (built.value.backbone.input_direction != wire::core::generation::detail::InputDirection::kReverse ||
      built.value.backbone.build_direction != wire::core::generation::detail::BuildDirection::kReverse ||
      built.value.support_chain.ordered_support_node_ids.empty()) {
    return false;
  }
  const auto first_it =
      built.value.support_chain.support_node_by_id.find(built.value.support_chain.ordered_support_node_ids.front());
  return first_it != built.value.support_chain.support_node_by_id.end() &&
         almost_equal(first_it->second.position, request.path.polyline.back());
}

bool test_backbone_terminal_without_explicit_pair_does_not_borrow_pair() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  auto run_path = [&](const std::vector<wire::core::Vec3d>& polyline)
      -> wire::core::EditResult<wire::core::GenerateBundleFromPathResult> {
    wire::core::BackboneSpec request{};
    request.path.polyline = polyline;
    request.interval_m = 20.0;
    request.pole_type_id = type_ids.front();
    add_backbone_bundle(request, wire::core::BundleKind::kLowVoltage);
    return state.GenerateFromBackboneSpec(request);
  };

  if (!run_path({{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}}).ok) {
    return false;
  }
  const auto branch = run_path({{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}});
  if (!branch.ok || branch.value.generated_span_ids.empty()) {
    return false;
  }
  const ObjectId tip_pole_id = find_pole_id_by_position(state, {0.0, 12.0, 0.0});
  if (tip_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto contract = state.support_layout_contract(branch.value.generated_span_ids.front());
  if (!contract.has_authority()) {
    return false;
  }
  const auto& start = contract.authority.seed->start;
  const auto& end = contract.authority.seed->end;
  const auto* endpoint = (start.owner_pole_id == tip_pole_id) ? &start : ((end.owner_pole_id == tip_pole_id) ? &end : nullptr);
  return endpoint != nullptr && !endpoint->used_junction_pair_side_assignment &&
         std::abs(endpoint->chosen_side_sign) <= 1e-9 &&
         endpoint->support_pair_peer_low == wire::core::kInvalidObjectId &&
         endpoint->support_pair_peer_high == wire::core::kInvalidObjectId;
}

bool test_backbone_materialization_does_not_change_pair_family() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  auto run_path = [&](const std::vector<wire::core::Vec3d>& polyline)
      -> wire::core::EditResult<wire::core::GenerateBundleFromPathResult> {
    wire::core::BackboneSpec request{};
    request.path.polyline = polyline;
    request.interval_m = 8.0;
    request.pole_type_id = type_ids.front();
    add_backbone_bundle(request, wire::core::BundleKind::kLowVoltage);
    return state.GenerateFromBackboneSpec(request);
  };

  if (!run_path({{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}}).ok) {
    return false;
  }
  const auto cross = run_path({{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}});
  if (!cross.ok || cross.value.generated_span_ids.empty()) {
    return false;
  }
  const ObjectId span_id = cross.value.generated_span_ids.front();
  const auto contract = state.support_layout_contract(span_id);
  const auto layout = state.view().inspect_support_layout(span_id);
  if (!contract.has_authority() || !layout.has_value()) {
    return false;
  }

  const bool start_ok =
      contract.authority.seed->start.support_pair_peer_low == layout->start_endpoint.support_authority.pair.pair_peer_low &&
      contract.authority.seed->start.support_pair_peer_high == layout->start_endpoint.support_authority.pair.pair_peer_high;
  const bool end_ok =
      contract.authority.seed->end.support_pair_peer_low == layout->end_endpoint.support_authority.pair.pair_peer_low &&
      contract.authority.seed->end.support_pair_peer_high == layout->end_endpoint.support_authority.pair.pair_peer_high;
  return start_ok && end_ok;
}

void dump_hv_endpoint_snapshot_diff(const std::vector<HvEndpointSnapshot>& before,
                                    const std::vector<HvEndpointSnapshot>& after, const char* tag) {
  std::unordered_map<std::string, const HvEndpointSnapshot*> before_by_key{};
  for (const auto& snapshot : before) {
    before_by_key.emplace(hv_endpoint_snapshot_key(snapshot), &snapshot);
  }
  for (const auto& snapshot_after : after) {
    const std::string key = hv_endpoint_snapshot_key(snapshot_after);
    const auto it_before = before_by_key.find(key);
    if (it_before == before_by_key.end()) {
      std::cerr << "[DBG] " << tag << " added key=" << key << "\n";
      continue;
    }
    const HvEndpointSnapshot& snapshot_before = *it_before->second;
    const std::size_t lane_count = std::min(
        {snapshot_before.port_ids.size(), snapshot_after.port_ids.size(), snapshot_before.local_y.size(),
         snapshot_after.local_y.size(), snapshot_before.world_positions.size(), snapshot_after.world_positions.size()});
    bool changed = false;
    std::ostringstream oss;
    for (std::size_t lane = 0; lane < lane_count; ++lane) {
      const double dy = std::abs(snapshot_before.local_y[lane] - snapshot_after.local_y[lane]);
      const double dx_world =
          std::abs(snapshot_before.world_positions[lane].x - snapshot_after.world_positions[lane].x);
      const double dy_world =
          std::abs(snapshot_before.world_positions[lane].y - snapshot_after.world_positions[lane].y);
      const double dz_world =
          std::abs(snapshot_before.world_positions[lane].z - snapshot_after.world_positions[lane].z);
      if (dy > 1e-6 || dx_world > 1e-6 || dy_world > 1e-6 || dz_world > 1e-6 ||
          snapshot_before.port_ids[lane] != snapshot_after.port_ids[lane]) {
        changed = true;
        oss << " lane=" << lane << " port=" << snapshot_before.port_ids[lane] << "->" << snapshot_after.port_ids[lane]
            << " localY=" << snapshot_before.local_y[lane] << "->" << snapshot_after.local_y[lane]
            << " world=(" << snapshot_before.world_positions[lane].x << "," << snapshot_before.world_positions[lane].y
            << "," << snapshot_before.world_positions[lane].z << ")->(" << snapshot_after.world_positions[lane].x
            << "," << snapshot_after.world_positions[lane].y << "," << snapshot_after.world_positions[lane].z << ")";
      }
    }
    const bool pole_changed =
        snapshot_before.row_layout_axis_mode != snapshot_after.row_layout_axis_mode ||
        std::abs(snapshot_before.layout_yaw_deg - snapshot_after.layout_yaw_deg) > 1e-6 ||
        std::abs(snapshot_before.final_yaw_deg - snapshot_after.final_yaw_deg) > 1e-6 ||
        std::abs(snapshot_before.support_axis_dir.x - snapshot_after.support_axis_dir.x) > 1e-6 ||
        std::abs(snapshot_before.support_axis_dir.y - snapshot_after.support_axis_dir.y) > 1e-6 ||
        std::abs(snapshot_before.support_axis_dir.z - snapshot_after.support_axis_dir.z) > 1e-6;
    if (changed || pole_changed) {
      std::cerr << "[DBG] " << tag << " key=" << key << " rowMode="
                << static_cast<int>(snapshot_before.row_layout_axis_mode) << "->"
                << static_cast<int>(snapshot_after.row_layout_axis_mode) << " layoutYaw="
                << snapshot_before.layout_yaw_deg << "->" << snapshot_after.layout_yaw_deg << " finalYaw="
                << snapshot_before.final_yaw_deg << "->" << snapshot_after.final_yaw_deg << " axis=("
                << snapshot_before.support_axis_dir.x << "," << snapshot_before.support_axis_dir.y << ","
                << snapshot_before.support_axis_dir.z << ")->(" << snapshot_after.support_axis_dir.x << ","
                << snapshot_after.support_axis_dir.y << "," << snapshot_after.support_axis_dir.z << ")"
                << oss.str() << "\n";
    }
  }
}

bool hv_route_has_parity_discontinuity(const CoreState& state, std::string* details) {
  const auto assignments = collect_template_assignments(state, wire::core::BundleKind::kHighVoltage);
  if (assignments.empty()) {
    if (details != nullptr) {
      *details = "hv orientations missing";
    }
    return true;
  }

  const int polyline_intersections = count_bundle_lane_polyline_xy_intersections(state, assignments);
  if (polyline_intersections != 0) {
    if (details != nullptr) {
      std::ostringstream oss;
      oss << "polyline_intersections=" << polyline_intersections;
      *details = oss.str();
    }
    return true;
  }

  const int adjacent_discontinuities =
      count_bundle_lane_adjacent_order_discontinuities(state, assignments);
  if (adjacent_discontinuities != 0) {
    if (details != nullptr) {
      std::ostringstream oss;
      oss << "adjacent discontinuities=" << adjacent_discontinuities;
      *details = oss.str();
    }
    return true;
  }

  std::vector<std::vector<wire::core::ObjectId>> base_ports_by_node{};
  std::vector<int> chosen_parity{};
  std::vector<wire::core::ObjectId> node_ids{};
  std::string error{};
  if (!reconstruct_assignment_node_bases(assignments, &base_ports_by_node, &chosen_parity, &node_ids, &error)) {
    if (details != nullptr) {
      *details = "parity_reconstruct_failed:" + error;
    }
    return true;
  }

  const std::size_t node_count = base_ports_by_node.size();
  if (node_count != 0 && node_count <= 20) {
    auto build_assignments_for_mask = [&](std::uint64_t mask) {
      std::vector<wire::core::SegmentLaneAssignment> candidate = assignments;
      for (std::size_t seg = 0; seg < assignments.size(); ++seg) {
        const bool parity_a = ((mask >> seg) & 1ull) != 0ull;
        const bool parity_b = ((mask >> (seg + 1)) & 1ull) != 0ull;
        candidate[seg].order_decision_choice_a =
            parity_a ? wire::core::OrderDecisionChoiceKind::kReversed : wire::core::OrderDecisionChoiceKind::kNormal;
        candidate[seg].order_decision_choice_b =
            parity_b ? wire::core::OrderDecisionChoiceKind::kReversed : wire::core::OrderDecisionChoiceKind::kNormal;
        candidate[seg].port_ids_a = apply_order_choice(base_ports_by_node[seg], candidate[seg].order_decision_choice_a);
        candidate[seg].port_ids_b =
            apply_order_choice(base_ports_by_node[seg + 1], candidate[seg].order_decision_choice_b);
      }
      return candidate;
    };

    std::uint64_t chosen_mask = 0ull;
    for (std::size_t i = 0; i < chosen_parity.size(); ++i) {
      if (chosen_parity[i] != 0) {
        chosen_mask |= (1ull << i);
      }
    }

    const int chosen_intersections = count_bundle_lane_polyline_xy_intersections(state, assignments);
    int best_intersections = std::numeric_limits<int>::max();
    for (std::uint64_t mask = 0ull; mask < (1ull << node_count); ++mask) {
      const auto candidate = build_assignments_for_mask(mask);
      const int intersections = count_bundle_lane_polyline_xy_intersections(state, candidate);
      best_intersections = std::min(best_intersections, intersections);
    }

    if (chosen_intersections != best_intersections) {
      if (details != nullptr) {
        std::ostringstream oss;
        oss << "suboptimal parity chosen=" << chosen_intersections << " best=" << best_intersections
            << " chosenMask=" << chosen_mask;
        *details = oss.str();
      }
      return true;
    }
  }
  return false;
}

bool hv_route_has_final_curve_twist(const CoreState& state, std::string* details) {
  const auto& assignments = state.view().last_lane_assignments();
  const int polyline_intersections = count_bundle_lane_polyline_xy_intersections(state, assignments);
  if (polyline_intersections != 0) {
    if (details != nullptr) {
      std::ostringstream oss;
      oss << "polyline_intersections=" << polyline_intersections;
      *details = oss.str();
    }
    return true;
  }
  const int final_curve_intersections = count_bundle_lane_detail_curve_xy_intersections(state, assignments);
  if (final_curve_intersections != 0) {
    if (details != nullptr) {
      std::ostringstream oss;
      oss << "final_curve_intersections=" << final_curve_intersections;
      *details = oss.str();
    }
    return true;
  }
  return false;
}

} // namespace

bool endpoint_has_authoritative_lowering(const wire::core::SupportLayoutEndpointView& endpoint) {
  return endpoint.lower_required && !endpoint.lowering_blocked_by_policy &&
         endpoint.support_group_id >= 0;
}

double insulator_lift_for_span_test(const CoreState& state, wire::core::ObjectId span_id) {
  const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
  if (span == nullptr) {
    return 0.0;
  }
  const wire::core::Bundle* bundle = state.view().edit_state().bundles.find(span->bundle_id);
  if (bundle == nullptr) {
    return 0.0;
  }
  const auto bundle_template_it = state.view().bundle_templates().find(bundle->bundle_template_id);
  if (bundle_template_it == state.view().bundle_templates().end()) {
    return 0.0;
  }
  const auto cable_template_it = state.view().cable_templates().find(bundle_template_it->second.cable_template_id);
  if (cable_template_it == state.view().cable_templates().end()) {
    return 0.0;
  }
  const wire::core::CableTemplate& cable_template = cable_template_it->second;
  if (!cable_template.requires_insulator) {
    return 0.0;
  }
  return std::max(0.0, cable_template.insulator_attachment_height_m);
}

const wire::core::EndpointContinuityDecision* assignment_decision_for_pole(
    const wire::core::SegmentLaneAssignment& assignment, wire::core::ObjectId pole_id) {
  if (assignment.pole_a_id == pole_id) {
    return &assignment.decision_a;
  }
  if (assignment.pole_b_id == pole_id) {
    return &assignment.decision_b;
  }
  return nullptr;
}

bool test_backbone_generation_includes_midair_support_nodes() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  midair.has_tangent_hint = true;
  midair.tangent_hint = {1.0, 0.0, 0.0};
  req.path.node_specs.push_back(midair);
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);

  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }
  const auto backbone = state.BuildBackboneResult();
  const auto* node = find_support_node_by_point_index(backbone, 1);
  return node != nullptr && node->support_kind == wire::core::SupportKind::kMidair && node->has_tangent_hint &&
         node->bundle_modes.size() == 1 && node->bundle_modes.front().mode == wire::core::BundleNodeMode::kNotPresent;
}

// Intent: Explicit node_ids from DrawPath picks should reuse an existing pole node instead of creating a duplicate.
bool test_backbone_generation_reuses_explicit_pole_node_id() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok) {
    return false;
  }

  const ObjectId anchor_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (anchor_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec second{};
  second.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec anchor{};
  anchor.point_index = 0;
  anchor.support_kind = wire::core::SupportKind::kPole;
  anchor.node_id = anchor_id;
  second.path.node_specs.push_back(anchor);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok) {
    return false;
  }
  if (generated_second.value.generated_pole_ids.size() != 1 || contains_id(generated_second.value.generated_pole_ids, anchor_id)) {
    return false;
  }

  const ObjectId new_end_id = find_pole_id_by_position(state, {0.0, 12.0, 0.0});
  if (new_end_id == wire::core::kInvalidObjectId || new_end_id == anchor_id) {
    return false;
  }
  const auto route = state.FindBackboneRoute(anchor_id, new_end_id);
  return route.size() == 2 && route.front() == anchor_id && route.back() == new_end_id;
}

// Intent: Explicit node_ids from DrawPath picks should reuse an existing non-pole support node in the new backbone.
bool test_backbone_generation_reuses_explicit_support_node_id() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  first.path.node_specs.push_back(midair);
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok) {
    return false;
  }
  const auto first_backbone = state.BuildBackboneResult();
  const auto* existing_midair = find_support_node_by_point_index(first_backbone, 1);
  if (existing_midair == nullptr || existing_midair->node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const ObjectId existing_midair_id = existing_midair->node_id;

  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {10.0, 12.0, 0.0}, {20.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = existing_midair_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok) {
    return false;
  }
  const auto second_backbone = state.BuildBackboneResult();
  const auto* reused = find_support_node_by_point_index(second_backbone, 0);
  return reused != nullptr && reused->node_id == existing_midair_id &&
         reused->support_kind == wire::core::SupportKind::kMidair;
}

// Intent: Extending DrawPath from a reused midair support node should still realize detail poles/spans on the new leg.
bool test_backbone_midair_extension_generates_detail_chain() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  first.path.node_specs.push_back(midair);
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok) {
    return false;
  }
  const auto first_backbone = state.BuildBackboneResult();
  const auto* existing_midair = find_support_node_by_point_index(first_backbone, 1);
  if (existing_midair == nullptr || existing_midair->node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const ObjectId existing_midair_id = existing_midair->node_id;

  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {22.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = existing_midair_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok) {
    return false;
  }

  return generated_second.value.generated_pole_ids.size() >= 1 && generated_second.value.generated_span_ids.size() >= 1;
}

// Intent: Midair-origin extension must include the first support-to-detail segment in the detailed chain.
bool test_backbone_midair_extension_includes_first_support_segment() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  first.path.node_specs.push_back(midair);
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok) {
    return false;
  }
  const auto first_backbone = state.BuildBackboneResult();
  const auto* existing_midair = find_support_node_by_point_index(first_backbone, 1);
  if (existing_midair == nullptr || existing_midair->node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const ObjectId existing_midair_id = existing_midair->node_id;

  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {34.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = existing_midair_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 6.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok || generated_second.value.generated_pole_ids.empty()) {
    return false;
  }

  const ObjectId terminal_pole_id = find_pole_id_by_position(state, {34.0, 0.0, 0.0});
  if (terminal_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const std::vector<ObjectId> route = state.FindBackboneRoute(existing_midair_id, terminal_pole_id);
  return !generated_second.value.generated_span_ids.empty() &&
         generated_second.value.generated_span_ids.size() == generated_second.value.generated_pole_ids.size() &&
         route.size() == generated_second.value.generated_pole_ids.size() + 1 &&
         route.front() == existing_midair_id && route.back() == terminal_pole_id;
}

// Intent: Midair picked on backbone can stay at backbone height while detailed branch starts from source span height.
bool test_backbone_midair_extension_single_click_stays_single_segment() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  first.path.node_specs.push_back(midair);
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok) {
    return false;
  }
  const auto first_backbone = state.BuildBackboneResult();
  const auto* existing_midair = find_support_node_by_point_index(first_backbone, 1);
  if (existing_midair == nullptr || existing_midair->node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {18.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = existing_midair->node_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok) {
    return false;
  }
  if (generated_second.value.generated_pole_ids.size() != 1 || generated_second.value.generated_span_ids.size() != 1) {
    return false;
  }

  const ObjectId terminal_pole_id = find_pole_id_by_position(state, {18.0, 8.0, 0.0});
  if (terminal_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const std::vector<ObjectId> route = state.FindBackboneRoute(existing_midair->node_id, terminal_pole_id);
  return route.size() == 2 && route.front() == existing_midair->node_id && route.back() == terminal_pole_id;
}

bool test_backbone_midair_branch_reuses_source_span_height() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok || generated_first.value.generated_span_ids.size() != 1) {
    return false;
  }
  const ObjectId source_span_id = generated_first.value.generated_span_ids.front();
  const wire::core::Span* source_span = state.view().edit_state().spans.find(source_span_id);
  if (source_span == nullptr) {
    return false;
  }
  const wire::core::Port* source_port_a = state.view().edit_state().ports.find(source_span->port_a_id);
  const wire::core::Port* source_port_b = state.view().edit_state().ports.find(source_span->port_b_id);
  if (source_port_a == nullptr || source_port_b == nullptr) {
    return false;
  }
  const double expected_z = 0.5 * (source_port_a->world_position.z + source_port_b->world_position.z);

  const ObjectId start_pole_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId end_pole_id = find_pole_id_by_position(state, {20.0, 0.0, 0.0});
  if (start_pole_id == wire::core::kInvalidObjectId || end_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span_id;
  pick.hit_pos_world = {10.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = start_pole_id;
  pick.segment_node_b_id = end_pole_id;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {20.0, 0.0, 0.0};

  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kLowVoltage};
  resolve.snap_radius_world = 0.75;
  resolve.create_midair_node = true;
  const auto picked_midair = state.ResolveBranchPick(pick, resolve);
  if (!picked_midair.ok || picked_midair.value.resolved_node_id == wire::core::kInvalidObjectId ||
      picked_midair.value.support_kind != wire::core::SupportKind::kMidair) {
    return false;
  }
  const ObjectId midair_id = picked_midair.value.resolved_node_id;
  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {20.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = midair_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok || generated_second.value.generated_span_ids.empty()) {
    return false;
  }

  const wire::core::Span* branch_span = nullptr;
  for (ObjectId span_id : generated_second.value.generated_span_ids) {
    const wire::core::Span* candidate = state.view().edit_state().spans.find(span_id);
    if (candidate == nullptr) {
      continue;
    }
    if (candidate->endpoint_node_a_id == midair_id || candidate->endpoint_node_b_id == midair_id) {
      branch_span = candidate;
      break;
    }
  }
  if (branch_span == nullptr) {
    return false;
  }
  const bool midair_is_a = branch_span->endpoint_node_a_id == midair_id;
  const bool midair_is_b = branch_span->endpoint_node_b_id == midair_id;
  if (!midair_is_a && !midair_is_b) {
    return false;
  }
  const ObjectId branch_port_id = midair_is_a ? branch_span->port_a_id : branch_span->port_b_id;
  const wire::core::Port* branch_port = state.view().edit_state().ports.find(branch_port_id);
  if (branch_port == nullptr) {
    return false;
  }
  return std::abs(branch_port->world_position.z - expected_z) <= 1e-6;
}

// Intent: Disallowed templates should not connect through a source-edge Midair branch, but the request should still succeed.
bool test_backbone_midair_branch_skips_disallowed_template_generation() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok || generated_first.value.generated_span_ids.size() != 1) {
    return false;
  }

  const ObjectId source_span_id = generated_first.value.generated_span_ids.front();
  const ObjectId start_pole_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId end_pole_id = find_pole_id_by_position(state, {20.0, 0.0, 0.0});
  if (start_pole_id == wire::core::kInvalidObjectId || end_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span_id;
  pick.hit_pos_world = {10.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = start_pole_id;
  pick.segment_node_b_id = end_pole_id;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {20.0, 0.0, 0.0};

  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kLowVoltage, wire::core::BundleKind::kHighVoltage};
  resolve.snap_radius_world = 0.75;
  resolve.create_midair_node = true;
  const auto picked_midair = state.ResolveBranchPick(pick, resolve);
  if (!picked_midair.ok || picked_midair.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {20.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = picked_midair.value.resolved_node_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kHighVoltage);
  const CoreCounts before_second = snapshot_counts(state);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  return generated_second.ok && generated_second.value.generated_pole_ids.empty() &&
         generated_second.value.generated_span_ids.empty() && generated_second.value.bundle_ids.empty() &&
         same_counts(before_second, snapshot_counts(state));
}

// Intent: Mixed template generation from a Midair branch should generate only bundles whose template allows midair branch.
bool test_backbone_midair_branch_generates_only_allowed_templates() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok || generated_first.value.generated_span_ids.size() != 1) {
    return false;
  }

  const ObjectId source_span_id = generated_first.value.generated_span_ids.front();
  const ObjectId start_pole_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId end_pole_id = find_pole_id_by_position(state, {20.0, 0.0, 0.0});
  if (start_pole_id == wire::core::kInvalidObjectId || end_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span_id;
  pick.hit_pos_world = {10.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = start_pole_id;
  pick.segment_node_b_id = end_pole_id;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {20.0, 0.0, 0.0};

  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kLowVoltage, wire::core::BundleKind::kHighVoltage};
  resolve.snap_radius_world = 0.75;
  resolve.create_midair_node = true;
  const auto picked_midair = state.ResolveBranchPick(pick, resolve);
  if (!picked_midair.ok || picked_midair.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {20.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = picked_midair.value.resolved_node_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(second, wire::core::BundleKind::kHighVoltage);

  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok || generated_second.value.generated_span_ids.empty() || generated_second.value.bundle_ids.size() != 1) {
    return false;
  }
  const wire::core::Bundle* bundle = state.view().edit_state().bundles.find(generated_second.value.bundle_ids.front());
  if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kLowVoltage) {
    return false;
  }
  for (const ObjectId span_id : generated_second.value.generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    const wire::core::Bundle* span_bundle = state.view().edit_state().bundles.find(span->bundle_id);
    if (span_bundle == nullptr || span_bundle->bundle_template_id != wire::core::BundleKind::kLowVoltage) {
      return false;
    }
  }
  return true;
}

// Intent: HV template keeps midair-branch policy disabled and rejects unsupported legacy mode values.

bool test_backbone_detail_generation_handles_building_support_node() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec building{};
  building.point_index = 1;
  building.support_kind = wire::core::SupportKind::kBuilding;
  req.path.node_specs.push_back(building);
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);

  const auto generated = state.GenerateFromBackboneSpec(req);
  return generated.ok && !generated.value.generated_span_ids.empty();
}

// Intent: Segment pick near endpoint snaps to node and does not add extra midair support node.

bool test_generate_grouped_line_high_voltage_three_phase() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  BackbonePathGenerateOptions options{};
  options.road.id = 1001;
  options.road.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 2.0, 0.0}, {30.0, 2.0, 0.0}};
  options.interval = 0.0;
  options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  options.direction_mode = wire::core::PathDirectionMode::kAuto;

  const auto result = generate_from_backbone_options(state, options);
  if (!result.ok) {
    return false;
  }
  if (result.value.pole_ids.size() != options.road.polyline.size()) {
    return false;
  }
  if (result.value.bundle_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const std::size_t expected_spans = (result.value.pole_ids.size() - 1) * 3;
  if (result.value.span_ids.size() != expected_spans) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(result.value.bundle_id);
  if (bundle == nullptr || bundle->conductor_count != 3) {
    return false;
  }
  if (result.value.lane_assignments.size() != result.value.pole_ids.size() - 1) {
    return false;
  }
  for (const auto& assignment : result.value.lane_assignments) {
    if (assignment.port_ids_a.size() != 3 || assignment.port_ids_b.size() != 3) {
      return false;
    }
  }
  for (const ObjectId span_id : result.value.span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    if (span->bundle_id != result.value.bundle_id) {
      return false;
    }
  }
  const auto validation = validate_now(state);
  if (!validation.ok()) {
    return false;
  }
  return true;
}

// Intent: Direction mode force flags should deterministically choose path orientation.
bool test_generate_grouped_line_direction_forced_reverse() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  BackbonePathGenerateOptions options{};
  options.road.id = 1002;
  options.road.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  options.interval = 0.0;
  options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  options.direction_mode = wire::core::PathDirectionMode::kReverse;

  const auto reverse = generate_from_backbone_options(state, options);
  if (!reverse.ok || reverse.value.pole_ids.empty()) {
    return false;
  }
  const auto* first_reverse = state.view().edit_state().poles.find(reverse.value.pole_ids.front());
  if (first_reverse == nullptr ||
      !almost_equal(first_reverse->world_transform.position, options.road.polyline.back())) {
    return false;
  }
  if (reverse.value.direction_debug.chosen != wire::core::PathDirectionChosen::kReverse) {
    return false;
  }
  return true;
}

// Intent: Group lane assignment on a U-shaped path should avoid lane-order inversions per segment.
bool test_grouped_line_lane_order_no_inversion_on_u_path() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  BackbonePathGenerateOptions options{};
  options.road.id = 1101;
  options.road.polyline = {
      {-18.0, -6.0, 0.0},
      {-6.0, -6.0, 0.0},
      {-6.0, 6.0, 0.0},
      {6.0, 6.0, 0.0},
      {6.0, -6.0, 0.0},
      {18.0, -6.0, 0.0},
  };
  options.interval = 0.0;
  options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  options.direction_mode = wire::core::PathDirectionMode::kAuto;

  const auto generated = generate_from_backbone_options(state, options);
  if (!generated.ok) {
    return false;
  }
  const LaneOrderMetrics metrics = compute_lane_order_metrics(state, generated.value.lane_assignments);
  if (metrics.y_inversions != 0) {
    dump_lane_assignment_debug(state, generated.value.lane_assignments, "C62_upath");
  }
  return metrics.y_inversions == 0;
}

// Intent: Group lane assignment on an acute corner path should avoid lane-order inversion.
bool test_grouped_line_acute_corner_no_inversion() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  BackbonePathGenerateOptions options{};
  options.road.id = 1103;
  options.road.polyline = {
      {-20.0, 0.0, 0.0},
      {-8.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {-2.0, 6.0, 0.0},
      {10.0, 6.0, 0.0},
  };
  options.interval = 0.0;
  options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kCommunication;
  options.bundle_count = 4;
  options.override_allow_mirror = true;
  options.allow_mirror = true;
  options.direction_mode = wire::core::PathDirectionMode::kAuto;

  const auto generated = generate_from_backbone_options(state, options);
  if (!generated.ok) {
    return false;
  }
  const LaneOrderMetrics metrics = compute_lane_order_metrics(state, generated.value.lane_assignments);
  return metrics.y_inversions == 0;
}

// Intent: Acute/zigzag path variants should avoid lane-order inversion under mirror two-choice policy.
bool test_grouped_line_acute_pattern_suite_no_inversion() {
  const std::vector<std::vector<wire::core::Vec3d>> paths = {
      {
          {-20.0, 0.0, 0.0},
          {-8.0, 0.0, 0.0},
          {2.0, 0.0, 0.0},
          {-2.0, 6.0, 0.0},
          {10.0, 6.0, 0.0},
      },
      {
          {-20.0, 0.0, 0.0},
          {-8.0, 0.0, 0.0},
          {2.0, 0.0, 0.0},
          {-2.0, -6.0, 0.0},
          {10.0, -6.0, 0.0},
      },
      {
          {-20.0, 0.0, 0.0},
          {-8.0, 0.0, 0.0},
          {0.0, 6.0, 0.0},
          {8.0, 0.0, 0.0},
          {16.0, 6.0, 0.0},
      },
      {
          {-18.0, -4.0, 0.0},
          {-8.0, -4.0, 0.0},
          {-2.0, 3.0, 0.0},
          {6.0, -3.0, 0.0},
          {14.0, 4.0, 0.0},
      },
  };

  for (std::size_t i = 0; i < paths.size(); ++i) {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return false;
    }
    BackbonePathGenerateOptions options{};
    options.road.id = 1200 + static_cast<wire::core::RoadId>(i);
    options.road.polyline = paths[i];
    options.interval = 0.0;
    options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kCommunication;
    options.bundle_count = 4;
    options.override_allow_mirror = true;
    options.allow_mirror = true;
    const auto generated = generate_from_backbone_options(state, options);
    if (!generated.ok) {
      return false;
    }
    const LaneOrderMetrics metrics = compute_lane_order_metrics(state, generated.value.lane_assignments);
    if (metrics.y_inversions != 0) {
      std::ostringstream tag;
      tag << "C86_path" << i;
      dump_lane_assignment_debug(state, generated.value.lane_assignments, tag.str().c_str());
      return false;
    }
  }
  return true;
}

// Intent: Acute high-voltage 3-phase path should also suppress lane-order inversion like other bundle types.
bool test_grouped_line_hv3_acute_no_phase_twist() {
  const std::vector<std::vector<wire::core::Vec3d>> paths = {
      {
          {-20.0, 0.0, 0.0},
          {-8.0, 0.0, 0.0},
          {2.0, 0.0, 0.0},
          {-2.0, 6.0, 0.0},
          {10.0, 6.0, 0.0},
      },
      {
          {-20.0, 0.0, 0.0},
          {-8.0, 0.0, 0.0},
          {2.0, 0.0, 0.0},
          {-2.0, -6.0, 0.0},
          {10.0, -6.0, 0.0},
      },
      {
          {-18.0, -4.0, 0.0},
          {-8.0, -4.0, 0.0},
          {-2.0, 3.0, 0.0},
          {6.0, -3.0, 0.0},
          {14.0, 4.0, 0.0},
      },
  };

  for (std::size_t i = 0; i < paths.size(); ++i) {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return false;
    }
    BackbonePathGenerateOptions options{};
    options.road.id = 1300 + static_cast<wire::core::RoadId>(i);
    options.road.polyline = paths[i];
    options.interval = 0.0;
    options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
    options.override_allow_mirror = true;
    options.allow_mirror = true;
    const auto generated = generate_from_backbone_options(state, options);
    if (!generated.ok) {
      return false;
    }
    const LaneOrderMetrics metrics = compute_lane_order_metrics(state, generated.value.lane_assignments);
    if (metrics.y_inversions != 0) {
      std::ostringstream tag;
      tag << "C87_path" << i;
      dump_lane_assignment_debug(state, generated.value.lane_assignments, tag.str().c_str());
      return false;
    }
  }
  return true;
}

// Intent: Captured zigzag DrawPath shape should keep HV3 bundle free of lane-order inversion.
bool test_backbone_hv3_capture_shape_no_inversion() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {19.6087, -16.6408, 0.0},
      {8.22759, -11.9276, 0.0},
      {16.8051, -20.9148, 0.0},
      {8.62249, -16.7209, 0.0},
      {12.2073, -24.74, 0.0},
      {4.69953, -21.4095, 0.0},
  };
  req.interval_m = 1000.0; // clicked points only
  req.pole_type_id = type_ids.front();
  wire::core::BackboneBundleSpec hv{};
  hv.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  req.bundles.push_back(hv);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    std::cerr << "[DBG] C99 generate_failed\n";
    return false;
  }
  const auto& orientations = state.view().last_generation_edge_orientations();
  if (orientations.empty()) {
    std::cerr << "[DBG] C99 orientations_empty\n";
    return false;
  }
  const auto assignments = collect_template_assignments(state, wire::core::BundleKind::kHighVoltage);
  const int polyline_intersections = count_bundle_lane_polyline_xy_intersections(state, assignments);
  const int adjacent_discontinuities = count_bundle_lane_adjacent_order_discontinuities(state, assignments);
  if (polyline_intersections != 0 || adjacent_discontinuities != 0) {
    dump_lane_assignment_debug(state, assignments, "C99_capture_shape");
    std::cerr << "[DBG] C99 polyline_intersections=" << polyline_intersections
              << " adjacent=" << adjacent_discontinuities << "\n";
    return false;
  }
  return true;
}

// Intent: Captured DrawPath shape should keep interior HV3 shared-pole lane order continuous.
bool test_backbone_hv3_capture_shape_no_adjacent_crossings() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {14.6925, -11.8125, 0.0},
      {0.711913, -17.8685, 0.0},
      {10.1084, -9.46792, 0.0},
      {-4.61348, -8.99765, 0.0},
      {-15.6287, -0.227707, 0.0},
      {-22.195, 6.65811, 0.0},
      {-12.9124, 15.285, 0.0},
      {-37.5524, 7.58507, 0.0},
  };
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  wire::core::BackboneBundleSpec hv{};
  hv.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  req.bundles.push_back(hv);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  const auto& assignments = state.view().last_lane_assignments();
  const int adjacent_discontinuities = count_bundle_lane_adjacent_order_discontinuities(state, assignments);
  if (adjacent_discontinuities != 0) {
    dump_lane_assignment_debug(state, assignments, "C109_capture_adjacent");
    return false;
  }
  return true;
}

// Intent: Latest captured CommunicationPole HV path should keep route-wide parity continuous instead of flipping mid-route.
bool test_backbone_hv3_latest_capture_shape_no_twist() {
  CoreState state;
  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [type_id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = type_id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C311 missing CommunicationPole\n";
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {2.38374, -21.7773, 0.0},
      {-12.8223, -15.1594, 0.0},
      {-17.5767, -14.0377, 0.0},
      {-16.1859, -5.45037, 0.0},
      {-15.6915, -4.78471, 0.0},
      {-13.6725, -5.22158, 0.0},
      {-12.7205, -9.02353, 0.0},
  };
  req.interval_m = 37.9821;
  req.pole_type_id = communication_pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    std::cerr << "[DBG] C311 generate_failed\n";
    return false;
  }

  std::string details;
  if (hv_route_has_parity_discontinuity(state, &details)) {
    std::cerr << "[DBG] C311 " << details << "\n";
    const auto assignments = collect_template_assignments(state, wire::core::BundleKind::kHighVoltage);
    dump_lane_assignment_debug(state, assignments, "C311_latest_capture_twist");
    dump_assignment_parity_bruteforce(state, assignments, "C311_latest_capture_twist");
    return false;
  }
  return true;
}

bool test_backbone_hv3_latest_capture_variant_bank_no_twist() {
  constexpr std::array<std::uint64_t, 24> kSeeds = {
      0x11ull, 0x22ull, 0x33ull, 0x44ull, 0x55ull, 0x66ull, 0x77ull, 0x88ull,
      0x99ull, 0xAAull, 0xBBull, 0xCCull, 0xDDull, 0xEEull, 0xFFull, 0x123ull,
      0x234ull, 0x345ull, 0x456ull, 0x567ull, 0x678ull, 0x789ull, 0x89Aull, 0x9ABull,
  };

  for (std::uint64_t seed : kSeeds) {
    CoreState state;
    PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
    for (const auto& [type_id, pole_type] : state.view().pole_types()) {
      if (pole_type.name == "CommunicationPole") {
        communication_pole_type_id = type_id;
        break;
      }
    }
    if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
      std::cerr << "[DBG] C314 missing CommunicationPole\n";
      return false;
    }

    wire::core::BackboneSpec req{};
    req.path.polyline = make_latest_capture_twist_variant_polyline(seed);
    req.interval_m = 37.9821;
    req.pole_type_id = communication_pole_type_id;
    add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
    const auto generated = state.GenerateFromBackboneSpec(req);
    if (!generated.ok) {
      std::cerr << "[DBG] C314 generate_failed seed=" << seed << " error=" << generated.error << "\n";
      return false;
    }

    std::string details{};
    if (hv_route_has_parity_discontinuity(state, &details)) {
      std::cerr << "[DBG] C314 seed=" << seed << " " << details << "\n";
      const auto assignments = collect_template_assignments(state, wire::core::BundleKind::kHighVoltage);
      dump_lane_assignment_debug(state, assignments, "C314_variant_bank_twist");
      dump_assignment_parity_bruteforce(state, assignments, "C314_variant_bank_twist");
      dump_bundle_lane_polyline_xy_intersections(state, assignments, "C314_variant_bank_twist");
      return false;
    }
  }

  return true;
}

// Intent: Latest captured CommunicationPole HV path should remain untwisted in final detail curves after recalc.
bool test_backbone_hv3_latest_capture_final_curve_no_twist() {
  CoreState state;
  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [type_id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = type_id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C316 missing CommunicationPole\n";
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {2.38374, -21.7773, 0.0},
      {-12.8223, -15.1594, 0.0},
      {-17.5767, -14.0377, 0.0},
      {-16.1859, -5.45037, 0.0},
      {-15.6915, -4.78471, 0.0},
      {-13.6725, -5.22158, 0.0},
      {-12.7205, -9.02353, 0.0},
  };
  req.interval_m = 37.9821;
  req.pole_type_id = communication_pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    std::cerr << "[DBG] C316 generate_failed\n";
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  const auto commit = state.Commit(options);
  if (!commit.validation.ok()) {
    std::cerr << "[DBG] C316 commit_failed\n";
    return false;
  }

  std::string details{};
  if (hv_route_has_final_curve_twist(state, &details)) {
    std::cerr << "[DBG] C316 " << details << "\n";
    const auto hv_assignments = collect_template_assignments(state, wire::core::BundleKind::kHighVoltage);
    dump_lane_assignment_debug(state, hv_assignments, "C316_final_curve_twist");
    dump_assignment_parity_bruteforce(state, hv_assignments, "C316_final_curve_twist");
    struct LaneCurveSegment {
      std::size_t segment_index = 0;
      std::size_t lane_index = 0;
      wire::core::ObjectId span_id = wire::core::kInvalidObjectId;
      wire::core::Vec3d a{};
      wire::core::Vec3d b{};
    };
    auto segments_intersect_xy_debug = [](const wire::core::Vec3d& a, const wire::core::Vec3d& b,
                                          const wire::core::Vec3d& c, const wire::core::Vec3d& d) {
      auto orient = [](const wire::core::Vec3d& p, const wire::core::Vec3d& q, const wire::core::Vec3d& r) {
        return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
      };
      const double o1 = orient(a, b, c);
      const double o2 = orient(a, b, d);
      const double o3 = orient(c, d, a);
      const double o4 = orient(c, d, b);
      return ((o1 > 1e-9 && o2 < -1e-9) || (o1 < -1e-9 && o2 > 1e-9)) &&
             ((o3 > 1e-9 && o4 < -1e-9) || (o3 < -1e-9 && o4 > 1e-9));
    };
    std::vector<LaneCurveSegment> lane_curve_segments{};
    for (const auto& assignment : state.view().last_lane_assignments()) {
      const auto* bundle = state.view().edit_state().bundles.find(assignment.bundle_id);
      if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
        continue;
      }
      const std::size_t lane_count = std::min(assignment.port_ids_a.size(), assignment.port_ids_b.size());
      for (std::size_t lane = 0; lane < lane_count; ++lane) {
        const wire::core::ObjectId port_a_id = assignment.port_ids_a[lane];
        const wire::core::ObjectId port_b_id = assignment.port_ids_b[lane];
        const wire::core::Span* matched_span = nullptr;
        for (const auto& span : state.view().edit_state().spans.items()) {
          const bool same_forward = span.port_a_id == port_a_id && span.port_b_id == port_b_id;
          const bool same_reverse = span.port_a_id == port_b_id && span.port_b_id == port_a_id;
          if (same_forward || same_reverse) {
            matched_span = &span;
            break;
          }
        }
        if (matched_span == nullptr) {
          continue;
        }
        const auto* curve = state.find_curve_cache(matched_span->id);
        if (curve == nullptr || curve->detail.sample_points.size() < 2) {
          continue;
        }
        const bool same_forward = matched_span->port_a_id == port_a_id && matched_span->port_b_id == port_b_id;
        const auto& points = curve->detail.sample_points;
        if (same_forward) {
          for (std::size_t i = 1; i < points.size(); ++i) {
            lane_curve_segments.push_back({assignment.segment_index, lane, matched_span->id, points[i - 1], points[i]});
          }
        } else {
          for (std::size_t i = points.size(); i-- > 1;) {
            lane_curve_segments.push_back({assignment.segment_index, lane, matched_span->id, points[i], points[i - 1]});
          }
        }
      }
    }
    int printed_intersections = 0;
    for (std::size_t i = 0; i < lane_curve_segments.size() && printed_intersections < 12; ++i) {
      for (std::size_t j = i + 1; j < lane_curve_segments.size() && printed_intersections < 12; ++j) {
        const auto& lhs = lane_curve_segments[i];
        const auto& rhs = lane_curve_segments[j];
        if (lhs.lane_index == rhs.lane_index) {
          continue;
        }
        if (!segments_intersect_xy_debug(lhs.a, lhs.b, rhs.a, rhs.b)) {
          continue;
        }
        ++printed_intersections;
        std::cerr << "[DBG] C316 intersection lhs(seg=" << lhs.segment_index << ",lane=" << lhs.lane_index
                  << ",span=" << lhs.span_id << ") rhs(seg=" << rhs.segment_index << ",lane=" << rhs.lane_index
                  << ",span=" << rhs.span_id << ")\n";
      }
    }
    for (const auto& span : state.view().edit_state().spans.items()) {
      const auto assignment = find_assignment_for_span(state, span.id);
      if (!assignment.has_value()) {
        continue;
      }
      const auto bundle = state.view().edit_state().bundles.find(span.bundle_id);
      if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
        continue;
      }
      const auto curve_view = state.view().inspect_detail_curve(span.id);
      const auto layout_view = state.view().inspect_support_layout(span.id);
      const auto* curve = state.find_curve_cache(span.id);
      if (!curve_view.has_value() || !layout_view.has_value() || curve == nullptr) {
        continue;
      }
      std::cerr << "[DBG] C316 span=" << span.id << " seg=" << assignment->segment_index
                << " flow=" << static_cast<int>(assignment->flow_kind)
                << " shape=" << static_cast<int>(curve_view->shape_policy)
                << " startRule=" << static_cast<int>(curve_view->start_tangent_rule)
                << " endRule=" << static_cast<int>(curve_view->end_tangent_rule)
                << " startW=" << curve_view->start_support_weight
                << " endW=" << curve_view->end_support_weight
                << " startDep=" << curve_view->start_departure_length_m
                << " endDep=" << curve_view->end_departure_length_m
                << " sameLevel=" << (layout_view->same_level_feasible ? 1 : 0)
                << " lower=" << (layout_view->default_lower_required ? 1 : 0)
                << " samples=" << curve->detail.sample_points.size()
                << " startEndpoint=" << layout_view->start_endpoint.endpoint_world.x << ","
                << layout_view->start_endpoint.endpoint_world.y
                << " startDir=" << layout_view->start_endpoint.departure_dir.x << ","
                << layout_view->start_endpoint.departure_dir.y
                << " endEndpoint=" << layout_view->end_endpoint.endpoint_world.x << ","
                << layout_view->end_endpoint.endpoint_world.y
                << " endDir=" << layout_view->end_endpoint.departure_dir.x << ","
                << layout_view->end_endpoint.departure_dir.y << "\n";
    }
    for (const auto& pole : state.view().edit_state().poles.items()) {
      const auto pole_view = state.view().inspect_pole(pole.id);
      if (!pole_view.has_value() || pole_view->row_layout_axis_category != wire::core::ConnectionCategory::kHighVoltage) {
        continue;
      }
      std::cerr << "[DBG] C316 pole=" << pole.id << " yaw=" << pole_view->final_yaw_deg
                << " layoutYaw=" << pole_view->layout_yaw_deg
                << " supportRule=" << static_cast<int>(pole_view->support_axis_rule)
                << " rowMode=" << static_cast<int>(pole_view->row_layout_axis_mode)
                << " axisCat=" << static_cast<int>(pole_view->row_layout_axis_category)
                << " supportAxis=" << pole_view->support_axis_dir.x << "," << pole_view->support_axis_dir.y
                << " forward=" << pole_view->forward_dir.x << "," << pole_view->forward_dir.y << "\n";
    }
    return false;
  }
  return true;
}

// Intent: Latest captured CommunicationPole HV path should keep lane polylines non-crossing across Commit(recalc).
bool test_backbone_hv3_latest_capture_commit_preserves_polyline_order() {
  CoreState state;
  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [type_id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = type_id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {2.38374, -21.7773, 0.0},
      {-12.8223, -15.1594, 0.0},
      {-17.5767, -14.0377, 0.0},
      {-16.1859, -5.45037, 0.0},
      {-15.6915, -4.78471, 0.0},
      {-13.6725, -5.22158, 0.0},
      {-12.7205, -9.02353, 0.0},
  };
  req.interval_m = 37.9821;
  req.pole_type_id = communication_pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  const int before_intersections_raw =
      count_bundle_lane_polyline_xy_intersections(state, state.view().last_lane_assignments());
  const auto before_snapshots = capture_hv_endpoint_snapshots(state);
  const int before_intersections =
      count_bundle_lane_polyline_xy_intersections(state, state.view().last_lane_assignments());
  if (before_intersections_raw != before_intersections) {
    std::cerr << "[DBG] C319 snapshot_side_effect raw=" << before_intersections_raw
              << " afterSnapshot=" << before_intersections << "\n";
  }
  if (before_intersections != 0) {
    std::cerr << "[DBG] C319 before_commit_polyline_intersections=" << before_intersections << "\n";
    dump_lane_assignment_debug(state, state.view().last_lane_assignments(), "C319_before_commit_polyline");
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  const auto commit = state.Commit(options);
  if (!commit.validation.ok()) {
    return false;
  }

  const auto after_snapshots = capture_hv_endpoint_snapshots(state);
  const int after_intersections =
      count_bundle_lane_polyline_xy_intersections(state, state.view().last_lane_assignments());
  if (after_intersections != 0) {
    std::cerr << "[DBG] C319 after_commit_polyline_intersections=" << after_intersections << "\n";
    dump_hv_endpoint_snapshot_diff(before_snapshots, after_snapshots, "C319_commit_diff");
    dump_lane_assignment_debug(state, state.view().last_lane_assignments(), "C319_commit_polyline");
    return false;
  }
  return true;
}

// Intent: Near-duplicate auto generic candidates collapse only below the 1.5m threshold.
bool test_backbone_auto_candidate_near_duplicate_boundary() {
  std::string error{};
  const auto obs_14 = observe_backbone_candidates(
      {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {11.4, 0.0, 0.0}, {25.0, 0.0, 0.0}}, 1000.0, false, false, {}, &error);
  if (!obs_14.has_value()) {
    std::cerr << "[DBG] C320 obs_14 failed: " << error << "\n";
    return false;
  }
  const auto obs_15 = observe_backbone_candidates(
      {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {11.5, 0.0, 0.0}, {25.0, 0.0, 0.0}}, 1000.0, false, false, {}, &error);
  if (!obs_15.has_value()) {
    std::cerr << "[DBG] C320 obs_15 failed: " << error << "\n";
    return false;
  }
  const auto obs_16 = observe_backbone_candidates(
      {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {11.6, 0.0, 0.0}, {25.0, 0.0, 0.0}}, 1000.0, false, false, {}, &error);
  if (!obs_16.has_value()) {
    std::cerr << "[DBG] C320 obs_16 failed: " << error << "\n";
    return false;
  }

  const bool ok_14 = obs_14->nodes.size() == 3 &&
                     find_backbone_node_near_world(*obs_14, {10.0, 0.0, 0.0}) == nullptr &&
                     find_backbone_node_near_world(*obs_14, {11.4, 0.0, 0.0}) != nullptr;
  const bool ok_15 = obs_15->nodes.size() == 4 && find_backbone_node_near_world(*obs_15, {10.0, 0.0, 0.0}) != nullptr &&
                     find_backbone_node_near_world(*obs_15, {11.5, 0.0, 0.0}) != nullptr;
  const bool ok_16 = obs_16->nodes.size() == 4 && find_backbone_node_near_world(*obs_16, {10.0, 0.0, 0.0}) != nullptr &&
                     find_backbone_node_near_world(*obs_16, {11.6, 0.0, 0.0}) != nullptr;
  if (!(ok_14 && ok_15 && ok_16)) {
    std::cerr << "[DBG] C320 sizes=" << obs_14->nodes.size() << "," << obs_15->nodes.size() << ","
              << obs_16->nodes.size() << "\n";
    return false;
  }
  return true;
}

// Intent: Manual vertices and tangent-hint vertices must not be collapsed by near-duplicate auto candidate cleanup.
bool test_backbone_manual_and_tangent_candidates_do_not_collapse() {
  std::string error{};
  const std::vector<wire::core::Vec3d> polyline = {
      {0.0, 0.0, 0.0},
      {10.0, 0.0, 0.0},
      {11.4, 0.0, 0.0},
      {25.0, 0.0, 0.0},
  };
  const auto manual_obs = observe_backbone_candidates(polyline, 1000.0, true, false, {}, &error);
  if (!manual_obs.has_value()) {
    std::cerr << "[DBG] C321 manual failed: " << error << "\n";
    return false;
  }
  wire::core::BackboneInputSpec::NodeSpec tangent{};
  tangent.point_index = 2;
  tangent.has_tangent_hint = true;
  tangent.tangent_hint = {1.0, 0.0, 0.0};
  const auto tangent_obs = observe_backbone_candidates(polyline, 1000.0, false, false, {tangent}, &error);
  if (!tangent_obs.has_value()) {
    std::cerr << "[DBG] C321 tangent failed: " << error << "\n";
    return false;
  }

  const bool manual_ok = manual_obs->nodes.size() == 4 &&
                         find_backbone_node_near_world(*manual_obs, {10.0, 0.0, 0.0}) != nullptr &&
                         find_backbone_node_near_world(*manual_obs, {11.4, 0.0, 0.0}) != nullptr;
  const bool tangent_ok = tangent_obs->nodes.size() == 4 &&
                          find_backbone_node_near_world(*tangent_obs, {10.0, 0.0, 0.0}) != nullptr &&
                          find_backbone_node_near_world(*tangent_obs, {11.4, 0.0, 0.0}) != nullptr;
  if (!(manual_ok && tangent_ok)) {
    std::cerr << "[DBG] C321 manual_size=" << manual_obs->nodes.size()
              << " tangent_size=" << tangent_obs->nodes.size()
              << " tangent_has_11_4="
              << (find_backbone_node_near_world(*tangent_obs, {11.4, 0.0, 0.0}) != nullptr ? 1 : 0) << "\n";
    return false;
  }
  return true;
}

// Intent: Short unpinned start/end segments should keep endpoint candidates instead of erasing the route endpoints.
bool test_backbone_short_unpinned_endpoint_segments_keep_endpoints() {
  std::string error{};
  const auto leading_obs = observe_backbone_candidates(
      {{0.0, 0.0, 0.0}, {1.4, 0.0, 0.0}, {20.0, 0.0, 0.0}}, 1000.0, false, false, {}, &error);
  if (!leading_obs.has_value()) {
    std::cerr << "[DBG] C322 leading failed: " << error << "\n";
    return false;
  }
  const auto trailing_obs = observe_backbone_candidates(
      {{0.0, 0.0, 0.0}, {18.6, 0.0, 0.0}, {20.0, 0.0, 0.0}}, 1000.0, false, false, {}, &error);
  if (!trailing_obs.has_value()) {
    std::cerr << "[DBG] C322 trailing failed: " << error << "\n";
    return false;
  }

  const bool leading_ok = leading_obs->nodes.size() == 2 &&
                          find_backbone_node_near_world(*leading_obs, {0.0, 0.0, 0.0}) != nullptr &&
                          find_backbone_node_near_world(*leading_obs, {20.0, 0.0, 0.0}) != nullptr &&
                          find_backbone_node_near_world(*leading_obs, {1.4, 0.0, 0.0}) == nullptr;
  const bool trailing_ok = trailing_obs->nodes.size() == 2 &&
                           find_backbone_node_near_world(*trailing_obs, {0.0, 0.0, 0.0}) != nullptr &&
                           find_backbone_node_near_world(*trailing_obs, {20.0, 0.0, 0.0}) != nullptr &&
                           find_backbone_node_near_world(*trailing_obs, {18.6, 0.0, 0.0}) == nullptr;
  if (!(leading_ok && trailing_ok)) {
    std::cerr << "[DBG] C322 leading_size=" << leading_obs->nodes.size()
              << " trailing_size=" << trailing_obs->nodes.size()
              << " leading_has_start=" << (find_backbone_node_near_world(*leading_obs, {0.0, 0.0, 0.0}) != nullptr ? 1 : 0)
              << " trailing_has_end=" << (find_backbone_node_near_world(*trailing_obs, {20.0, 0.0, 0.0}) != nullptr ? 1 : 0)
              << "\n";
    return false;
  }
  return true;
}

// Intent: Latest captured lowered HV spans should resolve through the grouped-lowered local-departure profile.
bool test_backbone_hv3_latest_capture_lowered_support_uses_local_departure_profile() {
  CoreState state;
  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [type_id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = type_id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {2.38374, -21.7773, 0.0},
      {-12.8223, -15.1594, 0.0},
      {-17.5767, -14.0377, 0.0},
      {-16.1859, -5.45037, 0.0},
      {-15.6915, -4.78471, 0.0},
      {-13.6725, -5.22158, 0.0},
      {-12.7205, -9.02353, 0.0},
  };
  req.interval_m = 37.9821;
  req.pole_type_id = communication_pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  bool saw_lowered_hv = false;
  for (const auto& span : state.view().edit_state().spans.items()) {
    const auto* bundle = state.view().edit_state().bundles.find(span.bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    const auto* support_layout = state.support_layout_projection(span.id).layout;
    if (support_layout == nullptr) {
      continue;
    }
    const bool same_level_feasible =
        support_layout->start.same_level_feasible && support_layout->end.same_level_feasible;
    const bool default_lower_required =
        support_layout->start.default_lower_required || support_layout->end.default_lower_required;
    if (same_level_feasible || !default_lower_required) {
      continue;
    }
    saw_lowered_hv = true;
    if (support_layout->detail_curve_profile_hint != wire::core::CurveProfileHint::kGroupedLoweredSupport) {
      return false;
    }
  }
  return saw_lowered_hv;
}

// Intent: Latest captured lowered HV spans should keep one shared route-local side-axis across an interior lowered corner.
bool test_backbone_hv3_latest_capture_lowered_support_departure_uses_shared_route_axis() {
  CoreState state;
  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [type_id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = type_id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {2.38374, -21.7773, 0.0},
      {-12.8223, -15.1594, 0.0},
      {-17.5767, -14.0377, 0.0},
      {-16.1859, -5.45037, 0.0},
      {-15.6915, -4.78471, 0.0},
      {-13.6725, -5.22158, 0.0},
      {-12.7205, -9.02353, 0.0},
  };
  req.interval_m = 37.9821;
  req.pole_type_id = communication_pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  struct LoweredEndpointSnapshot {
    wire::core::ObjectId span_id = wire::core::kInvalidObjectId;
    wire::core::ObjectId node_id = wire::core::kInvalidObjectId;
    bool is_start = false;
    bool has_side_axis = false;
    wire::core::SupportOrientationRuleKind support_orientation_rule = wire::core::SupportOrientationRuleKind::kRadial;
    wire::core::Vec3d side_axis{};
  };

  std::vector<LoweredEndpointSnapshot> lowered_endpoints{};
  bool saw_lowered_hv = false;
  for (const auto& span : state.view().edit_state().spans.items()) {
    const auto* bundle = state.view().edit_state().bundles.find(span.bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    const auto* support_layout = state.support_layout_projection(span.id).layout;
    if (support_layout == nullptr) {
      continue;
    }
    const bool same_level_feasible =
        support_layout->start.same_level_feasible && support_layout->end.same_level_feasible;
    const bool default_lower_required =
        support_layout->start.default_lower_required || support_layout->end.default_lower_required;
    if (same_level_feasible || !default_lower_required) {
      continue;
    }
    saw_lowered_hv = true;
    lowered_endpoints.push_back({span.id, span.endpoint_node_a_id, true, support_layout->start.has_side_axis,
                                 support_layout->start.support_orientation_rule, support_layout->start.side_axis});
    lowered_endpoints.push_back({span.id, span.endpoint_node_b_id, false, support_layout->end.has_side_axis,
                                 support_layout->end.support_orientation_rule, support_layout->end.side_axis});
  }
  if (!saw_lowered_hv) {
    return false;
  }

  for (std::size_t i = 0; i < lowered_endpoints.size(); ++i) {
    for (std::size_t j = i + 1; j < lowered_endpoints.size(); ++j) {
      const auto& a = lowered_endpoints[i];
      const auto& b = lowered_endpoints[j];
      if (a.node_id == wire::core::kInvalidObjectId || a.node_id != b.node_id || a.span_id == b.span_id) {
        continue;
      }
      if (!a.has_side_axis || !b.has_side_axis) {
        return false;
      }
      if (a.support_orientation_rule == wire::core::SupportOrientationRuleKind::kChord ||
          b.support_orientation_rule == wire::core::SupportOrientationRuleKind::kChord) {
        return false;
      }
      wire::core::Vec3d axis_a = a.side_axis;
      wire::core::Vec3d axis_b = b.side_axis;
      if (!wire::core::Normalize(&axis_a) || !wire::core::Normalize(&axis_b)) {
        return false;
      }
      if (std::abs(wire::core::Dot(axis_a, axis_b)) < 0.98) {
        return false;
      }
      return true;
    }
  }

  if (saw_lowered_hv) {
    // The latest capture should expose at least one interior lowered-corner node shared by adjacent HV spans.
    return false;
  }
  return true;
}

// Intent: ThreePhase group-kind should still permit mirror-two-choice when allow_lane_mirror=true.
bool test_grouped_line_threephase_policy_is_category_agnostic() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  BackbonePathGenerateOptions options{};
  options.road.id = 1308;
  options.road.polyline = {
      {-20.0, 0.0, 0.0},
      {-8.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {-2.0, 6.0, 0.0},
      {10.0, 6.0, 0.0},
  };
  options.interval = 0.0;
  options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  options.override_allow_mirror = true;
  options.allow_mirror = true;
  const auto generated = generate_from_backbone_options(state, options);
  if (!generated.ok) {
    return false;
  }
  const LaneOrderMetrics metrics = compute_lane_order_metrics(state, generated.value.lane_assignments);
  if (metrics.y_inversions != 0) {
    dump_lane_assignment_debug(state, generated.value.lane_assignments, "C89_hv_agnostic");
  }
  return metrics.y_inversions == 0;
}

// Intent: Path extension should preserve lane order at boundary pole between existing and newly generated segment.
bool test_backbone_extension_preserves_boundary_lane_order() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  auto make_request = [&](const std::vector<wire::core::Vec3d>& path) {
    wire::core::BackboneSpec req{};
    req.path.polyline = path;
    req.interval_m = 1000.0; // keep clicked vertices only
    req.pole_type_id = type_ids.front();
    wire::core::BackboneBundleSpec hv{};
    hv.bundle_template_id = wire::core::BundleKind::kHighVoltage;
    req.bundles.push_back(hv);
    return req;
  };

  const std::vector<wire::core::Vec3d> base_path = {
      {19.6087, -16.6408, 0.0},
      {8.22759, -11.9276, 0.0},
      {16.8051, -20.9148, 0.0},
      {8.62249, -16.7209, 0.0},
      {12.2073, -24.74, 0.0},
  };
  const std::vector<wire::core::Vec3d> extended_path = {
      {19.6087, -16.6408, 0.0},
      {8.22759, -11.9276, 0.0},
      {16.8051, -20.9148, 0.0},
      {8.62249, -16.7209, 0.0},
      {12.2073, -24.74, 0.0},
      {4.69953, -21.4095, 0.0},
  };

  const auto first = state.GenerateFromBackboneSpec(make_request(base_path));
  if (!first.ok) {
    return false;
  }
  const auto& first_orientations = state.view().last_generation_edge_orientations();
  if (first_orientations.empty()) {
    return false;
  }
  const auto* tail = &first_orientations.back();

  const auto second = state.GenerateFromBackboneSpec(make_request(extended_path));
  if (!second.ok) {
    return false;
  }
  const auto& second_orientations = state.view().last_generation_edge_orientations();
  const wire::core::BackboneEdgeOrientation* boundary = nullptr;
  for (const auto& orientation : second_orientations) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    if (orientation.node_a_id == tail->node_a_id && orientation.node_b_id == tail->node_b_id) {
      boundary = &orientation;
      break;
    }
  }
  if (boundary == nullptr) {
    for (const auto& orientation : second_orientations) {
      if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
        continue;
      }
      if (orientation.flipped_from_previous) {
        return false;
      }
    }
    return !second_orientations.empty();
  }
  return boundary->orientation == tail->orientation;
}

// Intent: Interval-driven extension should preserve boundary lane order without relying on stale cached assignments.
bool test_backbone_interval_extension_preserves_boundary_lane_order() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  auto make_request = [&](const std::vector<wire::core::Vec3d>& path) {
    wire::core::BackboneSpec req{};
    req.path.polyline = path;
    req.interval_m = 8.0;
    req.pole_type_id = type_ids.front();
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    return req;
  };

  const std::vector<wire::core::Vec3d> base_path = {
      {19.6087, -16.6408, 0.0},
      {8.22759, -11.9276, 0.0},
      {16.8051, -20.9148, 0.0},
      {8.62249, -16.7209, 0.0},
      {12.2073, -24.74, 0.0},
  };
  const std::vector<wire::core::Vec3d> extended_path = {
      {19.6087, -16.6408, 0.0},
      {8.22759, -11.9276, 0.0},
      {16.8051, -20.9148, 0.0},
      {8.62249, -16.7209, 0.0},
      {12.2073, -24.74, 0.0},
      {4.69953, -21.4095, 0.0},
  };

  const auto first = state.GenerateFromBackboneSpec(make_request(base_path));
  if (!first.ok) {
    return false;
  }
  const auto first_assignments = state.view().last_lane_assignments();
  if (count_bundle_lane_adjacent_order_discontinuities(state, first_assignments) != 0) {
    return false;
  }

  const auto second = state.GenerateFromBackboneSpec(make_request(extended_path));
  if (!second.ok) {
    return false;
  }
  const auto second_assignments = state.view().last_lane_assignments();
  const LaneOrderMetrics second_metrics = compute_lane_order_metrics(state, second_assignments);
  const int second_adjacent_discontinuities = count_bundle_lane_adjacent_order_discontinuities(state, second_assignments);
  if (second_metrics.y_inversions != 0 || second_adjacent_discontinuities != 0) {
    dump_lane_assignment_debug(state, second_assignments, "C208_interval_extension");
    return false;
  }
  return true;
}

// Intent: Enabling lane mirror must not worsen grouped-lane quality metrics (Y/Z order and layer continuity).
bool test_grouped_line_mirror_metric_non_regression() {
  const auto run_with_mirror = [](bool allow_mirror) -> std::pair<bool, LaneOrderMetrics> {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return {false, {}};
    }
    BackbonePathGenerateOptions options{};
    options.road.id = 1102;
    options.road.polyline = {
        {-20.0, 0.0, 0.0},
        {-8.0, 0.0, 0.0},
        {-8.0, 10.0, 0.0},
        {8.0, 10.0, 0.0},
        {8.0, -2.0, 0.0},
        {20.0, -2.0, 0.0},
    };
    options.interval = 0.0;
    options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kCommunication;
    options.bundle_count = 4;
    options.override_allow_mirror = true;
    options.allow_mirror = allow_mirror;
    options.direction_mode = wire::core::PathDirectionMode::kAuto;

    const auto generated = generate_from_backbone_options(state, options);
    if (!generated.ok) {
      return {false, {}};
    }
    return {true, compute_lane_order_metrics(state, generated.value.lane_assignments)};
  };

  const auto without_mirror = run_with_mirror(false);
  const auto with_mirror = run_with_mirror(true);
  if (!without_mirror.first || !with_mirror.first) {
    return false;
  }

  return with_mirror.second.weighted_score() <= without_mirror.second.weighted_score() &&
         with_mirror.second.y_inversions <= without_mirror.second.y_inversions &&
         with_mirror.second.z_inversions <= without_mirror.second.z_inversions;
}

// Intent: In T-junction, earlier DrawPath session should keep order=0 primary incident.

bool test_generate_from_backbone_spec_basic_hv_default_lanes() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 4.0, 0.0}};
  req.interval_m = 6.0;
  req.pole_type_id = type_ids.front();
  req.direction_mode = wire::core::PathDirectionMode::kAuto;
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);

  const auto result = state.GenerateFromBackboneSpec(req);
  if (!result.ok) {
    return false;
  }
  if (result.value.bundle_id == wire::core::kInvalidObjectId) {
    return false;
  }
  if (result.value.generated_pole_ids.size() < 2 || result.value.generated_span_ids.empty()) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(result.value.bundle_id);
  if (bundle == nullptr || bundle->conductor_count != 3) {
    return false;
  }
  for (const ObjectId span_id : result.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr || span->bundle_id != result.value.bundle_id) {
      return false;
    }
  }
  return validate_now(state).ok();
}

// Intent: Direction modes on path generation should all execute without failure.
bool test_generate_from_backbone_spec_direction_modes_nonfailing() {
  const std::array<wire::core::PathDirectionMode, 3> modes = {
      wire::core::PathDirectionMode::kForward,
      wire::core::PathDirectionMode::kReverse,
      wire::core::PathDirectionMode::kAuto,
  };
  for (const auto mode : modes) {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return false;
    }
    wire::core::BackboneSpec req{};
    req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 2.0, 0.0}};
    req.interval_m = 5.0;
    req.pole_type_id = type_ids.front();
    req.direction_mode = mode;
    add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
    const auto result = state.GenerateFromBackboneSpec(req);
    if (!result.ok || result.value.generated_span_ids.empty()) {
      return false;
    }
  }
  return true;
}

// Intent: Backbone generation should reject invalid input and keep state recoverable.
bool test_generate_from_backbone_spec_invalid_inputs_fail() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const CoreCounts before = snapshot_counts(state);

  wire::core::BackboneSpec too_short{};
  too_short.path.polyline = {{0.0, 0.0, 0.0}};
  too_short.interval_m = 5.0;
  too_short.pole_type_id = type_ids.front();
  add_backbone_bundle(too_short, wire::core::BundleKind::kLowVoltage);
  const auto r_short = state.GenerateFromBackboneSpec(too_short);
  if (r_short.ok || !regex_contains(r_short.error, "at least 2 points")) {
    return false;
  }

  wire::core::BackboneSpec bad_interval{};
  bad_interval.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  bad_interval.interval_m = 0.0;
  bad_interval.pole_type_id = type_ids.front();
  add_backbone_bundle(bad_interval, wire::core::BundleKind::kLowVoltage);
  const auto r_interval = state.GenerateFromBackboneSpec(bad_interval);
  if (r_interval.ok || !regex_contains(r_interval.error, "interval_m")) {
    return false;
  }

  wire::core::BackboneSpec bad_template{};
  bad_template.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  bad_template.interval_m = 5.0;
  bad_template.pole_type_id = type_ids.front();
  add_backbone_bundle(bad_template, static_cast<wire::core::BundleKind>(255));
  const auto r_category = state.GenerateFromBackboneSpec(bad_template);
  if (r_category.ok || !regex_contains(r_category.error, "template")) {
    return false;
  }

  wire::core::BackboneSpec bad_type{};
  bad_type.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  bad_type.interval_m = 5.0;
  bad_type.pole_type_id = 999999;
  add_backbone_bundle(bad_type, wire::core::BundleKind::kLowVoltage);
  const auto r_type = state.GenerateFromBackboneSpec(bad_type);
  if (r_type.ok || !regex_contains(r_type.error, "pole type")) {
    return false;
  }

  if (!same_counts(before, snapshot_counts(state))) {
    return false;
  }

  wire::core::BackboneSpec recover{};
  recover.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 0.0, 0.0}};
  recover.interval_m = 6.0;
  recover.pole_type_id = type_ids.front();
  add_backbone_bundle(recover, wire::core::BundleKind::kLowVoltage);
  return state.GenerateFromBackboneSpec(recover).ok;
}

bool test_backbone_reused_junction_pole_keeps_main_chain_forward() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  const auto main_generated = state.GenerateFromBackboneSpec(main_req);
  if (!main_generated.ok) {
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (shared.node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok) {
    return false;
  }

  const auto* center = state.view().edit_state().poles.find(shared.node_id);
  if (center == nullptr) {
    return false;
  }
  const double yaw = effective_pole_yaw_deg_test(*center);
  const bool horizontal =
      std::min(angle_diff_abs_deg(yaw, 0.0), angle_diff_abs_deg(yaw, 180.0)) <= 1e-6;
  const auto it_debug = state.view().pole_orientation_debug_records().find(shared.node_id);
  return horizontal && it_debug != state.view().pole_orientation_debug_records().end() &&
         it_debug->second.rule == wire::core::PoleForwardRule::kMainChainBisector;
}

bool test_backbone_branch_bundle_uses_branch_support_ports() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  const auto main_generated = state.GenerateFromBackboneSpec(main_req);
  if (!main_generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C174 center_missing_after_horizontal\n";
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.empty()) {
    return false;
  }

  int lowered_center_endpoints = 0;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      return false;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    const auto group = lowered_support_group_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || !group.has_value() || !endpoint_has_authoritative_lowering(*endpoint) ||
        endpoint->relation_kind != wire::core::JunctionRelationKind::kSideBranch ||
        group->support_group_id != endpoint->support_group_id || endpoint->branch_down_offset_m <= 1e-6) {
      return false;
    }
    ++lowered_center_endpoints;
  }
  return lowered_center_endpoints == 3;
}

bool test_backbone_branch_support_offsets_height_without_changing_layer() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  const auto main_generated = state.GenerateFromBackboneSpec(main_req);
  if (!main_generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  std::vector<const wire::core::Port*> main_ports{};
  for (ObjectId span_id : main_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    if (span->endpoint_node_a_id == center_id) {
      if (const auto* port = state.view().edit_state().ports.find(span->port_a_id); port != nullptr) {
        main_ports.push_back(port);
      }
    }
    if (span->endpoint_node_b_id == center_id) {
      if (const auto* port = state.view().edit_state().ports.find(span->port_b_id); port != nullptr) {
        main_ports.push_back(port);
      }
    }
  }
  if (main_ports.empty()) {
    return false;
  }
  double main_min_z = std::numeric_limits<double>::infinity();
  int main_layer = -1;
  for (const auto* port : main_ports) {
    main_min_z = std::min(main_min_z, port->world_position.z);
    if (main_layer < 0) {
      main_layer = port->template_layer;
    }
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.empty()) {
    return false;
  }

  const wire::core::Port* branch_port = nullptr;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    if (span->endpoint_node_a_id == center_id) {
      branch_port = state.view().edit_state().ports.find(span->port_a_id);
      break;
    }
    if (span->endpoint_node_b_id == center_id) {
      branch_port = state.view().edit_state().ports.find(span->port_b_id);
      break;
    }
  }
  return branch_port != nullptr && branch_port->placement_source == wire::core::PortPlacementSourceKind::kBranchSupport &&
         branch_port->world_position.z + 1e-6 < main_min_z && branch_port->template_layer == main_layer;
}

bool test_backbone_branch_support_lowers_hv3_bundle_uniformly() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }
  double main_min_z = std::numeric_limits<double>::infinity();
  for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
    if (port.owner_pole_id != center_id || port.layer != wire::core::PortLayer::kHighVoltage) {
      continue;
    }
    main_min_z = std::min(main_min_z, port.world_position.z);
  }
  if (!std::isfinite(main_min_z)) {
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.size() != 3) {
    return false;
  }

  std::vector<const wire::core::Port*> branch_ports{};
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    if (span->endpoint_node_a_id == center_id) {
      branch_ports.push_back(state.view().edit_state().ports.find(span->port_a_id));
    } else if (span->endpoint_node_b_id == center_id) {
      branch_ports.push_back(state.view().edit_state().ports.find(span->port_b_id));
    }
  }
  if (branch_ports.size() != 3 ||
      std::any_of(branch_ports.begin(), branch_ports.end(), [](const wire::core::Port* p) { return p == nullptr; })) {
    return false;
  }

  const ObjectId branch_tip_id = find_pole_id_by_position(state, {0.0, 12.0, 0.0});
  if (branch_tip_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto* center_pole = state.view().edit_state().poles.find(center_id);
  const auto* branch_tip_pole = state.view().edit_state().poles.find(branch_tip_id);
  if (center_pole == nullptr || branch_tip_pole == nullptr) {
    return false;
  }
  const wire::core::Vec3d branch_dir =
      helpers::normalize_xy_safe(branch_tip_pole->world_transform.position - center_pole->world_transform.position);
  const wire::core::Vec3d row_axis{-branch_dir.y, branch_dir.x, 0.0};
  std::sort(branch_ports.begin(), branch_ports.end(), [&](const wire::core::Port* a, const wire::core::Port* b) {
    const double da = helpers::dot_xy(a->world_position - center_pole->world_transform.position, row_axis);
    const double db = helpers::dot_xy(b->world_position - center_pole->world_transform.position, row_axis);
    return da < db;
  });

  const double z0 = branch_ports[0]->world_position.z;
  const double z1 = branch_ports[1]->world_position.z;
  const double z2 = branch_ports[2]->world_position.z;
  const double min_z = std::min({z0, z1, z2});
  const double max_z = std::max({z0, z1, z2});
  const double x_span =
      std::max({branch_ports[0]->world_position.x, branch_ports[1]->world_position.x, branch_ports[2]->world_position.x}) -
      std::min({branch_ports[0]->world_position.x, branch_ports[1]->world_position.x, branch_ports[2]->world_position.x});
  const double y_span =
      std::max({branch_ports[0]->world_position.y, branch_ports[1]->world_position.y, branch_ports[2]->world_position.y}) -
      std::min({branch_ports[0]->world_position.y, branch_ports[1]->world_position.y, branch_ports[2]->world_position.y});
  int support_group_id = -1;
  double down_offset_m = -1.0;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      return false;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    const auto group = lowered_support_group_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || !group.has_value() || !endpoint_has_authoritative_lowering(*endpoint) ||
        endpoint->relation_kind != wire::core::JunctionRelationKind::kSideBranch) {
      return false;
    }
    if (support_group_id < 0) {
      support_group_id = endpoint->support_group_id;
      down_offset_m = group->down_offset_m;
    } else if (support_group_id != endpoint->support_group_id ||
               !almost_equal(down_offset_m, group->down_offset_m, 1e-9)) {
      return false;
    }
  }
  const bool perpendicular_row = x_span > y_span * 2.0;
  const bool uniform_height = (max_z - min_z) <= 1e-6;
  const bool has_grouped_lowering = support_group_id >= 0 && down_offset_m > 1e-6;
  if (!uniform_height || !has_grouped_lowering) {
    std::cerr << "[DBG] C193 z0=" << z0 << " z1=" << z1 << " z2=" << z2 << " zSpan=" << (max_z - min_z)
              << " xSpan=" << x_span << " ySpan=" << y_span << " groupId=" << support_group_id
              << " downOffset=" << down_offset_m << "\n";
    }
  return uniform_height && perpendicular_row && max_z + 1e-6 < main_min_z && has_grouped_lowering;
}

bool test_backbone_branch_support_stays_local_to_root_pole() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}, {0.0, 24.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok) {
    return false;
  }

  const ObjectId mid_id = find_pole_id_by_position(state, {0.0, 12.0, 0.0});
  if (mid_id == wire::core::kInvalidObjectId) {
    return false;
  }

  std::vector<const wire::core::Port*> downstream_mid_ports{};
  int root_lowered_count = 0;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (span == nullptr || !layout_view.has_value()) {
      return false;
    }
    if (const auto root_endpoint = layout_endpoint_for_owner(*layout_view, center_id); root_endpoint.has_value()) {
      if (!endpoint_has_authoritative_lowering(*root_endpoint) ||
          root_endpoint->relation_kind != wire::core::JunctionRelationKind::kSideBranch ||
          !lowered_support_group_for_owner(*layout_view, center_id).has_value()) {
        return false;
      }
      ++root_lowered_count;
    }
    if (const auto mid_endpoint = layout_endpoint_for_owner(*layout_view, mid_id); mid_endpoint.has_value()) {
      if (endpoint_has_authoritative_lowering(*mid_endpoint) ||
          lowered_support_group_for_owner(*layout_view, mid_id).has_value()) {
        return false;
      }
    }
    const bool touches_mid = span->endpoint_node_a_id == mid_id || span->endpoint_node_b_id == mid_id;
    const bool touches_center = span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id;
    if (touches_mid && !touches_center) {
      const ObjectId port_id = span->endpoint_node_a_id == mid_id ? span->port_a_id : span->port_b_id;
      const auto* port = state.view().edit_state().ports.find(port_id);
      if (port == nullptr) {
        return false;
      }
      downstream_mid_ports.push_back(port);
    }
  }
  if (root_lowered_count != 3 || downstream_mid_ports.size() != 3) {
    return false;
  }

  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto* port : downstream_mid_ports) {
    min_z = std::min(min_z, port->world_position.z);
    max_z = std::max(max_z, port->world_position.z);
    min_x = std::min(min_x, port->world_position.x);
    max_x = std::max(max_x, port->world_position.x);
    min_y = std::min(min_y, port->world_position.y);
    max_y = std::max(max_y, port->world_position.y);
  }
  const bool flat_height = (max_z - min_z) <= 1e-6;
  const bool perpendicular_row = (max_x - min_x) > (max_y - min_y) * 2.0;
  if (!flat_height || !perpendicular_row) {
    std::cerr << "[DBG] C195 zSpan=" << (max_z - min_z) << " xSpan=" << (max_x - min_x)
              << " ySpan=" << (max_y - min_y) << "\n";
  }
  return flat_height && perpendicular_row;
}

bool test_backbone_branch_support_visual_stays_perpendicular_to_branch() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto* center_pole = state.view().edit_state().poles.find(center_id);
  if (center_pole == nullptr) {
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);

  bool found = false;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto placement = lowered_support_group_for_owner(*layout_view, center_id);
    if (!placement.has_value()) {
      continue;
    }
    found = true;
    const wire::core::Vec3d support_axis = normalize_xy_safe(placement->tip_world - placement->mount_world);
    const wire::core::Vec3d expected_axis = normalize_xy_safe(placement->side_axis);
    const double axis_alignment = std::abs(dot_xy(support_axis, expected_axis));
    if (axis_alignment < 0.95) {
      const double support_start_xy =
          std::sqrt(std::pow(placement->mount_world.x - center_pole->world_transform.position.x, 2.0) +
                    std::pow(placement->mount_world.y - center_pole->world_transform.position.y, 2.0));
      const wire::core::Vec3d attachment_world =
          placement->attachment_worlds.empty() ? placement->tip_world : placement->attachment_worlds.front();
      const wire::core::Vec3d hanger_delta = attachment_world - placement->tip_world;
      const double hanger_xy = std::sqrt(hanger_delta.x * hanger_delta.x + hanger_delta.y * hanger_delta.y);
      std::cerr << "[DBG] C196 axisAlignment=" << axis_alignment << " hangerXY=" << hanger_xy
                << " supportStartXY=" << support_start_xy << " mount=(" << placement->mount_world.x << ","
                << placement->mount_world.y << "," << placement->mount_world.z << ") tip=(" << placement->tip_world.x
                << "," << placement->tip_world.y << "," << placement->tip_world.z << ") attach=("
                << attachment_world.x << "," << attachment_world.y << "," << attachment_world.z << ") sideSign="
                << placement->chosen_side_sign << " origin=" << placement->origin
                << " sideRule=" << static_cast<int>(placement->side_assignment_rule)
                << " orientRule=" << static_cast<int>(placement->support_orientation_rule)
                << " hasSideAxis=" << placement->has_side_axis << " sideAxis=(" << placement->side_axis.x << ","
                << placement->side_axis.y << "," << placement->side_axis.z << ")\n";
      return false;
    }
  }
  return found;
}

struct GroupedRankEndpointSnapshot {
  ObjectId span_id = wire::core::kInvalidObjectId;
  bool is_start = false;
  ObjectId owner_pole_id = wire::core::kInvalidObjectId;
  int support_group_id = -1;
  int pair_height_rank = -1;
  bool has_signed_support_axis = false;
  ObjectId pair_peer_low = wire::core::kInvalidObjectId;
  ObjectId pair_peer_high = wire::core::kInvalidObjectId;
  bool has_pair_axis = false;
  wire::core::JunctionRelationKind relation_kind{};
  wire::core::ContinuityCategoryClass continuity_class{};
  bool has_visual_arm_geometry = false;
};

struct GroupedRankFamilySnapshot {
  ObjectId owner_pole_id = wire::core::kInvalidObjectId;
  int support_group_id = -1;
  int pair_height_rank = -1;
  bool has_signed_support_axis = false;
  ObjectId pair_peer_low = wire::core::kInvalidObjectId;
  ObjectId pair_peer_high = wire::core::kInvalidObjectId;
  bool has_pair_axis = false;
  wire::core::JunctionRelationKind relation_kind{};
  wire::core::ContinuityCategoryClass continuity_class{};
  int grouped_port_count = 0;
  std::vector<GroupedRankEndpointSnapshot> endpoints{};
};

bool grouped_rank_endpoint_less(const GroupedRankEndpointSnapshot& a, const GroupedRankEndpointSnapshot& b) {
  if (a.span_id != b.span_id) {
    return a.span_id < b.span_id;
  }
  if (a.is_start != b.is_start) {
    return a.is_start < b.is_start;
  }
  return a.support_group_id < b.support_group_id;
}

bool grouped_rank_family_less(const GroupedRankFamilySnapshot& a, const GroupedRankFamilySnapshot& b) {
  if (a.owner_pole_id != b.owner_pole_id) {
    return a.owner_pole_id < b.owner_pole_id;
  }
  return a.support_group_id < b.support_group_id;
}

bool grouped_rank_endpoint_equal(const GroupedRankEndpointSnapshot& a, const GroupedRankEndpointSnapshot& b) {
  return a.span_id == b.span_id && a.is_start == b.is_start && a.owner_pole_id == b.owner_pole_id &&
         a.support_group_id == b.support_group_id && a.pair_height_rank == b.pair_height_rank &&
         a.has_signed_support_axis == b.has_signed_support_axis && a.pair_peer_low == b.pair_peer_low &&
         a.pair_peer_high == b.pair_peer_high && a.has_pair_axis == b.has_pair_axis &&
         a.relation_kind == b.relation_kind && a.continuity_class == b.continuity_class &&
         a.has_visual_arm_geometry == b.has_visual_arm_geometry;
}

bool grouped_rank_family_equal(const GroupedRankFamilySnapshot& a, const GroupedRankFamilySnapshot& b) {
  if (a.owner_pole_id != b.owner_pole_id || a.support_group_id != b.support_group_id ||
      a.pair_height_rank != b.pair_height_rank || a.has_signed_support_axis != b.has_signed_support_axis ||
      a.pair_peer_low != b.pair_peer_low || a.pair_peer_high != b.pair_peer_high ||
      a.has_pair_axis != b.has_pair_axis || a.relation_kind != b.relation_kind ||
      a.continuity_class != b.continuity_class || a.grouped_port_count != b.grouped_port_count ||
      a.endpoints.size() != b.endpoints.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.endpoints.size(); ++i) {
    if (!grouped_rank_endpoint_equal(a.endpoints[i], b.endpoints[i])) {
      return false;
    }
  }
  return true;
}

bool collect_grouped_rank_family_snapshots(
    const CoreState& state, ObjectId owner_pole_id, std::vector<GroupedRankFamilySnapshot>* out,
    std::optional<wire::core::JunctionRelationKind> relation_filter = std::nullopt) {
  if (out == nullptr) {
    return false;
  }
  out->clear();
  std::unordered_map<std::uint64_t, GroupedRankFamilySnapshot> families{};
  for (const auto& span_entry : state.view().spans().items()) {
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    for (const auto& group : layout_view->lowered_support_groups) {
      if (group.owner_pole_id != owner_pole_id) {
        continue;
      }
      if (relation_filter.has_value() && group.relation_kind != *relation_filter) {
        continue;
      }
      const std::uint64_t key =
          (static_cast<std::uint64_t>(static_cast<std::uint32_t>(group.owner_pole_id)) << 32) ^
          static_cast<std::uint32_t>(group.support_group_id);
      auto& family = families[key];
      family.owner_pole_id = group.owner_pole_id;
      family.support_group_id = group.support_group_id;
      family.pair_height_rank = group.pair_height_rank;
      family.has_signed_support_axis = group.has_signed_support_axis;
      family.pair_peer_low = group.support_authority.pair.pair_peer_low;
      family.pair_peer_high = group.support_authority.pair.pair_peer_high;
      family.has_pair_axis = group.support_authority.pair.has_pair_axis;
      family.relation_kind = group.relation_kind;
      family.continuity_class = group.continuity_class;
      family.grouped_port_count = group.grouped_port_count;
    }

    const auto collect_endpoint = [&](const wire::core::SupportLayoutEndpointView& endpoint, bool is_start) {
      if (endpoint.owner_pole_id != owner_pole_id || !endpoint.lower_required ||
          endpoint.support_group_id < 0) {
        return;
      }
      if (relation_filter.has_value() && endpoint.relation_kind != *relation_filter) {
        return;
      }
      const std::uint64_t key =
          (static_cast<std::uint64_t>(static_cast<std::uint32_t>(endpoint.owner_pole_id)) << 32) ^
          static_cast<std::uint32_t>(endpoint.support_group_id);
      auto it = families.find(key);
      if (it == families.end()) {
        return;
      }
      GroupedRankEndpointSnapshot snapshot{};
      snapshot.span_id = span_entry.id;
      snapshot.is_start = is_start;
      snapshot.owner_pole_id = endpoint.owner_pole_id;
      snapshot.support_group_id = endpoint.support_group_id;
      snapshot.pair_height_rank = endpoint.pair_height_rank;
      snapshot.has_signed_support_axis = endpoint.has_signed_support_axis;
      snapshot.pair_peer_low = endpoint.support_authority.pair.pair_peer_low;
      snapshot.pair_peer_high = endpoint.support_authority.pair.pair_peer_high;
      snapshot.has_pair_axis = endpoint.support_authority.pair.has_pair_axis;
      snapshot.relation_kind = endpoint.relation_kind;
      snapshot.continuity_class = endpoint.continuity_class;
      snapshot.has_visual_arm_geometry = endpoint.has_visual_arm_geometry;
      it->second.endpoints.push_back(snapshot);
    };
    collect_endpoint(layout_view->start_endpoint, true);
    collect_endpoint(layout_view->end_endpoint, false);
  }

  out->reserve(families.size());
  for (auto& [_, family] : families) {
    std::sort(family.endpoints.begin(), family.endpoints.end(), grouped_rank_endpoint_less);
    out->push_back(std::move(family));
  }
  std::sort(out->begin(), out->end(), grouped_rank_family_less);
  return !out->empty();
}

bool grouped_rank_family_has_consistent_owner_and_endpoints(const GroupedRankFamilySnapshot& family) {
  if (family.support_group_id < 0 || family.owner_pole_id == wire::core::kInvalidObjectId || family.endpoints.empty()) {
    return false;
  }
  for (const auto& endpoint : family.endpoints) {
    if (endpoint.owner_pole_id != family.owner_pole_id || endpoint.support_group_id != family.support_group_id ||
        endpoint.relation_kind != family.relation_kind || endpoint.continuity_class != family.continuity_class) {
      return false;
    }
    if (family.pair_height_rank >= 0) {
      if (endpoint.pair_height_rank != family.pair_height_rank) {
        return false;
      }
    } else if (endpoint.pair_height_rank >= 0) {
      return false;
    }
  }
  return true;
}

bool grouped_rank_families_equal(const std::vector<GroupedRankFamilySnapshot>& a,
                                 const std::vector<GroupedRankFamilySnapshot>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!grouped_rank_family_equal(a[i], b[i])) {
      return false;
    }
  }
  return true;
}

bool test_backbone_tilted_branch_support_follows_tilted_pole_centerline() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  auto observe_group = [&](wire::core::Vec3d* mount_world, wire::core::Vec3d* tip_world,
                           wire::core::Vec3d* side_axis, int* support_group_id,
                           std::vector<wire::core::Vec3d>* attachment_worlds) -> bool {
    for (ObjectId span_id : branch_generated.value.generated_span_ids) {
      const auto layout_view = state.view().inspect_support_layout(span_id);
      if (!layout_view.has_value()) {
        continue;
      }
      const auto placement = lowered_support_group_for_owner(*layout_view, center_id);
      if (!placement.has_value()) {
        continue;
      }
      if (mount_world != nullptr) {
        *mount_world = placement->mount_world;
      }
      if (tip_world != nullptr) {
        *tip_world = placement->tip_world;
      }
      if (side_axis != nullptr) {
        *side_axis = normalize_xy_safe(placement->side_axis);
      }
      if (support_group_id != nullptr) {
        *support_group_id = placement->support_group_id;
      }
      if (attachment_worlds != nullptr) {
        *attachment_worlds = placement->attachment_worlds;
      }
      return true;
    }
    return false;
  };

  wire::core::Vec3d before_mount{};
  wire::core::Vec3d before_tip{};
  wire::core::Vec3d side_axis{};
  std::vector<GroupedRankFamilySnapshot> before_rank_families{};
  if (!observe_group(&before_mount, &before_tip, &side_axis, nullptr, nullptr)) {
    return false;
  }
  if (!collect_grouped_rank_family_snapshots(state, center_id, &before_rank_families) || before_rank_families.empty()) {
    std::cerr << "[DBG] C286 missing_rank_families_before center=" << center_id << "\n";
    return false;
  }
  for (const auto& family : before_rank_families) {
    if (!grouped_rank_family_has_consistent_owner_and_endpoints(family)) {
      std::cerr << "[DBG] C286 inconsistent_rank_before group=" << family.support_group_id
                << " rank=" << family.pair_height_rank << " endpoints=" << family.endpoints.size() << "\n";
      return false;
    }
  }

  const auto tilt_result = state.SetPoleTilt(center_id, 20.0);
  if (!tilt_result.ok) {
    return false;
  }
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  wire::core::Vec3d after_mount{};
  wire::core::Vec3d after_tip{};
  int support_group_id = -1;
  std::vector<wire::core::Vec3d> group_attachment_worlds{};
  std::vector<GroupedRankFamilySnapshot> after_rank_families{};
  if (!observe_group(&after_mount, &after_tip, nullptr, &support_group_id, &group_attachment_worlds)) {
    return false;
  }
  if (!collect_grouped_rank_family_snapshots(state, center_id, &after_rank_families) || after_rank_families.empty()) {
    std::cerr << "[DBG] C286 missing_rank_families_after center=" << center_id << "\n";
    return false;
  }
  if (!grouped_rank_families_equal(before_rank_families, after_rank_families)) {
    std::cerr << "[DBG] C286 rank_refresh_mismatch before=" << before_rank_families.size()
              << " after=" << after_rank_families.size() << "\n";
    return false;
  }

  const auto* pole = state.view().edit_state().poles.find(center_id);
  if (pole == nullptr) {
    return false;
  }

  const double layout_yaw_deg = state.effective_port_layout_yaw_deg(*pole, wire::core::ConnectionCategory::kHighVoltage);
  const wire::core::PoleFrame frame =
      wire::core::BuildPoleFrame(pole->world_transform, layout_yaw_deg);
  if (std::abs(frame.up.z) <= 1e-9) {
    return false;
  }
  const double local_z = (after_mount.z - frame.origin.z) / frame.up.z;
  const wire::core::Vec3d centerline_world = wire::core::LocalPointToWorld(frame, {0.0, 0.0, local_z});
  const wire::core::Vec3d center_to_mount = after_mount - centerline_world;
  const double along_axis = dot_xy(center_to_mount, side_axis);
  const double cross_axis = std::abs(center_to_mount.x * side_axis.y - center_to_mount.y * side_axis.x);
  const bool moved_xy = !almost_equal(before_mount.x, after_mount.x, 1e-6) || !almost_equal(before_mount.y, after_mount.y, 1e-6) ||
                        !almost_equal(before_tip.x, after_tip.x, 1e-6) || !almost_equal(before_tip.y, after_tip.y, 1e-6);

  if (!(moved_xy && std::abs(along_axis) > 0.05 && cross_axis < 1e-4)) {
    std::cerr << "[DBG] C286 movedXY=" << (moved_xy ? 1 : 0) << " beforeMount=(" << before_mount.x << ","
              << before_mount.y << "," << before_mount.z << ") afterMount=(" << after_mount.x << "," << after_mount.y
              << "," << after_mount.z << ") centerline=(" << centerline_world.x << "," << centerline_world.y << ","
              << centerline_world.z << ") sideAxis=(" << side_axis.x << "," << side_axis.y << "," << side_axis.z
              << ") alongAxis=" << along_axis << " crossAxis=" << cross_axis << "\n";
    return false;
  }

  std::vector<wire::core::Vec3d> endpoint_attachment_worlds{};
  for (const auto& span_entry : state.view().spans().items()) {
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto collect_endpoint = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
      if (endpoint.owner_pole_id == center_id && endpoint_has_authoritative_lowering(endpoint) &&
          endpoint.support_group_id == support_group_id) {
        endpoint_attachment_worlds.push_back(endpoint.endpoint_world);
      }
    };
    collect_endpoint(layout_view->start_endpoint);
    collect_endpoint(layout_view->end_endpoint);
  }
  if (group_attachment_worlds.size() != endpoint_attachment_worlds.size()) {
    std::cerr << "[DBG] C286 attachmentCount group=" << group_attachment_worlds.size()
              << " endpoint=" << endpoint_attachment_worlds.size() << " supportGroup=" << support_group_id << "\n";
    return false;
  }
  std::vector<bool> matched(endpoint_attachment_worlds.size(), false);
  for (const auto& attachment_world : group_attachment_worlds) {
    bool found_match = false;
    for (size_t i = 0; i < endpoint_attachment_worlds.size(); ++i) {
      if (matched[i]) {
        continue;
      }
      if (almost_equal(attachment_world, endpoint_attachment_worlds[i], 1e-6)) {
        matched[i] = true;
        found_match = true;
        break;
      }
    }
    if (!found_match) {
      std::cerr << "[DBG] C286 staleAttachment group=(" << attachment_world.x << "," << attachment_world.y << ","
                << attachment_world.z << ") supportGroup=" << support_group_id << "\n";
      return false;
    }
  }

  return true;
}

bool test_backbone_default_single_branch_stays_flat_without_branch_support() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.size() != 1) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);
  const auto span_view = state.view().inspect_span(branch_generated.value.generated_span_ids.front());
  const auto layout_view = state.view().inspect_support_layout(branch_generated.value.generated_span_ids.front());
  if (!span_view.has_value() || !layout_view.has_value()) {
    return false;
  }
  const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
  if (!endpoint.has_value()) {
    return false;
  }
  if (!(span_view->flow_kind == wire::core::BackboneFlowKind::kBranch && !endpoint->lower_required &&
        endpoint->support_group_id < 0 && endpoint->branch_down_offset_m == 0.0 &&
        !lowered_support_group_for_owner(*layout_view, center_id).has_value())) {
    std::cerr << "[DBG] C197 flow=" << static_cast<int>(span_view->flow_kind)
              << " lowerRequired=" << endpoint->lower_required
              << " groupId=" << endpoint->support_group_id
              << " downOffset=" << endpoint->branch_down_offset_m << "\n";
  }
  return span_view->flow_kind == wire::core::BackboneFlowKind::kBranch && !endpoint->lower_required &&
         endpoint->support_group_id < 0 && endpoint->branch_down_offset_m == 0.0 &&
         !lowered_support_group_for_owner(*layout_view, center_id).has_value();
}

bool test_backbone_communication_bundle_branch_stays_flat_without_branch_support() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kCommunication, 3);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kCommunication, 3);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.size() != 3) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);
  const auto span_view = state.view().inspect_span(branch_generated.value.generated_span_ids.front());
  const auto layout_view = state.view().inspect_support_layout(branch_generated.value.generated_span_ids.front());
  if (!span_view.has_value() || !layout_view.has_value()) {
    return false;
  }
  const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
  if (!endpoint.has_value()) {
    return false;
  }
  if (!(span_view->flow_kind == wire::core::BackboneFlowKind::kBranch && endpoint->lower_required &&
        endpoint->support_group_id < 0 && endpoint->lowering_blocked_by_policy &&
        endpoint->branch_down_offset_m == 0.0 &&
        !lowered_support_group_for_owner(*layout_view, center_id).has_value())) {
    std::cerr << "[DBG] C198 flow=" << static_cast<int>(span_view->flow_kind)
              << " lowerRequired=" << endpoint->lower_required
              << " groupId=" << endpoint->support_group_id
              << " blocked=" << endpoint->lowering_blocked_by_policy
              << " downOffset=" << endpoint->branch_down_offset_m
              << " generatedSpanCount=" << branch_generated.value.generated_span_ids.size() << "\n";
  }
  return span_view->flow_kind == wire::core::BackboneFlowKind::kBranch && endpoint->lower_required &&
         endpoint->support_group_id < 0 && endpoint->lowering_blocked_by_policy &&
         endpoint->branch_down_offset_m == 0.0 &&
         !lowered_support_group_for_owner(*layout_view, center_id).has_value();
}

bool test_backbone_hv3_branch_support_policy_applies_on_both_default_pole_types() {
  auto run_case = [](const char* pole_type_name) -> bool {
    CoreState state;
    PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
    for (const auto& [id, pole_type] : state.view().pole_types()) {
      if (pole_type.name == pole_type_name) {
        pole_type_id = id;
        break;
      }
    }
    if (pole_type_id == wire::core::kInvalidPoleTypeId) {
      std::cerr << "[DBG] C199 pole_type_missing name=" << pole_type_name << "\n";
      return false;
    }

    wire::core::BackboneSpec trunk{};
    trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
    trunk.interval_m = 1000.0;
    trunk.pole_type_id = pole_type_id;
    add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
    const auto trunk_generated = state.GenerateFromBackboneSpec(trunk);
    if (!trunk_generated.ok) {
      std::cerr << "[DBG] C199 trunk_generate_failed type=" << pole_type_name << "\n";
      return false;
    }

    const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
    if (center_id == wire::core::kInvalidObjectId) {
      return false;
    }
    std::unordered_set<ObjectId> main_port_ids{};
    for (ObjectId span_id : trunk_generated.value.generated_span_ids) {
      const auto* span = state.view().edit_state().spans.find(span_id);
      if (span == nullptr) {
        return false;
      }
      if (span->endpoint_node_a_id == center_id) {
        main_port_ids.insert(span->port_a_id);
      }
      if (span->endpoint_node_b_id == center_id) {
        main_port_ids.insert(span->port_b_id);
      }
    }

    wire::core::BackboneSpec branch{};
    branch.path.polyline = {{0.0, 0.0, 0.0}, {8.0, 12.0, 0.0}};
    wire::core::BackboneInputSpec::NodeSpec shared{};
    shared.point_index = 0;
    shared.support_kind = wire::core::SupportKind::kPole;
    shared.node_id = center_id;
    branch.path.node_specs.push_back(shared);
    branch.interval_m = 1000.0;
    branch.pole_type_id = pole_type_id;
    add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
    const auto branch_generated = state.GenerateFromBackboneSpec(branch);
    if (!branch_generated.ok || branch_generated.value.generated_span_ids.size() != 3) {
      std::cerr << "[DBG] C199 branch_generate_failed type=" << pole_type_name
                << " error=" << branch_generated.error
                << " count=" << branch_generated.value.generated_span_ids.size() << "\n";
      return false;
    }

    const auto* center_pole = state.view().edit_state().poles.find(center_id);
    if (center_pole == nullptr) {
      return false;
    }

    std::vector<const wire::core::Port*> root_ports{};
    double main_min_z = std::numeric_limits<double>::infinity();
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id != center_id || port.layer != wire::core::PortLayer::kHighVoltage ||
          !main_port_ids.contains(port.id)) {
        continue;
      }
      main_min_z = std::min(main_min_z, port.world_position.z);
    }
    int support_group_id = -1;
    double down_offset_m = -1.0;
    for (ObjectId span_id : branch_generated.value.generated_span_ids) {
      const auto* span = state.view().edit_state().spans.find(span_id);
      const auto layout_view = state.view().inspect_support_layout(span_id);
      if (span == nullptr || !layout_view.has_value()) {
        return false;
      }
      const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
      const auto group = lowered_support_group_for_owner(*layout_view, center_id);
      if (!endpoint.has_value() || !group.has_value() || !endpoint_has_authoritative_lowering(*endpoint) ||
          endpoint->relation_kind != wire::core::JunctionRelationKind::kSideBranch) {
        std::cerr << "[DBG] C199 branch_policy_missing type=" << pole_type_name << "\n";
        return false;
      }
      if (support_group_id < 0) {
        support_group_id = endpoint->support_group_id;
        down_offset_m = group->down_offset_m;
      } else if (support_group_id != endpoint->support_group_id ||
                 !almost_equal(down_offset_m, group->down_offset_m, 1e-9)) {
        return false;
      }
      const ObjectId port_id = span->endpoint_node_a_id == center_id ? span->port_a_id : span->port_b_id;
      const auto* port = state.view().edit_state().ports.find(port_id);
      if (port == nullptr) {
        return false;
      }
      root_ports.push_back(port);
    }
    if (root_ports.size() != 3 || !std::isfinite(main_min_z) || support_group_id < 0 || down_offset_m <= 1e-6) {
      std::cerr << "[DBG] C199 root_port_collection_failed type=" << pole_type_name
                << " branchPorts=" << root_ports.size() << " mainMinZ=" << main_min_z << "\n";
      return false;
    }

    const double min_z = std::min({root_ports[0]->world_position.z, root_ports[1]->world_position.z,
                                   root_ports[2]->world_position.z});
    const double max_z = std::max({root_ports[0]->world_position.z, root_ports[1]->world_position.z,
                                   root_ports[2]->world_position.z});
    if ((max_z - min_z) > 1e-6 || !(max_z + 1e-6 < main_min_z)) {
      std::cerr << "[DBG] C199 root_port_z_failed type=" << pole_type_name << " minZ=" << min_z
                << " maxZ=" << max_z << " mainMinZ=" << main_min_z << "\n";
      return false;
    }

    wire::core::CommitOptions options{};
    options.run_recalc = true;
    (void)state.Commit(options);

    bool found_support = false;
    for (ObjectId span_id : branch_generated.value.generated_span_ids) {
      const auto layout_view = state.view().inspect_support_layout(span_id);
      if (!layout_view.has_value()) {
        continue;
      }
      const auto placement = lowered_support_group_for_owner(*layout_view, center_id);
      if (!placement.has_value()) {
        continue;
      }
      found_support = true;
      const wire::core::Vec3d support_axis = normalize_xy_safe(placement->tip_world - placement->mount_world);
      const wire::core::Vec3d expected_axis = normalize_xy_safe(placement->side_axis);
      const double axis_alignment = std::abs(dot_xy(support_axis, expected_axis));
      if (axis_alignment < 0.97) {
        std::cerr << "[DBG] C199 support_axis_failed type=" << pole_type_name
                  << " axisAlignment=" << axis_alignment << " mount=(" << placement->mount_world.x << ","
                  << placement->mount_world.y << "," << placement->mount_world.z << ") tip=(" << placement->tip_world.x
                  << "," << placement->tip_world.y << "," << placement->tip_world.z << ") sideSign="
                  << placement->chosen_side_sign << " origin=" << placement->origin
                  << " sideRule=" << static_cast<int>(placement->side_assignment_rule)
                  << " orientRule=" << static_cast<int>(placement->support_orientation_rule)
                  << " hasSideAxis=" << placement->has_side_axis << " sideAxis=(" << placement->side_axis.x << ","
                  << placement->side_axis.y << "," << placement->side_axis.z << ")\n";
        return false;
      }
    }
    return found_support;
  };

  return run_case("DistributionPole") && run_case("CommunicationPole");
}

bool test_backbone_hv3_acute_corner_lowers_corner_bundle_without_branch_support() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {-10.0, 2.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() != 6) {
    std::cerr << "[DBG] C204 generate_failed error=" << generated.error
              << " spanCount=" << generated.value.generated_span_ids.size() << "\n";
    return false;
  }

  const ObjectId prev_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId next_id = find_pole_id_by_position(state, {-10.0, 2.0, 0.0});
  if (prev_id == wire::core::kInvalidObjectId || center_id == wire::core::kInvalidObjectId ||
      next_id == wire::core::kInvalidObjectId) {
    return false;
  }

  std::unordered_set<ObjectId> generated_port_ids{};
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      continue;
    }
    generated_port_ids.insert(span->port_a_id);
    generated_port_ids.insert(span->port_b_id);
  }

  auto collect_hv_ports = [&](ObjectId pole_id) {
    std::vector<const wire::core::Port*> ports{};
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id == pole_id && port.layer == wire::core::PortLayer::kHighVoltage &&
          generated_port_ids.contains(port.id)) {
        ports.push_back(&port);
      }
    }
    return ports;
  };
  const auto prev_ports = collect_hv_ports(prev_id);
  const auto center_ports = collect_hv_ports(center_id);
  const auto next_ports = collect_hv_ports(next_id);
  if (prev_ports.size() != 3 || center_ports.size() != 3 || next_ports.size() != 3) {
    std::cerr << "[DBG] C204 port_count_failed prev=" << prev_ports.size() << " center=" << center_ports.size()
              << " next=" << next_ports.size() << "\n";
    return false;
  }

  auto min_z = [](const std::vector<const wire::core::Port*>& ports) {
    double z = std::numeric_limits<double>::infinity();
    for (const auto* port : ports) {
      z = std::min(z, port->world_position.z);
    }
    return z;
  };
  auto max_z = [](const std::vector<const wire::core::Port*>& ports) {
    double z = -std::numeric_limits<double>::infinity();
    for (const auto* port : ports) {
      z = std::max(z, port->world_position.z);
    }
    return z;
  };

  const double prev_min_z = min_z(prev_ports);
  const double center_min_z = min_z(center_ports);
  const double center_max_z = max_z(center_ports);
  const double next_min_z = min_z(next_ports);
  const bool center_uniform_height = (center_max_z - center_min_z) <= 1e-6;
  const bool center_lowered = center_max_z + 1e-6 < std::min(prev_min_z, next_min_z);
  const bool center_not_branch_support = std::none_of(center_ports.begin(), center_ports.end(), [](const wire::core::Port* port) {
    return port != nullptr && port->placement_source == wire::core::PortPlacementSourceKind::kBranchSupport;
  });

  const auto& assignments = state.view().last_lane_assignments();
  const double expected_down_offset = 0.275;
  int acute_touch_count = 0;
  bool assignment_offset_ok = true;
  for (const auto& assignment : assignments) {
    if (assignment.pole_a_id != center_id && assignment.pole_b_id != center_id) {
      continue;
    }
    if (assignment.uses_branch_support) {
      std::cerr << "[DBG] C204 unexpected_branch_support segment=" << assignment.segment_index << "\n";
      return false;
    }
    if (std::abs(assignment.branch_down_offset_m - expected_down_offset) > 1e-9) {
      std::cerr << "[DBG] C204 wrong_down_offset segment=" << assignment.segment_index
                << " down=" << assignment.branch_down_offset_m
                << " turn=" << assignment.turn_angle_deg << "\n";
      assignment_offset_ok = false;
    }
    ++acute_touch_count;
  }

  if (!(center_uniform_height && center_lowered && center_not_branch_support && acute_touch_count == 2 &&
        assignment_offset_ok)) {
    std::cerr << "[DBG] C204 centerMinZ=" << center_min_z << " centerMaxZ=" << center_max_z
              << " prevMinZ=" << prev_min_z << " nextMinZ=" << next_min_z
              << " uniform=" << (center_uniform_height ? 1 : 0)
              << " lowered=" << (center_lowered ? 1 : 0)
              << " nonBranch=" << (center_not_branch_support ? 1 : 0)
              << " acuteSegments=" << acute_touch_count
              << " offsetOk=" << (assignment_offset_ok ? 1 : 0) << "\n";
  }
  return center_uniform_height && center_lowered && center_not_branch_support && acute_touch_count == 2 &&
         assignment_offset_ok;
}

bool test_backbone_hv3_acute_corner_lowering_survives_pole_refresh() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {-10.0, 2.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId prev_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId next_id = find_pole_id_by_position(state, {-10.0, 2.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId || prev_id == wire::core::kInvalidObjectId ||
      next_id == wire::core::kInvalidObjectId) {
    return false;
  }

  std::unordered_set<ObjectId> generated_port_ids{};
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      continue;
    }
    generated_port_ids.insert(span->port_a_id);
    generated_port_ids.insert(span->port_b_id);
  }

  auto min_generated_hv_z = [&](ObjectId pole_id) {
    double z = std::numeric_limits<double>::infinity();
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id == pole_id && port.layer == wire::core::PortLayer::kHighVoltage &&
          generated_port_ids.contains(port.id)) {
        z = std::min(z, port.world_position.z);
      }
    }
    return z;
  };

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr &&
        ((span->endpoint_node_a_id == center_id && span->endpoint_node_b_id == next_id) ||
         (span->endpoint_node_a_id == next_id && span->endpoint_node_b_id == center_id))) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto before_layout = state.view().inspect_support_layout(target_span_id);
  const auto before_endpoint = before_layout.has_value() ? layout_endpoint_for_owner(*before_layout, center_id) : std::nullopt;
  const auto before_group = before_layout.has_value() ? lowered_support_group_for_owner(*before_layout, center_id) : std::nullopt;
  if (!before_endpoint.has_value() || !before_group.has_value()) {
    return false;
  }

  const double before_center_z = min_generated_hv_z(center_id);
  const double before_neighbor_z = std::min(min_generated_hv_z(prev_id), min_generated_hv_z(next_id));
  if (!(std::isfinite(before_center_z) && std::isfinite(before_neighbor_z) && before_center_z + 1e-6 < before_neighbor_z)) {
    std::cerr << "[DBG] C205 before centerZ=" << before_center_z << " neighborZ=" << before_neighbor_z << "\n";
    return false;
  }

  const auto yaw_result = state.SetPoleManualYawOverride(center_id, 15.0);
  if (!yaw_result.ok) {
    std::cerr << "[DBG] C205 yaw_override_failed error=" << yaw_result.error << "\n";
    return false;
  }

  const double after_center_z = min_generated_hv_z(center_id);
  const double after_neighbor_z = std::min(min_generated_hv_z(prev_id), min_generated_hv_z(next_id));
  const auto after_layout = state.view().inspect_support_layout(target_span_id);
  const auto after_endpoint = after_layout.has_value() ? layout_endpoint_for_owner(*after_layout, center_id) : std::nullopt;
  const auto after_group = after_layout.has_value() ? lowered_support_group_for_owner(*after_layout, center_id) : std::nullopt;

  const bool ok = std::isfinite(after_center_z) && std::isfinite(after_neighbor_z) &&
                  after_center_z + 1e-6 < after_neighbor_z && after_endpoint.has_value() && after_group.has_value() &&
                  endpoint_has_authoritative_lowering(*after_endpoint) &&
                  before_endpoint->support_group_id == after_endpoint->support_group_id &&
                  almost_equal(before_group->mount_world.z, after_group->mount_world.z, 1e-6) &&
                  almost_equal(before_group->tip_world.z, after_group->tip_world.z, 1e-6);
  if (!ok) {
    std::cerr << "[DBG] C205 after centerZ=" << after_center_z << " neighborZ=" << after_neighbor_z
              << " beforeGroup=" << before_endpoint->support_group_id
              << " afterGroup=" << (after_endpoint.has_value() ? after_endpoint->support_group_id : -1) << "\n";
  }
  return ok;
}

// Intent: Latest captured CommunicationPole HV path should keep one row-order sign per shared pole across incident spans.

bool test_backbone_hv3_moderate_acute_corner_lowers_bundle_at_default_threshold() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {-8.0, 6.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() != 6) {
    std::cerr << "[DBG] C209 generate_failed error=" << generated.error
              << " spanCount=" << generated.value.generated_span_ids.size() << "\n";
    return false;
  }

  const ObjectId prev_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId next_id = find_pole_id_by_position(state, {-8.0, 6.0, 0.0});
  if (prev_id == wire::core::kInvalidObjectId || center_id == wire::core::kInvalidObjectId ||
      next_id == wire::core::kInvalidObjectId) {
    return false;
  }

  std::unordered_set<ObjectId> generated_port_ids{};
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      continue;
    }
    generated_port_ids.insert(span->port_a_id);
    generated_port_ids.insert(span->port_b_id);
  }

  auto collect_hv_ports = [&](ObjectId pole_id) {
    std::vector<const wire::core::Port*> ports{};
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id == pole_id && port.layer == wire::core::PortLayer::kHighVoltage &&
          generated_port_ids.contains(port.id)) {
        ports.push_back(&port);
      }
    }
    return ports;
  };
  const auto prev_ports = collect_hv_ports(prev_id);
  const auto center_ports = collect_hv_ports(center_id);
  const auto next_ports = collect_hv_ports(next_id);
  if (prev_ports.size() != 3 || center_ports.size() != 3 || next_ports.size() != 3) {
    return false;
  }

  auto min_z = [](const std::vector<const wire::core::Port*>& ports) {
    double z = std::numeric_limits<double>::infinity();
    for (const auto* port : ports) {
      z = std::min(z, port->world_position.z);
    }
    return z;
  };
  auto max_z = [](const std::vector<const wire::core::Port*>& ports) {
    double z = -std::numeric_limits<double>::infinity();
    for (const auto* port : ports) {
      z = std::max(z, port->world_position.z);
    }
    return z;
  };

  const double prev_min_z = min_z(prev_ports);
  const double center_min_z = min_z(center_ports);
  const double center_max_z = max_z(center_ports);
  const double next_min_z = min_z(next_ports);
  const bool center_uniform_height = (center_max_z - center_min_z) <= 1e-6;
  const bool center_lowered = center_max_z + 1e-6 < std::min(prev_min_z, next_min_z);

  int acute_touch_count = 0;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if (assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) {
      ++acute_touch_count;
    }
  }

  if (!(center_uniform_height && center_lowered && acute_touch_count == 2)) {
    std::cerr << "[DBG] C209 centerMinZ=" << center_min_z << " centerMaxZ=" << center_max_z
              << " prevMinZ=" << prev_min_z << " nextMinZ=" << next_min_z
              << " acuteSegments=" << acute_touch_count << "\n";
  }
  return center_uniform_height && center_lowered && acute_touch_count == 2;
}

bool test_backbone_junction_prefers_straighter_pair_over_first_drawn_primary() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec bent_main{};
  bent_main.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  bent_main.interval_m = 1000.0;
  bent_main.pole_type_id = type_ids.front();
  add_backbone_bundle(bent_main, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(bent_main).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec west{};
  west.path.polyline = {{0.0, 0.0, 0.0}, {-12.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  west.path.node_specs.push_back(shared);
  west.interval_m = 1000.0;
  west.pole_type_id = type_ids.front();
  add_backbone_bundle(west, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(west).ok) {
    return false;
  }

  const auto pole_view = state.view().inspect_pole(center_id);
  if (!pole_view.has_value()) {
    return false;
  }
  const bool horizontal =
      std::min(angle_diff_abs_deg(pole_view->final_yaw_deg, 0.0), angle_diff_abs_deg(pole_view->final_yaw_deg, 180.0)) <=
      1e-6;
  const bool pair_rule = pole_view->forward_rule == wire::core::PoleForwardRule::kMainChainBisector &&
                         pole_view->support_axis_rule == wire::core::PoleSupportAxisRule::kMainChainPair;
  if (!(horizontal && pair_rule)) {
    std::cerr << "[DBG] C194 yaw=" << pole_view->final_yaw_deg
              << " rule=" << static_cast<int>(pole_view->forward_rule)
              << " supportRule=" << static_cast<int>(pole_view->support_axis_rule) << "\n";
  }
  return horizontal && pair_rule;
}

bool test_backbone_cross_junction_nonmain_line_uses_underpass() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    std::cerr << "[DBG] C201 no pole types\n";
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kLowVoltage);
  const auto main_generated = state.GenerateFromBackboneSpec(main_req);
  if (!main_generated.ok) {
    std::cerr << "[DBG] C201 main generate failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C201 center pole missing after main\n";
    return false;
  }

  double main_center_z = -1.0;
  for (ObjectId span_id : main_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      std::cerr << "[DBG] C201 main span missing\n";
      return false;
    }
    const ObjectId center_port_id = (span->endpoint_node_a_id == center_id) ? span->port_a_id
                                   : (span->endpoint_node_b_id == center_id) ? span->port_b_id
                                                                             : wire::core::kInvalidObjectId;
    if (center_port_id == wire::core::kInvalidObjectId) {
      continue;
    }
    const auto* center_port = state.view().edit_state().ports.find(center_port_id);
    if (center_port == nullptr) {
      std::cerr << "[DBG] C201 main center port missing\n";
      return false;
    }
    main_center_z = std::max(main_center_z, center_port->world_position.z);
  }
  if (main_center_z <= 0.0) {
    std::cerr << "[DBG] C201 invalid main_center_z=" << main_center_z << "\n";
    return false;
  }

  wire::core::BackboneSpec cross_req{};
  cross_req.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross_req.path.node_specs.push_back(shared);
  cross_req.interval_m = 1000.0;
  cross_req.pole_type_id = type_ids.front();
  add_backbone_bundle(cross_req, wire::core::BundleKind::kLowVoltage);
  const auto cross_generated = state.GenerateFromBackboneSpec(cross_req);
  if (!cross_generated.ok || cross_generated.value.generated_span_ids.size() != 2) {
    std::cerr << "[DBG] C201 cross generate failed ok=" << (cross_generated.ok ? 1 : 0)
              << " spanCount=" << cross_generated.value.generated_span_ids.size() << "\n";
    return false;
  }

  const auto pole_view = state.view().inspect_pole(center_id);
  if (!pole_view.has_value()) {
    std::cerr << "[DBG] C201 pole inspect missing\n";
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    std::cerr << "[DBG] C201 junction inspect missing\n";
    return false;
  }

  bool has_point_like_branch = false;
  bool has_same_level_cross_relation = false;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const bool touches_center = assignment.pole_a_id == center_id || assignment.pole_b_id == center_id;
    if (!touches_center) {
      continue;
    }
    has_point_like_branch =
        has_point_like_branch ||
        (assignment.flow_kind == wire::core::BackboneFlowKind::kBranch &&
         assignment.flow_decision_rule == wire::core::BackboneFlowDecisionRule::kJunctionOrderBranch &&
         assignment.continuity_class == wire::core::ContinuityCategoryClass::kPointLike &&
         assignment.same_level_feasible &&
         assignment.lowering_kind == wire::core::BackboneLoweringKind::kNone);
  }

  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCrossUnderpass &&
        relation.continuity_class == wire::core::ContinuityCategoryClass::kPointLike &&
        !relation.default_lower_required && relation.same_level_feasible) {
      has_same_level_cross_relation = true;
      break;
    }
  }
  const bool ok = has_point_like_branch && has_same_level_cross_relation && std::isfinite(main_center_z);
  if (!ok) {
    std::cerr << "[DBG] C201 pointLikeBranch=" << (has_point_like_branch ? 1 : 0)
              << " sameLevelCrossRelation=" << (has_same_level_cross_relation ? 1 : 0)
              << " mainCenterZ=" << main_center_z
              << " centerYaw=" << pole_view->final_yaw_deg << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C201 relation neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " class=" << static_cast<int>(relation.continuity_class)
                << " defaultLower=" << (relation.default_lower_required ? 1 : 0)
                << " sameLevel=" << (relation.same_level_feasible ? 1 : 0)
                << " reason=" << static_cast<int>(relation.infeasible_reason) << "\n";
    }
    for (const auto& assignment : state.view().last_lane_assignments()) {
      std::cerr << "[DBG] C201 assignment poles=" << assignment.pole_a_id << "->" << assignment.pole_b_id
                << " flow=" << static_cast<int>(assignment.flow_kind)
                << " rule=" << static_cast<int>(assignment.flow_decision_rule)
                << " class=" << static_cast<int>(assignment.continuity_class)
                << " defaultLower=" << (assignment.default_lower_required ? 1 : 0)
                << " usesBranchSupport=" << (assignment.uses_branch_support ? 1 : 0)
                << " solver=" << (assignment.solver_used_same_level_constraint ? 1 : 0)
                << " special=" << (assignment.used_special_case_ports ? 1 : 0)
                << " lowering=" << static_cast<int>(assignment.lowering_kind)
                << " down=" << assignment.branch_down_offset_m << "\n";
    }
  }
  return ok;
}

bool test_backbone_all_templates_branch_keeps_hv_down_offset_on_communication_pole() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C210 missing CommunicationPole\n";
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = pole_type_id;
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
  add_backbone_bundle(trunk, wire::core::BundleKind::kOptical);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    std::cerr << "[DBG] C210 trunk generate failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C210 center missing\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {8.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = pole_type_id;
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(branch, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    std::cerr << "[DBG] C210 branch generate failed error=" << generated.error << "\n";
    return false;
  }

  bool hv_branch_found = false;
  bool hv_branch_lowered = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr ||
        (span->endpoint_node_a_id != center_id && span->endpoint_node_b_id != center_id)) {
      continue;
    }
    const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    hv_branch_found = true;
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    const auto group = lowered_support_group_for_owner(*layout_view, center_id);
    if (endpoint.has_value() && group.has_value() && endpoint_has_authoritative_lowering(*endpoint) &&
        endpoint->relation_kind == wire::core::JunctionRelationKind::kSideBranch &&
        endpoint->branch_down_offset_m > 1e-6) {
      hv_branch_lowered = true;
      break;
    }
  }
  if (!(hv_branch_found && hv_branch_lowered)) {
    std::cerr << "[DBG] C210 hvBranchFound=" << (hv_branch_found ? 1 : 0)
              << " hvBranchLowered=" << (hv_branch_lowered ? 1 : 0) << "\n";
    for (const auto& assignment : state.view().last_lane_assignments()) {
      const auto* bundle = state.view().edit_state().bundles.find(assignment.bundle_id);
      std::cerr << "[DBG] C210 assignment bundle="
                << (bundle != nullptr ? static_cast<int>(bundle->bundle_template_id) : -1)
                << " flow=" << static_cast<int>(assignment.flow_kind)
                << " lowerA=" << assignment.decision_a.lower_required
                << " lowerB=" << assignment.decision_b.lower_required
                << " down=" << assignment.branch_down_offset_m << "\n";
    }
  }
  return hv_branch_found && hv_branch_lowered;
}

bool test_backbone_all_templates_cross_keeps_underpass_on_communication_pole() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C211 missing CommunicationPole\n";
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = pole_type_id;
  add_backbone_bundle(main_req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(main_req, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(main_req, wire::core::BundleKind::kOptical);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    std::cerr << "[DBG] C211 main generate failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C211 center missing\n";
    return false;
  }

  wire::core::BackboneSpec cross_req{};
  cross_req.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross_req.path.node_specs.push_back(shared);
  cross_req.interval_m = 1000.0;
  cross_req.pole_type_id = pole_type_id;
  add_backbone_bundle(cross_req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(cross_req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(cross_req, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(cross_req, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(cross_req);
  if (!generated.ok) {
    std::cerr << "[DBG] C211 cross generate failed error=" << generated.error << "\n";
    return false;
  }

  int lowered_assignments = 0;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const bool touches_center = assignment.pole_a_id == center_id || assignment.pole_b_id == center_id;
    if (!touches_center) {
      continue;
    }
    if (assignment.flow_kind == wire::core::BackboneFlowKind::kBranch && assignment.branch_down_offset_m > 1e-6) {
      ++lowered_assignments;
    }
  }
  if (lowered_assignments == 0) {
    std::cerr << "[DBG] C211 loweredAssignments=0\n";
    for (const auto& assignment : state.view().last_lane_assignments()) {
      const auto* bundle = state.view().edit_state().bundles.find(assignment.bundle_id);
      std::cerr << "[DBG] C211 assignment bundle="
                << (bundle != nullptr ? static_cast<int>(bundle->bundle_template_id) : -1)
                << " poles=" << assignment.pole_a_id << "->" << assignment.pole_b_id
                << " flow=" << static_cast<int>(assignment.flow_kind)
                << " uses=" << (assignment.uses_branch_support ? 1 : 0)
                << " down=" << assignment.branch_down_offset_m << "\n";
    }
  }
  return lowered_assignments > 0;
}

bool test_backbone_capture_branch_then_acute_lowering_on_communication_pole() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C212 missing CommunicationPole\n";
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {
      {-18.59678, 11.0534, 0.0},
      {-6.59678, 11.0534, 0.0},
      {5.40322, 11.0534, 0.0},
  };
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = pole_type_id;
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(trunk, wire::core::BundleKind::kOptical);
  const auto trunk_generated = state.GenerateFromBackboneSpec(trunk);
  if (!trunk_generated.ok) {
    std::cerr << "[DBG] C212 trunk generate failed error=" << trunk_generated.error << "\n";
    return false;
  }

  const ObjectId node0 = find_pole_id_by_position(state, {-6.59678, 11.0534, 0.0}, 1e-4);
  auto min_hv_z_at_pole = [&](ObjectId pole_id) {
    double best = std::numeric_limits<double>::infinity();
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id == pole_id && port.layer == wire::core::PortLayer::kHighVoltage) {
        best = std::min(best, port.world_position.z);
      }
    }
    return best;
  };
  const double trunk_root_hv_z = min_hv_z_at_pole(node0);
  if (node0 == wire::core::kInvalidObjectId || !std::isfinite(trunk_root_hv_z)) {
    std::cerr << "[DBG] C212 shared root lookup failed\n";
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {-6.59678, 11.0534, 0.0},
      {-4.93216, 6.7054, 0.0},
      {-13.6709, -1.24875, 0.0},
      {-8.49996, -0.441201, 0.0},
  };
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = node0;
  req.path.node_specs.push_back(shared);
  req.interval_m = 8.0;
  req.pole_type_id = pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(req, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    std::cerr << "[DBG] C212 branch generate failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId node1 = find_pole_id_by_position(state, {-4.93216, 6.7054, 0.0}, 1e-4);
  const ObjectId node2 = find_pole_id_by_position(state, {-13.6709, -1.24875, 0.0}, 1e-4);
  const ObjectId node3 = find_pole_id_by_position(state, {-8.49996, -0.441201, 0.0}, 1e-4);
  if (node0 == wire::core::kInvalidObjectId || node1 == wire::core::kInvalidObjectId ||
      node2 == wire::core::kInvalidObjectId || node3 == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C212 node lookup failed\n";
    return false;
  }

  std::vector<const wire::core::SegmentLaneAssignment*> hv_assignments{};
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const auto* bundle = state.view().edit_state().bundles.find(assignment.bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == wire::core::BundleKind::kHighVoltage) {
      hv_assignments.push_back(&assignment);
    }
  }
  if (hv_assignments.size() < 3) {
    std::cerr << "[DBG] C212 hvAssignments=" << hv_assignments.size() << "\n";
    return false;
  }

  const wire::core::SegmentLaneAssignment* first_assignment = hv_assignments.front();
  const wire::core::EndpointContinuityDecision* first_root_decision = assignment_decision_for_pole(*first_assignment, node0);
  const bool first_is_branch_support =
      first_assignment->segment_index == 0 &&
      first_assignment->pole_a_id == node0 && first_assignment->pole_b_id == node1 &&
      first_root_decision != nullptr && first_root_decision->relation_kind == wire::core::JunctionRelationKind::kSideBranch &&
      first_root_decision->lower_required && first_root_decision->support_group_id >= 0 &&
      first_assignment->branch_down_offset_m > 1e-6;
  int acute_after_root_count = 0;
  bool acute_touches_explicit_corner = false;
  for (const auto* assignment : hv_assignments) {
    if (assignment->segment_index == 0) {
      continue;
    }
    const bool has_corner_lower =
        (assignment->decision_a.relation_kind == wire::core::JunctionRelationKind::kCornerContinuation &&
         assignment->decision_a.lower_required && assignment->decision_a.support_group_id >= 0) ||
        (assignment->decision_b.relation_kind == wire::core::JunctionRelationKind::kCornerContinuation &&
         assignment->decision_b.lower_required && assignment->decision_b.support_group_id >= 0);
    if (!has_corner_lower || assignment->branch_down_offset_m <= 1e-6) {
      continue;
    }
    ++acute_after_root_count;
    if (assignment->pole_a_id == node2 || assignment->pole_b_id == node2) {
      acute_touches_explicit_corner = true;
    }
  }

  auto min_generated_hv_z = [&](ObjectId pole_id) {
    double best = std::numeric_limits<double>::infinity();
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id != pole_id || port.layer != wire::core::PortLayer::kHighVoltage) {
        continue;
      }
      best = std::min(best, port.world_position.z);
    }
    return best;
  };

  double root_branch_hv_z = std::numeric_limits<double>::infinity();
  if (first_is_branch_support) {
    const auto& root_port_ids = (first_assignment->pole_a_id == node0) ? first_assignment->port_ids_a
                                                                        : first_assignment->port_ids_b;
    for (ObjectId port_id : root_port_ids) {
      const wire::core::Port* port = state.view().edit_state().ports.find(port_id);
      if (port != nullptr) {
        root_branch_hv_z = std::min(root_branch_hv_z, port->world_position.z);
      }
    }
  }
  const double node1_z = min_generated_hv_z(node1);
  const double node2_z = min_generated_hv_z(node2);
  const double node3_z = min_generated_hv_z(node3);
  const bool root_lowered =
      std::isfinite(root_branch_hv_z) && root_branch_hv_z + 1e-6 < trunk_root_hv_z;
  const bool acute_center_lowered =
      std::isfinite(node1_z) && std::isfinite(node2_z) && std::isfinite(node3_z) && node2_z + 1e-6 < node1_z &&
      node2_z + 1e-6 < node3_z;

  if (!(first_is_branch_support && acute_after_root_count >= 1 && acute_touches_explicit_corner && root_lowered &&
        acute_center_lowered)) {
    std::cerr << "[DBG] C212 firstBranch=" << (first_is_branch_support ? 1 : 0)
              << " acuteAfterRoot=" << acute_after_root_count
              << " acuteTouchesCorner=" << (acute_touches_explicit_corner ? 1 : 0)
              << " rootLowered=" << (root_lowered ? 1 : 0)
              << " trunkRootZ=" << trunk_root_hv_z << " branchRootZ=" << root_branch_hv_z
              << " node1Z=" << node1_z << " node2Z=" << node2_z << " node3Z=" << node3_z << "\n";
    for (const auto* assignment : hv_assignments) {
      std::cerr << "[DBG] C212 hv assignment poles=" << assignment->pole_a_id << "->" << assignment->pole_b_id
                << " flow=" << static_cast<int>(assignment->flow_kind)
                << " relA=" << static_cast<int>(assignment->decision_a.relation_kind)
                << " relB=" << static_cast<int>(assignment->decision_b.relation_kind)
                << " lowerA=" << assignment->decision_a.lower_required
                << " lowerB=" << assignment->decision_b.lower_required
                << " down=" << assignment->branch_down_offset_m << "\n";
    }
  }

  return first_is_branch_support && acute_after_root_count >= 1 && acute_touches_explicit_corner && root_lowered &&
         acute_center_lowered;
}

bool test_backbone_capture_generation_uses_requested_communication_pole_type() {
  CoreState state;
  std::optional<wire::core::PoleTypeDefinition> communication_pole_type{};
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type = pole_type;
      break;
    }
  }
  if (!communication_pole_type.has_value()) {
    std::cerr << "[DBG] C298 missing CommunicationPole\n";
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {15.4321, 2.78857, 4.76837e-07},
      {13.1155, 11.6235, 0.0},
      {0.657872, 14.3442, 0.0},
      {3.84698, 2.15965, 0.0},
      {5.62203, -4.20999, 0.0},
      {9.24589, -4.00139, 0.0},
      {12.7437, -2.26618, 0.0},
      {12.333, -0.168475, 0.0},
      {9.16973, 0.0271034, 0.0},
      {7.77719, 4.46854, 0.0},
      {11.9824, 3.43273, 0.0},
  };
  req.interval_m = 63.919;
  req.pole_type_id = communication_pole_type->id;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
  add_backbone_bundle(req, wire::core::BundleKind::kOptical);

  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    std::cerr << "[DBG] C298 generate failed error=" << generated.error << "\n";
    return false;
  }
  if (generated.value.generated_pole_ids.empty()) {
    std::cerr << "[DBG] C298 generated no poles\n";
    return false;
  }

  for (const ObjectId pole_id : generated.value.generated_pole_ids) {
    const wire::core::Pole* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr) {
      std::cerr << "[DBG] C298 missing generated pole id=" << pole_id << "\n";
      return false;
    }
    if (pole->pole_type_id != communication_pole_type->id ||
        std::abs(pole->height_m - communication_pole_type->default_height_m) > 1e-9) {
      std::cerr << "[DBG] C298 pole mismatch id=" << pole_id
                << " type=" << pole->pole_type_id
                << " expected_type=" << communication_pole_type->id
                << " height=" << pole->height_m
                << " expected_height=" << communication_pole_type->default_height_m << "\n";
      return false;
    }
  }

  return true;
}

bool test_backbone_hv_generation_uses_hv_category_height_on_communication_pole() {
  CoreState state;
  std::optional<wire::core::PoleTypeDefinition> communication_pole_type{};
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type = pole_type;
      break;
    }
  }
  if (!communication_pole_type.has_value()) {
    std::cerr << "[DBG] C299 missing CommunicationPole\n";
    return false;
  }

  constexpr int hv_layer = 2;
  double hv_base_z = -std::numeric_limits<double>::infinity();
  double non_hv_same_layer_base_z = -std::numeric_limits<double>::infinity();
  for (const auto& band : communication_pole_type->port_bands) {
    if (!band.enabled) {
      continue;
    }
    if (band.category == wire::core::ConnectionCategory::kHighVoltage) {
      hv_base_z = std::max(hv_base_z, band.height_max_m);
    } else if (band.layer == hv_layer) {
      non_hv_same_layer_base_z = std::max(non_hv_same_layer_base_z, band.height_max_m);
    }
  }
  if (!(std::isfinite(hv_base_z) && std::isfinite(non_hv_same_layer_base_z) && hv_base_z + 1e-9 < non_hv_same_layer_base_z)) {
    std::cerr << "[DBG] C299 invalid base z hv=" << hv_base_z << " other=" << non_hv_same_layer_base_z << "\n";
    return false;
  }

  wire::core::BackboneSpec request{};
  request.path.polyline = {{-12.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  request.interval_m = 1000.0;
  request.pole_type_id = communication_pole_type->id;
  add_backbone_bundle(request, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(request, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(request, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
  add_backbone_bundle(request, wire::core::BundleKind::kOptical);

  const auto generated = state.GenerateFromBackboneSpec(request);
  if (!generated.ok) {
    std::cerr << "[DBG] C299 generate failed error=" << generated.error << "\n";
    return false;
  }

  bool saw_hv_port = false;
  double max_generated_hv_z = -std::numeric_limits<double>::infinity();
  for (const auto& port : state.view().edit_state().ports.items()) {
    if (port.layer != wire::core::PortLayer::kHighVoltage ||
        port.category != wire::core::ConnectionCategory::kHighVoltage ||
        port.owner_pole_id == wire::core::kInvalidObjectId ||
        !contains_id(generated.value.generated_pole_ids, port.owner_pole_id)) {
      continue;
    }
    saw_hv_port = true;
    max_generated_hv_z = std::max(max_generated_hv_z, port.world_position.z);
  }

  const bool ok = saw_hv_port && max_generated_hv_z <= hv_base_z + 1e-6 &&
                  max_generated_hv_z + 0.5 < non_hv_same_layer_base_z;
  if (!ok) {
    std::cerr << "[DBG] C299 saw=" << saw_hv_port << " maxHV=" << max_generated_hv_z
              << " hvBase=" << hv_base_z << " otherBase=" << non_hv_same_layer_base_z << "\n";
  }
  return ok;
}

bool test_bundle_template_order_policy_drives_generation() {
  CoreState state;
  const auto hv_it = state.view().bundle_templates().find(wire::core::BundleKind::kHighVoltage);
  if (hv_it == state.view().bundle_templates().end()) {
    std::cerr << "[DBG] C300 missing HV bundle template\n";
    return false;
  }
  if (hv_it->second.order_decision_policy != wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous) {
    std::cerr << "[DBG] C300 unexpected default policy=" << static_cast<int>(hv_it->second.order_decision_policy) << "\n";
    return false;
  }

  wire::core::BundleTemplate updated = hv_it->second;
  updated.order_decision_policy = wire::core::OrderDecisionPolicyKind::kFixedOrder;
  const auto apply = state.UpdateBundleTemplate(updated);
  if (!apply.ok || !apply.value) {
    std::cerr << "[DBG] C300 UpdateBundleTemplate failed err=" << apply.error << " changed=" << apply.value << "\n";
    return false;
  }

  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    std::cerr << "[DBG] C300 no pole types\n";
    return false;
  }

  wire::core::BackboneSpec request{};
  request.path.polyline = {{-12.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  request.interval_m = 1000.0;
  request.pole_type_id = type_ids.front();
  add_backbone_bundle(request, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(request);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C300 generate failed err=" << generated.error << "\n";
    return false;
  }

  const auto assignment = find_assignment_for_span(state, generated.value.generated_span_ids.front());
  if (!assignment.has_value()) {
    std::cerr << "[DBG] C300 missing assignment\n";
    return false;
  }
  if (assignment->order_decision_policy != wire::core::OrderDecisionPolicyKind::kFixedOrder) {
    std::cerr << "[DBG] C300 assignment policy=" << static_cast<int>(assignment->order_decision_policy) << "\n";
    return false;
  }
  return true;
}

bool test_backbone_drop_template_uses_dedicated_bundle_and_service_layer() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    std::cerr << "[DBG] C301 no pole types\n";
    return false;
  }

  const auto drop_tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kDrop);
  const auto lv_tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  if (drop_tpl_it == state.view().bundle_templates().end() || lv_tpl_it == state.view().bundle_templates().end()) {
    std::cerr << "[DBG] C301 missing bundle templates\n";
    return false;
  }

  wire::core::BackboneSpec request{};
  request.path.polyline = {{-10.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  request.interval_m = 1000.0;
  request.pole_type_id = type_ids.front();
  add_backbone_bundle(request, wire::core::BundleKind::kDrop);

  const auto generated = state.GenerateFromBackboneSpec(request);
  if (!generated.ok || generated.value.bundle_ids.empty() || generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C301 generate failed err=" << generated.error << "\n";
    return false;
  }

  const auto* bundle = state.view().edit_state().bundles.find(generated.value.bundle_ids.front());
  const auto* span = state.view().edit_state().spans.find(generated.value.generated_span_ids.front());
  if (bundle == nullptr || span == nullptr) {
    std::cerr << "[DBG] C301 missing generated entities\n";
    return false;
  }

  bool saw_drop_port = false;
  for (const auto& port : state.view().edit_state().ports.items()) {
    if (port.owner_pole_id == wire::core::kInvalidObjectId ||
        !contains_id(generated.value.generated_pole_ids, port.owner_pole_id) ||
        port.category != wire::core::ConnectionCategory::kDrop) {
      continue;
    }
    saw_drop_port = true;
    if (port.layer != wire::core::PortLayer::kDrop) {
      std::cerr << "[DBG] C301 unexpected port layer=" << static_cast<int>(port.layer) << "\n";
      return false;
    }
  }

  const auto& drop_tpl = drop_tpl_it->second;
  const auto& lv_tpl = lv_tpl_it->second;
  const bool ok = saw_drop_port && bundle->bundle_template_id == wire::core::BundleKind::kDrop &&
                  span->layer == wire::core::SpanLayer::kDrop && span->kind == wire::core::SpanKind::kService &&
                  drop_tpl.default_layer == wire::core::SpanLayer::kDrop &&
                  drop_tpl.cable_template_id != lv_tpl.cable_template_id;
  if (!ok) {
    std::cerr << "[DBG] C301 sawDropPort=" << saw_drop_port
              << " bundleKind=" << static_cast<int>(bundle->bundle_template_id)
              << " spanLayer=" << static_cast<int>(span->layer)
              << " spanKind=" << static_cast<int>(span->kind)
              << " dropLayer=" << static_cast<int>(drop_tpl.default_layer)
              << " dropCable=" << drop_tpl.cable_template_id
              << " lvCable=" << lv_tpl.cable_template_id << "\n";
  }
  return ok;
}

bool test_inspection_all_templates_branch_keeps_hv_lowering_on_communication_pole() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C264 missing CommunicationPole\n";
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = pole_type_id;
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(trunk, wire::core::BundleKind::kOptical);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    std::cerr << "[DBG] C264 trunk generate failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C264 center missing\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {8.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = pole_type_id;
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(branch, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    std::cerr << "[DBG] C264 branch generate failed error=" << generated.error << "\n";
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr ||
        (span->endpoint_node_a_id != center_id && span->endpoint_node_b_id != center_id)) {
      continue;
    }
    const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == wire::core::BundleKind::kHighVoltage) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C264 target HV span missing\n";
    return false;
  }

  const auto span_view = state.view().inspect_span(target_span_id);
  const auto layout_view = state.view().inspect_support_layout(target_span_id);
  if (!span_view.has_value() || !layout_view.has_value()) {
    std::cerr << "[DBG] C264 inspection missing span=" << (span_view.has_value() ? 1 : 0)
              << " layout=" << (layout_view.has_value() ? 1 : 0) << "\n";
    return false;
  }

  const auto endpoint_is_branch_root = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
    return endpoint.owner_pole_id == center_id && endpoint.relation_kind == wire::core::JunctionRelationKind::kSideBranch &&
           endpoint_has_authoritative_lowering(endpoint) && endpoint.branch_down_offset_m > 1e-6;
  };

  const auto group = lowered_support_group_for_owner(*layout_view, center_id);
  const wire::core::SupportLayoutEndpointView* lowered_endpoint =
      endpoint_is_branch_root(layout_view->start_endpoint)
          ? &layout_view->start_endpoint
          : (endpoint_is_branch_root(layout_view->end_endpoint) ? &layout_view->end_endpoint : nullptr);
  const double expected_lift = insulator_lift_for_span_test(state, target_span_id);
  const bool ok = span_view->flow_kind == wire::core::BackboneFlowKind::kBranch &&
                  span_view->branch_down_offset_m > 1e-6 &&
                  layout_view->flow_kind == wire::core::BackboneFlowKind::kBranch &&
                  group.has_value() && lowered_endpoint != nullptr &&
                  group->support_group_id == lowered_endpoint->support_group_id &&
                  almost_equal(group->down_offset_m, lowered_endpoint->branch_down_offset_m, 1e-6) &&
                  almost_equal(lowered_endpoint->support_world, lowered_endpoint->endpoint_world, 1e-6) &&
                  almost_equal(lowered_endpoint->support_world.z - group->tip_world.z, expected_lift, 1e-6) &&
                  almost_equal(group->mount_world.z, group->tip_world.z, 1e-6);
  if (!ok) {
    std::cerr << "[DBG] C264 spanFlow=" << static_cast<int>(span_view->flow_kind)
              << " spanDown=" << span_view->branch_down_offset_m
              << " layoutFlow=" << static_cast<int>(layout_view->flow_kind)
              << " startRelation=" << static_cast<int>(layout_view->start_endpoint.relation_kind)
              << " startDown=" << layout_view->start_endpoint.branch_down_offset_m
              << " endRelation=" << static_cast<int>(layout_view->end_endpoint.relation_kind)
              << " endDown=" << layout_view->end_endpoint.branch_down_offset_m
              << " groupId=" << (group.has_value() ? group->support_group_id : -1)
              << " endpointZ=" << (lowered_endpoint != nullptr ? lowered_endpoint->support_world.z : -1.0)
              << " endpointWireZ=" << (lowered_endpoint != nullptr ? lowered_endpoint->endpoint_world.z : -1.0)
              << " mountZ=" << (group.has_value() ? group->mount_world.z : -1.0)
              << " tipZ=" << (group.has_value() ? group->tip_world.z : -1.0)
              << " expectedLift=" << expected_lift << "\n";
  }
  return ok;
}

bool test_inspection_capture_keeps_branch_then_acute_lowering_on_communication_pole() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C265 missing CommunicationPole\n";
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {
      {-18.59678, 11.0534, 0.0},
      {-6.59678, 11.0534, 0.0},
      {5.40322, 11.0534, 0.0},
  };
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = pole_type_id;
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(trunk, wire::core::BundleKind::kOptical);
  const auto trunk_generated = state.GenerateFromBackboneSpec(trunk);
  if (!trunk_generated.ok) {
    std::cerr << "[DBG] C265 trunk generate failed error=" << trunk_generated.error << "\n";
    return false;
  }

  const ObjectId node0 = find_pole_id_by_position(state, {-6.59678, 11.0534, 0.0}, 1e-4);
  if (node0 == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C265 shared root missing\n";
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {-6.59678, 11.0534, 0.0},
      {-4.93216, 6.7054, 0.0},
      {-13.6709, -1.24875, 0.0},
      {-8.49996, -0.441201, 0.0},
  };
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = node0;
  req.path.node_specs.push_back(shared);
  req.interval_m = 8.0;
  req.pole_type_id = pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(req, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    std::cerr << "[DBG] C265 branch generate failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId node1 = find_pole_id_by_position(state, {-4.93216, 6.7054, 0.0}, 1e-4);
  const ObjectId node2 = find_pole_id_by_position(state, {-13.6709, -1.24875, 0.0}, 1e-4);
  const ObjectId node3 = find_pole_id_by_position(state, {-8.49996, -0.441201, 0.0}, 1e-4);
  if (node1 == wire::core::kInvalidObjectId || node2 == wire::core::kInvalidObjectId ||
      node3 == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C265 node lookup failed\n";
    return false;
  }

  ObjectId branch_span_id = wire::core::kInvalidObjectId;
  ObjectId corner_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      continue;
    }
    const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    const bool is_root_segment =
        (span->endpoint_node_a_id == node0 && span->endpoint_node_b_id == node1) ||
        (span->endpoint_node_a_id == node1 && span->endpoint_node_b_id == node0);
    const bool is_corner_segment =
        ((span->endpoint_node_a_id == node1 && span->endpoint_node_b_id == node2) ||
         (span->endpoint_node_a_id == node2 && span->endpoint_node_b_id == node1) ||
         (span->endpoint_node_a_id == node2 && span->endpoint_node_b_id == node3) ||
         (span->endpoint_node_a_id == node3 && span->endpoint_node_b_id == node2));
    if (is_root_segment) {
      branch_span_id = span_id;
    } else if (corner_span_id == wire::core::kInvalidObjectId && is_corner_segment) {
      corner_span_id = span_id;
    }
  }
  if (branch_span_id == wire::core::kInvalidObjectId || corner_span_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C265 target spans missing branch=" << branch_span_id << " corner=" << corner_span_id << "\n";
    return false;
  }

  const auto branch_span = state.view().inspect_span(branch_span_id);
  const auto branch_layout = state.view().inspect_support_layout(branch_span_id);
  const auto corner_span = state.view().inspect_span(corner_span_id);
  const auto corner_layout = state.view().inspect_support_layout(corner_span_id);
  if (!branch_span.has_value() || !branch_layout.has_value() || !corner_span.has_value() || !corner_layout.has_value()) {
    std::cerr << "[DBG] C265 missing inspection branchSpan=" << (branch_span.has_value() ? 1 : 0)
              << " branchLayout=" << (branch_layout.has_value() ? 1 : 0)
              << " cornerSpan=" << (corner_span.has_value() ? 1 : 0)
              << " cornerLayout=" << (corner_layout.has_value() ? 1 : 0) << "\n";
    return false;
  }

  struct LoweredSnapshot {
    double support_z = std::numeric_limits<double>::quiet_NaN();
    double mount_z = std::numeric_limits<double>::quiet_NaN();
    double tip_z = std::numeric_limits<double>::quiet_NaN();
    double down_offset_m = 0.0;
  };
  const auto collect_lowered_snapshot = [&](const wire::core::SupportLayoutInspectionView& layout, ObjectId owner_pole_id,
                                            wire::core::JunctionRelationKind relation_kind)
      -> std::optional<LoweredSnapshot> {
    const wire::core::SupportLayoutEndpointView* endpoint = nullptr;
    if (layout.start_endpoint.owner_pole_id == owner_pole_id && layout.start_endpoint.relation_kind == relation_kind &&
        endpoint_has_authoritative_lowering(layout.start_endpoint)) {
      endpoint = &layout.start_endpoint;
    } else if (layout.end_endpoint.owner_pole_id == owner_pole_id && layout.end_endpoint.relation_kind == relation_kind &&
               endpoint_has_authoritative_lowering(layout.end_endpoint)) {
      endpoint = &layout.end_endpoint;
    }
    if (endpoint == nullptr) {
      return std::nullopt;
    }
    const auto group = lowered_support_group_for_owner(layout, owner_pole_id);
    if (!group.has_value()) {
      return std::nullopt;
    }
    LoweredSnapshot snapshot{};
    snapshot.support_z = endpoint->support_world.z;
    snapshot.mount_z = group->mount_world.z;
    snapshot.tip_z = group->tip_world.z;
    snapshot.down_offset_m = endpoint->branch_down_offset_m;
    return snapshot;
  };

  const auto branch_group = lowered_support_group_for_owner(*branch_layout, node0);
  const auto corner_group_node1 = lowered_support_group_for_owner(*corner_layout, node1);
  const auto corner_group_node2 = lowered_support_group_for_owner(*corner_layout, node2);
  const auto branch_snapshot =
      collect_lowered_snapshot(*branch_layout, node0, wire::core::JunctionRelationKind::kSideBranch);
  const auto corner_snapshot_node1 =
      collect_lowered_snapshot(*corner_layout, node1, wire::core::JunctionRelationKind::kCornerContinuation);
  const auto corner_snapshot_node2 =
      collect_lowered_snapshot(*corner_layout, node2, wire::core::JunctionRelationKind::kCornerContinuation);
  const std::optional<LoweredSnapshot> corner_snapshot =
      corner_snapshot_node1.has_value() ? corner_snapshot_node1 : corner_snapshot_node2;
  const bool branch_ok = branch_span->flow_kind == wire::core::BackboneFlowKind::kBranch &&
                         branch_span->branch_down_offset_m > 1e-6 &&
                         branch_group.has_value() && branch_snapshot.has_value();
  const bool corner_ok = corner_span->branch_down_offset_m > 1e-6 &&
                         std::max(corner_layout->start_endpoint.branch_down_offset_m,
                                  corner_layout->end_endpoint.branch_down_offset_m) > 1e-6 &&
                         (corner_group_node1.has_value() || corner_group_node2.has_value()) &&
                         corner_snapshot.has_value();
  const double expected_branch_lift = insulator_lift_for_span_test(state, branch_span_id);
  const double expected_corner_lift = insulator_lift_for_span_test(state, corner_span_id);
  const bool shared_one_step_height =
      branch_snapshot.has_value() && corner_snapshot.has_value() &&
      almost_equal(branch_snapshot->support_z - branch_snapshot->tip_z, expected_branch_lift, 1e-6) &&
      almost_equal(branch_snapshot->mount_z, branch_snapshot->tip_z, 1e-6) &&
      almost_equal(corner_snapshot->support_z - corner_snapshot->tip_z, expected_corner_lift, 1e-6) &&
      almost_equal(corner_snapshot->mount_z, corner_snapshot->tip_z, 1e-6) &&
      almost_equal(branch_snapshot->support_z, corner_snapshot->support_z, 1e-6) &&
      almost_equal(branch_snapshot->mount_z, corner_snapshot->mount_z, 1e-6) &&
      almost_equal(branch_snapshot->tip_z, corner_snapshot->tip_z, 1e-6) &&
      almost_equal(branch_snapshot->down_offset_m, corner_snapshot->down_offset_m, 1e-6);
  if (!(branch_ok && corner_ok && shared_one_step_height)) {
    std::cerr << "[DBG] C265 branchFlow=" << static_cast<int>(branch_span->flow_kind)
              << " branchDown=" << branch_span->branch_down_offset_m
              << " branchRelA=" << static_cast<int>(branch_layout->start_endpoint.relation_kind)
              << " branchRelB=" << static_cast<int>(branch_layout->end_endpoint.relation_kind)
              << " cornerSpanDown=" << corner_span->branch_down_offset_m
              << " cornerStartDown=" << corner_layout->start_endpoint.branch_down_offset_m
              << " cornerEndDown=" << corner_layout->end_endpoint.branch_down_offset_m
              << " branchZ=" << (branch_snapshot.has_value() ? branch_snapshot->support_z : -1.0)
              << " cornerZ=" << (corner_snapshot.has_value() ? corner_snapshot->support_z : -1.0)
              << " branchOffset=" << (branch_snapshot.has_value() ? branch_snapshot->down_offset_m : -1.0)
              << " cornerOffset=" << (corner_snapshot.has_value() ? corner_snapshot->down_offset_m : -1.0) << "\n";
  }

  return branch_ok && corner_ok && shared_one_step_height;
}

bool test_backbone_right_angle_junction_has_no_through_pair() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    std::cerr << "[DBG] C213 generate_failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const auto junction = state.view().inspect_junction(center_id);
  if (center_id == wire::core::kInvalidObjectId || !junction.has_value()) {
    std::cerr << "[DBG] C213 center_or_junction_missing center=" << center_id
              << " centerNode=" << static_cast<long long>(center_id)
              << " hasJunction=" << (junction.has_value() ? 1 : 0) << "\n";
    return false;
  }

  bool has_corner = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCornerContinuation) {
      has_corner = true;
      break;
    }
  }
  const bool ok = !junction->through_pair_accepted && junction->through_pair_straightness_score >= 0.0 &&
                  junction->through_pair_straightness_score < 0.3 && has_corner;
  if (!ok) {
    std::cerr << "[DBG] C213 accepted=" << (junction->through_pair_accepted ? 1 : 0)
              << " score=" << junction->through_pair_straightness_score
              << " routeCount=" << junction->route_incident_count << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C213 rel neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " inRoute=" << (relation.in_route ? 1 : 0)
                << " same=" << (relation.same_level_feasible ? 1 : 0) << "\n";
    }
  }
  return ok;
}

bool test_inspection_span_reads_flow_and_turn_from_lane_snapshot() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() < 2) {
    std::cerr << "[DBG] C266 generate_failed ok=" << (generated.ok ? 1 : 0)
              << " spanCount=" << generated.value.generated_span_ids.size()
              << " error=" << generated.error << "\n";
    return false;
  }

  const wire::core::ObjectId target_span_id = generated.value.generated_span_ids.back();
  const auto assignment = find_assignment_for_span(state, target_span_id);
  const auto span_view = state.view().inspect_span(target_span_id);
  if (!assignment.has_value() || !span_view.has_value()) {
    std::cerr << "[DBG] C266 missing assignment=" << (assignment.has_value() ? 1 : 0)
              << " spanView=" << (span_view.has_value() ? 1 : 0) << "\n";
    return false;
  }

  const bool ok = span_view->flow_kind == assignment->flow_kind &&
                  span_view->flow_rule == assignment->flow_decision_rule &&
                  span_view->flipped_from_previous == assignment->flipped_from_previous &&
                  almost_equal(span_view->turn_angle_deg, assignment->turn_angle_deg, 1e-9);
  if (!ok) {
    std::cerr << "[DBG] C266 spanFlow=" << static_cast<int>(span_view->flow_kind)
              << " assignmentFlow=" << static_cast<int>(assignment->flow_kind)
              << " spanRule=" << static_cast<int>(span_view->flow_rule)
              << " assignmentRule=" << static_cast<int>(assignment->flow_decision_rule)
              << " spanFlip=" << (span_view->flipped_from_previous ? 1 : 0)
              << " assignmentFlip=" << (assignment->flipped_from_previous ? 1 : 0)
              << " spanTurn=" << span_view->turn_angle_deg
              << " assignmentTurn=" << assignment->turn_angle_deg << "\n";
  }
  return ok;
}

bool test_backbone_local_corner_projects_to_main_without_local_through() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    std::cerr << "[DBG] C214 generate_failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const auto junction = state.view().inspect_junction(center_id);
  if (center_id == wire::core::kInvalidObjectId || !junction.has_value()) {
    std::cerr << "[DBG] C214 center_or_junction_missing center=" << center_id
              << " centerNode=" << static_cast<long long>(center_id)
              << " hasJunction=" << (junction.has_value() ? 1 : 0) << "\n";
    return false;
  }

  bool has_corner = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCornerContinuation) {
      has_corner = true;
      break;
    }
  }

  int main_touch_count = 0;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        assignment.flow_kind == wire::core::BackboneFlowKind::kMain) {
      ++main_touch_count;
    }
  }
  const bool ok = !junction->through_pair_accepted && has_corner && main_touch_count == 2;
  if (!ok) {
    std::cerr << "[DBG] C214 accepted=" << (junction->through_pair_accepted ? 1 : 0)
              << " mainTouchCount=" << main_touch_count << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C214 rel neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " inRoute=" << (relation.in_route ? 1 : 0) << "\n";
    }
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) {
        std::cerr << "[DBG] C214 assignment flow=" << static_cast<int>(assignment.flow_kind)
                  << " rule=" << static_cast<int>(assignment.flow_decision_rule)
                  << " lowering=" << static_cast<int>(assignment.lowering_kind) << "\n";
      }
    }
  }
  return ok;
}

bool test_backbone_separate_route_merge_keeps_corner_continuation_relation() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-18.59678, 11.0534, 0.0}, {-6.59678, 11.0534, 0.0}, {5.40322, 11.0534, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  const auto trunk_generated = state.GenerateFromBackboneSpec(trunk);
  if (!trunk_generated.ok) {
    std::cerr << "[DBG] C215 trunk_generate_failed error=" << trunk_generated.error << "\n";
    return false;
  }

  const ObjectId root_id = find_pole_id_by_position(state, {-6.59678, 11.0534, 0.0}, 1e-4);
  if (root_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C215 root_missing\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{-6.59678, 11.0534, 0.0}, {-4.93216, 6.7054, 0.0}, {-13.6709, -1.24875, 0.0}, {-8.49996, -0.441201, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = root_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 8.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    std::cerr << "[DBG] C215 branch_generate_failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId corner_id = find_pole_id_by_position(state, {-13.6709, -1.24875, 0.0}, 1e-4);
  const auto junction = state.view().inspect_junction(corner_id);
  if (corner_id == wire::core::kInvalidObjectId || !junction.has_value()) {
    std::cerr << "[DBG] C215 corner_or_junction_missing corner=" << corner_id
              << " cornerNode=" << static_cast<long long>(corner_id)
              << " hasJunction=" << (junction.has_value() ? 1 : 0) << "\n";
    return false;
  }

  bool has_corner = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCornerContinuation) {
      has_corner = true;
      break;
    }
  }

  bool has_acute_lowering = false;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == corner_id || assignment.pole_b_id == corner_id) &&
        assignment.lowering_kind == wire::core::BackboneLoweringKind::kAcuteCorner &&
        !assignment.same_level_feasible) {
      has_acute_lowering = true;
      break;
    }
  }
  const bool ok = !junction->through_pair_accepted && has_corner && has_acute_lowering;
  if (!ok) {
    std::cerr << "[DBG] C215 accepted=" << (junction->through_pair_accepted ? 1 : 0)
              << " routeCount=" << junction->route_incident_count << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C215 rel neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " inRoute=" << (relation.in_route ? 1 : 0)
                << " same=" << (relation.same_level_feasible ? 1 : 0)
                << " reason=" << static_cast<int>(relation.infeasible_reason) << "\n";
    }
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id == corner_id || assignment.pole_b_id == corner_id) {
        std::cerr << "[DBG] C215 assignment flow=" << static_cast<int>(assignment.flow_kind)
                  << " relA=" << static_cast<int>(assignment.relation_a)
                  << " relB=" << static_cast<int>(assignment.relation_b)
                  << " same=" << (assignment.same_level_feasible ? 1 : 0)
                  << " lowering=" << static_cast<int>(assignment.lowering_kind) << "\n";
      }
    }
  }
  return ok;
}

bool test_backbone_explicit_middle_bent_route_stays_corner_main_against_existing_chain() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {12.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec bent{};
  bent.path.polyline = {{4.0, -2.0, 0.0}, {12.0, 0.0, 0.0}, {16.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  bent.path.node_specs.push_back(shared);
  bent.interval_m = 1000.0;
  bent.pole_type_id = type_ids.front();
  add_backbone_bundle(bent, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(bent);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    return false;
  }

  int route_main_like_count = 0;
  int route_cross_count = 0;
  int route_branch_count = 0;
  for (const auto& relation : junction->local_relations) {
    if (!relation.in_route) {
      continue;
    }
    if (relation.kind == wire::core::JunctionRelationKind::kCornerContinuation ||
        relation.kind == wire::core::JunctionRelationKind::kThroughMain) {
      ++route_main_like_count;
    }
    if (relation.kind == wire::core::JunctionRelationKind::kCrossUnderpass) {
      ++route_cross_count;
    }
    if (relation.kind == wire::core::JunctionRelationKind::kSideBranch) {
      ++route_branch_count;
    }
  }

  int generated_touch_count = 0;
  int generated_main_count = 0;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const bool touches_center = assignment.pole_a_id == center_id || assignment.pole_b_id == center_id;
    if (!touches_center) {
      continue;
    }
    ++generated_touch_count;
    if (assignment.flow_kind == wire::core::BackboneFlowKind::kMain) {
      ++generated_main_count;
    }
  }

  const bool ok = route_main_like_count == 2 && route_cross_count == 0 && route_branch_count == 0 &&
                  generated_touch_count > 0 && generated_main_count == generated_touch_count;
  if (!ok) {
    std::cerr << "[DBG] C270 accepted=" << (junction->through_pair_accepted ? 1 : 0)
              << " routeMainLikeCount=" << route_main_like_count
              << " routeCrossCount=" << route_cross_count
              << " routeBranchCount=" << route_branch_count
              << " generatedTouchCount=" << generated_touch_count
              << " generatedMainCount=" << generated_main_count << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C270 rel neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " inRoute=" << (relation.in_route ? 1 : 0) << "\n";
    }
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) {
        std::cerr << "[DBG] C270 assignment flow=" << static_cast<int>(assignment.flow_kind)
                  << " relA=" << static_cast<int>(assignment.relation_a)
                  << " relB=" << static_cast<int>(assignment.relation_b)
                  << " lowerA=" << assignment.decision_a.lower_required
                  << " lowerB=" << assignment.decision_b.lower_required << "\n";
      }
    }
  }
  return ok;
}

bool test_backbone_mirror_does_not_change_relation_or_lowering_root() {
  struct MirrorSnapshot {
    bool ok = false;
    bool through_pair_accepted = false;
    std::vector<wire::core::JunctionRelationKind> relations{};
    std::vector<wire::core::BackboneLoweringKind> lowering_kinds{};
    std::vector<bool> same_level_flags{};
    std::vector<wire::core::SameLevelFeasibilityReason> same_level_reasons{};
    std::vector<double> down_offsets{};
  };

  auto run_case = [](bool allow_mirror) {
    MirrorSnapshot snapshot{};
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return snapshot;
    }

    auto tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kHighVoltage);
    if (tpl_it == state.view().bundle_templates().end()) {
      std::cerr << "[DBG] C216 template_missing allowMirror=" << (allow_mirror ? 1 : 0) << "\n";
      return snapshot;
    }
    wire::core::BundleTemplate tpl = tpl_it->second;
    tpl.allow_mirror = allow_mirror;
    const auto apply = state.UpdateBundleTemplate(tpl);
    if (!apply.ok) {
      std::cerr << "[DBG] C216 template_update_failed allowMirror=" << (allow_mirror ? 1 : 0)
                << " error=" << apply.error << "\n";
      return snapshot;
    }

    wire::core::BackboneSpec req{};
    req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
    req.interval_m = 1000.0;
    req.pole_type_id = type_ids.front();
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    const auto generated = state.GenerateFromBackboneSpec(req);
    if (!generated.ok) {
      std::cerr << "[DBG] C216 generate_failed allowMirror=" << (allow_mirror ? 1 : 0)
                << " error=" << generated.error << "\n";
      return snapshot;
    }

    const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
    const auto junction = state.view().inspect_junction(center_id);
    if (center_id == wire::core::kInvalidObjectId || !junction.has_value()) {
      std::cerr << "[DBG] C216 center_or_junction_missing allowMirror=" << (allow_mirror ? 1 : 0)
                << " center=" << center_id
                << " centerNode=" << static_cast<long long>(center_id)
                << " hasJunction=" << (junction.has_value() ? 1 : 0) << "\n";
      return snapshot;
    }

    snapshot.ok = true;
    snapshot.through_pair_accepted = junction->through_pair_accepted;
    for (const auto& relation : junction->local_relations) {
      if (relation.in_route) {
        snapshot.relations.push_back(relation.kind);
      }
    }
    std::sort(snapshot.relations.begin(), snapshot.relations.end());

    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id != center_id && assignment.pole_b_id != center_id) {
        continue;
      }
      snapshot.lowering_kinds.push_back(assignment.lowering_kind);
      snapshot.same_level_flags.push_back(assignment.same_level_feasible);
      snapshot.same_level_reasons.push_back(assignment.same_level_reason);
      snapshot.down_offsets.push_back(assignment.branch_down_offset_m);
    }
    return snapshot;
  };

  const MirrorSnapshot no_mirror = run_case(false);
  const MirrorSnapshot with_mirror = run_case(true);
  const bool ok = no_mirror.ok && with_mirror.ok &&
         no_mirror.through_pair_accepted == with_mirror.through_pair_accepted &&
         no_mirror.relations == with_mirror.relations &&
         no_mirror.lowering_kinds == with_mirror.lowering_kinds &&
         no_mirror.same_level_flags == with_mirror.same_level_flags &&
         no_mirror.same_level_reasons == with_mirror.same_level_reasons &&
         no_mirror.down_offsets == with_mirror.down_offsets;
  if (!ok) {
    std::cerr << "[DBG] C216 ok0=" << (no_mirror.ok ? 1 : 0) << " ok1=" << (with_mirror.ok ? 1 : 0)
              << " accepted0=" << (no_mirror.through_pair_accepted ? 1 : 0)
              << " accepted1=" << (with_mirror.through_pair_accepted ? 1 : 0)
              << " relCount0=" << no_mirror.relations.size()
              << " relCount1=" << with_mirror.relations.size()
              << " lowerCount0=" << no_mirror.lowering_kinds.size()
              << " lowerCount1=" << with_mirror.lowering_kinds.size() << "\n";
  }
  return ok;
}

bool test_backbone_cross_through_pair_can_still_be_same_level_infeasible() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  const auto trunk_generated = state.GenerateFromBackboneSpec(trunk);
  if (!trunk_generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    return false;
  }

  int infeasible_cross_relations = 0;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCrossUnderpass &&
        !relation.same_level_feasible) {
      ++infeasible_cross_relations;
    }
  }

  int lowered_cross_assignments = 0;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        assignment.lowering_kind == wire::core::BackboneLoweringKind::kCrossUnderpass &&
        !assignment.same_level_feasible) {
      ++lowered_cross_assignments;
    }
  }
  return junction->through_pair_accepted && infeasible_cross_relations >= 1 && lowered_cross_assignments >= 1;
}

bool test_backbone_comm_branch_same_level_can_be_blocked_by_policy() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    return false;
  }

  bool has_infeasible_side_branch = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kSideBranch &&
        !relation.same_level_feasible) {
      has_infeasible_side_branch = true;
      break;
    }
  }

  bool has_policy_block = false;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const auto* bundle = state.view().edit_state().bundles.find(assignment.bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kCommunication) {
      continue;
    }
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        !assignment.same_level_feasible &&
        assignment.same_level_reason == wire::core::SameLevelFeasibilityReason::kCategoryPolicyDisabled &&
        assignment.lowering_blocked_by_policy &&
        assignment.lowering_kind == wire::core::BackboneLoweringKind::kNone) {
      has_policy_block = true;
      break;
    }
  }
  return has_infeasible_side_branch && has_policy_block;
}

bool test_backbone_hv3_corner_feasibility_lowering_keeps_semantic_main() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    std::cerr << "[DBG] C219 generate_failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const auto junction = state.view().inspect_junction(center_id);
  if (center_id == wire::core::kInvalidObjectId || !junction.has_value()) {
    std::cerr << "[DBG] C219 center_or_junction_missing center=" << center_id
              << " centerNode=" << static_cast<long long>(center_id)
              << " hasJunction=" << (junction.has_value() ? 1 : 0) << "\n";
    return false;
  }

  bool has_corner = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCornerContinuation &&
        !relation.same_level_feasible) {
      has_corner = true;
      break;
    }
  }

  int acute_main_assignments = 0;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        assignment.flow_kind == wire::core::BackboneFlowKind::kMain &&
        assignment.lowering_kind == wire::core::BackboneLoweringKind::kAcuteCorner &&
        !assignment.same_level_feasible) {
      ++acute_main_assignments;
    }
  }
  const bool ok = !junction->through_pair_accepted && has_corner && acute_main_assignments == 2;
  if (!ok) {
    std::cerr << "[DBG] C219 accepted=" << (junction->through_pair_accepted ? 1 : 0)
              << " acuteMainAssignments=" << acute_main_assignments << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C219 rel neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " inRoute=" << (relation.in_route ? 1 : 0)
                << " same=" << (relation.same_level_feasible ? 1 : 0)
                << " reason=" << static_cast<int>(relation.infeasible_reason) << "\n";
    }
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) {
        std::cerr << "[DBG] C219 assignment flow=" << static_cast<int>(assignment.flow_kind)
                  << " relA=" << static_cast<int>(assignment.relation_a)
                  << " relB=" << static_cast<int>(assignment.relation_b)
                  << " same=" << (assignment.same_level_feasible ? 1 : 0)
                  << " lowering=" << static_cast<int>(assignment.lowering_kind)
                  << " down=" << assignment.branch_down_offset_m << "\n";
      }
    }
  }
  return ok;
}

bool test_backbone_acute_merge_feasibility_applies_across_route_boundary() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-18.59678, 11.0534, 0.0}, {-6.59678, 11.0534, 0.0}, {5.40322, 11.0534, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId root_id = find_pole_id_by_position(state, {-6.59678, 11.0534, 0.0}, 1e-4);
  if (root_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{-6.59678, 11.0534, 0.0}, {-4.93216, 6.7054, 0.0}, {-13.6709, -1.24875, 0.0}, {-8.49996, -0.441201, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = root_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 8.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    return false;
  }

  bool found_acute_infeasible = false;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if (assignment.lowering_kind == wire::core::BackboneLoweringKind::kAcuteCorner &&
        !assignment.same_level_feasible &&
        (assignment.relation_a == wire::core::JunctionRelationKind::kCornerContinuation ||
         assignment.relation_b == wire::core::JunctionRelationKind::kCornerContinuation)) {
      found_acute_infeasible = true;
      break;
    }
  }
  return found_acute_infeasible;
}

bool test_backbone_recalc_keeps_same_level_lowering_origin() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr && (span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id)) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto span_view = state.view().inspect_span(target_span_id);
  const auto layout_view = state.view().inspect_support_layout(target_span_id);
  if (!span_view.has_value() || !layout_view.has_value()) {
    return false;
  }

  return !span_view->same_level_feasible &&
         span_view->lowering_kind == wire::core::BackboneLoweringKind::kAcuteCorner &&
         !layout_view->same_level_feasible &&
         layout_view->lowering_kind == wire::core::BackboneLoweringKind::kAcuteCorner &&
         (layout_view->relation_a == wire::core::JunctionRelationKind::kCornerContinuation ||
          layout_view->relation_b == wire::core::JunctionRelationKind::kCornerContinuation);
}

bool test_backbone_hv3_corner_uses_constrained_band_solver() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  bool constrained_assignment = false;
  bool constrained_port = false;
  bool special_case_used = false;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id != center_id && assignment.pole_b_id != center_id) ||
        assignment.lowering_kind != wire::core::BackboneLoweringKind::kAcuteCorner ||
        assignment.same_level_feasible) {
      continue;
    }
    constrained_assignment =
        constrained_assignment || (assignment.solver_used_same_level_constraint && !assignment.used_special_case_ports);
    special_case_used = special_case_used || assignment.used_special_case_ports;
    for (ObjectId port_id : assignment.port_ids_a) {
      const auto* port = state.view().edit_state().ports.find(port_id);
      if (port != nullptr && port->owner_pole_id == center_id &&
          port->placement_source == wire::core::PortPlacementSourceKind::kPlacementBandConstrained) {
        constrained_port = true;
      }
    }
    for (ObjectId port_id : assignment.port_ids_b) {
      const auto* port = state.view().edit_state().ports.find(port_id);
      if (port != nullptr && port->owner_pole_id == center_id &&
          port->placement_source == wire::core::PortPlacementSourceKind::kPlacementBandConstrained) {
        constrained_port = true;
      }
    }
  }
  return constrained_assignment && constrained_port && !special_case_used;
}

bool test_backbone_hv3_corner_constrained_solver_spreads_root_ports() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(req).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  std::vector<ObjectId> root_ports{};
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if (assignment.lowering_kind != wire::core::BackboneLoweringKind::kAcuteCorner || assignment.same_level_feasible) {
      continue;
    }
    if (assignment.pole_a_id == center_id && assignment.decision_a.owner_pole_id == center_id &&
        assignment.decision_a.relation_kind == wire::core::JunctionRelationKind::kCornerContinuation &&
        assignment.decision_a.lower_required) {
      root_ports = assignment.port_ids_a;
      break;
    }
    if (assignment.pole_b_id == center_id && assignment.decision_b.owner_pole_id == center_id &&
        assignment.decision_b.relation_kind == wire::core::JunctionRelationKind::kCornerContinuation &&
        assignment.decision_b.lower_required) {
      root_ports = assignment.port_ids_b;
      break;
    }
  }
  if (root_ports.size() != 3) {
    return false;
  }

  std::set<wire::core::SlotSide> sides{};
  std::vector<wire::core::Vec3d> worlds{};
  for (ObjectId port_id : root_ports) {
    const wire::core::Port* port = state.view().edit_state().ports.find(port_id);
    if (port == nullptr) {
      return false;
    }
    sides.insert(port->template_side);
    worlds.push_back(port->world_position);
  }
  if (sides.size() != 3 || sides.count(wire::core::SlotSide::kLeft) == 0 ||
      sides.count(wire::core::SlotSide::kCenter) == 0 || sides.count(wire::core::SlotSide::kRight) == 0) {
    return false;
  }

  for (std::size_t i = 0; i < worlds.size(); ++i) {
    for (std::size_t j = i + 1; j < worlds.size(); ++j) {
      const wire::core::Vec3d delta = worlds[i] - worlds[j];
      if (std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) <= 1e-6) {
        return false;
      }
    }
  }
  return true;
}

bool test_backbone_cross_same_level_infeasible_can_use_constrained_solver() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value() || !junction->through_pair_accepted) {
    return false;
  }

  bool constrained_cross_assignment = false;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id != center_id && assignment.pole_b_id != center_id) ||
        assignment.lowering_kind != wire::core::BackboneLoweringKind::kCrossUnderpass ||
        assignment.same_level_feasible) {
      continue;
    }
    if (assignment.solver_used_same_level_constraint && !assignment.used_special_case_ports) {
      constrained_cross_assignment = true;
      break;
    }
  }
  if (!constrained_cross_assignment) {
    std::cerr << "[DBG] C223 no constrained cross assignment\n";
    for (const auto& assignment : state.view().last_lane_assignments()) {
      const bool touches_center = assignment.pole_a_id == center_id || assignment.pole_b_id == center_id;
      std::cerr << "[DBG] C223 assignment poles=" << assignment.pole_a_id << "->" << assignment.pole_b_id
                << " touchesCenter=" << (touches_center ? 1 : 0)
                << " lowering=" << static_cast<int>(assignment.lowering_kind)
                << " sameLevel=" << (assignment.same_level_feasible ? 1 : 0)
                << " solver=" << (assignment.solver_used_same_level_constraint ? 1 : 0)
                << " special=" << (assignment.used_special_case_ports ? 1 : 0)
                << " relA=" << static_cast<int>(assignment.relation_a)
                << " relB=" << static_cast<int>(assignment.relation_b) << "\n";
    }
  }
  return constrained_cross_assignment;
}

bool test_backbone_policy_blocked_unresolved_survives_recalc_inspection() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr && (span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id)) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto span_view = state.view().inspect_span(target_span_id);
  const auto layout_view = state.view().inspect_support_layout(target_span_id);
  if (!span_view.has_value() || !layout_view.has_value()) {
    return false;
  }

  return !span_view->same_level_feasible &&
         span_view->same_level_reason == wire::core::SameLevelFeasibilityReason::kCategoryPolicyDisabled &&
         span_view->lowering_blocked_by_policy &&
         span_view->unresolved_same_level_conflict &&
         span_view->lowering_kind == wire::core::BackboneLoweringKind::kNone &&
         !layout_view->same_level_feasible &&
         layout_view->same_level_reason == wire::core::SameLevelFeasibilityReason::kCategoryPolicyDisabled &&
         layout_view->lowering_blocked_by_policy &&
         layout_view->unresolved_same_level_conflict &&
         layout_view->lowering_kind == wire::core::BackboneLoweringKind::kNone &&
         (layout_view->start_endpoint.unresolved_same_level_conflict ||
          layout_view->end_endpoint.unresolved_same_level_conflict);
}

bool test_backbone_refresh_keeps_placement_constraint_origin() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }
  if (!state.SetPoleManualYawOverride(center_id, 15.0).ok) {
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr && (span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id)) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto layout_view = state.view().inspect_support_layout(target_span_id);
  if (!layout_view.has_value()) {
    return false;
  }

  const bool start_constrained = layout_view->start_endpoint.origin == "PlacementConstraint" &&
                                 layout_view->start_endpoint.port_source == "PlacementBandConstrained";
  const bool end_constrained = layout_view->end_endpoint.origin == "PlacementConstraint" &&
                               layout_view->end_endpoint.port_source == "PlacementBandConstrained";
  return layout_view->solver_used_same_level_constraint &&
         !layout_view->used_special_case_ports &&
         layout_view->lowering_kind == wire::core::BackboneLoweringKind::kAcuteCorner &&
         (layout_view->relation_a == wire::core::JunctionRelationKind::kCornerContinuation ||
          layout_view->relation_b == wire::core::JunctionRelationKind::kCornerContinuation) &&
         (start_constrained || end_constrained);
}

bool test_backbone_mirror_does_not_change_constrained_solver_usage() {
  struct MirrorSolverSnapshot {
    bool ok = false;
    std::vector<bool> solver_flags{};
    std::vector<bool> special_flags{};
    std::vector<bool> unresolved_flags{};
    std::vector<wire::core::SameLevelFeasibilityReason> reasons{};
    std::vector<wire::core::BackboneLoweringKind> lowering{};
  };

  auto run_case = [](bool allow_mirror) {
    MirrorSolverSnapshot snapshot{};
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return snapshot;
    }

    auto tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kHighVoltage);
    if (tpl_it == state.view().bundle_templates().end()) {
      return snapshot;
    }
    wire::core::BundleTemplate tpl = tpl_it->second;
    tpl.allow_mirror = allow_mirror;
    if (!state.UpdateBundleTemplate(tpl).ok) {
      return snapshot;
    }

    wire::core::BackboneSpec req{};
    req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
    req.interval_m = 1000.0;
    req.pole_type_id = type_ids.front();
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    if (!state.GenerateFromBackboneSpec(req).ok) {
      return snapshot;
    }

    const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
    if (center_id == wire::core::kInvalidObjectId) {
      return snapshot;
    }

    snapshot.ok = true;
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id != center_id && assignment.pole_b_id != center_id) {
        continue;
      }
      snapshot.solver_flags.push_back(assignment.solver_used_same_level_constraint);
      snapshot.special_flags.push_back(assignment.used_special_case_ports);
      snapshot.unresolved_flags.push_back(assignment.unresolved_same_level_conflict);
      snapshot.reasons.push_back(assignment.same_level_reason);
      snapshot.lowering.push_back(assignment.lowering_kind);
    }
    return snapshot;
  };

  const MirrorSolverSnapshot no_mirror = run_case(false);
  const MirrorSolverSnapshot with_mirror = run_case(true);
  return no_mirror.ok && with_mirror.ok &&
         no_mirror.solver_flags == with_mirror.solver_flags &&
         no_mirror.special_flags == with_mirror.special_flags &&
         no_mirror.unresolved_flags == with_mirror.unresolved_flags &&
         no_mirror.reasons == with_mirror.reasons &&
         no_mirror.lowering == with_mirror.lowering;
}

bool test_backbone_cross_relation_survives_support_layout_recalc() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr && (span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id)) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto span_view = state.view().inspect_span(target_span_id);
  const auto layout_view = state.view().inspect_support_layout(target_span_id);
  if (!span_view.has_value() || !layout_view.has_value()) {
    return false;
  }

  return !span_view->same_level_feasible &&
         span_view->lowering_kind == wire::core::BackboneLoweringKind::kCrossUnderpass &&
         !layout_view->same_level_feasible &&
         layout_view->lowering_kind == wire::core::BackboneLoweringKind::kCrossUnderpass &&
         (layout_view->relation_a == wire::core::JunctionRelationKind::kCrossUnderpass ||
          layout_view->relation_b == wire::core::JunctionRelationKind::kCrossUnderpass);
}

bool test_backbone_hv3_branch_default_lower_required() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(branch).ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    return false;
  }

  bool has_bundle_rule_branch = false;
  bool has_lowered_branch_assignment = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kSideBranch &&
        relation.continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
        relation.default_lower_required && !relation.same_level_feasible &&
        relation.infeasible_reason == wire::core::SameLevelFeasibilityReason::kBundleRule) {
      has_bundle_rule_branch = true;
      break;
    }
  }
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        assignment.continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
        assignment.default_lower_required &&
        ((assignment.decision_a.owner_pole_id == center_id && assignment.decision_a.lower_required &&
          assignment.decision_a.support_group_id >= 0) ||
         (assignment.decision_b.owner_pole_id == center_id && assignment.decision_b.lower_required &&
          assignment.decision_b.support_group_id >= 0)) &&
        !assignment.same_level_feasible) {
      has_lowered_branch_assignment = true;
      break;
    }
  }
  return has_bundle_rule_branch && has_lowered_branch_assignment;
}

bool test_backbone_hv3_corner_continuation_default_lower_required() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(req).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const auto junction = state.view().inspect_junction(center_id);
  if (center_id == wire::core::kInvalidObjectId || !junction.has_value()) {
    return false;
  }

  bool has_bundle_rule_corner = false;
  int lowered_corner_assignments = 0;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCornerContinuation &&
        relation.continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
        relation.default_lower_required && !relation.same_level_feasible &&
        relation.infeasible_reason == wire::core::SameLevelFeasibilityReason::kBundleRule) {
      has_bundle_rule_corner = true;
      break;
    }
  }
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        assignment.continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
        assignment.default_lower_required &&
        ((assignment.decision_a.owner_pole_id == center_id &&
          assignment.decision_a.relation_kind == wire::core::JunctionRelationKind::kCornerContinuation &&
          assignment.decision_a.lower_required && assignment.decision_a.support_group_id >= 0) ||
         (assignment.decision_b.owner_pole_id == center_id &&
          assignment.decision_b.relation_kind == wire::core::JunctionRelationKind::kCornerContinuation &&
          assignment.decision_b.lower_required && assignment.decision_b.support_group_id >= 0)) &&
        !assignment.same_level_feasible) {
      ++lowered_corner_assignments;
    }
  }
  return !junction->through_pair_accepted && has_bundle_rule_corner && lowered_corner_assignments == 2;
}

bool test_backbone_hv3_cross_only_through_pair_stays_same_level_candidate() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(cross).ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value() || !junction->through_pair_accepted) {
    return false;
  }

  int through_candidates = 0;
  int lowered_nonthrough = 0;
  for (const auto& relation : junction->local_relations) {
    if (relation.continuity_class != wire::core::ContinuityCategoryClass::kBundleLike) {
      continue;
    }
    if (relation.in_through_pair && relation.kind == wire::core::JunctionRelationKind::kThroughMain &&
        !relation.default_lower_required && relation.same_level_feasible) {
      ++through_candidates;
    } else if (!relation.in_through_pair &&
               relation.kind == wire::core::JunctionRelationKind::kCrossUnderpass &&
        relation.default_lower_required && !relation.same_level_feasible &&
               relation.infeasible_reason == wire::core::SameLevelFeasibilityReason::kBundleRule) {
      ++lowered_nonthrough;
    }
  }
  return through_candidates == 2 && lowered_nonthrough == 2;
}

bool test_backbone_point_like_branch_can_keep_same_level_when_clear() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(branch).ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    return false;
  }

  bool has_point_like_same_level_branch = false;
  bool has_unlowered_assignment = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kSideBranch &&
        relation.continuity_class == wire::core::ContinuityCategoryClass::kPointLike &&
        !relation.default_lower_required && relation.same_level_feasible) {
      has_point_like_same_level_branch = true;
      break;
    }
  }
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        assignment.continuity_class == wire::core::ContinuityCategoryClass::kPointLike &&
        !assignment.default_lower_required && assignment.same_level_feasible &&
        !assignment.decision_a.lower_required && !assignment.decision_b.lower_required &&
        assignment.decision_a.support_group_id < 0 && assignment.decision_b.support_group_id < 0) {
      has_unlowered_assignment = true;
      break;
    }
  }
  return has_point_like_same_level_branch && has_unlowered_assignment;
}

bool test_backbone_bundle_rule_policy_block_stays_unresolved() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    return false;
  }

  bool has_bundle_rule_relation = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kSideBranch &&
        relation.continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
        relation.default_lower_required && !relation.same_level_feasible &&
        relation.infeasible_reason == wire::core::SameLevelFeasibilityReason::kBundleRule) {
      has_bundle_rule_relation = true;
      break;
    }
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr && (span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id)) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto span_view = state.view().inspect_span(target_span_id);
  if (!span_view.has_value()) {
    return false;
  }
      return has_bundle_rule_relation &&
        span_view->continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
        span_view->default_lower_required &&
        !span_view->same_level_feasible &&
        span_view->same_level_reason == wire::core::SameLevelFeasibilityReason::kCategoryPolicyDisabled &&
        span_view->unresolved_same_level_conflict;
}

bool test_backbone_refresh_keeps_bundle_rule_origin() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    return false;
  }
  if (!state.SetPoleManualYawOverride(center_id, 15.0).ok) {
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr && (span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id)) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto span_view = state.view().inspect_span(target_span_id);
  const auto layout_view = state.view().inspect_support_layout(target_span_id);
  if (!span_view.has_value() || !layout_view.has_value()) {
    return false;
  }

  const bool ok = span_view->continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
         span_view->default_lower_required &&
         !span_view->same_level_feasible &&
         span_view->same_level_reason == wire::core::SameLevelFeasibilityReason::kBundleRule &&
         layout_view->continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
         layout_view->default_lower_required &&
         !layout_view->same_level_feasible &&
         layout_view->same_level_reason == wire::core::SameLevelFeasibilityReason::kBundleRule;
  return ok;
}

bool test_backbone_cross_lowered_pair_uses_opposite_junction_pair_sides() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec horizontal{};
  horizontal.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  horizontal.interval_m = 1000.0;
  horizontal.pole_type_id = type_ids.front();
  add_backbone_bundle(horizontal, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(horizontal).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec vertical{};
  vertical.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  vertical.interval_m = 1000.0;
  vertical.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  vertical.path.node_specs.push_back(shared);
  add_backbone_bundle(vertical, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(vertical);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  bool saw_non_center = false;
  bool saw_pair_side = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value() ||
        layout_view->lowering_kind != wire::core::BackboneLoweringKind::kCrossUnderpass) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value()) {
      continue;
    }
    if (endpoint->used_junction_pair_side_assignment) {
      saw_pair_side = true;
    }
    if (std::abs(endpoint->chosen_side_sign) > 0.5) {
      saw_non_center = true;
    }
  }
  if (!(saw_pair_side && saw_non_center)) {
    std::cerr << "[DBG] C234 pairSide=" << saw_pair_side << " nonCenter=" << saw_non_center << "\n";
  }
  return saw_pair_side && saw_non_center;
}

bool test_backbone_constrained_lowered_support_prefers_line_direction() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const wire::core::Vec3d pair_normal = normalize_xy_safe({0.0, 1.0, 0.0});
  bool saw_constrained_visual = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    const wire::core::Bundle* bundle =
        (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->origin != "PlacementConstraint") {
      continue;
    }
    if (endpoint->support_orientation_rule == wire::core::SupportOrientationRuleKind::kRadial) {
      std::cerr << "[DBG] C235 radial endpoint remained constrained\n";
      continue;
    }
    const auto placement = lowered_support_group_for_owner(*layout_view, center_id);
    if (!placement.has_value() || placement->origin != "PlacementConstraint") {
      continue;
    }
    const wire::core::Vec3d support_axis = normalize_xy_safe(placement->tip_world - placement->mount_world);
    if (std::abs(dot_xy(support_axis, pair_normal)) >= 0.97 &&
        placement->support_orientation_basis != wire::core::SupportOrientationBasisKind::kRadial) {
      saw_constrained_visual = true;
    } else {
      std::cerr << "[DBG] C235 bad axis align=" << std::abs(dot_xy(support_axis, pair_normal)) << " mount=("
                << placement->mount_world.x << "," << placement->mount_world.y << "," << placement->mount_world.z
                << ") tip=(" << placement->tip_world.x << "," << placement->tip_world.y << ","
                << placement->tip_world.z << ") sideSign=" << placement->chosen_side_sign << " origin="
                << placement->origin << " sideRule="
                << static_cast<int>(placement->side_assignment_rule) << " orientRule="
                << static_cast<int>(placement->support_orientation_rule) << "\n";
    }
  }
  return saw_constrained_visual;
}

bool test_backbone_bundle_branch_support_orientation_uses_bisector_when_available() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 14.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const wire::core::Vec3d branch_dir = normalize_xy_safe({10.0, 14.0, 0.0});
  bool found = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() ||
        endpoint->support_orientation_rule != wire::core::SupportOrientationRuleKind::kBisector) {
      continue;
    }
    const auto placement = lowered_support_group_for_owner(*layout_view, center_id);
    if (!placement.has_value()) {
      continue;
    }
    const wire::core::Vec3d trunk_dir = normalize_xy_safe({12.0, 0.0, 0.0});
    const wire::core::Vec3d expected_axis = normalize_xy_safe(branch_dir + trunk_dir);
    const wire::core::Vec3d support_axis = normalize_xy_safe(placement->tip_world - placement->mount_world);
    if (std::abs(dot_xy(support_axis, expected_axis)) >= 0.97 &&
        placement->support_orientation_basis != wire::core::SupportOrientationBasisKind::kRadial) {
      found = true;
    }
  }
  if (!found) {
    for (ObjectId span_id : generated.value.generated_span_ids) {
      const auto layout_view = state.view().inspect_support_layout(span_id);
      if (!layout_view.has_value()) {
        continue;
      }
      const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
      if (!endpoint.has_value()) {
        continue;
      }
      if (endpoint->lower_required &&
          endpoint->relation_kind == wire::core::JunctionRelationKind::kSideBranch &&
          endpoint->support_orientation_rule == wire::core::SupportOrientationRuleKind::kBisector &&
          (endpoint->support_orientation_basis ==
               wire::core::SupportOrientationBasisKind::kBisectorForward ||
           endpoint->support_orientation_basis ==
               wire::core::SupportOrientationBasisKind::kBisectorReverse)) {
        return true;
      }
    }
  }
  return found;
}

bool test_backbone_bundle_branch_lowering_stays_pole_local() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}, {12.0, 12.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.size() < 2) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  int grouped_segments = 0;
  int locally_lowered_segments = 0;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    if (!layout_view->lowered_support_groups.empty()) {
      ++grouped_segments;
    }
    const bool local_lower =
        layout_view->start_endpoint.lower_required || layout_view->end_endpoint.lower_required;
    if (local_lower) {
      ++locally_lowered_segments;
    }
  }
  if (!(grouped_segments >= 2 && grouped_segments == locally_lowered_segments)) {
    std::cerr << "[DBG] C250 grouped=" << grouped_segments << " local=" << locally_lowered_segments << "\n";
  }
  return grouped_segments >= 2 && grouped_segments == locally_lowered_segments;
}

bool test_backbone_cross_underpass_supports_share_one_side_group() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  std::vector<wire::core::LoweredSupportGroupInspectionView> placements{};
  std::vector<GroupedRankFamilySnapshot> rank_families{};
  struct EndpointAttachSnapshot {
    ObjectId span_id = wire::core::kInvalidObjectId;
    wire::core::Vec3d endpoint_world{};
    wire::core::Vec3d support_world{};
  };
  std::vector<EndpointAttachSnapshot> attach_points{};
  std::vector<wire::core::SupportLayoutEndpointView> lowered_endpoint_decisions{};
  for (const auto& span_entry : state.view().spans().items()) {
    const ObjectId span_id = span_entry.id;
    const auto support_layout = state.view().inspect_support_layout(span_id);
    if (!support_layout.has_value()) {
      continue;
    }
    for (const auto& placement : support_layout->lowered_support_groups) {
      if (placement.owner_pole_id == center_id &&
          placement.relation_kind == wire::core::JunctionRelationKind::kCrossUnderpass) {
        placements.push_back(placement);
      }
    }
    const auto collect_endpoint = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
      if (endpoint.owner_pole_id == center_id &&
          endpoint.relation_kind == wire::core::JunctionRelationKind::kCrossUnderpass &&
          endpoint.lower_required) {
        attach_points.push_back({span_id, endpoint.endpoint_world, endpoint.support_world});
        lowered_endpoint_decisions.push_back(endpoint);
      }
    };
    collect_endpoint(support_layout->start_endpoint);
    collect_endpoint(support_layout->end_endpoint);
    }
  if (placements.empty()) {
    std::cerr << "[DBG] C251 missing grouped lowered cross support\n";
    return false;
  }
  if (!collect_grouped_rank_family_snapshots(state, center_id, &rank_families,
                                             wire::core::JunctionRelationKind::kCrossUnderpass) ||
      rank_families.size() != 1 || !grouped_rank_family_has_consistent_owner_and_endpoints(rank_families.front())) {
    std::cerr << "[DBG] C251 rank family missing/invalid count=" << rank_families.size();
    if (!rank_families.empty()) {
      const auto& family = rank_families.front();
      std::cerr << " owner=" << family.owner_pole_id << " group=" << family.support_group_id
                << " rank=" << family.pair_height_rank << " endpoints=" << family.endpoints.size();
    }
    std::cerr << "\n";
    return false;
  }
  std::unordered_set<std::uint64_t> unique_supports{};
  for (const auto& placement : placements) {
    const std::uint64_t support_key =
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(placement.owner_pole_id)) << 32) ^
        static_cast<std::uint32_t>(placement.support_group_id);
    unique_supports.insert(support_key);
  }
  if (unique_supports.size() != 1) {
    std::cerr << "[DBG] C251 expected one unique grouped lowered cross support count=" << unique_supports.size()
              << " placements=" << placements.size() << "\n";
    return false;
  }
  const int group_id = placements.front().support_group_id;
  const double side_sign = placements.front().chosen_side_sign;
  const auto side_rule = placements.front().side_assignment_rule;
  const auto orientation_rule = placements.front().support_orientation_rule;
  const auto group_basis = placements.front().support_orientation_basis;
  const auto group_side = placements.front().chosen_side;
  for (const auto& placement : placements) {
    if (placement.grouping_rule != wire::core::SupportGroupingRuleKind::kDecisionGroup ||
        placement.support_group_id != group_id ||
        placement.side_assignment_rule != side_rule ||
        placement.support_orientation_rule != orientation_rule ||
        placement.side_assignment_rule == wire::core::SideAssignmentRuleKind::kPoleLocal ||
        placement.support_orientation_rule == wire::core::SupportOrientationRuleKind::kRadial ||
        std::abs(placement.chosen_side_sign - side_sign) > 1e-9 ||
        !almost_equal(placement.side_axis.x, placements.front().side_axis.x, 1e-6) ||
        !almost_equal(placement.side_axis.y, placements.front().side_axis.y, 1e-6) ||
        !almost_equal(placement.mount_world.x, placements.front().mount_world.x, 1e-6) ||
        !almost_equal(placement.mount_world.y, placements.front().mount_world.y, 1e-6) ||
        !almost_equal(placement.tip_world.x, placements.front().tip_world.x, 1e-6) ||
        !almost_equal(placement.tip_world.y, placements.front().tip_world.y, 1e-6)) {
        std::cerr << "[DBG] C251 split support group=" << placement.support_group_id << " ref=" << group_id
                  << " side=" << placement.chosen_side_sign << " refSide=" << side_sign << " mount=("
                  << placement.mount_world.x << "," << placement.mount_world.y << "," << placement.mount_world.z
                  << ") refMount=(" << placements.front().mount_world.x << "," << placements.front().mount_world.y
                  << "," << placements.front().mount_world.z << ")\n";
        return false;
      }
  }
  for (const auto& decision : lowered_endpoint_decisions) {
    if (decision.side_assignment_rule != side_rule ||
        decision.support_orientation_rule != orientation_rule ||
        decision.support_orientation_basis != group_basis ||
        decision.chosen_side != group_side ||
        std::abs(decision.chosen_side_sign - side_sign) > 1e-9) {
      std::cerr << "[DBG] C251 endpoint decision mismatch basis="
                << static_cast<int>(decision.support_orientation_basis) << " groupBasis="
                << static_cast<int>(group_basis) << " side=" << static_cast<int>(decision.chosen_side)
                << " groupSide=" << static_cast<int>(group_side) << "\n";
      return false;
    }
  }
  for (const auto& attach : attach_points) {
    const double expected_lift = insulator_lift_for_span_test(state, attach.span_id);
    if (!almost_equal(attach.support_world, attach.endpoint_world, 1e-6) ||
        !almost_equal(attach.support_world.z - placements.front().tip_world.z, expected_lift, 1e-6)) {
      std::cerr << "[DBG] C251 grouped attach mismatch span=" << attach.span_id << " support=("
                << attach.support_world.x << "," << attach.support_world.y << "," << attach.support_world.z
                << ") endpoint=(" << attach.endpoint_world.x << "," << attach.endpoint_world.y << ","
                << attach.endpoint_world.z << ") tipZ=" << placements.front().tip_world.z
                << " expectedLift=" << expected_lift << "\n";
      return false;
    }
  }
  for (const auto& decision : lowered_endpoint_decisions) {
    if (!decision.lower_required || decision.lowering_blocked_by_policy || decision.support_group_id != group_id) {
      std::cerr << "[DBG] C251 lower decision mismatch group=" << decision.support_group_id
                << " expected=" << group_id << " lowerRequired=" << decision.lower_required
                << " blocked=" << decision.lowering_blocked_by_policy << "\n";
      return false;
    }
  }
  return true;
}

bool test_backbone_refresh_keeps_local_lower_and_grouped_support() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  auto grouped_support_count_for_center = [&](ObjectId span_id) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      return 0;
    }
    int grouped_count = 0;
    for (const auto& placement : layout_view->lowered_support_groups) {
      if (placement.owner_pole_id == center_id &&
          placement.grouping_rule == wire::core::SupportGroupingRuleKind::kDecisionGroup) {
        ++grouped_count;
      }
    }
    return grouped_count;
  };

  struct EndpointSnapshot {
    int support_group_id = -1;
    wire::core::Vec3d endpoint_world{};
    wire::core::Vec3d support_world{};
    wire::core::SideAssignmentRuleKind side_assignment_rule = wire::core::SideAssignmentRuleKind::kPoleLocal;
    wire::core::SupportOrientationRuleKind support_orientation_rule = wire::core::SupportOrientationRuleKind::kRadial;
    wire::core::SupportOrientationBasisKind support_orientation_basis = wire::core::SupportOrientationBasisKind::kRadial;
    wire::core::Vec3d side_axis{};
    double chosen_side_sign = 0.0;
    bool lower_required = false;
    bool default_lower_required = false;
    wire::core::JunctionRelationKind relation_kind = wire::core::JunctionRelationKind::kNone;
  };
  struct GroupSnapshot {
    int support_group_id = -1;
    wire::core::Vec3d mount_world{};
    wire::core::Vec3d tip_world{};
    wire::core::SideAssignmentRuleKind side_assignment_rule = wire::core::SideAssignmentRuleKind::kPoleLocal;
    wire::core::SupportOrientationRuleKind support_orientation_rule = wire::core::SupportOrientationRuleKind::kRadial;
    wire::core::SupportOrientationBasisKind support_orientation_basis = wire::core::SupportOrientationBasisKind::kRadial;
    wire::core::Vec3d side_axis{};
    double chosen_side_sign = 0.0;
    bool lower_required = false;
    bool default_lower_required = false;
    wire::core::JunctionRelationKind relation_kind = wire::core::JunctionRelationKind::kNone;
  };
  auto snapshot_endpoint = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
    EndpointSnapshot s{};
    s.support_group_id = endpoint.support_group_id;
    s.endpoint_world = endpoint.endpoint_world;
    s.support_world = endpoint.support_world;
    s.side_assignment_rule = endpoint.side_assignment_rule;
    s.support_orientation_rule = endpoint.support_orientation_rule;
    s.support_orientation_basis = endpoint.support_orientation_basis;
    s.side_axis = endpoint.side_axis;
    s.chosen_side_sign = endpoint.chosen_side_sign;
    s.lower_required = endpoint.lower_required;
    s.default_lower_required = endpoint.default_lower_required;
    s.relation_kind = endpoint.relation_kind;
    return s;
  };
  auto snapshot_group_for_center = [&](const wire::core::SupportLayoutInspectionView& layout) {
    GroupSnapshot s{};
    for (const auto& g : layout.lowered_support_groups) {
      if (g.owner_pole_id != center_id) {
        continue;
      }
      s.support_group_id = g.support_group_id;
      s.mount_world = g.mount_world;
      s.tip_world = g.tip_world;
      s.side_assignment_rule = g.side_assignment_rule;
      s.support_orientation_rule = g.support_orientation_rule;
      s.support_orientation_basis = g.support_orientation_basis;
      s.side_axis = g.side_axis;
      s.chosen_side_sign = g.chosen_side_sign;
      s.lower_required = g.lower_required;
      s.relation_kind = g.relation_kind;
      break;
    }
    return s;
  };

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    if (grouped_support_count_for_center(span_id) > 0) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto before = state.view().inspect_support_layout(target_span_id);
  const int before_grouped = grouped_support_count_for_center(target_span_id);
  if (!before.has_value() || before_grouped == 0) {
    return false;
  }
  const double expected_lift = insulator_lift_for_span_test(state, target_span_id);
  const auto before_endpoint = before->start_endpoint.owner_pole_id == center_id
                                   ? snapshot_endpoint(before->start_endpoint)
                                   : snapshot_endpoint(before->end_endpoint);
  const auto before_group = snapshot_group_for_center(*before);
  if (!state.SetPoleManualYawOverride(center_id, 19.0).ok) {
    return false;
  }
  const auto after = state.view().inspect_support_layout(target_span_id);
  const int after_grouped = grouped_support_count_for_center(target_span_id);
  if (!after.has_value()) {
    return false;
  }
  const auto after_endpoint = after->start_endpoint.owner_pole_id == center_id
                                  ? snapshot_endpoint(after->start_endpoint)
                                  : snapshot_endpoint(after->end_endpoint);
  const auto after_group = snapshot_group_for_center(*after);
  return after.has_value() &&
         before->start_endpoint.lower_required == after->start_endpoint.lower_required &&
         before->end_endpoint.lower_required == after->end_endpoint.lower_required &&
         before_grouped == after_grouped && after_grouped > 0 &&
         before_endpoint.support_group_id == after_endpoint.support_group_id &&
         before_group.support_group_id == after_group.support_group_id &&
         almost_equal(before_endpoint.support_world, before_endpoint.endpoint_world, 1e-6) &&
         almost_equal(after_endpoint.support_world, after_endpoint.endpoint_world, 1e-6) &&
         almost_equal(before_endpoint.support_world.z - before_group.tip_world.z, expected_lift, 1e-6) &&
         almost_equal(after_endpoint.support_world.z - after_group.tip_world.z, expected_lift, 1e-6) &&
         almost_equal(before_group.mount_world.x, after_group.mount_world.x, 1e-6) &&
         almost_equal(before_group.mount_world.y, after_group.mount_world.y, 1e-6) &&
         almost_equal(before_group.mount_world.z, after_group.mount_world.z, 1e-6) &&
         almost_equal(before_group.tip_world.x, after_group.tip_world.x, 1e-6) &&
         almost_equal(before_group.tip_world.y, after_group.tip_world.y, 1e-6) &&
         almost_equal(before_group.tip_world.z, after_group.tip_world.z, 1e-6) &&
         before_endpoint.side_assignment_rule == after_endpoint.side_assignment_rule &&
         before_endpoint.support_orientation_rule == after_endpoint.support_orientation_rule &&
         before_endpoint.support_orientation_basis == after_endpoint.support_orientation_basis &&
         almost_equal(before_endpoint.side_axis.x, after_endpoint.side_axis.x, 1e-6) &&
         almost_equal(before_endpoint.side_axis.y, after_endpoint.side_axis.y, 1e-6) &&
         almost_equal(before_endpoint.chosen_side_sign, after_endpoint.chosen_side_sign, 1e-9) &&
         before_group.side_assignment_rule == after_group.side_assignment_rule &&
         before_group.support_orientation_rule == after_group.support_orientation_rule &&
         before_group.support_orientation_basis == after_group.support_orientation_basis &&
         almost_equal(before_group.side_axis.x, after_group.side_axis.x, 1e-6) &&
         almost_equal(before_group.side_axis.y, after_group.side_axis.y, 1e-6) &&
         almost_equal(before_group.chosen_side_sign, after_group.chosen_side_sign, 1e-9) &&
         before_endpoint.lower_required == after_endpoint.lower_required &&
         before_endpoint.default_lower_required == after_endpoint.default_lower_required &&
         before_endpoint.relation_kind == after_endpoint.relation_kind;
  }

bool test_backbone_grouped_support_visual_cache_uses_single_group_placement() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  std::vector<GroupedRankFamilySnapshot> rank_families{};
  if (!collect_grouped_rank_family_snapshots(state, center_id, &rank_families) || rank_families.empty()) {
    std::cerr << "[DBG] C271 missing_rank_families center=" << center_id << "\n";
    return false;
  }
  const bool all_families_consistent = std::all_of(rank_families.begin(), rank_families.end(),
                                                   [](const GroupedRankFamilySnapshot& family) {
                                                     return grouped_rank_family_has_consistent_owner_and_endpoints(family);
                                                   });
  if (!all_families_consistent) {
    std::cerr << "[DBG] C271 no_consistent_rank_family count=" << rank_families.size() << "\n";
    return false;
  }

  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    const wire::core::SpanVisualCacheEntry* visual = state.view().find_span_visual_cache(span_id);
    if (!layout_view.has_value() || visual == nullptr) {
      continue;
    }
    const auto group = lowered_support_group_for_owner(*layout_view, center_id);
    if (!group.has_value()) {
      continue;
    }

    std::vector<wire::core::Vec3d> span_attachment_worlds{};
    if (layout_view->start_endpoint.owner_pole_id == center_id && endpoint_has_authoritative_lowering(layout_view->start_endpoint)) {
      span_attachment_worlds.push_back(layout_view->start_endpoint.endpoint_world);
    }
    if (layout_view->end_endpoint.owner_pole_id == center_id && endpoint_has_authoritative_lowering(layout_view->end_endpoint)) {
      span_attachment_worlds.push_back(layout_view->end_endpoint.endpoint_world);
    }
    if (span_attachment_worlds.empty()) {
      continue;
    }
    const double expected_lift = insulator_lift_for_span_test(state, span_id);

    int matching_arms = 0;
    int matching_hangers = 0;
    int matching_insulators = 0;
    for (const auto& part : visual->parts) {
      if (part.kind == wire::core::VisualPartKind::kSupportArm &&
          almost_equal(part.a.x, group->mount_world.x, 1e-6) &&
          almost_equal(part.a.y, group->mount_world.y, 1e-6) &&
          almost_equal(part.a.z, group->mount_world.z, 1e-6) &&
          almost_equal(part.b.x, group->tip_world.x, 1e-6) &&
          almost_equal(part.b.y, group->tip_world.y, 1e-6) &&
          almost_equal(part.b.z, group->tip_world.z, 1e-6)) {
        ++matching_arms;
      }
      if (part.kind == wire::core::VisualPartKind::kFitting &&
          almost_equal(part.a.x, group->tip_world.x, 1e-6) &&
          almost_equal(part.a.y, group->tip_world.y, 1e-6) &&
          almost_equal(part.a.z, group->tip_world.z, 1e-6)) {
        const bool matches_attachment = std::any_of(
            span_attachment_worlds.begin(), span_attachment_worlds.end(), [&](const wire::core::Vec3d& attachment) {
              return almost_equal(part.b.x, attachment.x, 1e-6) && almost_equal(part.b.y, attachment.y, 1e-6) &&
                     almost_equal(part.b.z, attachment.z - expected_lift, 1e-6);
            });
        if (matches_attachment) {
          ++matching_hangers;
        }
      }
      if (part.kind == wire::core::VisualPartKind::kInsulator) {
        const bool matches_attachment = std::any_of(
            span_attachment_worlds.begin(), span_attachment_worlds.end(), [&](const wire::core::Vec3d& attachment) {
              return almost_equal(part.b.x, attachment.x, 1e-6) && almost_equal(part.b.y, attachment.y, 1e-6) &&
                     almost_equal(part.b.z, attachment.z, 1e-6);
            });
        if (matches_attachment) {
          ++matching_insulators;
        }
      }
    }
    if (matching_arms != 1 || matching_hangers != static_cast<int>(span_attachment_worlds.size()) ||
        matching_insulators != static_cast<int>(span_attachment_worlds.size())) {
      std::cerr << "[DBG] C271 span=" << span_id << " arms=" << matching_arms
                << " hangers=" << matching_hangers << " insulators=" << matching_insulators
                << " spanAttachmentCount=" << span_attachment_worlds.size()
                << " groupedPortCount=" << group->grouped_port_count << " expectedLift=" << expected_lift << "\n";
      for (const auto& attachment : span_attachment_worlds) {
        std::cerr << "[DBG] C271 attachment=(" << attachment.x << "," << attachment.y << "," << attachment.z
                  << ") hangerTargetZ=" << (attachment.z - expected_lift) << "\n";
      }
      for (const auto& part : visual->parts) {
        if (part.kind == wire::core::VisualPartKind::kFitting || part.kind == wire::core::VisualPartKind::kInsulator) {
          std::cerr << "[DBG] C271 part kind=" << static_cast<int>(part.kind) << " a=(" << part.a.x << "," << part.a.y
                    << "," << part.a.z << ") b=(" << part.b.x << "," << part.b.y << "," << part.b.z << ")\n";
        }
      }
      return false;
    }
    return true;
  }

  return false;
}

bool test_backbone_branch_lower_required_height_survives_to_grouped_placement() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  struct Snapshot {
    ObjectId span_id = wire::core::kInvalidObjectId;
    int support_group_id = -1;
    bool lower_required = false;
    bool default_lower_required = false;
    wire::core::JunctionRelationKind relation_kind = wire::core::JunctionRelationKind::kNone;
    wire::core::Vec3d support_world{};
    wire::core::Vec3d mount_world{};
    wire::core::Vec3d tip_world{};
  };
  auto collect = [&]() {
    std::vector<Snapshot> snapshots{};
    for (const auto& span_entry : state.view().spans().items()) {
      const ObjectId span_id = span_entry.id;
      const auto layout = state.view().inspect_support_layout(span_id);
      if (!layout.has_value()) {
        continue;
      }
      const auto collect_endpoint = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
        if (endpoint.owner_pole_id != center_id ||
            endpoint.relation_kind != wire::core::JunctionRelationKind::kSideBranch ||
            !endpoint.lower_required || endpoint.lowering_blocked_by_policy ||
            endpoint.support_group_id < 0) {
          return;
        }
        for (const auto& group : layout->lowered_support_groups) {
          if (group.owner_pole_id == center_id && group.support_group_id == endpoint.support_group_id) {
            Snapshot s{};
            s.span_id = span_id;
            s.support_group_id = endpoint.support_group_id;
            s.lower_required = endpoint.lower_required;
            s.default_lower_required = endpoint.default_lower_required;
            s.relation_kind = endpoint.relation_kind;
            s.support_world = endpoint.support_world;
            s.mount_world = group.mount_world;
            s.tip_world = group.tip_world;
            snapshots.push_back(s);
            return;
          }
        }
      };
      collect_endpoint(layout->start_endpoint);
      collect_endpoint(layout->end_endpoint);
    }
    std::sort(snapshots.begin(), snapshots.end(), [](const Snapshot& a, const Snapshot& b) {
      if (a.span_id != b.span_id) {
        return a.span_id < b.span_id;
      }
      return a.support_group_id < b.support_group_id;
    });
    return snapshots;
  };

  const auto before = collect();
  std::vector<GroupedRankFamilySnapshot> before_rank_families{};
  if (!collect_grouped_rank_family_snapshots(state, center_id, &before_rank_families) || before_rank_families.empty()) {
    std::cerr << "[DBG] C272 missing_rank_families_before center=" << center_id << "\n";
    return false;
  }
  for (const auto& family : before_rank_families) {
    if (!grouped_rank_family_has_consistent_owner_and_endpoints(family)) {
      std::cerr << "[DBG] C272 inconsistent_rank_before group=" << family.support_group_id
                << " rank=" << family.pair_height_rank << " endpoints=" << family.endpoints.size() << "\n";
      return false;
    }
  }
  if (before.empty()) {
    std::cerr << "[DBG] C272 no lowered branch endpoint/group snapshots\n";
    return false;
  }
  for (const auto& s : before) {
    const double expected_lift = insulator_lift_for_span_test(state, s.span_id);
    if (!almost_equal(s.support_world.z - s.tip_world.z, expected_lift, 1e-6) ||
        !almost_equal(s.mount_world.z, s.tip_world.z, 1e-6)) {
      std::cerr << "[DBG] C272 height mismatch span=" << s.span_id << " group=" << s.support_group_id
                << " supportZ=" << s.support_world.z << " mountZ=" << s.mount_world.z
                << " tipZ=" << s.tip_world.z << " expectedLift=" << expected_lift << "\n";
      return false;
    }
  }

  if (!state.SetPoleManualYawOverride(center_id, 23.0).ok) {
    return false;
  }
  const auto after = collect();
  std::vector<GroupedRankFamilySnapshot> after_rank_families{};
  if (!collect_grouped_rank_family_snapshots(state, center_id, &after_rank_families) || after_rank_families.empty()) {
    std::cerr << "[DBG] C272 missing_rank_families_after center=" << center_id << "\n";
    return false;
  }
  if (!grouped_rank_families_equal(before_rank_families, after_rank_families)) {
    std::cerr << "[DBG] C272 rank_refresh_mismatch before=" << before_rank_families.size()
              << " after=" << after_rank_families.size() << "\n";
    return false;
  }
  if (before.size() != after.size()) {
    std::cerr << "[DBG] C272 snapshot size changed before=" << before.size() << " after=" << after.size() << "\n";
    return false;
  }
  for (std::size_t i = 0; i < before.size(); ++i) {
    const Snapshot& b = before[i];
    const Snapshot& a = after[i];
    if (b.span_id != a.span_id || b.support_group_id != a.support_group_id || b.lower_required != a.lower_required ||
        b.default_lower_required != a.default_lower_required || b.relation_kind != a.relation_kind ||
        !almost_equal(b.support_world.z, a.support_world.z, 1e-6) ||
        !almost_equal(b.mount_world.z, a.mount_world.z, 1e-6) ||
        !almost_equal(b.tip_world.z, a.tip_world.z, 1e-6)) {
      std::cerr << "[DBG] C272 refresh mismatch i=" << i << " spanBefore=" << b.span_id
                << " spanAfter=" << a.span_id << " groupBefore=" << b.support_group_id
                << " groupAfter=" << a.support_group_id << " supportZBefore=" << b.support_world.z
                << " supportZAfter=" << a.support_world.z << " mountZBefore=" << b.mount_world.z
                << " mountZAfter=" << a.mount_world.z << " tipZBefore=" << b.tip_world.z
                << " tipZAfter=" << a.tip_world.z << "\n";
      return false;
    }
  }
  return true;
}

bool test_backbone_bundle_non_through_height_collapses_to_two_states() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}, {8.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(branch).ok) {
    return false;
  }

  const ObjectId corner_id = find_pole_id_by_position(state, {0.0, 12.0, 0.0});
  if (corner_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  struct Snapshot {
    double through_main_z = std::numeric_limits<double>::quiet_NaN();
    double through_main_offset_m = 0.0;
    double side_branch_z = std::numeric_limits<double>::quiet_NaN();
    double side_branch_offset_m = 0.0;
    double side_branch_mount_z = std::numeric_limits<double>::quiet_NaN();
    double side_branch_tip_z = std::numeric_limits<double>::quiet_NaN();
    double corner_z = std::numeric_limits<double>::quiet_NaN();
    double corner_offset_m = 0.0;
    double corner_mount_z = std::numeric_limits<double>::quiet_NaN();
    double corner_tip_z = std::numeric_limits<double>::quiet_NaN();
  };
  auto collect = [&]() {
    Snapshot snapshot{};
    for (const auto& span : state.view().spans().items()) {
      const auto layout = state.view().inspect_support_layout(span.id);
      if (!layout.has_value()) {
        continue;
      }
      const auto absorb = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
        if (endpoint.owner_pole_id == center_id &&
            endpoint.relation_kind == wire::core::JunctionRelationKind::kThroughMain &&
            !endpoint.lower_required) {
          if (!std::isfinite(snapshot.through_main_z)) {
            snapshot.through_main_z = endpoint.support_world.z;
            snapshot.through_main_offset_m = endpoint.branch_down_offset_m;
          } else if (!almost_equal(snapshot.through_main_z, endpoint.support_world.z, 1e-6)) {
            snapshot.through_main_z = std::numeric_limits<double>::quiet_NaN();
          }
        }
        if (endpoint.owner_pole_id == center_id &&
            endpoint.relation_kind == wire::core::JunctionRelationKind::kSideBranch &&
            endpoint.lower_required && !endpoint.lowering_blocked_by_policy) {
          if (!std::isfinite(snapshot.side_branch_z)) {
            snapshot.side_branch_z = endpoint.support_world.z;
            snapshot.side_branch_offset_m = endpoint.branch_down_offset_m;
          } else if (!almost_equal(snapshot.side_branch_z, endpoint.support_world.z, 1e-6) ||
                     !almost_equal(snapshot.side_branch_offset_m, endpoint.branch_down_offset_m, 1e-6)) {
            snapshot.side_branch_z = std::numeric_limits<double>::quiet_NaN();
          }
        }
        if (endpoint.owner_pole_id == corner_id &&
            endpoint.relation_kind == wire::core::JunctionRelationKind::kCornerContinuation &&
            endpoint.lower_required && !endpoint.lowering_blocked_by_policy) {
          if (!std::isfinite(snapshot.corner_z)) {
            snapshot.corner_z = endpoint.support_world.z;
            snapshot.corner_offset_m = endpoint.branch_down_offset_m;
          } else if (!almost_equal(snapshot.corner_z, endpoint.support_world.z, 1e-6) ||
                     !almost_equal(snapshot.corner_offset_m, endpoint.branch_down_offset_m, 1e-6)) {
            snapshot.corner_z = std::numeric_limits<double>::quiet_NaN();
          }
        }
      };
      absorb(layout->start_endpoint);
      absorb(layout->end_endpoint);
      if (const auto branch_group = lowered_support_group_for_owner(*layout, center_id); branch_group.has_value()) {
        if (!std::isfinite(snapshot.side_branch_mount_z)) {
          snapshot.side_branch_mount_z = branch_group->mount_world.z;
          snapshot.side_branch_tip_z = branch_group->tip_world.z;
        } else if (!almost_equal(snapshot.side_branch_mount_z, branch_group->mount_world.z, 1e-6) ||
                   !almost_equal(snapshot.side_branch_tip_z, branch_group->tip_world.z, 1e-6)) {
          snapshot.side_branch_mount_z = std::numeric_limits<double>::quiet_NaN();
          snapshot.side_branch_tip_z = std::numeric_limits<double>::quiet_NaN();
        }
      }
      if (const auto corner_group = lowered_support_group_for_owner(*layout, corner_id); corner_group.has_value()) {
        if (!std::isfinite(snapshot.corner_mount_z)) {
          snapshot.corner_mount_z = corner_group->mount_world.z;
          snapshot.corner_tip_z = corner_group->tip_world.z;
        } else if (!almost_equal(snapshot.corner_mount_z, corner_group->mount_world.z, 1e-6) ||
                   !almost_equal(snapshot.corner_tip_z, corner_group->tip_world.z, 1e-6)) {
          snapshot.corner_mount_z = std::numeric_limits<double>::quiet_NaN();
          snapshot.corner_tip_z = std::numeric_limits<double>::quiet_NaN();
        }
      }
    }
    return snapshot;
  };

  const Snapshot before = collect();
  if (!std::isfinite(before.through_main_z) || !std::isfinite(before.side_branch_z) || !std::isfinite(before.corner_z) ||
      !std::isfinite(before.side_branch_mount_z) || !std::isfinite(before.side_branch_tip_z) ||
      !std::isfinite(before.corner_mount_z) || !std::isfinite(before.corner_tip_z) ||
      before.side_branch_offset_m <= 1e-9 || before.corner_offset_m <= 1e-9) {
    std::cerr << "[DBG] C273 missing through/branch/corner snapshot main=" << before.through_main_z
              << " branch=" << before.side_branch_z << " corner=" << before.corner_z
              << " branchMount=" << before.side_branch_mount_z << " cornerMount=" << before.corner_mount_z
              << " branchOffset=" << before.side_branch_offset_m << " cornerOffset=" << before.corner_offset_m
              << "\n";
    return false;
  }
  if (std::abs(before.through_main_offset_m) > 1e-9 || !(before.through_main_z > before.side_branch_z)) {
    std::cerr << "[DBG] C273 main state mismatch mainZ=" << before.through_main_z
              << " branchZ=" << before.side_branch_z << " mainOffset=" << before.through_main_offset_m << "\n";
    return false;
  }
  if (!almost_equal(before.side_branch_z, before.corner_z, 1e-6) ||
      !almost_equal(before.side_branch_mount_z, before.corner_mount_z, 1e-6) ||
      !almost_equal(before.side_branch_tip_z, before.corner_tip_z, 1e-6) ||
      !almost_equal(before.side_branch_offset_m, before.corner_offset_m, 1e-6)) {
    std::cerr << "[DBG] C273 branch/corner mismatch branchZ=" << before.side_branch_z
              << " cornerZ=" << before.corner_z << " branchOffset=" << before.side_branch_offset_m
              << " cornerOffset=" << before.corner_offset_m << " branchMount=" << before.side_branch_mount_z
              << " cornerMount=" << before.corner_mount_z << " branchTip=" << before.side_branch_tip_z
              << " cornerTip=" << before.corner_tip_z << "\n";
    return false;
  }

  if (!state.SetPoleManualYawOverride(center_id, 19.0).ok) {
    return false;
  }
  const Snapshot after = collect();
  if (!almost_equal(before.through_main_z, after.through_main_z, 1e-6) ||
      !almost_equal(before.side_branch_z, after.side_branch_z, 1e-6) ||
      !almost_equal(before.corner_z, after.corner_z, 1e-6) ||
      !almost_equal(before.side_branch_mount_z, after.side_branch_mount_z, 1e-6) ||
      !almost_equal(before.side_branch_tip_z, after.side_branch_tip_z, 1e-6) ||
      !almost_equal(before.corner_mount_z, after.corner_mount_z, 1e-6) ||
      !almost_equal(before.corner_tip_z, after.corner_tip_z, 1e-6) ||
      !almost_equal(before.through_main_offset_m, after.through_main_offset_m, 1e-6) ||
      !almost_equal(before.side_branch_offset_m, after.side_branch_offset_m, 1e-6) ||
      !almost_equal(before.corner_offset_m, after.corner_offset_m, 1e-6)) {
    std::cerr << "[DBG] C273 refresh mismatch mainBefore=" << before.through_main_z
              << " mainAfter=" << after.through_main_z << " branchBefore=" << before.side_branch_z
              << " branchAfter=" << after.side_branch_z << " cornerBefore=" << before.corner_z
              << " cornerAfter=" << after.corner_z << " branchMountBefore=" << before.side_branch_mount_z
              << " branchMountAfter=" << after.side_branch_mount_z << " cornerMountBefore=" << before.corner_mount_z
              << " cornerMountAfter=" << after.corner_mount_z << "\n";
    return false;
  }
  return true;
}

bool test_backbone_crosslike_reuse_keeps_existing_straight_main_pair() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{8.06891, 4.30726, 0.0}, {14.9458, -0.818155, 0.0}, {20.266, -1.66916, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  const auto trunk_generated = state.GenerateFromBackboneSpec(trunk);
  if (!trunk_generated.ok) {
    std::cerr << "[DBG] C274 trunk generate failed error=" << trunk_generated.error << "\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {14.9458, -0.818155, 0.0}, 1e-4);
  const ObjectId existing_a_id = find_pole_id_by_position(state, {8.06891, 4.30726, 0.0}, 1e-4);
  const ObjectId existing_b_id = find_pole_id_by_position(state, {20.266, -1.66916, 0.0}, 1e-4);
  if (center_id == wire::core::kInvalidObjectId || existing_a_id == wire::core::kInvalidObjectId ||
      existing_b_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C274 existing node lookup failed center=" << center_id << " a=" << existing_a_id
              << " b=" << existing_b_id << "\n";
    return false;
  }

  wire::core::BackboneSpec crossing{};
  crossing.path.polyline = {{15.6658, 4.29683, 0.0}, {14.9458, -0.818155, 0.0}, {14.2857, -6.95258, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  crossing.path.node_specs.push_back(shared);
  crossing.interval_m = 1000.0;
  crossing.pole_type_id = type_ids.front();
  add_backbone_bundle(crossing, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(crossing);
  if (!generated.ok) {
    std::cerr << "[DBG] C274 crossing generate failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId new_a_id = find_pole_id_by_position(state, {15.6658, 4.29683, 0.0}, 1e-4);
  const ObjectId new_b_id = find_pole_id_by_position(state, {14.2857, -6.95258, 0.0}, 1e-4);
  const auto junction = state.view().inspect_junction(center_id);
  if (new_a_id == wire::core::kInvalidObjectId || new_b_id == wire::core::kInvalidObjectId || !junction.has_value()) {
    std::cerr << "[DBG] C274 new node or junction lookup failed newA=" << new_a_id << " newB=" << new_b_id
              << " hasJunction=" << (junction.has_value() ? 1 : 0) << "\n";
    return false;
  }

  auto is_existing_pair = [&](ObjectId a, ObjectId b) {
    return (a == existing_a_id && b == existing_b_id) || (a == existing_b_id && b == existing_a_id);
  };
  auto relation_kind_for = [&](ObjectId neighbor_id) {
    for (const auto& relation : junction->local_relations) {
      if (relation.neighbor_node_id == neighbor_id) {
        return relation.kind;
      }
    }
    return wire::core::JunctionRelationKind::kNone;
  };

  int generated_hv_count = 0;
  int generated_cross_under_count = 0;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto span_view = state.view().inspect_span(span_id);
    const auto* span = state.view().edit_state().spans.find(span_id);
    const auto* bundle = (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
    if (!span_view.has_value() || bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    ++generated_hv_count;
    if (span_view->lowering_kind == wire::core::BackboneLoweringKind::kCrossUnderpass &&
        !span_view->same_level_feasible) {
      ++generated_cross_under_count;
    }
  }

  const bool ok = junction->through_pair_accepted &&
                  is_existing_pair(junction->through_pair_neighbor_a_id, junction->through_pair_neighbor_b_id) &&
                  relation_kind_for(existing_a_id) == wire::core::JunctionRelationKind::kThroughMain &&
                  relation_kind_for(existing_b_id) == wire::core::JunctionRelationKind::kThroughMain &&
                  relation_kind_for(new_a_id) == wire::core::JunctionRelationKind::kCrossUnderpass &&
                  relation_kind_for(new_b_id) == wire::core::JunctionRelationKind::kCrossUnderpass &&
                  generated_hv_count == 6 && generated_cross_under_count == generated_hv_count;
  if (!ok) {
    std::cerr << "[DBG] C274 accepted=" << (junction->through_pair_accepted ? 1 : 0)
              << " pair=" << junction->through_pair_neighbor_a_id << "/" << junction->through_pair_neighbor_b_id
              << " existingAkind=" << static_cast<int>(relation_kind_for(existing_a_id))
              << " existingBkind=" << static_cast<int>(relation_kind_for(existing_b_id))
              << " newAkind=" << static_cast<int>(relation_kind_for(new_a_id))
              << " newBkind=" << static_cast<int>(relation_kind_for(new_b_id))
              << " generatedHv=" << generated_hv_count << " generatedCross=" << generated_cross_under_count << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C274 rel neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " inRoute=" << (relation.in_route ? 1 : 0)
                << " inPair=" << (relation.in_through_pair ? 1 : 0)
                << " same=" << (relation.same_level_feasible ? 1 : 0)
                << " defaultLower=" << (relation.default_lower_required ? 1 : 0) << "\n";
    }
    for (ObjectId span_id : generated.value.generated_span_ids) {
      const auto span_view = state.view().inspect_span(span_id);
      const auto* span = state.view().edit_state().spans.find(span_id);
      const auto* bundle = (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
      if (!span_view.has_value() || bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
        continue;
      }
      std::cerr << "[DBG] C274 span=" << span_id << " flow=" << static_cast<int>(span_view->flow_kind)
                << " lowering=" << static_cast<int>(span_view->lowering_kind)
                << " same=" << (span_view->same_level_feasible ? 1 : 0)
                << " defaultLower=" << (span_view->default_lower_required ? 1 : 0) << "\n";
    }
  }
  return ok;
}

bool test_backbone_grouped_support_membership_is_visible_on_all_bundle_lanes() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.size() < 3) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  int spans_with_group = 0;
  int expected_group_id = -1;
  wire::core::Vec3d expected_mount{};
  wire::core::Vec3d expected_tip{};
  bool expected_anchor_set = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      return false;
    }
    const auto group = lowered_support_group_for_owner(*layout_view, center_id);
    if (!group.has_value()) {
      std::cerr << "[DBG] C255 missing grouped support span=" << span_id << "\n";
      return false;
    }
    ++spans_with_group;
    if (expected_group_id < 0) {
      expected_group_id = group->support_group_id;
      expected_mount = group->mount_world;
      expected_tip = group->tip_world;
      expected_anchor_set = true;
    } else if (group->support_group_id != expected_group_id ||
               !almost_equal(group->mount_world.x, expected_mount.x, 1e-6) ||
               !almost_equal(group->mount_world.y, expected_mount.y, 1e-6) ||
               !almost_equal(group->mount_world.z, expected_mount.z, 1e-6) ||
               !almost_equal(group->tip_world.x, expected_tip.x, 1e-6) ||
               !almost_equal(group->tip_world.y, expected_tip.y, 1e-6) ||
               !almost_equal(group->tip_world.z, expected_tip.z, 1e-6)) {
      std::cerr << "[DBG] C255 mismatched support identity span=" << span_id << " group=" << group->support_group_id
                << " expected=" << expected_group_id << "\n";
      return false;
    }
    if (group->grouped_port_count < 3) {
      std::cerr << "[DBG] C255 grouped_port_count=" << group->grouped_port_count << " span=" << span_id << "\n";
      return false;
    }
  }
  return expected_anchor_set && spans_with_group == static_cast<int>(generated.value.generated_span_ids.size());
}

bool test_backbone_unrelated_generation_does_not_downgrade_existing_lowered_bundle_semantics() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.size() < 3) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  auto verify_branch_bundle = [&]() {
    for (ObjectId span_id : branch_generated.value.generated_span_ids) {
      const auto layout_view = state.view().inspect_support_layout(span_id);
      if (!layout_view.has_value()) {
        return false;
      }
      if (layout_view->continuity_class != wire::core::ContinuityCategoryClass::kBundleLike ||
          layout_view->order_decision_policy != wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous) {
        std::cerr << "[DBG] C256 downgraded span=" << span_id << " class="
                  << static_cast<int>(layout_view->continuity_class) << " orderPolicy="
                  << static_cast<int>(layout_view->order_decision_policy) << "\n";
        return false;
      }
      const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
      const auto group = lowered_support_group_for_owner(*layout_view, center_id);
      if (!endpoint.has_value() || !endpoint_has_authoritative_lowering(*endpoint) || !group.has_value() ||
          group->support_group_id != endpoint->support_group_id) {
        std::cerr << "[DBG] C256 missing grouped support span=" << span_id << "\n";
        return false;
      }
    }
    return true;
  };

  if (!verify_branch_bundle()) {
    return false;
  }

  wire::core::BackboneSpec unrelated{};
  unrelated.path.polyline = {{80.0, 0.0, 0.0}, {92.0, 0.0, 0.0}};
  unrelated.interval_m = 1000.0;
  unrelated.pole_type_id = type_ids.front();
  add_backbone_bundle(unrelated, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(unrelated).ok) {
    return false;
  }

  if (!state.SetPoleManualYawOverride(center_id, 12.0).ok) {
    return false;
  }

  return verify_branch_bundle();
}

bool test_backbone_corner_support_uses_connected_line_basis() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() < 2) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId corner_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (corner_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C253 missing corner pole\n";
    return false;
  }

  std::vector<wire::core::LoweredSupportGroupInspectionView> placements{};
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    for (const auto& placement : layout_view->lowered_support_groups) {
      if (placement.owner_pole_id == corner_id) {
        placements.push_back(placement);
      }
    }
  }
  if (placements.empty()) {
    for (const auto& span_entry : state.view().spans().items()) {
      const auto support_layout = state.view().inspect_support_layout(span_entry.id);
      if (!support_layout.has_value()) {
        continue;
      }
      const auto endpoint_matches = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
        return endpoint.owner_pole_id == corner_id && endpoint.lower_required &&
               (endpoint.support_orientation_basis ==
                    wire::core::SupportOrientationBasisKind::kBisectorForward ||
                endpoint.support_orientation_basis ==
                    wire::core::SupportOrientationBasisKind::kBisectorReverse);
      };
      if (endpoint_matches(support_layout->start_endpoint) || endpoint_matches(support_layout->end_endpoint)) {
        return true;
      }
    }
    std::cerr << "[DBG] C253 no corner visual placement and no non-radial lowered corner decision\n";
    return false;
  }
  const int group_id = placements.front().support_group_id;
  const double side_sign = placements.front().chosen_side_sign;
  for (const auto& placement : placements) {
    if (placement.support_group_id != group_id ||
        placement.side_assignment_rule != wire::core::SideAssignmentRuleKind::kBisector ||
        placement.support_orientation_rule != wire::core::SupportOrientationRuleKind::kBisector ||
        (placement.support_orientation_basis != wire::core::SupportOrientationBasisKind::kBisectorForward &&
         placement.support_orientation_basis != wire::core::SupportOrientationBasisKind::kBisectorReverse) ||
        placement.support_orientation_rule == wire::core::SupportOrientationRuleKind::kRadial ||
        std::abs(placement.chosen_side_sign - side_sign) > 1e-9 ||
        !almost_equal(placement.mount_world.x, placements.front().mount_world.x, 1e-6) ||
        !almost_equal(placement.mount_world.y, placements.front().mount_world.y, 1e-6) ||
        !almost_equal(placement.tip_world.x, placements.front().tip_world.x, 1e-6) ||
        !almost_equal(placement.tip_world.y, placements.front().tip_world.y, 1e-6)) {
      std::cerr << "[DBG] C253 corner group/sign mismatch group=" << placement.support_group_id
                << " refGroup=" << group_id << " orient=" << static_cast<int>(placement.support_orientation_rule)
                << " side=" << placement.chosen_side_sign << " refSide=" << side_sign << "\n";
      return false;
    }
  }
  return true;
}

bool test_backbone_lowered_bundle_midspan_support_uses_pair_based_orientation() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}, {12.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() < 9) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId downstream_corner_id = find_pole_id_by_position(state, {0.0, 12.0, 0.0});
  if (downstream_corner_id == wire::core::kInvalidObjectId) {
    return false;
  }

  struct Snapshot {
    ObjectId span_id = wire::core::kInvalidObjectId;
    int support_group_id = -1;
    ObjectId pair_peer_low = wire::core::kInvalidObjectId;
    ObjectId pair_peer_high = wire::core::kInvalidObjectId;
    wire::core::SideAssignmentRuleKind side_rule = wire::core::SideAssignmentRuleKind::kPoleLocal;
    wire::core::SupportOrientationRuleKind orientation_rule = wire::core::SupportOrientationRuleKind::kRadial;
    wire::core::SupportOrientationBasisKind orientation_basis = wire::core::SupportOrientationBasisKind::kRadial;
    double side_sign = 0.0;
    wire::core::Vec3d side_axis{};
  };
  auto collect = [&]() {
    std::vector<Snapshot> snapshots{};
    int expected_group_id = -1;
    ObjectId expected_pair_low = wire::core::kInvalidObjectId;
    ObjectId expected_pair_high = wire::core::kInvalidObjectId;
    double expected_side_sign = 0.0;
    wire::core::Vec3d expected_axis{};
    for (ObjectId span_id : generated.value.generated_span_ids) {
      const auto layout = state.view().inspect_support_layout(span_id);
      if (!layout.has_value()) {
        continue;
      }
      const auto authoritative_group = [&]() -> const wire::core::LoweredSupportGroupInspectionView* {
        for (const auto& group : layout->lowered_support_groups) {
          if (group.owner_pole_id == downstream_corner_id) {
            return &group;
          }
        }
        return nullptr;
      }();
      const auto inspect_endpoint = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
        if (endpoint.owner_pole_id != downstream_corner_id || !endpoint.lower_required ||
            endpoint.support_group_id < 0) {
          return true;
        }
        if (authoritative_group == nullptr || authoritative_group->support_group_id != endpoint.support_group_id) {
          return false;
        }
        const bool pair_based =
            endpoint.side_assignment_rule == wire::core::SideAssignmentRuleKind::kBisector &&
            endpoint.support_orientation_rule == wire::core::SupportOrientationRuleKind::kBisector &&
            (endpoint.support_orientation_basis ==
                 wire::core::SupportOrientationBasisKind::kBisectorForward ||
             endpoint.support_orientation_basis ==
                 wire::core::SupportOrientationBasisKind::kBisectorReverse);
        if (!pair_based || endpoint.support_orientation_rule == wire::core::SupportOrientationRuleKind::kRadial ||
            authoritative_group->pair_peer_low == wire::core::kInvalidObjectId ||
            authoritative_group->pair_peer_high == wire::core::kInvalidObjectId) {
          return false;
        }
        if (expected_group_id < 0) {
          expected_group_id = endpoint.support_group_id;
          expected_pair_low = authoritative_group->pair_peer_low;
          expected_pair_high = authoritative_group->pair_peer_high;
          expected_side_sign = endpoint.chosen_side_sign;
          expected_axis = endpoint.side_axis;
        }
        if (endpoint.support_group_id != expected_group_id ||
            authoritative_group->pair_peer_low != expected_pair_low ||
            authoritative_group->pair_peer_high != expected_pair_high ||
            std::abs(endpoint.chosen_side_sign - expected_side_sign) > 1e-9 ||
            !almost_equal(endpoint.side_axis.x, expected_axis.x, 1e-6) ||
            !almost_equal(endpoint.side_axis.y, expected_axis.y, 1e-6)) {
          return false;
        }
        Snapshot snapshot{};
        snapshot.span_id = span_id;
        snapshot.support_group_id = endpoint.support_group_id;
        snapshot.pair_peer_low = authoritative_group->pair_peer_low;
        snapshot.pair_peer_high = authoritative_group->pair_peer_high;
        snapshot.side_rule = endpoint.side_assignment_rule;
        snapshot.orientation_rule = endpoint.support_orientation_rule;
        snapshot.orientation_basis = endpoint.support_orientation_basis;
        snapshot.side_sign = endpoint.chosen_side_sign;
        snapshot.side_axis = endpoint.side_axis;
        snapshots.push_back(snapshot);
        return true;
      };
      if (!inspect_endpoint(layout->start_endpoint) || !inspect_endpoint(layout->end_endpoint)) {
        return std::optional<std::vector<Snapshot>>{};
      }
    }
    std::sort(snapshots.begin(), snapshots.end(), [](const Snapshot& a, const Snapshot& b) {
      if (a.span_id != b.span_id) {
        return a.span_id < b.span_id;
      }
      return a.support_group_id < b.support_group_id;
    });
    return std::optional<std::vector<Snapshot>>{snapshots};
  };

  const auto before = collect();
  if (!before.has_value()) {
    return false;
  }
  if (before->size() < 2) {
    return false;
  }
  if (!state.SetPoleManualYawOverride(downstream_corner_id, 17.0).ok) {
    return false;
  }
  const auto after = collect();
  if (!after.has_value()) {
    return false;
  }
  if (before->size() != after->size()) {
    return false;
  }
  for (std::size_t i = 0; i < before->size(); ++i) {
    const Snapshot& b = (*before)[i];
    const Snapshot& a = (*after)[i];
    if (b.span_id != a.span_id || b.support_group_id != a.support_group_id ||
        b.pair_peer_low != a.pair_peer_low || b.pair_peer_high != a.pair_peer_high ||
        b.side_rule != a.side_rule || b.orientation_rule != a.orientation_rule ||
        b.orientation_basis != a.orientation_basis || !almost_equal(b.side_sign, a.side_sign, 1e-9) ||
        !almost_equal(b.side_axis.x, a.side_axis.x, 1e-6) ||
        !almost_equal(b.side_axis.y, a.side_axis.y, 1e-6)) {
      return false;
    }
  }
  return true;
}

bool test_backbone_pair_based_orientation_allows_opposite_signs_per_pole() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}, {12.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() < 9) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout = state.view().inspect_support_layout(span_id);
    if (!layout.has_value()) {
      continue;
    }
    const auto& start = layout->start_endpoint;
    const auto& end = layout->end_endpoint;
    const bool both_grouped = start.lower_required && end.lower_required &&
                              start.support_group_id >= 0 && end.support_group_id >= 0;
    if (!both_grouped) {
      continue;
    }
    const bool pair_based =
        start.side_assignment_rule == wire::core::SideAssignmentRuleKind::kBisector &&
        end.side_assignment_rule == wire::core::SideAssignmentRuleKind::kBisector &&
        start.support_orientation_rule == wire::core::SupportOrientationRuleKind::kBisector &&
        end.support_orientation_rule == wire::core::SupportOrientationRuleKind::kBisector &&
        start.support_orientation_basis == wire::core::SupportOrientationBasisKind::kBisectorForward &&
        end.support_orientation_basis == wire::core::SupportOrientationBasisKind::kBisectorForward;
    if (!pair_based) {
      continue;
    }
    if (!almost_equal(start.side_axis.x, end.side_axis.x, 1e-6) ||
        !almost_equal(start.side_axis.y, end.side_axis.y, 1e-6)) {
      return false;
    }
    if (std::abs(start.chosen_side_sign) <= 1e-9 || std::abs(end.chosen_side_sign) <= 1e-9) {
      return false;
    }
    return start.chosen_side_sign * end.chosen_side_sign < 0.0;
  }

  return false;
}

bool test_backbone_point_like_orientation_rule_non_regression() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.size() != 1) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }
  const auto layout_view = state.view().inspect_support_layout(generated.value.generated_span_ids.front());
  if (!layout_view.has_value()) {
    return false;
  }
  const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
  const wire::core::Vec3d trunk_dir = normalize_xy_safe({12.0, 0.0, 0.0});
  const wire::core::Vec3d branch_dir = normalize_xy_safe({0.0, 12.0, 0.0});
  const wire::core::Vec3d expected_axis = normalize_xy_safe(trunk_dir + branch_dir);
  const double axis_alignment =
      endpoint.has_value() ? std::abs(dot_xy(normalize_xy_safe(endpoint->side_axis), expected_axis)) : -1.0;
  return endpoint.has_value() &&
         endpoint->continuity_class == wire::core::ContinuityCategoryClass::kPointLike &&
         endpoint->support_orientation_rule == wire::core::SupportOrientationRuleKind::kBisector &&
         endpoint->side_assignment_rule == wire::core::SideAssignmentRuleKind::kBisector &&
         endpoint->used_junction_pair_side_assignment && endpoint->has_side_axis &&
         axis_alignment >= 0.97 &&
         (endpoint->support_orientation_basis ==
              wire::core::SupportOrientationBasisKind::kBisectorForward ||
          endpoint->support_orientation_basis ==
              wire::core::SupportOrientationBasisKind::kBisectorReverse) &&
         layout_view->lowered_support_groups.empty();
}

bool test_backbone_same_level_cross_main_uses_through_pair_authority() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(cross).ok) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }
  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value() || !junction->through_pair_accepted) {
    return false;
  }
  int through_main_count = 0;
  int cross_like_count = 0;
  for (const auto& relation : junction->local_relations) {
    if (relation.kind == wire::core::JunctionRelationKind::kThroughMain) {
      ++through_main_count;
    } else if (relation.kind == wire::core::JunctionRelationKind::kCrossUnderpass) {
      ++cross_like_count;
    }
  }
  return through_main_count == 2 && cross_like_count == 2;
}

bool test_backbone_same_level_cross_underpass_uses_cross_pair_authority() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const PoleTypeId pole_type_id = type_ids.front();

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = pole_type_id;
  add_backbone_bundle(main_req, wire::core::BundleKind::kLowVoltage);
  const auto main_generated = state.GenerateFromBackboneSpec(main_req);
  if (!main_generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross_req{};
  cross_req.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross_req.path.node_specs.push_back(shared);
  cross_req.interval_m = 1000.0;
  cross_req.pole_type_id = pole_type_id;
  add_backbone_bundle(cross_req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross_req);
  if (!generated.ok) {
    return false;
  }
  const std::unordered_set<ObjectId> generated_span_set(generated.value.generated_span_ids.begin(),
                                                        generated.value.generated_span_ids.end());

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  std::optional<wire::core::Vec3d> through_axis{};
  for (ObjectId span_id : main_generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->lower_required ||
        endpoint->relation_kind != wire::core::JunctionRelationKind::kThroughMain ||
        endpoint->support_orientation_rule != wire::core::SupportOrientationRuleKind::kThroughPairNormal) {
      continue;
    }
    const wire::core::Vec3d axis = endpoint->has_signed_support_axis ? endpoint->signed_support_axis : endpoint->side_axis;
    const wire::core::Vec3d normalized = helpers::normalize_xy_safe(axis);
    if (std::abs(normalized.x) > 1e-9 || std::abs(normalized.y) > 1e-9) {
      through_axis = normalized;
      break;
    }
  }
  if (!through_axis.has_value()) {
    const auto pole_view = state.view().inspect_pole(center_id);
    if (pole_view.has_value() && pole_view->has_support_axis) {
      const wire::core::Vec3d normalized = helpers::normalize_xy_safe(pole_view->support_axis_dir);
      if (std::abs(normalized.x) > 1e-9 || std::abs(normalized.y) > 1e-9) {
        through_axis = normalized;
      }
    }
    if (!through_axis.has_value()) {
      return false;
    }
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    std::cerr << "[DBG] C340 inspect_junction_failed center=" << center_id << "\n";
    return false;
  }

  std::vector<ObjectId> expected_cross_peers{};
  for (const auto& relation : junction->local_relations) {
    if (relation.kind != wire::core::JunctionRelationKind::kCrossUnderpass ||
        !relation.same_level_feasible || relation.default_lower_required ||
        relation.continuity_class != wire::core::ContinuityCategoryClass::kPointLike) {
      continue;
    }
    expected_cross_peers.push_back(relation.neighbor_node_id);
  }
  std::sort(expected_cross_peers.begin(), expected_cross_peers.end());
  expected_cross_peers.erase(std::unique(expected_cross_peers.begin(), expected_cross_peers.end()),
                             expected_cross_peers.end());
  if (expected_cross_peers.size() != 2) {
    std::cerr << "[DBG] C340 scene_mismatch center=" << center_id << " crossPeerCount=" << expected_cross_peers.size()
              << " throughPair=(" << junction->through_pair_neighbor_a_id << "," << junction->through_pair_neighbor_b_id
              << ")\n";
    return false;
  }

  const auto expected_cross_pair =
      std::pair<ObjectId, ObjectId>{std::min(expected_cross_peers[0], expected_cross_peers[1]),
                                    std::max(expected_cross_peers[0], expected_cross_peers[1])};
  bool saw_same_level_cross_underpass = false;

  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->continuity_class != wire::core::ContinuityCategoryClass::kPointLike ||
        endpoint->lower_required || endpoint->in_through_pair ||
        endpoint->relation_kind != wire::core::JunctionRelationKind::kCrossUnderpass) {
      continue;
    }
    const auto endpoint_pair = std::pair<ObjectId, ObjectId>{endpoint->support_authority.pair.pair_peer_low,
                                                             endpoint->support_authority.pair.pair_peer_high};
    if (!endpoint->same_level_feasible || endpoint_pair != expected_cross_pair) {
      continue;
    }
    saw_same_level_cross_underpass = true;
    const wire::core::Vec3d actual_axis = helpers::normalize_xy_safe(
        endpoint->has_signed_support_axis ? endpoint->signed_support_axis : endpoint->side_axis);
    const double align = std::abs(helpers::dot_xy(*through_axis, actual_axis));
    const bool ok =
        endpoint->support_orientation_rule == wire::core::SupportOrientationRuleKind::kThroughPairNormal &&
        endpoint->side_assignment_rule == wire::core::SideAssignmentRuleKind::kThroughPairNormal &&
        endpoint->used_junction_pair_side_assignment && endpoint->has_side_axis &&
        std::abs(endpoint->chosen_side_sign) > 1e-9 && align <= 0.3 &&
        (endpoint->support_orientation_basis ==
             wire::core::SupportOrientationBasisKind::kPairNormalPositive ||
         endpoint->support_orientation_basis ==
             wire::core::SupportOrientationBasisKind::kPairNormalNegative) &&
        layout_view->lowered_support_groups.empty();
    return ok;
  }

  if (!saw_same_level_cross_underpass) {
    std::cerr << "[DBG] C340 scene_mismatch no_same_level_cross_underpass center=" << center_id << " expectedPair=("
              << expected_cross_pair.first << "," << expected_cross_pair.second << ")\n";
  }
  return false;
}

bool test_backbone_cross_visual_keeps_distinct_pair_axes() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = pole_type_id;
  add_backbone_bundle(main_req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(main_req, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(main_req, wire::core::BundleKind::kOptical);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross_req{};
  cross_req.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross_req.path.node_specs.push_back(shared);
  cross_req.interval_m = 1000.0;
  cross_req.pole_type_id = pole_type_id;
  add_backbone_bundle(cross_req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(cross_req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(cross_req, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(cross_req, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(cross_req);
  if (!generated.ok) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  std::optional<wire::core::Vec3d> through_axis{};
  std::optional<wire::core::Vec3d> cross_axis{};
  for (const auto& span_entry : state.view().spans().items()) {
    const ObjectId span_id = span_entry.id;
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->lower_required ||
        endpoint->continuity_class != wire::core::ContinuityCategoryClass::kPointLike ||
        !endpoint->has_side_axis || std::abs(endpoint->chosen_side_sign) <= 1e-9 ||
        !layout_view->lowered_support_groups.empty()) {
      if (endpoint.has_value() && endpoint->relation_kind == wire::core::JunctionRelationKind::kThroughMain &&
          endpoint->continuity_class == wire::core::ContinuityCategoryClass::kPointLike &&
          !endpoint->lower_required) {
        std::cerr << "[DBG] C341 skip_through span=" << span_id << " hasAxis=" << (endpoint->has_side_axis ? 1 : 0)
                  << " sign=" << endpoint->chosen_side_sign
                  << " basis=" << static_cast<int>(endpoint->support_orientation_basis)
                  << " sideRule=" << static_cast<int>(endpoint->side_assignment_rule)
                  << " orientRule=" << static_cast<int>(endpoint->support_orientation_rule)
                  << " groups=" << layout_view->lowered_support_groups.size() << "\n";
      }
      continue;
    }
    const auto arm = support_arm_part_for_owner(state, span_id, center_id);
    if (!arm.has_value()) {
      if (const auto* visual = state.view().find_span_visual_cache(span_id); visual != nullptr) {
        std::cerr << "[DBG] C341 visual_parts span=" << span_id << " count=" << visual->parts.size() << "\n";
        for (std::size_t part_index = 0; part_index < visual->parts.size(); ++part_index) {
          const auto& part = visual->parts[part_index];
          std::cerr << "[DBG] C341 part[" << part_index << "] kind=" << static_cast<int>(part.kind)
                    << " a=(" << part.a.x << "," << part.a.y << "," << part.a.z << ")"
                    << " b=(" << part.b.x << "," << part.b.y << "," << part.b.z << ")\n";
        }
      }
      std::cerr << "[DBG] C341 missing_arm span=" << span_id << " relation=" << static_cast<int>(endpoint->relation_kind)
                << " sign=" << endpoint->chosen_side_sign << " side=(" << endpoint->side_axis.x << ","
                << endpoint->side_axis.y << ")\n";
      continue;
    }
    const wire::core::Vec3d actual_axis = helpers::normalize_xy_safe(arm->b - arm->a);
    if (endpoint->relation_kind == wire::core::JunctionRelationKind::kThroughMain) {
      through_axis = actual_axis;
    } else if (endpoint->relation_kind == wire::core::JunctionRelationKind::kCrossUnderpass) {
      cross_axis = actual_axis;
    }
  }

  if (!through_axis.has_value() || !cross_axis.has_value()) {
    std::cerr << "[DBG] C341 missing_axis through=" << through_axis.has_value() << " cross=" << cross_axis.has_value()
              << " center=" << center_id << "\n";
    return false;
  }
  const double align = std::abs(helpers::dot_xy(*through_axis, *cross_axis));
  if (align > 0.3) {
    std::cerr << "[DBG] C341 bad_align through=(" << through_axis->x << "," << through_axis->y << ") cross=("
              << cross_axis->x << "," << cross_axis->y << ") align=" << align << "\n";
  }
  return align <= 0.3;
}

bool test_backbone_capture_like_cross_center_uses_pair_authority() {
  CoreState state;
  ObjectId center_id = wire::core::kInvalidObjectId;
  std::unordered_map<ObjectId, ObjectId> remapped_ids{};
  std::vector<ObjectId> generated_span_ids{};
  if (!build_minimal_latest_pair_cross_capture_scene(state, &center_id, &remapped_ids, &generated_span_ids)) {
    return false;
  }
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C342 crosslike_center_missing\n";
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    std::cerr << "[DBG] C342 inspect_junction_failed center=" << center_id << "\n";
    return false;
  }

  std::map<ObjectId, ObjectId> expected_companion{
      {remapped_ids.at(434), remapped_ids.at(446)},
      {remapped_ids.at(446), remapped_ids.at(434)},
      {remapped_ids.at(477), remapped_ids.at(489)},
      {remapped_ids.at(489), remapped_ids.at(477)},
  };
  auto expected_pair_axis = [&](ObjectId peer_id) {
    const ObjectId companion_id = expected_companion.at(peer_id);
    const wire::core::Vec3d peer = state.view().inspect_pole(peer_id)->position;
    const wire::core::Vec3d companion = state.view().inspect_pole(companion_id)->position;
    wire::core::Vec3d pair_forward = helpers::normalize_xy_safe(companion - peer);
    return helpers::normalize_xy_safe(wire::core::ComputeLateralAxis(pair_forward));
  };

  std::set<ObjectId> seen_peers{};
  std::map<ObjectId, double> seen_signs{};
  for (const auto& span_entry : state.view().spans().items()) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_entry.id);
    const wire::core::Bundle* bundle =
        (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
    if (span == nullptr || bundle == nullptr ||
        (span->endpoint_node_a_id != center_id && span->endpoint_node_b_id != center_id)) {
      continue;
    }
    const ObjectId peer_id = (span->endpoint_node_a_id == center_id) ? span->endpoint_node_b_id : span->endpoint_node_a_id;
    if (!expected_companion.contains(peer_id)) {
      continue;
    }
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->lower_required || !endpoint->same_level_feasible ||
        endpoint->continuity_class != wire::core::ContinuityCategoryClass::kPointLike) {
      continue;
    }
    if (endpoint->support_orientation_rule == wire::core::SupportOrientationRuleKind::kRadial ||
        endpoint->side_assignment_rule == wire::core::SideAssignmentRuleKind::kPoleLocal ||
        endpoint->support_orientation_rule != wire::core::SupportOrientationRuleKind::kThroughPairNormal ||
        endpoint->side_assignment_rule != wire::core::SideAssignmentRuleKind::kThroughPairNormal ||
        !endpoint->used_junction_pair_side_assignment || !endpoint->has_side_axis) {
      std::cerr << "[DBG] C342 span=" << span_entry.id << " peer=" << peer_id
                << " relation=" << static_cast<int>(endpoint->relation_kind) << " sideRule="
                << static_cast<int>(endpoint->side_assignment_rule) << " orientRule="
                << static_cast<int>(endpoint->support_orientation_rule) << " basis="
                << static_cast<int>(endpoint->support_orientation_basis) << " sign="
                << endpoint->chosen_side_sign << "\n";
      return false;
    }
    const wire::core::Vec3d expected_axis = expected_pair_axis(peer_id);
    const wire::core::Vec3d actual_axis = helpers::normalize_xy_safe(endpoint->side_axis);
    const double align = std::abs(helpers::dot_xy(expected_axis, actual_axis));
    if (align < 0.97) {
      std::cerr << "[DBG] C342 bad_axis peer=" << peer_id << " actual=(" << actual_axis.x << "," << actual_axis.y
                << ") expected=(" << expected_axis.x << "," << expected_axis.y << ") align=" << align << "\n";
      return false;
    }
    seen_peers.insert(peer_id);
    seen_signs[peer_id] = endpoint->chosen_side_sign;
  }

  const bool all_seen = seen_peers ==
                        std::set<ObjectId>{remapped_ids.at(434), remapped_ids.at(446), remapped_ids.at(477),
                                           remapped_ids.at(489)};
  const bool existing_opposite = seen_signs.contains(remapped_ids.at(434)) &&
                                 seen_signs.contains(remapped_ids.at(446)) &&
                                 seen_signs[remapped_ids.at(434)] * seen_signs[remapped_ids.at(446)] < 0.0;
  const bool generated_opposite = seen_signs.contains(remapped_ids.at(477)) &&
                                  seen_signs.contains(remapped_ids.at(489)) &&
                                  seen_signs[remapped_ids.at(477)] * seen_signs[remapped_ids.at(489)] < 0.0;
  if (!(all_seen && existing_opposite && generated_opposite)) {
    std::cerr << "[DBG] C342 seen=" << seen_peers.size() << " existingOpp=" << existing_opposite
              << " generatedOpp=" << generated_opposite << "\n";
  }
  return all_seen && existing_opposite && generated_opposite;
}

bool test_backbone_capture_like_cross_visual_keeps_route_and_existing_pair_axes_distinct() {
  CoreState state;
  ObjectId center_id = wire::core::kInvalidObjectId;
  std::unordered_map<ObjectId, ObjectId> remapped_ids{};
  std::vector<ObjectId> generated_span_ids{};
  if (!build_minimal_latest_pair_cross_capture_scene(state, &center_id, &remapped_ids, &generated_span_ids)) {
    return false;
  }
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C343 crosslike_center_missing\n";
    return false;
  }

  std::map<ObjectId, ObjectId> expected_companion{
      {remapped_ids.at(434), remapped_ids.at(446)},
      {remapped_ids.at(446), remapped_ids.at(434)},
      {remapped_ids.at(477), remapped_ids.at(489)},
      {remapped_ids.at(489), remapped_ids.at(477)},
  };
  auto expected_pair_axis = [&](ObjectId peer_id) {
    const ObjectId companion_id = expected_companion.at(peer_id);
    const wire::core::Vec3d peer = state.view().inspect_pole(peer_id)->position;
    const wire::core::Vec3d companion = state.view().inspect_pole(companion_id)->position;
    wire::core::Vec3d pair_forward = helpers::normalize_xy_safe(companion - peer);
    return helpers::normalize_xy_safe(wire::core::ComputeLateralAxis(pair_forward));
  };

  std::set<ObjectId> seen_peers{};
  for (const auto& span_entry : state.view().spans().items()) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_entry.id);
    const wire::core::Bundle* bundle =
        (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
    if (span == nullptr || bundle == nullptr ||
        (span->endpoint_node_a_id != center_id && span->endpoint_node_b_id != center_id)) {
      continue;
    }
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->lower_required || !endpoint->same_level_feasible ||
        endpoint->continuity_class != wire::core::ContinuityCategoryClass::kPointLike ||
        endpoint->support_orientation_rule == wire::core::SupportOrientationRuleKind::kRadial ||
        !endpoint->has_side_axis || std::abs(endpoint->chosen_side_sign) <= 1e-9) {
      continue;
    }
    const ObjectId peer_id = (span->endpoint_node_a_id == center_id) ? span->endpoint_node_b_id : span->endpoint_node_a_id;
    if (!expected_companion.contains(peer_id)) {
      continue;
    }
    const auto arm = support_arm_part_for_owner(state, span_entry.id, center_id);
    if (!arm.has_value()) {
      std::cerr << "[DBG] C343 missing_arm span=" << span_entry.id << " center=" << center_id << " peer=" << peer_id
                << "\n";
      return false;
    }
    const wire::core::Vec3d actual_axis = helpers::normalize_xy_safe(arm->b - arm->a);
    const wire::core::Vec3d expected_axis = expected_pair_axis(peer_id);
    const double align = std::abs(helpers::dot_xy(actual_axis, expected_axis));
    if (align < 0.97) {
      std::cerr << "[DBG] C343 bad_axis peer=" << peer_id << " actual=(" << actual_axis.x << "," << actual_axis.y
                << ") expected=(" << expected_axis.x << "," << expected_axis.y << ") align=" << align << "\n";
      return false;
    }
    seen_peers.insert(peer_id);
  }
  if (seen_peers !=
      std::set<ObjectId>{remapped_ids.at(434), remapped_ids.at(446), remapped_ids.at(477), remapped_ids.at(489)}) {
    std::cerr << "[DBG] C343 seen=" << seen_peers.size() << " center=" << center_id << "\n";
  }
  return seen_peers ==
         std::set<ObjectId>{remapped_ids.at(434), remapped_ids.at(446), remapped_ids.at(477), remapped_ids.at(489)};
}

bool build_minimal_latest_support_orientation_outer_scene(CoreState& state, ObjectId* out_center_id,
                                                          std::vector<ObjectId>* out_generated_span_ids) {
  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = pole_type.id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] minimal_support_outer missing_communication_pole_type\n";
    return false;
  }

  auto add_all_templates = [](wire::core::BackboneSpec& req) {
    add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
    add_backbone_bundle(req, wire::core::BundleKind::kOptical);
  };

  constexpr wire::core::Vec3d kPrev{-12.3574, -6.65056, 0.0};
  constexpr wire::core::Vec3d kCenter{-10.3237, -13.3838, 0.0};
  constexpr wire::core::Vec3d kNext{-5.25927, -18.1406, 0.0};
  constexpr wire::core::Vec3d kBranch{-17.4937, -15.6555, 0.0};
  constexpr wire::core::Vec3d kNewMain{-6.55614, -8.58746, 0.0};

  wire::core::BackboneSpec existing{}; 
  existing.path.polyline = {kPrev, kCenter, kNext};
  existing.interval_m = 1000.0;
  existing.pole_type_id = communication_pole_type_id;
  add_all_templates(existing);
  if (!state.GenerateFromBackboneSpec(existing).ok) {
    std::cerr << "[DBG] minimal_support_outer existing_generate_failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, kCenter, 1e-4);
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] minimal_support_outer center_missing\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {kCenter, kBranch};
  branch.interval_m = 1000.0;
  branch.pole_type_id = communication_pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec shared_branch{};
  shared_branch.point_index = 0;
  shared_branch.support_kind = wire::core::SupportKind::kPole;
  shared_branch.node_id = center_id;
  branch.path.node_specs.push_back(shared_branch);
  add_all_templates(branch);
  if (!state.GenerateFromBackboneSpec(branch).ok) {
    std::cerr << "[DBG] minimal_support_outer branch_generate_failed\n";
    return false;
  }

  wire::core::BackboneSpec new_main{};
  new_main.path.polyline = {kNewMain, kCenter};
  new_main.interval_m = 1000.0;
  new_main.pole_type_id = communication_pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec shared_main{};
  shared_main.point_index = 1;
  shared_main.support_kind = wire::core::SupportKind::kPole;
  shared_main.node_id = center_id;
  new_main.path.node_specs.push_back(shared_main);
  add_all_templates(new_main);
  const auto generated = state.GenerateFromBackboneSpec(new_main);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] minimal_support_outer new_main_generate_failed ok=" << generated.ok
              << " error=" << generated.error << "\n";
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] minimal_support_outer commit_failed\n";
    return false;
  }

  if (out_center_id != nullptr) {
    *out_center_id = center_id;
  }
  if (out_generated_span_ids != nullptr) {
    *out_generated_span_ids = generated.value.generated_span_ids;
  }
  return true;
}

bool test_backbone_capture_like_outer_same_level_cross_uses_pair_authority() {
  CoreState state;
  ObjectId center_id = wire::core::kInvalidObjectId;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_minimal_latest_support_orientation_outer_scene(state, &center_id, &generated_span_ids)) {
    return false;
  }

  const std::unordered_set<ObjectId> generated_span_set(generated_span_ids.begin(), generated_span_ids.end());
  bool saw_pair_authority = false;
  for (const auto& span_entry : state.view().spans().items()) {
    if (!generated_span_set.contains(span_entry.id)) {
      continue;
    }
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->lower_required || !endpoint->same_level_feasible ||
        endpoint->continuity_class != wire::core::ContinuityCategoryClass::kPointLike) {
      continue;
    }

    const bool expect_side_branch = endpoint->relation_kind == wire::core::JunctionRelationKind::kSideBranch;
    if (!expect_side_branch) {
      continue;
    }

    if (!endpoint_uses_pair_authority_without_local_fallback(*endpoint) ||
        !pair_authority_axis_family_is_consistent(*endpoint)) {
      return false;
    }
    saw_pair_authority = true;
  }
  return saw_pair_authority;
}

bool test_backbone_capture_like_outer_same_level_cross_visual_uses_pair_axis() {
  CoreState state;
  ObjectId center_id = wire::core::kInvalidObjectId;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_minimal_latest_support_orientation_outer_scene(state, &center_id, &generated_span_ids)) {
    return false;
  }

  const std::unordered_set<ObjectId> generated_span_set(generated_span_ids.begin(), generated_span_ids.end());
  bool saw_pair_axis = false;
  for (const auto& span_entry : state.view().spans().items()) {
    if (!generated_span_set.contains(span_entry.id)) {
      continue;
    }
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->lower_required || !endpoint->same_level_feasible ||
        endpoint->continuity_class != wire::core::ContinuityCategoryClass::kPointLike ||
        !endpoint_uses_pair_authority_without_local_fallback(*endpoint) ||
        !endpoint->has_side_axis || std::abs(endpoint->chosen_side_sign) <= 1e-9) {
      continue;
    }

    const bool expect_side_branch = endpoint->relation_kind == wire::core::JunctionRelationKind::kSideBranch;
    if (!expect_side_branch) {
      continue;
    }

    const auto arm = support_arm_part_for_owner(state, span_entry.id, center_id);
    if (!arm.has_value()) {
      std::cerr << "[DBG] C347 missing_arm span=" << span_entry.id << "\n";
      return false;
    }
    if (!endpoint->has_visual_arm_geometry) {
      std::cerr << "[DBG] C347 missing_visual_arm span=" << span_entry.id << "\n";
      return false;
    }
    const wire::core::Vec3d expected =
        normalize_xy_safe(endpoint->visual_arm_tip_world - endpoint->visual_arm_mount_world);
    const wire::core::Vec3d actual = normalize_xy_safe(arm->b - arm->a);
    const double align = std::abs(dot_xy(actual, expected));
    const bool mount_matches = almost_equal(arm->a.x, endpoint->visual_arm_mount_world.x, 1e-6) &&
                               almost_equal(arm->a.y, endpoint->visual_arm_mount_world.y, 1e-6) &&
                               almost_equal(arm->a.z, endpoint->visual_arm_mount_world.z, 1e-6);
    const bool tip_matches = almost_equal(arm->b.x, endpoint->visual_arm_tip_world.x, 1e-6) &&
                             almost_equal(arm->b.y, endpoint->visual_arm_tip_world.y, 1e-6) &&
                             almost_equal(arm->b.z, endpoint->visual_arm_tip_world.z, 1e-6);
    if (align < 0.97 || !mount_matches || !tip_matches) {
      std::cerr << "[DBG] C347 span=" << span_entry.id << " relation=" << static_cast<int>(endpoint->relation_kind)
                << " align=" << align << " expected=(" << expected.x << "," << expected.y << ") actual=("
                << actual.x << "," << actual.y << ") mountMatch=" << (mount_matches ? 1 : 0)
                << " tipMatch=" << (tip_matches ? 1 : 0) << "\n";
      return false;
    }
    saw_pair_axis = true;
  }
  return saw_pair_axis;
}

struct SupportAuthoritySnapshot {
  ObjectId span_id = wire::core::kInvalidObjectId;
  ObjectId owner_pole_id = wire::core::kInvalidObjectId;
  bool has_decision_seed = false;
  bool requires_decision_seed = false;
  wire::core::ResolvedSupportAuthority authority{};
  wire::core::SupportOrientationRuleKind orientation_rule = wire::core::SupportOrientationRuleKind::kRadial;
  wire::core::SideAssignmentRuleKind side_rule = wire::core::SideAssignmentRuleKind::kPoleLocal;
  wire::core::OrderDecisionPolicyKind order_policy = wire::core::OrderDecisionPolicyKind::kFixedOrder;
  wire::core::OrderDecisionChoiceKind order_choice = wire::core::OrderDecisionChoiceKind::kNormal;
  wire::core::OrderDecisionChoiceReason order_reason = wire::core::OrderDecisionChoiceReason::kFixedOrder;
  wire::core::SupportOrientationBasisKind orientation_basis = wire::core::SupportOrientationBasisKind::kRadial;
  wire::core::LateralSideChoiceKind chosen_side = wire::core::LateralSideChoiceKind::kCenter;
  wire::core::JunctionRelationKind relation_kind = wire::core::JunctionRelationKind::kNone;
  wire::core::ContinuityCategoryClass continuity_class = wire::core::ContinuityCategoryClass::kPointLike;
  bool in_through_pair = false;
  int pair_height_rank = -1;
  bool has_visual_arm_geometry = false;
  wire::core::Vec3d visual_arm_mount_world{};
  wire::core::Vec3d visual_arm_tip_world{};
  wire::core::Vec3d visual_insulator_base_world{};
};

bool build_replayed_latest_t_support_capture_scene(CoreState& state, std::vector<ObjectId>* out_generated_span_ids);

bool resolved_support_authority_matches_decision(const wire::core::SupportLayoutEndpointView& endpoint) {
  if (!endpoint.has_signed_support_axis || !endpoint.has_side_axis || std::abs(endpoint.chosen_side_sign) <= 1e-9) {
    return false;
  }
  if (endpoint.support_authority.pair.pair_peer_low == wire::core::kInvalidObjectId ||
      endpoint.support_authority.pair.pair_peer_high == wire::core::kInvalidObjectId ||
      endpoint.pair_height_rank < 0) {
    return false;
  }
  wire::core::Vec3d expected = endpoint.side_axis;
  if (endpoint.chosen_side_sign < 0.0) {
    expected.x = -expected.x;
    expected.y = -expected.y;
  }
  expected = normalize_xy_safe(expected);
  const wire::core::Vec3d actual = normalize_xy_safe(endpoint.signed_support_axis);
  return std::abs(dot_xy(expected, actual)) >= 0.99;
}

bool collect_pair_authority_snapshot(const CoreState& state, ObjectId center_id, SupportAuthoritySnapshot* out) {
  if (out == nullptr) {
    return false;
  }
  for (const auto& span_entry : state.view().spans().items()) {
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->lower_required || !endpoint->same_level_feasible ||
        endpoint->continuity_class != wire::core::ContinuityCategoryClass::kPointLike ||
        !endpoint_uses_pair_authority_without_local_fallback(*endpoint)) {
      continue;
    }
    if (!resolved_support_authority_matches_decision(*endpoint)) {
      continue;
    }
    out->span_id = span_entry.id;
    out->owner_pole_id = center_id;
    out->has_decision_seed = layout_view->has_decision_seed;
    out->requires_decision_seed = layout_view->requires_decision_seed;
    out->authority = endpoint->support_authority;
    out->orientation_rule = endpoint->support_orientation_rule;
    out->side_rule = endpoint->side_assignment_rule;
    out->order_policy = endpoint->order_decision_policy;
    out->order_choice = endpoint->order_decision_choice;
    out->order_reason = endpoint->order_decision_choice_reason;
    out->orientation_basis = endpoint->support_orientation_basis;
    out->chosen_side = endpoint->chosen_side;
    out->relation_kind = endpoint->relation_kind;
    out->continuity_class = endpoint->continuity_class;
    out->in_through_pair = endpoint->in_through_pair;
    out->pair_height_rank = endpoint->pair_height_rank;
    out->has_visual_arm_geometry = endpoint->has_visual_arm_geometry;
    out->visual_arm_mount_world = endpoint->visual_arm_mount_world;
    out->visual_arm_tip_world = endpoint->visual_arm_tip_world;
    out->visual_insulator_base_world = endpoint->visual_insulator_base_world;
    return true;
  }
  return false;
}

bool resolved_support_authority_equal(const wire::core::ResolvedSupportAuthority& a,
                                      const wire::core::ResolvedSupportAuthority& b) {
  return a.pair.pair_peer_low == b.pair.pair_peer_low && a.pair.pair_peer_high == b.pair.pair_peer_high &&
         a.pair.orientation_basis == b.pair.orientation_basis && a.pair.has_pair_axis == b.pair.has_pair_axis &&
         a.pair.height_rank == b.pair.height_rank &&
         almost_equal(a.pair.pair_axis.x, b.pair.pair_axis.x, 1e-9) &&
         almost_equal(a.pair.pair_axis.y, b.pair.pair_axis.y, 1e-9) &&
         almost_equal(a.pair.pair_axis.z, b.pair.pair_axis.z, 1e-9) &&
         a.has_signed_support_axis == b.has_signed_support_axis &&
         almost_equal(a.signed_support_axis.x, b.signed_support_axis.x, 1e-9) &&
         almost_equal(a.signed_support_axis.y, b.signed_support_axis.y, 1e-9) &&
         almost_equal(a.signed_support_axis.z, b.signed_support_axis.z, 1e-9);
}

bool support_authority_snapshot_equal(const SupportAuthoritySnapshot& a, const SupportAuthoritySnapshot& b) {
  return a.span_id == b.span_id && a.owner_pole_id == b.owner_pole_id && a.has_decision_seed == b.has_decision_seed &&
         a.requires_decision_seed == b.requires_decision_seed && a.orientation_rule == b.orientation_rule &&
         a.side_rule == b.side_rule && a.order_policy == b.order_policy && a.order_choice == b.order_choice &&
         a.order_reason == b.order_reason && a.orientation_basis == b.orientation_basis &&
         a.chosen_side == b.chosen_side && a.relation_kind == b.relation_kind &&
         a.continuity_class == b.continuity_class && a.in_through_pair == b.in_through_pair &&
         a.pair_height_rank == b.pair_height_rank &&
         a.has_visual_arm_geometry == b.has_visual_arm_geometry &&
         almost_equal(a.visual_arm_mount_world.x, b.visual_arm_mount_world.x, 1e-9) &&
         almost_equal(a.visual_arm_mount_world.y, b.visual_arm_mount_world.y, 1e-9) &&
         almost_equal(a.visual_arm_mount_world.z, b.visual_arm_mount_world.z, 1e-9) &&
         almost_equal(a.visual_arm_tip_world.x, b.visual_arm_tip_world.x, 1e-9) &&
         almost_equal(a.visual_arm_tip_world.y, b.visual_arm_tip_world.y, 1e-9) &&
         almost_equal(a.visual_arm_tip_world.z, b.visual_arm_tip_world.z, 1e-9) &&
         almost_equal(a.visual_insulator_base_world.x, b.visual_insulator_base_world.x, 1e-9) &&
         almost_equal(a.visual_insulator_base_world.y, b.visual_insulator_base_world.y, 1e-9) &&
         almost_equal(a.visual_insulator_base_world.z, b.visual_insulator_base_world.z, 1e-9) &&
         resolved_support_authority_equal(a.authority, b.authority);
}

std::optional<wire::core::AttachmentTemplateId> find_attachment_template_id_by_name(const CoreState& state,
                                                                                     std::string_view name) {
  for (const auto& [template_id, attachment_template] : state.view().attachment_templates()) {
    if (attachment_template.name == name) {
      return template_id;
    }
  }
  return std::nullopt;
}

bool configure_bundle_default_endpoint_attachment(CoreState& state, wire::core::BundleKind bundle_kind,
                                                  std::string_view attachment_name) {
  const auto bundle_it = state.view().bundle_templates().find(bundle_kind);
  if (bundle_it == state.view().bundle_templates().end()) {
    return false;
  }
  const auto cable_it = state.view().cable_templates().find(bundle_it->second.cable_template_id);
  if (cable_it == state.view().cable_templates().end()) {
    return false;
  }
  const auto attachment_id = find_attachment_template_id_by_name(state, attachment_name);
  if (!attachment_id.has_value()) {
    return false;
  }
  wire::core::CableTemplate updated = cable_it->second;
  updated.default_endpoint_attachment_template_id = *attachment_id;
  const auto apply = state.UpdateCableTemplate(updated);
  return apply.ok;
}

struct MaterializedSocketSnapshot {
  ObjectId span_id = wire::core::kInvalidObjectId;
  bool has_decision_seed = false;
  wire::core::SupportLayoutEndpointSourceKind start_source = wire::core::SupportLayoutEndpointSourceKind::kFallback;
  wire::core::SupportLayoutEndpointSourceKind end_source = wire::core::SupportLayoutEndpointSourceKind::kFallback;
  wire::core::EndpointAttachmentRequest start_request{};
  wire::core::EndpointAttachmentRequest end_request{};
  std::optional<int> start_resolved_socket_id{};
  std::optional<int> end_resolved_socket_id{};
};

bool attachment_request_equal(const wire::core::EndpointAttachmentRequest& a,
                              const wire::core::EndpointAttachmentRequest& b) {
  return a.kind == b.kind && a.attachment_id == b.attachment_id && a.requested_socket_id == b.requested_socket_id;
}

bool materialized_socket_snapshot_equal(const MaterializedSocketSnapshot& a, const MaterializedSocketSnapshot& b) {
  return a.span_id == b.span_id && a.has_decision_seed == b.has_decision_seed && a.start_source == b.start_source &&
         a.end_source == b.end_source && attachment_request_equal(a.start_request, b.start_request) &&
         attachment_request_equal(a.end_request, b.end_request) &&
         a.start_resolved_socket_id == b.start_resolved_socket_id && a.end_resolved_socket_id == b.end_resolved_socket_id;
}

bool detail_curve_socket_state_equal(const wire::core::DetailCurveInspectionView& a,
                                     const wire::core::DetailCurveInspectionView& b) {
  return a.start_endpoint_source == b.start_endpoint_source && a.end_endpoint_source == b.end_endpoint_source &&
         attachment_request_equal(a.start_attachment_request, b.start_attachment_request) &&
         attachment_request_equal(a.end_attachment_request, b.end_attachment_request) &&
         a.start_resolved_socket_id == b.start_resolved_socket_id && a.end_resolved_socket_id == b.end_resolved_socket_id;
}

bool detail_curve_geometry_equal(const wire::core::DetailCurveInspectionView& a,
                                 const wire::core::DetailCurveInspectionView& b) {
  if (a.shape_policy != b.shape_policy || a.adopted_continuity != b.adopted_continuity ||
      a.continuity_reason != b.continuity_reason || a.start_tangent_rule != b.start_tangent_rule ||
      a.end_tangent_rule != b.end_tangent_rule || a.segment_count != b.segment_count ||
      a.arc_length_sample_count != b.arc_length_sample_count || a.visible_interval_count != b.visible_interval_count ||
      a.hidden_interval_count != b.hidden_interval_count || a.replacement_path_count != b.replacement_path_count ||
      a.supplemental_path_count != b.supplemental_path_count) {
    return false;
  }
  if (!almost_equal(a.curve_length_m, b.curve_length_m, 1e-9) ||
      !almost_equal(a.handle_length_start_m, b.handle_length_start_m, 1e-9) ||
      !almost_equal(a.handle_length_end_m, b.handle_length_end_m, 1e-9) ||
      !almost_equal(a.start_support_weight, b.start_support_weight, 1e-9) ||
      !almost_equal(a.end_support_weight, b.end_support_weight, 1e-9) ||
      !almost_equal(a.start_chord_weight, b.start_chord_weight, 1e-9) ||
      !almost_equal(a.end_chord_weight, b.end_chord_weight, 1e-9)) {
    return false;
  }
  for (std::size_t i = 0; i < a.control_points.size(); ++i) {
    if (!almost_equal(a.control_points[i].x, b.control_points[i].x, 1e-9) ||
        !almost_equal(a.control_points[i].y, b.control_points[i].y, 1e-9) ||
        !almost_equal(a.control_points[i].z, b.control_points[i].z, 1e-9)) {
      return false;
    }
  }
  return detail_curve_socket_state_equal(a, b);
}

bool collect_materialized_socket_snapshot(const CoreState& state, ObjectId span_id, MaterializedSocketSnapshot* out) {
  if (out == nullptr) {
    return false;
  }
  const auto layout_view = state.view().inspect_support_layout(span_id);
  if (!layout_view.has_value()) {
    return false;
  }
  out->span_id = span_id;
  out->has_decision_seed = layout_view->has_decision_seed;
  out->start_source = layout_view->start_endpoint.endpoint_source;
  out->end_source = layout_view->end_endpoint.endpoint_source;
  out->start_request = layout_view->start_endpoint.attachment_request;
  out->end_request = layout_view->end_endpoint.attachment_request;
  out->start_resolved_socket_id = layout_view->start_endpoint.resolved_socket_id;
  out->end_resolved_socket_id = layout_view->end_endpoint.resolved_socket_id;
  return true;
}

bool build_socket_materialization_scene(CoreState& state, ObjectId* out_span_id) {
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  if (!configure_bundle_default_endpoint_attachment(state, wire::core::BundleKind::kLowVoltage,
                                                    "GENERIC_PASS_THROUGH")) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  const ObjectId span_id = generated.value.generated_span_ids.front();
  if (!state.SetSpanEndpointSocketOverride(span_id, true, 0).ok) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  if (out_span_id != nullptr) {
    *out_span_id = span_id;
  }
  return true;
}

double pair_height_step_for_span(const CoreState& state, ObjectId span_id) {
  const auto* span = state.view().edit_state().spans.find(span_id);
  if (span == nullptr) {
    return 0.0;
  }
  const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
  if (bundle == nullptr) {
    return 0.0;
  }
  const auto bundle_template_it = state.view().bundle_templates().find(bundle->bundle_template_id);
  if (bundle_template_it == state.view().bundle_templates().end()) {
    return 0.0;
  }
  return wire::core::generation::detail::BranchDownOffsetForCategory(bundle_template_it->second.category);
}

bool has_validation_issue(const wire::core::ValidationResult& validation, wire::core::ValidationSeverity severity,
                          const char* code, ObjectId object_id = wire::core::kInvalidObjectId) {
  return std::any_of(validation.issues.begin(), validation.issues.end(),
                     [&](const wire::core::ValidationIssue& issue) {
                       return issue.severity == severity && issue.code == code &&
                              (object_id == wire::core::kInvalidObjectId || issue.object_id == object_id);
                     });
}

bool test_backbone_support_layout_inspection_exposes_resolved_support_authority() {
  CoreState state;
  ObjectId center_id = wire::core::kInvalidObjectId;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_minimal_latest_support_orientation_outer_scene(state, &center_id, &generated_span_ids)) {
    return false;
  }

  SupportAuthoritySnapshot snapshot{};
  if (!collect_pair_authority_snapshot(state, center_id, &snapshot)) {
    std::cerr << "[DBG] C348 missing_pair_authority_snapshot center=" << center_id << "\n";
    return false;
  }
  if (!snapshot.has_decision_seed || snapshot.authority.pair.height_rank < 0 ||
      snapshot.authority.pair.pair_peer_low == wire::core::kInvalidObjectId ||
      snapshot.authority.pair.pair_peer_high == wire::core::kInvalidObjectId ||
      !snapshot.authority.has_signed_support_axis) {
    std::cerr << "[DBG] C348 seed=" << (snapshot.has_decision_seed ? 1 : 0)
              << " heightRank=" << snapshot.authority.pair.height_rank << " peers=("
              << snapshot.authority.pair.pair_peer_low << "," << snapshot.authority.pair.pair_peer_high
              << ") hasSigned=" << (snapshot.authority.has_signed_support_axis ? 1 : 0) << "\n";
    return false;
  }
  return true;
}

bool test_backbone_same_level_t_pair_height_rank_drives_support_offsets() {
  CoreState state;
  ObjectId center_id = wire::core::kInvalidObjectId;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_minimal_latest_support_orientation_outer_scene(state, &center_id, &generated_span_ids)) {
    return false;
  }

  const std::unordered_set<ObjectId> generated_span_set(generated_span_ids.begin(), generated_span_ids.end());
  std::optional<std::pair<ObjectId, ObjectId>> expected_pair{};
  bool saw_through_main = false;
  bool saw_side_branch = false;
  for (const auto& span_entry : state.view().spans().items()) {
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->lower_required || !endpoint->same_level_feasible ||
        endpoint->continuity_class != wire::core::ContinuityCategoryClass::kPointLike ||
        !endpoint_uses_pair_authority_without_local_fallback(*endpoint, false)) {
      continue;
    }
    const auto endpoint_pair = std::make_pair(endpoint->support_authority.pair.pair_peer_low,
                                              endpoint->support_authority.pair.pair_peer_high);
    if (!expected_pair.has_value()) {
      expected_pair = endpoint_pair;
    } else if (endpoint_pair != *expected_pair) {
      std::cerr << "[DBG] C353 pair_mismatch span=" << span_entry.id << " expected=(" << expected_pair->first << ","
                << expected_pair->second << ") got=(" << endpoint_pair.first << "," << endpoint_pair.second << ")\n";
      return false;
    }
    const double expected_step = pair_height_step_for_span(state, span_entry.id);
    if (endpoint->relation_kind == wire::core::JunctionRelationKind::kThroughMain) {
      saw_through_main = true;
      if (endpoint->support_orientation_rule != wire::core::SupportOrientationRuleKind::kBisector ||
          endpoint->side_assignment_rule != wire::core::SideAssignmentRuleKind::kBisector) {
        std::cerr << "[DBG] C353 through_rule span=" << span_entry.id << " sideRule="
                  << static_cast<int>(endpoint->side_assignment_rule) << " orientRule="
                  << static_cast<int>(endpoint->support_orientation_rule) << " relation="
                  << static_cast<int>(endpoint->relation_kind) << " continuity="
                  << static_cast<int>(endpoint->continuity_class) << " lower="
                  << (endpoint->lower_required ? 1 : 0) << " same="
                  << (endpoint->same_level_feasible ? 1 : 0) << " inPair="
                  << (endpoint->in_through_pair ? 1 : 0) << " peers=("
                  << endpoint->support_authority.pair.pair_peer_low << ","
                  << endpoint->support_authority.pair.pair_peer_high << ") owner=" << endpoint->owner_pole_id
                  << " generated=" << (generated_span_set.contains(span_entry.id) ? 1 : 0)
                  << " source=" << static_cast<int>(endpoint->endpoint_source)
                  << "\n";
        if (const auto junction = state.view().inspect_junction(center_id); junction.has_value()) {
          std::cerr << "[DBG] C353 center_junction accepted=" << (junction->through_pair_accepted ? 1 : 0)
                    << " pair=(" << junction->through_pair_neighbor_a_id << "," << junction->through_pair_neighbor_b_id
                    << ") count=" << junction->local_relations.size() << "\n";
          for (const auto& relation : junction->local_relations) {
            std::cerr << "[DBG] C353 rel peer=" << relation.neighbor_node_id << " kind="
                      << static_cast<int>(relation.kind) << " inRoute=" << (relation.in_route ? 1 : 0)
                      << " inPair=" << (relation.in_through_pair ? 1 : 0) << " class="
                      << static_cast<int>(relation.continuity_class) << " same="
                      << (relation.same_level_feasible ? 1 : 0) << " lower="
                      << (relation.default_lower_required ? 1 : 0) << "\n";
          }
        }
        return false;
      }
      if (!(endpoint->pair_height_rank == 0 && almost_equal(endpoint->branch_down_offset_m, 0.0, 1e-6))) {
        std::cerr << "[DBG] C353 through span=" << span_entry.id << " rank=" << endpoint->pair_height_rank
                  << " down=" << endpoint->branch_down_offset_m << "\n";
        return false;
      }
    } else if (endpoint->relation_kind == wire::core::JunctionRelationKind::kSideBranch) {
      saw_side_branch = true;
      if (endpoint->support_orientation_rule != wire::core::SupportOrientationRuleKind::kBisector ||
          endpoint->side_assignment_rule != wire::core::SideAssignmentRuleKind::kBisector) {
        std::cerr << "[DBG] C353 branch_rule span=" << span_entry.id << " sideRule="
                  << static_cast<int>(endpoint->side_assignment_rule) << " orientRule="
                  << static_cast<int>(endpoint->support_orientation_rule) << "\n";
        return false;
      }
      if (!(endpoint->pair_height_rank == 1 &&
            almost_equal(endpoint->branch_down_offset_m, expected_step, 1e-6))) {
        std::cerr << "[DBG] C353 branch span=" << span_entry.id << " rank=" << endpoint->pair_height_rank
                  << " down=" << endpoint->branch_down_offset_m << " step=" << expected_step << "\n";
        return false;
      }
    }
  }

  if (!(saw_through_main && saw_side_branch)) {
    std::cerr << "[DBG] C353 sawThroughMain=" << (saw_through_main ? 1 : 0)
              << " sawSideBranch=" << (saw_side_branch ? 1 : 0) << " generatedCount="
              << generated_span_set.size() << "\n";
  }
  return expected_pair.has_value() && saw_through_main && saw_side_branch;
}

bool test_backbone_one_shot_cross_pair_height_rank_separates_pairs() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = pole_type_id;
  add_backbone_bundle(main_req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(main_req, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(main_req, wire::core::BundleKind::kOptical);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross_req{};
  cross_req.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross_req.path.node_specs.push_back(shared);
  cross_req.interval_m = 1000.0;
  cross_req.pole_type_id = pole_type_id;
  add_backbone_bundle(cross_req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(cross_req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(cross_req, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(cross_req, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(cross_req);
  if (!generated.ok) {
    return false;
  }
  const std::unordered_set<ObjectId> generated_span_set(generated.value.generated_span_ids.begin(),
                                                        generated.value.generated_span_ids.end());

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }
  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value() || !junction->through_pair_accepted) {
    return false;
  }

  bool saw_cross_pair = false;
  for (const auto& span_entry : state.view().spans().items()) {
    if (!generated_span_set.contains(span_entry.id)) {
      continue;
    }
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->lower_required || !endpoint->same_level_feasible ||
        endpoint->continuity_class != wire::core::ContinuityCategoryClass::kPointLike ||
        endpoint->support_orientation_rule != wire::core::SupportOrientationRuleKind::kThroughPairNormal) {
      continue;
    }
    const double expected_step = pair_height_step_for_span(state, span_entry.id);
    if (endpoint->relation_kind == wire::core::JunctionRelationKind::kCrossUnderpass) {
      saw_cross_pair = true;
      if (!(endpoint->pair_height_rank == 1 &&
            almost_equal(endpoint->branch_down_offset_m, expected_step, 1e-6))) {
        return false;
      }
    }
  }
  return saw_cross_pair;
}

bool test_backbone_geometry_refresh_keeps_resolved_support_authority_seed() {
  CoreState state;
  ObjectId center_id = wire::core::kInvalidObjectId;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_minimal_latest_support_orientation_outer_scene(state, &center_id, &generated_span_ids)) {
    return false;
  }

  SupportAuthoritySnapshot before{};
  if (!collect_pair_authority_snapshot(state, center_id, &before)) {
    std::cerr << "[DBG] C349 before_missing_snapshot center=" << center_id << "\n";
    return false;
  }

  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.curve_samples += 1;
  const auto update = state.UpdateGeometrySettings(settings, true);
  if (!update.ok || !update.value) {
    std::cerr << "[DBG] C349 update_geometry_failed ok=" << (update.ok ? 1 : 0)
              << " changed=" << (update.value ? 1 : 0) << "\n";
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] C349 commit_failed\n";
    return false;
  }

  SupportAuthoritySnapshot after{};
  if (!collect_pair_authority_snapshot(state, center_id, &after)) {
    std::cerr << "[DBG] C349 after_missing_snapshot center=" << center_id << "\n";
    return false;
  }
  const bool ok = before.has_decision_seed && before.requires_decision_seed &&
                  support_authority_snapshot_equal(before, after);
  if (!ok) {
    std::cerr << "[DBG] C349 changed beforeSeed=" << (before.has_decision_seed ? 1 : 0)
              << " afterSeed=" << (after.has_decision_seed ? 1 : 0)
              << " beforeRelation=" << static_cast<int>(before.relation_kind)
              << " afterRelation=" << static_cast<int>(after.relation_kind)
              << " beforeClass=" << static_cast<int>(before.continuity_class)
              << " afterClass=" << static_cast<int>(after.continuity_class)
              << " beforePair=" << (before.in_through_pair ? 1 : 0)
              << " afterPair=" << (after.in_through_pair ? 1 : 0)
              << " beforeRule=" << static_cast<int>(before.orientation_rule)
              << " afterRule=" << static_cast<int>(after.orientation_rule)
              << " beforeOrder=" << static_cast<int>(before.order_choice)
              << " afterOrder=" << static_cast<int>(after.order_choice)
              << " beforeHeight=" << before.authority.pair.height_rank
              << " afterHeight=" << after.authority.pair.height_rank << "\n";
  }
  return ok;
}

bool test_backbone_render_refresh_does_not_change_resolved_support_authority_seed() {
  CoreState state;
  ObjectId center_id = wire::core::kInvalidObjectId;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_minimal_latest_support_orientation_outer_scene(state, &center_id, &generated_span_ids)) {
    return false;
  }

  SupportAuthoritySnapshot before{};
  if (!collect_pair_authority_snapshot(state, center_id, &before)) {
    std::cerr << "[DBG] C350 before_missing_snapshot center=" << center_id << "\n";
    return false;
  }
  const auto before_curve = state.view().inspect_detail_curve(before.span_id);
  if (!before_curve.has_value()) {
    std::cerr << "[DBG] C350 before_missing_curve span=" << before.span_id << "\n";
    return false;
  }

  wire::core::VisualSettings settings = state.view().visual_settings();
  settings.support_arm_extra_m += 0.05;
  const auto update = state.UpdateVisualSettings(settings, true);
  if (!update.ok || !update.value) {
    std::cerr << "[DBG] C350 update_visual_failed ok=" << (update.ok ? 1 : 0)
              << " changed=" << (update.value ? 1 : 0) << "\n";
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] C350 commit_failed\n";
    return false;
  }

  SupportAuthoritySnapshot after{};
  if (!collect_pair_authority_snapshot(state, center_id, &after)) {
    std::cerr << "[DBG] C350 after_missing_snapshot center=" << center_id << "\n";
    return false;
  }
  const auto after_curve = state.view().inspect_detail_curve(after.span_id);
  const bool ok = before.has_decision_seed && before.requires_decision_seed &&
                  support_authority_snapshot_equal(before, after) && after_curve.has_value() &&
                  detail_curve_geometry_equal(*before_curve, *after_curve);
  if (!ok) {
    std::cerr << "[DBG] C350 changed seed=" << (after.has_decision_seed ? 1 : 0)
              << " beforeRelation=" << static_cast<int>(before.relation_kind)
              << " afterRelation=" << static_cast<int>(after.relation_kind)
              << " beforeClass=" << static_cast<int>(before.continuity_class)
              << " afterClass=" << static_cast<int>(after.continuity_class)
              << " beforePair=" << (before.in_through_pair ? 1 : 0)
              << " afterPair=" << (after.in_through_pair ? 1 : 0)
              << " beforeRule=" << static_cast<int>(before.orientation_rule)
              << " afterRule=" << static_cast<int>(after.orientation_rule)
              << " beforeOrder=" << static_cast<int>(before.order_choice)
              << " afterOrder=" << static_cast<int>(after.order_choice)
              << " beforeHeight=" << before.authority.pair.height_rank
              << " afterHeight=" << after.authority.pair.height_rank
              << " curve=" << (after_curve.has_value() ? 1 : 0) << "\n";
  }
  return ok;
}

bool test_backbone_rebuild_without_seed_fails_and_keeps_cached_layout() {
  CoreState state;
  ObjectId center_id = wire::core::kInvalidObjectId;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_minimal_latest_support_orientation_outer_scene(state, &center_id, &generated_span_ids)) {
    return false;
  }

  SupportAuthoritySnapshot before{};
  if (!collect_pair_authority_snapshot(state, center_id, &before)) {
    std::cerr << "[DBG] C351 before_missing_snapshot center=" << center_id << "\n";
    return false;
  }

  wire::core::CoreStateTestHook::erase_cached_span_support_layout_seed(state, before.span_id);

  std::string error_message;
  const bool rebuild_ok =
      wire::core::CoreStateTestHook::rebuild_span_geometry_from_seed(state, before.span_id, &error_message);

  const auto layout_view = state.view().inspect_support_layout(before.span_id);
  if (!layout_view.has_value()) {
    std::cerr << "[DBG] C351 missing_layout_view span=" << before.span_id << "\n";
    return false;
  }
  const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
  if (!endpoint.has_value()) {
    std::cerr << "[DBG] C351 missing_endpoint center=" << center_id << " span=" << before.span_id << "\n";
    return false;
  }

  const bool has_warning = has_validation_issue(wire::core::CoreStateTestHook::validate(state),
                                                wire::core::ValidationSeverity::kWarning,
                                                "SupportLayoutDecisionSeedMissing", before.span_id);
  const bool ok = !rebuild_ok && !layout_view->has_decision_seed && layout_view->requires_decision_seed && has_warning &&
                  resolved_support_authority_equal(before.authority, endpoint->support_authority) &&
                  endpoint->relation_kind == before.relation_kind &&
                  endpoint->continuity_class == before.continuity_class &&
                  endpoint->in_through_pair == before.in_through_pair;
  if (!ok) {
    std::cerr << "[DBG] C351 rebuild=" << (rebuild_ok ? 1 : 0)
              << " err=" << error_message
              << " seed=" << (layout_view->has_decision_seed ? 1 : 0)
              << " requires=" << (layout_view->requires_decision_seed ? 1 : 0)
              << " warning=" << (has_warning ? 1 : 0)
              << " relation=" << static_cast<int>(endpoint->relation_kind)
              << " class=" << static_cast<int>(endpoint->continuity_class)
              << " inPair=" << (endpoint->in_through_pair ? 1 : 0) << "\n";
  }
  return ok;
}

bool test_backbone_geometry_refresh_without_seed_fails_and_keeps_dirty() {
  CoreState state;
  ObjectId center_id = wire::core::kInvalidObjectId;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_minimal_latest_support_orientation_outer_scene(state, &center_id, &generated_span_ids)) {
    return false;
  }

  SupportAuthoritySnapshot before{};
  if (!collect_pair_authority_snapshot(state, center_id, &before)) {
    std::cerr << "[DBG] C352 before_missing_snapshot center=" << center_id << "\n";
    return false;
  }

  wire::core::CoreStateTestHook::erase_cached_span_support_layout_seed(state, before.span_id);

  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.curve_samples += 1;
  const auto update = state.UpdateGeometrySettings(settings, true);
  if (!update.ok || !update.value) {
    std::cerr << "[DBG] C352 update_geometry_failed ok=" << (update.ok ? 1 : 0)
              << " changed=" << (update.value ? 1 : 0) << "\n";
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
    (void)state.Commit(options);
  const auto runtime_it = wire::core::CoreStateTestHook::span_runtime_states(state).find(before.span_id);
  const auto layout_view = state.view().inspect_support_layout(before.span_id);
  const bool has_warning =
      has_validation_issue(wire::core::CoreStateTestHook::validate(state), wire::core::ValidationSeverity::kWarning,
                           "SupportLayoutDecisionSeedMissing", before.span_id);
  const bool geometry_still_dirty = runtime_it != wire::core::CoreStateTestHook::span_runtime_states(state).end() &&
                                    helpers::has_dirty(&runtime_it->second, wire::core::DirtyBits::kGeometryRefresh);
  const auto endpoint = layout_view.has_value() ? layout_endpoint_for_owner(*layout_view, center_id) : std::nullopt;
  const bool ok = has_warning && geometry_still_dirty && layout_view.has_value() && !layout_view->has_decision_seed &&
                  layout_view->requires_decision_seed && endpoint.has_value() &&
                  resolved_support_authority_equal(before.authority, endpoint->support_authority) &&
                  endpoint->relation_kind == before.relation_kind &&
                  endpoint->continuity_class == before.continuity_class &&
                  endpoint->in_through_pair == before.in_through_pair;
  if (!ok) {
    std::cerr << "[DBG] C352 warning=" << (has_warning ? 1 : 0)
              << " dirty=" << (geometry_still_dirty ? 1 : 0)
              << " seed=" << (layout_view.has_value() && layout_view->has_decision_seed ? 1 : 0)
              << " requires=" << (layout_view.has_value() && layout_view->requires_decision_seed ? 1 : 0)
              << " endpoint=" << (endpoint.has_value() ? 1 : 0)
              << " relation="
              << (endpoint.has_value() ? static_cast<int>(endpoint->relation_kind) : -1)
              << " class="
              << (endpoint.has_value() ? static_cast<int>(endpoint->continuity_class) : -1)
              << " inPair=" << (endpoint.has_value() && endpoint->in_through_pair ? 1 : 0) << "\n";
  }
  return ok;
}

bool test_backbone_materialization_resolves_endpoint_socket_and_curve_consumes_it() {
  CoreState state;
  ObjectId span_id = wire::core::kInvalidObjectId;
  if (!build_socket_materialization_scene(state, &span_id)) {
    std::cerr << "[DBG] C358 scene_build_failed\n";
    return false;
  }

  MaterializedSocketSnapshot snapshot{};
  const auto curve_view = state.view().inspect_detail_curve(span_id);
  if (!collect_materialized_socket_snapshot(state, span_id, &snapshot) || !curve_view.has_value()) {
    std::cerr << "[DBG] C358 snapshot_missing span=" << span_id << "\n";
    return false;
  }

  const bool ok =
      snapshot.has_decision_seed &&
      snapshot.start_source == wire::core::SupportLayoutEndpointSourceKind::kAttachmentSocketOverride &&
      snapshot.end_source == wire::core::SupportLayoutEndpointSourceKind::kAttachmentSocket &&
      snapshot.start_request.kind == wire::core::EndpointAttachmentRequestKind::kAttachmentSocket &&
      snapshot.end_request.kind == wire::core::EndpointAttachmentRequestKind::kAttachmentSocket &&
      snapshot.start_request.requested_socket_id == std::optional<int>{0} &&
      snapshot.end_request.requested_socket_id == std::optional<int>{0} &&
      snapshot.start_resolved_socket_id == std::optional<int>{0} &&
      snapshot.end_resolved_socket_id == std::optional<int>{0} &&
      curve_view->start_attachment_request.kind == snapshot.start_request.kind &&
      curve_view->end_attachment_request.kind == snapshot.end_request.kind &&
      curve_view->start_attachment_request.requested_socket_id == snapshot.start_request.requested_socket_id &&
      curve_view->end_attachment_request.requested_socket_id == snapshot.end_request.requested_socket_id &&
      curve_view->start_resolved_socket_id == snapshot.start_resolved_socket_id &&
      curve_view->end_resolved_socket_id == snapshot.end_resolved_socket_id;
  if (!ok) {
    std::cerr << "[DBG] C358 seed=" << (snapshot.has_decision_seed ? 1 : 0)
              << " startSource=" << static_cast<int>(snapshot.start_source)
              << " endSource=" << static_cast<int>(snapshot.end_source)
              << " startReq=" << static_cast<int>(snapshot.start_request.kind)
              << " endReq=" << static_cast<int>(snapshot.end_request.kind)
              << " startSocket=" << snapshot.start_resolved_socket_id.value_or(-999)
              << " endSocket=" << snapshot.end_resolved_socket_id.value_or(-999)
              << " curveStart=" << curve_view->start_resolved_socket_id.value_or(-999)
              << " curveEnd=" << curve_view->end_resolved_socket_id.value_or(-999) << "\n";
  }
  return ok;
}

bool test_backbone_geometry_refresh_keeps_materialized_endpoint_socket_resolution() {
  CoreState state;
  ObjectId span_id = wire::core::kInvalidObjectId;
  if (!build_socket_materialization_scene(state, &span_id)) {
    std::cerr << "[DBG] C359 scene_build_failed\n";
    return false;
  }

  MaterializedSocketSnapshot before{};
  if (!collect_materialized_socket_snapshot(state, span_id, &before)) {
    std::cerr << "[DBG] C359 before_missing span=" << span_id << "\n";
    return false;
  }
  const auto before_curve = state.view().inspect_detail_curve(span_id);
  if (!before_curve.has_value()) {
    std::cerr << "[DBG] C359 before_missing_curve span=" << span_id << "\n";
    return false;
  }

  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.curve_samples += 1;
  const auto update = state.UpdateGeometrySettings(settings, true);
  if (!update.ok || !update.value) {
    std::cerr << "[DBG] C359 update_geometry_failed ok=" << (update.ok ? 1 : 0)
              << " changed=" << (update.value ? 1 : 0) << "\n";
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] C359 commit_failed\n";
    return false;
  }

  MaterializedSocketSnapshot after{};
  const auto after_curve = state.view().inspect_detail_curve(span_id);
  const bool ok = collect_materialized_socket_snapshot(state, span_id, &after) &&
                  materialized_socket_snapshot_equal(before, after) && after_curve.has_value() &&
                  detail_curve_socket_state_equal(*before_curve, *after_curve);
  if (!ok) {
    std::cerr << "[DBG] C359 beforeStart=" << before.start_resolved_socket_id.value_or(-999)
              << " afterStart=" << after.start_resolved_socket_id.value_or(-999)
              << " beforeEnd=" << before.end_resolved_socket_id.value_or(-999)
              << " afterEnd=" << after.end_resolved_socket_id.value_or(-999)
              << " beforeSource=" << static_cast<int>(before.start_source)
              << " afterSource=" << static_cast<int>(after.start_source)
              << " curve=" << (after_curve.has_value() ? 1 : 0) << "\n";
  }
  return ok;
}

bool test_backbone_latest_support_orientation_replay_uses_pair_authority() {
  CoreState state;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_replayed_latest_support_orientation_scene(state, &generated_span_ids)) {
    return false;
  }
  const std::unordered_set<ObjectId> generated_span_set(generated_span_ids.begin(), generated_span_ids.end());
  std::map<std::pair<ObjectId, ObjectId>, wire::core::Vec3d> sidebranch_axis_by_pair{};
  auto endpoint_requires_pair_authority = [](const wire::core::SupportLayoutEndpointView& endpoint) {
    return !endpoint.lower_required && endpoint.same_level_feasible &&
           endpoint.continuity_class == wire::core::ContinuityCategoryClass::kPointLike &&
           (endpoint.relation_kind == wire::core::JunctionRelationKind::kSideBranch ||
            endpoint.relation_kind == wire::core::JunctionRelationKind::kThroughMain);
  };
  std::size_t seen_spans = 0;
  for (const auto& span_entry : state.view().spans().items()) {
    if (!generated_span_set.contains(span_entry.id)) {
      continue;
    }
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto validate_endpoint = [&](const wire::core::SupportLayoutEndpointView& endpoint, const char* side_name) {
      if (!endpoint_requires_pair_authority(endpoint)) {
        return true;
      }
      if (!endpoint_uses_pair_authority_without_local_fallback(endpoint) ||
          !pair_authority_axis_family_is_consistent(endpoint)) {
        std::cerr << "[DBG] C344 span=" << span_entry.id << " endpoint=" << side_name
                  << " owner=" << endpoint.owner_pole_id << " relation=" << static_cast<int>(endpoint.relation_kind)
                  << " sideRule=" << static_cast<int>(endpoint.side_assignment_rule)
                  << " orientRule=" << static_cast<int>(endpoint.support_orientation_rule)
                  << " pair=" << endpoint.used_junction_pair_side_assignment
                  << " hasAxis=" << endpoint.has_side_axis << " sign=" << endpoint.chosen_side_sign
                  << " pairPeers=(" << endpoint.support_authority.pair.pair_peer_low << ","
                  << endpoint.support_authority.pair.pair_peer_high << ")\n";
        return false;
      }
      if (endpoint.relation_kind == wire::core::JunctionRelationKind::kSideBranch) {
        if (endpoint.support_orientation_rule != wire::core::SupportOrientationRuleKind::kBisector ||
            endpoint.side_assignment_rule != wire::core::SideAssignmentRuleKind::kBisector) {
          std::cerr << "[DBG] C344 sidebranch_not_bisector span=" << span_entry.id << " endpoint=" << side_name
                    << " owner=" << endpoint.owner_pole_id << " sideRule="
                    << static_cast<int>(endpoint.side_assignment_rule) << " orientRule="
                    << static_cast<int>(endpoint.support_orientation_rule) << "\n";
          return false;
        }
        const auto pair_key =
            std::make_pair(endpoint.support_authority.pair.pair_peer_low, endpoint.support_authority.pair.pair_peer_high);
        const wire::core::Vec3d axis = normalize_xy_safe(endpoint.side_axis);
        const auto axis_it = sidebranch_axis_by_pair.find(pair_key);
        if (axis_it == sidebranch_axis_by_pair.end()) {
          sidebranch_axis_by_pair.emplace(pair_key, axis);
        } else if (std::abs(dot_xy(axis_it->second, axis)) < 0.97) {
          std::cerr << "[DBG] C344 sidebranch_pair_axis_mismatch span=" << span_entry.id << " endpoint=" << side_name
                    << " pair=(" << pair_key.first << "," << pair_key.second << ") ref=(" << axis_it->second.x << ","
                    << axis_it->second.y << ") got=(" << axis.x << "," << axis.y << ")\n";
          return false;
        }
      }
      return true;
    };
    const bool start_requires = endpoint_requires_pair_authority(layout_view->start_endpoint);
    const bool end_requires = endpoint_requires_pair_authority(layout_view->end_endpoint);
    if (!start_requires && !end_requires) {
      continue;
    }
    if (!validate_endpoint(layout_view->start_endpoint, "start") ||
        !validate_endpoint(layout_view->end_endpoint, "end")) {
      return false;
    }
    ++seen_spans;
  }
  if (seen_spans == 0) {
    std::cerr << "[DBG] C344 seen_none\n";
  }
  return seen_spans > 0;
}

bool test_backbone_latest_support_orientation_replay_visual_follows_pair_axis() {
  CoreState state;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_replayed_latest_support_orientation_scene(state, &generated_span_ids)) {
    return false;
  }
  std::set<std::pair<ObjectId, ObjectId>> seen{};
  const std::unordered_set<ObjectId> generated_span_set(generated_span_ids.begin(), generated_span_ids.end());
  auto endpoint_requires_pair_visual = [](const wire::core::SupportLayoutEndpointView& endpoint) {
    return !endpoint.lower_required && endpoint.same_level_feasible &&
           endpoint.continuity_class == wire::core::ContinuityCategoryClass::kPointLike &&
           endpoint_uses_pair_authority_without_local_fallback(endpoint) &&
           endpoint.has_side_axis && std::abs(endpoint.chosen_side_sign) > 1e-9;
  };
  for (const auto& span_entry : state.view().spans().items()) {
    if (!generated_span_set.contains(span_entry.id)) {
      continue;
    }
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    auto validate_endpoint = [&](const wire::core::SupportLayoutEndpointView& endpoint, const char* side_name) {
      if (!endpoint_requires_pair_visual(endpoint)) {
        return true;
      }
      const auto arm = support_arm_part_for_owner(state, span_entry.id, endpoint.owner_pole_id);
      if (!arm.has_value()) {
        std::cerr << "[DBG] C345 missing_arm span=" << span_entry.id << " endpoint=" << side_name
                  << " owner=" << endpoint.owner_pole_id << "\n";
        return false;
      }
      if (!endpoint.has_visual_arm_geometry) {
        std::cerr << "[DBG] C345 missing_visual_arm span=" << span_entry.id << " endpoint=" << side_name
                  << " owner=" << endpoint.owner_pole_id << "\n";
        return false;
      }
      const wire::core::Vec3d actual_axis = helpers::normalize_xy_safe(arm->b - arm->a);
      const wire::core::Vec3d expected_axis =
          helpers::normalize_xy_safe(endpoint.visual_arm_tip_world - endpoint.visual_arm_mount_world);
      const double align = std::abs(helpers::dot_xy(actual_axis, expected_axis));
      const bool mount_matches = almost_equal(arm->a.x, endpoint.visual_arm_mount_world.x, 1e-6) &&
                                 almost_equal(arm->a.y, endpoint.visual_arm_mount_world.y, 1e-6) &&
                                 almost_equal(arm->a.z, endpoint.visual_arm_mount_world.z, 1e-6);
      const bool tip_matches = almost_equal(arm->b.x, endpoint.visual_arm_tip_world.x, 1e-6) &&
                               almost_equal(arm->b.y, endpoint.visual_arm_tip_world.y, 1e-6) &&
                               almost_equal(arm->b.z, endpoint.visual_arm_tip_world.z, 1e-6);
      if (align < 0.97 || !mount_matches || !tip_matches) {
        std::cerr << "[DBG] C345 bad_align span=" << span_entry.id << " endpoint=" << side_name
                  << " owner=" << endpoint.owner_pole_id
                  << " actual=(" << actual_axis.x << "," << actual_axis.y << ")"
                  << " expected=(" << expected_axis.x << "," << expected_axis.y << ")"
                  << " align=" << align << " mountMatch=" << (mount_matches ? 1 : 0)
                  << " tipMatch=" << (tip_matches ? 1 : 0) << "\n";
        return false;
      }
      seen.insert({endpoint.owner_pole_id, span_entry.id});
      return true;
    };
    if (!validate_endpoint(layout_view->start_endpoint, "start") ||
        !validate_endpoint(layout_view->end_endpoint, "end")) {
      return false;
    }
  }

  if (seen.empty()) {
    std::cerr << "[DBG] C345 seen_none\n";
  }
  return !seen.empty();
}

bool test_backbone_latest_capture_t_support_family_does_not_mix_pair_rules() {
  CoreState state;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_replayed_latest_t_support_capture_scene(state, &generated_span_ids)) {
    return false;
  }

  enum class PairRuleFamily { kThroughPairNormal, kBisector };
  struct PairFamilyState {
    bool saw_through_main = false;
    bool saw_side_branch = false;
    std::optional<PairRuleFamily> rule{};
  };

  const std::unordered_set<ObjectId> generated_span_set(generated_span_ids.begin(), generated_span_ids.end());
  std::map<std::pair<ObjectId, ObjectId>, PairFamilyState> pair_state_by_key{};
  auto register_endpoint = [&](ObjectId span_id, const char*, const wire::core::SupportLayoutEndpointView& endpoint) {
    if (!generated_span_set.contains(span_id) || endpoint.lower_required || !endpoint.same_level_feasible ||
        endpoint.continuity_class != wire::core::ContinuityCategoryClass::kPointLike ||
        !endpoint_uses_pair_authority_without_local_fallback(endpoint, false) ||
        (endpoint.relation_kind != wire::core::JunctionRelationKind::kThroughMain &&
         endpoint.relation_kind != wire::core::JunctionRelationKind::kSideBranch)) {
      return true;
    }
    const auto pair_key = std::make_pair(endpoint.support_authority.pair.pair_peer_low,
                                         endpoint.support_authority.pair.pair_peer_high);
    PairFamilyState& pair_state = pair_state_by_key[pair_key];
    pair_state.saw_through_main = pair_state.saw_through_main ||
                                  endpoint.relation_kind == wire::core::JunctionRelationKind::kThroughMain;
    pair_state.saw_side_branch = pair_state.saw_side_branch ||
                                 endpoint.relation_kind == wire::core::JunctionRelationKind::kSideBranch;
    const PairRuleFamily rule_family =
        (endpoint.support_orientation_rule == wire::core::SupportOrientationRuleKind::kBisector &&
         endpoint.side_assignment_rule == wire::core::SideAssignmentRuleKind::kBisector)
            ? PairRuleFamily::kBisector
            : PairRuleFamily::kThroughPairNormal;
    if (!pair_state.rule.has_value()) {
      pair_state.rule = rule_family;
      return true;
    }
    if (*pair_state.rule != rule_family) {
      return false;
    }
    return true;
  };

  for (const auto& span_entry : state.view().spans().items()) {
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    if (!register_endpoint(span_entry.id, "start", layout_view->start_endpoint) ||
        !register_endpoint(span_entry.id, "end", layout_view->end_endpoint)) {
      return false;
    }
  }

  bool saw_t_family = false;
  for (const auto& [pair_key, pair_state] : pair_state_by_key) {
    if (!(pair_state.saw_through_main && pair_state.saw_side_branch)) {
      continue;
    }
    saw_t_family = true;
    if (!pair_state.rule.has_value() || *pair_state.rule != PairRuleFamily::kBisector) {
      return false;
    }
  }
  return saw_t_family;
}

bool test_backbone_latest_capture_pole_orientation_does_not_disagree_with_support_selection() {
  CoreState state;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_replayed_latest_t_support_capture_scene(state, &generated_span_ids)) {
    return false;
  }

  bool saw_main_chain_pair = false;
  for (const auto& pole_entry : state.view().edit_state().poles.items()) {
    const auto pole_view = state.view().inspect_pole(pole_entry.id);
    if (!pole_view.has_value()) {
      continue;
    }
    if (pole_view->support_axis_rule != wire::core::PoleSupportAxisRule::kMainChainPair) {
      continue;
    }
    saw_main_chain_pair = true;
    if (pole_view->forward_rule == wire::core::PoleForwardRule::kFallback) {
      std::cerr << "[DBG] C364 pole_support_mismatch pole=" << pole_entry.id << " forwardRule="
                << static_cast<int>(pole_view->forward_rule) << " supportRule="
                << static_cast<int>(pole_view->support_axis_rule) << " neighbors="
                << pole_view->primary_neighbor_id << "/" << pole_view->secondary_neighbor_id << "\n";
      return false;
    }
  }
  if (!saw_main_chain_pair) {
    std::cerr << "[DBG] C364 no_main_chain_pair_pole_seen\n";
  }
  return saw_main_chain_pair;
}

bool test_backbone_non_grouped_support_visual_uses_endpoint_authority() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(cross).ok) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  for (const auto& span_entry : state.view().spans().items()) {
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->lower_required ||
        endpoint->support_orientation_rule == wire::core::SupportOrientationRuleKind::kRadial ||
        !endpoint->has_side_axis || std::abs(endpoint->chosen_side_sign) <= 1e-9 ||
        !layout_view->lowered_support_groups.empty()) {
      continue;
    }
    const auto arm = support_arm_part_for_owner(state, span_entry.id, center_id);
    if (!arm.has_value()) {
      continue;
    }
    if (!endpoint->has_visual_arm_geometry) {
      std::cerr << "[DBG] C327 missing_visual_arm span=" << span_entry.id << " owner=" << center_id << "\n";
      return false;
    }
    wire::core::Vec3d expected_axis = endpoint->visual_arm_tip_world - endpoint->visual_arm_mount_world;
    const wire::core::Vec3d expected = normalize_xy_safe(expected_axis);
    const wire::core::Vec3d actual = normalize_xy_safe(arm->b - arm->a);
    const double align = std::abs(dot_xy(actual, expected));
    const bool mount_matches = almost_equal(arm->a.x, endpoint->visual_arm_mount_world.x, 1e-6) &&
                               almost_equal(arm->a.y, endpoint->visual_arm_mount_world.y, 1e-6) &&
                               almost_equal(arm->a.z, endpoint->visual_arm_mount_world.z, 1e-6);
    const bool tip_matches = almost_equal(arm->b.x, endpoint->visual_arm_tip_world.x, 1e-6) &&
                             almost_equal(arm->b.y, endpoint->visual_arm_tip_world.y, 1e-6) &&
                             almost_equal(arm->b.z, endpoint->visual_arm_tip_world.z, 1e-6);
    if (align < 0.97 || !mount_matches || !tip_matches) {
      std::cerr << "[DBG] C327 span=" << span_entry.id << " align=" << align << " expected=(" << expected.x << ","
                << expected.y << ") actual=(" << actual.x << "," << actual.y << ") endpoint=("
                << endpoint->endpoint_world.x << "," << endpoint->endpoint_world.y << "," << endpoint->endpoint_world.z
                << ") armA=(" << arm->a.x << "," << arm->a.y << "," << arm->a.z << ") armB=(" << arm->b.x << ","
                << arm->b.y << "," << arm->b.z << ") mountMatch=" << (mount_matches ? 1 : 0)
                << " tipMatch=" << (tip_matches ? 1 : 0) << "\n";
    }
    return align >= 0.97 && mount_matches && tip_matches;
  }
  std::cerr << "[DBG] C327 no_same_level_non_grouped_arm\n";
  return false;
}

bool test_backbone_same_level_terminal_endpoint_does_not_invent_side_sign() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C335 generate_failed ok=" << generated.ok
              << " count=" << generated.value.generated_span_ids.size() << "\n";
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] C335 commit_failed\n";
    return false;
  }

  const ObjectId tip_id = find_pole_id_by_position(state, {12.0, 0.0, 0.0});
  if (tip_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C335 tip_missing\n";
    return false;
  }

  bool saw_endpoint = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    const wire::core::Bundle* bundle =
        (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kLowVoltage) {
      continue;
    }
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, tip_id);
    if (!endpoint.has_value() || endpoint->lower_required ||
        endpoint->continuity_class != wire::core::ContinuityCategoryClass::kPointLike ||
        endpoint->relation_kind != wire::core::JunctionRelationKind::kThroughMain) {
      continue;
    }
    saw_endpoint = true;
    if (endpoint->used_junction_pair_side_assignment || std::abs(endpoint->chosen_side_sign) > 1e-9) {
      std::cerr << "[DBG] C335 span=" << span_id << " pairSide=" << endpoint->used_junction_pair_side_assignment
                << " sign=" << endpoint->chosen_side_sign << " rule="
                << static_cast<int>(endpoint->side_assignment_rule) << " orient="
                << static_cast<int>(endpoint->support_orientation_rule) << "\n";
      return false;
    }
  }
  if (!saw_endpoint) {
    std::cerr << "[DBG] C335 no_terminal_endpoint\n";
  }
  return saw_endpoint;
}

bool endpoint_uses_pair_authority_without_local_fallback(const wire::core::SupportLayoutEndpointView& endpoint,
                                                         bool require_side_axis) {
  const bool has_pair_peers =
      endpoint.support_authority.pair.pair_peer_low != wire::core::kInvalidObjectId &&
      endpoint.support_authority.pair.pair_peer_high != wire::core::kInvalidObjectId &&
      endpoint.support_authority.pair.pair_peer_low != endpoint.support_authority.pair.pair_peer_high;
  const bool pair_owned_through =
      endpoint.support_orientation_rule == wire::core::SupportOrientationRuleKind::kThroughPairNormal &&
      endpoint.side_assignment_rule == wire::core::SideAssignmentRuleKind::kThroughPairNormal;
  const bool pair_owned_bisector =
      endpoint.support_orientation_rule == wire::core::SupportOrientationRuleKind::kBisector &&
      endpoint.side_assignment_rule == wire::core::SideAssignmentRuleKind::kBisector;
  return (pair_owned_through || pair_owned_bisector) && endpoint.used_junction_pair_side_assignment && has_pair_peers &&
         (!require_side_axis || endpoint.has_side_axis);
}

bool pair_authority_axis_family_is_consistent(const wire::core::SupportLayoutEndpointView& endpoint) {
  if (!endpoint_uses_pair_authority_without_local_fallback(endpoint)) {
    return false;
  }
  if (!endpoint.has_signed_support_axis) {
    return true;
  }
  const wire::core::Vec3d unsigned_axis = normalize_xy_safe(endpoint.side_axis);
  const wire::core::Vec3d signed_axis = normalize_xy_safe(endpoint.signed_support_axis);
  return std::abs(dot_xy(unsigned_axis, signed_axis)) >= 0.97;
}

bool test_backbone_capture_like_branch_terminal_borrows_peer_pair_authority() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C336 missing CommunicationPole\n";
    return false;
  }

  constexpr wire::core::Vec3d kTrunkPrev{-3.4062, 14.9913, 0.0};
  constexpr wire::core::Vec3d kCenter{-7.8911, 4.47279, 0.0};
  constexpr wire::core::Vec3d kTrunkNext{-10.3717, -1.47687, 0.0};
  constexpr wire::core::Vec3d kBranch{-0.389464, 3.90565, 0.0};

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {kTrunkPrev, kCenter, kTrunkNext};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = pole_type_id;
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(trunk, wire::core::BundleKind::kOptical);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    std::cerr << "[DBG] C336 trunk_generate_failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, kCenter, 1e-4);
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C336 center_missing\n";
    return false;
  }
  const auto center_junction = state.view().inspect_junction(center_id);
  if (!center_junction.has_value() || !center_junction->through_pair_accepted) {
    std::cerr << "[DBG] C336 center_pair_missing\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {kCenter, kBranch};
  branch.interval_m = 1000.0;
  branch.pole_type_id = pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(branch, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C336 branch_generate_failed ok=" << generated.ok
              << " count=" << generated.value.generated_span_ids.size() << "\n";
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] C336 commit_failed\n";
    return false;
  }

  const ObjectId tip_id = find_pole_id_by_position(state, kBranch, 1e-4);
  if (tip_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C336 tip_missing\n";
    return false;
  }
  bool saw_endpoint = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    const wire::core::Bundle* bundle =
        (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
    if (bundle == nullptr ||
        (bundle->bundle_template_id != wire::core::BundleKind::kCommunication &&
         bundle->bundle_template_id != wire::core::BundleKind::kOptical)) {
      continue;
    }
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, tip_id);
    if (!endpoint.has_value()) {
      continue;
    }
    if (endpoint->relation_kind != wire::core::JunctionRelationKind::kThroughMain || endpoint->lower_required ||
        !endpoint->same_level_feasible ||
        endpoint->continuity_class != wire::core::ContinuityCategoryClass::kPointLike) {
      continue;
    }
    saw_endpoint = true;
    if (!endpoint_uses_pair_authority_without_local_fallback(*endpoint)) {
      std::cerr << "[DBG] C336 span=" << span_id
                << " bundleTemplate=" << (bundle ? static_cast<int>(bundle->bundle_template_id) : -1)
                << " class=" << static_cast<int>(endpoint->continuity_class)
                << " sameLevel=" << endpoint->same_level_feasible
                << " lower=" << endpoint->lower_required
                << " relation=" << static_cast<int>(endpoint->relation_kind)
                << " sideRule=" << static_cast<int>(endpoint->side_assignment_rule)
                << " orientRule=" << static_cast<int>(endpoint->support_orientation_rule)
                << " pairSide=" << endpoint->used_junction_pair_side_assignment
                << " hasAxis=" << endpoint->has_side_axis << " sign=" << endpoint->chosen_side_sign
                << " pairPeers=(" << endpoint->support_authority.pair.pair_peer_low << ","
                << endpoint->support_authority.pair.pair_peer_high << ")"
                << " basis=" << static_cast<int>(endpoint->support_orientation_basis) << "\n";
      return false;
    }
  }

  if (!saw_endpoint) {
    std::cerr << "[DBG] C336 no_branch_terminal_through_main_endpoint\n";
  }
  return saw_endpoint;
}

bool test_backbone_capture_like_branch_terminal_visual_uses_borrowed_pair_axis() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C337 missing CommunicationPole\n";
    return false;
  }

  constexpr wire::core::Vec3d kTrunkPrev{-3.4062, 14.9913, 0.0};
  constexpr wire::core::Vec3d kCenter{-7.8911, 4.47279, 0.0};
  constexpr wire::core::Vec3d kTrunkNext{-10.3717, -1.47687, 0.0};
  constexpr wire::core::Vec3d kBranch{-0.389464, 3.90565, 0.0};

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {kTrunkPrev, kCenter, kTrunkNext};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = pole_type_id;
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(trunk, wire::core::BundleKind::kOptical);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    std::cerr << "[DBG] C337 trunk_generate_failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, kCenter, 1e-4);
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C337 center_missing\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {kCenter, kBranch};
  branch.interval_m = 1000.0;
  branch.pole_type_id = pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(branch, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C337 branch_generate_failed ok=" << generated.ok
              << " count=" << generated.value.generated_span_ids.size() << "\n";
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] C337 commit_failed\n";
    return false;
  }

  const ObjectId tip_id = find_pole_id_by_position(state, kBranch, 1e-4);
  if (tip_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C337 tip_missing\n";
    return false;
  }

  const auto center_view = state.view().inspect_pole(center_id);
  const auto tip_view = state.view().inspect_pole(tip_id);
  if (!center_view.has_value() || !center_view->has_support_axis || !tip_view.has_value() || !tip_view->has_support_axis) {
    std::cerr << "[DBG] C337 pole_view_missing centerHas=" << (center_view.has_value() && center_view->has_support_axis)
              << " tipHas=" << (tip_view.has_value() && tip_view->has_support_axis) << "\n";
    return false;
  }
  const wire::core::Vec3d center_axis = normalize_xy_safe(center_view->support_axis_dir);
  const wire::core::Vec3d tip_axis = normalize_xy_safe(tip_view->support_axis_dir);
  const double align = std::abs(dot_xy(center_axis, tip_axis));
  const bool ok = tip_view->support_axis_rule != wire::core::PoleSupportAxisRule::kConnectedDirectionFit && align >= 0.95;
  if (!ok) {
    std::cerr << "[DBG] C337 centerRule=" << static_cast<int>(center_view->support_axis_rule)
              << " centerAxis=(" << center_axis.x << "," << center_axis.y << ") tipRule="
              << static_cast<int>(tip_view->support_axis_rule) << " tipAxis=(" << tip_axis.x << "," << tip_axis.y
              << ") align=" << align << "\n";
  }
  return ok;
}

bool build_latest_capture_like_lowered_branch_scene(CoreState& state, PoleTypeId pole_type_id, ObjectId* out_center_id,
                                                    ObjectId* out_tip_id, std::vector<ObjectId>* out_span_ids) {
  constexpr wire::core::Vec3d kTrunkPrev{-14.495, -14.313, 0.0};
  constexpr wire::core::Vec3d kCenter{-6.94429, -20.313, 0.0};
  constexpr wire::core::Vec3d kTrunkNext{-1.31427, -20.6523, 0.0};
  constexpr wire::core::Vec3d kBranchTip{-7.43528, -26.4492, 0.0};

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {kTrunkPrev, kCenter, kTrunkNext};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = pole_type_id;
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(trunk, wire::core::BundleKind::kOptical);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, kCenter, 1e-4);
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {kCenter, kBranchTip};
  branch.interval_m = 7.15577;
  branch.pole_type_id = pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
  add_backbone_bundle(branch, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId tip_id = find_pole_id_by_position(state, kBranchTip, 1e-4);
  if (tip_id == wire::core::kInvalidObjectId) {
    return false;
  }
  if (out_center_id != nullptr) {
    *out_center_id = center_id;
  }
  if (out_tip_id != nullptr) {
    *out_tip_id = tip_id;
  }
  if (out_span_ids != nullptr) {
    *out_span_ids = generated.value.generated_span_ids;
  }
  return true;
}

bool build_replayed_latest_capture_scene(CoreState& state, ObjectId* out_tip_id, std::vector<ObjectId>* out_span_ids) {
  const std::filesystem::path capture_path = std::filesystem::path("captures") / "drawpath_repro_20260403_192731.txt";
  wire::core::BackboneSpec replay_request{};
  std::string error{};
  if (!restore_capture_request_scene(capture_path, state, &replay_request, &error)) {
    std::cerr << "[DBG] replay_restore_failed error=" << error << "\n";
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(replay_request);
  if (!generated.ok || generated.value.generated_span_ids.empty() || generated.value.generated_pole_ids.empty()) {
    std::cerr << "[DBG] replay_generate_failed ok=" << generated.ok << " error=" << generated.error << "\n";
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] replay_commit_failed\n";
    return false;
  }
  if (out_tip_id != nullptr) {
    *out_tip_id = generated.value.generated_pole_ids.front();
  }
  if (out_span_ids != nullptr) {
    *out_span_ids = generated.value.generated_span_ids;
  }
  return true;
}

bool build_replayed_latest_cross_capture_scene(CoreState& state, std::vector<ObjectId>* out_span_ids) {
  const std::filesystem::path capture_path = std::filesystem::path("captures") / "drawpath_repro_20260403_212228.txt";
  wire::core::BackboneSpec replay_request{};
  std::string error{};
  if (!restore_capture_request_scene(capture_path, state, &replay_request, &error)) {
    std::cerr << "[DBG] replay_cross_restore_failed error=" << error << "\n";
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(replay_request);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] replay_cross_generate_failed ok=" << generated.ok << " error=" << generated.error << "\n";
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] replay_cross_commit_failed\n";
    return false;
  }
  if (out_span_ids != nullptr) {
    *out_span_ids = generated.value.generated_span_ids;
  }
  return true;
}

bool build_replayed_latest_support_orientation_scene(CoreState& state, std::vector<ObjectId>* out_generated_span_ids) {
  const std::filesystem::path capture_path = std::filesystem::path("captures") / "drawpath_repro_20260404_230323.txt";
  wire::core::BackboneSpec replay_request{};
  std::string error{};
  if (!restore_capture_request_scene(capture_path, state, &replay_request, &error)) {
    std::cerr << "[DBG] replay_support_orientation_restore_failed error=" << error << "\n";
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(replay_request);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] replay_support_orientation_generate_failed ok=" << generated.ok
              << " error=" << generated.error << "\n";
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] replay_support_orientation_commit_failed\n";
    return false;
  }
  if (out_generated_span_ids != nullptr) {
    *out_generated_span_ids = generated.value.generated_span_ids;
  }
  return true;
}

bool build_replayed_latest_t_support_capture_scene(CoreState& state, std::vector<ObjectId>* out_generated_span_ids) {
  const std::filesystem::path capture_path = std::filesystem::path("captures") / "drawpath_repro_20260405_194034.txt";
  wire::core::BackboneSpec replay_request{};
  std::string error{};
  if (!restore_capture_request_scene(capture_path, state, &replay_request, &error)) {
    std::cerr << "[DBG] replay_latest_t_restore_failed error=" << error << "\n";
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(replay_request);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] replay_latest_t_generate_failed ok=" << generated.ok << " error=" << generated.error << "\n";
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] replay_latest_t_commit_failed\n";
    return false;
  }
  if (out_generated_span_ids != nullptr) {
    *out_generated_span_ids = generated.value.generated_span_ids;
  }
  return true;
}

bool build_minimal_latest_pair_cross_capture_scene(CoreState& state, ObjectId* out_center_id,
                                                   std::unordered_map<ObjectId, ObjectId>* out_remapped_ids,
                                                   std::vector<ObjectId>* out_generated_span_ids) {
  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = pole_type.id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] minimal_pair_cross missing_communication_pole_type\n";
    return false;
  }

  struct PoleSeed {
    ObjectId capture_id;
    wire::core::Vec3d position;
  };
  const std::array<PoleSeed, 5> seeds = {{
      {73, {-13.2912, 4.35985, 0.0}},
      {434, {-5.08433, 7.20028, 0.0}},
      {446, {-9.98962, -0.936008, 0.0}},
      {477, {-19.5972, 1.10245, 0.0}},
      {489, {-16.3878, 9.44052, 0.0}},
  }};

  std::unordered_map<ObjectId, ObjectId> remapped{};
  for (const PoleSeed& seed : seeds) {
    wire::core::Transformd tf{};
    tf.position = seed.position;
    const auto add = state.AddPole(tf, 10.0, "ReplayPole", wire::core::PoleKind::kConcrete,
                                   wire::core::PlacementMode::kManual);
    if (!add.ok) {
      std::cerr << "[DBG] minimal_pair_cross add_pole_failed captureId=" << seed.capture_id
                << " error=" << add.error << "\n";
      return false;
    }
    if (!state.ApplyPoleType(add.value, communication_pole_type_id).ok) {
      std::cerr << "[DBG] minimal_pair_cross apply_type_failed captureId=" << seed.capture_id << "\n";
      return false;
    }
    remapped[seed.capture_id] = add.value;
  }

  wire::core::BackboneSpec existing_req{};
  existing_req.path.polyline = {seeds[1].position, seeds[0].position, seeds[2].position};
  existing_req.interval_m = 1000.0;
  existing_req.pole_type_id = communication_pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec existing_shared{};
  existing_shared.point_index = 1;
  existing_shared.support_kind = wire::core::SupportKind::kPole;
  existing_shared.node_id = remapped.at(73);
  existing_req.path.node_specs.push_back(existing_shared);
  add_backbone_bundle(existing_req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(existing_req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(existing_req, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(existing_req, wire::core::BundleKind::kOptical);
  const auto existing_generated = state.GenerateFromBackboneSpec(existing_req);
  if (!existing_generated.ok || existing_generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] minimal_pair_cross existing_generate_failed ok=" << existing_generated.ok
              << " error=" << existing_generated.error
              << " spanCount=" << existing_generated.value.generated_span_ids.size() << "\n";
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {seeds[3].position, seeds[0].position, seeds[4].position};
  req.interval_m = 14.0475;
  req.pole_type_id = communication_pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = remapped.at(73);
  req.path.node_specs.push_back(shared);
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(req, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] minimal_pair_cross generate_failed ok=" << generated.ok
              << " error=" << generated.error << " spanCount=" << generated.value.generated_span_ids.size() << "\n";
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] minimal_pair_cross commit_failed\n";
    return false;
  }

  if (out_center_id != nullptr) {
    *out_center_id = remapped.at(73);
  }
  if (out_remapped_ids != nullptr) {
    *out_remapped_ids = remapped;
  }
  if (out_generated_span_ids != nullptr) {
    *out_generated_span_ids = generated.value.generated_span_ids;
  }
  return true;
}

bool test_backbone_capture_like_lowered_branch_main_endpoint_borrows_peer_pair_authority() {
  CoreState state;
  ObjectId tip_id = wire::core::kInvalidObjectId;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_replayed_latest_capture_scene(state, &tip_id, &generated_span_ids)) {
    std::cerr << "[DBG] C338 scene_build_failed\n";
    return false;
  }

  bool saw_endpoint = false;
  bool found_shift = false;
  bool found_lateral_visual = false;
  for (ObjectId span_id : generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    const wire::core::Bundle* bundle =
        (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, tip_id);
    if (!endpoint.has_value()) {
      continue;
    }
    const auto assignment = find_assignment_for_span(state, span_id);
    if (!assignment.has_value() ||
        assignment->decision_b.owner_pole_id != tip_id ||
        assignment->decision_b.relation_kind != wire::core::JunctionRelationKind::kThroughMain ||
        assignment->decision_b.continuity_class != wire::core::ContinuityCategoryClass::kBundleLike) {
      continue;
    }
    saw_endpoint = true;
    if (!endpoint_uses_pair_authority_without_local_fallback(*endpoint)) {
      std::cerr << "[DBG] C338 span=" << span_id
                << " sideRule=" << static_cast<int>(endpoint->side_assignment_rule)
                << " orientRule=" << static_cast<int>(endpoint->support_orientation_rule)
                << " pairSide=" << endpoint->used_junction_pair_side_assignment
                << " hasAxis=" << endpoint->has_side_axis << " sign=" << endpoint->chosen_side_sign
                << " lower=" << endpoint->lower_required
                << " sameLevel=" << endpoint->same_level_feasible
                << " class=" << static_cast<int>(endpoint->continuity_class)
                << " relation=" << static_cast<int>(endpoint->relation_kind) << " inPair="
                << assignment->decision_b.in_through_pair << " tipId=" << tip_id
                << " pairPeers=(" << endpoint->support_authority.pair.pair_peer_low << ","
                << endpoint->support_authority.pair.pair_peer_high << ")\n";
      return false;
    }
  }

  if (!saw_endpoint) {
    for (ObjectId span_id : generated_span_ids) {
      const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
      const wire::core::Bundle* bundle =
          (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
      const auto layout_view = state.view().inspect_support_layout(span_id);
      std::cerr << "[DBG] C338 scene span=" << span_id
                << " bundle=" << (bundle ? static_cast<int>(bundle->bundle_template_id) : -1);
      if (!layout_view.has_value()) {
        std::cerr << " no_layout\n";
        continue;
      }
      const auto tip_endpoint = layout_endpoint_for_owner(*layout_view, tip_id);
      if (!tip_endpoint.has_value()) {
        std::cerr << " no_tip_endpoint\n";
        continue;
      }
      std::cerr << " relation=" << static_cast<int>(tip_endpoint->relation_kind)
                << " class=" << static_cast<int>(tip_endpoint->continuity_class)
                << " lower=" << tip_endpoint->lower_required
                << " same=" << tip_endpoint->same_level_feasible
                << " sideRule=" << static_cast<int>(tip_endpoint->side_assignment_rule)
                << " orientRule=" << static_cast<int>(tip_endpoint->support_orientation_rule)
                << " pair=" << tip_endpoint->used_junction_pair_side_assignment
                << " sign=" << tip_endpoint->chosen_side_sign << "\n";
    }
    std::cerr << "[DBG] C338 no_bundle_through_main_tip_endpoint\n";
  }
  return saw_endpoint;
}

bool test_backbone_capture_like_lowered_branch_main_visual_uses_borrowed_pair_axis() {
  CoreState state;
  ObjectId tip_id = wire::core::kInvalidObjectId;
  std::vector<ObjectId> generated_span_ids{};
  if (!build_replayed_latest_capture_scene(state, &tip_id, &generated_span_ids)) {
    std::cerr << "[DBG] C339 scene_build_failed\n";
    return false;
  }

  const wire::core::Pole* tip_pole = state.view().edit_state().poles.find(tip_id);
  if (tip_pole == nullptr) {
    return false;
  }

  std::vector<GroupedRankFamilySnapshot> rank_families{};
  if (collect_grouped_rank_family_snapshots(state, tip_id, &rank_families) && !rank_families.empty()) {
    const bool all_families_consistent = std::all_of(rank_families.begin(), rank_families.end(),
                                                     [](const GroupedRankFamilySnapshot& family) {
                                                       return grouped_rank_family_has_consistent_owner_and_endpoints(family);
                                                     });
    if (!all_families_consistent) {
      std::cerr << "[DBG] C339 no_consistent_rank_family count=" << rank_families.size() << "\n";
      return false;
    }
  }

  bool saw_endpoint = false;
  bool found_bad_visual = false;
  for (ObjectId span_id : generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    const wire::core::Bundle* bundle =
        (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, tip_id);
    const auto assignment = find_assignment_for_span(state, span_id);
    if (!endpoint.has_value() || !assignment.has_value() ||
        assignment->decision_b.owner_pole_id != tip_id ||
        assignment->decision_b.relation_kind != wire::core::JunctionRelationKind::kThroughMain ||
        assignment->decision_b.continuity_class != wire::core::ContinuityCategoryClass::kBundleLike ||
        endpoint->support_orientation_rule == wire::core::SupportOrientationRuleKind::kRadial ||
        !endpoint->has_side_axis || std::abs(endpoint->chosen_side_sign) <= 1e-9) {
      continue;
    }
    saw_endpoint = true;
    const wire::core::Port* port = state.view().edit_state().ports.find(endpoint->port_id);
    if (port == nullptr) {
      continue;
    }
    const double dx = port->world_position.x - tip_pole->world_transform.position.x;
    const double dy = port->world_position.y - tip_pole->world_transform.position.y;
    const double planar = std::sqrt(dx * dx + dy * dy);
    if (port->template_side == wire::core::SlotSide::kCenter ||
        planar <= state.view().visual_settings().support_center_threshold_m + 1e-9) {
      continue;
    }
      const double expected_x = dx / planar;
      const double expected_y = dy / planar;
      bool saw_support_arm = false;
      if (const wire::core::SpanVisualCacheEntry* visual = state.view().find_span_visual_cache(span_id);
          visual != nullptr) {
        for (const auto& part : visual->parts) {
          if (part.kind != wire::core::VisualPartKind::kSupportArm) {
            continue;
          }
          const bool anchored_at_tip =
              almost_equal(part.a.x, tip_pole->world_transform.position.x, 1e-6) &&
              almost_equal(part.a.y, tip_pole->world_transform.position.y, 1e-6);
          if (!anchored_at_tip) {
            continue;
          }
          saw_support_arm = true;
          const double arm_dx = part.b.x - part.a.x;
          const double arm_dy = part.b.y - part.a.y;
          const double arm_len = std::sqrt(arm_dx * arm_dx + arm_dy * arm_dy);
          if (arm_len <= 1e-9) {
          std::cerr << "[DBG] C339 zero_arm span=" << span_id << "\n";
          found_bad_visual = true;
          continue;
        }
        const double arm_x = arm_dx / arm_len;
        const double arm_y = arm_dy / arm_len;
          const double align = arm_x * expected_x + arm_y * expected_y;
          if (align < 0.98) {
            std::cerr << "[DBG] C339 bad_visual span=" << span_id << " align=" << align << " expected=("
                      << expected_x << "," << expected_y << ") actual=(" << arm_x << "," << arm_y << ")"
                      << " port=(" << port->world_position.x << "," << port->world_position.y << ","
                      << port->world_position.z << ") armB=(" << part.b.x << "," << part.b.y << "," << part.b.z
                      << ")\n";
            found_bad_visual = true;
          }
        }
      }
    if (!saw_support_arm) {
      std::cerr << "[DBG] C339 missing_arm span=" << span_id << " portId=" << port->id << "\n";
      found_bad_visual = true;
    }
  }

  if (!saw_endpoint) {
    std::cerr << "[DBG] C339 no_bundle_through_main_tip_endpoint\n";
  }
  return saw_endpoint && !found_bad_visual;
}

bool test_backbone_non_lowered_cross_spans_do_not_expose_lowered_support_groups() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  bool saw_lowered_cross = false;
  bool saw_non_lowered_center_span = false;
  for (const auto& span_entry : state.view().spans().items()) {
    const auto* span = state.view().edit_state().spans.find(span_entry.id);
    if (span == nullptr || (span->endpoint_node_a_id != center_id && span->endpoint_node_b_id != center_id)) {
      continue;
    }
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value()) {
      continue;
    }
    if (endpoint->lower_required) {
      saw_lowered_cross = true;
      continue;
    }
    saw_non_lowered_center_span = true;
    if (!layout_view->lowered_support_groups.empty()) {
      std::cerr << "[DBG] C267 span=" << span_entry.id
                << " lowerRequired=" << endpoint->lower_required
                << " groupCount=" << layout_view->lowered_support_groups.size()
                << " relationA=" << static_cast<int>(layout_view->relation_a)
                << " relationB=" << static_cast<int>(layout_view->relation_b) << "\n";
      return false;
    }
  }

  return saw_lowered_cross && saw_non_lowered_center_span;
}

bool test_backbone_non_lowered_spans_do_not_inherit_acute_corner_support_groups() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {
      {-18.59678, 11.0534, 0.0},
      {-6.59678, 11.0534, 0.0},
      {5.40322, 11.0534, 0.0},
  };
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = pole_type_id;
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(trunk, wire::core::BundleKind::kOptical);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId root_id = find_pole_id_by_position(state, {-6.59678, 11.0534, 0.0}, 1e-4);
  if (root_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {
      {-6.59678, 11.0534, 0.0},
      {-4.93216, 6.7054, 0.0},
      {-13.6709, -1.24875, 0.0},
      {-8.49996, -0.441201, 0.0},
  };
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = root_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 8.0;
  branch.pole_type_id = pole_type_id;
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(branch, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  bool saw_lowered = false;
  bool saw_flat_after_lowered = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto span_view = state.view().inspect_span(span_id);
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!span_view.has_value() || !layout_view.has_value()) {
      continue;
    }
    const auto* span = state.view().edit_state().spans.find(span_id);
    const auto* bundle = (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    const bool has_lowered_endpoint =
        layout_view->start_endpoint.lower_required || layout_view->end_endpoint.lower_required;
    if (has_lowered_endpoint) {
      saw_lowered = true;
      continue;
    }
    saw_flat_after_lowered = true;
      if (!layout_view->lowered_support_groups.empty()) {
        std::cerr << "[DBG] C268 span=" << span_id
                  << " groupCount=" << layout_view->lowered_support_groups.size()
                  << " flow=" << static_cast<int>(span_view->flow_kind)
                  << " startLower=" << layout_view->start_endpoint.lower_required
                  << " endLower=" << layout_view->end_endpoint.lower_required << "\n";
        return false;
      }
  }

  return saw_lowered && saw_flat_after_lowered;
}

bool test_backbone_refresh_keeps_lowered_side_and_orientation_origin() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  if (!state.SetPoleManualYawOverride(center_id, 15.0).ok) {
    return false;
  }

  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value()) {
      continue;
    }
    if (layout_view->lowering_kind == wire::core::BackboneLoweringKind::kCrossUnderpass &&
        endpoint->support_orientation_rule != wire::core::SupportOrientationRuleKind::kRadial &&
        endpoint->has_side_axis &&
        std::abs(endpoint->chosen_side_sign) > 0.5) {
      return true;
    }
  }
  return false;
}

bool test_backbone_hv3_same_level_order_decision_is_permutable() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {15.2954, -10.2356, 0.0},
      {2.87944, -2.16948, 0.0},
      {12.0546, -16.1829, 0.0},
      {-2.43956, -5.96523, 0.0},
      {-14.3612, -6.28665, 0.0},
      {-14.1685, 6.36328, 0.0},
      {-2.66804, 5.17324, 0.0},
      {-13.2896, 13.4663, 0.0},
      {-20.9067, 19.9314, 0.0},
      {-24.7745, 14.455, 0.0},
  };
  req.interval_m = 8.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  bool saw_permutable = false;
  bool saw_compared_choice = false;
  for (const auto& orientation : state.view().last_generation_edge_orientations()) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    saw_permutable = saw_permutable ||
                     orientation.order_decision_policy == wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous;
    const bool compared_a = orientation.order_decision_choice_reason_a != wire::core::OrderDecisionChoiceReason::kFixedOrder;
    const bool compared_b = orientation.order_decision_choice_reason_b != wire::core::OrderDecisionChoiceReason::kFixedOrder;
    saw_compared_choice = saw_compared_choice || compared_a || compared_b;
  }
  if (!(saw_permutable && saw_compared_choice)) {
    std::cerr << "[DBG] C239 permutable=" << saw_permutable << " compared=" << saw_compared_choice << "\n";
  }
  return saw_permutable && saw_compared_choice;
}

bool test_backbone_hv3_lowered_order_decision_is_permutable() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  bool saw_lowered = false;
  bool saw_permutable = false;
  bool saw_compared_choice = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value() ||
        layout_view->lowering_kind != wire::core::BackboneLoweringKind::kCrossUnderpass) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value()) {
      continue;
    }
    saw_lowered = true;
    saw_permutable = saw_permutable ||
                     endpoint->order_decision_policy == wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous;
    saw_compared_choice = saw_compared_choice ||
                          endpoint->order_decision_choice_reason != wire::core::OrderDecisionChoiceReason::kFixedOrder;
  }
  if (!(saw_lowered && saw_permutable && saw_compared_choice)) {
    std::cerr << "[DBG] C240 lowered=" << saw_lowered << " permutable=" << saw_permutable
              << " compared=" << saw_compared_choice << "\n";
  }
  return saw_lowered && saw_permutable && saw_compared_choice;
}

bool test_backbone_fixed_order_bundle_skips_permutation() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  auto& bundle_templates = wire::core::CoreStateTestHook::bundle_templates(state);
  const auto it = bundle_templates.find(wire::core::BundleKind::kHighVoltage);
  if (it == bundle_templates.end()) {
    return false;
  }
  it->second.order_decision_policy = wire::core::OrderDecisionPolicyKind::kFixedOrder;

  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {15.2954, -10.2356, 0.0},
      {2.87944, -2.16948, 0.0},
      {12.0546, -16.1829, 0.0},
      {-2.43956, -5.96523, 0.0},
      {-14.3612, -6.28665, 0.0},
      {-14.1685, 6.36328, 0.0},
      {-2.66804, 5.17324, 0.0},
      {-13.2896, 13.4663, 0.0},
      {-20.9067, 19.9314, 0.0},
      {-24.7745, 14.455, 0.0},
  };
  req.interval_m = 8.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  bool saw_hv = false;
  for (const auto& orientation : state.view().last_generation_edge_orientations()) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    saw_hv = true;
    if (orientation.order_decision_policy != wire::core::OrderDecisionPolicyKind::kFixedOrder ||
        orientation.order_decision_choice_reason_a != wire::core::OrderDecisionChoiceReason::kFixedOrder ||
        orientation.order_decision_choice_reason_b != wire::core::OrderDecisionChoiceReason::kFixedOrder) {
      std::cerr << "[DBG] C241 policy=" << static_cast<int>(orientation.order_decision_policy)
                << " choiceA=" << static_cast<int>(orientation.order_decision_choice_a)
                << " reasonA=" << static_cast<int>(orientation.order_decision_choice_reason_a)
                << " choiceB=" << static_cast<int>(orientation.order_decision_choice_b)
                << " reasonB=" << static_cast<int>(orientation.order_decision_choice_reason_b) << "\n";
      return false;
    }
  }
  return saw_hv;
}

bool test_backbone_refresh_keeps_order_decision_choice() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId span_id = generated.value.generated_span_ids.front();
  const auto before = state.view().inspect_support_layout(span_id);
  if (!before.has_value()) {
    return false;
  }
  const auto before_endpoint = layout_endpoint_for_owner(*before, center_id);
  if (!before_endpoint.has_value()) {
    return false;
  }
  const auto before_choice = before_endpoint->order_decision_choice;
  const auto before_reason = before_endpoint->order_decision_choice_reason;
  const bool before_has_seed = before->has_decision_seed;
  const bool before_requires_seed = before->requires_decision_seed;

  if (!state.SetPoleManualYawOverride(center_id, 15.0).ok) {
    return false;
  }

  const auto after = state.view().inspect_support_layout(span_id);
  if (!after.has_value()) {
    return false;
  }
  const auto after_endpoint = layout_endpoint_for_owner(*after, center_id);
  return after_endpoint.has_value() &&
         before_has_seed && before_requires_seed &&
         after->has_decision_seed == before_has_seed &&
         after->requires_decision_seed == before_requires_seed &&
         before_endpoint->order_decision_policy == wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous &&
         after_endpoint->order_decision_policy == wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous &&
         after_endpoint->order_decision_choice == before_choice &&
         after_endpoint->order_decision_choice_reason == before_reason;
}

bool test_backbone_point_like_order_decision_non_regression() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 4.0, 0.0}};
  req.interval_m = 6.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  bool saw_lv = false;
  for (const auto& orientation : state.view().last_generation_edge_orientations()) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kLowVoltage) {
      continue;
    }
    saw_lv = true;
    if (orientation.order_decision_policy != wire::core::OrderDecisionPolicyKind::kFixedOrder ||
        orientation.order_decision_choice_reason_a != wire::core::OrderDecisionChoiceReason::kFixedOrder ||
        orientation.order_decision_choice_reason_b != wire::core::OrderDecisionChoiceReason::kFixedOrder) {
      std::cerr << "[DBG] C243 unexpected point-like bundle order policy=" << static_cast<int>(orientation.order_decision_policy)
                << " reasonA=" << static_cast<int>(orientation.order_decision_choice_reason_a)
                << " reasonB=" << static_cast<int>(orientation.order_decision_choice_reason_b) << "\n";
      return false;
    }
  }
  return saw_lv;
}

bool test_backbone_authoritative_endpoint_decision_matches_support_layout() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId span_id = generated.value.generated_span_ids.front();
  const auto layout_view = state.view().inspect_support_layout(span_id);
  const auto assignment = find_assignment_for_span(state, span_id);
  if (!layout_view.has_value() || !assignment.has_value()) {
    return false;
  }
  const bool same_start =
      layout_view->start_endpoint.owner_pole_id == assignment->decision_a.owner_pole_id &&
      layout_view->start_endpoint.order_decision_choice == assignment->decision_a.order_decision_choice &&
      layout_view->start_endpoint.chosen_side == assignment->decision_a.chosen_side &&
      layout_view->start_endpoint.support_orientation_basis == assignment->decision_a.support_orientation_basis &&
      layout_view->start_endpoint.lower_required == assignment->decision_a.lower_required &&
      layout_view->start_endpoint.relation_kind == assignment->decision_a.relation_kind;
  const bool same_end =
      layout_view->end_endpoint.owner_pole_id == assignment->decision_b.owner_pole_id &&
      layout_view->end_endpoint.order_decision_choice == assignment->decision_b.order_decision_choice &&
      layout_view->end_endpoint.chosen_side == assignment->decision_b.chosen_side &&
      layout_view->end_endpoint.support_orientation_basis == assignment->decision_b.support_orientation_basis &&
      layout_view->end_endpoint.lower_required == assignment->decision_b.lower_required &&
      layout_view->end_endpoint.relation_kind == assignment->decision_b.relation_kind;
  return same_start && same_end;
}

bool test_backbone_refresh_does_not_override_authoritative_endpoint_decision() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId span_id = generated.value.generated_span_ids.front();
  const auto before = state.view().inspect_support_layout(span_id);
  if (!before.has_value()) {
    return false;
  }
  const auto before_endpoint = layout_endpoint_for_owner(*before, center_id);
  if (!before_endpoint.has_value()) {
    return false;
  }
  const auto before_decision = *before_endpoint;

  if (!state.SetPoleManualYawOverride(center_id, 17.0).ok) {
    return false;
  }

  const auto after = state.view().inspect_support_layout(span_id);
  if (!after.has_value()) {
    return false;
  }
  const auto after_endpoint = layout_endpoint_for_owner(*after, center_id);
  return after_endpoint.has_value() &&
         after_endpoint->owner_pole_id == before_decision.owner_pole_id &&
         after_endpoint->owner_pole_id == after_endpoint->owner_pole_id &&
         after_endpoint->order_decision_choice == before_decision.order_decision_choice &&
         after_endpoint->chosen_side == before_decision.chosen_side &&
         after_endpoint->support_orientation_basis == before_decision.support_orientation_basis &&
         after_endpoint->lower_required == before_decision.lower_required;
}

bool test_backbone_authoritative_cross_pair_side_symmetry() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  std::optional<wire::core::LateralSideChoiceKind> shared_side{};
  bool saw_pair_based = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value() ||
        layout_view->lowering_kind != wire::core::BackboneLoweringKind::kCrossUnderpass) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value()) {
      continue;
    }
    saw_pair_based = saw_pair_based || endpoint->used_junction_pair_side_assignment;
    if (endpoint->chosen_side == wire::core::LateralSideChoiceKind::kCenter) {
      return false;
    }
    if (!shared_side.has_value()) {
      shared_side = endpoint->chosen_side;
    } else if (*shared_side != endpoint->chosen_side) {
      return false;
    }
  }
  return saw_pair_based && shared_side.has_value();
}

bool test_backbone_constrained_orientation_uses_authoritative_basis() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->origin != "PlacementConstraint") {
      continue;
    }
    const auto placement = lowered_support_group_for_owner(*layout_view, center_id);
    if (!placement.has_value() || placement->origin != "PlacementConstraint") {
      continue;
    }
    return placement->support_orientation_basis == endpoint->support_orientation_basis &&
           placement->order_decision_choice == endpoint->order_decision_choice &&
           placement->chosen_side == endpoint->chosen_side &&
           placement->support_orientation_basis != wire::core::SupportOrientationBasisKind::kRadial;
  }
  return false;
}

bool test_backbone_hv3_authoritative_order_decision_survives_refresh() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {15.2954, -10.2356, 0.0},
      {2.87944, -2.16948, 0.0},
      {12.0546, -16.1829, 0.0},
      {-2.43956, -5.96523, 0.0},
      {-14.3612, -6.28665, 0.0},
      {-14.1685, 6.36328, 0.0},
  };
  req.interval_m = 8.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId span_id = generated.value.generated_span_ids.front();
  const auto before = state.view().inspect_support_layout(span_id);
  if (!before.has_value()) {
    return false;
  }
  const auto before_decision = before->start_endpoint;
  if (before_decision.order_decision_policy != wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous) {
    return false;
  }

  const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
  if (span == nullptr) {
    return false;
  }
  if (!state.SetPoleManualYawOverride(span->endpoint_node_a_id, 11.0).ok) {
    return false;
  }

  const auto after = state.view().inspect_support_layout(span_id);
  return after.has_value() &&
         before->has_decision_seed && before->requires_decision_seed &&
         after->has_decision_seed == before->has_decision_seed &&
         after->requires_decision_seed == before->requires_decision_seed &&
         after->start_endpoint.order_decision_choice == before_decision.order_decision_choice &&
         after->start_endpoint.order_decision_choice_reason == before_decision.order_decision_choice_reason;
}

bool test_backbone_edge_orientation_uses_chosen_order_decision() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {8.0, 6.0, 0.0}, {18.0, 6.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  const auto& orientations = state.view().last_generation_edge_orientations();
  bool saw_hv = false;
  for (const auto& orientation : orientations) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    saw_hv = true;
    const auto expected = (orientation.order_decision_choice_a != orientation.order_decision_choice_b)
                              ? wire::core::LaneOrientation::kReversed
                              : wire::core::LaneOrientation::kNormal;
    if (orientation.orientation != expected) {
      std::cerr << "[DBG] C249 orientation mismatch edge=" << orientation.node_a_id << "->" << orientation.node_b_id
                << " orderA=" << static_cast<int>(orientation.order_decision_choice_a)
                << " orderB=" << static_cast<int>(orientation.order_decision_choice_b)
                << " orientation=" << static_cast<int>(orientation.orientation)
                << " expected=" << static_cast<int>(expected) << "\n";
      return false;
    }
  }
  return saw_hv;
}

bool test_backbone_branch_generation_preserves_existing_hv3_main_ports() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  const auto main_generated = state.GenerateFromBackboneSpec(main_req);
  if (!main_generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  std::vector<ObjectId> before_center_ports{};
  for (ObjectId span_id : main_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    if (span->endpoint_node_a_id == center_id) {
      before_center_ports.push_back(span->port_a_id);
    }
    if (span->endpoint_node_b_id == center_id) {
      before_center_ports.push_back(span->port_b_id);
    }
  }
  std::sort(before_center_ports.begin(), before_center_ports.end());
  before_center_ports.erase(std::unique(before_center_ports.begin(), before_center_ports.end()), before_center_ports.end());
  if (before_center_ports.size() != 3) {
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok) {
    return false;
  }

  std::vector<ObjectId> after_center_ports{};
  for (ObjectId span_id : main_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    if (span->endpoint_node_a_id == center_id) {
      after_center_ports.push_back(span->port_a_id);
    }
    if (span->endpoint_node_b_id == center_id) {
      after_center_ports.push_back(span->port_b_id);
    }
  }
  std::sort(after_center_ports.begin(), after_center_ports.end());
  after_center_ports.erase(std::unique(after_center_ports.begin(), after_center_ports.end()), after_center_ports.end());
  return before_center_ports == after_center_ports;
}

bool test_backbone_hv3_terminal_poles_use_distinct_template_bands() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  const ObjectId start_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId end_id = find_pole_id_by_position(state, {12.0, 0.0, 0.0});
  if (start_id == wire::core::kInvalidObjectId || end_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const wire::core::SegmentLaneAssignment* assignment = nullptr;
  for (const auto& candidate : state.view().last_lane_assignments()) {
    if ((candidate.pole_a_id == start_id && candidate.pole_b_id == end_id) ||
        (candidate.pole_a_id == end_id && candidate.pole_b_id == start_id)) {
      assignment = &candidate;
      break;
    }
  }
  if (assignment == nullptr) {
    return false;
  }
  const VisualSeparationMetrics visual_metrics = measure_lane_visual_separation_metrics(state, *assignment);
  const AxisRelationMetrics start_axis_metrics =
      measure_pole_axis_relation_metrics(state, start_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const AxisRelationMetrics end_axis_metrics =
      measure_pole_axis_relation_metrics(state, end_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});

  auto pole_is_visually_distinct = [&](ObjectId pole_id) {
    const wire::core::SegmentLaneAssignment* assignment_for_pole = nullptr;
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id == pole_id || assignment.pole_b_id == pole_id) {
        assignment_for_pole = &assignment;
        break;
      }
    }
    if (assignment_for_pole == nullptr) {
      return false;
    }
    const std::vector<ObjectId>& used_port_ids =
        (assignment_for_pole->pole_a_id == pole_id) ? assignment_for_pole->port_ids_a : assignment_for_pole->port_ids_b;
    if (used_port_ids.size() != 3) {
      return false;
    }
    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr) {
      return false;
    }
    const wire::core::Vec3d axis{1.0, 0.0, 0.0};
    const wire::core::Vec3d n = normalize_xy_safe(axis);
    const wire::core::Vec3d p{-n.y, n.x, 0.0};
    double min_along = std::numeric_limits<double>::infinity();
    double max_along = -std::numeric_limits<double>::infinity();
    double min_perp = std::numeric_limits<double>::infinity();
    double max_perp = -std::numeric_limits<double>::infinity();
    for (ObjectId port_id : used_port_ids) {
      const wire::core::Port* port = state.view().edit_state().ports.find(port_id);
      if (port == nullptr) {
        return false;
      }
      const wire::core::Vec3d delta = port->world_position - pole->world_transform.position;
      const double along = dot_xy(delta, n);
      const double perp = dot_xy(delta, p);
      min_along = std::min(min_along, along);
      max_along = std::max(max_along, along);
      min_perp = std::min(min_perp, perp);
      max_perp = std::max(max_perp, perp);
    }
    const double along_span = max_along - min_along;
    const double perp_span = max_perp - min_perp;
    return perp_span > 0.2 && perp_span > along_span * 2.0;
  };

  const bool ok = pole_is_visually_distinct(start_id) && pole_is_visually_distinct(end_id) && visual_metrics.topology_distinct &&
                  visual_metrics.visual_distinct && visual_metrics.projected_min_spacing_topview_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_start_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_end_m >= 0.10 && start_axis_metrics.valid &&
                  end_axis_metrics.valid && start_axis_metrics.angle_row_vs_span_deg >= 70.0 &&
                  end_axis_metrics.angle_row_vs_span_deg >= 70.0;
  if (!ok) {
    std::cerr << "[DBG] C185 visual=" << describe_visual_separation_metrics(visual_metrics)
              << " startAxis=" << describe_axis_relation_metrics(start_axis_metrics)
              << " endAxis=" << describe_axis_relation_metrics(end_axis_metrics) << "\n";
  }
  return ok;
}

bool test_backbone_hv3_terminal_fallback_ports_still_spread_perpendicular() {
  CoreState state;

  PoleTypeId fallback_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    bool has_hv_band = false;
    for (const auto& band : pole_type.port_bands) {
      if (band.enabled && band.category == wire::core::ConnectionCategory::kHighVoltage) {
        has_hv_band = true;
        break;
      }
    }
    if (!has_hv_band) {
      fallback_pole_type_id = pole_type.id;
      break;
    }
  }
  if (fallback_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = fallback_pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  const ObjectId start_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId end_id = find_pole_id_by_position(state, {12.0, 0.0, 0.0});
  if (start_id == wire::core::kInvalidObjectId || end_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const wire::core::SegmentLaneAssignment* assignment = nullptr;
  for (const auto& candidate : state.view().last_lane_assignments()) {
    if ((candidate.pole_a_id == start_id && candidate.pole_b_id == end_id) ||
        (candidate.pole_a_id == end_id && candidate.pole_b_id == start_id)) {
      assignment = &candidate;
      break;
    }
  }
  if (assignment == nullptr) {
    return false;
  }
  const VisualSeparationMetrics visual_metrics = measure_lane_visual_separation_metrics(state, *assignment);
  const AxisRelationMetrics start_axis_metrics =
      measure_pole_axis_relation_metrics(state, start_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const AxisRelationMetrics end_axis_metrics =
      measure_pole_axis_relation_metrics(state, end_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});

  auto pole_uses_perpendicular_generated_row = [&](ObjectId pole_id) {
    const wire::core::SegmentLaneAssignment* assignment_for_pole = nullptr;
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id == pole_id || assignment.pole_b_id == pole_id) {
        assignment_for_pole = &assignment;
        break;
      }
    }
    if (assignment_for_pole == nullptr) {
      return false;
    }

    const std::vector<ObjectId>& used_port_ids =
        (assignment_for_pole->pole_a_id == pole_id) ? assignment_for_pole->port_ids_a : assignment_for_pole->port_ids_b;
    if (used_port_ids.size() != 3) {
      return false;
    }

    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr) {
      return false;
    }

    bool saw_generated_row_port = false;
    const wire::core::Vec3d route_dir = normalize_xy_safe({1.0, 0.0, 0.0});
    const wire::core::Vec3d row_dir{-route_dir.y, route_dir.x, 0.0};
    double min_along = std::numeric_limits<double>::infinity();
    double max_along = -std::numeric_limits<double>::infinity();
    double min_perp = std::numeric_limits<double>::infinity();
    double max_perp = -std::numeric_limits<double>::infinity();
    for (ObjectId port_id : used_port_ids) {
      const wire::core::Port* port = state.view().edit_state().ports.find(port_id);
      if (port == nullptr) {
        return false;
      }
      if (port->generated_by_rule) {
        saw_generated_row_port = true;
      }
      const wire::core::Vec3d delta = port->world_position - pole->world_transform.position;
      const double along = dot_xy(delta, route_dir);
      const double perp = dot_xy(delta, row_dir);
      min_along = std::min(min_along, along);
      max_along = std::max(max_along, along);
      min_perp = std::min(min_perp, perp);
      max_perp = std::max(max_perp, perp);
    }
    const double along_span = max_along - min_along;
    const double perp_span = max_perp - min_perp;
    return saw_generated_row_port && perp_span > 0.2 && perp_span > along_span * 2.0;
  };

  const bool ok = pole_uses_perpendicular_generated_row(start_id) &&
                  pole_uses_perpendicular_generated_row(end_id) && visual_metrics.topology_distinct &&
                  visual_metrics.visual_distinct && visual_metrics.projected_min_spacing_topview_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_start_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_end_m >= 0.10 && start_axis_metrics.valid &&
                  end_axis_metrics.valid && start_axis_metrics.angle_row_vs_span_deg >= 70.0 &&
                  end_axis_metrics.angle_row_vs_span_deg >= 70.0;
  if (!ok) {
    std::cerr << "[DBG] C188 visual=" << describe_visual_separation_metrics(visual_metrics)
              << " startAxis=" << describe_axis_relation_metrics(start_axis_metrics)
              << " endAxis=" << describe_axis_relation_metrics(end_axis_metrics) << "\n";
  }
  return ok;
}

bool test_backbone_hv3_terminals_stay_perpendicular_on_communication_pole_with_all_templates() {
  CoreState state;

  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = pole_type.id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = communication_pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
  add_backbone_bundle(req, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  const ObjectId start_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId end_id = find_pole_id_by_position(state, {12.0, 0.0, 0.0});
  if (start_id == wire::core::kInvalidObjectId || end_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const wire::core::SegmentLaneAssignment* assignment = nullptr;
  for (const auto& candidate : state.view().last_lane_assignments()) {
    if (!((candidate.pole_a_id == start_id && candidate.pole_b_id == end_id) ||
          (candidate.pole_a_id == end_id && candidate.pole_b_id == start_id))) {
      continue;
    }
    const auto* bundle = state.view().edit_state().bundles.find(candidate.bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == wire::core::BundleKind::kHighVoltage) {
      assignment = &candidate;
      break;
    }
  }
  if (assignment == nullptr) {
    return false;
  }

  const VisualSeparationMetrics visual_metrics = measure_lane_visual_separation_metrics(state, *assignment);
  const AxisRelationMetrics start_axis_metrics =
      measure_pole_axis_relation_metrics(state, start_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const AxisRelationMetrics end_axis_metrics =
      measure_pole_axis_relation_metrics(state, end_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const bool ok = visual_metrics.topology_distinct && visual_metrics.visual_distinct &&
                  visual_metrics.projected_min_spacing_topview_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_start_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_end_m >= 0.10 && start_axis_metrics.valid &&
                  end_axis_metrics.valid && start_axis_metrics.angle_row_vs_span_deg >= 70.0 &&
                  end_axis_metrics.angle_row_vs_span_deg >= 70.0;
  if (!ok) {
    std::cerr << "[DBG] C189 visual=" << describe_visual_separation_metrics(visual_metrics)
              << " startAxis=" << describe_axis_relation_metrics(start_axis_metrics)
              << " endAxis=" << describe_axis_relation_metrics(end_axis_metrics) << "\n";
  }
  return ok;
}

bool test_backbone_communication_multilane_terminals_stay_perpendicular() {
  CoreState state;

  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = pole_type.id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = communication_pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  const ObjectId start_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId end_id = find_pole_id_by_position(state, {12.0, 0.0, 0.0});
  if (start_id == wire::core::kInvalidObjectId || end_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const wire::core::SegmentLaneAssignment* assignment = nullptr;
  for (const auto& candidate : state.view().last_lane_assignments()) {
    if (!((candidate.pole_a_id == start_id && candidate.pole_b_id == end_id) ||
          (candidate.pole_a_id == end_id && candidate.pole_b_id == start_id))) {
      continue;
    }
    const auto* bundle = state.view().edit_state().bundles.find(candidate.bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == wire::core::BundleKind::kCommunication) {
      assignment = &candidate;
      break;
    }
  }
  if (assignment == nullptr) {
    return false;
  }

  const VisualSeparationMetrics visual_metrics = measure_lane_visual_separation_metrics(state, *assignment);
  const AxisRelationMetrics start_axis_metrics =
      measure_pole_axis_relation_metrics(state, start_id, wire::core::PortLayer::kCommunication, {1.0, 0.0, 0.0});
  const AxisRelationMetrics end_axis_metrics =
      measure_pole_axis_relation_metrics(state, end_id, wire::core::PortLayer::kCommunication, {1.0, 0.0, 0.0});
  const bool ok = visual_metrics.topology_distinct && visual_metrics.visual_distinct &&
                  visual_metrics.projected_min_spacing_topview_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_start_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_end_m >= 0.10 && start_axis_metrics.valid &&
                  end_axis_metrics.valid && start_axis_metrics.angle_row_vs_span_deg >= 70.0 && end_axis_metrics.angle_row_vs_span_deg >= 70.0;
  if (!ok) {
    std::cerr << "[DBG] C190 visual=" << describe_visual_separation_metrics(visual_metrics)
              << " startAxis=" << describe_axis_relation_metrics(start_axis_metrics)
              << " endAxis=" << describe_axis_relation_metrics(end_axis_metrics) << "\n";
  }
  return ok;
}

bool test_backbone_clicked_existing_communication_poles_all_templates_keep_hv_terminal_separation() {
  CoreState state;

  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = pole_type.id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::Transformd left_tf{};
  left_tf.position = {-12.0, 0.0, 0.0};
  wire::core::Transformd right_tf{};
  right_tf.position = {12.0, 0.0, 0.0};
  const auto left_add = state.AddPole(left_tf, 10.0, "Left");
  const auto right_add = state.AddPole(right_tf, 10.0, "Right");
  if (!left_add.ok || !right_add.ok) {
    return false;
  }
  const ObjectId left_id = left_add.value;
  const ObjectId right_id = right_add.value;
  if (!state.ApplyPoleType(left_id, communication_pole_type_id).ok ||
      !state.ApplyPoleType(right_id, communication_pole_type_id).ok) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec left_spec{};
  left_spec.point_index = 0;
  left_spec.support_kind = wire::core::SupportKind::kPole;
  left_spec.node_id = left_id;
  req.path.node_specs.push_back(left_spec);
  wire::core::BackboneInputSpec::NodeSpec right_spec{};
  right_spec.point_index = 1;
  right_spec.support_kind = wire::core::SupportKind::kPole;
  right_spec.node_id = right_id;
  req.path.node_specs.push_back(right_spec);
  req.interval_m = 1000.0;
  req.pole_type_id = communication_pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
  add_backbone_bundle(req, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  const wire::core::SegmentLaneAssignment* assignment = nullptr;
  for (const auto& candidate : state.view().last_lane_assignments()) {
    if (!((candidate.pole_a_id == left_id && candidate.pole_b_id == right_id) ||
          (candidate.pole_a_id == right_id && candidate.pole_b_id == left_id))) {
      continue;
    }
    const auto* bundle = state.view().edit_state().bundles.find(candidate.bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == wire::core::BundleKind::kHighVoltage) {
      assignment = &candidate;
      break;
    }
  }
  if (assignment == nullptr) {
    return false;
  }

  const VisualSeparationMetrics visual_metrics = measure_lane_visual_separation_metrics(state, *assignment);
  const AxisRelationMetrics left_axis_metrics =
      measure_pole_axis_relation_metrics(state, left_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const AxisRelationMetrics right_axis_metrics =
      measure_pole_axis_relation_metrics(state, right_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const bool ok = visual_metrics.topology_distinct && visual_metrics.visual_distinct &&
                  visual_metrics.projected_min_spacing_topview_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_start_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_end_m >= 0.10 && left_axis_metrics.valid &&
                  right_axis_metrics.valid && left_axis_metrics.angle_row_vs_span_deg >= 70.0 && right_axis_metrics.angle_row_vs_span_deg >= 70.0;
  if (!ok) {
    std::cerr << "[DBG] C192 visual=" << describe_visual_separation_metrics(visual_metrics)
              << " leftAxis=" << describe_axis_relation_metrics(left_axis_metrics)
              << " rightAxis=" << describe_axis_relation_metrics(right_axis_metrics) << "\n";
  }
  return ok;
}


bool test_backbone_mixed_route_uses_edge_level_flow_classification() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    return false;
  }

  const ObjectId left_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (left_id == wire::core::kInvalidObjectId || center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec mixed_req{};
  mixed_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  mixed_req.interval_m = 1000.0;
  mixed_req.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec left_spec{};
  left_spec.point_index = 0;
  left_spec.support_kind = wire::core::SupportKind::kPole;
  left_spec.node_id = left_id;
  mixed_req.path.node_specs.push_back(left_spec);
  wire::core::BackboneInputSpec::NodeSpec center_spec{};
  center_spec.point_index = 1;
  center_spec.support_kind = wire::core::SupportKind::kPole;
  center_spec.node_id = center_id;
  mixed_req.path.node_specs.push_back(center_spec);
  add_backbone_bundle(mixed_req, wire::core::BundleKind::kLowVoltage);
  const auto mixed_generated = state.GenerateFromBackboneSpec(mixed_req);
  if (!mixed_generated.ok) {
    return false;
  }

  const auto& assignments = state.view().last_lane_assignments();
  if (assignments.size() != 2) {
    std::cerr << "[DBG] C138 assignmentCount=" << assignments.size() << "\n";
    return false;
  }
  const bool ok = assignments[0].flow_kind == wire::core::BackboneFlowKind::kMain &&
                  assignments[0].flow_decision_rule == wire::core::BackboneFlowDecisionRule::kJunctionOrderMain &&
                  !assignments[0].uses_branch_support &&
                  assignments[1].flow_kind == wire::core::BackboneFlowKind::kBranch &&
                  assignments[1].flow_decision_rule == wire::core::BackboneFlowDecisionRule::kJunctionOrderBranch &&
                  !assignments[1].uses_branch_support &&
                  assignments[1].branch_down_offset_m == 0.0;
  if (!ok) {
    for (std::size_t i = 0; i < assignments.size(); ++i) {
      std::cerr << "[DBG] C138 idx=" << i
                << " flow=" << static_cast<int>(assignments[i].flow_kind)
                << " rule=" << static_cast<int>(assignments[i].flow_decision_rule)
                << " branchSupport=" << (assignments[i].uses_branch_support ? 1 : 0)
                << " down=" << assignments[i].branch_down_offset_m << "\n";
    }
  }
  return ok;
}

bool test_backbone_branch_support_visual_cache_contains_support_placement() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);

  bool found_branch_support = false;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value() || layout_view->lowered_support_groups.empty()) {
      continue;
    }
    found_branch_support = found_branch_support ||
                           std::any_of(layout_view->lowered_support_groups.begin(),
                                       layout_view->lowered_support_groups.end(),
                                       [center_id](const wire::core::LoweredSupportGroupInspectionView& placement) {
                                         return placement.owner_pole_id == center_id && placement.down_offset_m > 0.0 &&
                                                placement.side != wire::core::SlotSide::kCenter;
                                       });
  }
  return found_branch_support;
}

bool test_backbone_near_straight_branch_still_classifies_as_branch() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {11.5, 0.8, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kLowVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok) {
    return false;
  }

  const auto& assignments = state.view().last_lane_assignments();
  const bool ok = assignments.size() == 1 && assignments.front().flow_kind == wire::core::BackboneFlowKind::kBranch &&
                  assignments.front().flow_decision_rule == wire::core::BackboneFlowDecisionRule::kJunctionOrderBranch;
  if (!ok) {
    std::cerr << "[DBG] C140 assignmentCount=" << assignments.size();
    if (!assignments.empty()) {
      std::cerr << " flow=" << static_cast<int>(assignments.front().flow_kind)
                << " rule=" << static_cast<int>(assignments.front().flow_decision_rule);
    }
    std::cerr << "\n";
  }
  return ok;
}

bool test_backbone_crosslike_single_edge_stays_main() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec horizontal{};
  horizontal.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  horizontal.interval_m = 1000.0;
  horizontal.pole_type_id = type_ids.front();
  add_backbone_bundle(horizontal, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(horizontal).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec vertical{};
  vertical.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared_center{};
  shared_center.point_index = 1;
  shared_center.support_kind = wire::core::SupportKind::kPole;
  shared_center.node_id = center_id;
  vertical.path.node_specs.push_back(shared_center);
  vertical.interval_m = 1000.0;
  vertical.pole_type_id = type_ids.front();
  add_backbone_bundle(vertical, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(vertical).ok) {
    return false;
  }

  wire::core::BackboneSpec single{};
  single.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec anchored{};
  anchored.point_index = 0;
  anchored.support_kind = wire::core::SupportKind::kPole;
  anchored.node_id = center_id;
  single.path.node_specs.push_back(anchored);
  single.interval_m = 1000.0;
  single.pole_type_id = type_ids.front();
  add_backbone_bundle(single, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(single);
  if (!generated.ok || generated.value.generated_span_ids.size() != 1) {
    return false;
  }

  const auto& assignments = state.view().last_lane_assignments();
  if (assignments.size() != 1) {
    return false;
  }
  const auto span_view = state.view().inspect_span(generated.value.generated_span_ids.front());
  const auto layout_view = state.view().inspect_support_layout(generated.value.generated_span_ids.front());
  return span_view.has_value() && layout_view.has_value() &&
         assignments.front().flow_kind == wire::core::BackboneFlowKind::kMain &&
         !assignments.front().decision_a.lower_required && !assignments.front().decision_b.lower_required &&
         assignments.front().decision_a.support_group_id < 0 && assignments.front().decision_b.support_group_id < 0 &&
         assignments.front().branch_down_offset_m == 0.0 &&
         layout_view->lowered_support_groups.empty() && span_view->same_level_feasible;
}

bool test_backbone_new_chain_uses_fallback_orientation_without_existing_main_context() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto it_debug = state.view().pole_orientation_debug_records().find(center_id);
  return it_debug != state.view().pole_orientation_debug_records().end() &&
         it_debug->second.rule == wire::core::PoleForwardRule::kFallback;
}

double support_axis_alignment_ratio(const CoreState& state, ObjectId pole_id, const wire::core::Vec3d& axis) {
  const auto pole_view = state.view().inspect_pole(pole_id);
  if (!pole_view.has_value() || !pole_view->has_support_axis) {
    return 0.0;
  }
  return std::abs(dot_xy(normalize_xy_safe(pole_view->support_axis_dir), normalize_xy_safe(axis)));
}

bool unique_pole_ports_spread_along_axis(const CoreState& state, ObjectId pole_id, wire::core::PortLayer layer,
                                         const wire::core::Vec3d& axis, double* out_along_span = nullptr,
                                         double* out_perp_span = nullptr) {
  std::vector<const wire::core::Port*> ports{};
  for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
    if (port.owner_pole_id == pole_id && port.layer == layer &&
        port.placement_source != wire::core::PortPlacementSourceKind::kBranchSupport) {
      ports.push_back(&port);
    }
  }
  if (ports.size() < 2) {
    return false;
  }
  const wire::core::Vec3d n = normalize_xy_safe(axis);
  const wire::core::Vec3d p{-n.y, n.x, 0.0};
  double min_along = std::numeric_limits<double>::infinity();
  double max_along = -std::numeric_limits<double>::infinity();
  double min_perp = std::numeric_limits<double>::infinity();
  double max_perp = -std::numeric_limits<double>::infinity();
  const auto* pole = state.view().edit_state().poles.find(pole_id);
  if (pole == nullptr) {
    return false;
  }
  for (const wire::core::Port* port : ports) {
    const wire::core::Vec3d delta = port->world_position - pole->world_transform.position;
    const double along = dot_xy(delta, n);
    const double perp = dot_xy(delta, p);
    min_along = std::min(min_along, along);
    max_along = std::max(max_along, along);
    min_perp = std::min(min_perp, perp);
    max_perp = std::max(max_perp, perp);
  }
  if (out_along_span != nullptr) {
    *out_along_span = max_along - min_along;
  }
  if (out_perp_span != nullptr) {
    *out_perp_span = max_perp - min_perp;
  }
  return (max_along - min_along) > (max_perp - min_perp) * 2.0;
}

double max_curve_lateral_overshoot_xy(const wire::core::DetailCurve& curve, std::size_t* out_peak_index = nullptr) {
  const wire::core::Vec3d start = curve.EvaluatePosition(0.0);
  const wire::core::Vec3d end = curve.EvaluatePosition(1.0);
  const wire::core::Vec3d chord_dir = normalize_xy_safe(end - start);
  if (std::abs(chord_dir.x) <= 1e-6 && std::abs(chord_dir.y) <= 1e-6) {
    return 0.0;
  }
  const wire::core::Vec3d lateral_axis{-chord_dir.y, chord_dir.x, 0.0};
  double max_abs_lateral = 0.0;
  std::size_t peak_index = 0;
  for (std::size_t i = 0; i < curve.sample_points.size(); ++i) {
    const double abs_lateral = std::abs(dot_xy(curve.sample_points[i] - start, lateral_axis));
    if (abs_lateral > max_abs_lateral) {
      max_abs_lateral = abs_lateral;
      peak_index = i;
    }
  }
  if (out_peak_index != nullptr) {
    *out_peak_index = peak_index;
  }
  return max_abs_lateral;
}

bool trace_contains_summary(const std::vector<wire::core::DecisionTraceEntry>& trace, wire::core::DecisionTraceTopic topic,
                            const std::string& rule, const std::string& token) {
  for (const auto& entry : trace) {
    if (entry.topic != topic) {
      continue;
    }
    if (!rule.empty() && entry.rule != rule) {
      continue;
    }
    if (entry.summary.find(token) != std::string::npos) {
      return true;
    }
  }
  return false;
}

std::optional<wire::core::SupportLayoutEndpointView> layout_endpoint_for_owner(
    const wire::core::SupportLayoutInspectionView& layout_view, wire::core::ObjectId owner_pole_id) {
  if (layout_view.start_endpoint.owner_pole_id == owner_pole_id) {
    return layout_view.start_endpoint;
  }
  if (layout_view.end_endpoint.owner_pole_id == owner_pole_id) {
    return layout_view.end_endpoint;
  }
  return std::nullopt;
}

std::optional<wire::core::LoweredSupportGroupInspectionView> lowered_support_group_for_owner(
    const wire::core::SupportLayoutInspectionView& layout_view, wire::core::ObjectId owner_pole_id) {
  for (const auto& group : layout_view.lowered_support_groups) {
    if (group.owner_pole_id == owner_pole_id) {
      return group;
    }
  }
  return std::nullopt;
}

std::optional<wire::core::SegmentLaneAssignment> find_assignment_for_span(const wire::core::CoreState& state,
                                                                          wire::core::ObjectId span_id) {
  const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
  if (span == nullptr) {
    return std::nullopt;
  }
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const bool same_forward = assignment.bundle_id == span->bundle_id && assignment.pole_a_id == span->endpoint_node_a_id &&
                              assignment.pole_b_id == span->endpoint_node_b_id;
    const bool same_reverse = assignment.bundle_id == span->bundle_id && assignment.pole_a_id == span->endpoint_node_b_id &&
                              assignment.pole_b_id == span->endpoint_node_a_id;
    if (same_forward || same_reverse) {
      return assignment;
    }
  }
  return std::nullopt;
}

std::optional<wire::core::VisualPart> support_arm_part_for_owner(const wire::core::CoreState& state,
                                                                 wire::core::ObjectId span_id,
                                                                 wire::core::ObjectId owner_pole_id,
                                                                 double z_tolerance) {
  const wire::core::Pole* pole = state.view().edit_state().poles.find(owner_pole_id);
  const wire::core::SpanVisualCacheEntry* visual = state.view().find_span_visual_cache(span_id);
  if (pole == nullptr || visual == nullptr) {
    return std::nullopt;
  }
  for (const auto& part : visual->parts) {
    if (part.kind != wire::core::VisualPartKind::kSupportArm) {
      continue;
    }
    if (!almost_equal(part.a.x, pole->world_transform.position.x, 1e-6) ||
        !almost_equal(part.a.y, pole->world_transform.position.y, 1e-6)) {
      continue;
    }
    if (std::abs(part.a.z - part.b.z) > z_tolerance) {
      continue;
    }
    return part;
  }
  return std::nullopt;
}

bool test_backbone_straight_chain_support_axis_stays_perpendicular_to_route() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(req).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto pole_view = state.view().inspect_pole(center_id);
  if (!pole_view.has_value()) {
    return false;
  }
  const AxisRelationMetrics metrics =
      measure_pole_axis_relation_metrics(state, center_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  double along = 0.0;
  double perp = 0.0;
  const double axis_align = support_axis_alignment_ratio(state, center_id, {0.0, 1.0, 0.0});
  const bool spread = unique_pole_ports_spread_along_axis(state, center_id, wire::core::PortLayer::kHighVoltage,
                                                          {0.0, 1.0, 0.0}, &along, &perp);
  if (!(axis_align > 0.99 && spread && along > 0.2 && perp < along && metrics.valid &&
        metrics.angle_row_vs_span_deg >= 70.0 && metrics.angle_forward_vs_span_deg <= 20.0)) {
    std::cerr << "[DBG] C173 pole=" << center_id << " rule=" << static_cast<int>(pole_view->support_axis_rule)
              << " axis=" << pole_view->support_axis_dir.x << "," << pole_view->support_axis_dir.y
              << " layoutYaw=" << pole_view->layout_yaw_deg << " finalYaw=" << pole_view->final_yaw_deg
              << " align=" << axis_align << " spread=" << (spread ? 1 : 0) << " along=" << along
              << " perp=" << perp << " metrics=" << describe_axis_relation_metrics(metrics) << "\n";
  }
  return axis_align > 0.99 && spread && along > 0.2 && perp < along && metrics.valid &&
         metrics.angle_row_vs_span_deg >= 70.0 && metrics.angle_forward_vs_span_deg <= 20.0;
}

bool test_backbone_cross_junction_support_axis_avoids_diagonal() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec horizontal{};
  horizontal.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  horizontal.interval_m = 1000.0;
  horizontal.pole_type_id = type_ids.front();
  add_backbone_bundle(horizontal, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(horizontal).ok) {
    std::cerr << "[DBG] C174 horizontal_generate_failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec vertical{};
  vertical.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  vertical.interval_m = 1000.0;
  vertical.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  vertical.path.node_specs.push_back(shared);
  add_backbone_bundle(vertical, wire::core::BundleKind::kHighVoltage);
  const auto vertical_generated = state.GenerateFromBackboneSpec(vertical);
  if (!vertical_generated.ok) {
    std::cerr << "[DBG] C174 vertical_generate_failed error=" << vertical_generated.error << "\n";
    return false;
  }

  const auto pole_view = state.view().inspect_pole(center_id);
  if (!pole_view.has_value() || !pole_view->has_support_axis) {
    std::cerr << "[DBG] C174 pole_view_missing pole=" << center_id << "\n";
    return false;
  }
  const AxisRelationMetrics metrics =
      measure_pole_axis_relation_metrics(state, center_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const wire::core::Vec3d diag_a = normalize_xy_safe({1.0, 1.0, 0.0});
  const wire::core::Vec3d diag_b = normalize_xy_safe({1.0, -1.0, 0.0});
  const double diag_dot = std::max(std::abs(dot_xy(normalize_xy_safe(pole_view->support_axis_dir), diag_a)),
                                   std::abs(dot_xy(normalize_xy_safe(pole_view->support_axis_dir), diag_b)));
  if (!(pole_view->support_axis_rule != wire::core::PoleSupportAxisRule::kConnectedDirectionFit && diag_dot < 0.8 &&
        metrics.valid && metrics.angle_row_vs_span_deg >= 70.0)) {
    std::cerr << "[DBG] C174 pole=" << center_id << " rule=" << static_cast<int>(pole_view->support_axis_rule)
              << " axis=" << pole_view->support_axis_dir.x << "," << pole_view->support_axis_dir.y
              << " layoutYaw=" << pole_view->layout_yaw_deg << " finalYaw=" << pole_view->final_yaw_deg
              << " diagDot=" << diag_dot
              << " metrics=" << describe_axis_relation_metrics(metrics) << "\n";
  }
  return pole_view->support_axis_rule != wire::core::PoleSupportAxisRule::kConnectedDirectionFit && diag_dot < 0.8 &&
         metrics.valid && metrics.angle_row_vs_span_deg >= 70.0;
}

bool test_backbone_cross_junction_layout_yaw_stays_on_pole_yaw() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec horizontal{};
  horizontal.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  horizontal.interval_m = 1000.0;
  horizontal.pole_type_id = type_ids.front();
  add_backbone_bundle(horizontal, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(horizontal).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec vertical{};
  vertical.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  vertical.interval_m = 1000.0;
  vertical.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  vertical.path.node_specs.push_back(shared);
  add_backbone_bundle(vertical, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(vertical).ok) {
    return false;
  }

  const auto pole_view = state.view().inspect_pole(center_id);
  if (!pole_view.has_value() || !pole_view->has_support_axis || !pole_view->has_layout_yaw || !pole_view->has_final_yaw) {
    return false;
  }

  const wire::core::Vec3d support_axis = normalize_xy_safe(pole_view->support_axis_dir);
  const double final_yaw_rad = pole_view->final_yaw_deg * (3.14159265358979323846 / 180.0);
  const wire::core::Vec3d final_forward = normalize_xy_safe({std::cos(final_yaw_rad), std::sin(final_yaw_rad), 0.0});
  const double forward_alignment = std::abs(dot_xy(support_axis, final_forward));
  const bool ok = forward_alignment > 0.95 &&
                  angle_diff_abs_deg(pole_view->layout_yaw_deg, pole_view->final_yaw_deg) <= 1e-6;
  if (!ok) {
    std::cerr << "[DBG] C302 pole=" << center_id << " axis=" << support_axis.x << "," << support_axis.y
              << " forwardAlign=" << forward_alignment
              << " layoutYaw=" << pole_view->layout_yaw_deg << " finalYaw=" << pole_view->final_yaw_deg << "\n";
  }
  return ok;
}

bool test_bundle_template_row_layout_axis_mode_drives_port_layout_yaw() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const auto lv_it = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  if (lv_it == state.view().bundle_templates().end()) {
    return false;
  }
  wire::core::BundleTemplate lv = lv_it->second;
  lv.row_layout_axis_mode = wire::core::RowLayoutAxisMode::kSupportAxis;
  const auto update = state.UpdateBundleTemplate(lv);
  if (!update.ok || !update.value) {
    return false;
  }

  wire::core::BackboneSpec horizontal{};
  horizontal.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  horizontal.interval_m = 1000.0;
  horizontal.pole_type_id = type_ids.front();
  add_backbone_bundle(horizontal, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(horizontal).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec vertical{};
  vertical.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  vertical.interval_m = 1000.0;
  vertical.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  vertical.path.node_specs.push_back(shared);
  add_backbone_bundle(vertical, wire::core::BundleKind::kLowVoltage);
  const auto vertical_generated = state.GenerateFromBackboneSpec(vertical);
  if (!vertical_generated.ok) {
    return false;
  }

  const auto pole_view = state.view().inspect_pole(center_id);
  const auto* pole = state.view().edit_state().poles.find(center_id);
  if (!pole_view.has_value() || !pole_view->has_support_axis || pole == nullptr) {
    return false;
  }

  const double row_layout_yaw_deg =
      state.effective_port_layout_yaw_deg(*pole, wire::core::ConnectionCategory::kLowVoltage);
  const double row_layout_yaw_rad = row_layout_yaw_deg * (3.14159265358979323846 / 180.0);
  const wire::core::Vec3d row_axis = normalize_xy_safe({-std::sin(row_layout_yaw_rad), std::cos(row_layout_yaw_rad), 0.0});
  const wire::core::Vec3d support_axis = normalize_xy_safe(pole_view->support_axis_dir);
  const double row_alignment = std::abs(dot_xy(row_axis, support_axis));
  const double row_vs_pole_yaw = angle_diff_abs_deg(row_layout_yaw_deg, pole_view->layout_yaw_deg);
  const bool ok = pole_view->row_layout_axis_mode == wire::core::RowLayoutAxisMode::kSupportAxis &&
                  pole_view->row_layout_axis_category == wire::core::ConnectionCategory::kLowVoltage &&
                  angle_diff_abs_deg(pole_view->layout_yaw_deg, pole_view->final_yaw_deg) <= 1e-6 &&
                  row_alignment > 0.99 && row_vs_pole_yaw >= 20.0;
  if (!ok) {
    std::cerr << "[DBG] C303 pole=" << center_id << " rowMode=" << static_cast<int>(pole_view->row_layout_axis_mode)
              << " rowCategory=" << static_cast<int>(pole_view->row_layout_axis_category)
              << " rowYaw=" << row_layout_yaw_deg << " poleYaw=" << pole_view->layout_yaw_deg
              << " finalYaw=" << pole_view->final_yaw_deg << " rowAlign=" << row_alignment << "\n";
  }
  return ok;
}

bool test_backbone_orthogonal_route_support_axis_stays_cardinal() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}, {12.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(req).ok) {
    return false;
  }
  const ObjectId corner_id = find_pole_id_by_position(state, {0.0, 12.0, 0.0});
  const auto pole_view = state.view().inspect_pole(corner_id);
  if (corner_id == wire::core::kInvalidObjectId || !pole_view.has_value() || !pole_view->has_support_axis) {
    return false;
  }
  const AxisRelationMetrics metrics =
      measure_pole_axis_relation_metrics(state, corner_id, wire::core::PortLayer::kLowVoltage, {1.0, 0.0, 0.0});
  const wire::core::Vec3d diag_a = normalize_xy_safe({1.0, 1.0, 0.0});
  const wire::core::Vec3d diag_b = normalize_xy_safe({1.0, -1.0, 0.0});
  const double diag_dot = std::max(std::abs(dot_xy(normalize_xy_safe(pole_view->support_axis_dir), diag_a)),
                                   std::abs(dot_xy(normalize_xy_safe(pole_view->support_axis_dir), diag_b)));
  const bool ok = pole_view->support_axis_rule == wire::core::PoleSupportAxisRule::kConnectedDirectionFit &&
                  diag_dot > 0.9 && metrics.valid &&
                  metrics.angle_row_vs_span_deg > 25.0 && metrics.angle_row_vs_span_deg < 65.0;
  if (!ok) {
    std::cerr << "[DBG] C175 diagDot=" << diag_dot << " metrics=" << describe_axis_relation_metrics(metrics) << "\n";
  }
  return ok;
}

bool test_backbone_branch_keeps_main_support_axis_non_diagonal() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    std::cerr << "[DBG] C176 trunk_generate_failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C176 center_missing_after_trunk\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch);
  if (!branch_generated.ok) {
    std::cerr << "[DBG] C176 branch_generate_failed error=" << branch_generated.error << "\n";
    return false;
  }

  const auto pole_view = state.view().inspect_pole(center_id);
  const AxisRelationMetrics metrics =
      measure_pole_axis_relation_metrics(state, center_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const double axis_align = support_axis_alignment_ratio(state, center_id, {0.0, 1.0, 0.0});
  double along = 0.0;
  double perp = 0.0;
  const bool spread = unique_pole_ports_spread_along_axis(state, center_id, wire::core::PortLayer::kHighVoltage,
                                                          {0.0, 1.0, 0.0}, &along, &perp);
  if (!(axis_align > 0.99 && spread && along > 0.2 && perp < along && metrics.valid &&
        metrics.angle_row_vs_span_deg >= 70.0 && metrics.angle_forward_vs_span_deg <= 20.0)) {
    std::cerr << "[DBG] C176 pole=" << center_id << " rule="
              << static_cast<int>(pole_view.has_value() ? pole_view->support_axis_rule
                                                        : wire::core::PoleSupportAxisRule::kFallback)
              << " axis="
              << (pole_view.has_value() ? pole_view->support_axis_dir.x : 0.0) << ","
              << (pole_view.has_value() ? pole_view->support_axis_dir.y : 0.0)
              << " layoutYaw=" << (pole_view.has_value() ? pole_view->layout_yaw_deg : 0.0)
              << " finalYaw=" << (pole_view.has_value() ? pole_view->final_yaw_deg : 0.0)
              << " primary=" << (pole_view.has_value() ? pole_view->primary_neighbor_id : wire::core::kInvalidObjectId)
              << " secondary="
              << (pole_view.has_value() ? pole_view->secondary_neighbor_id : wire::core::kInvalidObjectId)
              << " align=" << axis_align << " spread=" << (spread ? 1 : 0) << " along=" << along
              << " perp=" << perp << " metrics=" << describe_axis_relation_metrics(metrics) << "\n";
  }
  return axis_align > 0.99 && spread && along > 0.2 && perp < along && metrics.valid &&
         metrics.angle_row_vs_span_deg >= 70.0 && metrics.angle_forward_vs_span_deg <= 20.0;
}

bool test_backbone_single_edge_reuse_preserves_existing_straight_support_axis() {
  CoreState state;

  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = pole_type.id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C323 missing CommunicationPole\n";
    return false;
  }

  auto add_all_templates = [](wire::core::BackboneSpec& req) {
    add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
    add_backbone_bundle(req, wire::core::BundleKind::kOptical);
  };
  auto support_axis_for_pole = [&](ObjectId pole_id) {
    const auto pole_view = state.view().inspect_pole(pole_id);
    if (!pole_view.has_value() || !pole_view->has_support_axis) {
      return wire::core::Vec3d{};
    }
    return normalize_xy_safe(pole_view->support_axis_dir);
  };

  constexpr wire::core::Vec3d kTrunk0{10.8184, 4.94506, 0.0};
  constexpr wire::core::Vec3d kTrunk1{9.90598, 14.2226, 0.0};
  constexpr wire::core::Vec3d kTrunk2{-5.24054, 15.5595, 0.0};
  constexpr wire::core::Vec3d kCenter{-15.9599, -2.86144, 0.0};
  constexpr wire::core::Vec3d kTrunk4{-14.128, -7.28552, 0.0};
  constexpr wire::core::Vec3d kTrunk5{-8.03361, -13.365, 0.0};
  constexpr wire::core::Vec3d kBranchA{-24.2948, -1.36634, 0.0};
  constexpr wire::core::Vec3d kBranchB{-5.34189, -3.8355, 0.0};

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {kTrunk0, kTrunk1, kTrunk2, kCenter, kTrunk4, kTrunk5};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = communication_pole_type_id;
  add_all_templates(trunk);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    std::cerr << "[DBG] C323 trunk_generate_failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, kCenter, 1e-4);
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C323 center_missing_after_trunk\n";
    return false;
  }
  const wire::core::Vec3d baseline_axis = support_axis_for_pole(center_id);
  if ((baseline_axis.x * baseline_axis.x + baseline_axis.y * baseline_axis.y) <= 1e-12) {
    std::cerr << "[DBG] C323 baseline_axis_missing\n";
    return false;
  }
  const auto baseline_trace =
      state.view().collect_decision_trace({wire::core::EntityKind::kPole, static_cast<std::uint64_t>(center_id)});

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {kCenter, kBranchA};
  branch.interval_m = 1000.0;
  branch.pole_type_id = communication_pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec shared_branch{};
  shared_branch.point_index = 0;
  shared_branch.support_kind = wire::core::SupportKind::kPole;
  shared_branch.node_id = center_id;
  branch.path.node_specs.push_back(shared_branch);
  add_all_templates(branch);
  if (!state.GenerateFromBackboneSpec(branch).ok) {
    std::cerr << "[DBG] C323 first_branch_generate_failed\n";
    return false;
  }

  const wire::core::Vec3d after_branch_axis = support_axis_for_pole(center_id);
  const double branch_alignment = std::abs(dot_xy(after_branch_axis, baseline_axis));
  const auto branch_pole_view = state.view().inspect_pole(center_id);
  if (branch_alignment < 0.95 || !branch_pole_view.has_value() ||
      branch_pole_view->forward_rule == wire::core::PoleForwardRule::kFallback ||
      branch_pole_view->support_axis_rule == wire::core::PoleSupportAxisRule::kConnectedDirectionFit) {
    const auto branch_trace =
        state.view().collect_decision_trace({wire::core::EntityKind::kPole, static_cast<std::uint64_t>(center_id)});
    std::cerr << "[DBG] C323 branch_alignment=" << branch_alignment << " baseline=(" << baseline_axis.x << ","
              << baseline_axis.y << "," << baseline_axis.z << ") after=(" << after_branch_axis.x << ","
              << after_branch_axis.y << "," << after_branch_axis.z << ") supportRule="
              << static_cast<int>(branch_pole_view.has_value() ? branch_pole_view->support_axis_rule
                                                               : wire::core::PoleSupportAxisRule::kFallback);
    if (branch_pole_view.has_value()) {
      std::cerr << " forwardRule=" << static_cast<int>(branch_pole_view->forward_rule);
    }
    for (const auto& entry : baseline_trace) {
      std::cerr << " [base " << static_cast<int>(entry.topic) << ":" << entry.rule << ":" << entry.summary << "]";
    }
    for (const auto& entry : branch_trace) {
      std::cerr << " [" << static_cast<int>(entry.topic) << ":" << entry.rule << ":" << entry.summary << "]";
    }
    std::cerr << "\n";
    return false;
  }

  wire::core::BackboneSpec extension{};
  extension.path.polyline = {kBranchB, kCenter};
  extension.interval_m = 1000.0;
  extension.pole_type_id = communication_pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec shared_extension{};
  shared_extension.point_index = 1;
  shared_extension.support_kind = wire::core::SupportKind::kPole;
  shared_extension.node_id = center_id;
  extension.path.node_specs.push_back(shared_extension);
  add_all_templates(extension);
  if (!state.GenerateFromBackboneSpec(extension).ok) {
    std::cerr << "[DBG] C323 second_branch_generate_failed\n";
    return false;
  }

  const wire::core::Vec3d after_extension_axis = support_axis_for_pole(center_id);
  const double extension_alignment = std::abs(dot_xy(after_extension_axis, baseline_axis));
  const auto extension_trace =
      state.view().collect_decision_trace({wire::core::EntityKind::kPole, static_cast<std::uint64_t>(center_id)});
  const auto pole_view = state.view().inspect_pole(center_id);
  const bool ok = extension_alignment >= 0.95 && pole_view.has_value() &&
                  pole_view->forward_rule != wire::core::PoleForwardRule::kFallback &&
                  pole_view->support_axis_rule == wire::core::PoleSupportAxisRule::kMainChainPair;
  if (!ok) {
    std::cerr << "[DBG] C323 extension_alignment=" << extension_alignment << " baseline=(" << baseline_axis.x << ","
              << baseline_axis.y << "," << baseline_axis.z << ") after=(" << after_extension_axis.x << ","
              << after_extension_axis.y << "," << after_extension_axis.z << ") supportRule="
              << static_cast<int>(pole_view.has_value() ? pole_view->support_axis_rule
                                                        : wire::core::PoleSupportAxisRule::kFallback)
              << " forwardRule="
              << static_cast<int>(pole_view.has_value() ? pole_view->forward_rule
                                                        : wire::core::PoleForwardRule::kFallback)
              << "\n";
    for (const auto& entry : extension_trace) {
      std::cerr << "[ext " << static_cast<int>(entry.topic) << ":" << entry.rule << ":" << entry.summary << "]";
    }
  }
  return ok;
}

struct CaptureLikeIncrementalCrossObservation {
  ObjectId center_id = wire::core::kInvalidObjectId;
  ObjectId trunk_prev_id = wire::core::kInvalidObjectId;
  ObjectId trunk_next_id = wire::core::kInvalidObjectId;
  ObjectId branch_id = wire::core::kInvalidObjectId;
  ObjectId opposite_id = wire::core::kInvalidObjectId;
  wire::core::Vec3d baseline_axis{};
  wire::core::Vec3d after_branch_axis{};
  wire::core::Vec3d after_extension_axis{};
  wire::core::PoleForwardRule after_branch_forward_rule = wire::core::PoleForwardRule::kFallback;
  wire::core::PoleForwardRule after_extension_forward_rule = wire::core::PoleForwardRule::kFallback;
  wire::core::PoleSupportAxisRule after_branch_rule = wire::core::PoleSupportAxisRule::kFallback;
  wire::core::PoleSupportAxisRule after_extension_rule = wire::core::PoleSupportAxisRule::kFallback;
  ObjectId after_branch_primary_neighbor_id = wire::core::kInvalidObjectId;
  ObjectId after_branch_secondary_neighbor_id = wire::core::kInvalidObjectId;
  ObjectId after_extension_primary_neighbor_id = wire::core::kInvalidObjectId;
  ObjectId after_extension_secondary_neighbor_id = wire::core::kInvalidObjectId;
};

bool build_capture_like_incremental_cross_observation(CaptureLikeIncrementalCrossObservation* out) {
  if (out == nullptr) {
    return false;
  }

  CoreState state;
  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = pole_type.id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C324/C325 missing CommunicationPole\n";
    return false;
  }

  auto add_all_templates = [](wire::core::BackboneSpec& req) {
    add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
    add_backbone_bundle(req, wire::core::BundleKind::kOptical);
  };
  auto support_axis_for_pole = [&](ObjectId pole_id) {
    const auto pole_view = state.view().inspect_pole(pole_id);
    if (!pole_view.has_value() || !pole_view->has_support_axis) {
      return wire::core::Vec3d{};
    }
    return normalize_xy_safe(pole_view->support_axis_dir);
  };

  constexpr wire::core::Vec3d kTrunkPrev{2.23971, 14.9132, 0.0};
  constexpr wire::core::Vec3d kCenter{-9.08387, 7.15014, 0.0};
  constexpr wire::core::Vec3d kTrunkNext{-16.0024, -3.71055, 0.0};
  constexpr wire::core::Vec3d kBranch{-13.7777, 11.7497, 0.0};
  constexpr wire::core::Vec3d kOpposite{-2.38788, 1.38919, 0.0};

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {kTrunkPrev, kCenter, kTrunkNext};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = communication_pole_type_id;
  add_all_templates(trunk);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    std::cerr << "[DBG] C324/C325 trunk_generate_failed\n";
    return false;
  }

  out->trunk_prev_id = find_pole_id_by_position(state, kTrunkPrev, 1e-4);
  out->center_id = find_pole_id_by_position(state, kCenter, 1e-4);
  out->trunk_next_id = find_pole_id_by_position(state, kTrunkNext, 1e-4);
  if (out->trunk_prev_id == wire::core::kInvalidObjectId || out->center_id == wire::core::kInvalidObjectId ||
      out->trunk_next_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C324/C325 trunk_nodes_missing prev=" << out->trunk_prev_id << " center=" << out->center_id
              << " next=" << out->trunk_next_id << "\n";
    return false;
  }

  out->baseline_axis = support_axis_for_pole(out->center_id);
  if ((out->baseline_axis.x * out->baseline_axis.x + out->baseline_axis.y * out->baseline_axis.y) <= 1e-12) {
    std::cerr << "[DBG] C324/C325 baseline_axis_missing center=" << out->center_id << "\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {kCenter, kBranch};
  branch.interval_m = 1000.0;
  branch.pole_type_id = communication_pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec shared_branch{};
  shared_branch.point_index = 0;
  shared_branch.support_kind = wire::core::SupportKind::kPole;
  shared_branch.node_id = out->center_id;
  branch.path.node_specs.push_back(shared_branch);
  add_all_templates(branch);
  if (!state.GenerateFromBackboneSpec(branch).ok) {
    std::cerr << "[DBG] C324/C325 branch_generate_failed\n";
    return false;
  }
  out->branch_id = find_pole_id_by_position(state, kBranch, 1e-4);
  out->after_branch_axis = support_axis_for_pole(out->center_id);
  if (const auto debug_it = state.view().pole_orientation_debug_records().find(out->center_id);
      debug_it != state.view().pole_orientation_debug_records().end()) {
    out->after_branch_forward_rule = debug_it->second.rule;
    out->after_branch_rule = debug_it->second.support_axis_rule;
    out->after_branch_primary_neighbor_id = debug_it->second.primary_neighbor_id;
    out->after_branch_secondary_neighbor_id = debug_it->second.secondary_neighbor_id;
  } else if (const auto pole_view = state.view().inspect_pole(out->center_id); pole_view.has_value()) {
    out->after_branch_forward_rule = pole_view->forward_rule;
    out->after_branch_rule = pole_view->support_axis_rule;
    out->after_branch_primary_neighbor_id = pole_view->primary_neighbor_id;
    out->after_branch_secondary_neighbor_id = pole_view->secondary_neighbor_id;
  }

  wire::core::BackboneSpec extension{};
  extension.path.polyline = {kOpposite, kCenter};
  extension.interval_m = 1000.0;
  extension.pole_type_id = communication_pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec shared_extension{};
  shared_extension.point_index = 1;
  shared_extension.support_kind = wire::core::SupportKind::kPole;
  shared_extension.node_id = out->center_id;
  extension.path.node_specs.push_back(shared_extension);
  add_all_templates(extension);
  if (!state.GenerateFromBackboneSpec(extension).ok) {
    std::cerr << "[DBG] C324/C325 extension_generate_failed\n";
    return false;
  }
  out->opposite_id = find_pole_id_by_position(state, kOpposite, 1e-4);
  out->after_extension_axis = support_axis_for_pole(out->center_id);

  const auto debug_it = state.view().pole_orientation_debug_records().find(out->center_id);
  if (debug_it != state.view().pole_orientation_debug_records().end()) {
    out->after_extension_forward_rule = debug_it->second.rule;
    out->after_extension_rule = debug_it->second.support_axis_rule;
    out->after_extension_primary_neighbor_id = debug_it->second.primary_neighbor_id;
    out->after_extension_secondary_neighbor_id = debug_it->second.secondary_neighbor_id;
    out->after_extension_axis = normalize_xy_safe(debug_it->second.adopted_support_axis);
  } else if (const auto pole_view = state.view().inspect_pole(out->center_id); pole_view.has_value()) {
    out->after_extension_forward_rule = pole_view->forward_rule;
    out->after_extension_rule = pole_view->support_axis_rule;
    out->after_extension_primary_neighbor_id = pole_view->primary_neighbor_id;
    out->after_extension_secondary_neighbor_id = pole_view->secondary_neighbor_id;
  }
  return out->branch_id != wire::core::kInvalidObjectId && out->opposite_id != wire::core::kInvalidObjectId;
}

bool test_backbone_capture_like_incremental_cross_keeps_trunk_support_axis() {
  CaptureLikeIncrementalCrossObservation observation{};
  if (!build_capture_like_incremental_cross_observation(&observation)) {
    return false;
  }

  const double branch_alignment = std::abs(dot_xy(observation.after_branch_axis, observation.baseline_axis));
  const double extension_alignment = std::abs(dot_xy(observation.after_extension_axis, observation.baseline_axis));
  const bool ok = branch_alignment >= 0.95 && extension_alignment >= 0.95 &&
                  observation.after_branch_forward_rule != wire::core::PoleForwardRule::kFallback &&
                  observation.after_extension_forward_rule != wire::core::PoleForwardRule::kFallback &&
                  observation.after_branch_rule != wire::core::PoleSupportAxisRule::kConnectedDirectionFit &&
                  observation.after_extension_rule != wire::core::PoleSupportAxisRule::kConnectedDirectionFit;
  if (!ok) {
    std::cerr << "[DBG] C324 branchAlign=" << branch_alignment << " extensionAlign=" << extension_alignment
              << " baseline=(" << observation.baseline_axis.x << "," << observation.baseline_axis.y << ") branch=("
              << observation.after_branch_axis.x << "," << observation.after_branch_axis.y << ") extension=("
              << observation.after_extension_axis.x << "," << observation.after_extension_axis.y << ") center="
              << observation.center_id << " trunkPair=" << observation.trunk_prev_id << "/" << observation.trunk_next_id
              << " branchPair=" << observation.branch_id << "/" << observation.opposite_id
              << " branchForwardRule=" << static_cast<int>(observation.after_branch_forward_rule)
              << " extensionForwardRule=" << static_cast<int>(observation.after_extension_forward_rule)
              << " branchRule=" << static_cast<int>(observation.after_branch_rule)
              << " extensionRule=" << static_cast<int>(observation.after_extension_rule) << "\n";
    return false;
  }
  return true;
}

bool test_backbone_capture_like_incremental_cross_preserves_trunk_pair_authority() {
  CaptureLikeIncrementalCrossObservation observation{};
  if (!build_capture_like_incremental_cross_observation(&observation)) {
    return false;
  }

  CoreState state;
  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = pole_type.id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }
  auto add_all_templates = [](wire::core::BackboneSpec& req) {
    add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
    add_backbone_bundle(req, wire::core::BundleKind::kOptical);
  };
  constexpr wire::core::Vec3d kTrunkPrev{2.23971, 14.9132, 0.0};
  constexpr wire::core::Vec3d kCenter{-9.08387, 7.15014, 0.0};
  constexpr wire::core::Vec3d kTrunkNext{-16.0024, -3.71055, 0.0};
  constexpr wire::core::Vec3d kBranch{-13.7777, 11.7497, 0.0};
  constexpr wire::core::Vec3d kOpposite{-2.38788, 1.38919, 0.0};
  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {kTrunkPrev, kCenter, kTrunkNext};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = communication_pole_type_id;
  add_all_templates(trunk);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, kCenter, 1e-4);
  const ObjectId trunk_prev_id = find_pole_id_by_position(state, kTrunkPrev, 1e-4);
  const ObjectId trunk_next_id = find_pole_id_by_position(state, kTrunkNext, 1e-4);
  wire::core::BackboneSpec branch{};
  branch.path.polyline = {kCenter, kBranch};
  branch.interval_m = 1000.0;
  branch.pole_type_id = communication_pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec shared_branch{};
  shared_branch.point_index = 0;
  shared_branch.support_kind = wire::core::SupportKind::kPole;
  shared_branch.node_id = center_id;
  branch.path.node_specs.push_back(shared_branch);
  add_all_templates(branch);
  if (!state.GenerateFromBackboneSpec(branch).ok) {
    return false;
  }
  wire::core::BackboneSpec extension{};
  extension.path.polyline = {kOpposite, kCenter};
  extension.interval_m = 1000.0;
  extension.pole_type_id = communication_pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec shared_extension{};
  shared_extension.point_index = 1;
  shared_extension.support_kind = wire::core::SupportKind::kPole;
  shared_extension.node_id = center_id;
  extension.path.node_specs.push_back(shared_extension);
  add_all_templates(extension);
  if (!state.GenerateFromBackboneSpec(extension).ok) {
    return false;
  }

  const auto debug_it = state.view().pole_orientation_debug_records().find(center_id);
  if (debug_it == state.view().pole_orientation_debug_records().end()) {
    std::cerr << "[DBG] C325 missing_debug center=" << center_id << "\n";
    return false;
  }
  const auto& debug = debug_it->second;
  const std::unordered_set<ObjectId> expected{trunk_prev_id, trunk_next_id};
  const std::unordered_set<ObjectId> actual{debug.primary_neighbor_id, debug.secondary_neighbor_id};
  const bool ok = debug.rule != wire::core::PoleForwardRule::kFallback &&
                  debug.support_axis_rule == wire::core::PoleSupportAxisRule::kMainChainPair && actual == expected;
  if (!ok) {
    std::cerr << "[DBG] C325 forwardRule=" << static_cast<int>(debug.rule)
              << " supportRule=" << static_cast<int>(debug.support_axis_rule) << " primary="
              << debug.primary_neighbor_id << " secondary=" << debug.secondary_neighbor_id << " expected="
              << trunk_prev_id << "/" << trunk_next_id << "\n";
  }
  return ok;
}

bool test_backbone_connected_direction_fit_gate_respects_trunk_preserve() {
  CoreState state;

  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = pole_type.id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  auto add_all_templates = [](wire::core::BackboneSpec& req) {
    add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
    add_backbone_bundle(req, wire::core::BundleKind::kOptical);
  };

  constexpr wire::core::Vec3d kTrunk0{10.8184, 4.94506, 0.0};
  constexpr wire::core::Vec3d kTrunk1{9.90598, 14.2226, 0.0};
  constexpr wire::core::Vec3d kTrunk2{-5.24054, 15.5595, 0.0};
  constexpr wire::core::Vec3d kCenter{-15.9599, -2.86144, 0.0};
  constexpr wire::core::Vec3d kTrunk4{-14.128, -7.28552, 0.0};
  constexpr wire::core::Vec3d kTrunk5{-8.03361, -13.365, 0.0};
  constexpr wire::core::Vec3d kBranchA{-24.2948, -1.36634, 0.0};

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {kTrunk0, kTrunk1, kTrunk2, kCenter, kTrunk4, kTrunk5};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = communication_pole_type_id;
  add_all_templates(trunk);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, kCenter, 1e-4);
  const ObjectId trunk_prev_id = find_pole_id_by_position(state, kTrunk2, 1e-4);
  const ObjectId trunk_next_id = find_pole_id_by_position(state, kTrunk4, 1e-4);
  if (center_id == wire::core::kInvalidObjectId || trunk_prev_id == wire::core::kInvalidObjectId ||
      trunk_next_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {kCenter, kBranchA};
  branch.interval_m = 1000.0;
  branch.pole_type_id = communication_pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec shared_branch{};
  shared_branch.point_index = 0;
  shared_branch.support_kind = wire::core::SupportKind::kPole;
  shared_branch.node_id = center_id;
  branch.path.node_specs.push_back(shared_branch);
  add_all_templates(branch);
  if (!state.GenerateFromBackboneSpec(branch).ok) {
    return false;
  }

  const auto debug_it = state.view().pole_orientation_debug_records().find(center_id);
  if (debug_it == state.view().pole_orientation_debug_records().end()) {
    return false;
  }
  const auto& debug = debug_it->second;
  const std::unordered_set<ObjectId> expected{trunk_prev_id, trunk_next_id};
  const std::unordered_set<ObjectId> actual{debug.primary_neighbor_id, debug.secondary_neighbor_id};
  const bool ok = debug.support_axis_rule == wire::core::PoleSupportAxisRule::kMainChainPair &&
                  debug.support_axis_rule != wire::core::PoleSupportAxisRule::kConnectedDirectionFit &&
                  actual == expected;
  if (!ok) {
    std::cerr << "[DBG] C360 rule=" << static_cast<int>(debug.support_axis_rule) << " primary="
              << debug.primary_neighbor_id << " secondary=" << debug.secondary_neighbor_id << " expected="
              << trunk_prev_id << "/" << trunk_next_id << "\n";
  }
  return ok;
}

bool test_backbone_connected_direction_fit_gate_respects_continuation_pair() {
  CaptureLikeIncrementalCrossObservation observation{};
  if (!build_capture_like_incremental_cross_observation(&observation)) {
    return false;
  }

  const std::unordered_set<ObjectId> expected{observation.trunk_prev_id, observation.trunk_next_id};
  const std::unordered_set<ObjectId> branch_actual{observation.after_branch_primary_neighbor_id,
                                                   observation.after_branch_secondary_neighbor_id};
  const std::unordered_set<ObjectId> extension_actual{observation.after_extension_primary_neighbor_id,
                                                      observation.after_extension_secondary_neighbor_id};
  const bool ok =
      observation.after_branch_rule == wire::core::PoleSupportAxisRule::kMainChainPair &&
      observation.after_extension_rule == wire::core::PoleSupportAxisRule::kMainChainPair &&
      observation.after_branch_rule != wire::core::PoleSupportAxisRule::kConnectedDirectionFit &&
      observation.after_extension_rule != wire::core::PoleSupportAxisRule::kConnectedDirectionFit &&
      branch_actual == expected && extension_actual == expected;
  if (!ok) {
    std::cerr << "[DBG] C361 branchRule=" << static_cast<int>(observation.after_branch_rule)
              << " branchPair=" << observation.after_branch_primary_neighbor_id << "/"
              << observation.after_branch_secondary_neighbor_id << " extensionRule="
              << static_cast<int>(observation.after_extension_rule) << " extensionPair="
              << observation.after_extension_primary_neighbor_id << "/"
              << observation.after_extension_secondary_neighbor_id << " expected=" << observation.trunk_prev_id << "/"
              << observation.trunk_next_id << "\n";
  }
  return ok;
}

bool test_backbone_connected_direction_fit_gate_respects_explicit_pair_axis() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(req).ok) {
    return false;
  }

  const ObjectId left_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId right_id = find_pole_id_by_position(state, {12.0, 0.0, 0.0});
  if (left_id == wire::core::kInvalidObjectId || center_id == wire::core::kInvalidObjectId ||
      right_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto debug_it = state.view().pole_orientation_debug_records().find(center_id);
  if (debug_it == state.view().pole_orientation_debug_records().end()) {
    return false;
  }
  const auto& debug = debug_it->second;
  const std::unordered_set<ObjectId> expected{left_id, right_id};
  const std::unordered_set<ObjectId> actual{debug.primary_neighbor_id, debug.secondary_neighbor_id};
  const bool ok = debug.support_axis_rule == wire::core::PoleSupportAxisRule::kMainChainPair &&
                  debug.support_axis_rule != wire::core::PoleSupportAxisRule::kConnectedDirectionFit &&
                  actual == expected;
  if (!ok) {
    std::cerr << "[DBG] C362 rule=" << static_cast<int>(debug.support_axis_rule) << " primary="
              << debug.primary_neighbor_id << " secondary=" << debug.secondary_neighbor_id << " expected="
              << left_id << "/" << right_id << "\n";
  }
  return ok;
}

bool test_backbone_drawpath_plain_endpoint_fallback_without_attachment_input() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      return false;
    }
    const auto check_endpoint = [](const wire::core::SupportLayoutEndpointView& endpoint) {
      return endpoint.endpoint_source == wire::core::SupportLayoutEndpointSourceKind::kPlainSupport &&
             endpoint.attachment_request.kind == wire::core::EndpointAttachmentRequestKind::kNone &&
             !endpoint.attachment_request.attachment_id.has_value() &&
             !endpoint.attachment_request.requested_socket_id.has_value() && !endpoint.resolved_socket_id.has_value();
    };
    if (!check_endpoint(layout_view->start_endpoint) || !check_endpoint(layout_view->end_endpoint)) {
      return false;
    }
  }
  return true;
}

bool test_backbone_drawpath_branch_curve_stays_local_to_support_departure() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    std::cerr << "[DBG] C179 trunk_generate_failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C179 center_missing_after_trunk\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 18.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C179 branch_generate_failed error=" << branch_generated.error << "\n";
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);

  const ObjectId span_id = branch_generated.value.generated_span_ids.front();
  const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
  const auto span_view = state.view().inspect_span(span_id);
  const auto curve_view = state.view().inspect_detail_curve(span_id);
  const auto* curve = state.find_curve_cache(span_id);
  if (span == nullptr || !span_view.has_value() || !curve_view.has_value() || curve == nullptr) {
    return false;
  }

  const bool center_is_start = span->endpoint_node_a_id == center_id;
  const auto tangent_rule = center_is_start ? curve_view->start_tangent_rule : curve_view->end_tangent_rule;
  const double support_weight = center_is_start ? curve_view->start_support_weight : curve_view->end_support_weight;
  const double chord_weight = center_is_start ? curve_view->start_chord_weight : curve_view->end_chord_weight;
  const double departure_length_m =
      center_is_start ? curve_view->start_departure_length_m : curve_view->end_departure_length_m;
  const double lateral_ratio_limit =
      center_is_start ? curve_view->start_lateral_ratio_limit : curve_view->end_lateral_ratio_limit;
  const auto branch_trace =
      state.view().collect_decision_trace({wire::core::EntityKind::kSpan, static_cast<std::uint64_t>(span_id)});
  const bool branch_trace_has_tangent_policy =
      trace_contains_summary(branch_trace, wire::core::DecisionTraceTopic::kTangentGeneration, "BranchTangentRule",
                             "suppress=");
  std::size_t peak_index = 0;
  const double max_abs_lateral = max_curve_lateral_overshoot_xy(curve->detail, &peak_index);
  const bool peaks_locally =
      max_abs_lateral <= 0.05 || peak_index < (curve->detail.sample_points.size() / 2);
  const BranchRunoutMetrics runout_metrics = measure_branch_runout_metrics(state, span_id);
  if (!(span_view->flow_kind == wire::core::BackboneFlowKind::kBranch && span_view->uses_branch_support &&
          curve_view->shape_policy == wire::core::CurveShapePolicyKind::kBranchPass &&
          tangent_rule == wire::core::DetailCurveEndpointTangentRule::kBranchChordPriority &&
          support_weight < chord_weight && departure_length_m <= 1.10 + 1e-6 &&
          lateral_ratio_limit <= 0.05 + 1e-6 && curve_view->lateral_suppression >= 0.80 &&
          max_abs_lateral <= departure_length_m + 0.05 && peaks_locally && branch_trace_has_tangent_policy &&
          runout_metrics.lateral_runout_ratio <= 0.08 && runout_metrics.local_departure_dominates)) {
    std::cerr << "[DBG] C179 span=" << span_id << " flow=" << static_cast<int>(span_view->flow_kind)
              << " branchSupport=" << (span_view->uses_branch_support ? 1 : 0)
              << " shape=" << static_cast<int>(curve_view->shape_policy)
              << " tangentRule=" << static_cast<int>(tangent_rule) << " supportWeight=" << support_weight
              << " chordWeight=" << chord_weight << " dep=" << departure_length_m
              << " latLimit=" << lateral_ratio_limit << " suppress=" << curve_view->lateral_suppression
              << " maxLat=" << max_abs_lateral << " peakIndex=" << peak_index
              << " samples=" << curve->detail.sample_points.size()
              << " branchTraceHasPolicy=" << (branch_trace_has_tangent_policy ? 1 : 0)
              << " runout=" << describe_branch_runout_metrics(runout_metrics) << "\n";
  }
  return span_view->flow_kind == wire::core::BackboneFlowKind::kBranch && span_view->uses_branch_support &&
         curve_view->shape_policy == wire::core::CurveShapePolicyKind::kBranchPass &&
         tangent_rule == wire::core::DetailCurveEndpointTangentRule::kBranchChordPriority &&
         support_weight < chord_weight && departure_length_m <= 1.10 + 1e-6 &&
         lateral_ratio_limit <= 0.05 + 1e-6 && curve_view->lateral_suppression >= 0.80 &&
         max_abs_lateral <= departure_length_m + 0.05 && peaks_locally && branch_trace_has_tangent_policy &&
         runout_metrics.lateral_runout_ratio <= 0.08 && runout_metrics.local_departure_dominates;
}

bool test_backbone_drawpath_main_and_branch_are_distinct_in_trace_and_inspection() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  const auto trunk_generated = state.GenerateFromBackboneSpec(trunk);
  if (!trunk_generated.ok || trunk_generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C180 trunk_generate_failed error=" << trunk_generated.error << "\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C180 center_missing_after_trunk\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C180 branch_generate_failed error=" << branch_generated.error << "\n";
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);

  const ObjectId main_span_id = trunk_generated.value.generated_span_ids.front();
  const ObjectId branch_span_id = branch_generated.value.generated_span_ids.front();
  const auto main_view = state.view().inspect_span(main_span_id);
  const auto branch_view = state.view().inspect_span(branch_span_id);
  const auto branch_curve_view = state.view().inspect_detail_curve(branch_span_id);
  if (!main_view.has_value() || !branch_view.has_value() || !branch_curve_view.has_value()) {
    return false;
  }

  const auto main_trace =
      state.view().collect_decision_trace({wire::core::EntityKind::kSpan, static_cast<std::uint64_t>(main_span_id)});
  const auto branch_trace =
      state.view().collect_decision_trace({wire::core::EntityKind::kSpan, static_cast<std::uint64_t>(branch_span_id)});
  const bool branch_has_branch_tangent =
      trace_contains_summary(branch_trace, wire::core::DecisionTraceTopic::kTangentGeneration, "BranchTangentRule",
                               "support/chord=");
  const bool branch_has_branch_flow =
      trace_contains_summary(branch_trace, wire::core::DecisionTraceTopic::kFlowClassification, "", "flow=Branch");
  const bool main_has_main_flow = trace_contains_summary(main_trace, wire::core::DecisionTraceTopic::kFlowClassification,
                                                         "", "flow=Main");
  const bool main_has_main_tangent = trace_contains_summary(main_trace, wire::core::DecisionTraceTopic::kTangentGeneration,
                                                            "MainTangentRule", "support/chord=");
  const bool branch_has_lateral_policy =
      trace_contains_summary(branch_trace, wire::core::DecisionTraceTopic::kTangentGeneration, "BranchTangentRule",
                             "suppress=");
  if (!(main_view->flow_kind == wire::core::BackboneFlowKind::kMain && !main_view->uses_branch_support &&
          branch_view->flow_kind == wire::core::BackboneFlowKind::kBranch && branch_view->uses_branch_support &&
          branch_curve_view->shape_policy == wire::core::CurveShapePolicyKind::kBranchPass && main_has_main_flow &&
          main_has_main_tangent && branch_has_branch_flow && branch_has_branch_tangent &&
          branch_curve_view->lateral_suppression >= 0.80 && branch_has_lateral_policy)) {
    std::cerr << "[DBG] C180 mainSpan=" << main_span_id << " mainFlow=" << static_cast<int>(main_view->flow_kind)
              << " mainBranchSupport=" << (main_view->uses_branch_support ? 1 : 0) << " branchSpan=" << branch_span_id
              << " branchFlow=" << static_cast<int>(branch_view->flow_kind)
              << " branchSupport=" << (branch_view->uses_branch_support ? 1 : 0)
              << " branchShape=" << static_cast<int>(branch_curve_view->shape_policy)
              << " branchSuppress=" << branch_curve_view->lateral_suppression
              << " mainHasFlow=" << (main_has_main_flow ? 1 : 0)
              << " mainHasTangent=" << (main_has_main_tangent ? 1 : 0)
              << " branchHasFlow=" << (branch_has_branch_flow ? 1 : 0)
              << " branchHasTangent=" << (branch_has_branch_tangent ? 1 : 0)
              << " branchHasLateralPolicy=" << (branch_has_lateral_policy ? 1 : 0) << "\n";
  }
  return main_view->flow_kind == wire::core::BackboneFlowKind::kMain && !main_view->uses_branch_support &&
         branch_view->flow_kind == wire::core::BackboneFlowKind::kBranch && branch_view->uses_branch_support &&
         branch_curve_view->shape_policy == wire::core::CurveShapePolicyKind::kBranchPass && main_has_main_flow &&
         main_has_main_tangent && branch_has_branch_flow && branch_has_branch_tangent &&
         branch_curve_view->lateral_suppression >= 0.80 && branch_has_lateral_policy;
}

bool test_variation_settings_do_not_change_topology_flow_or_mirror() {
  auto generate_assignments = [](std::uint64_t seed) {
    CoreState state;
    wire::core::VariationSettings variation = state.view().variation_settings();
    variation.enabled = true;
    variation.global_seed = seed;
    variation.sag_variation_scale = 0.35;
    variation.branch_down_offset_variation_scale = 0.10;
    if (!state.UpdateVariationSettings(variation, true).ok) {
      return std::vector<wire::core::SegmentLaneAssignment>{};
    }

    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return std::vector<wire::core::SegmentLaneAssignment>{};
    }

    wire::core::BackboneSpec trunk{};
    trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
    trunk.interval_m = 1000.0;
    trunk.pole_type_id = type_ids.front();
    add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
    if (!state.GenerateFromBackboneSpec(trunk).ok) {
      return std::vector<wire::core::SegmentLaneAssignment>{};
    }

    const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
    if (center_id == wire::core::kInvalidObjectId) {
      return std::vector<wire::core::SegmentLaneAssignment>{};
    }

    wire::core::BackboneSpec branch{};
    branch.path.polyline = {{0.0, 0.0, 0.0}, {9.0, 6.0, 0.0}};
    wire::core::BackboneInputSpec::NodeSpec shared{};
    shared.point_index = 0;
    shared.support_kind = wire::core::SupportKind::kPole;
    shared.node_id = center_id;
    branch.path.node_specs.push_back(shared);
    branch.interval_m = 1000.0;
    branch.pole_type_id = type_ids.front();
    add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
    if (!state.GenerateFromBackboneSpec(branch).ok) {
      return std::vector<wire::core::SegmentLaneAssignment>{};
    }
    return state.view().last_lane_assignments();
  };

  const auto a = generate_assignments(1001);
  const auto b = generate_assignments(2002);
  if (a.empty() || a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].flow_kind != b[i].flow_kind || a[i].flow_decision_rule != b[i].flow_decision_rule ||
        a[i].order_decision_choice_a != b[i].order_decision_choice_a ||
        a[i].order_decision_choice_b != b[i].order_decision_choice_b ||
        a[i].flipped_from_previous != b[i].flipped_from_previous ||
        a[i].variation_flow_key != b[i].variation_flow_key) {
      return false;
    }
  }
  return true;
}

bool test_backbone_adjacent_branch_roots_use_route_local_bisector() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-20.0, 0.0, 0.0}, {-10.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    std::cerr << "[DBG] C279 trunk_generate_failed\n";
    return false;
  }

  const ObjectId root_a_id = find_pole_id_by_position(state, {-10.0, 0.0, 0.0});
  const ObjectId root_b_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (root_a_id == wire::core::kInvalidObjectId || root_b_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C279 root_missing a=" << root_a_id << " b=" << root_b_id << "\n";
    return false;
  }

  wire::core::BackboneSpec branch_a{};
  branch_a.path.polyline = {{-10.0, 0.0, 0.0}, {-18.0, -12.0, 0.0}};
  branch_a.interval_m = 1000.0;
  branch_a.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared_a{};
  shared_a.point_index = 0;
  shared_a.support_kind = wire::core::SupportKind::kPole;
  shared_a.node_id = root_a_id;
  branch_a.path.node_specs.push_back(shared_a);
  add_backbone_bundle(branch_a, wire::core::BundleKind::kHighVoltage);
  const auto generated_a = state.GenerateFromBackboneSpec(branch_a);
  if (!generated_a.ok || generated_a.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C279 branch_a_generate_failed error=" << generated_a.error << "\n";
    return false;
  }

  wire::core::BackboneSpec branch_b{};
  branch_b.path.polyline = {{0.0, 0.0, 0.0}, {6.0, 14.0, 0.0}};
  branch_b.interval_m = 1000.0;
  branch_b.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared_b{};
  shared_b.point_index = 0;
  shared_b.support_kind = wire::core::SupportKind::kPole;
  shared_b.node_id = root_b_id;
  branch_b.path.node_specs.push_back(shared_b);
  add_backbone_bundle(branch_b, wire::core::BundleKind::kHighVoltage);
  const auto generated_b = state.GenerateFromBackboneSpec(branch_b);
  if (!generated_b.ok || generated_b.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C279 branch_b_generate_failed error=" << generated_b.error << "\n";
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] C279 commit_failed\n";
    return false;
  }

  struct RootObservation {
    wire::core::Vec3d support_axis{};
    wire::core::Vec3d expected_axis{};
    wire::core::SupportOrientationRuleKind rule = wire::core::SupportOrientationRuleKind::kRadial;
    wire::core::SupportOrientationBasisKind basis = wire::core::SupportOrientationBasisKind::kRadial;
    double alignment = -1.0;
    int group_id = -1;
    ObjectId span_id = wire::core::kInvalidObjectId;
  };

  const auto observe_root = [&](const std::vector<ObjectId>& span_ids, ObjectId owner_pole_id,
                                const wire::core::Vec3d& branch_tip_world,
                                const wire::core::Vec3d& route_local_peer_world) -> std::optional<RootObservation> {
    const auto pole_view = state.view().inspect_pole(owner_pole_id);
    if (!pole_view.has_value()) {
      return std::nullopt;
    }
    const wire::core::Vec3d owner_world = pole_view->position;
    const wire::core::Vec3d expected_axis =
        normalize_xy_safe((branch_tip_world - owner_world) + (route_local_peer_world - owner_world));
    for (ObjectId span_id : span_ids) {
      const auto layout_view = state.view().inspect_support_layout(span_id);
      if (!layout_view.has_value()) {
        continue;
      }
      const auto endpoint = layout_endpoint_for_owner(*layout_view, owner_pole_id);
      const auto group = lowered_support_group_for_owner(*layout_view, owner_pole_id);
      if (!endpoint.has_value() || !group.has_value()) {
        continue;
      }
      RootObservation observation{};
      observation.support_axis = normalize_xy_safe(group->tip_world - group->mount_world);
      observation.expected_axis = expected_axis;
      observation.rule = group->support_orientation_rule;
      observation.basis = group->support_orientation_basis;
      observation.alignment = dot_xy(observation.support_axis, observation.expected_axis);
      observation.group_id = group->support_group_id;
      observation.span_id = span_id;
      return observation;
    }
    return std::nullopt;
  };

  const auto root_a = observe_root(generated_a.value.generated_span_ids, root_a_id, {-18.0, -12.0, 0.0},
                                   {0.0, 0.0, 0.0});
  const auto root_b = observe_root(generated_b.value.generated_span_ids, root_b_id, {6.0, 14.0, 0.0},
                                   {10.0, 0.0, 0.0});
  if (!root_a.has_value() || !root_b.has_value()) {
    std::cerr << "[DBG] C279 missing_observation a=" << (root_a.has_value() ? 1 : 0)
              << " b=" << (root_b.has_value() ? 1 : 0) << "\n";
    return false;
  }

  const bool a_ok = root_a->rule == wire::core::SupportOrientationRuleKind::kBisector &&
                    root_a->basis != wire::core::SupportOrientationBasisKind::kRadial && root_a->alignment >= 0.94 &&
                    root_a->support_axis.y < -0.10;
  const bool b_ok = root_b->rule == wire::core::SupportOrientationRuleKind::kBisector &&
                    root_b->basis != wire::core::SupportOrientationBasisKind::kRadial && root_b->alignment >= 0.94 &&
                    root_b->support_axis.y > 0.10;
  const bool opposite_y = (root_a->support_axis.y * root_b->support_axis.y) < -0.01;
  if (!(a_ok && b_ok && opposite_y)) {
    std::cerr << "[DBG] C279"
              << " aSpan=" << root_a->span_id << " aGroup=" << root_a->group_id << " aRule="
              << static_cast<int>(root_a->rule) << " aBasis=" << static_cast<int>(root_a->basis)
              << " aAlign=" << root_a->alignment << " aAxis=(" << root_a->support_axis.x << ","
              << root_a->support_axis.y << "," << root_a->support_axis.z << ")"
              << " aExpected=(" << root_a->expected_axis.x << "," << root_a->expected_axis.y << ","
              << root_a->expected_axis.z << ")"
              << " bSpan=" << root_b->span_id << " bGroup=" << root_b->group_id << " bRule="
              << static_cast<int>(root_b->rule) << " bBasis=" << static_cast<int>(root_b->basis)
              << " bAlign=" << root_b->alignment << " bAxis=(" << root_b->support_axis.x << ","
              << root_b->support_axis.y << "," << root_b->support_axis.z << ")"
              << " bExpected=(" << root_b->expected_axis.x << "," << root_b->expected_axis.y << ","
              << root_b->expected_axis.z << ")"
              << " oppositeY=" << (opposite_y ? 1 : 0) << "\n";
    return false;
  }

  return true;
}

bool test_backbone_main_bisector_support_axis_stays_perpendicular_on_acute_branch_root() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {
      {4.09661, -1.6937, 0.0},
      {6.62541, -4.13897, 0.0},
      {8.93477, -4.88286, 0.0},
      {13.4003, -2.73847, 0.0},
      {14.9671, -0.460638, 0.0},
  };
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    std::cerr << "[DBG] C285 trunk_generate_failed\n";
    return false;
  }

  const ObjectId root_id = find_pole_id_by_position(state, {8.93477, -4.88286, 0.0});
  if (root_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C285 root_missing\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{8.93477, -4.88286, 0.0}, {10.5102, 1.17011, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = root_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C285 branch_generate_failed error=" << generated.error << "\n";
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    std::cerr << "[DBG] C285 commit_failed\n";
    return false;
  }

  const auto pole_view = state.view().inspect_pole(root_id);
  if (!pole_view.has_value() || !pole_view->has_support_axis ||
      pole_view->forward_rule != wire::core::PoleForwardRule::kMainChainBisector) {
    std::cerr << "[DBG] C285 pole_view_missing_or_rule"
              << " has=" << (pole_view.has_value() ? 1 : 0)
              << " axis=" << (pole_view.has_value() && pole_view->has_support_axis ? 1 : 0)
              << " rule="
              << (pole_view.has_value() ? static_cast<int>(pole_view->forward_rule)
                                        : static_cast<int>(wire::core::PoleForwardRule::kFallback))
              << "\n";
    return false;
  }

  const double yaw_rad = pole_view->final_yaw_deg * (3.14159265358979323846 / 180.0);
  const wire::core::Vec3d forward = normalize_xy_safe({std::cos(yaw_rad), std::sin(yaw_rad), 0.0});
  const wire::core::Vec3d support_axis = normalize_xy_safe(pole_view->support_axis_dir);
  const wire::core::Vec3d lateral = normalize_xy_safe({-forward.y, forward.x, 0.0});
  const double forward_alignment = std::abs(dot_xy(support_axis, forward));
  const double lateral_alignment = std::abs(dot_xy(support_axis, lateral));
  if (!(forward_alignment <= 0.35 && lateral_alignment >= 0.93)) {
    std::cerr << "[DBG] C285 pole=" << root_id << " rule=" << static_cast<int>(pole_view->forward_rule)
              << " supportRule=" << static_cast<int>(pole_view->support_axis_rule) << " forward=(" << forward.x << ","
              << forward.y << "," << forward.z << ") support=(" << support_axis.x << "," << support_axis.y << ","
              << support_axis.z << ") lateral=(" << lateral.x << "," << lateral.y << "," << lateral.z
              << ") forwardAlign=" << forward_alignment << " lateralAlign=" << lateral_alignment << "\n";
    return false;
  }

  return true;
}

bool test_backbone_same_pole_branch_roots_keep_distinct_support_groups() {
  const std::vector<ObjectId> node_ids{55, 316, 400};
  const std::unordered_map<ObjectId, wire::core::SupportNode> support_node_by_id{
      {55, wire::core::SupportNode{.node_id = 55, .support_kind = wire::core::SupportKind::kPole, .position = {0.0, 0.0, 0.0}, .pole_id = 55}},
      {316, wire::core::SupportNode{.node_id = 316, .support_kind = wire::core::SupportKind::kPole, .position = {6.0, 14.0, 0.0}, .pole_id = 316}},
      {400, wire::core::SupportNode{.node_id = 400, .support_kind = wire::core::SupportKind::kPole, .position = {-6.0, 14.0, 0.0}, .pole_id = 400}},
  };
  const wire::core::EditState edit_state{};
  const wire::core::RelationIndex relation_index{};
  const wire::core::ConnectionIndex connection_index{};
  const wire::core::generation::detail::GroupedSpanSharedContext ctx{
      .node_ids = node_ids,
      .support_node_by_id = support_node_by_id,
      .edit_state = edit_state,
      .relation_index = relation_index,
      .connection_index = connection_index,
      .junction_relations_by_node = nullptr,
  };
  wire::core::BackboneLoweringPolicy lowering_policy{};
  lowering_policy.offset_m = 0.275;
  lowering_policy.enable_branch_support = true;

  wire::core::generation::detail::GroupedSpanLoweringDecider lowering(ctx, lowering_policy,
                                                                      wire::core::BundleKind::kHighVoltage, 0.4, 0.2);

  wire::core::generation::detail::SegmentRelationFeasibility feasibility{};
  feasibility.kind = wire::core::JunctionRelationKind::kSideBranch;
  feasibility.continuity_class = wire::core::ContinuityCategoryClass::kBundleLike;
  feasibility.in_through_pair = false;
  feasibility.default_lower_required = true;
  feasibility.same_level_feasible = false;
  feasibility.reason = wire::core::SameLevelFeasibilityReason::kBundleRule;

  wire::core::generation::detail::EndpointSideDecision side_decision{};
  side_decision.has_side_axis = true;
  side_decision.side_axis = {1.0, 0.0, 0.0};
  side_decision.chosen_side_sign = 1.0;
  side_decision.side_assignment_rule = wire::core::SideAssignmentRuleKind::kBisector;
  side_decision.support_orientation_rule = wire::core::SupportOrientationRuleKind::kBisector;

  const auto first = lowering.BuildEndpointDecision(feasibility, std::nullopt, side_decision, 55, 316,
                                                    wire::core::OrderDecisionPolicyKind::kFixedOrder,
                                                    wire::core::OrderDecisionChoiceKind::kNormal,
                                                    wire::core::OrderDecisionChoiceReason::kFixedOrder, false, false,
                                                    false, false);
  const auto second = lowering.BuildEndpointDecision(feasibility, std::nullopt, side_decision, 55, 400,
                                                     wire::core::OrderDecisionPolicyKind::kFixedOrder,
                                                     wire::core::OrderDecisionChoiceKind::kNormal,
                                                     wire::core::OrderDecisionChoiceReason::kFixedOrder, false, false,
                                                     false, false);
  const auto repeat_first = lowering.BuildEndpointDecision(feasibility, std::nullopt, side_decision, 55, 316,
                                                           wire::core::OrderDecisionPolicyKind::kFixedOrder,
                                                           wire::core::OrderDecisionChoiceKind::kNormal,
                                                           wire::core::OrderDecisionChoiceReason::kFixedOrder, false,
                                                           false, false, false);

  if (!(first.support_group_id >= 0 && second.support_group_id >= 0 &&
        first.support_group_id != second.support_group_id && first.support_group_id == repeat_first.support_group_id)) {
    std::cerr << "[DBG] C282 firstGroup=" << first.support_group_id << " secondGroup=" << second.support_group_id
              << " repeatFirstGroup=" << repeat_first.support_group_id << "\n";
    return false;
  }

  return true;
}

// Intent: Backbone generation must require bundles[] and reject legacy-only fields.
namespace {

void register_generation_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C38_GroupedLine_HV3", "Grouped line generation creates 3-lane high-voltage spans",
                         "Invariant", false, test_generate_grouped_line_high_voltage_three_phase);
  test_registry::AddTest(tests, "C39_Direction_ForcedReverse", "Grouped line honors forced reverse direction", "Exact",
                         false, test_generate_grouped_line_direction_forced_reverse);
  test_registry::AddTest(tests, "C47_Backbone_HVDefaultLanes",
                         "Backbone generation creates default HV bundle spans", "Invariant", false,
                         test_generate_from_backbone_spec_basic_hv_default_lanes);
  test_registry::AddTest(tests, "C48_Backbone_DirectionModes",
                         "Backbone generation supports Forward/Reverse/Auto direction modes", "Invariant", false,
                         test_generate_from_backbone_spec_direction_modes_nonfailing);
  test_registry::AddTest(tests, "C49_Backbone_InvalidInputs",
                         "Backbone generation rejects invalid input and remains recoverable", "Exact", true,
                         test_generate_from_backbone_spec_invalid_inputs_fail);
  test_registry::AddTest(tests, "C62_GroupedLine_NoInversionUPath",
                         "U-shaped grouped path keeps conductor order inversion-free", "Invariant", false,
                         test_grouped_line_lane_order_no_inversion_on_u_path);
  test_registry::AddTest(tests, "C63_GroupedLine_MirrorMetricNonRegression",
                         "Mirror enablement does not worsen grouped ordering metrics", "Invariant", false,
                         test_grouped_line_mirror_metric_non_regression);
  test_registry::AddTest(tests, "C76_GroupedLine_AcuteCornerNoInversion",
                         "Acute corner grouped path keeps conductor order stable", "Invariant", false,
                         test_grouped_line_acute_corner_no_inversion);
  test_registry::AddTest(tests, "C86_GroupedLine_AcutePatternSuite",
                         "Acute pattern suite stays inversion-free", "Invariant", false,
                         test_grouped_line_acute_pattern_suite_no_inversion);
  test_registry::AddTest(tests, "C87_GroupedLine_HV3AcuteNoTwist",
                         "HV3 acute grouped path avoids phase twist", "Invariant", false,
                         test_grouped_line_hv3_acute_no_phase_twist);
  test_registry::AddTest(tests, "C89_ThreePhasePolicy_CategoryAgnostic",
                         "Three-phase policy remains category agnostic", "Invariant", false,
                         test_grouped_line_threephase_policy_is_category_agnostic);
  test_registry::AddTest(tests, "C98_Backbone_ExtensionBoundaryOrder",
                         "Backbone extension preserves boundary conductor ordering", "Invariant", false,
                         test_backbone_extension_preserves_boundary_lane_order);
  test_registry::AddTest(tests, "C208_Backbone_IntervalExtensionBoundaryOrder",
                         "Interval-driven backbone extension keeps conductor ordering continuous at reused poles",
                         "Invariant", false, test_backbone_interval_extension_preserves_boundary_lane_order);
  test_registry::AddTest(tests, "C99_Backbone_HV3CaptureNoInversion",
                         "Captured HV3 backbone shape stays inversion-free", "Invariant", false,
                         test_backbone_hv3_capture_shape_no_inversion);
  test_registry::AddTest(tests, "C100_Backbone_MidairSupportNode",
                         "Backbone generation keeps explicit Midair support nodes", "Invariant", false,
                         test_backbone_generation_includes_midair_support_nodes);
  test_registry::AddTest(tests, "C103_Backbone_BuildingSupportDetailStable",
                         "Detail generation remains stable with building support nodes", "Invariant", false,
                         test_backbone_detail_generation_handles_building_support_node);
  test_registry::AddTest(tests, "C109_Backbone_HV3CaptureNoAdjacentCrossings",
                         "Captured HV3 backbone shape keeps interior shared-pole lane order continuous",
                         "Invariant", false,
                         test_backbone_hv3_capture_shape_no_adjacent_crossings);
  test_registry::AddTest(tests, "C311_Backbone_HV3LatestCaptureNoTwist",
                         "Latest captured CommunicationPole HV backbone shape keeps route parity and lane polylines continuous",
                         "Invariant", false, test_backbone_hv3_latest_capture_shape_no_twist);
  test_registry::AddTest(tests, "C314_Backbone_HV3LatestCaptureVariantBankNoTwist",
                         "Latest captured CommunicationPole HV shape stays parity/polyline-continuous across a fixed perturbation bank",
                         "Invariant", false, test_backbone_hv3_latest_capture_variant_bank_no_twist);
  test_registry::AddTest(tests, "C316_Backbone_HV3LatestCaptureFinalCurveNoTwist",
                         "Latest captured CommunicationPole HV shape keeps final detail curves untwisted after recalc",
                         "Invariant", false, test_backbone_hv3_latest_capture_final_curve_no_twist);
  test_registry::AddTest(tests, "C319_Backbone_HV3LatestCaptureCommitPreservesPolylineOrder",
                         "Latest captured CommunicationPole HV shape keeps lane polylines non-crossing across recalc",
                         "Invariant", false, test_backbone_hv3_latest_capture_commit_preserves_polyline_order);
  test_registry::AddTest(tests, "C320_Backbone_NearDuplicateAutoCandidateBoundary",
                         "Near-duplicate auto generic pole candidates collapse only below the 1.5m threshold",
                         "Invariant", false, test_backbone_auto_candidate_near_duplicate_boundary);
  test_registry::AddTest(tests, "C321_Backbone_ManualAndTangentCandidatesDoNotCollapse",
                         "Manual vertices and tangent-hint vertices are preserved even when they are within the near-duplicate auto-candidate threshold",
                         "Invariant", false, test_backbone_manual_and_tangent_candidates_do_not_collapse);
  test_registry::AddTest(tests, "C322_Backbone_ShortUnpinnedEndpointSegmentsKeepEndpoints",
                         "Short unpinned leading/trailing segments preserve the route endpoints instead of erasing them during near-duplicate auto-candidate collapse",
                         "Invariant", false, test_backbone_short_unpinned_endpoint_segments_keep_endpoints);
  test_registry::AddTest(tests, "C317_Backbone_HV3LatestCaptureLoweredSupportUsesLocalDepartureProfile",
                         "Latest captured lowered HV spans declare a local-departure detail-curve profile",
                         "Invariant", false, test_backbone_hv3_latest_capture_lowered_support_uses_local_departure_profile);
  test_registry::AddTest(tests, "C318_Backbone_HV3LatestCaptureLoweredSupportDepartureUsesSharedRouteAxis",
                         "Latest captured lowered HV spans keep one shared route-local side-axis across an interior lowered corner",
                         "Invariant", false, test_backbone_hv3_latest_capture_lowered_support_departure_uses_shared_route_axis);
  test_registry::AddTest(tests, "C110_Backbone_ReuseExplicitPoleNode",
                         "Backbone generation reuses explicitly picked pole nodes", "Invariant", false,
                         test_backbone_generation_reuses_explicit_pole_node_id);
  test_registry::AddTest(tests, "C111_Backbone_ReuseExplicitSupportNode",
                         "Backbone generation reuses explicitly picked non-pole support nodes", "Invariant", false,
                         test_backbone_generation_reuses_explicit_support_node_id);
  test_registry::AddTest(tests, "C112_Backbone_MidairExtensionDetailChain",
                         "Midair extension still produces detail poles and spans", "Invariant", false,
                         test_backbone_midair_extension_generates_detail_chain);
  test_registry::AddTest(tests, "C113_Backbone_MidairExtensionFirstSupportSegment",
                         "Midair extension includes the first support-to-detail segment in the route",
                         "Invariant", false, test_backbone_midair_extension_includes_first_support_segment);
  test_registry::AddTest(tests, "C114_Backbone_MidairBranchUsesSourceSpanHeight",
                         "Midair branch keeps backbone pick at abstract height while detail starts from source span height",
                         "Invariant", false, test_backbone_midair_branch_reuses_source_span_height);
  test_registry::AddTest(tests, "C115_Backbone_MidairSingleClickNoExtraBridge",
                         "Midair extension from one clicked endpoint keeps a single direct segment", "Invariant", false,
                         test_backbone_midair_extension_single_click_stays_single_segment);
  test_registry::AddTest(tests, "C116_Backbone_TemplateSkipsMidairBranchGeneration",
                         "Disallowed template skips source-edge midair branch generation without failing request",
                         "Invariant", false, test_backbone_midair_branch_skips_disallowed_template_generation);
  test_registry::AddTest(tests, "C118_Backbone_MidairBranchMixedTemplates",
                         "Midair branch generation keeps only templates that allow midair branch", "Invariant", false,
                         test_backbone_midair_branch_generates_only_allowed_templates);
  test_registry::AddTest(tests, "C133_Backbone_MainChainPoleForward",
                         "Reused junction pole keeps forward aligned to the existing main chain", "Invariant", false,
                         test_backbone_reused_junction_pole_keeps_main_chain_forward);
  test_registry::AddTest(tests, "C134_Backbone_BundleBranchCreatesGroupedLoweredSupportAtRoot",
                         "Bundle-like branch root exposes authoritative lowered decision and grouped support", "Invariant",
                         false, test_backbone_branch_bundle_uses_branch_support_ports);
  test_registry::AddTest(tests, "C135_Backbone_BranchSupportDownOffset",
                         "Branch support lowers attachment height without rewriting layer semantics", "Invariant",
                         false, test_backbone_branch_support_offsets_height_without_changing_layer);
  test_registry::AddTest(tests, "C193_Backbone_BundleBranchUsesOneSharedLowerStep",
                         "HV3 bundle branch uses one shared lower step across the whole bundle while keeping a perpendicular row",
                         "Invariant", false, test_backbone_branch_support_lowers_hv3_bundle_uniformly);
  test_registry::AddTest(tests, "C195_Backbone_BundleBranchLoweringStaysLocalToRootPole",
                         "Bundle-like branch lowering stays on the branch root and downstream branch poles return to a flat perpendicular row",
                         "Invariant", false, test_backbone_branch_support_stays_local_to_root_pole);
  test_registry::AddTest(tests, "C196_Backbone_BranchSupportVisualPerpendicular",
                         "Branch support visual placement stays perpendicular to the branch and hangs vertically under the lane attachment",
                         "Invariant", false, test_backbone_branch_support_visual_stays_perpendicular_to_branch);
  test_registry::AddTest(tests, "C286_Backbone_TiltedBranchSupportFollowsTiltedPoleCenterline",
                         "Tilted pole refresh moves grouped branch support with the tilted pole centerline instead of reusing pre-tilt XY anchors",
                         "Invariant", false, test_backbone_tilted_branch_support_follows_tilted_pole_centerline);
  test_registry::AddTest(tests, "C197_Backbone_DefaultSingleBranchStaysFlat",
                         "DefaultSingle branch stays branch-flow without authoritative lowered decision or height drop",
                         "Invariant", false, test_backbone_default_single_branch_stays_flat_without_branch_support);
  test_registry::AddTest(tests, "C198_Backbone_CommunicationBundleBranchPolicyBlocked",
                         "Communication multi-bundle branch exposes a policy-blocked lowered decision without materializing grouped support",
                         "Invariant", false, test_backbone_communication_bundle_branch_stays_flat_without_branch_support);
  test_registry::AddTest(tests, "C199_Backbone_HV3BundleBranchUsesOneSharedLowerStepOnBothPoleTypes",
                         "HV3 bundle branch stays on one shared lower step and keeps a perpendicular grouped support on both default pole types",
                         "Invariant", false, test_backbone_hv3_branch_support_policy_applies_on_both_default_pole_types);
  test_registry::AddTest(tests, "C204_Backbone_HV3CornerUsesOneSharedLowerStep",
                         "HV3 corner continuation uses the same one-step lowered height across the whole bundle without turning it into branch support",
                         "Invariant", false, test_backbone_hv3_acute_corner_lowers_corner_bundle_without_branch_support);
  test_registry::AddTest(tests, "C205_Backbone_HV3CornerOneStepLowerSurvivesPoleRefresh",
                         "HV3 corner one-step lowered support identity survives pole refresh without dropping back to template height",
                         "Invariant", false, test_backbone_hv3_acute_corner_lowering_survives_pole_refresh);
  test_registry::AddTest(tests, "C290_Backbone_HV3CornerConstrainedSolverSpreadsRootPorts",
                         "HV3 acute-corner constrained solver keeps the corner-root lane ports on distinct slots instead of collapsing them onto one point",
                         "Invariant", false, test_backbone_hv3_corner_constrained_solver_spreads_root_ports);
  test_registry::AddTest(tests, "C209_Backbone_HV3ModerateCornerUsesOneSharedLowerStep",
                         "HV3 moderate corner still collapses to one shared lower step under the default corner threshold",
                         "Invariant", false, test_backbone_hv3_moderate_acute_corner_lowers_bundle_at_default_threshold);
  test_registry::AddTest(tests, "C194_Backbone_JunctionPrefersStraighterMainPair",
                         "Junction main selection prefers the straighter continuation pair over first-drawn primary",
                         "Invariant", false, test_backbone_junction_prefers_straighter_pair_over_first_drawn_primary);
  test_registry::AddTest(tests, "C201_Backbone_PointLikeCrossCanStaySameLevel",
                         "Point-like cross keeps branch classification but may stay same-level when near-node clearance allows",
                         "Invariant", false, test_backbone_cross_junction_nonmain_line_uses_underpass);
  test_registry::AddTest(tests, "C210_Backbone_AllTemplatesBranchKeepsHVAtOneStepLower",
                         "All-template DrawPath branch on communication poles keeps HV at one shared lower step",
                         "Invariant", false,
                         test_backbone_all_templates_branch_keeps_hv_down_offset_on_communication_pole);
  test_registry::AddTest(tests, "C211_Backbone_AllTemplatesCrossStillUsesUnderpassOnCommunicationPole",
                         "All-template DrawPath cross on communication poles still keeps at least one non-main line lowered",
                         "Invariant", false,
                         test_backbone_all_templates_cross_keeps_underpass_on_communication_pole);
  test_registry::AddTest(tests, "C212_Backbone_CaptureBranchAndCornerShareOneStepLowerOnCommunicationPole",
                         "Captured all-template communication-pole path keeps the HV branch root and later corner segments on the same one-step lowered height",
                         "Invariant", false, test_backbone_capture_branch_then_acute_lowering_on_communication_pole);
  test_registry::AddTest(tests, "C264_Inspection_AllTemplatesBranchKeepsHVOneStepLowerOnCommunicationPole",
                         "Inspection surface keeps HV branch lowering on the same one-step grouped height in the all-template branch case",
                         "Invariant", false,
                         test_inspection_all_templates_branch_keeps_hv_lowering_on_communication_pole);
  test_registry::AddTest(tests, "C265_Inspection_CaptureKeepsBranchAndCornerOnOneStepLower",
                         "Inspection surface keeps the first HV branch segment and later HV corner segments on the same one-step lowered height in the captured communication-pole case",
                         "Invariant", false,
                         test_inspection_capture_keeps_branch_then_acute_lowering_on_communication_pole);
  test_registry::AddTest(tests, "C266_Inspection_SpanReadsFlowAndTurnFromLaneSnapshot",
                         "inspect_span reads flow rule and turn/flip metadata from the authoritative lane snapshot instead of leaving lane-era fields stale",
                         "Invariant", false, test_inspection_span_reads_flow_and_turn_from_lane_snapshot);
  test_registry::AddTest(tests, "C213_Backbone_RightAngleJunctionHasNoThroughPair",
                         "Right-angle junction exposes a rejected through pair and corner continuation relation in inspection/trace",
                         "Invariant", false, test_backbone_right_angle_junction_has_no_through_pair);
  test_registry::AddTest(tests, "C214_Backbone_LocalCornerProjectsToMainWithoutLocalThrough",
                         "Local corner can project to semantic main while inspection still reports no local through pair",
                         "Invariant", false, test_backbone_local_corner_projects_to_main_without_local_through);
  test_registry::AddTest(tests, "C215_Backbone_SeparateRouteMergeKeepsCornerContinuationRelation",
                         "Separate-route merge still records corner continuation and acute lowering on the later corner node",
                         "Invariant", false, test_backbone_separate_route_merge_keeps_corner_continuation_relation);
  test_registry::AddTest(tests, "C216_Backbone_MirrorDoesNotChangeRelationOrLoweringRoot",
                         "Mirror enablement changes lane order choices only and does not change relation or lowering roots",
                         "Invariant", false, test_backbone_mirror_does_not_change_relation_or_lowering_root);
  test_registry::AddTest(tests, "C217_Backbone_CrossAcceptedThroughButSameLevelInfeasible",
                         "Cross junction can accept a through pair while the non-through HV3 route still fails same-level clearance and lowers",
                         "Invariant", false, test_backbone_cross_through_pair_can_still_be_same_level_infeasible);
  test_registry::AddTest(tests, "C218_Backbone_CommBranchSameLevelBlockedByPolicy",
                         "Communication bundle branch can be same-level infeasible while category policy blocks lowering and reports that block",
                         "Invariant", false, test_backbone_comm_branch_same_level_can_be_blocked_by_policy);
  test_registry::AddTest(tests, "C219_Backbone_HV3CornerFeasibilityLoweringKeepsSemanticMain",
                         "HV3 right-angle main keeps semantic main flow while local corner feasibility drives acute lowering",
                         "Invariant", false, test_backbone_hv3_corner_feasibility_lowering_keeps_semantic_main);
  test_registry::AddTest(tests, "C220_Backbone_AcuteMergeFeasibilityAppliesAcrossRouteBoundary",
                         "Acute merge keeps a corner-based lowering root even when the corner arrives from a separate route boundary",
                         "Invariant", false, test_backbone_acute_merge_feasibility_applies_across_route_boundary);
  test_registry::AddTest(tests, "C221_Backbone_RecalcKeepsSameLevelLoweringOrigin",
                         "Recalc keeps same-level infeasibility and lowering origin visible on support layout and span inspection",
                         "Invariant", false, test_backbone_recalc_keeps_same_level_lowering_origin);
  test_registry::AddTest(tests, "C222_Backbone_HV3CornerUsesConstrainedBandSolver",
                         "HV3 corner uses constrained placement-band ports instead of special-case ports when same-level feasibility fails",
                         "Invariant", false, test_backbone_hv3_corner_uses_constrained_band_solver);
  test_registry::AddTest(tests, "C223_Backbone_CrossConstraintUsesSolver",
                         "Cross underpass can keep through classification while solving near-node clearance through constrained band placement",
                         "Invariant", false, test_backbone_cross_same_level_infeasible_can_use_constrained_solver);
  test_registry::AddTest(tests, "C224_Backbone_PolicyBlockedConflictSurvivesRecalc",
                         "Policy-blocked same-level conflicts remain visible as unresolved after recalc instead of silently disappearing",
                         "Invariant", false, test_backbone_policy_blocked_unresolved_survives_recalc_inspection);
  test_registry::AddTest(tests, "C225_Backbone_RefreshKeepsPlacementConstraintOrigin",
                         "Refresh keeps placement-constraint origin and constrained band source instead of snapping lowered corner ports back to normal bands",
                         "Invariant", false, test_backbone_refresh_keeps_placement_constraint_origin);
  test_registry::AddTest(tests, "C226_Backbone_MirrorDoesNotChangeConstrainedSolverUsage",
                         "Mirror continues to affect lane-order only and does not change constrained-solver usage or unresolved same-level state",
                         "Invariant", false, test_backbone_mirror_does_not_change_constrained_solver_usage);
  test_registry::AddTest(tests, "C227_Backbone_CrossRelationSurvivesSupportLayoutRecalc",
                         "Cross underpass relation and lowering survive support-layout recalc instead of collapsing entirely into Main/Branch",
                         "Invariant", false, test_backbone_cross_relation_survives_support_layout_recalc);
  test_registry::AddTest(tests, "C228_Backbone_HV3BranchDefaultLower",
                         "Bundle-like HV3 side branch defaults to lower even when same-level clearance would otherwise look acceptable",
                         "Invariant", false, test_backbone_hv3_branch_default_lower_required);
  test_registry::AddTest(tests, "C229_Backbone_HV3CornerDefaultLower",
                         "Bundle-like HV3 corner continuation defaults to lower instead of staying on the main row",
                         "Invariant", false, test_backbone_hv3_corner_continuation_default_lower_required);
  test_registry::AddTest(tests, "C230_Backbone_HV3CrossOnlyThroughPairSameLevel",
                         "Bundle-like HV3 cross keeps same-level continuity only for the accepted ThroughPair",
                         "Invariant", false, test_backbone_hv3_cross_only_through_pair_stays_same_level_candidate);
  test_registry::AddTest(tests, "C231_Backbone_PointLikeBranchCanStaySameLevel",
                         "Point-like branch remains feasibility-driven and can stay same-level when clearance allows",
                         "Invariant", false, test_backbone_point_like_branch_can_keep_same_level_when_clear);
  test_registry::AddTest(tests, "C232_Backbone_BundleRulePolicyBlockedUnresolved",
                         "Bundle-like default lower remains visible as unresolved when category policy blocks lowering",
                         "Invariant", false, test_backbone_bundle_rule_policy_block_stays_unresolved);
  test_registry::AddTest(tests, "C233_Backbone_RefreshKeepsBundleRuleOrigin",
                         "Refresh keeps bundle-rule lowering origin instead of dropping back to generic same-level placement",
                         "Invariant", false, test_backbone_refresh_keeps_bundle_rule_origin);
  test_registry::AddTest(tests, "C234_Backbone_CrossLoweredPairSymmetricSides",
                         "Cross lowered pair uses one junction-pair side group instead of splitting into left/right supports per endpoint",
                         "Invariant", false, test_backbone_cross_lowered_pair_uses_opposite_junction_pair_sides);
  test_registry::AddTest(tests, "C235_Backbone_ConstrainedLoweredSupportOrientation",
                         "Constrained-placement lowered support uses a line/chord-oriented visual rule instead of falling back to pole-radial orientation",
                         "Invariant", false, test_backbone_constrained_lowered_support_prefers_line_direction);
  test_registry::AddTest(tests, "C236_Backbone_BundleBranchOrientationUsesBisectorWhenAvailable",
                         "Bundle-like lowered branch root uses bisector orientation when available instead of falling back to branch chord",
                         "Invariant", false, test_backbone_bundle_branch_support_orientation_uses_bisector_when_available);
  test_registry::AddTest(tests, "C237_Backbone_PointLikeOrientationNonRegression",
                         "Point-like branch uses pair-aware same-level support orientation without creating lowered support groups",
                         "Invariant", false, test_backbone_point_like_orientation_rule_non_regression);
  test_registry::AddTest(tests, "C267_Backbone_NonLoweredCrossSpansDoNotExposeLoweredSupportGroups",
                         "Cross junction main spans with lower_required=false do not expose grouped lowered support",
                         "Invariant", false, test_backbone_non_lowered_cross_spans_do_not_expose_lowered_support_groups);
  test_registry::AddTest(tests, "C268_Backbone_NonLoweredSpansDoNotInheritAcuteCornerSupportGroups",
                         "Bundle spans with lower_required=false do not inherit grouped lowered support from lowered segments",
                         "Invariant", false, test_backbone_non_lowered_spans_do_not_inherit_acute_corner_support_groups);
  test_registry::AddTest(tests, "C238_Backbone_RefreshKeepsSideOrientationOrigin",
                         "Refresh keeps lowered side/orientation origin visible instead of collapsing constrained or cross-lowered spans back to generic radial support rules",
                         "Invariant", false, test_backbone_refresh_keeps_lowered_side_and_orientation_origin);
  test_registry::AddTest(tests, "C239_Backbone_HV3SameLevelOrderDecisionPermutable",
                         "HV3 ThroughPair path keeps an authoritative non-fixed order decision instead of reverting to fixed-order",
                         "Invariant", false, test_backbone_hv3_same_level_order_decision_is_permutable);
  test_registry::AddTest(tests, "C240_Backbone_HV3LoweredOrderDecisionPermutable",
                         "Lowered HV3 cross keeps an authoritative non-fixed order decision at the lowered endpoint",
                         "Invariant", false, test_backbone_hv3_lowered_order_decision_is_permutable);
  test_registry::AddTest(tests, "C241_Backbone_FixedOrderBundleUntouched",
                         "Fixed-order bundle skips non-fixed order evaluation entirely",
                         "Invariant", false, test_backbone_fixed_order_bundle_skips_permutation);
  test_registry::AddTest(tests, "C242_Backbone_RefreshKeepsOrderDecision",
                         "Refresh keeps the chosen HV3 order decision instead of re-flipping after constrained lowered generation",
                         "Invariant", false, test_backbone_refresh_keeps_order_decision_choice);
  test_registry::AddTest(tests, "C243_Backbone_PointLikeOrderDecisionNonRegression",
                         "Point-like low-voltage route stays fixed-order and does not opt into HV3 order permutation",
                         "Invariant", false, test_backbone_point_like_order_decision_non_regression);
  test_registry::AddTest(tests, "C244_Backbone_AuthoritativeDecisionMatchesSupportLayout",
                         "Support layout copies grouped endpoint decisions without reinterpreting chosen order, side, or orientation basis",
                         "Invariant", false, test_backbone_authoritative_endpoint_decision_matches_support_layout);
  test_registry::AddTest(tests, "C245_Backbone_RefreshDoesNotOverrideAuthoritativeDecision",
                         "Refresh keeps the chosen order decision, side, and orientation basis instead of re-flipping them downstream",
                         "Invariant", false, test_backbone_refresh_does_not_override_authoritative_endpoint_decision);
  test_registry::AddTest(tests, "C246_Backbone_AuthoritativeCrossPairSideSymmetry",
                         "Cross lowered pair keeps one authoritative shared side choice instead of re-splitting into endpoint-local left/right supports",
                         "Invariant", false, test_backbone_authoritative_cross_pair_side_symmetry);
  test_registry::AddTest(tests, "C247_Backbone_ConstrainedOrientationUsesAuthoritativeBasis",
                         "Constrained placement support visuals reuse the authoritative orientation basis instead of recomputing a radial rule",
                         "Invariant", false, test_backbone_constrained_orientation_uses_authoritative_basis);
  test_registry::AddTest(tests, "C248_Backbone_HV3AuthoritativeOrderDecisionSurvivesRefresh",
                         "HV3 chosen order decision survives refresh as an authoritative result without downstream override",
                         "Invariant", false, test_backbone_hv3_authoritative_order_decision_survives_refresh);
  test_registry::AddTest(tests, "C249_Backbone_EdgeOrientationUsesChosenOrderDecision",
                         "Edge orientation is derived from the chosen endpoint order decision instead of legacy mirror state",
                         "Invariant", false, test_backbone_edge_orientation_uses_chosen_order_decision);
  test_registry::AddTest(tests, "C250_Backbone_BundleBranchLoweringIsPoleLocal",
                         "Bundle-like branch lowering is decided from each pole endpoint locally instead of relying on run propagation",
                         "Invariant", false, test_backbone_bundle_branch_lowering_stays_pole_local);
  test_registry::AddTest(tests, "C251_Backbone_CrossUnderpassSupportUsesSharedSideGroup",
                         "Cross underpass lowered pair reuses one side group and one support anchor instead of splitting left/right per endpoint",
                         "Invariant", false, test_backbone_cross_underpass_supports_share_one_side_group);
  test_registry::AddTest(tests, "C252_Backbone_RefreshKeepsLocalLowerAndGroupedSupport",
                         "Refresh keeps pole-local lowering decisions and grouped lowered-support identity instead of recomputing them per-port",
                         "Invariant", false, test_backbone_refresh_keeps_local_lower_and_grouped_support);
  test_registry::AddTest(tests, "C255_Backbone_GroupedSupportVisibleOnAllBundleLanes",
                         "Grouped lowered support remains visible on every participating HV3 lane instead of only a representative span",
                         "Invariant", false, test_backbone_grouped_support_membership_is_visible_on_all_bundle_lanes);
  test_registry::AddTest(tests, "C256_Backbone_UnrelatedGenerationDoesNotDowngradeLoweredBundle",
                         "Unrelated later generation does not cause existing lowered HV3 spans to fall back to point-like/radial semantics during refresh",
                         "Invariant", false, test_backbone_unrelated_generation_does_not_downgrade_existing_lowered_bundle_semantics);
  test_registry::AddTest(tests, "C253_Backbone_CornerSupportUsesConnectedLineBasis",
                         "Bundle-like lowered corner uses only connected lowered lines to choose a non-radial support basis and shared support identity",
                         "Invariant", false, test_backbone_corner_support_uses_connected_line_basis);
  test_registry::AddTest(tests, "C275_Backbone_LoweredBundleMidspanSupportUsesPairBasedOrientation",
                         "Bundle-like lowered midspan support uses the route-pair bisector instead of falling back to pole-local radial orientation",
                         "Invariant", false, test_backbone_lowered_bundle_midspan_support_uses_pair_based_orientation);
  test_registry::AddTest(tests, "C276_Backbone_PairAxisSharedButSignFlipsPerPole",
                         "Pair-based lowered support keeps one shared axis/basis while opposite poles may take opposite side signs",
                         "Invariant", false, test_backbone_pair_based_orientation_allows_opposite_signs_per_pole);
  test_registry::AddTest(tests, "C136_Backbone_HV3MainPortsStableAfterBranch",
                         "Adding an HV3 branch keeps existing main-chain ports stable", "Invariant", false,
                         test_backbone_branch_generation_preserves_existing_hv3_main_ports);
  test_registry::AddTest(tests, "C185_Backbone_HV3TerminalSlotsDistinct",
                         "HV3 DrawPath terminals realize distinct template bands instead of fallback category ports",
                         "Invariant", false, test_backbone_hv3_terminal_poles_use_distinct_template_bands);
  test_registry::AddTest(tests, "C188_Backbone_HV3TerminalFallbackRowPerpendicular",
                         "HV3 DrawPath terminals without HV bands still realize a perpendicular generated row without generic fallback",
                         "Invariant", false, test_backbone_hv3_terminal_fallback_ports_still_spread_perpendicular);
  test_registry::AddTest(tests, "C189_Backbone_HV3CommunicationPoleAllTemplatesTerminalRow",
                         "HV3 remains visually separated and perpendicular on communication poles even when all DrawPath templates are selected",
                         "Invariant", false,
                         test_backbone_hv3_terminals_stay_perpendicular_on_communication_pole_with_all_templates);
  test_registry::AddTest(tests, "C190_Backbone_CommunicationTerminalRowPerpendicular",
                         "Communication multi-lane terminals stay visually separated and perpendicular on communication poles",
                         "Invariant", false, test_backbone_communication_multilane_terminals_stay_perpendicular);
  test_registry::AddTest(tests, "C192_Backbone_ClickedExistingCommunicationPoles_AllTemplatesStaySeparated",
                         "Clicked existing communication poles keep HV terminals visually separated and perpendicular when all DrawPath templates are selected",
                         "Invariant", false,
                         test_backbone_clicked_existing_communication_poles_all_templates_keep_hv_terminal_separation);
  test_registry::AddTest(tests, "C298_Backbone_CaptureGenerationUsesRequestedCommunicationPoleType",
                         "Captured all-template generation keeps generated pole instances on the requested communication pole type and height",
                         "Invariant", false, test_backbone_capture_generation_uses_requested_communication_pole_type);
  test_registry::AddTest(tests, "C299_Backbone_HVGenerationUsesHVCategoryHeightOnCommunicationPole",
                         "CommunicationPole HV generation uses HV category base height instead of another layer-mate category",
                         "Invariant", false, test_backbone_hv_generation_uses_hv_category_height_on_communication_pole);
  test_registry::AddTest(tests, "C300_BundleTemplate_OrderPolicyDrivesGeneration",
                         "BundleTemplate order policy is carried into grouped generation instead of being re-decided from category-specific code",
                         "Invariant", false, test_bundle_template_order_policy_drives_generation);
  test_registry::AddTest(tests, "C301_Backbone_DropTemplateUsesDedicatedBundleAndServiceLayer",
                         "Drop backbone generation uses dedicated drop bundle/layer and service span kind",
                         "Invariant", false, test_backbone_drop_template_uses_dedicated_bundle_and_service_layer);
  test_registry::AddTest(tests, "C138_Backbone_MixedRouteEdgeFlow",
                         "Mixed route classifies main and branch per edge instead of one route-level flow",
                         "Invariant", false, test_backbone_mixed_route_uses_edge_level_flow_classification);
  test_registry::AddTest(tests, "C139_Backbone_BranchSupportVisualPlacement",
                         "Branch support placement is derived into visual cache for minimal support structure",
                         "Invariant", false, test_backbone_branch_support_visual_cache_contains_support_placement);
  test_registry::AddTest(tests, "C140_Backbone_BranchClassificationIgnoresNearStraightAngle",
                         "Existing-chain branch stays branch even when geometry is nearly straight", "Invariant",
                         false, test_backbone_near_straight_branch_still_classifies_as_branch);
  test_registry::AddTest(tests, "C269_Backbone_CrossLikeSingleEdgeStaysMain",
                         "Single-edge DrawPath from a cross-like existing node stays Main instead of inheriting branch/cross lowering from old neighbors",
                         "Invariant", false, test_backbone_crosslike_single_edge_stays_main);
  test_registry::AddTest(tests, "C270_Backbone_ExplicitMiddleBentRouteStaysMainLike",
                         "Bent route through an explicit middle anchor stays main-like against an existing straight chain instead of falling to branch/cross classification",
                         "Invariant", false,
                         test_backbone_explicit_middle_bent_route_stays_corner_main_against_existing_chain);
  test_registry::AddTest(tests, "C271_Backbone_GroupedSupportVisualUsesSinglePlacement",
                         "Grouped lowered support visual cache uses one group placement but only span-local grouped attachments",
                         "Invariant", false, test_backbone_grouped_support_visual_cache_uses_single_group_placement);
  test_registry::AddTest(tests, "C272_Backbone_BranchLowerRequiredHeightSurvivesPlacementAndRefresh",
                         "Bundle-like non-through lower_required propagates to grouped placement height and stays unchanged after refresh/recalc",
                         "Invariant", false, test_backbone_branch_lower_required_height_survives_to_grouped_placement);
  test_registry::AddTest(tests, "C273_Backbone_NonThroughHeightCollapsesToOneLowerStep",
                         "Bundle-like non-through uses one shared lower step while ThroughMain stays at template height",
                         "Invariant", false, test_backbone_bundle_non_through_height_collapses_to_two_states);
  test_registry::AddTest(tests, "C274_Backbone_CrossLikeReuseKeepsExistingStraightMainPair",
                         "Cross-like reuse keeps the existing straight main pair instead of promoting a new crossing path to ThroughMain",
                         "Invariant", false, test_backbone_crosslike_reuse_keeps_existing_straight_main_pair);
  test_registry::AddTest(tests, "C150_Backbone_NewChainOrientationFallback",
                         "Poles without existing chain or primary context use the explicit fallback orientation rule",
                         "Invariant", false, test_backbone_new_chain_uses_fallback_orientation_without_existing_main_context);
  test_registry::AddTest(tests, "C173_Backbone_StraightSupportAxisPerpendicularToRoute",
                         "Straight DrawPath keeps pole support axis and port row perpendicular to the route axis",
                         "Invariant", false, test_backbone_straight_chain_support_axis_stays_perpendicular_to_route);
  test_registry::AddTest(tests, "C174_Backbone_CrossSupportAxisNotDiagonal",
                         "Cross junction keeps support axis on the preserved pair/trunk family instead of collapsing to a diagonal pole-global fit",
                         "Invariant", false, test_backbone_cross_junction_support_axis_avoids_diagonal);
  test_registry::AddTest(tests, "C302_Backbone_PoleLayoutYawStaysOnPoleYaw",
                         "HV support-axis solving does not change the pole layout yaw reported for the pole itself",
                         "Invariant", false, test_backbone_cross_junction_layout_yaw_stays_on_pole_yaw);
  test_registry::AddTest(tests, "C303_BundleTemplate_RowLayoutAxisModeDrivesPortLayoutYaw",
                         "BundleTemplate row layout axis mode is resolved into pole row-layout decisions instead of category hardcode",
                         "Invariant", false, test_bundle_template_row_layout_axis_mode_drives_port_layout_yaw);
  test_registry::AddTest(tests, "C175_Backbone_OrthogonalSupportAxisStaysCardinal",
                         "Orthogonal DrawPath corner fits support axis to connected directions instead of freezing to a cardinal axis",
                         "Invariant", false, test_backbone_orthogonal_route_support_axis_stays_cardinal);
  test_registry::AddTest(tests, "C176_Backbone_BranchKeepsMainSupportAxis",
                         "Adding a branch turns support heading toward connected directions while keeping the row geometry readable",
                         "Invariant", false, test_backbone_branch_keeps_main_support_axis_non_diagonal);
  test_registry::AddTest(tests, "C326_Backbone_SameLevelCrossMainUsesThroughPairAuthority",
                         "Same-level cross topology keeps one accepted through pair instead of rebinding the node around later route additions",
                         "Invariant", false, test_backbone_same_level_cross_main_uses_through_pair_authority);
  test_registry::AddTest(tests, "C340_Backbone_SameLevelCrossUnderpassUsesCrossPairAuthority",
                         "Same-level cross underpass endpoints use the cross-pair axis instead of endpoint-local bisector or main-pair fallback",
                         "Invariant", false, test_backbone_same_level_cross_underpass_uses_cross_pair_authority);
  test_registry::AddTest(tests, "C341_Backbone_CrossVisualKeepsDistinctPairAxes",
                         "One-shot cross visual keeps through-pair and cross-pair support arms on distinct axes instead of collapsing them together",
                         "Invariant", false, test_backbone_cross_visual_keeps_distinct_pair_axes);
  test_registry::AddTest(tests, "C342_Backbone_CaptureLikeCrossCenterUsesPairAuthority",
                         "Latest capture-like cross center keeps both route-pair and existing-pair endpoints on pair-based authority instead of falling back to pole-local radial",
                         "Invariant", false, test_backbone_capture_like_cross_center_uses_pair_authority);
  test_registry::AddTest(tests, "C343_Backbone_CaptureLikeCrossVisualKeepsRouteAndExistingPairAxesDistinct",
                         "Latest capture-like cross visual keeps route-pair and existing-pair support arms on distinct axes instead of collapsing them together",
                         "Invariant", false, test_backbone_capture_like_cross_visual_keeps_route_and_existing_pair_axes_distinct);
  test_registry::AddTest(tests, "C344_Backbone_LatestReplaySupportOrientationUsesPairAuthority",
                         "Latest replayed support-orientation capture keeps same-level cross/T endpoints on pair authority instead of Bisector or pole-local radial fallback",
                         "Invariant", false, test_backbone_latest_support_orientation_replay_uses_pair_authority);
  test_registry::AddTest(tests, "C345_Backbone_LatestReplaySupportVisualFollowsPairAxis",
                         "Latest replayed support-orientation capture keeps same-level support arms aligned with pair-based endpoint authority instead of collapsing toward one pole-local direction",
                         "Invariant", false, test_backbone_latest_support_orientation_replay_visual_follows_pair_axis);
  test_registry::AddTest(tests, "C327_Backbone_NonGroupedSupportVisualUsesEndpointAuthority",
                         "Same-level non-grouped support visual follows endpoint support authority instead of pole-center radial fallback",
                         "Invariant", false, test_backbone_non_grouped_support_visual_uses_endpoint_authority);
  test_registry::AddTest(tests, "C335_Backbone_SameLevelTerminalEndpointDoesNotInventSideSign",
                         "Same-level terminal endpoint without pair authority keeps zero side sign instead of inventing a left/right choice from local port offsets",
                         "Invariant", false, test_backbone_same_level_terminal_endpoint_does_not_invent_side_sign);
  test_registry::AddTest(tests, "C336_Backbone_CaptureLikeBranchTerminalBorrowsPeerPairAuthority",
                         "Latest capture-like CommunicationPole branch terminal borrows the peer through-pair authority instead of falling back to pole-local radial",
                         "Invariant", false, test_backbone_capture_like_branch_terminal_borrows_peer_pair_authority);
  test_registry::AddTest(tests, "C337_Backbone_CaptureLikeBranchTerminalVisualUsesBorrowedPairAxis",
                         "Latest capture-like CommunicationPole branch terminal visual arm follows the borrowed peer pair axis instead of a radial fallback",
                         "Invariant", false, test_backbone_capture_like_branch_terminal_visual_uses_borrowed_pair_axis);
  test_registry::AddTest(tests, "C338_Backbone_CaptureLikeLoweredBranchMainBorrowsPeerPairAuthority",
                         "Latest capture-like lowered bundle branch main endpoint borrows the peer through-pair authority instead of falling back to pole-local radial",
                         "Invariant", false, test_backbone_capture_like_lowered_branch_main_endpoint_borrows_peer_pair_authority);
  test_registry::AddTest(tests, "C339_Backbone_CaptureLikeLoweredBranchMainVisualUsesBorrowedPairAxis",
                         "Latest capture-like lowered bundle branch visual arm follows the actual endpoint port radial direction instead of collapsing to one shared side-axis",
                         "Invariant", false, test_backbone_capture_like_lowered_branch_main_visual_uses_borrowed_pair_axis);
  test_registry::AddTest(
      tests, "C323_Backbone_SingleEdgeReusePreservesExistingStraightSupportAxis",
      "Single-edge reuse on a captured CommunicationPole trunk keeps the existing straight support axis instead of "
      "switching to the newer branch-side pair",
      "Invariant", false, test_backbone_single_edge_reuse_preserves_existing_straight_support_axis);
  test_registry::AddTest(
      tests, "C324_Backbone_CaptureLikeIncrementalCrossKeepsTrunkSupportAxis",
      "Capture-like incremental cross keeps the original trunk support axis instead of rotating the main support toward the later cross pair",
      "Invariant", false, test_backbone_capture_like_incremental_cross_keeps_trunk_support_axis);
  test_registry::AddTest(
      tests, "C325_Backbone_CaptureLikeIncrementalCrossPreservesTrunkPairAuthority",
      "Capture-like incremental cross keeps the pole support-axis authority on the original trunk pair instead of rebinding to the later cross pair",
      "Invariant", false, test_backbone_capture_like_incremental_cross_preserves_trunk_pair_authority);
  test_registry::AddTest(
      tests, "C360_Backbone_ConnectedDirectionFitGateRespectsTrunkPreserve",
      "ConnectedDirectionFit stays off when a preserved trunk pair is available, and the chosen neighbors stay on that trunk pair",
      "Invariant", false, test_backbone_connected_direction_fit_gate_respects_trunk_preserve);
  test_registry::AddTest(
      tests, "C361_Backbone_ConnectedDirectionFitGateRespectsContinuationPair",
      "ConnectedDirectionFit stays off when a continuation pair is available on a cross-like incremental node, and the chosen neighbors stay on the trunk continuation pair",
      "Invariant", false, test_backbone_connected_direction_fit_gate_respects_continuation_pair);
  test_registry::AddTest(
      tests, "C362_Backbone_ConnectedDirectionFitGateRespectsExplicitPairAxis",
      "ConnectedDirectionFit stays off when an explicit two-neighbor pair axis is available, and the chosen neighbors stay on that pair",
      "Invariant", false, test_backbone_connected_direction_fit_gate_respects_explicit_pair_axis);
  test_registry::AddTest(
      tests, "C365_BackbonePipeline_BuildDirectionFromInputDirection",
      "BackboneBuilder initializes build direction from the input direction and keeps reversed click order",
      "Invariant", false, test_backbone_pipeline_build_direction_initializes_from_input_direction);
  test_registry::AddTest(
      tests, "C366_Backbone_NoBorrowedPairAtTerminal",
      "Terminal endpoint without an explicit pair does not borrow peer pair authority downstream",
      "Invariant", false, test_backbone_terminal_without_explicit_pair_does_not_borrow_pair);
  test_registry::AddTest(
      tests, "C367_Backbone_MaterializationDoesNotChangePairFamily",
      "Materialization consumes saved pair family without changing support pair peers",
      "Invariant", false, test_backbone_materialization_does_not_change_pair_family);
  test_registry::AddTest(
      tests, "C363_Backbone_LatestCaptureTSupportFamilyDoesNotMixPairRules",
      "Latest captured T-support family keeps one pair-owned rule family instead of mixing ThroughPairNormal and Bisector inside the same pair",
      "Symptom", false, test_backbone_latest_capture_t_support_family_does_not_mix_pair_rules);
  test_registry::AddTest(
      tests, "C364_Backbone_LatestCapturePoleOrientationDoesNotDisagreeWithSupportSelection",
      "Latest captured pole does not keep PoleOrientation on Fallback when SupportLayoutSelection already resolved MainChainPair",
      "Authority", false, test_backbone_latest_capture_pole_orientation_does_not_disagree_with_support_selection);
  test_registry::AddTest(
      tests, "C346_Backbone_CaptureLikeOuterSameLevelCrossUsesPairAuthority",
      "Capture-like outer same-level cross keeps pair authority on newly generated side-branch endpoints without retroactively rewriting existing spans",
      "Invariant", false, test_backbone_capture_like_outer_same_level_cross_uses_pair_authority);
  test_registry::AddTest(
      tests, "C347_Backbone_CaptureLikeOuterSameLevelCrossVisualUsesPairAxis",
      "Capture-like outer same-level cross keeps newly generated side-branch support arms on the pair axis without retroactive existing-span refresh",
      "Invariant", false, test_backbone_capture_like_outer_same_level_cross_visual_uses_pair_axis);
  test_registry::AddTest(
      tests, "C348_Backbone_SupportLayoutInspectionExposesResolvedSupportAuthority",
      "Support layout inspection exposes the generated decision seed and resolved support authority for pair-aware endpoints",
      "Invariant", false, test_backbone_support_layout_inspection_exposes_resolved_support_authority);
  test_registry::AddTest(
      tests, "C349_Backbone_GeometryRefreshKeepsResolvedSupportAuthoritySeed",
      "Geometry refresh rebuilds support layout and curve from the generated seed without changing resolved support authority",
      "Invariant", false, test_backbone_geometry_refresh_keeps_resolved_support_authority_seed);
  test_registry::AddTest(
      tests, "C350_Backbone_RenderRefreshDoesNotChangeResolvedSupportAuthoritySeed",
      "Render refresh updates visual cache only and leaves resolved support authority unchanged",
      "Invariant", false, test_backbone_render_refresh_does_not_change_resolved_support_authority_seed);
  test_registry::AddTest(
      tests, "C351_Backbone_RebuildWithoutSeedFailsAndKeepsCachedLayout",
      "Explicit rebuild without a decision seed fails and keeps the cached support layout unchanged",
      "Invariant", false, test_backbone_rebuild_without_seed_fails_and_keeps_cached_layout);
  test_registry::AddTest(
      tests, "C352_Backbone_GeometryRefreshWithoutSeedFailsAndKeepsDirty",
      "Geometry refresh without a decision seed fails, keeps geometry dirty, and reports the missing seed",
      "Invariant", false, test_backbone_geometry_refresh_without_seed_fails_and_keeps_dirty);
  test_registry::AddTest(
      tests, "C358_Backbone_MaterializationResolvesEndpointSocketAndCurveConsumesIt",
      "Support layout materialization resolves endpoint sockets once and detail-curve generation consumes that resolved socket state",
      "Invariant", false, test_backbone_materialization_resolves_endpoint_socket_and_curve_consumes_it);
  test_registry::AddTest(
      tests, "C359_Backbone_GeometryRefreshKeepsMaterializedEndpointSocketResolution",
      "Geometry refresh keeps the materialized endpoint socket decision unchanged",
      "Invariant", false, test_backbone_geometry_refresh_keeps_materialized_endpoint_socket_resolution);
  test_registry::AddTest(
      tests, "C353_Backbone_SameLevelTPairHeightRankDrivesSupportOffsets",
      "Same-level T keeps the main pair at rank 0 and lowers the branch pair by one owner-controlled step",
      "Invariant", false, test_backbone_same_level_t_pair_height_rank_drives_support_offsets);
  test_registry::AddTest(
      tests, "C354_Backbone_OneShotCrossPairHeightRankSeparatesPairs",
      "One-shot cross keeps the topology through-pair accepted and lowers the generated cross pair by one owner-controlled step",
      "Invariant", false, test_backbone_one_shot_cross_pair_height_rank_separates_pairs);
  test_registry::AddTest(tests, "C177_Backbone_DrawPathPlainEndpointFallback",
                         "DrawPath backbone without attachment input falls back to plain support endpoints",
                         "Invariant", false, test_backbone_drawpath_plain_endpoint_fallback_without_attachment_input);
  test_registry::AddTest(tests, "C179_Backbone_DrawPathBranchCurveStaysLocal",
                         "DrawPath branch curve keeps support departure local and converges back toward chord",
                         "Invariant", false, test_backbone_drawpath_branch_curve_stays_local_to_support_departure);
  test_registry::AddTest(tests, "C180_Backbone_DrawPathMainBranchTraceReadable",
                         "DrawPath span inspection and trace make main/branch differences explicit",
                         "Invariant", false, test_backbone_drawpath_main_and_branch_are_distinct_in_trace_and_inspection);
  test_registry::AddTest(tests, "C155_Variation_DoesNotAffectTopologyOrMirror",
                         "Variation settings do not change deterministic flow classification or mirror decisions",
                         "Invariant", false, test_variation_settings_do_not_change_topology_flow_or_mirror);
  test_registry::AddTest(tests, "C279_Backbone_AdjacentBranchRootsUseRouteLocalBisector",
                         "Adjacent bundle branch roots use route-local bisector orientation instead of collapsing to one shared direction",
                         "Invariant", false, test_backbone_adjacent_branch_roots_use_route_local_bisector);
  test_registry::AddTest(tests, "C285_Backbone_MainBisectorSupportAxisStaysPerpendicularOnAcuteBranchRoot",
                         "Acute main-chain branch root keeps pole support axis perpendicular to the main bisector instead of nearly parallel to it",
                         "Invariant", false, test_backbone_main_bisector_support_axis_stays_perpendicular_on_acute_branch_root);
  test_registry::AddTest(tests, "C282_Backbone_SamePoleBranchRootsKeepDistinctSupportGroups",
                         "Two bundle branch roots on the same pole keep distinct lowered support groups instead of collapsing to one shared direction",
                         "Invariant", false, test_backbone_same_pole_branch_roots_keep_distinct_support_groups);
}

WIRE_REGISTER_TEST_SUITE(register_generation_tests);

} // namespace
