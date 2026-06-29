#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <map>
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
#include "../src/generation/support_policy.hpp"

using namespace helpers;

std::optional<wire::core::SegmentLaneAssignment> find_assignment_for_span(const wire::core::CoreState& state,
                                                                          wire::core::ObjectId span_id);
std::optional<wire::core::VisualPart> support_arm_part_for_owner(const wire::core::CoreState& state,
                                                                 wire::core::ObjectId span_id,
                                                                 wire::core::ObjectId owner_pole_id,
                                                                 double z_tolerance = 1e-6);
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
  observation.nodes = state.SavedBackboneResult().nodes;
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
  const auto backbone = state.SavedBackboneResult();
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
  const auto route = state.FindSavedBackboneRoute(anchor_id, new_end_id);
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
  const auto first_backbone = state.SavedBackboneResult();
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
  const auto second_backbone = state.SavedBackboneResult();
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
  const auto first_backbone = state.SavedBackboneResult();
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
  const auto first_backbone = state.SavedBackboneResult();
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
  const std::vector<ObjectId> route = state.FindSavedBackboneRoute(existing_midair_id, terminal_pole_id);
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
  const auto first_backbone = state.SavedBackboneResult();
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
  const std::vector<ObjectId> route = state.FindSavedBackboneRoute(existing_midair->node_id, terminal_pole_id);
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
// Intent: Latest captured CommunicationPole HV path should keep lane polylines non-crossing across Commit(recalc).
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
    const auto* support_layout = state.span_layout(span.id).entry;
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
    const auto* support_layout = state.span_layout(span.id).entry;
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
  test_registry::AddTest(tests, "C135_Backbone_BranchSupportDownOffset",
                         "Branch support lowers attachment height without rewriting layer semantics", "Invariant",
                         false, test_backbone_branch_support_offsets_height_without_changing_layer);
  test_registry::AddTest(tests, "C204_Backbone_HV3CornerUsesOneSharedLowerStep",
                         "HV3 corner continuation uses the same one-step lowered height across the whole bundle without turning it into branch support",
                         "Invariant", false, test_backbone_hv3_acute_corner_lowers_corner_bundle_without_branch_support);
  test_registry::AddTest(tests, "C209_Backbone_HV3ModerateCornerUsesOneSharedLowerStep",
                         "HV3 moderate corner still collapses to one shared lower step under the default corner threshold",
                         "Invariant", false, test_backbone_hv3_moderate_acute_corner_lowers_bundle_at_default_threshold);
  test_registry::AddTest(tests, "C201_Backbone_PointLikeCrossCanStaySameLevel",
                         "Point-like cross keeps branch classification but may stay same-level when near-node clearance allows",
                         "Invariant", false, test_backbone_cross_junction_nonmain_line_uses_underpass);
  test_registry::AddTest(tests, "C211_Backbone_AllTemplatesCrossStillUsesUnderpassOnCommunicationPole",
                         "All-template DrawPath cross on communication poles still keeps at least one non-main line lowered",
                         "Invariant", false,
                         test_backbone_all_templates_cross_keeps_underpass_on_communication_pole);
  test_registry::AddTest(tests, "C212_Backbone_CaptureBranchAndCornerShareOneStepLowerOnCommunicationPole",
                         "Captured all-template communication-pole path keeps the HV branch root and later corner segments on the same one-step lowered height",
                         "Invariant", false, test_backbone_capture_branch_then_acute_lowering_on_communication_pole);
  test_registry::AddTest(tests, "C136_Backbone_HV3MainPortsStableAfterBranch",
                         "Adding an HV3 branch keeps existing main-chain ports stable", "Invariant", false,
                         test_backbone_branch_generation_preserves_existing_hv3_main_ports);
  test_registry::AddTest(tests, "C185_Backbone_HV3TerminalSlotsDistinct",
                         "HV3 DrawPath terminals realize distinct template bands instead of fallback category ports",
                         "Invariant", false, test_backbone_hv3_terminal_poles_use_distinct_template_bands);
  test_registry::AddTest(tests, "C189_Backbone_HV3CommunicationPoleAllTemplatesTerminalRow",
                         "HV3 remains visually separated and perpendicular on communication poles even when all DrawPath templates are selected",
                         "Invariant", false,
                         test_backbone_hv3_terminals_stay_perpendicular_on_communication_pole_with_all_templates);
  test_registry::AddTest(tests, "C190_Backbone_CommunicationTerminalRowPerpendicular",
                         "Communication multi-lane terminals stay visually separated and perpendicular on communication poles",
                         "Invariant", false, test_backbone_communication_multilane_terminals_stay_perpendicular);
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
  test_registry::AddTest(tests, "C140_Backbone_BranchClassificationIgnoresNearStraightAngle",
                         "Existing-chain branch stays branch even when geometry is nearly straight", "Invariant",
                         false, test_backbone_near_straight_branch_still_classifies_as_branch);
  test_registry::AddTest(tests, "C155_Variation_DoesNotAffectTopologyOrMirror",
                         "Variation settings do not change deterministic flow classification or mirror decisions",
                         "Invariant", false, test_variation_settings_do_not_change_topology_flow_or_mirror);
}

WIRE_REGISTER_TEST_SUITE(register_generation_tests);

} // namespace
