#include "wire/core/core_state.hpp"
#include "../pole_orientation_utils.hpp"
#include "../support_orientation_utils.hpp"
#include "detail_utils.hpp"
#include "grouped_span_common.hpp"
#include "grouped_span_lane_preparation.hpp"
#include "grouped_span_lowering.hpp"
#include "grouped_span_orientation.hpp"
#include "support_policy.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace wire::core {

using namespace generation::detail;

namespace {} // namespace

EditResult<std::vector<ObjectId>>
CoreState::generate_grouped_spans_between_support_nodes(
    const std::vector<ObjectId>& node_ids, const std::unordered_map<ObjectId, SupportNode>& support_node_by_id,
    ObjectId bundle_id, ConnectionCategory category, int conductor_count, double spacing_m, bool maintain_lane_order,
    bool allow_lane_mirror, OrderDecisionPolicyKind order_decision_policy, BackboneFlowKind flow_kind,
    const BackboneLoweringPolicy& lowering_policy,
    const std::unordered_map<ObjectId, JunctionRelation>* junction_relations_by_node,
    std::vector<SegmentLaneAssignment>* out_lane_assignments,
    std::vector<BackboneEdgeOrientation>* out_edge_orientations, BundleKind bundle_template_id) {
  EditResult<std::vector<ObjectId>> result;
  if (node_ids.size() < 2) {
    result.error = "at least 2 support nodes are required";
    return result;
  }
  const int lane_count = std::max(1, conductor_count);
  const PortLayer target_port_layer = category_to_port_layer(category);
  const bool use_lane_row_geometry = maintain_lane_order && lane_count > 1;
  const GroupedSpanSharedContext grouped_span_ctx{node_ids, support_node_by_id, edit_state_access(),
                                                  relation_index_access(), connection_index_access(),
                                                  junction_relations_by_node};
  auto support_position = [&](ObjectId node_id) -> Vec3d { return grouped_span_ctx.support_position(node_id); };
  auto support_pole = [&](ObjectId node_id) -> const Pole* { return grouped_span_ctx.support_pole(node_id); };
  auto layout_yaw_for_pole = [&](const Pole& pole) { return effective_pole_layout_yaw_deg(pole); };
  auto incident_relation_kind_for = [&](ObjectId node_id, ObjectId peer_id) -> JunctionRelationKind {
    const JunctionIncidentRelation* incident = grouped_span_ctx.incident_relation_for(node_id, peer_id);
    return (incident == nullptr) ? JunctionRelationKind::kNone : incident->kind;
  };
  auto span_context_for_segment = [&](ObjectId node_a, ObjectId node_b) -> ConnectionContext {
    if (category == ConnectionCategory::kDrop) {
      return ConnectionContext::kDropAdd;
    }
    auto relation_rank = [](JunctionRelationKind kind) {
      switch (kind) {
      case JunctionRelationKind::kCrossUnderpass:
        return 4;
      case JunctionRelationKind::kSideBranch:
        return 3;
      case JunctionRelationKind::kCornerContinuation:
        return 2;
      case JunctionRelationKind::kThroughMain:
        return 1;
      case JunctionRelationKind::kNone:
      default:
        return 0;
      }
    };
    const JunctionRelationKind relation_a = incident_relation_kind_for(node_a, node_b);
    const JunctionRelationKind relation_b = incident_relation_kind_for(node_b, node_a);
    const JunctionRelationKind segment_relation =
        (relation_rank(relation_a) >= relation_rank(relation_b)) ? relation_a : relation_b;
    if (segment_relation == JunctionRelationKind::kSideBranch ||
        segment_relation == JunctionRelationKind::kCrossUnderpass) {
      return ConnectionContext::kBranchAdd;
    }
    if (segment_relation == JunctionRelationKind::kCornerContinuation) {
      return ConnectionContext::kCornerPass;
    }
    const Pole* pole_a = support_pole(node_a);
    const Pole* pole_b = support_pole(node_b);
    const bool corner_pass = (pole_a != nullptr && pole_a->context.kind == PoleContextKind::kCorner) ||
                             (pole_b != nullptr && pole_b->context.kind == PoleContextKind::kCorner);
    return corner_pass ? ConnectionContext::kCornerPass : ConnectionContext::kTrunkContinue;
  };
  const BundleTemplate* bundle_template = find_bundle_template(bundle_template_id);
  const double grouped_support_fanout_spacing_m =
      std::max(0.1, (bundle_template == nullptr) ? spacing_m : bundle_template->grouped_support_fanout_spacing_m);
  GroupedSpanLoweringDecider lowering(grouped_span_ctx, lowering_policy, bundle_template_id, spacing_m,
                                      grouped_support_fanout_spacing_m);
  GroupedSpanOrientationDecider orientation(grouped_span_ctx);
  GroupedSpanLanePreparer lane_preparer(*this, grouped_span_ctx, lowering, orientation, bundle_id, category,
                                        bundle_template_id, target_port_layer, lane_count, spacing_m,
                                        use_lane_row_geometry, order_decision_policy, &result.change_set);
  auto sync_assignment_from_decisions = [&](SegmentLaneAssignment* assignment) {
    if (assignment == nullptr) {
      return;
    }
    assignment->relation_a = assignment->decision_a.relation_kind;
    assignment->relation_b = assignment->decision_b.relation_kind;
    assignment->continuity_class =
        (assignment->decision_a.continuity_class == ContinuityCategoryClass::kBundleLike ||
         assignment->decision_b.continuity_class == ContinuityCategoryClass::kBundleLike)
            ? ContinuityCategoryClass::kBundleLike
            : ContinuityCategoryClass::kPointLike;
    assignment->default_lower_required =
        assignment->decision_a.default_lower_required || assignment->decision_b.default_lower_required;
    assignment->same_level_feasible =
        assignment->decision_a.same_level_feasible && assignment->decision_b.same_level_feasible;
    assignment->same_level_reason =
        assignment->same_level_feasible ? SameLevelFeasibilityReason::kNone
                                        : assignment->decision_a.same_level_feasible
                                              ? assignment->decision_b.same_level_reason
                                              : assignment->decision_a.same_level_reason;
    assignment->projected_spacing_topview_m =
        (!assignment->decision_a.same_level_feasible && !assignment->decision_b.same_level_feasible)
            ? std::min(assignment->decision_a.projected_spacing_topview_m,
                       assignment->decision_b.projected_spacing_topview_m)
            : (!assignment->decision_a.same_level_feasible ? assignment->decision_a.projected_spacing_topview_m
                                                           : assignment->decision_b.projected_spacing_topview_m);
    assignment->required_clearance_m =
        std::max(assignment->decision_a.required_clearance_m, assignment->decision_b.required_clearance_m);
    assignment->lowering_blocked_by_policy =
        assignment->decision_a.lowering_blocked_by_policy || assignment->decision_b.lowering_blocked_by_policy;
    assignment->unresolved_same_level_conflict =
        assignment->decision_a.unresolved_same_level_conflict || assignment->decision_b.unresolved_same_level_conflict;
  };
  bool first_seeded_from_previous = false;
  EditResult<std::vector<ObjectId>> first_ports =
      lane_preparer.EnsurePorts(node_ids.front(), node_ids[1], 0, true, &first_seeded_from_previous);
  if (!first_ports.ok) {
    result.error = first_ports.error;
    return result;
  }
  if (static_cast<int>(first_ports.value.size()) != lane_count) {
    result.error = "failed to seed first segment lanes";
    return result;
  }
  std::vector<std::vector<ObjectId>> base_ports_by_node(node_ids.size());
  base_ports_by_node.front() = first_ports.value;

  struct BundleOrderScore {
    int segment_xy_intersections = 0;
    int cross_y = 0;
    int cross_z = 0;
    int layer_jump = 0;
    double span_z_delta = 0.0;
  };
  auto secondary_score_less = [&](const BundleOrderScore& a, const BundleOrderScore& b) {
    const auto key_a = std::make_tuple(a.cross_z, a.segment_xy_intersections, a.layer_jump,
                                       static_cast<long long>(std::llround(a.span_z_delta * 1000.0)));
    const auto key_b = std::make_tuple(b.cross_z, b.segment_xy_intersections, b.layer_jump,
                                       static_cast<long long>(std::llround(b.span_z_delta * 1000.0)));
    return key_a < key_b;
  };
  auto evaluate_increment = [&](ObjectId node_a, ObjectId node_b, const std::vector<ObjectId>& lanes_a,
                                const std::vector<ObjectId>& lanes_b) -> BundleOrderScore {
    BundleOrderScore score{};
    const Pole* pa = support_pole(node_a);
    const Pole* pb = support_pole(node_b);
    const Vec3d pos_a = support_position(node_a);
    const Vec3d pos_b = support_position(node_b);

    Vec3d segment_dir{1.0, 0.0, 0.0};
    if (((pos_b.x - pos_a.x) * (pos_b.x - pos_a.x) + (pos_b.y - pos_a.y) * (pos_b.y - pos_a.y) +
         (pos_b.z - pos_a.z) * (pos_b.z - pos_a.z)) > 1e-12) {
      segment_dir = pos_b - pos_a;
      if (!normalize_xy(&segment_dir) || !std::isfinite(segment_dir.x) || !std::isfinite(segment_dir.y)) {
        segment_dir = {1.0, 0.0, 0.0};
      }
    }
    const Vec3d lateral_axis{-segment_dir.y, segment_dir.x, 0.0};
    auto axis_for_node = [&](ObjectId node_id, ObjectId peer_id) -> Vec3d {
      const Vec3d axis = orientation.CanonicalSideAxisForOrder(node_id, peer_id);
      if (axis.x != 0.0 || axis.y != 0.0) {
        return axis;
      }
      return lateral_axis;
    };
    const Vec3d axis_a = axis_for_node(node_a, node_b);
    const Vec3d axis_b = axis_for_node(node_b, node_a);
    double y_sign_b = 1.0;
    if (dot_xy(axis_a, axis_b) < 0.0) {
      y_sign_b = -1.0;
    }
    const bool use_local_y_metric = use_lane_row_geometry;

    std::vector<double> y_a(static_cast<std::size_t>(lane_count), 0.0);
    std::vector<double> y_b(static_cast<std::size_t>(lane_count), 0.0);
    std::vector<double> z_a(static_cast<std::size_t>(lane_count), 0.0);
    std::vector<double> z_b(static_cast<std::size_t>(lane_count), 0.0);

    for (int lane = 0; lane < lane_count; ++lane) {
      const std::size_t idx = static_cast<std::size_t>(lane);
      const Port* port_a = edit_state_access().ports.find(lanes_a[idx]);
      const Port* port_b = edit_state_access().ports.find(lanes_b[idx]);
      if (port_a == nullptr || port_b == nullptr) {
        score.layer_jump += 4;
        score.span_z_delta += 5.0;
        continue;
      }
      if (use_local_y_metric) {
        const Vec3d local_a =
            (pa == nullptr)
                ? port_a->world_position
                : WorldPointToLocal(BuildPoleFrame(pa->world_transform, layout_yaw_for_pole(*pa)),
                                    port_a->world_position);
        const Vec3d local_b =
            (pb == nullptr)
                ? port_b->world_position
                : WorldPointToLocal(BuildPoleFrame(pb->world_transform, layout_yaw_for_pole(*pb)),
                                    port_b->world_position);
        y_a[idx] = local_a.y;
        y_b[idx] = local_b.y * y_sign_b;
      } else {
        const Vec3d da = port_a->world_position - pos_a;
        const Vec3d db = port_b->world_position - pos_b;
        y_a[idx] = dot_xy(da, axis_a);
        y_b[idx] = dot_xy(db, axis_b) * y_sign_b;
      }
      z_a[idx] = port_a->world_position.z;
      z_b[idx] = port_b->world_position.z;
      score.layer_jump += std::abs(port_a->template_layer - port_b->template_layer);
      score.span_z_delta += std::abs(port_a->world_position.z - port_b->world_position.z);
    }

    for (int i = 0; i < lane_count; ++i) {
      for (int j = i + 1; j < lane_count; ++j) {
        const std::size_t ii = static_cast<std::size_t>(i);
        const std::size_t jj = static_cast<std::size_t>(j);
        const Port* port_ai = edit_state_access().ports.find(lanes_a[ii]);
        const Port* port_bi = edit_state_access().ports.find(lanes_b[ii]);
        const Port* port_aj = edit_state_access().ports.find(lanes_a[jj]);
        const Port* port_bj = edit_state_access().ports.find(lanes_b[jj]);
        if (port_ai != nullptr && port_bi != nullptr && port_aj != nullptr && port_bj != nullptr &&
            segments_intersect_xy_strict_local(port_ai->world_position, port_bi->world_position,
                                               port_aj->world_position, port_bj->world_position)) {
          ++score.segment_xy_intersections;
        }
        const double dy_a = y_a[ii] - y_a[jj];
        const double dy_b = y_b[ii] - y_b[jj];
        constexpr double kOrderEps = 1e-4;
        if ((dy_a > kOrderEps && dy_b < -kOrderEps) || (dy_a < -kOrderEps && dy_b > kOrderEps)) {
          ++score.cross_y;
        }
        const double dz_a = z_a[ii] - z_a[jj];
        const double dz_b = z_b[ii] - z_b[jj];
        if ((dz_a > kOrderEps && dz_b < -kOrderEps) || (dz_a < -kOrderEps && dz_b > kOrderEps)) {
          ++score.cross_z;
        }
      }
    }
    return score;
  };
  auto count_adjacent_segment_xy_intersections = [&](const std::vector<ObjectId>& prev_a,
                                                     const std::vector<ObjectId>& prev_b,
                                                     const std::vector<ObjectId>& curr_a,
                                                     const std::vector<ObjectId>& curr_b) {
    int intersections = 0;
    for (int i = 0; i < lane_count; ++i) {
      const std::size_t ii = static_cast<std::size_t>(i);
      const Port* c0 = edit_state_access().ports.find(curr_a[ii]);
      const Port* c1 = edit_state_access().ports.find(curr_b[ii]);
      if (c0 == nullptr || c1 == nullptr) {
        continue;
      }
      for (int j = 0; j < lane_count; ++j) {
        if (i == j) {
          continue;
        }
        const std::size_t jj = static_cast<std::size_t>(j);
        const Port* p0 = edit_state_access().ports.find(prev_a[jj]);
        const Port* p1 = edit_state_access().ports.find(prev_b[jj]);
        if (p0 == nullptr || p1 == nullptr) {
          continue;
        }
        if (segments_intersect_xy_strict_local(c0->world_position, c1->world_position, p0->world_position,
                                               p1->world_position)) {
          ++intersections;
        }
      }
    }
    return intersections;
  };

  const bool order_decision_permutable =
      order_decision_policy == OrderDecisionPolicyKind::kPermutableHomogeneous && lane_count > 1;
  const bool allow_order_decision_reverse = order_decision_permutable;
  constexpr double kAngleEps = 1e-6;
  constexpr double kReverseStraightAngleDeg = 179.999;

  const std::size_t segment_count = node_ids.size() - 1;
  std::vector<std::vector<ObjectId>> prepared_ports_b(segment_count);
  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    const ObjectId node_a = node_ids[seg];
    const ObjectId node_b = node_ids[seg + 1];
    const ObjectId right_order_peer = node_a;
    EditResult<std::vector<ObjectId>> right_ports =
        lane_preparer.EnsurePorts(node_b, right_order_peer, static_cast<int>(seg), false, nullptr);
    if (!right_ports.ok) {
      result.error = right_ports.error;
      return result;
    }
    if (static_cast<int>(right_ports.value.size()) != lane_count) {
      result.error = "failed to prepare right-side lane candidates";
      return result;
    }
    prepared_ports_b[seg] = std::move(right_ports.value);
    base_ports_by_node[seg + 1] = prepared_ports_b[seg];
  }

  auto order_for_choice = [&](const std::vector<ObjectId>& base_ports, OrderDecisionChoiceKind choice) {
    std::vector<ObjectId> ordered = base_ports;
    if (choice == OrderDecisionChoiceKind::kReversed) {
      std::reverse(ordered.begin(), ordered.end());
    }
    return ordered;
  };
  auto choice_for_parity = [](int parity) {
    return (parity != 0) ? OrderDecisionChoiceKind::kReversed : OrderDecisionChoiceKind::kNormal;
  };
  auto compute_turn_angle_deg = [&](std::size_t segment_index) -> double {
    if (segment_index == 0 || segment_index + 1 >= node_ids.size()) {
      return 180.0;
    }
    const Vec3d prev = support_position(node_ids[segment_index - 1]);
    const Vec3d curr = support_position(node_ids[segment_index]);
    const Vec3d next = support_position(node_ids[segment_index + 1]);
    const Vec3d in_check = prev - curr;
    const Vec3d out_check = next - curr;
    if ((in_check.x * in_check.x + in_check.y * in_check.y + in_check.z * in_check.z) <= 1e-12 ||
        (out_check.x * out_check.x + out_check.y * out_check.y + out_check.z * out_check.z) <= 1e-12) {
      return 180.0;
    }
    // Use vertex interior angle: straight-through == 180deg, acute corner < threshold.
    Vec3d in_dir = prev - curr;
    Vec3d out_dir = next - curr;
    if (!normalize_xy(&in_dir) || !normalize_xy(&out_dir)) {
      return 180.0;
    }
    const double d = std::clamp(dot_xy(in_dir, out_dir), -1.0, 1.0);
    const double angle = std::acos(d) * (180.0 / kPi);
    if (!std::isfinite(angle)) {
      return 180.0;
    }
    return angle;
  };

  struct OrientationPlanScore {
    BundleOrderScore order{};
    int adjacent_xy_intersections = 0;
    int orientation_flips = 0;
    int acute_orientation_flips = 0;
  };
  auto add_order_score = [](BundleOrderScore* dst, const BundleOrderScore& src) {
    if (dst == nullptr) {
      return;
    }
    dst->segment_xy_intersections += src.segment_xy_intersections;
    dst->cross_y += src.cross_y;
    dst->cross_z += src.cross_z;
    dst->layer_jump += src.layer_jump;
    dst->span_z_delta += src.span_z_delta;
  };
  auto orientation_plan_less = [&](const OrientationPlanScore& a, const OrientationPlanScore& b) {
    if (use_lane_row_geometry && a.adjacent_xy_intersections != b.adjacent_xy_intersections) {
      return a.adjacent_xy_intersections < b.adjacent_xy_intersections;
    }
    if (a.order.cross_y != b.order.cross_y) {
      return a.order.cross_y < b.order.cross_y;
    }
    if (a.order.segment_xy_intersections != b.order.segment_xy_intersections) {
      return a.order.segment_xy_intersections < b.order.segment_xy_intersections;
    }
    if (a.acute_orientation_flips != b.acute_orientation_flips) {
      return a.acute_orientation_flips < b.acute_orientation_flips;
    }
    if (a.orientation_flips != b.orientation_flips) {
      return a.orientation_flips < b.orientation_flips;
    }
    return secondary_score_less(a.order, b.order);
  };

  std::vector<double> turn_angle_by_segment(segment_count, 180.0);
  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    turn_angle_by_segment[seg] = compute_turn_angle_deg(seg);
  }

  std::vector<int> node_parity(node_ids.size(), 0);
  if (segment_count == 1) {
    const std::vector<int> first_candidates =
        (first_seeded_from_previous || !allow_order_decision_reverse) ? std::vector<int>{0} : std::vector<int>{0, 1};
    OrientationPlanScore best_score{};
    bool has_best = false;
    for (int parity_a : first_candidates) {
      const std::vector<ObjectId> ports_a = order_for_choice(base_ports_by_node[0], choice_for_parity(parity_a));
      for (int parity_b : {0, 1}) {
        if (!allow_order_decision_reverse && parity_b != 0) {
          continue;
        }
        const std::vector<ObjectId> ports_b = order_for_choice(base_ports_by_node[1], choice_for_parity(parity_b));
        OrientationPlanScore candidate{};
        candidate.order = evaluate_increment(node_ids[0], node_ids[1], ports_a, ports_b);
        if (!has_best || orientation_plan_less(candidate, best_score)) {
          has_best = true;
          best_score = candidate;
          node_parity[0] = parity_a;
          node_parity[1] = parity_b;
        }
      }
    }
  } else {
    struct DpCell {
      bool reachable = false;
      OrientationPlanScore score{};
      int prev_prev_parity = -1;
    };
    std::vector<std::array<std::array<DpCell, 2>, 2>> dp(segment_count + 1);
    const std::vector<int> first_candidates =
        (first_seeded_from_previous || !allow_order_decision_reverse) ? std::vector<int>{0} : std::vector<int>{0, 1};

    for (int parity_0 : first_candidates) {
      const std::vector<ObjectId> ports_0 = order_for_choice(base_ports_by_node[0], choice_for_parity(parity_0));
      for (int parity_1 : {0, 1}) {
        if (!allow_order_decision_reverse && parity_1 != 0) {
          continue;
        }
        const std::vector<ObjectId> ports_1 = order_for_choice(base_ports_by_node[1], choice_for_parity(parity_1));
        DpCell& cell = dp[1][parity_0][parity_1];
        cell.reachable = true;
        cell.prev_prev_parity = -1;
        cell.score.order = evaluate_increment(node_ids[0], node_ids[1], ports_0, ports_1);
      }
    }

    for (std::size_t step = 1; step < segment_count; ++step) {
      for (int parity_prev_prev : {0, 1}) {
        for (int parity_prev : {0, 1}) {
          const DpCell& cell = dp[step][parity_prev_prev][parity_prev];
          if (!cell.reachable) {
            continue;
          }
          const int prev_orientation = parity_prev_prev ^ parity_prev;
          for (int parity_curr : {0, 1}) {
            if (!allow_order_decision_reverse && parity_curr != 0) {
              continue;
            }
            const std::vector<ObjectId> ports_prev =
                order_for_choice(base_ports_by_node[step], choice_for_parity(parity_prev));
            const std::vector<ObjectId> ports_curr =
                order_for_choice(base_ports_by_node[step + 1], choice_for_parity(parity_curr));
            OrientationPlanScore candidate = cell.score;
            add_order_score(&candidate.order,
                            evaluate_increment(node_ids[step], node_ids[step + 1], ports_prev, ports_curr));
            if (use_lane_row_geometry) {
              const std::vector<ObjectId> ports_prev_prev =
                  order_for_choice(base_ports_by_node[step - 1], choice_for_parity(parity_prev_prev));
              candidate.adjacent_xy_intersections +=
                  count_adjacent_segment_xy_intersections(ports_prev_prev, ports_prev, ports_prev, ports_curr);
            }
            const int curr_orientation = parity_prev ^ parity_curr;
            if (curr_orientation != prev_orientation) {
              const bool counts_as_flip = (turn_angle_by_segment[step] + kAngleEps < kReverseStraightAngleDeg);
              if (counts_as_flip) {
                ++candidate.orientation_flips;
                const bool is_acute_turn =
                    (turn_angle_by_segment[step] + kAngleEps < layout_settings_.corner_threshold_deg);
                if (is_acute_turn) {
                  ++candidate.acute_orientation_flips;
                }
              }
            }

            DpCell& next = dp[step + 1][parity_prev][parity_curr];
            if (!next.reachable || orientation_plan_less(candidate, next.score)) {
              next.reachable = true;
              next.score = candidate;
              next.prev_prev_parity = parity_prev_prev;
            }
          }
        }
      }
    }

    bool has_best = false;
    OrientationPlanScore best_score{};
    int best_prev = 0;
    int best_curr = 0;
    for (int parity_prev : {0, 1}) {
      for (int parity_curr : {0, 1}) {
        const DpCell& cell = dp[segment_count][parity_prev][parity_curr];
        if (!cell.reachable) {
          continue;
        }
        if (!has_best || orientation_plan_less(cell.score, best_score)) {
          has_best = true;
          best_score = cell.score;
          best_prev = parity_prev;
          best_curr = parity_curr;
        }
      }
    }
    if (!has_best) {
      result.error = "failed to resolve deterministic node orientation plan";
      return result;
    }

    node_parity[node_ids.size() - 2] = best_prev;
    node_parity[node_ids.size() - 1] = best_curr;
    for (std::size_t step = segment_count; step > 1; --step) {
      const DpCell& cell = dp[step][node_parity[step - 1]][node_parity[step]];
      node_parity[step - 2] = cell.prev_prev_parity;
    }
  }

  auto build_local_order_score = [&](std::size_t seg, int parity_a, int parity_b, int prev_parity,
                                     bool has_prev_segment) {
    OrientationPlanScore score{};
    const std::vector<ObjectId> ports_a = order_for_choice(base_ports_by_node[seg], choice_for_parity(parity_a));
    const std::vector<ObjectId> ports_b =
        order_for_choice(base_ports_by_node[seg + 1], choice_for_parity(parity_b));
    score.order = evaluate_increment(node_ids[seg], node_ids[seg + 1], ports_a, ports_b);
    if (use_lane_row_geometry && has_prev_segment) {
      const std::vector<ObjectId> prev_ports =
          order_for_choice(base_ports_by_node[seg - 1], choice_for_parity(prev_parity));
      score.adjacent_xy_intersections =
          count_adjacent_segment_xy_intersections(prev_ports, ports_a, ports_a, ports_b);
    }
    if (has_prev_segment) {
      const int prev_orientation = prev_parity ^ parity_a;
      const int curr_orientation = parity_a ^ parity_b;
      if (curr_orientation != prev_orientation &&
          (turn_angle_by_segment[seg] + kAngleEps < kReverseStraightAngleDeg)) {
        ++score.orientation_flips;
        if (turn_angle_by_segment[seg] + kAngleEps < layout_settings_.corner_threshold_deg) {
          ++score.acute_orientation_flips;
        }
      }
    }
    return score;
  };
  auto order_choice_reason_from_scores = [&](const OrientationPlanScore& chosen, const OrientationPlanScore& alternate,
                                             OrderDecisionPolicyKind policy) {
    if (policy != OrderDecisionPolicyKind::kPermutableHomogeneous) {
      return OrderDecisionChoiceReason::kFixedOrder;
    }
    if (use_lane_row_geometry && chosen.adjacent_xy_intersections != alternate.adjacent_xy_intersections) {
      return OrderDecisionChoiceReason::kCrossingFewer;
    }
    if (chosen.order.cross_y != alternate.order.cross_y) {
      return OrderDecisionChoiceReason::kCrossingFewer;
    }
    if (chosen.order.segment_xy_intersections != alternate.order.segment_xy_intersections) {
      return OrderDecisionChoiceReason::kCrossingFewer;
    }
    if (chosen.order.cross_z != alternate.order.cross_z ||
        std::abs(chosen.order.span_z_delta - alternate.order.span_z_delta) > 1e-6) {
      return OrderDecisionChoiceReason::kSpacingBetter;
    }
    if (chosen.acute_orientation_flips != alternate.acute_orientation_flips ||
        chosen.orientation_flips != alternate.orientation_flips) {
      return OrderDecisionChoiceReason::kTwistSmaller;
    }
    return OrderDecisionChoiceReason::kKeptDefault;
  };

  std::vector<OrderDecisionChoiceKind> node_order_choices(node_ids.size(), OrderDecisionChoiceKind::kNormal);
  for (std::size_t i = 0; i < node_ids.size(); ++i) {
    node_order_choices[i] = choice_for_parity(node_parity[i]);
  }

  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    const ObjectId node_a = node_ids[seg];
    const ObjectId node_b = node_ids[seg + 1];
    const ConnectionContext span_context = span_context_for_segment(node_a, node_b);

    SegmentLaneAssignment assignment{};
    assignment.segment_index = seg;
    assignment.pole_a_id = node_a;
    assignment.pole_b_id = node_b;
    assignment.bundle_id = bundle_id;
    assignment.flow_kind = flow_kind;
    assignment.order_decision_policy = order_decision_policy;
    assignment.order_decision_choice_a = node_order_choices[seg];
    assignment.order_decision_choice_b = node_order_choices[seg + 1];
    assignment.port_ids_a = order_for_choice(base_ports_by_node[seg], assignment.order_decision_choice_a);
    assignment.port_ids_b = order_for_choice(base_ports_by_node[seg + 1], assignment.order_decision_choice_b);
    const bool chosen_orientation_flip =
        (assignment.order_decision_choice_a != assignment.order_decision_choice_b);
    const double turn_angle_deg = compute_turn_angle_deg(seg);
    const bool is_acute_turn = (seg > 0) && (turn_angle_deg + kAngleEps < layout_settings_.corner_threshold_deg);
    LaneFlipReason flip_reason = LaneFlipReason::kNone;
    bool flipped_from_previous = false;
    const bool previous_orientation_flip =
        (seg > 0) ? (node_order_choices[seg - 1] != node_order_choices[seg]) : false;
    if (seg > 0 && chosen_orientation_flip != previous_orientation_flip &&
        (turn_angle_deg + kAngleEps < kReverseStraightAngleDeg)) {
      flipped_from_previous = true;
      if (is_acute_turn) {
        flip_reason = LaneFlipReason::kAcuteTurn;
      }
    }
    if (order_decision_policy == OrderDecisionPolicyKind::kPermutableHomogeneous) {
      const int prev_parity = (seg > 0) ? node_parity[seg - 1] : 0;
      const OrientationPlanScore chosen_score =
          build_local_order_score(seg, node_parity[seg], node_parity[seg + 1], prev_parity, seg > 0);
      const OrientationPlanScore alternate_a =
          build_local_order_score(seg, node_parity[seg] ^ 1, node_parity[seg + 1], prev_parity, seg > 0);
      const OrientationPlanScore alternate_b =
          build_local_order_score(seg, node_parity[seg], node_parity[seg + 1] ^ 1, prev_parity, seg > 0);
      assignment.order_decision_choice_reason_a =
          order_choice_reason_from_scores(chosen_score, alternate_a, order_decision_policy);
      assignment.order_decision_choice_reason_b =
          order_choice_reason_from_scores(chosen_score, alternate_b, order_decision_policy);
    } else {
      assignment.order_decision_choice_reason_a = OrderDecisionChoiceReason::kFixedOrder;
      assignment.order_decision_choice_reason_b = OrderDecisionChoiceReason::kFixedOrder;
    }
    assignment.flipped_from_previous = flipped_from_previous;
    assignment.flip_reason = flip_reason;
    assignment.turn_angle_deg = turn_angle_deg;
    const SegmentRelationFeasibility relation_a = lowering.SegmentRelationFeasibilityFor(node_a, node_b);
    const SegmentRelationFeasibility relation_b = lowering.SegmentRelationFeasibilityFor(node_b, node_a);
    SegmentRelationFeasibility effective_relation_a = relation_a;
    SegmentRelationFeasibility effective_relation_b = relation_b;
    assignment.relation_a = lowering.NodeRelationKindAt(seg);
    assignment.relation_b = lowering.NodeRelationKindAt(seg + 1);
    if (static_cast<int>(assignment.port_ids_a.size()) != lane_count ||
        static_cast<int>(assignment.port_ids_b.size()) != lane_count) {
      result.error = "failed to materialize lane assignment plan";
      return result;
    }

    auto ports_use_branch_support = [&](const std::vector<ObjectId>& port_ids) {
      return std::any_of(port_ids.begin(), port_ids.end(), [&](ObjectId port_id) {
        const Port* port = edit_state_access().ports.find(port_id);
        return port != nullptr && port->placement_source == PortPlacementSourceKind::kBranchSupport;
      });
    };
    auto ports_use_constrained_solver = [&](const std::vector<ObjectId>& port_ids) {
      return std::any_of(port_ids.begin(), port_ids.end(), [&](ObjectId port_id) {
        const Port* port = edit_state_access().ports.find(port_id);
        return port != nullptr && port->placement_source == PortPlacementSourceKind::kPlacementBandConstrained;
      });
    };
    auto ports_use_special_case_source = [&](const std::vector<ObjectId>& port_ids) {
      return std::any_of(port_ids.begin(), port_ids.end(), [&](ObjectId port_id) {
        const Port* port = edit_state_access().ports.find(port_id);
        return port != nullptr &&
               (port->placement_source == PortPlacementSourceKind::kGenerated ||
                port->placement_source == PortPlacementSourceKind::kBranchSupport ||
                port->placement_source == PortPlacementSourceKind::kAerialBranch);
      });
    };
    assignment.uses_branch_support =
        ports_use_branch_support(assignment.port_ids_a) || ports_use_branch_support(assignment.port_ids_b);
    assignment.solver_used_same_level_constraint =
        ports_use_constrained_solver(assignment.port_ids_a) || ports_use_constrained_solver(assignment.port_ids_b);
    auto relation_rank = [](JunctionRelationKind kind) {
      switch (kind) {
      case JunctionRelationKind::kCrossUnderpass:
        return 4;
      case JunctionRelationKind::kSideBranch:
        return 3;
      case JunctionRelationKind::kCornerContinuation:
        return 2;
      case JunctionRelationKind::kThroughMain:
        return 1;
      case JunctionRelationKind::kNone:
      default:
        return 0;
      }
    };
    const JunctionRelationKind segment_relation_kind =
        (relation_rank(assignment.relation_a) >= relation_rank(assignment.relation_b)) ? assignment.relation_a
                                                                                        : assignment.relation_b;
    const bool segment_same_level_feasible =
        effective_relation_a.same_level_feasible && effective_relation_b.same_level_feasible;
    assignment.used_special_case_ports =
        !assignment.solver_used_same_level_constraint && !segment_same_level_feasible &&
        (ports_use_special_case_source(assignment.port_ids_a) || ports_use_special_case_source(assignment.port_ids_b));
    const bool endpoint_policy_block_a = lowering.EndpointLoweringBlockedByPolicy(effective_relation_a);
    const bool endpoint_policy_block_b = lowering.EndpointLoweringBlockedByPolicy(effective_relation_b);
    assignment.lowering_blocked_by_policy = endpoint_policy_block_a || endpoint_policy_block_b;
    assignment.lowering_blocked_by_policy =
        endpoint_policy_block_a || endpoint_policy_block_b;
    if (assignment.lowering_blocked_by_policy) {
      assignment.same_level_reason = SameLevelFeasibilityReason::kCategoryPolicyDisabled;
    }
    assignment.unresolved_same_level_conflict = !segment_same_level_feasible && assignment.lowering_blocked_by_policy;
    const auto pair_info_a = orientation.LoweredSupportPairInfoForEndpoint(node_a, node_b, effective_relation_a);
    const auto pair_info_b = orientation.LoweredSupportPairInfoForEndpoint(node_b, node_a, effective_relation_b);
    EndpointSideDecision side_decision_a =
        orientation.FinalizeEndpointSideDecision(node_a, node_b, orientation.PreferredSideAxisForEndpoint(node_a, node_b,
                                                                                          effective_relation_a));
    EndpointSideDecision side_decision_b =
        orientation.FinalizeEndpointSideDecision(node_b, node_a, orientation.PreferredSideAxisForEndpoint(node_b, node_a,
                                                                                          effective_relation_b));
    if (effective_relation_a.continuity_class == ContinuityCategoryClass::kBundleLike &&
        (effective_relation_a.default_lower_required || !effective_relation_a.same_level_feasible)) {
      side_decision_a = orientation.NormalizeGroupSideDecision(side_decision_a);
    }
    if (effective_relation_b.continuity_class == ContinuityCategoryClass::kBundleLike &&
        (effective_relation_b.default_lower_required || !effective_relation_b.same_level_feasible)) {
      side_decision_b = orientation.NormalizeGroupSideDecision(side_decision_b);
    }
    orientation.FinalizeSideSignForPorts(&side_decision_a, node_a, node_b, assignment.port_ids_a);
    orientation.FinalizeSideSignForPorts(&side_decision_b, node_b, node_a, assignment.port_ids_b);
    EndpointContinuityDecision raw_decision_a =
        lowering.BuildEndpointDecision(effective_relation_a, pair_info_a, side_decision_a, node_a, node_b,
                                order_decision_policy,
                                assignment.order_decision_choice_a, assignment.order_decision_choice_reason_a,
                                assignment.solver_used_same_level_constraint, assignment.used_special_case_ports,
                                endpoint_policy_block_a, assignment.unresolved_same_level_conflict);
    EndpointContinuityDecision raw_decision_b =
        lowering.BuildEndpointDecision(effective_relation_b, pair_info_b, side_decision_b, node_b, node_a,
                                order_decision_policy,
                                assignment.order_decision_choice_b, assignment.order_decision_choice_reason_b,
                                assignment.solver_used_same_level_constraint, assignment.used_special_case_ports,
                                endpoint_policy_block_b, assignment.unresolved_same_level_conflict);
    orientation.PrimeGroupEndpointDecision(raw_decision_a, node_b, pair_info_a, side_decision_a);
    orientation.PrimeGroupEndpointDecision(raw_decision_b, node_a, pair_info_b, side_decision_b);
    assignment.decision_a = orientation.CanonicalizeGroupEndpointDecision(raw_decision_a);
    assignment.decision_b = orientation.CanonicalizeGroupEndpointDecision(raw_decision_b);
    sync_assignment_from_decisions(&assignment);
    const bool decision_lowered_a = UsesAuthoritativeGroupedLoweredSupport(assignment.decision_a);
    const bool decision_lowered_b = UsesAuthoritativeGroupedLoweredSupport(assignment.decision_b);
    assignment.lowering_kind = lowering.LoweringKindForSegment(segment_relation_kind, decision_lowered_a,
                                                               decision_lowered_b);
    assignment.branch_down_offset_m = (decision_lowered_a || decision_lowered_b) ? lowering.effective_branch_down_offset_m() : 0.0;

    for (int lane = 0; lane < lane_count; ++lane) {
      const ObjectId port_a_id = assignment.port_ids_a[static_cast<std::size_t>(lane)];
      const ObjectId port_b_id = assignment.port_ids_b[static_cast<std::size_t>(lane)];
      const auto add = AddSpan(port_a_id, port_b_id, SpanKind::kDistribution, category_to_span_layer(category),
                               bundle_id);
      if (!add.ok) {
        result.error = add.error;
        return result;
      }
      append_change_set(result.change_set, add.change_set);
      result.value.push_back(add.value);

      const Port* pa = edit_state_access().ports.find(port_a_id);
      const Port* pb = edit_state_access().ports.find(port_b_id);
      (void)pa;
      (void)pb;

      Span* span = edit_state_access().spans.find(add.value);
      if (span != nullptr) {
        span->endpoint_node_a_id = node_a;
        span->endpoint_node_b_id = node_b;
        span->placement_context = span_context;
        span->generated_by_rule = true;
        span->generation.generated = true;
        add_unique_id(result.change_set.updated_ids, span->id);
      }
    }

    if (out_lane_assignments != nullptr) {
      out_lane_assignments->push_back(assignment);
    }
    if (out_edge_orientations != nullptr) {
      BackboneEdgeOrientation edge_orientation{};
      edge_orientation.node_a_id = assignment.pole_a_id;
      edge_orientation.node_b_id = assignment.pole_b_id;
      edge_orientation.bundle_template_id = bundle_template_id;
      edge_orientation.flow_decision_rule = assignment.flow_decision_rule;
      edge_orientation.flow_kind = flow_kind;
      edge_orientation.relation_a = assignment.relation_a;
      edge_orientation.relation_b = assignment.relation_b;
      edge_orientation.continuity_class = assignment.continuity_class;
      edge_orientation.default_lower_required = assignment.default_lower_required;
      edge_orientation.same_level_feasible = assignment.same_level_feasible;
      edge_orientation.same_level_reason = assignment.same_level_reason;
      edge_orientation.projected_spacing_topview_m = assignment.projected_spacing_topview_m;
      edge_orientation.required_clearance_m = assignment.required_clearance_m;
      edge_orientation.lowering_blocked_by_policy = assignment.lowering_blocked_by_policy;
      edge_orientation.unresolved_same_level_conflict = assignment.unresolved_same_level_conflict;
      edge_orientation.solver_used_same_level_constraint = assignment.solver_used_same_level_constraint;
      edge_orientation.used_special_case_ports = assignment.used_special_case_ports;
      edge_orientation.order_decision_policy = assignment.order_decision_policy;
      edge_orientation.order_decision_choice_a = assignment.order_decision_choice_a;
      edge_orientation.order_decision_choice_b = assignment.order_decision_choice_b;
      edge_orientation.order_decision_choice_reason_a = assignment.order_decision_choice_reason_a;
      edge_orientation.order_decision_choice_reason_b = assignment.order_decision_choice_reason_b;
    edge_orientation.orientation =
        (assignment.order_decision_choice_a != assignment.order_decision_choice_b) ? LaneOrientation::kReversed
                                                                               : LaneOrientation::kNormal;
      edge_orientation.uses_branch_support = assignment.uses_branch_support;
      edge_orientation.lowering_kind = assignment.lowering_kind;
      edge_orientation.branch_down_offset_m = assignment.branch_down_offset_m;
      edge_orientation.flipped_from_previous = assignment.flipped_from_previous;
      edge_orientation.flip_reason = assignment.flip_reason;
      edge_orientation.turn_angle_deg = assignment.turn_angle_deg;
      out_edge_orientations->push_back(edge_orientation);
    }
  }

  result.ok = !result.value.empty();
  if (!result.ok && result.error.empty()) {
    result.error = "failed to generate grouped spans";
  }
  return result;
}

} // namespace wire::core




