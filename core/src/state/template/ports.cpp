#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace wire::core {

namespace {

constexpr double kZeroLengthEps = 1e-9;
ConnectionCategory port_layer_to_category(PortLayer layer) {
  switch (layer) {
  case PortLayer::kHighVoltage:
    return ConnectionCategory::kHighVoltage;
  case PortLayer::kLowVoltage:
    return ConnectionCategory::kLowVoltage;
  case PortLayer::kCommunication:
    return ConnectionCategory::kCommunication;
  case PortLayer::kOptical:
    return ConnectionCategory::kOptical;
  case PortLayer::kDrop:
    return ConnectionCategory::kDrop;
  default:
    return ConnectionCategory::kLowVoltage;
  }
}

PortLayer span_layer_to_port_layer(SpanLayer layer) {
  switch (layer) {
  case SpanLayer::kHighVoltage:
    return PortLayer::kHighVoltage;
  case SpanLayer::kLowVoltage:
    return PortLayer::kLowVoltage;
  case SpanLayer::kCommunication:
    return PortLayer::kCommunication;
  case SpanLayer::kOptical:
    return PortLayer::kOptical;
  case SpanLayer::kDrop:
    return PortLayer::kDrop;
  default:
    return PortLayer::kUnknown;
  }
}

ConnectionCategory span_layer_to_category(SpanLayer layer) {
  switch (layer) {
  case SpanLayer::kHighVoltage:
    return ConnectionCategory::kHighVoltage;
  case SpanLayer::kLowVoltage:
    return ConnectionCategory::kLowVoltage;
  case SpanLayer::kCommunication:
    return ConnectionCategory::kCommunication;
  case SpanLayer::kOptical:
    return ConnectionCategory::kOptical;
  case SpanLayer::kDrop:
    return ConnectionCategory::kDrop;
  default:
    return ConnectionCategory::kLowVoltage;
  }
}

int target_template_layer_for_category(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kHighVoltage:
    return 2;
  case ConnectionCategory::kLowVoltage:
    return 1;
  case ConnectionCategory::kCommunication:
    return 1;
  case ConnectionCategory::kOptical:
    return 1;
  case ConnectionCategory::kDrop:
    return 0;
  default:
    return 1;
  }
}

int role_score_for_context(SlotRole role, ConnectionContext context) {
  switch (context) {
  case ConnectionContext::kTrunkContinue:
  case ConnectionContext::kCornerPass:
    if (role == SlotRole::kTrunkPreferred)
      return 120;
    if (role == SlotRole::kNeutral)
      return 40;
    return 0;
  case ConnectionContext::kBranchAdd:
    if (role == SlotRole::kBranchPreferred)
      return 120;
    if (role == SlotRole::kNeutral)
      return 20;
    if (role == SlotRole::kTrunkPreferred)
      return 10;
    return 0;
  case ConnectionContext::kDropAdd:
    if (role == SlotRole::kDropPreferred)
      return 120;
    if (role == SlotRole::kNeutral)
      return 10;
    return 0;
  default:
    return 0;
  }
}

SlotSide inner_side_for_turn(double turn_sign) {
  if (turn_sign > 1e-9) {
    return SlotSide::kLeft;
  }
  if (turn_sign < -1e-9) {
    return SlotSide::kRight;
  }
  return SlotSide::kCenter;
}

double apply_corner_side_scale(double local_y, SlotSide slot_side, double turn_sign, double side_scale) {
  if (slot_side == SlotSide::kCenter) {
    return local_y;
  }
  // Always widen non-center lanes for clearance; keep outer side wider than inner side.
  const double inner_scale = 1.0 + (side_scale - 1.0) * 0.35;
  const SlotSide inner_side = inner_side_for_turn(turn_sign);
  if (inner_side == SlotSide::kCenter) {
    return local_y * side_scale;
  }
  if (slot_side == inner_side) {
    return local_y * inner_scale;
  }
  return local_y * side_scale;
}

template <typename TValue> void append_unique(std::vector<TValue>& dst, const std::vector<TValue>& src) {
  for (const TValue& value : src) {
    if (std::find(dst.begin(), dst.end(), value) == dst.end()) {
      dst.push_back(value);
    }
  }
}

void append_change_set(ChangeSet& dst, const ChangeSet& src) {
  append_unique(dst.created_ids, src.created_ids);
  append_unique(dst.updated_ids, src.updated_ids);
  append_unique(dst.deleted_ids, src.deleted_ids);
}

Vec3d local_to_world_on_pole(const Transformd& tf, double yaw_deg, const Vec3d& local) {
  return LocalPointToWorld(BuildPoleFrame(tf, yaw_deg), local);
}

const std::vector<ObjectId>& relation_ids_or_empty(const std::unordered_map<ObjectId, std::vector<ObjectId>>& index,
                                                   ObjectId owner_id) {
  static const std::vector<ObjectId> kEmpty;
  const auto it = index.find(owner_id);
  return (it == index.end()) ? kEmpty : it->second;
}

} // namespace

PortLayer CoreState::category_to_port_layer(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kHighVoltage:
    return PortLayer::kHighVoltage;
  case ConnectionCategory::kLowVoltage:
    return PortLayer::kLowVoltage;
  case ConnectionCategory::kCommunication:
    return PortLayer::kCommunication;
  case ConnectionCategory::kOptical:
    return PortLayer::kOptical;
  case ConnectionCategory::kDrop:
    return PortLayer::kDrop;
  default:
    return PortLayer::kUnknown;
  }
}

SpanLayer CoreState::category_to_span_layer(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kHighVoltage:
    return SpanLayer::kHighVoltage;
  case ConnectionCategory::kLowVoltage:
    return SpanLayer::kLowVoltage;
  case ConnectionCategory::kCommunication:
    return SpanLayer::kCommunication;
  case ConnectionCategory::kOptical:
    return SpanLayer::kOptical;
  case ConnectionCategory::kDrop:
    return SpanLayer::kDrop;
  default:
    return SpanLayer::kUnknown;
  }
}

BundleKind CoreState::category_to_bundle_kind(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kHighVoltage:
    return BundleKind::kHighVoltage;
  case ConnectionCategory::kLowVoltage:
    return BundleKind::kLowVoltage;
  case ConnectionCategory::kCommunication:
    return BundleKind::kCommunication;
  case ConnectionCategory::kOptical:
    return BundleKind::kOptical;
  case ConnectionCategory::kDrop:
    return BundleKind::kDrop;
  default:
    return BundleKind::kLowVoltage;
  }
}

PortKind CoreState::category_to_port_kind(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kCommunication:
  case ConnectionCategory::kOptical:
    return PortKind::kCommunication;
  default:
    return PortKind::kPower;
  }
}

std::vector<PortPlacementBand> CoreState::sorted_port_bands(const PoleTypeDefinition& pole_type,
                                                            ConnectionCategory category) const {
  std::vector<PortPlacementBand> out;
  for (const PortPlacementBand& band : pole_type.port_bands) {
    if (band.enabled && band.category == category) {
      out.push_back(band);
    }
  }
  std::sort(out.begin(), out.end(), [](const PortPlacementBand& a, const PortPlacementBand& b) {
    if (a.priority != b.priority) {
      return a.priority > b.priority;
    }
    return a.band_id < b.band_id;
  });
  return out;
}

bool CoreState::is_port_band_used(ObjectId pole_id, const PortPlacementBand& band) const {
  for (ObjectId port_id : relation_ids_or_empty(runtime_.relation_index.ports_by_pole, pole_id)) {
    const Port* port = authoritative_.edit_state.ports.find(port_id);
    if (port == nullptr) {
      continue;
    }
    if (port->generated_from_template && port->category == band.category && port->template_layer == band.layer &&
        port->template_side == band.side && port->template_role == band.role) {
      return true;
    }
  }
  return false;
}

EditResult<ObjectId> CoreState::ensure_pole_connection_port(const PortResolutionRequest& request) {
  EditResult<ObjectId> result;
  const Pole* pole = authoritative_.edit_state.poles.find(request.pole_id);
  const PortResolutionHints& hints = request.hints;
  const ContinuityCategoryClass continuity_class_hint = hints.continuity_class;
  const bool default_lower_required_hint = hints.default_lower_required;
  const bool same_level_feasible_hint = hints.same_level_feasible;
  const SameLevelFeasibilityReason same_level_reason_hint = hints.same_level_reason;
  const double projected_spacing_topview_hint_m = hints.projected_spacing_topview_m;
  const double required_clearance_hint_m = hints.required_clearance_m;
  const JunctionRelationKind relation_kind_hint = hints.relation_kind;

  PortResolutionDebugRecord debug{};
  debug.pole_id = request.pole_id;
  debug.peer_pole_id = request.peer_pole_id;
  debug.reference_span_id = request.reference_span_id;
  debug.category = request.category;
  debug.connection_context = request.connection_context;
  debug.pole_context = request.pole_context;
  debug.corner_angle_deg = request.corner_angle_deg;
  debug.corner_turn_sign = request.corner_turn_sign;
  debug.side_scale = compute_side_scale(request.pole_context, request.corner_angle_deg);
  debug.continuity_class_hint = continuity_class_hint;
  debug.default_lower_required_hint = default_lower_required_hint;
  debug.same_level_feasible_hint = same_level_feasible_hint;
  debug.same_level_reason_hint = same_level_reason_hint;
  debug.projected_spacing_topview_hint_m = projected_spacing_topview_hint_m;
  debug.required_clearance_hint_m = required_clearance_hint_m;
  debug.relation_kind_hint = relation_kind_hint;

  auto push_debug = [&]() {
    debug_.port_resolution_debug_records.push_back(debug);
    constexpr std::size_t kMaxDebugRecords = 256;
    if (debug_.port_resolution_debug_records.size() > kMaxDebugRecords) {
      debug_.port_resolution_debug_records.erase(debug_.port_resolution_debug_records.begin());
    }
  };

  if (pole == nullptr) {
    result.error = "pole not found";
    debug.result = result.error;
    push_debug();
    return result;
  }

  auto connection_count = [&](ObjectId port_id) -> std::size_t {
    auto it = runtime_.connection_index.spans_by_port.find(port_id);
    if (it == runtime_.connection_index.spans_by_port.end()) {
      return 0;
    }
    return it->second.size();
  };

  std::vector<Port*> owned_ports;
  owned_ports.reserve(relation_ids_or_empty(runtime_.relation_index.ports_by_pole, request.pole_id).size());
  for (ObjectId port_id : relation_ids_or_empty(runtime_.relation_index.ports_by_pole, request.pole_id)) {
    Port* port = authoritative_.edit_state.ports.find(port_id);
    if (port == nullptr) {
      continue;
    }
    owned_ports.push_back(port);
  }

  auto same_side_layer_usage = [&](SlotSide side, int layer) -> std::size_t {
    std::size_t count = 0;
    for (const Port* port : owned_ports) {
      if (port->template_side != side || port->template_layer != layer) {
        continue;
      }
      if (connection_count(port->id) > 0) {
        ++count;
      }
    }
    return count;
  };

  const PoleTypeDefinition* pole_type = find_pole_type(pole->pole_type_id);
  if (pole_type != nullptr) {
    auto bands = sorted_port_bands(*pole_type, request.category);
    const double layout_yaw = effective_port_layout_yaw_deg(*pole, request.category);
    const PoleFrame pole_frame = BuildPoleFrame(pole->world_transform, layout_yaw);

    struct BandSolveResult {
      double lateral_m = 0.0;
      double height_m = 0.0;
      bool min_spacing_satisfied = false;
      bool overflow_used = false;
      std::size_t conflict_count = 0;
      double nearest_distance_m = 0.0;
    };

    const int target_layer = target_template_layer_for_category(request.category);
    const Pole* peer_pole = authoritative_.edit_state.poles.find(request.peer_pole_id);
    const SlotSide preferred_side = preferred_side_from_geometry(*pole, peer_pole, 0.10);
    const bool prefer_non_center = (preferred_side != SlotSide::kCenter);
    std::vector<std::pair<ObjectId, Vec3d>> occupied_locals{};
    occupied_locals.reserve(owned_ports.size());
    for (const Port* port : owned_ports) {
      if (port == nullptr) {
        continue;
      }
      occupied_locals.emplace_back(port->id, WorldPointToLocal(pole_frame, port->world_position));
    }

    auto evaluate_band_position = [&](const PortPlacementBand& band, ObjectId ignored_port_id, double lateral_m,
                                      double height_m) {
      std::size_t conflict_count = 0;
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (const auto& occupied : occupied_locals) {
        if (occupied.first == ignored_port_id) {
          continue;
        }
        const Vec3d& local = occupied.second;
        const double distance = std::hypot(local.y - lateral_m, local.z - height_m);
        nearest_distance = std::min(nearest_distance, distance);
        if (distance + 1e-9 < band.min_spacing_m) {
          ++conflict_count;
        }
      }
      return std::pair<std::size_t, double>{conflict_count,
                                            std::isfinite(nearest_distance) ? nearest_distance : band.min_spacing_m};
    };

    auto solve_in_band = [&](const PortPlacementBand& band, ObjectId ignored_port_id, double target_lateral,
                             double target_height) -> BandSolveResult {
      BandSolveResult solve{};
      solve.lateral_m = std::clamp(target_lateral, band.lateral_min_m, band.lateral_max_m);
      solve.height_m = std::clamp(target_height, band.height_min_m, band.height_max_m);

      auto [base_conflicts, base_nearest] =
          evaluate_band_position(band, ignored_port_id, solve.lateral_m, solve.height_m);
      solve.conflict_count = base_conflicts;
      solve.nearest_distance_m = base_nearest;
      solve.min_spacing_satisfied = (base_conflicts == 0);
      if (solve.min_spacing_satisfied) {
        return solve;
      }

      const double lateral_step = std::max(0.04, band.min_spacing_m * 0.5);
      const double height_step = std::max(0.04, band.min_spacing_m * 0.45);
      std::size_t best_conflicts = base_conflicts;
      double best_nearest = base_nearest;
      double best_lateral = solve.lateral_m;
      double best_height = solve.height_m;

      for (int li = -8; li <= 8; ++li) {
        const double candidate_lateral =
            std::clamp(target_lateral + static_cast<double>(li) * lateral_step, band.lateral_min_m, band.lateral_max_m);
        const int max_hi = (band.overflow_policy == BandOverflowPolicy::kRaiseHeight ||
                            band.overflow_policy == BandOverflowPolicy::kConstrainedFallback)
                               ? 8
                               : 2;
        for (int hi = -max_hi; hi <= max_hi; ++hi) {
          const double candidate_height = std::clamp(target_height + static_cast<double>(hi) * height_step, band.height_min_m,
                                                     band.height_max_m);
          const auto [conflicts, nearest] =
              evaluate_band_position(band, ignored_port_id, candidate_lateral, candidate_height);
          const bool better = (conflicts < best_conflicts) || (conflicts == best_conflicts && nearest > best_nearest);
          if (!better) {
            continue;
          }
          best_conflicts = conflicts;
          best_nearest = nearest;
          best_lateral = candidate_lateral;
          best_height = candidate_height;
        }
      }

      solve.lateral_m = best_lateral;
      solve.height_m = best_height;
      solve.conflict_count = best_conflicts;
      solve.nearest_distance_m = best_nearest;
      solve.min_spacing_satisfied = (best_conflicts == 0);
      solve.overflow_used = !solve.min_spacing_satisfied;
      return solve;
    };

    auto solve_constrained_fallback = [&](const PortPlacementBand& band, double preferred_lateral) -> BandSolveResult {
      BandSolveResult solve{};
      double max_local_z = band.height_max_m;
      for (const auto& occupied : occupied_locals) {
        max_local_z = std::max(max_local_z, occupied.second.z);
      }
      solve.lateral_m = std::clamp(preferred_lateral, band.lateral_min_m, band.lateral_max_m);
      solve.height_m = max_local_z + std::max(0.05, band.min_spacing_m);
      auto [conflicts, nearest] = evaluate_band_position(band, kInvalidObjectId, solve.lateral_m, solve.height_m);
      solve.conflict_count = conflicts;
      solve.nearest_distance_m = nearest;
      solve.min_spacing_satisfied = (conflicts == 0);
      solve.overflow_used = true;
      return solve;
    };

    auto find_band_port = [&](const PortPlacementBand& band) -> std::pair<Port*, std::size_t> {
      Port* band_port = nullptr;
      std::size_t band_port_usage = std::numeric_limits<std::size_t>::max();
      for (Port* owned : owned_ports) {
        if (owned == nullptr) {
          continue;
        }
        if (std::find(request.excluded_port_ids.begin(), request.excluded_port_ids.end(), owned->id) !=
            request.excluded_port_ids.end()) {
          continue;
        }
        if (!owned->generated_from_template || owned->category != band.category || owned->template_layer != band.layer ||
            owned->template_side != band.side || owned->template_role != band.role) {
          continue;
        }
        const std::size_t usage = connection_count(owned->id);
        if (band_port == nullptr || usage < band_port_usage || (usage == band_port_usage && owned->id < band_port->id)) {
          band_port = owned;
          band_port_usage = usage;
        }
      }
      return {band_port, (band_port == nullptr) ? 0u : band_port_usage};
    };

    int best_total = std::numeric_limits<int>::min();
    int best_tie = -1;
    const PortPlacementBand* best_band = nullptr;
    Port* best_port = nullptr;
    BandSolveResult best_solve{};

    int candidate_rank = 0;
    for (const PortPlacementBand& band : bands) {
      PlacementCandidateDebug candidate{};
      candidate.candidate_rank = candidate_rank++;
      candidate.band_id = band.band_id;
      candidate.band_layer = band.layer;
      candidate.band_side = band.side;
      candidate.band_role = band.role;
      candidate.band_lateral_min_m = band.lateral_min_m;
      candidate.band_lateral_max_m = band.lateral_max_m;
      candidate.band_height_min_m = band.height_min_m;
      candidate.band_height_max_m = band.height_max_m;
      candidate.continuity_class_hint = continuity_class_hint;
      candidate.default_lower_required_hint = default_lower_required_hint;
      candidate.same_level_feasible_hint = same_level_feasible_hint;
      candidate.same_level_reason_hint = same_level_reason_hint;
      candidate.projected_spacing_topview_hint_m = projected_spacing_topview_hint_m;
      candidate.required_clearance_hint_m = required_clearance_hint_m;
      candidate.relation_kind_hint = relation_kind_hint;
      candidate.category_score = (band.category == request.category) ? 500 : -100000;
      if (candidate.category_score < 0) {
        candidate.eligible = false;
        candidate.reason = "category mismatch";
        debug.candidates.push_back(candidate);
        continue;
      }

      candidate.role_score = role_score_for_context(band.role, request.connection_context);
      candidate.context_score = 0;
      if (request.connection_context == ConnectionContext::kCornerPass) {
        candidate.context_score += (band.side == SlotSide::kCenter) ? 10 : 30;
      } else if (request.connection_context == ConnectionContext::kBranchAdd) {
        candidate.context_score += (band.side == SlotSide::kCenter) ? 0 : 20;
      } else if (request.connection_context == ConnectionContext::kDropAdd) {
        candidate.context_score += (band.side == SlotSide::kCenter) ? 25 : 0;
      }

      candidate.layer_score = 60 - (20 * std::abs(band.layer - target_layer));
      candidate.side_score = 0;
      if (prefer_non_center) {
        if (band.side == preferred_side) {
          candidate.side_score += 220;
        } else if (band.side == SlotSide::kCenter) {
          candidate.side_score -= 15;
        }
      } else {
        // Geometry cannot decide left/right near center; only apply tiny deterministic fallback.
        if (((request.branch_index & 1u) == 0u && band.side == SlotSide::kLeft) ||
            ((request.branch_index & 1u) == 1u && band.side == SlotSide::kRight)) {
          candidate.side_score += 2;
        }
      }
      if (request.prefer_template_match) {
        if (band.layer == request.preferred_template_layer) {
          candidate.side_score += 25;
        }
        if (band.side == request.preferred_template_side) {
          candidate.side_score += 90;
        }
        if (band.role == request.preferred_template_role) {
          candidate.role_score += 30;
        }
      }
      const double preferred_lateral =
          (prefer_non_center && band.side == preferred_side)
              ? std::clamp(band.lateral_center_m + ((preferred_side == SlotSide::kLeft) ? -0.05 : 0.05),
                           band.lateral_min_m, band.lateral_max_m)
              : std::clamp(band.lateral_center_m, band.lateral_min_m, band.lateral_max_m);
      const double preferred_height =
          request.prefer_template_match && band.layer == request.preferred_template_layer
              ? std::clamp(band.height_center_m + 0.05, band.height_min_m, band.height_max_m)
              : std::clamp(band.height_center_m, band.height_min_m, band.height_max_m);

      if (!same_level_feasible_hint) {
        if (band.side != SlotSide::kCenter) {
          candidate.side_score += 60;
        } else {
          candidate.side_score -= 35;
        }
        if (default_lower_required_hint) {
          if (band.side != SlotSide::kCenter) {
            candidate.side_score += 55;
          } else {
            candidate.side_score -= 70;
          }
        }
        switch (relation_kind_hint) {
        case JunctionRelationKind::kSideBranch:
        case JunctionRelationKind::kCrossUnderpass:
          if (band.role == SlotRole::kBranchPreferred) {
            candidate.role_score += 140;
          } else if (band.role == SlotRole::kTrunkPreferred) {
            candidate.role_score -= 40;
          }
          break;
        case JunctionRelationKind::kCornerContinuation:
          if (band.side != SlotSide::kCenter) {
            candidate.context_score += 50;
          }
          break;
        default:
          break;
        }
        const double lowered_height_bias = std::max(0.0, preferred_height - band.height_center_m);
        candidate.context_score += static_cast<int>(std::round(lowered_height_bias * 180.0));
      }
      candidate.priority_score = band.priority;

      auto [band_port, band_port_usage] = find_band_port(band);

      candidate.usage_count = (band_port == nullptr) ? 0 : connection_count(band_port->id);
      const bool can_reuse_existing_port = (band_port != nullptr && candidate.usage_count == 0);
      const bool has_unused_sibling_band =
          std::any_of(bands.begin(), bands.end(), [&](const PortPlacementBand& sibling) {
            if (sibling.band_id == band.band_id) {
              return false;
            }
            const auto [sibling_port, sibling_usage] = find_band_port(sibling);
            return sibling_port == nullptr || sibling_usage == 0;
          });
      const bool defer_reuse_until_unused_siblings_consumed =
          band.allow_multiple && candidate.usage_count > 0 && has_unused_sibling_band;
      const bool allow_constrained_extra_port =
          !same_level_feasible_hint && band_port != nullptr && candidate.usage_count > 0;
      const bool can_create_new_port = request.allow_generate_port && !defer_reuse_until_unused_siblings_consumed &&
                                       (band_port == nullptr || (band.allow_multiple && candidate.usage_count > 0) ||
                                        allow_constrained_extra_port);
      candidate.resolved_port_id = can_reuse_existing_port ? band_port->id : kInvalidObjectId;
      if (can_reuse_existing_port) {
        candidate.usage_score = 80;
      } else if (can_create_new_port) {
        candidate.usage_score = 75 - (band.allow_multiple ? (25 * static_cast<int>(candidate.usage_count)) : 0);
      } else {
        candidate.usage_score = -100000;
      }
      candidate.congestion_count = same_side_layer_usage(band.side, band.layer);
      candidate.congestion_score = -15 * static_cast<int>(candidate.congestion_count);

      BandSolveResult solve{};
      if (can_reuse_existing_port || can_create_new_port) {
        solve = solve_in_band(band, can_reuse_existing_port ? band_port->id : kInvalidObjectId, preferred_lateral,
                              preferred_height);
        if (!solve.min_spacing_satisfied &&
            (band.overflow_policy == BandOverflowPolicy::kConstrainedFallback || !same_level_feasible_hint)) {
          solve = solve_constrained_fallback(band, preferred_lateral);
        }
      } else {
        solve.lateral_m = preferred_lateral;
        solve.height_m = preferred_height;
        solve.min_spacing_satisfied = true;
      }
      candidate.resolved_lateral_m = solve.lateral_m;
      candidate.resolved_height_m = solve.height_m;
      candidate.min_spacing_satisfied = solve.min_spacing_satisfied;
      candidate.overflow_used = solve.overflow_used;
      if ((can_reuse_existing_port || can_create_new_port) && !solve.min_spacing_satisfied) {
        candidate.usage_score -= 35 + static_cast<int>(solve.conflict_count) * 8;
      }

      candidate.tie_breaker = static_cast<int>(
          deterministic_tiebreak_0_255(request.pole_id, band.band_id, request.category, request.connection_context,
                                       request.peer_pole_id, request.reference_span_id, request.branch_index));
      candidate.total_score = candidate.category_score + candidate.context_score + candidate.layer_score +
                              candidate.side_score + candidate.role_score + candidate.priority_score +
                              candidate.usage_score + candidate.congestion_score + (candidate.tie_breaker / 16);

      candidate.eligible = candidate.usage_score > -100000 &&
                           ((can_reuse_existing_port || can_create_new_port) ? solve.min_spacing_satisfied : true);
      if (!candidate.eligible) {
        if (defer_reuse_until_unused_siblings_consumed) {
          candidate.reason = "unused sibling preferred";
        } else
        candidate.reason = can_create_new_port ? "band conflict unresolved" : "band unavailable";
      } else if (candidate.overflow_used) {
        candidate.reason = "constrained fallback";
      } else {
        candidate.reason = "ok";
      }
      debug.candidates.push_back(candidate);

      if (!candidate.eligible) {
        continue;
      }
      if (candidate.total_score > best_total ||
          (candidate.total_score == best_total && candidate.tie_breaker > best_tie)) {
        best_total = candidate.total_score;
        best_tie = candidate.tie_breaker;
        best_band = &band;
        best_port = can_reuse_existing_port ? band_port : nullptr;
        best_solve = solve;
      }
    }

    if (best_band != nullptr) {
      const bool use_constrained_source = !same_level_feasible_hint;
      const PortPlacementSourceKind selected_source =
          use_constrained_source ? PortPlacementSourceKind::kPlacementBandConstrained
                                 : PortPlacementSourceKind::kPlacementBand;

      auto realize_selected_world = [&]() {
        Vec3d adjusted_local{0.0, best_solve.lateral_m, best_solve.height_m};
        const bool apply_angle_correction = authoritative_.layout_settings.angle_correction_enabled &&
                                            request.pole_context == PoleContextKind::kCorner &&
                                            best_band->side != SlotSide::kCenter;
        double applied_scale = 1.0;
        if (apply_angle_correction) {
          adjusted_local.y =
              apply_corner_side_scale(adjusted_local.y, best_band->side, request.corner_turn_sign, debug.side_scale);
          if (std::abs(best_solve.lateral_m) > 1e-9) {
            applied_scale = std::abs(adjusted_local.y / best_solve.lateral_m);
          }
        }
        adjusted_local = apply_pole_clearance_to_local(*pole, adjusted_local, best_band->side);
        return std::pair<Vec3d, double>{local_to_world_on_pole(pole->world_transform, layout_yaw, adjusted_local),
                                        apply_angle_correction ? applied_scale : 1.0};
      };

      if (best_port != nullptr) {
        const auto [selected_world, applied_scale] = realize_selected_world();
        const bool moved = std::abs(best_port->world_position.x - selected_world.x) > 1e-9 ||
                           std::abs(best_port->world_position.y - selected_world.y) > 1e-9 ||
                           std::abs(best_port->world_position.z - selected_world.z) > 1e-9;
        if (moved) {
          best_port->world_position = selected_world;
          add_unique_id(result.change_set.updated_ids, best_port->id);
        }
        best_port->angle_correction_applied = authoritative_.layout_settings.angle_correction_enabled &&
                                              request.pole_context == PoleContextKind::kCorner &&
                                              best_band->side != SlotSide::kCenter;
        best_port->side_scale_applied = applied_scale;
        apply_port_position_mode(*best_port, PortPositionMode::kAuto, selected_source);
        result.ok = true;
        result.value = best_port->id;
        debug.selected_port_id = best_port->id;
        debug.overflow_triggered = false;
        debug.solver_used_same_level_constraint = use_constrained_source;
        debug.result = "selected existing template-owned port";
        push_debug();
        return result;
      }

      const auto [world_position, applied_scale] = realize_selected_world();
      EditResult<ObjectId> add_port_result =
          AddPort(request.pole_id, world_position, category_to_port_kind(request.category),
                  category_to_port_layer(request.category), best_band->local_direction);
      if (!add_port_result.ok) {
        debug.result = "failed to create template-owned port: " + add_port_result.error;
        push_debug();
        return add_port_result;
      }
      Port* created = authoritative_.edit_state.ports.find(add_port_result.value);
      if (created != nullptr) {
        created->category = request.category;
        created->template_layer = best_band->layer;
        created->template_side = best_band->side;
        created->template_role = best_band->role;
        created->generated_from_template = true;
        created->generated_by_rule = true;
        created->placement_context = request.connection_context;
        created->angle_correction_applied = authoritative_.layout_settings.angle_correction_enabled &&
                                            request.pole_context == PoleContextKind::kCorner &&
                                            best_band->side != SlotSide::kCenter;
        created->side_scale_applied = applied_scale;
        apply_port_position_mode(*created, PortPositionMode::kAuto, selected_source);
        add_unique_id(add_port_result.change_set.updated_ids, created->id);
      }
      debug.selected_port_id = add_port_result.value;
      debug.created_new_port = true;
      debug.overflow_triggered = best_solve.overflow_used;
      debug.solver_used_same_level_constraint = use_constrained_source;
      debug.result = best_solve.overflow_used ? "created constrained fallback port" : "created template-owned port";
      push_debug();
      return add_port_result;
    }
  }

  std::ostringstream oss;
  oss << "no placement band available for pole " << request.pole_id << " category " << static_cast<int>(request.category);
  result.error = oss.str();
  debug.result = result.error;
  push_debug();
  return result;
}

std::uint8_t CoreState::deterministic_tiebreak_0_255(ObjectId pole_id, int tiebreak_key, ConnectionCategory category,
                                                      ConnectionContext context, ObjectId peer_pole_id,
                                                      ObjectId reference_span_id, std::uint32_t branch_index) {
  auto mix = [](std::uint32_t h, std::uint64_t v) -> std::uint32_t {
    h ^= static_cast<std::uint32_t>(v & 0xFFFFFFFFu);
    h *= 16777619u;
    h ^= static_cast<std::uint32_t>((v >> 32) & 0xFFFFFFFFu);
    h *= 16777619u;
    return h;
  };

  std::uint32_t h = 2166136261u;
  h = mix(h, pole_id);
  h = mix(h, static_cast<std::uint64_t>(tiebreak_key));
  h = mix(h, static_cast<std::uint64_t>(static_cast<std::uint8_t>(category)));
  h = mix(h, static_cast<std::uint64_t>(static_cast<std::uint8_t>(context)));
  h = mix(h, peer_pole_id);
  h = mix(h, reference_span_id);
  h = mix(h, branch_index);
  return static_cast<std::uint8_t>(h & 0xFFu);
}

bool CoreState::is_valid_slot_side(SlotSide side) {
  return side == SlotSide::kLeft || side == SlotSide::kCenter || side == SlotSide::kRight;
}

bool CoreState::is_valid_slot_role(SlotRole role) {
  return role == SlotRole::kNeutral || role == SlotRole::kTrunkPreferred || role == SlotRole::kBranchPreferred ||
         role == SlotRole::kDropPreferred;
}


} // namespace wire::core
