#include "wire/core/core_state.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <unordered_set>
#include <utility>
#include <vector>

namespace wire::core {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kZeroLengthEps = 1e-9;
constexpr double kSharpCornerInteriorAngleMaxDeg = 75.0;

template <typename TValue>
void append_unique(std::vector<TValue>& dst, const std::vector<TValue>& src) {
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
  append_unique(dst.dirty_span_ids, src.dirty_span_ids);
}

double normalize_yaw_deg(double yaw_deg) {
  double out = std::fmod(yaw_deg, 360.0);
  if (out <= -180.0) {
    out += 360.0;
  } else if (out > 180.0) {
    out -= 360.0;
  }
  return out;
}

double compute_corner_interior_angle_deg_at_point(const std::vector<Vec3d>& points, std::size_t index) {
  if (points.size() < 3 || index == 0 || index + 1 >= points.size()) {
    return 0.0;
  }
  const Vec3d a{points[index - 1].x - points[index].x, points[index - 1].y - points[index].y,
                points[index - 1].z - points[index].z};
  const Vec3d b{points[index + 1].x - points[index].x, points[index + 1].y - points[index].y,
                points[index + 1].z - points[index].z};
  const double la = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
  const double lb = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
  if (la <= 1e-9 || lb <= 1e-9) {
    return 0.0;
  }
  double dot = (a.x * b.x + a.y * b.y + a.z * b.z) / (la * lb);
  dot = std::clamp(dot, -1.0, 1.0);
  const double interior_deg = std::acos(dot) * (180.0 / kPi);
  return std::clamp(interior_deg, 0.0, 180.0);
}

bool compute_corner_bisector_xy(const std::vector<Vec3d>& points, std::size_t index, Vec3d* out_bisector_xy) {
  if (out_bisector_xy == nullptr) {
    return false;
  }
  if (points.size() < 3 || index == 0 || index + 1 >= points.size()) {
    return false;
  }
  // Interior-angle bisector uses rays from corner to prev/next.
  Vec3d in_dir{
      points[index - 1].x - points[index].x,
      points[index - 1].y - points[index].y,
      0.0,
  };
  Vec3d out_dir{
      points[index + 1].x - points[index].x,
      points[index + 1].y - points[index].y,
      0.0,
  };
  const double in_len = std::sqrt(in_dir.x * in_dir.x + in_dir.y * in_dir.y);
  const double out_len = std::sqrt(out_dir.x * out_dir.x + out_dir.y * out_dir.y);
  if (in_len <= 1e-9 || out_len <= 1e-9) {
    return false;
  }
  in_dir.x /= in_len;
  in_dir.y /= in_len;
  out_dir.x /= out_len;
  out_dir.y /= out_len;

  Vec3d bisector{
      in_dir.x + out_dir.x,
      in_dir.y + out_dir.y,
      0.0,
  };
  const double b_len = std::sqrt(bisector.x * bisector.x + bisector.y * bisector.y);
  if (b_len <= 1e-9) {
    return false;
  }
  bisector.x /= b_len;
  bisector.y /= b_len;
  *out_bisector_xy = bisector;
  return true;
}

Transformd make_auto_pole_transform(const std::vector<Vec3d>& points, std::size_t index) {
  Transformd tf{};
  tf.position = points[index];

  Vec3d tangent{};
  if (points.size() >= 2) {
    if (index == 0) {
      tangent = points[1] - points[0];
    } else if (index + 1 >= points.size()) {
      tangent = points[index] - points[index - 1];
    } else {
      tangent = points[index + 1] - points[index - 1];
    }
  }

  const double len2 = tangent.x * tangent.x + tangent.y * tangent.y + tangent.z * tangent.z;
  if (len2 > 1e-12) {
    double yaw_deg = std::atan2(tangent.y, tangent.x) * (180.0 / kPi);
    // Sharp-corner rule is based on corner interior angle (< 75 deg).
    const double corner_interior_deg = compute_corner_interior_angle_deg_at_point(points, index);
    if (corner_interior_deg > 1e-6 && corner_interior_deg < kSharpCornerInteriorAngleMaxDeg) {
      Vec3d bisector_xy{};
      if (compute_corner_bisector_xy(points, index, &bisector_xy)) {
        // Keep slot spread axis (local Y) perpendicular to bisector to avoid lane compression at acute corners.
        yaw_deg = std::atan2(bisector_xy.y, bisector_xy.x) * (180.0 / kPi);
      } else {
        // Fallback keeps legacy behavior when bisector is degenerate.
        yaw_deg += 90.0;
      }
    }
    tf.rotation_euler_deg.z = normalize_yaw_deg(yaw_deg);
  }
  return tf;
}

} // namespace

EditResult<std::vector<ObjectId>> CoreState::GeneratePolesAlongRoad(const RoadSegment& road, double interval,
                                                                    PoleTypeId pole_type_id) {
  EditResult<std::vector<ObjectId>> result;
  if (road.polyline.size() < 2) {
    result.error = "road polyline must contain at least 2 points";
    return result;
  }
  if (interval <= 0.0) {
    result.error = "interval must be > 0";
    return result;
  }
  if (find_pole_type(pole_type_id) == nullptr) {
    result.error = "pole type not found";
    return result;
  }

  const std::vector<Vec3d> points = sample_polyline_points(road.polyline, interval);
  if (points.size() < 2) {
    result.error = "failed to sample road points";
    return result;
  }
  return generate_poles_from_points(road, pole_type_id, points);
}

EditResult<std::vector<ObjectId>> CoreState::generate_poles_from_points(const RoadSegment& road,
                                                                        PoleTypeId pole_type_id,
                                                                        const std::vector<Vec3d>& points) {
  EditResult<std::vector<ObjectId>> result;
  if (road.polyline.size() < 2) {
    result.error = "road polyline must contain at least 2 points";
    return result;
  }
  if (find_pole_type(pole_type_id) == nullptr) {
    result.error = "pole type not found";
    return result;
  }
  if (points.size() < 2) {
    result.error = "failed to build pole points";
    return result;
  }

  const std::uint64_t session_id = next_generation_session_id_++;
  for (std::size_t i = 0; i < points.size(); ++i) {
    Transformd tf = make_auto_pole_transform(points, i);
    EditResult<ObjectId> add_pole_result = AddPole(tf, 10.0, "AutoPole", PoleKind::kConcrete);
    if (!add_pole_result.ok) {
      result.error = add_pole_result.error;
      return result;
    }
    Pole* pole = edit_state_.poles.find(add_pole_result.value);
    if (pole != nullptr) {
      pole->context = classify_pole_context_from_path(points, i, 0);
      pole->generation.generated = true;
      pole->generation.source = GenerationSource::kRoadAuto;
      pole->generation.generation_session_id = session_id;
      pole->generation.generation_order = static_cast<std::uint32_t>(i);
      add_unique_id(add_pole_result.change_set.updated_ids, pole->id);
    }

    EditResult<ObjectId> apply_result = ApplyPoleType(add_pole_result.value, pole_type_id);
    if (!apply_result.ok) {
      result.error = apply_result.error;
      return result;
    }

    result.value.push_back(add_pole_result.value);
    append_change_set(result.change_set, add_pole_result.change_set);
    append_change_set(result.change_set, apply_result.change_set);
  }

  result.ok = true;
  return result;
}

EditResult<std::vector<ObjectId>> CoreState::GenerateSpansBetweenPoles(const std::vector<ObjectId>& poles,
                                                                       ConnectionCategory category) {
  EditResult<std::vector<ObjectId>> result;
  if (poles.size() < 2) {
    result.error = "at least 2 poles are required";
    return result;
  }

  const std::uint64_t session_id = next_generation_session_id_++;
  std::ostringstream errors;
  bool has_failure = false;
  ObjectId carry_port_on_next_left = kInvalidObjectId;

  for (std::size_t i = 0; i + 1 < poles.size(); ++i) {
    AddConnectionByPoleOptions options{};
    options.preferred_port_a_id = carry_port_on_next_left;
    options.branch_index = static_cast<std::uint32_t>(i);

    const Pole* pole_a = edit_state_.poles.find(poles[i]);
    const Pole* pole_b = edit_state_.poles.find(poles[i + 1]);
    if (pole_a != nullptr) {
      options.pole_context_a = pole_a->context.kind;
      options.corner_angle_deg_a = pole_a->context.corner_angle_deg;
      options.corner_turn_sign_a = pole_a->context.corner_turn_sign;
    }
    if (pole_b != nullptr) {
      options.pole_context_b = pole_b->context.kind;
      options.corner_angle_deg_b = pole_b->context.corner_angle_deg;
      options.corner_turn_sign_b = pole_b->context.corner_turn_sign;
    }
    const bool corner_pass = (pole_a != nullptr && pole_a->context.kind == PoleContextKind::kCorner) ||
                             (pole_b != nullptr && pole_b->context.kind == PoleContextKind::kCorner);
    options.connection_context = corner_pass ? ConnectionContext::kCornerPass : ConnectionContext::kTrunkContinue;

    EditResult<AddConnectionByPoleResult> add_result = AddConnectionByPole(poles[i], poles[i + 1], category, options);
    if (!add_result.ok) {
      has_failure = true;
      errors << "[segment " << i << "] " << add_result.error << "; ";
      carry_port_on_next_left = kInvalidObjectId;
      continue;
    }

    carry_port_on_next_left = add_result.value.port_b_id;

    Span* span = edit_state_.spans.find(add_result.value.span_id);
    if (span != nullptr) {
      span->generation.generated = true;
      span->generation.source = GenerationSource::kRoadAuto;
      span->generation.generation_session_id = session_id;
      span->generation.generation_order = static_cast<std::uint32_t>(i);
      span->generated_by_rule = true;
      add_unique_id(add_result.change_set.updated_ids, span->id);
    }

    result.value.push_back(add_result.value.span_id);
    append_change_set(result.change_set, add_result.change_set);
  }

  if (result.value.empty()) {
    result.error = has_failure ? errors.str() : "failed to generate spans";
    return result;
  }
  if (has_failure) {
    result.error = errors.str();
    return result;
  }

  result.ok = true;
  return result;
}

EditResult<CoreState::GenerateSimpleLineResult> CoreState::GenerateSimpleLine(const RoadSegment& road, double interval,
                                                                              PoleTypeId pole_type_id,
                                                                              ConnectionCategory category) {
  EditResult<GenerateSimpleLineResult> result;
  const std::uint64_t session_id = next_generation_session_id_++;

  EditResult<std::vector<ObjectId>> poles_result = GeneratePolesAlongRoad(road, interval, pole_type_id);
  if (!poles_result.ok) {
    result.error = poles_result.error;
    return result;
  }

  for (std::size_t i = 0; i < poles_result.value.size(); ++i) {
    Pole* pole = edit_state_.poles.find(poles_result.value[i]);
    if (pole != nullptr) {
      pole->generation.generated = true;
      pole->generation.source = GenerationSource::kRoadAuto;
      pole->generation.generation_session_id = session_id;
      pole->generation.generation_order = static_cast<std::uint32_t>(i);
    }
  }

  EditResult<std::vector<ObjectId>> spans_result = GenerateSpansBetweenPoles(poles_result.value, category);
  if (!spans_result.ok) {
    result.error = spans_result.error;
    return result;
  }
  for (std::size_t i = 0; i < spans_result.value.size(); ++i) {
    Span* span = edit_state_.spans.find(spans_result.value[i]);
    if (span != nullptr) {
      span->generation.generated = true;
      span->generation.source = GenerationSource::kRoadAuto;
      span->generation.generation_session_id = session_id;
      span->generation.generation_order = static_cast<std::uint32_t>(i);
      span->generated_by_rule = true;
    }
  }

  result.ok = true;
  result.value.pole_ids = poles_result.value;
  result.value.span_ids = spans_result.value;
  result.value.generation_session_id = session_id;
  append_change_set(result.change_set, poles_result.change_set);
  append_change_set(result.change_set, spans_result.change_set);
  return result;
}

EditResult<CoreState::GenerateSimpleLineResult>
CoreState::GenerateSimpleLineFromPoints(const RoadSegment& road, PoleTypeId pole_type_id, ConnectionCategory category) {
  EditResult<GenerateSimpleLineResult> result;
  const std::uint64_t session_id = next_generation_session_id_++;

  EditResult<std::vector<ObjectId>> poles_result = generate_poles_from_points(road, pole_type_id, road.polyline);
  if (!poles_result.ok) {
    result.error = poles_result.error;
    return result;
  }

  for (std::size_t i = 0; i < poles_result.value.size(); ++i) {
    Pole* pole = edit_state_.poles.find(poles_result.value[i]);
    if (pole != nullptr) {
      pole->generation.generated = true;
      pole->generation.source = GenerationSource::kRoadAuto;
      pole->generation.generation_session_id = session_id;
      pole->generation.generation_order = static_cast<std::uint32_t>(i);
    }
  }

  EditResult<std::vector<ObjectId>> spans_result = GenerateSpansBetweenPoles(poles_result.value, category);
  if (!spans_result.ok) {
    result.error = spans_result.error;
    return result;
  }
  for (std::size_t i = 0; i < spans_result.value.size(); ++i) {
    Span* span = edit_state_.spans.find(spans_result.value[i]);
    if (span != nullptr) {
      span->generation.generated = true;
      span->generation.source = GenerationSource::kRoadAuto;
      span->generation.generation_session_id = session_id;
      span->generation.generation_order = static_cast<std::uint32_t>(i);
      span->generated_by_rule = true;
    }
  }

  result.ok = true;
  result.value.pole_ids = poles_result.value;
  result.value.span_ids = spans_result.value;
  result.value.generation_session_id = session_id;
  append_change_set(result.change_set, poles_result.change_set);
  append_change_set(result.change_set, spans_result.change_set);
  return result;
}

std::uint64_t CoreState::hash_path_points(const std::vector<Vec3d>& points) {
  std::uint64_t h = 1469598103934665603ull;
  const std::uint64_t prime = 1099511628211ull;
  auto mix = [&](std::uint64_t v) {
    h ^= v;
    h *= prime;
  };
  mix(static_cast<std::uint64_t>(points.size()));
  for (const Vec3d& p : points) {
    mix(static_cast<std::uint64_t>(std::llround(p.x * 1000.0)));
    mix(static_cast<std::uint64_t>(std::llround(p.y * 1000.0)));
    mix(static_cast<std::uint64_t>(std::llround(p.z * 1000.0)));
  }
  return h;
}

int CoreState::inversion_count(const std::vector<double>& values) {
  int inv = 0;
  for (std::size_t i = 0; i < values.size(); ++i) {
    for (std::size_t j = i + 1; j < values.size(); ++j) {
      if (values[i] > values[j]) {
        ++inv;
      }
    }
  }
  return inv;
}

PathDirectionCostBreakdown CoreState::evaluate_path_direction_cost(const std::vector<Vec3d>& points,
                                                                   const ConductorGroupSpec& group_spec) const {
  PathDirectionCostBreakdown cost{};
  if (points.size() < 3) {
    return cost;
  }

  int side_flips = 0;
  int sign_changes = 0;
  int corner_compression = 0;
  int branch_conflicts = 0;
  int last_side_sign = 0;
  int last_turn_sign = 0;

  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const Vec3d& prev = points[i - 1];
    const Vec3d& curr = points[i];
    const Vec3d& next = points[i + 1];
    const double angle = compute_corner_angle_deg(prev, curr, next);
    const double turn = compute_corner_turn_sign_xy(prev, curr, next);

    int side_sign = 0;
    if (turn > 1e-9) {
      side_sign = 1;
    } else if (turn < -1e-9) {
      side_sign = -1;
    }
    if (last_side_sign != 0 && side_sign != 0 && side_sign != last_side_sign) {
      ++side_flips;
    }
    if (last_turn_sign != 0 && side_sign != 0 && side_sign != last_turn_sign) {
      ++sign_changes;
    }
    last_side_sign = (side_sign != 0) ? side_sign : last_side_sign;
    last_turn_sign = (side_sign != 0) ? side_sign : last_turn_sign;

    if (angle > layout_settings_.corner_threshold_deg && angle < 70.0) {
      ++corner_compression;
    }

    if (group_spec.conductor_count > 1 && std::abs(turn) > 1e-9 && angle > 45.0) {
      ++branch_conflicts;
    }
  }

  cost.estimated_cross_penalty = sign_changes * std::max(1, group_spec.conductor_count - 1);
  cost.side_flip_penalty = side_flips;
  cost.layer_jump_penalty = 0;
  cost.corner_compression_penalty = corner_compression;
  cost.branch_conflict_penalty = branch_conflicts;
  cost.total = cost.estimated_cross_penalty * path_direction_cost_weights_.estimated_cross_penalty +
               cost.side_flip_penalty * path_direction_cost_weights_.side_flip_penalty +
               cost.layer_jump_penalty * path_direction_cost_weights_.layer_jump_penalty +
               cost.corner_compression_penalty * path_direction_cost_weights_.corner_compression_penalty +
               cost.branch_conflict_penalty * path_direction_cost_weights_.branch_conflict_penalty;
  return cost;
}

PathDirectionChosen CoreState::choose_path_direction(const GenerateGroupedLineOptions& options,
                                                     const std::vector<Vec3d>& sampled_points,
                                                     PathDirectionEvaluationDebug* out_debug) const {
  PathDirectionEvaluationDebug debug{};
  debug.road_id = options.road.id;
  debug.requested_mode = options.direction_mode;

  if (options.direction_mode == PathDirectionMode::kForward) {
    debug.chosen = PathDirectionChosen::kForward;
    debug.reason = "forced forward";
    if (out_debug != nullptr) {
      *out_debug = debug;
    }
    return debug.chosen;
  }
  if (options.direction_mode == PathDirectionMode::kReverse) {
    debug.chosen = PathDirectionChosen::kReverse;
    debug.reason = "forced reverse";
    if (out_debug != nullptr) {
      *out_debug = debug;
    }
    return debug.chosen;
  }

  debug.forward_cost = evaluate_path_direction_cost(sampled_points, options.group_spec);
  std::vector<Vec3d> reversed = sampled_points;
  std::reverse(reversed.begin(), reversed.end());
  debug.reverse_cost = evaluate_path_direction_cost(reversed, options.group_spec);

  if (debug.forward_cost.total < debug.reverse_cost.total) {
    debug.chosen = PathDirectionChosen::kForward;
    debug.reason = "lower forward cost";
  } else if (debug.reverse_cost.total < debug.forward_cost.total) {
    debug.chosen = PathDirectionChosen::kReverse;
    debug.reason = "lower reverse cost";
  } else {
    // Deterministic tie-breaker.
    const std::uint64_t h = hash_path_points(sampled_points) ^ static_cast<std::uint64_t>(options.road.id);
    debug.chosen = ((h & 1ull) == 0ull) ? PathDirectionChosen::kForward : PathDirectionChosen::kReverse;
    debug.reason = "deterministic tie-break";
  }
  if (out_debug != nullptr) {
    *out_debug = debug;
  }
  return debug.chosen;
}

EditResult<std::vector<ObjectId>>
CoreState::generate_grouped_spans_between_poles(const std::vector<ObjectId>& poles, ObjectId bundle_id,
                                                const ConductorGroupSpec& group_spec,
                                                std::vector<SegmentLaneAssignment>* out_lane_assignments) {
  EditResult<std::vector<ObjectId>> result;
  if (poles.size() < 2) {
    result.error = "at least 2 poles are required";
    return result;
  }
  const int lane_count = std::max(1, group_spec.conductor_count);
  std::vector<ObjectId> carry_ports;

  auto ensure_ports = [&](ObjectId pole_id, ObjectId peer_id, int segment_index) -> EditResult<std::vector<ObjectId>> {
    EditResult<std::vector<ObjectId>> ports_result;
    for (const Port& port : edit_state_.ports.items()) {
      if (port.owner_pole_id == pole_id && port.category == group_spec.category) {
        ports_result.value.push_back(port.id);
      }
    }
    const Pole* pole = edit_state_.poles.find(pole_id);
    std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
      const Port* pa = edit_state_.ports.find(a);
      const Port* pb = edit_state_.ports.find(b);
      if (pa == nullptr || pb == nullptr || pole == nullptr) {
        return a < b;
      }
      const double ya = to_local_on_pole(*pole, pa->world_position).y;
      const double yb = to_local_on_pole(*pole, pb->world_position).y;
      if (std::abs(ya - yb) > 1e-9) {
        return ya < yb;
      }
      return a < b;
    });

    std::unordered_set<ObjectId> unique(ports_result.value.begin(), ports_result.value.end());
    int attempts = 0;
    while (static_cast<int>(ports_result.value.size()) < lane_count && attempts < lane_count * 16) {
      ++attempts;
      int slot_id = -1;
      SlotSelectionRequest request{};
      request.pole_id = pole_id;
      request.peer_pole_id = peer_id;
      request.category = group_spec.category;
      request.connection_context = ConnectionContext::kTrunkContinue;
      request.branch_index = static_cast<std::uint32_t>(segment_index);
      if (const Pole* p = edit_state_.poles.find(pole_id); p != nullptr) {
        request.pole_context = p->context.kind;
        request.corner_angle_deg = p->context.corner_angle_deg;
        request.corner_turn_sign = p->context.corner_turn_sign;
      }
      EditResult<ObjectId> one = ensure_pole_slot_port(request, &slot_id);
      if (!one.ok) {
        ports_result.error = one.error;
        return ports_result;
      }
      append_change_set(result.change_set, one.change_set);
      if (unique.insert(one.value).second) {
        ports_result.value.push_back(one.value);
      } else {
        // If slot allocator repeated same port, force-create a deterministic fallback port.
        const Pole* p = edit_state_.poles.find(pole_id);
        if (p != nullptr) {
          const double lane_offset = 0.2 * static_cast<double>(ports_result.value.size() + 1);
          EditResult<ObjectId> extra =
              AddPort(pole_id,
                      {p->world_transform.position.x, p->world_transform.position.y + lane_offset,
                       p->world_transform.position.z + p->height_m * 0.8},
                      category_to_port_kind(group_spec.category), category_to_port_layer(group_spec.category));
          if (extra.ok && unique.insert(extra.value).second) {
            ports_result.value.push_back(extra.value);
            append_change_set(result.change_set, extra.change_set);
          }
        }
      }
    }
    if (static_cast<int>(ports_result.value.size()) < lane_count) {
      ports_result.error = "insufficient ports for grouped generation";
      return ports_result;
    }
    if (static_cast<int>(ports_result.value.size()) > lane_count) {
      ports_result.value.resize(static_cast<std::size_t>(lane_count));
    }
    if (pole != nullptr) {
      std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
        const Port* pa = edit_state_.ports.find(a);
        const Port* pb = edit_state_.ports.find(b);
        if (pa == nullptr || pb == nullptr) {
          return a < b;
        }
        const double ya = to_local_on_pole(*pole, pa->world_position).y;
        const double yb = to_local_on_pole(*pole, pb->world_position).y;
        if (std::abs(ya - yb) > 1e-9) {
          return ya < yb;
        }
        return a < b;
      });
    }
    ports_result.ok = true;
    return ports_result;
  };

  for (std::size_t seg = 0; seg + 1 < poles.size(); ++seg) {
    const ObjectId pole_a = poles[seg];
    const ObjectId pole_b = poles[seg + 1];

    std::vector<ObjectId> lanes_a;
    if (carry_ports.empty()) {
      auto initial_ports = ensure_ports(pole_a, pole_b, static_cast<int>(seg));
      if (!initial_ports.ok) {
        result.error = initial_ports.error;
        return result;
      }
      lanes_a = initial_ports.value;
    } else {
      lanes_a = carry_ports;
    }

    auto right_ports_result = ensure_ports(pole_b, pole_a, static_cast<int>(seg));
    if (!right_ports_result.ok) {
      result.error = right_ports_result.error;
      return result;
    }
    const std::vector<ObjectId>& lanes_b_base = right_ports_result.value;

    std::vector<ObjectId> lanes_b = lanes_b_base;
    bool mirrored = false;
    if (group_spec.allow_lane_mirror && lane_count > 1) {
      std::vector<double> y_id;
      std::vector<double> y_mirror;
      y_id.reserve(static_cast<std::size_t>(lane_count));
      y_mirror.reserve(static_cast<std::size_t>(lane_count));
      const Pole* pb = edit_state_.poles.find(pole_b);
      for (int lane = 0; lane < lane_count; ++lane) {
        const Port* p_id = edit_state_.ports.find(lanes_b_base[static_cast<std::size_t>(lane)]);
        const Port* p_mr = edit_state_.ports.find(lanes_b_base[static_cast<std::size_t>(lane_count - 1 - lane)]);
        y_id.push_back((pb == nullptr || p_id == nullptr) ? 0.0 : to_local_on_pole(*pb, p_id->world_position).y);
        y_mirror.push_back((pb == nullptr || p_mr == nullptr) ? 0.0 : to_local_on_pole(*pb, p_mr->world_position).y);
      }
      const int c_id = inversion_count(y_id);
      const int c_mr = inversion_count(y_mirror);
      if (c_mr < c_id) {
        mirrored = true;
        lanes_b.assign(lanes_b_base.rbegin(), lanes_b_base.rend());
      }
    }

    SegmentLaneAssignment assignment{};
    assignment.segment_index = seg;
    assignment.pole_a_id = pole_a;
    assignment.pole_b_id = pole_b;
    assignment.bundle_id = bundle_id;
    assignment.mirrored = mirrored;

    for (int lane = 0; lane < lane_count; ++lane) {
      CoreState::AddConnectionByPoleOptions options{};
      options.bundle_id = bundle_id;
      options.auto_create_bundle = false;
      options.allow_generate_port = true;
      options.preferred_port_a_id = lanes_a[static_cast<std::size_t>(lane)];
      options.preferred_port_b_id = lanes_b[static_cast<std::size_t>(lane)];
      options.connection_context = ConnectionContext::kTrunkContinue;
      options.branch_index = static_cast<std::uint32_t>(lane);
      if (const Pole* pa = edit_state_.poles.find(pole_a); pa != nullptr) {
        options.pole_context_a = pa->context.kind;
        options.corner_angle_deg_a = pa->context.corner_angle_deg;
        options.corner_turn_sign_a = pa->context.corner_turn_sign;
      }
      if (const Pole* pb = edit_state_.poles.find(pole_b); pb != nullptr) {
        options.pole_context_b = pb->context.kind;
        options.corner_angle_deg_b = pb->context.corner_angle_deg;
        options.corner_turn_sign_b = pb->context.corner_turn_sign;
      }

      const auto add = AddConnectionByPole(pole_a, pole_b, group_spec.category, options);
      if (!add.ok) {
        result.error = add.error;
        return result;
      }
      append_change_set(result.change_set, add.change_set);
      result.value.push_back(add.value.span_id);

      const Port* pa = edit_state_.ports.find(add.value.port_a_id);
      const Port* pb = edit_state_.ports.find(add.value.port_b_id);
      assignment.port_ids_a.push_back(add.value.port_a_id);
      assignment.port_ids_b.push_back(add.value.port_b_id);
      assignment.slot_ids_a.push_back((pa == nullptr) ? -1 : pa->source_slot_id);
      assignment.slot_ids_b.push_back((pb == nullptr) ? -1 : pb->source_slot_id);

      Span* span = edit_state_.spans.find(add.value.span_id);
      if (span != nullptr) {
        span->generated_by_rule = true;
        span->generation.generated = true;
      }
    }

    carry_ports = assignment.port_ids_b;
    if (out_lane_assignments != nullptr) {
      out_lane_assignments->push_back(assignment);
    }
  }

  result.ok = !result.value.empty();
  if (!result.ok && result.error.empty()) {
    result.error = "failed to generate grouped spans";
  }
  return result;
}

EditResult<CoreState::GenerateGroupedLineResult>
CoreState::GenerateGroupedLine(const GenerateGroupedLineOptions& options) {
  EditResult<GenerateGroupedLineResult> result;
  if (options.road.polyline.size() < 2) {
    result.error = "road polyline must contain at least 2 points";
    return result;
  }
  if (find_pole_type(options.pole_type_id) == nullptr) {
    result.error = "pole type not found";
    return result;
  }
  if (options.group_spec.conductor_count <= 0) {
    result.error = "conductor_count must be > 0";
    return result;
  }

  std::vector<Vec3d> sampled_points;
  if (options.interval > 0.0) {
    sampled_points = sample_polyline_points(options.road.polyline, options.interval);
  } else {
    sampled_points = options.road.polyline;
  }
  if (sampled_points.size() < 2) {
    result.error = "failed to build path points";
    return result;
  }

  PathDirectionEvaluationDebug direction_debug{};
  const PathDirectionChosen chosen = choose_path_direction(options, sampled_points, &direction_debug);
  if (chosen == PathDirectionChosen::kReverse) {
    std::reverse(sampled_points.begin(), sampled_points.end());
  }

  RoadSegment selected_road = options.road;
  selected_road.polyline = sampled_points;
  EditResult<std::vector<ObjectId>> poles_result =
      generate_poles_from_points(selected_road, options.pole_type_id, sampled_points);
  if (!poles_result.ok) {
    result.error = poles_result.error;
    return result;
  }

  BundleKind bundle_kind = category_to_bundle_kind(options.group_spec.category);
  EditResult<ObjectId> bundle_result =
      AddBundle(options.group_spec.conductor_count, std::max(0.01, options.group_spec.lane_spacing_m), bundle_kind);
  if (!bundle_result.ok) {
    result.error = bundle_result.error;
    return result;
  }

  auto to_wire_group_kind = [](ConnectionCategory category) -> WireGroupKind {
    switch (category) {
    case ConnectionCategory::kHighVoltage:
      return WireGroupKind::kPowerHighVoltage;
    case ConnectionCategory::kLowVoltage:
    case ConnectionCategory::kDrop:
      return WireGroupKind::kPowerLowVoltage;
    case ConnectionCategory::kCommunication:
      return WireGroupKind::kComm;
    case ConnectionCategory::kOptical:
      return WireGroupKind::kOptical;
    default:
      return WireGroupKind::kUnknown;
    }
  };
  auto default_lane_role = [&](int lane_index) -> WireLaneRole {
    if (options.group_spec.group_kind == ConductorGroupKind::kThreePhase && options.group_spec.conductor_count >= 3) {
      if (lane_index == 0)
        return WireLaneRole::kPhaseA;
      if (lane_index == 1)
        return WireLaneRole::kPhaseB;
      if (lane_index == 2)
        return WireLaneRole::kPhaseC;
      return WireLaneRole::kAux;
    }
    switch (options.group_spec.category) {
    case ConnectionCategory::kCommunication:
      return WireLaneRole::kCommLine;
    case ConnectionCategory::kOptical:
      return WireLaneRole::kOpticalFiber;
    case ConnectionCategory::kDrop:
      return WireLaneRole::kNeutral;
    default:
      return WireLaneRole::kUnknown;
    }
  };

  EditResult<ObjectId> wire_group_result = AddWireGroup(to_wire_group_kind(options.group_spec.category));
  if (!wire_group_result.ok) {
    result.error = wire_group_result.error;
    return result;
  }
  const int lane_count = std::max(1, options.group_spec.conductor_count);
  std::vector<ObjectId> wire_lane_ids;
  wire_lane_ids.reserve(static_cast<std::size_t>(lane_count));
  for (int lane_index = 0; lane_index < lane_count; ++lane_index) {
    EditResult<ObjectId> lane_result = AddWireLane(wire_group_result.value, lane_index, default_lane_role(lane_index));
    if (!lane_result.ok) {
      result.error = lane_result.error;
      return result;
    }
    wire_lane_ids.push_back(lane_result.value);
    append_change_set(result.change_set, lane_result.change_set);
  }

  std::vector<SegmentLaneAssignment> lane_assignments;
  EditResult<std::vector<ObjectId>> spans_result = generate_grouped_spans_between_poles(
      poles_result.value, bundle_result.value, options.group_spec, &lane_assignments);
  if (!spans_result.ok) {
    result.error = spans_result.error;
    return result;
  }

  for (std::size_t i = 0; i < spans_result.value.size(); ++i) {
    const std::size_t lane_index = (lane_count <= 0) ? 0 : (i % static_cast<std::size_t>(lane_count));
    const ObjectId lane_id = wire_lane_ids[lane_index];
    EditResult<ObjectId> assign_result = AssignSpanToWireLane(spans_result.value[i], wire_group_result.value, lane_id);
    if (!assign_result.ok) {
      result.error = assign_result.error;
      return result;
    }
    append_change_set(result.change_set, assign_result.change_set);
  }

  const std::uint64_t session_id = next_generation_session_id_++;
  if (WireGroup* group = edit_state_.wire_groups.find(wire_group_result.value); group != nullptr) {
    group->generation_session_id = session_id;
    group->user_edited = false;
    add_unique_id(result.change_set.updated_ids, group->id);
  }
  for (std::size_t i = 0; i < poles_result.value.size(); ++i) {
    Pole* pole = edit_state_.poles.find(poles_result.value[i]);
    if (pole != nullptr) {
      pole->generation.generated = true;
      pole->generation.source = GenerationSource::kRoadAuto;
      pole->generation.generation_session_id = session_id;
      pole->generation.generation_order = static_cast<std::uint32_t>(i);
    }
  }
  for (std::size_t i = 0; i < spans_result.value.size(); ++i) {
    Span* span = edit_state_.spans.find(spans_result.value[i]);
    if (span != nullptr) {
      span->generation.generated = true;
      span->generation.source = GenerationSource::kRoadAuto;
      span->generation.generation_session_id = session_id;
      span->generation.generation_order = static_cast<std::uint32_t>(i);
      span->bundle_id = bundle_result.value;
      span->generated_by_rule = true;
    }
  }

  direction_debug.chosen = chosen;
  last_path_direction_debug_ = direction_debug;
  path_direction_debug_records_.push_back(direction_debug);
  if (path_direction_debug_records_.size() > 128) {
    path_direction_debug_records_.erase(path_direction_debug_records_.begin());
  }
  last_lane_assignments_ = lane_assignments;

  result.ok = true;
  result.value.pole_ids = poles_result.value;
  result.value.span_ids = spans_result.value;
  result.value.bundle_id = bundle_result.value;
  result.value.wire_group_id = wire_group_result.value;
  result.value.wire_lane_ids = wire_lane_ids;
  result.value.lane_assignments = lane_assignments;
  result.value.direction_debug = direction_debug;
  result.value.generation_session_id = session_id;
  append_change_set(result.change_set, poles_result.change_set);
  append_change_set(result.change_set, bundle_result.change_set);
  append_change_set(result.change_set, wire_group_result.change_set);
  append_change_set(result.change_set, spans_result.change_set);
  return result;
}

EditResult<CoreState::GenerateWireGroupFromPathResult>
CoreState::GenerateFromGuide(const GenerationRequest& request) {
  EditResult<GenerateWireGroupFromPathResult> result;
  if (request.path.polyline.size() < 2) {
    result.error = "guide path must contain at least 2 points";
    return result;
  }
  if (request.interval_m <= 0.0) {
    result.error = "interval_m must be > 0";
    return result;
  }
  if (!is_supported_category(request.category)) {
    result.error = "unsupported connection category";
    return result;
  }
  if (find_pole_type(request.pole_type_id) == nullptr) {
    result.error = "pole type not found";
    return result;
  }

  int lane_count = request.requested_lane_count;
  if (lane_count <= 0) {
    lane_count = default_lane_count_for_category(request.category);
  }
  if (lane_count <= 0) {
    result.error = "failed to resolve lane count";
    return result;
  }

  std::vector<Vec3d> guide_points = request.path.polyline;
  if (request.direction_mode == PathDirectionMode::kReverse) {
    std::reverse(guide_points.begin(), guide_points.end());
  }

  struct CandidatePole {
    Vec3d world{};
    std::size_t segment_index = 0;
    int vertex_index = -1;
    double t = 0.0;
    PlacementMode mode = PlacementMode::kAuto;
  };

  auto add_unique_candidate = [&](std::vector<CandidatePole>& candidates, const CandidatePole& candidate) {
    if (candidates.empty()) {
      candidates.push_back(candidate);
      return;
    }
    const Vec3d d = candidate.world - candidates.back().world;
    if ((d.x * d.x + d.y * d.y + d.z * d.z) > 1e-10) {
      candidates.push_back(candidate);
    }
  };

  auto outside_avoid = [&](const Vec3d& point) -> bool {
    if (request.constraints.avoid_radius_m <= 0.0 || request.constraints.avoid_points.empty()) {
      return true;
    }
    const double r2 = request.constraints.avoid_radius_m * request.constraints.avoid_radius_m;
    for (const Vec3d& avoid : request.constraints.avoid_points) {
      const Vec3d d = point - avoid;
      if ((d.x * d.x + d.y * d.y + d.z * d.z) <= r2) {
        return false;
      }
    }
    return true;
  };

  std::vector<CandidatePole> candidates{};
  candidates.reserve(guide_points.size() * 2);
  for (std::size_t i = 0; i + 1 < guide_points.size(); ++i) {
    const Vec3d a = guide_points[i];
    const Vec3d b = guide_points[i + 1];
    const Vec3d seg = b - a;
    const double seg_len = std::sqrt(seg.x * seg.x + seg.y * seg.y + seg.z * seg.z);
    if (seg_len <= kZeroLengthEps) {
      continue;
    }
    Vec3d dir = seg;
    dir.x /= seg_len;
    dir.y /= seg_len;
    dir.z /= seg_len;

    const Vec3d side_dir{-dir.y, dir.x, 0.0};
    const Vec3d lateral{
        side_dir.x * request.constraints.lateral_offset_m,
        side_dir.y * request.constraints.lateral_offset_m,
        side_dir.z * request.constraints.lateral_offset_m,
    };

    CandidatePole start{};
    start.world = a;
    start.segment_index = i;
    start.vertex_index = static_cast<int>(i);
    start.t = 0.0;
    start.mode = PlacementMode::kManual;
    add_unique_candidate(candidates, start);

    if (request.interval_m > 0.0) {
      const double step_m = std::max(0.5, std::max(request.interval_m * 0.25, request.constraints.avoid_radius_m * 0.5));
      for (double dist = request.interval_m; dist < seg_len - 1e-9; dist += request.interval_m) {
        const double t0 = std::clamp(dist / seg_len, 0.0, 1.0);
        Vec3d point{a.x + seg.x * t0 + lateral.x, a.y + seg.y * t0 + lateral.y, a.z + seg.z * t0 + lateral.z};
        bool placed = outside_avoid(point);
        if (!placed && request.constraints.avoid_radius_m > 0.0) {
          constexpr int kMaxTries = 8;
          for (int k = 1; k <= kMaxTries && !placed; ++k) {
            for (double sign : {1.0, -1.0}) {
              const double shifted_t = std::clamp((dist + sign * step_m * static_cast<double>(k)) / seg_len, 0.0, 1.0);
              if (shifted_t <= 1e-9 || shifted_t >= 1.0 - 1e-9) {
                continue;
              }
              Vec3d shifted{a.x + seg.x * shifted_t + lateral.x, a.y + seg.y * shifted_t + lateral.y,
                            a.z + seg.z * shifted_t + lateral.z};
              if (outside_avoid(shifted)) {
                point = shifted;
                placed = true;
                break;
              }
            }
          }
        }
        if (!placed) {
          continue;
        }
        CandidatePole auto_candidate{};
        auto_candidate.world = point;
        auto_candidate.segment_index = i;
        auto_candidate.vertex_index = -1;
        auto_candidate.t = t0;
        auto_candidate.mode = PlacementMode::kAuto;
        add_unique_candidate(candidates, auto_candidate);
      }
    }
  }
  CandidatePole end{};
  end.world = guide_points.back();
  end.segment_index = guide_points.size() - 2;
  end.vertex_index = static_cast<int>(guide_points.size() - 1);
  end.t = 1.0;
  end.mode = PlacementMode::kManual;
  add_unique_candidate(candidates, end);

  if (candidates.size() < 2) {
    result.error = "failed to build guide candidates";
    return result;
  }

  const CoreState snapshot = *this;
  const std::uint64_t session_id = next_generation_session_id_++;

  auto find_near_pole = [&](const Vec3d& world, PlacementMode preferred_mode) -> ObjectId {
    constexpr double kReuseRadius = 0.25;
    const double reuse_r2 = kReuseRadius * kReuseRadius;
    ObjectId best_id = kInvalidObjectId;
    double best_d2 = reuse_r2 + 1.0;
    bool best_mode_match = false;
    for (const Pole& pole : edit_state_.poles.items()) {
      const Vec3d d = pole.world_transform.position - world;
      const double d2 = d.x * d.x + d.y * d.y + d.z * d.z;
      if (d2 > reuse_r2) {
        continue;
      }
      const bool mode_match = (pole.placement_mode == preferred_mode);
      if (best_id == kInvalidObjectId || (mode_match && !best_mode_match) ||
          (mode_match == best_mode_match && d2 < best_d2)) {
        best_id = pole.id;
        best_d2 = d2;
        best_mode_match = mode_match;
      }
    }
    return best_id;
  };

  std::vector<ObjectId> ordered_pole_ids{};
  ordered_pole_ids.reserve(candidates.size());

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const CandidatePole& candidate = candidates[i];
    ObjectId pole_id = find_near_pole(candidate.world, candidate.mode);
    if (pole_id != kInvalidObjectId) {
      Pole* pole = edit_state_.poles.find(pole_id);
      if (pole != nullptr) {
        bool updated = false;
        if (candidate.mode == PlacementMode::kManual) {
          pole->placement_mode = PlacementMode::kManual;
          pole->user_edited = true;
          updated = true;
        }

        if (candidate.vertex_index >= 0) {
          pole->context = classify_pole_context_from_path(guide_points, static_cast<std::size_t>(candidate.vertex_index), 0);
          updated = true;
          // Reused poles must follow current corner-orientation rule unless explicitly overridden.
          if (!pole->orientation_override_flag && !pole->orientation_control.manual_yaw_override) {
            const Transformd auto_tf = make_auto_pole_transform(guide_points, static_cast<std::size_t>(candidate.vertex_index));
            pole->world_transform.rotation_euler_deg.z = auto_tf.rotation_euler_deg.z;
            updated = true;
          }
        } else {
          pole->context.kind = PoleContextKind::kStraight;
          if (!pole->orientation_override_flag && !pole->orientation_control.manual_yaw_override) {
            const Vec3d dir = guide_points[candidate.segment_index + 1] - guide_points[candidate.segment_index];
            if ((dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) > 1e-12) {
              pole->world_transform.rotation_euler_deg.z = normalize_yaw_deg(std::atan2(dir.y, dir.x) * (180.0 / kPi));
              updated = true;
            }
          }
        }

        if (updated) {
          add_unique_id(result.change_set.updated_ids, pole->id);
        }
      }
      ordered_pole_ids.push_back(pole_id);
      continue;
    }

    Transformd tf{};
    tf.position = candidate.world;
    if (candidate.vertex_index >= 0) {
      tf = make_auto_pole_transform(guide_points, static_cast<std::size_t>(candidate.vertex_index));
      tf.position = candidate.world;
    } else {
      const Vec3d dir = guide_points[candidate.segment_index + 1] - guide_points[candidate.segment_index];
      if ((dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) > 1e-12) {
        tf.rotation_euler_deg.z = normalize_yaw_deg(std::atan2(dir.y, dir.x) * (180.0 / kPi));
      }
    }

    EditResult<ObjectId> add_pole =
        AddPole(tf, 10.0, "GuidePole", PoleKind::kConcrete, candidate.mode);
    if (!add_pole.ok) {
      *this = snapshot;
      result.error = add_pole.error;
      return result;
    }
    Pole* pole = edit_state_.poles.find(add_pole.value);
    if (pole != nullptr) {
      if (candidate.vertex_index >= 0) {
        pole->context = classify_pole_context_from_path(guide_points, static_cast<std::size_t>(candidate.vertex_index), 0);
      } else {
        pole->context.kind = PoleContextKind::kStraight;
      }
      pole->generation.generated = true;
      pole->generation.source = GenerationSource::kRoadAuto;
      pole->generation.generation_session_id = session_id;
      pole->generation.generation_order = static_cast<std::uint32_t>(ordered_pole_ids.size());
      add_unique_id(add_pole.change_set.updated_ids, pole->id);
    }
    EditResult<ObjectId> apply_type = ApplyPoleType(add_pole.value, request.pole_type_id);
    if (!apply_type.ok) {
      *this = snapshot;
      result.error = apply_type.error;
      return result;
    }
    append_change_set(result.change_set, add_pole.change_set);
    append_change_set(result.change_set, apply_type.change_set);
    ordered_pole_ids.push_back(add_pole.value);
    result.value.generated_pole_ids.push_back(add_pole.value);
  }

  if (ordered_pole_ids.size() < 2) {
    *this = snapshot;
    result.error = "failed to create or resolve guide poles";
    return result;
  }
  {
    std::vector<ObjectId> compact_ids{};
    compact_ids.reserve(ordered_pole_ids.size());
    for (ObjectId id : ordered_pole_ids) {
      if (compact_ids.empty() || compact_ids.back() != id) {
        compact_ids.push_back(id);
      }
    }
    ordered_pole_ids.swap(compact_ids);
  }
  if (ordered_pole_ids.size() < 2) {
    *this = snapshot;
    result.error = "failed to build valid pole chain";
    return result;
  }

  auto count_existing_segment_spans = [&](ObjectId pole_a, ObjectId pole_b) -> int {
    int count = 0;
    const SpanLayer target_layer = category_to_span_layer(request.category);
    for (const Span& span : edit_state_.spans.items()) {
      if (span.layer != target_layer) {
        continue;
      }
      const Port* pa = edit_state_.ports.find(span.port_a_id);
      const Port* pb = edit_state_.ports.find(span.port_b_id);
      if (pa == nullptr || pb == nullptr) {
        continue;
      }
      const bool direct = (pa->owner_pole_id == pole_a && pb->owner_pole_id == pole_b);
      const bool reverse = (pa->owner_pole_id == pole_b && pb->owner_pole_id == pole_a);
      if (direct || reverse) {
        ++count;
      }
    }
    return count;
  };

  int missing_total = 0;
  std::size_t first_missing_segment = ordered_pole_ids.size();
  for (std::size_t i = 0; i + 1 < ordered_pole_ids.size(); ++i) {
    const int existing_count = count_existing_segment_spans(ordered_pole_ids[i], ordered_pole_ids[i + 1]);
    const int missing = std::max(0, lane_count - existing_count);
    missing_total += missing;
    if (missing > 0 && first_missing_segment == ordered_pole_ids.size()) {
      first_missing_segment = i;
    }
  }

  ObjectId bundle_id = kInvalidObjectId;
  ObjectId wire_group_id = kInvalidObjectId;
  std::vector<ObjectId> wire_lane_ids{};
  if (missing_total > 0) {
    const BundleKind bundle_kind = category_to_bundle_kind(request.category);
    const double spacing = (request.category == ConnectionCategory::kHighVoltage) ? 0.45 : 0.20;
    EditResult<ObjectId> bundle_result = AddBundle(lane_count, spacing, bundle_kind);
    if (!bundle_result.ok) {
      *this = snapshot;
      result.error = bundle_result.error;
      return result;
    }
    bundle_id = bundle_result.value;
    append_change_set(result.change_set, bundle_result.change_set);

    auto to_wire_group_kind = [](ConnectionCategory category) -> WireGroupKind {
      switch (category) {
      case ConnectionCategory::kHighVoltage:
        return WireGroupKind::kPowerHighVoltage;
      case ConnectionCategory::kLowVoltage:
      case ConnectionCategory::kDrop:
        return WireGroupKind::kPowerLowVoltage;
      case ConnectionCategory::kCommunication:
        return WireGroupKind::kComm;
      case ConnectionCategory::kOptical:
        return WireGroupKind::kOptical;
      default:
        return WireGroupKind::kUnknown;
      }
    };
    EditResult<ObjectId> wire_group_result = AddWireGroup(to_wire_group_kind(request.category));
    if (!wire_group_result.ok) {
      *this = snapshot;
      result.error = wire_group_result.error;
      return result;
    }
    wire_group_id = wire_group_result.value;
    append_change_set(result.change_set, wire_group_result.change_set);
    if (WireGroup* group = edit_state_.wire_groups.find(wire_group_id); group != nullptr) {
      group->generation_session_id = session_id;
      group->user_edited = false;
      add_unique_id(result.change_set.updated_ids, group->id);
    }

    for (int lane = 0; lane < lane_count; ++lane) {
      WireLaneRole role = WireLaneRole::kUnknown;
      if (request.category == ConnectionCategory::kHighVoltage && lane_count >= 3) {
        role = (lane == 0) ? WireLaneRole::kPhaseA : ((lane == 1) ? WireLaneRole::kPhaseB
                                                                   : ((lane == 2) ? WireLaneRole::kPhaseC
                                                                                  : WireLaneRole::kAux));
      } else if (request.category == ConnectionCategory::kCommunication) {
        role = WireLaneRole::kCommLine;
      } else if (request.category == ConnectionCategory::kOptical) {
        role = WireLaneRole::kOpticalFiber;
      }
      EditResult<ObjectId> lane_result = AddWireLane(wire_group_id, lane, role);
      if (!lane_result.ok) {
        *this = snapshot;
        result.error = lane_result.error;
        return result;
      }
      wire_lane_ids.push_back(lane_result.value);
      append_change_set(result.change_set, lane_result.change_set);
    }
  }

  if (missing_total > 0 && first_missing_segment < ordered_pole_ids.size() - 1) {
    std::vector<ObjectId> local_poles{};
    local_poles.insert(local_poles.end(), ordered_pole_ids.begin() + static_cast<std::ptrdiff_t>(first_missing_segment),
                       ordered_pole_ids.end());

    ConductorGroupSpec group_spec{};
    group_spec.category = request.category;
    group_spec.conductor_count = lane_count;
    group_spec.group_kind = (lane_count <= 1)
                                ? ConductorGroupKind::kSingle
                                : ((request.category == ConnectionCategory::kHighVoltage && lane_count == 3)
                                       ? ConductorGroupKind::kThreePhase
                                       : ConductorGroupKind::kParallel);
    group_spec.lane_spacing_m = (request.category == ConnectionCategory::kHighVoltage) ? 0.45 : 0.20;
    group_spec.maintain_lane_order = true;
    group_spec.allow_lane_mirror = true;

    std::vector<SegmentLaneAssignment> lane_assignments{};
    EditResult<std::vector<ObjectId>> spans_result =
        generate_grouped_spans_between_poles(local_poles, bundle_id, group_spec, &lane_assignments);
    if (!spans_result.ok) {
      *this = snapshot;
      result.error = spans_result.error;
      return result;
    }
    append_change_set(result.change_set, spans_result.change_set);

    for (std::size_t i = 0; i < spans_result.value.size(); ++i) {
      const ObjectId span_id = spans_result.value[i];
      if (wire_group_id != kInvalidObjectId && !wire_lane_ids.empty()) {
        const std::size_t lane_index = i % wire_lane_ids.size();
        EditResult<ObjectId> assign_result = AssignSpanToWireLane(span_id, wire_group_id, wire_lane_ids[lane_index]);
        if (!assign_result.ok) {
          *this = snapshot;
          result.error = assign_result.error;
          return result;
        }
        append_change_set(result.change_set, assign_result.change_set);
      }
      Span* span = edit_state_.spans.find(span_id);
      if (span != nullptr) {
        span->generation.generated = true;
        span->generation.source = GenerationSource::kRoadAuto;
        span->generation.generation_session_id = session_id;
        span->generation.generation_order = static_cast<std::uint32_t>(result.value.generated_span_ids.size());
        span->generated_by_rule = true;
        add_unique_id(result.change_set.updated_ids, span->id);
      }
      result.value.generated_span_ids.push_back(span_id);
    }
  }

  if (wire_group_id != kInvalidObjectId) {
    result.value.wire_group_id = wire_group_id;
    result.value.wire_lane_ids = wire_lane_ids;
  }

  result.ok = true;
  return result;
}

EditResult<CoreState::GenerateWireGroupFromPathResult>
CoreState::GenerateWireGroupFromPath(const GenerateWireGroupFromPathInput& input) {
  GenerationRequest request{};
  request.path.polyline = input.polyline;
  request.interval_m = input.interval_m;
  request.pole_type_id = input.pole_type_id;
  request.category = input.category;
  request.direction_mode = input.direction_mode;
  request.requested_lane_count = input.requested_lane_count;
  return GenerateFromGuide(request);
}

} // namespace wire::core
