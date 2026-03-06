#include "wire/core/core_state.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

namespace wire::core {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kZeroLengthEps = 1e-9;
constexpr double kSharpCornerInteriorAngleMaxDeg = 75.0;

struct SharpCornerOrientationDebug {
  bool applied = false;
  double theta_deg = 0.0;
  Vec3d bisector_dir{};
  Vec3d side_dir{};
};

struct AutoPoleTransformResult {
  Transformd transform{};
  SharpCornerOrientationDebug sharp{};
};

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

bool normalize_xy(Vec3d* v) {
  if (v == nullptr) {
    return false;
  }
  const double len = std::sqrt(v->x * v->x + v->y * v->y);
  if (len <= 1e-9) {
    return false;
  }
  v->x /= len;
  v->y /= len;
  v->z = 0.0;
  return true;
}

double dot_xy(const Vec3d& a, const Vec3d& b) { return a.x * b.x + a.y * b.y; }

Vec3d left_perp_xy(const Vec3d& v) { return Vec3d{-v.y, v.x, 0.0}; }

SharpCornerOrientationDebug compute_sharp_corner_orientation(const std::vector<Vec3d>& points, std::size_t index,
                                                             double sharp_threshold_deg,
                                                             const Vec3d* preferred_side_dir_xy) {
  SharpCornerOrientationDebug out{};
  if (points.size() < 3 || index == 0 || index + 1 >= points.size()) {
    return out;
  }

  // Corner-interior definition:
  // u0 = normalize(p[i-1]-p[i]) (corner -> prev), u1 = normalize(p[i+1]-p[i]) (corner -> next)
  // theta = acos(dot(u0,u1)), b = normalize(u0+u1)
  Vec3d u0{
      points[index - 1].x - points[index].x,
      points[index - 1].y - points[index].y,
      0.0,
  };
  Vec3d u1{
      points[index + 1].x - points[index].x,
      points[index + 1].y - points[index].y,
      0.0,
  };
  if (!normalize_xy(&u0) || !normalize_xy(&u1)) {
    return out;
  }

  double d = dot_xy(u0, u1);
  d = std::clamp(d, -1.0, 1.0);
  out.theta_deg = std::acos(d) * (180.0 / kPi);
  if (out.theta_deg <= 1e-6 || out.theta_deg > sharp_threshold_deg + 1e-6) {
    return out;
  }

  Vec3d bisector{
      u0.x + u1.x,
      u0.y + u1.y,
      0.0,
  };
  if (!normalize_xy(&bisector)) {
    // v0 ~= -v1 (degenerate): fallback to default tangent orientation.
    return out;
  }
  out.bisector_dir = bisector;

  // side candidate = cross(up, b) in XY.
  Vec3d side1 = left_perp_xy(bisector);
  if (!normalize_xy(&side1)) {
    return out;
  }
  Vec3d side2{-side1.x, -side1.y, 0.0};
  Vec3d selected = side1;
  Vec3d inward{};
  bool has_inward = false;

  // Turn sign from travel directions (prev->curr, curr->next).
  Vec3d t_in{
      points[index].x - points[index - 1].x,
      points[index].y - points[index - 1].y,
      0.0,
  };
  Vec3d t_out{
      points[index + 1].x - points[index].x,
      points[index + 1].y - points[index].y,
      0.0,
  };
  if (!normalize_xy(&t_in) || !normalize_xy(&t_out)) {
    return out;
  }
  const double turn = t_in.x * t_out.y - t_in.y * t_out.x;
  if (std::abs(turn) > 1e-9) {
    // Inward direction from incoming travel side: left for left-turn, right for right-turn.
    inward = (turn > 0.0) ? left_perp_xy(t_in) : Vec3d{t_in.y, -t_in.x, 0.0};
    if (normalize_xy(&inward)) {
      has_inward = true;
      const double inward_proj_1 = dot_xy(side1, inward);
      const double inward_proj_2 = dot_xy(side2, inward);
      // Pick side that points less toward the inner corner.
      selected = (inward_proj_1 <= inward_proj_2) ? side1 : side2;
    }
  }

  // Resolve sharp-corner left/right sign with local continuity to suppress zigzag flip/twist.
  if (preferred_side_dir_xy != nullptr) {
    Vec3d preferred = *preferred_side_dir_xy;
    if (normalize_xy(&preferred)) {
      auto side_score = [&](const Vec3d& side) -> double {
        const double inward_penalty = has_inward ? std::max(0.0, dot_xy(side, inward)) : 0.0;
        const double continuity_penalty = 0.5 * (1.0 - dot_xy(side, preferred));
        return inward_penalty * 4.0 + continuity_penalty;
      };
      const Vec3d alternate{-selected.x, -selected.y, 0.0};
      if (side_score(alternate) + 1e-9 < side_score(selected)) {
        selected = alternate;
      }
    }
  }

  out.side_dir = selected;
  out.applied = true;
  return out;
}

void apply_sharp_debug_to_context(PoleContextInfo* context, const SharpCornerOrientationDebug& sharp) {
  if (context == nullptr) {
    return;
  }
  context->sharp_orientation_applied = sharp.applied;
  context->sharp_theta_deg = sharp.theta_deg;
  context->sharp_bisector_dir = sharp.bisector_dir;
  context->sharp_side_dir = sharp.side_dir;
}

AutoPoleTransformResult make_auto_pole_transform(const std::vector<Vec3d>& points, std::size_t index,
                                                 const Vec3d* preferred_side_dir_xy = nullptr) {
  AutoPoleTransformResult out{};
  out.transform.position = points[index];

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
    out.sharp =
        compute_sharp_corner_orientation(points, index, kSharpCornerInteriorAngleMaxDeg, preferred_side_dir_xy);
    if (out.sharp.applied) {
      // Align local Y (slot side axis) to selected side_dir.
      yaw_deg = std::atan2(out.sharp.side_dir.y, out.sharp.side_dir.x) * (180.0 / kPi) - 90.0;
    }
    out.transform.rotation_euler_deg.z = normalize_yaw_deg(yaw_deg);
  }
  return out;
}

Vec3d side_axis_from_yaw_deg(double yaw_deg) {
  const double rad = (yaw_deg + 90.0) * (kPi / 180.0);
  return {std::cos(rad), std::sin(rad), 0.0};
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

  const std::uint64_t session_id = next_generation_session_id_access()++;
  Vec3d preferred_side_dir{0.0, 0.0, 0.0};
  bool has_preferred_side_dir = false;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const AutoPoleTransformResult auto_tf =
        make_auto_pole_transform(points, i, has_preferred_side_dir ? &preferred_side_dir : nullptr);
    preferred_side_dir = side_axis_from_yaw_deg(auto_tf.transform.rotation_euler_deg.z);
    has_preferred_side_dir = true;
    EditResult<ObjectId> add_pole_result = AddPole(auto_tf.transform, 10.0, "AutoPole", PoleKind::kConcrete);
    if (!add_pole_result.ok) {
      result.error = add_pole_result.error;
      return result;
    }
    Pole* pole = edit_state_access().poles.find(add_pole_result.value);
    if (pole != nullptr) {
      pole->context = classify_pole_context_from_path(points, i, 0);
      apply_sharp_debug_to_context(&pole->context, auto_tf.sharp);
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

  const std::uint64_t session_id = next_generation_session_id_access()++;
  std::ostringstream errors;
  bool has_failure = false;
  ObjectId carry_port_on_next_left = kInvalidObjectId;

  for (std::size_t i = 0; i + 1 < poles.size(); ++i) {
    AddConnectionByPoleOptions options{};
    options.bundle_template_id = category_to_bundle_kind(category);
    options.use_bundle_template = true;
    options.preferred_port_a_id = carry_port_on_next_left;
    options.branch_index = static_cast<std::uint32_t>(i);

    const Pole* pole_a = edit_state_access().poles.find(poles[i]);
    const Pole* pole_b = edit_state_access().poles.find(poles[i + 1]);
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

    Span* span = edit_state_access().spans.find(add_result.value.span_id);
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
  const std::uint64_t session_id = next_generation_session_id_access()++;

  EditResult<std::vector<ObjectId>> poles_result = GeneratePolesAlongRoad(road, interval, pole_type_id);
  if (!poles_result.ok) {
    result.error = poles_result.error;
    return result;
  }

  for (std::size_t i = 0; i < poles_result.value.size(); ++i) {
    Pole* pole = edit_state_access().poles.find(poles_result.value[i]);
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
    Span* span = edit_state_access().spans.find(spans_result.value[i]);
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
  const std::uint64_t session_id = next_generation_session_id_access()++;

  EditResult<std::vector<ObjectId>> poles_result = generate_poles_from_points(road, pole_type_id, road.polyline);
  if (!poles_result.ok) {
    result.error = poles_result.error;
    return result;
  }

  for (std::size_t i = 0; i < poles_result.value.size(); ++i) {
    Pole* pole = edit_state_access().poles.find(poles_result.value[i]);
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
    Span* span = edit_state_access().spans.find(spans_result.value[i]);
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

  if (options.group_spec.maintain_lane_order) {
    // Keep DrawPath input order deterministic when lane continuity is requested.
    debug.chosen = PathDirectionChosen::kForward;
    debug.reason = "maintain_lane_order keeps forward path order";
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
                                                std::vector<SegmentLaneAssignment>* out_lane_assignments,
                                                std::vector<BackboneEdgeOrientation>* out_edge_orientations,
                                                BundleKind bundle_template_id) {
  EditResult<std::vector<ObjectId>> result;
  if (poles.size() < 2) {
    result.error = "at least 2 poles are required";
    return result;
  }
  const int lane_count = std::max(1, group_spec.conductor_count);
  const PortLayer target_port_layer = category_to_port_layer(group_spec.category);
  std::unordered_map<ObjectId, std::vector<ObjectId>> pole_lane_ports_cache{};
  std::unordered_map<ObjectId, Vec3d> pole_side_axis_hints{};
  std::vector<Vec3d> side_axis_by_index(poles.size(), Vec3d{0.0, 1.0, 0.0});
  for (std::size_t i = 0; i < poles.size(); ++i) {
    const Pole* pole = edit_state_access().poles.find(poles[i]);
    if (pole == nullptr) {
      continue;
    }
    Vec3d tangent{1.0, 0.0, 0.0};
    if (i == 0 && i + 1 < poles.size()) {
      const Pole* next = edit_state_access().poles.find(poles[i + 1]);
      if (next != nullptr) {
        tangent = next->world_transform.position - pole->world_transform.position;
      }
    } else if (i + 1 == poles.size() && i > 0) {
      const Pole* prev = edit_state_access().poles.find(poles[i - 1]);
      if (prev != nullptr) {
        tangent = pole->world_transform.position - prev->world_transform.position;
      }
    } else if (i > 0 && i + 1 < poles.size()) {
      const Pole* prev = edit_state_access().poles.find(poles[i - 1]);
      const Pole* next = edit_state_access().poles.find(poles[i + 1]);
      if (prev != nullptr && next != nullptr) {
        const Vec3d in_dir = pole->world_transform.position - prev->world_transform.position;
        const Vec3d out_dir = next->world_transform.position - pole->world_transform.position;
        tangent = in_dir + out_dir;
        if (!normalize_xy(&tangent)) {
          tangent = out_dir;
        }
      }
    }
    if (!normalize_xy(&tangent)) {
      tangent = {1.0, 0.0, 0.0};
    }
    side_axis_by_index[i] = Vec3d{-tangent.y, tangent.x, 0.0};
  }
  for (std::size_t i = 1; i < side_axis_by_index.size(); ++i) {
    if (dot_xy(side_axis_by_index[i - 1], side_axis_by_index[i]) < 0.0) {
      side_axis_by_index[i].x = -side_axis_by_index[i].x;
      side_axis_by_index[i].y = -side_axis_by_index[i].y;
    }
  }
  for (std::size_t i = 0; i < poles.size(); ++i) {
    const Pole* pole = edit_state_access().poles.find(poles[i]);
    if (pole == nullptr) {
      continue;
    }
    pole_side_axis_hints[pole->id] = side_axis_by_index[i];
  }
  auto ensure_ports = [&](ObjectId pole_id, ObjectId peer_id, int segment_index, bool prefer_existing_neighbor_order,
                          bool* out_seeded_from_previous = nullptr) -> EditResult<std::vector<ObjectId>> {
    EditResult<std::vector<ObjectId>> ports_result;
    if (out_seeded_from_previous != nullptr) {
      *out_seeded_from_previous = false;
    }
    if (const auto it_cached = pole_lane_ports_cache.find(pole_id); it_cached != pole_lane_ports_cache.end()) {
      if (static_cast<int>(it_cached->second.size()) == lane_count) {
        ports_result.value = it_cached->second;
        ports_result.ok = true;
        return ports_result;
      }
    }
    for (const Port& port : edit_state_access().ports.items()) {
      if (port.owner_pole_id == pole_id && port.layer == target_port_layer) {
        ports_result.value.push_back(port.id);
      }
    }
    const Pole* pole = edit_state_access().poles.find(pole_id);

    auto port_links_to_neighbor = [&](ObjectId port_id, ObjectId neighbor_pole_id) -> int {
      int count = 0;
      const auto it = connection_index_access().spans_by_port.find(port_id);
      if (it == connection_index_access().spans_by_port.end()) {
        return 0;
      }
      for (ObjectId span_id : it->second) {
        const Span* span = edit_state_access().spans.find(span_id);
        if (span == nullptr) {
          continue;
        }
        const ObjectId other_port_id = (span->port_a_id == port_id) ? span->port_b_id : span->port_a_id;
        const Port* other_port = edit_state_access().ports.find(other_port_id);
        if (other_port == nullptr || other_port->layer != target_port_layer) {
          continue;
        }
        if (other_port->owner_pole_id == neighbor_pole_id) {
          ++count;
        }
      }
      return count;
    };

    ObjectId continuity_neighbor_id = kInvalidObjectId;
    if (prefer_existing_neighbor_order) {
      std::unordered_map<ObjectId, int> neighbor_counts{};
      for (ObjectId port_id : ports_result.value) {
        const auto it = connection_index_access().spans_by_port.find(port_id);
        if (it == connection_index_access().spans_by_port.end()) {
          continue;
        }
        for (ObjectId span_id : it->second) {
          const Span* span = edit_state_access().spans.find(span_id);
          if (span == nullptr) {
            continue;
          }
          const ObjectId other_port_id = (span->port_a_id == port_id) ? span->port_b_id : span->port_a_id;
          const Port* other_port = edit_state_access().ports.find(other_port_id);
          if (other_port == nullptr || other_port->layer != target_port_layer) {
            continue;
          }
          const ObjectId other_pole_id = other_port->owner_pole_id;
          if (other_pole_id == kInvalidObjectId || other_pole_id == pole_id || other_pole_id == peer_id) {
            continue;
          }
          neighbor_counts[other_pole_id] += 1;
        }
      }
      int best_count = 0;
      for (const auto& [neighbor_id, count] : neighbor_counts) {
        if (count > best_count || (count == best_count && count > 0 && neighbor_id < continuity_neighbor_id)) {
          best_count = count;
          continuity_neighbor_id = neighbor_id;
        }
      }
    }

    std::unordered_set<ObjectId> unique(ports_result.value.begin(), ports_result.value.end());
    int attempts = 0;
    int fallback_created = 0;
    while (static_cast<int>(ports_result.value.size()) < lane_count && attempts < lane_count * 16) {
      ++attempts;
      int slot_id = -1;
      SlotSelectionRequest request{};
      request.pole_id = pole_id;
      request.peer_pole_id = peer_id;
      request.category = group_spec.category;
      request.connection_context = ConnectionContext::kTrunkContinue;
      request.branch_index = static_cast<std::uint32_t>(segment_index);
      if (const Pole* p = edit_state_access().poles.find(pole_id); p != nullptr) {
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
        const Pole* p = edit_state_access().poles.find(pole_id);
        if (p != nullptr) {
          const int fallback_index = fallback_created++;
          const double lane_sign = (fallback_index % 2 == 0) ? 1.0 : -1.0;
          const int fallback_ring = (fallback_index / 2) + 1;
          const double fallback_spacing = std::max(0.1, group_spec.lane_spacing_m);
          const double lane_offset = fallback_spacing * static_cast<double>(fallback_ring);
          Vec3d lateral_axis{0.0, 1.0, 0.0};
          if (const Pole* peer = edit_state_access().poles.find(peer_id); peer != nullptr) {
            Vec3d dir_xy = peer->world_transform.position - p->world_transform.position;
            if (normalize_xy(&dir_xy) && std::isfinite(dir_xy.x) && std::isfinite(dir_xy.y)) {
              lateral_axis = Vec3d{-dir_xy.y, dir_xy.x, 0.0};
            }
          }
          const Vec3d lane_delta{lateral_axis.x * lane_sign * lane_offset, lateral_axis.y * lane_sign * lane_offset,
                                 0.0};
          const Vec3d world = p->world_transform.position + lane_delta;
          EditResult<ObjectId> extra =
              AddPort(pole_id, {world.x, world.y, p->world_transform.position.z + p->height_m * 0.8},
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
    if (prefer_existing_neighbor_order && continuity_neighbor_id != kInvalidObjectId) {
      // Preserve boundary lane order on extension by seeding from prior assignment at continuity neighbor.
      std::vector<ObjectId> seeded_order{};
      for (const SegmentLaneAssignment& assignment : last_lane_assignments_access()) {
        const Bundle* assignment_bundle = edit_state_access().bundles.find(assignment.bundle_id);
        if (assignment_bundle == nullptr || assignment_bundle->kind != bundle_template_id) {
          continue;
        }
        const std::vector<ObjectId>* candidate_order = nullptr;
        if (assignment.pole_a_id == continuity_neighbor_id && assignment.pole_b_id == pole_id) {
          candidate_order = &assignment.port_ids_b;
        } else if (assignment.pole_a_id == pole_id && assignment.pole_b_id == continuity_neighbor_id) {
          candidate_order = &assignment.port_ids_a;
        }
        if (candidate_order == nullptr || static_cast<int>(candidate_order->size()) < lane_count) {
          continue;
        }
        bool valid = true;
        for (int lane = 0; lane < lane_count; ++lane) {
          const ObjectId port_id = (*candidate_order)[static_cast<std::size_t>(lane)];
          const Port* port = edit_state_access().ports.find(port_id);
          if (port == nullptr || port->owner_pole_id != pole_id || port->layer != target_port_layer) {
            valid = false;
            break;
          }
        }
        if (!valid) {
          continue;
        }
        seeded_order.assign(candidate_order->begin(), candidate_order->begin() + static_cast<std::ptrdiff_t>(lane_count));
      }
      if (static_cast<int>(seeded_order.size()) == lane_count) {
        ports_result.value = std::move(seeded_order);
        pole_lane_ports_cache[pole_id] = ports_result.value;
        if (out_seeded_from_previous != nullptr) {
          *out_seeded_from_previous = true;
        }
        ports_result.ok = true;
        return ports_result;
      }
    }

    if (group_spec.maintain_lane_order && pole != nullptr) {
      const double spacing = std::max(0.1, group_spec.lane_spacing_m);
      const double center = (static_cast<double>(lane_count) - 1.0) * 0.5;
      std::vector<double> target_local_y(static_cast<std::size_t>(lane_count), 0.0);
      for (int lane = 0; lane < lane_count; ++lane) {
        target_local_y[static_cast<std::size_t>(lane)] = (static_cast<double>(lane) - center) * spacing;
      }

      Vec3d stable_side_axis{0.0, 1.0, 0.0};
      Vec3d yaw_side_axis = side_axis_from_yaw_deg(pole->world_transform.rotation_euler_deg.z);
      if (std::isfinite(yaw_side_axis.x) && std::isfinite(yaw_side_axis.y)) {
        stable_side_axis = yaw_side_axis;
      } else if (const auto it_axis = pole_side_axis_hints.find(pole_id); it_axis != pole_side_axis_hints.end()) {
        stable_side_axis = it_axis->second;
      } else if (const Pole* peer = edit_state_access().poles.find(peer_id); peer != nullptr) {
        Vec3d dir_xy = peer->world_transform.position - pole->world_transform.position;
        if (normalize_xy(&dir_xy) && std::isfinite(dir_xy.x) && std::isfinite(dir_xy.y)) {
          stable_side_axis = Vec3d{-dir_xy.y, dir_xy.x, 0.0};
        }
      }

      std::vector<ObjectId> ordered_ports(static_cast<std::size_t>(lane_count), kInvalidObjectId);
      std::unordered_set<ObjectId> used{};
      for (int lane = 0; lane < lane_count; ++lane) {
        const double target_y = target_local_y[static_cast<std::size_t>(lane)];
        ObjectId best_id = kInvalidObjectId;
        double best_dist = std::numeric_limits<double>::max();
        for (ObjectId candidate_id : ports_result.value) {
          if (used.find(candidate_id) != used.end()) {
            continue;
          }
          const Port* candidate = edit_state_access().ports.find(candidate_id);
          if (candidate == nullptr || candidate->owner_pole_id != pole_id || candidate->layer != target_port_layer) {
            continue;
          }
          const double local_y = dot_xy(candidate->world_position - pole->world_transform.position, stable_side_axis);
          const double dist = std::abs(local_y - target_y);
          if (dist < best_dist) {
            best_dist = dist;
            best_id = candidate_id;
          }
        }

        const double kTargetMatchTolerance = std::max(0.02, spacing * 0.1);
        if (best_id == kInvalidObjectId || best_dist > kTargetMatchTolerance) {
          const Vec3d tangent_axis{stable_side_axis.y, -stable_side_axis.x, 0.0};
          const double tangent_offset = target_y * 0.35;
          const Vec3d world =
              pole->world_transform.position +
              Vec3d{stable_side_axis.x * target_y + tangent_axis.x * tangent_offset,
                    stable_side_axis.y * target_y + tangent_axis.y * tangent_offset, 0.0};
          EditResult<ObjectId> extra =
              AddPort(pole_id, {world.x, world.y, pole->world_transform.position.z + pole->height_m * 0.8},
                      category_to_port_kind(group_spec.category), category_to_port_layer(group_spec.category));
          if (!extra.ok) {
            ports_result.error = extra.error;
            return ports_result;
          }
          append_change_set(result.change_set, extra.change_set);
          best_id = extra.value;
        }

        used.insert(best_id);
        ordered_ports[static_cast<std::size_t>(lane)] = best_id;
      }

      ports_result.value = std::move(ordered_ports);
      pole_lane_ports_cache[pole_id] = ports_result.value;
      ports_result.ok = true;
      return ports_result;
    }

    auto side_rank = [](SlotSide side) -> int {
      switch (side) {
      case SlotSide::kLeft:
        return 0;
      case SlotSide::kCenter:
        return 1;
      case SlotSide::kRight:
        return 2;
      default:
        return 3;
      }
    };
    auto local_y_of = [&](const Port* p) -> double {
      if (p == nullptr || pole == nullptr) {
        return 0.0;
      }
      return to_local_on_pole(*pole, p->world_position).y;
    };
    auto order_key = [&](const Port* p) -> std::tuple<double, int, int, int, int, ObjectId> {
      if (p == nullptr) {
        return {0.0, 1, 999, 999999, 999999, kInvalidObjectId};
      }
      const bool has_template_slot = p->source_slot_id >= 0;
      return {local_y_of(p), has_template_slot ? 0 : 1, p->template_layer, side_rank(p->template_side),
              has_template_slot ? p->source_slot_id : 999999, p->id};
    };
    std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
      const Port* pa = edit_state_access().ports.find(a);
      const Port* pb = edit_state_access().ports.find(b);
      return order_key(pa) < order_key(pb);
    });

    if (static_cast<int>(ports_result.value.size()) > lane_count) {
      if (continuity_neighbor_id != kInvalidObjectId) {
        std::stable_sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
          const int score_a = port_links_to_neighbor(a, continuity_neighbor_id);
          const int score_b = port_links_to_neighbor(b, continuity_neighbor_id);
          if (score_a != score_b) {
            return score_a > score_b;
          }
          const Port* pa = edit_state_access().ports.find(a);
          const Port* pb = edit_state_access().ports.find(b);
          return order_key(pa) < order_key(pb);
        });
        ports_result.value.resize(static_cast<std::size_t>(lane_count));
        std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
          const Port* pa = edit_state_access().ports.find(a);
          const Port* pb = edit_state_access().ports.find(b);
          return order_key(pa) < order_key(pb);
        });
      } else {
        const std::size_t target = static_cast<std::size_t>(lane_count);
        const std::size_t total = ports_result.value.size();
        std::size_t best_start = 0;
        double best_abs_mean = std::numeric_limits<double>::max();
        double best_span = std::numeric_limits<double>::max();
        for (std::size_t start = 0; start + target <= total; ++start) {
          double sum = 0.0;
          double y_min = std::numeric_limits<double>::max();
          double y_max = -std::numeric_limits<double>::max();
          for (std::size_t i = start; i < start + target; ++i) {
            const Port* p = edit_state_access().ports.find(ports_result.value[i]);
            const double y = local_y_of(p);
            sum += y;
            y_min = std::min(y_min, y);
            y_max = std::max(y_max, y);
          }
          const double abs_mean = std::abs(sum / static_cast<double>(target));
          const double span = y_max - y_min;
          if (abs_mean + 1e-9 < best_abs_mean || (std::abs(abs_mean - best_abs_mean) <= 1e-9 && span < best_span)) {
            best_abs_mean = abs_mean;
            best_span = span;
            best_start = start;
          }
        }
        std::vector<ObjectId> centered{};
        centered.reserve(target);
        for (std::size_t i = best_start; i < best_start + target; ++i) {
          centered.push_back(ports_result.value[i]);
        }
        ports_result.value.swap(centered);
      }
    }
    (void)peer_id;
    if (static_cast<int>(ports_result.value.size()) > lane_count) {
      ports_result.value.resize(static_cast<std::size_t>(lane_count));
    }
    pole_lane_ports_cache[pole_id] = ports_result.value;
    ports_result.ok = true;
    return ports_result;
  };

  EditResult<std::vector<ObjectId>> first_ports = ensure_ports(poles.front(), poles[1], 0, true, nullptr);
  if (!first_ports.ok) {
    result.error = first_ports.error;
    return result;
  }
  if (static_cast<int>(first_ports.value.size()) != lane_count) {
    result.error = "failed to seed first segment lanes";
    return result;
  }
  const std::vector<ObjectId> first_lanes_a = first_ports.value;

  struct MirrorScore {
    int cross_y = 0;
    int cross_z = 0;
    int layer_jump = 0;
    double span_z_delta = 0.0;
  };
  auto secondary_score_less = [&](const MirrorScore& a, const MirrorScore& b) {
    const auto key_a = std::make_tuple(a.cross_z, a.layer_jump, static_cast<long long>(std::llround(a.span_z_delta * 1000.0)));
    const auto key_b = std::make_tuple(b.cross_z, b.layer_jump, static_cast<long long>(std::llround(b.span_z_delta * 1000.0)));
    return key_a < key_b;
  };

  auto evaluate_increment = [&](ObjectId pole_a, ObjectId pole_b, const std::vector<ObjectId>& lanes_a,
                                const std::vector<ObjectId>& lanes_b) -> MirrorScore {
    MirrorScore score{};
    const Pole* pa = edit_state_access().poles.find(pole_a);
    const Pole* pb = edit_state_access().poles.find(pole_b);

    Vec3d segment_dir{1.0, 0.0, 0.0};
    if (pa != nullptr && pb != nullptr) {
      segment_dir = pb->world_transform.position - pa->world_transform.position;
      if (!normalize_xy(&segment_dir) || !std::isfinite(segment_dir.x) || !std::isfinite(segment_dir.y)) {
        segment_dir = {1.0, 0.0, 0.0};
      }
    }
    const Vec3d lateral_axis{-segment_dir.y, segment_dir.x, 0.0};
    auto axis_for_pole = [&](ObjectId pole_id) -> Vec3d {
      if (const Pole* pole = edit_state_access().poles.find(pole_id); pole != nullptr) {
        Vec3d yaw_side = side_axis_from_yaw_deg(pole->world_transform.rotation_euler_deg.z);
        if (std::isfinite(yaw_side.x) && std::isfinite(yaw_side.y)) {
          return yaw_side;
        }
      }
      const auto it = pole_side_axis_hints.find(pole_id);
      if (it != pole_side_axis_hints.end() && std::isfinite(it->second.x) && std::isfinite(it->second.y)) {
        return it->second;
      }
      return lateral_axis;
    };
    const Vec3d axis_a = axis_for_pole(pole_a);
    const Vec3d axis_b = axis_for_pole(pole_b);

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
      const Vec3d da = port_a->world_position - ((pa == nullptr) ? Vec3d{} : pa->world_transform.position);
      const Vec3d db = port_b->world_position - ((pb == nullptr) ? Vec3d{} : pb->world_transform.position);
      y_a[idx] = dot_xy(da, axis_a);
      y_b[idx] = dot_xy(db, axis_b);
      z_a[idx] = port_a->world_position.z;
      z_b[idx] = port_b->world_position.z;
      score.layer_jump += std::abs(port_a->template_layer - port_b->template_layer);
      score.span_z_delta += std::abs(port_a->world_position.z - port_b->world_position.z);
    }

    for (int i = 0; i < lane_count; ++i) {
      for (int j = i + 1; j < lane_count; ++j) {
        const std::size_t ii = static_cast<std::size_t>(i);
        const std::size_t jj = static_cast<std::size_t>(j);
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

  const bool allow_mirror = group_spec.allow_lane_mirror && lane_count > 1;
  constexpr double kAngleEps = 1e-6;

  const std::size_t segment_count = poles.size() - 1;
  std::vector<std::vector<ObjectId>> prepared_ports_b(segment_count);
  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    const ObjectId pole_a = poles[seg];
    const ObjectId pole_b = poles[seg + 1];
    const ObjectId right_order_peer = pole_a;
    EditResult<std::vector<ObjectId>> right_ports =
        ensure_ports(pole_b, right_order_peer, static_cast<int>(seg), false, nullptr);
    if (!right_ports.ok) {
      result.error = right_ports.error;
      return result;
    }
    if (static_cast<int>(right_ports.value.size()) != lane_count) {
      result.error = "failed to prepare right-side lane candidates";
      return result;
    }
    prepared_ports_b[seg] = std::move(right_ports.value);
  }

  struct MirrorResolution {
    std::vector<bool> mirrors{};
    MirrorScore total{};
  };
  auto run_resolution = [&](const std::vector<ObjectId>& first_seed) -> MirrorResolution {
    MirrorResolution out{};
    out.mirrors.assign(segment_count, false);
    std::vector<ObjectId> carry = first_seed;
    bool previous_mirror = false;
    bool has_previous_mirror = false;
    auto best_next_cross_y = [&](std::size_t next_seg, const std::vector<ObjectId>& carry_now) -> int {
      if (next_seg >= segment_count) {
        return 0;
      }
      const MirrorScore n_no = evaluate_increment(poles[next_seg], poles[next_seg + 1], carry_now, prepared_ports_b[next_seg]);
      int best = n_no.cross_y;
      if (allow_mirror) {
        std::vector<ObjectId> n_mirrored = prepared_ports_b[next_seg];
        std::reverse(n_mirrored.begin(), n_mirrored.end());
        const MirrorScore n_yes = evaluate_increment(poles[next_seg], poles[next_seg + 1], carry_now, n_mirrored);
        best = std::min(best, n_yes.cross_y);
      }
      return best;
    };
    for (std::size_t seg = 0; seg < segment_count; ++seg) {
      const MirrorScore score_no = evaluate_increment(poles[seg], poles[seg + 1], carry, prepared_ports_b[seg]);
      std::vector<ObjectId> mirrored = prepared_ports_b[seg];
      std::reverse(mirrored.begin(), mirrored.end());
      const MirrorScore score_yes = evaluate_increment(poles[seg], poles[seg + 1], carry, mirrored);

      bool use_mirror = false;
      if (allow_mirror) {
        if (score_yes.cross_y < score_no.cross_y) {
          use_mirror = true;
        } else if (score_yes.cross_y > score_no.cross_y) {
          use_mirror = false;
        } else {
          const int look_no = score_no.cross_y + best_next_cross_y(seg + 1, prepared_ports_b[seg]);
          const int look_yes = score_yes.cross_y + best_next_cross_y(seg + 1, mirrored);
          if (look_yes < look_no) {
            use_mirror = true;
          } else if (look_yes > look_no) {
            use_mirror = false;
          } else if (has_previous_mirror) {
            use_mirror = previous_mirror;
          } else if (secondary_score_less(score_yes, score_no)) {
            use_mirror = true;
          }
        }
      }
      out.mirrors[seg] = use_mirror;
      std::vector<ObjectId> lanes_b = use_mirror ? mirrored : prepared_ports_b[seg];
      const MirrorScore chosen = use_mirror ? score_yes : score_no;
      out.total.cross_y += chosen.cross_y;
      out.total.cross_z += chosen.cross_z;
      out.total.layer_jump += chosen.layer_jump;
      out.total.span_z_delta += chosen.span_z_delta;
      carry = std::move(lanes_b);
      previous_mirror = use_mirror;
      has_previous_mirror = true;
    }
    return out;
  };

  std::vector<ObjectId> chosen_first_lanes_a = first_lanes_a;
  std::vector<bool> resolved_mirrors(segment_count, false);
  {
    const MirrorResolution normal = run_resolution(first_lanes_a);
    MirrorResolution best = normal;
    if (lane_count > 1) {
      std::vector<ObjectId> reversed_seed = first_lanes_a;
      std::reverse(reversed_seed.begin(), reversed_seed.end());
      const MirrorResolution reversed = run_resolution(reversed_seed);
      bool choose_reversed = false;
      if (reversed.total.cross_y < normal.total.cross_y) {
        choose_reversed = true;
      } else if (reversed.total.cross_y == normal.total.cross_y &&
                 secondary_score_less(reversed.total, normal.total)) {
        choose_reversed = true;
      }
      if (choose_reversed) {
        chosen_first_lanes_a = std::move(reversed_seed);
        best = reversed;
      }
    }
    resolved_mirrors = best.mirrors;
  }

  std::vector<ObjectId> carry_ports = chosen_first_lanes_a;
  bool previous_mirror = false;
  bool has_previous_mirror = false;

  auto compute_turn_angle_deg = [&](std::size_t segment_index) -> double {
    if (segment_index == 0 || segment_index + 1 >= poles.size()) {
      return 180.0;
    }
    const Pole* prev = edit_state_access().poles.find(poles[segment_index - 1]);
    const Pole* curr = edit_state_access().poles.find(poles[segment_index]);
    const Pole* next = edit_state_access().poles.find(poles[segment_index + 1]);
    if (prev == nullptr || curr == nullptr || next == nullptr) {
      return 180.0;
    }
    // Use vertex interior angle: straight-through == 180deg, acute corner < threshold.
    Vec3d in_dir = prev->world_transform.position - curr->world_transform.position;
    Vec3d out_dir = next->world_transform.position - curr->world_transform.position;
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

  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    const ObjectId pole_a = poles[seg];
    const ObjectId pole_b = poles[seg + 1];

    SegmentLaneAssignment assignment{};
    assignment.segment_index = seg;
    assignment.pole_a_id = pole_a;
    assignment.pole_b_id = pole_b;
    assignment.bundle_id = bundle_id;
    assignment.port_ids_a = carry_ports;
    assignment.port_ids_b = prepared_ports_b[seg];

    bool chosen_mirror = allow_mirror ? resolved_mirrors[seg] : false;
    const double turn_angle_deg = compute_turn_angle_deg(seg);
    const bool is_acute_turn = (seg > 0) && (turn_angle_deg + kAngleEps < layout_settings_.corner_threshold_deg);
    LaneFlipReason flip_reason = LaneFlipReason::kNone;
    bool flipped_from_previous = false;
    if (has_previous_mirror && chosen_mirror != previous_mirror) {
      flipped_from_previous = true;
      if (is_acute_turn) {
        flip_reason = LaneFlipReason::kAcuteTurn;
      }
    }
    if (chosen_mirror) {
      std::reverse(assignment.port_ids_b.begin(), assignment.port_ids_b.end());
    }
    assignment.mirrored = chosen_mirror;
    assignment.flipped_from_previous = flipped_from_previous;
    assignment.flip_reason = flip_reason;
    assignment.turn_angle_deg = turn_angle_deg;
    if (static_cast<int>(assignment.port_ids_a.size()) != lane_count ||
        static_cast<int>(assignment.port_ids_b.size()) != lane_count) {
      result.error = "failed to materialize lane assignment plan";
      return result;
    }

    for (int lane = 0; lane < lane_count; ++lane) {
      const ObjectId port_a_id = assignment.port_ids_a[static_cast<std::size_t>(lane)];
      const ObjectId port_b_id = assignment.port_ids_b[static_cast<std::size_t>(lane)];
      const auto add = AddSpan(port_a_id, port_b_id, SpanKind::kDistribution, category_to_span_layer(group_spec.category),
                               bundle_id);
      if (!add.ok) {
        result.error = add.error;
        return result;
      }
      append_change_set(result.change_set, add.change_set);
      result.value.push_back(add.value);

      const Port* pa = edit_state_access().ports.find(port_a_id);
      const Port* pb = edit_state_access().ports.find(port_b_id);
      assignment.slot_ids_a.push_back((pa == nullptr) ? -1 : pa->source_slot_id);
      assignment.slot_ids_b.push_back((pb == nullptr) ? -1 : pb->source_slot_id);

      Span* span = edit_state_access().spans.find(add.value);
      if (span != nullptr) {
        span->placement_context = ConnectionContext::kTrunkContinue;
        span->generated_by_rule = true;
        span->generation.generated = true;
      }
    }

    previous_mirror = chosen_mirror;
    has_previous_mirror = true;
    carry_ports = assignment.port_ids_b;
    if (out_lane_assignments != nullptr) {
      out_lane_assignments->push_back(assignment);
    }
    if (out_edge_orientations != nullptr) {
      BackboneEdgeOrientation edge_orientation{};
      edge_orientation.node_a_id = assignment.pole_a_id;
      edge_orientation.node_b_id = assignment.pole_b_id;
      edge_orientation.bundle_template_id = bundle_template_id;
      edge_orientation.orientation = assignment.mirrored ? LaneOrientation::kReversed : LaneOrientation::kNormal;
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
  auto is_count_compatible = [&](const BundleTemplate& tpl) -> bool {
    if (tpl.count_rule == BundleCountRuleKind::kFixed) {
      return options.group_spec.conductor_count == tpl.fixed_count;
    }
    return options.group_spec.conductor_count >= tpl.min_count && options.group_spec.conductor_count <= tpl.max_count;
  };
  auto group_affinity_score = [&](const BundleTemplate& tpl) -> int {
    if (options.group_spec.conductor_count <= 1) {
      return 0;
    }
    if (options.group_spec.group_kind == ConductorGroupKind::kThreePhase) {
      return tpl.preserve_conductor_identity ? 2 : 0;
    }
    if (options.group_spec.group_kind == ConductorGroupKind::kParallel) {
      return tpl.preserve_conductor_identity ? 0 : 2;
    }
    // Legacy callers often keep kSingle default while specifying count>1.
    return 1;
  };

  std::vector<const BundleTemplate*> compatible_templates{};
  compatible_templates.reserve(bundle_templates_.size());
  for (const auto& [_, tpl] : bundle_templates_) {
    if (is_count_compatible(tpl)) {
      compatible_templates.push_back(&tpl);
    }
  }
  if (compatible_templates.empty()) {
    result.error = "no bundle template matches grouped-line options; use BackboneSpec.bundles[] explicitly";
    return result;
  }

  const BundleKind preferred_kind = category_to_bundle_kind(options.group_spec.category);
  std::sort(compatible_templates.begin(), compatible_templates.end(),
            [&](const BundleTemplate* a, const BundleTemplate* b) {
              const int preferred_a = (a->id == preferred_kind) ? 1 : 0;
              const int preferred_b = (b->id == preferred_kind) ? 1 : 0;
              if (preferred_a != preferred_b) {
                return preferred_a > preferred_b;
              }
              const int affinity_a = group_affinity_score(*a);
              const int affinity_b = group_affinity_score(*b);
              if (affinity_a != affinity_b) {
                return affinity_a > affinity_b;
              }
              if (a->count_rule != b->count_rule) {
                return a->count_rule == BundleCountRuleKind::kFixed;
              }
              return static_cast<int>(a->id) < static_cast<int>(b->id);
            });
  const BundleTemplate* selected_template = compatible_templates.front();

  BackboneSpec req{};
  req.path.polyline = options.road.polyline;
  req.interval_m = (options.interval > 0.0) ? options.interval : (std::numeric_limits<double>::max() / 4.0);
  req.pole_type_id = options.pole_type_id;
  req.direction_mode = options.direction_mode;

  BackboneBundleSpec bundle{};
  bundle.bundle_template_id = selected_template->id;
  bundle.layer = selected_template->default_layer;
  if (selected_template->count_rule == BundleCountRuleKind::kRange) {
    bundle.count = options.group_spec.conductor_count;
  }
  req.bundles.push_back(bundle);

  const EditResult<GenerateBundleFromPathResult> adapted = GenerateFromBackboneSpec(req);
  if (!adapted.ok) {
    result.error = adapted.error;
    return result;
  }
  result.ok = true;
  result.value.pole_ids = adapted.value.generated_pole_ids;
  result.value.span_ids = adapted.value.generated_span_ids;
  result.value.bundle_id = adapted.value.bundle_id;
  result.value.lane_assignments = last_lane_assignments_access();
  result.value.direction_debug = last_path_direction_debug_;
  if (!adapted.value.generated_span_ids.empty()) {
    const Span* span = edit_state_access().spans.find(adapted.value.generated_span_ids.front());
    if (span != nullptr) {
      result.value.generation_session_id = span->generation.generation_session_id;
    }
  }
  result.change_set = adapted.change_set;
  return result;
}

EditResult<CoreState::GenerateBundleFromPathResult>
CoreState::GenerateFromBackboneSpec(const BackboneSpec& spec) {
  const BackboneSpec& request = spec;
  EditResult<GenerateBundleFromPathResult> result;
  if (request.path.polyline.size() < 2) {
    result.error = "backbone input path must contain at least 2 points";
    return result;
  }
  if (request.interval_m <= 0.0) {
    result.error = "interval_m must be > 0";
    return result;
  }
  if (find_pole_type(request.pole_type_id) == nullptr) {
    result.error = "pole type not found";
    return result;
  }

  std::unordered_map<std::size_t, BackboneInputSpec::NodeSpec> node_spec_by_index{};
  for (const BackboneInputSpec::NodeSpec& node_spec : request.path.node_specs) {
    if (node_spec.point_index >= request.path.polyline.size()) {
      result.error = "node_specs point_index is out of path range";
      return result;
    }
    node_spec_by_index[node_spec.point_index] = node_spec;
  }

  std::unordered_map<std::size_t, std::unordered_map<BundleKind, BundleNodeMode>> node_bundle_mode_by_point{};
  for (const BackboneSpec::NodeBundleModeSpec& mode_spec : request.node_bundle_modes) {
    if (mode_spec.point_index >= request.path.polyline.size()) {
      result.error = "node_bundle_modes point_index is out of path range";
      return result;
    }
    node_bundle_mode_by_point[mode_spec.point_index][mode_spec.bundle_template_id] = mode_spec.mode;
  }

  struct ResolvedBundlePlan {
    BundleKind template_id = BundleKind::kLowVoltage;
    ConnectionCategory category = ConnectionCategory::kLowVoltage;
    SpanLayer layer = SpanLayer::kUnknown;
    int count = 1;
    double spacing_m = 0.2;
    bool allow_mirror = true;
    bool preserve_conductor_identity = false;
    bool allow_midair_node = true;
    bool allow_midair_branch = true;
  };

  const std::vector<BackboneBundleSpec>& bundle_requests = request.bundles;
  if (bundle_requests.empty()) {
    result.error = "bundles[] must contain at least one bundle request";
    return result;
  }

  std::vector<ResolvedBundlePlan> bundle_plans{};
  bundle_plans.reserve(bundle_requests.size());
  for (const BackboneBundleSpec& bundle_request : bundle_requests) {
    const BundleTemplate* bundle_template = find_bundle_template(bundle_request.bundle_template_id);
    if (bundle_template == nullptr) {
      result.error = "bundle template not found";
      return result;
    }
    ResolvedBundlePlan plan{};
    plan.template_id = bundle_template->id;
    plan.category = bundle_template->category;
    plan.layer = (bundle_request.layer == SpanLayer::kUnknown) ? bundle_template->default_layer : bundle_request.layer;
    plan.spacing_m = bundle_template->default_spacing_m;
    plan.allow_mirror = bundle_template->allow_mirror;
    plan.preserve_conductor_identity = bundle_template->preserve_conductor_identity;
    plan.allow_midair_node = bundle_template->allow_midair_node;
    plan.allow_midair_branch = bundle_template->allow_midair_branch;
    if (bundle_template->count_rule == BundleCountRuleKind::kFixed) {
      if (bundle_request.count > 0) {
        result.error = "count override is not allowed for fixed bundle template";
        return result;
      }
      plan.count = bundle_template->fixed_count;
    } else {
      plan.count = (bundle_request.count > 0) ? bundle_request.count : bundle_template->default_count;
      if (plan.count < bundle_template->min_count || plan.count > bundle_template->max_count) {
        result.error = "bundle count is out of template range";
        return result;
      }
    }
    if (plan.count <= 0) {
      result.error = "resolved bundle count must be > 0";
      return result;
    }
    if (plan.layer == SpanLayer::kUnknown) {
      result.error = "bundle layer could not be resolved";
      return result;
    }
    bundle_plans.push_back(plan);
  }

  auto support_kind_for_point = [&](std::size_t point_index) -> SupportKind {
    const auto it = node_spec_by_index.find(point_index);
    if (it == node_spec_by_index.end()) {
      return SupportKind::kPole;
    }
    return it->second.support_kind;
  };

  for (std::size_t point_index = 0; point_index < request.path.polyline.size(); ++point_index) {
    const SupportKind support_kind = support_kind_for_point(point_index);
    if (support_kind == SupportKind::kPole) {
      continue;
    }
    for (const ResolvedBundlePlan& plan : bundle_plans) {
      if (!plan.allow_midair_node) {
        result.error = "bundle template does not allow midair support node";
        return result;
      }
    }
    const auto it_modes = node_bundle_mode_by_point.find(point_index);
    if (it_modes == node_bundle_mode_by_point.end()) {
      continue;
    }
    for (const auto& [bundle_template_id, mode] : it_modes->second) {
      const auto it_plan =
          std::find_if(bundle_plans.begin(), bundle_plans.end(),
                       [&](const ResolvedBundlePlan& p) { return p.template_id == bundle_template_id; });
      if (it_plan == bundle_plans.end()) {
        result.error = "node_bundle_modes references bundle template that is not selected";
        return result;
      }
      if (mode != BundleNodeMode::kNotPresent && !it_plan->allow_midair_node) {
        result.error = "bundle template does not allow midair support node";
        return result;
      }
      if (mode == BundleNodeMode::kBranch && !it_plan->allow_midair_branch) {
        result.error = "bundle template does not allow midair branch";
        return result;
      }
    }
  }

  std::vector<Vec3d> guide_points = request.path.polyline;
  PathDirectionEvaluationDebug direction_debug{};
  direction_debug.requested_mode = request.direction_mode;
  direction_debug.chosen = PathDirectionChosen::kForward;
  if (request.direction_mode == PathDirectionMode::kReverse) {
    std::reverse(guide_points.begin(), guide_points.end());
    direction_debug.chosen = PathDirectionChosen::kReverse;
  }
  last_path_direction_debug_ = direction_debug;
  path_direction_debug_records_access().push_back(direction_debug);
  if (path_direction_debug_records_access().size() > 128) {
    path_direction_debug_records_access().erase(path_direction_debug_records_access().begin());
  }

  struct CandidatePole {
    Vec3d world{};
    std::size_t segment_index = 0;
    int vertex_index = -1;
    double t = 0.0;
    PlacementMode mode = PlacementMode::kAuto;
    SupportKind support_kind = SupportKind::kPole;
    bool has_tangent_hint = false;
    Vec3d tangent_hint{};
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
  const bool pin_endpoints = request.pole_placement.pin_endpoints;
  const bool pin_vertices = request.pole_placement.pin_vertices;
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
    const bool is_start_endpoint = (i == 0);
    start.mode = (pin_vertices || (pin_endpoints && is_start_endpoint)) ? PlacementMode::kManual : PlacementMode::kAuto;
    if (const auto it = node_spec_by_index.find(i); it != node_spec_by_index.end()) {
      start.support_kind = it->second.support_kind;
      start.has_tangent_hint = it->second.has_tangent_hint;
      start.tangent_hint = it->second.tangent_hint;
    }
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
  end.mode = pin_endpoints ? PlacementMode::kManual : PlacementMode::kAuto;
  if (const auto it = node_spec_by_index.find(guide_points.size() - 1); it != node_spec_by_index.end()) {
    end.support_kind = it->second.support_kind;
    end.has_tangent_hint = it->second.has_tangent_hint;
    end.tangent_hint = it->second.tangent_hint;
  }
  add_unique_candidate(candidates, end);

  if (candidates.size() < 2) {
    result.error = "failed to build guide candidates";
    return result;
  }

  const CoreState snapshot = *this;
  const std::uint64_t session_id = next_generation_session_id_access()++;
  std::vector<AutoPoleTransformResult> guide_auto_transforms{};
  guide_auto_transforms.reserve(guide_points.size());
  Vec3d preferred_side_dir{0.0, 0.0, 0.0};
  bool has_preferred_side_dir = false;
  for (std::size_t i = 0; i < guide_points.size(); ++i) {
    const AutoPoleTransformResult auto_tf =
        make_auto_pole_transform(guide_points, i, has_preferred_side_dir ? &preferred_side_dir : nullptr);
    guide_auto_transforms.push_back(auto_tf);
    preferred_side_dir = side_axis_from_yaw_deg(auto_tf.transform.rotation_euler_deg.z);
    has_preferred_side_dir = true;
  }

  auto find_near_pole = [&](const Vec3d& world, PlacementMode preferred_mode) -> ObjectId {
    constexpr double kReuseRadius = 0.25;
    const double reuse_r2 = kReuseRadius * kReuseRadius;
    ObjectId best_id = kInvalidObjectId;
    double best_d2 = reuse_r2 + 1.0;
    bool best_mode_match = false;
    for (const Pole& pole : edit_state_access().poles.items()) {
      if (request.pole_placement.restrict_reuse_to_session) {
        if (request.pole_placement.reuse_session_id == 0 ||
            pole.generation.generation_session_id != request.pole_placement.reuse_session_id) {
          continue;
        }
      }
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
  std::vector<ObjectId> ordered_support_node_ids{};
  ordered_support_node_ids.reserve(candidates.size());
  std::unordered_map<ObjectId, SupportNode> support_node_by_id{};
  ObjectId next_virtual_support_id = 0x8000000000000000ull;

  auto ensure_support_node = [&](ObjectId node_id, const CandidatePole& candidate, ObjectId pole_id) {
    SupportNode& node = support_node_by_id[node_id];
    node.node_id = node_id;
    node.support_kind = candidate.support_kind;
    node.position = candidate.world;
    node.pole_id = pole_id;
    node.path_point_index = candidate.vertex_index;
    node.has_tangent_hint = candidate.has_tangent_hint;
    node.tangent_hint = candidate.tangent_hint;
  };

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const CandidatePole& candidate = candidates[i];
    if (candidate.support_kind != SupportKind::kPole) {
      const ObjectId virtual_node_id = next_virtual_support_id++;
      ensure_support_node(virtual_node_id, candidate, kInvalidObjectId);
      ordered_support_node_ids.push_back(virtual_node_id);
      continue;
    }
    ObjectId pole_id = find_near_pole(candidate.world, candidate.mode);
    if (pole_id != kInvalidObjectId) {
      Pole* pole = edit_state_access().poles.find(pole_id);
      if (pole != nullptr) {
        const Pole old_pole = *pole;
        bool updated = false;
        if (candidate.mode == PlacementMode::kManual) {
          apply_pole_placement_mode(*pole, PlacementMode::kManual);
          updated = true;
        }

        if (candidate.vertex_index >= 0) {
          pole->context = classify_pole_context_from_path(guide_points, static_cast<std::size_t>(candidate.vertex_index), 0);
          const AutoPoleTransformResult& auto_tf =
              guide_auto_transforms[static_cast<std::size_t>(candidate.vertex_index)];
          apply_sharp_debug_to_context(&pole->context, auto_tf.sharp);
          updated = true;
          // Reused poles must follow current corner-orientation rule unless explicitly overridden.
          if (!pole->orientation_override_flag && !pole->orientation_control.manual_yaw_override) {
            pole->world_transform.rotation_euler_deg.z = auto_tf.transform.rotation_euler_deg.z;
            updated = true;
          }
        } else {
          pole->context.kind = PoleContextKind::kStraight;
          apply_sharp_debug_to_context(&pole->context, SharpCornerOrientationDebug{});
          if (!pole->orientation_override_flag && !pole->orientation_control.manual_yaw_override) {
            const Vec3d dir = guide_points[candidate.segment_index + 1] - guide_points[candidate.segment_index];
            if ((dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) > 1e-12) {
              pole->world_transform.rotation_euler_deg.z = normalize_yaw_deg(std::atan2(dir.y, dir.x) * (180.0 / kPi));
              updated = true;
            }
          }
        }

        if (updated) {
          // Reused poles need endpoint reprojection after context/yaw updates.
          finalize_pole_transform_update(pole->id, old_pole, &result.change_set);
        }
      }
      ordered_pole_ids.push_back(pole_id);
      ensure_support_node(pole_id, candidate, pole_id);
      ordered_support_node_ids.push_back(pole_id);
      continue;
    }

    Transformd tf{};
    SharpCornerOrientationDebug created_sharp_debug{};
    bool has_created_sharp_debug = false;
    tf.position = candidate.world;
    if (candidate.vertex_index >= 0) {
      const AutoPoleTransformResult& auto_tf =
          guide_auto_transforms[static_cast<std::size_t>(candidate.vertex_index)];
      tf = auto_tf.transform;
      tf.position = candidate.world;
      created_sharp_debug = auto_tf.sharp;
      has_created_sharp_debug = true;
    } else {
      const Vec3d dir = guide_points[candidate.segment_index + 1] - guide_points[candidate.segment_index];
      if ((dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) > 1e-12) {
        tf.rotation_euler_deg.z = normalize_yaw_deg(std::atan2(dir.y, dir.x) * (180.0 / kPi));
      }
    }

    EditResult<ObjectId> add_pole =
        AddPole(tf, 10.0, "PathPole", PoleKind::kConcrete, candidate.mode);
    if (!add_pole.ok) {
      *this = snapshot;
      result.error = add_pole.error;
      return result;
    }
    Pole* pole = edit_state_access().poles.find(add_pole.value);
    if (pole != nullptr) {
      if (candidate.vertex_index >= 0) {
        pole->context = classify_pole_context_from_path(guide_points, static_cast<std::size_t>(candidate.vertex_index), 0);
        apply_sharp_debug_to_context(&pole->context, has_created_sharp_debug ? created_sharp_debug
                                                                              : SharpCornerOrientationDebug{});
      } else {
        pole->context.kind = PoleContextKind::kStraight;
        apply_sharp_debug_to_context(&pole->context, SharpCornerOrientationDebug{});
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
    ensure_support_node(add_pole.value, candidate, add_pole.value);
    ordered_support_node_ids.push_back(add_pole.value);
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
  {
    std::vector<ObjectId> compact_support_ids{};
    compact_support_ids.reserve(ordered_support_node_ids.size());
    for (ObjectId id : ordered_support_node_ids) {
      if (compact_support_ids.empty() || compact_support_ids.back() != id) {
        compact_support_ids.push_back(id);
      }
    }
    ordered_support_node_ids.swap(compact_support_ids);
  }
  if (ordered_pole_ids.size() < 2) {
    *this = snapshot;
    result.error = "failed to build valid pole chain";
    return result;
  }

  struct ChainEdgeKey {
    ObjectId a = kInvalidObjectId;
    ObjectId b = kInvalidObjectId;
    bool operator==(const ChainEdgeKey& other) const { return a == other.a && b == other.b; }
  };
  struct ChainEdgeKeyHash {
    std::size_t operator()(const ChainEdgeKey& key) const {
      const std::size_t h1 = std::hash<ObjectId>{}(key.a);
      const std::size_t h2 = std::hash<ObjectId>{}(key.b);
      return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
  };

  BackboneResult generation_backbone{};
  std::unordered_set<ChainEdgeKey, ChainEdgeKeyHash> unique_chain_edges{};
  std::unordered_map<ObjectId, std::unordered_map<ObjectId, std::uint32_t>> incident_first_order{};

  auto update_incident_order = [&](ObjectId node_id, ObjectId neighbor_id, std::uint32_t order_index) {
    auto& by_neighbor = incident_first_order[node_id];
    auto it = by_neighbor.find(neighbor_id);
    if (it == by_neighbor.end() || order_index < it->second) {
      by_neighbor[neighbor_id] = order_index;
    }
  };

  auto support_position = [&](ObjectId node_id) -> Vec3d {
    const auto it = support_node_by_id.find(node_id);
    if (it != support_node_by_id.end()) {
      return it->second.position;
    }
    if (const Pole* pole = edit_state_access().poles.find(node_id); pole != nullptr) {
      return pole->world_transform.position;
    }
    return {};
  };

  for (std::size_t i = 0; i + 1 < ordered_support_node_ids.size(); ++i) {
    const ObjectId a = ordered_support_node_ids[i];
    const ObjectId b = ordered_support_node_ids[i + 1];
    if (a == kInvalidObjectId || b == kInvalidObjectId || a == b) {
      continue;
    }
    update_incident_order(a, b, static_cast<std::uint32_t>(i));
    update_incident_order(b, a, static_cast<std::uint32_t>(i));

    const ObjectId key_a = std::min(a, b);
    const ObjectId key_b = std::max(a, b);
    if (unique_chain_edges.insert({key_a, key_b}).second) {
      BackboneEdge edge{};
      edge.node_a = key_a;
      edge.node_b = key_b;
      generation_backbone.edges.push_back(edge);
    }
  }
  std::sort(generation_backbone.edges.begin(), generation_backbone.edges.end(),
            [](const BackboneEdge& lhs, const BackboneEdge& rhs) {
              if (lhs.node_a != rhs.node_a) {
                return lhs.node_a < rhs.node_a;
              }
              return lhs.node_b < rhs.node_b;
            });

  auto normalize_dir = [](const Vec3d& v) -> Vec3d {
    const double len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len <= 1e-9) {
      return {0.0, 0.0, 0.0};
    }
    return {v.x / len, v.y / len, v.z / len};
  };
  auto dot = [](const Vec3d& a, const Vec3d& b) -> double { return a.x * b.x + a.y * b.y + a.z * b.z; };

  const BackboneResult existing_backbone = BuildBackboneResult();
  std::unordered_map<ObjectId, ObjectId> existing_primary_neighbor_by_node{};
  std::unordered_map<ObjectId, std::uint64_t> existing_prioritized_session_by_node{};
  std::unordered_map<ObjectId, std::unordered_map<ObjectId, std::uint64_t>> existing_incident_session_by_node{};
  for (const JunctionInfo& junction : existing_backbone.junctions) {
    existing_prioritized_session_by_node[junction.node_id] = junction.prioritized_session_id;
    for (const JunctionIncident& incident : junction.incidents) {
      existing_incident_session_by_node[junction.node_id][incident.neighbor_node_id] = incident.source_session_id;
      if (incident.primary) {
        existing_primary_neighbor_by_node[junction.node_id] = incident.neighbor_node_id;
      }
    }
  }

  std::unordered_map<ObjectId, ObjectId> backbone_primary_neighbors{};
  for (const auto& [node_id, neighbors] : incident_first_order) {
    if (neighbors.size() < 3) {
      continue;
    }
    const Vec3d center_pos = support_position(node_id);

    struct Candidate {
      ObjectId neighbor_id = kInvalidObjectId;
      Vec3d dir{};
    };
    std::vector<Candidate> candidates{};
    candidates.reserve(neighbors.size());
    for (const auto& [neighbor_id, first_order] : neighbors) {
      (void)first_order;
      const Vec3d neighbor_pos = support_position(neighbor_id);
      Candidate c{};
      c.neighbor_id = neighbor_id;
      c.dir = normalize_dir(neighbor_pos - center_pos);
      candidates.push_back(c);
    }
    if (candidates.size() < 3) {
      continue;
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) { return a.neighbor_id < b.neighbor_id; });

    int anchor_index = -1;
    bool used_neighbor_continuity = false;
    const auto it_existing_primary = existing_primary_neighbor_by_node.find(node_id);
    if (it_existing_primary != existing_primary_neighbor_by_node.end()) {
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].neighbor_id == it_existing_primary->second) {
          anchor_index = static_cast<int>(i);
          used_neighbor_continuity = true;
          break;
        }
      }
    }
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      if (anchor_index >= 0) {
        break;
      }
      const auto it_prev = backbone_primary_neighbors.find(candidates[i].neighbor_id);
      if (it_prev == backbone_primary_neighbors.end() || it_prev->second != node_id) {
        continue;
      }
      const int idx = static_cast<int>(i);
      if (anchor_index < 0 || candidates[static_cast<std::size_t>(idx)].neighbor_id <
                                  candidates[static_cast<std::size_t>(anchor_index)].neighbor_id) {
        anchor_index = idx;
        used_neighbor_continuity = true;
      }
    }

    if (anchor_index < 0) {
      double best_pair_straight = -2.0;
      int best_pair_anchor = -1;
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        for (std::size_t j = i + 1; j < candidates.size(); ++j) {
          const double straight_score = dot(candidates[i].dir, Vec3d{-candidates[j].dir.x, -candidates[j].dir.y,
                                                                      -candidates[j].dir.z});
          int pair_anchor = static_cast<int>(i);
          if (candidates[j].neighbor_id < candidates[i].neighbor_id) {
            pair_anchor = static_cast<int>(j);
          }
          if (straight_score > best_pair_straight + 1e-9 ||
              (std::abs(straight_score - best_pair_straight) <= 1e-9 &&
               (best_pair_anchor < 0 || candidates[static_cast<std::size_t>(pair_anchor)].neighbor_id <
                                             candidates[static_cast<std::size_t>(best_pair_anchor)].neighbor_id))) {
            best_pair_straight = straight_score;
            best_pair_anchor = pair_anchor;
          }
        }
      }
      anchor_index = (best_pair_anchor >= 0) ? best_pair_anchor : 0;
    }

    int opposite_index = -1;
    double best_straight = -2.0;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const int idx = static_cast<int>(i);
      if (idx == anchor_index) {
        continue;
      }
      const double straight_score =
          dot(candidates[static_cast<std::size_t>(anchor_index)].dir,
              Vec3d{-candidates[i].dir.x, -candidates[i].dir.y, -candidates[i].dir.z});
      if (straight_score > best_straight + 1e-9 ||
          (std::abs(straight_score - best_straight) <= 1e-9 &&
           candidates[i].neighbor_id <
               candidates[static_cast<std::size_t>(opposite_index < 0 ? idx : opposite_index)].neighbor_id)) {
        best_straight = straight_score;
        opposite_index = idx;
      }
    }

    std::vector<int> order_indices{};
    order_indices.push_back(anchor_index);
    if (opposite_index >= 0 && opposite_index != anchor_index) {
      order_indices.push_back(opposite_index);
    }
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      const int idx = static_cast<int>(i);
      if (idx == anchor_index || idx == opposite_index) {
        continue;
      }
      order_indices.push_back(idx);
    }

    JunctionInfo junction{};
    junction.node_id = node_id;
    junction.prioritized_session_id = session_id;
    const auto it_existing_prioritized = existing_prioritized_session_by_node.find(node_id);
    if (it_existing_prioritized != existing_prioritized_session_by_node.end()) {
      junction.prioritized_session_id = it_existing_prioritized->second;
    }
    junction.used_neighbor_continuity = used_neighbor_continuity;
    for (std::size_t rank = 0; rank < order_indices.size(); ++rank) {
      const Candidate& candidate = candidates[static_cast<std::size_t>(order_indices[rank])];
      JunctionIncident incident{};
      incident.neighbor_node_id = candidate.neighbor_id;
      incident.order = static_cast<int>(rank);
      incident.primary = (rank == 0);
      incident.source_session_id = session_id;
      const auto it_existing_node = existing_incident_session_by_node.find(node_id);
      if (it_existing_node != existing_incident_session_by_node.end()) {
        const auto it_existing_source = it_existing_node->second.find(candidate.neighbor_id);
        if (it_existing_source != it_existing_node->second.end()) {
          incident.source_session_id = it_existing_source->second;
        }
      }
      junction.incidents.push_back(incident);
    }
    if (!junction.incidents.empty()) {
      backbone_primary_neighbors[node_id] = junction.incidents.front().neighbor_node_id;
    }
    generation_backbone.junctions.push_back(std::move(junction));
  }
  std::sort(generation_backbone.junctions.begin(), generation_backbone.junctions.end(),
            [](const JunctionInfo& a, const JunctionInfo& b) { return a.node_id < b.node_id; });

  generation_backbone.nodes.reserve(support_node_by_id.size());
  for (const auto& [node_id, base_node] : support_node_by_id) {
    SupportNode node = base_node;
    const int incident_degree = static_cast<int>(incident_first_order[node_id].size());
    std::unordered_map<BundleKind, BundleNodeMode> mode_by_bundle{};
    for (const ResolvedBundlePlan& plan : bundle_plans) {
      BundleNodeMode mode = BundleNodeMode::kNotPresent;
      if (incident_degree <= 0) {
        mode = BundleNodeMode::kNotPresent;
      } else if (incident_degree == 1) {
        mode = BundleNodeMode::kTerminate;
      } else if (incident_degree == 2) {
        mode = BundleNodeMode::kPassThrough;
      } else {
        mode = BundleNodeMode::kBranch;
      }
      mode_by_bundle[plan.template_id] = mode;
    }
    if (node.path_point_index >= 0) {
      const auto it_mode_spec = node_bundle_mode_by_point.find(static_cast<std::size_t>(node.path_point_index));
      if (it_mode_spec != node_bundle_mode_by_point.end()) {
        for (const auto& [bundle_template_id, mode] : it_mode_spec->second) {
          mode_by_bundle[bundle_template_id] = mode;
        }
      }
    }

    node.bundle_modes.clear();
    node.bundle_modes.reserve(mode_by_bundle.size());
    for (const auto& [bundle_template_id, mode] : mode_by_bundle) {
      SupportNodeBundleMode bundle_mode{};
      bundle_mode.bundle_template_id = bundle_template_id;
      bundle_mode.mode = mode;
      node.bundle_modes.push_back(bundle_mode);
    }
    std::sort(node.bundle_modes.begin(), node.bundle_modes.end(),
              [](const SupportNodeBundleMode& a, const SupportNodeBundleMode& b) {
                return static_cast<int>(a.bundle_template_id) < static_cast<int>(b.bundle_template_id);
              });
    generation_backbone.nodes.push_back(std::move(node));
  }
  std::sort(generation_backbone.nodes.begin(), generation_backbone.nodes.end(),
            [](const SupportNode& a, const SupportNode& b) { return a.node_id < b.node_id; });

  auto count_existing_segment_spans = [&](ObjectId pole_a, ObjectId pole_b, const ResolvedBundlePlan& plan) -> int {
    int count = 0;
    for (const Span& span : edit_state_access().spans.items()) {
      if (span.layer != plan.layer || span.bundle_id == kInvalidObjectId) {
        continue;
      }
      const Bundle* bundle = edit_state_access().bundles.find(span.bundle_id);
      if (bundle == nullptr || bundle->kind != plan.template_id) {
        continue;
      }
      const Port* pa = edit_state_access().ports.find(span.port_a_id);
      const Port* pb = edit_state_access().ports.find(span.port_b_id);
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

  std::vector<SegmentLaneAssignment> all_lane_assignments{};
  for (const ResolvedBundlePlan& plan : bundle_plans) {
    int missing_total = 0;
    std::size_t first_missing_segment = ordered_pole_ids.size();
    for (std::size_t i = 0; i + 1 < ordered_pole_ids.size(); ++i) {
      const int existing_count = count_existing_segment_spans(ordered_pole_ids[i], ordered_pole_ids[i + 1], plan);
      const int missing = std::max(0, plan.count - existing_count);
      missing_total += missing;
      if (missing > 0 && first_missing_segment == ordered_pole_ids.size()) {
        first_missing_segment = i;
      }
    }
    if (missing_total <= 0) {
      continue;
    }

    EditResult<ObjectId> bundle_result = AddBundle(plan.count, plan.spacing_m, plan.template_id);
    if (!bundle_result.ok) {
      *this = snapshot;
      result.error = bundle_result.error;
      return result;
    }
    const ObjectId bundle_id = bundle_result.value;
    append_change_set(result.change_set, bundle_result.change_set);
    result.value.bundle_ids.push_back(bundle_id);
    if (result.value.bundle_id == kInvalidObjectId) {
      result.value.bundle_id = bundle_id;
    }

    if (first_missing_segment >= ordered_pole_ids.size() - 1) {
      continue;
    }
    std::vector<ObjectId> local_poles{};
    local_poles.insert(local_poles.end(), ordered_pole_ids.begin() + static_cast<std::ptrdiff_t>(first_missing_segment),
                       ordered_pole_ids.end());

    ConductorGroupSpec group_spec{};
    group_spec.category = plan.category;
    group_spec.conductor_count = plan.count;
    group_spec.group_kind =
        (plan.count <= 1)
            ? ConductorGroupKind::kSingle
            : (plan.preserve_conductor_identity ? ConductorGroupKind::kThreePhase : ConductorGroupKind::kParallel);
    group_spec.lane_spacing_m = plan.spacing_m;
    group_spec.maintain_lane_order = true;
    group_spec.allow_lane_mirror = plan.allow_mirror;

    std::vector<SegmentLaneAssignment> lane_assignments{};
    std::vector<BackboneEdgeOrientation> edge_orientations{};
    EditResult<std::vector<ObjectId>> spans_result = generate_grouped_spans_between_poles(
        local_poles, bundle_id, group_spec, &lane_assignments, &edge_orientations, plan.template_id);
    if (!spans_result.ok) {
      *this = snapshot;
      result.error = spans_result.error;
      return result;
    }
    append_change_set(result.change_set, spans_result.change_set);
    all_lane_assignments.insert(all_lane_assignments.end(), lane_assignments.begin(), lane_assignments.end());
    generation_backbone.edge_orientations.insert(generation_backbone.edge_orientations.end(), edge_orientations.begin(),
                                                 edge_orientations.end());

    for (std::size_t i = 0; i < spans_result.value.size(); ++i) {
      const ObjectId span_id = spans_result.value[i];
      Span* span = edit_state_access().spans.find(span_id);
      if (span != nullptr) {
        span->layer = plan.layer;
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

  last_lane_assignments_access() = all_lane_assignments;
  last_generation_backbone_ = generation_backbone;
  result.ok = true;
  return result;
}

EditResult<CoreState::GenerateBundleFromPathResult>
CoreState::RegenerateSessionAutoParts(std::uint64_t generation_session_id, const BackboneSpec& request) {
  EditResult<GenerateBundleFromPathResult> result;
  if (generation_session_id == 0) {
    result.error = "generation_session_id must be non-zero";
    return result;
  }

  const CoreState snapshot = *this;
  ChangeSet cleanup_changes{};

  std::vector<ObjectId> target_span_ids{};
  target_span_ids.reserve(edit_state_access().spans.size());
  std::unordered_set<ObjectId> candidate_bundle_ids{};
  for (const Span& span : edit_state_access().spans.items()) {
    if (span.generation.generation_session_id != generation_session_id) {
      continue;
    }
    target_span_ids.push_back(span.id);
    if (span.bundle_id != kInvalidObjectId) {
      candidate_bundle_ids.insert(span.bundle_id);
    }
  }

  for (ObjectId span_id : target_span_ids) {
    const Span* span_ptr = edit_state_access().spans.find(span_id);
    if (span_ptr == nullptr) {
      continue;
    }
    const Span span = *span_ptr;
    remove_span_from_indexes(span);
    edit_state_access().spans.remove(span_id);
    span_runtime_states_access().erase(span_id);
    remove_span_from_caches(span_id);
    add_unique_id(cleanup_changes.deleted_ids, span_id);

    std::vector<ObjectId> remove_attachments{};
    for (const Attachment& attachment : edit_state_access().attachments.items()) {
      if (attachment.span_id == span_id) {
        remove_attachments.push_back(attachment.id);
      }
    }
    for (ObjectId attachment_id : remove_attachments) {
      edit_state_access().attachments.remove(attachment_id);
      add_unique_id(cleanup_changes.deleted_ids, attachment_id);
    }
  }

  auto erase_removed_spans_from_queue = [&](std::vector<ObjectId>& queue) {
    queue.erase(std::remove_if(queue.begin(), queue.end(),
                               [&](ObjectId id) {
                                 return std::find(target_span_ids.begin(), target_span_ids.end(), id) !=
                                        target_span_ids.end();
                               }),
                queue.end());
  };
  erase_removed_spans_from_queue(dirty_queue_access().topology_dirty_span_ids);
  erase_removed_spans_from_queue(dirty_queue_access().geometry_dirty_span_ids);
  erase_removed_spans_from_queue(dirty_queue_access().bounds_dirty_span_ids);
  erase_removed_spans_from_queue(dirty_queue_access().render_dirty_span_ids);
  erase_removed_spans_from_queue(dirty_queue_access().raycast_dirty_span_ids);

  std::vector<ObjectId> target_auto_pole_ids{};
  target_auto_pole_ids.reserve(edit_state_access().poles.size());
  for (const Pole& pole : edit_state_access().poles.items()) {
    if (pole.generation.generation_session_id != generation_session_id) {
      continue;
    }
    if (pole.placement_mode == PlacementMode::kAuto) {
      target_auto_pole_ids.push_back(pole.id);
    }
  }

  for (ObjectId pole_id : target_auto_pole_ids) {
    const Pole* pole = edit_state_access().poles.find(pole_id);
    if (pole == nullptr) {
      continue;
    }
    std::vector<ObjectId> owned_ports{};
    std::vector<ObjectId> owned_anchors{};
    if (const auto it = relation_index_access().ports_by_pole.find(pole_id);
        it != relation_index_access().ports_by_pole.end()) {
      owned_ports = it->second;
    }
    if (const auto it = relation_index_access().anchors_by_pole.find(pole_id);
        it != relation_index_access().anchors_by_pole.end()) {
      owned_anchors = it->second;
    }

    bool has_live_connections = false;
    bool has_manual_port = false;
    for (ObjectId port_id : owned_ports) {
      const Port* owned_port = edit_state_access().ports.find(port_id);
      if (owned_port != nullptr && owned_port->position_mode == PortPositionMode::kManual) {
        has_manual_port = true;
      }
      auto it = connection_index_access().spans_by_port.find(port_id);
      if (it != connection_index_access().spans_by_port.end() && !it->second.empty()) {
        has_live_connections = true;
        break;
      }
    }
    if (has_live_connections || has_manual_port) {
      continue;
    }

    for (ObjectId port_id : owned_ports) {
      connection_index_access().spans_by_port.erase(port_id);
      const Port* port_ptr = edit_state_access().ports.find(port_id);
      const Port port_copy = (port_ptr == nullptr) ? Port{} : *port_ptr;
      if (edit_state_access().ports.remove(port_id)) {
        if (port_copy.owner_pole_id != kInvalidObjectId) {
          index_remove(relation_index_access().ports_by_pole, port_copy.owner_pole_id, port_id);
        }
        add_unique_id(cleanup_changes.deleted_ids, port_id);
      }
    }
    for (ObjectId anchor_id : owned_anchors) {
      connection_index_access().spans_by_anchor.erase(anchor_id);
      const Anchor* anchor_ptr = edit_state_access().anchors.find(anchor_id);
      const Anchor anchor_copy = (anchor_ptr == nullptr) ? Anchor{} : *anchor_ptr;
      if (edit_state_access().anchors.remove(anchor_id)) {
        if (anchor_copy.owner_pole_id != kInvalidObjectId) {
          index_remove(relation_index_access().anchors_by_pole, anchor_copy.owner_pole_id, anchor_id);
        }
        add_unique_id(cleanup_changes.deleted_ids, anchor_id);
      }
    }
    if (edit_state_access().poles.remove(pole_id)) {
      add_unique_id(cleanup_changes.deleted_ids, pole_id);
    }
  }

  for (ObjectId bundle_id : candidate_bundle_ids) {
    bool used = false;
    for (const Span& span : edit_state_access().spans.items()) {
      if (span.bundle_id == bundle_id) {
        used = true;
        break;
      }
    }
    if (!used && edit_state_access().bundles.remove(bundle_id)) {
      relation_index_access().spans_by_bundle.erase(bundle_id);
      add_unique_id(cleanup_changes.deleted_ids, bundle_id);
    }
  }

  BackboneSpec regen_request = request;
  regen_request.pole_placement.restrict_reuse_to_session = true;
  regen_request.pole_placement.reuse_session_id = generation_session_id;
  EditResult<GenerateBundleFromPathResult> regenerated = GenerateFromBackboneSpec(regen_request);
  if (!regenerated.ok) {
    *this = snapshot;
    result.error = regenerated.error;
    return result;
  }

  for (ObjectId pole_id : regenerated.value.generated_pole_ids) {
    Pole* pole = edit_state_access().poles.find(pole_id);
    if (pole != nullptr) {
      pole->generation.generation_session_id = generation_session_id;
      add_unique_id(regenerated.change_set.updated_ids, pole_id);
    }
  }
  for (ObjectId span_id : regenerated.value.generated_span_ids) {
    Span* span = edit_state_access().spans.find(span_id);
    if (span != nullptr) {
      span->generation.generation_session_id = generation_session_id;
      add_unique_id(regenerated.change_set.updated_ids, span_id);
    }
  }
  append_change_set(regenerated.change_set, cleanup_changes);
  return regenerated;
}

} // namespace wire::core
