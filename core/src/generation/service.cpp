#include "wire/core/core_state.hpp"
#include "../pole_orientation_utils.hpp"
#include "detail_utils.hpp"

#include <array>
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

using namespace generation::detail;


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
        compute_auto_pole_transform(points, i, has_preferred_side_dir ? &preferred_side_dir : nullptr);
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


} // namespace wire::core

