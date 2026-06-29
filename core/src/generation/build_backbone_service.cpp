#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "../pole_orientation_utils.hpp"
#include "detail_utils.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

namespace wire::core {

using namespace generation::detail;

namespace {

BundleKind bundle_template_for_category(ConnectionCategory category) {
  switch (category) {
  case ConnectionCategory::kHighVoltage:
    return BundleKind::kHighVoltage;
  case ConnectionCategory::kCommunication:
    return BundleKind::kCommunication;
  case ConnectionCategory::kOptical:
    return BundleKind::kOptical;
  case ConnectionCategory::kDrop:
    return BundleKind::kDrop;
  case ConnectionCategory::kLowVoltage:
  default:
    return BundleKind::kLowVoltage;
  }
}

double polyline_length_local(const std::vector<Vec3d>& polyline) {
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    const Vec3d delta = polyline[i + 1] - polyline[i];
    total += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
  }
  return total;
}

BackboneSpec make_backbone_request_from_road(const RoadSegment& road, double interval, PoleTypeId pole_type_id,
                                             ConnectionCategory category, bool clicked_points_only) {
  BackboneSpec request{};
  request.path.polyline = road.polyline;
  request.pole_type_id = pole_type_id;
  request.interval_m = clicked_points_only ? std::max(1.0, polyline_length_local(road.polyline) + 1.0) : interval;
  BackboneBundleSpec bundle{};
  bundle.bundle_template_id = bundle_template_for_category(category);
  request.bundles.push_back(bundle);
  return request;
}

GenerateSimpleLineResult make_simple_line_result_from_backbone(const CoreState& state,
                                                               const GenerateBundleFromPathResult& generated) {
  GenerateSimpleLineResult out{};
  out.pole_ids = generated.generated_pole_ids;
  out.span_ids = generated.generated_span_ids;
  if (!generated.generated_span_ids.empty()) {
    const Span* span = state.view().edit_state().spans.find(generated.generated_span_ids.back());
    if (span != nullptr) {
      out.generation_session_id = span->generation.generation_session_id;
    }
  } else if (!generated.generated_pole_ids.empty()) {
    const Pole* pole = state.view().edit_state().poles.find(generated.generated_pole_ids.back());
    if (pole != nullptr) {
      out.generation_session_id = pole->generation.generation_session_id;
    }
  }
  return out;
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

EditResult<GenerateSimpleLineResult> CoreState::GenerateSimpleLine(const RoadSegment& road, double interval,
                                                                   PoleTypeId pole_type_id,
                                                                   ConnectionCategory category) {
  EditResult<GenerateSimpleLineResult> result;
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

  const BackboneSpec request = make_backbone_request_from_road(road, interval, pole_type_id, category, false);
  const EditResult<GenerateBundleFromPathResult> generated = GenerateFromBackboneSpec(request);
  if (!generated.ok) {
    result.error = generated.error;
    return result;
  }

  result.ok = true;
  result.value = make_simple_line_result_from_backbone(*this, generated.value);
  result.change_set = generated.change_set;
  return result;
}

EditResult<GenerateSimpleLineResult>
CoreState::GenerateSimpleLineFromPoints(const RoadSegment& road, PoleTypeId pole_type_id, ConnectionCategory category) {
  EditResult<GenerateSimpleLineResult> result;
  if (road.polyline.size() < 2) {
    result.error = "road polyline must contain at least 2 points";
    return result;
  }
  if (find_pole_type(pole_type_id) == nullptr) {
    result.error = "pole type not found";
    return result;
  }

  const BackboneSpec request = make_backbone_request_from_road(road, 0.0, pole_type_id, category, true);
  const EditResult<GenerateBundleFromPathResult> generated = GenerateFromBackboneSpec(request);
  if (!generated.ok) {
    result.error = generated.error;
    return result;
  }

  result.ok = true;
  result.value = make_simple_line_result_from_backbone(*this, generated.value);
  result.change_set = generated.change_set;
  return result;
}

} // namespace wire::core
