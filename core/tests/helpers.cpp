#include "helpers.hpp"

#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <unordered_map>

namespace helpers {
namespace {

constexpr double kPi = 3.14159265358979323846;

double length_xy(const wire::core::Vec3d& v) { return std::sqrt(v.x * v.x + v.y * v.y); }

double distance3d(const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  const wire::core::Vec3d d = a - b;
  return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
}

double distance_xy(const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  const wire::core::Vec3d d = a - b;
  return std::sqrt(d.x * d.x + d.y * d.y);
}

double axis_angle_deg(const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  const wire::core::Vec3d na = normalize_xy_safe(a);
  const wire::core::Vec3d nb = normalize_xy_safe(b);
  const double la = length_xy(na);
  const double lb = length_xy(nb);
  if (la <= 1e-9 || lb <= 1e-9) {
    return 0.0;
  }
  const double c = std::clamp(std::abs(dot_xy(na, nb)), 0.0, 1.0);
  return std::acos(c) * (180.0 / kPi);
}

wire::core::Vec3d farthest_pair_axis_xy(const std::vector<wire::core::Vec3d>& points) {
  wire::core::Vec3d best{0.0, 0.0, 0.0};
  double best_len2 = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1; j < points.size(); ++j) {
      const wire::core::Vec3d d = points[j] - points[i];
      const double len2 = d.x * d.x + d.y * d.y;
      if (len2 > best_len2) {
        best = d;
        best_len2 = len2;
      }
    }
  }
  return normalize_xy_safe(best);
}

double min_pairwise_distance3d(const std::vector<wire::core::Vec3d>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  double min_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1; j < points.size(); ++j) {
      min_distance = std::min(min_distance, distance3d(points[i], points[j]));
    }
  }
  return std::isfinite(min_distance) ? min_distance : 0.0;
}

double min_pairwise_distance_xy(const std::vector<wire::core::Vec3d>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  double min_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1; j < points.size(); ++j) {
      min_distance = std::min(min_distance, distance_xy(points[i], points[j]));
    }
  }
  return std::isfinite(min_distance) ? min_distance : 0.0;
}

double mean_pairwise_distance_xy(const std::vector<wire::core::Vec3d>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  double sum = 0.0;
  int count = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1; j < points.size(); ++j) {
      sum += distance_xy(points[i], points[j]);
      ++count;
    }
  }
  return (count > 0) ? (sum / static_cast<double>(count)) : 0.0;
}

const wire::core::Span* find_span_by_ports(const CoreState& state, ObjectId port_a_id, ObjectId port_b_id) {
  for (const wire::core::Span& span : state.view().edit_state().spans.items()) {
    const bool same_forward = span.port_a_id == port_a_id && span.port_b_id == port_b_id;
    const bool same_reverse = span.port_a_id == port_b_id && span.port_b_id == port_a_id;
    if (same_forward || same_reverse) {
      return &span;
    }
  }
  return nullptr;
}

} // namespace

CoreCounts snapshot_counts(const CoreState& state) {
  return {
      state.view().edit_state().poles.size(),   state.view().edit_state().ports.size(), state.view().edit_state().anchors.size(),
      state.view().edit_state().bundles.size(), state.view().edit_state().spans.size(), state.view().edit_state().attachments.size(),
  };
}

bool same_counts(const CoreCounts& a, const CoreCounts& b) {
  return a.poles == b.poles && a.ports == b.ports && a.anchors == b.anchors && a.bundles == b.bundles &&
         a.spans == b.spans && a.attachments == b.attachments;
}

bool regex_contains(const std::string& text, const std::string& pattern) {
  return std::regex_search(text, std::regex(pattern));
}

bool has_dirty(const wire::core::SpanRuntimeState* state, DirtyBits bits) {
  return state != nullptr && wire::core::any(state->dirty_bits, bits);
}

bool contains_id(const std::vector<ObjectId>& ids, ObjectId id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

bool almost_equal(double a, double b, double eps) { return std::abs(a - b) <= eps; }

bool almost_equal(const wire::core::Vec3d& a, const wire::core::Vec3d& b, double eps) {
  return almost_equal(a.x, b.x, eps) && almost_equal(a.y, b.y, eps) && almost_equal(a.z, b.z, eps);
}

wire::core::Vec3d normalize_xy_safe(const wire::core::Vec3d& v) {
  const double len = std::sqrt(v.x * v.x + v.y * v.y);
  if (len <= 1e-12) {
    return {0.0, 0.0, 0.0};
  }
  return {v.x / len, v.y / len, 0.0};
}

double dot_xy(const wire::core::Vec3d& a, const wire::core::Vec3d& b) { return a.x * b.x + a.y * b.y; }

wire::core::Vec3d local_side_axis_from_yaw(double yaw_deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double rad = yaw_deg * (kPi / 180.0);
  return normalize_xy_safe(wire::core::Vec3d{-std::sin(rad), std::cos(rad), 0.0});
}

double angle_diff_abs_deg(double a, double b) {
  double d = std::fmod(a - b, 360.0);
  if (d <= -180.0) {
    d += 360.0;
  } else if (d > 180.0) {
    d -= 360.0;
  }
  return std::abs(d);
}

wire::core::Vec3d rotate_xy_by_yaw_test(const wire::core::Vec3d& local, double yaw_deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double rad = yaw_deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {local.x * c - local.y * s, local.x * s + local.y * c, local.z};
}

double effective_pole_yaw_deg_test(const wire::core::Pole& pole) { return pole.world_transform.rotation_euler_deg.z; }

wire::core::Vec3d to_local_on_pole_test(const wire::core::Pole& pole, const wire::core::Vec3d& world) {
  const wire::core::Vec3d delta = world - pole.world_transform.position;
  return rotate_xy_by_yaw_test(delta, -effective_pole_yaw_deg_test(pole));
}

bool aabb_valid(const wire::core::AABBd& aabb) {
  return aabb.min.x <= aabb.max.x && aabb.min.y <= aabb.max.y &&
         wire::core::HeightAlongWorldUp(aabb.min) <= wire::core::HeightAlongWorldUp(aabb.max);
}

bool starts_with(const std::string& value, const std::string& prefix) { return value.rfind(prefix, 0) == 0; }

wire::core::ValidationResult validate_now(CoreState& state) {
  wire::core::CommitOptions options{};
  options.run_recalc = false;
  options.run_validate = true;
  return state.Commit(options).validation;
}

std::vector<PoleTypeId> sorted_pole_type_ids(const CoreState& state) {
  std::vector<PoleTypeId> ids;
  ids.reserve(state.view().pole_types().size());
  for (const auto& [id, _] : state.view().pole_types()) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

wire::core::EditResult<BackbonePathGenerateResult> generate_from_backbone_options(
    CoreState& state, const BackbonePathGenerateOptions& options) {
  wire::core::EditResult<BackbonePathGenerateResult> result{};
  if (options.bundle_count < 0) {
    result.error = "bundle_count must be >= 0";
    return result;
  }
  if (state.view().bundle_templates().find(options.bundle_template_id) == state.view().bundle_templates().end()) {
    result.error = "bundle template not found";
    return result;
  }

  auto& mutable_templates = wire::core::CoreStateTestHook::bundle_templates(state);
  const auto mutable_it = mutable_templates.find(options.bundle_template_id);
  const bool has_prev_allow_mirror = mutable_it != mutable_templates.end();
  const bool prev_allow_mirror = has_prev_allow_mirror ? mutable_it->second.allow_mirror : true;
  if (options.override_allow_mirror && has_prev_allow_mirror) {
    mutable_it->second.allow_mirror = options.allow_mirror;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = options.road.polyline;
  req.interval_m = (options.interval > 0.0) ? options.interval : (std::numeric_limits<double>::max() / 4.0);
  req.pole_type_id = options.pole_type_id;
  req.direction_mode = options.direction_mode;
  wire::core::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = options.bundle_template_id;
  if (options.bundle_count > 0) {
    bundle.count = options.bundle_count;
  }
  req.bundles.push_back(bundle);

  const auto adapted = state.GenerateFromBackboneSpec(req);
  if (options.override_allow_mirror && has_prev_allow_mirror) {
    mutable_it->second.allow_mirror = prev_allow_mirror;
  }
  if (!adapted.ok) {
    result.error = adapted.error;
    return result;
  }

  result.ok = true;
  result.value.pole_ids = adapted.value.generated_pole_ids;
  result.value.span_ids = adapted.value.generated_span_ids;
  result.value.bundle_id = adapted.value.bundle_id;
  result.value.lane_assignments = state.view().last_lane_assignments();
  result.value.direction_debug = state.view().last_path_direction_debug();
  if (!adapted.value.generated_span_ids.empty()) {
    const auto* span = state.view().edit_state().spans.find(adapted.value.generated_span_ids.front());
    if (span != nullptr) {
      result.value.generation_session_id = span->generation.generation_session_id;
    }
  }
  result.change_set = adapted.change_set;
  return result;
}

LaneOrderMetrics compute_lane_order_metrics(const CoreState& state,
                                            const std::vector<wire::core::SegmentLaneAssignment>& assignments) {
  LaneOrderMetrics metrics{};
  auto to_layout_local = [&](const wire::core::Pole& pole, const wire::core::Vec3d& world) {
    double layout_yaw_deg = effective_pole_yaw_deg_test(pole);
    if (const auto pole_view = state.view().inspect_pole(pole.id); pole_view.has_value() && pole_view->has_layout_yaw) {
      layout_yaw_deg = pole_view->layout_yaw_deg;
    }
    return wire::core::WorldPointToLocal(
        wire::core::BuildPoleFrame(pole.world_transform, layout_yaw_deg), world);
  };
  for (const auto& assignment : assignments) {
    const auto* pole_a = state.view().edit_state().poles.find(assignment.pole_a_id);
    const auto* pole_b = state.view().edit_state().poles.find(assignment.pole_b_id);
    if (pole_a == nullptr || pole_b == nullptr) {
      continue;
    }
    const std::size_t lane_count = std::min(assignment.port_ids_a.size(), assignment.port_ids_b.size());
    if (lane_count < 2) {
      continue;
    }

    std::vector<double> y_a(lane_count, 0.0);
    std::vector<double> y_b(lane_count, 0.0);
    std::vector<double> z_a(lane_count, 0.0);
    std::vector<double> z_b(lane_count, 0.0);
    std::vector<int> layer_a(lane_count, 0);
    std::vector<int> layer_b(lane_count, 0);
    double y_sign_b = 1.0;
    const auto pole_view_a = state.view().inspect_pole(assignment.pole_a_id);
    const auto pole_view_b = state.view().inspect_pole(assignment.pole_b_id);
    if (pole_view_a.has_value() && pole_view_b.has_value() && pole_view_a->has_support_axis && pole_view_b->has_support_axis) {
      const wire::core::Vec3d axis_a = normalize_xy_safe(pole_view_a->support_axis_dir);
      const wire::core::Vec3d axis_b = normalize_xy_safe(pole_view_b->support_axis_dir);
      if (dot_xy(axis_a, axis_b) < 0.0) {
        y_sign_b = -1.0;
      }
    }

    for (std::size_t lane = 0; lane < lane_count; ++lane) {
      const auto* port_a = state.view().edit_state().ports.find(assignment.port_ids_a[lane]);
      const auto* port_b = state.view().edit_state().ports.find(assignment.port_ids_b[lane]);
      if (port_a == nullptr || port_b == nullptr) {
        continue;
      }
      const wire::core::Vec3d local_a = to_layout_local(*pole_a, port_a->world_position);
      const wire::core::Vec3d local_b = to_layout_local(*pole_b, port_b->world_position);
      y_a[lane] = local_a.y;
      y_b[lane] = local_b.y * y_sign_b;
      z_a[lane] = port_a->world_position.z;
      z_b[lane] = port_b->world_position.z;
      layer_a[lane] = port_a->template_layer;
      layer_b[lane] = port_b->template_layer;
      metrics.layer_jumps += std::abs(layer_a[lane] - layer_b[lane]);
    }

    for (std::size_t i = 0; i < lane_count; ++i) {
      for (std::size_t j = i + 1; j < lane_count; ++j) {
        const double dy_a = y_a[i] - y_a[j];
        const double dy_b = y_b[i] - y_b[j];
        constexpr double kOrderEps = 1e-4;
        if ((dy_a > kOrderEps && dy_b < -kOrderEps) || (dy_a < -kOrderEps && dy_b > kOrderEps)) {
          ++metrics.y_inversions;
        }
        const double dz_a = z_a[i] - z_a[j];
        const double dz_b = z_b[i] - z_b[j];
        if ((dz_a > kOrderEps && dz_b < -kOrderEps) || (dz_a < -kOrderEps && dz_b > kOrderEps)) {
          ++metrics.z_inversions;
        }
      }
    }
  }
  return metrics;
}

void dump_lane_assignment_debug(const CoreState& state,
                                const std::vector<wire::core::SegmentLaneAssignment>& assignments, const char* tag) {
  std::cerr << "[DBG] " << tag << " assignment_count=" << assignments.size() << "\n";
  auto to_layout_local = [&](const wire::core::Pole& pole, const wire::core::Vec3d& world) {
    double layout_yaw_deg = effective_pole_yaw_deg_test(pole);
    if (const auto pole_view = state.view().inspect_pole(pole.id); pole_view.has_value() && pole_view->has_layout_yaw) {
      layout_yaw_deg = pole_view->layout_yaw_deg;
    }
    return wire::core::WorldPointToLocal(
        wire::core::BuildPoleFrame(pole.world_transform, layout_yaw_deg), world);
  };
  for (const auto& assignment : assignments) {
    const auto* pole_a = state.view().edit_state().poles.find(assignment.pole_a_id);
    const auto* pole_b = state.view().edit_state().poles.find(assignment.pole_b_id);
    if (pole_a == nullptr || pole_b == nullptr) {
      continue;
    }
    std::vector<double> y_a{};
    std::vector<double> y_b{};
    double y_sign_b = 1.0;
    const auto pole_view_a = state.view().inspect_pole(assignment.pole_a_id);
    const auto pole_view_b = state.view().inspect_pole(assignment.pole_b_id);
    if (pole_view_a.has_value() && pole_view_b.has_value() && pole_view_a->has_support_axis && pole_view_b->has_support_axis) {
      const wire::core::Vec3d axis_a = normalize_xy_safe(pole_view_a->support_axis_dir);
      const wire::core::Vec3d axis_b = normalize_xy_safe(pole_view_b->support_axis_dir);
      if (dot_xy(axis_a, axis_b) < 0.0) {
        y_sign_b = -1.0;
      }
    }
    const std::size_t lane_count = std::min(assignment.port_ids_a.size(), assignment.port_ids_b.size());
    for (std::size_t lane = 0; lane < lane_count; ++lane) {
      const auto* pa = state.view().edit_state().ports.find(assignment.port_ids_a[lane]);
      const auto* pb = state.view().edit_state().ports.find(assignment.port_ids_b[lane]);
      if (pa == nullptr || pb == nullptr) {
        y_a.push_back(0.0);
        y_b.push_back(0.0);
        continue;
      }
      y_a.push_back(to_layout_local(*pole_a, pa->world_position).y);
      y_b.push_back(to_layout_local(*pole_b, pb->world_position).y * y_sign_b);
    }
    int inv = 0;
    constexpr double kOrderEps = 1e-4;
    for (std::size_t i = 0; i < lane_count; ++i) {
      for (std::size_t j = i + 1; j < lane_count; ++j) {
        const double dy_a = y_a[i] - y_a[j];
        const double dy_b = y_b[i] - y_b[j];
        if ((dy_a > kOrderEps && dy_b < -kOrderEps) || (dy_a < -kOrderEps && dy_b > kOrderEps)) {
          ++inv;
        }
      }
    }
    const bool orientation_reversed = assignment.bundle_order_choice_a != assignment.bundle_order_choice_b;
    std::cerr << "[DBG] " << tag << " seg=" << assignment.segment_index << " inv=" << inv
              << " orientationReversed=" << (orientation_reversed ? 1 : 0) << " yA=";
    for (double v : y_a) {
      std::cerr << v << ",";
    }
    std::cerr << " yB=";
    for (double v : y_b) {
      std::cerr << v << ",";
    }
    std::cerr << "\n";
  }
}

static double orient2d_xy_test(const wire::core::Vec3d& a, const wire::core::Vec3d& b, const wire::core::Vec3d& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

static bool xy_bbox_overlap_test(const wire::core::Vec3d& a, const wire::core::Vec3d& b, const wire::core::Vec3d& c,
                                 const wire::core::Vec3d& d) {
  const double min_ax = std::min(a.x, b.x);
  const double max_ax = std::max(a.x, b.x);
  const double min_ay = std::min(a.y, b.y);
  const double max_ay = std::max(a.y, b.y);
  const double min_cx = std::min(c.x, d.x);
  const double max_cx = std::max(c.x, d.x);
  const double min_cy = std::min(c.y, d.y);
  const double max_cy = std::max(c.y, d.y);
  return !(max_ax < min_cx || max_cx < min_ax || max_ay < min_cy || max_cy < min_ay);
}

static bool segments_intersect_xy_strict_test(const wire::core::Vec3d& a, const wire::core::Vec3d& b,
                                              const wire::core::Vec3d& c, const wire::core::Vec3d& d) {
  if (!xy_bbox_overlap_test(a, b, c, d)) {
    return false;
  }
  constexpr double kEps = 1e-9;
  const double o1 = orient2d_xy_test(a, b, c);
  const double o2 = orient2d_xy_test(a, b, d);
  const double o3 = orient2d_xy_test(c, d, a);
  const double o4 = orient2d_xy_test(c, d, b);
  const bool ab_straddle_cd = ((o1 > kEps && o2 < -kEps) || (o1 < -kEps && o2 > kEps));
  const bool cd_straddle_ab = ((o3 > kEps && o4 < -kEps) || (o3 < -kEps && o4 > kEps));
  return ab_straddle_cd && cd_straddle_ab;
}

int count_lane_segment_xy_intersections(const CoreState& state,
                                        const std::vector<wire::core::SegmentLaneAssignment>& assignments) {
  int intersections = 0;
  for (const auto& assignment : assignments) {
    const std::size_t lane_count = std::min(assignment.port_ids_a.size(), assignment.port_ids_b.size());
    if (lane_count < 2) {
      continue;
    }
    for (std::size_t i = 0; i < lane_count; ++i) {
      const auto* a0 = state.view().edit_state().ports.find(assignment.port_ids_a[i]);
      const auto* a1 = state.view().edit_state().ports.find(assignment.port_ids_b[i]);
      if (a0 == nullptr || a1 == nullptr) {
        continue;
      }
      for (std::size_t j = i + 1; j < lane_count; ++j) {
        const auto* b0 = state.view().edit_state().ports.find(assignment.port_ids_a[j]);
        const auto* b1 = state.view().edit_state().ports.find(assignment.port_ids_b[j]);
        if (b0 == nullptr || b1 == nullptr) {
          continue;
        }
        if (segments_intersect_xy_strict_test(a0->world_position, a1->world_position, b0->world_position,
                                              b1->world_position)) {
          ++intersections;
        }
      }
    }
  }
  return intersections;
}

int count_bundle_lane_polyline_xy_intersections(const CoreState& state,
                                                const std::vector<wire::core::SegmentLaneAssignment>& assignments) {
  std::unordered_map<ObjectId, std::vector<const wire::core::SegmentLaneAssignment*>> by_bundle{};
  for (const auto& assignment : assignments) {
    by_bundle[assignment.bundle_id].push_back(&assignment);
  }

  int intersections = 0;
  for (auto& [_, bundle_assignments] : by_bundle) {
    if (bundle_assignments.empty()) {
      continue;
    }
    std::sort(bundle_assignments.begin(), bundle_assignments.end(),
              [](const wire::core::SegmentLaneAssignment* a, const wire::core::SegmentLaneAssignment* b) {
                if (a->segment_index != b->segment_index) {
                  return a->segment_index < b->segment_index;
                }
                if (a->pole_a_id != b->pole_a_id) {
                  return a->pole_a_id < b->pole_a_id;
                }
                return a->pole_b_id < b->pole_b_id;
              });

    std::size_t lane_count = std::numeric_limits<std::size_t>::max();
    for (const auto* assignment : bundle_assignments) {
      lane_count = std::min(lane_count, std::min(assignment->port_ids_a.size(), assignment->port_ids_b.size()));
    }
    if (lane_count == std::numeric_limits<std::size_t>::max() || lane_count < 2) {
      continue;
    }

    std::vector<std::vector<std::pair<wire::core::Vec3d, wire::core::Vec3d>>> lane_segments(lane_count);
    for (const auto* assignment : bundle_assignments) {
      const std::size_t assignment_lane_count =
          std::min(lane_count, std::min(assignment->port_ids_a.size(), assignment->port_ids_b.size()));
      for (std::size_t lane = 0; lane < assignment_lane_count; ++lane) {
        const auto* a = state.view().edit_state().ports.find(assignment->port_ids_a[lane]);
        const auto* b = state.view().edit_state().ports.find(assignment->port_ids_b[lane]);
        if (a == nullptr || b == nullptr) {
          continue;
        }
        lane_segments[lane].push_back({a->world_position, b->world_position});
      }
    }

    for (std::size_t i = 0; i < lane_count; ++i) {
      for (std::size_t j = i + 1; j < lane_count; ++j) {
        for (const auto& seg_i : lane_segments[i]) {
          for (const auto& seg_j : lane_segments[j]) {
            if (segments_intersect_xy_strict_test(seg_i.first, seg_i.second, seg_j.first, seg_j.second)) {
              ++intersections;
            }
          }
        }
      }
    }
  }
  return intersections;
}

int count_bundle_lane_adjacent_order_discontinuities(const CoreState& state,
                                                     const std::vector<wire::core::SegmentLaneAssignment>& assignments) {
  std::unordered_map<ObjectId, std::vector<const wire::core::SegmentLaneAssignment*>> by_bundle{};
  for (const auto& assignment : assignments) {
    by_bundle[assignment.bundle_id].push_back(&assignment);
  }

  int discontinuities = 0;
  for (auto& [_, bundle_assignments] : by_bundle) {
    if (bundle_assignments.empty()) {
      continue;
    }
    std::sort(bundle_assignments.begin(), bundle_assignments.end(),
              [](const wire::core::SegmentLaneAssignment* a, const wire::core::SegmentLaneAssignment* b) {
                if (a->segment_index != b->segment_index) {
                  return a->segment_index < b->segment_index;
                }
                if (a->pole_a_id != b->pole_a_id) {
                  return a->pole_a_id < b->pole_a_id;
                }
                return a->pole_b_id < b->pole_b_id;
              });

    std::size_t min_segment_index = std::numeric_limits<std::size_t>::max();
    std::size_t max_segment_index = 0;
    for (const auto* assignment : bundle_assignments) {
      min_segment_index = std::min(min_segment_index, assignment->segment_index);
      max_segment_index = std::max(max_segment_index, assignment->segment_index);
    }

    auto to_layout_local = [&](const wire::core::Pole& pole, const wire::core::Vec3d& world) {
      double layout_yaw_deg = effective_pole_yaw_deg_test(pole);
      if (const auto pole_view = state.view().inspect_pole(pole.id); pole_view.has_value() && pole_view->has_layout_yaw) {
        layout_yaw_deg = pole_view->layout_yaw_deg;
      }
      return wire::core::WorldPointToLocal(wire::core::BuildPoleFrame(pole.world_transform, layout_yaw_deg), world);
    };

    for (std::size_t idx = 1; idx < bundle_assignments.size(); ++idx) {
      const auto* prev = bundle_assignments[idx - 1];
      const auto* curr = bundle_assignments[idx];
      if (prev->segment_index == min_segment_index || curr->segment_index == max_segment_index) {
        // Terminal fan-out is covered by C185. Here we focus on interior shared-pole order continuity.
        continue;
      }
      if (prev->segment_index + 1 != curr->segment_index || prev->pole_b_id != curr->pole_a_id) {
        continue;
      }

      const auto* shared_pole = state.view().edit_state().poles.find(curr->pole_a_id);
      if (shared_pole == nullptr) {
        continue;
      }

      const std::size_t lane_count = std::min(prev->port_ids_b.size(), curr->port_ids_a.size());
      std::vector<double> prev_y(lane_count, 0.0);
      std::vector<double> curr_y(lane_count, 0.0);
      for (std::size_t i = 0; i < lane_count; ++i) {
        const auto* prev_port = state.view().edit_state().ports.find(prev->port_ids_b[i]);
        const auto* curr_port = state.view().edit_state().ports.find(curr->port_ids_a[i]);
        if (prev_port == nullptr || curr_port == nullptr) {
          continue;
        }
        prev_y[i] = to_layout_local(*shared_pole, prev_port->world_position).y;
        curr_y[i] = to_layout_local(*shared_pole, curr_port->world_position).y;
      }

      constexpr double kOrderEps = 1e-4;
      for (std::size_t i = 0; i < lane_count; ++i) {
        for (std::size_t j = i + 1; j < lane_count; ++j) {
          const double dy_prev = prev_y[i] - prev_y[j];
          const double dy_curr = curr_y[i] - curr_y[j];
          if ((dy_prev > kOrderEps && dy_curr < -kOrderEps) || (dy_prev < -kOrderEps && dy_curr > kOrderEps)) {
            ++discontinuities;
          }
        }
      }
    }
  }
  return discontinuities;
}

int count_mirrored_assignments(const std::vector<wire::core::SegmentLaneAssignment>& assignments) {
  int mirrored = 0;
  for (const auto& assignment : assignments) {
    if (assignment.bundle_order_choice_a != assignment.bundle_order_choice_b) {
      ++mirrored;
    }
  }
  return mirrored;
}

const wire::core::JunctionInfo* find_junction(const wire::core::BackboneResult& backbone, ObjectId node_id) {
  for (const auto& junction : backbone.junctions) {
    if (junction.node_id == node_id) {
      return &junction;
    }
  }
  return nullptr;
}

const wire::core::SupportNode* find_support_node_by_point_index(const wire::core::BackboneResult& backbone,
                                                                int point_index) {
  for (const auto& node : backbone.nodes) {
    if (node.path_point_index == point_index) {
      return &node;
    }
  }
  return nullptr;
}

ObjectId find_pole_id_by_position(const CoreState& state, const wire::core::Vec3d& pos, double eps) {
  for (const auto& pole : state.view().poles().items()) {
    if (std::abs(pole.world_transform.position.x - pos.x) <= eps &&
        std::abs(pole.world_transform.position.y - pos.y) <= eps &&
        std::abs(pole.world_transform.position.z - pos.z) <= eps) {
      return pole.id;
    }
  }
  return wire::core::kInvalidObjectId;
}

bool is_monotonic(const std::vector<double>& values) {
  if (values.size() < 2) {
    return true;
  }
  bool non_decreasing = true;
  bool non_increasing = true;
  for (std::size_t i = 1; i < values.size(); ++i) {
    if (values[i] + 1e-9 < values[i - 1]) {
      non_decreasing = false;
    }
    if (values[i] > values[i - 1] + 1e-9) {
      non_increasing = false;
    }
  }
  return non_decreasing || non_increasing;
}

void add_backbone_bundle(wire::core::BackboneSpec& req, wire::core::BundleKind template_id,
                         wire::core::SpanLayer layer, int count) {
  wire::core::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = template_id;
  bundle.layer = layer;
  bundle.count = count;
  req.bundles.push_back(bundle);
}

wire::core::BundleKind bundle_template_for_category_test(wire::core::ConnectionCategory category) {
  switch (category) {
  case wire::core::ConnectionCategory::kHighVoltage:
    return wire::core::BundleKind::kHighVoltage;
  case wire::core::ConnectionCategory::kCommunication:
    return wire::core::BundleKind::kCommunication;
  case wire::core::ConnectionCategory::kOptical:
    return wire::core::BundleKind::kOptical;
  case wire::core::ConnectionCategory::kLowVoltage:
  case wire::core::ConnectionCategory::kDrop:
  default:
    return wire::core::BundleKind::kLowVoltage;
  }
}

AxisRelationMetrics measure_pole_axis_relation_metrics(const CoreState& state, ObjectId pole_id, wire::core::PortLayer layer,
                                                       const wire::core::Vec3d& span_axis) {
  AxisRelationMetrics metrics{};
  metrics.span_chord_axis = normalize_xy_safe(span_axis);
  const auto pole_view = state.view().inspect_pole(pole_id);
  if (!pole_view.has_value()) {
    return metrics;
  }

  std::vector<wire::core::Vec3d> port_points{};
  for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
    if (port.owner_pole_id == pole_id && port.layer == layer &&
        port.placement_source != wire::core::PortPlacementSourceKind::kBranchSupport) {
      port_points.push_back(port.world_position);
    }
  }
  metrics.row_axis = farthest_pair_axis_xy(port_points);
  if (length_xy(metrics.row_axis) <= 1e-9 && pole_view->has_support_axis) {
    metrics.row_axis = normalize_xy_safe(pole_view->support_axis_dir);
  }

  if (pole_view->has_forward) {
    metrics.support_forward_axis = normalize_xy_safe(pole_view->forward_dir);
  }
  if (length_xy(metrics.support_forward_axis) <= 1e-9 && length_xy(metrics.row_axis) > 1e-9) {
    metrics.support_forward_axis = normalize_xy_safe({metrics.row_axis.y, -metrics.row_axis.x, 0.0});
  }

  metrics.angle_row_vs_span_deg = axis_angle_deg(metrics.row_axis, metrics.span_chord_axis);
  metrics.angle_forward_vs_span_deg = axis_angle_deg(metrics.support_forward_axis, metrics.span_chord_axis);
  metrics.valid = length_xy(metrics.row_axis) > 1e-9 && length_xy(metrics.span_chord_axis) > 1e-9;
  return metrics;
}

VisualSeparationMetrics measure_lane_visual_separation_metrics(const CoreState& state,
                                                               const wire::core::SegmentLaneAssignment& assignment,
                                                               double sample_length_m) {
  VisualSeparationMetrics metrics{};
  const std::size_t lane_count = std::min(assignment.port_ids_a.size(), assignment.port_ids_b.size());
  metrics.port_count = static_cast<int>(lane_count);
  if (lane_count < 2) {
    return metrics;
  }

  std::vector<wire::core::Vec3d> start_ports{};
  std::vector<wire::core::Vec3d> end_ports{};
  std::vector<wire::core::Vec3d> start_endpoints{};
  std::vector<wire::core::Vec3d> end_endpoints{};
  std::vector<wire::core::Vec3d> near_start_points{};
  std::vector<wire::core::Vec3d> near_end_points{};
  std::unordered_set<ObjectId> unique_start_ports{};
  std::unordered_set<ObjectId> unique_end_ports{};

  for (std::size_t lane = 0; lane < lane_count; ++lane) {
    const ObjectId port_a_id = assignment.port_ids_a[lane];
    const ObjectId port_b_id = assignment.port_ids_b[lane];
    unique_start_ports.insert(port_a_id);
    unique_end_ports.insert(port_b_id);

    const wire::core::Port* port_a = state.view().edit_state().ports.find(port_a_id);
    const wire::core::Port* port_b = state.view().edit_state().ports.find(port_b_id);
    if (port_a == nullptr || port_b == nullptr) {
      continue;
    }
    start_ports.push_back(port_a->world_position);
    end_ports.push_back(port_b->world_position);

    const wire::core::Span* span = find_span_by_ports(state, port_a_id, port_b_id);
    if (span == nullptr) {
      continue;
    }
    const bool forward_matches_assignment = (span->port_a_id == port_a_id && span->port_b_id == port_b_id);
    const wire::core::SpanSupportLayoutEntry* layout = state.find_span_support_layout(span->id);
    if (layout != nullptr) {
      start_endpoints.push_back(forward_matches_assignment ? layout->start.endpoint_world : layout->end.endpoint_world);
      end_endpoints.push_back(forward_matches_assignment ? layout->end.endpoint_world : layout->start.endpoint_world);
    }
    const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span->id);
    if (curve != nullptr) {
      const double total = std::max(0.0, curve->detail.Length());
      const double s = std::min(std::max(0.05, sample_length_m), std::max(0.05, total * 0.25));
      if (forward_matches_assignment) {
        near_start_points.push_back(curve->detail.PositionAtLength(std::min(s, total)));
        near_end_points.push_back(curve->detail.PositionAtLength(std::max(0.0, total - s)));
      } else {
        near_start_points.push_back(curve->detail.PositionAtLength(std::max(0.0, total - s)));
        near_end_points.push_back(curve->detail.PositionAtLength(std::min(s, total)));
      }
    } else {
      near_start_points.push_back(port_a->world_position);
      near_end_points.push_back(port_b->world_position);
    }
  }

  const double start_port_spacing = min_pairwise_distance3d(start_ports);
  const double end_port_spacing = min_pairwise_distance3d(end_ports);
  metrics.min_port_spacing_m = std::min(start_port_spacing, end_port_spacing);
  const double start_endpoint_spacing = min_pairwise_distance3d(start_endpoints);
  const double end_endpoint_spacing = min_pairwise_distance3d(end_endpoints);
  if (!start_endpoints.empty() && !end_endpoints.empty()) {
    metrics.min_endpoint_spacing_m = std::min(start_endpoint_spacing, end_endpoint_spacing);
  } else {
    metrics.min_endpoint_spacing_m = std::max(start_endpoint_spacing, end_endpoint_spacing);
  }
  metrics.min_wire_spacing_near_start_m = min_pairwise_distance3d(near_start_points);
  metrics.min_wire_spacing_near_end_m = min_pairwise_distance3d(near_end_points);
  const double proj_start_min = min_pairwise_distance_xy(near_start_points);
  const double proj_end_min = min_pairwise_distance_xy(near_end_points);
  const double proj_start_mean = mean_pairwise_distance_xy(near_start_points);
  const double proj_end_mean = mean_pairwise_distance_xy(near_end_points);
  metrics.projected_min_spacing_topview_m = std::min(proj_start_min, proj_end_min);
  metrics.projected_mean_spacing_topview_m = 0.5 * (proj_start_mean + proj_end_mean);
  metrics.topology_distinct = unique_start_ports.size() == lane_count && unique_end_ports.size() == lane_count;

  const double spacing_floor = std::min(
      {metrics.projected_min_spacing_topview_m, metrics.min_wire_spacing_near_start_m, metrics.min_wire_spacing_near_end_m});
  metrics.visual_separation_score = std::clamp(spacing_floor / 0.12, 0.0, 1.0);
  metrics.visual_distinct = metrics.topology_distinct && metrics.projected_min_spacing_topview_m >= 0.10 &&
                            metrics.min_wire_spacing_near_start_m >= 0.10 && metrics.min_wire_spacing_near_end_m >= 0.10;
  return metrics;
}

BranchRunoutMetrics measure_branch_runout_metrics(const CoreState& state, ObjectId span_id) {
  BranchRunoutMetrics metrics{};
  const wire::core::SpanSupportLayoutEntry* layout = state.find_span_support_layout(span_id);
  const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  if (layout == nullptr || curve == nullptr) {
    return metrics;
  }

  const bool branch_at_start = layout->start.origin == wire::core::SupportLayoutOriginKind::kBranchSupport ||
                               layout->start.flow_kind == wire::core::BackboneFlowKind::kBranch ||
                               layout->start.local_departure_length_m >= layout->end.local_departure_length_m;
  const wire::core::Vec3d root = branch_at_start ? curve->detail.EvaluatePosition(0.0) : curve->detail.EvaluatePosition(1.0);
  const wire::core::Vec3d other =
      branch_at_start ? curve->detail.EvaluatePosition(1.0) : curve->detail.EvaluatePosition(0.0);
  const wire::core::Vec3d chord_axis = normalize_xy_safe(other - root);
  if (length_xy(chord_axis) <= 1e-9) {
    return metrics;
  }
  const wire::core::Vec3d lateral_axis{-chord_axis.y, chord_axis.x, 0.0};
  const double total = curve->detail.Length();
  metrics.chord_length_m = std::sqrt((other.x - root.x) * (other.x - root.x) + (other.y - root.y) * (other.y - root.y));
  metrics.support_departure_length_m =
      branch_at_start ? layout->start.local_departure_length_m : layout->end.local_departure_length_m;

  for (const wire::core::Vec3d& point : curve->detail.sample_points) {
    const double lateral = std::abs(dot_xy(point - root, lateral_axis));
    metrics.max_lateral_runout_m = std::max(metrics.max_lateral_runout_m, lateral);
  }

  const double departure_s = std::min(std::max(0.05, metrics.support_departure_length_m), std::max(0.05, total));
  const double mid_s = std::clamp(total * 0.5, 0.0, std::max(0.0, total));
  const wire::core::Vec3d departure_point =
      branch_at_start ? curve->detail.PositionAtLength(departure_s) : curve->detail.PositionAtLength(std::max(0.0, total - departure_s));
  const wire::core::Vec3d mid_point = curve->detail.PositionAtLength(mid_s);
  metrics.departure_lateral_offset_m = std::abs(dot_xy(departure_point - root, lateral_axis));
  metrics.midspan_lateral_offset_m = std::abs(dot_xy(mid_point - root, lateral_axis));
  metrics.lateral_runout_ratio =
      (metrics.chord_length_m > 1e-9) ? (metrics.max_lateral_runout_m / metrics.chord_length_m) : 0.0;
  metrics.local_departure_dominates =
      metrics.midspan_lateral_offset_m <= metrics.departure_lateral_offset_m + 0.05 &&
      metrics.max_lateral_runout_m <= std::max(metrics.support_departure_length_m + 0.05,
                                               metrics.departure_lateral_offset_m + 0.08);
  return metrics;
}

std::string describe_axis_relation_metrics(const AxisRelationMetrics& metrics) {
  std::ostringstream oss;
  oss << "rowAxis=" << metrics.row_axis.x << "," << metrics.row_axis.y << " forwardAxis="
      << metrics.support_forward_axis.x << "," << metrics.support_forward_axis.y << " spanAxis="
      << metrics.span_chord_axis.x << "," << metrics.span_chord_axis.y << " angleRowVsSpan="
      << metrics.angle_row_vs_span_deg << " angleForwardVsSpan=" << metrics.angle_forward_vs_span_deg
      << " valid=" << (metrics.valid ? 1 : 0);
  return oss.str();
}

std::string describe_visual_separation_metrics(const VisualSeparationMetrics& metrics) {
  std::ostringstream oss;
  oss << "ports=" << metrics.port_count << " minPort=" << metrics.min_port_spacing_m
      << " minEndpoint=" << metrics.min_endpoint_spacing_m << " nearStart=" << metrics.min_wire_spacing_near_start_m
      << " nearEnd=" << metrics.min_wire_spacing_near_end_m << " topMin=" << metrics.projected_min_spacing_topview_m
      << " topMean=" << metrics.projected_mean_spacing_topview_m << " score=" << metrics.visual_separation_score
      << " topologyDistinct=" << (metrics.topology_distinct ? 1 : 0)
      << " visualDistinct=" << (metrics.visual_distinct ? 1 : 0);
  return oss.str();
}

std::string describe_branch_runout_metrics(const BranchRunoutMetrics& metrics) {
  std::ostringstream oss;
  oss << "maxLat=" << metrics.max_lateral_runout_m << " ratio=" << metrics.lateral_runout_ratio
      << " depLat=" << metrics.departure_lateral_offset_m << " midLat=" << metrics.midspan_lateral_offset_m
      << " depLen=" << metrics.support_departure_length_m << " chord=" << metrics.chord_length_m
      << " local=" << (metrics.local_departure_dominates ? 1 : 0);
  return oss.str();
}

wire::core::EditResult<wire::core::CoreState::AddConnectionByPoleResult>
add_connection_by_category(wire::core::CoreState& state, wire::core::ObjectId pole_a_id, wire::core::ObjectId pole_b_id,
                           wire::core::ConnectionCategory category,
                           wire::core::AddConnectionByPoleOptions options) {
  if (!options.use_bundle_template) {
    options.bundle_template_id = bundle_template_for_category_test(category);
    options.use_bundle_template = true;
  }
  return state.AddConnectionByPole(pole_a_id, pole_b_id, category, options);
}

bool has_selected_port_in_candidates(const wire::core::PortResolutionDebugRecord& record) {
  if (record.selected_port_id == wire::core::kInvalidObjectId) {
    return true;
  }
  if (record.created_new_port) {
    return true;
  }
  for (const auto& candidate : record.candidates) {
    if (candidate.resolved_port_id == record.selected_port_id) {
      return true;
    }
  }
  return false;
}

} // namespace helpers



#include "wire/core/coord_utils.hpp"
