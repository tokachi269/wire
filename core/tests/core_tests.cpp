#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include "wire/core/core_state.hpp"

namespace {

using wire::core::ConnectionCategory;
using wire::core::CoreState;
using wire::core::DirtyBits;
using wire::core::ObjectId;
using wire::core::PoleTypeId;
using wire::core::PortKind;
using wire::core::PortLayer;
using wire::core::SpanKind;
using wire::core::SpanLayer;

struct TestCase {
  const char* name;
  const char* intent;
  const char* oracle;
  bool abnormal;
  std::function<bool(void)> run;
};

struct CoreCounts {
  std::size_t poles = 0;
  std::size_t ports = 0;
  std::size_t anchors = 0;
  std::size_t bundles = 0;
  std::size_t spans = 0;
  std::size_t attachments = 0;
};

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

bool almost_equal(double a, double b, double eps = 1e-9) { return std::abs(a - b) <= eps; }

bool almost_equal(const wire::core::Vec3d& a, const wire::core::Vec3d& b, double eps = 1e-9) {
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
  return {
      local.x * c - local.y * s,
      local.x * s + local.y * c,
      local.z,
  };
}

double effective_pole_yaw_deg_test(const wire::core::Pole& pole) {
  double yaw = pole.world_transform.rotation_euler_deg.z;
  if (pole.orientation_control.manual_yaw_override) {
    yaw = pole.orientation_control.manual_yaw_deg;
  }
  if (pole.orientation_control.flip_180) {
    yaw += 180.0;
  }
  return yaw;
}

wire::core::Vec3d to_local_on_pole_test(const wire::core::Pole& pole, const wire::core::Vec3d& world) {
  const wire::core::Vec3d delta = world - pole.world_transform.position;
  return rotate_xy_by_yaw_test(delta, -effective_pole_yaw_deg_test(pole));
}

bool aabb_valid(const wire::core::AABBd& aabb) {
  return aabb.min.x <= aabb.max.x && aabb.min.y <= aabb.max.y && aabb.min.z <= aabb.max.z;
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

struct LaneOrderMetrics {
  int y_inversions = 0;
  int z_inversions = 0;
  int layer_jumps = 0;

  [[nodiscard]] int weighted_score() const {
    return y_inversions * 1000 + z_inversions * 600 + layer_jumps * 30;
  }
};

LaneOrderMetrics compute_lane_order_metrics(const CoreState& state,
                                            const std::vector<wire::core::SegmentLaneAssignment>& assignments) {
  LaneOrderMetrics metrics{};
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

    for (std::size_t lane = 0; lane < lane_count; ++lane) {
      const auto* port_a = state.view().edit_state().ports.find(assignment.port_ids_a[lane]);
      const auto* port_b = state.view().edit_state().ports.find(assignment.port_ids_b[lane]);
      if (port_a == nullptr || port_b == nullptr) {
        continue;
      }
      y_a[lane] = to_local_on_pole_test(*pole_a, port_a->world_position).y;
      y_b[lane] = to_local_on_pole_test(*pole_b, port_b->world_position).y;
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
        if (dy_a * dy_b < -1e-9) {
          ++metrics.y_inversions;
        }
        const double dz_a = z_a[i] - z_a[j];
        const double dz_b = z_b[i] - z_b[j];
        if (dz_a * dz_b < -1e-9) {
          ++metrics.z_inversions;
        }
      }
    }
  }
  return metrics;
}

double orient2d_xy_test(const wire::core::Vec3d& a, const wire::core::Vec3d& b, const wire::core::Vec3d& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool xy_bbox_overlap_test(const wire::core::Vec3d& a, const wire::core::Vec3d& b, const wire::core::Vec3d& c,
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

bool segments_intersect_xy_strict_test(const wire::core::Vec3d& a, const wire::core::Vec3d& b,
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

bool has_selected_slot_in_candidates(const wire::core::SlotSelectionDebugRecord& record) {
  if (record.selected_slot_id < 0) {
    return true;
  }
  for (const auto& candidate : record.candidates) {
    if (candidate.slot_id == record.selected_slot_id) {
      return true;
    }
  }
  return false;
}

template <typename T> std::vector<ObjectId> collect_sorted_ids(const std::vector<T>& items) {
  std::vector<ObjectId> ids;
  ids.reserve(items.size());
  for (const auto& item : items) {
    ids.push_back(item.id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

// Intent: IdGenerator should be monotonic and resettable without collisions in one sequence.
bool test_id_generator_monotonic_and_reset() {
  wire::core::IdGenerator gen(1);
  if (gen.next() != 1 || gen.next() != 2 || gen.next() != 3) {
    return false;
  }
  if (gen.peek() != 4) {
    return false;
  }
  gen.reset(42);
  return gen.peek() == 42 && gen.next() == 42 && gen.next() == 43;
}

// Intent: ObjectStore should keep lookup integrity across insert/update/remove.
bool test_object_store_integrity() {
  struct Dummy {
    ObjectId id = 0;
    int payload = 0;
  };

  wire::core::ObjectStore<Dummy> store;
  store.insert(Dummy{1, 10});
  store.insert(Dummy{2, 20});
  store.insert(Dummy{3, 30});
  if (store.size() != 3 || !store.contains(2)) {
    return false;
  }
  if (store.find(3) == nullptr || store.find(3)->payload != 30) {
    return false;
  }
  if (!store.remove(2) || store.contains(2) || store.size() != 2) {
    return false;
  }
  if (store.find(1) == nullptr || store.find(3) == nullptr) {
    return false;
  }

  // Upsert must replace value without changing count.
  store.insert(Dummy{3, 99});
  return store.size() == 2 && store.find(3) != nullptr && store.find(3)->payload == 99;
}

// Intent: AddSpan should create runtime state and initial dirty bits.
bool test_span_runtime_initialized_on_add() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 9.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {4.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const auto span_result = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage);
  if (!span_result.ok) {
    return false;
  }

  const auto* runtime = state.view().find_span_runtime_state(span_result.value);
  return runtime != nullptr && runtime->span_id == span_result.value && runtime->data_version > 0 &&
         has_dirty(runtime, DirtyBits::kTopology | DirtyBits::kGeometry);
}

// Intent: MovePole should dirty only related spans.
bool test_move_pole_dirties_only_related_span() {
  CoreState state;
  const ObjectId pole_a = state.AddPole({}, 9.0, "A").value;
  const ObjectId pole_b = state.AddPole({}, 9.0, "B").value;

  const ObjectId a1 = state.AddPort(pole_a, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId a2 = state.AddPort(pole_a, {3.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b1 = state.AddPort(pole_b, {10.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b2 = state.AddPort(pole_b, {13.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;

  const ObjectId related = state.AddSpan(a1, a2, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  const ObjectId unrelated = state.AddSpan(b1, b2, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.Commit().recalc_stats;

  wire::core::Transformd moved{};
  moved.position = {2.0, 0.0, 0.0};
  const auto move_result = state.MovePole(pole_a, moved);
  if (!move_result.ok) {
    return false;
  }

  return has_dirty(state.view().find_span_runtime_state(related), DirtyBits::kGeometry) &&
         !has_dirty(state.view().find_span_runtime_state(unrelated), DirtyBits::kGeometry);
}

// Intent: SplitSpan should replace old span with two new spans and create split port.
bool test_split_span_creates_two_spans_and_port() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 9.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {10.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span_id = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.Commit().recalc_stats;

  const auto split_result = state.SplitSpan(span_id, 0.5);
  if (!split_result.ok) {
    return false;
  }

  if (state.view().edit_state().spans.find(span_id) != nullptr) {
    return false;
  }
  if (state.view().edit_state().ports.find(split_result.value.new_port_id) == nullptr) {
    return false;
  }
  if (state.view().edit_state().spans.find(split_result.value.new_span_a_id) == nullptr ||
      state.view().edit_state().spans.find(split_result.value.new_span_b_id) == nullptr) {
    return false;
  }
  return validate_now(state).ok();
}

// Intent: ApplyPoleType should create template ports owned by the pole.
bool test_apply_pole_type_generates_template_ports() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.size() < 2) {
    return false;
  }

  const ObjectId pole_id = state.AddPole({}, 10.0, "P").value;
  const auto apply_result = state.ApplyPoleType(pole_id, pole_type_ids[0]);
  if (!apply_result.ok) {
    return false;
  }

  const auto detail = state.GetPoleDetail(pole_id);
  if (detail.pole == nullptr || detail.pole_type == nullptr || detail.owned_ports.empty()) {
    return false;
  }
  for (const auto* port : detail.owned_ports) {
    if (port->owner_pole_id != pole_id) {
      return false;
    }
  }
  return true;
}

// Intent: Different PoleType should produce different port slot composition.
bool test_different_pole_types_produce_different_port_layouts() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.size() < 2) {
    return false;
  }

  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole({}, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, pole_type_ids[0]).ok) {
    return false;
  }
  if (!state.ApplyPoleType(pole_b, pole_type_ids[1]).ok) {
    return false;
  }

  const auto detail_a = state.GetPoleDetail(pole_a);
  const auto detail_b = state.GetPoleDetail(pole_b);
  if (detail_a.owned_ports.empty() || detail_b.owned_ports.empty()) {
    return false;
  }
  if (detail_a.owned_ports.size() != detail_b.owned_ports.size()) {
    return true;
  }

  std::vector<int> slots_a;
  std::vector<int> slots_b;
  for (const auto* port : detail_a.owned_ports)
    slots_a.push_back(port->source_slot_id);
  for (const auto* port : detail_b.owned_ports)
    slots_b.push_back(port->source_slot_id);
  std::sort(slots_a.begin(), slots_a.end());
  std::sort(slots_b.begin(), slots_b.end());
  return slots_a != slots_b;
}

// Intent: Auto allocation should consume unused slot first for repeated same-category connections.
bool test_auto_port_allocation_prefers_unused_slots() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {20.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, pole_type_ids[0]).ok || !state.ApplyPoleType(pole_b, pole_type_ids[0]).ok) {
    return false;
  }

  const auto first = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kHighVoltage);
  const auto second = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kHighVoltage);
  if (!first.ok || !second.ok) {
    return false;
  }
  return first.value.slot_a_id >= 0 && second.value.slot_a_id >= 0 && first.value.slot_a_id != second.value.slot_a_id;
}

// Intent: Pole->Pole connection should create span and keep dirty/index/change-set consistent.
bool test_add_connection_by_pole_updates_dirty_version_and_indices() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {16.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, pole_type_ids[0]);
  (void)state.ApplyPoleType(pole_b, pole_type_ids[0]);

  const auto connection = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage);
  if (!connection.ok) {
    return false;
  }

  const auto* span = state.view().edit_state().spans.find(connection.value.span_id);
  const auto* port_a = state.view().edit_state().ports.find(connection.value.port_a_id);
  const auto* port_b = state.view().edit_state().ports.find(connection.value.port_b_id);
  const auto* runtime = state.view().find_span_runtime_state(connection.value.span_id);
  if (span == nullptr || port_a == nullptr || port_b == nullptr || runtime == nullptr) {
    return false;
  }
  if (port_a->owner_pole_id != pole_a || port_b->owner_pole_id != pole_b) {
    return false;
  }

  auto it_a = state.view().connection_index().spans_by_port.find(port_a->id);
  auto it_b = state.view().connection_index().spans_by_port.find(port_b->id);
  if (it_a == state.view().connection_index().spans_by_port.end() || it_b == state.view().connection_index().spans_by_port.end()) {
    return false;
  }

  return contains_id(it_a->second, span->id) && contains_id(it_b->second, span->id) &&
         has_dirty(runtime, DirtyBits::kGeometry) && contains_id(connection.change_set.created_ids, span->id) &&
         contains_id(connection.change_set.dirty_span_ids, span->id);
}

// Intent: AddDropFromPole should create service span from source pole to free endpoint.
bool test_add_drop_from_pole_creates_service_connection() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  (void)state.ApplyPoleType(pole, pole_type_ids[0]);
  const auto drop = state.AddDropFromPole(pole, {5.0, 3.0, 2.0});
  if (!drop.ok) {
    return false;
  }

  const auto* span = state.view().edit_state().spans.find(drop.value.span_id);
  const auto* source_port = state.view().edit_state().ports.find(drop.value.source_port_id);
  const auto* target_port = state.view().edit_state().ports.find(drop.value.target_port_id);
  if (span == nullptr || source_port == nullptr || target_port == nullptr) {
    return false;
  }
  return source_port->owner_pole_id == pole && target_port->owner_pole_id == wire::core::kInvalidObjectId &&
         validate_now(state).ok();
}

// Intent: AddDropFromSpan should split source span then add drop span from split port.
bool test_add_drop_from_span_splits_and_connects_drop() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {20.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, pole_type_ids[0]);
  (void)state.ApplyPoleType(pole_b, pole_type_ids[0]);

  const auto base = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage);
  if (!base.ok) {
    return false;
  }

  const auto drop = state.AddDropFromSpan(base.value.span_id, 0.5, {10.0, 4.0, 2.0});
  if (!drop.ok) {
    return false;
  }

  if (state.view().edit_state().spans.find(base.value.span_id) != nullptr) {
    return false;
  }
  const auto* split_port = state.view().edit_state().ports.find(drop.value.split_port_id);
  const auto* drop_span = state.view().edit_state().spans.find(drop.value.span_id);
  if (split_port == nullptr || drop_span == nullptr) {
    return false;
  }
  auto split_it = state.view().connection_index().spans_by_port.find(drop.value.split_port_id);
  if (split_it == state.view().connection_index().spans_by_port.end()) {
    return false;
  }
  return split_it->second.size() >= 3 && validate_now(state).ok();
}

// Intent: Line mode curve cache should be generated deterministically for the same input.
bool test_curve_cache_line_mode_is_deterministic() {
  CoreState state;
  wire::core::GeometrySettings settings{};
  settings.curve_samples = 9;
  settings.sag_enabled = false;
  settings.sag_factor = 0.0;
  (void)state.UpdateGeometrySettings(settings, false);

  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 5.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {10.0, 0.0, 5.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.Commit().recalc_stats;

  const auto* curve1 = state.find_curve_cache(span);
  if (curve1 == nullptr || curve1->points.size() != 9) {
    return false;
  }

  const auto move_result = state.MovePort(a, {0.0, 0.0, 5.0});
  if (!move_result.ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto* curve2 = state.find_curve_cache(span);
  if (curve2 == nullptr || curve2->points.size() != curve1->points.size()) {
    return false;
  }

  if (!almost_equal(curve2->points.front(), {0.0, 0.0, 5.0}) ||
      !almost_equal(curve2->points.back(), {10.0, 0.0, 5.0})) {
    return false;
  }
  for (std::size_t i = 0; i < curve1->points.size(); ++i) {
    if (!almost_equal(curve1->points[i], curve2->points[i])) {
      return false;
    }
  }
  return true;
}

// Intent: Sag mode should change middle points while keeping endpoints fixed.
bool test_sag_mode_changes_midpoint_and_keeps_endpoints() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 4.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {12.0, 0.0, 4.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;

  wire::core::GeometrySettings line{};
  line.curve_samples = 11;
  line.sag_enabled = false;
  line.sag_factor = 0.05;
  (void)state.UpdateGeometrySettings(line, true);
  (void)state.Commit().recalc_stats;
  const auto* line_curve = state.find_curve_cache(span);
  if (line_curve == nullptr || line_curve->points.size() != 11) {
    return false;
  }
  const std::vector<wire::core::Vec3d> line_points = line_curve->points;

  wire::core::GeometrySettings sag = line;
  sag.sag_enabled = true;
  sag.sag_factor = 0.10;
  (void)state.UpdateGeometrySettings(sag, true);
  (void)state.Commit().recalc_stats;
  const auto* sag_curve = state.find_curve_cache(span);
  if (sag_curve == nullptr || sag_curve->points.size() != 11) {
    return false;
  }

  const std::size_t mid = sag_curve->points.size() / 2;
  return almost_equal(sag_curve->points.front(), line_points.front()) &&
         almost_equal(sag_curve->points.back(), line_points.back()) && sag_curve->points[mid].z < line_points[mid].z;
}

// Intent: Geometry dirty should rebuild only related span and propagate bounds/render versions.
bool test_geometry_bounds_version_follow_and_locality() {
  CoreState state;
  wire::core::GeometrySettings settings{};
  settings.curve_samples = 8;
  settings.sag_enabled = false;
  settings.sag_factor = 0.0;
  (void)state.UpdateGeometrySettings(settings, false);

  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId p1 = state.AddPort(pole, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId p2 = state.AddPort(pole, {5.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId p3 = state.AddPort(pole, {10.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId p4 = state.AddPort(pole, {15.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span_a = state.AddSpan(p1, p2, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  const ObjectId span_b = state.AddSpan(p3, p4, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.Commit().recalc_stats;

  const auto* before_b = state.view().find_span_runtime_state(span_b);
  if (before_b == nullptr) {
    return false;
  }
  const std::uint64_t span_b_geometry_before = before_b->geometry_version;

  const auto move_result = state.MovePort(p1, {0.0, 1.0, 1.0});
  if (!move_result.ok) {
    return false;
  }
  const auto* dirty_a = state.view().find_span_runtime_state(span_a);
  const auto* dirty_b = state.view().find_span_runtime_state(span_b);
  if (dirty_a == nullptr || dirty_b == nullptr) {
    return false;
  }
  if (!has_dirty(dirty_a, DirtyBits::kGeometry) || has_dirty(dirty_b, DirtyBits::kGeometry)) {
    return false;
  }

  (void)state.Commit().recalc_stats;
  const auto* after_a = state.view().find_span_runtime_state(span_a);
  const auto* after_b = state.view().find_span_runtime_state(span_b);
  if (after_a == nullptr || after_b == nullptr) {
    return false;
  }

  return after_a->geometry_version == after_a->data_version && after_a->bounds_version == after_a->data_version &&
         after_a->render_version == after_a->data_version && after_b->geometry_version == span_b_geometry_before;
}

// Intent: Bounds cache should be generated and valid from curve cache.
bool test_bounds_cache_generated_and_valid() {
  CoreState state;
  wire::core::GeometrySettings settings{};
  settings.curve_samples = 7;
  settings.sag_enabled = true;
  settings.sag_factor = 0.08;
  (void)state.UpdateGeometrySettings(settings, false);

  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 5.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {10.0, 2.0, 5.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.Commit().recalc_stats;

  const auto* curve = state.find_curve_cache(span);
  const auto* bounds = state.find_bounds_cache(span);
  if (curve == nullptr || bounds == nullptr) {
    return false;
  }
  if (curve->points.size() < 2 || bounds->segments.size() != curve->points.size() - 1) {
    return false;
  }
  if (!aabb_valid(bounds->whole)) {
    return false;
  }
  for (const auto& segment : bounds->segments) {
    if (!aabb_valid(segment)) {
      return false;
    }
  }
  return true;
}

// Intent: Bounds should follow geometry change when sag setting changes.
bool test_bounds_follow_geometry_change() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 6.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {12.0, 0.0, 6.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;

  wire::core::GeometrySettings line{};
  line.curve_samples = 9;
  line.sag_enabled = false;
  line.sag_factor = 0.1;
  (void)state.UpdateGeometrySettings(line, true);
  (void)state.Commit().recalc_stats;
  const auto* bounds_line = state.find_bounds_cache(span);
  if (bounds_line == nullptr) {
    return false;
  }
  const double line_min_z = bounds_line->whole.min.z;

  wire::core::GeometrySettings sag = line;
  sag.sag_enabled = true;
  (void)state.UpdateGeometrySettings(sag, true);
  (void)state.Commit().recalc_stats;
  const auto* bounds_sag = state.find_bounds_cache(span);
  if (bounds_sag == nullptr) {
    return false;
  }

  return bounds_sag->whole.min.z < line_min_z;
}

// Intent: GeneratePolesAlongRoad places poles at interval and applies pole type.
bool test_generate_poles_along_road_basic() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 10;
  road.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  const auto result = state.GeneratePolesAlongRoad(road, 5.0, type_ids.front());
  if (!result.ok) {
    return false;
  }
  if (result.value.size() != 5) {
    return false;
  }

  for (std::size_t i = 0; i < result.value.size(); ++i) {
    const auto* pole = state.view().edit_state().poles.find(result.value[i]);
    if (pole == nullptr) {
      return false;
    }
    if (pole->pole_type_id != type_ids.front()) {
      return false;
    }
    if (!pole->generation.generated || pole->generation.source != wire::core::GenerationSource::kRoadAuto) {
      return false;
    }
    if (!almost_equal(pole->world_transform.position.y, 0.0) || !almost_equal(pole->world_transform.position.z, 0.0)) {
      return false;
    }
  }
  return true;
}

// Intent: Pole context classification should mark terminal/straight/corner from road shape.
bool test_pole_context_classification_basic() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment straight{};
  straight.id = 901;
  straight.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  const auto straight_result = state.GeneratePolesAlongRoad(straight, 5.0, type_ids.front());
  if (!straight_result.ok || straight_result.value.size() < 3) {
    return false;
  }

  const auto* first = state.view().edit_state().poles.find(straight_result.value.front());
  const auto* middle = state.view().edit_state().poles.find(straight_result.value[1]);
  const auto* last = state.view().edit_state().poles.find(straight_result.value.back());
  if (first == nullptr || middle == nullptr || last == nullptr) {
    return false;
  }
  if (first->context.kind != wire::core::PoleContextKind::kTerminal ||
      last->context.kind != wire::core::PoleContextKind::kTerminal ||
      middle->context.kind != wire::core::PoleContextKind::kStraight) {
    return false;
  }

  CoreState corner_state;
  const auto corner_type_ids = sorted_pole_type_ids(corner_state);
  if (corner_type_ids.empty()) {
    return false;
  }
  wire::core::RoadSegment corner{};
  corner.id = 902;
  corner.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.0, 10.0, 0.0}};
  const auto corner_result = corner_state.GeneratePolesAlongRoad(corner, 5.0, corner_type_ids.front());
  if (!corner_result.ok) {
    return false;
  }

  bool found_corner = false;
  for (ObjectId pole_id : corner_result.value) {
    const auto* pole = corner_state.view().edit_state().poles.find(pole_id);
    if (pole != nullptr && pole->context.kind == wire::core::PoleContextKind::kCorner) {
      if (pole->context.corner_angle_deg <= 0.0 || pole->context.side_scale < 1.0) {
        return false;
      }
      found_corner = true;
      break;
    }
  }
  return found_corner;
}

// Intent: Angle correction should stay bounded and produce finite corrected port values.
bool test_angle_correction_bounds_and_finite() {
  CoreState state;
  wire::core::LayoutSettings layout{};
  layout.angle_correction_enabled = true;
  layout.corner_threshold_deg = 5.0;
  layout.min_side_scale = 1.0;
  layout.max_side_scale = 1.6;
  if (!state.UpdateLayoutSettings(layout).ok) {
    return false;
  }

  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::RoadSegment road{};
  road.id = 903;
  road.polyline = {{0.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, {8.0, 8.0, 0.0}};
  const auto result = state.GeneratePolesAlongRoad(road, 4.0, type_ids.front());
  if (!result.ok) {
    return false;
  }

  bool found_corrected_port = false;
  for (ObjectId pole_id : result.value) {
    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr || pole->context.kind != wire::core::PoleContextKind::kCorner) {
      continue;
    }
    if (pole->context.side_scale < layout.min_side_scale || pole->context.side_scale > layout.max_side_scale) {
      return false;
    }
    for (const auto& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id != pole_id) {
        continue;
      }
      if (!std::isfinite(port.world_position.x) || !std::isfinite(port.world_position.y) ||
          !std::isfinite(port.world_position.z) || !std::isfinite(port.side_scale_applied)) {
        return false;
      }
      if (port.angle_correction_applied) {
        found_corrected_port = true;
      }
    }
  }
  return found_corrected_port;
}

// Intent: Corner turn sign should bias correction so outer side expands more than inner side.
bool test_corner_turn_sign_biases_outer_side() {
  auto check_turn = [](const std::vector<wire::core::Vec3d>& polyline, bool expect_left_turn_outer_right) -> bool {
    CoreState state;
    wire::core::LayoutSettings layout{};
    layout.angle_correction_enabled = true;
    layout.corner_threshold_deg = 5.0;
    layout.min_side_scale = 1.0;
    layout.max_side_scale = 1.8;
    if (!state.UpdateLayoutSettings(layout).ok) {
      return false;
    }

    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return false;
    }

    wire::core::RoadSegment road{};
    road.id = expect_left_turn_outer_right ? 905 : 906;
    road.polyline = polyline;
    const auto result = state.GeneratePolesAlongRoad(road, 4.0, type_ids.front());
    if (!result.ok) {
      return false;
    }

    const wire::core::Pole* corner_pole = nullptr;
    for (ObjectId pole_id : result.value) {
      const auto* pole = state.view().edit_state().poles.find(pole_id);
      if (pole != nullptr && pole->context.kind == wire::core::PoleContextKind::kCorner) {
        corner_pole = pole;
        break;
      }
    }
    if (corner_pole == nullptr) {
      return false;
    }

    const wire::core::Port* left_slot_port = nullptr;
    const wire::core::Port* right_slot_port = nullptr;
    for (const auto& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id != corner_pole->id) {
        continue;
      }
      if (port.source_slot_id == 200) {
        left_slot_port = &port;
      } else if (port.source_slot_id == 201) {
        right_slot_port = &port;
      }
    }
    if (left_slot_port == nullptr || right_slot_port == nullptr) {
      return false;
    }

    const double left_offset = std::abs(left_slot_port->world_position.y - corner_pole->world_transform.position.y);
    const double right_offset = std::abs(right_slot_port->world_position.y - corner_pole->world_transform.position.y);
    if (expect_left_turn_outer_right) {
      return corner_pole->context.corner_turn_sign > 0.0 && right_offset > left_offset &&
             right_slot_port->side_scale_applied > left_slot_port->side_scale_applied;
    }
    return corner_pole->context.corner_turn_sign < 0.0 && left_offset > right_offset &&
           left_slot_port->side_scale_applied > right_slot_port->side_scale_applied;
  };

  const bool left_ok = check_turn({{0.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, {8.0, 8.0, 0.0}}, true);
  const bool right_ok = check_turn({{0.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, {8.0, -8.0, 0.0}}, false);
  return left_ok && right_ok;
}

// Intent: Acute corners should auto-widen lane spacing (category-agnostic), while keeping outer>inner bias.
bool test_acute_corner_auto_widens_lane_spacing() {
  auto lane_width_for_interior = [](double interior_deg) -> double {
    CoreState state;
    wire::core::LayoutSettings layout{};
    layout.angle_correction_enabled = true;
    layout.corner_threshold_deg = 5.0;
    layout.min_side_scale = 1.0;
    layout.max_side_scale = 1.8;
    if (!state.UpdateLayoutSettings(layout).ok) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    constexpr double kPi = 3.14159265358979323846;
    const double rad = interior_deg * (kPi / 180.0);
    const double out_heading = kPi - rad;
    wire::core::RoadSegment road{};
    road.id = static_cast<std::uint64_t>(990 + static_cast<int>(interior_deg));
    road.polyline = {
        {0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0},
        {10.0 + 10.0 * std::cos(out_heading), 10.0 * std::sin(out_heading), 0.0},
    };
    const auto gen =
        state.GenerateSimpleLineFromPoints(road, type_ids.front(), wire::core::ConnectionCategory::kLowVoltage);
    if (!gen.ok || gen.value.pole_ids.size() != 3) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    const auto* pole = state.view().edit_state().poles.find(gen.value.pole_ids[1]);
    if (pole == nullptr) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const wire::core::Port* p_left = nullptr;
    const wire::core::Port* p_right = nullptr;
    for (const auto& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id != pole->id) {
        continue;
      }
      if (port.source_slot_id == 200) {
        p_left = &port;
      } else if (port.source_slot_id == 201) {
        p_right = &port;
      }
    }
    if (p_left == nullptr || p_right == nullptr) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const wire::core::Vec3d local_left = to_local_on_pole_test(*pole, p_left->world_position);
    const wire::core::Vec3d local_right = to_local_on_pole_test(*pole, p_right->world_position);
    return std::abs(local_right.y - local_left.y);
  };

  const double width_acute = lane_width_for_interior(45.0);
  const double width_obtuse = lane_width_for_interior(120.0);
  if (!std::isfinite(width_acute) || !std::isfinite(width_obtuse)) {
    return false;
  }
  // Base template width is 0.8m between slot 200(-0.4) and 201(+0.4).
  return width_acute > width_obtuse && width_acute > 0.8 + 1e-6;
}

// Intent: Branch context should bias slot selection away from trunk-only slot choice.
bool test_slot_selection_context_bias() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {10.0, 0.0, 0.0};
  wire::core::Transformd c{};
  c.position = {10.0, 8.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  const ObjectId pole_c = state.AddPole(c, 10.0, "C").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok ||
      !state.ApplyPoleType(pole_c, type_ids.front()).ok) {
    return false;
  }

  wire::core::CoreState::AddConnectionByPoleOptions trunk_options{};
  trunk_options.connection_context = wire::core::ConnectionContext::kTrunkContinue;
  const auto trunk =
      state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage, trunk_options);
  if (!trunk.ok) {
    return false;
  }

  wire::core::CoreState::AddConnectionByPoleOptions branch_options{};
  branch_options.connection_context = wire::core::ConnectionContext::kBranchAdd;
  const auto branch =
      state.AddConnectionByPole(pole_b, pole_c, wire::core::ConnectionCategory::kLowVoltage, branch_options);
  if (!branch.ok) {
    return false;
  }
  return branch.value.slot_a_id >= 0 && branch.value.slot_a_id != trunk.value.slot_b_id;
}

// Intent: Slot selection tie-break should be deterministic and debug log should be internally consistent.
bool test_slot_selection_deterministic_and_debug_integrity() {
  auto run_once = []() -> std::pair<int, wire::core::SlotSelectionDebugRecord> {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return {-1, {}};
    }
    const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
    wire::core::Transformd b{};
    b.position = {12.0, 0.0, 0.0};
    const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
    (void)state.ApplyPoleType(pole_a, type_ids.front());
    (void)state.ApplyPoleType(pole_b, type_ids.front());

    wire::core::CoreState::AddConnectionByPoleOptions options{};
    options.connection_context = wire::core::ConnectionContext::kCornerPass;
    options.branch_index = 3;
    const auto result = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage, options);
    if (!result.ok || state.view().slot_selection_debug_records().empty()) {
      return {-1, {}};
    }
    return {result.value.slot_a_id, state.view().slot_selection_debug_records().back()};
  };

  const auto first = run_once();
  const auto second = run_once();
  if (first.first < 0 || second.first < 0 || first.first != second.first) {
    return false;
  }
  if (first.second.candidates.empty()) {
    return false;
  }
  return has_selected_slot_in_candidates(first.second);
}

// Intent: GenerateSimpleLine over corner path should propagate corner/trunk contexts into created spans.
bool test_generate_simple_line_corner_context_integration() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 904;
  road.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {12.0, 12.0, 0.0}};
  const auto result =
      state.GenerateSimpleLine(road, 4.0, type_ids.front(), wire::core::ConnectionCategory::kLowVoltage);
  if (!result.ok || result.value.span_ids.empty()) {
    return false;
  }

  bool has_corner_context = false;
  for (ObjectId span_id : result.value.span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr && span->placement_context == wire::core::ConnectionContext::kCornerPass) {
      has_corner_context = true;
      break;
    }
  }
  return has_corner_context && validate_now(state).ok();
}

// Intent: DrawPath generation should place one pole per clicked point and auto-compute pole yaw.
bool test_generate_simple_line_from_points_exact_poles_and_orientation() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 907;
  road.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.0, 10.0, 0.0}};
  const auto result =
      state.GenerateSimpleLineFromPoints(road, type_ids.front(), wire::core::ConnectionCategory::kLowVoltage);
  if (!result.ok) {
    return false;
  }
  if (result.value.pole_ids.size() != road.polyline.size()) {
    return false;
  }
  if (result.value.span_ids.size() != road.polyline.size() - 1) {
    return false;
  }

  for (std::size_t i = 0; i < road.polyline.size(); ++i) {
    const auto* pole = state.view().edit_state().poles.find(result.value.pole_ids[i]);
    if (pole == nullptr) {
      return false;
    }
    if (!almost_equal(pole->world_transform.position, road.polyline[i])) {
      return false;
    }
  }

  const auto* first = state.view().edit_state().poles.find(result.value.pole_ids[0]);
  const auto* middle = state.view().edit_state().poles.find(result.value.pole_ids[1]);
  const auto* last = state.view().edit_state().poles.find(result.value.pole_ids[2]);
  if (first == nullptr || middle == nullptr || last == nullptr) {
    return false;
  }
  if (!almost_equal(first->world_transform.rotation_euler_deg.z, 0.0, 1e-6)) {
    return false;
  }
  if (!almost_equal(last->world_transform.rotation_euler_deg.z, 90.0, 1e-6)) {
    return false;
  }
  return almost_equal(middle->world_transform.rotation_euler_deg.z, 45.0, 1e-6);
}

// Intent: Sharp corner should align pole local-Y(side axis) perpendicular to bisector and avoid inward direction.
bool test_generate_simple_line_from_points_sharp_corner_perpendicular_orientation() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 931;
  road.polyline = {
      {0.0, 0.0, 0.0},
      {10.0, 0.0, 0.0},
      {5.0, 8.660254037844386, 0.0}, // interior ~= 60 deg at middle
  };
  const auto result = state.GenerateSimpleLineFromPoints(road, pole_type_ids.front(), ConnectionCategory::kLowVoltage);
  if (!result.ok || result.value.pole_ids.size() != 3) {
    return false;
  }

  const auto* middle = state.view().edit_state().poles.find(result.value.pole_ids[1]);
  if (middle == nullptr) {
    return false;
  }
  const wire::core::Vec3d u0 = normalize_xy_safe(road.polyline[0] - road.polyline[1]); // corner->prev
  const wire::core::Vec3d u1 = normalize_xy_safe(road.polyline[2] - road.polyline[1]); // corner->next
  const wire::core::Vec3d bisector = normalize_xy_safe({u0.x + u1.x, u0.y + u1.y, 0.0});
  if (std::abs(bisector.x) < 1e-9 && std::abs(bisector.y) < 1e-9) {
    return false;
  }

  const wire::core::Vec3d side_from_yaw = local_side_axis_from_yaw(middle->world_transform.rotation_euler_deg.z);
  const wire::core::Vec3d side_debug = normalize_xy_safe(middle->context.sharp_side_dir);
  const wire::core::Vec3d bisector_debug = normalize_xy_safe(middle->context.sharp_bisector_dir);

  const wire::core::Vec3d t_in = normalize_xy_safe(road.polyline[1] - road.polyline[0]);
  const wire::core::Vec3d t_out = normalize_xy_safe(road.polyline[2] - road.polyline[1]);
  const double turn = t_in.x * t_out.y - t_in.y * t_out.x;
  wire::core::Vec3d inward{};
  if (turn > 1e-9) {
    inward = normalize_xy_safe({-t_in.y, t_in.x, 0.0});
  } else if (turn < -1e-9) {
    inward = normalize_xy_safe({t_in.y, -t_in.x, 0.0});
  }

  const bool ok_sharp = middle->context.sharp_orientation_applied;
  const bool ok_perp = std::abs(dot_xy(side_from_yaw, bisector)) <= 1e-6;
  const bool ok_side = std::abs(dot_xy(side_debug, side_from_yaw) - 1.0) <= 1e-6;
  const bool ok_b = std::abs(dot_xy(bisector_debug, bisector) - 1.0) <= 1e-6;
  const bool ok_inward =
      (std::abs(inward.x) < 1e-9 && std::abs(inward.y) < 1e-9) ? true : (dot_xy(side_from_yaw, inward) <= 1e-9);
  return ok_sharp && ok_perp && ok_side && ok_b && ok_inward;
}

// Intent: Sharp-corner threshold (75deg) should apply at <=75 and be disabled above 75.
bool test_sharp_corner_threshold_boundary_orientation() {
  auto generate_middle = [](double corner_interior_deg) -> std::pair<double, bool> {
    CoreState state;
    const auto pole_type_ids = sorted_pole_type_ids(state);
    if (pole_type_ids.empty()) {
      return {std::numeric_limits<double>::quiet_NaN(), false};
    }
    constexpr double kPi = 3.14159265358979323846;
    const double interior_rad = corner_interior_deg * (kPi / 180.0);
    const double out_heading_rad = kPi - interior_rad;
    wire::core::RoadSegment road{};
    road.id = static_cast<std::uint64_t>(950 + static_cast<int>(corner_interior_deg * 10.0));
    // Unequal segment lengths so "bisector" and "chord(prev->next)" are different.
    road.polyline = {
        {-5.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {20.0 * std::cos(out_heading_rad), 20.0 * std::sin(out_heading_rad), 0.0},
    };
    const auto result =
        state.GenerateSimpleLineFromPoints(road, pole_type_ids.front(), wire::core::ConnectionCategory::kLowVoltage);
    if (!result.ok || result.value.pole_ids.size() != 3) {
      return {std::numeric_limits<double>::quiet_NaN(), false};
    }
    const auto* middle = state.view().edit_state().poles.find(result.value.pole_ids[1]);
    if (middle == nullptr) {
      return {std::numeric_limits<double>::quiet_NaN(), false};
    }
    return {middle->world_transform.rotation_euler_deg.z, middle->context.sharp_orientation_applied};
  };

  const auto [yaw74, sharp74] = generate_middle(74.0);
  const auto [yaw75, sharp75] = generate_middle(75.0);
  const auto [yaw76, sharp76] = generate_middle(76.0);
  if (!std::isfinite(yaw74) || !std::isfinite(yaw75) || !std::isfinite(yaw76)) {
    return false;
  }

  return sharp74 && sharp75 && !sharp76 &&
         angle_diff_abs_deg(yaw74, yaw76) > 1e-3 &&
         angle_diff_abs_deg(yaw75, yaw76) > 1e-3;
}

// Intent: Reused guide vertex poles should be reoriented by current sharp-corner rule when not manually overridden.
bool test_generate_from_guide_reused_vertex_reorients_to_corner_rule() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::GenerationRequest req_obtuse{};
  req_obtuse.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {15.0, 8.660254037844386, 0.0}}; // interior ~=120
  req_obtuse.interval_m = 100.0;                                                                     // only vertices
  req_obtuse.pole_type_id = type_ids.front();
  req_obtuse.category = ConnectionCategory::kLowVoltage;
  req_obtuse.requested_lane_count = 1;
  const auto first = state.GenerateFromGuide(req_obtuse);
  if (!first.ok) {
    return false;
  }

  ObjectId vertex_id = wire::core::kInvalidObjectId;
  for (const auto& pole : state.view().edit_state().poles.items()) {
    if (almost_equal(pole.world_transform.position, req_obtuse.path.polyline[1], 1e-6)) {
      vertex_id = pole.id;
      break;
    }
  }
  if (vertex_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::GenerationRequest req_acute = req_obtuse;
  req_acute.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {2.9289321881345245, 7.0710678118654755, 0.0}}; // ~45
  const auto second = state.GenerateFromGuide(req_acute);
  if (!second.ok) {
    return false;
  }
  const auto* vertex = state.view().edit_state().poles.find(vertex_id);
  if (vertex == nullptr) {
    return false;
  }

  const wire::core::Vec3d u0 = normalize_xy_safe(req_acute.path.polyline[0] - req_acute.path.polyline[1]); // corner->prev
  const wire::core::Vec3d u1 = normalize_xy_safe(req_acute.path.polyline[2] - req_acute.path.polyline[1]); // corner->next
  const wire::core::Vec3d bisector = normalize_xy_safe({u0.x + u1.x, u0.y + u1.y, 0.0});
  const wire::core::Vec3d side_from_yaw = local_side_axis_from_yaw(vertex->world_transform.rotation_euler_deg.z);
  return vertex->context.sharp_orientation_applied && std::abs(dot_xy(side_from_yaw, bisector)) <= 1e-6;
}

// Intent: Reused poles must reproject template-owned ports after yaw/context changes during guide regeneration.
bool test_generate_from_guide_reused_pole_reprojects_owned_ports() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::GenerationRequest req_obtuse{};
  req_obtuse.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {15.0, 8.660254037844386, 0.0}}; // interior ~=120
  req_obtuse.interval_m = 100.0; // vertices only
  req_obtuse.pole_type_id = type_ids.front();
  req_obtuse.category = ConnectionCategory::kLowVoltage;
  req_obtuse.requested_lane_count = 1;
  const auto first = state.GenerateFromGuide(req_obtuse);
  if (!first.ok) {
    return false;
  }

  ObjectId vertex_id = wire::core::kInvalidObjectId;
  for (const auto& pole : state.view().edit_state().poles.items()) {
    if (almost_equal(pole.world_transform.position, req_obtuse.path.polyline[1], 1e-6)) {
      vertex_id = pole.id;
      break;
    }
  }
  if (vertex_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const wire::core::Pole* before_pole = state.view().edit_state().poles.find(vertex_id);
  if (before_pole == nullptr) {
    return false;
  }
  double yaw_before = before_pole->world_transform.rotation_euler_deg.z;

  ObjectId slot200_id = wire::core::kInvalidObjectId;
  ObjectId slot201_id = wire::core::kInvalidObjectId;
  wire::core::Vec3d slot200_before{};
  wire::core::Vec3d slot201_before{};
  for (const auto& port : state.view().edit_state().ports.items()) {
    if (port.owner_pole_id != vertex_id) {
      continue;
    }
    if (port.source_slot_id == 200) {
      slot200_id = port.id;
      slot200_before = port.world_position;
    } else if (port.source_slot_id == 201) {
      slot201_id = port.id;
      slot201_before = port.world_position;
    }
  }
  if (slot200_id == wire::core::kInvalidObjectId || slot201_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::GenerationRequest req_acute = req_obtuse;
  req_acute.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {2.9289321881345245, 7.0710678118654755, 0.0}}; // ~45
  const auto second = state.GenerateFromGuide(req_acute);
  if (!second.ok) {
    return false;
  }

  const wire::core::Pole* after_pole = state.view().edit_state().poles.find(vertex_id);
  const wire::core::Port* slot200_after = state.view().edit_state().ports.find(slot200_id);
  const wire::core::Port* slot201_after = state.view().edit_state().ports.find(slot201_id);
  if (after_pole == nullptr || slot200_after == nullptr || slot201_after == nullptr) {
    return false;
  }

  const double yaw_after = after_pole->world_transform.rotation_euler_deg.z;
  const bool yaw_changed = angle_diff_abs_deg(yaw_before, yaw_after) > 1e-3;
  const bool moved200 = !almost_equal(slot200_after->world_position, slot200_before, 1e-6);
  const bool moved201 = !almost_equal(slot201_after->world_position, slot201_before, 1e-6);
  return yaw_changed && (moved200 || moved201);
}

// Intent: Guide generation should stay robust on duplicate points and keep generated poles finite/on-path-z.
bool test_generate_from_guide_with_duplicate_points_is_robust() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::GenerationRequest req{};
  req.path.polyline = {
      {0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0}, // duplicate
      {20.0, 0.0, 0.0},
      {20.0, 0.0, 0.0}, // duplicate
      {40.0, 0.0, 0.0},
  };
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  req.category = ConnectionCategory::kLowVoltage;
  req.requested_lane_count = 1;
  const auto result = state.GenerateFromGuide(req);
  if (!result.ok || result.value.generated_pole_ids.size() < 2) {
    return false;
  }
  for (ObjectId pole_id : result.value.generated_pole_ids) {
    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr) {
      return false;
    }
    const auto& p = pole->world_transform.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      return false;
    }
    if (!almost_equal(p.z, 0.0, 1e-9)) {
      return false;
    }
  }
  return validate_now(state).ok();
}

// Intent: Guide reverse mode should preserve generated geometry set (same positions, different order allowed).
bool test_generate_from_guide_reverse_mode_position_symmetry() {
  auto run_mode = [](wire::core::PathDirectionMode mode) {
    CoreState state;
    struct ModeResult {
      bool ok = false;
      std::vector<wire::core::Vec3d> poles{};
      std::vector<wire::core::Vec3d> guide{};
    };
    ModeResult out{};
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return out;
    }
    wire::core::GenerationRequest req{};
    req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 6.0, 0.0}};
    req.interval_m = 6.0;
    req.pole_type_id = type_ids.front();
    req.category = ConnectionCategory::kLowVoltage;
    req.requested_lane_count = 1;
    req.direction_mode = mode;
    const auto result = state.GenerateFromGuide(req);
    if (!result.ok) {
      return out;
    }
    out.ok = true;
    out.guide = req.path.polyline;
    out.poles.reserve(result.value.generated_pole_ids.size());
    for (ObjectId pole_id : result.value.generated_pole_ids) {
      const auto* pole = state.view().edit_state().poles.find(pole_id);
      if (pole == nullptr) {
        out.ok = false;
        return out;
      }
      const auto& p = pole->world_transform.position;
      out.poles.push_back(p);
    }
    return out;
  };

  auto point_to_segment_distance = [](const wire::core::Vec3d& p, const wire::core::Vec3d& a,
                                      const wire::core::Vec3d& b) -> double {
    const wire::core::Vec3d ab = b - a;
    const wire::core::Vec3d ap = p - a;
    const double ab2 = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
    if (ab2 <= 1e-12) {
      const wire::core::Vec3d d = p - a;
      return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    }
    const double t = std::clamp((ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) / ab2, 0.0, 1.0);
    const wire::core::Vec3d q{a.x + ab.x * t, a.y + ab.y * t, a.z + ab.z * t};
    const wire::core::Vec3d d = p - q;
    return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
  };

  auto is_on_polyline = [&](const wire::core::Vec3d& p, const std::vector<wire::core::Vec3d>& guide) -> bool {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < guide.size(); ++i) {
      best = std::min(best, point_to_segment_distance(p, guide[i], guide[i + 1]));
    }
    return best <= 1e-6;
  };

  const auto forward = run_mode(wire::core::PathDirectionMode::kForward);
  const auto reverse = run_mode(wire::core::PathDirectionMode::kReverse);
  if (!forward.ok || !reverse.ok) {
    return false;
  }
  if (forward.poles.size() != reverse.poles.size() || forward.poles.size() < 2) {
    return false;
  }
  for (const auto& p : forward.poles) {
    if (!is_on_polyline(p, forward.guide)) {
      return false;
    }
  }
  for (const auto& p : reverse.poles) {
    if (!is_on_polyline(p, reverse.guide)) {
      return false;
    }
  }
  const wire::core::Vec3d start = forward.guide.front();
  const wire::core::Vec3d end = forward.guide.back();
  bool has_start = false;
  bool has_end = false;
  for (const auto& p : forward.poles) {
    has_start = has_start || almost_equal(p, start, 1e-6);
    has_end = has_end || almost_equal(p, end, 1e-6);
  }
  return has_start && has_end;
}

// Intent: Avoid constraints should prevent generated auto poles from entering forbidden radius.
bool test_generate_from_guide_respects_avoid_constraints() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::GenerationRequest req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  req.category = ConnectionCategory::kLowVoltage;
  req.requested_lane_count = 1;
  req.constraints.avoid_points = {{10.0, 0.0, 0.0}};
  req.constraints.avoid_radius_m = 1.0;
  const auto result = state.GenerateFromGuide(req);
  if (!result.ok || result.value.generated_pole_ids.empty()) {
    return false;
  }
  const wire::core::Vec3d avoid = req.constraints.avoid_points.front();
  const double avoid_r2 = req.constraints.avoid_radius_m * req.constraints.avoid_radius_m;
  for (ObjectId pole_id : result.value.generated_pole_ids) {
    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr) {
      return false;
    }
    const auto& p = pole->world_transform.position;
    const wire::core::Vec3d d{p.x - avoid.x, p.y - avoid.y, p.z - avoid.z};
    const double d2 = d.x * d.x + d.y * d.y + d.z * d.z;
    if (d2 <= avoid_r2 + 1e-9) {
      return false;
    }
  }
  return true;
}

// Intent: Preferred side should come from geometry (peer location), not branch_index parity.
bool test_preferred_side_uses_geometry() {
  auto pick_side = [](double peer_y) -> wire::core::SlotSide {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return wire::core::SlotSide::kCenter;
    }
    const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
    wire::core::Transformd b{};
    b.position = {10.0, peer_y, 0.0};
    const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
    (void)state.ApplyPoleType(pole_a, type_ids.front());
    (void)state.ApplyPoleType(pole_b, type_ids.front());
    state.clear_slot_selection_debug_records();

    wire::core::CoreState::AddConnectionByPoleOptions options{};
    options.connection_context = wire::core::ConnectionContext::kBranchAdd;
    options.branch_index = 7; // deliberately fixed; geometry should dominate.
    const auto add = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage, options);
    if (!add.ok) {
      return wire::core::SlotSide::kCenter;
    }
    for (const auto& debug : state.view().slot_selection_debug_records()) {
      if (debug.pole_id != pole_a || debug.selected_slot_id < 0) {
        continue;
      }
      const auto detail = state.GetPoleDetail(pole_a);
      if (detail.pole_type == nullptr) {
        return wire::core::SlotSide::kCenter;
      }
      for (const auto& slot : detail.pole_type->port_slots) {
        if (slot.slot_id == debug.selected_slot_id) {
          return slot.side;
        }
      }
    }
    return wire::core::SlotSide::kCenter;
  };

  const wire::core::SlotSide right = pick_side(+8.0);
  const wire::core::SlotSide left = pick_side(-8.0);
  return right == wire::core::SlotSide::kRight && left == wire::core::SlotSide::kLeft;
}

// Intent: GenerateGroupedLine should produce grouped spans with lane assignments and bundle linkage.
bool test_generate_grouped_line_high_voltage_three_phase() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::CoreState::GenerateGroupedLineOptions options{};
  options.road.id = 1001;
  options.road.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 2.0, 0.0}, {30.0, 2.0, 0.0}};
  options.interval = 0.0;
  options.pole_type_id = type_ids.front();
  options.group_spec.category = wire::core::ConnectionCategory::kHighVoltage;
  options.group_spec.group_kind = wire::core::ConductorGroupKind::kThreePhase;
  options.group_spec.conductor_count = 3;
  options.group_spec.lane_spacing_m = 0.45;
  options.group_spec.maintain_lane_order = true;
  options.group_spec.allow_lane_mirror = true;
  options.direction_mode = wire::core::PathDirectionMode::kAuto;

  const auto result = state.GenerateGroupedLine(options);
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
    if (assignment.slot_ids_a.size() != 3 || assignment.slot_ids_b.size() != 3) {
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

  wire::core::CoreState::GenerateGroupedLineOptions options{};
  options.road.id = 1002;
  options.road.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  options.interval = 0.0;
  options.pole_type_id = type_ids.front();
  options.group_spec.category = wire::core::ConnectionCategory::kLowVoltage;
  options.group_spec.group_kind = wire::core::ConductorGroupKind::kSingle;
  options.group_spec.conductor_count = 1;
  options.direction_mode = wire::core::PathDirectionMode::kReverse;

  const auto reverse = state.GenerateGroupedLine(options);
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

  wire::core::CoreState::GenerateGroupedLineOptions options{};
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
  options.group_spec.category = wire::core::ConnectionCategory::kHighVoltage;
  options.group_spec.group_kind = wire::core::ConductorGroupKind::kThreePhase;
  options.group_spec.conductor_count = 3;
  options.group_spec.allow_lane_mirror = true;
  options.group_spec.maintain_lane_order = true;
  options.direction_mode = wire::core::PathDirectionMode::kAuto;

  const auto generated = state.GenerateGroupedLine(options);
  if (!generated.ok) {
    return false;
  }
  const LaneOrderMetrics metrics = compute_lane_order_metrics(state, generated.value.lane_assignments);
  return metrics.y_inversions == 0;
}

// Intent: Group lane assignment on an acute corner path should avoid per-segment XY crossings.
bool test_grouped_line_acute_corner_no_xy_crossing() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::CoreState::GenerateGroupedLineOptions options{};
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
  options.group_spec.category = wire::core::ConnectionCategory::kCommunication;
  options.group_spec.group_kind = wire::core::ConductorGroupKind::kParallel;
  options.group_spec.conductor_count = 4;
  options.group_spec.allow_lane_mirror = true;
  options.group_spec.maintain_lane_order = true;
  options.direction_mode = wire::core::PathDirectionMode::kAuto;

  const auto generated = state.GenerateGroupedLine(options);
  if (!generated.ok) {
    return false;
  }
  return count_lane_segment_xy_intersections(state, generated.value.lane_assignments) == 0;
}

// Intent: Enabling lane mirror must not worsen grouped-lane quality metrics (Y/Z order and layer continuity).
bool test_grouped_line_mirror_metric_non_regression() {
  const auto run_with_mirror = [](bool allow_mirror) -> std::pair<bool, LaneOrderMetrics> {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return {false, {}};
    }
    wire::core::CoreState::GenerateGroupedLineOptions options{};
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
    options.group_spec.category = wire::core::ConnectionCategory::kLowVoltage;
    options.group_spec.group_kind = wire::core::ConductorGroupKind::kParallel;
    options.group_spec.conductor_count = 4;
    options.group_spec.allow_lane_mirror = allow_mirror;
    options.group_spec.maintain_lane_order = true;
    options.direction_mode = wire::core::PathDirectionMode::kAuto;

    const auto generated = state.GenerateGroupedLine(options);
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

// Intent: Bundle query API should return only spans that reference the target bundle.
bool test_bundle_query_spans_by_bundle() {
  CoreState state;
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd b{};
  b.position = {8.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  const ObjectId port_a = state.AddPort(pole_a, {0.0, 0.0, 7.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId port_b = state.AddPort(pole_b, {8.0, 0.0, 7.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const auto add_bundle = state.AddBundle(2, 0.2, wire::core::BundleKind::kLowVoltage);
  if (!add_bundle.ok) {
    return false;
  }
  const auto add_span = state.AddSpan(port_a, port_b, SpanKind::kDistribution, SpanLayer::kLowVoltage, add_bundle.value);
  if (!add_span.ok) {
    return false;
  }

  const auto grouped_spans = state.GetSpansByBundle(add_bundle.value);
  if (grouped_spans.size() != 1 || grouped_spans[0] != add_span.value) {
    return false;
  }
  const auto& relation_index = state.view().relation_index();
  const auto bundle_it = relation_index.spans_by_bundle.find(add_bundle.value);
  if (bundle_it == relation_index.spans_by_bundle.end() ||
      std::find(bundle_it->second.begin(), bundle_it->second.end(), add_span.value) == bundle_it->second.end()) {
    return false;
  }
  const auto pole_port_it = relation_index.ports_by_pole.find(pole_a);
  if (pole_port_it == relation_index.ports_by_pole.end() ||
      std::find(pole_port_it->second.begin(), pole_port_it->second.end(), port_a) == pole_port_it->second.end()) {
    return false;
  }
  return validate_now(state).ok();
}

// Intent: Bundle-referenced spans must reject invalid bundle IDs.
bool test_add_span_invalid_bundle_fails() {
  CoreState state;
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd b{};
  b.position = {6.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  const ObjectId port_a = state.AddPort(pole_a, {0.0, 0.0, 7.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId port_b = state.AddPort(pole_b, {6.0, 0.0, 7.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const auto bad = state.AddSpan(port_a, port_b, SpanKind::kDistribution, SpanLayer::kLowVoltage, 999999);
  return !bad.ok && regex_contains(bad.error, "bundle does not exist");
}

// Intent: Spans without bundle assignment remain valid and keep legacy behavior.
bool test_unbundled_span_compatibility() {
  CoreState state;
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd b{};
  b.position = {9.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  const ObjectId port_a = state.AddPort(pole_a, {0.0, 0.0, 7.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId port_b = state.AddPort(pole_b, {9.0, 0.0, 7.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span_id = state.AddSpan(port_a, port_b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  const auto* span_before = state.view().edit_state().spans.find(span_id);
  if (span_before == nullptr || span_before->bundle_id != wire::core::kInvalidObjectId) {
    return false;
  }

  const auto recalc = state.Commit().recalc_stats;
  const auto* runtime = state.view().find_span_runtime_state(span_id);
  return runtime != nullptr && recalc.geometry_processed > 0 && validate_now(state).ok();
}

// Intent: DrawPath-oriented bundle API should create category-default lanes and assign spans to one bundle.
bool test_generate_bundle_from_path_basic_hv_default_lanes() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  CoreState::GenerateBundleFromPathInput input{};
  input.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 4.0, 0.0}};
  input.interval_m = 6.0;
  input.pole_type_id = type_ids.front();
  input.category = ConnectionCategory::kHighVoltage;
  input.direction_mode = wire::core::PathDirectionMode::kAuto;
  input.requested_lane_count = 0; // category standard

  const auto result = state.GenerateBundleFromPath(input);
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
bool test_generate_bundle_from_path_direction_modes_nonfailing() {
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
    CoreState::GenerateBundleFromPathInput input{};
    input.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 2.0, 0.0}};
    input.interval_m = 5.0;
    input.pole_type_id = type_ids.front();
    input.category = ConnectionCategory::kLowVoltage;
    input.direction_mode = mode;
    input.requested_lane_count = 0;
    const auto result = state.GenerateBundleFromPath(input);
    if (!result.ok || result.value.generated_span_ids.empty()) {
      return false;
    }
  }
  return true;
}

// Intent: Bundle path generation should reject invalid input and keep state recoverable.
bool test_generate_bundle_from_path_invalid_inputs_fail() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const CoreCounts before = snapshot_counts(state);

  CoreState::GenerateBundleFromPathInput too_short{};
  too_short.polyline = {{0.0, 0.0, 0.0}};
  too_short.interval_m = 5.0;
  too_short.pole_type_id = type_ids.front();
  too_short.category = ConnectionCategory::kLowVoltage;
  const auto r_short = state.GenerateBundleFromPath(too_short);
  if (r_short.ok || !regex_contains(r_short.error, "at least 2 points")) {
    return false;
  }

  CoreState::GenerateBundleFromPathInput bad_interval{};
  bad_interval.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  bad_interval.interval_m = 0.0;
  bad_interval.pole_type_id = type_ids.front();
  bad_interval.category = ConnectionCategory::kLowVoltage;
  const auto r_interval = state.GenerateBundleFromPath(bad_interval);
  if (r_interval.ok || !regex_contains(r_interval.error, "interval_m")) {
    return false;
  }

  CoreState::GenerateBundleFromPathInput bad_category{};
  bad_category.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  bad_category.interval_m = 5.0;
  bad_category.pole_type_id = type_ids.front();
  bad_category.category = static_cast<ConnectionCategory>(255);
  const auto r_category = state.GenerateBundleFromPath(bad_category);
  if (r_category.ok || !regex_contains(r_category.error, "unsupported")) {
    return false;
  }

  CoreState::GenerateBundleFromPathInput bad_type{};
  bad_type.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  bad_type.interval_m = 5.0;
  bad_type.pole_type_id = 999999;
  bad_type.category = ConnectionCategory::kLowVoltage;
  const auto r_type = state.GenerateBundleFromPath(bad_type);
  if (r_type.ok || !regex_contains(r_type.error, "pole type")) {
    return false;
  }

  if (!same_counts(before, snapshot_counts(state))) {
    return false;
  }

  CoreState::GenerateBundleFromPathInput recover{};
  recover.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 0.0, 0.0}};
  recover.interval_m = 6.0;
  recover.pole_type_id = type_ids.front();
  recover.category = ConnectionCategory::kLowVoltage;
  return state.GenerateBundleFromPath(recover).ok;
}

// Intent: Fixed bundle template must reject explicit count override inputs.
bool test_backbone_bundle_template_fixed_count_rejects_override() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {16.0, 0.0, 0.0}};
  req.interval_m = 8.0;
  req.pole_type_id = type_ids.front();
  req.direction_mode = wire::core::PathDirectionMode::kAuto;
  wire::core::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  bundle.count = 3; // fixed template should reject any explicit count input.
  req.bundles.push_back(bundle);
  const auto result = state.GenerateFromBackboneSpec(req);
  return !result.ok && regex_contains(result.error, "count override");
}

// Intent: Variable communication bundle should scale generated span count by requested count.
bool test_backbone_bundle_template_comm_variable_count_scales_spans() {
  auto run_with_count = [&](int count) -> std::pair<bool, std::size_t> {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return {false, 0};
    }
    wire::core::BackboneSpec req{};
    req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 0.0, 0.0}};
    req.interval_m = 6.0;
    req.pole_type_id = type_ids.front();
    wire::core::BackboneBundleSpec bundle{};
    bundle.bundle_template_id = wire::core::BundleKind::kCommunication;
    bundle.count = count;
    req.bundles.push_back(bundle);
    const auto result = state.GenerateFromBackboneSpec(req);
    if (!result.ok || result.value.bundle_id == wire::core::kInvalidObjectId || result.value.generated_pole_ids.size() < 2) {
      return {false, 0};
    }
    const auto* created_bundle = state.view().edit_state().bundles.find(result.value.bundle_id);
    if (created_bundle == nullptr || created_bundle->conductor_count != count) {
      return {false, 0};
    }
    const std::size_t segment_count = result.value.generated_pole_ids.size() - 1;
    return {true, segment_count == 0 ? 0 : result.value.generated_span_ids.size() / segment_count};
  };

  const auto one = run_with_count(1);
  const auto three = run_with_count(3);
  return one.first && three.first && one.second == 1 && three.second == 3;
}

// Intent: Variable communication bundle must reject out-of-range counts.
bool test_backbone_bundle_template_comm_count_out_of_range_fails() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {16.0, 0.0, 0.0}};
  req.interval_m = 8.0;
  req.pole_type_id = type_ids.front();
  wire::core::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = wire::core::BundleKind::kCommunication;
  bundle.count = 9; // COMM_BUNDLE max is 8.
  req.bundles.push_back(bundle);
  const auto result = state.GenerateFromBackboneSpec(req);
  return !result.ok && regex_contains(result.error, "out of template range");
}

// Intent: Multiple bundle template requests should generate multiple bundles and combined spans.
bool test_backbone_bundle_template_multi_request_generates_multiple_bundles() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {16.0, 0.0, 0.0}};
  req.interval_m = 8.0;
  req.pole_type_id = type_ids.front();

  wire::core::BackboneBundleSpec lv{};
  lv.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  lv.count = 2;
  req.bundles.push_back(lv);

  wire::core::BackboneBundleSpec comm{};
  comm.bundle_template_id = wire::core::BundleKind::kCommunication;
  comm.count = 1;
  req.bundles.push_back(comm);

  const auto result = state.GenerateFromBackboneSpec(req);
  if (!result.ok) {
    return false;
  }
  if (result.value.bundle_ids.size() != 2 || result.value.generated_pole_ids.size() < 2) {
    return false;
  }

  const std::size_t segment_count = result.value.generated_pole_ids.size() - 1;
  const std::size_t expected_span_count = segment_count * 3; // LV2 + COMM1
  if (result.value.generated_span_ids.size() != expected_span_count) {
    return false;
  }

  std::size_t low_voltage_bundle_count = 0;
  std::size_t communication_bundle_count = 0;
  for (ObjectId bundle_id : result.value.bundle_ids) {
    const auto* bundle = state.view().edit_state().bundles.find(bundle_id);
    if (bundle == nullptr) {
      return false;
    }
    if (bundle->kind == wire::core::BundleKind::kLowVoltage) {
      ++low_voltage_bundle_count;
    } else if (bundle->kind == wire::core::BundleKind::kCommunication) {
      ++communication_bundle_count;
    }
  }
  return low_voltage_bundle_count == 1 && communication_bundle_count == 1;
}

// Intent: Newly created ports should default to Auto position mode.
bool test_port_position_mode_defaults_auto() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const auto add = state.AddPort(pole, {0.0, 0.0, 6.0}, PortKind::kPower, PortLayer::kLowVoltage);
  if (!add.ok) {
    return false;
  }
  const auto* port = state.view().edit_state().ports.find(add.value);
  if (port == nullptr) {
    return false;
  }
  return port->position_mode == wire::core::PortPositionMode::kAuto && !port->user_edited_position;
}

// Intent: Manual set/reset APIs should transition mode and keep local dirty scope.
bool test_port_manual_set_and_reset_to_auto() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {10.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, type_ids.front());
  (void)state.ApplyPoleType(pole_b, type_ids.front());
  const auto add = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage);
  if (!add.ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;

  const ObjectId port_id = add.value.port_a_id;
  const wire::core::Vec3d manual_pos{123.0, 45.0, 9.0};
  const auto manual = state.SetPortWorldPositionManual(port_id, manual_pos);
  if (!manual.ok) {
    return false;
  }
  const auto* runtime = state.view().find_span_runtime_state(add.value.span_id);
  const auto* manual_port = state.view().edit_state().ports.find(port_id);
  if (runtime == nullptr || manual_port == nullptr) {
    return false;
  }
  if (!has_dirty(runtime, DirtyBits::kGeometry)) {
    return false;
  }
  if (manual_port->position_mode != wire::core::PortPositionMode::kManual ||
      manual_port->placement_source != wire::core::PortPlacementSourceKind::kManualEdit ||
      !manual_port->user_edited_position || !almost_equal(manual_port->world_position, manual_pos)) {
    return false;
  }

  const auto reset = state.ResetPortPositionToAuto(port_id);
  if (!reset.ok) {
    return false;
  }
  const auto* reset_port = state.view().edit_state().ports.find(port_id);
  if (reset_port == nullptr) {
    return false;
  }
  return reset_port->position_mode == wire::core::PortPositionMode::kAuto && !reset_port->user_edited_position &&
         reset_port->placement_source != wire::core::PortPlacementSourceKind::kManualEdit &&
         !almost_equal(reset_port->world_position, manual_pos);
}

// Intent: Manual port position must not be overwritten by auto pole flip recomputation.
bool test_manual_port_not_overwritten_by_auto_relayout() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {12.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, type_ids.front());
  (void)state.ApplyPoleType(pole_b, type_ids.front());
  const auto add = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage);
  if (!add.ok) {
    return false;
  }
  const ObjectId manual_port_id = add.value.port_a_id;
  const wire::core::Vec3d manual_pos{77.0, -12.0, 8.0};
  if (!state.SetPortWorldPositionManual(manual_port_id, manual_pos).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  if (!state.SetPoleFlip180(pole_a, true).ok) {
    return false;
  }
  const auto* port_after = state.view().edit_state().ports.find(manual_port_id);
  if (port_after == nullptr) {
    return false;
  }
  return port_after->position_mode == wire::core::PortPositionMode::kManual &&
         almost_equal(port_after->world_position, manual_pos);
}

// Intent: Pole transform changes must reproject owned Auto ports while preserving Manual ports.
bool test_move_pole_reprojects_auto_ports_and_preserves_manual_ports() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::Transformd tf{};
  tf.position = {0.0, 0.0, 0.0};
  const ObjectId pole_id = state.AddPole(tf, 10.0, "P").value;
  if (!state.ApplyPoleType(pole_id, type_ids.front()).ok) {
    return false;
  }

  ObjectId auto_port_id = wire::core::kInvalidObjectId;
  ObjectId manual_port_id = wire::core::kInvalidObjectId;
  wire::core::Vec3d auto_before{};
  wire::core::Vec3d manual_before{};
  for (const auto& port : state.view().edit_state().ports.items()) {
    if (port.owner_pole_id != pole_id) {
      continue;
    }
    if (port.source_slot_id == 200) {
      auto_port_id = port.id;
      auto_before = port.world_position;
    } else if (port.source_slot_id == 201) {
      manual_port_id = port.id;
      manual_before = port.world_position;
    }
  }
  if (auto_port_id == wire::core::kInvalidObjectId || manual_port_id == wire::core::kInvalidObjectId) {
    return false;
  }
  if (!state.SetPortWorldPositionManual(manual_port_id, manual_before).ok) {
    return false;
  }

  wire::core::Transformd moved{};
  moved.position = {3.0, 4.0, 0.0};
  moved.rotation_euler_deg.z = 90.0;
  if (!state.MovePole(pole_id, moved).ok) {
    return false;
  }

  const auto* auto_after = state.view().edit_state().ports.find(auto_port_id);
  const auto* manual_after = state.view().edit_state().ports.find(manual_port_id);
  if (auto_after == nullptr || manual_after == nullptr) {
    return false;
  }
  const bool auto_moved = !almost_equal(auto_after->world_position, auto_before, 1e-6);
  const bool manual_kept = almost_equal(manual_after->world_position, manual_before, 1e-6);
  return auto_moved && manual_kept && manual_after->position_mode == wire::core::PortPositionMode::kManual;
}

// Intent: Guide generation keeps explicitly pinned poles fixed when the path is extended.
bool test_generate_from_guide_keeps_manual_boundaries_stable() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::GenerationRequest req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  req.category = ConnectionCategory::kLowVoltage;
  req.requested_lane_count = 1;
  const auto first = state.GenerateFromGuide(req);
  if (!first.ok || first.value.generated_pole_ids.size() < 2) {
    return false;
  }

  auto find_pole_at = [&](const wire::core::Vec3d& pos) -> const wire::core::Pole* {
    const wire::core::Pole* found = nullptr;
    for (const auto& pole : state.view().edit_state().poles.items()) {
      if (almost_equal(pole.world_transform.position, pos)) {
        if (found == nullptr || pole.placement_mode == wire::core::PlacementMode::kManual) {
          found = &pole;
        }
      }
    }
    return found;
  };

  const auto* start_before = find_pole_at({0.0, 0.0, 0.0});
  const auto* mid_before = find_pole_at({20.0, 0.0, 0.0});
  if (start_before == nullptr || mid_before == nullptr) {
    return false;
  }
  const ObjectId start_id = start_before->id;
  const ObjectId mid_id = mid_before->id;
  const auto start_pos_before = start_before->world_transform.position;
  const auto mid_pos_before = mid_before->world_transform.position;
  if (!state.SetPolePlacementMode(start_id, wire::core::PlacementMode::kManual).ok ||
      !state.SetPolePlacementMode(mid_id, wire::core::PlacementMode::kManual).ok) {
    return false;
  }
  start_before = state.view().edit_state().poles.find(start_id);
  mid_before = state.view().edit_state().poles.find(mid_id);
  if (start_before == nullptr || mid_before == nullptr ||
      start_before->placement_mode != wire::core::PlacementMode::kManual ||
      mid_before->placement_mode != wire::core::PlacementMode::kManual) {
    return false;
  }

  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {40.0, 0.0, 0.0}};
  const auto second = state.GenerateFromGuide(req);
  if (!second.ok) {
    return false;
  }
  const auto* start_after = state.view().edit_state().poles.find(start_id);
  const auto* mid_after = state.view().edit_state().poles.find(mid_id);
  if (start_after == nullptr || mid_after == nullptr) {
    return false;
  }
  return almost_equal(start_after->world_transform.position, start_pos_before) &&
         almost_equal(mid_after->world_transform.position, mid_pos_before) &&
         start_after->placement_mode == wire::core::PlacementMode::kManual &&
         mid_after->placement_mode == wire::core::PlacementMode::kManual;
}

// Intent: Guide regeneration should avoid full rebuild and add only missing tail segments.
bool test_generate_from_guide_local_update_no_duplicate_unchanged_segments() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::GenerationRequest req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  req.category = ConnectionCategory::kLowVoltage;
  req.requested_lane_count = 1;

  const auto first = state.GenerateFromGuide(req);
  if (!first.ok || first.value.generated_span_ids.empty()) {
    return false;
  }
  const std::size_t spans_after_first = state.view().edit_state().spans.size();

  const auto second = state.GenerateFromGuide(req);
  if (!second.ok) {
    return false;
  }
  if (!second.value.generated_span_ids.empty()) {
    return false;
  }
  if (state.view().edit_state().spans.size() != spans_after_first) {
    return false;
  }

  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
  const auto third = state.GenerateFromGuide(req);
  if (!third.ok || third.value.generated_span_ids.empty()) {
    return false;
  }
  return state.view().edit_state().spans.size() > spans_after_first;
}

// Intent: Guide polyline vertices should not become forced-manual by default.
bool test_generate_from_guide_vertices_are_not_forced_manual_by_default() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::GenerationRequest req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  req.category = ConnectionCategory::kLowVoltage;
  req.requested_lane_count = 1;
  const auto generated = state.GenerateFromGuide(req);
  if (!generated.ok) {
    return false;
  }

  auto find_pole_at = [&](const wire::core::Vec3d& pos) -> const wire::core::Pole* {
    for (const auto& pole : state.view().edit_state().poles.items()) {
      if (almost_equal(pole.world_transform.position, pos)) {
        return &pole;
      }
    }
    return nullptr;
  };

  const auto* start = find_pole_at({0.0, 0.0, 0.0});
  const auto* mid = find_pole_at({10.0, 0.0, 0.0});
  const auto* end = find_pole_at({20.0, 0.0, 0.0});
  if (start == nullptr || mid == nullptr || end == nullptr) {
    return false;
  }
  return start->placement_mode == wire::core::PlacementMode::kAuto &&
         end->placement_mode == wire::core::PlacementMode::kAuto &&
         mid->placement_mode == wire::core::PlacementMode::kAuto;
}

// Intent: pin_vertices option should allow explicit manual pinning of guide vertices.
bool test_generate_from_guide_pin_vertices_option() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::GenerationRequest req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  req.category = ConnectionCategory::kLowVoltage;
  req.requested_lane_count = 1;
  req.pole_placement.pin_vertices = true;
  const auto generated = state.GenerateFromGuide(req);
  if (!generated.ok) {
    return false;
  }

  for (const auto& pole : state.view().edit_state().poles.items()) {
    if (almost_equal(pole.world_transform.position, wire::core::Vec3d{10.0, 0.0, 0.0})) {
      return pole.placement_mode == wire::core::PlacementMode::kManual;
    }
  }
  return false;
}

// Intent: Pole placement mode should be user-switchable between Auto and Manual.
bool test_set_pole_placement_mode_auto_manual_roundtrip() {
  CoreState state;
  const ObjectId pole_id = state.AddPole({}, 10.0, "P").value;
  const auto* pole = state.view().edit_state().poles.find(pole_id);
  if (pole == nullptr || pole->placement_mode != wire::core::PlacementMode::kAuto) {
    return false;
  }
  const auto to_manual = state.SetPolePlacementMode(pole_id, wire::core::PlacementMode::kManual);
  if (!to_manual.ok) {
    return false;
  }
  pole = state.view().edit_state().poles.find(pole_id);
  if (pole == nullptr || pole->placement_mode != wire::core::PlacementMode::kManual || !pole->user_edited) {
    return false;
  }
  const auto to_auto = state.SetPolePlacementMode(pole_id, wire::core::PlacementMode::kAuto);
  if (!to_auto.ok) {
    return false;
  }
  pole = state.view().edit_state().poles.find(pole_id);
  return pole != nullptr && pole->placement_mode == wire::core::PlacementMode::kAuto && !pole->user_edited;
}

// Intent: Session-local regeneration should preserve manual poles while updating auto-generated parts.
bool test_regenerate_session_auto_parts_keeps_manual_pole() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::GenerationRequest req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  req.category = ConnectionCategory::kLowVoltage;
  req.requested_lane_count = 1;
  const auto first = state.GenerateFromGuide(req);
  if (!first.ok || first.value.generated_pole_ids.empty()) {
    return false;
  }
  const auto* any_generated = state.view().edit_state().poles.find(first.value.generated_pole_ids.front());
  if (any_generated == nullptr || any_generated->generation.generation_session_id == 0) {
    return false;
  }
  const std::uint64_t session_id = any_generated->generation.generation_session_id;

  ObjectId middle_id = wire::core::kInvalidObjectId;
  for (const auto& pole : state.view().edit_state().poles.items()) {
    if (pole.generation.generation_session_id == session_id &&
        almost_equal(pole.world_transform.position, wire::core::Vec3d{10.0, 0.0, 0.0})) {
      middle_id = pole.id;
      break;
    }
  }
  if (middle_id == wire::core::kInvalidObjectId) {
    return false;
  }

  if (!state.SetPolePlacementMode(middle_id, wire::core::PlacementMode::kManual).ok) {
    return false;
  }
  wire::core::Transformd moved{};
  moved.position = {11.0, 1.0, 0.0};
  if (!state.MovePole(middle_id, moved).ok) {
    return false;
  }

  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
  const auto regen = state.RegenerateSessionAutoParts(session_id, req);
  if (!regen.ok) {
    return false;
  }

  const auto* middle_after = state.view().edit_state().poles.find(middle_id);
  if (middle_after == nullptr) {
    return false;
  }
  return middle_after->placement_mode == wire::core::PlacementMode::kManual &&
         almost_equal(middle_after->world_transform.position, moved.position);
}

// Intent: Session-local regeneration must not overwrite manual port position edits.
bool test_regenerate_session_auto_parts_keeps_manual_port() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::GenerationRequest req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  req.category = ConnectionCategory::kLowVoltage;
  req.requested_lane_count = 1;
  const auto first = state.GenerateFromGuide(req);
  if (!first.ok || first.value.generated_span_ids.empty()) {
    return false;
  }

  const auto* any_generated = state.view().edit_state().poles.find(first.value.generated_pole_ids.front());
  if (any_generated == nullptr || any_generated->generation.generation_session_id == 0) {
    return false;
  }
  const std::uint64_t session_id = any_generated->generation.generation_session_id;

  const auto* span = state.view().edit_state().spans.find(first.value.generated_span_ids.front());
  if (span == nullptr) {
    return false;
  }
  const auto* port = state.view().edit_state().ports.find(span->port_a_id);
  if (port == nullptr || port->owner_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const ObjectId manual_port_id = port->id;
  const ObjectId manual_port_owner = port->owner_pole_id;
  if (!state.SetPolePlacementMode(manual_port_owner, wire::core::PlacementMode::kManual).ok) {
    return false;
  }
  const wire::core::Vec3d manual_pos{77.0, -14.0, 8.0};
  if (!state.SetPortWorldPositionManual(manual_port_id, manual_pos).ok) {
    return false;
  }

  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {35.0, 0.0, 0.0}};
  const auto regen = state.RegenerateSessionAutoParts(session_id, req);
  if (!regen.ok) {
    return false;
  }

  const auto* port_after = state.view().edit_state().ports.find(manual_port_id);
  return port_after != nullptr && port_after->position_mode == wire::core::PortPositionMode::kManual &&
         almost_equal(port_after->world_position, manual_pos);
}

// Intent: Manual port edits must remain stable even when the owner pole stays Auto during session regeneration.
bool test_regenerate_session_auto_parts_keeps_manual_port_on_auto_pole() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::GenerationRequest req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  req.category = ConnectionCategory::kLowVoltage;
  req.requested_lane_count = 1;
  const auto first = state.GenerateFromGuide(req);
  if (!first.ok || first.value.generated_span_ids.empty() || first.value.generated_pole_ids.empty()) {
    return false;
  }

  const auto* any_generated = state.view().edit_state().poles.find(first.value.generated_pole_ids.front());
  if (any_generated == nullptr || any_generated->generation.generation_session_id == 0) {
    return false;
  }
  const std::uint64_t session_id = any_generated->generation.generation_session_id;

  const auto* span = state.view().edit_state().spans.find(first.value.generated_span_ids.front());
  if (span == nullptr) {
    return false;
  }
  const auto* port = state.view().edit_state().ports.find(span->port_a_id);
  if (port == nullptr || port->owner_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const ObjectId manual_port_id = port->id;
  const ObjectId owner_pole_id = port->owner_pole_id;
  const auto* owner_before = state.view().edit_state().poles.find(owner_pole_id);
  if (owner_before == nullptr || owner_before->placement_mode != wire::core::PlacementMode::kAuto) {
    return false;
  }

  const wire::core::Vec3d manual_pos{61.0, -9.0, 7.0};
  if (!state.SetPortWorldPositionManual(manual_port_id, manual_pos).ok) {
    return false;
  }

  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {35.0, 0.0, 0.0}};
  const auto regen = state.RegenerateSessionAutoParts(session_id, req);
  if (!regen.ok) {
    return false;
  }

  const auto* owner_after = state.view().edit_state().poles.find(owner_pole_id);
  const auto* port_after = state.view().edit_state().ports.find(manual_port_id);
  if (owner_after == nullptr || port_after == nullptr) {
    return false;
  }
  return owner_after->placement_mode == wire::core::PlacementMode::kAuto &&
         port_after->position_mode == wire::core::PortPositionMode::kManual &&
         almost_equal(port_after->world_position, manual_pos);
}

// Intent: Session-local regeneration must not mutate objects that belong to other sessions.
bool test_regenerate_session_auto_parts_isolation_across_sessions() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::GenerationRequest req_a{};
  req_a.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  req_a.interval_m = 10.0;
  req_a.pole_type_id = type_ids.front();
  req_a.category = ConnectionCategory::kLowVoltage;
  req_a.requested_lane_count = 1;
  const auto gen_a = state.GenerateFromGuide(req_a);
  if (!gen_a.ok || gen_a.value.generated_pole_ids.empty()) {
    return false;
  }
  const auto* pole_a0 = state.view().edit_state().poles.find(gen_a.value.generated_pole_ids.front());
  if (pole_a0 == nullptr || pole_a0->generation.generation_session_id == 0) {
    return false;
  }
  const std::uint64_t session_a = pole_a0->generation.generation_session_id;

  wire::core::GenerationRequest req_b{};
  req_b.path.polyline = {{0.0, 30.0, 0.0}, {20.0, 30.0, 0.0}};
  req_b.interval_m = 10.0;
  req_b.pole_type_id = type_ids.front();
  req_b.category = ConnectionCategory::kLowVoltage;
  req_b.requested_lane_count = 1;
  const auto gen_b = state.GenerateFromGuide(req_b);
  if (!gen_b.ok || gen_b.value.generated_pole_ids.empty()) {
    return false;
  }
  const auto* pole_b0 = state.view().edit_state().poles.find(gen_b.value.generated_pole_ids.front());
  if (pole_b0 == nullptr || pole_b0->generation.generation_session_id == 0) {
    return false;
  }
  const std::uint64_t session_b = pole_b0->generation.generation_session_id;

  std::vector<std::pair<ObjectId, wire::core::Vec3d>> session_b_poles{};
  std::vector<ObjectId> session_b_spans{};
  for (const auto& pole : state.view().edit_state().poles.items()) {
    if (pole.generation.generation_session_id == session_b) {
      session_b_poles.push_back({pole.id, pole.world_transform.position});
    }
  }
  for (const auto& span : state.view().edit_state().spans.items()) {
    if (span.generation.generation_session_id == session_b) {
      session_b_spans.push_back(span.id);
    }
  }
  if (session_b_poles.empty()) {
    return false;
  }

  req_a.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
  const auto regen = state.RegenerateSessionAutoParts(session_a, req_a);
  if (!regen.ok) {
    return false;
  }

  for (const auto& [pole_id, pos] : session_b_poles) {
    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr || !almost_equal(pole->world_transform.position, pos)) {
      return false;
    }
  }
  for (ObjectId span_id : session_b_spans) {
    if (state.view().edit_state().spans.find(span_id) == nullptr) {
      return false;
    }
  }
  return true;
}

// Intent: Backbone graph should be built from wire-grouped spans and support route search.
bool test_backbone_edges_and_route_search() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::GenerationRequest req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 0.0, 0.0}};
  req.interval_m = 6.0;
  req.pole_type_id = type_ids.front();
  req.category = ConnectionCategory::kHighVoltage;
  req.requested_lane_count = 3;
  const auto generated = state.GenerateFromGuide(req);
  if (!generated.ok || generated.value.generated_pole_ids.size() < 2) {
    return false;
  }

  const auto edges = state.BuildBackboneEdges();
  if (edges.empty()) {
    return false;
  }
  bool has_grouped_edge = false;
  for (const auto& edge : edges) {
    if (!edge.bundles.empty()) {
      has_grouped_edge = true;
      break;
    }
  }
  if (!has_grouped_edge) {
    return false;
  }

  const ObjectId start = generated.value.generated_pole_ids.front();
  const ObjectId end = generated.value.generated_pole_ids.back();
  const auto route = state.FindBackboneRoute(start, end);
  return !route.empty() && route.front() == start && route.back() == end;
}

// Intent: Pole flip_180 should rotate owned ports and dirty only connected spans.
bool test_set_pole_flip180_updates_ports_and_dirty() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {12.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, type_ids.front());
  (void)state.ApplyPoleType(pole_b, type_ids.front());
  const auto add = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage);
  if (!add.ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;

  const auto* port_before = state.view().edit_state().ports.find(add.value.port_a_id);
  if (port_before == nullptr) {
    return false;
  }
  const wire::core::Vec3d before_pos = port_before->world_position;
  const auto flip = state.SetPoleFlip180(pole_a, true);
  if (!flip.ok) {
    return false;
  }

  const auto* port_after = state.view().edit_state().ports.find(add.value.port_a_id);
  const auto* runtime = state.view().find_span_runtime_state(add.value.span_id);
  if (port_after == nullptr || runtime == nullptr) {
    return false;
  }
  if (almost_equal(before_pos, port_after->world_position)) {
    return false;
  }
  if (!has_dirty(runtime, DirtyBits::kGeometry)) {
    return false;
  }
  const auto* pole = state.view().edit_state().poles.find(pole_a);
  return pole != nullptr && pole->orientation_control.flip_180;
}

// Intent: GenerateSimpleLine should fail when polyline has less than 2 points.
bool test_generate_simple_line_fails_with_short_polyline() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const CoreCounts before = snapshot_counts(state);
  wire::core::RoadSegment road{};
  road.id = 80;
  road.polyline = {{0.0, 0.0, 0.0}};
  const auto result = state.GenerateSimpleLine(road, 5.0, type_ids.front(), ConnectionCategory::kLowVoltage);
  if (result.ok) {
    return false;
  }
  if (result.error != "road polyline must contain at least 2 points") {
    return false;
  }
  if (!same_counts(before, snapshot_counts(state))) {
    return false;
  }

  // Recovery: next valid input should succeed.
  road.polyline.push_back({5.0, 0.0, 0.0});
  return state.GenerateSimpleLine(road, 5.0, type_ids.front(), ConnectionCategory::kLowVoltage).ok;
}

// Intent: GenerateSimpleLine should fail when interval is non-positive.
bool test_generate_simple_line_fails_with_invalid_interval() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const CoreCounts before = snapshot_counts(state);
  wire::core::RoadSegment road{};
  road.id = 81;
  road.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  const auto result = state.GenerateSimpleLine(road, 0.0, type_ids.front(), ConnectionCategory::kLowVoltage);
  if (result.ok) {
    return false;
  }
  if (result.error != "interval must be > 0") {
    return false;
  }
  if (!same_counts(before, snapshot_counts(state))) {
    return false;
  }
  return state.GenerateSimpleLine(road, 5.0, type_ids.front(), ConnectionCategory::kLowVoltage).ok;
}

// Intent: AddConnectionByPole should fail for same pole and allow recovery.
bool test_add_connection_same_pole_fails_and_recovers() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok) {
    return false;
  }

  const CoreCounts before = snapshot_counts(state);
  const auto bad = state.AddConnectionByPole(pole_a, pole_a, ConnectionCategory::kLowVoltage);
  if (bad.ok || !regex_contains(bad.error, "same pole")) {
    return false;
  }
  if (!same_counts(before, snapshot_counts(state))) {
    return false;
  }

  wire::core::Transformd pole_b_tf{};
  pole_b_tf.position = {12.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(pole_b_tf, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_b, type_ids.front()).ok) {
    return false;
  }
  const ObjectId pa = state.AddPort(pole_a, {0.0, 2.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId pb = state.AddPort(pole_b, {12.0, 2.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  return state.AddSpan(pa, pb, SpanKind::kDistribution, SpanLayer::kLowVoltage).ok;
}

// Intent: AddSpan should reject missing ports and remain recoverable.
bool test_add_span_missing_ports_fails_and_recovers() {
  CoreState state;
  const CoreCounts before = snapshot_counts(state);
  const auto bad = state.AddSpan(111, 222, SpanKind::kDistribution, SpanLayer::kLowVoltage);
  if (bad.ok || bad.error != "span ports do not exist") {
    return false;
  }
  if (!same_counts(before, snapshot_counts(state))) {
    return false;
  }

  const ObjectId pole_a = state.AddPole({}, 9.0, "A").value;
  const ObjectId pole_b = state.AddPole({}, 9.0, "B").value;
  const ObjectId a = state.AddPort(pole_a, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole_b, {3.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  return state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).ok;
}

// Intent: SplitSpan should reject invalid t and remain recoverable.
bool test_split_span_invalid_t_fails_and_recovers() {
  CoreState state;
  const ObjectId pole_a = state.AddPole({}, 9.0, "A").value;
  const ObjectId pole_b = state.AddPole({}, 9.0, "B").value;
  const ObjectId a = state.AddPort(pole_a, {0.0, 0.0, 3.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole_b, {6.0, 0.0, 3.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;

  const auto bad = state.SplitSpan(span, 0.0);
  if (bad.ok || bad.error != "split t must be in (0, 1)") {
    return false;
  }
  if (state.view().edit_state().spans.find(span) == nullptr) {
    return false;
  }
  return state.SplitSpan(span, 0.5).ok;
}

// Intent: GenerateSpansBetweenPoles connects adjacent poles and keeps index/runtime consistent.
bool test_generate_spans_between_poles_basic() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  std::vector<ObjectId> poles;
  for (int i = 0; i < 4; ++i) {
    wire::core::Transformd tf{};
    tf.position = {static_cast<double>(i) * 10.0, 0.0, 0.0};
    const ObjectId pole_id = state.AddPole(tf, 10.0, "P", wire::core::PoleKind::kConcrete).value;
    if (!state.ApplyPoleType(pole_id, type_ids.front()).ok) {
      return false;
    }
    poles.push_back(pole_id);
  }

  const auto result = state.GenerateSpansBetweenPoles(poles, ConnectionCategory::kLowVoltage);
  if (!result.ok) {
    return false;
  }
  if (result.value.size() != poles.size() - 1) {
    return false;
  }
  for (ObjectId span_id : result.value) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    const auto* runtime = state.view().find_span_runtime_state(span_id);
    if (span == nullptr || runtime == nullptr) {
      return false;
    }
    if (!span->generation.generated || span->generation.source != wire::core::GenerationSource::kRoadAuto) {
      return false;
    }
    if (!has_dirty(runtime, DirtyBits::kGeometry)) {
      return false;
    }
  }
  return validate_now(state).ok();
}

// Intent: Repeated auto-connect passes should keep increasing spans (not capped to two lines).
bool test_generate_spans_between_poles_multiple_passes() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  std::vector<ObjectId> poles;
  for (int i = 0; i < 5; ++i) {
    wire::core::Transformd tf{};
    tf.position = {static_cast<double>(i) * 12.0, 0.0, 0.0};
    const ObjectId pole_id = state.AddPole(tf, 10.0, "P", wire::core::PoleKind::kConcrete).value;
    if (!state.ApplyPoleType(pole_id, type_ids.front()).ok) {
      return false;
    }
    poles.push_back(pole_id);
  }

  const std::size_t per_pass = poles.size() - 1;
  for (int pass = 0; pass < 6; ++pass) {
    const auto result = state.GenerateSpansBetweenPoles(poles, ConnectionCategory::kLowVoltage);
    if (!result.ok || result.value.size() != per_pass) {
      return false;
    }
    const std::size_t expected_total = per_pass * static_cast<std::size_t>(pass + 1);
    if (state.view().edit_state().spans.size() != expected_total) {
      return false;
    }
  }
  return validate_now(state).ok();
}

// Intent: LowVoltage auto-connect should use at least three distinct slots before reuse.
bool test_generate_spans_between_poles_uses_third_slot_before_reuse() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  std::vector<ObjectId> poles;
  for (int i = 0; i < 4; ++i) {
    wire::core::Transformd tf{};
    tf.position = {static_cast<double>(i) * 10.0, 0.0, 0.0};
    const ObjectId pole_id = state.AddPole(tf, 10.0, "P", wire::core::PoleKind::kConcrete).value;
    if (!state.ApplyPoleType(pole_id, type_ids.front()).ok) {
      return false;
    }
    poles.push_back(pole_id);
  }

  std::vector<int> slot_ids;
  for (int pass = 0; pass < 3; ++pass) {
    const auto result = state.GenerateSpansBetweenPoles(poles, ConnectionCategory::kLowVoltage);
    if (!result.ok || result.value.empty()) {
      return false;
    }
    const auto* first_span = state.view().edit_state().spans.find(result.value.front());
    if (first_span == nullptr) {
      return false;
    }
    const auto* first_port = state.view().edit_state().ports.find(first_span->port_a_id);
    if (first_port == nullptr || first_port->owner_pole_id != poles.front()) {
      return false;
    }
    if (first_port->source_slot_id < 0) {
      return false;
    }
    slot_ids.push_back(first_port->source_slot_id);
  }

  std::sort(slot_ids.begin(), slot_ids.end());
  slot_ids.erase(std::unique(slot_ids.begin(), slot_ids.end()), slot_ids.end());
  return slot_ids.size() >= 3;
}

// Intent: GenerateSimpleLine should create poles+spans and drive geometry/bounds recalc.
bool test_generate_simple_line_integration() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 44;
  road.polyline = {{0.0, 0.0, 0.0}, {15.0, 5.0, 0.0}, {30.0, 0.0, 0.0}};
  const auto result = state.GenerateSimpleLine(road, 6.0, type_ids.front(), ConnectionCategory::kLowVoltage);
  if (!result.ok) {
    return false;
  }
  if (result.value.pole_ids.size() < 3) {
    return false;
  }
  if (result.value.span_ids.size() != result.value.pole_ids.size() - 1) {
    return false;
  }
  if (result.value.generation_session_id == 0) {
    return false;
  }

  for (ObjectId pole_id : result.value.pole_ids) {
    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr || pole->generation.generation_session_id != result.value.generation_session_id) {
      return false;
    }
  }
  for (ObjectId span_id : result.value.span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr || span->generation.generation_session_id != result.value.generation_session_id) {
      return false;
    }
  }

  if (state.view().dirty_queue().geometry_dirty_span_ids.size() < result.value.span_ids.size()) {
    return false;
  }

  (void)state.Commit().recalc_stats;
  for (ObjectId span_id : result.value.span_ids) {
    const auto* runtime = state.view().find_span_runtime_state(span_id);
    const auto* curve = state.find_curve_cache(span_id);
    const auto* bounds = state.find_bounds_cache(span_id);
    if (runtime == nullptr || curve == nullptr || bounds == nullptr) {
      return false;
    }
    if (runtime->geometry_version != runtime->data_version || runtime->bounds_version != runtime->data_version ||
        runtime->render_version != runtime->data_version) {
      return false;
    }
  }
  return validate_now(state).ok();
}

// Intent: Auto-generated line should reuse the same intermediate pole port for through continuity.
bool test_generate_simple_line_reuses_intermediate_ports() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 55;
  road.polyline = {{0.0, 0.0, 0.0}, {40.0, 0.0, 0.0}};
  const auto result = state.GenerateSimpleLine(road, 10.0, type_ids.front(), ConnectionCategory::kLowVoltage);
  if (!result.ok || result.value.pole_ids.size() < 4) {
    return false;
  }

  for (std::size_t i = 1; i + 1 < result.value.pole_ids.size(); ++i) {
    const ObjectId pole_id = result.value.pole_ids[i];
    std::vector<ObjectId> used_ports;
    for (ObjectId span_id : result.value.span_ids) {
      const auto* span = state.view().edit_state().spans.find(span_id);
      if (span == nullptr) {
        return false;
      }
      const auto* port_a = state.view().edit_state().ports.find(span->port_a_id);
      const auto* port_b = state.view().edit_state().ports.find(span->port_b_id);
      if (port_a == nullptr || port_b == nullptr) {
        return false;
      }
      if (port_a->owner_pole_id == pole_id) {
        used_ports.push_back(port_a->id);
      }
      if (port_b->owner_pole_id == pole_id) {
        used_ports.push_back(port_b->id);
      }
    }
    if (used_ports.size() != 2) {
      return false;
    }
    if (used_ports[0] != used_ports[1]) {
      return false;
    }
  }
  return true;
}

// Intent: Display IDs should increment independently per prefix while ObjectId remains global.
bool test_display_id_is_per_prefix_sequence() {
  CoreState state;
  const ObjectId pole1 = state.AddPole({}, 10.0, "P1").value;
  const ObjectId pole2 = state.AddPole({}, 10.0, "P2").value;
  const ObjectId port1 = state.AddPort(pole1, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId port2 = state.AddPort(pole2, {1.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span1 = state.AddSpan(port1, port2, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;

  const auto* p1 = state.view().edit_state().poles.find(pole1);
  const auto* p2 = state.view().edit_state().poles.find(pole2);
  const auto* pt1 = state.view().edit_state().ports.find(port1);
  const auto* pt2 = state.view().edit_state().ports.find(port2);
  const auto* sp1 = state.view().edit_state().spans.find(span1);
  if (p1 == nullptr || p2 == nullptr || pt1 == nullptr || pt2 == nullptr || sp1 == nullptr) {
    return false;
  }

  return p1->display_id == "P-000001" && p2->display_id == "P-000002" && pt1->display_id == "PT-000001" &&
         pt2->display_id == "PT-000002" && starts_with(sp1->display_id, "SP-000001");
}

// Intent: Viewer default demo should have enough spans to visually inspect behavior.
bool test_demo_state_has_dense_spans() {
  CoreState state = wire::core::make_demo_state();
  return state.view().edit_state().poles.size() >= 3 && state.view().edit_state().spans.size() >= 10 &&
         state.view().edit_state().ports.size() >= 20;
}

// Intent: Clearing session debug records must not mutate persistent entities or connectivity.
bool test_clear_debug_records_is_entity_noop() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  wire::core::Transformd pole_a_tf{};
  pole_a_tf.position = {0.0, 0.0, 0.0};
  wire::core::Transformd pole_b_tf{};
  pole_b_tf.position = {12.0, 0.0, 0.0};
  const auto add_a = state.AddPole(pole_a_tf, 10.0, "A");
  const auto add_b = state.AddPole(pole_b_tf, 10.0, "B");
  if (!add_a.ok || !add_b.ok) {
    return false;
  }
  const ObjectId pole_a = add_a.value;
  const ObjectId pole_b = add_b.value;
  if (!state.ApplyPoleType(pole_a, pole_type_ids[0]).ok || !state.ApplyPoleType(pole_b, pole_type_ids[0]).ok) {
    return false;
  }
  if (!state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage).ok) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 10;
  road.polyline = {{0.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, {12.0, 3.0, 0.0}};
  CoreState::GenerateGroupedLineOptions options{};
  options.road = road;
  options.interval = 0.0;
  options.pole_type_id = pole_type_ids[0];
  options.group_spec.category = ConnectionCategory::kLowVoltage;
  options.group_spec.conductor_count = 2;
  const auto grouped = state.GenerateGroupedLine(options);
  if (!grouped.ok) {
    return false;
  }

  const std::size_t slot_debug_before = state.view().slot_selection_debug_records().size();
  const std::size_t path_debug_before = state.view().path_direction_debug_records().size();

  const CoreCounts before = snapshot_counts(state);
  const auto poles_before = collect_sorted_ids(state.view().edit_state().poles.items());
  const auto ports_before = collect_sorted_ids(state.view().edit_state().ports.items());
  const auto spans_before = collect_sorted_ids(state.view().edit_state().spans.items());
  const bool validate_before = validate_now(state).ok();

  state.clear_slot_selection_debug_records();
  state.clear_path_direction_debug_records();

  const CoreCounts after = snapshot_counts(state);
  const auto poles_after = collect_sorted_ids(state.view().edit_state().poles.items());
  const auto ports_after = collect_sorted_ids(state.view().edit_state().ports.items());
  const auto spans_after = collect_sorted_ids(state.view().edit_state().spans.items());

  return slot_debug_before >= state.view().slot_selection_debug_records().size() &&
         path_debug_before >= state.view().path_direction_debug_records().size() &&
         state.view().slot_selection_debug_records().empty() && state.view().path_direction_debug_records().empty() &&
         same_counts(before, after) && poles_before == poles_after && ports_before == ports_after &&
         spans_before == spans_after && validate_now(state).ok() == validate_before;
}

// Intent: Rebuilding derived caches must not change entity identity/counts.
bool test_recalc_cache_pipeline_is_entity_noop() {
  CoreState state = wire::core::make_demo_state();
  if (state.view().edit_state().spans.empty()) {
    return false;
  }

  const CoreCounts before = snapshot_counts(state);
  const auto poles_before = collect_sorted_ids(state.view().edit_state().poles.items());
  const auto ports_before = collect_sorted_ids(state.view().edit_state().ports.items());
  const auto spans_before = collect_sorted_ids(state.view().edit_state().spans.items());

  (void)state.Commit().recalc_stats;
  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.curve_samples = std::max(2, settings.curve_samples + 2);
  const auto update = state.UpdateGeometrySettings(settings, true);
  if (!update.ok) {
    return false;
  }
  const auto recalc = state.Commit().recalc_stats;

  const CoreCounts after = snapshot_counts(state);
  const auto poles_after = collect_sorted_ids(state.view().edit_state().poles.items());
  const auto ports_after = collect_sorted_ids(state.view().edit_state().ports.items());
  const auto spans_after = collect_sorted_ids(state.view().edit_state().spans.items());

  return recalc.geometry_processed > 0 && same_counts(before, after) && poles_before == poles_after &&
         ports_before == ports_after && spans_before == spans_after && validate_now(state).ok();
}

} // namespace

int main() {
  const std::vector<TestCase> tests = {
      {"C01_IdGenerator", "IdGenerator monotonic and reset behavior", "Exact", false,
       test_id_generator_monotonic_and_reset},
      {"C02_ObjectStore", "ObjectStore add/find/remove integrity", "Exact", false, test_object_store_integrity},
      {"C03_Phase3_SpanRuntime_Initialized", "AddSpan initializes runtime+dirty", "Exact", false,
       test_span_runtime_initialized_on_add},
      {"C04_Phase3_MovePole_LocalDirty", "MovePole dirties only related span", "Invariant", false,
       test_move_pole_dirties_only_related_span},
      {"C05_Phase3_SplitSpan_Basic", "SplitSpan replaces structure and keeps integrity", "Invariant", false,
       test_split_span_creates_two_spans_and_port},
      {"C06_Phase35_ApplyPoleType_GeneratesPorts", "ApplyPoleType generates owned template ports", "Invariant", false,
       test_apply_pole_type_generates_template_ports},
      {"C07_Phase35_PoleType_DifferentLayouts", "Different pole types produce different slot layout", "Invariant",
       false, test_different_pole_types_produce_different_port_layouts},
      {"C08_Phase35_AutoAlloc_UnusedPriority", "Auto allocation prefers unused slots", "Invariant", false,
       test_auto_port_allocation_prefers_unused_slots},
      {"C09_Phase35_AddConnectionByPole_Basic", "Pole->Pole connection updates dirty/index/changeset", "Invariant",
       false, test_add_connection_by_pole_updates_dirty_version_and_indices},
      {"C10_AddConnectionByPole_FailSamePole", "Same-pole connect fails with diagnostics and recovers", "Exact", true,
       test_add_connection_same_pole_fails_and_recovers},
      {"C11_Phase35_AddDropFromPole_Basic", "Drop from pole creates service span", "Invariant", false,
       test_add_drop_from_pole_creates_service_connection},
      {"C12_Phase35_AddDropFromSpan_Basic", "Drop from span splits and connects", "Invariant", false,
       test_add_drop_from_span_splits_and_connects_drop},
      {"C13_Phase4_Curve_LineDeterministic", "Line mode curve cache is deterministic", "Exact", false,
       test_curve_cache_line_mode_is_deterministic},
      {"C14_Phase4_Curve_SagBasic", "Sag mode changes midpoint and keeps endpoints", "Invariant", false,
       test_sag_mode_changes_midpoint_and_keeps_endpoints},
      {"C15_Phase4_DirtyVersion_LocalGeometryBounds", "Geometry dirty propagates to bounds/render locally", "Exact",
       false, test_geometry_bounds_version_follow_and_locality},
      {"C16_Phase4_Bounds_Generated", "Bounds cache is generated and valid", "Invariant", false,
       test_bounds_cache_generated_and_valid},
      {"C17_Phase4_Bounds_FollowGeometry", "Bounds follows geometry setting change", "Invariant", false,
       test_bounds_follow_geometry_change},
      {"C18_Phase4_DemoState_Dense", "Demo state has dense enough spans for initial viewer", "Invariant", false,
       test_demo_state_has_dense_spans},
      {"C19_Phase45_GeneratePolesAlongRoad_Basic", "Road interval generates pole line with pole type", "Invariant",
       false, test_generate_poles_along_road_basic},
      {"C20_Phase46_GenerateSimpleLine_FailShortPolyline", "Simple line short polyline fails with state unchanged",
       "Exact", true, test_generate_simple_line_fails_with_short_polyline},
      {"C21_Phase46_GenerateSimpleLine_FailInvalidInterval", "Simple line invalid interval fails with state unchanged",
       "Exact", true, test_generate_simple_line_fails_with_invalid_interval},
      {"C22_AddSpan_FailMissingPorts", "AddSpan invalid port failure leaves state recoverable", "Exact", true,
       test_add_span_missing_ports_fails_and_recovers},
      {"C23_SplitSpan_FailInvalidT", "SplitSpan invalid t failure leaves state recoverable", "Exact", true,
       test_split_span_invalid_t_fails_and_recovers},
      {"C24_Phase45_GenerateSpansBetweenPoles_Basic", "Adjacent poles are auto connected", "Exact", false,
       test_generate_spans_between_poles_basic},
      {"C25_Phase45_GenerateSpansBetweenPoles_MultiPass", "Repeated auto-connect adds more spans", "Exact", false,
       test_generate_spans_between_poles_multiple_passes},
      {"C26_Phase47_AutoConnect_UsesThirdSlot", "Auto-connect uses at least third low-voltage slot before reuse",
       "Invariant", false, test_generate_spans_between_poles_uses_third_slot_before_reuse},
      {"C27_Phase45_GenerateSimpleLine_Integration", "Simple line generation integrates dirty/recalc/caches",
       "Invariant", false, test_generate_simple_line_integration},
      {"C28_Phase45_GenerateSimpleLine_Continuity", "Intermediate poles reuse same through-port", "Invariant", false,
       test_generate_simple_line_reuses_intermediate_ports},
      {"C29_Phase45_DisplayId_PerPrefix", "Display IDs increment per prefix", "Exact", false,
       test_display_id_is_per_prefix_sequence},
      {"C30_Phase47_PoleContext_Classification", "Pole context classification marks terminal/straight/corner",
       "Invariant", false, test_pole_context_classification_basic},
      {"C31_Phase47_AngleCorrection_Bounds", "Angle correction side scale stays finite and bounded", "Invariant", false,
       test_angle_correction_bounds_and_finite},
      {"C32_Phase47_SlotSelection_ContextBias", "Branch context biases slot choice away from trunk-only", "Invariant",
       false, test_slot_selection_context_bias},
      {"C33_Phase47_SlotSelection_DeterministicDebug", "Slot tie-break is deterministic and debug record is coherent",
       "Exact", false, test_slot_selection_deterministic_and_debug_integrity},
      {"C34_Phase47_GenerateSimpleLine_CornerContext", "Corner path generation uses corner context on spans",
       "Invariant", false, test_generate_simple_line_corner_context_integration},
      {"C35_Phase47_CornerTurnSign_OuterBias", "Corner turn sign expands outer side more than inner side", "Invariant",
       false, test_corner_turn_sign_biases_outer_side},
      {"C61_Phase48h_AcuteCorner_AutoWidenSpacing",
       "Acute corners auto-widen lane spacing without category-specific branching", "Invariant", false,
       test_acute_corner_auto_widens_lane_spacing},
      {"C62_Phase48d_GroupLane_NoInversionOnU",
       "Grouped lane assignment on U-path avoids per-segment lane-order inversions", "Invariant", false,
       test_grouped_line_lane_order_no_inversion_on_u_path},
      {"C76_Phase49_GroupLane_AcuteNoXYCrossing",
       "Grouped lane assignment on acute-corner path avoids per-segment XY crossings", "Invariant", false,
       test_grouped_line_acute_corner_no_xy_crossing},
      {"C63_Phase48d_GroupLane_MirrorMetricNonRegression",
       "Enabling lane mirror does not worsen Y/Z/layer grouped-lane quality metrics", "Invariant", false,
       test_grouped_line_mirror_metric_non_regression},
      {"C36_Phase47_DrawPath_ClickPointsExact", "DrawPath generation uses clicked points directly and sets pole yaw",
       "Exact", false, test_generate_simple_line_from_points_exact_poles_and_orientation},
      {"C37_Phase48_PreferredSide_Geometry", "Preferred side is decided by peer geometry", "Invariant", false,
       test_preferred_side_uses_geometry},
      {"C38_Phase48_GroupedLine_HV3", "Grouped line generation creates 3-lane high-voltage spans", "Invariant", false,
       test_generate_grouped_line_high_voltage_three_phase},
      {"C39_Phase48_Direction_ForcedReverse", "Grouped line honors forced reverse direction", "Exact", false,
       test_generate_grouped_line_direction_forced_reverse},
      {"C40_Phase48_PoleFlip180_Dirty", "Pole flip180 updates owned ports and dirties connected spans", "Invariant",
       false, test_set_pole_flip180_updates_ports_and_dirty},
      {"C41_Phase4x_ClearDebug_NoEntityMutation", "Clearing session debug records does not mutate core entities",
       "Exact", false, test_clear_debug_records_is_entity_noop},
      {"C42_Phase4x_RecalcCache_NoEntityMutation", "Derived cache rebuild does not mutate entity identity/counts",
       "Invariant", false, test_recalc_cache_pipeline_is_entity_noop},
      {"C43_Phase4x_SharpCorner_SideAxisPerpendicular",
       "Sharp-corner pole side axis is perpendicular to bisector and points away from inward side", "Invariant", false,
       test_generate_simple_line_from_points_sharp_corner_perpendicular_orientation},
      {"C44_Phase48a_Bundle_AssignQuery", "Bundle assignment and query API works on spans", "Invariant", false,
       test_bundle_query_spans_by_bundle},
      {"C45_Phase48a_Bundle_InvalidReject", "Invalid bundle references are rejected", "Exact", true,
       test_add_span_invalid_bundle_fails},
      {"C46_Phase48a_Bundle_LegacyCompat", "Unbundled spans remain valid and legacy recalc behavior remains",
       "Invariant", false, test_unbundled_span_compatibility},
      {"C47_Phase48b_DrawPathBundle_HVDefault",
       "Bundle path generation creates default HV bundle spans", "Invariant", false,
       test_generate_bundle_from_path_basic_hv_default_lanes},
      {"C48_Phase48b_DrawPathBundle_DirectionModes", "Bundle path generation supports Forward/Reverse/Auto modes",
       "Invariant", false, test_generate_bundle_from_path_direction_modes_nonfailing},
      {"C49_Phase48b_DrawPathBundle_InvalidInputs", "Bundle path generation rejects invalid input and recovers",
       "Exact", true, test_generate_bundle_from_path_invalid_inputs_fail},
      {"C73_Phase49_Template_FixedCountReject",
       "Fixed bundle template rejects explicit count override", "Exact", true,
       test_backbone_bundle_template_fixed_count_rejects_override},
      {"C74_Phase49_Template_CommVariableCount",
       "Communication bundle template scales span generation by requested count", "Invariant", false,
       test_backbone_bundle_template_comm_variable_count_scales_spans},
      {"C75_Phase49_Template_CommRangeReject",
       "Communication bundle template rejects out-of-range count without clamping", "Exact", true,
       test_backbone_bundle_template_comm_count_out_of_range_fails},
      {"C77_Phase49_Template_MultiBundleRequest",
       "Multiple bundle templates in one BackboneSpec generate multiple bundles and combined spans", "Invariant",
       false, test_backbone_bundle_template_multi_request_generates_multiple_bundles},
      {"C50_Phase48c_PortMode_DefaultAuto", "New ports default to Auto mode", "Exact", false,
       test_port_position_mode_defaults_auto},
      {"C51_Phase48c_PortMode_ManualReset", "Manual set/reset toggles port mode and keeps dirty local", "Invariant",
       false, test_port_manual_set_and_reset_to_auto},
      {"C52_Phase48c_PortMode_ProtectFromRelayout", "Manual ports are not overwritten by auto pole relayout",
       "Invariant", false, test_manual_port_not_overwritten_by_auto_relayout},
      {"C71_Phase48k_MovePole_ReprojectAutoPreserveManual",
       "MovePole reprojects owned Auto ports and preserves Manual ports", "Invariant", false,
       test_move_pole_reprojects_auto_ports_and_preserves_manual_ports},
      {"C53_Phase48h_Guide_ManualBoundaryStable",
       "Guide extension keeps explicitly pinned poles fixed", "Invariant", false,
       test_generate_from_guide_keeps_manual_boundaries_stable},
      {"C54_Phase48h_Guide_LocalUpdate",
       "Guide regeneration adds only missing segments without duplicating unchanged spans", "Invariant", false,
       test_generate_from_guide_local_update_no_duplicate_unchanged_segments},
      {"C55_Phase48h_Backbone_Route", "Backbone edges are built from grouped spans and route search works",
       "Invariant", false, test_backbone_edges_and_route_search},
      {"C56_Phase48h_SharpCorner_ThresholdBoundary",
       "Sharp-corner orientation applies at <=75deg and disables above threshold", "Invariant", false,
       test_sharp_corner_threshold_boundary_orientation},
      {"C60_Phase48h_Guide_ReusedVertexReorient",
       "Reused guide vertex pole is reoriented by sharp-corner rule when not manually overridden", "Invariant", false,
       test_generate_from_guide_reused_vertex_reorients_to_corner_rule},
      {"C70_Phase48h_Guide_ReusedVertexPortReproject",
       "Reused guide vertex reprojections move template-owned ports when corner orientation changes", "Invariant", false,
       test_generate_from_guide_reused_pole_reprojects_owned_ports},
      {"C57_Phase48h_Guide_DuplicatePointsRobust",
       "Guide generation with duplicate points stays finite and keeps path Z", "Invariant", false,
       test_generate_from_guide_with_duplicate_points_is_robust},
      {"C58_Phase48h_Guide_ReverseSymmetry",
       "Guide reverse mode preserves generated pole position set", "Invariant", false,
       test_generate_from_guide_reverse_mode_position_symmetry},
      {"C59_Phase48h_Guide_AvoidConstraint",
       "Guide generation avoids forbidden radius around avoid_points", "Invariant", false,
       test_generate_from_guide_respects_avoid_constraints},
      {"C64_Phase48i_Guide_NoForcedManualVertices",
       "Guide generation does not force manual poles by default (including endpoints)", "Exact", false,
       test_generate_from_guide_vertices_are_not_forced_manual_by_default},
      {"C65_Phase48i_Guide_PinVerticesOption", "Guide pin_vertices option explicitly pins intermediate vertices",
       "Exact", false, test_generate_from_guide_pin_vertices_option},
      {"C66_Phase48i_PolePlacementMode_Roundtrip", "Pole placement mode can round-trip Auto<->Manual by API", "Exact",
       false, test_set_pole_placement_mode_auto_manual_roundtrip},
      {"C67_Phase48i_Regenerate_KeepManualPole",
       "Session-local regeneration preserves manually pinned pole position", "Invariant", false,
       test_regenerate_session_auto_parts_keeps_manual_pole},
      {"C68_Phase48i_Regenerate_KeepManualPort",
       "Session-local regeneration preserves manual port position edits", "Invariant", false,
       test_regenerate_session_auto_parts_keeps_manual_port},
      {"C72_Phase48i_Regenerate_KeepManualPortOnAutoPole",
       "Session-local regeneration preserves manual port edits even when owner pole remains Auto", "Invariant", false,
       test_regenerate_session_auto_parts_keeps_manual_port_on_auto_pole},
      {"C69_Phase48i_Regenerate_SessionIsolation",
       "Session-local regeneration does not mutate other generation sessions", "Invariant", false,
       test_regenerate_session_auto_parts_isolation_across_sessions},
  };

  bool all_passed = true;
  for (const TestCase& test : tests) {
    const bool passed = test.run();
    std::cout << (passed ? "[PASS] " : "[FAIL] ") << test.name << " [" << test.oracle << "]["
              << (test.abnormal ? "Abnormal" : "Normal") << "]"
              << " - " << test.intent << "\n";
    all_passed = all_passed && passed;
  }

  if (!all_passed) {
    std::cerr << "core tests failed\n";
    return 1;
  }

  std::cout << "core tests passed (" << tests.size() << " cases)\n";
  return 0;
}



