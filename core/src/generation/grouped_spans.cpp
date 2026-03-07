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
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace wire::core {

using namespace generation::detail;

EditResult<std::vector<ObjectId>>
CoreState::generate_grouped_spans_between_poles(const std::vector<ObjectId>& poles, ObjectId bundle_id,
                                                ConnectionCategory category, int conductor_count, double spacing_m,
                                                bool maintain_lane_order, bool allow_lane_mirror,
                                                std::vector<SegmentLaneAssignment>* out_lane_assignments,
                                                std::vector<BackboneEdgeOrientation>* out_edge_orientations,
                                                BundleKind bundle_template_id) {
  EditResult<std::vector<ObjectId>> result;
  if (poles.size() < 2) {
    result.error = "at least 2 poles are required";
    return result;
  }
  const int lane_count = std::max(1, conductor_count);
  const PortLayer target_port_layer = category_to_port_layer(category);
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
  std::unordered_map<ObjectId, std::size_t> pole_index_by_id{};
  pole_index_by_id.reserve(poles.size());
  for (std::size_t i = 0; i < poles.size(); ++i) {
    pole_index_by_id[poles[i]] = i;
  }
  auto canonical_side_axis_for_order = [&](ObjectId pole_id, ObjectId peer_id) -> Vec3d {
    if (const auto it = pole_side_axis_hints.find(pole_id);
        it != pole_side_axis_hints.end() && std::isfinite(it->second.x) && std::isfinite(it->second.y)) {
      return it->second;
    }
    const Pole* pole = edit_state_access().poles.find(pole_id);
    if (pole != nullptr) {
      if (const Pole* peer = edit_state_access().poles.find(peer_id); peer != nullptr) {
        Vec3d dir_xy = peer->world_transform.position - pole->world_transform.position;
        if (normalize_xy(&dir_xy) && std::isfinite(dir_xy.x) && std::isfinite(dir_xy.y)) {
          return Vec3d{-dir_xy.y, dir_xy.x, 0.0};
        }
      }
      Vec3d yaw_side_axis = side_axis_from_yaw_deg(pole->world_transform.rotation_euler_deg.z);
      if (std::isfinite(yaw_side_axis.x) && std::isfinite(yaw_side_axis.y)) {
        return yaw_side_axis;
      }
    }
    return Vec3d{0.0, 1.0, 0.0};
  };
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
    const bool use_hv_scaffold_geometry = (bundle_template_id == BundleKind::kHighVoltage);
    std::unordered_set<ObjectId> unique(ports_result.value.begin(), ports_result.value.end());
    int attempts = 0;
    int fallback_created = 0;
    while (static_cast<int>(ports_result.value.size()) < lane_count && attempts < lane_count * 16) {
      ++attempts;
      int slot_id = -1;
      SlotSelectionRequest request{};
      request.pole_id = pole_id;
      request.peer_pole_id = peer_id;
      request.category = category;
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
          const double fallback_spacing = std::max(0.1, spacing_m);
          const double lane_offset = fallback_spacing * static_cast<double>(fallback_ring);
          Vec3d world{};
          if (use_hv_scaffold_geometry) {
            const Vec3d local{0.0, lane_sign * lane_offset, p->height_m * 0.8};
            world = local_to_world_on_pole_local(p->world_transform, effective_pole_yaw_deg(*p), local);
          } else {
            const Vec3d lateral_axis = canonical_side_axis_for_order(pole_id, peer_id);
            const Vec3d lane_delta{lateral_axis.x * lane_sign * lane_offset, lateral_axis.y * lane_sign * lane_offset,
                                   0.0};
            world = p->world_transform.position + lane_delta;
            world.z = p->world_transform.position.z + p->height_m * 0.8;
          }
          EditResult<ObjectId> extra =
              AddPort(pole_id, world,
                      category_to_port_kind(category), category_to_port_layer(category));
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

    const bool use_scaffold_layout =
        maintain_lane_order && bundle_template_id == BundleKind::kHighVoltage && lane_count > 1;
    if (use_scaffold_layout && pole != nullptr) {
      const double spacing = std::max(0.1, spacing_m);
      const double center = (static_cast<double>(lane_count) - 1.0) * 0.5;
      std::vector<double> target_local_y(static_cast<std::size_t>(lane_count), 0.0);
      for (int lane = 0; lane < lane_count; ++lane) {
        target_local_y[static_cast<std::size_t>(lane)] = (static_cast<double>(lane) - center) * spacing;
      }

      const Vec3d stable_side_axis = canonical_side_axis_for_order(pole_id, peer_id);
      const auto it_index = pole_index_by_id.find(pole_id);
      const std::size_t node_index = (it_index == pole_index_by_id.end()) ? poles.size() : it_index->second;
      auto desired_world_for_target = [&](double target_y) {
        const Vec3d base = pole->world_transform.position;
        if (!use_hv_scaffold_geometry || node_index == poles.size() || node_index == 0 || node_index + 1 >= poles.size()) {
          const Vec3d local{0.0, target_y, pole->height_m * 0.8};
          return local_to_world_on_pole_local(pole->world_transform, effective_pole_yaw_deg(*pole), local);
        }

        const Pole* prev = edit_state_access().poles.find(poles[node_index - 1]);
        const Pole* next = edit_state_access().poles.find(poles[node_index + 1]);
        if (prev == nullptr || next == nullptr) {
          const Vec3d local{0.0, target_y, pole->height_m * 0.8};
          return local_to_world_on_pole_local(pole->world_transform, effective_pole_yaw_deg(*pole), local);
        }

        Vec3d dir_in = base - prev->world_transform.position;
        Vec3d dir_out = next->world_transform.position - base;
        if (!normalize_xy(&dir_in) || !normalize_xy(&dir_out)) {
          const Vec3d local{0.0, target_y, pole->height_m * 0.8};
          return local_to_world_on_pole_local(pole->world_transform, effective_pole_yaw_deg(*pole), local);
        }
        const Vec3d normal_in{-dir_in.y, dir_in.x, 0.0};
        const Vec3d normal_out{-dir_out.y, dir_out.x, 0.0};

        Vec3d joined_xy{};
        const Vec3d offset_in{base.x + normal_in.x * target_y, base.y + normal_in.y * target_y, base.z};
        const Vec3d offset_out{base.x + normal_out.x * target_y, base.y + normal_out.y * target_y, base.z};
        if (line_intersection_xy_local(offset_in, dir_in, offset_out, dir_out, &joined_xy)) {
          joined_xy.z = base.z + pole->height_m * 0.8;
          const double dx = joined_xy.x - base.x;
          const double dy = joined_xy.y - base.y;
          const double dist = std::sqrt(dx * dx + dy * dy);
          const double limit = std::max(spacing * 8.0, std::abs(target_y) * 8.0 + 0.2);
          if (dist <= limit || dist <= 1e-9) {
            return joined_xy;
          }
          const double scale = limit / dist;
          return Vec3d{base.x + dx * scale, base.y + dy * scale, base.z + pole->height_m * 0.8};
        }

        const Vec3d local{0.0, target_y, pole->height_m * 0.8};
        return local_to_world_on_pole_local(pole->world_transform, effective_pole_yaw_deg(*pole), local);
      };

      std::vector<ObjectId> ordered_ports(static_cast<std::size_t>(lane_count), kInvalidObjectId);
      std::unordered_set<ObjectId> used{};
      for (int lane = 0; lane < lane_count; ++lane) {
        const double target_y = target_local_y[static_cast<std::size_t>(lane)];
        const Vec3d desired_world = desired_world_for_target(target_y);
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
          if (use_hv_scaffold_geometry && candidate->source_slot_id >= 0) {
            continue;
          }
          const double dist = use_hv_scaffold_geometry
                                  ? std::sqrt(std::pow(candidate->world_position.x - desired_world.x, 2.0) +
                                              std::pow(candidate->world_position.y - desired_world.y, 2.0))
                                  : std::abs(dot_xy(candidate->world_position - pole->world_transform.position,
                                                     stable_side_axis) -
                                             target_y);
          if (dist < best_dist) {
            best_dist = dist;
            best_id = candidate_id;
          }
        }

        const double kTargetMatchTolerance =
            use_hv_scaffold_geometry ? std::max(0.05, spacing * 0.35) : std::max(0.02, spacing * 0.1);
        if (best_id == kInvalidObjectId || best_dist > kTargetMatchTolerance) {
          EditResult<ObjectId> extra =
              AddPort(pole_id, desired_world, category_to_port_kind(category), category_to_port_layer(category));
          if (!extra.ok) {
            ports_result.error = extra.error;
            return ports_result;
          }
          append_change_set(result.change_set, extra.change_set);
          best_id = extra.value;
        }
        if (use_hv_scaffold_geometry) {
          Port* selected = edit_state_access().ports.find(best_id);
          if (selected != nullptr && selected->owner_pole_id == pole_id && selected->source_slot_id < 0) {
            selected->world_position = desired_world;
            add_unique_id(result.change_set.updated_ids, selected->id);
          }
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
    if (static_cast<int>(ports_result.value.size()) > lane_count) {
      ports_result.value.resize(static_cast<std::size_t>(lane_count));
    }
    pole_lane_ports_cache[pole_id] = ports_result.value;
    ports_result.ok = true;
    return ports_result;
    ports_result.error = "owner pole not found for grouped generation";
    return ports_result;
  };

  bool first_seeded_from_previous = false;
  EditResult<std::vector<ObjectId>> first_ports =
      ensure_ports(poles.front(), poles[1], 0, true, &first_seeded_from_previous);
  if (!first_ports.ok) {
    result.error = first_ports.error;
    return result;
  }
  if (static_cast<int>(first_ports.value.size()) != lane_count) {
    result.error = "failed to seed first segment lanes";
    return result;
  }
  std::vector<std::vector<ObjectId>> base_ports_by_node(poles.size());
  base_ports_by_node.front() = first_ports.value;

  struct MirrorScore {
    int cross_y = 0;
    int cross_z = 0;
    int layer_jump = 0;
    double span_z_delta = 0.0;
  };
  auto secondary_score_less = [&](const MirrorScore& a, const MirrorScore& b) {
    const auto key_a =
        std::make_tuple(a.cross_z, a.layer_jump, static_cast<long long>(std::llround(a.span_z_delta * 1000.0)));
    const auto key_b =
        std::make_tuple(b.cross_z, b.layer_jump, static_cast<long long>(std::llround(b.span_z_delta * 1000.0)));
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
    const bool use_local_y_metric = (bundle_template_id == BundleKind::kHighVoltage);

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
            (pa == nullptr) ? port_a->world_position : to_local_on_pole(*pa, port_a->world_position);
        const Vec3d local_b =
            (pb == nullptr) ? port_b->world_position : to_local_on_pole(*pb, port_b->world_position);
        y_a[idx] = local_a.y;
        y_b[idx] = local_b.y;
      } else {
        const Vec3d da = port_a->world_position - ((pa == nullptr) ? Vec3d{} : pa->world_transform.position);
        const Vec3d db = port_b->world_position - ((pb == nullptr) ? Vec3d{} : pb->world_transform.position);
        y_a[idx] = dot_xy(da, axis_a);
        y_b[idx] = dot_xy(db, axis_b);
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

  const bool allow_mirror = allow_lane_mirror && lane_count > 1;
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
    base_ports_by_node[seg + 1] = prepared_ports_b[seg];
  }

  auto order_for_parity = [&](const std::vector<ObjectId>& base_ports, bool reversed) {
    std::vector<ObjectId> ordered = base_ports;
    if (reversed) {
      std::reverse(ordered.begin(), ordered.end());
    }
    return ordered;
  };
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

  struct OrientationPlanScore {
    MirrorScore mirror{};
    int adjacent_xy_intersections = 0;
    int orientation_flips = 0;
    int acute_orientation_flips = 0;
  };
  auto add_mirror_score = [](MirrorScore* dst, const MirrorScore& src) {
    if (dst == nullptr) {
      return;
    }
    dst->cross_y += src.cross_y;
    dst->cross_z += src.cross_z;
    dst->layer_jump += src.layer_jump;
    dst->span_z_delta += src.span_z_delta;
  };
  auto orientation_plan_less = [&](const OrientationPlanScore& a, const OrientationPlanScore& b) {
    if (bundle_template_id == BundleKind::kHighVoltage && a.adjacent_xy_intersections != b.adjacent_xy_intersections) {
      return a.adjacent_xy_intersections < b.adjacent_xy_intersections;
    }
    if (a.mirror.cross_y != b.mirror.cross_y) {
      return a.mirror.cross_y < b.mirror.cross_y;
    }
    if (a.acute_orientation_flips != b.acute_orientation_flips) {
      return a.acute_orientation_flips < b.acute_orientation_flips;
    }
    if (a.orientation_flips != b.orientation_flips) {
      return a.orientation_flips < b.orientation_flips;
    }
    return secondary_score_less(a.mirror, b.mirror);
  };

  std::vector<double> turn_angle_by_segment(segment_count, 180.0);
  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    turn_angle_by_segment[seg] = compute_turn_angle_deg(seg);
  }

  std::vector<int> node_parity(poles.size(), 0);
  if (segment_count == 1) {
    const std::vector<int> first_candidates = first_seeded_from_previous ? std::vector<int>{0} : std::vector<int>{0, 1};
    OrientationPlanScore best_score{};
    bool has_best = false;
    for (int parity_a : first_candidates) {
      const std::vector<ObjectId> ports_a = order_for_parity(base_ports_by_node[0], parity_a != 0);
      for (int parity_b : {0, 1}) {
        if (!allow_mirror && parity_b != 0) {
          continue;
        }
        const std::vector<ObjectId> ports_b = order_for_parity(base_ports_by_node[1], parity_b != 0);
        OrientationPlanScore candidate{};
        candidate.mirror = evaluate_increment(poles[0], poles[1], ports_a, ports_b);
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
    const std::vector<int> first_candidates = first_seeded_from_previous ? std::vector<int>{0} : std::vector<int>{0, 1};

    for (int parity_0 : first_candidates) {
      const std::vector<ObjectId> ports_0 = order_for_parity(base_ports_by_node[0], parity_0 != 0);
      for (int parity_1 : {0, 1}) {
        if (!allow_mirror && parity_1 != 0) {
          continue;
        }
        const std::vector<ObjectId> ports_1 = order_for_parity(base_ports_by_node[1], parity_1 != 0);
        DpCell& cell = dp[1][parity_0][parity_1];
        cell.reachable = true;
        cell.prev_prev_parity = -1;
        cell.score.mirror = evaluate_increment(poles[0], poles[1], ports_0, ports_1);
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
            if (!allow_mirror && parity_curr != 0) {
              continue;
            }
            const std::vector<ObjectId> ports_prev = order_for_parity(base_ports_by_node[step], parity_prev != 0);
            const std::vector<ObjectId> ports_curr = order_for_parity(base_ports_by_node[step + 1], parity_curr != 0);
            OrientationPlanScore candidate = cell.score;
            add_mirror_score(&candidate.mirror,
                             evaluate_increment(poles[step], poles[step + 1], ports_prev, ports_curr));
            if (bundle_template_id == BundleKind::kHighVoltage) {
              const std::vector<ObjectId> ports_prev_prev =
                  order_for_parity(base_ports_by_node[step - 1], parity_prev_prev != 0);
              candidate.adjacent_xy_intersections +=
                  count_adjacent_segment_xy_intersections(ports_prev_prev, ports_prev, ports_prev, ports_curr);
            }
            const int curr_orientation = parity_prev ^ parity_curr;
            if (curr_orientation != prev_orientation) {
              ++candidate.orientation_flips;
              const bool is_acute_turn =
                  (turn_angle_by_segment[step] + kAngleEps < layout_settings_.corner_threshold_deg);
              if (is_acute_turn) {
                ++candidate.acute_orientation_flips;
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

    node_parity[poles.size() - 2] = best_prev;
    node_parity[poles.size() - 1] = best_curr;
    for (std::size_t step = segment_count; step > 1; --step) {
      const DpCell& cell = dp[step][node_parity[step - 1]][node_parity[step]];
      node_parity[step - 2] = cell.prev_prev_parity;
    }
  }

  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    const ObjectId pole_a = poles[seg];
    const ObjectId pole_b = poles[seg + 1];

    SegmentLaneAssignment assignment{};
    assignment.segment_index = seg;
    assignment.pole_a_id = pole_a;
    assignment.pole_b_id = pole_b;
    assignment.bundle_id = bundle_id;
    assignment.port_ids_a = order_for_parity(base_ports_by_node[seg], node_parity[seg] != 0);
    assignment.port_ids_b = order_for_parity(base_ports_by_node[seg + 1], node_parity[seg + 1] != 0);

    const bool chosen_mirror = ((node_parity[seg] ^ node_parity[seg + 1]) != 0);
    const double turn_angle_deg = compute_turn_angle_deg(seg);
    const bool is_acute_turn = (seg > 0) && (turn_angle_deg + kAngleEps < layout_settings_.corner_threshold_deg);
    LaneFlipReason flip_reason = LaneFlipReason::kNone;
    bool flipped_from_previous = false;
    const bool previous_mirror = (seg > 0) ? ((node_parity[seg - 1] ^ node_parity[seg]) != 0) : false;
    if (seg > 0 && chosen_mirror != previous_mirror) {
      flipped_from_previous = true;
      if (is_acute_turn) {
        flip_reason = LaneFlipReason::kAcuteTurn;
      }
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
      assignment.slot_ids_a.push_back((pa == nullptr) ? -1 : pa->source_slot_id);
      assignment.slot_ids_b.push_back((pb == nullptr) ? -1 : pb->source_slot_id);

      Span* span = edit_state_access().spans.find(add.value);
      if (span != nullptr) {
        span->placement_context = ConnectionContext::kTrunkContinue;
        span->generated_by_rule = true;
        span->generation.generated = true;
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

} // namespace wire::core

