#include "wire/core/core_state.hpp"
#include "../pole_orientation_utils.hpp"
#include "detail_utils.hpp"
#include "support_policy.hpp"

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

namespace {} // namespace

EditResult<std::vector<ObjectId>>
CoreState::generate_grouped_spans_between_support_nodes(
    const std::vector<ObjectId>& node_ids, const std::unordered_map<ObjectId, SupportNode>& support_node_by_id,
    ObjectId bundle_id, ConnectionCategory category, int conductor_count, double spacing_m, bool maintain_lane_order,
    bool allow_lane_mirror, BackboneFlowKind flow_kind, double branch_down_offset_m,
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
  const bool uses_branch_support =
      (flow_kind == BackboneFlowKind::kBranch) && branch_down_offset_m > 1e-6;
  const double effective_branch_down_offset_m = uses_branch_support ? std::max(0.0, branch_down_offset_m) : 0.0;
  std::unordered_map<ObjectId, std::vector<ObjectId>> node_lane_ports_cache{};
  std::unordered_map<ObjectId, Vec3d> node_side_axis_hints{};
  auto support_position = [&](ObjectId node_id) -> Vec3d {
    if (const auto it = support_node_by_id.find(node_id); it != support_node_by_id.end()) {
      return it->second.position;
    }
    if (const Pole* pole = edit_state_access().poles.find(node_id); pole != nullptr) {
      return pole->world_transform.position;
    }
    return {};
  };
  auto support_pole = [&](ObjectId node_id) -> const Pole* {
    if (const auto it = support_node_by_id.find(node_id); it != support_node_by_id.end()) {
      if (it->second.pole_id != kInvalidObjectId) {
        if (const Pole* pole = edit_state_access().poles.find(it->second.pole_id); pole != nullptr) {
          return pole;
        }
      }
      if (it->second.support_kind != SupportKind::kPole) {
        return nullptr;
      }
    }
    return edit_state_access().poles.find(node_id);
  };
  auto support_kind = [&](ObjectId node_id) -> SupportKind {
    if (const auto it = support_node_by_id.find(node_id); it != support_node_by_id.end()) {
      return it->second.support_kind;
    }
    return (edit_state_access().poles.find(node_id) != nullptr) ? SupportKind::kPole : SupportKind::kMidair;
  };
  auto support_axis_for_pole = [&](const Pole& pole) -> Vec3d {
    if (const auto it = pole_orientation_debug_records_.find(pole.id); it != pole_orientation_debug_records_.end()) {
      Vec3d axis = it->second.adopted_support_axis;
      if (normalize_xy(&axis) && std::isfinite(axis.x) && std::isfinite(axis.y)) {
        return axis;
      }
    }
    Vec3d axis = side_axis_from_yaw_deg(effective_pole_layout_yaw_deg(pole));
    if (normalize_xy(&axis) && std::isfinite(axis.x) && std::isfinite(axis.y)) {
      return axis;
    }
    return Vec3d{0.0, 1.0, 0.0};
  };
  auto layout_yaw_for_pole = [&](const Pole& pole) { return effective_pole_layout_yaw_deg(pole); };
  auto highest_owned_port_z_for_pole = [&](const Pole& pole) -> std::optional<double> {
    const auto it_ports = relation_index_access().ports_by_pole.find(pole.id);
    if (it_ports == relation_index_access().ports_by_pole.end()) {
      return std::nullopt;
    }
    const PoleFrame frame = BuildPoleFrame(pole.world_transform, layout_yaw_for_pole(pole));
    const int target_layer = generation::detail::TemplateLayerForCategory(category);
    double best_z = -std::numeric_limits<double>::infinity();
    for (ObjectId port_id : it_ports->second) {
      const Port* port = edit_state_access().ports.find(port_id);
      if (port == nullptr || port->owner_pole_id != pole.id) {
        continue;
      }
      const bool layer_match = (port->layer == target_port_layer);
      const bool template_layer_match = (port->template_layer == target_layer);
      if (!layer_match && !template_layer_match) {
        continue;
      }
      const double local_z = WorldPointToLocal(frame, port->world_position).z;
      if (std::isfinite(local_z)) {
        best_z = std::max(best_z, local_z);
      }
    }
    return std::isfinite(best_z) ? std::optional<double>{best_z} : std::nullopt;
  };
  auto lane_row_base_z_for_pole = [&](const Pole& pole) {
    if (const auto port_z = highest_owned_port_z_for_pole(pole); port_z.has_value()) {
      return *port_z;
    }
    double best_z = -std::numeric_limits<double>::infinity();
    const int target_layer = generation::detail::TemplateLayerForCategory(category);
    if (const PoleTypeDefinition* pole_type = find_pole_type(pole.pole_type_id); pole_type != nullptr) {
      for (const PortSlotTemplate& slot : pole_type->port_slots) {
        if (!slot.enabled) {
          continue;
        }
        if (slot.layer == target_layer) {
          best_z = std::max(best_z, slot.local_position.z);
        }
      }
      if (!std::isfinite(best_z)) {
        for (const PortSlotTemplate& slot : pole_type->port_slots) {
          if (slot.enabled && slot.category == category) {
            best_z = std::max(best_z, slot.local_position.z);
          }
        }
      }
    }
    if (std::isfinite(best_z)) {
      return best_z;
    }
    return std::max(0.5, pole.height_m * 0.8);
  };
  auto span_context_for_segment = [&](ObjectId node_a, ObjectId node_b) -> ConnectionContext {
    if (category == ConnectionCategory::kDrop) {
      return ConnectionContext::kDropAdd;
    }
    if (uses_branch_support) {
      return ConnectionContext::kBranchAdd;
    }
    const Pole* pole_a = support_pole(node_a);
    const Pole* pole_b = support_pole(node_b);
    const bool corner_pass = (pole_a != nullptr && pole_a->context.kind == PoleContextKind::kCorner) ||
                             (pole_b != nullptr && pole_b->context.kind == PoleContextKind::kCorner);
    return corner_pass ? ConnectionContext::kCornerPass : ConnectionContext::kTrunkContinue;
  };
  auto resolve_span_endpoint_node = [&](const Span& span, const Port* port, bool is_a) -> ObjectId {
    const ObjectId explicit_node_id = is_a ? span.endpoint_node_a_id : span.endpoint_node_b_id;
    if (explicit_node_id != kInvalidObjectId) {
      return explicit_node_id;
    }
    return (port == nullptr) ? kInvalidObjectId : port->owner_pole_id;
  };
  auto node_requires_branch_support = [&](ObjectId node_id, ObjectId peer_id) -> bool {
    if (!uses_branch_support) {
      return false;
    }
    const auto it_ports = relation_index_access().ports_by_pole.find(node_id);
    if (it_ports == relation_index_access().ports_by_pole.end()) {
      return false;
    }
    for (ObjectId port_id : it_ports->second) {
      const Port* port = edit_state_access().ports.find(port_id);
      if (port == nullptr || port->owner_pole_id != node_id || port->layer != target_port_layer) {
        continue;
      }
      const auto it_spans = connection_index_access().spans_by_port.find(port_id);
      if (it_spans == connection_index_access().spans_by_port.end()) {
        continue;
      }
      for (ObjectId span_id : it_spans->second) {
        const Span* span = edit_state_access().spans.find(span_id);
        if (span == nullptr || span->bundle_id == kInvalidObjectId) {
          continue;
        }
        const Bundle* bundle = edit_state_access().bundles.find(span->bundle_id);
        if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id) {
          continue;
        }
        const Port* port_a = edit_state_access().ports.find(span->port_a_id);
        const Port* port_b = edit_state_access().ports.find(span->port_b_id);
        if (port_a == nullptr || port_b == nullptr) {
          continue;
        }
        const ObjectId other_node_id =
            (span->port_a_id == port_id) ? resolve_span_endpoint_node(*span, port_b, false)
                                         : resolve_span_endpoint_node(*span, port_a, true);
        if (other_node_id != kInvalidObjectId && other_node_id != node_id && other_node_id != peer_id) {
          return true;
        }
      }
    }
    return false;
  };
  std::vector<Vec3d> side_axis_by_index(node_ids.size(), Vec3d{0.0, 1.0, 0.0});
  for (std::size_t i = 0; i < node_ids.size(); ++i) {
    if (const Pole* pole = support_pole(node_ids[i]); pole != nullptr) {
      side_axis_by_index[i] = support_axis_for_pole(*pole);
      continue;
    }
    const Vec3d center = support_position(node_ids[i]);
    Vec3d tangent{1.0, 0.0, 0.0};
    if (i == 0 && i + 1 < node_ids.size()) {
      tangent = support_position(node_ids[i + 1]) - center;
    } else if (i + 1 == node_ids.size() && i > 0) {
      tangent = center - support_position(node_ids[i - 1]);
    } else if (i > 0 && i + 1 < node_ids.size()) {
      const Vec3d in_dir = center - support_position(node_ids[i - 1]);
      const Vec3d out_dir = support_position(node_ids[i + 1]) - center;
      tangent = in_dir + out_dir;
      if (!normalize_xy(&tangent)) {
        tangent = out_dir;
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
  for (std::size_t i = 0; i < node_ids.size(); ++i) {
    node_side_axis_hints[node_ids[i]] = side_axis_by_index[i];
  }
  std::unordered_map<ObjectId, std::size_t> node_index_by_id{};
  node_index_by_id.reserve(node_ids.size());
  for (std::size_t i = 0; i < node_ids.size(); ++i) {
    node_index_by_id[node_ids[i]] = i;
  }
  auto canonical_side_axis_for_order = [&](ObjectId node_id, ObjectId peer_id) -> Vec3d {
    if (const auto it = node_side_axis_hints.find(node_id);
        it != node_side_axis_hints.end() && std::isfinite(it->second.x) && std::isfinite(it->second.y)) {
      return it->second;
    }
    const Pole* pole = support_pole(node_id);
    if (pole != nullptr) {
      Vec3d dir_xy = support_position(peer_id) - pole->world_transform.position;
      if (normalize_xy(&dir_xy) && std::isfinite(dir_xy.x) && std::isfinite(dir_xy.y)) {
        return ComputeLateralAxis(dir_xy);
      }
      Vec3d yaw_side_axis = support_axis_for_pole(*pole);
      if (std::isfinite(yaw_side_axis.x) && std::isfinite(yaw_side_axis.y)) {
        return yaw_side_axis;
      }
    }
    return Vec3d{0.0, 1.0, 0.0};
  };
  auto ensure_ports = [&](ObjectId node_id, ObjectId peer_id, int segment_index, bool prefer_existing_neighbor_order,
                          bool* out_seeded_from_previous = nullptr) -> EditResult<std::vector<ObjectId>> {
    EditResult<std::vector<ObjectId>> ports_result;
    if (out_seeded_from_previous != nullptr) {
      *out_seeded_from_previous = false;
    }
    if (const auto it_cached = node_lane_ports_cache.find(node_id); it_cached != node_lane_ports_cache.end()) {
      if (static_cast<int>(it_cached->second.size()) == lane_count) {
        ports_result.value = it_cached->second;
        ports_result.ok = true;
        return ports_result;
      }
    }
    const SupportKind kind = support_kind(node_id);
    const Pole* pole = support_pole(node_id);
    if (kind != SupportKind::kPole || pole == nullptr) {
      const Vec3d base_position = support_position(node_id);
      const Vec3d side_axis = canonical_side_axis_for_order(node_id, peer_id);
      auto nonpole_order_key = [&](const Port* port) {
        if (port == nullptr) {
          return std::tuple<double, double, ObjectId>(0.0, 0.0, kInvalidObjectId);
        }
        return std::tuple<double, double, ObjectId>(dot_xy(port->world_position - base_position, side_axis),
                                                    port->world_position.z, port->id);
      };
      auto sort_nonpole_ports = [&](std::vector<ObjectId>* port_ids) {
        if (port_ids == nullptr) {
          return;
        }
        std::sort(port_ids->begin(), port_ids->end(), [&](ObjectId a, ObjectId b) {
          const Port* pa = edit_state_access().ports.find(a);
          const Port* pb = edit_state_access().ports.find(b);
          return nonpole_order_key(pa) < nonpole_order_key(pb);
        });
      };
      std::unordered_set<ObjectId> unique_nonpole_ports{};
      for (const Span& span : edit_state_access().spans.items()) {
        if (span.bundle_id == kInvalidObjectId) {
          continue;
        }
        const Bundle* bundle = edit_state_access().bundles.find(span.bundle_id);
        if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id) {
          continue;
        }
        const Port* pa = edit_state_access().ports.find(span.port_a_id);
        const Port* pb = edit_state_access().ports.find(span.port_b_id);
        if (pa == nullptr || pb == nullptr) {
          continue;
        }
        if (resolve_span_endpoint_node(span, pa, true) == node_id && pa->layer == target_port_layer &&
            unique_nonpole_ports.insert(pa->id).second) {
          ports_result.value.push_back(pa->id);
        }
        if (resolve_span_endpoint_node(span, pb, false) == node_id && pb->layer == target_port_layer &&
            unique_nonpole_ports.insert(pb->id).second) {
          ports_result.value.push_back(pb->id);
        }
      }
      sort_nonpole_ports(&ports_result.value);
      if (static_cast<int>(ports_result.value.size()) > lane_count) {
        ports_result.value.resize(static_cast<std::size_t>(lane_count));
      }

      const auto it_support = support_node_by_id.find(node_id);
      if (static_cast<int>(ports_result.value.size()) < lane_count && it_support != support_node_by_id.end() &&
          it_support->second.has_source_edge) {
        struct SourcePortSample {
          Vec3d world{};
          double side = 0.0;
        };
        std::vector<SourcePortSample> samples{};
        const SupportNode& support = it_support->second;
        for (const Span& span : edit_state_access().spans.items()) {
          if (span.bundle_id == kInvalidObjectId) {
            continue;
          }
          const Bundle* bundle = edit_state_access().bundles.find(span.bundle_id);
          if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id) {
            continue;
          }
          const Port* pa = edit_state_access().ports.find(span.port_a_id);
          const Port* pb = edit_state_access().ports.find(span.port_b_id);
          if (pa == nullptr || pb == nullptr || pa->layer != target_port_layer || pb->layer != target_port_layer) {
            continue;
          }
          const ObjectId endpoint_a = resolve_span_endpoint_node(span, pa, true);
          const ObjectId endpoint_b = resolve_span_endpoint_node(span, pb, false);
          double t = support.source_edge_t;
          if (endpoint_a == support.source_edge_node_a_id && endpoint_b == support.source_edge_node_b_id) {
            // keep t
          } else if (endpoint_a == support.source_edge_node_b_id && endpoint_b == support.source_edge_node_a_id) {
            t = 1.0 - t;
          } else {
            continue;
          }
          const Vec3d world{
              pa->world_position.x + (pb->world_position.x - pa->world_position.x) * t,
              pa->world_position.y + (pb->world_position.y - pa->world_position.y) * t,
              pa->world_position.z + (pb->world_position.z - pa->world_position.z) * t,
          };
          samples.push_back({world, dot_xy(world - base_position, side_axis)});
        }
        std::sort(samples.begin(), samples.end(), [](const SourcePortSample& a, const SourcePortSample& b) {
          if (std::abs(a.side - b.side) > 1e-9) {
            return a.side < b.side;
          }
          if (std::abs(a.world.z - b.world.z) > 1e-9) {
            return a.world.z < b.world.z;
          }
          if (std::abs(a.world.y - b.world.y) > 1e-9) {
            return a.world.y < b.world.y;
          }
          return a.world.x < b.world.x;
        });
        for (const SourcePortSample& sample : samples) {
          if (static_cast<int>(ports_result.value.size()) >= lane_count) {
            break;
          }
          EditResult<ObjectId> add_port =
              AddPort(kInvalidObjectId, sample.world, category_to_port_kind(category), target_port_layer);
          if (!add_port.ok) {
            ports_result.error = add_port.error;
            return ports_result;
          }
          append_change_set(result.change_set, add_port.change_set);
          Port* created_port = edit_state_access().ports.find(add_port.value);
          if (created_port != nullptr) {
            created_port->generated_by_rule = true;
            created_port->placement_context = ConnectionContext::kBranchAdd;
            apply_port_position_mode(*created_port, PortPositionMode::kAuto, PortPlacementSourceKind::kAerialBranch);
            add_unique_id(result.change_set.updated_ids, created_port->id);
          }
          ports_result.value.push_back(add_port.value);
        }
      }

      ports_result.value.reserve(static_cast<std::size_t>(lane_count));
      while (static_cast<int>(ports_result.value.size()) < lane_count) {
        EditResult<ObjectId> add_port =
            AddPort(kInvalidObjectId, base_position, category_to_port_kind(category), target_port_layer);
        if (!add_port.ok) {
          ports_result.error = add_port.error;
          return ports_result;
        }
        append_change_set(result.change_set, add_port.change_set);
        Port* created_port = edit_state_access().ports.find(add_port.value);
        if (created_port != nullptr) {
          created_port->generated_by_rule = true;
          created_port->placement_context = ConnectionContext::kBranchAdd;
          apply_port_position_mode(*created_port, PortPositionMode::kAuto, PortPlacementSourceKind::kAerialBranch);
          add_unique_id(result.change_set.updated_ids, created_port->id);
        }
        ports_result.value.push_back(add_port.value);
      }
      sort_nonpole_ports(&ports_result.value);
      node_lane_ports_cache[node_id] = ports_result.value;
      ports_result.ok = true;
      return ports_result;
    }
    const bool use_branch_support_here = node_requires_branch_support(node_id, peer_id);
    if (use_branch_support_here) {
      const Vec3d peer_delta = support_position(peer_id) - pole->world_transform.position;
      Vec3d branch_forward_axis = peer_delta;
      if (!normalize_xy(&branch_forward_axis)) {
        branch_forward_axis = {1.0, 0.0, 0.0};
      }
      const Vec3d branch_row_axis = ComputeLateralAxis(branch_forward_axis);
      const Vec3d stable_side_axis = canonical_side_axis_for_order(node_id, peer_id);
      double side_sign = (dot_xy(stable_side_axis, branch_row_axis) >= 0.0) ? 1.0 : -1.0;
      if (std::abs(dot_xy(stable_side_axis, branch_row_axis)) <= 1e-9) {
        side_sign = 1.0;
      }
      const SlotSide branch_side = (side_sign >= 0.0) ? SlotSide::kRight : SlotSide::kLeft;
      const double branch_support_yaw_deg =
          std::atan2(branch_forward_axis.y, branch_forward_axis.x) * (180.0 / kPi);

      const bool use_scaffold_layout = use_lane_row_geometry;
      double main_support_base_z_m = lane_row_base_z_for_pole(*pole);
      double branch_base_z_m = main_support_base_z_m;
      if (use_scaffold_layout) {
        main_support_base_z_m = std::min(main_support_base_z_m, pole->height_m * 0.8);
        branch_base_z_m = main_support_base_z_m;
      }
      branch_base_z_m = std::max(0.5, main_support_base_z_m - effective_branch_down_offset_m);

      const double lane_spacing = std::max(0.1, spacing_m);
      const double center = (static_cast<double>(lane_count) - 1.0) * 0.5;
      const double half_span = center * lane_spacing;
        const double min_outboard =
            pole_radius_at_height_m(*pole, branch_base_z_m) + cache_state_access().geometry_settings.pole_clearance_m +
            half_span + 0.25;
        const double branch_support_offset_m = std::max(0.65, min_outboard);
        ports_result.value.reserve(static_cast<std::size_t>(lane_count));
        for (int lane = 0; lane < lane_count; ++lane) {
          const double lane_offset = (static_cast<double>(lane) - center) * lane_spacing;
          const double lane_z_m = branch_base_z_m;
          Vec3d local{0.0, side_sign * branch_support_offset_m + lane_offset, lane_z_m};
        local = apply_pole_clearance_to_local(*pole, local, branch_side);
        const Vec3d world =
            local_to_world_on_pole_local(pole->world_transform, branch_support_yaw_deg, local);
        EditResult<ObjectId> add_port =
            AddPort(node_id, world, category_to_port_kind(category), category_to_port_layer(category));
        if (!add_port.ok) {
          ports_result.error = add_port.error;
          return ports_result;
        }
        append_change_set(result.change_set, add_port.change_set);
        Port* created_port = edit_state_access().ports.find(add_port.value);
        if (created_port != nullptr) {
          created_port->category = category;
          created_port->source_slot_id = -1;
          created_port->template_layer = generation::detail::TemplateLayerForCategory(category);
          created_port->template_side = branch_side;
          created_port->template_role = SlotRole::kBranchPreferred;
          created_port->generated_from_template = false;
          created_port->generated_by_rule = true;
          created_port->placement_context = ConnectionContext::kBranchAdd;
          apply_port_position_mode(*created_port, PortPositionMode::kAuto, PortPlacementSourceKind::kBranchSupport);
          add_unique_id(result.change_set.updated_ids, created_port->id);
        }
        ports_result.value.push_back(add_port.value);
      }
      std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
        const Port* pa = edit_state_access().ports.find(a);
        const Port* pb = edit_state_access().ports.find(b);
        const double ya = (pa == nullptr)
                              ? 0.0
                              : WorldPointToLocal(BuildPoleFrame(pole->world_transform, branch_support_yaw_deg),
                                                  pa->world_position)
                                    .y;
        const double yb = (pb == nullptr)
                              ? 0.0
                              : WorldPointToLocal(BuildPoleFrame(pole->world_transform, branch_support_yaw_deg),
                                                  pb->world_position)
                                    .y;
        if (std::abs(ya - yb) > 1e-9) {
          return ya < yb;
        }
        return a < b;
      });
      node_lane_ports_cache[node_id] = ports_result.value;
      ports_result.ok = true;
      return ports_result;
    }
    auto port_connection_count = [&](ObjectId port_id) -> std::size_t {
      const auto it = connection_index_access().spans_by_port.find(port_id);
      return (it == connection_index_access().spans_by_port.end()) ? 0 : it->second.size();
    };

    if (use_lane_row_geometry && pole != nullptr) {
      std::unordered_set<ObjectId> unique{};
      std::vector<ObjectId> reusable_generated_ports{};

      if (const auto it_ports = relation_index_access().ports_by_pole.find(node_id);
          it_ports != relation_index_access().ports_by_pole.end()) {
        for (ObjectId port_id : it_ports->second) {
          const Port* port = edit_state_access().ports.find(port_id);
          if (port == nullptr || port->layer != target_port_layer || port->category != category) {
            continue;
          }
          if (port->generated_by_rule && port_connection_count(port_id) == 0) {
            reusable_generated_ports.push_back(port_id);
          }
        }
      }

      const double target_z_m = lane_row_base_z_for_pole(*pole);
      const double spacing = std::max(0.1, spacing_m);
      const double center = (static_cast<double>(lane_count) - 1.0) * 0.5;
      for (int lane = 0; lane < lane_count; ++lane) {
        const double target_y = (static_cast<double>(lane) - center) * spacing;
        const Vec3d local{0.0, target_y, target_z_m};
        const Vec3d world = local_to_world_on_pole_local(pole->world_transform, layout_yaw_for_pole(*pole), local);

        ObjectId port_id = kInvalidObjectId;
        while (!reusable_generated_ports.empty() && port_id == kInvalidObjectId) {
          const ObjectId candidate_id = reusable_generated_ports.back();
          reusable_generated_ports.pop_back();
          if (unique.find(candidate_id) == unique.end()) {
            port_id = candidate_id;
          }
        }

        if (port_id != kInvalidObjectId) {
          Port* reused_port = edit_state_access().ports.find(port_id);
          if (reused_port == nullptr) {
            continue;
          }
          reused_port->world_position = world;
          reused_port->category = category;
          reused_port->generated_by_rule = true;
          reused_port->placement_context = ConnectionContext::kTrunkContinue;
          reused_port->template_layer = generation::detail::TemplateLayerForCategory(category);
          reused_port->template_side = (target_y < -1e-9) ? SlotSide::kLeft
                                                           : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
          apply_port_position_mode(*reused_port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
          add_unique_id(result.change_set.updated_ids, reused_port->id);
          unique.insert(port_id);
          ports_result.value.push_back(port_id);
          continue;
        }

        EditResult<ObjectId> add_port =
            AddPort(node_id, world, category_to_port_kind(category), category_to_port_layer(category));
        if (!add_port.ok) {
          ports_result.error = add_port.error;
          return ports_result;
        }
        append_change_set(result.change_set, add_port.change_set);
        Port* created_port = edit_state_access().ports.find(add_port.value);
        if (created_port != nullptr) {
          created_port->category = category;
          created_port->generated_by_rule = true;
          created_port->placement_context = ConnectionContext::kTrunkContinue;
          created_port->template_layer = generation::detail::TemplateLayerForCategory(category);
          created_port->template_side = (target_y < -1e-9) ? SlotSide::kLeft
                                                           : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
          apply_port_position_mode(*created_port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
          add_unique_id(result.change_set.updated_ids, created_port->id);
        }
        unique.insert(add_port.value);
        ports_result.value.push_back(add_port.value);
      }
    } else {
      for (const Port& port : edit_state_access().ports.items()) {
        if (port.owner_pole_id == node_id && port.layer == target_port_layer) {
          ports_result.value.push_back(port.id);
        }
      }
    }

    auto port_links_to_neighbor = [&](ObjectId port_id, ObjectId neighbor_node_id) -> int {
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
        if (other_port->owner_pole_id == neighbor_node_id) {
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
          const ObjectId other_node_id = other_port->owner_pole_id;
          if (other_node_id == kInvalidObjectId || other_node_id == node_id || other_node_id == peer_id) {
            continue;
          }
          neighbor_counts[other_node_id] += 1;
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
    if (static_cast<int>(ports_result.value.size()) < lane_count) {
      ports_result.error = "insufficient ports for grouped generation";
      return ports_result;
    }
    if (prefer_existing_neighbor_order && continuity_neighbor_id != kInvalidObjectId) {
      // Preserve boundary lane order on extension by seeding from prior assignment at continuity neighbor.
      std::vector<ObjectId> seeded_order{};
      for (const SegmentLaneAssignment& assignment : last_lane_assignments_access()) {
        const Bundle* assignment_bundle = edit_state_access().bundles.find(assignment.bundle_id);
        if (assignment_bundle == nullptr || assignment_bundle->bundle_template_id != bundle_template_id) {
          continue;
        }
        const std::vector<ObjectId>* candidate_order = nullptr;
        if (assignment.pole_a_id == continuity_neighbor_id && assignment.pole_b_id == node_id) {
          candidate_order = &assignment.port_ids_b;
        } else if (assignment.pole_a_id == node_id && assignment.pole_b_id == continuity_neighbor_id) {
          candidate_order = &assignment.port_ids_a;
        }
        if (candidate_order == nullptr || static_cast<int>(candidate_order->size()) < lane_count) {
          continue;
        }
        bool valid = true;
        for (int lane = 0; lane < lane_count; ++lane) {
          const ObjectId port_id = (*candidate_order)[static_cast<std::size_t>(lane)];
          const Port* port = edit_state_access().ports.find(port_id);
          if (port == nullptr || port->owner_pole_id != node_id || port->layer != target_port_layer) {
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
        node_lane_ports_cache[node_id] = ports_result.value;
        if (out_seeded_from_previous != nullptr) {
          *out_seeded_from_previous = true;
        }
        ports_result.ok = true;
        return ports_result;
      }
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
      return WorldPointToLocal(BuildPoleFrame(pole->world_transform, layout_yaw_for_pole(*pole)), p->world_position).y;
    };
    auto order_key = [&](const Port* p) -> std::tuple<double, int, int, int, int, ObjectId> {
      if (p == nullptr) {
        return {0.0, 1, 999, 999999, 999999, kInvalidObjectId};
      }
      return {local_y_of(p), 0, p->template_layer, side_rank(p->template_side),
              static_cast<int>(std::llround(HeightAlongWorldUp(p->world_position) * 1000.0)), p->id};
    };

    const bool use_scaffold_layout = use_lane_row_geometry;
    if (use_scaffold_layout && pole != nullptr) {
      const double spacing = std::max(0.1, spacing_m);
      const double center = (static_cast<double>(lane_count) - 1.0) * 0.5;
      std::vector<double> target_local_y(static_cast<std::size_t>(lane_count), 0.0);
      for (int lane = 0; lane < lane_count; ++lane) {
        target_local_y[static_cast<std::size_t>(lane)] = (static_cast<double>(lane) - center) * spacing;
      }

      const Vec3d stable_side_axis = canonical_side_axis_for_order(node_id, peer_id);
      const auto it_index = node_index_by_id.find(node_id);
      const std::size_t node_index = (it_index == node_index_by_id.end()) ? node_ids.size() : it_index->second;
      const bool is_terminal_node = (node_index == 0 || node_index + 1 >= node_ids.size());
      if (is_terminal_node) {
        auto port_connection_count = [&](ObjectId port_id) -> std::size_t {
          const auto it = connection_index_access().spans_by_port.find(port_id);
          return (it == connection_index_access().spans_by_port.end()) ? 0 : it->second.size();
        };
        auto terminal_order_key = [&](const Port* p) -> std::tuple<int, int, double, ObjectId> {
          if (p == nullptr) {
            return {999, 999, 0.0, kInvalidObjectId};
          }
          return {p->template_layer, side_rank(p->template_side), local_y_of(p), p->id};
        };
        std::sort(ports_result.value.begin(), ports_result.value.end(), [&](ObjectId a, ObjectId b) {
          const Port* pa = edit_state_access().ports.find(a);
          const Port* pb = edit_state_access().ports.find(b);
          return terminal_order_key(pa) < terminal_order_key(pb);
        });

        std::vector<ObjectId> normalized_ports{};
        normalized_ports.reserve(static_cast<std::size_t>(lane_count));
        std::vector<ObjectId> reusable_fallback_ports{};
        reusable_fallback_ports.reserve(ports_result.value.size());
        for (ObjectId port_id : ports_result.value) {
          const Port* port = edit_state_access().ports.find(port_id);
          if (port == nullptr) {
            continue;
          }
          if (port_connection_count(port_id) == 0) {
            reusable_fallback_ports.push_back(port_id);
          }
        }

        const double target_z_m = lane_row_base_z_for_pole(*pole);

        auto realize_terminal_world = [&](double target_y) {
          const Vec3d local{0.0, target_y, target_z_m};
          return local_to_world_on_pole_local(pole->world_transform, layout_yaw_for_pole(*pole), local);
        };

        for (int lane = static_cast<int>(normalized_ports.size()); lane < lane_count; ++lane) {
          const double target_y = target_local_y[static_cast<std::size_t>(lane)];
          const Vec3d world = realize_terminal_world(target_y);
          if (!reusable_fallback_ports.empty()) {
            const ObjectId port_id = reusable_fallback_ports.front();
            reusable_fallback_ports.erase(reusable_fallback_ports.begin());
            Port* port = edit_state_access().ports.find(port_id);
            if (port != nullptr) {
              port->world_position = world;
              port->category = category;
              port->generated_by_rule = true;
              port->template_layer = generation::detail::TemplateLayerForCategory(category);
              port->template_side =
                  (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
              apply_port_position_mode(*port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
              add_unique_id(result.change_set.updated_ids, port->id);
              normalized_ports.push_back(port_id);
              continue;
            }
          }

          EditResult<ObjectId> add_port =
              AddPort(node_id, world, category_to_port_kind(category), category_to_port_layer(category));
          if (!add_port.ok) {
            ports_result.error = add_port.error;
            return ports_result;
          }
          append_change_set(result.change_set, add_port.change_set);
          Port* created_port = edit_state_access().ports.find(add_port.value);
          if (created_port != nullptr) {
            created_port->category = category;
            created_port->generated_by_rule = true;
            created_port->placement_context = ConnectionContext::kTrunkContinue;
            created_port->template_layer = generation::detail::TemplateLayerForCategory(category);
            created_port->template_side =
                (target_y < -1e-9) ? SlotSide::kLeft : ((target_y > 1e-9) ? SlotSide::kRight : SlotSide::kCenter);
            apply_port_position_mode(*created_port, PortPositionMode::kAuto, PortPlacementSourceKind::kGenerated);
            add_unique_id(result.change_set.updated_ids, created_port->id);
          }
          normalized_ports.push_back(add_port.value);
        }

        std::sort(normalized_ports.begin(), normalized_ports.end(), [&](ObjectId a, ObjectId b) {
          const Port* pa = edit_state_access().ports.find(a);
          const Port* pb = edit_state_access().ports.find(b);
          return local_y_of(pa) < local_y_of(pb);
        });
        if (static_cast<int>(normalized_ports.size()) > lane_count) {
          normalized_ports.resize(static_cast<std::size_t>(lane_count));
        }

        ports_result.value = std::move(normalized_ports);
        node_lane_ports_cache[node_id] = ports_result.value;
        ports_result.ok = true;
        return ports_result;
      }

      const bool use_scaffold_geometry = use_lane_row_geometry;
      auto desired_world_for_target = [&](double target_y) {
        const Vec3d base = pole->world_transform.position;
        const double base_z_m = lane_row_base_z_for_pole(*pole);
        if (!use_scaffold_geometry || node_index == node_ids.size()) {
          const Vec3d local{0.0, target_y, base_z_m};
          return local_to_world_on_pole_local(pole->world_transform, layout_yaw_for_pole(*pole), local);
        }

        const Vec3d prev = support_position(node_ids[node_index - 1]);
        const Vec3d next = support_position(node_ids[node_index + 1]);
        if ((prev - base).x * (prev - base).x + (prev - base).y * (prev - base).y <= 1e-12 ||
            (next - base).x * (next - base).x + (next - base).y * (next - base).y <= 1e-12) {
          const Vec3d local{0.0, target_y, base_z_m};
          return local_to_world_on_pole_local(pole->world_transform, layout_yaw_for_pole(*pole), local);
        }

        Vec3d dir_in = base - prev;
        Vec3d dir_out = next - base;
        if (!normalize_xy(&dir_in) || !normalize_xy(&dir_out)) {
          const Vec3d local{0.0, target_y, base_z_m};
          return local_to_world_on_pole_local(pole->world_transform, layout_yaw_for_pole(*pole), local);
        }
        Vec3d normal_in = ComputeLateralAxis(dir_in);
        Vec3d normal_out = ComputeLateralAxis(dir_out);
        if (!normalize_xy(&normal_in)) {
          normal_in = stable_side_axis;
        }
        if (!normalize_xy(&normal_out)) {
          normal_out = stable_side_axis;
        }
        if (dot_xy(normal_in, stable_side_axis) < 0.0) {
          normal_in.x = -normal_in.x;
          normal_in.y = -normal_in.y;
        }
        if (dot_xy(normal_out, stable_side_axis) < 0.0) {
          normal_out.x = -normal_out.x;
          normal_out.y = -normal_out.y;
        }

        Vec3d joined_xy{};
        const Vec3d offset_in{base.x + normal_in.x * target_y, base.y + normal_in.y * target_y,
                              HeightAlongWorldUp(base)};
        const Vec3d offset_out{base.x + normal_out.x * target_y, base.y + normal_out.y * target_y,
                               HeightAlongWorldUp(base)};
        if (line_intersection_xy_local(offset_in, dir_in, offset_out, dir_out, &joined_xy)) {
          SetHeightAlongWorldUp(&joined_xy, HeightAlongWorldUp(base) + base_z_m);
          const double dx = joined_xy.x - base.x;
          const double dy = joined_xy.y - base.y;
          const double dist = std::sqrt(dx * dx + dy * dy);
          const double limit = std::max(spacing * 8.0, std::abs(target_y) * 8.0 + 0.2);
          if (dist <= limit || dist <= 1e-9) {
            return joined_xy;
          }
          const double scale = limit / dist;
          return Vec3d{base.x + dx * scale, base.y + dy * scale, HeightAlongWorldUp(base) + base_z_m};
        }

        const Vec3d local{0.0, target_y, base_z_m};
        return local_to_world_on_pole_local(pole->world_transform, layout_yaw_for_pole(*pole), local);
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
          if (candidate == nullptr || candidate->owner_pole_id != node_id || candidate->layer != target_port_layer) {
            continue;
          }
          const double dist = use_scaffold_geometry
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
            use_scaffold_geometry ? std::max(0.05, spacing * 0.35) : std::max(0.02, spacing * 0.1);
        if (best_id == kInvalidObjectId || best_dist > kTargetMatchTolerance) {
          EditResult<ObjectId> extra =
              AddPort(node_id, desired_world, category_to_port_kind(category), category_to_port_layer(category));
          if (!extra.ok) {
            ports_result.error = extra.error;
            return ports_result;
          }
          append_change_set(result.change_set, extra.change_set);
          best_id = extra.value;
        }
        if (use_scaffold_geometry) {
          Port* selected = edit_state_access().ports.find(best_id);
          if (selected != nullptr && selected->owner_pole_id == node_id) {
            selected->world_position = desired_world;
            add_unique_id(result.change_set.updated_ids, selected->id);
          }
        }

        used.insert(best_id);
        ordered_ports[static_cast<std::size_t>(lane)] = best_id;
      }

      ports_result.value = std::move(ordered_ports);
      node_lane_ports_cache[node_id] = ports_result.value;
      ports_result.ok = true;
      return ports_result;
    }
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
    node_lane_ports_cache[node_id] = ports_result.value;
    ports_result.ok = true;
    return ports_result;
  };

  bool first_seeded_from_previous = false;
  EditResult<std::vector<ObjectId>> first_ports =
      ensure_ports(node_ids.front(), node_ids[1], 0, true, &first_seeded_from_previous);
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
  auto evaluate_increment = [&](ObjectId node_a, ObjectId node_b, const std::vector<ObjectId>& lanes_a,
                                const std::vector<ObjectId>& lanes_b) -> MirrorScore {
    MirrorScore score{};
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
    auto axis_for_node = [&](ObjectId node_id) -> Vec3d {
      if (const Pole* pole = support_pole(node_id); pole != nullptr) {
        Vec3d support_axis = support_axis_for_pole(*pole);
        if (std::isfinite(support_axis.x) && std::isfinite(support_axis.y)) {
          return support_axis;
        }
      }
      const auto it = node_side_axis_hints.find(node_id);
      if (it != node_side_axis_hints.end() && std::isfinite(it->second.x) && std::isfinite(it->second.y)) {
        return it->second;
      }
      return lateral_axis;
    };
    const Vec3d axis_a = axis_for_node(node_a);
    const Vec3d axis_b = axis_for_node(node_b);
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
  constexpr double kReverseStraightAngleDeg = 179.999;

  const std::size_t segment_count = node_ids.size() - 1;
  std::vector<std::vector<ObjectId>> prepared_ports_b(segment_count);
  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    const ObjectId node_a = node_ids[seg];
    const ObjectId node_b = node_ids[seg + 1];
    const ObjectId right_order_peer = node_a;
    EditResult<std::vector<ObjectId>> right_ports =
        ensure_ports(node_b, right_order_peer, static_cast<int>(seg), false, nullptr);
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
    if (use_lane_row_geometry && a.adjacent_xy_intersections != b.adjacent_xy_intersections) {
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

  std::vector<int> node_parity(node_ids.size(), 0);
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
        candidate.mirror = evaluate_increment(node_ids[0], node_ids[1], ports_a, ports_b);
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
        cell.score.mirror = evaluate_increment(node_ids[0], node_ids[1], ports_0, ports_1);
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
                             evaluate_increment(node_ids[step], node_ids[step + 1], ports_prev, ports_curr));
            if (use_lane_row_geometry) {
              const std::vector<ObjectId> ports_prev_prev =
                  order_for_parity(base_ports_by_node[step - 1], parity_prev_prev != 0);
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
    assignment.port_ids_a = order_for_parity(base_ports_by_node[seg], node_parity[seg] != 0);
    assignment.port_ids_b = order_for_parity(base_ports_by_node[seg + 1], node_parity[seg + 1] != 0);
    const bool chosen_mirror = ((node_parity[seg] ^ node_parity[seg + 1]) != 0);
    const double turn_angle_deg = compute_turn_angle_deg(seg);
    const bool is_acute_turn = (seg > 0) && (turn_angle_deg + kAngleEps < layout_settings_.corner_threshold_deg);
    LaneFlipReason flip_reason = LaneFlipReason::kNone;
    bool flipped_from_previous = false;
    const bool previous_mirror = (seg > 0) ? ((node_parity[seg - 1] ^ node_parity[seg]) != 0) : false;
    if (seg > 0 && chosen_mirror != previous_mirror && (turn_angle_deg + kAngleEps < kReverseStraightAngleDeg)) {
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

    auto ports_use_branch_support = [&](const std::vector<ObjectId>& port_ids) {
      return std::any_of(port_ids.begin(), port_ids.end(), [&](ObjectId port_id) {
        const Port* port = edit_state_access().ports.find(port_id);
        return port != nullptr && port->placement_source == PortPlacementSourceKind::kBranchSupport;
      });
    };
    assignment.uses_branch_support =
        ports_use_branch_support(assignment.port_ids_a) || ports_use_branch_support(assignment.port_ids_b);
    assignment.branch_down_offset_m = assignment.uses_branch_support ? effective_branch_down_offset_m : 0.0;

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
      edge_orientation.flow_kind = flow_kind;
      edge_orientation.orientation = assignment.mirrored ? LaneOrientation::kReversed : LaneOrientation::kNormal;
      edge_orientation.uses_branch_support = assignment.uses_branch_support;
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

