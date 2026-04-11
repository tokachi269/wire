#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"
#include "../build_backbone/build_backbone_types.hpp"
#include "../detail_utils.hpp"
#include "../../pole_orientation_utils.hpp"

#include <unordered_map>

namespace wire::core {

using namespace generation::detail;

EditResult<BackboneSupportChainPlan> CoreState::build_backbone_support_chain_plan(
    const BackboneGenerationRequestPlan& request_plan) const {
  EditResult<BackboneSupportChainPlan> result{};
  BackboneSupportChainPlan plan{};
  const CoreView core_view = view();
  const EditState& edit_state = core_view.edit_state();
  const BackboneResult existing_backbone = BuildBackboneResult();

  std::unordered_map<ObjectId, SupportNode> existing_support_nodes{};
  existing_support_nodes.reserve(existing_backbone.nodes.size());
  for (const SupportNode& node : existing_backbone.nodes) {
    existing_support_nodes.emplace(node.node_id, node);
  }

  auto node_spec_for_point = [&](std::size_t point_index) -> const BackboneInputSpec::NodeSpec* {
    const auto it = request_plan.node_spec_by_index.find(point_index);
    return (it == request_plan.node_spec_by_index.end()) ? nullptr : &it->second;
  };

  auto find_near_pole = [&](const Vec3d& world, PlacementMode preferred_mode) -> ObjectId {
    constexpr double kReuseRadius = 0.25;
    const double reuse_r2 = kReuseRadius * kReuseRadius;
    ObjectId best_id = kInvalidObjectId;
    double best_d2 = reuse_r2 + 1.0;
    bool best_mode_match = false;
    for (const Pole& pole : edit_state.poles.items()) {
      if (request_plan.request.pole_placement.restrict_reuse_to_session) {
        if (request_plan.request.pole_placement.reuse_session_id == 0 ||
            pole.generation.generation_session_id != request_plan.request.pole_placement.reuse_session_id) {
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

  auto ensure_support_node = [&](ObjectId node_id, const SupportNodeCandidate& candidate, ObjectId pole_id) {
    SupportNode& node = plan.support_node_by_id[node_id];
    node.node_id = node_id;
    node.support_kind = candidate.support_kind;
    node.position = candidate.world;
    node.pole_id = pole_id;
    node.path_point_index = candidate.vertex_index;
    node.has_tangent_hint = candidate.has_tangent_hint;
    node.tangent_hint = candidate.tangent_hint;
  };

  Vec3d preferred_side_dir{0.0, 0.0, 0.0};
  bool has_preferred_side_dir = false;
  ObjectId next_virtual_support_id = 0x8000000000000000ull;
  ObjectId next_planned_pole_id = 0x4000000000000000ull;

  plan.resolutions.reserve(request_plan.candidates.size());
  plan.ordered_pole_ids.reserve(request_plan.candidates.size());
  plan.ordered_support_node_ids.reserve(request_plan.candidates.size());

  for (std::size_t i = 0; i < request_plan.candidates.size(); ++i) {
    const SupportNodeCandidate& candidate = request_plan.candidates[i];
    const BackboneInputSpec::NodeSpec* explicit_node_spec =
        (candidate.vertex_index >= 0) ? node_spec_for_point(static_cast<std::size_t>(candidate.vertex_index)) : nullptr;
    const ObjectId explicit_node_id =
        (explicit_node_spec != nullptr) ? explicit_node_spec->node_id : kInvalidObjectId;

    BackboneSupportResolution resolution{};
    resolution.candidate_index = static_cast<int>(i);
    resolution.support_kind = candidate.support_kind;
    resolution.placement_mode = candidate.mode;

    if (candidate.support_kind != SupportKind::kPole) {
      ObjectId support_node_id = explicit_node_id;
      if (support_node_id == kInvalidObjectId) {
        support_node_id = next_virtual_support_id++;
        resolution.kind = BackboneSupportResolutionKind::kCreateVirtualSupport;
      } else {
        const auto it_existing = existing_support_nodes.find(support_node_id);
        if (it_existing == existing_support_nodes.end()) {
          result.error = "node_specs node_id for support node was not found";
          return result;
        }
        if (it_existing->second.support_kind != candidate.support_kind) {
          result.error = "node_specs node_id support kind does not match path node kind";
          return result;
        }
        plan.support_node_by_id[support_node_id] = it_existing->second;
        resolution.kind = BackboneSupportResolutionKind::kReuseSupportNode;
        resolution.existing_node_id = support_node_id;
      }
      resolution.planned_node_id = support_node_id;
      ensure_support_node(support_node_id, candidate, kInvalidObjectId);
      plan.ordered_support_node_ids.push_back(support_node_id);
      plan.resolutions.push_back(resolution);
      continue;
    }

    ObjectId pole_id = explicit_node_id;
    if (pole_id != kInvalidObjectId && edit_state.poles.find(pole_id) == nullptr) {
      result.error = "node_specs node_id for pole was not found";
      return result;
    }
    if (pole_id == kInvalidObjectId) {
      pole_id = find_near_pole(candidate.world, candidate.mode);
    }
    if (pole_id != kInvalidObjectId) {
      const Pole* existing_pole = edit_state.poles.find(pole_id);
      if (existing_pole == nullptr) {
        result.error = "resolved reused pole was not found";
        return result;
      }
      resolution.kind = BackboneSupportResolutionKind::kReusePole;
      resolution.existing_node_id = pole_id;
      resolution.planned_node_id = pole_id;
      Pole authored_pole = *existing_pole;
      if (candidate.mode == PlacementMode::kManual) {
        apply_pole_placement_mode(authored_pole, PlacementMode::kManual);
      }
      if (candidate.vertex_index >= 0) {
        Vec3d base_rotation_euler_deg = existing_pole->world_transform.rotation_euler_deg;
        base_rotation_euler_deg.z = 0.0;
        const AutoPoleTransformResult auto_tf =
            compute_auto_pole_transform(request_plan.guide_points, static_cast<std::size_t>(candidate.vertex_index),
                                        has_preferred_side_dir ? &preferred_side_dir : nullptr, &base_rotation_euler_deg);
        authored_pole.context =
            classify_pole_context_from_path(request_plan.guide_points, static_cast<std::size_t>(candidate.vertex_index), 0);
        apply_sharp_debug_to_context(&authored_pole.context, auto_tf.sharp);
        if (!has_pole_orientation_override(authored_pole.id)) {
          authored_pole.world_transform.rotation_euler_deg.z = auto_tf.transform.rotation_euler_deg.z;
        }
        const PoleFrame preferred_frame = BuildPoleFrame(auto_tf.transform, auto_tf.transform.rotation_euler_deg.z);
        preferred_side_dir = preferred_frame.lateral;
        has_preferred_side_dir = Normalize(&preferred_side_dir);
      } else {
        authored_pole.context.kind = PoleContextKind::kStraight;
        apply_sharp_debug_to_context(&authored_pole.context, SharpCornerOrientationDebug{});
        if (!has_pole_orientation_override(authored_pole.id)) {
          const Vec3d dir =
              request_plan.guide_points[candidate.segment_index + 1] - request_plan.guide_points[candidate.segment_index];
          if ((dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) > 1e-12) {
            authored_pole.world_transform.rotation_euler_deg.z =
                normalize_yaw_deg(std::atan2(dir.y, dir.x) * (180.0 / kPi));
          }
        }
      }
      resolution.authored_transform = authored_pole.world_transform;
      resolution.authored_context = authored_pole.context;
      plan.ordered_pole_ids.push_back(pole_id);
      ensure_support_node(pole_id, candidate, pole_id);
      plan.ordered_support_node_ids.push_back(pole_id);
      plan.resolutions.push_back(resolution);
      continue;
    }

    Transformd tf{};
    PoleContextInfo context{};
    tf.position = candidate.world;
    if (candidate.vertex_index >= 0) {
      const AutoPoleTransformResult auto_tf =
          compute_auto_pole_transform(request_plan.guide_points, static_cast<std::size_t>(candidate.vertex_index),
                                      has_preferred_side_dir ? &preferred_side_dir : nullptr);
      tf = auto_tf.transform;
      tf.position = candidate.world;
      context = classify_pole_context_from_path(request_plan.guide_points, static_cast<std::size_t>(candidate.vertex_index), 0);
      apply_sharp_debug_to_context(&context, auto_tf.sharp);
      const PoleFrame preferred_frame = BuildPoleFrame(tf, tf.rotation_euler_deg.z);
      preferred_side_dir = preferred_frame.lateral;
      has_preferred_side_dir = Normalize(&preferred_side_dir);
    } else {
      context.kind = PoleContextKind::kStraight;
      apply_sharp_debug_to_context(&context, SharpCornerOrientationDebug{});
      const Vec3d dir =
          request_plan.guide_points[candidate.segment_index + 1] - request_plan.guide_points[candidate.segment_index];
      if ((dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) > 1e-12) {
        tf.rotation_euler_deg.z = normalize_yaw_deg(std::atan2(dir.y, dir.x) * (180.0 / kPi));
      }
    }

    const ObjectId planned_pole_id = next_planned_pole_id++;
    resolution.kind = BackboneSupportResolutionKind::kCreatePole;
    resolution.planned_node_id = planned_pole_id;
    resolution.authored_transform = tf;
    resolution.authored_context = context;
    plan.pole_creations.push_back({planned_pole_id, tf, candidate.mode, static_cast<int>(i), context,
                                   request_plan.request.pole_type_id, PoleKind::kConcrete, 10.0, "PathPole"});
    plan.generated_pole_ids.push_back(planned_pole_id);
    plan.ordered_pole_ids.push_back(planned_pole_id);
    ensure_support_node(planned_pole_id, candidate, planned_pole_id);
    plan.ordered_support_node_ids.push_back(planned_pole_id);
    plan.resolutions.push_back(resolution);
  }

  auto compact_ids = [](std::vector<ObjectId>* ids) {
    if (ids == nullptr) {
      return;
    }
    std::vector<ObjectId> compact{};
    compact.reserve(ids->size());
    for (ObjectId id : *ids) {
      if (compact.empty() || compact.back() != id) {
        compact.push_back(id);
      }
    }
    ids->swap(compact);
  };
  compact_ids(&plan.ordered_pole_ids);
  compact_ids(&plan.ordered_support_node_ids);
  if (plan.ordered_support_node_ids.size() < 2) {
    result.error = "failed to build valid support-node chain";
    return result;
  }

  result.value = std::move(plan);
  result.ok = true;
  return result;
}

} // namespace wire::core
