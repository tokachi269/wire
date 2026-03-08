#include "wire/core/core_state.hpp"
#include "../pole_orientation_utils.hpp"
#include "backbone_prepare.hpp"
#include "detail_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wire::core {

using namespace generation::detail;

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

  generation::detail::NodeSpecByIndex node_spec_by_index{};
  generation::detail::NodeBundleModeByPoint node_bundle_mode_by_point{};
  if (!generation::detail::build_backbone_node_maps(request, &node_spec_by_index,
                                                              &node_bundle_mode_by_point, &result.error)) {
    return result;
  }

  struct BundlePlan {
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

  std::vector<BundlePlan> bundle_plans{};
  bundle_plans.reserve(bundle_requests.size());
  for (const BackboneBundleSpec& bundle_request : bundle_requests) {
    const BundleTemplate* bundle_template = find_bundle_template(bundle_request.bundle_template_id);
    if (bundle_template == nullptr) {
      result.error = "bundle template not found";
      return result;
    }
    BundlePlan plan{};
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
  auto node_spec_for_point = [&](std::size_t point_index) -> const BackboneInputSpec::NodeSpec* {
    const auto it = node_spec_by_index.find(point_index);
    return (it == node_spec_by_index.end()) ? nullptr : &it->second;
  };

  auto request_point_reuses_source_edge_support = [&](std::size_t point_index) -> bool {
    const BackboneInputSpec::NodeSpec* node_spec = node_spec_for_point(point_index);
    if (node_spec == nullptr || node_spec->support_kind == SupportKind::kPole ||
        node_spec->node_id == kInvalidObjectId) {
      return false;
    }
    for (const SupportNode& node : last_generation_backbone_.nodes) {
      if (node.node_id == node_spec->node_id) {
        return node.has_source_edge;
      }
    }
    return false;
  };

  bool request_has_non_pole_points = false;
  bool request_has_source_edge_branch_points = false;
  for (std::size_t point_index = 0; point_index < request.path.polyline.size(); ++point_index) {
    if (support_kind_for_point(point_index) == SupportKind::kPole) {
      continue;
    }
    request_has_non_pole_points = true;
    if (request_point_reuses_source_edge_support(point_index)) {
      request_has_source_edge_branch_points = true;
    }
  }

  std::vector<BundlePlan> active_bundle_plans{};
  active_bundle_plans.reserve(bundle_plans.size());
  for (const BundlePlan& plan : bundle_plans) {
    if (request_has_non_pole_points && !plan.allow_midair_node) {
      continue;
    }
    if (request_has_source_edge_branch_points && !plan.allow_midair_branch) {
      continue;
    }
    active_bundle_plans.push_back(plan);
  }

  for (std::size_t point_index = 0; point_index < request.path.polyline.size(); ++point_index) {
    const SupportKind support_kind = support_kind_for_point(point_index);
    if (support_kind == SupportKind::kPole) {
      continue;
    }
    const auto it_modes = node_bundle_mode_by_point.find(point_index);
    if (it_modes == node_bundle_mode_by_point.end()) {
      continue;
    }
    for (const auto& [bundle_template_id, mode] : it_modes->second) {
      const auto it_plan =
          std::find_if(bundle_plans.begin(), bundle_plans.end(),
                       [&](const BundlePlan& p) { return p.template_id == bundle_template_id; });
      if (it_plan == bundle_plans.end()) {
        result.error = "node_bundle_modes references bundle template that is not selected";
        return result;
      }
      if (mode != BundleNodeMode::kNotPresent && mode != BundleNodeMode::kPassThrough) {
        result.error = "unsupported bundle node mode";
        return result;
      }
    }
  }

  std::vector<Vec3d> guide_points{};
  PathDirectionEvaluationDebug direction_debug{};
  generation::detail::build_backbone_guide_points(request, &guide_points, &direction_debug);
  last_path_direction_debug_ = direction_debug;
  path_direction_debug_records_access().push_back(direction_debug);
  if (path_direction_debug_records_access().size() > 128) {
    path_direction_debug_records_access().erase(path_direction_debug_records_access().begin());
  }

  if (active_bundle_plans.empty()) {
    result.ok = true;
    return result;
  }

  std::vector<generation::detail::SupportNodeCandidate> candidates{};
  if (!generation::detail::build_backbone_candidates(request, guide_points, node_spec_by_index,
                                                               &candidates, &result.error)) {
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
        compute_auto_pole_transform(guide_points, i, has_preferred_side_dir ? &preferred_side_dir : nullptr);
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

  auto ensure_support_node = [&](ObjectId node_id, const generation::detail::SupportNodeCandidate& candidate,
                                 ObjectId pole_id) {
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
    const generation::detail::SupportNodeCandidate& candidate = candidates[i];
    const BackboneInputSpec::NodeSpec* explicit_node_spec =
        (candidate.vertex_index >= 0) ? node_spec_for_point(static_cast<std::size_t>(candidate.vertex_index)) : nullptr;
    const ObjectId explicit_node_id =
        (explicit_node_spec != nullptr) ? explicit_node_spec->node_id : kInvalidObjectId;
    if (candidate.support_kind != SupportKind::kPole) {
      ObjectId support_node_id = explicit_node_id;
      if (support_node_id == kInvalidObjectId) {
        support_node_id = next_virtual_support_id++;
      } else {
        const SupportNode* existing_node = nullptr;
        for (const SupportNode& node : last_generation_backbone_.nodes) {
          if (node.node_id == support_node_id) {
            existing_node = &node;
            break;
          }
        }
        if (existing_node == nullptr) {
          result.error = "node_specs node_id for support node was not found";
          return result;
        }
        if (existing_node->support_kind != candidate.support_kind) {
          result.error = "node_specs node_id support kind does not match path node kind";
          return result;
        }
        support_node_by_id[support_node_id] = *existing_node;
      }
      ensure_support_node(support_node_id, candidate, kInvalidObjectId);
      ordered_support_node_ids.push_back(support_node_id);
      continue;
    }
    ObjectId pole_id = explicit_node_id;
    if (pole_id != kInvalidObjectId && edit_state_access().poles.find(pole_id) == nullptr) {
      result.error = "node_specs node_id for pole was not found";
      return result;
    }
    if (pole_id == kInvalidObjectId) {
      pole_id = find_near_pole(candidate.world, candidate.mode);
    }
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

  if (ordered_support_node_ids.size() < 2) {
    *this = snapshot;
    result.error = "failed to create or resolve guide support nodes";
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
  if (ordered_support_node_ids.size() < 2) {
    *this = snapshot;
    result.error = "failed to build valid support-node chain";
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

  const BackboneResult& existing_backbone = last_generation_backbone_;
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
    std::unordered_map<BundleKind, BundleNodeMode> mode_by_bundle{};
    for (const BundlePlan& plan : bundle_plans) {
      mode_by_bundle[plan.template_id] = BundleNodeMode::kNotPresent;
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

  auto resolve_span_endpoint_node = [&](const Span& span, const Port* port, bool is_a) -> ObjectId {
    const ObjectId explicit_node_id = is_a ? span.endpoint_node_a_id : span.endpoint_node_b_id;
    if (explicit_node_id != kInvalidObjectId) {
      return explicit_node_id;
    }
    return (port == nullptr) ? kInvalidObjectId : port->owner_pole_id;
  };
  auto count_existing_segment_spans = [&](ObjectId node_a, ObjectId node_b, const BundlePlan& plan) -> int {
    int count = 0;
    for (const Span& span : edit_state_access().spans.items()) {
      if (span.layer != plan.layer || span.bundle_id == kInvalidObjectId) {
        continue;
      }
      const Bundle* bundle = edit_state_access().bundles.find(span.bundle_id);
      if (bundle == nullptr || bundle->bundle_template_id != plan.template_id) {
        continue;
      }
      const Port* pa = edit_state_access().ports.find(span.port_a_id);
      const Port* pb = edit_state_access().ports.find(span.port_b_id);
      if (pa == nullptr || pb == nullptr) {
        continue;
      }
      const ObjectId span_node_a = resolve_span_endpoint_node(span, pa, true);
      const ObjectId span_node_b = resolve_span_endpoint_node(span, pb, false);
      const bool direct = (span_node_a == node_a && span_node_b == node_b);
      const bool reverse = (span_node_a == node_b && span_node_b == node_a);
      if (direct || reverse) {
        ++count;
      }
    }
    return count;
  };

  std::vector<SegmentLaneAssignment> all_lane_assignments{};
  for (const BundlePlan& plan : active_bundle_plans) {
    int missing_total = 0;
    std::size_t first_missing_segment = ordered_support_node_ids.size();
    for (std::size_t i = 0; i + 1 < ordered_support_node_ids.size(); ++i) {
      const int existing_count =
          count_existing_segment_spans(ordered_support_node_ids[i], ordered_support_node_ids[i + 1], plan);
      const int missing = std::max(0, plan.count - existing_count);
      missing_total += missing;
      if (missing > 0 && first_missing_segment == ordered_support_node_ids.size()) {
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

    if (first_missing_segment >= ordered_support_node_ids.size() - 1) {
      continue;
    }
    std::vector<ObjectId> local_support_nodes{};
    local_support_nodes.insert(local_support_nodes.end(),
                               ordered_support_node_ids.begin() + static_cast<std::ptrdiff_t>(first_missing_segment),
                               ordered_support_node_ids.end());

    std::vector<SegmentLaneAssignment> lane_assignments{};
    std::vector<BackboneEdgeOrientation> edge_orientations{};
    EditResult<std::vector<ObjectId>> spans_result = generate_grouped_spans_between_support_nodes(
        local_support_nodes, support_node_by_id, bundle_id, plan.category, plan.count, plan.spacing_m, true,
        plan.allow_mirror, &lane_assignments, &edge_orientations, plan.template_id);
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


} // namespace wire::core

