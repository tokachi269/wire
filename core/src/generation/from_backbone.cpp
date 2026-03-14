#include "wire/core/core_state.hpp"
#include "../pole_orientation_utils.hpp"
#include "backbone_prepare.hpp"
#include "detail_utils.hpp"
#include "support_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wire::core {

using namespace generation::detail;

namespace {

Vec3d normalize_forward_xy(const Vec3d& value) {
  Vec3d out{value.x, value.y, 0.0};
  if (!Normalize(&out)) {
    return {};
  }
  return out;
}

Vec3d choose_continuous_axis(const Vec3d& axis, const Vec3d& previous_forward) {
  Vec3d out = axis;
  if (!Normalize(&out)) {
    return {};
  }
  Vec3d prev = normalize_forward_xy(previous_forward);
  if (Dot(out, prev) < 0.0) {
    out = ScaleVec(out, -1.0);
  }
  return out;
}

std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}

std::uint64_t hash_combine(std::uint64_t seed, std::uint64_t value) {
  return splitmix64(seed ^ (value + 0x9E3779B97F4A7C15ull + (seed << 6) + (seed >> 2)));
}

std::uint64_t make_flow_variation_key(std::uint64_t generation_session_id, BundleKind bundle_template_id,
                                      BackboneFlowKind flow_kind, ObjectId run_start_node_id,
                                      ObjectId run_end_node_id) {
  std::uint64_t key = hash_combine(generation_session_id, static_cast<std::uint64_t>(bundle_template_id));
  key = hash_combine(key, static_cast<std::uint64_t>(flow_kind));
  key = hash_combine(key, static_cast<std::uint64_t>(run_start_node_id));
  key = hash_combine(key, static_cast<std::uint64_t>(run_end_node_id));
  return key;
}

} // namespace

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
    bool enable_branch_down_offset = false;
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
    plan.enable_branch_down_offset = bundle_template->enable_branch_down_offset;
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

  const BackboneResult existing_network_backbone = BuildBackboneResult();

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
          if (!has_pole_orientation_override(pole->id)) {
            pole->world_transform.rotation_euler_deg.z = auto_tf.transform.rotation_euler_deg.z;
            updated = true;
          }
        } else {
          pole->context.kind = PoleContextKind::kStraight;
          apply_sharp_debug_to_context(&pole->context, SharpCornerOrientationDebug{});
          if (!has_pole_orientation_override(pole->id)) {
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

  const BackboneResult& existing_backbone = existing_network_backbone;
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

    double best_pair_straight = -2.0;
    int best_pair_i = -1;
    int best_pair_j = -1;
    int best_pair_anchor = -1;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      for (std::size_t j = i + 1; j < candidates.size(); ++j) {
        const double straight_score =
            dot(candidates[i].dir, Vec3d{-candidates[j].dir.x, -candidates[j].dir.y, -candidates[j].dir.z});
        int pair_anchor = static_cast<int>(i);
        if (candidates[j].neighbor_id < candidates[i].neighbor_id) {
          pair_anchor = static_cast<int>(j);
        }
        if (straight_score > best_pair_straight + 1e-9 ||
            (std::abs(straight_score - best_pair_straight) <= 1e-9 &&
             (best_pair_anchor < 0 || candidates[static_cast<std::size_t>(pair_anchor)].neighbor_id <
                                           candidates[static_cast<std::size_t>(best_pair_anchor)].neighbor_id))) {
          best_pair_straight = straight_score;
          best_pair_i = static_cast<int>(i);
          best_pair_j = static_cast<int>(j);
          best_pair_anchor = pair_anchor;
        }
      }
    }
    auto belongs_to_best_pair = [&](ObjectId neighbor_id) {
      if (best_pair_i < 0 || best_pair_j < 0) {
        return false;
      }
      return candidates[static_cast<std::size_t>(best_pair_i)].neighbor_id == neighbor_id ||
             candidates[static_cast<std::size_t>(best_pair_j)].neighbor_id == neighbor_id;
    };

    int anchor_index = -1;
    bool used_neighbor_continuity = false;
    const auto it_existing_primary = existing_primary_neighbor_by_node.find(node_id);
    if (it_existing_primary != existing_primary_neighbor_by_node.end() &&
        belongs_to_best_pair(it_existing_primary->second)) {
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
      if (!belongs_to_best_pair(candidates[i].neighbor_id)) {
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
      anchor_index = (best_pair_anchor >= 0) ? best_pair_anchor : 0;
    }

    int opposite_index = -1;
    if (best_pair_i >= 0 && best_pair_j >= 0) {
      opposite_index = (anchor_index == best_pair_i) ? best_pair_j : best_pair_i;
    }
    if (opposite_index < 0 || opposite_index == anchor_index) {
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

  std::unordered_map<ObjectId, Vec3d> existing_node_position_by_id{};
  existing_node_position_by_id.reserve(existing_network_backbone.nodes.size() + edit_state_access().poles.size());
  for (const SupportNode& node : existing_network_backbone.nodes) {
    existing_node_position_by_id[node.node_id] = node.position;
  }
  for (const Pole& pole : edit_state_access().poles.items()) {
    existing_node_position_by_id.try_emplace(pole.id, pole.world_transform.position);
  }

  std::unordered_map<ObjectId, std::vector<ObjectId>> existing_adjacency{};
  for (const BackboneEdge& edge : existing_network_backbone.edges) {
    if (edge.node_a == kInvalidObjectId || edge.node_b == kInvalidObjectId || edge.node_a == edge.node_b) {
      continue;
    }
    existing_adjacency[edge.node_a].push_back(edge.node_b);
    existing_adjacency[edge.node_b].push_back(edge.node_a);
  }
  for (auto& [_, neighbors] : existing_adjacency) {
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
  }

  std::unordered_map<ObjectId, const JunctionInfo*> existing_junction_by_node{};
  existing_junction_by_node.reserve(existing_network_backbone.junctions.size());
  for (const JunctionInfo& junction : existing_network_backbone.junctions) {
    existing_junction_by_node[junction.node_id] = &junction;
  }
  std::unordered_map<ObjectId, const JunctionInfo*> active_junction_by_node = existing_junction_by_node;
  active_junction_by_node.reserve(existing_junction_by_node.size() + generation_backbone.junctions.size());
  for (const JunctionInfo& junction : generation_backbone.junctions) {
    active_junction_by_node[junction.node_id] = &junction;
  }

  std::unordered_map<ObjectId, std::vector<ObjectId>> route_neighbors_by_node{};
  for (std::size_t i = 0; i < ordered_support_node_ids.size(); ++i) {
    const ObjectId node_id = ordered_support_node_ids[i];
    std::vector<ObjectId>& neighbors = route_neighbors_by_node[node_id];
    if (i > 0) {
      const ObjectId prev = ordered_support_node_ids[i - 1];
      if (prev != node_id && std::find(neighbors.begin(), neighbors.end(), prev) == neighbors.end()) {
        neighbors.push_back(prev);
      }
    }
    if (i + 1 < ordered_support_node_ids.size()) {
      const ObjectId next = ordered_support_node_ids[i + 1];
      if (next != node_id && std::find(neighbors.begin(), neighbors.end(), next) == neighbors.end()) {
        neighbors.push_back(next);
      }
    }
  }

  auto current_support_position = [&](ObjectId node_id) -> Vec3d {
    if (const auto it = support_node_by_id.find(node_id); it != support_node_by_id.end()) {
      return it->second.position;
    }
    if (const Pole* pole = edit_state_access().poles.find(node_id); pole != nullptr) {
      return pole->world_transform.position;
    }
    if (const auto it = existing_node_position_by_id.find(node_id); it != existing_node_position_by_id.end()) {
      return it->second;
    }
    return {};
  };
  auto connected_neighbors_for_support_axis = [&](ObjectId node_id) {
    std::vector<ObjectId> neighbors{};
    for (const Span& span : edit_state_access().spans.items()) {
      const Port* port_a = edit_state_access().ports.find(span.port_a_id);
      const Port* port_b = edit_state_access().ports.find(span.port_b_id);
      if (port_a == nullptr || port_b == nullptr) {
        continue;
      }
      if (port_a->owner_pole_id == node_id && port_b->owner_pole_id != kInvalidObjectId && port_b->owner_pole_id != node_id) {
        neighbors.push_back(port_b->owner_pole_id);
      }
      if (port_b->owner_pole_id == node_id && port_a->owner_pole_id != kInvalidObjectId && port_a->owner_pole_id != node_id) {
        neighbors.push_back(port_a->owner_pole_id);
      }
    }
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    return neighbors;
  };

  auto existing_continuation_neighbors_for_orientation = [&](ObjectId node_id) {
    std::vector<ObjectId> neighbors{};
    if (const auto it = active_junction_by_node.find(node_id); it != active_junction_by_node.end()) {
      const JunctionInfo* junction = it->second;
      if (junction != nullptr && junction->incidents.size() >= 2) {
        const JunctionIncident* primary_incident = nullptr;
        const JunctionIncident* continuation_incident = nullptr;
        for (const JunctionIncident& incident : junction->incidents) {
          if ((incident.primary || incident.order == 0) &&
              (primary_incident == nullptr ||
               (incident.primary && !primary_incident->primary) ||
               (incident.order >= 0 && (primary_incident->order < 0 || incident.order < primary_incident->order)))) {
            primary_incident = &incident;
          }
          if (incident.order == 1 &&
              (continuation_incident == nullptr || incident.neighbor_node_id < continuation_incident->neighbor_node_id)) {
            continuation_incident = &incident;
          }
        }
        const bool has_continuation_pair =
            primary_incident != nullptr && continuation_incident != nullptr &&
            primary_incident->neighbor_node_id != continuation_incident->neighbor_node_id &&
            (junction->used_neighbor_continuity || junction->incidents.size() == 2);
        if (has_continuation_pair) {
          neighbors.push_back(primary_incident->neighbor_node_id);
          neighbors.push_back(continuation_incident->neighbor_node_id);
          return neighbors;
        }
      }
    }
    const auto it_route = route_neighbors_by_node.find(node_id);
    const std::size_t route_degree = (it_route == route_neighbors_by_node.end()) ? 0 : it_route->second.size();
    if (route_degree == 1) {
      if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 2) {
        return it->second;
      }
    }
    if (route_degree == 0) {
      if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 2) {
        return it->second;
      }
    }
    return neighbors;
  };
  auto existing_primary_neighbor_for_orientation = [&](ObjectId node_id) -> ObjectId {
    if (const auto it = active_junction_by_node.find(node_id); it != active_junction_by_node.end()) {
      const JunctionInfo* junction = it->second;
      if (junction != nullptr) {
        for (const JunctionIncident& incident : junction->incidents) {
          if (incident.primary || incident.order == 0) {
            return incident.neighbor_node_id;
          }
        }
      }
    }
    const auto it_route = route_neighbors_by_node.find(node_id);
    const std::size_t route_degree = (it_route == route_neighbors_by_node.end()) ? 0 : it_route->second.size();
    if (route_degree == 1) {
      if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 1) {
        return it->second.front();
      }
    }
    if (route_degree == 0) {
      if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 1) {
        return it->second.front();
      }
    }
    return kInvalidObjectId;
  };
  auto preferred_straight_main_pair_for_orientation = [&](ObjectId node_id) -> std::pair<ObjectId, ObjectId> {
    const auto it_route = route_neighbors_by_node.find(node_id);
    const std::size_t route_degree = (it_route == route_neighbors_by_node.end()) ? 0 : it_route->second.size();
    if (route_degree == 0) {
      return {kInvalidObjectId, kInvalidObjectId};
    }

    std::vector<ObjectId> combined_neighbors{};
    if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end()) {
      combined_neighbors = it->second;
    }
    if (it_route != route_neighbors_by_node.end()) {
      for (ObjectId neighbor_id : it_route->second) {
        if (std::find(combined_neighbors.begin(), combined_neighbors.end(), neighbor_id) == combined_neighbors.end()) {
          combined_neighbors.push_back(neighbor_id);
        }
      }
    }
    std::sort(combined_neighbors.begin(), combined_neighbors.end());
    combined_neighbors.erase(std::unique(combined_neighbors.begin(), combined_neighbors.end()), combined_neighbors.end());
    if (combined_neighbors.size() < 2) {
      return {kInvalidObjectId, kInvalidObjectId};
    }

    struct Candidate {
      ObjectId neighbor_id = kInvalidObjectId;
      Vec3d dir{};
    };
    std::vector<Candidate> candidates{};
    candidates.reserve(combined_neighbors.size());
    const Vec3d center = current_support_position(node_id);
    for (ObjectId neighbor_id : combined_neighbors) {
      const Vec3d dir = normalize_forward_xy(current_support_position(neighbor_id) - center);
      if (!std::isfinite(dir.x) || !std::isfinite(dir.y)) {
        continue;
      }
      candidates.push_back({neighbor_id, dir});
    }
    if (candidates.size() < 2) {
      return {kInvalidObjectId, kInvalidObjectId};
    }

    double best_score = -2.0;
    int best_i = -1;
    int best_j = -1;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      for (std::size_t j = i + 1; j < candidates.size(); ++j) {
        const double straight_score =
            dot(candidates[i].dir, Vec3d{-candidates[j].dir.x, -candidates[j].dir.y, -candidates[j].dir.z});
        if (straight_score > best_score + 1e-9) {
          best_score = straight_score;
          best_i = static_cast<int>(i);
          best_j = static_cast<int>(j);
        }
      }
    }
    if (best_i < 0 || best_j < 0) {
      return {kInvalidObjectId, kInvalidObjectId};
    }

    std::vector<ObjectId> existing_pair = existing_continuation_neighbors_for_orientation(node_id);
    if (existing_pair.size() < 2) {
      if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 2) {
        existing_pair = it->second;
      }
    }
    if (existing_pair.size() >= 2) {
      const Vec3d existing_a = normalize_forward_xy(current_support_position(existing_pair[0]) - center);
      const Vec3d existing_b = normalize_forward_xy(current_support_position(existing_pair[1]) - center);
      const double existing_score = dot(existing_a, Vec3d{-existing_b.x, -existing_b.y, -existing_b.z});
      if (!(best_score > existing_score + 1e-6)) {
        return {kInvalidObjectId, kInvalidObjectId};
      }
    }

    ObjectId primary_neighbor_id = candidates[static_cast<std::size_t>(best_i)].neighbor_id;
    ObjectId secondary_neighbor_id = candidates[static_cast<std::size_t>(best_j)].neighbor_id;
    const ObjectId existing_primary_neighbor_id = existing_primary_neighbor_for_orientation(node_id);
    if (existing_primary_neighbor_id == primary_neighbor_id || existing_primary_neighbor_id == secondary_neighbor_id) {
      if (existing_primary_neighbor_id == secondary_neighbor_id) {
        std::swap(primary_neighbor_id, secondary_neighbor_id);
      }
    } else if (secondary_neighbor_id < primary_neighbor_id) {
      std::swap(primary_neighbor_id, secondary_neighbor_id);
    }
    return {primary_neighbor_id, secondary_neighbor_id};
  };
  auto continuation_neighbors_for_orientation = [&](ObjectId node_id) {
    const auto preferred_pair = preferred_straight_main_pair_for_orientation(node_id);
    if (preferred_pair.first != kInvalidObjectId && preferred_pair.second != kInvalidObjectId &&
        preferred_pair.first != preferred_pair.second) {
      return std::vector<ObjectId>{preferred_pair.first, preferred_pair.second};
    }
    return existing_continuation_neighbors_for_orientation(node_id);
  };
  auto primary_neighbor_for_orientation = [&](ObjectId node_id) -> ObjectId {
    const auto preferred_pair = preferred_straight_main_pair_for_orientation(node_id);
    if (preferred_pair.first != kInvalidObjectId) {
      return preferred_pair.first;
    }
    return existing_primary_neighbor_for_orientation(node_id);
  };
  auto has_existing_main_flow_context = [&](ObjectId node_id) -> bool {
    if (active_junction_by_node.contains(node_id)) {
      return true;
    }
    const auto it_route = route_neighbors_by_node.find(node_id);
    const std::size_t route_degree = (it_route == route_neighbors_by_node.end()) ? 0 : it_route->second.size();
    if (route_degree > 1) {
      return false;
    }
    if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end()) {
      return !it->second.empty();
    }
    return false;
  };
  auto choose_support_axis_for_layout = [&](ObjectId node_id, const Vec3d& center, const Vec3d& previous_support_axis,
                                            PoleOrientationDebugRecord* debug) {
    Vec3d chosen_axis = normalize_forward_xy(previous_support_axis);
    bool has_axis = Normalize(&chosen_axis);
    auto adopt_axis = [&](const Vec3d& axis, PoleSupportAxisRule rule, ObjectId primary_neighbor_id,
                          ObjectId secondary_neighbor_id) {
      Vec3d normalized_axis = normalize_forward_xy(axis);
      if (!Normalize(&normalized_axis)) {
        return false;
      }
      Vec3d row_axis = ComputeLateralAxis(normalized_axis);
      if (!Normalize(&row_axis)) {
        return false;
      }
      chosen_axis = choose_continuous_axis(row_axis, previous_support_axis);
      has_axis = Normalize(&chosen_axis);
      if (has_axis && debug != nullptr) {
        debug->support_axis_rule = rule;
        debug->primary_neighbor_id = primary_neighbor_id;
        debug->secondary_neighbor_id = secondary_neighbor_id;
      }
      return has_axis;
    };

    const std::vector<ObjectId> connected_neighbors = connected_neighbors_for_support_axis(node_id);
    if (connected_neighbors.size() == 2) {
      const ObjectId neighbor_a = connected_neighbors[0];
      const ObjectId neighbor_b = connected_neighbors[1];
      const Vec3d axis = current_support_position(neighbor_a) - center;
      if (adopt_axis(axis, PoleSupportAxisRule::kMainChainPair, neighbor_a, neighbor_b)) {
        return chosen_axis;
      }
    }

    if (const auto it = active_junction_by_node.find(node_id); it != active_junction_by_node.end()) {
      const std::vector<ObjectId> continuation_neighbors = continuation_neighbors_for_orientation(node_id);
      if (continuation_neighbors.size() >= 2) {
        const Vec3d axis = current_support_position(continuation_neighbors[0]) - center;
        if (adopt_axis(axis, PoleSupportAxisRule::kMainChainPair, continuation_neighbors[0], continuation_neighbors[1])) {
          return chosen_axis;
        }
      }
      const ObjectId primary_neighbor_id = primary_neighbor_for_orientation(node_id);
      if (primary_neighbor_id != kInvalidObjectId) {
        const Vec3d axis = current_support_position(primary_neighbor_id) - center;
        if (adopt_axis(axis, PoleSupportAxisRule::kPrimaryIncident, primary_neighbor_id, kInvalidObjectId)) {
          return chosen_axis;
        }
      }
    }

    if (const auto it_route = route_neighbors_by_node.find(node_id); it_route != route_neighbors_by_node.end() &&
                                                         !it_route->second.empty()) {
      ObjectId primary_neighbor_id = it_route->second.front();
      ObjectId secondary_neighbor_id = kInvalidObjectId;
      PoleSupportAxisRule rule = PoleSupportAxisRule::kMainChainSingle;
      if (it_route->second.size() >= 2) {
        const ObjectId candidate_a = it_route->second[0];
        const ObjectId candidate_b = it_route->second[1];
        const Vec3d delta_a = current_support_position(candidate_a) - center;
        const Vec3d delta_b = current_support_position(candidate_b) - center;
        const double len2_a = delta_a.x * delta_a.x + delta_a.y * delta_a.y;
        const double len2_b = delta_b.x * delta_b.x + delta_b.y * delta_b.y;
        primary_neighbor_id = (len2_b > len2_a + 1e-9) ? candidate_b : candidate_a;
        secondary_neighbor_id = (primary_neighbor_id == candidate_a) ? candidate_b : candidate_a;
        rule = PoleSupportAxisRule::kMainChainPair;
      }
      const Vec3d axis = current_support_position(primary_neighbor_id) - center;
      if (adopt_axis(axis, rule, primary_neighbor_id, secondary_neighbor_id)) {
        return chosen_axis;
      }
    }

    if (debug != nullptr) {
      debug->support_axis_rule = PoleSupportAxisRule::kFallback;
    }
    return has_axis ? chosen_axis : Vec3d{};
  };

  pole_orientation_debug_records_.clear();
  std::unordered_set<ObjectId> oriented_poles{};
  for (std::size_t ordered_index = 0; ordered_index < ordered_support_node_ids.size(); ++ordered_index) {
    const ObjectId node_id = ordered_support_node_ids[ordered_index];
    if (!oriented_poles.insert(node_id).second) {
      continue;
    }
    Pole* pole = edit_state_access().poles.find(node_id);
    if (pole == nullptr) {
      continue;
    }

    PoleOrientationDebugRecord debug{};
    debug.pole_id = pole->id;
    const Vec3d center = current_support_position(node_id);
    const Vec3d previous_forward = RotateAroundWorldUpDeg(WorldForward(), effective_pole_yaw_deg(*pole));
    const double previous_layout_yaw = effective_pole_layout_yaw_deg(*pole);
    const Vec3d previous_support_axis = side_axis_from_yaw_deg(previous_layout_yaw);
    Vec3d chosen_forward = normalize_forward_xy(previous_forward);
    bool has_chosen_forward = Normalize(&chosen_forward);
    Vec3d chosen_support_axis = choose_support_axis_for_layout(node_id, center, previous_support_axis, &debug);
    if (!Normalize(&chosen_support_axis)) {
      chosen_support_axis = normalize_forward_xy(previous_support_axis);
    }
    if (ordered_index > 0) {
      const ObjectId prev_node_id = ordered_support_node_ids[ordered_index - 1];
      if (prev_node_id != node_id) {
        const Pole* prev_pole = edit_state_access().poles.find(prev_node_id);
        if (prev_pole != nullptr) {
          const auto it_prev_debug = pole_orientation_debug_records_.find(prev_pole->id);
          if (it_prev_debug != pole_orientation_debug_records_.end()) {
            Vec3d previous_route_axis = normalize_forward_xy(it_prev_debug->second.adopted_support_axis);
            if (Normalize(&previous_route_axis)) {
              chosen_support_axis = choose_continuous_axis(chosen_support_axis, previous_route_axis);
            }
          }
        }
      }
    }
    const bool apply_main_flow_orientation = has_existing_main_flow_context(node_id);

    if (apply_main_flow_orientation) {
      const std::vector<ObjectId> continuation_neighbors = continuation_neighbors_for_orientation(node_id);
      if (continuation_neighbors.size() >= 2) {
        const Vec3d dir_a = normalize_forward_xy(current_support_position(continuation_neighbors[0]) - center);
        const Vec3d dir_b = normalize_forward_xy(current_support_position(continuation_neighbors[1]) - center);
        Vec3d axis = normalize_forward_xy(dir_a + dir_b);
        if (Normalize(&axis)) {
          chosen_forward = choose_continuous_axis(axis, previous_forward);
          has_chosen_forward = Normalize(&chosen_forward);
          debug.rule = PoleForwardRule::kMainChainBisector;
          debug.primary_neighbor_id = continuation_neighbors[0];
          debug.secondary_neighbor_id = continuation_neighbors[1];
        } else {
          axis = normalize_forward_xy(dir_a - dir_b);
          if (Normalize(&axis)) {
            chosen_forward = choose_continuous_axis(axis, previous_forward);
            has_chosen_forward = Normalize(&chosen_forward);
            debug.rule = PoleForwardRule::kMainChainBisector;
            debug.primary_neighbor_id = continuation_neighbors[0];
            debug.secondary_neighbor_id = continuation_neighbors[1];
          }
        }
      } else {
        const ObjectId primary_neighbor_id = primary_neighbor_for_orientation(node_id);
        if (primary_neighbor_id != kInvalidObjectId) {
          Vec3d axis = normalize_forward_xy(current_support_position(primary_neighbor_id) - center);
          if (Normalize(&axis)) {
            chosen_forward = choose_continuous_axis(axis, previous_forward);
            has_chosen_forward = true;
            debug.rule = active_junction_by_node.contains(node_id) ? PoleForwardRule::kPrimaryIncident
                                                                   : PoleForwardRule::kMainChainSingle;
            debug.primary_neighbor_id = primary_neighbor_id;
          }
        } else {
          debug.rule = PoleForwardRule::kFallback;
        }
      }
    }

    if (!has_chosen_forward) {
      chosen_forward = normalize_forward_xy(previous_forward);
      debug.rule = PoleForwardRule::kFallback;
    }
    debug.adopted_forward = chosen_forward;
    debug.adopted_support_axis = chosen_support_axis;
    pole_orientation_debug_records_[pole->id] = debug;

    const double next_layout_yaw = effective_pole_layout_yaw_deg(*pole);
    const double layout_yaw_delta = normalize_yaw_deg(next_layout_yaw - previous_layout_yaw);

    if (!has_chosen_forward || has_pole_orientation_override(pole->id)) {
      if (std::abs(layout_yaw_delta) > 1e-6) {
        const Pole old_pole = *pole;
        refresh_owned_endpoints_from_pole(pole->id, &result.change_set, &old_pole, &previous_layout_yaw);
      }
      continue;
    }

    const double desired_yaw = normalize_yaw_deg(std::atan2(chosen_forward.y, chosen_forward.x) * (180.0 / kPi));
    double yaw_delta = desired_yaw - pole->world_transform.rotation_euler_deg.z;
    yaw_delta = std::fmod(yaw_delta + 540.0, 360.0) - 180.0;
    if (std::abs(yaw_delta) <= 1e-6) {
      if (std::abs(layout_yaw_delta) > 1e-6) {
        const Pole old_pole = *pole;
        refresh_owned_endpoints_from_pole(pole->id, &result.change_set, &old_pole, &previous_layout_yaw);
      }
      continue;
    }

    const Pole old_pole = *pole;
    pole->world_transform.rotation_euler_deg.z = desired_yaw;
    finalize_pole_transform_update(pole->id, old_pole, &result.change_set);
  }

  struct EdgeFlowInfo {
    BackboneFlowKind kind = BackboneFlowKind::kMain;
    BackboneFlowDecisionRule rule = BackboneFlowDecisionRule::kDefaultMain;
  };
  auto classify_edge_flow_at_node = [&](ObjectId node_id, ObjectId peer_id) {
    EdgeFlowInfo info{};
    const auto preferred_pair = preferred_straight_main_pair_for_orientation(node_id);
    if (preferred_pair.first != kInvalidObjectId && preferred_pair.second != kInvalidObjectId) {
      const bool in_pair = (peer_id == preferred_pair.first || peer_id == preferred_pair.second);
      info.kind = in_pair ? BackboneFlowKind::kMain : BackboneFlowKind::kBranch;
      info.rule = in_pair ? BackboneFlowDecisionRule::kJunctionOrderMain
                          : BackboneFlowDecisionRule::kJunctionOrderBranch;
      return info;
    }
    if (const auto it = active_junction_by_node.find(node_id); it != active_junction_by_node.end()) {
      const JunctionInfo* junction = it->second;
      if (junction != nullptr && !junction->incidents.empty()) {
        for (const JunctionIncident& incident : junction->incidents) {
          if (incident.neighbor_node_id != peer_id) {
            continue;
          }
          info.kind = (incident.order >= 2) ? BackboneFlowKind::kBranch : BackboneFlowKind::kMain;
          info.rule = (incident.order >= 2) ? BackboneFlowDecisionRule::kJunctionOrderBranch
                                            : BackboneFlowDecisionRule::kJunctionOrderMain;
          return info;
        }
        if (junction->incidents.size() >= 2) {
          info.kind = BackboneFlowKind::kBranch;
          info.rule = BackboneFlowDecisionRule::kJunctionOrderBranch;
          return info;
        }
      }
    }
    if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 2) {
      const bool in_pair = std::find(it->second.begin(), it->second.end(), peer_id) != it->second.end();
      info.kind = in_pair ? BackboneFlowKind::kMain : BackboneFlowKind::kBranch;
      info.rule = in_pair ? BackboneFlowDecisionRule::kExistingChainMain
                          : BackboneFlowDecisionRule::kExistingChainBranch;
      return info;
    }
    return info;
  };
  auto classify_edge_flow = [&](ObjectId node_a, ObjectId node_b) {
    const EdgeFlowInfo flow_a = classify_edge_flow_at_node(node_a, node_b);
    const EdgeFlowInfo flow_b = classify_edge_flow_at_node(node_b, node_a);
    if (flow_a.kind == BackboneFlowKind::kBranch) {
      return flow_a;
    }
    if (flow_b.kind == BackboneFlowKind::kBranch) {
      return flow_b;
    }
    if (flow_a.rule != BackboneFlowDecisionRule::kDefaultMain) {
      return flow_a;
    }
    if (flow_b.rule != BackboneFlowDecisionRule::kDefaultMain) {
      return flow_b;
    }
    return flow_a;
  };

  std::vector<EdgeFlowInfo> edge_flow_by_segment{};
  edge_flow_by_segment.reserve((ordered_support_node_ids.size() > 1) ? (ordered_support_node_ids.size() - 1) : 0);
  for (std::size_t i = 0; i + 1 < ordered_support_node_ids.size(); ++i) {
    edge_flow_by_segment.push_back(classify_edge_flow(ordered_support_node_ids[i], ordered_support_node_ids[i + 1]));
  }

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
    for (std::size_t run_start = first_missing_segment; run_start + 1 < ordered_support_node_ids.size();) {
      const EdgeFlowInfo flow_info = edge_flow_by_segment[run_start];
      std::size_t run_end = run_start;
      while (run_end + 1 < ordered_support_node_ids.size() - 1 &&
             edge_flow_by_segment[run_end + 1].kind == flow_info.kind) {
        ++run_end;
      }

      std::vector<ObjectId> local_support_nodes{};
      local_support_nodes.insert(local_support_nodes.end(),
                                 ordered_support_node_ids.begin() + static_cast<std::ptrdiff_t>(run_start),
                                 ordered_support_node_ids.begin() + static_cast<std::ptrdiff_t>(run_end + 2));
      const std::uint64_t variation_flow_key =
          make_flow_variation_key(session_id, plan.template_id, flow_info.kind, local_support_nodes.front(),
                                  local_support_nodes.back());

      std::vector<SegmentLaneAssignment> lane_assignments{};
      std::vector<BackboneEdgeOrientation> edge_orientations{};
      const double branch_down_offset_m =
          (flow_info.kind == BackboneFlowKind::kBranch && plan.enable_branch_down_offset)
              ? generation::detail::BranchDownOffsetForCategory(plan.category)
              : 0.0;
      EditResult<std::vector<ObjectId>> spans_result = generate_grouped_spans_between_support_nodes(
          local_support_nodes, support_node_by_id, bundle_id, plan.category, plan.count, plan.spacing_m, true,
          plan.allow_mirror, flow_info.kind, branch_down_offset_m, &lane_assignments, &edge_orientations,
          plan.template_id);
      if (!spans_result.ok) {
        *this = snapshot;
        result.error = spans_result.error;
        return result;
      }
      append_change_set(result.change_set, spans_result.change_set);
      for (std::size_t i = 0; i < lane_assignments.size(); ++i) {
        lane_assignments[i].segment_index += run_start;
        lane_assignments[i].variation_flow_key = variation_flow_key;
        lane_assignments[i].flow_decision_rule = edge_flow_by_segment[run_start + i].rule;
      }
      for (std::size_t i = 0; i < edge_orientations.size(); ++i) {
        edge_orientations[i].variation_flow_key = variation_flow_key;
        edge_orientations[i].flow_decision_rule = edge_flow_by_segment[run_start + i].rule;
      }
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
        auto runtime_it = span_runtime_states_access().find(span_id);
        if (runtime_it != span_runtime_states_access().end()) {
          runtime_it->second.variation_flow_key = variation_flow_key;
        }
        result.value.generated_span_ids.push_back(span_id);
      }

      run_start = run_end + 1;
    }
  }

  last_lane_assignments_access() = all_lane_assignments;
  last_generation_backbone_ = generation_backbone;
  result.ok = true;
  return result;
}


} // namespace wire::core

