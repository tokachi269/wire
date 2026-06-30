#include "path_pick_policy.hpp"

#include "wire/core/coord_utils.hpp"

#include <algorithm>

namespace {

bool IsTemplateSelected(std::uint32_t selected_template_mask, wire::core::BundleKind kind) {
  const auto bit = (1u << static_cast<unsigned>(kind));
  return (selected_template_mask & bit) != 0;
}

double SegmentTxy(const wire::core::Vec3d& p, const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double ab2 = abx * abx + aby * aby;
  if (ab2 <= 1e-12) {
    return 0.0;
  }
  const double apx = p.x - a.x;
  const double apy = p.y - a.y;
  return std::clamp((apx * abx + apy * aby) / ab2, 0.0, 1.0);
}

} // namespace

std::vector<wire::core::BundleKind> SelectedBundleTemplateKinds(const wire::core::CoreView& view,
                                                                std::uint32_t selected_template_mask) {
  std::vector<wire::core::BundleKind> selected{};
  for (const auto& [kind, _] : view.bundle_templates()) {
    if (IsTemplateSelected(selected_template_mask, kind)) {
      selected.push_back(kind);
    }
  }
  std::sort(selected.begin(), selected.end(), [](auto a, auto b) { return static_cast<int>(a) < static_cast<int>(b); });
  return selected;
}

namespace {

wire::core::BundleKind ResolveHitSpanTemplateOrDefault(const wire::core::CoreView& view,
                                                       const wire::core::PickResult& pick) {
  if (pick.hit_kind == wire::core::PickHitKind::kSegment) {
    if (const wire::core::Span* span = view.edit_state().spans.find(pick.hit_id); span != nullptr) {
      if (const wire::core::Bundle* bundle = view.edit_state().bundles.find(span->bundle_id); bundle != nullptr) {
        return bundle->bundle_template_id;
      }
    }
  }
  return wire::core::BundleKind::kLowVoltage;
}

} // namespace

std::vector<wire::core::BundleKind> ResolveTemplateKindsForPathPick(const wire::core::CoreView& view,
                                                                    std::uint32_t selected_template_mask,
                                                                    const wire::core::PickResult& pick) {
  const auto selected = SelectedBundleTemplateKinds(view, selected_template_mask);
  if (!selected.empty()) {
    return selected;
  }
  return {ResolveHitSpanTemplateOrDefault(view, pick)};
}

std::string FindMidairBranchBlockedTemplateName(const wire::core::CoreView& view,
                                                const std::vector<wire::core::BundleKind>& template_ids) {
  for (wire::core::BundleKind kind : template_ids) {
    const auto it = view.bundle_templates().find(kind);
    if (it == view.bundle_templates().end()) {
      continue;
    }
    if (!it->second.allow_midair_branch) {
      return it->second.name.empty() ? std::to_string(static_cast<int>(kind)) : it->second.name;
    }
  }
  return {};
}

wire::core::PickResult NormalizeDrawPathPick(const wire::core::CoreView& view, const wire::core::PickResult& pick,
                                             double endpoint_snap_radius_world) {
  if (pick.hit_kind != wire::core::PickHitKind::kSegment || !pick.has_segment_endpoints || endpoint_snap_radius_world <= 0.0) {
    return pick;
  }

  const double endpoint_snap_r2 = endpoint_snap_radius_world * endpoint_snap_radius_world;
  const double t = SegmentTxy(pick.hit_pos_world, pick.segment_endpoint_a_world, pick.segment_endpoint_b_world);
  const bool near_start = t <= 0.2;
  const bool near_end = t >= 0.8;
  if (!near_start && !near_end) {
    return pick;
  }

  const double d2_a = wire::core::DistanceSquaredXY(pick.hit_pos_world, pick.segment_endpoint_a_world);
  const double d2_b = wire::core::DistanceSquaredXY(pick.hit_pos_world, pick.segment_endpoint_b_world);
  if (near_start && d2_a <= endpoint_snap_r2 && d2_a <= d2_b) {
    wire::core::PickResult normalized = pick;
    if (pick.segment_node_a_id != wire::core::kInvalidObjectId) {
      if (const wire::core::Pole* pole = view.edit_state().poles.find(pick.segment_node_a_id); pole != nullptr) {
        normalized.hit_kind = wire::core::PickHitKind::kNode;
        normalized.hit_id = pick.segment_node_a_id;
        normalized.hit_pos_world = pole->world_transform.position;
      } else {
        normalized.hit_pos_world = normalized.segment_endpoint_a_world;
      }
    } else {
      normalized.hit_pos_world = normalized.segment_endpoint_a_world;
    }
    return normalized;
  }
  if (near_end && d2_b <= endpoint_snap_r2 && d2_b <= d2_a) {
    wire::core::PickResult normalized = pick;
    if (pick.segment_node_b_id != wire::core::kInvalidObjectId) {
      if (const wire::core::Pole* pole = view.edit_state().poles.find(pick.segment_node_b_id); pole != nullptr) {
        normalized.hit_kind = wire::core::PickHitKind::kNode;
        normalized.hit_id = pick.segment_node_b_id;
        normalized.hit_pos_world = pole->world_transform.position;
      } else {
        normalized.hit_pos_world = normalized.segment_endpoint_b_world;
      }
    } else {
      normalized.hit_pos_world = normalized.segment_endpoint_b_world;
    }
    return normalized;
  }
  return pick;
}

wire::core::PickResult CanonicalizeDrawPathPick(const wire::core::CoreView& view, const wire::core::PickResult& pick,
                                                const wire::core::Vec3d& ground_hover_world, bool has_ground_hit,
                                                double snap_radius_world) {
  wire::core::PickResult normalized = NormalizeDrawPathPick(view, pick, snap_radius_world);
  if (normalized.hit_kind == wire::core::PickHitKind::kNode) {
    return normalized;
  }

  wire::core::Vec3d anchor_point = normalized.hit_pos_world;
  bool has_anchor_point = (normalized.hit_kind != wire::core::PickHitKind::kEmpty);
  if (!has_anchor_point && has_ground_hit) {
    anchor_point = ground_hover_world;
    has_anchor_point = true;
  }
  if (!has_anchor_point) {
    return normalized;
  }

  const wire::core::PickResult promoted = PromoteGroundHoverToNearbyPolePick(view, anchor_point, snap_radius_world);
  if (promoted.hit_kind != wire::core::PickHitKind::kEmpty) {
    return promoted;
  }
  return normalized;
}

wire::core::PickResult PromoteGroundHoverToNearbyPolePick(const wire::core::CoreView& view,
                                                          const wire::core::Vec3d& ground_hover_world,
                                                          double snap_radius_world) {
  if (snap_radius_world <= 0.0) {
    return {};
  }

  const double snap_radius_squared = snap_radius_world * snap_radius_world;
  const wire::core::Pole* best_pole = nullptr;
  double best_distance_squared = snap_radius_squared + 1.0;
  for (const wire::core::Pole& pole : view.edit_state().poles.items()) {
    const double distance_squared = wire::core::DistanceSquaredXY(pole.world_transform.position, ground_hover_world);
    if (distance_squared > snap_radius_squared) {
      continue;
    }
    if (best_pole == nullptr || distance_squared < best_distance_squared) {
      best_pole = &pole;
      best_distance_squared = distance_squared;
    }
  }

  if (best_pole == nullptr) {
    return {};
  }

  wire::core::PickResult promoted{};
  promoted.hit_kind = wire::core::PickHitKind::kNode;
  promoted.hit_id = best_pole->id;
  promoted.hit_pos_world = best_pole->world_transform.position;
  return promoted;
}
