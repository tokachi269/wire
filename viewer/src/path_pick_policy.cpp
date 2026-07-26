#include "path_pick_policy.hpp"

#include "city/wire/coord_utils.hpp"

#include <algorithm>

namespace {

bool IsTemplateSelected(std::uint32_t selected_template_mask, city::wire::BundleKind kind) {
  const auto bit = (1u << static_cast<unsigned>(kind));
  return (selected_template_mask & bit) != 0;
}

double SegmentTxy(const city::wire::Vec3d& p, const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
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

std::vector<city::wire::BundleKind> SelectedBundleTemplateKinds(const city::wire::CoreView& view,
                                                                std::uint32_t selected_template_mask) {
  std::vector<city::wire::BundleKind> selected{};
  for (const auto& [kind, _] : view.bundle_templates()) {
    if (IsTemplateSelected(selected_template_mask, kind)) {
      selected.push_back(kind);
    }
  }
  std::sort(selected.begin(), selected.end(), [](auto a, auto b) { return static_cast<int>(a) < static_cast<int>(b); });
  return selected;
}

namespace {

city::wire::BundleKind ResolveHitSpanTemplateOrDefault(const city::wire::CoreView& view,
                                                       const city::wire::PickResult& pick) {
  if (pick.hit_kind == city::wire::PickHitKind::kSegment) {
    if (const city::wire::Span* span = view.edit_state().spans.find(pick.hit_id); span != nullptr) {
      if (const city::wire::Bundle* bundle = view.edit_state().bundles.find(span->bundle_id); bundle != nullptr) {
        return bundle->bundle_template_id;
      }
    }
  }
  return city::wire::BundleKind::kLowVoltage;
}

} // namespace

std::vector<city::wire::BundleKind> ResolveTemplateKindsForPathPick(const city::wire::CoreView& view,
                                                                    std::uint32_t selected_template_mask,
                                                                    const city::wire::PickResult& pick) {
  const auto selected = SelectedBundleTemplateKinds(view, selected_template_mask);
  if (!selected.empty()) {
    return selected;
  }
  return {ResolveHitSpanTemplateOrDefault(view, pick)};
}

std::string FindMidairBranchBlockedTemplateName(const city::wire::CoreView& view,
                                                const std::vector<city::wire::BundleKind>& template_ids) {
  for (city::wire::BundleKind kind : template_ids) {
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

city::wire::PickResult NormalizeDrawPathPick(const city::wire::CoreView& view, const city::wire::PickResult& pick,
                                             double endpoint_snap_radius_world) {
  if (pick.hit_kind != city::wire::PickHitKind::kSegment || !pick.has_segment_endpoints || endpoint_snap_radius_world <= 0.0) {
    return pick;
  }

  const double endpoint_snap_r2 = endpoint_snap_radius_world * endpoint_snap_radius_world;
  const double t = SegmentTxy(pick.hit_pos_world, pick.segment_endpoint_a_world, pick.segment_endpoint_b_world);
  const bool near_start = t <= 0.2;
  const bool near_end = t >= 0.8;
  if (!near_start && !near_end) {
    return pick;
  }

  const double d2_a = city::wire::DistanceSquaredXY(pick.hit_pos_world, pick.segment_endpoint_a_world);
  const double d2_b = city::wire::DistanceSquaredXY(pick.hit_pos_world, pick.segment_endpoint_b_world);
  if (near_start && d2_a <= endpoint_snap_r2 && d2_a <= d2_b) {
    city::wire::PickResult normalized = pick;
    if (pick.segment_node_a_id != city::wire::kInvalidObjectId) {
      if (const city::wire::Pole* pole = view.edit_state().poles.find(pick.segment_node_a_id); pole != nullptr) {
        normalized.hit_kind = city::wire::PickHitKind::kNode;
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
    city::wire::PickResult normalized = pick;
    if (pick.segment_node_b_id != city::wire::kInvalidObjectId) {
      if (const city::wire::Pole* pole = view.edit_state().poles.find(pick.segment_node_b_id); pole != nullptr) {
        normalized.hit_kind = city::wire::PickHitKind::kNode;
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

city::wire::PickResult CanonicalizeDrawPathPick(const city::wire::CoreView& view, const city::wire::PickResult& pick,
                                                const city::wire::Vec3d& ground_hover_world, bool has_ground_hit,
                                                double snap_radius_world) {
  city::wire::PickResult normalized = NormalizeDrawPathPick(view, pick, snap_radius_world);
  if (normalized.hit_kind == city::wire::PickHitKind::kNode) {
    return normalized;
  }

  city::wire::Vec3d anchor_point = normalized.hit_pos_world;
  bool has_anchor_point = (normalized.hit_kind != city::wire::PickHitKind::kEmpty);
  if (!has_anchor_point && has_ground_hit) {
    anchor_point = ground_hover_world;
    has_anchor_point = true;
  }
  if (!has_anchor_point) {
    return normalized;
  }

  const city::wire::PickResult promoted = PromoteGroundHoverToNearbyPolePick(view, anchor_point, snap_radius_world);
  if (promoted.hit_kind != city::wire::PickHitKind::kEmpty) {
    return promoted;
  }
  return normalized;
}

city::wire::PickResult PromoteGroundHoverToNearbyPolePick(const city::wire::CoreView& view,
                                                          const city::wire::Vec3d& ground_hover_world,
                                                          double snap_radius_world) {
  if (snap_radius_world <= 0.0) {
    return {};
  }

  const double snap_radius_squared = snap_radius_world * snap_radius_world;
  const city::wire::Pole* best_pole = nullptr;
  double best_distance_squared = snap_radius_squared + 1.0;
  for (const city::wire::Pole& pole : view.edit_state().poles.items()) {
    const double distance_squared = city::wire::DistanceSquaredXY(pole.world_transform.position, ground_hover_world);
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

  city::wire::PickResult promoted{};
  promoted.hit_kind = city::wire::PickHitKind::kNode;
  promoted.hit_id = best_pole->id;
  promoted.hit_pos_world = best_pole->world_transform.position;
  return promoted;
}
