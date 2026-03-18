#include "support_orientation_utils.hpp"

#include "wire/core/coord_utils.hpp"

#include <cmath>

namespace wire::core {
namespace {

double DotHorizontal(const Vec3d& a, const Vec3d& b) {
  return a.x * b.x + a.y * b.y;
}

Vec3d ChordForwardForSupport(const Port& port, const Span& span, const Port& other_port, const Pole& pole,
                             const SupportLayoutEndpoint* layout_endpoint, const EditState& edit_state) {
  const ObjectId endpoint_node_id =
      (port.id == span.port_a_id) ? span.endpoint_node_a_id : span.endpoint_node_b_id;
  const ObjectId peer_node_id =
      (port.id == span.port_a_id) ? span.endpoint_node_b_id : span.endpoint_node_a_id;
  if (endpoint_node_id != kInvalidObjectId && peer_node_id != kInvalidObjectId) {
    if (const Pole* endpoint_pole = edit_state.poles.find(endpoint_node_id); endpoint_pole != nullptr) {
      if (const Pole* peer_pole = edit_state.poles.find(peer_node_id); peer_pole != nullptr) {
        Vec3d forward = peer_pole->world_transform.position - endpoint_pole->world_transform.position;
        forward.z = 0.0;
        if (Normalize(&forward) && IsFiniteXY(forward)) {
          return forward;
        }
      }
    }
  }
  Vec3d forward = (layout_endpoint != nullptr) ? layout_endpoint->departure_dir : Vec3d{};
  forward.z = 0.0;
  if (Normalize(&forward) && IsFiniteXY(forward)) {
    return forward;
  }
  forward = other_port.world_position - port.world_position;
  forward.z = 0.0;
  if (Normalize(&forward) && IsFiniteXY(forward)) {
    return forward;
  }
  if (other_port.owner_pole_id != kInvalidObjectId) {
    if (const Pole* other_pole = edit_state.poles.find(other_port.owner_pole_id); other_pole != nullptr) {
      forward = other_pole->world_transform.position - pole.world_transform.position;
      forward.z = 0.0;
      if (Normalize(&forward) && IsFiniteXY(forward)) {
        return forward;
      }
    }
  }
  forward = {(port.id == span.port_a_id) ? 1.0 : -1.0, 0.0, 0.0};
  return SafeHorizontalNormalized(forward, {1.0, 0.0, 0.0});
}

} // namespace

bool IsFiniteXY(const Vec3d& v) {
  return std::isfinite(v.x) && std::isfinite(v.y);
}

bool UsesGroupedLoweredSupport(const SupportLayoutEndpoint& endpoint, BackboneLoweringKind lowering_kind) {
  if (lowering_kind == BackboneLoweringKind::kNone) {
    return endpoint.branch_down_offset_m > 1e-6 &&
           (endpoint.origin == SupportLayoutOriginKind::kBranchSupport ||
            endpoint.origin == SupportLayoutOriginKind::kPlacementConstraint);
  }
  if (endpoint.decision.support_orientation_basis != SupportOrientationBasisKind::kRadial) {
    return true;
  }
  if (endpoint.origin == SupportLayoutOriginKind::kBranchSupport ||
      endpoint.origin == SupportLayoutOriginKind::kPlacementConstraint) {
    return true;
  }
  return endpoint.decision.continuity_class == ContinuityCategoryClass::kBundleLike &&
         lowering_kind != BackboneLoweringKind::kNone &&
         (endpoint.decision.default_lower_required || !endpoint.decision.same_level_feasible ||
          endpoint.decision.solver_used_same_level_constraint);
}

int SupportGroupIdForEndpoint(ObjectId owner_pole_id, const EndpointContinuityDecision& decision) {
  std::size_t seed = static_cast<std::size_t>(owner_pole_id);
  seed ^= static_cast<std::size_t>(decision.relation_kind) << 8;
  seed ^= static_cast<std::size_t>(decision.continuity_class) << 16;
  seed ^= static_cast<std::size_t>(decision.support_orientation_rule) << 24;
  seed ^= static_cast<std::size_t>(decision.side_assignment_rule) << 32;
  seed ^= static_cast<std::size_t>(decision.used_junction_pair_side_assignment ? 1u : 0u) << 40;
  seed ^= static_cast<std::size_t>(decision.in_through_pair ? 1u : 0u) << 41;
  seed ^= static_cast<std::size_t>(decision.lower_required ? 1u : 0u) << 42;
  return static_cast<int>(seed % 1000000000ull);
}

Vec3d SafeHorizontalNormalized(Vec3d v, const Vec3d& fallback) {
  v.z = 0.0;
  if (Normalize(&v) && IsFiniteXY(v)) {
    return v;
  }
  Vec3d alt = fallback;
  alt.z = 0.0;
  if (Normalize(&alt) && IsFiniteXY(alt)) {
    return alt;
  }
  return {1.0, 0.0, 0.0};
}

Vec3d CanonicalSharedSupportAxis(Vec3d axis, const Vec3d& fallback) {
  axis = SafeHorizontalNormalized(axis, fallback);
  if (axis.x < -1e-9 || (std::abs(axis.x) <= 1e-9 && axis.y < -1e-9)) {
    axis = ScaleVec(axis, -1.0);
  }
  return axis;
}

Vec3d AuthoritativeSupportAxisForEndpoint(const SupportLayoutEndpoint& endpoint, const Vec3d& fallback) {
  Vec3d axis = endpoint.has_side_axis ? endpoint.side_axis : fallback;
  axis = SafeHorizontalNormalized(axis, fallback);
  if (std::abs(endpoint.chosen_side_sign) > 1e-9) {
    axis = ScaleVec(axis, (endpoint.chosen_side_sign >= 0.0) ? 1.0 : -1.0);
  }
  return axis;
}

Vec3d AuthoritativeSupportAxisForGroup(const LoweredSupportGroupPlacement& group, const Vec3d& fallback) {
  Vec3d axis = group.has_side_axis ? group.side_axis : fallback;
  return CanonicalSharedSupportAxis(axis, fallback);
}

Vec3d ResolveSupportAxisForEndpoint(const Port& port, const Span& span, const Port& other_port, const Pole& pole,
                                    const SupportLayoutEndpoint* layout_endpoint, const EditState& edit_state) {
  const double dx = port.world_position.x - pole.world_transform.position.x;
  const double dy = port.world_position.y - pole.world_transform.position.y;
  const double planar = std::sqrt(dx * dx + dy * dy);
  Vec3d pole_radial{
      (planar <= 1e-9) ? 1.0 : (dx / planar),
      (planar <= 1e-9) ? 0.0 : (dy / planar),
      0.0,
  };
  if (!Normalize(&pole_radial) || !IsFiniteXY(pole_radial)) {
    pole_radial = {1.0, 0.0, 0.0};
  }

  if (layout_endpoint != nullptr && layout_endpoint->has_side_axis &&
      layout_endpoint->decision.support_orientation_basis != SupportOrientationBasisKind::kRadial &&
      IsFiniteXY(layout_endpoint->side_axis)) {
    return AuthoritativeSupportAxisForEndpoint(*layout_endpoint, pole_radial);
  }

  Vec3d forward = ChordForwardForSupport(port, span, other_port, pole, layout_endpoint, edit_state);
  Vec3d support_axis = ComputeLateralAxis(forward);
  support_axis = SafeHorizontalNormalized(support_axis, pole_radial);

  if (layout_endpoint != nullptr && layout_endpoint->has_side_axis &&
      std::abs(layout_endpoint->decision.chosen_side_sign) > 1e-9 && IsFiniteXY(layout_endpoint->side_axis)) {
    Vec3d desired = SafeHorizontalNormalized(layout_endpoint->side_axis, support_axis);
    desired = ScaleVec(desired, (layout_endpoint->decision.chosen_side_sign >= 0.0) ? 1.0 : -1.0);
    if (DotHorizontal(support_axis, desired) < 0.0) {
      support_axis = ScaleVec(support_axis, -1.0);
    }
  } else if (DotHorizontal(support_axis, pole_radial) < 0.0) {
    support_axis = ScaleVec(support_axis, -1.0);
  }
  return support_axis;
}

} // namespace wire::core