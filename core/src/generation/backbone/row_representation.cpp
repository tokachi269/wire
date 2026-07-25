#include "row_representation.hpp"

#include "emit_shared.hpp"
#include "wire/core/coord_utils.hpp"
#include "wire/core/support/numeric_tolerances.hpp"
#include "wire/core/core_view.hpp"

#include <algorithm>
#include <cmath>

namespace wire::core::generation::backbone {
namespace {

constexpr double kSharpCornerInteriorAngleMaxDeg = 74.0;
constexpr double kRadiansToDegrees = 57.2957795130823208768;

Vec3d node_position(const CoreState& state, ObjectId node_id) {
  const SavedBackboneNode* node = state.view().backbone_node(node_id);
  if (node == nullptr) {
    return {};
  }
  if (node->pole_id != kInvalidObjectId) {
    const Pole* pole = state.view().poles().find(node->pole_id);
    if (pole != nullptr) {
      return pole->world_transform.position;
    }
  }
  return node->position;
}

EditResult<Vec3d> directed_edge(const CoreState& state, ObjectId edge_id) {
  EditResult<Vec3d> out{};
  const SavedBackboneEdge* edge = state.view().backbone_edge(edge_id);
  if (edge == nullptr) {
    out.error = "backbone internal: endpoint row edge is missing";
    return out;
  }
  Vec3d direction =
      node_position(state, edge->node_b) - node_position(state, edge->node_a);
  if (!NormalizeXY(&direction)) {
    out.error = "backbone unsupported: endpoint row edge has zero length";
    return out;
  }
  out.value = direction;
  out.ok = true;
  return out;
}

Vec3d away_from_node(const SavedBackboneEdge& edge, ObjectId node_id,
                     const Vec3d& directed) {
  if (edge.node_a == node_id) {
    return directed;
  }
  if (edge.node_b == node_id) {
    return ScaleVec(directed, -1.0);
  }
  return {};
}

} // namespace

bool IsSharpBackboneInteriorAngle(double interior_angle_deg) {
  return interior_angle_deg <= kSharpCornerInteriorAngleMaxDeg + kAngleToleranceDeg;
}

EditResult<EndpointRowRepresentation> DeriveEndpointRowRepresentation(
    const CoreState& state, const SavedBackbonePortBinding& binding) {
  EditResult<EndpointRowRepresentation> out{};
  const SavedBackboneEdgeBundle* edge_bundle =
      state.view().backbone_edge_bundle(binding.edge_bundle_id);
  const SavedBackboneEdge* edge =
      edge_bundle == nullptr ? nullptr
                             : state.view().backbone_edge(edge_bundle->edge_id);
  if (edge_bundle == nullptr || edge == nullptr ||
      binding.row_key.node_id == kInvalidObjectId ||
      binding.row_key.edge_id != edge->edge_id) {
    out.error = "backbone internal: endpoint row identity is invalid";
    return out;
  }
  EditResult<Vec3d> edge_direction = directed_edge(state, edge->edge_id);
  if (!edge_direction.ok) {
    out.error = edge_direction.error;
    return out;
  }

  const SavedBackboneRowContinuity* matched = nullptr;
  bool endpoint_is_a = false;
  for (const SavedBackboneRowContinuity& continuity :
       state.view().backbone().row_continuities) {
    if (continuity.node_id != binding.row_key.node_id) {
      continue;
    }
    const bool is_a =
        continuity.a.edge_bundle_id == binding.edge_bundle_id &&
        continuity.a.lane_index == binding.lane_index;
    const bool is_b =
        continuity.b.edge_bundle_id == binding.edge_bundle_id &&
        continuity.b.lane_index == binding.lane_index;
    if (!is_a && !is_b) {
      continue;
    }
    if (matched != nullptr) {
      out.error =
          "backbone internal: endpoint belongs to multiple row continuities";
      return out;
    }
    matched = &continuity;
    endpoint_is_a = is_a;
  }

  EndpointRowRepresentation representation{};
  representation.row_key.node_id = binding.row_key.node_id;
  representation.row_key.edge_a = edge->edge_id;
  representation.row_axis = ComputeLateralAxis(edge_direction.value);
  if (matched == nullptr) {
    if (!NormalizeXY(&representation.row_axis)) {
      out.error = "backbone unsupported: open endpoint row axis is invalid";
      return out;
    }
    representation.layout_yaw_deg =
        PortLayoutYawDeg(representation.row_axis);
    out.value = representation;
    out.ok = true;
    return out;
  }

  const SavedBackboneRowContinuityEndpoint& peer_endpoint =
      endpoint_is_a ? matched->b : matched->a;
  const SavedBackboneEdgeBundle* peer_edge_bundle =
      state.view().backbone_edge_bundle(peer_endpoint.edge_bundle_id);
  const SavedBackboneEdge* peer_edge =
      peer_edge_bundle == nullptr
          ? nullptr
          : state.view().backbone_edge(peer_edge_bundle->edge_id);
  if (peer_edge_bundle == nullptr || peer_edge == nullptr ||
      peer_edge->edge_id == edge->edge_id) {
    out.error = "backbone internal: endpoint row peer is invalid";
    return out;
  }
  EditResult<Vec3d> peer_direction =
      directed_edge(state, peer_edge->edge_id);
  if (!peer_direction.ok) {
    out.error = peer_direction.error;
    return out;
  }
  Vec3d away =
      away_from_node(*edge, binding.row_key.node_id, edge_direction.value);
  Vec3d peer_away = away_from_node(*peer_edge, binding.row_key.node_id,
                                   peer_direction.value);
  if (!NormalizeXY(&away) || !NormalizeXY(&peer_away)) {
    out.error = "backbone internal: endpoint row peer is not incident";
    return out;
  }
  const double interior_angle_deg =
      std::acos(std::clamp(Dot(away, peer_away), -1.0, 1.0)) *
      kRadiansToDegrees;
  representation.connected = true;
  representation.sharp =
      IsSharpBackboneInteriorAngle(interior_angle_deg);
  representation.peer_edge_bundle_id = peer_endpoint.edge_bundle_id;
  representation.peer_lane_index = peer_endpoint.lane_index;
  if (!representation.sharp) {
    const bool own_is_low = edge->edge_id < peer_edge->edge_id;
    const Vec3d low = own_is_low ? away : peer_away;
    const Vec3d high = own_is_low ? peer_away : away;
    representation.row_axis = ComputeLateralAxis(high - low);
    representation.row_key.edge_a =
        std::min(edge->edge_id, peer_edge->edge_id);
    representation.row_key.edge_b =
        std::max(edge->edge_id, peer_edge->edge_id);
  }
  if (!NormalizeXY(&representation.row_axis)) {
    out.error = "backbone unsupported: derived endpoint row axis is invalid";
    return out;
  }
  representation.layout_yaw_deg =
      PortLayoutYawDeg(representation.row_axis);
  out.value = representation;
  out.ok = true;
  return out;
}

} // namespace wire::core::generation::backbone
