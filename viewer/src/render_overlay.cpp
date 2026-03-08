#include "render_overlay.hpp"
#include "backbone_plane.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "host_coords.hpp"
#include "ui_common.hpp"
#include "wire/core/coord_utils.hpp"

namespace {

constexpr float kAxisLength = 2.0f;
static wire::core::Vec3d Lerp(const wire::core::Vec3d& a, const wire::core::Vec3d& b, double t) {
  return wire::core::Vec3d{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}

static wire::core::Vec3d PoleTopPoint(const wire::core::Pole& pole) {
  const wire::core::PoleFrame frame =
      wire::core::BuildPoleFrame(pole.world_transform, pole.world_transform.rotation_euler_deg.z);
  return wire::core::LocalPointToWorld(frame, wire::core::ScaleVec(wire::core::WorldUp(), pole.height_m));
}

static Color VisualPartColor(wire::core::VisualPartKind kind) {
  switch (kind) {
  case wire::core::VisualPartKind::kSupportArm:
    return BROWN;
  case wire::core::VisualPartKind::kInsulator:
    return SKYBLUE;
  case wire::core::VisualPartKind::kFitting:
    return LIGHTGRAY;
  default:
    return GRAY;
  }
}

bool IsHostPointInFrontOfCamera(const Camera3D& camera, const Vector3& host_point) {
  const Vector3 to_point{
      host_point.x - camera.position.x,
      host_point.y - camera.position.y,
      host_point.z - camera.position.z,
  };
  const Vector3 forward{
      camera.target.x - camera.position.x,
      camera.target.y - camera.position.y,
      camera.target.z - camera.position.z,
  };
  const float dot = to_point.x * forward.x + to_point.y * forward.y + to_point.z * forward.z;
  return dot > 0.0f;
}

bool IsProjectedPointOnScreen(const Vector2& projected, float margin) {
  return projected.x >= -margin && projected.y >= -margin &&
         projected.x <= static_cast<float>(GetScreenWidth()) + margin &&
         projected.y <= static_cast<float>(GetScreenHeight()) + margin;
}

std::vector<wire::core::Vec3d> AabbCorners(const wire::core::AABBd& box) {
  return {
      {box.min.x, box.min.y, box.min.z},
      {box.min.x, box.min.y, box.max.z},
      {box.min.x, box.max.y, box.min.z},
      {box.min.x, box.max.y, box.max.z},
      {box.max.x, box.min.y, box.min.z},
      {box.max.x, box.min.y, box.max.z},
      {box.max.x, box.max.y, box.min.z},
      {box.max.x, box.max.y, box.max.z},
  };
}

bool IsBoundsVisibleApprox(const Camera3D& camera, const wire::core::AABBd& bounds) {
  constexpr float kScreenMarginPx = 64.0f;
  for (const wire::core::Vec3d& corner : AabbCorners(bounds)) {
    const Vector3 host_point = InternalToHostWorld(corner);
    if (!IsHostPointInFrontOfCamera(camera, host_point)) {
      continue;
    }
    const Vector2 projected = GetWorldToScreen(host_point, camera);
    if (IsProjectedPointOnScreen(projected, kScreenMarginPx)) {
      return true;
    }
  }
  return false;
}

Color DirtyColorForSpan(const wire::core::SpanRuntimeState* runtime_state) {
  if (runtime_state == nullptr) {
    return GRAY;
  }
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kTopology))
    return PURPLE;
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kGeometry))
    return YELLOW;
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kBounds))
    return BLUE;
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kRender))
    return RED;
  if (wire::core::any(runtime_state->dirty_bits, wire::core::DirtyBits::kRaycast))
    return GREEN;
  return SKYBLUE;
}

void DrawAxesImpl() {
  DrawLine3D(ToRaylib({0.0, 0.0, 0.0}), ToRaylib({kAxisLength, 0.0, 0.0}), RED);
  DrawLine3D(ToRaylib({0.0, 0.0, 0.0}), ToRaylib({0.0, kAxisLength, 0.0}), GREEN);
  DrawLine3D(ToRaylib({0.0, 0.0, 0.0}), ToRaylib({0.0, 0.0, kAxisLength}), BLUE);
}

void DrawPickHighlightImpl(const CoreState& state, const wire::core::PickResult& pick, bool has_resolution,
                           const wire::core::CoreState::ResolveBranchPickResult& resolution) {
  if (pick.hit_kind == wire::core::PickHitKind::kEmpty) {
    return;
  }
  const auto& edit = state.view().edit_state();
  if (pick.hit_kind == wire::core::PickHitKind::kNode || pick.hit_kind == wire::core::PickHitKind::kBuilding) {
    DrawSphere(ToRaylib(pick.hit_pos_world), 0.16f, Color{255, 238, 120, 210});
    DrawSphereWires(ToRaylib(pick.hit_pos_world), 0.22f, 10, 14, Color{255, 255, 170, 255});
  } else if (pick.hit_kind == wire::core::PickHitKind::kSegment) {
    bool has_segment = false;
    wire::core::Vec3d a{};
    wire::core::Vec3d b{};
    if (const wire::core::Span* span = edit.spans.find(pick.hit_id); span != nullptr) {
      const wire::core::Port* pa = edit.ports.find(span->port_a_id);
      const wire::core::Port* pb = edit.ports.find(span->port_b_id);
      if (pa != nullptr && pb != nullptr) {
        a = pa->world_position;
        b = pb->world_position;
        has_segment = true;
      }
    }
    if (!has_segment && pick.has_segment_endpoints) {
      a = pick.segment_endpoint_a_world;
      b = pick.segment_endpoint_b_world;
      has_segment = true;
    }
    if (has_segment) {
      DrawLine3D(ToRaylib(a), ToRaylib(b), Color{80, 255, 180, 255});
      DrawSphere(ToRaylib(a), 0.08f, Color{90, 255, 200, 220});
      DrawSphere(ToRaylib(b), 0.08f, Color{90, 255, 200, 220});
    }
    DrawSphere(ToRaylib(pick.hit_pos_world), 0.09f, Color{80, 255, 180, 230});
  }
  if (has_resolution) {
    const bool is_midair = (resolution.resolution == wire::core::CoreState::PickBranchResolutionKind::kMidair);
    const Color resolved_color = is_midair ? Color{80, 220, 255, 235} : Color{255, 220, 90, 235};
    DrawSphere(ToRaylib(resolution.position), 0.15f, resolved_color);
    DrawSphereWires(ToRaylib(resolution.position), 0.20f, 10, 16, Color{255, 255, 255, 220});
  }
}

void DrawBackboneOverlayImpl(const wire::core::BackboneResult& backbone, const ViewerUiState& ui_state) {
  std::unordered_map<ObjectId, wire::core::Vec3d> node_position_by_id{};
  node_position_by_id.reserve(backbone.nodes.size());
  for (const wire::core::SupportNode& node : backbone.nodes) {
    node_position_by_id[node.node_id] = ProjectBackbonePointToDisplayPlane(node.position);
  }

  for (const wire::core::BackboneEdge& edge : backbone.edges) {
    const auto it_a = node_position_by_id.find(edge.node_a);
    const auto it_b = node_position_by_id.find(edge.node_b);
    if (it_a == node_position_by_id.end() || it_b == node_position_by_id.end()) {
      continue;
    }
    DrawLine3D(ToRaylib(it_a->second), ToRaylib(it_b->second), Color{255, 216, 110, 185});
  }

  for (const wire::core::SupportNode& node : backbone.nodes) {
    const bool in_draw_path = std::find(ui_state.draw_path_point_node_ids.begin(), ui_state.draw_path_point_node_ids.end(),
                                        node.node_id) != ui_state.draw_path_point_node_ids.end();
    Color color = Color{255, 212, 92, 210};
    float radius = 0.12f;
    if (node.support_kind == wire::core::SupportKind::kMidair) {
      color = Color{80, 220, 255, 220};
      radius = 0.14f;
    } else if (node.support_kind == wire::core::SupportKind::kBuilding) {
      color = Color{140, 255, 120, 220};
      radius = 0.14f;
    }
    if (in_draw_path) {
      color = Color{255, 250, 180, 235};
      radius += 0.03f;
    }
    const wire::core::Vec3d display = ProjectBackbonePointToDisplayPlane(node.position);
    DrawSphere(ToRaylib(display), radius, color);
    DrawSphereWires(ToRaylib(display), radius + 0.05f, 9, 14, Color{255, 255, 255, 180});
  }
}

static Color ColorFromRgba(std::uint32_t rgba) {
  return Color{
      static_cast<unsigned char>((rgba >> 24) & 0xFFu),
      static_cast<unsigned char>((rgba >> 16) & 0xFFu),
      static_cast<unsigned char>((rgba >> 8) & 0xFFu),
      static_cast<unsigned char>(rgba & 0xFFu),
  };
}
} // namespace

void UpdatePreferredVisibleSpans(const CoreState& state, const Camera3D& camera, ViewerUiState& ui_state) {
  ui_state.preferred_visible_span_ids.clear();
  for (const wire::core::Span& span : state.view().edit_state().spans.items()) {
    const wire::core::BoundsCacheEntry* bounds = state.find_bounds_cache(span.id);
    if (bounds == nullptr) {
      continue;
    }
    if (!IsBoundsVisibleApprox(camera, bounds->whole)) {
      continue;
    }
    ui_state.preferred_visible_span_ids.push_back(span.id);
  }
  ui_state.preferred_visible_span_count = static_cast<int>(ui_state.preferred_visible_span_ids.size());
}

void DrawAxes() { DrawAxesImpl(); }

void DrawPickHighlight(const CoreState& state, const wire::core::PickResult& pick, bool has_resolution,
                       const wire::core::CoreState::ResolveBranchPickResult& resolution) {
  DrawPickHighlightImpl(state, pick, has_resolution, resolution);
}

void DrawBackboneOverlay(const wire::core::BackboneResult& backbone, const ViewerUiState& ui_state) {
  DrawBackboneOverlayImpl(backbone, ui_state);
}

void DrawCore(const CoreState& state, const ViewerUiState& ui_state) {
  const auto view = state.view();
  const auto& edit = view.edit_state();
  const wire::core::BackboneResult backbone = state.BuildBackboneResult();
  ObjectId selected_bundle_id = wire::core::kInvalidObjectId;
  if (ui_state.selected_type == SelectedType::kSpan && ui_state.selected_id != wire::core::kInvalidObjectId) {
    const wire::core::Span* selected_span = edit.spans.find(ui_state.selected_id);
    if (selected_span != nullptr) {
      selected_bundle_id = selected_span->bundle_id;
    }
  }

  for (const wire::core::Pole& pole : edit.poles.items()) {
    const wire::core::Vec3d pole_base_ue = pole.world_transform.position;
    const wire::core::Vec3d pole_top_ue = PoleTopPoint(pole);
    Color color = DARKGRAY;
    if (ui_state.selected_type == SelectedType::kPole && ui_state.selected_id == pole.id) {
      color = GOLD;
    }
    DrawCylinderWiresEx(ToRaylib(pole_base_ue), ToRaylib(pole_top_ue), 0.12f, 0.08f, 10, color);
  }

  const bool show_backbone_overlay =
      ui_state.draw_show_backbone_overlay && (ui_state.mode == EditMode::kDrawPath || ui_state.mode == EditMode::kBranch);
  if (!show_backbone_overlay) {
    for (const wire::core::SupportNode& node : backbone.nodes) {
      if (node.support_kind == wire::core::SupportKind::kPole) {
        continue;
      }
      const Color color = (node.support_kind == wire::core::SupportKind::kMidair) ? Color{80, 220, 255, 230}
                                                                                   : Color{140, 255, 120, 230};
      DrawSphere(ToRaylib(node.position), 0.11f, color);
    }
  }

  for (const wire::core::Port& port : edit.ports.items()) {
    Color color = (port.position_mode == wire::core::PortPositionMode::kManual) ? MAGENTA : ORANGE;
    if (ui_state.selected_type == SelectedType::kPort && ui_state.selected_id == port.id) {
      color = GOLD;
    }
    DrawSphere(ToRaylib(port.world_position), 0.09f, color);
  }

  for (const wire::core::Anchor& anchor : edit.anchors.items()) {
    Color color = PURPLE;
    if (ui_state.selected_type == SelectedType::kAnchor && ui_state.selected_id == anchor.id) {
      color = GOLD;
    }
    DrawSphere(ToRaylib(anchor.world_position), 0.08f, color);
  }

  for (const wire::core::Span& span : edit.spans.items()) {
    const wire::core::Port* start_port = edit.ports.find(span.port_a_id);
    const wire::core::Port* end_port = edit.ports.find(span.port_b_id);
    if (start_port == nullptr || end_port == nullptr) {
      continue;
    }
    const wire::core::SpanRuntimeState* runtime_state = state.view().find_span_runtime_state(span.id);
    Color color = DirtyColorForSpan(runtime_state);
    if (ui_state.selected_type == SelectedType::kSpan && ui_state.selected_id == span.id) {
      color = GOLD;
    } else if (ui_state.show_selected_bundle_highlight && selected_bundle_id != wire::core::kInvalidObjectId &&
               span.bundle_id == selected_bundle_id) {
      color = ORANGE;
    }

    const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span.id);
    const wire::core::SpanRenderCacheEntry* render = state.view().find_span_render_cache(span.id);
    const float wire_radius =
        static_cast<float>((render == nullptr) ? 0.01 : std::max(0.0005, render->wire_radius_m));
    const Color wire_color = (render == nullptr) ? color : ColorFromRgba(render->color_rgba);
    if (curve != nullptr && curve->points.size() >= 2) {
      for (std::size_t i = 0; i + 1 < curve->points.size(); ++i) {
        DrawCylinderEx(ToRaylib(curve->points[i]), ToRaylib(curve->points[i + 1]), wire_radius, wire_radius, 8,
                       wire_color);
      }
    } else {
      DrawCylinderEx(ToRaylib(start_port->world_position), ToRaylib(end_port->world_position), wire_radius, wire_radius,
                     8, wire_color);
    }

    const wire::core::BoundsCacheEntry* bounds = state.find_bounds_cache(span.id);
    if (bounds != nullptr) {
      if (ui_state.show_whole_aabb) {
        DrawBoundingBox(ToRaylibBounds(bounds->whole), DARKGREEN);
      }
      if (ui_state.show_segment_aabb) {
        for (const wire::core::AABBd& segment : bounds->segments) {
          DrawBoundingBox(ToRaylibBounds(segment), Color{70, 130, 180, 180});
        }
      }
    }

    const wire::core::SpanVisualCacheEntry* visual = state.view().find_span_visual_cache(span.id);
    if (visual != nullptr) {
      for (const wire::core::VisualPart& part : visual->parts) {
        const Color part_color = VisualPartColor(part.kind);
        if (part.kind == wire::core::VisualPartKind::kInsulator) {
          DrawCylinderEx(ToRaylib(part.a), ToRaylib(part.b), static_cast<float>(part.radius_m),
                         static_cast<float>(part.radius_m), 8, part_color);
        } else {
          DrawLine3D(ToRaylib(part.a), ToRaylib(part.b), part_color);
        }
      }
    }
  }

  if (show_backbone_overlay) {
    DrawBackboneOverlayImpl(backbone, ui_state);
  }

  for (const wire::core::Attachment& attachment : edit.attachments.items()) {
    const wire::core::Span* span = edit.spans.find(attachment.span_id);
    if (span == nullptr) {
      continue;
    }
    const wire::core::Port* port_a = edit.ports.find(span->port_a_id);
    const wire::core::Port* port_b = edit.ports.find(span->port_b_id);
    if (port_a == nullptr || port_b == nullptr) {
      continue;
    }
    wire::core::Vec3d pos = Lerp(port_a->world_position, port_b->world_position, attachment.t);
    wire::core::OffsetAlongWorldUp(&pos, attachment.offset_m);

    Color color = MAGENTA;
    if (ui_state.selected_type == SelectedType::kAttachment && ui_state.selected_id == attachment.id) {
      color = GOLD;
    }
    DrawCubeV(ToRaylib(pos), Vector3{0.14f, 0.14f, 0.14f}, color);
  }

  if (ui_state.mode == EditMode::kBranch && ui_state.branch_pick_enabled) {
    DrawPickHighlightImpl(state, ui_state.branch_hover_pick, ui_state.branch_hover_has_resolution,
                          ui_state.branch_hover_resolution);
  }
  if (ui_state.mode == EditMode::kDrawPath && ui_state.draw_pick_enabled) {
    DrawPickHighlightImpl(state, ui_state.draw_hover_pick, ui_state.draw_hover_has_resolution,
                          ui_state.draw_hover_resolution);
  }
}

