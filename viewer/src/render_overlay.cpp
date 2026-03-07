#include "render_overlay.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "ui_common.hpp"

constexpr float kAxisLength = 2.0f;

static wire::core::Vec3d Lerp(const wire::core::Vec3d& a, const wire::core::Vec3d& b, double t) {
  return wire::core::Vec3d{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
}

static wire::core::Vec3d RotateXDeg(const wire::core::Vec3d& v, double deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double rad = deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {v.x, v.y * c - v.z * s, v.y * s + v.z * c};
}

static wire::core::Vec3d RotateYDeg(const wire::core::Vec3d& v, double deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double rad = deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {v.x * c + v.z * s, v.y, -v.x * s + v.z * c};
}

static wire::core::Vec3d RotateZDeg(const wire::core::Vec3d& v, double deg) {
  constexpr double kPi = 3.14159265358979323846;
  const double rad = deg * (kPi / 180.0);
  const double c = std::cos(rad);
  const double s = std::sin(rad);
  return {v.x * c - v.y * s, v.x * s + v.y * c, v.z};
}

static wire::core::Vec3d RotateEulerXYZDeg(const wire::core::Vec3d& v, const wire::core::Vec3d& euler_deg) {
  return RotateZDeg(RotateYDeg(RotateXDeg(v, euler_deg.x), euler_deg.y), euler_deg.z);
}

static wire::core::Vec3d PoleTopPoint(const wire::core::Pole& pole) {
  const wire::core::Vec3d local_up{0.0, 0.0, pole.height_m};
  return pole.world_transform.position + RotateEulerXYZDeg(local_up, pole.world_transform.rotation_euler_deg);
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

void DrawAxes() {
  DrawLine3D(ToRaylib({0.0, 0.0, 0.0}), ToRaylib({kAxisLength, 0.0, 0.0}), RED);
  DrawLine3D(ToRaylib({0.0, 0.0, 0.0}), ToRaylib({0.0, kAxisLength, 0.0}), GREEN);
  DrawLine3D(ToRaylib({0.0, 0.0, 0.0}), ToRaylib({0.0, 0.0, kAxisLength}), BLUE);
}

void DrawPickHighlight(const CoreState& state, const wire::core::PickResult& pick, bool has_resolution,
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

void DrawBackboneOverlay(const wire::core::BackboneResult& backbone, const ViewerUiState& ui_state) {
  std::unordered_map<ObjectId, wire::core::Vec3d> node_position_by_id{};
  node_position_by_id.reserve(backbone.nodes.size());
  for (const wire::core::SupportNode& node : backbone.nodes) {
    node_position_by_id[node.node_id] = node.position;
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
    DrawSphere(ToRaylib(node.position), radius, color);
    DrawSphereWires(ToRaylib(node.position), radius + 0.05f, 9, 14, Color{255, 255, 255, 180});
  }
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
    if (curve != nullptr && curve->points.size() >= 2) {
      for (std::size_t i = 0; i + 1 < curve->points.size(); ++i) {
        DrawLine3D(ToRaylib(curve->points[i]), ToRaylib(curve->points[i + 1]), color);
      }
    } else {
      DrawLine3D(ToRaylib(start_port->world_position), ToRaylib(end_port->world_position), color);
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
    DrawBackboneOverlay(backbone, ui_state);
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
    pos.z += attachment.offset_m;

    Color color = MAGENTA;
    if (ui_state.selected_type == SelectedType::kAttachment && ui_state.selected_id == attachment.id) {
      color = GOLD;
    }
    DrawCubeV(ToRaylib(pos), Vector3{0.14f, 0.14f, 0.14f}, color);
  }

  if (ui_state.mode == EditMode::kBranch && ui_state.branch_pick_enabled) {
    DrawPickHighlight(state, ui_state.branch_hover_pick, ui_state.branch_hover_has_resolution,
                      ui_state.branch_hover_resolution);
  }
  if (ui_state.mode == EditMode::kDrawPath && ui_state.draw_pick_enabled) {
    DrawPickHighlight(state, ui_state.draw_hover_pick, ui_state.draw_hover_has_resolution, ui_state.draw_hover_resolution);
  }
}



