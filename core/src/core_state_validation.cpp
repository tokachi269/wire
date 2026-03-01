#include "wire/core/core_state.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace wire::core {

namespace {

template <typename TKey>
std::unordered_map<TKey, std::vector<ObjectId>>
canonical_index_map(const std::unordered_map<TKey, std::vector<ObjectId>>& map) {
  auto out = map;
  for (auto& [_, ids] : out) {
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  }
  return out;
}

} // namespace

bool ValidationResult::has_errors() const {
  for (const ValidationIssue& issue : issues) {
    if (issue.severity == ValidationSeverity::kError) {
      return true;
    }
  }
  return false;
}

ValidationResult CoreState::Validate() const {
  ValidationResult result;
  const CoreView core = view();
  const EditState& edit_state = core.edit_state();
  const LayoutSettings& layout_settings = core.layout_settings();
  const CacheState& cache_state = core.cache_state();
  const ConnectionIndex& connection_index = core.connection_index();
  const auto& span_runtime_states = core.span_runtime_states();
  const auto& pole_types = core.pole_types();
  const auto& slot_debug_records = core.slot_selection_debug_records();

  for (const Pole& pole : edit_state.poles.items()) {
    if (pole.pole_type_id != kInvalidPoleTypeId && !pole_types.contains(pole.pole_type_id)) {
      result.issues.push_back(
          {ValidationSeverity::kError, "PoleTypeMissing", "Pole references unknown PoleType", pole.id});
    }
    if (!std::isfinite(pole.context.corner_angle_deg) || !std::isfinite(pole.context.corner_turn_sign) ||
        !std::isfinite(pole.context.side_scale) || !std::isfinite(pole.context.sharp_theta_deg) ||
        !std::isfinite(pole.context.sharp_bisector_dir.x) || !std::isfinite(pole.context.sharp_bisector_dir.y) ||
        !std::isfinite(pole.context.sharp_bisector_dir.z) || !std::isfinite(pole.context.sharp_side_dir.x) ||
        !std::isfinite(pole.context.sharp_side_dir.y) || !std::isfinite(pole.context.sharp_side_dir.z)) {
      result.issues.push_back(
          {ValidationSeverity::kError, "PoleContextInvalid", "Pole context has non-finite value", pole.id});
    }
    if (pole.context.corner_turn_sign < -1.0 - 1e-9 || pole.context.corner_turn_sign > 1.0 + 1e-9) {
      result.issues.push_back({
          ValidationSeverity::kWarning,
          "PoleTurnSignOutOfRange",
          "Pole corner_turn_sign is out of range",
          pole.id,
      });
    }
    if (pole.context.side_scale < layout_settings.min_side_scale - 1e-9 ||
        pole.context.side_scale > layout_settings.max_side_scale + 1e-9) {
      result.issues.push_back({ValidationSeverity::kWarning, "PoleSideScaleOutOfRange",
                               "Pole side_scale is out of configured range", pole.id});
    }
    if (!std::isfinite(pole.orientation_control.manual_yaw_deg)) {
      result.issues.push_back(
          {ValidationSeverity::kError, "PoleOrientationInvalid", "Pole manual yaw is non-finite", pole.id});
    }
    if (pole.placement_mode == PlacementMode::kManual && !pole.user_edited) {
      result.issues.push_back({
          ValidationSeverity::kWarning,
          "PoleManualWithoutUserEdited",
          "Manual pole should have user_edited=true",
          pole.id,
      });
    }
  }

  for (const Port& port : edit_state.ports.items()) {
    if (port.owner_pole_id != kInvalidObjectId && edit_state.poles.find(port.owner_pole_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "PortOwnerMissing", "Port owner pole is missing", port.id});
    }
    if (port.source_slot_id >= 0 && port.owner_pole_id != kInvalidObjectId) {
      const Pole* owner = edit_state.poles.find(port.owner_pole_id);
      const PoleTypeDefinition* pole_type = nullptr;
      if (owner != nullptr) {
        auto it = pole_types.find(owner->pole_type_id);
        if (it != pole_types.end()) {
          pole_type = &it->second;
        }
      }
      bool slot_found = false;
      if (pole_type != nullptr) {
        for (const PortSlotTemplate& slot : pole_type->port_slots) {
          if (slot.slot_id != port.source_slot_id) {
            continue;
          }
          slot_found = true;
          if (slot.category != port.category) {
            result.issues.push_back({
                ValidationSeverity::kError,
                "PortSlotCategoryMismatch",
                "Port category differs from slot category",
                port.id,
            });
          }
          if (!is_valid_slot_side(slot.side) || !is_valid_slot_role(slot.role)) {
            result.issues.push_back({
                ValidationSeverity::kError,
                "PortSlotAttributeInvalid",
                "Slot side/role contains invalid value",
                port.id,
            });
          }
          break;
        }
      }
      if (!slot_found) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "PortSlotMissing",
            "Port source slot id is not defined on owner PoleType",
            port.id,
        });
      }
    }
    if (!std::isfinite(port.world_position.x) || !std::isfinite(port.world_position.y) ||
        !std::isfinite(port.world_position.z) || !std::isfinite(port.side_scale_applied)) {
      result.issues.push_back(
          {ValidationSeverity::kError, "PortTransformInvalid", "Port position or side_scale is non-finite", port.id});
    }
    if (port.side_scale_applied < layout_settings.min_side_scale - 1e-9 ||
        port.side_scale_applied > layout_settings.max_side_scale + 1e-9) {
      result.issues.push_back({ValidationSeverity::kWarning, "PortSideScaleOutOfRange",
                               "Port side_scale_applied is out of range", port.id});
    }
    if (port.position_mode == PortPositionMode::kManual) {
      if (!port.user_edited_position) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "PortManualWithoutUserEdited",
            "Manual port should have user_edited_position=true",
            port.id,
        });
      }
      if (port.placement_source != PortPlacementSourceKind::kManualEdit) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "PortManualSourceMismatch",
            "Manual port should have placement_source=ManualEdit",
            port.id,
        });
      }
    } else {
      if (port.user_edited_position) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "PortAutoWithUserEdited",
            "Auto port should not keep user_edited_position=true",
            port.id,
        });
      }
    }
  }

  for (const Anchor& anchor : edit_state.anchors.items()) {
    if (anchor.owner_pole_id != kInvalidObjectId && edit_state.poles.find(anchor.owner_pole_id) == nullptr) {
      result.issues.push_back(
          {ValidationSeverity::kError, "AnchorOwnerMissing", "Anchor owner pole is missing", anchor.id});
    }
  }

  std::unordered_map<ObjectId, std::unordered_map<int, ObjectId>> lane_index_map{};
  for (const WireLane& lane : edit_state.wire_lanes.items()) {
    if (edit_state.wire_groups.find(lane.wire_group_id) == nullptr) {
      result.issues.push_back(
          {ValidationSeverity::kError, "WireLaneGroupMissing", "WireLane owner group is missing", lane.id});
    }
    if (lane.lane_index < 0) {
      result.issues.push_back(
          {ValidationSeverity::kError, "WireLaneIndexInvalid", "WireLane lane_index must be >= 0", lane.id});
      continue;
    }
    auto& per_group = lane_index_map[lane.wire_group_id];
    auto [it, inserted] = per_group.emplace(lane.lane_index, lane.id);
    if (!inserted) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "WireLaneIndexDuplicate",
          "WireLane lane_index duplicates inside one wire_group",
          lane.id,
      });
    }
  }

  for (const Span& span : edit_state.spans.items()) {
    const Port* port_a = edit_state.ports.find(span.port_a_id);
    const Port* port_b = edit_state.ports.find(span.port_b_id);
    if (port_a == nullptr || port_b == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanPortMissing", "Span references missing port", span.id});
      continue;
    }
    if (span.port_a_id == span.port_b_id) {
      result.issues.push_back({ValidationSeverity::kError, "SpanSelfReference", "Span has same endpoint", span.id});
    }
    if (has_zero_length(*port_a, *port_b)) {
      result.issues.push_back({ValidationSeverity::kWarning, "SpanZeroLength", "Span endpoints overlap", span.id});
    }
    if (span.bundle_id != kInvalidObjectId && edit_state.bundles.find(span.bundle_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanBundleMissing", "Span bundle is missing", span.id});
    }
    if (span.wire_group_id != kInvalidObjectId && edit_state.wire_groups.find(span.wire_group_id) == nullptr) {
      result.issues.push_back(
          {ValidationSeverity::kError, "SpanWireGroupMissing", "Span wire_group is missing", span.id});
    }
    if (span.wire_group_id != kInvalidObjectId && span.wire_lane_id == kInvalidObjectId) {
      result.issues.push_back({
          ValidationSeverity::kWarning,
          "SpanWireLaneUnset",
          "Span wire_group is set but wire_lane is not set",
          span.id,
      });
    }
    if (span.wire_lane_id != kInvalidObjectId) {
      if (span.wire_group_id == kInvalidObjectId) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "SpanWireGroupUnset",
            "Span wire_lane is set but wire_group is not set",
            span.id,
        });
      }
      const WireLane* lane = edit_state.wire_lanes.find(span.wire_lane_id);
      if (lane == nullptr) {
        result.issues.push_back(
            {ValidationSeverity::kError, "SpanWireLaneMissing", "Span wire_lane is missing", span.id});
      } else if (span.wire_group_id != kInvalidObjectId && lane->wire_group_id != span.wire_group_id) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "SpanWireLaneGroupMismatch",
            "Span wire_lane belongs to different wire_group",
            span.id,
        });
      }
    }
    if (span.anchor_a_id != kInvalidObjectId && edit_state.anchors.find(span.anchor_a_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanAnchorMissing", "Span anchorA is missing", span.id});
    }
    if (span.anchor_b_id != kInvalidObjectId && edit_state.anchors.find(span.anchor_b_id) == nullptr) {
      result.issues.push_back({ValidationSeverity::kError, "SpanAnchorMissing", "Span anchorB is missing", span.id});
    }
  }

  const auto expected_port_index = canonical_index_map(make_expected_port_index(edit_state));
  const auto expected_anchor_index = canonical_index_map(make_expected_anchor_index(edit_state));
  const auto actual_port_index = canonical_index_map(connection_index.spans_by_port);
  const auto actual_anchor_index = canonical_index_map(connection_index.spans_by_anchor);

  if (expected_port_index != actual_port_index) {
    result.issues.push_back(
        {ValidationSeverity::kError, "PortIndexMismatch", "Port->Span index mismatch", kInvalidObjectId});
  }
  if (expected_anchor_index != actual_anchor_index) {
    result.issues.push_back(
        {ValidationSeverity::kError, "AnchorIndexMismatch", "Anchor->Span index mismatch", kInvalidObjectId});
  }
  for (const auto& [port_id, span_ids] : connection_index.spans_by_port) {
    if (edit_state.ports.find(port_id) == nullptr) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "PortIndexDanglingPort",
          "Port index references removed port",
          port_id,
      });
      continue;
    }
    for (ObjectId span_id : span_ids) {
      if (edit_state.spans.find(span_id) == nullptr) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "PortIndexDanglingSpan",
            "Port index references removed span",
            port_id,
        });
        break;
      }
    }
  }
  for (const auto& [anchor_id, span_ids] : connection_index.spans_by_anchor) {
    if (edit_state.anchors.find(anchor_id) == nullptr) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "AnchorIndexDanglingAnchor",
          "Anchor index references removed anchor",
          anchor_id,
      });
      continue;
    }
    for (ObjectId span_id : span_ids) {
      if (edit_state.spans.find(span_id) == nullptr) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "AnchorIndexDanglingSpan",
            "Anchor index references removed span",
            anchor_id,
        });
        break;
      }
    }
  }

  for (const Span& span : edit_state.spans.items()) {
    auto it = span_runtime_states.find(span.id);
    if (it == span_runtime_states.end()) {
      result.issues.push_back(
          {ValidationSeverity::kError, "SpanRuntimeMissing", "Span runtime state missing", span.id});
      continue;
    }
    if (it->second.span_id != span.id) {
      result.issues.push_back({ValidationSeverity::kError, "SpanRuntimeCorrupt", "Span runtime id mismatch", span.id});
    }
  }
  for (const auto& [span_id, runtime] : span_runtime_states) {
    if (edit_state.spans.find(span_id) == nullptr || runtime.span_id != span_id) {
      result.issues.push_back(
          {ValidationSeverity::kError, "SpanRuntimeDangling", "Runtime state points to removed span", span_id});
    }
  }

  for (const Span& span : edit_state.spans.items()) {
    const auto runtime_it = span_runtime_states.find(span.id);
    if (runtime_it == span_runtime_states.end()) {
      continue;
    }
    const SpanRuntimeState& runtime = runtime_it->second;

    auto curve_it = cache_state.curve_cache.by_span.find(span.id);
    if (curve_it != cache_state.curve_cache.by_span.end()) {
      const CurveCacheEntry& curve = curve_it->second;
      if (curve.points.size() < 2) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "CurveSampleCountInvalid",
            "Curve cache has less than 2 points",
            span.id,
        });
      }
      if (curve.source_version != runtime.geometry_version) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "GeometryVersionMismatch",
            "Curve cache sourceVersion does not match geometryVersion",
            span.id,
        });
      }
    }

    auto bounds_it = cache_state.bounds_cache.by_span.find(span.id);
    if (bounds_it != cache_state.bounds_cache.by_span.end()) {
      const BoundsCacheEntry& bounds = bounds_it->second;
      if (bounds.whole.min.x > bounds.whole.max.x || bounds.whole.min.y > bounds.whole.max.y ||
          bounds.whole.min.z > bounds.whole.max.z) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "BoundsInvalid",
            "Whole bounds has min > max",
            span.id,
        });
      }
      for (const AABBd& segment : bounds.segments) {
        if (segment.min.x > segment.max.x || segment.min.y > segment.max.y || segment.min.z > segment.max.z) {
          result.issues.push_back({
              ValidationSeverity::kError,
              "SegmentBoundsInvalid",
              "Segment bounds has min > max",
              span.id,
          });
          break;
        }
      }
      if (bounds.source_version != runtime.bounds_version) {
        result.issues.push_back({
            ValidationSeverity::kWarning,
            "BoundsVersionMismatch",
            "Bounds cache sourceVersion does not match boundsVersion",
            span.id,
        });
      }
    }
  }

  for (const SlotSelectionDebugRecord& debug : slot_debug_records) {
    if (!std::isfinite(debug.corner_turn_sign)) {
      result.issues.push_back({
          ValidationSeverity::kError,
          "SlotSelectionDebugInvalid",
          "Slot selection debug corner_turn_sign is non-finite",
          debug.pole_id,
      });
    }
    if (debug.selected_slot_id >= 0) {
      bool found = false;
      for (const SlotCandidateDebug& c : debug.candidates) {
        if (c.slot_id == debug.selected_slot_id) {
          found = true;
          if (c.total_score != c.category_score + c.context_score + c.layer_score + c.side_score + c.role_score +
                                   c.priority_score + c.usage_score + c.congestion_score + c.tie_breaker) {
            result.issues.push_back({
                ValidationSeverity::kWarning,
                "SlotSelectionScoreInconsistent",
                "Slot candidate score breakdown does not match total_score",
                debug.pole_id,
            });
          }
          break;
        }
      }
      if (!found) {
        result.issues.push_back({
            ValidationSeverity::kError,
            "SlotSelectionDebugMismatch",
            "Selected slot id does not exist in candidate list",
            debug.pole_id,
        });
      }
    }
  }

  return result;
}

} // namespace wire::core
