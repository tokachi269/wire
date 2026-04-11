#include "wire/core/core_state.hpp"
#include "detail_utils.hpp"

#include <algorithm>
#include <unordered_set>
#include <vector>

namespace wire::core {

using namespace generation::detail;

EditResult<GenerateBundleFromPathResult>
CoreState::RegenerateSessionAutoParts(std::uint64_t generation_session_id, const BackboneSpec& request) {
  EditResult<GenerateBundleFromPathResult> result;
  if (generation_session_id == 0) {
    result.error = "generation_session_id must be non-zero";
    return result;
  }

  const CoreState snapshot = *this;
  ChangeSet cleanup_changes{};

  std::vector<ObjectId> target_span_ids{};
  target_span_ids.reserve(edit_state_access().spans.size());
  std::unordered_set<ObjectId> candidate_bundle_ids{};
  std::unordered_set<ObjectId> candidate_generated_port_ids{};
  for (const Span& span : edit_state_access().spans.items()) {
    if (span.generation.generation_session_id != generation_session_id) {
      continue;
    }
    target_span_ids.push_back(span.id);
    candidate_generated_port_ids.insert(span.port_a_id);
    candidate_generated_port_ids.insert(span.port_b_id);
    if (span.bundle_id != kInvalidObjectId) {
      candidate_bundle_ids.insert(span.bundle_id);
    }
  }

  for (ObjectId span_id : target_span_ids) {
    const Span* span_ptr = edit_state_access().spans.find(span_id);
    if (span_ptr == nullptr) {
      continue;
    }
    const Span span = *span_ptr;
    remove_span_from_indexes(span);
    edit_state_access().spans.remove(span_id);
    span_runtime_states_access().erase(span_id);
    remove_span_from_caches(span_id);
    add_unique_id(cleanup_changes.deleted_ids, span_id);

    std::vector<ObjectId> remove_attachments{};
    for (const Attachment& attachment : edit_state_access().attachments.items()) {
      if (attachment.span_id == span_id) {
        remove_attachments.push_back(attachment.id);
      }
    }
    for (ObjectId attachment_id : remove_attachments) {
      edit_state_access().attachments.remove(attachment_id);
      add_unique_id(cleanup_changes.deleted_ids, attachment_id);
    }
  }

  auto erase_removed_spans_from_queue = [&](std::vector<ObjectId>& queue) {
    queue.erase(std::remove_if(queue.begin(), queue.end(),
                               [&](ObjectId id) {
                                 return std::find(target_span_ids.begin(), target_span_ids.end(), id) !=
                                        target_span_ids.end();
                               }),
                queue.end());
  };
  erase_removed_spans_from_queue(dirty_queue_access().topology_dirty_span_ids);
  erase_removed_spans_from_queue(dirty_queue_access().decision_dirty_span_ids);
  erase_removed_spans_from_queue(dirty_queue_access().geometry_dirty_span_ids);
  erase_removed_spans_from_queue(dirty_queue_access().bounds_dirty_span_ids);
  erase_removed_spans_from_queue(dirty_queue_access().render_dirty_span_ids);
  erase_removed_spans_from_queue(dirty_queue_access().raycast_dirty_span_ids);

  for (ObjectId port_id : candidate_generated_port_ids) {
    const Port* port_ptr = edit_state_access().ports.find(port_id);
    if (port_ptr == nullptr) {
      continue;
    }
    const Port port = *port_ptr;
    if (port.position_mode == PortPositionMode::kManual || !port.generated_by_rule || port.generated_from_template) {
      continue;
    }
    const auto it_live_spans = connection_index_access().spans_by_port.find(port_id);
    if (it_live_spans != connection_index_access().spans_by_port.end() && !it_live_spans->second.empty()) {
      continue;
    }
    connection_index_access().spans_by_port.erase(port_id);
    if (edit_state_access().ports.remove(port_id)) {
      if (port.owner_pole_id != kInvalidObjectId) {
        index_remove(relation_index_access().ports_by_pole, port.owner_pole_id, port_id);
      }
      add_unique_id(cleanup_changes.deleted_ids, port_id);
    }
  }

  std::vector<ObjectId> target_auto_pole_ids{};
  target_auto_pole_ids.reserve(edit_state_access().poles.size());
  for (const Pole& pole : edit_state_access().poles.items()) {
    if (pole.generation.generation_session_id != generation_session_id) {
      continue;
    }
    if (pole.placement_mode == PlacementMode::kAuto) {
      target_auto_pole_ids.push_back(pole.id);
    }
  }

  for (ObjectId pole_id : target_auto_pole_ids) {
    const Pole* pole = edit_state_access().poles.find(pole_id);
    if (pole == nullptr) {
      continue;
    }
    std::vector<ObjectId> owned_ports{};
    std::vector<ObjectId> owned_anchors{};
    if (const auto it = relation_index_access().ports_by_pole.find(pole_id);
        it != relation_index_access().ports_by_pole.end()) {
      owned_ports = it->second;
    }
    if (const auto it = relation_index_access().anchors_by_pole.find(pole_id);
        it != relation_index_access().anchors_by_pole.end()) {
      owned_anchors = it->second;
    }

    bool has_live_connections = false;
    bool has_manual_port = false;
    for (ObjectId port_id : owned_ports) {
      const Port* owned_port = edit_state_access().ports.find(port_id);
      if (owned_port != nullptr && owned_port->position_mode == PortPositionMode::kManual) {
        has_manual_port = true;
      }
      auto it = connection_index_access().spans_by_port.find(port_id);
      if (it != connection_index_access().spans_by_port.end() && !it->second.empty()) {
        has_live_connections = true;
        break;
      }
    }
    if (has_live_connections || has_manual_port) {
      continue;
    }

    for (ObjectId port_id : owned_ports) {
      connection_index_access().spans_by_port.erase(port_id);
      const Port* port_ptr = edit_state_access().ports.find(port_id);
      const Port port_copy = (port_ptr == nullptr) ? Port{} : *port_ptr;
      if (edit_state_access().ports.remove(port_id)) {
        if (port_copy.owner_pole_id != kInvalidObjectId) {
          index_remove(relation_index_access().ports_by_pole, port_copy.owner_pole_id, port_id);
        }
        add_unique_id(cleanup_changes.deleted_ids, port_id);
      }
    }
    for (ObjectId anchor_id : owned_anchors) {
      connection_index_access().spans_by_anchor.erase(anchor_id);
      const Anchor* anchor_ptr = edit_state_access().anchors.find(anchor_id);
      const Anchor anchor_copy = (anchor_ptr == nullptr) ? Anchor{} : *anchor_ptr;
      if (edit_state_access().anchors.remove(anchor_id)) {
        if (anchor_copy.owner_pole_id != kInvalidObjectId) {
          index_remove(relation_index_access().anchors_by_pole, anchor_copy.owner_pole_id, anchor_id);
        }
        add_unique_id(cleanup_changes.deleted_ids, anchor_id);
      }
    }
    if (edit_state_access().poles.remove(pole_id)) {
      add_unique_id(cleanup_changes.deleted_ids, pole_id);
    }
  }

  for (ObjectId bundle_id : candidate_bundle_ids) {
    bool used = false;
    for (const Span& span : edit_state_access().spans.items()) {
      if (span.bundle_id == bundle_id) {
        used = true;
        break;
      }
    }
    if (!used && edit_state_access().bundles.remove(bundle_id)) {
      relation_index_access().spans_by_bundle.erase(bundle_id);
      add_unique_id(cleanup_changes.deleted_ids, bundle_id);
    }
  }

  BackboneSpec regen_request = request;
  regen_request.pole_placement.restrict_reuse_to_session = true;
  regen_request.pole_placement.reuse_session_id = generation_session_id;
  EditResult<GenerateBundleFromPathResult> regenerated = GenerateFromBackboneSpec(regen_request);
  if (!regenerated.ok) {
    *this = snapshot;
    result.error = regenerated.error;
    return result;
  }

  for (ObjectId pole_id : regenerated.value.generated_pole_ids) {
    Pole* pole = edit_state_access().poles.find(pole_id);
    if (pole != nullptr) {
      pole->generation.generation_session_id = generation_session_id;
      add_unique_id(regenerated.change_set.updated_ids, pole_id);
    }
  }
  for (ObjectId span_id : regenerated.value.generated_span_ids) {
    Span* span = edit_state_access().spans.find(span_id);
    if (span != nullptr) {
      span->generation.generation_session_id = generation_session_id;
      add_unique_id(regenerated.change_set.updated_ids, span_id);
    }
  }
  append_change_set(regenerated.change_set, cleanup_changes);
  return regenerated;
}

} // namespace wire::core

