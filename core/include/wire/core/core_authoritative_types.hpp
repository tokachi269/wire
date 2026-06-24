#pragma once

#include <cstddef>
#include <vector>

#include "wire/core/entities.hpp"
#include "wire/core/object_store.hpp"
#include "wire/core/workflow_types.hpp"

namespace wire::core {

struct SavedBackboneNode {
  ObjectId node_id = kInvalidObjectId;
  ObjectId pole_id = kInvalidObjectId;
  SupportKind support_kind = SupportKind::kPole;
  Vec3d position{};
  bool has_source_edge = false;
  ObjectId source_edge_node_a = kInvalidObjectId;
  ObjectId source_edge_node_b = kInvalidObjectId;
  double source_edge_t = 0.0;
  int path_point_index = -1;
  std::vector<SupportNodeBundleMode> bundle_modes{};
};

struct SavedBackboneEdge {
  ObjectId edge_id = kInvalidObjectId;
  ObjectId node_a = kInvalidObjectId;
  ObjectId node_b = kInvalidObjectId;
  std::size_t route = 0;
  std::size_t order = 0;
  Vec3d dir{};
};

struct SavedBackboneEdgeRef {
  ObjectId edge_id = kInvalidObjectId;
  ObjectId node_a = kInvalidObjectId;
  ObjectId node_b = kInvalidObjectId;
  bool created = false;
};

struct SavedBackboneEdgeBundle {
  ObjectId edge_bundle_id = kInvalidObjectId;
  ObjectId edge_id = kInvalidObjectId;
  ObjectId bundle_id = kInvalidObjectId;
  bool edge_forward = true;
  std::size_t route = 0;
  std::size_t order = 0;
  Vec3d dir{};
  std::vector<ObjectId> span_ids{};
};

struct SavedBackboneRowKey {
  ObjectId node_id = kInvalidObjectId;
  bool source_is_open = false;
  ObjectId source_edge_a = kInvalidObjectId;
  ObjectId source_edge_b = kInvalidObjectId;
  bool operator==(const SavedBackboneRowKey& other) const {
    return node_id == other.node_id && source_is_open == other.source_is_open &&
           source_edge_a == other.source_edge_a && source_edge_b == other.source_edge_b;
  }
};

struct SavedBackbonePortBinding {
  ObjectId edge_bundle_id = kInvalidObjectId;
  SavedBackboneRowKey row_key{};
  std::size_t lane_index = 0;
  BundleKind bundle_template_id = BundleKind::kLowVoltage;
  PortKind port_kind = PortKind::kGeneric;
  PortLayer port_layer = PortLayer::kUnknown;
  ObjectId port_id = kInvalidObjectId;
};

struct SavedBackboneSpanBinding {
  ObjectId edge_bundle_id = kInvalidObjectId;
  std::size_t lane_index = 0;
  ObjectId span_id = kInvalidObjectId;
};

struct SavedBackboneGraph {
  std::vector<SavedBackboneNode> nodes{};
  std::vector<SavedBackboneEdge> edges{};
  std::vector<SavedBackboneEdgeBundle> edge_bundles{};
  std::vector<SavedBackbonePortBinding> port_bindings{};
  std::vector<SavedBackboneSpanBinding> span_bindings{};
};

struct EditState {
  // Entity-layer authoritative stores.
  ObjectStore<Pole> poles;
  ObjectStore<Port> ports;
  ObjectStore<Anchor> anchors;
  ObjectStore<Bundle> bundles;
  ObjectStore<Span> spans;
  ObjectStore<Attachment> attachments;
};

} // namespace wire::core
