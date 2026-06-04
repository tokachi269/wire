#pragma once

#include <cstddef>
#include <vector>

#include "wire/core/entities.hpp"
#include "wire/core/object_store.hpp"

namespace wire::core {

struct SavedBackboneNode {
  ObjectId node_id = kInvalidObjectId;
  ObjectId pole_id = kInvalidObjectId;
  Vec3d position{};
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

struct SavedBackboneGraph {
  std::vector<SavedBackboneNode> nodes{};
  std::vector<SavedBackboneEdge> edges{};
  std::vector<SavedBackboneEdgeBundle> edge_bundles{};
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
