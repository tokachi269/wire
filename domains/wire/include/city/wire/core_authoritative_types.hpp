#pragma once

#include <cstddef>
#include <vector>

#include "city/wire/entities.hpp"
#include "city/wire/object_store.hpp"
#include "city/wire/workflow_types.hpp"

namespace city::wire {

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
  double lateral_offset_m = 0.0;
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
};

struct SavedBackboneRowKey {
  ObjectId node_id = kInvalidObjectId;
  ObjectId edge_id = kInvalidObjectId;
  bool operator==(const SavedBackboneRowKey& other) const {
    return node_id == other.node_id && edge_id == other.edge_id;
  }
};

struct SavedBackbonePortBinding {
  ObjectId edge_bundle_id = kInvalidObjectId;
  SavedBackboneRowKey row_key{};
  std::size_t lane_index = 0;
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  PortKind port_kind = PortKind::kGeneric;
  PortLayer port_layer = PortLayer::kUnknown;
  int placement_band_id = 0;
  int support_level = 0;
  int support_group_id = -1;
  double layout_yaw_deg = 0.0;
  ObjectId port_id = kInvalidObjectId;
};

struct SavedBackboneSpanBinding {
  ObjectId edge_bundle_id = kInvalidObjectId;
  std::size_t lane_index = 0;
  ObjectId span_id = kInvalidObjectId;
};

struct SavedBackboneRowContinuityEndpoint {
  ObjectId edge_bundle_id = kInvalidObjectId;
  std::size_t lane_index = 0;
};

struct SavedBackboneRowContinuity {
  ObjectId node_id = kInvalidObjectId;
  SavedBackboneRowContinuityEndpoint a{};
  SavedBackboneRowContinuityEndpoint b{};
};

struct SavedBackboneGraph {
  std::vector<SavedBackboneNode> nodes{};
  std::vector<SavedBackboneEdge> edges{};
  std::vector<SavedBackboneEdgeBundle> edge_bundles{};
  std::vector<SavedBackbonePortBinding> port_bindings{};
  std::vector<SavedBackboneSpanBinding> span_bindings{};
  std::vector<SavedBackboneRowContinuity> row_continuities{};
};

// Explicit-Apply descriptor for one concrete Bundle variation scope.
// Concrete Bundle/Port/Span/SavedBackboneGraph state remains the current
// physical authority. Variation membership retains exact graph edge identity
// and lane continuity required after count reaches 0 without copying graph
// nodes, edge geometry, or a representative Bundle placement.
struct SavedBackboneBundleVariationInstance {
  std::uint64_t placement_key = 0;
  ObjectId bundle_id = kInvalidObjectId;
};

struct SavedBackboneBundleVariationContinuity {
  ObjectId node_id = kInvalidObjectId;
  ObjectId edge_a = kInvalidObjectId;
  std::size_t lane_a = 0;
  ObjectId edge_b = kInvalidObjectId;
  std::size_t lane_b = 0;
};

struct SavedBackboneBundleVariationMembership {
  BundleTemplateId bundle_template_id = kInvalidBundleTemplateId;
  int conductor_count = 0;
  std::vector<ObjectId> edge_ids{};
  std::vector<SavedBackboneBundleVariationContinuity> row_continuities{};
};

struct SavedBackboneBundleVariation {
  ObjectId variation_id = kInvalidObjectId;
  RouteBundleVariationInput descriptor{};
  std::vector<SavedBackboneBundleVariationInstance> instances{};
  std::vector<SavedBackboneBundleVariationMembership> memberships{};
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

} // namespace city::wire
