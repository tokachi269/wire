#pragma once

#include "wire/core/entities.hpp"
#include "wire/core/object_store.hpp"

namespace wire::core {

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
