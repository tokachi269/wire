#pragma once

#include <cstdint>
#include <optional>

#include "city/wire/id.hpp"

namespace city::wire {

enum class LayoutEndpointSourceKind : std::uint8_t {
  kPlainSupport = 0,
  kAttachmentSocket = 1,
  kAttachmentSocketOverride = 2,
  kFallback = 3,
};

enum class EndpointAttachmentRequestKind : std::uint8_t {
  kNone = 0,
  kAttachmentAuto = 1,
  kAttachmentSocket = 2,
  kDanglingSocket = 3,
};

struct EndpointAttachmentRequest {
  EndpointAttachmentRequestKind kind = EndpointAttachmentRequestKind::kNone;
  std::optional<ObjectId> attachment_id{};
  std::optional<int> requested_socket_id{};

  [[nodiscard]] bool has_attachment() const noexcept { return attachment_id.has_value(); }
  [[nodiscard]] bool has_socket_request() const noexcept { return requested_socket_id.has_value(); }
};

} // namespace city::wire
