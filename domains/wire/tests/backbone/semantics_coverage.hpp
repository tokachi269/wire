#pragma once

#include "../registry.hpp"

#include "city/wire/core_state.hpp"

#include <cstddef>
#include <string>

namespace backbone_tests::semantics_coverage {

enum class Operation {
  kAddOneEdge,
  kAddTwoEdges,
  kMovePoleAngle,
  kUpdatePlacement,
  kSaveLoad,
  kRegenerate,
};

enum class Entry {
  kCoreApi,
  kWasmAdapter,
  kViewerAction,
};

bool Observe(Operation operation, Entry entry, const city::wire::CoreState& state,
             city::wire::ObjectId pole_id, city::wire::ObjectId reference_edge_bundle_id,
             std::size_t lane_index, std::string* error);
bool ObserveEmpty(Operation operation, Entry entry, const city::wire::CoreState& state, std::string* error);
bool ObserveMidspan(Operation operation, Entry entry, city::wire::CoreState& state,
                    city::wire::ObjectId source_edge_id, std::string* error);
bool ValidateRuntimeCoverage(std::string* error);
void ResetRuntimeCoverage();

} // namespace backbone_tests::semantics_coverage

#define WIRE_TEST_OBSERVE_BOS(operation, entry, state, pole_id, edge_bundle_id, lane_index) \
  do {                                                                                    \
    std::string wire_semantics_error;                                                     \
    WIRE_TEST_EXPECT_ANCHOR(                                                              \
        ::backbone_tests::semantics_coverage::Observe(                                    \
            operation, entry, state, pole_id, edge_bundle_id, lane_index,                 \
            &wire_semantics_error),                                                       \
        wire_semantics_error);                                                            \
  } while (false)

#define WIRE_TEST_OBSERVE_BOS_EMPTY(operation, entry, state)                     \
  do {                                                                    \
    std::string wire_semantics_error;                                     \
    WIRE_TEST_EXPECT_ANCHOR(                                              \
        ::backbone_tests::semantics_coverage::ObserveEmpty(               \
            operation, entry, state, &wire_semantics_error),                     \
        wire_semantics_error);                                            \
  } while (false)

#define WIRE_TEST_OBSERVE_BOS_MIDSPAN(operation, entry, state, source_edge_id) \
  do {                                                                         \
    std::string wire_semantics_error;                                          \
    WIRE_TEST_EXPECT_ANCHOR(                                                   \
        ::backbone_tests::semantics_coverage::ObserveMidspan(                  \
            operation, entry, state, source_edge_id, &wire_semantics_error),   \
        wire_semantics_error);                                                 \
  } while (false)
