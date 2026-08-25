#pragma once

#include "../registry.hpp"

#include "city/wire/core_state.hpp"

#include <cstddef>
#include <limits>
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

struct ObservationToken {
  std::size_t index = std::numeric_limits<std::size_t>::max();
};

bool Observe(Operation operation, Entry entry, const city::wire::CoreState& state,
             city::wire::ObjectId pole_id, city::wire::ObjectId reference_edge_bundle_id,
             std::size_t lane_index, std::string* error, ObservationToken* token = nullptr);
bool ObserveEmpty(Operation operation, Entry entry, const city::wire::CoreState& state,
                  std::string* error, ObservationToken* token = nullptr);
bool ObserveMidspan(Operation operation, Entry entry, city::wire::CoreState& state,
                    city::wire::ObjectId source_edge_id, std::string* error,
                    ObservationToken* token = nullptr);
bool RecordObservationEvidence(ObservationToken token, test_registry::AssertionKind kind,
                               std::string* error);
bool ValidateRecordedObservationEvidence(std::string* error);
bool ValidateRuntimeCoverage(std::string* error);
void ResetRuntimeCoverage();

} // namespace backbone_tests::semantics_coverage

#define WIRE_TEST_EXPECT_BOS_KIND(token, kind, condition, message)            \
  do {                                                                        \
    WIRE_TEST_EXPECT_KIND(kind, condition, message);                          \
    std::string wire_semantics_evidence_error;                                \
    WIRE_TEST_EXPECT(                                                         \
        ::backbone_tests::semantics_coverage::RecordObservationEvidence(      \
            token, kind, &wire_semantics_evidence_error),                     \
        wire_semantics_evidence_error);                                       \
  } while (false)

#define WIRE_TEST_EXPECT_BOS_ORACLE(token, condition, message) \
  WIRE_TEST_EXPECT_BOS_KIND(token, ::test_registry::AssertionKind::kOracle, condition, message)

#define WIRE_TEST_EXPECT_BOS_ANCHOR(token, condition, message) \
  WIRE_TEST_EXPECT_BOS_KIND(token, ::test_registry::AssertionKind::kAnchor, condition, message)

#define WIRE_TEST_EXPECT_BOS_PRESENCE(token, condition, message) \
  WIRE_TEST_EXPECT_BOS_KIND(token, ::test_registry::AssertionKind::kPresence, condition, message)

#define WIRE_TEST_EXPECT_BOS_DIFFERENTIAL(token, condition, message) \
  WIRE_TEST_EXPECT_BOS_KIND(token, ::test_registry::AssertionKind::kDifferential, condition, message)

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
