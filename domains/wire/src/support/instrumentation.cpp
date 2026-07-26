#include "instrumentation.hpp"

namespace city::wire::instrumentation {
namespace {

#if defined(WIRE_TEST_INSTRUMENTATION)
Counters counters{};
#endif

void increment(std::size_t Counters::*field) {
#if defined(WIRE_TEST_INSTRUMENTATION)
  counters.*field += 1;
#else
  static_cast<void>(field);
#endif
}

} // namespace

void reset() {
#if defined(WIRE_TEST_INSTRUMENTATION)
  counters = {};
#endif
}

Counters snapshot() {
#if defined(WIRE_TEST_INSTRUMENTATION)
  return counters;
#else
  return {};
#endif
}

void count_fixture_rule_merge() { increment(&Counters::fixture_rule_merge_count); }
void count_fixture_plan_build() { increment(&Counters::fixture_plan_build_count); }
void count_model_emit() { increment(&Counters::model_materialization_count); }
void count_endpoint_placement_fallback() { increment(&Counters::endpoint_placement_fallback_count); }
void count_row_fixture_fallback() { increment(&Counters::row_fixture_fallback_count); }
void count_affected_span_derive() { increment(&Counters::affected_span_derive_count); }
void count_group_cache_refresh() { increment(&Counters::support_group_rebuild_count); }
void count_pair_build() { increment(&Counters::pair_build_count); }
void count_stable_row_slot_lookup() { increment(&Counters::stable_row_slot_lookup_count); }

void set_fixture_rule_count(std::size_t count) {
#if defined(WIRE_TEST_INSTRUMENTATION)
  counters.last_fixture_rule_count = count;
#else
  static_cast<void>(count);
#endif
}

} // namespace city::wire::instrumentation
