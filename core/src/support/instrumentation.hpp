#pragma once

#include <cstddef>

namespace wire::core::instrumentation {

struct Counters {
  std::size_t fixture_rule_merge_count = 0;
  std::size_t fixture_plan_build_count = 0;
  std::size_t model_materialization_count = 0;
  std::size_t endpoint_placement_fallback_count = 0;
  std::size_t row_fixture_fallback_count = 0;
  std::size_t affected_span_derive_count = 0;
  std::size_t support_group_rebuild_count = 0;
};

void reset();
Counters snapshot();
void count_fixture_rule_merge();
void count_fixture_plan_build();
void count_model_emit();
void count_endpoint_placement_fallback();
void count_row_fixture_fallback();
void count_affected_span_derive();
void count_group_cache_refresh();

} // namespace wire::core::instrumentation
