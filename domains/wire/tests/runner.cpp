#include <iostream>
#include <string>
#include <string_view>

#include "backbone/semantics_coverage.hpp"
#include "registry.hpp"

int main(int argc, char** argv) {
  const test_registry::TestRegistry tests = test_registry::BuildRegisteredTests();
  const std::string_view filter = (argc >= 2 && argv[1] != nullptr) ? std::string_view(argv[1]) : std::string_view{};

  std::string ledger_error;
  if (!test_registry::ValidateSpecLedger(tests, &ledger_error)) {
    std::cerr << "spec ledger mismatch: " << ledger_error << "\n";
    return 1;
  }

  bool all_passed = true;
  backbone_tests::semantics_coverage::ResetRuntimeCoverage();
  for (const test_registry::TestCase& test : tests) {
    if (!filter.empty() && std::string_view(test.case_id).find(filter) == std::string_view::npos) {
      continue;
    }
    test_registry::ClearFailureReason();
    test_registry::BeginTestCase(test.case_id, test.family);
    const bool passed = test.run();
    const char* family = test.family == test_registry::TestFamily::kSourceGuard ? "SourceGuard" : "Behavior";
    std::cout << (passed ? "[PASS] " : "[FAIL] ") << test.case_id << " [" << family << "][" << test.oracle << "]["
              << (test.abnormal ? "Abnormal" : "Normal") << "]"
              << " - " << test.intent << "\n";
    if (!passed && !test_registry::FailureReason().empty()) {
      std::cerr << "  reason: " << test_registry::FailureReason() << "\n";
    }
    all_passed = all_passed && passed;
    test_registry::EndTestCase();
  }

  if (filter.empty() && all_passed) {
    std::string coverage_error;
    if (!backbone_tests::semantics_coverage::ValidateRuntimeCoverage(&coverage_error)) {
      std::cerr << "backbone semantics coverage failed: " << coverage_error << "\n";
      all_passed = false;
    }
  }
  if (filter.empty()) {
    const std::vector<std::string> derived_only =
        test_registry::TestCaseIdsWithOnlyDerivedEquality();
    std::cout << "derived-equality-only cases (" << derived_only.size() << "):";
    for (const std::string& case_id : derived_only) {
      std::cout << " " << case_id;
    }
    std::cout << "\n";
  }
  if (!all_passed) {
    std::cerr << "core tests failed\n";
    return 1;
  }

  std::cout << "core tests passed (" << tests.size() << " cases)\n";
  return 0;
}
