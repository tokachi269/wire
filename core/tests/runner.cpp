#include <iostream>
#include <string>
#include <string_view>

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
  for (const test_registry::TestCase& test : tests) {
    if (!filter.empty() && std::string_view(test.case_id).find(filter) == std::string_view::npos) {
      continue;
    }
    test_registry::ClearFailureReason();
    const bool passed = test.run();
    std::cout << (passed ? "[PASS] " : "[FAIL] ") << test.case_id << " [" << test.oracle << "]["
              << (test.abnormal ? "Abnormal" : "Normal") << "]"
              << " - " << test.intent << "\n";
    if (!passed && !test_registry::FailureReason().empty()) {
      std::cerr << "  reason: " << test_registry::FailureReason() << "\n";
    }
    all_passed = all_passed && passed;
  }

  if (!all_passed) {
    std::cerr << "core tests failed\n";
    return 1;
  }

  std::cout << "core tests passed (" << tests.size() << " cases)\n";
  return 0;
}
