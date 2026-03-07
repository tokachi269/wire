#include <iostream>
#include <string>

#include "registry.hpp"

int main() {
  const test_registry::TestRegistry tests = test_registry::BuildRegisteredTests();

  std::string ledger_error;
  if (!test_registry::ValidateSpecLedger(tests, &ledger_error)) {
    std::cerr << "spec ledger mismatch: " << ledger_error << "\n";
    return 1;
  }

  bool all_passed = true;
  for (const test_registry::TestCase& test : tests) {
    const bool passed = test.run();
    std::cout << (passed ? "[PASS] " : "[FAIL] ") << test.case_id << " [" << test.oracle << "]["
              << (test.abnormal ? "Abnormal" : "Normal") << "]"
              << " - " << test.intent << "\n";
    all_passed = all_passed && passed;
  }

  if (!all_passed) {
    std::cerr << "core tests failed\n";
    return 1;
  }

  std::cout << "core tests passed (" << tests.size() << " cases)\n";
  return 0;
}
