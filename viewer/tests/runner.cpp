#include <iostream>

#include "registry.hpp"

int main() {
  const viewer_test_registry::TestRegistry tests = viewer_test_registry::BuildRegisteredTests();
  bool all_passed = true;
  for (const viewer_test_registry::TestCase& test : tests) {
    const bool passed = test.run();
    std::cout << (passed ? "[PASS] " : "[FAIL] ") << test.case_id << " - " << test.intent << "\n";
    all_passed = all_passed && passed;
  }
  if (!all_passed) {
    std::cerr << "viewer tests failed\n";
    return 1;
  }
  std::cout << "viewer tests passed (" << tests.size() << " cases)\n";
  return 0;
}
