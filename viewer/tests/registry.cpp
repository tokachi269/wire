#include "registry.hpp"

#include <algorithm>
#include <string>

namespace viewer_test_registry {
namespace {

std::vector<SuiteRegisterFn>& RegisteredSuites() {
  static std::vector<SuiteRegisterFn> suites{};
  return suites;
}

int CaseNumber(const char* case_id) {
  if (case_id == nullptr || case_id[0] != 'V') {
    return -1;
  }
  return std::stoi(std::string(case_id + 1));
}

} // namespace

void AddTest(TestRegistry& tests, const char* case_id, const char* intent, TestFn run) {
  tests.push_back(TestCase{case_id, intent, run});
}

void RegisterSuite(SuiteRegisterFn register_fn) {
  RegisteredSuites().push_back(register_fn);
}

SuiteRegistration::SuiteRegistration(SuiteRegisterFn register_fn) {
  RegisterSuite(register_fn);
}

TestRegistry BuildRegisteredTests() {
  TestRegistry tests{};
  for (SuiteRegisterFn suite : RegisteredSuites()) {
    suite(tests);
  }
  std::sort(tests.begin(), tests.end(), [](const TestCase& a, const TestCase& b) {
    const int a_num = CaseNumber(a.case_id);
    const int b_num = CaseNumber(b.case_id);
    if (a_num != b_num) {
      return a_num < b_num;
    }
    return std::string(a.case_id) < std::string(b.case_id);
  });
  return tests;
}

} // namespace viewer_test_registry
