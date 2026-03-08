#pragma once

#include <string>
#include <vector>

namespace viewer_test_registry {

using TestFn = bool (*)();

struct TestCase {
  const char* case_id = "";
  const char* intent = "";
  TestFn run = nullptr;
};

using TestRegistry = std::vector<TestCase>;
using SuiteRegisterFn = void (*)(TestRegistry&);

void AddTest(TestRegistry& tests, const char* case_id, const char* intent, TestFn run);
void RegisterSuite(SuiteRegisterFn register_fn);
TestRegistry BuildRegisteredTests();

class SuiteRegistration {
public:
  explicit SuiteRegistration(SuiteRegisterFn register_fn);
};

} // namespace viewer_test_registry

#define WIRE_REGISTER_VIEWER_TEST_SUITE(register_fn) \
  namespace { ::viewer_test_registry::SuiteRegistration register_fn##_registration(register_fn); }
