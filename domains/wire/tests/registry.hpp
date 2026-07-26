#pragma once

#include <string>
#include <vector>

namespace test_registry {

using TestFn = bool (*)();

struct TestCase {
  const char* case_id = "";
  const char* intent = "";
  const char* oracle = "";
  bool abnormal = false;
  TestFn run = nullptr;
};

using TestRegistry = std::vector<TestCase>;
using SuiteRegisterFn = void (*)(TestRegistry&);

void AddTest(TestRegistry& tests, const char* case_id, const char* intent, const char* oracle, bool abnormal,
             TestFn run);
void RegisterSuite(SuiteRegisterFn register_fn);
TestRegistry BuildRegisteredTests();
bool ValidateSpecLedger(const TestRegistry& tests, std::string* error);
void ClearFailureReason();
void SetFailureReason(std::string reason);
const std::string& FailureReason();

class SuiteRegistration {
public:
  explicit SuiteRegistration(SuiteRegisterFn register_fn);
};

} // namespace test_registry

#define WIRE_TEST_EXPECT(condition, message)       \
  do {                                            \
    if (!(condition)) {                           \
      ::test_registry::SetFailureReason(message); \
      return false;                               \
    }                                             \
  } while (false)

#define WIRE_REGISTER_TEST_SUITE(register_fn) \
  namespace { ::test_registry::SuiteRegistration register_fn##_registration(register_fn); }
