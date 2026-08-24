#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace test_registry {

using TestFn = bool (*)();

enum class TestFamily : std::uint8_t {
  kBehavior = 0,
  kSourceGuard = 1,
};

enum class AssertionKind : std::uint8_t {
  kOracle = 0,
  kAnchor = 1,
  kPresence = 2,
  kDifferential = 3,
  kDerivedEquality = 4,
};

struct TestCase {
  const char* case_id = "";
  const char* intent = "";
  const char* oracle = "";
  bool abnormal = false;
  TestFamily family = TestFamily::kBehavior;
  TestFn run = nullptr;
};

using TestRegistry = std::vector<TestCase>;
using SuiteRegisterFn = void (*)(TestRegistry&);

void AddTest(TestRegistry& tests, const char* case_id, const char* intent, const char* oracle, bool abnormal,
             TestFn run);
void AddSourceGuardTest(TestRegistry& tests, const char* case_id, const char* intent, const char* oracle,
                        bool abnormal, TestFn run);
void RegisterSuite(SuiteRegisterFn register_fn);
TestRegistry BuildRegisteredTests();
bool ValidateSpecLedger(const TestRegistry& tests, std::string* error);
void BeginTestCase(const char* case_id, TestFamily family);
void EndTestCase();
void RecordAssertion(AssertionKind kind);
bool CurrentTestHasIndependentAssertion();
const std::string& CurrentTestCaseId();
TestFamily CurrentTestFamily();
bool TestCaseHasIndependentAssertion(const std::string& case_id);
std::vector<std::string> TestCaseIdsWithOnlyDerivedEquality();
const std::vector<AssertionKind>& CurrentTestAssertions();
void ClearFailureReason();
void SetFailureReason(std::string reason);
const std::string& FailureReason();

class SuiteRegistration {
public:
  explicit SuiteRegistration(SuiteRegisterFn register_fn);
};

} // namespace test_registry

#define WIRE_TEST_EXPECT(condition, message)          \
  do {                                                \
    if (!(condition)) {                               \
      if (::test_registry::FailureReason().empty()) { \
        ::test_registry::SetFailureReason(message);   \
      }                                               \
      return false;                                   \
    }                                                 \
  } while (false)

#define WIRE_TEST_EXPECT_KIND(kind, condition, message) \
  do {                                                 \
    ::test_registry::RecordAssertion(kind);            \
    WIRE_TEST_EXPECT(condition, message);               \
  } while (false)

#define WIRE_TEST_EXPECT_ORACLE(condition, message) \
  WIRE_TEST_EXPECT_KIND(::test_registry::AssertionKind::kOracle, condition, message)

#define WIRE_TEST_EXPECT_ANCHOR(condition, message) \
  WIRE_TEST_EXPECT_KIND(::test_registry::AssertionKind::kAnchor, condition, message)

#define WIRE_TEST_EXPECT_PRESENCE(condition, message) \
  WIRE_TEST_EXPECT_KIND(::test_registry::AssertionKind::kPresence, condition, message)

#define WIRE_TEST_EXPECT_DIFFERENTIAL(condition, message) \
  WIRE_TEST_EXPECT_KIND(::test_registry::AssertionKind::kDifferential, condition, message)

#define WIRE_TEST_EXPECT_DERIVED_EQUALITY(condition, message) \
  WIRE_TEST_EXPECT_KIND(::test_registry::AssertionKind::kDerivedEquality, condition, message)
#define WIRE_REGISTER_TEST_SUITE(register_fn) \
  namespace { ::test_registry::SuiteRegistration register_fn##_registration(register_fn); }
