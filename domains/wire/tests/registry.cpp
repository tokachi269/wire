#include "registry.hpp"

#include <algorithm>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#ifndef WIRE_TEST_SPEC_LEDGER_PATH
#define WIRE_TEST_SPEC_LEDGER_PATH "domains/wire/tests/spec_ledger.md"
#endif

namespace test_registry {
namespace {

std::vector<SuiteRegisterFn>& RegisteredSuites() {
  static std::vector<SuiteRegisterFn> suites;
  return suites;
}

std::string& CurrentFailureReason() {
  static std::string reason;
  return reason;
}

std::string& ActiveCaseId() {
  static std::string case_id;
  return case_id;
}

TestFamily& ActiveFamily() {
  static TestFamily family = TestFamily::kBehavior;
  return family;
}

std::unordered_map<std::string, bool>& CompletedEvidence() {
  static std::unordered_map<std::string, bool> evidence;
  return evidence;
}
std::vector<AssertionKind>& CurrentAssertions() {
  static std::vector<AssertionKind> assertions;
  return assertions;
}

int CaseNumber(const char* case_id) {
  if (case_id == nullptr) {
    return -1;
  }
  std::cmatch match;
  static const std::regex kCasePrefix(R"((C)(\d+))");
  if (!std::regex_search(case_id, match, kCasePrefix) || match.size() < 3) {
    return -1;
  }
  return std::stoi(match[2].str());
}

std::string CasePrefix(const char* case_id) {
  if (case_id == nullptr) {
    return {};
  }
  std::cmatch match;
  static const std::regex kCasePrefix(R"((C\d+))");
  if (!std::regex_search(case_id, match, kCasePrefix) || match.size() < 2) {
    return {};
  }
  return match[1].str();
}

template <typename T>
std::string JoinSorted(const T& values) {
  std::vector<std::string> sorted(values.begin(), values.end());
  std::sort(sorted.begin(), sorted.end(), [](const std::string& a, const std::string& b) {
    const int a_num = CaseNumber(a.c_str());
    const int b_num = CaseNumber(b.c_str());
    if (a_num != b_num) {
      return a_num < b_num;
    }
    return a < b;
  });
  std::ostringstream out;
  for (std::size_t i = 0; i < sorted.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << sorted[i];
  }
  return out.str();
}

} // namespace

void AddTest(TestRegistry& tests, const char* case_id, const char* intent, const char* oracle, bool abnormal,
             TestFn run) {
  tests.push_back(TestCase{case_id, intent, oracle, abnormal, TestFamily::kBehavior, run});
}

void AddSourceGuardTest(TestRegistry& tests, const char* case_id, const char* intent,
                        const char* oracle, bool abnormal, TestFn run) {
  tests.push_back(TestCase{case_id, intent, oracle, abnormal, TestFamily::kSourceGuard, run});
}
void RegisterSuite(SuiteRegisterFn register_fn) {
  RegisteredSuites().push_back(register_fn);
}

SuiteRegistration::SuiteRegistration(SuiteRegisterFn register_fn) {
  RegisterSuite(register_fn);
}

void ClearFailureReason() {
  CurrentFailureReason().clear();
}

void BeginTestCase(const char* case_id, TestFamily family) {
  ActiveCaseId() = case_id == nullptr ? "" : case_id;
  ActiveFamily() = family;
  CurrentAssertions().clear();
}

void EndTestCase() {
  if (!ActiveCaseId().empty()) {
    CompletedEvidence()[ActiveCaseId()] = CurrentTestHasIndependentAssertion();
  }
  ActiveCaseId().clear();
  CurrentAssertions().clear();
}

void RecordAssertion(AssertionKind kind) {
  std::vector<AssertionKind>& assertions = CurrentAssertions();
  if (std::find(assertions.begin(), assertions.end(), kind) == assertions.end()) {
    assertions.push_back(kind);
  }
}

bool CurrentTestHasIndependentAssertion() {
  return std::any_of(CurrentAssertions().begin(), CurrentAssertions().end(), [](AssertionKind kind) {
    return kind == AssertionKind::kOracle || kind == AssertionKind::kAnchor ||
           kind == AssertionKind::kPresence || kind == AssertionKind::kDifferential;
  });
}
const std::string& CurrentTestCaseId() {
  return ActiveCaseId();
}

TestFamily CurrentTestFamily() {
  return ActiveFamily();
}

bool TestCaseHasIndependentAssertion(const std::string& case_id) {
  const auto it = CompletedEvidence().find(case_id);
  return it != CompletedEvidence().end() && it->second;
}

const std::vector<AssertionKind>& CurrentTestAssertions() {
  return CurrentAssertions();
}

void SetFailureReason(std::string reason) {
  CurrentFailureReason() = std::move(reason);
}

const std::string& FailureReason() {
  return CurrentFailureReason();
}

TestRegistry BuildRegisteredTests() {
  TestRegistry tests;
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

bool ValidateSpecLedger(const TestRegistry& tests, std::string* error) {
  std::ifstream in(WIRE_TEST_SPEC_LEDGER_PATH);
  if (!in.is_open()) {
    if (error != nullptr) {
      *error = std::string("unable to open ") + WIRE_TEST_SPEC_LEDGER_PATH;
    }
    return false;
  }

  std::unordered_set<std::string> registered_ids;
  std::set<std::string> duplicate_registered_ids;
  for (const TestCase& test : tests) {
    const std::string prefix = CasePrefix(test.case_id);
    if (prefix.empty()) {
      if (error != nullptr) {
        *error = std::string("invalid case id: ") + (test.case_id == nullptr ? "<null>" : test.case_id);
      }
      return false;
    }
    if (!registered_ids.insert(prefix).second) {
      duplicate_registered_ids.insert(prefix);
    }
  }
  if (!duplicate_registered_ids.empty()) {
    if (error != nullptr) {
      *error = "duplicate registered case ids: " + JoinSorted(duplicate_registered_ids);
    }
    return false;
  }

  std::unordered_set<std::string> ledger_ids;
  std::set<std::string> duplicate_ledger_ids;
  const std::regex row_pattern(R"(\|\s*(C\d+)\s*\|)");
  std::string line;
  while (std::getline(in, line)) {
    std::smatch match;
    if (!std::regex_search(line, match, row_pattern) || match.size() < 2) {
      continue;
    }
    const std::string id = match[1].str();
    if (!ledger_ids.insert(id).second) {
      duplicate_ledger_ids.insert(id);
    }
  }
  if (!duplicate_ledger_ids.empty()) {
    if (error != nullptr) {
      *error = "duplicate ledger case ids: " + JoinSorted(duplicate_ledger_ids);
    }
    return false;
  }

  std::set<std::string> missing_in_ledger;
  for (const std::string& id : registered_ids) {
    if (!ledger_ids.contains(id)) {
      missing_in_ledger.insert(id);
    }
  }

  std::set<std::string> missing_in_tests;
  for (const std::string& id : ledger_ids) {
    if (!registered_ids.contains(id)) {
      missing_in_tests.insert(id);
    }
  }

  if (!missing_in_ledger.empty() || !missing_in_tests.empty()) {
    if (error != nullptr) {
      std::ostringstream out;
      if (!missing_in_ledger.empty()) {
        out << "missing in spec ledger: " << JoinSorted(missing_in_ledger);
      }
      if (!missing_in_tests.empty()) {
        if (out.tellp() > 0) {
          out << "; ";
        }
        out << "missing in registered tests: " << JoinSorted(missing_in_tests);
      }
      *error = out.str();
    }
    return false;
  }

  return true;
}

} // namespace test_registry
