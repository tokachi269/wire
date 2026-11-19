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

std::unordered_map<std::string, std::vector<AssertionKind>>& CompletedEvidence() {
  static std::unordered_map<std::string, std::vector<AssertionKind>> evidence;
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

const char* AssertionKindName(AssertionKind kind) {
  switch (kind) {
  case AssertionKind::kOracle:
    return "oracle";
  case AssertionKind::kAnchor:
    return "anchor";
  case AssertionKind::kPresence:
    return "presence";
  case AssertionKind::kDifferential:
    return "differential";
  case AssertionKind::kDerivedEquality:
    return "derived_equality";
  }
  return "";
}

std::vector<std::string> LedgerRowCells(const std::string& line) {
  std::vector<std::string> cells{};
  std::string cell{};
  for (char character : line) {
    if (character == '|') {
      cells.push_back(cell);
      cell.clear();
      continue;
    }
    cell.push_back(character);
  }
  cells.push_back(cell);
  for (std::string& value : cells) {
    const std::size_t begin = value.find_first_not_of(" \t");
    const std::size_t end = value.find_last_not_of(" \t");
    value = begin == std::string::npos ? std::string{} : value.substr(begin, end - begin + 1);
  }
  if (!cells.empty() && cells.front().empty()) {
    cells.erase(cells.begin());
  }
  if (!cells.empty() && cells.back().empty()) {
    cells.pop_back();
  }
  return cells;
}

std::set<std::string> BacktickTokens(const std::string& text) {
  std::set<std::string> tokens{};
  std::size_t at = text.find('`');
  while (at != std::string::npos) {
    const std::size_t end = text.find('`', at + 1);
    if (end == std::string::npos) {
      break;
    }
    tokens.insert(text.substr(at + 1, end - at - 1));
    at = text.find('`', end + 1);
  }
  return tokens;
}

std::string JoinAspects(const std::set<std::string>& values) {
  std::string out{};
  for (const std::string& value : values) {
    if (!out.empty()) {
      out += " ";
    }
    out += value;
  }
  return out.empty() ? std::string{"none"} : out;
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
    CompletedEvidence()[ActiveCaseId()] = CurrentAssertions();
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
  if (it == CompletedEvidence().end()) return false;
  return std::any_of(it->second.begin(), it->second.end(), [](AssertionKind kind) {
    return kind == AssertionKind::kOracle || kind == AssertionKind::kAnchor ||
           kind == AssertionKind::kPresence || kind == AssertionKind::kDifferential;
  });
}

std::vector<std::string> TestCaseIdsWithOnlyDerivedEquality() {
  std::vector<std::string> out{};
  for (const auto& [case_id, assertions] : CompletedEvidence()) {
    const bool has_derived = std::find(assertions.begin(), assertions.end(),
                                       AssertionKind::kDerivedEquality) != assertions.end();
    const bool has_independent = std::any_of(
        assertions.begin(), assertions.end(), [](AssertionKind kind) {
          return kind == AssertionKind::kOracle || kind == AssertionKind::kAnchor ||
                 kind == AssertionKind::kPresence || kind == AssertionKind::kDifferential;
        });
    if (has_derived && !has_independent) out.push_back(case_id);
  }
  std::sort(out.begin(), out.end(), [](const std::string& a, const std::string& b) {
    const int a_num = CaseNumber(a.c_str());
    const int b_num = CaseNumber(b.c_str());
    return a_num == b_num ? a < b : a_num < b_num;
  });
  return out;
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

bool ValidateRecordedEvidence(const TestRegistry& tests, std::string* error) {
  // Evidence列は実行時に記録したassertion種別と一致させる (宣言だけの付与と種別を失う回帰を防ぐ)
  std::ifstream in(WIRE_TEST_SPEC_LEDGER_PATH);
  if (!in.is_open()) {
    if (error != nullptr) {
      *error = std::string("unable to open ") + WIRE_TEST_SPEC_LEDGER_PATH;
    }
    return false;
  }
  const std::set<std::string> runtime_aspects{"anchor", "derived_equality", "differential",
                                              "oracle", "presence"};
  const std::set<std::string> known_aspects{
      "anchor",   "derived_equality", "differential", "legacy_unclassified",
      "metamorphic", "oracle", "presence", "source_guard"};
  std::unordered_map<std::string, TestFamily> family_by_case{};
  for (const TestCase& test : tests) {
    family_by_case[CasePrefix(test.case_id)] = test.family;
  }
  std::unordered_map<std::string, std::set<std::string>> recorded_by_case{};
  for (const auto& [case_id, kinds] : CompletedEvidence()) {
    std::set<std::string> names{};
    for (AssertionKind kind : kinds) {
      names.insert(AssertionKindName(kind));
    }
    recorded_by_case[CasePrefix(case_id.c_str())] = names;
  }
  std::vector<std::string> problems{};
  const std::regex row_pattern(R"(\|\s*(C\d+)\s*\|)");
  std::string line;
  while (std::getline(in, line)) {
    std::smatch match;
    if (!std::regex_search(line, match, row_pattern) || match.size() < 2) {
      continue;
    }
    const std::string id = match[1].str();
    const std::vector<std::string> cells = LedgerRowCells(line);
    if (cells.size() < 5) {
      problems.push_back(id + ": ledger row has no evidence cell");
      continue;
    }
    const std::set<std::string> declared = BacktickTokens(cells[4]);
    if (declared.empty()) {
      problems.push_back(id + ": evidence cell is empty");
      continue;
    }
    bool unknown = false;
    for (const std::string& token : declared) {
      if (!known_aspects.contains(token)) {
        problems.push_back(id + ": unknown evidence " + token);
        unknown = true;
      }
    }
    if (unknown) {
      continue;
    }
    if (declared.contains("legacy_unclassified")) {
      if (declared.size() != 1) {
        problems.push_back(id + ": legacy_unclassified must stand alone");
      }
      continue;
    }
    const auto family_it = family_by_case.find(id);
    const bool source_guard_family =
        family_it != family_by_case.end() && family_it->second == TestFamily::kSourceGuard;
    if (declared.contains("source_guard") != source_guard_family) {
      problems.push_back(id + (source_guard_family
                                   ? ": source_guard family is not declared"
                                   : ": source_guard is declared for a behavior case"));
      continue;
    }
    std::set<std::string> declared_runtime{};
    for (const std::string& token : declared) {
      if (runtime_aspects.contains(token)) {
        declared_runtime.insert(token);
      }
    }
    if (declared_runtime.empty()) {
      continue;
    }
    const auto recorded_it = recorded_by_case.find(id);
    const std::set<std::string> recorded =
        recorded_it == recorded_by_case.end() ? std::set<std::string>{} : recorded_it->second;
    std::set<std::string> missing{};
    for (const std::string& token : declared_runtime) {
      if (!recorded.contains(token)) {
        missing.insert(token);
      }
    }
    if (!missing.empty()) {
      problems.push_back(id + ": declared [" + JoinAspects(declared_runtime) + "] but recorded [" +
                         JoinAspects(recorded) + "]");
    }
  }
  if (!problems.empty()) {
    if (error != nullptr) {
      std::ostringstream out;
      for (std::size_t i = 0; i < problems.size(); ++i) {
        if (i > 0) {
          out << "; ";
        }
        out << problems[i];
      }
      *error = out.str();
    }
    return false;
  }
  return true;
}

} // namespace test_registry
