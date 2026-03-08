#include "path_pick_policy.hpp"

#include <algorithm>

namespace {

bool IsTemplateSelected(std::uint32_t selected_template_mask, wire::core::BundleKind kind) {
  const auto bit = (1u << static_cast<unsigned>(kind));
  return (selected_template_mask & bit) != 0;
}

} // namespace

std::vector<wire::core::BundleKind> SelectedBundleTemplateKinds(const wire::core::CoreState& state,
                                                                std::uint32_t selected_template_mask) {
  std::vector<wire::core::BundleKind> selected{};
  for (const auto& [kind, _] : state.view().bundle_templates()) {
    if (IsTemplateSelected(selected_template_mask, kind)) {
      selected.push_back(kind);
    }
  }
  std::sort(selected.begin(), selected.end(), [](auto a, auto b) { return static_cast<int>(a) < static_cast<int>(b); });
  return selected;
}

wire::core::BundleKind ResolveBundleTemplateForPathPick(const wire::core::CoreState& state,
                                                        std::uint32_t selected_template_mask,
                                                        const wire::core::PickResult& pick) {
  const auto selected = SelectedBundleTemplateKinds(state, selected_template_mask);
  if (!selected.empty()) {
    return selected.front();
  }
  if (pick.hit_kind == wire::core::PickHitKind::kSegment) {
    if (const wire::core::Span* span = state.view().edit_state().spans.find(pick.hit_id); span != nullptr) {
      if (const wire::core::Bundle* bundle = state.view().edit_state().bundles.find(span->bundle_id); bundle != nullptr) {
        return bundle->bundle_template_id;
      }
    }
  }
  return wire::core::BundleKind::kLowVoltage;
}

std::vector<wire::core::BundleKind> ResolveTemplateKindsForPathPick(const wire::core::CoreState& state,
                                                                    std::uint32_t selected_template_mask,
                                                                    const wire::core::PickResult& pick) {
  const auto selected = SelectedBundleTemplateKinds(state, selected_template_mask);
  if (!selected.empty()) {
    return selected;
  }
  return {ResolveBundleTemplateForPathPick(state, selected_template_mask, pick)};
}

std::string FindMidairBranchBlockedTemplateName(const wire::core::CoreState& state,
                                                const std::vector<wire::core::BundleKind>& template_ids) {
  for (wire::core::BundleKind kind : template_ids) {
    const auto it = state.view().bundle_templates().find(kind);
    if (it == state.view().bundle_templates().end()) {
      continue;
    }
    if (!it->second.allow_midair_branch) {
      return it->second.name.empty() ? std::to_string(static_cast<int>(kind)) : it->second.name;
    }
  }
  return {};
}
