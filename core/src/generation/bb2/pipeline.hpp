#pragma once

#include "wire/core/core_state.hpp"

#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace wire::core::generation::bb2 {

inline constexpr std::size_t bad = std::numeric_limits<std::size_t>::max();

struct node {
  std::size_t id = bad;
  Vec3d pos{};
  SupportKind support = SupportKind::kPole;
  ObjectId pole = kInvalidObjectId;
  ObjectId saved = kInvalidObjectId;
  bool is_new = true;
  bool pinned = false;
  bool has_tangent = false;
  Vec3d tangent{};
  std::vector<SupportNodeBundleMode> bundle_modes{};
};

struct link {
  std::size_t id = bad;
  std::size_t a = bad;
  std::size_t b = bad;
  std::size_t route = 0;
  std::size_t order = bad;
  Vec3d dir{};
  ObjectId saved = kInvalidObjectId;
  bool is_new = true;
  std::size_t arow = bad;
  std::size_t brow = bad;
};

struct graph {
  std::vector<node> nodes{};
  std::vector<link> links{};
};

struct src {
  bool is_open = false;
  std::size_t id = bad;
};

struct pair {
  std::size_t id = bad;
  std::size_t node = bad;
  std::size_t left = bad;
  std::size_t right = bad;
  Vec3d axis{};
};

struct open {
  std::size_t id = bad;
  std::size_t node = bad;
  std::size_t link = bad;
  Vec3d axis{};
};

struct row {
  std::size_t id = bad;
  std::size_t node = bad;
  src source{};
  Vec3d axis{};
};

struct pairs {
  std::vector<link> links{};
  std::vector<pair> joins{};
  std::vector<open> opens{};
  std::vector<row> rows{};
};

enum class intent_reason : std::uint8_t {
  none = 0,
  node_mode_pass_through = 1,
  conflicting_rows = 2,
};

struct row_intent {
  std::size_t row = bad;
  std::size_t bundle = bad;
  CurvePassMode pass_mode = CurvePassMode::kPassThrough;
  bool lower_required = false;
  intent_reason reason = intent_reason::none;
};

struct intent {
  std::vector<row_intent> rows{};
};

struct group_member {
  std::size_t row = bad;
  std::size_t bundle = bad;
};

struct group {
  std::size_t id = bad;
  std::size_t node = bad;
  std::vector<group_member> row_members{};
  Vec3d group_axis{};
  int vertical_order = 0;
  double lower_offset_m = 0.0;
};

struct groups {
  std::vector<group> items{};
};

struct trow {
  std::size_t row = bad;
  std::size_t node = bad;
  src source{};
  Vec3d axis{};
  ObjectId pole = kInvalidObjectId;
  std::vector<std::vector<ObjectId>> ports{};
};

struct tspan {
  ObjectId id = kInvalidObjectId;
  std::size_t link = bad;
  std::size_t bundle = bad;
  std::size_t lane = bad;
  std::size_t arow = bad;
  std::size_t brow = bad;
};

struct topo {
  std::vector<ObjectId> bundles{};
  std::vector<std::size_t> bundle_specs{};
  std::vector<ObjectId> poles{};
  std::vector<ObjectId> new_poles{};
  std::vector<tspan> spans{};
  std::vector<trow> rows{};
};

struct rules {
  SpanLayoutRules data{};
};

struct layout {
  std::vector<SpanLayoutEntry> entries{};
};

struct curve {
  std::vector<std::pair<ObjectId, DetailCurve>> data{};
};

struct bounds {
  std::vector<std::pair<ObjectId, BoundsCacheEntry>> data{};
};

struct geom {
  curve curves{};
  bounds boxes{};
};

struct draw {
  std::vector<std::pair<ObjectId, SpanVisualCacheEntry>> visuals{};
  std::vector<std::pair<ObjectId, SpanRenderCacheEntry>> renders{};
};

class pipeline {
public:
  pipeline(CoreState& state, const BackboneSpec& spec) : state_(state), spec_(spec) {}

  [[nodiscard]] EditResult<bool> prepare();
  [[nodiscard]] EditResult<bool> check() const;
  [[nodiscard]] EditResult<GenerateBundleFromPathResult> build();

private:
  [[nodiscard]] EditResult<pairs> make(const graph& made) const;
  [[nodiscard]] EditResult<intent> make(const pairs& ps) const;
  [[nodiscard]] EditResult<groups> make(const pairs& ps, const intent& intents) const;
  [[nodiscard]] EditResult<bool> check(const pairs& ps) const;
  [[nodiscard]] EditResult<topo> emit(const pairs& ps);
  [[nodiscard]] EditResult<bool> emit_poles(topo* made, ChangeSet* changes);
  [[nodiscard]] EditResult<bool> emit_bundles(topo* made, ChangeSet* changes);
  [[nodiscard]] EditResult<bool> emit_ports(topo* made, const pairs& ps, ChangeSet* changes);
  [[nodiscard]] EditResult<bool> emit_spans(topo* made, const pairs& ps, ChangeSet* changes);
  [[nodiscard]] rules make(const topo& made, const groups& placement) const;
  [[nodiscard]] EditResult<layout> make(const rules& made) const;
  [[nodiscard]] geom make(const layout& made) const;
  [[nodiscard]] draw make(const layout& placed, const geom& shaped) const;
  void save(const rules& made);
  void save(const layout& made);
  void save(geom made);
  void save(draw made);
  [[nodiscard]] EditResult<bool> save_graph(const topo& made, const pairs& ps);
  [[nodiscard]] std::size_t local(std::size_t input_point) const;

  CoreState& state_;
  const BackboneSpec& spec_;
  bool ready_ = false;
  graph g_{};
  std::vector<std::size_t> active_bundle_indices_{};
  std::vector<std::size_t> local_by_input_{};
};

} // namespace wire::core::generation::bb2
