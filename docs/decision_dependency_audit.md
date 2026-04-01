# 決定値依存監査メモ

## 対象

このメモは、次の4項目について

- どこで初回決定しているか
- どこへ複写しているか
- どこで再解釈しているか

を整理したものです。

対象は次の4つです。

- `lower_required`
- `pair`
- `side_axis / chosen_side_sign`
- `support placement`

目的は命名整理ではありません。
「一度決めた値を後段がまた決めていないか」を見えるようにすることです。

## 現状観測

- 現在の `Commit(run_recalc=true)` は redraw だけではありません。`ProcessDirtyQueues()` の `kGeometry` 経路で `rebuild_span_curve()` が走り、その前段として support layout と detail-curve 入力の再構築まで含みます。
- そのため fresh generate と `Commit(recalc)` は同じ見た目の refresh ではなく、別経路としてズレる余地があります。
- ただし「recalc が毎回 lane continuity や `lower_required` を再決定している」とまでは断定しません。観測上は downstream が upstream authority を消費し直し、その過程で fallback / refresh / resolver が入るため、実質的に再決定っぽく振る舞う場面がある、という整理に留めます。
- HV 系の切り分けでも、fresh generate の時点で壊れる系と、`Commit(recalc)` で悪化する系が混在していました。

## `kGeometry` 発火元棚卸し

### 1. topology / 新規 span 生成

- `core/src/state/state.cpp:366-420` `AddSpan()`
  - `DirtyBits::kTopology | DirtyBits::kGeometry`
  - 新しい span に対して decision seed / support layout / curve を最初から作る経路です。
  - 分類上は `Decision` と `GeometryRefresh` の両方を含みます。

### 2. endpoint 実座標の変更

- `core/src/state/state.cpp:592-609` `MovePort()`
- `core/src/state/state.cpp:612-679` `ResetPortPositionToAuto()`
- `core/src/state/state.cpp:682-694` `MoveAnchor()`
- `core/src/state/endpoint_refresh_service.cpp:175-187`
- `core/src/state/endpoint_refresh_service.cpp:210-214`
  - 接続先 span に `DirtyBits::kGeometry` を立てます。
  - これは本来 `GeometryRefresh` 寄りです。endpoint world / anchor world が変わるので curve と support placement を作り直す必要はありますが、upstream decision authority 自体は変えていません。

### 3. pole 変形からの owned endpoint 再投影

- `core/src/state/state.cpp:468-483` `MovePole()`
- `core/src/state/state.cpp:486-590` `ApplyPoleTilt()`
- `core/src/state/state.cpp:697-718` `SetPoleFlip180()`
- `core/src/state/state.cpp:721-749` `SetPoleManualYawOverride()`
- `core/src/state/state.cpp:752-780` `ClearPoleOrientationOverride()`
- `core/src/state/state.cpp:903-913` `finalize_pole_transform_update()`
- `core/src/state/state.cpp:1833-1866` `UpdatePoleTypeDefinition()` -> `ApplyPoleType()` -> endpoint refresh
  - これらは最終的に `refresh_owned_endpoints_from_pole()` から接続 span へ `DirtyBits::kGeometry` を波及させます。
  - 分類上は `GeometryRefresh` 寄りです。pole 側の姿勢や port/anchor 投影が変わるため layout/materialization はやり直しますが、decision seed を積極的に作り直す操作ではありません。

### 4. span support / attachment override

- `core/src/state/state.cpp:783-804` `SetSpanEndpointSocketOverride()`
- `core/src/state/state.cpp:807-834` `ClearSpanEndpointSocketOverride()`
- `core/src/state/state.cpp:837-861` `SetSpanBranchDownOffsetOverride()`
- `core/src/state/state.cpp:864-881` `ClearSpanBranchDownOffsetOverride()`
  - socket / down offset は support layout endpoint の materialization 入力そのものです。
  - 分類上は `Decision/Layout recompute` 側です。現在は `kGeometry` に入っていますが、単なる curve refresh より一段上流です。

### 5. attachment / template / reference length / global settings

- `core/src/state/state.cpp:423-464` `AddAttachment()`
- `core/src/state/template_mutation_service.cpp:165-285` `UpdateBundleTemplate()`
- `core/src/state/template_mutation_service.cpp:292-327` `UpdateAttachmentTemplate()`
- `core/src/state/template_mutation_service.cpp:330-363` `ResetAllSpanReferenceLengths()`
- `core/src/state/state.cpp:1549-1574` `UpdateGeometrySettings()`
- `core/src/state/state.cpp:1624-1657` `UpdateVariationSettings()`
- `core/src/state/state.cpp:1660-1686` `UpdateContextProfile()`
  - ここは一番混ざっています。
  - `UpdateGeometrySettings()` はほぼ `GeometryRefresh` です。
  - `AddAttachment()` と `UpdateAttachmentTemplate()` は endpoint socket / attachment style / line effect に触れるので `GeometryRefresh` だけではなく support-layout materialization を含みます。
  - `UpdateBundleTemplate()` は変更内容で責務が割れます。
    - `visual_only_change` でも cable template 差し替えにより endpoint mode / continuity / material style が変わり得るため、現実には `RenderRefresh` だけでは閉じません。
    - `detail_change` は support wire band を触るので `Decision/Layout recompute` 寄りです。
    - `topology_change` は regeneration 扱いで `kGeometry` の外に出しているため、ここでは別経路です。
  - `UpdateVariationSettings()` は sag variation と branch down offset variation の両方に触るため、`GeometryRefresh` と support-layout materialization が混ざります。
  - `UpdateContextProfile()` は style route 経由で attachment style / endpoint mode / material style に影響するため、`RenderRefresh` だけではなく curve 入力側にも入っています。

### 現時点の粗い仕分け

- `Decision/Layout recompute` 寄り
  - `AddSpan()`
  - span endpoint socket override の set/clear
  - span branch down offset override の set/clear
  - `UpdateBundleTemplate()` の `detail_change`
  - `UpdateContextProfile()`
- `GeometryRefresh` 寄り
  - `MovePort()`
  - `ResetPortPositionToAuto()`
  - `MoveAnchor()`
  - pole move / tilt / yaw / flip / pole type refresh からの endpoint refresh
  - `UpdateGeometrySettings()`
  - `ResetAllSpanReferenceLengths()`
- 混在
  - `AddAttachment()`
  - `UpdateAttachmentTemplate()`
  - `UpdateBundleTemplate()` の `visual_only_change`
  - `UpdateVariationSettings()`

## 1. `lower_required`

### 初回決定

- relation レベルの feasibility は `annotate_same_level_feasibility()` で最初に入っています。
  - `core/src/generation/from_backbone.cpp:2124`
  - `core/src/generation/from_backbone.cpp:2150`
  - `core/src/generation/from_backbone.cpp:2187`
- endpoint レベルの `decision.lower_required` は、その feasibility から `build_endpoint_decision()` で作られています。
  - `core/src/generation/grouped_spans.cpp:1137`

### 複写

- endpoint decision から assignment 集約値へ複写しています。
- assignment から `SpanSupportLayoutEntry` へ複写しています。
- decision から `SupportLayoutEndpoint` へ複写しています。
- recalc refresh 時に、既存 layout の値を新しい layout へ再複写しています。
- inspection DTO にも複写しています。

### 再解釈

- `make_support_layout_seed_endpoint()` では `lower_required` を「ここで down offset を materialize するか」の判定に使っています。
- `UsesAuthoritativeGroupedLoweredSupport()` では
  `owner_pole_id + lower_required + !lowering_blocked_by_policy + support_group_id`
  を grouped lowered support の使用条件として再解釈しています。
- validator では `lower_required` を具体的な二状態高さルールに落としています。

### 読み

- この4項目の中では比較的ましです。
- 問題は boolean 自体の再計算より、複数構造に複写された後で意味スイッチとして使われていることです。

## 2. `pair`

### 初回決定

- pair 候補の記録は `record_support_pair()` で最初に作られています。
- endpoint decision への pair 反映は `build_endpoint_decision()` です。
- grouped lowered 用の canonical pair は `canonicalize_group_endpoint_decision()` でもう一度入れています。

### 複写

- pair は support-group key の構成要素にも使っています。
- endpoint decision から `SupportGroupDecision` へ複写しています。
- `SupportGroupDecision` から `LoweredSupportGroupPlacement` へ複写しています。
- inspection DTO にも複写しています。

### 再解釈

- 現在の recalc は pair の専用補修はしていません。主に generation で入った値を support-group decision / placement へ流し直す構造ですが、fresh generate と完全同値の refresh だとはまだ言えません。
- validator では endpoint decision / support-group decision / grouped placement の一致を要求しています。

### 読み

- `pair` はまだ single source ではありません。
- recalc 側の補完は外しましたが、複数構造へ複写される構図自体は残っています。

## 3. `side_axis / chosen_side_sign`

### 初回決定

- pole orientation や tangent 順から base hint を作っています。
- endpoint の side-axis policy は `preferred_side_axis_for_endpoint()` で選んでいます。
- endpoint の side sign は `finalize_side_sign_for_ports()` で決めています。
- grouped lowered 用の canonical axis/sign は `resolve_group_side_axis()` とその後続でまた決めています。
- final endpoint decision への反映は `build_endpoint_decision()` です。
- grouped canonical 化で endpoint decision をもう一度上書きしています。

### 複写

- assignment に per-end の axis/sign を持っています。
- seeded support-layout endpoint に decision の axis/sign を複写しています。
- decision から `SupportGroupDecision` へ複写しています。
- `SupportGroupDecision` から grouped placement へ複写しています。
- recalc refresh で decision から layout endpoint に再複写しています。
- inspection DTO にも複写しています。

### 再解釈

- generation 側には grouped lowered 用 canonicalize が残っています。
- placement builder で `side_axis + sign` から具体的な support axis を再構成しています。
- validator でも mount/tip から物理軸を再構成して authoritative axis と比較しています。

### 読み

- この4項目の中で一番絡んでいます。
- recalc 側の axis/sign 補修は外しました。
- ただし generation 側 canonicalize と、placement/validator 側の物理軸再構成はまだ別段にあります。

## 4. `support placement`

### 初回決定

- 現状、完全な単一決定点ではありませんが、grouped support については `SupportGroupDecision` 側へ寄せ始めています。
- 実際には次の段階構成です。
  - seed endpoint placement は port world position と offset 決定から始まる
  - grouped support decision が grouped support の `down_offset_m` と `support_world` を持つ
  - grouped placement がその決定結果から mount/tip を materialize する
  - layout endpoint の placement は group decision と grouped placement から書き戻す

### 複写

- seed support layout は endpoint の `support_world`、`automatic_branch_down_offset_m`、`branch_down_offset_m` を保持します。
- group decision は `support_world`、`down_offset_m`、`attachment_worlds` を保持します。
- group placement はそれらを `LoweredSupportGroupPlacement` に複写します。
- recalc は grouped placement を layout endpoint 側にも書き戻します。
- inspection DTO にも endpoint / placement geometry を複写しています。

### 再解釈

- 現在の grouped support decision は、`support_world` を endpoint seed の高さから拾わず、
  `pole + category template height - branch_down_offset_m` で導出しています。
- grouped support decision の `down_offset_m` は、group 作成時の endpoint 値を一度だけ採用し、
  後続 endpoint から max/min 集約で作り直さない形に寄せています。
- grouped placement builder は次を材料に物理 geometry を再構成しています。
  - decision axis/sign
  - attachment 由来 fallback axis
  - support z
  - pole clearance / visual settings
- validator でも placement policy を再度知っています。
  - 二状態高さ
  - endpoint support height は grouped tip height と一致
  - mount/tip axis は authoritative axis に整列

### 読み

- 4項目の中で最も single source から遠いです。
- ただし grouped support の `support_world` と endpoint 書き戻しは、`SupportGroupDecision` を中心にかなり寄せられました。
- 未整理なのは `down_offset_m` の初回決定がまだ endpoint 値依存なことと、validator が policy を持っていることです。

## まとめ

### まだ比較的まし

- `lower_required`

### 補修は外したが複写が多い

- `pair`

### 最も絡んでいる決定値

- `side_axis / chosen_side_sign`

### 最も分散している materialization 値

- `support placement`

## 直近の含意

一方向依存を本当に固定したいなら、次にやるべきことは `recalc` 廃止ではなく、`recalc` を `Refresh` 寄りに縮めることです。

- `Refresh` に残すもの: curve 再サンプル、bounds 更新、visual cache 更新、raycast 更新
- 別 dirty へ出すもの: support layout 再解決、lowered/grouped 再評価、lane continuity / orientation 再評価
- `down_offset_m` の初回決定が endpoint 値依存なこと
- validator が structural check だけでなく policy を持っていること

その前に未完なのは次です。

- いまの `kGeometry` がどこで decision 再解釈まで飲み込んでいるかの棚卸し
- 既存 call site を `Decision` / `GeometryRefresh` / `RenderRefresh` のどれに落とすべきかの仕分け

recalc の decision 補修は外し、grouped support の placement も `SupportGroupDecision` 中心に寄せました。
次の主戦場は `kGeometry` の棚卸しと、`down_offset_m` を含む decision authority の正本化です。

## `rebuild_span_curve()` 内の切り分け

### 入口

- `core/src/recalc/recalc_pipeline.cpp:283-313` `rebuild_span_curve()`
  - 実質は `generate_span_support_layout()` と `generate_span_curve()` に責務が分かれています。
  - さらに `cache_span_support_layout()` の中で `rebuild_all_lowered_support_groups()` が走るため、単一 span の rebuild に見えて cache 全体の grouped support materialization まで含みます。

### authority を読むだけの処理

- `core/src/recalc/support_layout_materialization.cpp:662-715`
  - `find_span_support_layout_seed()` があれば seed を使う。
  - seed がなければ `find_span_support_layout()` の既存 layout から authoritative decision を複写する。
  - `apply_support_layout_decision_seed()` / `apply_authoritative_support_layout_decisions()` は decision 値の消費です。
- `core/src/recalc/recalc_pipeline.cpp:581-594`
  - `make_curve_constraint_from_support_layout()` は layout endpoint を curve constraint に落とすだけです。
  - `BuildDetailCurve()` に入る前のこの段階では、authority の読取に近いです。
- `core/src/recalc/support_layout_materialization.cpp:518-559`
  - `build_grouped_support_placement_from_decision()` は `SupportGroupDecision` から mount/tip/support axis を materialize する段です。
  - これは geometry 導出であって、decision の再決定とは分けて扱うべきです。

### 実質再解釈っぽく振る舞う処理

- `core/src/recalc/detail_curve_input_resolution.cpp:234-243`
  - `existing_layout` がなければ `support_layout_flow_kind_for_span()` へ落ちます。
  - ここは authority 消費ではなく、port placement source からの fallback 判定です。
- `core/src/recalc/support_layout_materialization.cpp:269-277`
  - `fallback_branch_down_offset_for_support_port()` は template 高さと port 実高さの差から automatic down offset を再導出します。
  - seed / authoritative layout がない時の fallback ですが、実質的には support 高さ決定に関わります。
- `core/src/recalc/support_layout_materialization.cpp:314-373`
  - `build_support_layout_endpoint()` は `make_curve_constraint_from_port()`、default socket 解決、attachment socket 適用、fallback endpoint source 判定まで持っています。
  - ここは authority を layout endpoint geometry へ落とすだけではなく、attachment/socket の resolver を通すので、materialization と heuristic が混在しています。
- `core/src/recalc/support_layout_materialization.cpp:435-458`
  - `grouped_lowered_route_local_departure_dir()` は `side_axis` と peer 位置から departure tangent を組み直します。
  - authority の消費先ではありますが、curve 入力の向きを再構成するので再解釈っぽく見える箇所です。
- `core/src/recalc/detail_curve_input_resolution.cpp:63-98`
  - `detail_curve_profile_hint_from_support_layout()` は grouped lowered / composite height transition を layout の見た目条件から選びます。
  - decision field の単純複写ではなく、curve policy を別段で選んでいます。
- `core/src/recalc/detail_curve.cpp:539-590`
  - `ChooseShapePolicy()` は endpoint offset / tangent alignment / grouped lowered profile / corner 条件から shape policy を選びます。
- `core/src/recalc/detail_curve.cpp:765-816`
  - `DecideContinuity()` は tangent hint と chord 条件から continuity を選び直します。
- `core/src/recalc/detail_curve.cpp:1016-1057`
  - poor-quality fallback で tangent scale を落とし、必要なら `G2 -> G1` へ degrade します。
  - これは semantic decision の再決定ではなく curve solver fallback ですが、見た目上は「fresh generate と別判断」に見えやすい箇所です。

### 今の境界整理

- `generate_span_support_layout()`
  - authority consumption
  - support-layout materialization
  - fallback / resolver
  - grouped support write-back
- `generate_span_curve()`
  - support layout を curve constraint に変換
  - curve shape / continuity / quality fallback の選択

現状の問題は、これらが全部 `DirtyBits::kGeometry` の 1 本に乗っていることです。
`rebuild_span_curve()` 自体は短いですが、意味としては `GeometryRefresh` ではなく `LayoutRefresh + CurvePolicySelection + CurveBuild` をまとめて呼んでいます。
