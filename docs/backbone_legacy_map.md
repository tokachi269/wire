# backbone legacy map

この文書は、v1 / recalc / support-layout / 旧テストの現行依存を整理する地図。
履歴や milestone log ではない。現在の設計契約は `redesign.md`、退避済み履歴は `docs/archive/redesign_history_2026-06.md` を読む。

## 分類

| 分類 | 判断 |
|---|---|
| A | bb2 本流から未使用。物理削除候補 |
| B | viewer / public query が読む。先に neutral output へ移す |
| C | tests だけが読む。制約を bb2 構造へ移植してから退役 |
| D | v1 専用として隔離 |
| E | bb2 未対応 scenario のため残っている。supported 化か unsupported 固定を決める |

## v1 / recalc / support-layout 残存依存

| family / file | 現在の caller | 分類 | bb2 generation 中に読むか | 次の切断先 |
|---|---|---|---|---|
| `core/src/recalc/recalc_pipeline.cpp` / `Commit` / dirty queue | v1 runtime、手動 debug、state load/update、旧テスト | D/E | no | supported post-edit は direct derive へ移す。残りは v1/manual/debug に隔離する |
| `core/src/recalc/support_layout_materialization.*` | recalc pipeline、旧 support-layout tests | D/E | no | user-visible 制約だけ bb2 rules/layout/geom/draw へ移植。object shape は移植しない |
| `core/src/recalc/support_layout_projection.*` | recalc、validation、旧 tests | D/E | no | neutral `span_layout` / `span_layout_state` / direct caches へ読み替える |
| `core/src/recalc/detail_curve*` | recalc curve tests、v1 geometry rebuild | D/E | no | bb2 は `core/src/generation/bb2/out.*` を使う。必要な curve 制約だけ移植する |
| `core/src/recalc/style_context.cpp` / `variation.cpp` | v1 render/style rebuild | D/E | no | viewer-blocking visual requirement が出た場合だけ bb2 draw 側で明示実装する |
| `CoreView::inspect_support_layout` / `SupportLayoutInspectionView` | 旧 tests | C/D | no | viewer / capture からは外した。残りは旧 tests を分類して v1 専用へ隔離する |
| `support_layout_contract` / `support_layout_projection` accessors | recalc、validation、旧 tests | C/D/E | no | no-authority 確認は `span_layout_state` へ移行済み。残りは v1/debug/test に隔離する |
| validation の support-layout authority checks | validation-only、旧 tests | D/E | no | bb2 normal path の blocker にしない。必要制約だけ neutral validation へ移す |
| old `backbone_pipeline` / `bundle_spans` / grouped span generation | guard strings、旧履歴、削除済み family | A | no | production 復活禁止。残る参照が test guard だけなら削除対象ではない |
| `core/src/generation/build_backbone_service.cpp` | public generation entrypoint | E/F | yes, as service glue | bb2 mainline rename までは残す。v1 fallback を戻さない |
| public `BuildBackboneResult` / `BuildBackboneEdges` / `FindBackboneRoute` | public query、viewer/query tests | B | no v1 fallback | saved graph backed query として維持。旧名 rename は mainline 化後 |
| `core/tools/capture_replay.cpp` | debug/capture tooling | mainline/debug | no | neutral span layout / rules / geom / draw / saved graph を読む。support-layout inspection を戻さない |

## viewer 境界

| 領域 | 現在の状態 | 判断 | 次の切断先 |
|---|---|---|---|
| viewer normal path | `span_layout`, `span_layout_state`, rules, curve, bounds, visual/render, saved backbone result を読む | mainline | 旧 support-layout contract を戻さない |
| pole-height normal UI | `inspect_support_layout` 依存は削除済み | mainline | 追加作業なし |
| selected `SupportLayout` manual debug panel | neutral `span_layout` / rules / geom / draw / saved frontier を読む | mainline/debug | `inspect_support_layout` を戻さない |
| manual `Run Legacy Recalc` | 明示操作で `Commit(run_recalc)` を呼ぶ | D | normal path ではない。削除時は代替 direct derive 操作が必要 |
| validation-only commit | validation/debug 経路 | D | bb2 normal path の blocker にしない |
| capture replay | neutral span layout / rules / bounds / visual / render を読む | mainline/debug | support-layout inspection を戻さない |

## 旧テスト family

旧テストは 1 件ずつ直さない。family 単位で移植または退役する。

| family | 旧 surface | 分類 | bb2 replacement / 判断 |
|---|---|---|---|
| unsupported leaves state unchanged | old `Commit` / dirty queue / count checks | A | bb2 preflight と object/saved-graph count checks で扱う。新 supported scenario 追加時だけ拡張 |
| duplicate does not mutate | duplicate generation + old output count checks | A | duplicate preflight と saved binding checks で扱う。rollback-after-emit を戻さない |
| output cache presence | `support_layout_projection`, curve/bounds/visual/render after recalc | C/D | 詳細は下の「output cache presence family」を参照。bb2 required outputs は rules/layout/geom/draw を直接確認する |
| viewer-required output | viewer scene/capture constraints | A/B | viewer representative scenes と bb2 scenario tests で扱う。既に pass するケースへの test 追加だけでは進捗扱いにしない |
| post-edit direct derive determinism | `Commit(run_recalc)` after port/pole edits | B | supported edit が recalc を必要とする場合だけ direct derive tests へ移植 |
| public backbone query | `BuildBackboneResult`, `BuildBackboneEdges`, `FindBackboneRoute` | A/B | saved graph backed query として維持。span-derived fallback は戻さない |
| support-layout authority/seed/projection internals | `inspect_support_layout`, `support_layout_contract`, authority seed/projection fields | C/D | v1 legacy。必要制約は `SavedBackboneGraph` + `SpanLayoutRules` + `SpanLayoutEntry` + support group で表す |
| recalc persistence / materialization internals | `Commit(run_recalc)`, support-layout materialization, detail curve recalc, support group repair | C/D | 詳細は下の「recalc persistence family」を参照。dirty queue / materialization order は bb2 要件にしない |
| attachment/socket support layout | socket resolution through support-layout materialization | B/C | bb2 attachment/socket requirement が明確になるまで mainline 完了条件にしない |
| cable style / insulator / supplemental render | recalc style context, full visual cache parts | B/C | viewer-blocking visual requirement が出た場合だけ bb2 draw へ移植 |

## output cache presence family

旧テストは `support_layout_projection` と recalc 後 cache を使って「出力が欠けない」制約を見ていた。
この family の制約は bb2 でも必要だが、旧 projection / authority / seed object の存在は bb2 の完了条件にしない。

| old surface | current callers | bb2 replacement | 判断 |
|---|---|---|---|
| `support_layout_projection(span).layout` | `core/tests/geometry.cpp`, `core/tests/generation.cpp`, `core/tests/template_policy.cpp`, `core/tests/bundle_visuals.cpp`, `core/tests/helpers.cpp` | `span_layout(span)`, `span_layout_state(span)`, `span_layout_rules(span)` | C/D。layout existence 制約だけ移植し、projection object shape は移植しない |
| curve / bounds cache after rebuild | `core/tests/geometry.cpp`, `core/tests/generation.cpp`, `core/tests/workflow.cpp`, `core/tests/bundle_visuals.cpp` | bb2 `geom` output、`CurveCacheEntry`、`BoundsCacheEntry` | C。生成直後と direct derive 後に欠けないことを bb2 側で見る |
| visual / render cache after rebuild | `core/tests/bundle_visuals.cpp`, `core/tests/bb2.cpp` | bb2 `draw` output、`SpanVisualCacheEntry`、`SpanRenderCacheEntry` | C。viewer required output として必要な範囲だけ見る。full support visual semantics は別 family |
| support-layout authority / seed / projection fields | support-layout 旧 tests | `SpanLayoutRules`, `SpanLayoutEntry`, `SpanLayoutState.input_required=false` | D。旧 contract そのものは v1 専用。bb2 へ戻さない |

bb2 側で既に固定している代表テスト:

| bb2 test | 守る制約 |
|---|---|
| C379 | generated span に topology / rules / layout / curve / bounds がある |
| C447-C457 | neutral layout read/cache boundary を使い、旧 projection を正規観測口にしない |
| C511-C514 | draw cache は geom/layout から保存される |
| C524 / C539 | supported request は saved graph / rules / layout / geom / draw を bb2 本流で作る |
| C611-C613 | direct derive は saved rules から layout / geom / draw を復元し、recalc へ戻らない |

この family を今後触るときの判断:

* 旧テストが「出力 cache が存在する」ことを守っていたなら、bb2 の neutral output で移植する。
* 旧テストが `support_layout_projection` / authority / seed / projection の形を要求しているだけなら、v1 専用として隔離する。
* viewer で実際に必要な visual / render が欠ける場合だけ、bb2 draw requirement として supported scenario を増やす。
* 既に pass するケースに C 番号を足すだけなら進捗扱いにしない。

## recalc persistence family

旧テストは `Commit(run_recalc)` 後も generation 時の意味が痩せないことを見ていた。
この制約は重要だが、recalc dirty queue、materialization order、support-layout object shape は bb2 の要件にしない。

| representative old cases | original constraint | v1 implementation detail | bb2 replacement / 判断 |
|---|---|---|---|
| C221/C224/C225/C227/C232 | recalc / refresh 後も lowering、unresolved conflict、relation origin が消えない | `Commit(run_recalc)` 後に `inspect_span` / `inspect_support_layout` の旧 relation/lowering fields を読む | 退役済み。bb2 では `SpanLayoutRules`、support group、`SpanLayoutEntry`、geom/draw、direct derive determinism で見る。dirty queue persistence は移植しない |
| C234-C237 | lowered/cross/branch の support side / orientation が recalc materialization 後も pair-aware に残る | `SupportLayoutInspectionView` endpoint と lowered support group の orientation/side fields | 退役済み。bb2 では pair/open/row + support group placement + layout/geom/draw output で見る。old support endpoint decision fields は移植しない |
| C196/C286 | branch support visual が recalc 後の grouped support view / pole tilt に追従する | lowered support group inspection の mount/tip/attachment shape | 退役済み。viewer-blocking visual だけ bb2 draw/support group requirement に移す。full recalc grouped support inspection shape は v1 専用 |

この family を今後触るときの判断:

* 「supported edit 後に表示出力が再導出される」は `DeriveGeneratedSpanOutputs` と direct cache checks で見る。
* 「lowering / placement が消えない」は support group、layout endpoint、geom bounds、draw placeholder で見る。
* `Commit(run_recalc)` が何件 dirty queue を処理したか、どの順で materialize したかは bb2 に移植しない。
* 旧テスト内の一時 debug 出力は制約ではないため削除対象。

## support-layout authority / seed / projection test family

旧テストは `inspect_support_layout` と support-layout seed/authority/projection fields を使って、
generation が決めた pair / side / lowering / order が refresh や materialization で再判断されないことを見ていた。
この family の制約は重要だが、旧 seed/projection object identity は bb2 の要件にしない。

| representative old cases | original constraint | v1 implementation detail | bb2 replacement / 判断 |
|---|---|---|---|
| C348-C352 | support pair / side / semantic relation が refresh で変わらない | `has_decision_seed`, `support_authority`, `relation_kind`, `continuity_class` を `SupportLayoutInspectionView` で読む | 退役済み。bb2 では `SavedBackboneGraph` + pair/open/row + `SpanLayoutRules` + direct derive determinism で見る。旧 seed identity は移植しない |
| C353-C354 | same-level T/cross 相当で高さ class が分かれる | `pair_height_rank`, `branch_down_offset_m`, `relation_kind` を inspection endpoint で読む | 退役済み。bb2 では support group / layout endpoint / geom bounds で lower offset と出力差を確認する。T/cross kind は作らない |
| C244-C248 | authoritative endpoint decision が support layout / refresh 後も変わらない | order decision / side / orientation basis / seed fields を `SupportLayoutEndpointView` で読む | 退役済み。bb2 では pair/open/row、support group、layout/geom/draw consumer chain、direct derive determinism で見る。旧 endpoint field shape は移植しない |
| C263 | grouped support decision と placement が cache から一貫して読める | `grouped_authority_cache_complete`, `authoritative_group_cache_present`, lowered support inspection view | D/C。旧 cache completeness field は v1 専用。必要なら bb2 support group + `SpanLayoutEntry.lowered_support_group_keys` + draw placeholder で制約化する |
| C240/C242 | lowered endpoint の order decision が refresh 後も維持される | SupportLayout endpoint の order decision/reason fields | 退役済み。bb2 では placement/support group と direct derive 後の layout/geom/draw 不変条件へ移す。旧 endpoint field shape は移植しない |

この family を今後触るときの判断:

* 「refresh / derive 後に上流決定が変わらない」は bb2 direct derive test へ移す。
* 「pair / side / lowering を下流が再判断しない」は `pairs make(graph)` / support group / layout owner の boundary test で見る。
* `has_decision_seed`, authority object identity, projection internals, grouped authority cache completeness は v1 専用として隔離する。
* 旧 `relation_kind` / ThroughMain / SideBranch / CrossUnderpass ラベルは bb2 の正本に戻さない。
## 削除候補と残存理由

| 対象 | 状態 | 残す理由 | 削除条件 |
|---|---|---|---|
| old grouped span engine family | production 削除済み | guard/test/履歴以外の caller がない前提 | production caller が残っていないことを確認し続ける |
| support-layout materialization | 残存 | recalc / v1 / old tests が読む | v1 専用隔離後、必要制約を bb2 に移植 |
| support-layout authority/seed/projection inspection | 残存 | old tests が読む | old tests を v1 専用へ隔離し、必要制約だけ bb2 direct outputs へ移植 |
| recalc pipeline | 残存 | v1 / manual debug / validation / old tests | bb2 normal path と supported post-edit から完全に外れた後に削る |
| public backbone query 旧名 | 残存 | viewer/public API が読む | saved graph backed API へ rename できる段階で整理 |
| `bb2` namespace/name | 残存 | v1 がまだ同居している | v1/recalc/support-layout 本流依存削除後に mainline 名へ rename |
| viewer support-layout inspection helper overloads | 削除済み | selected SupportLayout panel が neutral output 化済み | `SupportLayoutEndpointView` / `LoweredSupportGroupInspectionView` 用 helper を戻さない |

## 次に消せる family

優先順:

1. old tests の support-layout authority/seed/projection family を v1 専用へ隔離し、必要制約だけ bb2 direct outputs へ移植する。
2. recalc persistence family のうち、viewer-visible output 制約だけを bb2 direct derive / support group / draw へ移植する。
3. viewer/public query から旧名 API を外せる状態になってから mainline rename を検討する。

## 運用ルール

* この map は現行判断用。古くなった snapshot は archive へ移す。
* `redesign.md` には詳細表を戻さない。active contract は短く保つ。
* 新しい一覧を作る場合は、削除可否、caller、次の切断先を必ず含める。
* C 番号や commit log をこの map に増やさない。
| C250-C256 / C264-C265 | branch/cross/corner lowering と grouped support identity が inspection/refresh 後も維持される | `SupportLayoutInspectionView` endpoint / lowered support group / old relation fields を読む | 退役済み。bb2 では support group、layout endpoint、geom/draw output、saved graph frontier、direct derive determinism で見る。旧 inspection field shape と recalc refresh surface は移植しない |
| C260-C263 / C267-C268 | support-layout cache validation、authoritative support-group inspection、non-lowered span の lowered group 非継承 | mutated `SpanSupportLayoutEntry` / support group authority cache / `SupportLayoutInspectionView.lowered_support_groups` を読む | 退役済み。bb2 では support group と `SpanLayoutEntry` / geom / draw の生成結果で見る。旧 cache completeness / endpoint projection validation は移植しない |
| C205 / C233 / C238 / C272-C273 / C275-C276 | refresh 後も lowering origin、side/orientation、pair-based support identity が維持される | `SetPoleManualYawOverride` 後に `SupportLayoutInspectionView` endpoint / lowered group fields を読む | 退役済み。bb2 では support group、intent、`SpanLayoutEntry`、geom/draw、direct derive determinism で見る。旧 refresh/recalc endpoint field surface は移植しない |
