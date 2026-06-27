# backbone legacy map

この文書は、v1 / recalc / support-layout / 旧テストの現行依存を整理する地図。
履歴や milestone log ではない。現在の設計契約は `redesign.md`、退避済み履歴は
`docs/archive/redesign_history_2026-06.md` を読む。

## 分類

| 分類 | 判断 |
|---|---|
| A | bb2 本流から未使用。物理削除候補 |
| B | viewer / public query が読む。先に neutral output へ移す |
| C | tests だけが読む。制約を bb2 構造へ移植してから退役 |
| D | v1 専用として隔離 |
| E | bb2 未対応 scenario のため残る。supported 化か unsupported 固定を決める |

## 現行依存

| family / surface | 現在の caller | 分類 | bb2 generation 中に読むか | 次の切断先 |
|---|---|---|---|---|
| `core/src/generation/bb2` | bb2 generation / direct derive | mainline | yes | v1 / recalc / materialization を読まない状態を維持 |
| `SpanLayoutRules` / `SpanLayoutEntry` / `SpanLayoutState` | bb2、viewer normal/debug、public inspection | mainline | yes | support-layout authority/seed/projection を戻さない |
| saved backbone graph | bb2、viewer/query、frontier、route query | mainline | yes | topology authority。span/layout/seed/curve/port position から復元しない |
| `BuildSavedBackboneResult` / `BuildBackboneEdges` / `FindBackboneRoute` | public query、viewer、tests | B | no v1 fallback | saved graph backed query として維持。旧名 rename は mainline 化後 |
| `span_layout_types.hpp` の endpoint/support-group decision data | neutral layout/support group data と validation | mainline | no fallback path | 旧 authority object は削除済み。decision / placement として扱う |
| `ValidateFast` の support-group projection checks | viewer validation panel、tests | D/E | no | normal path blocker にしない。必要制約だけ neutral validation へ移す |
| `DirtyBits` / span runtime dirty marking | editing/runtime/viewer dirty overlay | mainline runtime | no recalc owner | mutation tracking と表示用。dirty queue/recalc ではない |
| direct derive `DeriveGeneratedSpanOutputs` | post-edit output rederive | mainline | no recalc | saved rules/ports から layout/geom/draw を再導出。recalc へ戻さない |

## 削除済み family

| family | 状態 | 戻さないもの |
|---|---|---|
| `core/src/recalc` directory | 削除済み | dirty queue / recalc rebuild / support-layout materialization |
| old `backbone_pipeline` / `bundle_spans` / grouped span generation | production 削除済み | bb2 からの wrapper / fallback |
| `CoreView::inspect_support_layout` / `SupportLayoutInspectionView` | public/viewer から削除済み | support-layout debug panel の復活 |
| `support_layout_contract` / `support_layout_projection` public accessors | 削除済み | authority/seed/projection を bb2 観測口に戻すこと |
| manual viewer `Run Legacy Recalc` | 削除済み | viewer normal/debug から recalc を起動する経路 |
| `support_orientation_utils.*` | 削除済み | validation-only helper の別ファイル化。必要分は validator 内へ局所化 |
| viewer/public inspection の `SupportLayout` entity/selection 名 | neutral `SpanLayout` 名へ置換済み | normal UI / public inspection に旧 support-layout entity 名を戻すこと |
| `support_layout_types.hpp` ファイル名 | `span_layout_types.hpp` へ置換済み | neutral layout 型を旧 support-layout ファイル名へ戻すこと |
| `ResolvedSupportAuthority` / `JunctionPairAuthority` / `support_authority` | 削除済み | default のまま保存型に残る旧 authority object |

## viewer 境界

| 領域 | 現在の状態 | 判断 |
|---|---|---|
| viewer normal path | saved graph、neutral span layout、rules、geom、draw を読む | mainline |
| selected SpanLayout debug panel | neutral span output と saved frontier を読む | mainline/debug |
| capture / replay | neutral span layout / rules / bounds / visual / render を読む | mainline/debug |
| validation panel | `ValidateFast` を読む | validation-only。bb2 generation の blocker にしない |

## 旧テスト family

旧テストは 1 件ずつ直さない。family 単位で移植または退役する。

| family | 旧 surface | 判断 |
|---|---|---|
| unsupported / duplicate state unchanged | old `Commit` / dirty queue / count checks | bb2 preflight と object/saved-graph count checks で扱う |
| output cache presence | `support_layout_projection` と recalc 後 cache | bb2 rules/layout/geom/draw と direct derive tests で扱う |
| recalc persistence | `Commit(run_recalc)` 後に meaning が痩せないこと | bb2 direct derive、support group、layout/geom/draw determinism へ移植済み。dirty queue順序は移植しない |
| support-layout authority/seed/projection | `inspect_support_layout`、authority/seed/projection fields | bb2 では SavedBackboneGraph + pair/open/row + rules/layout/support group で見る。旧 object shape は移植しない |
| attachment/socket/full insulator visual | support-layout materialization、full visual cache parts | bb2 supported scope 外。viewer-blocking になった場合だけ新 scenario として定義 |
| public backbone query | `BuildBackboneResult`, `BuildBackboneEdges`, `FindBackboneRoute` | saved graph backed query として維持。span-derived fallback は戻さない |

## 削除候補と残存理由

| 対象 | 状態 | 削除条件 |
|---|---|---|
| public backbone query 旧名 | viewer/public API が読む | bb2 が mainline 名へ rename できる段階で整理 |
| `bb2` namespace/name | v1 同居中の暫定名 | v1/recalc/support-layout 本流依存を削り切った後に mainline 名へ rename |
| dirty bits | editing/runtime/viewer が読む | recalc ではなく mutation tracking として残すか、用途別に分ける |

## 次に消せる family

1. public query 旧名を mainline API 名へ移せるか、viewer/public caller 単位で確認する。
2. dirty bits を recalc 残骸ではなく mutation tracking として残すか、用途別に分ける。

## 運用ルール

* この map は現行判断用。古くなった snapshot は archive へ移す。
* `redesign.md` には詳細表を戻さない。active contract は短く保つ。
* 新しい一覧を作る場合は、削除可否、caller、次の切断先を必ず含める。
* C 番号や commit log をこの map に増やさない。
