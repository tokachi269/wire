# backbone legacy map

この文書は、v1 / recalc / support-layout / 旧テストの現行依存を整理する地図。
履歴や milestone log ではない。現在の設計契約は `redesign.md`、退避済み履歴は
`docs/archive/redesign_history_2026-06.md` を読む。

## 分類

| 分類 | 判断 |
|---|---|
| A | backbone 本流から未使用。物理削除候補 |
| B | viewer / public query が読む。先に neutral output へ移す |
| C | tests だけが読む。制約を backbone 構造へ移植してから退役 |
| D | v1 専用として隔離 |
| E | backbone 未対応 scenario のため残る。supported 化か unsupported 固定を決める |

## 現行依存

| family / surface | 現在の caller | 分類 | backbone generation 中に読むか | 次の切断先 |
|---|---|---|---|---|
| `core/src/generation/backbone` | backbone generation / direct derive | mainline | yes | v1 / recalc / materialization を読まない状態を維持 |
| `SpanLayoutRules` / `SpanLayoutEntry` / `SpanLayoutState` | backbone、viewer normal/debug、public inspection | mainline | yes | support-layout authority/seed/projection を戻さない |
| saved backbone graph | backbone、viewer/query、frontier、route query | mainline | yes | topology authority。span/layout/seed/curve/port position から復元しない |
| `SavedBackboneResult` / `SavedBackboneEdges` / `FindSavedBackboneRoute` | public query、viewer、tests | mainline | no v1 fallback | saved graph backed query として維持 |
| `span_layout_types.hpp` の endpoint/support-group decision data | neutral layout/support group data と validation | mainline | no fallback path | 旧 authority object は削除済み。decision / placement として扱う |
| `ValidateFast` の support-group layout checks | viewer validation panel、tests | validation-only | no | normal path blocker にしない。neutral layout/support-group 制約として維持 |
| coarse update boundary / `UpdateKind` | editing/runtime/direct derive | mainline runtime | no recalc owner | kRegenerate/kReposition/kReshape/kRedraw の4分類だけ。operation-specific dirty enum を増やさない |
| direct derive `DeriveGeneratedSpanOutputs` | post-edit output rederive | mainline | no recalc | saved rules/ports から layout/geom/draw を再導出。recalc へ戻さない |
| test family manifest / architecture lint | core/viewer test build | merge guard | no | unclassified tests/files、禁止依存、旧 recalc/support-layout、city-domain identity の復活を fail する |
| manual topology API (`AddConnectionByPole` / `AddDrop*` / `SplitSpan`) | public API boundary、retired test definitions | E | no | normal/default caller は除去済み。SavedGraph mutation 仕様がないため mutation 前 `unsupported` |

## 削除済み family

| family | 状態 | 戻さないもの |
|---|---|---|
| `core/src/recalc` directory | 削除済み | dirty queue / recalc rebuild / support-layout materialization |
| old `backbone_pipeline` / `bundle_spans` / grouped span generation | production 削除済み | backbone からの wrapper / fallback |
| `CoreView::inspect_support_layout` / `SupportLayoutInspectionView` | public/viewer から削除済み | support-layout debug panel の復活 |
| `support_layout_contract` / `support_layout_projection` public accessors | 削除済み | authority/seed/projection を backbone 観測口に戻すこと |
| manual viewer `Run Legacy Recalc` | 削除済み | viewer normal/debug から recalc を起動する経路 |
| `support_orientation_utils.*` | 削除済み | validation-only helper の別ファイル化。必要分は validator 内へ局所化 |
| viewer/public inspection の `SupportLayout` entity/selection 名 | neutral `SpanLayout` 名へ置換済み | normal UI / public inspection に旧 support-layout entity 名を戻すこと |
| `support_layout_types.hpp` ファイル名 | `span_layout_types.hpp` へ置換済み | neutral layout 型を旧 support-layout ファイル名へ戻すこと |
| `ResolvedSupportAuthority` / `JunctionPairAuthority` / `support_authority` | 削除済み | default のまま保存型に残る旧 authority object |
| `BuildBackboneResult` / `BuildBackboneEdges` / `FindBackboneRoute` public query 旧名 | 削除済み | span-derived fallback 名や build 名の public query を戻すこと |
| `GeneratePolesAlongRoad` / `GenerateSpansBetweenPoles` / `GenerateSimpleLine*` | 削除済み | backbone を包む旧 road-generation API や別 result 型 |
| `RoadSegment` / `RoadId` core workflow type | 削除済み | external road modelをwire入力型として戻すこと |
| building-specific core support/pick enum | opaque `kExternal` へ置換済み | external adapter の対象種別を wire core に戻すこと |
| `wire_capture_replay` / replay scripts | 削除済み | legacy topology API をdebug tool都合で戻すこと |
| `regeneration_required` / `TemplateDependencyState` | 削除済み | executorのないmarker-only成功を戻すこと |
| `DirtyBits` / `dirty_span_ids` / viewer dirty overlay | 削除済み | marker-only updateや新しいdirty queueを戻すこと |
| manual topology API implementations | 削除済み | graph 外で port/span を生成する `AddConnectionByPole` / `AddDrop*` / `SplitSpan` 実装 |
| manual topology allocator test family | 登録解除 | port allocator、drop split、template override の旧 object shape を backbone 要件へ戻すこと |
| `build_backbone_service.cpp` | 削除済み | pole-only generation と旧 auto-connect allocator |
| `RegenerateSessionAutoParts` / `rebuild_from_backbone.cpp` | 削除済み | generated span の session 走査、snapshot rollback、出力からの identity 復元 |
| `last_generation_junction_relations` | 削除済み | SavedBackboneGraph junction と競合する inspection fallback |

## viewer 境界

| 領域 | 現在の状態 | 判断 |
|---|---|---|
| viewer normal path | saved graph、neutral span layout、rules、geom、draw を読む | mainline |
| selected SpanLayout debug panel | neutral span output と saved frontier を読む | mainline/debug |
| validation panel | `ValidateFast` を読む | validation-only。backbone generation の blocker にしない |

## 旧テスト family

旧テストは 1 件ずつ直さない。family 単位で移植または退役する。

| family | 旧 surface | 判断 |
|---|---|---|
| unsupported / duplicate state unchanged | old `Commit` / dirty queue / count checks | backbone preflight と object/saved-graph count checks で扱う |
| output cache presence | `support_layout_projection` と recalc 後 cache | backbone rules/layout/geom/draw と direct derive tests で扱う |
| recalc persistence | `Commit(run_recalc)` 後に meaning が痩せないこと | backbone direct derive、support group、layout/geom/draw determinism へ移植済み。dirty queue順序は移植しない |
| support-layout authority/seed/projection | `inspect_support_layout`、authority/seed/projection fields | backbone では SavedBackboneGraph + pair/open/row + rules/layout/support group で見る。旧 object shape は移植しない |
| attachment/socket/full insulator visual | support-layout materialization、full visual cache parts | backbone supported scope 外。viewer-blocking になった場合だけ新 scenario として定義 |
| public backbone query | `SavedBackboneResult`, `SavedBackboneEdges`, `FindSavedBackboneRoute` | saved graph backed query として維持。span-derived fallback は戻さない |
| session regeneration | generated span/pole の session id と cleanup 順序 | 旧 family を退役。post-edit は coarse update plan + direct derive、構造変更は明示 regenerate/unsupported |
| junction relation / constrained solver snapshot | relation snapshot、through-pair score、solver/order decision debug | 旧 family を退役。connectivity は pair/open/row、placement は support group、観測は saved graph/layout |
| pole-global support axis | `pole.context`、preserved trunk pair、ConnectedDirectionFit、global yaw | 旧 family を退役。row axis は connectivity、pole yaw は表示用従属値 |
| junction session priority | `prioritized_session_id`、generation session由来のprimary/order | 旧 family を退役。public junction はSavedGraph adjacency summaryであり、connectivity authorityではない |
| fixed-count explicit override | fixed template の exact count 指定を常に reject | 旧期待を退役。backbone は exact count を no-op supported、mismatch は mutation 前 reject とする |
| guide local regeneration | 同一 BackboneSpec 再実行 no-op と延長時の末端追加 | 旧 local regeneration family を退役。duplicate は saved span binding preflight で unsupported/state unchanged とする |
| duplicate guide points | 重複 route point を成功扱いにする guide robustness | general input cleanup scenario は未採用。zero-length/不足情報は unsupported とし、新 supported scenario として定義するまで戻さない |
| state-service override/internal mutation | override inspection、owned endpoint reuse、template mutation service の旧 internal surface | 旧 service test family を退役。backbone direct derive / UpdateKind / neutral output の制約へ必要分だけ移植する |
| support-node inspection by pole id | `SupportNode` id と pole id の同一視、rebuilt backbone inspection surface | 旧 inspection family を退役。SavedBackboneGraph/public saved result を正とし、support node entity は別途 neutral API で扱う |

## 削除候補と残存理由

| 対象 | 状態 | 削除条件 |
|---|---|---|
| `pending_support_nodes` | 未保存の segment/building/pole pick を次の backbone request へ渡す transient input | generation result を保存しない。保存後の node は SavedBackboneGraph だけを読む |
| manual topology API declarations | public surface は mutation 前 `unsupported` | replacement operation の SavedGraph mutation 契約が決まるまで削除候補 |

## unsupported 境界

| 対象 | 現在の扱い | 判定 |
|---|---|---|
| malformed path/node mode/template/count/layer/band | `prepare` / `check` で mutation 前 reject | PASS |
| SavedBackboneGraph の無い existing node/scene | `prepare` で `unsupported` | PASS |
| ambiguous/missing port binding、duplicate edge bundle/span lane | preflight で `unsupported` | PASS |
| zero-length/ambiguous connectivity、解けない avoid constraints | `check` / `pairs make` で `unsupported` | PASS |
| route-local regenerate | update executor が `unsupported` | PASS |
| full migration、general routing、preview/undo、本格 attachment visual | public operation 自体を提供しない | scope 外 |
| layout/variation/context と bundle/cable/attachment decision の post-edit 更新 | generated outputがある場合はmutation前 `unsupported` | PASS |
| cable sag/color、geometry、visual の安全な post-edit 更新 | `kReshape` / `kRedraw` でdirect derive | PASS |
| pole type definition / instance reapply | context-only band編集とactive endpoint更新が未分離 | FAIL: merge blocker |
| manual topology API family | normal/default caller なし。public call は mutation 前 reject | PASS |

## 次に消せる family

`AddConnectionByPole` / `AddDrop*` / `SplitSpan` の caller は一括分類済み。
test setup は backbone fixture または topology 非依存の unit fixture へ移し、旧 allocator shape のケースは family 退役した。
production internal caller はなく、public surface は SavedGraph mutation 仕様が決まるまで mutation 前 `unsupported` とする。

`SegmentLaneAssignment` / `last_generation_lane_assignments` / 旧 generation test suite は退役済み。
viewer capture は current span、neutral layout、saved binding、decision trace を直接読む。
`last_generation_support_nodes` の generation snapshot 用途も退役済み。
未保存 pick だけを `pending_support_nodes` に保持し、backbone generation result は SavedBackboneGraph にのみ保存する。

## 運用ルール

* この map は現行判断用。古くなった snapshot は archive へ移す。
* `redesign.md` には詳細表を戻さない。active contract は短く保つ。
* 新しい一覧を作る場合は、削除可否、caller、次の切断先を必ず含める。
* C 番号や commit log をこの map に増やさない。
