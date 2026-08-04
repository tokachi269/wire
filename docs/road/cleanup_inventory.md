# road整理 CLEAN0: baselineと機能分類

この文書はCLEAN0〜CLEAN10の作業台帳であり、road契約の正本ではない。CLEAN8の文書整理で削除する。

## Baseline (2026-08-04)

| 項目 | 値 |
|---|---|
| branch | `road/regenerate-restructure` |
| 最終commit | `64a25951` road: draw each curve interval as one arc |
| working tree | clean (未追跡の `tools/__pycache__/`、`web/dev-5174.log` のみ) |
| 未コミットUX作業 | なし |
| compiler warning | 2 (`C4834` road.cpp、CLEAN1で修正済み。baseline測定時はビルドが最新で再コンパイルされず検出できていなかった) |
| road Core tests | 54 pass |
| architecture contract tests | 27 pass |
| wire Core tests | 541 pass |
| Web tests | 144 pass |
| road-arch-lint / arch-lint / test-family-lint | pass |
| persistence | `road_graph_version=11` |

## 規模

| 対象 | 行数 |
|---|---|
| road production (`src` + `include`) | 10,405 |
| `road.cpp` | 3,610 |
| road tests | 5,865 |
| docs/road | 1,109 (README 5 / architecture 349 / operation_semantics 234 / plan 457 / supported_operations 64) |

## 公開面の規模

| 対象 | 数 |
|---|---|
| `RoadState` public method | 34 |
| road public 自由関数 | 15 |
| public request型 | 32 |
| road WASM export | 37 |
| Web bridge road method | 32 |
| **うちWeb actionsが実際に呼ぶもの** | **18** |
| `SavedRoadGraph` field | 13 |
| generation関数 | 10 |

Web bridgeの32件中14件はweb/src内に呼び出し元がない。

## 分類表

### 削除 (CLEAN1: 消費者のない道路沿い配置API)

| 対象 | コード位置 | consumer | 理由 |
|---|---|---|---|
| `RepeatingPlacementPolicy` / `RoadSideRef` / `CorridorSideRef` / `RoadSideIntervalRef` / `ResolveCorridorSideRef` / `ResolveRoadSidePosition` / `DeriveRepeatingPlacementDistances` | `common_types.hpp` 4, `road.hpp` 6, `road.cpp` 11, contract tests 17, `docs/road/architecture.md` 3 | なし | 街路樹・標識・建物・propのconsumerが未実装。WASM/Web露出もゼロ |

計41箇所5ファイル。Web/WASMに露出していないため削除の影響は core とテストに閉じる。

### 削除 (CLEAN2: 旧ADD LANE preview)

| 対象 | コード位置 | consumer | 理由 |
|---|---|---|---|
| `previewAddLane` / `preview_add_lane` / `roadPreviewAddLane` | `bindings.cpp` 2, `bridge/wire.ts` 2, `bridge/wasm.ts` 1, `actions.test.ts` 3, `wasm.test.ts` 1, `road_arch_lint.py` 2 | **なし** (web/srcに呼び出し元ゼロ) | pointer移動ごとに state複製→全処理→`generate_road`→mesh返却する経路。現行UIは軽量guideを使う |

計11箇所6ファイル。

### 内部化 (CLEAN3: 低レベル公開操作)

| 対象 | 公開面 | Web consumer | 判断 |
|---|---|---|---|
| `AddLaneConnection` / `AddBoundaryContinuation` | public method + request型 | なし (WASM export無し) | 内部化。ADD LANE・junctionが内部で使う処理はprivate helperへ |
| `AddTransition` / `AttachSectionTransition` / `AddTransitionToSegment` | public method + request型、WASM `applyTransition` | なし | 内部化。`applyTransition` は `AddTransitionToSegment` を呼ぶだけ |
| `SetApproachSetbackOverride` / `SetApproachLateralShiftOverride` / `ResetApproachOverrideField` / `ResetAllApproachOverrides` | public method + WASM 4件 | なし | raw `ApproachKey` を直接受ける。内部化 |
| `SetBoundaryMarkingPolicy` / `ResetBoundaryMarkingPolicy` / `SetLaneSideMarkingPolicy` / `ResetLaneSideMarkingPolicy` | public method + WASM 4件 | なし | raw boundary/band IDを直接受ける。内部化 |
| `SuppressAutoMarking` / `ResetAutoMarkingSuppression` / `SetJunctionMarkingOverride` / `DeleteJunctionMarkingOverride` | public method + WASM 6件 | なし | marking overrideの低レベル操作。内部化 |

### 保留 (CLEAN4)

| 対象 | 保存されるか | コード位置 | 停止条件該当 |
|---|---|---|---|
| 手動線 `ManualLineMarking` | **保存される** (`SavedRoadGraph.manual_lines`, archive v11) | 110箇所15ファイル(手動面と合算) | **要判断** |
| 手動面 `ManualAreaMarking` | **保存される** (`SavedRoadGraph.manual_areas`, archive v11) | 同上 | **要判断** |
| 部分削除 `DeleteSegmentRange` | 保存されない(操作のみ) | `road.cpp`, WASM, bridge。web consumerなし | 削除可 |
| raw lane topology編集 | **保存される** (`lane_connections`, `boundary_continuations`, archive v11) | 169箇所19ファイル(部分削除と合算) | **要判断**。ただしADD LANE・junctionが内部で使用 |

**CLEAN4到達時に停止して報告する項目**: 手動線・手動面・`lane_connections`・`boundary_continuations` は archive v11 に保存されている。既存ユーザーデータ(IndexedDB workspace)の互換性が必要かを確認するまでschemaを削除しない。`lane_connections` / `boundary_continuations` はADD LANEとjunctionが内部利用するため、**保存型としては残し、公開操作だけを内部化する**可能性が高い。

### CLEAN5判定対象: Branch / Merge

| 対象 | コード位置 | 状態 |
|---|---|---|
| `AddConnectedLaneSegment` + `branch-lane` / `merge-lane` UI | 51箇所13ファイル。WASM 5、bridge 4、actions 2、road_tests 18 | web actionsから実際に呼ばれている(`roadAddConnectedLaneSegment`、`roadPreviewConnectedLaneSegment`)。POC条件の実測が必要 |

### 本流 (残す)

`RoadNode` / `RoadSegment` / `RoadCorridor`、`AddSegment` / `AddSegmentConnectedTo` / `AddSegmentConnectedToSegment` / `ExtendCorridorFromEnd` / `PreviewDrawnInterval`、`EditSegmentShape` / `MoveNode` / `DeleteSegment` / `SplitSegmentAtDistance`、`AddSectionTemplate` / `EditSectionTemplate`(共有テンプレート編集)、`AddLane`、`Save` / `Load`、`ValidateGraphInvariants`、断面テンプレート4種、generation 10関数。

## generation工程 (CLEAN6監査対象)

`generate_road` / `derive_node_incidence` / `derive_segment_shapes` / `derive_segment_sections` / `derive_topology_paths` / `derive_segment_lane_paths` / `resolve_connections` / `resolve_connection_geometry` / `derive_markings` / `emit_geometry`

工程数は削減目標にしない。CLEAN6で各工程の consumer を確認し、削除機能専用の分岐だけを消す。

## 進捗

- [x] CLEAN0 baseline + 分類表
- [x] CLEAN1 配置API削除 — production/正本docsから41箇所を削除。contract testは`corridor_sample_distances`へ置換して距離↔世界座標の契約を維持
- [x] CLEAN2 旧ADD LANE preview削除 — WASM export・bridge・型・mock・testを削除。lintは維持規則から復活禁止規則へ変更
- [~] CLEAN3 低レベル操作内部化 — **Web/WASM境界は完了**(低レベル15 exportとbridgeを削除、Webから直接呼べない)。**C++ public method の内部化は未了**(下記参照)

### CLEAN3b 残作業: C++ public methodの内部化

Webから外した15操作に対応する `RoadState` public methodは残っている。内部化には呼び出しテスト9関数の書き換えが必要で、うち次はAddLaneで表現できるか未確認。

| test | 使用API | 論点 |
|---|---|---|
| `M3_lane_count_change_begins_and_terminates_lines` | AddTransition | 2→3車線をAddLaneで作れるか |
| `M6_transition_without_boundary_mapping_is_unsupported` | AddTransition | AddLane経由では到達不能なら検証対象が消える |
| `P2_supports_taper_lane_reduction_and_median_end` | AddTransition | 車線減少・中央帯終端をAddLaneで作れるか |
| `P2_requires_transition_for_mixed_section_connection` | AddTransition | 異断面接続 |
| `P2_section_transition_and_manual_markings` | AddTransition | 手動markingはCLEAN4保留対象 |
| `junction_movements_are_explicit_lane_connections` | AddLaneConnection | 曖昧junctionでの明示指定は`plan_unique_junction_topology`の逃げ道 |
| `add_lane_connects_the_only_matching_junction_approach` | AddLaneConnection | 同上 |
| `derived_segment_owns_all_semantic_distances` | AddTransitionToSegment | setup用途 |
| `all_public_operation_validation_failures_are_atomic` | 全public op列挙 | 公開面縮小に追従 |

`AddLane` は transition を1件だけ生成し(`kTaperIn` / 逆向きで `kTaperOut`)、`lane_connections` / `boundary_continuations` は生成しない(それらは `plan_unique_junction_topology` がjunctionで自動生成する)。**AddLaneが車線減少・異断面接続・曖昧junction指定を表現できない場合、低レベルAPIの削除は高水準operationの機能欠落になる**(停止条件「低レベル公開APIを消すと高水準operationが成立しない」)。
- [~] CLEAN4 保留機能整理 — 手動線・手動面・部分削除を標準UI/Web/WASM境界から除去し `backlog.md` へ。保存fieldは維持(schema不変、migration不要)。**C++ public API の除去は CLEAN3b と同時に行う**
- [x] CLEAN5 Branch/Merge判定 — **POC条件を満たさないため削除**。RoadPanelが raw BoundaryId の `<select>` を出し(`RoadPanel.svelte:143`)、`boundaries[0]` を既定値として自動選択していた(`road_actions.ts:434`)。goal §5.1 の「raw BoundaryIdを入力させない」「不正な初期値を自動選択しない」の2点に違反。UI/state/actions/bridge/WASM/`AddConnectedLaneSegment`/専用request/test/docsを削除し、lintへ復活禁止規則を追加。junctionとADD LANEが使う内部lane topologyは維持
- [~] CLEAN6 パイプライン・保存型整理 — 生産者のない `OperationPlan` field 3件(`remove_lane_connections` / `remove_boundary_continuations` / `add_connection_policy_overrides`)を削除。**generation工程のconsumer監査と保存型整理は未了**
- [~] CLEAN7 road/wire命名契約整理 — `build` 監査を実施し、派生生成に残っていた `build_boundaries` / `build_surface_styles` を `derive_*` へ改名(road production の `build` 残存は0)。**failure分類・action result・draw session契約の監査は未了**
- [~] CLEAN8 文書整理 — 削除済みAPI(AddConnectedLaneSegment / DeleteSegmentRange / AddManualLine / AddManualArea)の記述を正本docsから除去。README を索引化し supported_operations の Branch/Merge 記述を実態へ修正。**plan.md の分離とこの文書の削除は未了**
- [~] CLEAN9 ソース物理整理 — 削除後に再計測: `road.cpp` 3,610→3,361行、`RoadState` 実装34メソッド、road production 10,405→10,095行。**分割は必要**(生成・編集・断面・車線・削除が1ファイルに同居し、無関係な操作を横断して読む必要がある)。実施は未了。候補は `road_create.cpp` / `road_edit.cpp` / `road_section.cpp` / `road_lane.cpp` / `road_delete.cpp`。CLEAN3b の内部化で `RoadState` の公開面が確定してから分割する(先に分割すると内部化で再度動かすことになる)
- [~] CLEAN10 局所性・回帰 — 全回帰は green(road 50 / contract 27 / wire 541 / web 137、lint 3種、compiler warning 0、diff-check、working tree clean)。局所性testはcontract testに34箇所(`branch_and_tail_extension_preserve_existing_corridor` 等、branch/延長/split後もaffected set外のcorridor距離↔世界座標が不変)。**ADD LANE・node移動・template編集・segment削除の局所性testは未追加**
