# road整理 CLEAN0: baselineと機能分類

この文書はCLEAN0〜CLEAN10の作業台帳であり、road契約の正本ではない。CLEAN8の文書整理で削除する。

## Baseline (2026-08-04)

| 項目 | 値 |
|---|---|
| branch | `road/regenerate-restructure` |
| 最終commit | `64a25951` road: draw each curve interval as one arc |
| working tree | clean (未追跡の `tools/__pycache__/`、`web/dev-5174.log` のみ) |
| 未コミットUX作業 | なし |
| compiler warning | 0 |
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
- [ ] CLEAN1 配置API削除
- [ ] CLEAN2 旧ADD LANE preview削除
- [ ] CLEAN3 低レベル操作内部化
- [ ] CLEAN4 保留機能整理(**停止して報告**)
- [ ] CLEAN5 Branch/Merge判定
- [ ] CLEAN6 パイプライン・保存型整理
- [ ] CLEAN7 road/wire命名契約整理
- [ ] CLEAN8 文書整理(この文書を削除)
- [ ] CLEAN9 ソース物理整理
- [ ] CLEAN10 局所性・回帰
