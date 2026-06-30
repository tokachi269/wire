# コアモデル棚卸し（Phase4.x 設計監査）

## 1. 対象範囲
- 目的: 新機能追加や保存読込の前に、型責務を固定する。
- 優先: 振る舞い追加ではなく境界の明確化。
- 調査対象コード: `core/include/wire/core/*.hpp`, `core/src/core_state*.cpp`。

## 2. 採用レイヤモデル
```text
Definition Layer
  -> Entity Layer
    -> (read by) Operation/Workflow Layer
    -> (read by) Cache/Derived Layer

禁止する逆依存:
  Entity -> Workflow, Entity -> Cache, Definition -> Entity
```

## 3. 依存ルール（固定）
- Definition 型は runtime entity ID を参照しない。
- Entity 型は debug/session ログを所有しない。
- Cache 型は派生専用で、正本扱いしない。
- Workflow/debug 型は entity を参照してよいが、entity から依存してはいけない。
- viewer 状態を core entity struct に埋め込まない。

## 4. 型棚卸し

### 4.1 Definition Layer
| 型 | ファイル | 責務（1行） | 主依存 | 保存分類 |
|---|---|---|---|---|
| `PoleTypeDefinition` | `core/include/wire/core/entities.hpp` | 再利用可能な電柱テンプレートとして pole 本体高さと pole 上配置正本（`PortPlacementBand` / `AnchorSlotTemplate`）を定義する。 | `PoleTypeId`, placement metadata | `PersistCore` |
| `LayoutSettings` | `core/include/wire/core/core_state.hpp` | 配置補正ポリシーのパラメータを定義する。 | scalar params | `PersistCore`（設定） |
| `PathDirectionCostWeights` | `core/include/wire/core/debug_types.hpp` | 経路方向評価の重みを定義する。 | scalar params | `PersistCore`（設定） |
| 基本 enum 群（`ConnectionCategory` など） | `core/include/wire/core/entities.hpp` | テンプレート/実体/編集で使う分類語彙を定義する。 | none | `PersistCore` |

### 4.2 Entity Layer
| 型 | ファイル | 責務（1行） | 主依存 | 保存分類 |
|---|---|---|---|---|
| `Pole` | `core/include/wire/core/entities.hpp` | 支持構造の識別・姿勢・テンプレ参照を保持する。 | `Transformd`, `PoleTypeId` | `PersistCore` |
| `Port` | `core/include/wire/core/entities.hpp` | world 空間の実接続点（Auto/Manual 位置モード含む）を保持する。 | `ObjectId`, `Vec3d`, category/layer | `PersistCore` |
| `Span` | `core/include/wire/core/entities.hpp` | 2つの `Port` 間の実接続を保持する。 | `port_a_id`, `port_b_id`, `bundle_id` | `PersistCore` |
| `Anchor` | `core/include/wire/core/entities.hpp` | Span 支持に使う実支持点を保持する。 | `owner_pole_id`, `Vec3d` | `PersistCore` |
| `Bundle` | `core/include/wire/core/entities.hpp` | 複数本配線の束属性を保持する。 | `conductor_count`, `kind`, spacing | `PersistCore` |
| `Attachment` | `core/include/wire/core/entities.hpp` | Span 上 `t` 位置の付属物を保持する。 | `span_id`, `t` | `PersistCore` |
| `EditState` | `core/include/wire/core/core_state.hpp` | 編集対象 entity を ID 索引ストアで保持する。 | `ObjectStore<T>` | `PersistCore` |
| `ConnectionIndex` | `core/include/wire/core/core_state.hpp` | `Span` 接続関係（port/anchor -> span 群）の検索マップを保持する。 | entity IDs | `PersistCore`（再構築可能） |
| `RelationIndex` | `core/include/wire/core/core_state.hpp` | 所有/所属関係（pole -> ports/anchors, bundle -> spans）の検索マップを保持する。 | entity IDs | `PersistCore`（再構築可能） |
| `IdGenerator` | `core/include/wire/core/id.hpp` | 単調増加の一意 ID を発行する。 | none | `PersistCore` |
| `make_display_id` + 表示IDカウンタ | `core/include/wire/core/id.hpp` / `core_state.hpp` | 接頭辞付き表示 ID を生成する。 | prefix + counter | `PersistCore` |

### 4.3 Operation/Workflow Layer
| 型 | ファイル | 責務（1行） | 主依存 | 保存分類 |
|---|---|---|---|---|
| `BackboneInputSpec`, `BackboneGenerationConstraints`, `BackbonePolePlacementOptions`, `BackboneBundleSpec`, `BackboneSpec` | `core/include/wire/core/workflow_types.hpp` | 道路概念に依存しない backbone 指向の生成入力を定義する。 | path + bundles[] + constraints + mode | `SessionDebug` |
| `BundleTemplate` | `core/include/wire/core/workflow_types.hpp` | 束テンプレ（固定本数/可変範囲、既定layer、mirror許可）と、関連 `CableTemplate` / `PoleTypeDefinition` 参照を定義する。 | template id + count rules + related refs | `PersistCore` |
| `BackboneResult` | `core/include/wire/core/workflow_types.hpp` | 入力仕様から分離した骨格グラフ結果を保持する。 | `BackboneEdge[]` | `DerivedCache` |
| `ConductorGroupSpec` | `core/include/wire/core/workflow_types.hpp` | 束生成1回分の意図（カテゴリ/本数/種別）を定義する。 | category/count/kind | `SessionDebug` |
| `ConductorLaneId`, `ConductorGroupState` | `core/include/wire/core/workflow_types.hpp` | 束生成時の lane 順序管理（内部）を保持する。 | bundle ref + lane order | `SessionDebug` |
| `ChangeSet`, `EditResult<T>` | `core/include/wire/core/core_state.hpp` | 操作結果の外部観測差分を返す。 | entity IDs + error | `SessionDebug` |
| `GenerateBundleFromPathResult`（`GenerateFromBackboneSpec` の戻り） | `core/include/wire/core/core_state.hpp` | backbone 生成結果の生成ID群を定義する。 | generated bundle/span/pole IDs | `SessionDebug` |
| `PathDirectionCostBreakdown`, `PathDirectionEvaluationDebug` | `core/include/wire/core/debug_types.hpp` | 経路方向評価の診断内訳を保持する。 | scoring values | `SessionDebug` |
| 配置候補診断レコード群 | `core/include/wire/core/debug_types.hpp` | 配置候補の選定診断とスコア内訳を保持する。 | score fields + IDs | `SessionDebug` |
| `ValidationIssue`, `ValidationResult` | `core/include/wire/core/core_state.hpp` | 構造妥当性と診断を返す。 | code/message/object_id | `SessionDebug` |

### 4.4 Cache/Derived Layer
| 型 | ファイル | 責務（1行） | 主依存 | 保存分類 |
|---|---|---|---|---|
| `SpanRuntimeState` | `core/include/wire/core/core_state.hpp` | span 単位の dirty/version 追随状態を保持する。 | span ID + versions | `DerivedCache` |
| `UpdateKind`, `UpdatePlan` | `core/include/wire/core/core_runtime_types.hpp` | bb2 の粗い再導出境界を定義する。 | update kind + affected IDs | `SessionDebug` |
| `DirtyBits` / `SpanRuntimeState` | `core/include/wire/core/core_runtime_types.hpp` | mutation tracking と viewer dirty overlay 用の span runtime 状態を保持する。 | span ID + dirty flags | `DerivedCache` |
| `GeometrySettings` | `core/include/wire/core/core_state.hpp` | 曲線生成設定を保持する。 | scalar params | `PersistCore`（設定） |
| `CurveCacheEntry`, `CurveCache` | `core/include/wire/core/core_state.hpp` | span 曲線サンプルを保持する。 | span IDs -> points | `DerivedCache` |
| `BoundsCacheEntry`, `BoundsCache` | `core/include/wire/core/core_state.hpp` | span/区間 AABB を保持する。 | span IDs -> AABBs | `DerivedCache` |
| `LayoutEndpoint`, `SpanLayoutEntry`, `SpanLayoutCache` | `core/include/wire/core/span_layout_types.hpp` | span layout の派生ビューと span 単位の参照キーを保持する。 | span ID / endpoint decision / group key | `DerivedCache` |
| `SupportGroupDecision` | `core/include/wire/core/span_layout_types.hpp` | support group の placement decision を保持する。 | support group key + placement fields | `DerivedCache` |
| `LoweredSupportGroupPlacement` | `core/include/wire/core/span_layout_types.hpp` | support group の配置出力を保持する。 | support group key -> placement | `DerivedCache` |
| `CacheState` | `core/include/wire/core/core_state.hpp` | 派生キャッシュ群を保持する。 | cache structs | mixed（設定 + 派生） |

## 5. 責務混在の検出（分割候補）
| 現状型/フィールド | 混在理由 | 分離方向 |
|---|---|---|
| `Pole::context` (`PoleContextInfo`) | 経路解析ヒント（workflow）を entity に持っている。 | `pole_id` キーの workflow テーブルへ移動。必要最小限だけ永続化。 |
| `Port` の `template_layer/side/role` と補正フラグ | テンプレ由来情報/生成診断と実接続点が同居。現状は `ApplyPoleType(...)` がこの metadata を使って auto port を再投影する。 | `PortPlacementMeta`（永続）と `PortPlacementDebug`（セッション）へ分割。 |
| `Pole`/`Span` 内 `GenerationMeta` | セッション順序/由来と永続トポロジが同居。 | `generated/source` は必要時のみ残し、順序情報は workflow 側へ移す。 |
| `CacheState` が設定と派生を同居 | 設定寿命とキャッシュ寿命が異なる。 | `GenerationSettingsState` と `DerivedCacheState` 分割。 |
| `ConnectionIndex` / `RelationIndex` の分類 | 再構築可能だが高速参照のため正本近傍にある。 | 当面は維持し、Phase5 serializer で再構築可能扱いを明示。 |

## 6. 命名整理（固定語彙）
- テンプレ配置ヒント: 定義側の候補情報
- `port`: runtime 接続点（`Port`）
- `span`: runtime 接続エッジ
- `bundle`: span 群を束ねる正本単位
- `lane`: workflow/debug 専用シーケンス概念（entity ID ではない）
- `road/path`: workflow 入力（UI は DrawPath 表示可）
- `debug record`: セッション診断（配置候補診断、`PathDirectionEvaluationDebug`）

## 7. Persist/Derived/Session 方針（Phase5準備）

### 7.1 基本分類
- `PersistCore`: entity ストア（`poles`, `ports`, `spans`, `anchors`, `bundles`, `attachments`）、テンプレ定義（`pole_types`）、設定（`layout`, `geometry`）、ID 状態（`next_id`, display counters）
- `DerivedCache`: curve/bounds cache、span layout cache、support group cache、span runtime dirty flags
- `SessionDebug`: 配置候補/path/lane 診断、直近デバッグスナップショット、操作一時入出力

### 7.2 保存しない対象（固定）
- `CurveCache` / `BoundsCache`
- `SpanLayoutCache`（support group decision / placement を含む）
- `SpanRuntimeState`
- 配置候補選定の debug records
- `path_direction_debug_records_`
- `last_path_direction_debug_`

## 8. このフェーズで適用済みの最小リファクタ
- ヘッダにレイヤ境界コメントを追加し、テンプレ配置ヒントと `Port` の混同を防止。
- `CoreState` の debug/cache フィールドに session/derived 境界コメントを追加。
- mutable 更新経路を封鎖（`edit_state/cache/id_generator` の mutable 公開を廃止、`ObjectStore` mutable 実体は `CoreState` 内に限定）。
- viewer/tool 向けに read-only `CoreView` を導入。
- validation 実装を `state/state.cpp` から `validation/validator.cpp` へ分離。
- 回帰テスト追加:
  - デバッグ記録クリアで entity 状態が変わらない
  - キャッシュ再計算で entity のID/件数が変わらない

## 9. 継続着手チェック
- チーム内でテンプレ配置ヒント（定義）と `Port`（実体）の読み分けが安定している。
- debug/session データを非正本として扱えている。
- Phase5 serializer で `DerivedCache`/`SessionDebug` を捨ててもトポロジ損失がない。
- 外部向け grouped-authoring 概念は `Bundle` で統一、lane は内部 workflow/debug 限定。
