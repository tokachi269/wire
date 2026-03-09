# コアモデル設計方針

## 目的
この文書は `core` の設計ルールを固定し、機能追加時に隠れた更新経路やレイヤ違反が再発しないようにするためのものです。

## レイヤモデル

1. `Definition`
- テンプレート/静的定義。
- 例: `PoleTypeDefinition`, `PortSlotTemplate`, `AnchorSlotTemplate`。

2. `Entity`（永続正本）
- 実行時の正本ネットワーク実体。
- 例: `Pole`, `Port`, `Anchor`, `Bundle`, `Span`, `Attachment`。

3. `Workflow`（操作入力/出力、非正本）
- 生成/編集要求や一時的な計画構造。
- 例: `RoadSegment`, `BackboneInputSpec`, `BackboneSpec`, `ConductorGroupSpec`。

4. `Cache/Debug`（派生/セッション）
- 再構築可能なキャッシュと診断情報。
- 例: `CurveCache`, `BoundsCache`, `DirtyQueue`, `SpanRuntimeState`, path/slot デバッグ記録。

## 依存ルール（固定）

- `Definition` は `Entity` に依存しない。
- `Entity` は `Workflow` / `Debug` に依存しない。
- `Workflow` は `Definition` / `Entity` を参照してよいが、`Entity` を直接更新してはいけない。
- `Cache/Debug` は `Entity` を参照してよいが、正本になってはいけない。
- viewer/tool の状態は entity モデルに入れない。

## 変更ルール（固定）

- 正本更新は必ず `CoreState` の編集 API を通す。
- 外部から内部ストアへの mutable 参照は公開しない。
- `ObjectStore` の mutable 実体（`items_mutable`）は `CoreState` 内部に限定する。
- viewer/tool からのストア直接変更は設計上禁止する。

## 不変条件（最低限）

1. 参照整合
- `Span.port_a_id` / `port_b_id` は既存 `Port` を指す。
- `Port.owner_pole_id` が設定されている場合は既存 `Pole` を指す。
- `Span.anchor_*` が設定されている場合は既存 `Anchor` を指す。

2. Bundle/Span 整合
- `Span.bundle_id` が設定されている場合、参照先 `Bundle` が存在する。
- 複数本配線は同一 `bundle_id` を共有する複数 `Span` で表現する。
- Bundle本数は `BundleTemplate` で決定する（固定テンプレは上書き不可、可変テンプレは範囲制約内のみ）。
- lane は workflow/debug メタデータであり、entity 不変条件には含めない。

3. インデックス/キャッシュ整合
- `ConnectionIndex` は `Span` 関係と一致する。
- `RelationIndex` は `Pole/Bundle` との所有・所属関係と一致する。
- 全 `Span` に `SpanRuntimeState` が存在し、ダングリング状態を残さない。
- index 更新は編集 API の内部で `index_add/index_remove` を共通利用し、個別ロジックの重複を作らない。

4. Manual/Auto 優先
- `Pole.placement_mode=Manual` はユーザー意図配置（再生成時に尊重）。
- `Port.position_mode=Manual` は自動上書きしない。
- `Auto` は再生成/再投影対象になり得る。

## 編集優先ルール

- 手動編集は自動生成より優先。
- 再生成は Auto 部分を先に対象化し、Manual 部分は既定で保持。
- Dirty 伝播は局所（接続 span/anchor）に限定し、全体波及を避ける。

## 命名ルール

- `slot`: テンプレート候補点（runtime endpoint ではない）
- `port`: runtime 接続点
- `span`: port 間 runtime edge
- `bundle`: span 群を束ねる正本単位
- `lane`: workflow/debug 用の内部シーケンスラベル（公開 entity API には出さない）

## 自動決定 / override / 派生の境界

将来ユーザーが触り得る値でも、最初から全てを editable source にしない。
まず「正本として保持すべきか」「明示 override の正本にすべきか」「再生成可能な導出か」を固定する。

### 分類ルール

1. 正本
- ユーザーや workflow が意味として保存すべき入力/接続/所有関係。
- 再生成で消えてはいけない。

2. 明示override可能な正本
- 既定では自動決定されるが、将来ユーザーが固定したい可能性がある値。
- override フラグ/値は entity 側に置き、自動決定結果とは別に保持する。

3. 自動決定される導出結果
- Backbone / junction / template / topology から都度決め直せる結果。
- debug や workflow result に残してよいが、重い独立正本概念にはしない。

4. 詳細形状層の派生データ
- 見た目曲線、距離属性、attachment 内部経路などの再計算可能な詳細。
- 正本 entity や workflow spec に保存しない。

### 現時点の固定分類

- `Backbone / junction order / primary`: `3. 自動決定される導出結果`
  - 置き場: `workflow_types.hpp` の `JunctionInfo`, `JunctionIncident`, `BackboneResult`
- `Pole forward / yaw`: `2. 明示override可能な正本`
  - 置き場: entity 正本は `Pole.orientation_control`
  - 自動採用結果の置き場: `debug_types.hpp` の `PoleOrientationDebugRecord`
- `main / branch classification`: `3. 自動決定される導出結果`
  - 置き場: `BackboneFlowKind`, `BackboneEdgeOrientation`, `SegmentLaneAssignment`
- `main support / branch support`: `3. 自動決定される導出結果`
  - 置き場: support 配置ロジックと `PortPlacementSourceKind`
- `branch down offset`: `3. 自動決定される導出結果`
  - 置き場: support/attachment 配置計算の値。layer は書き換えない
- `mirror decision`: `3. 自動決定される導出結果`
  - 置き場: lane assignment / edge orientation debug
- `attachment endpoint / socket相当`: `1. 正本` と `4. 詳細形状層の派生データ` に分離
  - 正本: どの attachment/socket に接続しているかという接続関係
  - 派生: attachment 内部の guide path / endpoint offset / 特殊曲線
- `DetailCurve / arc-length table / distance attributes`: `4. 詳細形状層の派生データ`
  - 置き場: `detail_curve.hpp`, `CurveCacheEntry.detail`, render cache

### 将来 override 候補

- Pole yaw / forward の明示固定
- branch support style の明示選択
- branch down offset の template/style 単位 override
- attachment socket 選択

まだ override UI や編集 API を持たない項目でも、値の置き場は先に分離しておく。
「未実装だから導出と正本を同居させる」は禁止。
