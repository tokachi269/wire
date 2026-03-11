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
- 既定では自動決定されるが、ユーザーやツールが固定したい可能性がある値。
- override フラグ/値は `Override` 正式層、または互換維持が必要な場合のみ entity 側の受け皿に置き、自動決定結果とは別に保持する。

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

### override 候補の整理

#### 既に formal override 層へ導入済み

- Pole yaw / forward 固定
- branch down offset の span 単位 override
- attachment socket 選択

#### 将来 override 候補

- branch support style の明示選択
- branch down offset の template/style 単位 override
- mirror 選択
- flow classification の例外指定

まだ override UI や編集 API を持たない項目でも、値の置き場は先に分離しておく。
「未実装だから導出と正本を同居させる」は禁止。

## 公開概念とアクセス境界

外から見せる面は、内部配列や局所補助値ではなく「意味のある概念単位」に限定する。
この節は、将来の inspector / debug / 外部利用 / mod API の基準面を固定する。

### アクセス分類

- `参照可能`
  - `EntityRef` / `View` / `CoreView` 相当の読み取り面から到達してよい。
- `readonly`
  - 値は見せるが、外部は直接変更しない。再生成で更新される。
- `直接編集可能`
  - 正本として保持し、正式な編集 API / command 経路から変更できる。
- `overrideのみ可能`
  - 自動決定結果そのものは編集しない。結果を変えたいときは明示 override を正本へ差し込む。
- `非公開`
  - 内部途中構造、局所補助値、SoA 実体など。外部 API や inspector にそのまま出さない。

### 概念ごとの公開可否

| 概念 | 層 | 外から参照 | 基本アクセス | 直接編集/override の考え方 | 外に出さないもの |
|---|---|---|---|---|---|
| `Pole` | 正本 | 可 | `readonly + 一部直接編集可能` | 位置、tilt、pole type、placement mode は直接編集。forward/yaw は `overrideのみ可能` | 再計算都合の局所補助値、内部インデックス更新手順 |
| `Junction` | 導出 | 可 | `readonly` | 編集不可。変えたい場合は backbone input / override policy 側で解く | route 探索途中の候補列、スコア途中値 |
| `BackboneEdge` | 導出 | 可 | `readonly` | 編集不可。入力 path / bundle spec / override を通して結果が変わる | 探索局所状態、順序評価の途中変数 |
| `SupportNode` | workflow/導出 | 可 | `readonly` | 直接編集しない。入力 spec または正本 pole/anchor 編集で変わる | session 限定の pick 補助や中間判定 |
| `SupportLayout` | 派生（detail 前段） | 可 | `readonly` | 編集不可。support style override や branch down offset override の受け皿候補 | grouped span の局所補助変数、port 計算途中値 |
| `Span` | 正本 | 可 | `readonly + 一部直接編集可能` | 接続関係、attachment 参照、anchor 参照は command で編集。曲線結果は編集しない | curve 制御点、visible/hidden interval の導出結果 |
| `Bundle` | 正本 | 可 | `readonly + 一部直接編集可能` | template 選択、規格値、spacing は template/policy 経由で編集。main/branch 結果は編集しない | lane debug、mirror 診断の途中値 |
| `DetailCurve` | 詳細形状派生 | 可 | `readonly` | 編集不可。continuity / sag / tangent rule override が必要なら正本 override 経由 | 制御点を source として永続化しない |
| `AttachmentEndpoint` | 正本接続 + 派生 endpoint | 可 | `readonly + 一部overrideのみ可能` | どの attachment/socket に接続するかは source/override 候補。内部 path や hidden interval は派生 readonly | internal replacement path、描画用サンプル列 |
| `Template` | 定義 | 可 | `readonly + 直接編集可能` | `PoleTypeDefinition`, `CableTemplate`, `BundleTemplate`, `AttachmentTemplate` は正式編集対象 | template 適用時の一時解決結果 |
| `Override` | 正本 | 可 | `直接編集可能` | 自動決定結果を変える正式な入口。派生値の直編集を禁止するための器 | override 評価の途中 trace 断片 |

### 主要プロパティの分類

#### Pole

- `editable`
  - `world_transform`
  - `height_m`
  - `pole_type_id`
  - `placement_mode`
  - `orientation_control.flip_180`
- `overrideable`
  - `orientation_control.manual_yaw_override`
  - `forward/yaw fixed override`
- `readonly`
  - `context`
  - 自動採用された向き
  - 関連 `SupportLayout` / 接続 span 一覧
- `hidden`
  - index 更新の内部状態
  - dirty queue への伝播実装

#### Junction / BackboneEdge

- `readonly`
  - `incidents`
  - `order`
  - `primary`
  - `flow kind`
  - `edge orientation`
- `overrideable`
  - 将来の `flow classification override`
  - 将来の `mirror override`
- `hidden`
  - 探索候補列
  - コスト比較の途中値

#### SupportNode

- `readonly`
  - `support_kind`
  - `position`
  - `pole_id`
  - `source_edge_*`
  - `bundle_modes`
- `editable`
  - なし（直接は持たない）
- `hidden`
  - pick/session 補助の局所状態

#### SupportLayout

- `readonly`
  - `flow_kind`
  - `pass_mode`
  - `origin`
  - `port_source`
  - `endpoint_world`
  - `departure_dir`
  - `local_departure_length_m`
  - `branch_down_offset_m`
  - `attachment_id`
  - `socket_id`
- `overrideable`
  - 将来の `support style override`
  - `branch down offset override`
  - `attachment socket selection override`
- `hidden`
  - support layout 生成途中の候補比較
  - grouped span の局所補助配列

#### Span / Bundle

- `editable`
  - `Span.port_a_id / port_b_id`
  - `Span.anchor_*`
  - `Span.endpoint_attachment_*`
  - `Span.endpoint_socket_*`
  - `Bundle` の template/policy 参照
- `overrideable`
  - 将来の `mirror override`
  - 将来の `support style override`
- `readonly`
  - main/branch 分類結果
  - lane / mirror の採用結果
  - render 用 reference length の結果解釈
- `hidden`
  - lane assignment の途中候補
  - mirror 評価の内部比較値

#### DetailCurve

- `readonly`
  - endpoint
  - tangent rule
  - continuity mode / reason
  - shape policy
  - visible/hidden/replacement interval
  - arc-length table
  - distance attributes
- `overrideable`
  - なし。変えたい場合は上位の continuity / sag / attachment / support override を通す
- `hidden`
  - 制御点探索の fallback 途中値
  - quality 反復の一時補助値

#### AttachmentEndpoint / Template / Override

- `AttachmentEndpoint.editable`
  - attachment 参照
  - source 側の socket 選択
- `AttachmentEndpoint.overrideable`
  - socket 選択
  - endpoint mode 選択
- `AttachmentEndpoint.readonly`
  - internal path 置換結果
  - hidden/replaced interval
- `Template.editable`
  - `PoleTypeDefinition`
  - `CableTemplate`
  - `BundleTemplate`
  - `AttachmentTemplate`
- `Override.editable`
  - pole forward
  - support style
  - branch down offset
  - mirror 選択
  - flow classification exception
  - attachment socket selection

### inspector に載せやすい項目

- `Pole`
  - transform, height, pole type, placement/orientation override 状態
- `Span`
  - endpoints, bundle, attachments, flow kind, related support layout, related detail curve
- `SupportLayout`
  - origin, endpoint, departure dir, local departure length, branch down offset
- `DetailCurve`
  - continuity, tangent rule, shape policy, length, visible/hidden/replacement interval
- `Template`
  - 各 template の定義値
- `Override`
  - 有効/無効、対象、上書き値

### mod API に公開しやすい項目

- 安定 ID で参照できる source 概念
  - `Pole`, `Span`, `Bundle`, `Template`, `Override`
- readonly derived 概念
  - `Junction`, `BackboneEdge`, `SupportLayout`, `DetailCurve`
- 決定理由を追う trace 面
  - orientation / flow / support layout / continuity / sag

### 外に出さないと決めた内部情報

- `ObjectStore` の mutable 実体
- SoA 内部配列そのもの
- `recalc_pipeline.cpp` の途中変数
- grouped span の局所 lane 候補配列
- curve 制御点生成の fallback 反復補助値
- dirty queue の push 順序や commit 内部状態
- viewer 専用の一時操作状態

### この分類によって守る境界

1. 正本と導出
- `Pole`, `Span`, `Bundle`, `Template` を source とし、`SupportLayout`, `DetailCurve`, flow/mirror 結果は readonly derived に留める。

2. override と direct edit
- 自動決定結果そのものを直編集しない。
- 結果を変えたい場合は、source の正式編集か明示 override を通す。

3. 外部 API と内部実装
- 外部は `View` / `Ref` / `Meta` 相当の意味単位だけを見る。
- 内部途中構造、SoA 配列、局所補助値は公開しない。
