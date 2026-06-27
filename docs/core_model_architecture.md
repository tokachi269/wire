# コアモデル設計方針

## 目的
`core` の責務境界を固定し、同じ意味の判定が複数層へ再発することを防ぐ。

## 基本原則
- 正本更新は `CoreState` の正規 API だけを通す。
- 後段が前段の意味を再判定しない。
- 旧経路を残したまま新経路を足さない。
- viewer/inspection は補正層ではなく read-only consumer にする。

## パイプライン（固定）

1. Input Normalization
- 役割: 入力を安定化する（node/edge/incident の整列、category 正規化）。
- 禁止: lower 判定、support 向き、group 決定、curve 形状決定。

2. Relation / Decision
- 役割: 意味を決める。
- 正本: `EndpointContinuityDecision`, `SupportGroupDecision`。
- 保持する主値: `support_group_id`, `lower_required`, `relation_kind`, `side_axis`, `chosen_side_sign`, `support_orientation_rule`, `support_orientation_basis`。
- 禁止: world 座標の support 実体化、viewer 向け補正。

3. Layout / Placement
- 役割: saved decision を endpoint placement へ変換する。
- 正本: `SpanLayoutEntry` と support group placement。
- 固定経路: `SpanLayoutRules -> SupportGroupDecision -> SpanLayoutEntry`。
- 禁止: `support_group_id` 再計算、`lower_required` 再判定、orientation 再解釈。

4. Detail Curve
- 役割: support/endpoint 拘束から曲線詳細を作る。
- 出力: `SpanCurveSpec`, `DetailCurve`。
- 禁止: lower/group/orientation の再判定、support の増殖。

5. Presentation / Inspection
- 役割: 表示・検査ビュー生成。
- 禁止: dedupe で意味補正、group 推定、lower 再判断、orientation 再計算。

## 現状のズレ
- `core/src/recalc` と support-layout materialization は削除済み。
- bb2 generation と direct derive は recalc / materialization / support-layout contract を読まない。
- 残る旧語は validation-only の support-group consistency check と public query 旧名が中心。
- viewer normal path は neutral span output と saved graph を読む。

## lowered support の設計固定
- group identity は `owner_pole_id + support_group_id`。
- `support_group_id` は decision 正本フィールドとして保持し、downstream で再導出しない。
- one owner pole + one support group = one authoritative placement。
- grouped lowered support は non-radial basis を必須とする。
- T/cross/branch kind は正本にしない。pair/open/row と support group で表す。

## データ境界
- Definition: テンプレート/静的定義。
- Entity: `Pole / Port / Anchor / Bundle / Span / Attachment`。
- Workflow: 入力 spec と決定結果。
- Cache/Debug: 再構築可能な派生（span layout / curve / bounds / draw / inspection 用 view）。

禁止する逆依存:
- Definition -> Entity
- Entity -> Workflow
- Entity -> Cache/Debug

## 変更時チェックリスト
- どの段の正本を更新したかを明示したか。
- 後段で同じ意味を再判定していないか。
- geom / draw が decision/layout recompute を抱え込んでいないか。
- 旧経路を削除したか（未使用で放置していないか）。
- validator/test で不変条件を固定したか。
- direct derive 前後で意味が不変かを確認したか。

## 必須不変条件（最低限）
- `support_group_id` は authoritative field で downstream 再計算しない。
- grouped lowered support は radial basis を使わない。
- 同一 support group で side/orientation basis が一致する。
- direct derive 前後で次が不変:
  - `support_group_id`
  - `support_world`, `mount_world`, `tip_world`
  - `support_orientation_rule`, `support_orientation_basis`
  - `side_axis`, `chosen_side_sign`
  - `lower_required`, `default_lower_required`, `relation_kind`
