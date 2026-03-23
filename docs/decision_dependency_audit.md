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

- 現在の recalc は pair を補修しません。generation で入った値を support-group decision / placement へ複写するだけです。
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

一方向依存を本当に固定したいなら、次に止めるべきなのは次です。

- `down_offset_m` の初回決定が endpoint 値依存なこと
- validator が structural check だけでなく policy を持っていること

recalc の decision 補修は外し、grouped support の placement も `SupportGroupDecision` 中心に寄せました。
次の主戦場は `down_offset_m` の正本化です。
