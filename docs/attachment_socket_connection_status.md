# attachment / socket 接続状況

## 目的
この文書は、`attachment / socket` が「コード上に存在する」ことと、「DrawPath 通常利用で実際に体験できる」ことを分けて整理するためのものです。

## 前提
- 現在の主要入口は `DrawPath`
- `DrawPath -> BackboneSpec -> BackboneResult -> support layout -> detail curve` を通常経路とみなす
- `DrawPath` で入力できないものは、runtime があっても `通常入口未接続` と扱う

## 1. 実装一覧

### 1.1 正本 / runtime
- `Span.endpoint_attachment_a_id / endpoint_attachment_b_id`
- `Span.endpoint_socket_a_id / endpoint_socket_b_id`
- `Attachment`
- `AttachmentTemplate`
- `AttachmentSocketTemplate`
- `AttachmentInternalPathTemplate`

主な所在:
- [entities.hpp](/d:/GitHub/wire/core/include/wire/core/entities.hpp)
- [workflow_types.hpp](/d:/GitHub/wire/core/include/wire/core/workflow_types.hpp)

### 1.2 support layout / detail curve 解決
- attachment socket の endpoint 解決
- plain support / attachment socket / override / fallback の source 判定
- support layout から detail curve への endpoint/departure 引き渡し

主な所在:
- [recalc_pipeline.cpp](/d:/GitHub/wire/core/src/recalc/recalc_pipeline.cpp)

### 1.3 inspection / trace / override
- `SupportLayoutEndpointView.endpoint_source`
- `SupportLayoutEndpointView.attachment_input_present`
- `SupportLayoutEndpointView.socket_override_active`
- `DetailCurveInspectionView.start_endpoint_source / end_endpoint_source`
- `DetailCurveInspectionView.start_attachment_input_present / end_attachment_input_present`
- `OverrideInspectionView.endpointSocketA / endpointSocketB`
- `DecisionTraceTopic::kSupportLayoutSelection` の `AttachmentEndpointSelection`

主な所在:
- [inspection.hpp](/d:/GitHub/wire/core/include/wire/core/inspection.hpp)
- [inspection.cpp](/d:/GitHub/wire/core/src/state/inspection.cpp)
- [panels.cpp](/d:/GitHub/wire/viewer/src/panels.cpp)

### 1.4 DrawPath 入口
- `BackboneSpec.path`
- `BackboneSpec.bundles`
- `BackboneInputSpec::NodeSpec`

DrawPath 側には endpoint attachment/socket 入力はありません。

主な所在:
- [workflow_types.hpp](/d:/GitHub/wire/core/include/wire/core/workflow_types.hpp)
- [draw_path.cpp](/d:/GitHub/wire/viewer/src/draw_path.cpp)

## 2. 接続状況分類

| 要素 | 状態 | 現状 |
|---|---|---|
| attachment/socket の entity/runtime 基盤 | 実装済み | `Span` と `AttachmentTemplate` 系に保持あり |
| attachment socket endpoint 解決 | detail curve まで接続済み | support layout が解決し、detail curve が同じ endpoint を使う |
| endpoint socket override | runtime まで接続済み | 明示 override で support layout source が切り替わる |
| inspection / trace | 実装済み | endpoint source, input 有無, socket override, curve endpoint source を観測できる |
| DrawPath 通常入口からの attachment 入力 | DrawPath 未接続 | `BackboneSpec` に input 面がなく、viewer でも作っていない |
| DrawPath 通常入口からの socket input | DrawPath 未接続 | 同上 |
| DrawPath 通常利用での endpoint 振る舞い | viewer 体験で有効 | `PlainSupport` fallback と `input=none` を inspection/trace/log で観測できる |
| attachment internal path の authored 利用 | core 側基盤あり、通常入口未接続 | runtime はあるが DrawPath から authoring できない |
| support style / mirror / flow classification と attachment の統合 authoring | 実質未実装 | placeholder/別論点。DrawPath 体験には未露出 |

## 3. DrawPath 通常経路での実際の流れ

### 3.1 入力
- `DrawPath` は polyline, node reuse, bundle template を `BackboneSpec` へ入れる
- endpoint attachment/socket は入れない

根拠:
- [draw_path.cpp](/d:/GitHub/wire/viewer/src/draw_path.cpp)
- [workflow_types.hpp](/d:/GitHub/wire/core/include/wire/core/workflow_types.hpp)

### 3.2 support layout
- `build_support_layout_endpoint()` が attachment/socket を解決する器を持つ
- ただし DrawPath 通常経路では `attachment_id == invalid` かつ `socket_id < 0`
- そのため `endpoint_source = PlainSupport`

根拠:
- [recalc_pipeline.cpp](/d:/GitHub/wire/core/src/recalc/recalc_pipeline.cpp)

### 3.3 detail curve
- `make_curve_constraint_from_support_layout()` が support layout の endpoint/departure をそのまま渡す
- DrawPath 通常経路では curve も plain support endpoint を使う

根拠:
- [recalc_pipeline.cpp](/d:/GitHub/wire/core/src/recalc/recalc_pipeline.cpp)

## 4. viewer で効いていないように見える理由

理由は主に `DrawPath 未接続` です。

- `DrawPath` から attachment/socket を authoring できない
- core runtime はあるが、通常入口から値が来ない
- その結果 viewer 通常利用では `PlainSupport` fallback しか見えない
- これは「curve 反映不足」が主因ではなく、「入口から入力が来ない」が主因

## 5. DrawPath 通常経路で見えるもの
- `PlainSupport` fallback
- `input=none`
- `socket=-1`
- override があれば `AttachmentSocketOverride`
- support layout と detail curve が同じ endpoint source を見ていること

## 6. DrawPath 通常経路ではまだ見えないもの
- explicit attachment endpoint authoring
- explicit socket selection authoring
- attachment internal path authored replacement
- DrawPath からの attachment driven endpoint escape

## 7. 未接続箇所
- `BackboneSpec` に endpoint attachment/socket request がない
- `DrawPath` UI に attachment/socket authoring 面がない
- `viewer` の通常 generate path では attachment/socket request を作っていない

## 8. DrawPath へ繋ぐ最小追加案

### 段階1: 入力面
- `BackboneSpec` か `BackboneInputSpec::NodeSpec` とは別に、span endpoint ごとの attachment/socket request を持てる型を追加する
- `DrawPath` 側で「どの endpoint にどの attachment/socket を使うか」を指定できるようにする

### 段階2: 正本反映
- 生成後の `Span.endpoint_attachment_*`
- `Span.endpoint_socket_*`
へ request を落とす

### 段階3: 既存解決器の再利用
- support layout は既存の `build_support_layout_endpoint()` をそのまま使う
- detail curve は既存の `make_curve_constraint_from_support_layout()` をそのまま使う

つまり、大きく足りないのは runtime ではなく `DrawPath 入口` です。

## 9. 関連テスト
- [C159](/d:/GitHub/wire/core/tests/spec_ledger.md): endpoint attachment/socket 設定時に curve endpoint へ反映
- [C165](/d:/GitHub/wire/core/tests/spec_ledger.md): support layout と detail curve が同じ socket endpoint source を使う
- [C169](/d:/GitHub/wire/core/tests/spec_ledger.md): socket override roundtrip
- [C177](/d:/GitHub/wire/core/tests/spec_ledger.md): DrawPath 通常経路は `PlainSupport` fallback
- [C178](/d:/GitHub/wire/core/tests/spec_ledger.md): DrawPath 通常経路は `input=none` を trace できる
