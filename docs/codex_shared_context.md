# Codex共有コンテキスト（現行）

この文書は、チャットをまたいで設計意図を維持するための共有メモです。
仕様確定文書ではなく、現時点の意思決定と優先順位を明示します。

## 1. 何を作っているか
- 電柱・電線ネットワークの生成/編集/検証基盤。
- 自動生成を主役にしつつ、必要な箇所だけ手直しできる構成を目指す。
- core はエンジン非依存、viewer は検証用。

## 2. 優先順位（固定）
1. 見た目の自然さ
2. 自動生成の一貫性（手で1本ずつ作る前提にしない）
3. 正本モデルの整合（更新経路が追えること）
4. 局所再生成（無関係要素を巻き込まない）
5. 将来拡張性（保存/探索/UE連携）

## 3. 現在の正本モデル
- `Pole`, `Port`, `Anchor`, `Bundle`, `Span`, `Attachment`
- 複数本配線は `Bundle + 複数 Span`
- `lane` は workflow/debug の内部概念で、公開正本 API にはしない
- Bundle本数は `BundleTemplate` で決める（固定テンプレは上書き禁止、可変テンプレは範囲制約）。

## 4. 重要な境界ルール
- DrawPath は入力ツール。正本ではない。
- 生成器の中心入力は `BackboneSpec`。
- `BackboneSpec.bundles[]` で束テンプレを指定する。
- `slot` はテンプレ候補、`Port` は実接続点（混同しない）。
- 更新は編集 API 経由のみ。外部から mutable 参照で正本を直接触らない。

## 5. Manual/Auto ルール
- Manual はユーザー明示操作で作る。
- DrawPath 点を既定で強制 Manual 化しない。
- 再生成時は Manual Pole/Port を保持し、Auto 部分を優先更新する。

## 6. Backbone の位置づけ
- Backbone は Pole 間骨格とルート探索の層。
- Port/Span 詳細編集や見た目ルール本体は詳細層の責務。
- 入力条件（Spec）と生成結果（Result）は型で分離する。

## 7. 既知の注意点
- 正本と派生を混在させると破綻しやすい。
- 生成都合の一時情報を Entity に埋め込みすぎない。
- 文字列用語（Guide/slot/lane）が混ざると実装者解釈が割れるため、用語は docs/wire.md に合わせる。

## 8. 現在の実装フォーカス
- 正本安定化（更新経路の集約）
- DrawPath 生成の Manual 誤適用防止
- Pole Pin/Unpin と局所再生成の成立
- Backbone と詳細層の責務分離維持

## 9. 参照順序（新規チャット時）
1. `README.md`
2. `docs/wire.md`
3. `docs/core_model_architecture.md`
4. `docs/core_model_inventory.md`
5. `docs/chat_handoff_checklist.md`

## 10. Current Snapshot（2026-03-06）
- いま動くもの:
  - BackboneSpec 経路での HV_3PH 鋭角パスにおける lane ねじれ抑制。
  - `wire_core_tests` は 103/103 PASS。
  - DrawPath 系ケース（C86/C88/C99 含む）の lane 順反転検査が通過。
- いま壊れているもの:
  - 現時点で再現固定の failing case なし（最新 `wire_core_tests` 基準）。
- 既知リスク:
  - ねじれ判定は「pole局所Yの順序反転」基準。厳密XY交差（扇状近傍を含む）は評価主軸にしない。
  - `GenerateGroupedLine` 互換入口は内部で BackboneSpec に委譲しているが API 自体は残存。
- 未着手/保留:
  - 互換入口の完全廃止（公開 API 整理）。
  - viewer 側の可視デバッグ（mirror 適用区間 / junction order 表示）の恒久UI化。

## 11. Decision Log（直近）
- 2026-03-06 / Accepted:
  - 決定: ねじれ評価の主指標を区間法線ではなく「pole局所Y順序反転」に統一。
  - 理由: 鋭角区間で軸が反転し、同一配線でも偽陽性が出るため。
  - 影響: `generation_service.cpp` の mirror 評価軸、`core_tests.cpp` の inversion 観測軸。
  - 覆す条件: 実運用キャプチャで局所Y基準でも連続的にねじれ見えが残る場合。
- 2026-03-06 / Accepted:
  - 決定: lane ねじれ対策は mirror 2択のみ（任意 permutation 不採用）を維持。
  - 理由: 自由並び替えは挙動が不安定化しやすく、編集予測性を下げるため。
  - 影響: 生成器の割当ロジック、テスト観点（C85/C86/C99）。
  - 覆す条件: mirror 2択で要件ケースを満たせない失敗が複数パターンで確認された場合。
- 2026-03-06 / Accepted:
  - 決定: `ensure_ports` と `evaluate_increment` の side 軸は pole yaw を優先し、fallback のみヒント軸を使う。
  - 理由: ポート生成軸と評価軸の不一致が mirror 誤判定を誘発していたため。
  - 影響: 鋭角パスでの mirror 選択安定化、C76/C86/C87/C88/C99 の回帰抑制。
  - 覆す条件: yaw 優先で既存の直線・鈍角ケースに退行が出る場合。

## 12. 48h Task Board
1. P1: 互換入口整理（GenerateGroupedLine の新規利用停止を明文化）
   - Done: docs と public header で「BackboneSpec.bundles[] 必須」を明示し、viewer 呼び出しを新入口へ統一。
   - 依存: viewer 呼び出し箇所の棚卸し。
2. P2: ねじれ検証の運用固定（capture -> テスト化フロー）
   - Done: capture 再現点列を追加し、最小1件を恒久回帰（既存 C99 を維持し追加候補を1件以上）。
   - 依存: viewer からの capture 入力フォーマット確定。
3. P3: 低優先の幾何交差指標の扱い整理
   - Done: 「隣接扇状のXY厳密交差は品質参考値であり、合否主指標にしない」を docs に追記。
   - 依存: テストポリシー文言更新。

## 13. 次回開始パック（そのまま貼付可）
- ゴール:
  - Backbone 主導生成の一貫性維持（特に HV_3PH の鋭角・延長ケース）。
  - 正本 API を BackboneSpec 中心へさらに集約。
- 現在状態:
  - 103/103 tests pass。C76/C86/C87/C88/C99 通過。
  - ねじれ評価軸は pole局所Y順序で統一済み。
- 直近決定:
  - mirror 2択維持、任意並び替え禁止。
  - side軸は pole yaw 優先（生成/評価で統一）。
  - 厳密XY交差は主指標にしない。
- 次の48h候補:
  - 互換入口の利用禁止を docs+viewer 呼び出しで固定。
  - capture 追加分の恒久テスト化。
  - テストポリシー文言の明文化。
- 制約:
  - 正本直書き禁止、公開API経由のみ。
  - `slot`(候補) / `Port`(実体) の用語混同禁止。
  - Manual保持優先、全体再生成を既定にしない。
