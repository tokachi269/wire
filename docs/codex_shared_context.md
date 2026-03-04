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
