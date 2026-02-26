# wire 詳細ドキュメント（README補足）

README は入口に限定し、仕様・設計・運用詳細はこのドキュメントに集約します。

## 1. プロジェクトの狙い
`wire` は、電柱・電線ネットワークを自動生成・編集・検証するためのネイティブ C++ 基盤です。

- 見た目の自然さを優先
- 自動生成を主導線にする
- 将来の UE 連携や疎通計算へ拡張可能な構造にする

## 2. 基本方針
- 座標系: UE 準拠
- 数値精度: 内部 `double`
- ID: 64bit 永続 ID（再利用しない）
- 更新: `CoreState` の公開 API 経由のみ
- 再計算: `Span` 単位の Dirty/Version 管理

## 3. データモデルの考え方
### 3.1 4層モデル
- Definition: テンプレート/ルール
- Entity: `Pole`, `Port`, `Span`, `WireGroup`, `WireLane` など
- Workflow: 生成入力・診断・セッション情報
- Cache: `CurveCache`, `BoundsCache`, `DirtyQueue`, `SpanRuntimeState`

### 3.2 用語の固定
- `slot`: テンプレート上の候補点
- `Port`: 実在する接続点（実体）
- `Span`: 2つの Port を結ぶ接続区間
- `WireGroup` / `WireLane`: 複数本配線の論理単位
- `Bundle`: 見た目寄りの補助属性

## 4. 現在の実装方針（Phase4.x）
- 新機能追加より設計棚卸しを優先
- 型の責務・依存方向・命名の揺れを整理
- 保存対象（Persist）と再生成対象（Derived）を分離

## 5. viewer で確認できること
- 配置 / 接続 / 分岐 / 詳細 / DrawPath の基本導線
- Dirty 可視化、AABB 可視化、Version追随確認
- `WireGroup` / `WireLane` 情報、Port Auto/Manual 状態の確認

## 6. 将来拡張の前提
- `WireGroup` を複数本配線の主単位に寄せる
- `Port` 近傍表現（碍子など）は Port 側責務に寄せる
- `Bundle` は論理正本ではなく見た目補助として扱う

## 7. 参照
- 要求仕様: `wire.md`
- 設計経緯: `codex_shared_context.md`
- 型棚卸し: `docs/core_model_inventory.md`
