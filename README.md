# wire

電柱・電線ネットワーク生成基盤（ネイティブ C++ コアライブラリ + 開発用軽量ビューワ）

## 概要
`wire` は、電柱・電線ネットワークを自動生成・編集・検証するための基盤です。  
見た目の再現に加えて、接続管理、部分再計算（Dirty/Version）、選択、将来の保存読込やカリング拡張を前提に設計しています。

- コアはゲームエンジン非依存
- viewer は core を呼び出す検証用アプリ
- まずネイティブ C++ で実装し、将来 UE 連携を想定

## 主な特徴
- DrawPath 入力からの自動生成
- `Port` を中心にした接続点分離設計（slot はテンプレ候補、Port は実接続点）
- `Span` 単位の部分再計算（DirtyQueue + Version）
- `WireGroup` / `WireLane` による複数本配線の論理管理
- raylib + ImGui の軽量 viewer で可視化・編集・検証

## 構成
```text
core/      コアライブラリ（データモデル、編集API、再計算、テスト）
viewer/    開発用軽量ビューワ
docs/      設計棚卸しドキュメント
wire.md    要求仕様書
```

## データモデル

コアは以下の **4層モデル** で構成されます。 [5](#0-4) 

### エンティティ層（永続化対象）

| エンティティ | 説明 |
|---|---|
| `Pole` | 電柱・鉄塔等の支持構造 |
| `Port` | 電線が接続する具体的な接続点（実在する接続エンドポイント） |
| `Anchor` | 支持専用点（碍子・支持金具等） |
| `Bundle` | 導体束の属性（本数・相間距離等）。見た目寄り補助情報 |
| `WireGroup` | 複数スパンをまとめる論理/生成単位 |
| `WireLane` | WireGroup 内の1本識別・順序情報 |
| `Span` | 2つの Port 間を結ぶ実体的な接続区間 |
| `Attachment` | Span 上のパラメトリック位置 `t` に配置される付属物（ダンパ等） | [6](#0-5) 

> **slot と Port の区別**：`slot`（`PortSlotTemplate`）はテンプレート上の配置候補であり、`Port` は実在する接続点です。この区別は厳格に保たれます。 [7](#0-6) 

### キャッシュ層（永続化対象外・再生成可能）

`CurveCache`・`BoundsCache`・`DirtyQueue`・`SpanRuntimeState` 等は保存対象外です。 [8](#0-7) 


## 設計方針
- 座標系は UE 準拠、内部計算は `double`
- 64bit 永続 ID（再利用しない）
- 更新は `CoreState` の編集 API 経由のみ
- 双方向依存を避けた一方向 Dirty 伝播

## ビルド
```powershell
cmake -S . -B build -DWIRE_BUILD_VIEWER=ON
cmake --build build --config Debug --target wire_core_tests
cmake --build build --config Debug --target wire_viewer
```

## テスト
```powershell
build\core\Debug\wire_core_tests.exe
```

## フォーマット
```powershell
cmake --build build --config Debug --target format-check
cmake --build build --config Debug --target format
```

## viewer 実行
```powershell
build\viewer\Debug\wire_viewer.exe
```

## 実装フェーズ

| フェーズ | 内容 |
|---|---|
| Phase 0 | ビューワ骨組み（カメラ・座標軸・グリッド） |
| Phase 1 | コア土台（ID・ObjectStore・CoreState最小） |
| Phase 2 | 編集状態モデル + Add系編集API |
| Phase 3 | Dirty/Version/DirtyQueue + Move/Delete/Split |
| Phase 4 | 最小幾何生成（Curve/Bounds） |
| Phase 4.x | **設計棚卸し**（型一覧・責務・4層分類・命名整理） ← 現在 |
| Phase 5 | 保存読込（編集状態のみ） |
| Phase 6 | レイキャスト基礎・選択機能 |
| Phase 7 | カリング/LOD調整 |
| Phase 8〜 | 表現強化・将来拡張（風揺れ・電気計算・UE接続等） | [20](#0-19) 

---

詳細は `wire.md` と `codex_shared_context.md` を参照してください。
