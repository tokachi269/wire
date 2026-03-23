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
- `Bundle` を正本とした複数本配線管理（Laneは外部仕様に露出しない）
- raylib + ImGui の軽量 viewer で可視化・編集・検証

## 構成
```text
core/      コアライブラリ（データモデル、編集API、再計算、テスト）
viewer/    開発用軽量ビューワ
docs/      設計棚卸しドキュメント
wire.md    要求仕様書
```

## データモデル

コアは以下の **4層モデル** で構成されます。

### エンティティ層（永続化対象）

| エンティティ | 説明 |
|---|---|
| `Pole` | 電柱・鉄塔等の支持構造 |
| `Port` | 電線が接続する具体的な接続点（実在する接続エンドポイント） |
| `Anchor` | 支持専用点（碍子・支持金具等） |
| `Bundle` | 複数本配線の正本単位（本数・相間距離等） |
| `Span` | 2つの Port 間を結ぶ実体的な接続区間 |
| `Attachment` | Span 上のパラメトリック位置 `t` に配置される付属物（ダンパ等） |

> **slot と Port の区別**：`slot`（`PortSlotTemplate`）はテンプレート上の配置候補であり、`Port` は実在する接続点です。この区別は厳格に保たれます。

### キャッシュ層（永続化対象外・再生成可能）

`CurveCache`・`BoundsCache`・`DirtyQueue`・`SpanRuntimeState` 等は保存対象外です。


## 設計方針
- 座標系は UE 準拠、内部計算は `double`
- 64bit 永続 ID（再利用しない）
- 更新は `CoreState` の編集 API 経由のみ
- 双方向依存を避けた一方向 Dirty 伝播

## ビルド

実績あるコマンドだけをまとめたものは [docs/command_cheatsheet.md](docs/command_cheatsheet.md) を参照。

通常の build/test は `Visual Studio 18 2026` generator を使います。`Ninja` は不要です。

### core のみを clean build する
```cmd
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B build-vs18-coretests -G "Visual Studio 18 2026" -A x64 -DWIRE_BUILD_VIEWER=OFF
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-vs18-coretests --config Debug --target wire_core_tests
```

### viewer を build する

viewer は local dependency source dirs を渡す手順を基準にします。既に `build-viewer\_deps\raylib-src`、`build-viewer\_deps\imgui-src`、`build-viewer\_deps\rlimgui-src` があるならそのまま再利用できます。

```cmd
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B build-vs18-viewer-local -G "Visual Studio 18 2026" -A x64 -DWIRE_BUILD_VIEWER=ON -DWIRE_VIEWER_FETCH_DEPS=OFF -DWIRE_RAYLIB_SOURCE_DIR=D:/GitHub/wire/build-viewer/_deps/raylib-src -DWIRE_IMGUI_SOURCE_DIR=D:/GitHub/wire/build-viewer/_deps/imgui-src -DWIRE_RLIMGUI_SOURCE_DIR=D:/GitHub/wire/build-viewer/_deps/rlimgui-src
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-vs18-viewer-local --config Debug --target wire_viewer
```

依存ソースを別の場所に置いている場合は、`WIRE_RAYLIB_SOURCE_DIR`、`WIRE_IMGUI_SOURCE_DIR`、`WIRE_RLIMGUI_SOURCE_DIR` をそのパスに置き換えます。

## テスト
```cmd
build-vs18-coretests\core\Debug\wire_core_tests.exe
```

## フォーマット
```cmd
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-vs18-coretests --config Debug --target format-check
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-vs18-coretests --config Debug --target format
```

## UML 図生成

`clang-uml` は PCH を除いた専用の `compile_commands.json` を `build/clang-uml-db` に作ってから使います。

```cmd
powershell -NoProfile -ExecutionPolicy Bypass -File tools\prepare_clang_uml_compile_db.ps1 -WorkspaceRoot .
"C:\Program Files\clang-uml\bin\clang-uml.exe" -c .clang-uml -l
"C:\Program Files\clang-uml\bin\clang-uml.exe" -c .clang-uml -n core_packages -g plantuml -p
```

任意の configure 済み build dir から CMake target で実行することもできます。

```cmd
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-vs18-coretests --config Debug --target uml-list
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-vs18-coretests --config Debug --target uml
```

生成された `.puml` は `docs/diagrams/clang-uml/` に出力されます。既定の `.clang-uml` には以下の図を定義しています。

- `core_packages`
- `core_state_context`
- `viewer_packages`

`clang-uml` を `PATH` で見つけられない場合は、configure 時に `-DWIRE_CLANG_UML_PROGRAM="C:/Program Files/clang-uml/bin/clang-uml.exe"` を指定してください。

## viewer 実行
```cmd
build-vs18-viewer-local\viewer\Debug\wire_viewer.exe
```

## 実装フェーズ

| フェーズ | 内容 |
|---|---|
| Phase 0 | ビューワ骨組み（カメラ・座標軸・グリッド） |
| Phase 1 | コア土台（ID・ObjectStore・CoreState最小） |
| Phase 2 | 編集状態モデル + Add系編集API |
| Phase 3 | Dirty/Version/DirtyQueue + Move/Delete/Split |
| Phase 4 | 最小幾何生成（Curve/Bounds） |
| Phase 4.x | **設計棚卸し**（型一覧・責務・4層分類・命名整理） |
| Phase 4.9 | 公開面整理（inspection / override / support layout / DetailCurve 方針の固定） ← 現在 |
| Phase 5 | 保存読込（編集状態のみ） |
| Phase 6 | レイキャスト基礎・選択機能 |
| Phase 7 | カリング/LOD調整 |
| Phase 8〜 | 表現強化・将来拡張（風揺れ・電気計算・UE接続等） |

---

詳細は [docs/README.md](docs/README.md) を入口に参照してください。

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/tokachi269/wire)
