# コマンドチートシート

Windows ローカル環境で実際に通った configure / build / run コマンドだけを残す。

## 前提

- C++ compiler は通常の PowerShell には乗っていない。
- 先に Visual Studio Developer Shell を今の PowerShell に読み込む。
- 既存の `build/` が古い Visual Studio インスタンス情報を持っている場合は、新しい build ディレクトリを使う。

## 1. Developer Shell を読む

```powershell
Import-Module "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
Enter-VsDevShell -VsInstallPath "C:\Program Files\Microsoft Visual Studio\18\Community" -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64'
```

## 2. viewer を新しい build ディレクトリで configure / build

`build/` が壊れている可能性を避ける標準手順。

```powershell
cmake -S . -B build-viewer -G Ninja -DCMAKE_BUILD_TYPE=Debug -DWIRE_BUILD_VIEWER=ON
cmake --build build-viewer --target wire_viewer
```

## 3. viewer を起動

```powershell
.\build-viewer\viewer\wire_viewer.exe
```

## 4. core tests を build / run

```powershell
cmake -S . -B build-core -G Ninja -DCMAKE_BUILD_TYPE=Debug -DWIRE_BUILD_VIEWER=OFF
cmake --build build-core --target wire_core_tests
.\build-core\core\wire_core_tests.exe
```

## 5. viewer ありで core tests も build する

```powershell
cmake -S . -B build-viewer -G Ninja -DCMAKE_BUILD_TYPE=Debug -DWIRE_BUILD_VIEWER=ON
cmake --build build-viewer --target wire_core_tests
.\build-viewer\core\wire_core_tests.exe
```

## 6. 既存 build で format

`build/` の configure が通っている場合だけ使う。

```powershell
cmake --build build --target format-check
cmake --build build --target format
```

## 7. viewer をオフライン build

依存をローカル配置している場合。

```powershell
cmake -S . -B build-offline -G Ninja -DCMAKE_BUILD_TYPE=Debug `
  -DWIRE_BUILD_VIEWER=ON `
  -DWIRE_VIEWER_FETCH_DEPS=OFF `
  -DWIRE_RAYLIB_SOURCE_DIR=C:\deps\raylib `
  -DWIRE_IMGUI_SOURCE_DIR=C:\deps\imgui `
  -DWIRE_RLIMGUI_SOURCE_DIR=C:\deps\rlImGui
cmake --build build-offline --target wire_viewer
.\build-offline\viewer\wire_viewer.exe
```

## 8. よくある失敗

### Visual Studio 2022/Community instance not known

原因:
古い `build/` が過去の Visual Studio インスタンス情報を持っている。

対処:
`build-viewer/` や `build-core/` のような新しい build ディレクトリを使う。

### No CMAKE_CXX_COMPILER could be found

原因:
Developer Shell を読まずに通常の PowerShell から Ninja configure している。

対処:
最初にこの文書の `Developer Shell を読む` を実行する。

### fatal error C1083: include ファイルを開けません。'optional': No such file or directory

原因:
`cl.exe` 自体は見えているが、MSVC の標準ライブラリ include 環境が入っていない PowerShell から build している。

対処:
同じターミナルで最初にこの文書の `Developer Shell を読む` を実行してから、再度 `cmake --build build-viewer --target wire_viewer` を実行する。