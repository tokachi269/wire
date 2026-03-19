# コマンドチートシート（2026-03 現在有効）

このリポジトリで実際に通る最小コマンドだけを残す。  
PowerShell 5 前提（`&&` は PowerShell 側で使わない）。

## 0. 前提
- 作業ディレクトリ: `D:\GitHub\wire`
- VS 環境初期化: `vcvars64.bat` を通して実行する
- 既定 build dir: `build-viewer`（Ninja + Debug）

## 1. configure（初回のみ）
```powershell
cmake -S . -B build-viewer -G Ninja -DCMAKE_BUILD_TYPE=Debug -DWIRE_BUILD_VIEWER=ON
```

## 2. core tests を build/run
```powershell
cmd.exe /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && cmake --build build-viewer --target wire_core_tests'
cmd.exe /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && build-viewer\core\wire_core_tests.exe'
```

## 3. viewer tests を build/run
```powershell
cmd.exe /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && cmake --build build-viewer --target wire_viewer_tests'
cmd.exe /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && ctest --test-dir build-viewer --output-on-failure'
```

## 4. viewer を build/run
```powershell
cmd.exe /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && cmake --build build-viewer --target wire_viewer'
.\build-viewer\viewer\wire_viewer.exe
```

## 5. fresh build（stale 回避）
```powershell
cmd.exe /c '"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" && cmake --build build-viewer --target wire_core_tests --clean-first'
```

## 6. よくある失敗と対処
- `build is not a directory`
  - `cmake -S . -B build-viewer ...` を先に実行する。
- `include <algorithm> が見つからない / C1083`
  - `vcvars64.bat` を通して build/run する。
- `PowerShell で && が使えない`
  - `cmd.exe /c '\"...vcvars64.bat\" && ...'` 形式で実行する。
