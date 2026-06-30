# 開発コマンド

コマンドはrepository rootで実行する。

## Core

```powershell
& "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B build-vs18-coretests -G "Visual Studio 18 2026" -A x64 -DWIRE_BUILD_VIEWER=OFF
& "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-vs18-coretests --config Debug --target wire_core_tests
& .\build-vs18-coretests\core\Debug\wire_core_tests.exe
& .\build-vs18-coretests\core\Debug\wire_core_tests.exe backbone
```

## Viewer

```powershell
& "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B build-vs18-viewer-fetch -G "Visual Studio 18 2026" -A x64 -DWIRE_BUILD_VIEWER=ON -DWIRE_VIEWER_FETCH_DEPS=ON
& "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-vs18-viewer-fetch --config Debug --target wire_viewer_tests wire_viewer
& .\build-vs18-viewer-fetch\viewer\Debug\wire_viewer_tests.exe
```

## Lint

```powershell
python tools\arch_lint.py
python tools\test_family_lint.py
git diff --check
```

`wire_core_tests`のbuildはarchitecture/test-family lintも実行する。
通常buildにNinjaは不要。
