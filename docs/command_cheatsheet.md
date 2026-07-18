# command_cheatsheet.md

workdir: `D:\GitHub\wire`

## core tests

```cmd
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B build-vs18-coretests -G "Visual Studio 18 2026" -A x64 -DWIRE_BUILD_VIEWER=OFF
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-vs18-coretests --config Debug --target wire_core_tests
build-vs18-coretests\core\Debug\wire_core_tests.exe
```

## viewer

Fetch viewer dependencies when local source trees are not already present.

```cmd
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B build-vs18-viewer-fetch -G "Visual Studio 18 2026" -A x64 -DWIRE_BUILD_VIEWER=ON -DWIRE_VIEWER_FETCH_DEPS=ON
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-vs18-viewer-fetch --config Debug --target wire_viewer
build-vs18-viewer-fetch\viewer\Debug\wire_viewer.exe
```

If you already have `build-viewer\_deps\raylib-src`, `build-viewer\_deps\imgui-src`, and `build-viewer\_deps\rlimgui-src`, you can instead use local source directories with `WIRE_VIEWER_FETCH_DEPS=OFF`.

## notes

- No `Ninja` is required for the normal build/test path.
- If you open `Developer Command Prompt` or `Developer PowerShell for VS` first, `vcvars64.bat` is not needed.
- If you use plain PowerShell, keep the build and run in the same initialized shell or wrap them with `cmd.exe /c`.
- Building `wire_core_tests` also runs architecture and test-family lint targets.

## lint

```cmd
python tools\arch_lint.py
python tools\test_family_lint.py
git diff --check
```

## clang-uml

```cmd
powershell -NoProfile -ExecutionPolicy Bypass -File tools\prepare_clang_uml_compile_db.ps1 -WorkspaceRoot .
"C:\Program Files\clang-uml\bin\clang-uml.exe" -c .clang-uml -l
"C:\Program Files\clang-uml\bin\clang-uml.exe" -c .clang-uml -n core_packages -g plantuml -p
```

## web viewer wasm

`emcc` / `emcmake` が PATH にある Emscripten shell で実行する。

```powershell
Set-Location web
npm install
npm run wasm:build
npm test
npm run build
```

`npm test` は wasm 成果物を前提に実行し、結果報告では skip が 0 であることを確認する。

個別に wasm だけを再構築する場合:

```powershell
Set-Location web
npm run wasm:build
```

開発 server:

```powershell
Set-Location web
npm run wasm:build
npm run dev
```
