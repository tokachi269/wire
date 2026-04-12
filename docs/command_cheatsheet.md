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

## clang-uml

```cmd
powershell -NoProfile -ExecutionPolicy Bypass -File tools\prepare_clang_uml_compile_db.ps1 -WorkspaceRoot .
"C:\Program Files\clang-uml\bin\clang-uml.exe" -c .clang-uml -l
"C:\Program Files\clang-uml\bin\clang-uml.exe" -c .clang-uml -n core_packages -g plantuml -p
```
