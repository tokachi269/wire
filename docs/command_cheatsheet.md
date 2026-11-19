# command_cheatsheet.md

workdir: `D:\GitHub\wire`

## Core test

```cmd
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B build-vs18-coretests -G "Visual Studio 18 2026" -A x64 -DWIRE_BUILD_VIEWER=OFF
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-vs18-coretests --config Debug --target wire_core_tests
build-vs18-coretests\domains\wire\Debug\wire_core_tests.exe
```

## Viewer

Viewerが依存するライブラリのsource treeがローカルにない場合は、次のcommandで取得する。

```cmd
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" -S . -B build-vs18-viewer-fetch -G "Visual Studio 18 2026" -A x64 -DWIRE_BUILD_VIEWER=ON -DWIRE_VIEWER_FETCH_DEPS=ON
"C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build build-vs18-viewer-fetch --config Debug --target wire_viewer
build-vs18-viewer-fetch\viewer\Debug\wire_viewer.exe
```

`build-viewer\_deps\raylib-src`、`build-viewer\_deps\imgui-src`、`build-viewer\_deps\rlimgui-src`がすでにある場合は、`WIRE_VIEWER_FETCH_DEPS=OFF`を指定してローカルのsourceディレクトリを使える。

## 注意事項

- 通常のbuildとテストでは`Ninja`は不要。
- 最初に`Developer Command Prompt`または`Developer PowerShell for VS`を開いた場合、`vcvars64.bat`は不要。
- 通常のPowerShellを使う場合は、初期化した同じシェル内でbuildと実行を行うか、`cmd.exe /c`で囲む。
- `wire_core_tests`のbuild時には、arch lintとtest-family lintのターゲットも実行される。

## lint

```cmd
python tools\harness\test_architecture_lint.py
python tools\harness\test_architecture_observation.py
python tools\test_arch_lint.py
python tools\arch_lint.py
python tools\test_family_lint.py
git diff --check
```

## Architecture observation

`graph`は既定で製品コードを対象とする。`--scope tests`または`--scope tools`を指定すると、それぞれテストやtoolを対象にできる。DSM、co-change、hotspotの数値は、レビューの判断材料として使う。品質の合否判定には使わない。

Reflexionのdivergence判定は既存arch lintを再利用する。

```cmd
python tools\architecture_observation.py graph --format markdown
python tools\architecture_observation.py graph --scope tests --format markdown
python tools\architecture_observation.py reflexion --base <task-start-sha> --format markdown
python tools\architecture_observation.py delta --base <task-start-sha> --format markdown
python tools\architecture_observation.py history --recent-days 180 --format markdown
python tools\architecture_observation.py hotspot --format markdown
```

`reflexion`では、許可された依存関係を全て網羅するgraphを仮定しない。既存の必須・禁止契約、既存lintの結果、モデル化されていない関係を分けて表示する。`delta`では、自動で確定できる構造上の事実と、人によるレビューが必要な候補を分ける。pathやtokenから意味論の変更を断定しない。

`history`はfirst-parent chain上で各commitをfirst parentとの差分として一度だけ数える。大量変更を含む場合と除く場合を、inclusive/exclusiveとして表示する。

## clang-tidy

`compile_commands.json`を出力できるgeneratorでconfigureした後、補助ターゲットを実行する。

```cmd
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
cmake -S . -B build-clang-tidy -G Ninja -DWIRE_BUILD_VIEWER=OFF -DWIRE_ENABLE_PCH=OFF -DWIRE_ENABLE_FORMAT_TARGETS=OFF -DWIRE_ENABLE_UML_TARGETS=OFF
cmake --build build-clang-tidy --target tidy-check
```

調整中に小さなディレクトリ配下だけを検査する場合は、次のcommandを使う。

```cmd
python tools\clang_tidy_check.py --build-dir build-clang-tidy --path domains/wire/src/state
```

## clang-uml

```cmd
powershell -NoProfile -ExecutionPolicy Bypass -File tools\prepare_clang_uml_compile_db.ps1 -WorkspaceRoot .
"C:\Program Files\clang-uml\bin\clang-uml.exe" -c .clang-uml -l
"C:\Program Files\clang-uml\bin\clang-uml.exe" -c .clang-uml -n core_packages -g plantuml -p
```

## Web viewer WASM

`emcc` / `emcmake`がPATHにあるEmscriptenのシェルで実行する。

```powershell
Set-Location web
npm install
npm run wasm:build
npm test
npm run build
```

`npm test`はWASMの成果物を用意してから実行し、結果を報告する際はskipが0であることを確認する。

WASMだけを個別に再構築する場合は、次のcommandを使う。

```powershell
Set-Location web
npm run wasm:build
```

開発サーバーは次のcommandで起動する。

```powershell
Set-Location web
npm run dev
```

`npm run dev`は起動時にCore/road/WASM bindingのsource fingerprintを確認する。生成済みWASMが同じsourceに対応していれば、そのまま起動する。古い場合だけ、一度自動でbuildする。起動中もCore/road/WASM bindingの変更を監視し、source fingerprintが変わった場合はWASMを自動でbuildしてブラウザを完全に再読み込みする。

Git commit、文書、テスト、Web UIだけの変更では、WASMを再buildしない。
