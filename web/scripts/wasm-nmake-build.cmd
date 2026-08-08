@echo off
setlocal

set "MODE=%~1"
set "ROOT=%~dp0.."
set "WASM_SOURCE=%ROOT%\wasm"
set "WASM_BUILD=%WASM_SOURCE%\build-nmake"
set "VCVARS=C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"

for /f "delims=" %%I in ('node "%ROOT%\scripts\wasm-source-hash.mjs" 2^>nul') do set "WIRE_BUILD_SOURCE_HASH=%%I"
if not defined WIRE_BUILD_SOURCE_HASH set "WIRE_BUILD_SOURCE_HASH=unknown"
for /f "delims=" %%I in ('node -p "require(process.argv[1]).version" "%ROOT%\package.json" 2^>nul') do set "WIRE_BUILD_VERSION=%%I"
if not defined WIRE_BUILD_VERSION set "WIRE_BUILD_VERSION=unknown"

if not exist "%VCVARS%" (
  echo Missing Visual Studio vcvars64.bat: %VCVARS% 1>&2
  exit /b 1
)

call "%VCVARS%" >nul
if errorlevel 1 exit /b %errorlevel%

for /f "delims=" %%I in ('where nmake 2^>nul') do (
  set "NMAKE=%%I"
  goto :found_nmake
)
echo Missing nmake after vcvars64.bat 1>&2
exit /b 1
:found_nmake

if "%MODE%"=="configure" (
  emcmake cmake -S "%WASM_SOURCE%" -B "%WASM_BUILD%" -G "NMake Makefiles" -DCMAKE_MAKE_PROGRAM="%NMAKE%" -DBUILD_TESTING=OFF -DWIRE_ENABLE_PCH=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXE_LINKER_FLAGS_RELEASE=-O3 -DWIRE_BUILD_SOURCE_HASH="%WIRE_BUILD_SOURCE_HASH%" -DWIRE_BUILD_VERSION="%WIRE_BUILD_VERSION%"
  exit /b %errorlevel%
)

if "%MODE%"=="build" (
  emcmake cmake -S "%WASM_SOURCE%" -B "%WASM_BUILD%" -G "NMake Makefiles" -DCMAKE_MAKE_PROGRAM="%NMAKE%" -DBUILD_TESTING=OFF -DWIRE_ENABLE_PCH=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXE_LINKER_FLAGS_RELEASE=-O3 -DWIRE_BUILD_SOURCE_HASH="%WIRE_BUILD_SOURCE_HASH%" -DWIRE_BUILD_VERSION="%WIRE_BUILD_VERSION%"
  if errorlevel 1 exit /b 1
  cmake --build "%WASM_BUILD%" --target wire_web_core
  if errorlevel 1 exit /b 1
  > "%ROOT%\src\wasm-generated\source-hash.txt" echo %WIRE_BUILD_SOURCE_HASH%
  exit /b 0
)

echo Usage: wasm-nmake-build.cmd configure^|build 1>&2
exit /b 2
