@echo off
setlocal

set "MODE=%~1"
set "ROOT=%~dp0.."
set "VCVARS=C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"

if not exist "%VCVARS%" (
  echo Missing Visual Studio vcvars64.bat: %VCVARS% 1>&2
  exit /b 1
)

call "%VCVARS%" >nul
if errorlevel 1 exit /b %errorlevel%

if "%MODE%"=="configure" (
  cd /d "%ROOT%"
  emcmake cmake -S wasm -B wasm/build-nmake -G "NMake Makefiles" -DBUILD_TESTING=OFF -DWIRE_ENABLE_PCH=OFF
  exit /b %errorlevel%
)

if "%MODE%"=="build" (
  cd /d "%ROOT%"
  if not exist "wasm\build-nmake\CMakeCache.txt" (
    emcmake cmake -S wasm -B wasm/build-nmake -G "NMake Makefiles" -DBUILD_TESTING=OFF -DWIRE_ENABLE_PCH=OFF
    if errorlevel 1 exit /b 1
  )
  cmake --build wasm/build-nmake --target wire_web_core
  exit /b %errorlevel%
)

echo Usage: wasm-nmake-build.cmd configure^|build 1>&2
exit /b 2
