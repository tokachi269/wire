param(
  [Parameter(Mandatory = $true)]
  [string]$Path,
  [string]$BuildDir = "build-viewer",
  [string]$VsInstallPath = "C:\Program Files\Microsoft Visual Studio\18\Community"
)

$ErrorActionPreference = "Stop"

$devShellModule = Join-Path $VsInstallPath "Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
if (-not (Test-Path $devShellModule)) {
  throw "Visual Studio DevShell module was not found: $devShellModule"
}

Import-Module $devShellModule
Enter-VsDevShell -VsInstallPath $VsInstallPath -SkipAutomaticLocation -DevCmdArguments '-arch=x64 -host_arch=x64' | Out-Null

cmake --build $BuildDir --target wire_capture_replay
& ".\$BuildDir\core\wire_capture_replay.exe" $Path