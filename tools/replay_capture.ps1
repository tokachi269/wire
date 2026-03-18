param(
  [Parameter(Mandatory = $true)]
  [string]$Path,
  [string]$ExePath = "build\clangd\core\wire_capture_replay.exe"
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path $ExePath)) {
  throw "Replay executable was not found: $ExePath"
}

if (-not (Test-Path $Path)) {
  throw "Capture file was not found: $Path"
}

$resolvedExe = (Resolve-Path $ExePath).Path
$resolvedCapture = (Resolve-Path $Path).Path

& $resolvedExe $resolvedCapture
exit $LASTEXITCODE