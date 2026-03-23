param(
  [string]$WorkspaceRoot = (Get-Location).Path,
  [string]$InputFile = "",
  [string]$OutputDir = ""
)

$ErrorActionPreference = "Stop"

function Resolve-ExistingPath {
  param([string[]]$Candidates)
  foreach ($candidate in $Candidates) {
    if ([string]::IsNullOrWhiteSpace($candidate)) {
      continue
    }
    if (Test-Path -LiteralPath $candidate) {
      return (Resolve-Path -LiteralPath $candidate).Path
    }
  }
  return $null
}

$root = (Resolve-Path -LiteralPath $WorkspaceRoot).Path

if ([string]::IsNullOrWhiteSpace($InputFile)) {
  $InputFile = Resolve-ExistingPath @(
    (Join-Path $root "build\clang-uml\compile_commands.json"),
    (Join-Path $root "out\build\x64-Debug\compile_commands.json"),
    (Join-Path $root "build\clangd\compile_commands.json")
  )
} elseif (Test-Path -LiteralPath $InputFile) {
  $InputFile = (Resolve-Path -LiteralPath $InputFile).Path
}

if (-not $InputFile) {
  throw "No compile_commands.json found. Checked build\\clang-uml, out\\build\\x64-Debug, and build\\clangd."
}

if ([string]::IsNullOrWhiteSpace($OutputDir)) {
  $OutputDir = Join-Path $root "build\clang-uml-db"
}

New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null
$OutputFile = Join-Path $OutputDir "compile_commands.json"

$entries = Get-Content -LiteralPath $InputFile -Raw | ConvertFrom-Json
$sanitized = New-Object System.Collections.Generic.List[object]

foreach ($entry in $entries) {
  if ($null -eq $entry.file) {
    continue
  }

  if ($entry.file -match '(^|[\\/])cmake_pch\.(c|cxx|cpp)$') {
    continue
  }

  $clone = [ordered]@{}
  foreach ($property in $entry.PSObject.Properties) {
    $clone[$property.Name] = $property.Value
  }

  if ($clone.Contains("command")) {
    $command = [string]$clone["command"]
    $command = [regex]::Replace($command, '\s/MP(?=\s|$)', '')
    $command = [regex]::Replace($command, '\s/(Yc|Yu|Fp|FI)\S+', '')
    $command = [regex]::Replace($command, '\s+', ' ').Trim()
    $clone["command"] = $command
  }

  if ($clone.Contains("arguments")) {
    $arguments = @()
    foreach ($arg in $clone["arguments"]) {
      $text = [string]$arg
      if ($text -eq "/MP") {
        continue
      }
      if ($text -match '^/(Yc|Yu|Fp|FI)') {
        continue
      }
      $arguments += $text
    }
    $clone["arguments"] = $arguments
  }

  $sanitized.Add([pscustomobject]$clone)
}

$sanitized | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $OutputFile -Encoding utf8
Write-Output "Prepared clang-uml compilation database: $OutputFile"
