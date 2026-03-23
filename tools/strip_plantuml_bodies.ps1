param(
  [Parameter(Mandatory = $true)]
  [string[]]$Files
)

$ErrorActionPreference = "Stop"

foreach ($file in $Files) {
  if (-not (Test-Path -LiteralPath $file)) {
    throw "PlantUML file not found: $file"
  }

  $lines = Get-Content -LiteralPath $file
  $result = New-Object System.Collections.Generic.List[string]
  $skipping = $false

  foreach ($line in $lines) {
    if (-not $skipping -and $line -match '^(class|enum|abstract class|interface|struct)\s+\S+\s*\{$') {
      $result.Add($line)
      $result.Add("}")
      $skipping = $true
      continue
    }

    if ($skipping) {
      if ($line -eq "}") {
        $skipping = $false
      }
      continue
    }

    $result.Add($line)
  }

  Set-Content -LiteralPath $file -Value $result -Encoding utf8
  Write-Output "Stripped member bodies: $file"
}
