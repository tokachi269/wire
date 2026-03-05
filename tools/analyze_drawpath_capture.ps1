param(
  [string]$Path = ""
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Get-LatestCapturePath {
  $latest = Get-ChildItem "captures" -File -ErrorAction SilentlyContinue |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1
  if ($null -eq $latest) {
    throw "capture file not found in captures/"
  }
  return $latest.FullName
}

function Orient2dXY([double[]]$a, [double[]]$b, [double[]]$c) {
  return (($b[0] - $a[0]) * ($c[1] - $a[1])) - (($b[1] - $a[1]) * ($c[0] - $a[0]))
}

function SegmentsIntersectXYStrict([double[]]$a, [double[]]$b, [double[]]$c, [double[]]$d) {
  $minAx = [Math]::Min($a[0], $b[0]); $maxAx = [Math]::Max($a[0], $b[0])
  $minAy = [Math]::Min($a[1], $b[1]); $maxAy = [Math]::Max($a[1], $b[1])
  $minCx = [Math]::Min($c[0], $d[0]); $maxCx = [Math]::Max($c[0], $d[0])
  $minCy = [Math]::Min($c[1], $d[1]); $maxCy = [Math]::Max($c[1], $d[1])
  if ($maxAx -lt $minCx -or $maxCx -lt $minAx -or $maxAy -lt $minCy -or $maxCy -lt $minAy) {
    return $false
  }
  $eps = 1e-9
  $o1 = Orient2dXY $a $b $c
  $o2 = Orient2dXY $a $b $d
  $o3 = Orient2dXY $c $d $a
  $o4 = Orient2dXY $c $d $b
  $ab = (($o1 -gt $eps -and $o2 -lt -$eps) -or ($o1 -lt -$eps -and $o2 -gt $eps))
  $cd = (($o3 -gt $eps -and $o4 -lt -$eps) -or ($o3 -lt -$eps -and $o4 -gt $eps))
  return ($ab -and $cd)
}

if ([string]::IsNullOrWhiteSpace($Path)) {
  $Path = Get-LatestCapturePath
}
if (-not (Test-Path $Path)) {
  throw "capture file not found: $Path"
}

$lines = Get-Content $Path
$rxBundle = '^lane\[(\d+)\]\.bundle_id=(\d+)$'
$rxSegment = '^lane\[(\d+)\]\.segment_index=(\d+)$'
$rxCount = '^lane\[(\d+)\]\.count=(\d+)$'
$rxAPos = '^lane\[(\d+)\]\.pair\[(\d+)\]\.a\.pos=([-0-9\.eE]+),([-0-9\.eE]+),([-0-9\.eE]+)$'
$rxBPos = '^lane\[(\d+)\]\.pair\[(\d+)\]\.b\.pos=([-0-9\.eE]+),([-0-9\.eE]+),([-0-9\.eE]+)$'

$assignments = @{}
foreach ($line in $lines) {
  if ($line -match $rxBundle) {
    $idx = [int]$matches[1]
    if (-not $assignments.ContainsKey($idx)) {
      $assignments[$idx] = [ordered]@{ bundle = 0L; seg = 0; count = 0; A = @{}; B = @{} }
    }
    $assignments[$idx].bundle = [int64]$matches[2]
    continue
  }
  if ($line -match $rxSegment) {
    $idx = [int]$matches[1]
    if (-not $assignments.ContainsKey($idx)) {
      $assignments[$idx] = [ordered]@{ bundle = 0L; seg = 0; count = 0; A = @{}; B = @{} }
    }
    $assignments[$idx].seg = [int]$matches[2]
    continue
  }
  if ($line -match $rxCount) {
    $idx = [int]$matches[1]
    if (-not $assignments.ContainsKey($idx)) {
      $assignments[$idx] = [ordered]@{ bundle = 0L; seg = 0; count = 0; A = @{}; B = @{} }
    }
    $assignments[$idx].count = [int]$matches[2]
    continue
  }
  if ($line -match $rxAPos) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    if (-not $assignments.ContainsKey($idx)) {
      $assignments[$idx] = [ordered]@{ bundle = 0L; seg = 0; count = 0; A = @{}; B = @{} }
    }
    $assignments[$idx].A[$lane] = @([double]$matches[3], [double]$matches[4], [double]$matches[5])
    continue
  }
  if ($line -match $rxBPos) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    if (-not $assignments.ContainsKey($idx)) {
      $assignments[$idx] = [ordered]@{ bundle = 0L; seg = 0; count = 0; A = @{}; B = @{} }
    }
    $assignments[$idx].B[$lane] = @([double]$matches[3], [double]$matches[4], [double]$matches[5])
    continue
  }
}

$byBundle = @{}
foreach ($idx in $assignments.Keys) {
  $a = $assignments[$idx]
  if (-not $byBundle.ContainsKey($a.bundle)) {
    $byBundle[$a.bundle] = @()
  }
  $byBundle[$a.bundle] += ,$a
}

$rows = @()
foreach ($bundleId in ($byBundle.Keys | Sort-Object)) {
  $list = $byBundle[$bundleId] | Sort-Object seg
  if ($list.Count -eq 0) { continue }
  $laneCount = [int](($list | ForEach-Object { $_.count } | Measure-Object -Minimum).Minimum)
  if ($laneCount -lt 2) {
    $rows += [pscustomobject]@{
      bundle_id = $bundleId; lanes = $laneCount; segment_cross = 0; adjacent_cross = 0; global_cross = 0
    }
    continue
  }

  $laneSegments = @{}
  for ($lane = 0; $lane -lt $laneCount; ++$lane) {
    $laneSegments[$lane] = @()
  }
  foreach ($seg in $list) {
    for ($lane = 0; $lane -lt $laneCount; ++$lane) {
      if ($seg.A.ContainsKey($lane) -and $seg.B.ContainsKey($lane)) {
        $laneSegments[$lane] += ,@($seg.A[$lane], $seg.B[$lane])
      }
    }
  }

  $segmentCross = 0
  foreach ($seg in $list) {
    for ($i = 0; $i -lt $laneCount; ++$i) {
      if (-not ($seg.A.ContainsKey($i) -and $seg.B.ContainsKey($i))) { continue }
      for ($j = $i + 1; $j -lt $laneCount; ++$j) {
        if (-not ($seg.A.ContainsKey($j) -and $seg.B.ContainsKey($j))) { continue }
        if (SegmentsIntersectXYStrict $seg.A[$i] $seg.B[$i] $seg.A[$j] $seg.B[$j]) {
          $segmentCross++
        }
      }
    }
  }

  $adjacentCross = 0
  for ($idx = 0; $idx + 1 -lt $list.Count; ++$idx) {
    $left = $list[$idx]
    $right = $list[$idx + 1]
    for ($i = 0; $i -lt $laneCount; ++$i) {
      for ($j = $i + 1; $j -lt $laneCount; ++$j) {
        if ($left.A.ContainsKey($i) -and $left.B.ContainsKey($i) -and $right.A.ContainsKey($j) -and $right.B.ContainsKey($j)) {
          if (SegmentsIntersectXYStrict $left.A[$i] $left.B[$i] $right.A[$j] $right.B[$j]) {
            $adjacentCross++
          }
        }
        if ($left.A.ContainsKey($j) -and $left.B.ContainsKey($j) -and $right.A.ContainsKey($i) -and $right.B.ContainsKey($i)) {
          if (SegmentsIntersectXYStrict $left.A[$j] $left.B[$j] $right.A[$i] $right.B[$i]) {
            $adjacentCross++
          }
        }
      }
    }
  }

  $globalCross = 0
  for ($i = 0; $i -lt $laneCount; ++$i) {
    for ($j = $i + 1; $j -lt $laneCount; ++$j) {
      foreach ($si in $laneSegments[$i]) {
        foreach ($sj in $laneSegments[$j]) {
          if (SegmentsIntersectXYStrict $si[0] $si[1] $sj[0] $sj[1]) {
            $globalCross++
          }
        }
      }
    }
  }

  $rows += [pscustomobject]@{
    bundle_id = $bundleId
    lanes = $laneCount
    segment_cross = $segmentCross
    adjacent_cross = $adjacentCross
    global_cross = $globalCross
  }
}

Write-Output ("capture: {0}" -f $Path)
$rows | Format-Table -AutoSize
