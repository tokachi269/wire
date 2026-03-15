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
$captureVersion = 0
$rxCaptureVersion = '^capture\.version=(\d+)$'
$rxBundleLegacy = '^lane\[(\d+)\]\.bundle_id=(\d+)$'
$rxSegmentLegacy = '^lane\[(\d+)\]\.segment_index=(\d+)$'
$rxCountLegacy = '^lane\[(\d+)\]\.count=(\d+)$'
$rxAPosLegacy = '^lane\[(\d+)\]\.pair\[(\d+)\]\.a\.pos=([-0-9\.eE]+),([-0-9\.eE]+),([-0-9\.eE]+)$'
$rxBPosLegacy = '^lane\[(\d+)\]\.pair\[(\d+)\]\.b\.pos=([-0-9\.eE]+),([-0-9\.eE]+),([-0-9\.eE]+)$'

$rxSegment = '^result\.lane_assignment\[(\d+)\]\.segment_index=(\d+)$'
$rxBundle = '^result\.lane_assignment\[(\d+)\]\.bundle_id=(\d+)$'
$rxCount = '^result\.lane_assignment\[(\d+)\]\.lane_count=(\d+)$'
$rxAPos = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.port_a_world=([-0-9\.eE]+),([-0-9\.eE]+),([-0-9\.eE]+)$'
$rxBPos = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.port_b_world=([-0-9\.eE]+),([-0-9\.eE]+),([-0-9\.eE]+)$'
$rxSpanId = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.span_id=(\d+)$'
$rxSpanFlow = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.span_flow_kind_label=(.+)$'
$rxSpanLowering = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.span_lowering_kind_label=(.+)$'
$rxSpanSameLevel = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.span_same_level_feasible=(\d+)$'
$rxSpanSameLevelReason = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.span_same_level_reason=(.+)$'
$rxLayoutRelationA = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.layout_relation_a=(.+)$'
$rxLayoutRelationB = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.layout_relation_b=(.+)$'
$rxLayoutOrderA = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.layout_start_bundle_order_choice=(.+)$'
$rxLayoutOrderB = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.layout_end_bundle_order_choice=(.+)$'
$rxLayoutReasonA = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.layout_start_bundle_order_reason=(.+)$'
$rxLayoutReasonB = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.layout_end_bundle_order_reason=(.+)$'
$rxLayoutSideA = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.layout_start_side=(.+)$'
$rxLayoutSideB = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.layout_end_side=(.+)$'
$rxLayoutGroupCount = '^result\.lane_assignment\[(\d+)\]\.lane\[(\d+)\]\.layout_lowered_support_group_count=(\d+)$'
$rxSnapshotNodeCount = '^result\.backbone\.snapshot_node_count=(\d+)$'
$rxSnapshotEdgeCount = '^result\.backbone\.snapshot_edge_count=(\d+)$'
$rxRebuiltNodeCount = '^result\.backbone\.rebuilt_node_count=(\d+)$'
$rxRebuiltEdgeCount = '^result\.backbone\.rebuilt_edge_count=(\d+)$'
$rxCurrentSpanCount = '^result\.current_span_count=(\d+)$'

$snapshotNodeCount = $null
$snapshotEdgeCount = $null
$rebuiltNodeCount = $null
$rebuiltEdgeCount = $null
$currentSpanCount = $null

function Ensure-Assignment([hashtable]$Map, [int]$Index) {
  if (-not $Map.ContainsKey($Index)) {
    $Map[$Index] = [ordered]@{
      bundle = 0L
      seg = 0
      count = 0
      A = @{}
      B = @{}
      lanes = @{}
    }
  }
}

function Ensure-Lane([hashtable]$Assignment, [int]$LaneIndex) {
  if (-not $Assignment['lanes'].ContainsKey($LaneIndex)) {
    $Assignment['lanes'][$LaneIndex] = [ordered]@{
      span_id = 0L
      flow = ""
      lowering = ""
      same_level_feasible = ""
      same_level_reason = ""
      relation_a = ""
      relation_b = ""
      order_a = ""
      order_b = ""
      reason_a = ""
      reason_b = ""
      side_a = ""
      side_b = ""
      lowered_support_group_count = 0
    }
  }
}

$assignments = @{}
foreach ($line in $lines) {
  if ($line -match $rxCaptureVersion) {
    $captureVersion = [int]$matches[1]
    continue
  }
  if ($line -match $rxSnapshotNodeCount) {
    $snapshotNodeCount = [int]$matches[1]
    continue
  }
  if ($line -match $rxSnapshotEdgeCount) {
    $snapshotEdgeCount = [int]$matches[1]
    continue
  }
  if ($line -match $rxRebuiltNodeCount) {
    $rebuiltNodeCount = [int]$matches[1]
    continue
  }
  if ($line -match $rxRebuiltEdgeCount) {
    $rebuiltEdgeCount = [int]$matches[1]
    continue
  }
  if ($line -match $rxCurrentSpanCount) {
    $currentSpanCount = [int]$matches[1]
    continue
  }
  if ($line -match $rxBundleLegacy) {
    $idx = [int]$matches[1]
    Ensure-Assignment $assignments $idx
    $assignments[$idx]['bundle'] = [int64]$matches[2]
    continue
  }
  if ($line -match $rxSegmentLegacy) {
    $idx = [int]$matches[1]
    Ensure-Assignment $assignments $idx
    $assignments[$idx]['seg'] = [int]$matches[2]
    continue
  }
  if ($line -match $rxCountLegacy) {
    $idx = [int]$matches[1]
    Ensure-Assignment $assignments $idx
    $assignments[$idx]['count'] = [int]$matches[2]
    continue
  }
  if ($line -match $rxAPosLegacy) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    $assignments[$idx]['A'][$lane] = @([double]$matches[3], [double]$matches[4], [double]$matches[5])
    continue
  }
  if ($line -match $rxBPosLegacy) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    $assignments[$idx]['B'][$lane] = @([double]$matches[3], [double]$matches[4], [double]$matches[5])
    continue
  }
  if ($line -match $rxBundle) {
    $idx = [int]$matches[1]
    Ensure-Assignment $assignments $idx
    $assignments[$idx]['bundle'] = [int64]$matches[2]
    continue
  }
  if ($line -match $rxSegment) {
    $idx = [int]$matches[1]
    Ensure-Assignment $assignments $idx
    $assignments[$idx]['seg'] = [int]$matches[2]
    continue
  }
  if ($line -match $rxCount) {
    $idx = [int]$matches[1]
    Ensure-Assignment $assignments $idx
    $assignments[$idx]['count'] = [int]$matches[2]
    continue
  }
  if ($line -match $rxAPos) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['A'][$lane] = @([double]$matches[3], [double]$matches[4], [double]$matches[5])
    continue
  }
  if ($line -match $rxBPos) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['B'][$lane] = @([double]$matches[3], [double]$matches[4], [double]$matches[5])
    continue
  }
  if ($line -match $rxSpanId) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['span_id'] = [int64]$matches[3]
    continue
  }
  if ($line -match $rxSpanFlow) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['flow'] = $matches[3]
    continue
  }
  if ($line -match $rxSpanLowering) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['lowering'] = $matches[3]
    continue
  }
  if ($line -match $rxSpanSameLevel) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['same_level_feasible'] = $matches[3]
    continue
  }
  if ($line -match $rxSpanSameLevelReason) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['same_level_reason'] = $matches[3]
    continue
  }
  if ($line -match $rxLayoutRelationA) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['relation_a'] = $matches[3]
    continue
  }
  if ($line -match $rxLayoutRelationB) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['relation_b'] = $matches[3]
    continue
  }
  if ($line -match $rxLayoutOrderA) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['order_a'] = $matches[3]
    continue
  }
  if ($line -match $rxLayoutOrderB) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['order_b'] = $matches[3]
    continue
  }
  if ($line -match $rxLayoutReasonA) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['reason_a'] = $matches[3]
    continue
  }
  if ($line -match $rxLayoutReasonB) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['reason_b'] = $matches[3]
    continue
  }
  if ($line -match $rxLayoutSideA) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['side_a'] = $matches[3]
    continue
  }
  if ($line -match $rxLayoutSideB) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['side_b'] = $matches[3]
    continue
  }
  if ($line -match $rxLayoutGroupCount) {
    $idx = [int]$matches[1]; $lane = [int]$matches[2]
    Ensure-Assignment $assignments $idx
    Ensure-Lane $assignments[$idx] $lane
    $assignments[$idx]['lanes'][$lane]['lowered_support_group_count'] = [int]$matches[3]
    continue
  }
}

$byBundle = @{}
foreach ($idx in $assignments.Keys) {
  $a = $assignments[$idx]
  if (-not $byBundle.ContainsKey($a['bundle'])) {
    $byBundle[$a['bundle']] = @()
  }
  $assignmentObject = [pscustomobject]@{
    bundle = $a['bundle']
    seg = $a['seg']
    count = $a['count']
    A = $a['A']
    B = $a['B']
    lanes = $a['lanes']
  }
  $byBundle[$a['bundle']] += ,$assignmentObject
}

$rows = @()
$decisionRows = @()
foreach ($bundleId in ($byBundle.Keys | Sort-Object)) {
  [object[]]$list = @(@($byBundle[$bundleId]) | Sort-Object seg)
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

  foreach ($seg in $list) {
    foreach ($laneKey in ($seg.lanes.Keys | Sort-Object)) {
      $laneInfo = $seg.lanes[$laneKey]
      $decisionRows += [pscustomobject]@{
        bundle_id = $bundleId
        segment_index = $seg.seg
        lane = $laneKey
        span_id = $laneInfo['span_id']
        flow = $laneInfo['flow']
        lowering = $laneInfo['lowering']
        same_level = $laneInfo['same_level_feasible']
        same_level_reason = $laneInfo['same_level_reason']
        relation = (($laneInfo['relation_a'], $laneInfo['relation_b']) -join ' -> ').Trim()
        order = (($laneInfo['order_a'], $laneInfo['order_b']) -join ' -> ').Trim()
        order_reason = (($laneInfo['reason_a'], $laneInfo['reason_b']) -join ' -> ').Trim()
        side = (($laneInfo['side_a'], $laneInfo['side_b']) -join ' -> ').Trim()
        lowered_support_groups = $laneInfo['lowered_support_group_count']
      }
    }
  }
}

Write-Output ("capture: {0}" -f $Path)
Write-Output ("capture.version: {0}" -f $captureVersion)
if ($null -ne $snapshotNodeCount -or $null -ne $rebuiltNodeCount -or $null -ne $currentSpanCount) {
  Write-Output (("snapshot backbone: nodes={0} edges={1} | rebuilt backbone: nodes={2} edges={3} | current spans={4}") -f
      $snapshotNodeCount, $snapshotEdgeCount, $rebuiltNodeCount, $rebuiltEdgeCount, $currentSpanCount)
}
$rows | Format-Table -AutoSize
if ($decisionRows.Count -gt 0) {
  Write-Output ""
  Write-Output "decision summary:"
  $decisionRows | Sort-Object bundle_id, segment_index, lane | Format-Table -AutoSize
}
