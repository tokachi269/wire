# Coreの方針を表す定数

T3は、混同しやすい定数を分類する調査である。本書は監査記録であり、定数の移行は行わない。T3の作業で定数を移動したり、値を調整したりしない。

## 物理・数学上の定数

| 定数 | 所在 | 分類 | 補足 |
|---|---|---|---|
| `kPi` / `kTwoPi` / `kRadiansToDegrees` | `coord_utils.hpp`、geometry/backboneのsource | 物理・数学上の定数 | 数学的な変換定数。重複は別途整理できるが、値は見た目の方針ではない。 |
| `kLengthToleranceM`, `kLengthSquaredToleranceM2`, `kStrictLengthToleranceM` | `numeric_tolerances.hpp` | 固定された契約 | 数値比較の方針。互換性を維持するための許容誤差であり、見た目の調整値ではない。 |
| `kInv` | variation/styleの乱数処理 | 物理・数学上の定数 | 53bitの乱数を単位区間のdoubleへ変換する。 |

## 固定された契約

| 定数 | 所在 | 分類 | 補足 |
|---|---|---|---|
| `kDefaultCornerThresholdDeg` | `core_runtime_types.hpp` | 固定された契約 | `LayoutSettings`で公開する既定のlayout設定。隠れたコード変更ではなく、設定から調整する。 |
| `kMaxCornerSideScale` | `core_runtime_types.hpp` | 固定された契約 | 公開されたlayoutの上限。変更すると生成幾何の方針も変わる。 |
| `kSharpCornerInteriorAngleMaxDeg` | `row_representation.cpp` | 固定された契約 | ユーザー向け設定へ移すまでは、通常角と鋭角の表現境界を定義する。 |
| `kRegenerateChangeMask` | `state/template/update.cpp` | 固定された契約 | テンプレート更新方針のbitmask契約。見た目の値ではない。 |
| `kMaxDebugRecords` | `state/template/ports.cpp` | 固定された契約 | debug storageの上限。生成幾何の値ではない。 |

## 見た目の方針

| 定数 | 所在 | 分類 | 補足 |
|---|---|---|---|
| `kNodePatchHorizontalLengthM` | `curve_parts.cpp` | 見た目の方針 | NodePatchの境界長。将来はcableやcategoryに依存する可能性がある。 |
| `kNodePatchMaxSpanFraction` | `curve_parts.cpp` | 見た目の方針 | 短いSpanの大部分がpatch境界に使われることを防ぐ。 |
| `kPatchMetersPerSegment` | `curve_parts.cpp` | 見た目の方針 | patch samplingの距離方向の分割密度。 |
| `kPatchRadiansPerSegment` | `curve_parts.cpp` | 見た目の方針 | patch samplingの角度方向の分割密度。 |
| `kFallbackTangentScale` | `detail_curve.cpp` | 見た目の方針 | 扱いにくい接線に対する品質fallbackのhandle scale。 |

## 調整候補

| 定数 | 所在 | 分類 | 補足 |
|---|---|---|---|
| `kAvoidClearanceM` | `pipeline.cpp` | 調整候補 | routeの回避迂回に使うclearance。物理法則ではなく入力・制約の方針。 |
| `kAutoCollapseDistanceM` | `pipeline.cpp` | 調整候補 | 近接nodeを自動統合する距離。ユーザー操作とtopologyの方針。 |
| `kG2MinChordLengthM` | `detail_curve.cpp` | 調整候補 | G2相当のdetail連続性を試みる最小長。 |
| `kG2EndpointOffsetRatioLimit` | `detail_curve.cpp` | 調整候補 | detail curveの品質閾値。 |
| `kG2EndpointOffsetMetersLimit` | `detail_curve.cpp` | 調整候補 | detail curveの品質閾値。 |
| `kCandidateAttempts` | `population.cpp` | 調整候補 | population探索の試行回数。決定的ではあるが、分布品質へ影響する。 |

## 現時点の判断

- T3では定数をconfigやテンプレートフィールドへ移さない。
- topology、モデルassembly、永続化のbug修正に付随して、見た目の方針や調整候補を変更しない。
- 見える幾何へ影響する定数を追加する場合は、同じ変更でこの表へ追加する。
