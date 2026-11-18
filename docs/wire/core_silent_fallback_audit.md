# Coreの暗黙fallback監査

状態: 完了済み監査の固定記録。本書はR5の調査結果を記録するもので、継続更新する網羅性マトリクスではない。

R5では、欠落または退化した幾何を既定値へ置き換えるCoreの経路を分類した。fallback追加を認めるものではない。新しいfallback相当の処理は先にここで分類し、挙動を変更する場合は別途fail-first testを用意する。

## 分類

| 領域 | 所在 | fallbackの内容 | 分類 | 理由・次の対応 |
|---|---|---|---|---|
| Pole/layout yawの正規化 | `domains/wire/include/city/wire/coord_utils.hpp` `HorizontalNormalizedOr` | 水平成分がゼロのvectorをworld forwardへ置き換える | 上流の不整合候補 | 対応済みbackbone rowはlayoutへ渡る前に非zeroの軸を持つべきである。既存の呼び出し箇所は互換性のため残すが、新しいbackbone row-axis利用箇所では呼び出し前に検証する。 |
| BackboneのPort配置 | `domains/wire/src/generation/backbone/emit_shared.cpp` | `PortLayoutYawDeg`、`PortLocalPosition`、`PortWorldPosition`内の`HorizontalNormalizedOr(row_axis)` | 上流の不整合候補 | 通常経路では、軸がゼロのbackbone edgeやrowはpair/row構築時に既に拒否する。対応済み生成でこのfallbackへ到達するなら、それ以前のguardが欠けている。 |
| endpointの意味上のside axis | `domains/wire/src/generation/backbone/pipeline.cpp` | `HorizontalNormalizedOr(source->group_axis)` | 保留 | support-groupの幾何から導出する値である。focusedなfail-first testにより、ゼロのgroup axisが対応済み出力へ到達すると証明された場合は内部errorにする。R5では挙動を変更しない。 |
| Model assemblyのcached plan fallback | `domains/wire/src/generation/backbone/model_assembly.cpp` | fixture placement planがなければendpoint配置を再計算し、計測counterを増やす | 上流の不整合候補 | 通常生成と再配置の経路では、C791/C784がendpointとrow fixtureのfallback counterをゼロに保つことを検査する。診断用の安全策としてのみ残し、対応済み経路の成立には使わない。 |
| Span visual frameのsampling | `domains/wire/src/generation/backbone/span_visual_assembly.cpp` | segmentや接線がゼロの場合、`unit_or`が固定軸を返す | 正当な退化処理 | visual assemblyは1点だけのsupport pathやclampされたpathをsampleし、有限のdebug/visual出力を作ることがある。既存のgeometry testが決定的で有限の出力を検証する。 |
| Detail curveの接線blend | `domains/wire/src/geometry/detail_curve.cpp` | 接線blendが相殺される場合、`BlendDirections`が呼び出し元のfallbackを返す | 正当な退化処理 | 鋭角・競合する接線に対する明示的な品質fallbackである。`DetailCurveQuality.fallback_iterations`へ記録し、geometry testで検証する。 |
| Detail curveの連続性引き下げ | `domains/wire/src/geometry/detail_curve.cpp` | 回数を制限した再試行で連続性制約を下げる | 正当な退化処理 | 観測可能な品質metadataを持つ明示的なsolver方針であり、暗黙のtopology推測ではない。 |
| Curve frameの構築 | `domains/wire/src/geometry/curve/curve.cpp` | 垂直またはゼロのframe vectorに対する`normalized_or`と、最も整列しない軸へのfallback | 正当な退化処理 | cable curveの構築では、垂直Spanや退化test caseでも有限値を保つ必要がある。未対応のmethodは引き続き明示的に拒否する。 |
| Template Port solverの収容超過 | `domains/wire/src/state/template/ports.cpp` | 希望する配置に収まらない場合、制約内の代替Port候補を使う | 保留 | templateが所有するmanual Port解決であり、R5の挙動変更の対象外である。backboneの継続関係のfallbackではない。制約を厳しくする前に、専用のtemplate-port契約が必要になる。 |
| Template registryのspacing | `domains/wire/src/state/template/registry.cpp` | cable templateを引けない場合、既定のsupport fanout spacingを使う | 保留 | 既定registryの構築では組み込みtemplateのbootstrapを認める。生成済みstateの処理へこの方法を流用しない。 |

## 現在のguard

- `model_assembly.cpp`のendpointとrow fixtureのfallback counterは、大規模routeの操作guardでゼロを維持することを期待する。
- backbone入力のNaN/inf検証は、fallbackの多いgeometry処理へ渡る前に、外部座標の非有限値を拒否する。
- 未対応のbackbone入力は明示的な`unsupported`または`invalid input`を返す。旧生成経路へfallbackしてはならない。

## 挙動変更前に必要な対応

- 対応済み要求がゼロの軸を持って到達することをfail-first scenarioで示した後に限り、backbone row-axisの`HorizontalNormalizedOr`呼び出し箇所を明示的なvalidationへ置き換える。
- 担当するsupport-group不変条件を特定した後に限り、`pipeline.cpp`のgroup-axis fallbackを内部errorへ変更する。
- template Portの制約内fallbackは、template-port testとともに別途分類する。backboneの堅牢性改善の一部として変更しない。
