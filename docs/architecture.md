# アーキテクチャ

このドキュメントは、現在の backbone 生成本流の契約をまとめる。
現在の実装は移行中に `bb2` と呼んでいたが、production code と現行testは `backbone` を正とする。

## 全体構造

```text
BackboneSpec
  -> backbone generation
  -> SavedBackboneGraph
  -> pair / open / row
  -> SpanLayoutRules
  -> support group / SpanLayoutEntry
  -> DetailCurve / bounds
  -> visual / render cache
  -> viewer / export adapter
```

## 正本と派生

| 領域 | 決定者 | 責務 |
|---|---|---|
| topology | `SavedBackboneGraph` | node、edge、edge bundle、port/span binding、frontier |
| connectivity | `pairs make(graph)` | pair、open、row |
| placement | support group / row placement | row separation、vertical order、lowering offset |
| rules | `SpanLayoutRules` | span layout intent |
| layout | `SpanLayoutEntry` | `support_world` と `endpoint_world` |
| geom | `DetailCurve` / bounds | layout endpointからの形状派生 |
| draw | visual / render cache | layout/geomからの表示出力 |

生成済みのspan、layout、curve、bounds、visual、port位置からtopologyを復元してはいけない。
同じ意味を複数段で再判断せず、下流は上流の決定済み値だけを消費する。

## backbone generation

`GenerateFromBackboneSpec()` は `core/src/generation/backbone` のpipelineだけを呼ぶ。
未対応入力はv1へfallbackせず、mutation前に`unsupported`を返す。

処理順は次の通り。

1. inputの`prepare`と`check`
2. graphからpair/open/rowを確定
3. duplicate edge bundle/span bindingをpreflight
4. intentとsupport groupを確定
5. pole、bundle、port、spanを生成
6. `SavedBackboneGraph`とbindingを保存
7. rules、layout、geom、drawを保存

context linkは判断入力であり、生成・保存対象ではない。
T/cross/branchのkind enumは作らず、pair/open/rowの組合せで表す。

### row conflict と endpoint offset

通常のroute bendはlowering対象ではない。
同一nodeのrow conflictで重なりを避ける必要がある場合だけ、`BundleTemplate::enable_branch_down_offset` と
`branch_endpoint_offset_m`を使い、対象bundleのjunction側`endpoint_world`をoffsetする。
`support_world`とport位置は元の取付位置を保持する。
LV/HVなどのcategory名自体はlowering条件にしない。

ownerlessなmidair branchの取付位置は、saved edge、edge bundle、lane、port bindingから解決する。
別bundleのportやnodeのground位置から高さを推測せず、解決できなければmutation前に`unsupported`とする。
`allow_midair_branch=false`のtemplateはmidair branchの生成対象にしない。

## post-edit update

更新分類は次の4種類だけとする。

| `UpdateKind` | 変更範囲 | 再導出 |
|---|---|---|
| `kRegenerate` | topology / identity / connectivity | generation。安全にできなければ`unsupported` |
| `kReposition` | support/endpoint位置 | layout -> geom -> draw |
| `kReshape` | curve/bounds/shape | geom -> draw |
| `kRedraw` | visual/render | drawのみ |

操作名ごとのdirty enumは追加しない。
post-edit APIは、派生出力を更新して成功するか、mutation前に拒否する。
staleなlayout/geom/drawを残したまま成功してはいけない。

`UpdatePoleTypeDefinition`は、対象typeをactive backbone poleが使用中ならmutation前に拒否する。
非backbone poleだけへのdefinition再適用は許可する。active objectのtemplate migrationは現行scopeに含めない。

`DeriveGeneratedSpanOutputs()` は、保存済み rules / layout source / `SavedBackboneGraph` binding から
layout、geom、drawを再導出する入口である。topology、pair/open/row、port identityを再判断してはいけない。

## validationとinspection

`ValidateFast()`、`Validate()`、inspectionはstateを観測して問題を報告する。
不足情報の補完、topologyの推測、state mutationは行わない。

## viewerとrender/export

viewerは`SavedBackboneGraph`、rules、layout、geom、visual/render cacheを読むconsumerである。
viewerが不足したtopology、pair、row、loweringを推測または補正してはいけない。

coreはbackend非依存のcurve、bounds、primitive、style参照を出力する。
UE、Blender、viewer、exporter固有のasset/material型はadapter側で解決する。

現在のsupport visualは、lowered endpointに対する`support_world -> endpoint_world`の`SupportArm` placeholderまでである。
crossarmはsupport group単位、insulatorはlane attachment単位の生成として未実装であり、
`enable_insulators`や`requires_insulator`はbackbone visual出力でまだ完全には消費していない。
viewerが不足fixtureを推測して補ってはいけない。

## cable curve

cable centerlineの正本はBezier制御点ではなく、attachment endpoint、gravity、sag、tangent policy、
canonical direction、curve familyである。`core/src/geometry/curve`がこの意味入力からsample、arc length、
frame、boundsを生成し、具体的な計算方式は`CurveMethod`で差し替える。

main spanの既定方式はparabolic sagとし、中心線へ横揺れnoiseを入れない。
bundle lane、band、helix、noiseは安定したcenterlineとcanonical direction基準frameからvisual layerで展開する。
G2接続は現時点の必須条件ではない。support/insulator leadとjumperはmain spanとは別のcurve familyとして扱い、
未対応familyは別方式へsilent fallbackせず明示的に拒否する。

span-local attachment blend方式は採用しない。continuousな本線接続部を各span端に個別に押し込むと、
sample polyline上でG1が崩れやすく、main spanから接続部へ不自然に切り替わる。terminal endpointには
接続部curveを作らず、continuous endpointもfixtureとして扱わない。次の候補は、junction周辺の
`NodePatchCurve`、その外側を結ぶ`EdgeBodyCurve`、fixture/lead/jumper用の別curveを分ける方式である。
ただし長いrun全体を毎回正本として再計算する方式にはせず、dirty node + incident edge + 必要な1-hop程度の
更新範囲に抑える。

## wire domain境界

wire coreはroad、rail、building、terrain、cityのdomain型を知らない。
外部systemはworld position、wire template/profile、opaqueなexternal anchor tokenへ解決してからcoreを呼ぶ。
`SavedBackboneGraph`はwire topologyであり、city topologyではない。

## Poleの位置づけ

`Pole`は物理support entityであり、topology rootではない。
support nodeはpole、ownerless point、external anchorを表せる。
spanはport間の生成結果であり、topology authorityではない。

## 境界guard

- public API: `core/include/wire/core`
- private generation: `core/src/generation/backbone`
- state ownership: `CoreState`
- read-only query: `CoreView`とconst query
- dependency guard: `tools/arch_lint.py`
- test ownership guard: `tools/test_family_lint.py`
