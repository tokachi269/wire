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

pair row axisは`pairs make(graph)`だけが決める。
通常cornerでは前後linkの単位接線和から二等分方向を作り、その直交方向をrow axisにする。
径間長の差でrow axisを回さず、各incident spanのlane順が反転しない範囲に保つ。
鋭角cornerは共有pair rowにせず、各incident edgeに直交する2つのopen rowと明示jumper relationへ分ける。
jumperはlayout ruleにpeer port identityを保存したderived visualであり、logical spanやSavedBackboneGraph edgeを増やさない。
この判定はbundle templateを読まず、connectivity段の局所幾何だけで決める。
pole facingはこのcorner decisionの`node_forward`を消費し、角度や二等分線を再計算しない。
旧angle correctionは緩角向けの補助に限定し、倍率上限は`kMaxCornerSideScale`(1.7)とする。

### row conflict と endpoint offset

通常のroute bendはlowering対象ではない。
同一nodeのrow conflictで重なりを避ける必要がある場合だけ、`BundleTemplate::enable_branch_down_offset` と
`branch_endpoint_offset_m`を使い、対象bundleのjunction側`endpoint_world`をoffsetする。
`support_world`とport位置は元の取付位置を保持する。
LV/HVなどのcategory名自体はlowering条件にしない。

ownerlessなmidair branchのsource identityは、saved edge、edge bundle、lane、port bindingから特定する。
world接続点はそのsource identityからcurrent curve projectionとして導出する派生値である。
別bundleのportやviewer hit worldから接続点を推測せず、source identityを解決できなければmutation前に`unsupported`とする。
`allow_midair_branch=false`のtemplateはmidair branchの生成対象にしない。
pipeline前半(pairs/intent/groups/topo/emit/save_graph)はsource cableのcurve座標を要求しない。
source-edge由来のownerless portをmaterializeする場合も、その`world_position`はpreview/cacheであり正本ではない。
layout/derive段でsource identityをcurrent curve projectionへ解決し、branch endpointを現在のsource curveへ追従させる。
既存source edgeからのbranchは、事前curve座標をpipeline前半の入力にしない。source edge自体とそこから伸びるbranchを同じ`BackboneSpec`で同時に表す入力形式は現APIにはまだ無いので、二度pipeline実行で補わない。
viewerは後追いでsnap targetを明示し、source-edge snapではhit worldではなくsource edge/t/bundle/laneを渡す。

## pipeline build entry

backbone pipeline の実行入口は `build(build_input)` だけである。通常生成と saved-scope 再生成は pipeline の別実装ではなく、
`build_input` の違いだけである。regenerate は CoreState の post-edit operation 側の概念であり、
pipeline stage の概念ではない。

`build_input` は graph、active bundle scope、local path mapping を運ぶ。通常生成は `prepare` 済み graph から
`build_input_from_spec` で入力を作り、saved-scope 再生成は保存済み backbone identity から復元した graph を
`build_input_from_saved_scope` で入力にする。bundle template や pole type の差分は pipeline input に override として持たせず、
trial/proposal 側の state に反映してから同じ `build` を通す。
ただし `prepare()` はまだ pipeline member に graph を構築する既存構造を残している。
次段階で必要なら、`prepare()` 自体を `build_input` 生成器へ寄せる。
`build` は pairs -> intent -> groups -> topo/emit -> save_graph -> rules -> layout -> geom -> draw の共通stage列を通す。
adapter は pair / emit / rules / layout / geom / draw の判断を持たない。
operation 固有の差分は post-edit API と `regenerate_backbone_edge_bundles` 側に留め、pipeline へ別stageや専用fallbackとして持ち込まない。

pipeline の preflight は、入力・identity・binding・構造上その時点で判定できる失敗だけを早期検出する。
source edge の current curve projection や `EvaluatePosition(source_t)` のように後半の派生 geometry が必要な失敗は、
preflight へ移さない。atomicity は preflight の完全性ではなく、全 stage 成功後にだけ本 state へ反映する
trial/proposal 境界で守る。trial を削除できるのは、MutationPlan、copy-on-write state、rollback journal、
または immutable proposal などの transaction 方式に置換できた場合だけである。

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
`kRegenerate` は topology / identity / connectivity 級差分を分類し、通常更新経路で拒否する境界である。
`execute_update_plan` は `kRegenerate` を恒久的に拒否する。
regenerate は各 post-edit API が編集差分を添えて統一入口を直接呼ぶ。
`UpdatePlan` は差分入力を運ばないため、plan 経由の regenerate 実行は設計として採用しない。

統一 regenerate は、編集差分から影響 scope を解決し、保存済み入力から scope の pipeline graph を組み直し、
既存 pipeline を部分再実行して binding を reconcile する。既存 binding は再利用し、増えたものは生成し、
消えたものは退役する。差分別の migration operation は作らず、対応範囲は scenario 単位で拡張する。
現対応は `UpdateBundleTemplate` の fixed count 増減、`UpdateCableTemplate` の backbone continuity policy decision 差分、`UpdatePoleTypeDefinition` の active backbone pole 構造差分、`ApplyBundleRelatedPoleTypeToExistingPoles` の related pole type 適用、backbone span の endpoint socket / branch-down override、`UpdateLayoutSettings` の全 backbone route 再導出である。
同一 edge に複数 edge_bundle がある場合は saved edge_bundles 順を生成時の bundle spec 順として扱い、
group offset を再構成する。3点以上routeのpair rowは、saved edge の route/order と saved node から
pipeline graph を復元して再確定する。
存続する span の attachment は span id とともに保持する。退役する span に attachment がある場合だけ、
暗黙削除せず mutation 前に `unsupported` で拒否する。`UpdateCableTemplate` の continuity policy は route scope ごとに同じ入口を通し、既存 span の curve decision を編集後 cable template で再導出する。non-backbone span を含む decision 差分は未対応として拒否する。default endpoint attachment 変更は post-edit 経路で attachment 生成/退役/置換をまだ消費しないため、構造反映の実用 scenario まで保留する。

`UpdatePoleTypeDefinition`は、対象typeをactive backbone poleが使用中でもplacement-only差分なら
`kReposition`として既存auto portを再配置し、layout -> geom -> drawを再導出する。
band追加・削除、enabled/side/layer/role/priority変更、anchor slot変更などの構造差分は
対象 pole の incident edge を route-local bundle scope に展開し、統一 regenerate で emit から再解決する。
manual portはtemplate placement更新では動かさない。統一 regenerate でも存続 lane の manual port は
world position と manual marker を保持し、退役 lane の manual port は mutation 前に拒否する。

backbone span の endpoint socket / branch-down override は `override_state` が正本である。
API は本 state を直接書かず、trial state に override を入れて対象 span の edge を統一 regenerate する。
layout rule は override 解決を消費し、socket は endpoint source / resolved socket、branch-down は endpoint offset と curve に反映する。

`UpdateLayoutSettings` は layout settings を trial state に入れ、保存済み edge の route/order/node 連続 component と bundle instance ごとの scope を統一 regenerate する。
route番号だけでは別 generation の edge が混ざるため、scope 復元は同一 route 番号に加えて隣接 node と order の連続性を要求する。

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

main spanの既定方式はparabolic sagとし、支持点でsag勾配を持つ実接線を維持する。
端点微分が0になるdecorative offsetをmain cable centerlineへ使わない。中心線へ横揺れnoiseを入れない。
bundle lane、band、helix、noiseは安定したcenterlineとcanonical direction基準frameからvisual layerで展開する。
G2接続は現時点の必須条件ではない。support/insulator leadとjumperはmain spanとは別のcurve familyとして扱い、
未対応familyは別方式へsilent fallbackせず明示的に拒否する。

span-local attachment blend方式は採用しない。continuousな本線接続部を各span端に個別に押し込むと、
sample polyline上でG1が崩れやすく、main spanから接続部へ不自然に切り替わる。

現在は派生debug/cacheとして`VisualCurvePart`を持ち、最小単位を`NodePatchCurve`と`EdgeBodyCurve`へ分ける。
未接続のterminal endpointには`NodePatchCurve`を作らない。末端へ新しいedgeを延長した場合は、
`pairs make(graph)`がdegree 2のsaved/new edgeをcontinuationとして確定し、そのpair rowをpatchが消費する。
branch追加後もmulti-incident全体を丸めず、pair rowが明示するthrough 2-edgeだけを維持する。
node / bundle template / lane / 保存済みplacement band単位でpatchを分離し、位置近似やband再探索で接続を推測しない。
main cable patchはattachmentを通過せず、incoming/outgoing boundary間を
turn内側で単調に結ぶ1区間filletとする。境界では`EdgeBodyCurve`のparabolic sag実接線とG1接続する。
attachmentは参照として保持し、insulator/clampへの接続は将来の別`LeadCurve`が所有する。
`EdgeBodyCurve`は正式`CableCurve`とadaptive
tessellationを共有する。attachmentは動かさず、boundaryはmain spanの外向き実接線を所定の水平距離まで
延長した位置へ置く。短いspanでは水平距離をspan長の25%以下に制限する。branch自体やfixture境界は、明示的な
fixture/lead/jumper仕様が入るまでpatchを推測しない。source-edge途中分岐のsource projectionは
SavedBackboneSpanBindingから解決した派生curveを評価し、port間chordで補間しない。

`NodePatchCurve`と`EdgeBodyCurve`はtopology正本ではない。source node / edge / span / bundle / lane、boundary point、
boundary tangentをdebug/captureで見えるようにするための派生出力である。描画やexport用に分割してもよいが、
分割後のspan片が接続部curveのauthorityになってはいけない。長いrun全体を毎回正本として再計算する方式にはせず、
dirty node + incident edge + 必要な1-hop程度の更新範囲に抑える。

### cable population

CableInstance / CableSection / carrier の設計語は docs/cable_instance_section.md を参照する。
`CableSectionKey` は section scope の識別子であり、`logical_span_id` を含む。見た目上連続する1本の
identity は `VisualCurvePart.cable_run_id` として visual derive 層で派生する。run は採用済み
NodePatch pair で接続された section の連結成分で、canonical key は成分内最小の
`(edge_bundle_id, logical_span_id, rule_owner_id, rule_id, instance_index)` である。run id は
`SavedBackboneGraph`、binding、template に保存しない。

## BundleTemplate identity

`BundleTemplateId` が bundle template の永続 identity である。`BundleKind` は LowVoltage /
Communication などの default category/tag であり、identity ではない。同じ `BundleKind` を持つ
`BundleTemplate` は複数存在できる。

`bundle_templates` の key、`Bundle.bundle_template_id`、backbone spec / saved graph / binding、
population rule owner、regenerate scope は `BundleTemplateId` を使う。`BundleKind` から一意の
template を引く API は持たない。kind で探す場合は grouping/filter として複数 id を返す。

`BundleTemplate.population_rules` は、その bundle が派生させる追加の平行線を定義する。
rules が空なら追加線は無い。global enable や global seed は持たない。

population は derived output であり、`SavedBackboneGraph`、`Span`、`Port`を増やさない。
layout後に base span と追加線を `CableSectionLayout` へ揃え、同じ `EdgeBodyCurve` /
`NodePatchCurve` 生成へ渡す。追加線専用の curve、tessellation、render 経路は持たない。
section identity は logical span、edge bundle、bundle template、明示 rule id、instance index から作り、
両 endpoint は同じ instance index で対応させる。配置不能時は support や route を作らず omit diagnostic を残す。

rule 変更は `UpdateBundleTemplate` の `kReshape` 差分であり、topology/regenerate 差分にしない。
配置は rule の explicit seed、logical span、edge bundle、rule id、instance index から決定的に導出する。
lateral / height は rule 範囲と endpoint band 範囲の交差内で stable random 配置する。

rule は `CableSectionProfile` を持つ。`kFree` は自前の endpoint と sag で吊る平行線、
`kWrap` は同じ span の base cable を carrier とする巻き付き実線である。wrap section は
carrier の最終 curve(boundary 適用後)から `sample_wrap_helix_points` で centerline を派生し、
独自の sag や band 配置を持たず、`end_trim_m` で support 手前に留まり、node patch へ参加しない。
carrier は rule 宣言から解決し、geometry 近傍から探さない。位相は instance index で等分し、
巻き方向は rule の +1/-1 で明示する(C689/C690)。

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
