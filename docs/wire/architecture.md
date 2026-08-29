# Wire architecture

このドキュメントは、現在の backbone 生成本流の契約をまとめる。
現在の実装は移行中に `bb2` と呼んでいたが、production code と現行testは `backbone` を正とする。
操作前の状態と操作の組合せごとの期待遷移は
`backbone_operation_semantics.md`を正本とする。共通契約は`../architecture.md`を参照する。
未定義セルは実装で補完しない。

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
  -> derived decoration materialization
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
| derived decoration | Core visual generation | support/span周辺の局所設備・短い配線・inline deviceをWire domainの意味から派生 |
| settings | `CoreStateAuthoritativeStorage` | geometry / visual / variation / context / layout のユーザー設定 |

生成済みのspan、layout、curve、bounds、visual、port位置からtopologyを復元してはいけない。
同じ意味を複数段で再判断せず、下流は上流の決定済み値だけを消費する。
ユーザーが Update API で設定し derived 出力に影響する値は authoritative に置き、runtime cache に mirror を持たない。

### support / inline detail

support detail は authoritative entity ではないが、Wire domain の意味を必要とする derived geometry /
materialization である。したがって生成、配置、接続先解決、curve、material semantics は Core visual generation が
所有する。viewer / export adapter は Core が返した `VisualModelInstance` と `VisualCurvePart` を消費し、
高さ、接続、material、可視性、support identity を再判断しない。

個々のlocal cable、fan-out、inline device、primitive equipmentは通常Spanではなく、SavedBackboneGraph、
Span、Bundle、Port bindingへ保存しない。support/inline detail は通常の derived visual cache として
正本から毎回再導出する。

GLBを読む責務はasset adapterに留める。adapterはmodel key、local transform、名前付きsocketのmetadataを
`ModelAssemblyTemplate` / `ModelAssemblyPart` / `ModelAssemblySocket` としてCoreへ渡すが、`connect_*`
socketからcarrierを探したり、local cable curveやmaterialを決めたりしない。socket名のWire上の意味はCoreだけが
解釈する。CoreはGLB parserを持たない。

未完成のsupport detail catalogは、Coreの派生・保存契約を変えずviewerのpresentation availabilityとして
非表示にしてよい。この境界で配置、接続先、curve、material semanticsを再計算してはいけない。

### session draft state

`ResolveBranchPick()` が作る pending support node は、次の draw request へ pick 結果を渡すための
session draft である。authoritative topology ではなく、保存対象ではない。

pending support node の生存期間は次の通り。

- `ResolveBranchPick()` は必要な場合だけ pending support node を作る。dry-run は作らない。
- `GenerateFromBackboneSpec()` が pending node を参照して成功した場合、その pending node は消費済みとして削除する。
- draw path の cancel / clear は `ClearPendingSupportNodes()` を呼び、未消費 pending を破棄する。
- save / load は pending を保持しない。load 後に古い pending node id が request に残っていた場合、
  preflight は mutation 前に `unknown node reference` として拒否する。

生成失敗時は本stateへcommitしないため、pendingの消費も行わない。retryやclearは同じsession draft契約に従う。

## 永続化契約

保存対象は identity と authoritative storage のみとし、runtime cache と debug storage は保存しない。
load は保存済み topology / binding / settings から既存 pipeline を通して layout、curve、bounds、visual を再導出する。
同一stateの save -> load では authoritative の再保存byteが一致し、派生出力の意味値と浮動小数bitが一致することを
roundtrip等価性とする。runtime固有のversionや計測値、container addressは等価性に含めない。

永続形式はversionを完全一致で判定する。未知field、必須field欠落、truncation、重複field、構文不正は拒否し、
部分的に読み飛ばさない。loadは新しいtrial stateでparse、index再構築、派生再導出、validationを完了してから
member-wise move commitする。どの段階で失敗しても、本stateは変更前と同一でなければならない。

## backbone generation

`GenerateFromBackboneSpec()` は `domains/wire/src/generation/backbone` のpipelineだけを呼ぶ。
未対応入力はv1へfallbackせず、mutation前に`unsupported`を返す。
外部入力の数値検証はpipeline preflight先頭の `validate_backbone_spec_external_input` が所有する。
対象は `BackboneSpec.path.polyline`、`NodeSpec.tangent_hint`、`interval_m`、`constraints.avoid_points`、
`constraints.avoid_radius_m`、`constraints.lateral_offset_m`、`pole_placement.max_tilt_deg`、
および各 `BackboneBundleSpec` の `height_m` / `lateral_m` / `spacing_m` である。
NaN / inf、負のinterval、負のavoid radius、負のmax tilt、負のspacingは mutation 前に `invalid input` として拒否する。

### EditResult error kind

`EditResult` は人間向けの `error` 文字列に加えて、機械可読な `CommitFailureCategory` と `reason_code` を返す。

- `kValidation`: 外部入力が不正で、ユーザー入力またはadapter payloadを直せばよいもの。
- `kUnsupported`: 入力は読めたが、現在の仕様で扱わないもの。分類に迷う既存エラーはここへ倒す。
- `kInternal`: 保存済み正本や派生再構築の整合が壊れており、通常操作では起きてはいけないもの。

既存の `error` 文字列は診断情報として維持する。境界adapterは `effective_failure_category()` で分類済み値を読み、
表示層は文字列prefixを再解釈しない。

処理順は次の通り。

1. inputの`prepare`と`check`
2. graphから操作中endpointの候補関係と暫定rowを確定
3. duplicate edge bundle/span bindingをpreflight
4. intentとsupport groupを確定
5. pole、bundle、port、spanを生成
6. `SavedBackboneGraph`とbindingを保存
7. rules、layout、geom、drawを保存

context linkは判断入力であり、生成・保存対象ではない。
T/cross/branchのkind enumは作らず、continuityと派生rowの組合せで表す。

`preserve_conductor_identity=false`かつ`order_decision_policy=kPermutableHomogeneous`の
multi-lane rowは、XY上で横一列に並ぶ配置だけをsupportedとする。両端の
`last lane - first lane`のXY方向を比較し、dotが負の場合だけ片端のlane対応を全体反転する。
これは1 bitのmirror決定であり、任意permutationを探索しない。first/lastのXY方向が得られない縦積み、
または両row方向が直交して1 bitで決められない配置はfallbackせずunsupportedとする。

promotionで既存Portのrow frameが変わる場合は、全Port frameの確定後に`emit_ports`が
`is_new=false`のcontext edgeだけを対象とし、既存edgeの反対側rowに対するmirrorを最終位置から1回だけ決める。
Port entityは維持し、対象edge bundleのPortBindingとSpan endpointをidentityまたは全体reverseのどちらかで
一括更新する。context linkはこの判断入力にだけ使い、`tspan`化または`save_graph()`の保存対象にしない。
`emit_spans`は`is_new=true`のedgeだけを対象とし、新規Span endpointを最終Port位置から同じ1 bit規則で決める。
同じedgeを両経路で処理してはいけない。

異なるedge bundleを`SavedBackboneRowContinuity`で接続する場合も、permutable laneの対応は
両rowの最終Port列`last lane - first lane`のXY方向から1 bitだけ決める。方向のdotが負ならB側laneを
全体反転し、非負なら維持する。peer edgeの選択は既存のcontinuity候補規則が所有し、この幾何判定は
peer edgeを選ばない。鋭角判定やJumper materializationはlane対応を再決定せず、保存済みcontinuityの
異なるA/B lane indexを消費する。横方向が得られない、または両row方向が直交して1 bitで決められない
配置はfallbackせずunsupportedとする。

Span endpoint対応とcross-edge continuity対応は異なる正本関係である。前者は最終`trow`、後者は
Span endpoint反映後の`SavedBackbonePortBinding`を各1回だけ読む。生成中の物理slotである`trow` laneを
保存済みbinding laneの代用にせず、両経路は上記の1 bitベクトル判定だけを共有する。

`PathDirectionMode`はユーザーが引いた向きの意味を持ち、signed lateral offsetやsource/branchの進行方向へ
適用する。permutable laneのnon-crossingとcanonical topology identityは`PathDirectionMode`へ依存させない。
同じ物理pathをReverseで生成した場合、signed lateral offsetの物理側は反転する。

midair branchのLeadは`SourceEdgeProjectionRef.from_node_id`が示すsource edge方向へsource curve tangentを
向けてからbranch boundaryへ接続する。branch endpointへの位置ベクトルでsource tangentの符号を選ばない。
attachment pointはsource curve projectionのまま維持し、branch側は確定後のEdgeBody boundary tangentを使う。

### pole / port配置座標

pole local frameの配置原点は、poleのtiltを含む中心軸とする。mesh表面やmesh下端を原点にしない。
`PortPlacementBand.lateral_center_m`は中心軸から測ったbandの既定位置であり、
`BackboneSpec.constraints.lateral_offset_m = 0`は選択されたband位置から追加移動しないことを意味する。

`BackboneSpec::bundles` の各要素は1つのBundle placementである。同じ`BundleTemplateId`を
複数要素から参照してよく、各要素は独立した`Bundle` identityを生成する。placementの
explicit height/lateralはpole local中心軸を原点とする絶対位置であり、spreadは`Bundle.phase_spacing_m`
としてBundleが所有する。Pole band、category、個別Spanをplacement identityとして代用しない。
主線、endpoint fixture、support path、helixは同じBundle placementから解決されたPortを読む。
explicit placementでもpole band identityはfixture・roleの解決に使うが、そのheight/lateral centerを
配置値へ加算しない。legacy/API入力でexplicit指定がない場合だけband既定位置を使用する。

同一category/layerにlane数ぶんの異なるlateral位置を持つbandがある場合、laneはpriorityで採用したbandを
lateral順に1つずつ使用する。既定HV 3相は左・中央・右bandを各laneが使用し、3相全体を片側のpole表面へ寄せない。
異なるband位置がlane数に足りない場合だけ、priority最上位の1 bandをrow中心としてlane spacingを展開する。
保存済みport bindingはlaneごとのplacement band identityを保持する。
`SavedBackbonePortBinding::lane_index`はSpan側のlogical conductor identity、`port_id`はそのendpointで選ばれた
physical trow laneのPort identityである。`placement_band_id`は`port_id`の実physical placementに属し、
mirror時にlogical laneとphysical laneが異なる場合もlogical lane番号から再選択しない。

pole表面へ直接取り付けるportや部品は、中心軸原点を変えず、その高さのsection半径とstandoff / clearanceから
表面位置を導出する。表面位置を既定offsetへ混ぜず、laneごとに後処理してbundle重心をずらしてはならない。

接続相手は`SavedBackboneRowContinuity`だけが保持し、row表現は共通のendpoint row導出が現在幾何から決める。
生成中routeの隣接も同じcontinuityへ記録する。route/orderは永続化しない導出補助であり、接続相手やpair/open表現の判定入力にしない。
saved load / regenerateのroute復元は、最初に`edge_bundle_id`単位のrow continuity componentを確定し、
そのcomponent内だけをphysical edge routeへ投影する。同じphysical edgeが複数の独立bundle componentへ属する場合、
physical edgeは各componentの派生routeへそれぞれ現れてよい。scope/component確定前に`edge_id`だけのglobal adjacencyへ
collapseせず、bundle template IDや同じtemplateを使う別Bundle placementをcomponent identityの代用にしない。
通常cornerでは前後linkの単位接線和から二等分方向を作り、その直交方向をrow axisにする。
径間長の差でrow axisを回さず、各incident spanのlane順が反転しない範囲に保つ。
鋭角cornerはcontinuityを維持したまま、各incident edgeに直交する2つのdead-end rowとjumperへ派生する。
jumperのpeer portはcontinuityから導出し、layout ruleやPortへ接続正本として保存しない。jumperはlogical spanやSavedBackboneGraph edgeを増やさない。
この判定はbundle templateを読まず、connectivity段の局所幾何だけで決める。
pole facingはこのcorner decisionの`node_forward`を消費し、角度や二等分線を再計算しない。
旧angle correctionは緩角向けの補助に限定し、倍率上限は`kMaxCornerSideScale`(1.7)とする。

### row conflict と endpoint offset

通常のroute bendはlowering対象ではない。
同一nodeのrow conflictでは、物理fixture rowごとに1 support levelを割り当てる。level 0は基準位置、
level 1以降は`abs(BundleTemplate::branch_endpoint_offset_m) * level`だけ順に下げる。
1 levelへ複数の物理fixture rowを載せない。通常pairは共有rowなので1つのlevelだけを使うが、
鋭角pairは2つのfixture rowとjumperへ派生するため、2つの異なる空きlevelを使う。
jumperはcontinuity表現であり、2つのrowを同じplacement levelへまとめる根拠にしない。
この多段配置は`BundleTemplate::enable_branch_down_offset`が有効なbundle placementだけに適用し、
無効なplacementはrow数に関係なくlevel 0を維持する。
`SavedBackbonePortBinding`はrowごとの`support_level`と`support_group_id`を保存し、
save/loadやincremental generationで同じ配置判断を再利用する。
Port生成はpole bandまたはexplicit placementの論理anchorだけを使い、row数や保存済みPort高さから
別のZ slotを決めない。support level/groupを確定するrow placementが唯一の段差decisionであり、
Span layoutはそのdecisionから解決済みのbranch endpoint offsetを読む。
段変更後の最終wire socketを`support_world`と`endpoint_world`の両方に使い、port位置は論理anchorとして保持する。
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
preflight へ移さない。post-edit regenerate の atomicity は preflight の完全性ではなく、全 stage 成功後にだけ本 state へ反映する
trial/proposal 境界で守る。trial を削除できるのは、MutationPlan、copy-on-write state、rollback journal、
または immutable proposal などの transaction 方式に置換できた場合だけである。通常生成も isolated trial を通す。

## public view の参照寿命

`CoreState` / `CoreView` から取得した pointer、reference、view は、同じ `CoreState` に対する次の non-const operation
まで有効である。non-const operation の成功・失敗を跨いだアドレス同一性は保証しない。長期保持が必要な consumer は
`ObjectId` を保持し、操作後に再取得する。

| API / result | 参照元 storage | 無効になり得る操作 | mutation 跨ぎ安定性 |
|---|---|---|---|
| `SpanLayoutView` / `SpanLayoutRulesView` | `SpanLayoutCache.records_by_span` の `unordered_map` 内 `optional` | record erase、layout/rule 再保存、storage 代入。insert の rehash は iterator を無効化する | 保証しない |
| `CoreView` の Pole / Port / Span / Bundle / Attachment pointer、`PoleDetailInfo` | `EditState` の `ObjectStore` | vector の insert/reallocation、erase の末尾要素移動、storage 代入 | 保証しない |
| backbone node / edge / binding pointer と CoreView の map/vector reference | `SavedBackboneGraph`、runtime index、debug vector/map | vector insert/erase、map erase/rehash、storage 代入 | 保証しない |
| curve / bounds / visual / render cache pointer、visual curve cache reference | runtime cache の `unordered_map` / vector | cache entry erase/再保存、vector 更新、storage 代入。rehash は iterator を無効化する | 保証しない |
| inspection result 内 pointer (`PoleDetailInfo` 等) | 上記 view が指す storage | 上記と同じ | 保証しない |

現在 public contract として mutation を跨ぐ参照安定性を保証する consumer はない。既存 test も value / id / ChangeSet を観測し、
pointer address を invariant にしてはならない。

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

### transaction 契約

preflight は、入力・identity・binding・構造条件の失敗を mutation 前に検出する。
pipeline 後半では projection 評価など派生 geometry 固有の失敗が起こり得る。
これを preflight へ移すことは C720（front half は curve projection を読まない）に反するため行わない。
post-edit regenerate と通常生成の commit は全 stage が成功したときだけ本 state へ反映する。どの stage で失敗しても、本 state は変更前と同一でなければならない。
trial（state copy）はこの failure 保証の現行実装であり、MutationPlan / journal / copy-on-write 等の代替 transaction 方式へ置換できた場合だけ削除できる。
preflight を増やしたことを理由に本 state 直接変更へ戻すことは禁止する。

`GenerateFromBackboneSpec` は state copy の isolated trial で `prepare`、`check`、`build` を実行し、成功時だけ storage を move commit する。
`GenerationTiming.state_copy_ms` は copy コストを記録し、66 pole級 populated state に対する copy gate は generation total の20%以内とする。

統一 regenerate は、編集差分から影響 scope を解決し、保存済み入力から scope の pipeline graph を組み直し、
既存 pipeline を部分再実行して binding を reconcile する。既存 binding は再利用し、増えたものは生成し、
消えたものは退役する。差分別の migration operation は作らず、対応範囲は scenario 単位で拡張する。
`UpdateBundleTemplate` の fixed count 増減は、同じtemplateを使う全placementを一括scopeにせず、
exact `Bundle` identityごとにsaved row continuityで接続された`edge_bundle_id` componentを選び、全component成功後に一度だけcommitする。
`UpdateBackboneBundleConductorCount` はexact `Bundle` identity、現在の`BundleTemplate` count policy、要求countを入力とする。
`kRange`は範囲内の要求だけを受理し、`kFixed`の個別変更は拒否する。lane topologyは先頭からのprefixとして扱い、
増加時は既存lane identityを保って末尾laneだけを生成し、減少時は末尾laneだけを退役する。
全continuity componentを同じtrialで既存pipelineへreplayし、binding reconcileが完了した場合だけcommitする。
退役laneにuser Attachmentまたはmanual Portがある場合はauthority不変で拒否する。
`RetireBackboneBundle` はexact `Bundle` identityから、そのBundleを参照する全`SavedBackboneEdgeBundle`を
完全退役するCore topology operationである。削除ownerは既存pipelineの`retire_untouched`に置き、
Span、専用Port、binding、row continuity、default endpoint Attachment、orphan edgeを同じtrialで除去し、
derived outputとruntime indexを残存authorityから再構築してから一度だけcommitする。
branch / cross / sharpはcategoryで除外せずexact Bundle identityで全componentを対象にする。
source-edge branch Bundleはsourceを残したまま退役できるが、source Bundleを消すと存続branchのprojection
authorityが失われる場合はdependencyを推測せずmutation前に拒否する。user Attachment、manual Port、
Span overrideを暗黙に破棄してはならない。
`AddBackboneBundleInstance` はexact anchor `Bundle`が所有する全`SavedBackboneEdgeBundle` membershipと
`SavedBackboneRowContinuity`のlane対応を唯一のoracleに、同じtemplate、conductor count、lane policyを持つ
別`Bundle` identityを追加するCore topology operationである。callerは非zero `placement_key`とexplicit
height / lateral / spacingだけを指定する。anchor relationから作る一時的なrow continuity constraintを
既存pipelineのpairing入力へ渡し、pairing -> intent -> support grouping -> emit -> save_graphのstage順を維持する。
通常の候補規則で同じpairになったことを複製の根拠にせず、save_graphもconstraintのlane対応をそのまま保存する。
constraintはInputであり保存しない。新Bundleのmembershipとcontinuityがanchorと同型であることをtrial内で
検証し、全componentとderived/runtime rebuildが成功した場合だけ一度commitする。anchor、peer、既存Port / Spanを
変更または共有しない。branch / cross / sharpはanchor relationを複製できる限り対象とする。
source-edge dependencyはexact source Bundle mappingがInputにない現段階では推測せずmutation前に拒否する。
Attachment、manual Port、Span overrideは新instanceへ複製しない。
`ReconcileBackboneBundleInstances`はcallerが明示したexact current Bundle IDsとdesired concrete
`BackboneBundleSpec`集合をplacement keyで対応付けるCore topology operationである。scope membership、route、
branch membership、anchorをcategory、template名、位置、配列順から推測しない。desired-only entryはcallerが
current scope内のexact anchor Bundle IDを指定し、survive / add / retireを1つのouter trialで既存のplacement、
count、instance add、exact retire operationへ委譲する。desiredはnonzeroでscope内一意のplacement key、既存templateの
default layer、positive spacingを持つexplicit placementに限定する。survivorはObjectIdと存続lane identityを維持する。
addはanchorと同じtemplate / lane topologyに限り、0 -> 1やsource-edge exact mappingを推測しない。
input vector順は意味を持たずplacement key順に処理する。explicit placementだけを対象とするため、個別Addでappendされる
saved edge-bundle順をplacementやgroup offsetの新しいauthorityにしない。全operation、derived/runtime rebuild、validation、
desired集合postconditionが成功した場合だけ一度commitし、失敗時はouter stateを変更しない。
現対応は `UpdateBundleTemplate` の fixed count 増減と `kTopology` policy 差分、`UpdateCableTemplate` の backbone continuity policy / default endpoint attachment decision 差分、`UpdatePoleTypeDefinition` の active backbone pole 構造差分、`ApplyBundleRelatedPoleTypeToExistingPoles` の related pole type 適用、backbone span の endpoint socket / branch-down override、`UpdateLayoutSettings` の全 backbone route 再導出である。
同一 edge に複数 edge_bundle がある場合は saved edge_bundles 順を生成時の bundle spec 順として扱い、
group offset を再構成する。3点以上routeの接続は saved row continuity と saved node から
pipeline graph を復元し、row表現は現在幾何から再導出する。
row key / lane が一致する binding は再利用し、不一致の binding は retire + emit で reconcile する。存続する user attachment は span id とともに保持し、退役spanに user attachment があれば mutation 前に `unsupported` で拒否する。`AttachmentOrigin::kDefaultEndpoint` は trial 内で退役できる。`UpdateCableTemplate` の continuity policy と default endpoint attachment は route scope ごとに同じ入口を通し、既存spanのcurve decisionとauto endpoint attachmentを編集後 template へreconcileする。non-backbone span を含む decision 差分は未対応として拒否する。

`UpdatePoleTypeDefinition`は、対象typeをactive backbone poleが使用中でもplacement-only差分なら
`kReposition`として既存auto portを再配置し、layout -> geom -> drawを再導出する。
band追加・削除、enabled/side/layer/role/priority変更、anchor slot変更などの構造差分は
対象 pole の incident edge を route-local bundle scope に展開し、統一 regenerate で emit から再解決する。
manual portはtemplate placement更新では動かさない。統一 regenerate でも存続 lane の manual port は
world position と manual marker を保持し、退役 lane の manual port は mutation 前に拒否する。

backbone span の endpoint socket / branch-down override は `override_state` が正本である。
API は本 state を直接書かず、trial state に override を入れて対象 span の edge を統一 regenerate する。
layout rule は override 解決を消費し、socket は endpoint source / resolved socket、branch-down は endpoint offset と curve に反映する。

`UpdateLayoutSettings` は layout settings を trial state に入れ、row continuity で接続された edge_bundle component と bundle instance ごとの scope を統一 regenerate する。
scope 復元は saved row continuity component を正本とし、保存済み edge の route/order は判定入力にしない。

`DeriveGeneratedSpanOutputs()` は、保存済み rules / layout source / `SavedBackboneGraph` binding から
layout、geom、drawを再導出する入口である。topology、continuity、port identityを再判断してはいけない。
row/fixture/patch/jumperは保存表現を読まず、continuityと現在幾何から再導出する。

## validationとinspection

`ValidateFast()`、`Validate()`、inspectionはstateを観測して問題を報告する。
不足情報の補完、topologyの推測、state mutationは行わない。

## viewerとrender/export

viewerは`SavedBackboneGraph`、rules、layout、geom、visual/render cacheを読むconsumerである。
viewerが不足したtopology、pair、row、loweringを推測または補正してはいけない。

coreはbackend非依存のcurve、bounds、primitive、style参照を出力する。
UE、Blender、viewer、exporter固有のasset/material型はadapter側で解決する。

lowered endpointでは、段変更を反映した最終fixture socketを`support_world`と`endpoint_world`の両方に使う。
旧`support_world -> endpoint_world`の`SupportArm` placeholderは生成しない。
row fixtureとPort fixtureはgeneric model assemblyから派生し、旧い用途boolや高さscalarを決定者にしない。
viewerが不足fixtureを推測して補ってはいけない。

## cable curve

cable centerlineの正本はBezier制御点ではなく、attachment endpoint、gravity、sag、tangent policy、
canonical direction、curve familyである。`domains/wire/src/geometry/curve`がこの意味入力からsample、arc length、
frame、boundsを生成し、具体的な計算方式は`CurveMethod`で差し替える。

main spanの既定方式はparabolic sagとし、支持点でsag勾配を持つ実接線を維持する。
端点微分が0になるdecorative offsetをmain cable centerlineへ使わない。中心線へ横揺れnoiseを入れない。
continuity policyは端点接線とhandleを決めるが、main spanのsag profileを別方式へ切り替えない。
`CableTemplate.sag_factor`はspan全体に1回適用する単一のratioであり、start/endごとの加算値ではない。
Endpoint constraintを経由するcurve方式でも、両端へ同じratioを重複適用してはならない。
このratioが指定する物理sag量は`endpoint chord length * ratio`であり、span長、pass種別、continuity、
曲げ剛性を理由に別倍率で再解釈しない。これらは端点接線やhandleを決めても、sag量を変更しない。
bundle lane、band、helix、noiseは安定したcenterlineとcanonical direction基準frameからvisual layerで展開する。
G2接続は現時点の必須条件ではない。support/insulator leadとjumperはmain spanとは別のcurve familyとして扱い、
未対応familyは別方式へsilent fallbackせず明示的に拒否する。

span-local attachment blend方式は採用しない。continuousな本線接続部を各span端に個別に押し込むと、
sample polyline上でG1が崩れやすく、main spanから接続部へ不自然に切り替わる。

現在は派生debug/cacheとして`VisualCurvePart`を持ち、最小単位を`NodePatchCurve`と`EdgeBodyCurve`へ分ける。
未接続のterminal endpointには`NodePatchCurve`を作らない。末端へ新しいedgeを延長した場合は、
一意な未接続endpointが2つ揃った操作でcontinuityを記録し、通常角ならそのpairをpatchが消費する。
branch追加後もmulti-incident全体を丸めず、continuityが明示するthrough 2-edgeだけを維持する。
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

scoped visual rebuildでは、`changed spans`、その端点でconnection visualを書き換える`affected nodes`、
materializationのため読むだけの`context spans`を分ける。EdgeBody等のspan-owned partはchanged spanだけ、
NodePatch/Jumper等のnode-owned partはaffected nodeだけを削除・置換する。context spanの反対側nodeは
削除対象へ昇格させない。context不足時に既存connectionを削除してsilent skipすることは禁止する。

### route bundle variation

Decision は route bundle variation、Owner は Wire generation input resolver とする。入力は人間が記述した
random bundle rule、route seed、preferred side、`PoleTypeDefinition` / `BundleTemplate` 定義であり、出力は
concrete な `BackboneBundleSpec` entries である。consumer は既存 backbone generation pipeline だけで、
pipeline / layout / curve / viewer は randomization を再判断しない。

random rule と route seed は自動的にphysical topologyを再構築するauthorityではなく、explicit Apply用の
descriptorとして保存できるInputである。resolver は初回生成または明示Apply時だけ Bundle 数とpole-local の
height / lateral を確定し、通常の Bundle / Port / Span / saved placement として保存する。load時はdescriptorを
resolveせず、保存済みconcrete topologyをそのまま復元する。persistent validationもdescriptorの構造と保存relationを
検査し、current sampling/hash algorithmが同じplacement key集合を返すことはload条件にしない。Generate / Applyの
transaction内だけ、その場のresolved resultとcommit対象instanceを厳密対応させる。
ruleのconductor countはtopology上の導体数であり、見た目の束を作るためには変動させない。
HVの3相配置はrandomization対象外とする。非HVのsupported placementは
連続座標をそのまま使わず、同じroute-wide sideにある有限のsupport slotへ寄せる。
load、regenerate、通常 update は保存済み concrete placement を使い、reroll しない。同一 Bundle の placement を
Pole ごとに変えず、同一 route 内で side を反転しない。road-facing side は外部から与える単純な sign であり、
Wire は Road domain や道路の意味を解釈しない。
保存済みdescriptorはexact Bundle scopeと、0件になった後の再materializeに必要なphysical edge ID /
row continuityを保持する。参照中の`SavedBackboneGraph` node / edge skeletonは最後のBundleが0件になっても保持し、
variation側へnode・edge geometry・代表Bundle placementを複製しない。現在存在するphysical objectのauthorityは
Bundle / Port / Span / SavedBackboneGraphであり、0件時に残るgraph skeletonとvariation membershipはfuture replay authorityである。descriptorは
branch membershipやpairingを再判断しない。`ApplyBackboneBundleVariation`はdescriptorをresolveしたdesired concrete
specを既存reconcileへ渡し、descriptor更新とconcrete topology更新を同じouter trialでcommitする。1 -> 0 -> 1は
保持されたgraph edgeとmembership continuityをpipelineのexplicit constraintとして再生するが、initial 0 -> 1はmembership sourceがないため
unsupportedとする。recipe-backed Bundleの個別placement/count/add/retireは許可せずexplicit Applyへ集約する。
`placement_key`は1つのvariation scope内でdescriptorとconcrete instanceを対応付けるcorrelationであり、
repository全体のBundle identityではない。physical identityは`Bundle::id`であるため、同じseedの別variationが同じ
`placement_key`を持っても互いのscopeへ入らない。

`ExtendBackboneBundleVariation`のBundle membershipはCore-ownedである。adapterはpath / exact node referenceだけを渡し、
Coreが保存済みinstanceからfull concrete Bundle specを再構築して通常pipelineへ渡す。callerがpartial Bundle specを渡す
extensionは拒否する。0件のrule groupも、別のlive groupによるextension後に保存membershipを同じexact pathへ更新し、
後の1 -> 0 -> 1で古いroute shapeへ戻さない。
0件groupを延長するときは通常pipelineをpairing / support grouping / continuityの唯一のownerとして使うため、trial内だけ
auto placementのtemporary Bundleをmaterializeする。結果からedge IDとcontinuityだけをcaptureし、temporary Bundleは保存しない。
support node復元、physical edge復元、representative placement保存の各経路は持たない。
variation extensionのbranch pickも、Coreがvariation IDからexact live instanceを引き、そのBundleTemplate集合だけを既存
`ResolveBranchPick`へ渡す。descriptorにruleがあっても0 instanceのtemplateはphysical branch scopeへ含めない。
WASM/Webは選択Spanのexact `bundle_id`から保存済みdescriptorを参照し、選択中scopeの調整を
`ApplyBackboneBundleVariation`へ渡す。adapterはcategory、template名、geometryからscopeを推測せず、初回生成も
Webでresolveしたconcrete specを通常Generateへ渡す経路ではなく、descriptorをCoreのvariation生成入口へ渡す。
既存routeの調整値は保存済みdescriptorに対する相対値であり、neutral Applyはrulesを変えない。active draw中の同じ
variationへのApplyと個別Bundle編集は拒否し、preview / commitは同じCore variation operationをtrial / real stateで
それぞれ実行する。viewerはrecipeを解釈せず、Apply後の通常sceneだけを描画する。

非HV endpointの支持表現は、authoritative Bundle placementからgeneration時に導出する。pole surface近傍は
Direct attachment、明確に離れたplacementはSupported rowとし、新しいSupportRow entityや保存fieldは作らない。
同じpole、同じrow方向、同じside、互換fixture semanticsで高さが近いBundleはdeterministicに1 rowへまとめる。
LVはLV内、CommunicationとOpticalは相互にだけ共有候補とし、rowのreachは最外memberとmarginから求め、
短すぎる支持物は生成しない。supportの有無、grouping、高さ、reachはCore derived placementが所有し、viewerの
asset adapterは形状だけを提供する。support groupingはrow fixture materializationより先に確定し、Supported rowは
shared supportを1つだけ生成してper-Bundle row fixtureを生成しない。endpoint fixtureはlogical cableごとに必要な数だけ
生成し、visual member数では増やさない。

旧 cable population の visual-only duplication は退役する。Bundle 本数の variation を
`CableSectionLayout` の追加描画として生成せず、すべて通常の authoritative topology として生成する。

## BundleTemplate identity

`BundleTemplateId` が bundle template の永続 identity である。`BundleKind` は LowVoltage /
Communication などの default category/tag であり、identity ではない。同じ `BundleKind` を持つ
`BundleTemplate` は複数存在できる。

`bundle_templates` の key、`Bundle.bundle_template_id`、backbone spec / saved graph / binding、
random bundle rule、regenerate scope は `BundleTemplateId` を使う。`BundleKind` から一意の
template を引く API は持たない。kind で探す場合は grouping/filter として複数 id を返す。

## span visual assembly

span visual assembly は derived visual output であり、authoritative topology ではない。
assembly の単位とidentity ownerはsource logical span / Bundleであり、Bundle の全 topology laneを物理的に束ねるものではない。
1 logical spanは設定により1本または複数の近接visual memberとして描画できるが、Span、Port、Bundle、attachment、
CableRun identityは増やさない。

members はbase sectionと、そのlogical cableをつなぐNodePatch / Lead / Jumperから派生するvisual構成要素である。
線種ごとの`SpanVisualAssemblyTemplate`が基準となる形状を所有し、globalな線の乱れ倍率は
`VisualSettings`がcenter wanderとmember-relative wanderへ同じ比率で作用させる。倍率は基準値を
置き換えず、HVの物理lane/crossarm形状には作用しない。
main spanのsupport path とmembersはhelixの内側に置き、support pathは内周上部に接し、membersは下側に配置する。
helix は endpoint trim 区間だけ生成し、電柱や attachment へ接続しない。

visual memberの断面はcenter curveに直交するlateral/up 2次元平面へcompactに配置する。1本はcenter、
2本は対向、3本は三角形、4本は正方形相当、5本以上は小さな決定的円形配置とし、packing solverは持たない。
`visual_member_spacing_m`はこの基準断面のmember中心間隔である。基準断面offsetはendpointでも維持し、
全memberをlogical endpointの1点へ収束させない。authoritative Port / logical endpointはvisual member endpoint群の
重心であり、member数に応じてPort、Span、Bundle、attachment、fixtureを増やさない。

Communication、Optical、support path、helix、member twist、member wanderは同じlogical-span assembly pipelineの
設定差で表現する。別のedge-bundle groupingやcategory専用wander ownerを持たず、geometry近傍からmemberを探索しない。

非HV main spanはsingle-memberを含め、arc length `s` に対するdeterministicなcenter-path offsetを全中間sampleへ適用する。
physical bundleではこのcenter variationに、より小さいmember-relative variationを重ねる。support pathとhelixも同じ
center pathへ追従する。center variationのenvelopeはspan長を`L`として
`E(s) = 16 (s/L)^2 (1-s/L)^2` とし、`E(0)=E(L)=0`かつ`E'(0)=E'(L)=0`を満たす。
したがってattachment endpointとendpoint tangentを維持し、直線区間とwander区間の離散境界を作らない。
HV main spanとHV arrangementにはcenter variationを適用しない。

main spanのsagは`ResolvedSpanCurveInputs.effective_sag_ratio`を最終curveまで一貫して使う。非HVは既存の
hierarchical variationから小さな差を導出し、HVはvariation multiplierを適用せず従来値を維持する。

`BundleTemplate.span_visual_assembly` はvisual member数・間隔、center variation、member-relative variation、
support、helixを含むassemblyの正本設定である。visual member数はsaved topologyへ保存せず、保存済みBundle placementから
決定的に再導出する。radius が 0 の場合は、
support path からの member offset、member wire radius、helix wire radius、clearance を含む最小半径を
derived 側で求める。contained member は support path のnormalized arc-length位置へ対応付け、
helix内周から出ないように断面offsetをclampする。member-relative variationは基準packingと線径の間に残るmarginの
`member_wander_ratio`分だけを使い、endpoint近傍では弱めても基準断面offsetは弱めない。arc lengthに沿う少数の
低周波成分をlateral/upの両方向へ適用し、sampleごとの独立random、member交差、containment radiusからの離脱を許さない。
CommunicationとOpticalは同じpacking、containment、margin-based wander処理を使う。Opticalのsupport path、member、helixは
同じcenter pathとcontainment radiusを共有する。member数・phaseはstableなBundle placement keyとlogical laneから
決定的に導出し、load、derived rebuild、regenerateでrerollしない。
NodePatch / Lead / Jumperもmain spanと同じBundle placement key、logical lane、compact cross-sectionを使い、接続区間だけ
center curve 1本へ戻さない。support pathとhelixはmain spanだけの補助表現であり、接続区間へ重複生成しない。
明示radiusは、support wireとhelix wireの径およびclearanceを収められない値を設定時に拒否する。

support path は helix と独立して有効化できる。全support pathはendpoint解決後に
make_primary_curve_betweenで主曲線を1回構築する。support_wire_pole_band_id == 0は
endpoint fixture socketまで解決済みのmember endpointを入力とし、endpoint_trim_m区間で同じ接続点へ
収束しながら中央部を線径分だけ離す。正数bandは明示band endpointを同じ主曲線生成へ入力する。
helixはどちらのsupport endpoint方式でも利用できる。既定OPTICALはband 0を使い、placement高さ変更時も
support、contained member、helix断面を同じmember endpointから再導出する。
したがってHV/LV/Opticalでcurve familyを分岐せず、複数laneはlaneごとのspanに1本ずつ派生する。
support-onlyではmember curveへcontainmentを適用しない。

## wire domain境界

wire coreはroad、rail、building、terrain、cityのdomain型を知らない。
外部systemはworld position、wire template/profile、opaqueなexternal anchor tokenへ解決してからcoreを呼ぶ。
`SavedBackboneGraph`はwire topologyであり、city topologyではない。

## Poleの位置づけ

`Pole`は物理support entityであり、topology rootではない。
support nodeはpole、ownerless point、external anchorを表せる。
spanはport間の生成結果であり、topology authorityではない。

## 境界guard

- public API: `domains/wire/include/city/wire`
- private generation: `domains/wire/src/generation/backbone`
- state ownership: `CoreState`
- read-only query: `CoreView`とconst query
- dependency guard: `tools/arch_lint.py`
- test ownership guard: `tools/test_family_lint.py`
