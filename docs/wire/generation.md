# Wire generation

この文書はWire backbone generationのstage、placement、identity mapping、build/rebuild、validation boundaryを所有する。操作×状態と操作固有lifecycleは[`backbone_operation_semantics.md`](backbone_operation_semantics.md)、authoritative/persisted stateは[`state_and_persistence.md`](state_and_persistence.md)を正本とする。

## Pipeline

`GenerateFromBackboneSpec()`は`domains/wire/src/generation/backbone`のpipelineだけを呼ぶ。未対応入力はv1へfallbackせず、mutation前に`unsupported`を返す。

処理順は次の通り。

1. inputの`prepare`と`check`
2. graphから操作中endpointの候補関係と暫定rowを確定
3. duplicate edge bundle/span bindingをpreflight
4. intentとsupport groupを確定
5. pole、bundle、port、spanを生成
6. `SavedBackboneGraph`とbindingを保存
7. rules、layout、geom、drawを保存

context linkは判断入力であり、生成・保存対象ではない。T/cross/branchのkind enumは作らず、continuityと派生rowの組合せで表す。

backbone pipelineの実行入口は`build(build_input)`だけである。通常生成とsaved-scope再生成はpipelineの別実装ではなく、`build_input`の違いだけである。regenerateは`CoreState`のpost-edit operation側の概念であり、pipeline stageの概念ではない。

`build_input`はgraph、active bundle scope、local path mappingを運ぶ。通常生成は`prepare`済みgraphから`build_input_from_spec`で入力を作り、saved-scope再生成は保存済みbackbone identityから復元したgraphを`build_input_from_saved_scope`で入力にする。bundle templateやpole typeの差分はpipeline inputにoverrideとして持たせず、trial/proposal側のstateに反映してから同じ`build`を通す。

`prepare()`はまだpipeline memberにgraphを構築する既存構造を残している。次段階で必要なら`prepare()`自体を`build_input`生成器へ寄せる。`build`はpairs -> intent -> groups -> topo/emit -> save_graph -> rules -> layout -> geom -> drawの共通stage列を通す。adapterはpair / emit / rules / layout / geom / drawの判断を持たない。operation固有の差分はpost-edit APIと`regenerate_backbone_edge_bundles`側に留め、pipelineへ別stageや専用fallbackとして持ち込まない。

## Placement

pole local frameの配置原点はpoleのtiltを含む中心軸とする。mesh表面やmesh下端を原点にしない。`PortPlacementBand.lateral_center_m`は中心軸から測ったbandの既定位置であり、`BackboneSpec.constraints.lateral_offset_m = 0`は選択されたband位置から追加移動しないことを意味する。

`BackboneSpec::bundles`の各要素は1つのBundle placementである。同じ`BundleTemplateId`を複数要素から参照してよく、各要素は独立した`Bundle` identityを生成する。placementのexplicit height/lateralはpole local中心軸を原点とする絶対位置であり、spreadは`Bundle.phase_spacing_m`としてBundleが所有する。Pole band、category、個別Spanをplacement identityとして代用しない。主線、endpoint fixture、support path、helixは同じBundle placementから解決されたPortを読む。

explicit placementでもpole band identityはfixture・roleの解決に使うが、そのheight/lateral centerを配置値へ加算しない。legacy/API入力でexplicit指定がない場合だけband既定位置を使用する。

同一category/layerにlane数ぶんの異なるlateral位置を持つbandがある場合、laneはpriorityで採用したbandをlateral順に1つずつ使用する。既定HV 3相は左・中央・右bandを各laneが使用し、3相全体を片側のpole表面へ寄せない。異なるband位置がlane数に足りない場合だけ、priority最上位の1 bandをrow中心としてlane spacingを展開する。保存済みport bindingはlaneごとのplacement band identityを保持する。

`SavedBackbonePortBinding::lane_index`はSpan側のlogical conductor identity、`port_id`はそのendpointで選ばれたphysical trow laneのPort identityである。`placement_band_id`は`port_id`の実physical placementに属し、mirror時にlogical laneとphysical laneが異なる場合もlogical lane番号から再選択しない。

pole表面へ直接取り付けるportや部品は、中心軸原点を変えず、その高さのsection半径とstandoff / clearanceから表面位置を導出する。表面位置を既定offsetへ混ぜず、laneごとに後処理してbundle重心をずらしてはならない。

通常cornerでは前後linkの単位接線和から二等分方向を作り、その直交方向をrow axisにする。径間長の差でrow axisを回さず、各incident spanのlane順が反転しない範囲に保つ。鋭角cornerはcontinuityを維持したまま、各incident edgeに直交する2つのdead-end rowとjumperへ派生する。jumperのpeer portはcontinuityから導出し、layout ruleやPortへ接続正本として保存しない。jumperはlogical spanや`SavedBackboneGraph` edgeを増やさない。この判定はbundle templateを読まず、connectivity段の局所幾何だけで決める。pole facingはこのcorner decisionの`node_forward`を消費し、角度や二等分線を再計算しない。旧angle correctionは緩角向けの補助に限定し、倍率上限は`kMaxCornerSideScale` (1.7)とする。

### Row conflict and endpoint offset

通常のroute bendはlowering対象ではない。同一nodeのrow conflictでは、物理fixture rowごとに1 support levelを割り当てる。level 0は基準位置、level 1以降は`abs(BundleTemplate::branch_endpoint_offset_m) * level`だけ順に下げる。1 levelへ複数の物理fixture rowを載せない。通常pairは共有rowなので1つのlevelだけを使うが、鋭角pairは2つのfixture rowとjumperへ派生するため、2つの異なる空きlevelを使う。jumperはcontinuity表現であり、2つのrowを同じplacement levelへまとめる根拠にしない。

この多段配置は`BundleTemplate::enable_branch_down_offset`が有効なbundle placementだけに適用し、無効なplacementはrow数に関係なくlevel 0を維持する。`SavedBackbonePortBinding`はrowごとの`support_level`と`support_group_id`を保存し、save/loadやincremental generationで同じ配置判断を再利用する。

Port生成はpole bandまたはexplicit placementの論理anchorだけを使い、row数や保存済みPort高さから別のZ slotを決めない。support level/groupを確定するrow placementが唯一の段差decisionであり、Span layoutはそのdecisionから解決済みのbranch endpoint offsetを読む。段変更後の最終wire socketを`support_world`と`endpoint_world`の両方に使い、port位置は論理anchorとして保持する。LV/HVなどのcategory名自体はlowering条件にしない。

### Derived non-HV support

非HV endpointの支持表現はauthoritative Bundle placementからgeneration時に導出する。pole surface近傍はDirect attachment、明確に離れたplacementはSupported rowとし、新しいSupportRow entityや保存fieldは作らない。

同じpole、同じrow方向、同じside、互換fixture semanticsで高さが近いBundleはdeterministicに1 rowへまとめる。LVはLV内、CommunicationとOpticalは相互にだけ共有候補とする。rowのreachは最外memberとmarginから求め、短すぎる支持物は生成しない。supportの有無、grouping、高さ、reachはCore derived placementが所有し、viewerのasset adapterは形状だけを提供する。support groupingはrow fixture materializationより先に確定し、Supported rowはshared supportを1つだけ生成してper-Bundle row fixtureを生成しない。endpoint fixtureはlogical cableごとに必要な数だけ生成し、visual member数では増やさない。

## Identity mapping

`preserve_conductor_identity=false`かつ`order_decision_policy=kPermutableHomogeneous`のmulti-lane rowは、XY上で横一列に並ぶ配置だけをsupportedとする。両端の`last lane - first lane`のXY方向を比較し、dotが負の場合だけ片端のlane対応を全体反転する。これは1 bitのmirror決定であり、任意permutationを探索しない。first/lastのXY方向が得られない縦積み、または両row方向が直交して1 bitで決められない配置はfallbackせずunsupportedとする。

promotionで既存Portのrow frameが変わる場合は、全Port frameの確定後に`emit_ports`が`is_new=false`のcontext edgeだけを対象とし、既存edgeの反対側rowに対するmirrorを最終位置から1回だけ決める。Port entityは維持し、対象edge bundleのPortBindingとSpan endpointをidentityまたは全体reverseのどちらかで一括更新する。context linkはこの判断入力にだけ使い、`tspan`化または`save_graph()`の保存対象にしない。`emit_spans`は`is_new=true`のedgeだけを対象とし、新規Span endpointを最終Port位置から同じ1 bit規則で決める。同じedgeを両経路で処理してはいけない。

異なるedge bundleを`SavedBackboneRowContinuity`で接続する場合も、permutable laneの対応は両rowの最終Port列`last lane - first lane`のXY方向から1 bitだけ決める。方向のdotが負ならB側laneを全体反転し、非負なら維持する。peer edgeの選択は既存のcontinuity候補規則が所有し、この幾何判定はpeer edgeを選ばない。鋭角判定やJumper materializationはlane対応を再決定せず、保存済みcontinuityの異なるA/B lane indexを消費する。横方向が得られない、または両row方向が直交して1 bitで決められない配置はfallbackせずunsupportedとする。

Span endpoint対応とcross-edge continuity対応は異なる正本関係である。前者は最終`trow`、後者はSpan endpoint反映後の`SavedBackbonePortBinding`を各1回だけ読む。生成中の物理slotである`trow` laneを保存済みbinding laneの代用にせず、両経路は上記の1 bitベクトル判定だけを共有する。

`PathDirectionMode`はユーザーが引いた向きの意味を持ち、signed lateral offsetやsource/branchの進行方向へ適用する。permutable laneのnon-crossingとcanonical topology identityは`PathDirectionMode`へ依存させない。同じ物理pathをReverseで生成した場合、signed lateral offsetの物理側は反転する。

midair branchのLeadは`SourceEdgeProjectionRef.from_node_id`が示すsource edge方向へsource curve tangentを向けてからbranch boundaryへ接続する。branch endpointへの位置ベクトルでsource tangentの符号を選ばない。attachment pointはsource curve projectionのまま維持し、branch側は確定後のEdgeBody boundary tangentを使う。

接続相手は`SavedBackboneRowContinuity`だけが保持し、row表現は共通のendpoint row導出が現在幾何から決める。生成中routeの隣接も同じcontinuityへ記録する。route/orderは永続化しない導出補助であり、接続相手やpair/open表現の判定入力にしない。

saved load / regenerateのroute復元は、最初に`edge_bundle_id`単位のrow continuity componentを確定し、そのcomponent内だけをphysical edge routeへ投影する。同じphysical edgeが複数の独立bundle componentへ属する場合、physical edgeは各componentの派生routeへそれぞれ現れてよい。scope/component確定前に`edge_id`だけのglobal adjacencyへcollapseせず、bundle template IDや同じtemplateを使う別Bundle placementをcomponent identityの代用にしない。

ownerlessなmidair branchのsource identityはsaved edge、edge bundle、lane、port bindingから特定する。world接続点はそのsource identityからcurrent curve projectionとして導出する派生値である。別bundleのportやviewer hit worldから接続点を推測せず、source identityを解決できなければmutation前に`unsupported`とする。`allow_midair_branch=false`のtemplateはmidair branchの生成対象にしない。

pipeline前半(pairs/intent/groups/topo/emit/save_graph)はsource cableのcurve座標を要求しない。source-edge由来のownerless portをmaterializeする場合も、その`world_position`はpreview/cacheであり正本ではない。layout/derive段でsource identityをcurrent curve projectionへ解決し、branch endpointを現在のsource curveへ追従させる。

既存source edgeからのbranchは、事前curve座標をpipeline前半の入力にしない。source edge自体とそこから伸びるbranchを同じ`BackboneSpec`で同時に表す入力形式は現APIにはまだ無いので、二度pipeline実行で補わない。viewerは後追いでsnap targetを明示し、source-edge snapではhit worldではなくsource edge/t/bundle/laneを渡す。

## Build-Rebuild

更新分類は次の4種類だけとし、操作名ごとのdirty enumは追加しない。

| `UpdateKind` | 変更範囲 | 再導出 |
|---|---|---|
| `kRegenerate` | topology / identity / connectivity | generation。安全にできなければ`unsupported` |
| `kReposition` | support/endpoint位置 | layout -> geom -> draw |
| `kReshape` | curve/bounds/shape | geom -> draw |
| `kRedraw` | visual/render | drawのみ |

post-edit APIは派生出力を更新して成功するか、mutation前に拒否する。staleなlayout/geom/drawを残したまま成功してはいけない。`kRegenerate`はtopology / identity / connectivity級差分を分類し、通常更新経路で拒否する境界である。`execute_update_plan`は`kRegenerate`を恒久的に拒否する。regenerateは各post-edit APIが編集差分を添えて統一入口を直接呼ぶ。`UpdatePlan`は差分入力を運ばないため、plan経由のregenerate実行は設計として採用しない。

post-edit regenerateと通常生成のcommitは全stageが成功したときだけ本stateへ反映する。どのstageで失敗しても、本stateは変更前と同一でなければならない。trial (state copy)はこのfailure保証の現行実装であり、MutationPlan / journal / copy-on-write等の代替transaction方式へ置換できた場合だけ削除できる。preflightを増やしたことを理由に本state直接変更へ戻すことは禁止する。

`GenerateFromBackboneSpec`はstate copyのisolated trialで`prepare`、`check`、`build`を実行し、成功時だけstorageをmove commitする。`GenerationTiming.state_copy_ms`はcopyコストを記録し、66 pole級populated stateに対するcopy gateはgeneration totalの20%以内とする。

統一regenerateは編集差分から影響scopeを解決し、保存済み入力からscopeのpipeline graphを組み直し、既存pipelineを部分再実行してbindingをreconcileする。既存bindingは再利用し、増えたものは生成し、消えたものは退役する。差分別のmigration operationは作らず、対応範囲はscenario単位で拡張する。操作固有のsupported/unsupportedとidentity保持は[`backbone_operation_semantics.md`](backbone_operation_semantics.md)を正本とする。

`DeriveGeneratedSpanOutputs()`は保存済みrules / layout source / `SavedBackboneGraph` bindingからlayout、geom、drawを再導出する入口である。topology、continuity、port identityを再判断してはいけない。row/fixture/patch/jumperは保存表現を読まず、continuityと現在幾何から再導出する。

## Validation boundary

外部入力の数値検証はpipeline preflight先頭の`validate_backbone_spec_external_input`が所有する。対象は`BackboneSpec.path.polyline`、`NodeSpec.tangent_hint`、`interval_m`、`constraints.avoid_points`、`constraints.avoid_radius_m`、`constraints.lateral_offset_m`、`pole_placement.max_tilt_deg`、および各`BackboneBundleSpec`の`height_m` / `lateral_m` / `spacing_m`である。NaN / inf、負のinterval、負のavoid radius、負のmax tilt、負のspacingはmutation前に`invalid input`として拒否する。

pipelineのpreflightは、入力・identity・binding・構造上その時点で判定できる失敗だけを早期検出する。source edgeのcurrent curve projectionや`EvaluatePosition(source_t)`のように後半の派生geometryが必要な失敗はpreflightへ移さない。post-edit regenerateのatomicityはpreflightの完全性ではなく、全stage成功後にだけ本stateへ反映するtrial/proposal境界で守る。trialを削除できるのはMutationPlan、copy-on-write state、rollback journal、immutable proposal等のtransaction方式に置換できた場合だけである。通常生成もisolated trialを通す。

`ValidateFast()`、`Validate()`、inspectionはstateを観測して問題を報告する。不足情報の補完、topologyの推測、state mutationは行わない。

## Dependency boundary

private generationのownerは`domains/wire/src/generation/backbone`であり、source layerと禁止dependencyは[`tools/arch_manifest.json`](../../tools/arch_manifest.json)および既存architecture lintが検査する。CMake target graphや別のlayer taxonomyをこの文書のauthorityにしない。
