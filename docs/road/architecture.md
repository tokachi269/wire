# Road architecture

この文書は`city::road`のarchitectureを定義する。P0-P2 testの通過だけでは設計準拠を意味しない。
現行実装を根拠に未定義の意味を補完せず、下記のownerと依存方向を維持する。

共通契約は`../architecture.md`、操作×状態は`operation_semantics.md`、prototype要求範囲は`plan.md`を参照する。

## Feature freeze

RS0-RS8完了まではroadの新しい見た目、形状、断面、UI、model、高架機能を追加しない。
既存P0-P2幾何はscenario coverageとして維持できるが、旧ownerや複数buildを温存する理由にはしない。

## Existing implementation mapping

| 既存の参照 | roadで採用する契約 | road固有の差異 |
|---|---|---|
| wire saved graph | node / segment identityを持つauthoritative graph | segment curve endpointはnodeが所有する |
| wire operation semantics | operation前に操作×状態を定義する | roadはshape、section、marking requestを持つ |
| wire trial + unified build | roadもplanをtrial authorityへ適用して一度だけBuildする | operation固有preflightとbuild decisionの責務を分ける |
| wire derived curve | nodeとshapeからCanonicalAlignmentを導出する | roadの内部shapeはユーザー編集対象 |
| wire ID connectivity | node / segment IDとstationで接続対象を指定する | 座標近接によるidentity推測は禁止 |
| wire authority/runtime分離 | input / authoritative / derivedを物理分離する | 型はroad固有でありwire型を流用しない |

## Naming

公開surfaceはwireと同じくPascalCaseを使う。`RoadState`、operation request、authoritative entity、
derived read modelはこの規則に従う。

`city::road::build`と`city::road::draw`はwireの`generation::backbone`と同じくlower snake caseを使う。
pipeline内部でroad文脈を繰り返さず、ownerを表す短い中心語を使う。

| 意味 | 名前 |
|---|---|
| build owner | `pipeline` |
| 実行順 | `stage` |
| node incidence | `topology` / `endpoint` |
| build entry | `make_topology`、`make_alignments`、`make_connections`、`make_sampling`、`make_sections`、`make_gates`、`make_junctions` |
| manual/auto解決 | `resolve_layouts`、`resolve_markings` |
| mesh emission | `draw::make_segment`、`draw::make_connection`、`draw::make_junction`、`draw::make_markings` |

`BuildContext`、`BuildXxx`、`XxxStage`のようにpipeline文脈を重ねた名前は使わない。
一方、`station_m`、`ApproachKey`、`SectionEvaluation`などroad固有の意味を持つ語は、
wireの短い名前へ機械的に言い換えない。
## State ownership

### Input

- Straight / Curvedなどのtool mode
- viewport click、drag中handle、preview point
- snap結果の`segment_id + station_m`または`node_id`
- section、transition、marking、policy操作request

tool modeとpreviewは保存しない。adapterはrequestへ型変換するだけで、接続、transition action、ID、fallbackを決めない。

### Authoritative

- `RoadNode`: IDと確定world position。segment endpoint位置の唯一の正本
- `RoadSegment`: ID、endpoint node ID、`SegmentShape`、section timeline参照
- `SegmentShape`: endpointから出るhandle vector、内部knot、内部handle。endpoint座標は持たない
- `CrossSectionTemplate`と`SectionTransition`の意味入力
- manual markingのowner IDとowner-local station / lateral値
- surface / marking style identityは`SurfaceStyleId` / `MarkingStyleId`で保存する
- ユーザーが明示した`NodeConnectionPolicyOverride`
- next ID state

`RoadSegment`へ完全Pathやendpoint座標を保存しない。自動junctionの存在、connection kind、gate、setback、
corner radius、meshは保存しない。

### Derived

- topology indexとnode degree
- `CanonicalAlignment`: `node_a.position + SegmentShape + node_b.position`から作るcubic Bezier span列
- arc-length table、station、tangent、frame、bounds
- `ApproachKey`: `node_id + segment_id + endpoint_role`の完全一致でsegment端を識別するderived identity
- `NodeConnectionDecision`: PassThrough / Corner / Junction / Unsupported、approach order、endpoint section ID、
  approachごとのsetback / gate station、corner policy、適用override、reason
- SamplingPlan、SectionEvaluationTable、ConnectionGateTable、ConnectionGeometryTable、JunctionGeometryTable
- `RenderStyleRef`付きsurface、curb、sidewalk、marking、terrain mask、mesh、viewer payload

StraightとCurvedはinput modeだけである。Straight requestはoperation planで直線shapeへ正規化し、Build後の
CanonicalAlignmentは他のcurveと同じcubic Bezier span列になる。

同じPathを一括で追加した場合とdegree 1終端へ逐次延長した場合は、同じauthoritative `SegmentShape`へ正規化する。
操作確定はnode / segment境界を作る理由にしない。branch、intersection、section境界、明示splitだけが境界を作る。

## Operations

全public operationは次の一経路だけを使う。

```text
Request
  -> Preflight(current authoritative, request)
  -> OperationPlan
  -> Apply(plan, trial authoritative + trial next_id)
  -> Build(trial authoritative) exactly once
  -> invariant
  -> Commit(authoritative + derived + next_id)
```

validation、unsupported、build、invariantのどこで失敗しても、authoritative serialization bytes、
derived deterministic hash、next ID、inspection / query結果は操作前と完全一致する。

public operationから別public operationを呼ばない。ApplyはplanにないID確保、接続判断、fallbackを行わない。
splitは`target_segment_id + station_m`を使い、target identityやstationを座標から再推測しない。

degree 1終端への同一道路延長は`ExtendSegment`として既存segmentと終端nodeを同じplanでreplaceする。
append / prependするPathは共通normalizerを通してから`SegmentShape`へ変換する。operationはcornerを別実装せず、
一括追加と逐次延長が同じnormalizerを使う。

`EditSegmentPath`相当の外部入力はcoreで次へ分ける。

- `EditSegmentShape`: handle、internal knotだけを変更し、node位置を変更しない
- `MoveNode`: `RoadNode.position`だけを変更し、接続segmentのCanonicalAlignmentを再導出する

## Build stages

```text
1. topology
2. alignments
3. connections
4. auto layout
5. resolved layout
6. sampling
7. sections
8. gates
9. junctions
10. marking anchors
11. marking intents
12. marking continuity
13. resolved markings
14. draw
15. invariant
```

`pipeline`はauthoritative read-only view、前段output、test counterを持つ。各stepはauthoritativeを書き換えない。
### connections

degree 2以上の接続nodeについて一度だけPassThrough / Corner / Junction / Unsupportedを決める。degree 0/1は
segment端として直接扱い、ApproachKey、decision、gateを要求しない。入力はtopology、
CanonicalAlignment、`NodeConnectionPolicyOverride`、一箇所のpolicy constantsだけとする。出力へcorner radius、
approachごとのsetback / gate station、endpoint section ID、approach order、適用override、diagnostic reasonを含める。
接続角度、最大approach数、endpoint section互換、setback、corner controlはこのstageだけが決める。
approach orderはworld上面のtangent角度で並べ、同角度は`ApproachKey`のID順でtie-breakする。後段はdegree、
角度、格納順から再判定・再sortしない。

自動decisionは保存しない。authoritative policyはAuto / ForcePassThrough / ForceCorner / ForceJunctionだけである。

### auto layout

`NodeConnectionDecision`が決めたconnection kind、approach order、auto setback、auto gate stationから
`AutoNodeLayout`を作る。approach tangentはnodeからsegment内部へ向かう方向、lateral正方向はそのtangentの左側とする。
degree 0/1にはnode layout entityを作らない。

### resolved layout

`AutoNodeLayout + ApproachGeometryOverride`から`ResolvedNodeLayout`を作る唯一のstageである。manual fieldがあればmanual値、
なければauto値を使う。manual値をautoへ近似削除したり、NaNやmagic numberでAutoを表さない。
setbackはnode基準点からsegment内部方向への非負距離で、start endpointは`station=setback`、end endpointは
`station=segment_length-setback`とする。lateral shiftはresolved lateral方向を正とする。

### sampling

segment start / end、各Bezier span境界、transition start / end、各approachのgate station、manual markingに必要な
stationをsemantic stationとして一度だけ決め、sortと数値幾何epsによる重複除去を行う。gate stationは
`ResolvedNodeLayout`から読み、再計算しない。surface、marking、maskはsemantic stationを共有し、
用途別refinementだけを追加できる。

### sections

断面、transition mapping、boundary ID、element ID、role、lateral、height、normal、outer boundary、marking anchorの
唯一の決定者とする。全SamplingPlan用途stationの評価を`segment_id + station_m`で一意に供給する。
Continue / TaperIn / TaperOut / EndCapは実際の形状評価で消費し、未実装actionはunsupportedにする。
SurfaceBandの`SurfaceStyleId`と、BoundaryRoleからbuilt-in `SurfaceStyleId`への変換もこのstageだけが行う。

### Connection and junction stages

gatesは`ResolvedNodeLayout.gate_station_m`で`SectionEvaluationTable`を一意検索し、resolved frameと
評価済みboundaryを持つgateを一度だけ生成する。断面を再評価せず、overrideを直接読まず、該当評価がない場合はfallbackせず失敗する。
`ConnectionGate.approach`をidentityの正とし、冗長なnode / segment fieldはidentity判定に使わない。

junctionsはdecision、resolved gate、評価済みboundaryだけを読み、degreeや角度を再判定しない。
boundary ID / role / occurrenceの明示対応から`ConnectionGeometry`と`JunctionGeometry`を生成し、corner curve、
surface strip / region、junction perimeterを解決する。停止線・ゼブラなどのmarking quadは生成しない。
P1 junctionはouter edge、2組のcurb boundary、carriageway側boundaryというrole / boundary ID構造だけを対応対象とし、その他は
reason付き`kUnsupported`とする。

### marking anchors

次段のmarking graph用にsemantic anchorだけを導出する。anchor identityはowner ID、`ApproachKey`、boundary ID、semantic roleから
決定し、位置や配列indexから作らない。ApproachGate / ApproachCenter / JunctionCorner anchorは`ResolvedNodeLayout`以降だけを読む。

### MarkingIntent / Continuation / ResolvedMarkingGraph

マーキングの正本は、ユーザーが明示した`AutoMarkingPolicy`、manual marking、suppression / junction overrideだけである。
自動線そのものは保存しない。Buildでは次の順で派生する。

`SectionEvaluation`、`JunctionArea`、manual marking
→ `MarkingIntent`
→ `MarkingContinuation`
→ `ResolvedMarkingGraph`
→ marking mesh

`ResolvedMarkingGraph`はmarking geometryの唯一の決定表で、すべてworld geometry、owner、role、style、continuation actionを持つ。
segment線は`MarkingTrackKey(segment_id, boundary_id, role)`で識別し、boundary配列indexやworld位置からidentityを作らない。
default junction markingはgateで終了する。交差点内部で中央線等を継続する場合は明示overrideだけが接続を作る。

### draw

drawは評価済みsegment断面、`ConnectionGeometry`、`JunctionGeometry`、`ResolvedMarkingGraph`を
頂点、index、normal / UV、`RenderStyleRef`付きmeshへ変換するだけとする。`SavedRoadGraph`やauthoritative headerを読まず、
section評価、connection kind、setback、gate、approach order、corner control、junction perimeter、boundary対応、
停止線・ゼブラ配置、marking継続、style、線幅を決めない。

## Style ownership

| 層 | 契約 |
|---|---|
| authoritative / request | `SurfaceStyleId` / `MarkingStyleId`を保存・入力する。自由文字列をidentityに使わない |
| built-in catalog | road内の最小built-in styleだけを`common_types.hpp`で定義する |
| derived / draw | `RenderStyleRef(domain, value)`だけを運ぶ。表示名文字列へ戻さない |
| viewer adapter | `RenderStyleRef`をviewer material key文字列へ変換する唯一の境界。未知IDはfallbackしない |

manual markingのstyle IDは保存だけでなくdraw inputへ渡し、生成meshの`RenderStyleRef`へ反映する。
BoundaryProfileには個別style fieldを持たせない。curb等の描画styleはBoundaryRoleからsectionsで導出する。

## Identity

- 接続と所有はIDで決める。同位置、eps内、配列index、頂点番号、名前からidentityを推測しない。
- segment端のidentityは`ApproachKey(node_id, segment_id, endpoint_role)`の完全一致だけで決める。
  node移動、shape編集、approach角度順、segment格納順で変化しない。
- boundary / element対応はstable ID、role、explicit transition mappingで決める。
- `almost_same`やdistance-epsはBezier連続性、ゼロ長、平行性など数値幾何だけに使える。
- 同位置に複数nodeが存在しても別identityである。

## Approach override lifecycle

authoritativeに保存するのは`ApproachGeometryOverride`のmanual setback / manual lateral shiftだけである。全fieldがAutoの空overrideは保存しない。
単独segmentとdegree 1 endpointにはlayout/overrideを要求しない。degree 2のCorner/PassThrough、degree 3/4のJunctionで必要な場合だけ
Auto/Resolved layoutを導出する。

- extension: 同一segmentを更新するため`ApproachKey`は維持される。manual fieldは維持し、Auto fieldは最新auto値へ追従する。
- node move / shape edit: `ApproachKey`は維持され、Resolved layoutだけを再導出する。
- segment split: 元segment start側overrideは元segmentに残る。元segment end側overrideは新しい外側segment endへIDでmappingする。内部nodeへ複製しない。
- segment delete / node delete: 該当segment/nodeのoverrideは同じOperationPlanで削除し、orphanを残さない。
- connection kind変更: 同じ`ApproachKey`かつfieldが有効なら維持する。layout対象外になったmanual overrideはBuildでunsupportedになり、黙って無視しない。

## Persistence target

road persistenceはversion 7とする。保存対象はauthoritative stateとnext IDだけとする。
CanonicalAlignment、decision、gate、evaluation、sampling、junction、mesh、mask、auto markingは保存しない。
AutoNodeLayout / ResolvedNodeLayout / MarkingAnchorも保存しない。ApproachGeometryOverrideはmanual fieldがある場合だけ保存する。

形式は一行一fieldのnamed indexed `key=value`で、カンマ位置に意味を持たせない。save field順は固定し、
top-level entityはID順にcanonical serializeする。順序に意味がある`SegmentShape.internal_knots`やPath spanは保存順を維持する。
doubleはload後に同じbinary doubleへ戻る表現で保存する。

version 6以前と未知versionは明示rejectし、migrationしない。loadはparse、field構造検証、型変換、
`ValidateAuthoritativeGraph`、Build、DerivedInvariantを通過した場合だけ新stateを返す。duplicate key、missing key、
unknown key、count不一致、enum範囲外、非有限値、重複ID、欠損参照をrejectする。

## Preflight and validation

operation preflightはrequestの全fieldについて、ID存在、有限性、enum範囲、style ID、正であるべき寸法を検査する。
正しい入力だが現在扱えない接続形状・transition付きsplit・prepend station移動は`kUnsupported`に残す。
valid authoritativeから派生表が欠ける、drawへ不整合なresolved read modelが渡る等は`kInternal`とする。
`ValidateAuthoritativeGraph`はloadとoperation trial後に共通利用し、保存正本の完全性だけを見る。Build stageでしか判断できない
幾何対応可否は混ぜない。

## Module boundaries

```text
include/city/road/input_types/
include/city/road/authoritative_types/
include/city/road/derived_types/
src/operations/
src/build/
src/draw/
src/persistence/
```

物理分割はownerを固定するために行う。drawはauthoritative headerをincludeせず、adapterはauthoritative型を構築しない。
各build ownerは`src/build/topology.cpp`、`alignment.cpp`、`connection.cpp`、
`sampling.cpp`、`section.cpp`、`gate.cpp`、`junction.cpp`へ分離し、
`pipeline.cpp`だけが順に呼ぶ。`RoadState::BuildDerived()`は`build::make(graph_)`の結果交換だけを行う。

## Migration status

| RS | 状態 | 完了条件 |
|---|---|---|
| RS0 | complete | 文書整合、機能凍結、進捗表 |
| RS1 | complete | failure / identity / decision契約testと物理境界lintがfail-firstで存在 |
| RS2 | complete | 全public operationがplan + trial + single Build、operation入れ子なし |
| RS3 | complete | nodeだけがendpoint authority、SegmentShapeとCanonicalAlignmentを分離 |
| RS4 | complete | 必須9 stageとread-only pipeline。NodeConnectionDecisionがapproach単位のsetbackを所有し、setbackの決定箇所が一つであること。SamplingPlanがgate stationをsemantic stationとして含み、ConnectionGateがSectionEvaluationを再評価しないこと |
| RS5 | complete | draw純化、boundary ID対応。drawがcorner controlやjunction形状を決めず、断面roleを探索して意味を推測しないこと。JunctionGeometryTableが描画前の唯一の交差点形状決定表であること |
| RS6 | complete | adapterはRequest変換だけ、fallbackとauthoritative構築なし |
| RS7 | complete | version 6のみ、旧version reject、authoritative roundtrip / corruption test |
| RS8 | complete | test分類、seed fuzz、全road / wire / web / lint回帰 |

station splitはprototype実装であり、architecture migration完了の証拠ではない。
