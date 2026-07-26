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
- surface、curb、sidewalk、marking、terrain mask、mesh、viewer payload

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
  -> DerivedInvariantStage
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
1. TopologyIndexStage
2. CanonicalAlignmentStage
3. NodeConnectionDecisionStage
4. SamplingPlanStage
5. SectionEvaluationStage
6. ConnectionGateStage
7. JunctionGeometryStage
8. MaterializationStage
9. DerivedInvariantStage
```

BuildContextはauthoritative read-only view、前段derived output、diagnostic、test counterを持つ。
stageはauthoritativeを書き換えない。

### NodeConnectionDecisionStage

degree 2以上の接続nodeについて一度だけPassThrough / Corner / Junction / Unsupportedを決める。degree 0/1は
segment端として直接扱い、ApproachKey、decision、gateを要求しない。入力はtopology、
CanonicalAlignment、`NodeConnectionPolicyOverride`、一箇所のpolicy constantsだけとする。出力へcorner radius、
approachごとのsetback / gate station、endpoint section ID、approach order、適用override、diagnostic reasonを含める。
接続角度、最大approach数、endpoint section互換、setback、corner controlはこのstageだけが決める。
approach orderはworld上面のtangent角度で並べ、同角度は`ApproachKey`のID順でtie-breakする。後段はdegree、
角度、格納順から再判定・再sortしない。

自動decisionは保存しない。authoritative policyはAuto / ForcePassThrough / ForceCorner / ForceJunctionだけである。

### SamplingPlanStage

segment start / end、各Bezier span境界、transition start / end、各approachのgate station、manual markingに必要な
stationをsemantic stationとして一度だけ決め、sortと数値幾何epsによる重複除去を行う。gate stationは
`NodeConnectionDecision`から読み、再計算しない。surface、marking、maskはsemantic stationを共有し、
用途別refinementだけを追加できる。

### SectionEvaluationStage

断面、transition mapping、boundary ID、element ID、role、lateral、height、normal、outer boundary、marking anchorの
唯一の決定者とする。全SamplingPlan用途stationの評価を`segment_id + station_m`で一意に供給する。
Continue / TaperIn / TaperOut / EndCapは実際の形状評価で消費し、未実装actionはunsupportedにする。

### Connection and junction stages

ConnectionGateStageは`NodeConnectionDecision.gate_station_m`で`SectionEvaluationTable`を一意検索し、frameと
評価済みboundaryを持つgateを一度だけ生成する。断面を再評価せず、該当評価がない場合はfallbackせず失敗する。
`ConnectionGate.approach`をidentityの正とし、冗長なnode / segment fieldはidentity判定に使わない。

JunctionGeometryStageはdecision、gate、評価済みboundaryだけを読み、degreeや角度を再判定しない。
boundary ID / role / occurrenceの明示対応から`ConnectionGeometry`と`JunctionGeometry`を生成し、corner curve、
surface strip / region、junction perimeter、自動停止線・ゼブラquadをmaterialization前に解決する。
P1 junctionは`sidewalk | curb | asphalt... | curb | sidewalk`の明示構造だけを対応対象とし、その他は
reason付き`kUnsupported`とする。

### MaterializationStage

materializationは評価済みsegment断面、`ConnectionGeometry`、`JunctionGeometry`、専用manual-marking read modelを
頂点、index、normal / UV、material groupへ変換するだけとする。`SavedRoadGraph`やauthoritative headerを読まず、
section評価、connection kind、setback、gate、approach order、corner control、junction perimeter、boundary対応、
停止線・ゼブラ配置を決めない。

## Identity

- 接続と所有はIDで決める。同位置、eps内、配列index、頂点番号、名前からidentityを推測しない。
- segment端のidentityは`ApproachKey(node_id, segment_id, endpoint_role)`の完全一致だけで決める。
  node移動、shape編集、approach角度順、segment格納順で変化しない。
- boundary / element対応はstable ID、role、explicit transition mappingで決める。
- `almost_same`やdistance-epsはBezier連続性、ゼロ長、平行性など数値幾何だけに使える。
- 同位置に複数nodeが存在しても別identityである。

## Persistence target

構造移行後に新しいroad versionを定義する。保存対象はauthoritative stateとnext IDだけとする。
CanonicalAlignment、decision、gate、evaluation、sampling、junction、mesh、mask、auto markingは保存しない。

旧road versionは明示rejectし、endpoint付きalignmentからmigrationしない。loadは新trialへparse、validation、Buildを行い、
成功時だけstate交換する。構文不正、未知field、非有限値、重複ID、欠損参照をrejectする。

## Module boundaries

```text
include/city/road/input_types/
include/city/road/authoritative_types/
include/city/road/derived_types/
src/operations/
src/build/
src/materialization/
src/persistence/
```

物理分割はownerを固定するために行う。materializationはauthoritative headerをincludeせず、adapterはauthoritative型を構築しない。
各build ownerは`src/build/topology.cpp`、`canonical_alignment.cpp`、`node_connection_decision.cpp`、
`sampling_plan.cpp`、`section_evaluation.cpp`、`connection_gate.cpp`、`junction_geometry.cpp`へ分離し、
`build_pipeline.cpp`だけが順に呼ぶ。`RoadState::BuildDerived()`は`BuildRoad(graph_)`の結果交換だけを行う。

## Migration status

| RS | 状態 | 完了条件 |
|---|---|---|
| RS0 | complete | 文書整合、機能凍結、進捗表 |
| RS1 | complete | failure / identity / decision契約testと物理境界lintがfail-firstで存在 |
| RS2 | complete | 全public operationがplan + trial + single Build、operation入れ子なし |
| RS3 | complete | nodeだけがendpoint authority、SegmentShapeとCanonicalAlignmentを分離 |
| RS4 | complete | 必須9 stageとread-only BuildContext。NodeConnectionDecisionがapproach単位のsetbackを所有し、setbackの決定箇所が一つであること。SamplingPlanがgate stationをsemantic stationとして含み、ConnectionGateがSectionEvaluationを再評価しないこと |
| RS5 | complete | materialization純化、boundary ID対応。materializationがcorner controlやjunction形状を決めず、断面roleを探索して意味を推測しないこと。JunctionGeometryTableが描画前の唯一の交差点形状決定表であること |
| RS6 | complete | adapterはRequest変換だけ、fallbackとauthoritative構築なし |
| RS7 | complete | version 5のみ、旧version reject、authoritative roundtrip / corruption test |
| RS8 | complete | test分類、seed fuzz、全road / wire / web / lint回帰 |

station splitはprototype実装であり、architecture migration完了の証拠ではない。
