# Road architecture

この文書は`city::road`の移行先architectureを定義する。現在の`RoadState`、`road.hpp`、`road.cpp`は
この境界へ未移行であり、P0-P2 testの通過は設計準拠を意味しない。現行実装を根拠に未定義の意味を補完しない。

共通契約は`../architecture.md`、操作×状態は`operation_semantics.md`、prototype要求範囲は`plan.md`を参照する。

## Feature freeze

RS0-RS8完了まではroadの新しい見た目、形状、断面、UI、model、高架機能を追加しない。
既存P0-P2幾何はscenario coverageとして維持できるが、旧ownerや複数buildを温存する理由にはしない。

## Existing implementation mapping

| 既存の参照 | roadで採用する契約 | road固有の差異 |
|---|---|---|
| wire saved graph | node / segment identityを持つauthoritative graph | segment curve endpointはnodeが所有する |
| wire operation semantics | operation前に操作×状態を定義する | roadはshape、section、marking requestを持つ |
| wire trial + unified build | roadもplanをtrial authorityへ適用して一度だけBuildする | 現在のpublic operation入れ子と`RebuildDerived()`直結は移行対象 |
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
- `NodeConnectionDecision`: PassThrough / Corner / Junction / Unsupported
- SamplingPlan、SectionEvaluationTable、ConnectionGateTable、JunctionGeometryTable
- surface、curb、sidewalk、marking、terrain mask、mesh、viewer payload

StraightとCurvedはinput modeだけである。Straight requestはoperation planで直線shapeへ正規化し、Build後の
CanonicalAlignmentは他のcurveと同じcubic Bezier span列になる。

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

各nodeについて一度だけPassThrough / Corner / Junction / Unsupportedを決める。入力はtopology、
CanonicalAlignment、`NodeConnectionPolicyOverride`、一箇所のpolicy constantsだけとする。出力へcorner radius、
setback、approach order、junction owner、diagnostic reasonを含める。後段はdegreeや角度から再判定しない。

自動decisionは保存しない。authoritative policyはAuto / ForcePassThrough / ForceCorner / ForceJunctionだけである。

### SamplingPlanStage

endpoint、gate、transition境界、section keyframe、internal knotなどのsemantic stationを一度だけ決める。
surface、marking、maskはsemantic stationを共有し、用途別refinementだけを追加できる。

### SectionEvaluationStage

断面、transition mapping、boundary ID、element ID、role、lateral、height、normal、outer boundary、marking anchorの
唯一の決定者とする。Continue / TaperIn / TaperOut / EndCapは実際の形状評価で消費し、未実装actionはunsupportedにする。

### Connection and junction stages

ConnectionGateStageはSectionEvaluationTableとNodeConnectionDecisionからgateを一度だけ生成する。
JunctionGeometryStageはdecision、gate、policy overrideだけを読み、degreeや角度を再判定しない。

### MaterializationStage

materializationはCanonicalAlignmentTable、SamplingPlan、SectionEvaluationTable、NodeConnectionDecisionTable、
ConnectionGateTable、JunctionGeometryTableと専用manual-marking read modelだけを読む。`SavedRoadGraph`を読まず、
section評価、connection kind、setback、gate、junction存在、boundary対応を決めない。

## Identity

- 接続と所有はIDで決める。同位置、eps内、配列index、頂点番号、名前からidentityを推測しない。
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

## Migration status

| RS | 状態 | 完了条件 |
|---|---|---|
| RS0 | complete | 文書整合、機能凍結、進捗表 |
| RS1 | complete | failure / identity / decision契約testと物理境界lintがfail-firstで存在 |
| RS2 | complete | 全public operationがplan + trial + single Build、operation入れ子なし |
| RS3 | complete | nodeだけがendpoint authority、SegmentShapeとCanonicalAlignmentを分離 |
| RS4 | complete | 必須9 stageとread-only BuildContext |
| RS5 | complete | materialization純化、boundary ID対応 |
| RS6 | complete | adapterはRequest変換だけ、fallbackとauthoritative構築なし |
| RS7 | complete | version 5のみ、旧version reject、authoritative roundtrip / corruption test |
| RS8 | complete | test分類、seed fuzz、全road / wire / web / lint回帰 |

station splitはprototype実装であり、architecture migration完了の証拠ではない。
