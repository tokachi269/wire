# Road architecture

この文書は`city::road`のarchitectureを定義する。P0-P2 testの通過だけでは設計準拠を意味しない。
現行実装を根拠に未定義の意味を補完せず、下記のownerと依存方向を維持する。

共通契約は`../architecture.md`、操作×状態は`operation_semantics.md`、prototype要求範囲は`plan.md`を参照する。

## Naming

生成系の動詞は`../architecture_naming.md`に従う。roadの派生生成入口は`generate_road`であり、`derive` / `resolve` / `emit` / `validate`を責務の名前として使う。

road固有の名詞(`RoadNode`、`RoadSegment`、`CrossSection`、`SectionTransition`、`Junction`、
`ConnectionGate`、`ApproachKey`、`Marking`)はwireへ揃えない。

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
- `DerivedSegment`: CanonicalAlignment、semantic / surface station、`SectionEvaluation`
- `ResolvedConnection`: PassThrough / Corner / Junction / Unsupported、approach order、corner policy、適用override、reason
- `ResolvedApproach`: endpoint section ID、auto値、override後のresolved値、gate station、`ConnectionGate`
- `DerivedMarking`: owner、boundary ID、role、style、幅、world points / polygon
- `RenderStyleRef`付きsurface、curb、sidewalk、marking、terrain mask、mesh、viewer payload

StraightとCurvedはinput modeだけである。Straight requestはoperation planで直線shapeへ正規化し、生成後の
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
  -> generate_road(trial authoritative)
  -> invariant
  -> Commit(authoritative + derived + next_id)
```

validation、unsupported、generate、invariantのどこで失敗しても、authoritative serialization bytes、
derived deterministic hash、next ID、inspection / query結果は操作前と完全一致する。

public operationから別public operationを呼ばない。ApplyはplanにないID確保、接続判断、fallbackを行わない。
splitは`target_segment_id + station_m`を使い、target identityやstationを座標から再推測しない。

degree 1終端への同一道路延長は`ExtendSegment`として既存segmentと終端nodeを同じplanでreplaceする。
append / prependするPathは共通normalizerを通してから`SegmentShape`へ変換する。operationはcornerを別実装せず、
一括追加と逐次延長が同じnormalizerを使う。

`EditSegmentPath`相当の外部入力はcoreで次へ分ける。

- `EditSegmentShape`: handle、internal knotだけを変更し、node位置を変更しない
- `MoveNode`: `RoadNode.position`だけを変更し、接続segmentのCanonicalAlignmentを再導出する

## Generate

```text
authoritative graph
  -> edit plan
  -> trial
  -> generate_road
       derive_segments      canonical alignment / semantic stations / section evaluation
       resolve_connections  node decision / auto values / user override / gate / connection geometry
       derive_markings      boundary lines / manual markings / junction markings
       emit_geometry        surface, connection, junction and marking meshes
  -> validate
  -> commit
```

`generate_road(graph)`は純粋関数であり、`RoadState`へ途中結果を書かない。完成した`DerivedRoad`だけを返し、
失敗時は部分的な派生を公開しない。工程を細分するかどうかは実装都合であり、契約はownerと依存方向だけを固定する。

### derive_segments

`CanonicalAlignment`、semantic station、surface station、`SectionEvaluation`を`DerivedSegment`が所有する。
sampling planやsection tableを独立した長寿命read modelとして公開しない。
単独segmentとdegree 1終端は、connection側の解決を要求せずにこの段階だけで面とmarkingに必要な形状を持つ。

### resolve_connections

degree 2以上の接続nodeについて、接続種別、approach順、auto setback、ユーザーoverride、gate frame、
corner / junction geometryを一度に決める。autoとresolvedを別工程・別tableにしない。
`ResolvedApproach`がauto値とresolved値を両方持ち、inspectionはそこから読む。
degree 0/1のnodeにはconnection entity、gate、layoutを作らない。

接続前にpair、mapping、gateを正本へ用意する構造は持たない。必要な接続はgraphから導出する。

### derive_markings

線はowner segmentとboundary IDで識別する。断面遷移では同じboundary IDが続く限り線を継続し、
現れた位置で開始、消えた位置で終了する。隣接車線が退化している区間には線を出さない。
配列indexや位置近接で別boundaryへつなぎ替えない。

交差点内はdefaultでgate終了とし、`JunctionMarkingOverride`が明示したsource / targetだけが接続線を作る。
最も近い線や角度差最小の線を自動選択しない。

### emit_geometry

解決済みgeometryだけを受け取り、頂点・index・normal・UV・mesh groupと有限性検査を行う。
接続種別、setback、override適用、boundary対応、marking範囲、owner、style、線幅を決めない。

## Style ownership

| 層 | 契約 |
|---|---|
| authoritative / request | `SurfaceStyleId` / `MarkingStyleId`を保存・入力する。自由文字列をidentityに使わない |
| built-in catalog | road内の最小built-in styleだけを`common_types.hpp`で定義する |
| derived / emit | `RenderStyleRef(domain, value)`だけを運ぶ。表示名文字列へ戻さない |
| viewer adapter | `RenderStyleRef`をviewer material key文字列へ変換する唯一の境界。未知IDはfallbackしない |

manual markingのstyle IDは保存だけでなくemit inputへ渡し、生成meshの`RenderStyleRef`へ反映する。
BoundaryProfileには個別style fieldを持たせない。curb等の描画styleはBoundaryRoleからsection評価で導出する。

## Identity

- 接続と所有はIDで決める。同位置、eps内、配列index、頂点番号、名前からidentityを推測しない。
- segment端のidentityは`ApproachKey(node_id, segment_id, endpoint_role)`の完全一致だけで決める。
  node移動、shape編集、approach角度順、segment格納順で変化しない。
- boundary / element対応はstable ID、role、explicit transition mappingで決める。
- `almost_same`やdistance-epsはBezier連続性、ゼロ長、平行性など数値幾何だけに使える。
- 同位置に複数nodeが存在しても別identityである。

## Approach override lifecycle

authoritativeに保存するのは`ApproachGeometryOverride`のmanual setback / manual lateral shiftだけである。全fieldがAutoの空overrideは保存しない。
単独segmentとdegree 1 endpointにはconnection/overrideを要求しない。degree 2のCorner/PassThrough、degree 3/4のJunctionでだけ
`ResolvedConnection`を導出する。

- extension: 同一segmentを更新するため`ApproachKey`は維持される。manual fieldは維持し、Auto fieldは最新auto値へ追従する。
- node move / shape edit: `ApproachKey`は維持され、`ResolvedConnection`だけを再導出する。
- segment split: 元segment start側overrideは元segmentに残る。元segment end側overrideは新しい外側segment endへIDでmappingする。内部nodeへ複製しない。
- segment delete / node delete: 該当segment/nodeのoverrideは同じOperationPlanで削除し、orphanを残さない。
- connection kind変更: 同じ`ApproachKey`かつfieldが有効なら維持する。layout対象外になったmanual overrideはgenerateでunsupportedになり、黙って無視しない。

## Persistence target

road persistenceはversion 9とする。保存対象はauthoritative stateとnext IDだけとする。
CanonicalAlignment、connection、gate、section evaluation、junction geometry、mesh、mask、auto markingは保存しない。
resolved connection、derived marking、mesh、maskも保存しない。ApproachGeometryOverrideはmanual fieldがある場合だけ保存する。

形式は一行一fieldのnamed indexed `key=value`で、カンマ位置に意味を持たせない。save field順は固定し、
top-level entityはID順にcanonical serializeする。順序に意味がある`SegmentShape.internal_knots`やPath spanは保存順を維持する。
doubleはload後に同じbinary doubleへ戻る表現で保存する。

version 8以前と未知versionは明示rejectし、migrationしない。loadはparse、field構造検証、型変換、
`ValidateAuthoritativeGraph`、`generate_road`、不変条件検査を通過した場合だけ新stateを返す。duplicate key、missing key、
unknown key、count不一致、enum範囲外、非有限値、重複ID、欠損参照をrejectする。

## Preflight and validation

operation preflightはrequestの全fieldについて、ID存在、有限性、enum範囲、style ID、正であるべき寸法を検査する。
正しい入力だが現在扱えない接続形状・transition付きsplit・prepend station移動は`kUnsupported`に残す。
valid authoritativeから派生が欠ける、emitへ不整合なresolved geometryが渡る等は`kInternal`とする。
`ValidateAuthoritativeGraph`はloadとoperation trial後に共通利用し、保存正本の完全性だけを見る。generateでしか判断できない
幾何対応可否は混ぜない。

## Module boundaries

```text
include/city/road/input_types/
include/city/road/authoritative_types/
include/city/road/derived_types/
src/generation/     generate, segments, connections, markings, emit
src/geometry/       alignment / section / junction の純粋幾何
src/operations/
src/persistence/
```

物理分割はownerを固定するために行い、工程数と一致させない。emitはauthoritative headerをincludeせず、
adapterはauthoritative型を構築しない。geometryはgraphを読むがderivedを書かない。
