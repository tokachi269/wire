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
- snap結果の`segment_id + segment_distance_m`または`node_id`
- section、transition、marking、policy操作request

tool modeとpreviewは保存しない。adapterはrequestへ型変換するだけで、接続、transition action、ID、fallbackを決めない。

### Authoritative

- `RoadNode`: IDと確定world position。segment endpoint位置の唯一の正本
- `RoadSegment`: 局所編集単位のID、endpoint node ID、局所`SegmentShape`、section timeline参照
- `RoadCorridor`: 共通道路定義を参照する方向付き`RoadSegment`列。分岐を含まない
- `SegmentShape`: endpointから出るhandle vector、内部knot、内部handle。endpoint座標は持たない
- `SegmentShape.intent`: tool由来の編集意図。Straightはnode移動時のlinear handle再導出とinspectionだけに使い、generation / emitの別経路を作らない
- `CrossSectionTemplate`と`SectionTransition`の意味入力
- `LaneBand`: template内で安定した`LaneId`、所属surface、横方向範囲、segment正本方向に対する進行方向
- `LaneConnection`: `LaneEndpointKey(segment_id, lane_id, endpoint_role)`間の確定した車線接続
- `BoundaryContinuation`: `BoundaryEndpointKey(segment_id, boundary_id, endpoint_role)`間の確定した境界継続
- manual markingのowner IDとowner-local distance / lateral値
- surface / marking style identityは`SurfaceStyleId` / `MarkingStyleId`で保存する
- ユーザーが明示した`NodeConnectionPolicyOverride`
- next ID state

`RoadSegment`へendpoint座標を保存しない。一回で確定したPathは一つのsegmentとして複数spanを持てる。
確定済みsegmentの延長は新segmentを作り、既存shapeへspanを追記しない。degree 2 nodeは明示的な確定境界として
正当であり、同一道路定義でも自動統合しない。自動junctionの存在、connection kind、gate、setback、
corner radius、meshは保存しない。

### Derived

- topology indexとnode degree
- `CanonicalAlignment`: `node_a.position + SegmentShape + node_b.position`から作るcubic Bezier span列
- arc-length table、distance、tangent、frame、bounds
- `ApproachKey`: `node_id + segment_id + endpoint_role`の完全一致でsegment端を識別するderived identity
- `DerivedSegment`: CanonicalAlignment、semantic / surface distance、`SectionEvaluation`
- `ResolvedConnection`: PassThrough / Corner / Junction / Unsupported、approach order、corner policy、適用override、reason
- `ResolvedApproach`: endpoint section ID、auto値、override後のresolved値、gate distance、`ConnectionGate`
- `DerivedMarking`: owner、boundary ID、role、style、幅、world points / polygon
- `RenderStyleRef`付きsurface、curb、sidewalk、marking、terrain mask、mesh、viewer payload

StraightとCurvedは同じ`SegmentShape`とcubic Bezier APIを使う。Straight requestはoperation planで、
`P1 = P0 + (P3 - P0) / 3`、`P2 = P0 + 2 * (P3 - P0) / 3`のlinear cubicへ正規化する。
`SegmentShape.intent`はStraightとして保存するが、sampling、connection、emitはBezierを共通に読む。
後から直線かどうかを制御点の近接比較で判定して編集意味を決めない。

角度を持つStraight segment同士の丸みは`RoadSegment.shape`へ混入しない。degree 2の屈曲nodeでは、
各segmentをgate distanceまで直線として使い、その間のcorner curveを`ResolvedConnection.connection_geometry`
として導出する。corner生成は隣接segmentの制御点を書き換えない。

一回で確定したPathは、複数spanを内部に持つ一つのRoadSegmentへ正規化する。確定済みsegmentを
corridor終端から延長した場合は、既存segmentを書き換えず、新しいRoadSegmentをcorridor末尾へ追加する。

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
splitは`target_segment_id + segment_distance_m`を使い、target identityやdistanceを座標から再推測しない。

degree 1のcorridor終端への同一道路延長は`ExtendCorridorFromEnd`として新しいnodeとsegmentを同じplanで追加する。
既存segment、既存node位置、既存handleをreplaceしない。corridor先頭への延長は、既存corridor distance参照の
移行規則がないためunsupportedとする。segment間の滑らかさは隣接する二つのsegmentだけを入力とする
derived connection geometryが担当する。

`EditSegmentPath`相当の外部入力はcoreで次へ分ける。

- `EditSegmentShape`: handle、internal knotだけを変更し、node位置を変更しない
- `MoveNode`: `RoadNode.position`を変更する。Straight intentを持つincident segmentは新しい両端node位置からlinear handleを再導出し、Curve intentのhandleはそのsegment-local shapeとして維持する

## Generate

```text
authoritative graph
  -> edit plan
  -> trial
  -> generate_road
       derive_segments      canonical alignment / semantic distances / section evaluation
       resolve_connections  node decision / auto values / user override / gate / connection geometry
       derive_markings      boundary lines / manual markings / junction markings
       emit_geometry        surface, connection, junction and marking meshes
  -> validate
  -> commit
```

`generate_road(graph)`は純粋関数であり、`RoadState`へ途中結果を書かない。完成した`DerivedRoad`だけを返し、
失敗時は部分的な派生を公開しない。工程を細分するかどうかは実装都合であり、契約はownerと依存方向だけを固定する。

工程数は削減目標ではない。各工程は入力・出力・consumer・失敗分類を説明できることだけを求める。
consumerのない工程と、削除した機能のためだけの分岐は置かない。

| 工程 | 入力 | 出力 | consumer |
|---|---|---|---|
| `derive_node_incidence` | graph nodes / segments | node別のincident endpoint | `resolve_connections` |
| `derive_segment_shapes` | node位置 + `SegmentShape` | `CanonicalAlignment`、長さ | `derive_segment_sections`、`resolve_connections`、viewer |
| `resolve_connections` | incidence、alignment、policy override、approach override | `ResolvedConnection`(種別・approach順・auto値・resolved値・gate) | `derive_segment_sections`、`resolve_connection_geometry`、inspection |
| `derive_segment_sections` | alignment、template、transition、gate distance | semantic / surface distance、`SectionEvaluation` | `resolve_connection_geometry`、`derive_markings`、`emit_geometry` |
| `derive_topology_paths` | lane connection、boundary continuation | junction内のlane / boundary経路 | `derive_segment_lane_paths`、`derive_markings` |
| `derive_segment_lane_paths` | section評価、topology経路 | segment内のlane経路 | scene payload、`derive_markings` |
| `resolve_connection_geometry` | `ResolvedConnection`、gate断面 | corner / junction surface | `emit_geometry` |
| `derive_markings` | section boundary、marking policy、junction override | `DerivedMarking` | `emit_geometry` |
| `emit_geometry` | 解決済みgeometryのみ | mesh、terrain mask | viewer |

失敗分類は全工程共通で、入力不正は`kInvalidInput`、現在の対応範囲外は`kNotImplemented`、
正しい正本から派生が欠ける場合は`kInternalError`とする。保留機能のデータを持たない通常道路が、
その機能の処理を理由に失敗してはならない。

### derive_segments

`CanonicalAlignment`、semantic distance、surface distance、`SectionEvaluation`を`DerivedSegment`が所有する。
sampling planやsection tableを独立した長寿命read modelとして公開しない。
単独segmentとdegree 1終端は、connection側の解決を要求せずにこの段階だけで面とmarkingに必要な形状を持つ。

### resolve_connections

degree 2以上の接続nodeについて、接続種別、approach順、auto setback、ユーザーoverride、gate frame、
corner / junction geometryを一度に決める。autoとresolvedを別工程・別tableにしない。
`ResolvedApproach`がauto値とresolved値を両方持ち、inspectionはそこから読む。
degree 0/1のnodeにはconnection entity、gate、layoutを作らない。

接続前にpair、mapping、gateを正本へ用意する構造は持たない。必要な接続はgraphから導出する。
一般的な斜交接続を固定の最小・最大角度だけで拒否しない。junctionのauto setbackは各approachの
実断面外半幅、隣接approachとの交差角、共通corner radiusの接線長から個別に導出し、必要距離をsegment内へ確保できない場合や
有限な境界geometryを生成できない場合だけを`unsupported`とする。
Junction geometryはtemplate IDや表示styleで断面を分類しない。gateごとに左右のroad outer、curb、
shoulder、carriageway edgeをboundary roleとstrip adjacencyから解決し、存在する帯だけを生成する。
片側だけcurbがある断面も同じ解決器を通し、候補が複数で意味が一意にならない側だけをunsupportedとする。
carriagewayとshoulderは同じAsphalt surface regionとしてcurb innerまで一度だけemitし、
carriageway edgeはsurface分割ではなくshoulderを示すsemantic markingとして保持する。

### derive_markings

線はowner segmentとboundary IDで識別する。断面遷移では同じboundary IDが続く限り線を継続し、
現れた位置で開始、消えた位置で終了する。隣接車線が退化している区間には線を出さない。
配列indexや位置近接で別boundaryへつなぎ替えない。

CenterLineとLaneSeparatorは交差点内でdefault gate終了とし、`JunctionMarkingOverride`が明示したsource / targetだけが接続線を作る。
CarriagewayEdgeは隣接sideのrole/styleが一致する場合、解決済みjunction perimeter curveへ自動接続する。
片側だけに存在する線はgate終了とし、明示`BoundaryContinuation`と同じendpoint pairへ自動線を重複生成しない。
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
- lane endpointは`LaneEndpointKey`、boundary endpointは`BoundaryEndpointKey`の完全一致だけで識別する。
  lane/boundaryの横方向順序、world位置、距離、sample indexから接続先を推測しない。
- `almost_same`やdistance-epsはBezier連続性、ゼロ長、平行性など数値幾何だけに使える。
- 同位置に複数nodeが存在しても別identityである。

## Lane topology ownership

`LaneConnection`と`BoundaryContinuation`はauthoritativeである。ユーザー操作またはoperationで確定した
source/target endpointを保存し、generation時にgeometryの近さから作り直さない。`LaneConnectionKind`と`BoundaryContinuationKind`は
生成とinspectionの意味を補助するが、identityの代わりにはしない。高速道路、出口、合流専用の保存型は作らず、
一般道の車線追加、分岐、合流、交差点movementも同じlane/boundary接続で表す。

車線中心と道路境界は別の正本である。lane接続からshoulderや道路外端の継続を暗黙導出せず、必要な境界は
`BoundaryContinuation`で明示する。lane/boundaryのworld pathはこれらの正本から派生し、保存しない。

外側lane追加は、追加側と反対にある直近の断面境界を`SectionTransition`の固定基準とする。
curbやmedian edgeのように幅と高さを持つ境界も、その境界の断面順で左側のsampleを固定する。
反対側に境界がない単一strip断面では断面外端を固定する。既存laneの内側境界を
同じIDの断面sampleから一度だけ解決し、その外側にだけ新laneを展開する。全断面の中心合わせやworld位置の
近接再bindは行わない。新laneの幅はtransition中に0から指定幅へ単調に増加し、既存shoulder幅は維持したまま
外側へ移動する。
複数laneが同じcarriageway stripを共有していても、lane範囲を含む断面順から外側laneを解決する。laneとstripの
一対一対応やstrip全幅占有をADD LANEの前提にしない。固定boundaryは選択側の内側方向にある最初の安定boundaryを使う。

ADD LANEの変化開始と3車線完成位置は、一つのRoadSegment内の`SegmentPosition.t`として保存する。開始・完成位置を作るためにsegmentを
分割せず、物理距離やcorridor-globalなLaneAddition entityを保存しない。boundary anchorで決まった横方向原点は明示された維持終点までの後続segmentへ派生伝播し、
追加前から存在するlaneを再中心化しない。通常幅以降のsegmentはtransitionを所有しないため、後続segmentは通常の完成断面として扱う。

接続可否はtemplate IDではなくendpointの実断面で判定する。同じ物理・意味定義なら別template IDでも接続し、
degree 2でlane layoutが異なる場合はnode手前のsection transitionを要求する。junctionでは各approachの断面を独立して
解決し、未指定laneはgateで終了する。1件のLaneConnectionを全断面の解決済み判定には使わない。
道路中心や最寄りlaneから接続先を補わない。
新segmentをtargetにする分岐と、新segmentをsourceにする合流は同じoperation契約を使う。
`resolve_connections`はstraight ContinuationまたはMergeのlane center対応からtarget approachの共通lateral shiftを一度だけ導出し、
角度を持つbranch sourceはmainline shiftの決定入力から除外する。
`DerivedLanePath`と`DerivedBoundaryPath`は保存せず、authoritative connection IDから毎回生成する。Splitの両側境界から
`DerivedSeparationArea`を作るが、lane connectionから境界接続を逆算しない。

laneとboundaryの多重度は同じ意味に揃える。Continuationは一対一、Splitは同一sourceから複数target、Mergeは複数sourceから
同一targetを許可する。同一source-target pairは常に重複としてrejectする。
対向道路でも接続単位は`LaneEndpointKey`であり、道路全体の方向分類からbranch対象を広げない。選択方向と反対のlaneおよび
median boundaryは、明示された接続以外では既存mainline上に維持する。

交差点内の進行は`LaneConnectionKind::kJunctionMovement`として同じauthoritative topologyへ保存するが、geometryは
`resolve_junction_movement_path`がjunction gate間だけに生成する。merge/splitの長い接続区間resolverとは分け、
共通のG1 cubic primitiveだけを共有する。source/targetは常に`LaneEndpointKey`で明示し、turn roleや角度から推測しない。

segment間の接続白線は`DerivedBoundaryPath`を唯一のgeometry入力とする。marking生成はboundary endpointやlane centerから
別Bezierを作らない。Split区間はsource boundaryのrole/style、Merge区間はtarget boundaryのrole/styleを使い、
Continuationでrole/styleが変わる場合はunsupportedとする。接続線はjunction ownershipでgate間だけを所有する。

## Approach override lifecycle

authoritativeに保存するのは`ApproachGeometryOverride`のmanual setback / manual lateral shiftだけである。全fieldがAutoの空overrideは保存しない。
単独segmentとdegree 1 endpointにはconnection/overrideを要求しない。degree 2のCorner/PassThrough、degree 3/4のJunctionでだけ
`ResolvedConnection`を導出する。

- extension: 既存segmentの`ApproachKey`は不変で、新segment endpointに新しい`ApproachKey`を作る。
  既存manual fieldは維持し、Auto fieldは最新auto値へ追従する。
- node move / shape edit: `ApproachKey`は維持され、`ResolvedConnection`だけを再導出する。
- segment split: 元segment start側overrideは元segmentに残る。元segment end側overrideは新しい外側segment endへIDでmappingする。内部nodeへ複製しない。
- segment delete / node delete: 該当segment/nodeのoverrideは同じOperationPlanで削除し、orphanを残さない。
- connection kind変更: 同じ`ApproachKey`かつfieldが有効なら維持する。layout対象外になったmanual overrideはgenerateでunsupportedになり、黙って無視しない。

## Persistence target

road persistence version 11は局所segment/corridorに加えてlane/boundary topologyを保存する。保存対象はauthoritative stateとnext IDだけとする。
CanonicalAlignment、connection、gate、section evaluation、junction geometry、mesh、mask、auto markingは保存しない。
resolved connection、derived marking、mesh、maskも保存しない。ApproachGeometryOverrideはmanual fieldがある場合だけ保存する。
`LaneConnection`と`BoundaryContinuation`はID順に保存し、lane/boundary path geometryは保存しない。

Web/WASM境界はCoreが返す軽量なlane world pathをhoverと選択表示にだけ使う。編集requestは選択済みの
`LaneEndpointKey` / `BoundaryEndpointKey`を明示して渡す。AddLaneはeditorで選択した道路位置を確定前にsegment-local `t`へ変換し、
`SegmentPosition`と明示した維持終点nodeを渡す。
Webはlane接続curve、境界対応、接続相手を再計算しない。標準道路描画のpointer moveは入力Bezierと断面幅からローカルguideだけを表示し、Core operationを実行しない。ClickまたはEnterの明示commitで初めてCore generationとvalidationを行う。
control-point編集やlane編集の明示previewはtrial stateで対応するCore operationを実行できる。preview結果は確定結果と同じresolved mesh経路を使い、preview前後でauthoritative stateを変更しない。
AddLaneの固定基準はCoreがlaneの方向・追加側・断面順から決める。Webはrole、配列順、数値IDから候補を推測しない。

形式は一行一fieldのnamed indexed `key=value`で、カンマ位置に意味を持たせない。save field順は固定し、
top-level entityはID順にcanonical serializeする。順序に意味がある`SegmentShape.internal_knots`やPath spanは保存順を維持する。
doubleはload後に同じbinary doubleへ戻る表現で保存する。

旧versionと未知versionは明示rejectし、migrationしない。loadはparse、field構造検証、型変換、
`ValidateAuthoritativeGraph`、`generate_road`、不変条件検査を通過した場合だけ新stateを返す。duplicate key、missing key、
unknown key、count不一致、enum範囲外、非有限値、重複ID、欠損参照をrejectする。

## Preflight and validation

operation preflightはrequestの全fieldについて、ID存在、有限性、enum範囲、style ID、正であるべき寸法を検査する。
正しい入力だが現在扱えない接続形状・transition付きsplit・prepend distance移動は`kUnsupported`に残す。
valid authoritativeから派生が欠ける、emitへ不整合なresolved geometryが渡る等は`kInternal`とする。
`ValidateAuthoritativeGraph`はloadとoperation trial後に共通利用し、保存正本の完全性だけを見る。generateでしか判断できない
幾何対応可否は混ぜない。

## Segment locality

`RoadSegment`は局所追加、局所削除、split、shape編集、局所所有物の単位である。長い道路全体や一回の
描画セッションを表さない。shape編集の意味上の影響範囲は対象segment、その両端connection、
対象segmentを参照する局所所有物に限定する。

## RoadCorridor

`RoadCorridor`は`RoadCorridorId`、終端延長時の既定断面である`section_template_id`、順序付き
`DirectedSegmentRef`を正本として持つ。実体のない別`RoadDefinitionId`は作らない。
`reversed=false`はsegment startからend、`true`はendからstartへ進む。corridor内のsegmentは端点IDで
連続し、重複せず、他corridorへ重複所属しない。位置近接は連続性判定に使わない。
断面変更を含むcorridorではsegmentごとの`section_template`が異なってよい。template IDの一致を
接続可否やcorridor所属の判定に使わない。

corridor distanceは先頭から単調増加する。内部segment境界は後続segmentのlocal 0、corridor終端は末尾
segmentのlocal lengthへ解決する。reversed segmentでは`local = length - corridor-local`とする。
長さ、累積distance、解決結果は派生であり保存しない。

分岐は既存corridorへ枝を混入せず、新しい`RoadCorridor`を作る。共有nodeは許可するが、本線corridorの
segment列、方向、始点、distance、周期配置位相は変更しない。

## Corridor editing

`SplitSegmentAtDistance`はDe Casteljau分割を使い、元segment IDをstart側、新IDをend側へ割り当てる。
corridor内では元参照を二つの向き付き参照へ置換し、reversed参照ではcorridor方向に沿う順序へ反転する。
split前後でcorridor全長とworld位置を維持する。

通常削除はsegment単位で行う。corridorが分断される場合、
始点側が元corridor IDを維持し、終点側へ新IDを割り当てる。空corridorは保存しない。segment-local markingは
segment distanceでstart/end側へ明示移行し、境界を跨ぐmanual lineは現段階ではunsupportedとする。
approach overrideとjunction marking overrideはendpoint ID対応で移行し、一意に対応しないものはunsupportedとする。
通常のViewer削除からこのoperationを呼ばない。

`RoadSegment`はユーザーが一つとして選択・編集・削除する確定区間である。一回で確定したPathは複数Bezier spanを
内部に持ってよい。span、sample、mesh、marking runの境界をRoadSegment identityへ昇格しない。確定済みsegmentを
後から延長する操作は新しいRoadSegmentを追加する。

Viewerの通常削除はhover hitが持つ`RoadSegmentId`のsegment全体をハイライトし、同じIDを一回のクリックで
`DeleteSegment`へ渡す。Coreは選択segmentだけを削除し、結果として必要なcorridor分断を決定論的に行う。
通常削除はrange境界、split node、replacement segmentを生成しない。

道路沿いへ街路樹・標識・建物を並べるための参照型と周期配置policyは、consumerを実装する時点で
そのconsumerの要件から設計する。先行して型だけを置かない。

## Section axes

断面は物理`SectionStrip`、利用上の`StripFunction`、表示`SurfaceStyleId`、`LaneBand`によるlane割当を
別軸として扱う。styleからfunctionを推測しない。`Shoulder`はcurb幅や段差の代用ではなく独立した水平stripである。
車道外側線はcarriageway利用領域とshoulderのsemantic boundaryへ置き、shoulderがない場合だけ外側boundaryを使う。

現行persistence versionは11。局所segment、corridor、directed ref、section strip、lane allocation、
lane connection、boundary continuationをnamed fieldで保存し、version 10以前はmigrationせず明示rejectする。
corridor長、累積distance、connection geometryは派生なので保存しない。

testはproductionと同じ`ValidateGraphInvariants`を代表scenarioの観測点とseed付き操作列の
各stepで呼ぶ。test専用の別invariantや、派生値同士だけを比較する代替検査を作らない。

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
