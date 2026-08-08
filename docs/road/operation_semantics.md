# Road operation semantics (P0-P2 prototype scenarios)

Road drawing follows the shared contract in `docs/editor/draw_interaction.md`.
After the start anchor, each Click commits the displayed interval immediately.
Enter commits the current lightweight guide when one exists and ends the session after success. Escape
discards only transient drawing state and never calls road undo or deletion.

`city::road` の supported scenario を操作と状態の組で固定する。
表にない組み合わせは推測せず `unsupported` とする。
この表のscenario通過はarchitecture migration完了を意味しない。各public operationの実装契約は次で固定する。

```text
Request -> Preflight -> OperationPlan -> trial Apply -> generate_road -> Validate -> Commit
```

失敗時はauthoritative bytes、derived hash、next ID、inspection / query結果を一切変更しない。
public operationから別public operationを呼ばない。

Preflightはrequestの形式、ID存在、有限性、明白な値域と、planを作るために必要な局所構造だけを検証する。
接続角度、endpoint section互換、branch / junction approach上限、接続setback、split後を含むtrial topologyの成立性は
trial generateの`resolve_connections`以降が一度だけ決める。operationは同じgeometry / section policyを
再実装せず、trial generateの`kUnsupported`を操作結果として返す。

## Public operation preflight audit

| Operation | Request field | Preflight validation | generate decision |
|---|---|---|---|
| AddSegment | alignment | finite、span端点がID作成順に連続、非ゼロ | self-intersectionは文書化されたRequirementConstraint、junction互換限界はNotImplemented |
| AddSegment | layout_template | ID exists | endpoint section互換 |
| ExtendCorridorFromEnd | corridor_id / endpoint_node_id | ID exists、corridor末尾nodeと一致、degree-one | corridor先頭延長はunsupported |
| ExtendCorridorFromEnd | extension | finite、連続、非ゼロ、endpointへ補正可能 | 新しい局所segmentと隣接connection |
| ExtendCorridorFromEnd | layout_template | corridorのroad definitionと一致 | 異断面延長はunsupported |
| SplitSegmentAtDistance | segment_id / segment_distance_m | ID exists、finite、0とlengthを除く範囲内 | De Casteljau分割とowner移行 |
| AddSegmentConnectedTo | alignment / connected_endpoint | finite、連続、非ゼロ、指定したstart/end endpointをnodeへ補正可能 | 接続角、degree、setback |
| AddSegmentConnectedTo | layout_template / connected_node | ID exists | endpoint section互換 |
| AddSegmentConnectedToSegment | alignment / connected_endpoint | finite、連続、非ゼロ、指定したstart/end endpointが明示distance点と一致 | split後topology、junction互換 |
| AddSegmentConnectedToSegment | target_segment / segment_distance_m / layout_template | ID exists、distance finite、distance interior | transition付きsplit unsupported |
| EditSegmentShape | segment_id / shape | ID exists、全handle/knot finite | 接続後shapeのgenerate可否 |
| MoveNode | node_id / position | ID exists、position finite | incident segment / connection再導出 |
| DeleteSegment | segment_id | ID exists | 不要transition / marking / policy除去 |
| AddRoadLayoutTemplate | layout_template | ID一意、strip/lane/boundary ID一意、参照整合、width正、finite、enum valid、known SurfaceStyleId、lane side markingはcarriageway stripのみ | trial section evaluation。IDはCoreが採番して返す |
| EditRoadLayoutTemplate | layout_template | ID exists、strip/lane/boundary ID一意、参照整合、width正、finite、enum valid、known SurfaceStyleId、lane side markingはcarriageway stripのみ | 既存segment再評価。boundary policyとlane side policyはこの操作だけが変える |
| AddLane | corridor/direction/side/変化開始SegmentPosition/完成SegmentPosition/維持終点node/lane width | corridorとsegment/nodeが存在、tがfiniteかつ[0,1]、同一segment内でcorridor方向の順序が有効、幅が正 | Coreが外側laneと固定boundaryを解決し、最初のsegmentへtransition、明示終点までの後続segmentへ完成断面を設定 |

外部入力不正は`kValidation`、正しい入力だがP0-P2で対応しない構造は`kUnsupported`、正しい正本から派生表やresolved read modelが欠ける場合は`kInternal`とする。

新規`RoadState`は`layout_templates`が空である。道路製品として提供する断面の一覧と具体値は
Webが所有し(`web/src/road_templates.ts`)、新規workspaceの作成時にだけ`AddRoadLayoutTemplate`で
登録する。Loadは保存済みの断面をそのまま使い、Web presetを再注入しない。
template IDは互換性判定には使わず、接続可否はendpointの実断面で判定する。
ADD LANEが作る変更後断面はCoreが作る。

`RoadLayoutTransition`、`LaneConnection`、`BoundaryContinuation`、`ApproachGeometryOverride`、
`AutoMarkingOverride`、`JunctionMarkingOverride`、`ManualLineMarking`、`ManualAreaMarking`は
保存される正本のままだが、これらを直接書き換える公開操作はない。IDと参照の妥当性は
`ValidateAuthoritativeGraph`が、断面と線が成立するかは`generate_road`が判定する。
これらの行を持つ道路はloadで入り、`AddLane`とjunction生成が内部で書く。

## 状態

| 状態 | 意味 |
|---|---|
| empty | segment がない |
| isolated | 接続されていない segment |
| connected | node IDを共有する同一断面segment。connection kindはbuild派生で保存しない |
| transitioning | segment が 1 個の RoadLayoutTransition を参照する |
| marked | segment が ManualLineMarking / ManualAreaMarking を所有する |

## 操作 x 状態

| 操作 | empty | isolated | connected | transitioning | marked |
|---|---|---|---|---|---|
| AddSegment | supported | supported | supported | supported | supported |
| ExtendCorridorFromEnd | validation | corridor末尾かつ同一道路定義ならsupported | 対象終端がdegree 1ならsupported | 新segmentへ局所所有、既存segment不変 | 新segmentへ局所所有、既存segment不変 |
| SplitSegmentAtDistance | validation | 内部distanceならsupported | 新degree 2 nodeを作りderived connection | distanceで一意移行、境界跨ぎはunsupported | distanceで一意移行、境界跨ぎはunsupported |
| AddSegmentConnectedTo | validation | gate断面ID一致のみsupported | gate断面ID一致かつdegree 4までsupported | 終端gateがto断面ならsupported | gate断面ID一致のみsupported |
| AddSegmentConnectedToSegment | validation | 同一断面かつ明示`segment_id + segment_distance_m`ならsupported | 同一断面かつ明示`segment_id + segment_distance_m`ならsupported | unsupported | 同一断面かつ明示`segment_id + segment_distance_m`ならsupported |
| EditSegmentShape | validation | supported | node位置を変えずshapeだけ変更 | DistanceRef規則で再評価 | owner-local markingを追従 |
| MoveNode | validation | endpoint nodeを移動 | 接続全segmentを再導出 | DistanceRef規則で再評価 | owner-local markingを追従 |
| DeleteSegment | validation | 選択したRoadSegment 1件だけを削除 | corridor分断は決定論的に処理 | 対象segmentのownerだけを除去 | 対象segmentのmarkingを除去 |
| Viewer delete road | 1 segment pick | `DeleteSegment` | hoverしたRoadSegment全体を1クリックで削除 | hoverとclickは同じ明示segment IDを使う | splitやrange境界を作らない |
| Add/EditRoadLayoutTemplate | supported | supported | supported | supported | supported |
| AddLane | validation | segmentをsplitせずtでsupported | transition自体は同一segment内、完成断面の明示終点までの伝播はsupported | 1 segment 1 transition。同一区間の競合は具体的なvalidation | supported |

## P1 node semantics

- `RoadSegment`はユーザーが一つとして選択・編集・削除する確定区間である。一回で確定したPathは複数Bezier
  spanを内部に持ってよく、span境界、sample、mesh分割、marking runをsegment境界へ昇格しない。
- 一回で確定した`A -> B -> C`は1 RoadSegmentになり得る。`A -> B`を確定した後の`B -> C`延長は
  新しいRoadSegmentを作る。確定境界が同じ操作同士について決定性を要求する。
- degree 1のcorridor末尾から同じ道路定義を伸ばす既定操作は`ExtendCorridorFromEnd`とする。既存segmentと
  既存nodeを変更せず、新node、新segment、corridor末尾参照だけを追加する。
- corridor先頭延長はcorridor-owned distance参照の移行規則が未実装なのでunsupportedとする。
- 逆方向入力はID、segment向き、distance原点が異なるためraw serialization一致を要求しない。
  CanonicalAlignmentを同じ向きへ正規化した形状、material別mesh頂点集合、marking形状の一致を要求する。
- segment途中への接続は`target_segment_id + segment_distance_m + connected_endpoint`を入力とし、alignmentの指定endpointをsplit nodeへ接続する。座標近接から対象segment、距離、接続端を再推測しない。
- Line / Bezierは入力toolの区別に限定し、正本はendpointを含まない`SegmentShape`、完全Pathは派生`CanonicalAlignment`とする。
  Line入力は各spanをlinear cubic Bezierとして保存し、`SegmentShape.intent = Straight`を持つ。
  Bezier評価、sampling、emitはLine / Bezierで分岐しない。曲線segmentの分割はDe Casteljau分割を使う。
- Bezierを連続描画するときは、Core previewが直前segmentのcubic endpoint derivativeを読み、次segmentの
  開始handleをその方向へsnapする。Straight predecessorも同じ規則を使う。hover previewとクリック確定は
  同じsnap済みPathを使用し、commit operationは受け取ったcontrol pointを補正しない。
  G1入力のために接続種別、正本field、補正operationを追加しない。
- cubic endpoint tangentはcontrol handleを正とする。span内部もanalytic derivativeを使う。明示的な非G1
  internal knotは距離差分で平滑化せず、前後offset lineの交点を作る共通miter frameとして扱う。
- Viewerのalignment編集では、segment端点handleは`MoveNode`、内部2 control handleは
  `EditSegmentShape`へ送る。直線segmentもlinear cubic Bezierの4 control pointを表示し、
  内部control handleを動かした時点でcurve intentへ変更する。
- ViewerのDelete roadはhoverした`RoadSegmentId`をそのまま`DeleteSegment`へ渡し、1回のクリックで
  そのユーザー区間全体を削除する。hover表示と削除対象は同じIDを使い、位置から再検索しない。
- Line segmentをnodeやsegment distanceへsnapする場合は、snap後の始点と入力終点からlinear cubicを再作成する。
  始点だけを動かしてhandleを一部だけ補正しない。
- `SplitSegmentAtDistance`は元segment IDをstart側へ維持し、end側segmentとsplit nodeへ新IDを付ける。
  corridor内がreversedなら、corridor進行方向を維持する順序で二参照へ置換する。
- segment-local manual point/areaはdistanceがsplit点より前ならstart側、後ならend側へ移してdistanceを減算する。
  split点を跨ぐmanual line、transition、意味を維持できないoverrideがある場合はunsupportedとする。
  corridor-owned参照を一意に移行できない場合はunsupportedとし、近接rebindや黙示削除を行わない。
- degree 2 の自動decisionはPassThroughまたはCorner。対向する2本は幅を増やさず連続し、屈曲する2本はgate接線へG1接続する中心曲線を一度だけ導出し、その移動frameへ同じ断面を掃引する。停止線・ゼブラは生成しない。
  Straight segmentの正本はcornerのために曲げず、丸みはnode所有のderived connectorだけに置く。
- Corner内のboundary markingは同じ掃引済みboundary curveを使用して両segmentの線を接続する。boundary配列順やworld位置で対応を推測しない。
- degree 3/4 の自動decisionは対応範囲ならJunction。自動junctionの存在は保存しない。
- Junctionの断面側はtemplate IDで分岐せず、各approachの左右についてroad outer、任意のcurb、
  任意のshoulder境界、carriageway edgeをboundary roleとstrip adjacencyから一度だけ解決する。
  片側歩道なし、shoulderあり、curbなしも同じ規則で扱う。同じ側にcarriageway edge候補が複数あり
  意味を一意に解けない断面だけをunsupportedとする。
- degree 3/4の角度検査は接線順で隣接するapproachの道路本体chord角だけを検査する。
  非隣接approachをdegree 2 cornerの45–135度制限へ通さない。同じ接線から別方向へ
  分かれるBezier approachは、chord角が対応範囲ならstable ID順で扱う。
- ユーザーが明示した場合だけ`NodeConnectionPolicyOverride`を保存する。overrideを削除するとAutoへ戻る。
- 実交差点の各gate位置は固定距離にしない。接続角と各approachの断面幅に加え、共通corner radiusを置くための接線長から、approach同士と丸め面が重ならないsetbackを一度だけ決定する。
- 実交差点はcarriagewayだけでなく、gate断面のshoulder境界、sidewalk、curbを隣接approach間へ接続する。同じAsphaltであるcarriagewayとshoulderは交差点内で一つのsurface regionが所有し、重複meshを作らない。
- 異なるside断面の接続はcarriageway edge同士とroad outer同士を固定し、その間のface列を対応させる。片側だけにあるshoulderはcarriageway edgeで幅0へ退化させ、gutterの上面端をroad outerへ接続しない。各gateのgutter輪郭寸法は変更しない。
- 交差点cornerとdegree 2の屈曲connectorは、gate接線を共有するBezier形状として派生する。segment側とconnector側で接線を再解釈しない。
- MoveNodeではStraight intentを持つincident segmentだけを新node位置からlinear handleへ再導出する。Curve intentのcontrol handlesは維持する。nodeに接続しない遠方segmentは変更しない。
- 自動停止線・ゼブラは実交差点だけに生成する。向きと高さはgate frameおよびSectionEvaluationの横断勾配から導出する。

## Simple path

- 単独segmentとdegree 1終端はjunction接続準備を要求しない。
- `ApproachKey`と`ConnectionGate`はdegree 2以上の明示接続nodeだけに生成する。
- auto値とmanual overrideは`resolve_connections`内で合成する。degree 1 endpoint overrideはunsupported。
- segment内部の通常曲線は`CanonicalAlignment`とSectionEvaluationから直接生成し、junctionsへ渡さない。
- `RoadGraph -> generate_road`の一回で必要な派生物を生成し、操作履歴や事前tableの有無でstage経路を変えない。

## Approach geometry override

- `ApproachKey = node_id + segment_id + endpoint_role`だけをidentityとする。
- setbackはnodeからsegment内部方向への非負距離。startは`distance=setback`、endは`distance=length-setback`。
- 接続segmentの可否を固定最小長では決めない。各接続が必要とするsetbackがsegment内へ収まり、両端gateが重ならない場合は短いsegmentも受け入れる。
- lateral shiftはresolved approach lateral方向を正とする。
- manual fieldだけを`ApproachGeometryOverride`へ保存する。auto setback / auto lateral shiftは保存しない。
- reset fieldで該当manual fieldを消し、全fieldがAutoになったoverride entityは削除する。
- segment削除では該当segment/nodeのoverrideを同じOperationPlanで削除する。
- segment splitでは元segment外側endpointのoverrideだけを新しい外側segmentへID mappingし、内部nodeへ複製しない。

## P2 transition semantics

- `DistanceRef::FromStart(d)` は `d`、`FromEnd(d)` は `length-d`、`Ratio(u)` は `length*u` に解決する。
- 解決後は `0 <= start < end <= length` を満たす。満たさなければ validation で正本を変更しない。
- transition前は `from_template`、transition後は `to_template`、区間内は線形補間する。
- `anchor` は補間中に固定する断面基準で、Center / LeftEdge / RightEdge / 明示BoundaryIdのいずれか。
- `AddLane`は方向と側から外側laneと固定boundaryをCoreで一意に解決する。laneの外側順はstrip順とstrip内のlane範囲から決め、1 laneが1 strip全幅を占有することを要求しない。固定boundaryは選択側から内側へ探索し、利用者へboundary IDを要求せず、固定boundary内側の既存laneを移動しない。
- `AddLane`がjunction approachへ到達した場合、退出lane数、進入lane数、boundary role、両側strip functionが一致する接続先が1 approachだけなら、既存対応を維持して不足する`LaneConnection`と`BoundaryContinuation`を同じplanへ追加する。候補がなければlaneはgateで意図的に終了し、候補が複数なら接続先を推測せず具体的なunsupportedを返す。
- `ADD LANE`の正本位置は、変化開始と3車線完成の`SegmentPosition(segment_id, t)`である。`t`はsegment正本方向の始点を0、終点を1とし、物理taper長、junctionからの距離、corridor-globalなlane additionは保存しない。transitionは一つのsegment内だけに置き、開始・完成位置を作るためにsegmentをsplitしない。
  操作は明示した維持終点まで複数segmentへ作用できる。最初のsegmentだけがtransitionを持ち、後続segmentは完成断面を通常のsectionとして持つ。道路形状や長さが変わっても`t`を維持し、物理位置は新しい形状へ追従する。reversed corridorではCoreがcorridor方向の順序を検証し、segment localのfrom/toへ正規化する。
  追加lane幅だけを0から指定幅へ単調に増やし、その外側のshoulderとcurbを外へ移す。断面全体を再中心化しない。
- 対応範囲はforward/reversedを含むcorridor内の複数segmentで、transitionを持つsegmentに別のtransitionを重ねないlane追加である。競合は具体的なvalidationとして拒否する。
- element対応は ID で行う。出現は `TaperIn`、消滅は `TaperOut` または `EndCap` を明示する。
- 1 segmentに同時に接続できる transition は1個。短距離多重transitionはP2非対象。
- 異なるendpoint sectionでも、共通lane identityとsemantic sideを一意に解決できる接続は同じ
  `resolve_connections`経路で生成する。degree 2とjunctionで別のsection互換規則を持たず、carriageway edgeと
  road outerを固定し、中間side faceの不足はcarriageway edgeで幅0へ退化させる。laneまたはboundaryの対応が
  一意でない場合だけ明示topologyを要求し、operation preflightは断面を再評価しない。

## P2 marking semantics

- boundary marking は `DerivedSegment.sections` の boundary ID / `AutoMarkingPolicy` を唯一の入力とし、emitはsection ruleを読まない。
- ManualLineMarking の Path と ManualAreaMarking の frame は owner segment local `(distance, lateral)`。
  ManualAreaMarking は `rotation_rad` を持ち、0 はowner segment distance方向を意味する。
- 自動線は`DerivedMarking`へ導出してからmesh化する。segment、junction、manual のowner境界はgateとmanual entity IDで決め、位置近接で再bindしない。
- 保存するのはlocal値で、world meshだけを派生する。segment alignment編集時はlocal値を維持して再導出する。
- 境界への線要求は`BoundaryProfile.marking`と、carriageway `RoadLayoutStrip`の`LaneSideMarkingPolicy`だけ。
  lane side要求は要素順序で隣接する`BoundaryProfile`へ解決し、断面外端側の要求はunsupportedとする。
- 同一境界への複数要求は、role / style / geometry ruleが一致する場合だけ1本へ統合する。
  一致しない要求はunsupportedとし、暗黙の優先順位を持たない。
- 車線増減では、boundary IDが継続する線はContinue、出現する線はBegin、消滅する線はTerminateとする。
  隣接band幅が退化している区間には線を出さない。
- transition前後でboundary IDのroleが変わる場合、および`TransitionAction::kUnsupported`がmarking付き要素を
  指す場合はunsupportedとする。近い新boundaryへ自動でrebindしない。
- junction markingのうちCenterLineとLaneSeparatorはdefaultでgate終了し、`JunctionMarkingOverride`で明示した場合だけ交差点内接続を作る。shoulderを区切るCarriagewayEdgeは、隣接する2 approachの対応boundaryでrole/styleが一致する場合、sidewalkと同じjunction perimeter curveへ自動接続する。片側に線がなければgateで終了し、明示`BoundaryContinuation`があるendpoint pairへ自動線を重ねない。
  targetは明示IDのみで、最寄り・正面・角度差から推測しない。

### 操作 x marking状態

| 操作 | policyなし | boundary policyあり | lane side policyあり | 競合要求 | suppressionあり |
|---|---|---|---|---|---|
| EditRoadLayoutTemplate (boundary policy) | supported | 置換 | 一致すれば統合 | unsupported | supported (自動線は非生成のまま) |
| EditRoadLayoutTemplate (lane side policy) | supported | 一致すれば統合 | 置換 | unsupported | supported |
| 保存された`AutoMarkingOverride` | validation (track不在) | supported | supported | - | 冪等 |
| 保存された`JunctionMarkingOverride` | supported | supported | supported | - | supported |

## Lane topology authority

- `LaneId`と`BoundaryId`は、segmentが参照する断面template内のstable identityである。横方向の配列indexはidentityではない。
- lane endpointは`LaneEndpointKey(segment_id, lane_id, endpoint_role)`、boundary endpointは`BoundaryEndpointKey(segment_id, boundary_id, endpoint_role)`で指定する。
- 確定したlane接続は`LaneConnection`、境界継続は`BoundaryContinuation`として保存する。world位置や近さから保存接続を再構成しない。
- `LaneTravelDirection`はsegment正本方向に対する`AlongSegment` / `AgainstSegment`であり、接続時の方向整合に使う。
- `LaneConnection.source`はsource laneの退出端、`target`はtarget laneの進入端でなければならず、両端は同じ`RoadNodeId`を共有する。
- Continuation / Transitionは一対一、Splitは同一sourceから複数target、Mergeは複数sourceから同一targetを許可する。JunctionMovementは明示された複数movementを許可する。同一source-targetの重複は常にvalidation rejectする。
- `BoundaryContinuationKind`も同じ多重度契約を持つ。Continuationは一対一、Splitは同一sourceから複数target、Mergeは複数sourceから同一targetだけを許可する。
- 同一surface strip内のlane lateral intervalは重複させない。vector順はidentityに使わず、横方向順序はintervalから検証する。
- 高速道路、出口、合流専用の正本型を追加せず、車線追加、分岐、合流、交差点movementを同じ接続型で表す。
- lane center接続からshoulder、車道端、白線の継続を推測しない。必要なboundary接続は別に明示する。
  異なる断面を接続する場合も、source laneとtarget laneをIDで明示しない操作はunsupportedのままとする。
- straightなContinuationのtarget approach lateral shiftは、明示された全lane center対応が要求する共通値を一度だけ導出する。
  複数mappingが異なるshiftを要求する場合は形状を曲げて合わせずunsupportedとする。
- Splitの`DerivedLanePath`はsource lane centerからbranch lane centerへG1接続する1本のBezierで、endpoint接線と
  横・長手方向の単調性を検証する。mainline Continuationはbranchのために曲げない。
- branchへ明示した2本のBoundaryContinuationから有限なseparation areaを派生する。lane centerから境界を推測しない。
- Mergeはbranch lane centerから明示したmainline lane centerへG1接続する。straightなmainline mappingだけでtarget approachの共通shiftを決め、角度を持つbranch mappingはmainlineを曲げる入力にしない。
- 対向断面のbranchは選択した`LaneEndpointKey`と境界だけをSplitへ載せる。反対方向laneはContinuationのまま維持し、median boundaryをbranchへ暗黙流用しない。
- `JunctionMovement`はdegree 3/4のjunctionでだけsupportedとする。同一source laneから複数target laneを明示できるが、source退出・target進入・同一nodeを満たさない接続はvalidation、junction以外へのmovement指定はunsupportedとする。
- movement geometryはsource/target gate上のlane centerをG1 cubicで結ぶ。直進のminimum radiusは正の無限大を許可し、turnは有限かつ正のradiusを要求する。
- Split/Mergeの接続白線は対応する`DerivedBoundaryPath`へ追従する。Splitではsource marking policy、Mergeではtarget marking policyを接続区間へ使い、線専用の自由Bezierやlane centerからの境界推測を行わない。
- ViewerのAddLaneは方向・側、変化開始位置、3車線完成位置、3車線を維持する終点を順に選択し、segment IDと`t`および終点node IDをCoreへ渡す。固定する断面境界または外端はCoreが決める。raw lane/boundary/template IDは通常UIへ出さない。BranchLane / MergeLaneはCore payloadのlane pathをhoverし、選択した`LaneEndpointKey`をpreviewと確定の両方へ同一値で渡す。
- AddLaneのdirectionとsideはcorridor方向を基準とする。`DirectedSegmentRef.reversed`ではCoreがsegment localのdirection、side、transitionのfrom/toへ正規化し、同じ操作意味を維持する。
- Viewerはlane/boundary接続先や接続geometryを推測しない。AddLaneのpointer previewは選択位置と影響範囲の軽量guideだけで、全Core generationを行わない。確定失敗時は選択を維持し、理由を一度表示する。
