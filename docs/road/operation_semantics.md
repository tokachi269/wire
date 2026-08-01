# Road operation semantics (P0-P2 prototype scenarios)

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
| AddSegment | alignment | finite、span端点がID作成順に連続、非ゼロ | self-intersection、junction互換 |
| AddSegment | section_template | ID exists | endpoint section互換 |
| ExtendCorridorFromEnd | corridor_id / endpoint_node_id | ID exists、corridor末尾nodeと一致、degree-one | corridor先頭延長はunsupported |
| ExtendCorridorFromEnd | extension | finite、連続、非ゼロ、endpointへ補正可能 | 新しい局所segmentと隣接connection |
| ExtendCorridorFromEnd | section_template | corridorのroad definitionと一致 | 異断面延長はunsupported |
| SplitSegmentAtDistance | segment_id / segment_distance_m | ID exists、finite、0とlengthを除く範囲内 | De Casteljau分割とowner移行 |
| DeleteSegmentRange | segment_id / start/end segment distance | ID exists、finite、start < end、segment範囲内 | split、削除、corridor分断、owner移行 |
| AddSegmentConnectedTo | alignment | finite、連続、非ゼロ、start nodeへ補正可能 | 接続角、degree、setback |
| AddSegmentConnectedTo | section_template / start_node | ID exists | endpoint section互換 |
| AddSegmentConnectedToSegment | alignment | finite、連続、非ゼロ、明示distance点と一致 | split後topology、junction互換 |
| AddSegmentConnectedToSegment | start_segment / segment_distance_m / section_template | ID exists、distance finite、distance interior | transition付きsplit unsupported |
| EditSegmentShape | segment_id / shape | ID exists、全handle/knot finite | 接続後shapeのgenerate可否 |
| MoveNode | node_id / position | ID exists、position finite | incident segment / connection再導出 |
| DeleteSegment | segment_id | ID exists | 不要transition / marking / policy除去 |
| SetApproachSetbackOverride | ApproachKey / setback_m | node exists、segment exists、endpoint role matches、finite、non-negative、layout target exists | gate overlap、segment length、junction quality |
| SetApproachLateralShiftOverride | ApproachKey / lateral_shift_m | node exists、segment exists、endpoint role matches、finite、layout target exists | self-intersection、junction quality |
| ResetApproachOverrideField | ApproachKey / field | node exists、segment exists、endpoint role matches、field enum valid | resolved layout rebuild |
| ResetAllApproachOverrides | ApproachKey | node exists、segment exists、endpoint role matches | resolved layout rebuild |
| AddSectionTemplate | section_template | ID一意、strip/lane/boundary ID一意、参照整合、width正、finite、enum valid、known SurfaceStyleId | trial section evaluation |
| EditSectionTemplate | section_template | ID exists、strip/lane/boundary ID一意、参照整合、width正、finite、enum valid、known SurfaceStyleId | 既存segment再評価 |
| AddTransition | from/to/start/end/anchor/rules | template exists、distance finite/range、rule element exists、enum valid | element出現/消滅action対応 |
| AddTransitionToSegment | segment_id + transition request | segment exists、transition preflight | 既存transition replace、segment section整合 |
| AddLaneTransition | corridor/direction/side/distances/lane width/anchor boundary | corridor exists、finite、正の幅、同一forward segment、anchorが選択方向の外側laneの内側境界 | anchor内側を固定し、追加laneと外側shoulderだけを単調に展開 |
| AttachSectionTransition | segment_id / transition_id | ID exists、from_template matches segment、distance range valid | segment再評価 |
| AddManualLine | owner_segment_id / path / style_id | owner exists、path finite、owner-local distance範囲、known MarkingStyleId | owner section上への投影 |
| AddManualArea | owner_segment_id / frame_origin / width / length / style_id | owner exists、finite、width/length正、distance範囲、known MarkingStyleId | owner section上への投影 |
| SetBoundaryMarkingPolicy | section_template_id / boundary_id / policy | template exists、boundary exists、role enum valid、known MarkingStyleId | 要求統合、継続再評価 |
| SetLaneSideMarkingPolicy | section_template_id / strip_id / side / policy | template exists、strip exists、carriageway strip、role enum valid、known MarkingStyleId | 隣接boundaryへの解決、要求統合 |
| ResetLaneSideMarkingPolicy | section_template_id / strip_id / side | 同上(policyは無効値) | 要求統合、継続再評価 |
| SuppressAutoMarking / ResetAutoMarkingSuppression | AutoMarkingKey | owner種別ごとにID存在、segmentはtrack必須、junctionはapproach必須 | 該当自動線のみ非生成 |
| SetJunctionMarkingOverride | node_id / action / source / target | node exists、approachがnodeに属する、boundaryがgateに存在、ConnectToApproachはtarget必須 | junction-owned pathの生成 |
| DeleteJunctionMarkingOverride | override_id | ID非ゼロ | default terminateへ復帰 |

外部入力不正は`kValidation`、正しい入力だがP0-P2で対応しない構造は`kUnsupported`、正しい正本から派生表やresolved read modelが欠ける場合は`kInternal`とする。

## 状態

| 状態 | 意味 |
|---|---|
| empty | segment がない |
| isolated | 接続されていない segment |
| connected | node IDを共有する同一断面segment。connection kindはbuild派生で保存しない |
| transitioning | segment が 1 個の SectionTransition を参照する |
| marked | segment が ManualLineMarking / ManualAreaMarking を所有する |

## 操作 x 状態

| 操作 | empty | isolated | connected | transitioning | marked |
|---|---|---|---|---|---|
| AddSegment | supported | supported | supported | supported | supported |
| ExtendCorridorFromEnd | validation | corridor末尾かつ同一道路定義ならsupported | 対象終端がdegree 1ならsupported | 新segmentへ局所所有、既存segment不変 | 新segmentへ局所所有、既存segment不変 |
| SplitSegmentAtDistance | validation | 内部distanceならsupported | 新degree 2 nodeを作りderived connection | distanceで一意移行、境界跨ぎはunsupported | distanceで一意移行、境界跨ぎはunsupported |
| DeleteSegmentRange | validation | 内部範囲ならsupported | 残存端点を再評価 | 範囲交差ownerはunsupported | 範囲交差ownerはunsupported |
| AddSegmentConnectedTo | validation | gate断面ID一致のみsupported | gate断面ID一致かつdegree 4までsupported | 終端gateがto断面ならsupported | gate断面ID一致のみsupported |
| AddSegmentConnectedToSegment | validation | 同一断面かつ明示`segment_id + segment_distance_m`ならsupported | 同一断面かつ明示`segment_id + segment_distance_m`ならsupported | unsupported | 同一断面かつ明示`segment_id + segment_distance_m`ならsupported |
| EditSegmentShape | validation | supported | node位置を変えずshapeだけ変更 | DistanceRef規則で再評価 | owner-local markingを追従 |
| MoveNode | validation | endpoint nodeを移動 | 接続全segmentを再導出 | DistanceRef規則で再評価 | owner-local markingを追従 |
| DeleteSegment | validation | 選択したRoadSegment 1件だけを削除 | corridor分断は決定論的に処理 | 対象segmentのownerだけを除去 | 対象segmentのmarkingを除去 |
| Viewer delete road | 1 segment pick | `DeleteSegment` | hoverしたRoadSegment全体を1クリックで削除 | hoverとclickは同じ明示segment IDを使う | splitやrange境界を作らない |
| Add/EditSectionTemplate | supported | supported | supported | supported | supported |
| AttachSectionTransition | validation | supported | supported | replace supported | supported |
| AddLaneTransition | validation | corridor末尾segment内ならsupported | corridor末尾segment内ならsupported | overlapはunsupported | supported |
| AddManualLine / AddManualArea | validation | supported | supported | supported | supported |

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
- segment途中への接続は`target_segment_id + segment_distance_m`を入力とし、alignmentを該当距離で分割する。座標近接から対象segmentや距離を再推測しない。
- Line / Bezierは入力toolの区別に限定し、正本はendpointを含まない`SegmentShape`、完全Pathは派生`CanonicalAlignment`とする。
  Line入力は各spanをlinear cubic Bezierとして保存し、`SegmentShape.intent = Straight`を持つ。
  Bezier評価、sampling、emitはLine / Bezierで分岐しない。曲線segmentの分割はDe Casteljau分割を使う。
- Bezierを連続描画するときは、viewerの一時入力状態が直前segmentの終端接線を保持し、次segmentの開始handleを
  その方向へsnapする。hover previewとクリック確定は同じsnap済み点を使用する。
  G1入力のためにCoreの接続種別、正本field、補正operationを追加しない。
- Viewerのalignment編集では、segment端点handleは`MoveNode`、内部2 control handleは
  `EditSegmentShape`へ送る。直線segmentもlinear cubic Bezierの4 control pointを表示し、
  内部control handleを動かした時点でcurve intentへ変更する。
- ViewerのDelete roadはhoverした`RoadSegmentId`をそのまま`DeleteSegment`へ渡し、1回のクリックで
  そのユーザー区間全体を削除する。hover表示と削除対象は同じIDを使い、位置から再検索しない。
- `DeleteSegmentRange`は明示距離範囲を切り取る高度編集であり、標準削除UIから呼ばない。
- Line segmentをnodeやsegment distanceへsnapする場合は、snap後の始点と入力終点からlinear cubicを再作成する。
  始点だけを動かしてhandleを一部だけ補正しない。
- `SplitSegmentAtDistance`は元segment IDをstart側へ維持し、end側segmentとsplit nodeへ新IDを付ける。
  corridor内がreversedなら、corridor進行方向を維持する順序で二参照へ置換する。
- segment-local manual point/areaはdistanceがsplit点より前ならstart側、後ならend側へ移してdistanceを減算する。
  split点を跨ぐmanual line、transition、意味を維持できないoverrideがある場合はunsupportedとする。
- `DeleteSegmentRange`でcorridorが分断された場合は始点側が元corridor IDを維持し、終点側へ新IDを付ける。
  corridor-owned参照を一意に移行できない場合はunsupportedとし、近接rebindや黙示削除を行わない。
- degree 2 の自動decisionはPassThroughまたはCorner。対向する2本は幅を増やさず連続し、屈曲する2本はgate接線へG1接続する中心曲線を一度だけ導出し、その移動frameへ同じ断面を掃引する。停止線・ゼブラは生成しない。
  Straight segmentの正本はcornerのために曲げず、丸みはnode所有のderived connectorだけに置く。
- Corner内のboundary markingは同じ掃引済みboundary curveを使用して両segmentの線を接続する。boundary配列順やworld位置で対応を推測しない。
- degree 3/4 の自動decisionは対応範囲ならJunction。自動junctionの存在は保存しない。
- degree 3/4の角度検査は接線順で隣接するapproachの道路本体chord角だけを検査する。
  非隣接approachをdegree 2 cornerの45–135度制限へ通さない。同じ接線から別方向へ
  分かれるBezier approachは、chord角が対応範囲ならstable ID順で扱う。
- ユーザーが明示した場合だけ`NodeConnectionPolicyOverride`を保存する。overrideを削除するとAutoへ戻る。
- 実交差点の各gate位置は固定距離にしない。接続角と各approachの断面幅から、approach同士が重ならないsetbackを一度だけ決定する。
- 実交差点はcarriagewayだけでなく、gate断面のsidewalkとcurbを隣接approach間へ接続する。
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
- `AddLaneTransition`は明示`anchor_boundary_id`の両側位置を同じ断面評価から導出し、anchor内側の既存laneを移動しない。
  追加lane幅だけを0から指定幅へ単調に増やし、その外側のshoulderとcurbを外へ移す。断面全体を再中心化しない。
- 最初の対応範囲は、1本のforward corridor segment内で完結し、既存transitionと重ならないlane追加である。
  複数segmentまたぎ、reversed segment、既存transitionとの合成は推測せずunsupportedとする。
- element対応は ID で行う。出現は `TaperIn`、消滅は `TaperOut` または `EndCap` を明示する。
- 1 segmentに同時に接続できる transition は1個。短距離多重transitionはP2非対象。
- 異なる断面をnodeへ直接接続しない。trial generateの`resolve_connections`が各approachのendpoint section IDを
  一度だけ解決し、全approachで完全一致する場合だけ接続する。operation preflightは断面を再評価しない。

## P2 marking semantics

- boundary marking は `DerivedSegment.sections` の boundary ID / `AutoMarkingPolicy` を唯一の入力とし、emitはsection ruleを読まない。
- ManualLineMarking の Path と ManualAreaMarking の frame は owner segment local `(distance, lateral)`。
  ManualAreaMarking は `rotation_rad` を持ち、0 はowner segment distance方向を意味する。
- 自動線は`DerivedMarking`へ導出してからmesh化する。segment、junction、manual のowner境界はgateとmanual entity IDで決め、位置近接で再bindしない。
- 保存するのはlocal値で、world meshだけを派生する。segment alignment編集時はlocal値を維持して再導出する。
- 境界への線要求は`BoundaryProfile.marking`と、carriageway `SectionStrip`の`LaneSideMarkingPolicy`だけ。
  lane side要求は要素順序で隣接する`BoundaryProfile`へ解決し、断面外端側の要求はunsupportedとする。
- 同一境界への複数要求は、role / style / geometry ruleが一致する場合だけ1本へ統合する。
  一致しない要求はunsupportedとし、暗黙の優先順位を持たない。
- 車線増減では、boundary IDが継続する線はContinue、出現する線はBegin、消滅する線はTerminateとする。
  隣接band幅が退化している区間には線を出さない。
- transition前後でboundary IDのroleが変わる場合、および`TransitionAction::kUnsupported`がmarking付き要素を
  指す場合はunsupportedとする。近い新boundaryへ自動でrebindしない。
- junction markingのdefaultはgate終了。`JunctionMarkingOverride`で明示した場合だけ交差点内接続を作る。
  targetは明示IDのみで、最寄り・正面・角度差から推測しない。

### 操作 x marking状態

| 操作 | policyなし | boundary policyあり | lane side policyあり | 競合要求 | suppressionあり |
|---|---|---|---|---|---|
| SetBoundaryMarkingPolicy | supported | 置換 | 一致すれば統合 | unsupported | supported (自動線は非生成のまま) |
| SetLaneSideMarkingPolicy | supported | 一致すれば統合 | 置換 | unsupported | supported |
| SuppressAutoMarking | validation (track不在) | supported | supported | - | 冪等 |
| SetJunctionMarkingOverride | supported | supported | supported | - | supported |

## Lane topology authority

- `LaneId`と`BoundaryId`は、segmentが参照する断面template内のstable identityである。横方向の配列indexはidentityではない。
- lane endpointは`LaneEndpointKey(segment_id, lane_id, endpoint_role)`、boundary endpointは`BoundaryEndpointKey(segment_id, boundary_id, endpoint_role)`で指定する。
- 確定したlane接続は`LaneConnection`、境界継続は`BoundaryContinuation`として保存する。world位置や近さから保存接続を再構成しない。
- `LaneTravelDirection`はsegment正本方向に対する`AlongSegment` / `AgainstSegment`であり、接続時の方向整合に使う。
- `LaneConnection.source`はsource laneの退出端、`target`はtarget laneの進入端でなければならず、両端は同じ`RoadNodeId`を共有する。
- Continuation / Transitionは一対一、Splitは同一sourceから複数target、Mergeは複数sourceから同一targetを許可する。JunctionMovementは明示された複数movementを許可する。同一source-targetの重複は常にvalidation rejectする。
- 同一surface strip内のlane lateral intervalは重複させない。vector順はidentityに使わず、横方向順序はintervalから検証する。
- 高速道路、出口、合流専用の正本型を追加せず、車線追加、分岐、合流、交差点movementを同じ接続型で表す。
- lane center接続からshoulder、車道端、白線の継続を推測しない。必要なboundary接続は別に明示する。
