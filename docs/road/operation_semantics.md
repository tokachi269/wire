# Road operation semantics (P0-P2 prototype scenarios)

`city::road` の supported scenario を操作と状態の組で固定する。
表にない組み合わせは推測せず `unsupported` とする。
この表のscenario通過はarchitecture migration完了を意味しない。各public operationの実装契約は次で固定する。

```text
Request -> Preflight -> OperationPlan -> trial Apply -> single Build -> Validate -> Commit
```

失敗時はauthoritative bytes、derived hash、next ID、inspection / query結果を一切変更しない。
public operationから別public operationを呼ばない。

Preflightはrequestの形式、ID存在、有限性、明白な値域と、planを作るために必要な局所構造だけを検証する。
接続角度、endpoint section互換、branch / junction approach上限、接続setback、split後を含むtrial topologyの成立性は
trial Buildの`connections`以降が一度だけ決める。operationは同じgeometry / section policyを
再実装せず、trial Buildの`kUnsupported`を操作結果として返す。

## Public operation preflight audit

| Operation | Request field | Preflight validation | Build decision |
|---|---|---|---|
| AddSegment | alignment | finite、連続、非ゼロ、NormalizeRoadPath可能 | self-intersection、junction互換 |
| AddSegment | section_template | ID exists | endpoint section互換 |
| ExtendSegment | segment_id / endpoint_node_id | ID exists、endpoint node is segment endpoint、degree-one | prepend station-owned state unsupported、trial connection |
| ExtendSegment | extension | finite、連続、非ゼロ、endpointへ補正可能 | normalized shape対応 |
| ExtendSegment | section_template | existing segmentと一致 | 異断面延長はunsupported |
| AddSegmentConnectedTo | alignment | finite、連続、非ゼロ、start nodeへ補正可能 | 接続角、degree、setback |
| AddSegmentConnectedTo | section_template / start_node | ID exists | endpoint section互換 |
| AddSegmentConnectedToSegment | alignment | finite、連続、非ゼロ、明示station点と一致 | split後topology、junction互換 |
| AddSegmentConnectedToSegment | start_segment / station_m / section_template | ID exists、station finite、station interior | transition付きsplit unsupported |
| EditSegmentShape | segment_id / shape | ID exists、全handle/knot finite | 接続後shapeのBuild可否 |
| MoveNode | node_id / position | ID exists、position finite | incident segment / connection再導出 |
| DeleteSegment | segment_id | ID exists | 不要transition / marking / policy除去 |
| SetApproachSetbackOverride | ApproachKey / setback_m | node exists、segment exists、endpoint role matches、finite、non-negative、layout target exists | gate overlap、segment length、junction quality |
| SetApproachLateralShiftOverride | ApproachKey / lateral_shift_m | node exists、segment exists、endpoint role matches、finite、layout target exists | self-intersection、junction quality |
| ResetApproachOverrideField | ApproachKey / field | node exists、segment exists、endpoint role matches、field enum valid | resolved layout rebuild |
| ResetAllApproachOverrides | ApproachKey | node exists、segment exists、endpoint role matches | resolved layout rebuild |
| AddSectionTemplate | section_template | ID一意、band/boundary ID一意、width正、finite、enum valid、known SurfaceStyleId | trial section evaluation |
| EditSectionTemplate | section_template | ID exists、band/boundary ID一意、width正、finite、enum valid、known SurfaceStyleId | 既存segment再評価 |
| AddTransition | from/to/start/end/anchor/rules | template exists、station finite/range、rule element exists、enum valid | element出現/消滅action対応 |
| AddTransitionToSegment | segment_id + transition request | segment exists、transition preflight | 既存transition replace、segment section整合 |
| AttachSectionTransition | segment_id / transition_id | ID exists、from_template matches segment、station range valid | segment再評価 |
| AddManualLine | owner_segment_id / path / style_id | owner exists、path finite、owner-local station範囲、known MarkingStyleId | owner section上への投影 |
| AddManualArea | owner_segment_id / frame_origin / width / length / style_id | owner exists、finite、width/length正、station範囲、known MarkingStyleId | owner section上への投影 |
| SetBoundaryMarkingPolicy | section_template_id / boundary_id / policy | template exists、boundary exists、role enum valid、known MarkingStyleId | 要求統合、継続再評価 |
| SetLaneSideMarkingPolicy | section_template_id / band_element_id / side / policy | template exists、band exists、carriageway band、role enum valid、known MarkingStyleId | 隣接boundaryへの解決、要求統合 |
| ResetLaneSideMarkingPolicy | section_template_id / band_element_id / side | 同上(policyは無効値) | 要求統合、継続再評価 |
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
| ExtendSegment | validation | degree 1終端かつ同一断面ならsupported | 対象終端がdegree 1ならsupported | appendのみsupported、prependはunsupported | appendのみsupported、prependはunsupported |
| AddSegmentConnectedTo | validation | gate断面ID一致のみsupported | gate断面ID一致かつdegree 4までsupported | 終端gateがto断面ならsupported | gate断面ID一致のみsupported |
| AddSegmentConnectedToSegment | validation | 同一断面かつ明示`segment_id + station_m`ならsupported | 同一断面かつ明示`segment_id + station_m`ならsupported | unsupported | 同一断面かつ明示`segment_id + station_m`ならsupported |
| EditSegmentShape | validation | supported | node位置を変えずshapeだけ変更 | StationRef規則で再評価 | owner-local markingを追従 |
| MoveNode | validation | endpoint nodeを移動 | 接続全segmentを再導出 | StationRef規則で再評価 | owner-local markingを追従 |
| DeleteSegment | validation | supported | supported (不要junctionを除去) | transition参照を除去 | owned markingを除去 |
| Add/EditSectionTemplate | supported | supported | supported | supported | supported |
| AttachSectionTransition | validation | supported | supported | replace supported | supported |
| AddManualLine / AddManualArea | validation | supported | supported | supported | supported |

## P1 node semantics

- 同じ道路形状・接続関係を表す操作は、描画の確定回数にかかわらず同じ正規化結果になる。
  `A -> B -> C`を一度に渡す場合と、`A -> B`を確定後にdegree 1終端から`B -> C`へ延長する場合は、
  authoritative bytes、CanonicalAlignment、断面、mesh、marking、derived observationが一致する。
- degree 1終端から同じsectionの道路を伸ばす既定操作は`ExtendSegment`とする。既存segment IDと終端node IDを維持し、
  終端nodeを新位置へ移動して旧終端を`SegmentShape.internal_knots`へ取り込む。操作確定は道路構造の境界にしない。
- `ExtendSegment`は対象nodeが対象segmentの終端でdegree 1、sectionが同一、node policy overrideがない場合だけsupportedとする。
  branch、segment途中への接続、異なるsection、明示的な独立接続は新segmentを作る。
- node Bを内部knotへ吸収するときは、前後spanを共通のpath normalizationへ渡す。line / Bezierや一括 / 逐次で
  別のcorner規則を持たず、同じ入力Pathに正規化してから`SegmentShape`へ変換する。
- node_b側appendは既存のowner-local stationを維持する。node_a側prependは既存stationの意味を移動させるため、
  transitionまたはmanual markingを持つsegmentではunsupportedとする。
- 逆方向入力はID、segment向き、station原点が異なるためraw serialization一致を要求しない。
  CanonicalAlignmentを同じ向きへ正規化した形状、material別mesh頂点集合、marking形状の一致を要求する。
- segment途中への接続は`target_segment_id + station_m`を入力とし、alignmentを該当stationで分割する。座標近接から対象segmentやstationを再推測しない。
- Line / Bezierは入力toolの区別に限定し、正本はendpointを含まない`SegmentShape`、完全Pathは派生`CanonicalAlignment`とする。曲線segmentの分割はDe Casteljau分割を使う。
- degree 2 の自動decisionはPassThroughまたはCorner。対向する2本は幅を増やさず連続し、屈曲する2本は同じ断面を保った曲線connectorを派生する。停止線・ゼブラは生成しない。
- degree 3/4 の自動decisionは対応範囲ならJunction。自動junctionの存在は保存しない。
- ユーザーが明示した場合だけ`NodeConnectionPolicyOverride`を保存する。overrideを削除するとAutoへ戻る。
- 実交差点の各gate位置は固定距離にしない。接続角と各approachの断面幅から、approach同士が重ならないsetbackを一度だけ決定する。
- 実交差点はcarriagewayだけでなく、gate断面のsidewalkとcurbを隣接approach間へ接続する。
- 交差点cornerとdegree 2の屈曲connectorは、gate接線を共有するBezier形状として派生する。segment側とconnector側で接線を再解釈しない。
- 自動停止線・ゼブラは実交差点だけに生成する。向きと高さはgate frameおよびSectionEvaluationの横断勾配から導出する。

## Simple path

- 単独segmentとdegree 1終端はjunction接続準備を要求しない。
- `ApproachKey`、`NodeConnectionDecision`、`ConnectionGate`はdegree 2以上の明示接続nodeだけに生成する。
- `AutoNodeLayout` / `ResolvedNodeLayout`はdegree 2以上のlayout対象だけに生成する。degree 1 endpoint overrideはunsupported。
- segment内部の通常曲線は`CanonicalAlignment`とSectionEvaluationから直接生成し、junctionsへ渡さない。
- `RoadGraph -> Build`の一回で必要な派生物を生成し、操作履歴や事前tableの有無でstage経路を変えない。

## Approach geometry override

- `ApproachKey = node_id + segment_id + endpoint_role`だけをidentityとする。
- setbackはnodeからsegment内部方向への非負距離。startは`station=setback`、endは`station=length-setback`。
- lateral shiftはresolved approach lateral方向を正とする。
- manual fieldだけを`ApproachGeometryOverride`へ保存する。auto setback / auto lateral shiftは保存しない。
- reset fieldで該当manual fieldを消し、全fieldがAutoになったoverride entityは削除する。
- segment削除では該当segment/nodeのoverrideを同じOperationPlanで削除する。
- segment splitでは元segment外側endpointのoverrideだけを新しい外側segmentへID mappingし、内部nodeへ複製しない。

## P2 transition semantics

- `StationRef::FromStart(d)` は `d`、`FromEnd(d)` は `length-d`、`Ratio(u)` は `length*u` に解決する。
- 解決後は `0 <= start < end <= length` を満たす。満たさなければ validation で正本を変更しない。
- transition前は `from_template`、transition後は `to_template`、区間内は線形補間する。
- `anchor` は補間中に固定する断面基準で、Center / LeftEdge / RightEdge のいずれか。
- element対応は ID で行う。出現は `TaperIn`、消滅は `TaperOut` または `EndCap` を明示する。
- 1 segmentに同時に接続できる transition は1個。短距離多重transitionはP2非対象。
- 異なる断面をnodeへ直接接続しない。trial Buildの`connections`が各approachのendpoint section IDを
  一度だけ解決し、全approachで完全一致する場合だけ接続する。operation preflightは断面を再評価しない。

## P2 marking semantics

- boundary marking は SectionEvaluationTable の boundary ID / `AutoMarkingPolicy` を唯一の入力とし、
  drawはsection ruleを読まない。
- ManualLineMarking の Path と ManualAreaMarking の frame は owner segment local `(station, lateral)`。
  ManualAreaMarking は `rotation_rad` を持ち、0 はowner segment station方向を意味する。
- 自動線は MarkingIntent / Continuation / ResolvedMarkingGraph を通ってからmesh化する。
  segment、junction、manual のowner境界はgateとmanual entity IDで決め、位置近接で再bindしない。
- 保存するのはlocal値で、world meshだけを派生する。segment alignment編集時はlocal値を維持して再導出する。
- 境界への線要求は`BoundaryProfile.marking`と、carriageway `SurfaceBand`の`LaneSideMarkingPolicy`だけ。
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
