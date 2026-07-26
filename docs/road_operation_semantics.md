# Road operation semantics (P0-P2)

`city::road` の supported scenario を操作と状態の組で固定する。
表にない組み合わせは推測せず `unsupported` とする。

## 状態

| 状態 | 意味 |
|---|---|
| empty | segment がない |
| isolated | 接続されていない segment |
| connected | node を共有する同一断面 segment。degree 2 は単純接続、degree 3/4 は実交差点 |
| transitioning | segment が 1 個の SectionTransition を参照する |
| marked | segment が ManualLineMarking / ManualAreaMarking を所有する |

## 操作 x 状態

| 操作 | empty | isolated | connected | transitioning | marked |
|---|---|---|---|---|---|
| AddSegment | supported | supported | supported | supported | supported |
| AddSegmentConnectedTo | validation | gate断面ID一致のみsupported | gate断面ID一致かつdegree 4までsupported | 終端gateがto断面ならsupported | gate断面ID一致のみsupported |
| AddSegmentConnectedToSegment | validation | 同一断面の直線のみsupported | 同一断面の直線のみsupported | unsupported | 同一断面の直線のみsupported |
| EditSegmentPath | validation | supported | 接続nodeを動かさない範囲のみsupported | StationRef規則で再評価 | owner-local markingを追従 |
| DeleteSegment | validation | supported | supported (不要junctionを除去) | transition参照を除去 | owned markingを除去 |
| Add/EditSectionTemplate | supported | supported | supported | supported | supported |
| AttachSectionTransition | validation | supported | supported | replace supported | supported |
| AddManualLine / AddManualArea | validation | supported | supported | supported | supported |

## P1 node semantics

- degree 2 は `JunctionDefinition` を保存しない。対向する2本は幅を増やさずそのまま連続し、屈曲する2本は同じ断面を保った曲線connectorを派生する。停止線・ゼブラは生成しない。
- degree 3/4 だけを実交差点として `JunctionDefinition` を保存する。
- 実交差点の各gate位置は固定距離にしない。接続角と各approachの断面幅から、approach同士が重ならないsetbackを一度だけ決定する。
- 実交差点はcarriagewayだけでなく、gate断面のsidewalkとcurbを隣接approach間へ接続する。
- 交差点cornerとdegree 2の屈曲connectorは、gate接線を共有するBezier形状として派生する。segment側とconnector側で接線を再解釈しない。
- 自動停止線・ゼブラは実交差点だけに生成する。向きと高さはgate frameおよびSectionEvaluationの横断勾配から導出する。

## P2 transition semantics

- `StationRef::FromStart(d)` は `d`、`FromEnd(d)` は `length-d`、`Ratio(u)` は `length*u` に解決する。
- 解決後は `0 <= start < end <= length` を満たす。満たさなければ validation で正本を変更しない。
- transition前は `from_template`、transition後は `to_template`、区間内は線形補間する。
- `anchor` は補間中に固定する断面基準で、Center / LeftEdge / RightEdge のいずれか。
- element対応は ID で行う。出現は `TaperIn`、消滅は `TaperOut` または `EndCap` を明示する。
- 1 segmentに同時に接続できる transition は1個。短距離多重transitionはP2非対象。
- 異なる断面をnodeへ直接接続しない。既存approachの終端gateをtransitionの`to_template`まで評価し、追加segmentの断面IDと一致した場合だけ接続する。

## P2 marking semantics

- boundary marking は SectionEvaluationTable の boundary ID / rule を唯一の入力とする。
- ManualLineMarking の Path と ManualAreaMarking の frame は owner segment local `(station, lateral)`。
- 保存するのはlocal値で、world meshだけを派生する。segment alignment編集時はlocal値を維持して再導出する。
