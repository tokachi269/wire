# Road architecture

この文書は`city::road`の移行先architectureを定義する。現在の`RoadState`と`road.cpp`は
この境界へ未移行の箇所を含む。現行実装を根拠に未定義の意味を補完しない。

共通契約は`../architecture.md`、操作×状態は`operation_semantics.md`、要求範囲は`plan.md`を参照する。

## Existing implementation mapping

| 既存の参照 | roadで採用する契約 | 差異 |
|---|---|---|
| wire `SavedBackboneGraph` | node / segment identityを持つ`SavedRoadGraph` | road alignmentはユーザー編集対象 |
| wire operation semantics | `operation_semantics.md`を実装前の正本にする | road固有状態を定義する |
| wire trial + unified build | road operationもtrial graphから統一buildする | 現在の`RebuildDerived()`直結をstage分割する |
| wire curveの一方向導出 | road alignmentからstation/frame/surfaceを派生する | roadは編集済みalignment自体を正本にできる |
| wire ID connectivity | split/connectはsegment/node IDとstationで指定する | 座標近接による接続推測は禁止 |
| wire authoritative/runtime分離 | saved typesとderived typesを別header/moduleへ分ける | 現在は`road.hpp`に混在している |

## State ownership

### Input

- Straight / Curvedなどのtool mode
- viewport click、drag中handle、preview point
- snap結果の`segment_id + station_m`または`node_id`
- section/marking編集request

tool modeとpreviewは保存しない。adapterはcore operationが必要とするIDと数値を渡し、接続を再判定しない。

### Authoritative

- `RoadNode`: IDと確定world position
- `RoadSegment`: ID、endpoint node ID、canonical alignment、section参照
- section template、section transitionの意味入力
- manual markingのowner IDとowner-local station/lateral値
- ユーザーが明示したjunction policy override

StraightとCurvedを別kindで保存しない。canonical alignmentはcubic Bezier span列へ統一し、
Straight入力はoperation境界で1/3、2/3 handleを持つ直線Bezierへ正規化する。

### Derived

- node degreeとpass-through / corner / junction評価
- split projection、arc-length table、station、tangent、frame
- ConnectionGate、degree 2 connector、JunctionArea
- SectionEvaluation、surface、curb、sidewalk、marking、terrain mask
- mesh、bounds、viewer payload

node degreeから求められるjunctionの存在を保存しない。junction overrideはnode IDへ付くpolicyだけを保存し、
交差点geometryやgateを正本にしない。

## Operations

- AddSegmentはtool inputをcanonical Bezierへ正規化してからgraphへ追加する。
- ConnectToNodeは明示node IDだけを使う。
- SplitAndConnectは`target_segment_id + station_m`を使い、BezierをDe Casteljau分割する。
- EditAlignmentは対象segmentのalignment ownerだけを変更する。
- section、transition、marking操作はowner IDを明示する。
- 各操作はtrial graphへ適用し、統一build成功後にcommitする。

座標が線上に近いことからtarget identityやsplit位置をcoreで再推測しない。viewerのpick座標はpreview用であり、
確定操作にはcoreが評価したstationを使う。

## Build stages

```text
SavedRoadGraph
  -> topology index / validation
  -> canonical alignment / arc-length table
  -> node connection decision
  -> section evaluation
  -> gate / connector / junction geometry
  -> surface / marking / terrain materialization
  -> viewer / export payload
```

node connection decisionは1回だけ行う。後段はdegreeや角度からpass-through / corner / junctionを再判定せず、
決定済みのderived decisionを消費する。policy定数はdecision ownerに置き、mesh builderへ埋め込まない。

## Module boundaries

移行後は少なくとも次の責務を物理分離する。

```text
include/city/road/authoritative_types.hpp
include/city/road/input_types.hpp
include/city/road/derived_types.hpp
src/operations/
src/build/topology/
src/build/alignment/
src/build/section/
src/build/junction/
src/materialization/
src/persistence/
```

ファイル分割自体を目的にせず、authorityを書けるownerとderivedだけを読む層をinclude境界で固定する。

## Migration order

1. operation semanticsをcanonical BezierとID/station入力へ更新する。
2. authoritative/input/derived型を分離する。
3. Line/Bezier正本分岐をcanonical Bezierへ統一する。
4. split/connectをID + station APIへ移す。
5. node connection decisionを単一stageへ寄せ、保存済み`JunctionDefinition`をpolicy overrideへ移行する。
6. section、junction、materializationを決定済みderived inputだけを読む構造へ分ける。
7. persistenceを新authorityのroundtrip契約へ更新する。

移行中もUI操作を変更しない。既存scenarioの制約を抽出して新operation testへ移植し、旧内部表現への期待は持ち込まない。

## Migration status

- 完了: 正本Pathをcubic Bezier span列へ統一し、Straight入力をoperation境界で正規化した。
- 完了: segment途中接続を`target_segment_id + station_m`入力へ変更し、複数spanを含むPathをDe Casteljau分割する。
- 完了: archive version 4はspanだけを保存し、version 1-3のLine / Bezierをload時にcanonical spanへ移行する。
- 未完: authoritative / input / derived型の物理分離、統一build stage、junction policy overrideへの移行。
