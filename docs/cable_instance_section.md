# CableInstance / CableSection

このドキュメントは、追加線・束ね線・光ケーブルのぐるぐる表現を扱うための設計語を固定する。
production の carrier / wrap 生成はまだ実装しない。

## 基本語

`Span` は topology 上の区間であり、見た目上の1本のケーブル identity ではない。
描画、export、hit-test のために `CableInstance` を span 単位へ分割してもよいが、span が cable continuity を決めてはいけない。

`CableInstance` は、logical topology の連続性から派生する、見た目上連続した1本のケーブルまたは線である。
`SavedBackboneGraph` の span ではなく、topology authority でもない。

`CableSection` は `CableInstance` 上の parameter range である。
見た目の振る舞い、trim、wrap direction、splice、support との見え方が変わる範囲を表す。
`CableSection` は topology、connectivity、curve authority を決めない。

`carrier` は、ある section が追従または巻き付く対象 cable / section を表す明示関係である。
汎用名の `CableRef` は使わず、関係名として `carrier` を使う。

## free cable と carried cable

free cable は endpoint、sag、curve policy から centerline を作る。
carried / wrapped section は carrier curve、radius / offset、pitch、phase、direction から centerline を作る。
carried section は独立した sag / catenary input を持たない。

この違いは `CableSection` の curve / visual behavior で表す。
`WrapCable` や `SupportWire` のような新しい中心型は作らない。
support wire は `CableInstance` の role または cable template の性質であり、wrap は section の振る舞いである。

## 実ケーブルと表面detail

実ケーブルが別ケーブルに巻き付く場合は、別の `CableInstance` として扱う。
その `CableSection` は carried / wrap behavior と明示的な `carrier` を持つ。

同じケーブルの表面模様、補助helix、coiled appearance は `CableInstance` を増やさない。
これは visual detail であり、現行の `CableSupplementalPathTemplate::ProfileKind::kCoiledCable` はこの分類に入る。
現在の実装は carrier を持たず、既存の curve 上に offset / coil sample を追加するため、real carrier-following wrap cable ではない。

## carrier ownership

carrier は geometry proximity から推測しない。

禁止:
- nearest cable を carrier にする
- overlap した cable を carrier にする
- 同じ band だから carrier とみなす
- viewer が距離で carrier を選ぶ

将来の production では、template / rule / lane declaration が carrier 候補を示し、generation がそれを派生 carrier relation へ解決する。
現段階では `SavedBackboneGraph`、pairs、topology authority に carrier を保存しない。

## section end behavior

光ケーブルや通信線は support に直接接続しないことがある。
これは topology 分割ではなく、section end behavior として扱う。

例:
- `direct_to_support`
- `trim_before_support`
- `lead_to_support`
- `splice`

`CableTermination` という top-level 型はまだ作らない。

## wrap direction と splice

wrap direction、pitch、phase、splice は `CableSection` ごとに変わり得る。
section boundary は必ずしも span boundary ではない。
途中反転や継ぎ接ぎを表すために topology span を増やしてはいけない。

## 現行コードとの対応

`VisualCurvePartKind::kEdgeBody` と `kNodePatch` は派生 visual curve の分割であり、topology 正本ではない。
`kLead` と `kJumper` は将来の別curve family用の分類であり、main cable continuity を代替しない。

population rule は `CableSectionProfile` で free / wrap を宣言する。`kFree` は追加平行線、
`kWrap` は「同じ span の base cable」を carrier とする巻き付き実線である(A型のspan-local MVP)。
wrap section は carrier の最終 curve から `sample_wrap_helix_points` で centerline を派生し、
独自 sag と band 配置を持たず、`end_trim_m`(= `trim_before_support` の実装)で support 手前に留まり、
node patch へ参加しない。位相は instance index で等分、巻き方向は rule の +1/-1。carrier は
rule 宣言から解決する(近傍探索禁止は上記のまま)。span を跨いで連続する wrap と、別 instance を
carrier とする wrap は run-level identity が入るまで実装しない。
現行 population の runtime 名は `CableInstanceKey` / `CableSectionLayout` に寄せる。
ただし現時点の `CableInstanceKey.logical_span_id` は span-fragment scope であり、run-level identity は未実装である。

`CableContinuityPolicyHint`、NodePatch continuity、span / bundle / lane binding は現在の近い continuity 情報である。
`CableInstance` はこの既存 continuity 結果から派生するべきであり、別レイヤで新しい continuity solver を作らない。
現時点では run-level の安定した `CableInstance` identity は未実装であり、production carrier / wrap の blocker である。

## 禁止する実装

- 追加線を `SavedBackboneGraph` span として保存する
- `CableSection` を第二の topology 正本にする
- span ごとに CableInstance continuity を再推測する
- carrier を座標や近接で探す
- `CableMember`、`CableMemberGroup`、`MemberGroup`、`Group`、`WrapCable`、`SupportWire`、`CableTermination`、`CableRef` を新しい中心語として増やす
- viewer 側で wrap / carrier / continuity を補正する

## production 前の blocker

- run-level の `CableInstance` identity と section range の派生元を固定する
- carrier declaration を template / rule / lane で表す最小形を決める
- carried section が carrier の arc length / frame を使う境界を決める
- run-level identity が入るまでは、`CableInstanceKey.logical_span_id` が span-fragment scope であることを明示し続ける
- existing `kCoiledCable` surface detail と real carrier-following wrap cable を同じ経路で混ぜない
