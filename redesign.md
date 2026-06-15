# Backbone生成パイプライン再設計仕様

## 目的

backbone から span 生成までの流れを、処理順と責務が読める一本の pipeline にする。
後段は upstream が保存した正本を読むだけにし、local geometry や既存 layout から意味を作り直さない。

## 処理順

1. `BackboneBuilder`
2. `JunctionInputBuilder`
3. `JunctionPairResolver`
4. `JunctionLevelResolver`
5. `PoleFacingResolver`
6. `BundleSpanBuilder`
7. `SpanLayoutRuleBuilder`

`BackbonePipeline` は `prepare()` / `check()` / `build()` だけを持つ。

## BackboneGraph

backbone が持つ正本:

* nodes
* edges
* junctions
* `InputDirection`
* `BuildDirection`
* `JunctionPairs`
* `JunctionLevelRules`

backbone が持たないもの:

* final pole facing
* lane order
* final port order
* socket の resolved 結果
* materialization 後の world geometry

順序番号を正本にしない。downstream は local geometry から正方向を再生成しない。

## JunctionInputBuilder

`JunctionInputFacts` は入力から観測できる事実だけを持つ。

入れてよいもの:

* node id
* route incidents
* external incidents
* incident directions
* build direction
* explicit user constraints
* support node geometry

入れないもの:

* 既存 span
* 既存 layout
* 既存 seed

禁止:

* through pair / cross pair を選ぶ
* junction axis を選ぶ
* level rule / lowering を決める

## JunctionPairResolver

`JunctionInputFacts` から、生成で使う pair と axis を選ぶ。

出力:

* `JunctionPairs`
* `through_pair_by_node`
* `cross_pair_by_node`
* `junction_axis_by_node`

`main / branch / cross / corner` のようなラベルは正本ではない。必要なら debug 表示として派生させる。

禁止:

* level rule / lowering を決める
* pole facing を決める
* grouped span 都合で pair を足す
* bundle spec で pair / axis を変える
* 既存 span から pair を借りる

## JunctionLevelResolver

pair / axis 選択後に、level rule を決める。

入力:

* `JunctionInputFacts`
* `JunctionPairs`
* bundle spec

出力:

* `JunctionLevelRules`
* `same_level_allowed`
* `must_lower`
* `level_rule`

level rule は facts ではなく生成ルールとして扱う。

禁止:

* through pair / cross pair / junction axis を変える
* 既存 layout / seed で lowering を変える

## PoleFacingResolver

pole facing は従属値。

入力:

* external override
* `JunctionPairs`
* `junction_axis_by_node`
* `BuildDirection`

禁止:

* pole facing から pair / lane / side / lowering を決める
* bundle category 分岐を持つ
* support axis selection を主導する
* grouped span 側の不足を埋める

## BundleSpanBuilder

grouped span は consumer。

入力:

* backbone
* `JunctionPairs`
* `JunctionLevelRules`
* build direction
* node side axis

責務:

* port row を作る
* span を作る
* endpoint layout rule を保存する

禁止:

* pair を発明する
* side axis を local geometry から作る
* sign を補完する
* existing seed を読んで意味を変える

## 既存物の扱い

既存 pole / bundle / span / port は再利用制約であり、意味決定の正本ではない。

許可:

* 既存 pole の再利用
* 既存 bundle/span の不足分判定
* 既存 port の再利用
* 明示 override の尊重

禁止:

* 既存 span で through pair / cross pair を変える
* 既存 layout で side axis を変える
* 既存 seed で lowering を変える
* 既存有無で同じ input backbone の意味結果が変わる

## 命名

採用する中心名:

* `BackbonePipeline`
* `BackboneBuilder`
* `JunctionInputBuilder`
* `JunctionPairResolver`
* `JunctionLevelResolver`
* `PoleFacingResolver`
* `BundleSpanBuilder`
* `SpanLayoutRuleBuilder`
* `JunctionInputFacts`
* `JunctionPairs`
* `JunctionLevelRules`
* `PoleFacing`
* `EndpointLayoutRule`
* `SpanLayoutRules`

避ける中心名:

* `Authority`
* `DecisionSeed`
* `ProjectionView`
* `Materialization`
* `Commit`
* `Execute`
* `Planner`
* `Policy`

## Acceptance

* 既存 span なし/ありで同じ input backbone の `through_pair`, `cross_pair`, `junction_axis` が一致する。
* bundle spec を変えても `through_pair`, `cross_pair`, `junction_axis` は変わらない。
* bundle spec によって変わるのは `JunctionLevelRules` と span/layout 結果だけ。
* pole facing override を変えても pair / axis / level rule / lane / side sign が変わらない。
* `SpanLayoutRules` 保存後、後段は保存済み rule を consume するだけにする。

## bb2 milestone 1

`bb2` は v1 を整理する場所ではなく、v1 を読まない小さい新本流として扱う。

milestone 1 の対応入力:

* `path.polyline.size() >= 2`
* `bundles.size() == 1`
* pole support のみ
* existing span / layout / seed を意味決定に使わない

milestone 1 の必須出力:

* topology: poles / bundle / ports / spans
* rules: `SpanLayoutRules`
* layout: `SpanSupportLayoutEntry`
* geom: `DetailCurve` / `BoundsCacheEntry`

milestone 1 の非保証:

* visual support parts
* render styling
* attachment
* insulator
* grouped lowered support visual
* style context
* post-edit refresh

cache の扱い:

* rules / layout は bb2 生成結果の一部。
* curve / bounds は生成直後に保存される deterministic derived output。
* visual / render は draw 要件が固まるまで生成しない。

viewer 表示要件:

* milestone 1 では curve があれば wire 表示可能で十分。
* render cache が無い場合の viewer fallback は許可する。
* support arm / insulator が無いことは milestone 1 の欠落ではない。

recalc 不使用:

* bb2 generation 中は recalc / Commit / dirty queue を使わない。
* post-edit 再派生は milestone 1 の対象外。
* 後で必要なら bb2 専用の明示再導出 API を別途作る。

## bb2 milestone 2

pair は初期正本として `graph` から一度だけ確定する。T / cross / branch の kind label は作らず、node-local な incident continuity として扱う。

入力構造:

* `node`: input/build direction 適用後の support point。
* `link`: node 間の入力 edge。`dir` は `a -> b` の有向進行方向。

pair 正本:

* `pairs make(graph)` が唯一の確定点。
* `pair`: 同じ route 上で連続する incoming / outgoing link の continuity。
* `open`: terminal など、接続相手を持たない incident。
* `row`: port placement unit。必ず pair または open を source に持つ。

下流の扱い:

* `emit` / `rules` / `layout` / `geom` は `pairs` を読む。
* row axis は `pairs make` の出力を正とし、downstream は route geometry から作り直さない。
* `topo` は生成実体だけを持ち、pair/open の組み合わせを解釈しない。

禁止:

* T / cross / branch の enum や kind label を bb2 に作る。
* pair / open / row の数で rules / layout / geom に分岐を散らす。
* existing span / layout / seed / bundle spec / pole facing で pair を変える。
* zero length link や ambiguous incident を fallback で補う。

## bb2 milestone 3

existing pole node は新規 pole を作るかどうかだけを変える。pair / open / row の意味決定には使わない。

node:

* `pole`: existing pole id。未指定なら invalid。
* `is_new`: bb2 が pole を新規作成するなら true。
* `pos`: existing pole なら pole position、new node なら input point。

対応:

* `node_specs.node_id` で既存 pole を指定できる。
* existing pole node では `AddPole` せず、`topo.poles` に既存 id を入れる。
* `generated_pole_ids` は新規作成 pole だけを返す。

禁止:

* existing span / port / layout / seed / recalc state を pair 決定に使う。
* existing pole 周辺の接続状況で row axis を変える。
* existing port resolution をこの milestone に混ぜる。

## bb2 milestone 4

multiple bundle は同じ `graph` / `pairs` / `row` を共有する。bundle spec は topology emission 以降でだけ使い、pair / axis を変えない。

対応:

* `bundles.size() >= 1`
* bundle ごとに bundle entity / ports / spans を生成する。
* `topo.spans` は `link`, `bundle`, `lane` を持ち、rules はこの metadata から endpoint を読む。

禁止:

* bundle spec / bundle count / bundle kind を `pairs make(graph)` で読む。
* bundle ごとに pair / row axis を作り直す。
* multiple bundle を T / cross / branch kind の代替分岐にする。

## bb2 milestone 5

port height は pole type の `PortPlacementBand` から読む。固定高さを使わず、missing band は unsupported とする。

対応:

* new pole は生成後に適用された actual pole type の band を使う。
* existing pole は request pole type ではなく、その pole の actual pole type の band を使う。
* bundle category と resolved layer に一致する enabled band のうち、priority / band id 順の先頭を使う。

禁止:

* band が無いときに固定値や geometry で高さを補う。
* band の lateral center を row axis と混ぜる。
* height / pole type / bundle template を `pairs make(graph)` で読む。

## bb2 milestone 6

`constraints.lateral_offset_m` は port placement の追加 offset としてだけ扱う。新しい `levels` 層は作らず、空の予約型も残さない。

対応:

* `lateral_offset_m` を `row.axis` 方向の offset に加える。
* `avoid_points` / `avoid_radius_m` は unsupported のまま。
* layout / curve / bounds は port world position から一方向に追従する。

禁止:

* `lateral_offset_m` を `pairs make(graph)` で読む。
* lateral offset のために row axis や sign resolver を作り直す。
* lowering / same-level / draw をこの milestone に混ぜる。

## bb2 milestone 7

`node_bundle_modes` は `kNotPresent` だけを no-op として受ける。no-op は生成意味に影響しないという意味で、壊れた入力を無視する意味ではない。

対応:

* `point_index` は input path point index として範囲チェックだけ行う。
* `bundle_template_id` は request 内の bundle を参照している必要がある。
* `mode == kNotPresent` は許可し、生成結果へ反映しない。

禁止:

* `kPassThrough` を受ける。
* `node_bundle_modes` を `pairs make(graph)` で読む。
* node mode から pair / row / port / rule を変える。

## bb2 milestone 8

`topo.rows` は generated port の置き場だけにしない。`pairs.rows` で確定した row id / node / source / axis をそのまま運び、後段が row の意味を推測しない形にする。

対応:

* `trow` は row id / node / source / axis / pole / ports を持つ。
* `emit_ports` は `ps.rows` から row id / node / source / axis をコピーする。
* row ordering と port placement は現状維持する。

禁止:

* `topo.rows` 側で source を推測する。
* `emit_spans` / rules / layout / geom で pair/open/source を再解釈する。
* pairs rename / link row mapping 分離 / midair / pass-through / avoid / draw をこの milestone に混ぜる。

## bb2 milestone 9

`rules` は `pairs.links` を読まない。`emit_spans` が span endpoint の row index を `tspan` に保存し、`rules` は `topo` だけを読む。

対応:

* `tspan` は `arow` / `brow` を持つ。
* `tspan.arow` / `tspan.brow` は `topo.rows` を引くための row index として扱う。
* 現時点では `ps.rows[].id == topo.rows index` 前提を維持する。
* `rules make` は `make(const topo&)` とする。

禁止:

* `rules make` が `pairs` を受け取る。
* `rules make` が `ps.links` から endpoint row を読む。
* row id/index 分離、pairs rename、link row mapping 分離をこの milestone に混ぜる。

## bb2 milestone 10

`SavedBackboneGraph` を bb2 の長期正本として追加する。Pole / Port / Span / Bundle は生成結果であり、接続正本は saved graph が持つ。

対応:

* authoritative state は saved backbone nodes / edges / edge bundles を持つ。
* node は node id / pole id / position を持つ。
* edge は edge id / node_a / node_b / route / order / dir を持つ。
* edge bundle は edge id / bundle id / traversal / span ids を持つ。
* runtime index は `node_edges`, `edge_bundles`, `edge_bundle_spans`, `pole_node`, `span_edge_bundle` を持つ。
* bb2 は generated spans を saved edge bundle に bind する。

禁止:

* M10 で `BuildBackboneEdges()` を置き換える。
* M10 で branch / cross / lowering / affected frontier を入れる。
* v1 由来 scene を saved graph へ migration する。

## bb2 milestone 11

saved graph を使った変更対象収集の入口を追加する。全 span scan や dirty flag 収集ではなく、runtime index から pole/span 周辺を O(incident) で読む。

対応:

* `BackboneFrontier` は node ids / edge ids / span ids / pole ids を持つ。
* `pole_frontier(pole_id)` は `pole_node -> node_edges -> edge_bundles -> edge_bundle_spans` を読む。
* `span_frontier(span_id)` は `span_edge_bundle -> edge_bundles -> edge_bundle_spans` を読む。
* A-B-C に B-D を追加した場合、B frontier は B に接続する saved edges/spans を返す。

禁止:

* frontier 収集で `BuildBackboneEdges()` を使う。
* frontier 収集で `Span -> Port -> Pole` から backbone を復元する。
* M11 で branch / cross / lowering / graph migration / BuildBackboneEdges 置換へ進む。

## bb2 milestone 12

same pole pair の saved edge を重複作成しない。edge は物理 segment、edge bundle はその segment に載る bundle、span は edge bundle から生成された結果として扱う。

対応:

* `SavedBackboneGraph` は `nodes` / `edges` / `edge_bundles` を持つ。
* `SavedBackboneEdgeBundle` は edge id / bundle id / edge_forward / route / order / dir / span ids を持つ。
* `BackboneIndex.edge_by_nodes` は same node pair の saved edge resolution に使う。
* saved edge の route / order / dir は初回作成時の値として固定し、resolution 時に上書きしない。
* `pole_frontier` / `span_frontier` は edge bundle 経由で bundle ids と span ids を読む。

禁止:

* same node pair の追加生成で saved edge を重複作成する。
* reverse 生成で saved edge の dir / route / order を上書きする。
* span を backbone edge 直下の主正本として扱う。
* M12 で multiple physical edge per node pair / branch / cross / lowering を扱う。

## bb2 milestone 13

bb2 layout 保存は旧 support-layout/recalc 経路を通らない。`cache_span_layout` は direct layout store として扱う。

対応:

* `cache_span_layout` は `span_layout_cache.store_layout` だけを呼ぶ。
* `cache_span_support_layout` は v1/recalc 用として残す。
* bb2 は `cache_span_support_layout` を呼ばない。
* direct layout 保存は authority / seed / support group rebuild を作らない。

禁止:

* bb2 layout 保存で `cache_span_support_layout` へ委譲する。
* bb2 layout 保存で lowered support group rebuild を走らせる。
* bb2 source に `support_layout_` entrypoint を戻す。

## bb2 milestone 14

bb2 は saved graph 内部 vector を直接読まない。saved graph の mutation/read-for-save は `CoreState` が owner になる。

対応:

* `save_backbone_edge` は `SavedBackboneEdgeRef` を返す。
* `SavedBackboneEdgeRef` は edge id / saved endpoint nodes / created を持つ。
* edge resolution 時も saved edge の endpoint nodes を返す。
* bb2 は `SavedBackboneEdgeRef` から `edge_forward` を決める。

禁止:

* bb2 source で `authoritative_.backbone` を読む。
* bb2 source で saved edge vector を探索する。
* edge save API 境界変更で `edge_spans/span_edge` 互換 index を戻す。

## bb2 milestone 15

bb2 layout 層は旧 `SpanSupportLayoutEntry` / `SupportLayout*` 型名を使わない。M13 の direct 保存口を維持し、bb2 が作る layout は neutral な `SpanLayoutEntry` 系へ寄せる。

対応:

* `SpanLayoutEntry` / `LayoutEndpoint` / `LayoutSemantic` / `LayoutOriginKind` / `LayoutEndpointSourceKind` を neutral 型名として使う。
* `bb2::layout` は `SpanLayoutEntry` を持つ。
* `CoreState::cache_span_layout` は `SpanLayoutEntry` を受け、direct store だけを行う。
* v1/recalc 用の `SpanSupportLayoutEntry` / `SupportLayout*` 名は core 内に残す。

禁止:

* bb2 source に `SpanSupportLayoutEntry` / `SupportLayoutEndpoint` / `SupportLayoutSemanticDecision` / `SupportLayoutOriginKind` / `SupportLayoutEndpointSourceKind` を戻す。
* `cache_span_layout` から `cache_span_support_layout` へ委譲する。
* layout 型名変更と同時に draw / lowering / branch / cross を入れる。

## bb2 milestone 16

bb2 の layout 読み出しは `span_layout` を正規 API にする。`support_layout_projection` / `support_layout_contract` は v1/recalc/inspection 用に残すが、bb2 acceptance の layout 存在確認には使わない。

対応:

* `SpanLayoutView` を追加する。
* `CoreState::span_layout` / `CoreView::span_layout` は保存済み `SpanLayoutEntry` だけを読む。
* bb2 tests の layout 存在確認は `span_layout(span).has_layout()` を使う。
* no-authority 確認だけは一時的に `support_layout_contract` を観測口として残す。

禁止:

* `span_layout` accessor が authority / contract / seed を読む。
* bb2 acceptance が layout 存在確認に `support_layout_projection` を使う。
* M16 で `SupportLayoutCache` の全面 rename や inspection API rename を始める。

## bb2 milestone 17

bb2 acceptance は旧 support layout contract を観測しない。layout 状態の確認は `span_layout_state` を正規 API にする。

対応:

* `SpanLayoutState` は `has_rules` / `has_layout` / `input_required` だけを持つ。
* `CoreState::span_layout_state` / `CoreView::span_layout_state` は保存済み rules/layout と追加入力要求だけを読む。
* bb2 tests は `support_layout_contract` を呼ばず、`span_layout_state(span).input_required == false` を見る。

禁止:

* `SpanLayoutState` に authority / seed / contract 名を入れる。
* `span_layout_state` accessor が seed store path や contract view を読む。
* M17 で v1/recalc 用 `support_layout_contract` を削除する。

## bb2 milestone 18

bb2 の layout cache owner 名は `span_layout_cache` を正にする。neutral API は `SpanLayoutRules` / `SpanLayoutEntry` / `SpanLayoutState` の保存先として読む。

対応:

* `CacheState` は `SpanLayoutCache span_layout_cache` を持つ。
* `CoreState::span_layout` / `span_layout_state` / `span_layout_rules` は `span_layout_cache` を読む。
* `CoreState::cache_span_layout` / `cache_span_rules` は `span_layout_cache` へ保存する。
* v1/recalc 用の support layout API は残すが、同じ `span_layout_cache` を保存先として使う。

禁止:

* `CacheState` に `support_layout_cache` field を戻す。
* bb2 source に `support_layout_cache` / `support_layout_projection` / `support_layout_contract` を出す。
* M18 で support group / seed / authority 内部構造の全面 rename を始める。

## bb2 milestone 19

bb2 は existing pole に新規 route を接続するとき、saved backbone graph を context として読む。context link は pair/open/row 決定にだけ使い、生成対象にはしない。

対応:

* `graph` は今回入力 route link と saved graph context link を持つ。
* `link.is_new=true` は今回生成対象、`is_new=false` は context とする。
* `CoreView::backbone_node` / `backbone_edge` / `backbone_node_for_pole` を graph 読み出し口にする。
* `pairs make(graph)` は new/context をまとめて読み、same route + consecutive order を pair、残りを open にする。
* `emit_spans` は `is_new` link だけを生成する。

禁止:

* bb2 が `authoritative_.backbone` を直接読む。
* context link から port/span/rules/layout/geom を生成する。
* T/cross/branch kind enum/label を追加する。
* existing span/layout/seed を意味決定に使う。

## bb2 milestone 20

same saved edge + bundle の低レベル保存口は同じ `SavedBackboneEdgeBundle` を返す。span は edge_bundle の生成結果として追加する。

対応:

* `bind_backbone_bundle` は `edge_id + bundle_id` で既存 edge_bundle を再利用する。
* 既存 edge_bundle の `edge_forward` / route / order / dir は初回値のまま保持する。
* 新規生成された span は既存 edge_bundle の `span_ids` に bind する。
* different bundle は同じ edge 上でも別 edge_bundle にする。

禁止:

* same edge + bundle で edge_bundle を重複作成する。
* reverse 生成で edge_bundle metadata を上書きする。
* existing port/span resolution、duplicate span 抑制、lowering、pass-through、draw を M20 に混ぜる。

## bb2 milestone 21

bb2 generation request としての duplicate same edge + bundle は no-op/resolution ではなく unsupported とする。低レベル保存口の idempotent bind は残すが、bb2 は emit 前に duplicate を検出して止める。

対応:

* duplicate 判定は saved graph の `edge_bundle` 単位で行う。
* 同じ saved edge + bundle template を再生成しようとしたら unsupported にする。
* different bundle は同じ saved edge 上でも許可する。
* reject は emit 前に行い、pole/port/bundle/span/rules/layout/geom を増やさない。

禁止:

* existing port/span resolution を M21 に混ぜる。
* existing span/layout/seed から duplicate や不足分を判定する。
* lane 単位の差分追加、bundle template 差分更新、geometry 近似比較を行う。
* v1/recalc/materialization を duplicate 判定に使う。

## bb2 milestone 22

bb2 は connectivity row と生成された port object の対応を saved backbone graph に保存する。これは existing port resolution の実装ではなく、将来 resolution を座標推測ではなく saved identity で行うための準備である。

対応:

* `SavedBackbonePortBinding` を保存する。
* binding は `edge_bundle_id + row_key + lane_index -> port_id` を持つ。
* `row_key` は saved node id と row source の saved edge id から作る。
* context-only row には port binding を作らない。
* 同じ `edge_bundle_id + row_key + lane_index` の binding は duplicate として拒否する。

禁止:

* existing port resolution を M22 に混ぜる。
* port 位置近似で同一性を判定する。
* existing span/layout/seed を binding 判定に使う。
* row id/index の全面整理や pairs rename を M22 に混ぜる。

## bb2 milestone 23

bb2 は saved row-port binding と compatible scope が一致する場合だけ existing port id を resolve する。port resolution は port identity の問題であり、pair/open/row/rules/layout/geom の意味決定は変えない。

対応:

* `row_key + lane_index + bundle_template + port kind + port layer` で compatible な saved port binding を探す。
* binding の port が存在し、同じ pole に属し、同じ port kind/layer の場合だけ resolve する。
* binding が無ければ従来通り new port を作る。
* 複数 binding が同じ port を指す場合は compatible scope が一致する場合だけ同一 identity として扱う。
* 複数 binding が別 port に割れる場合、または同一 port に incompatible scope を cross-bind しようとした場合は unsupported にする。
* resolved port は new span endpoint として使う。

禁止:

* existing span resolution を M23 に混ぜる。
* 別 bundle/category/layer の port を `row_key + lane_index` だけで共有する。
* port 位置や layout から row を復元する。
* support materialization / recalc / seed を resolution 判定に使う。
* lowering / pass-through / draw / avoid を M23 に混ぜる。

## bb2 milestone 24

bb2 は branch/cross 相当の existing junction context で row が重ならないよう、port placement だけに row separation を入れる。T/cross/branch kind enum は作らない。

対応:

* `pairs make(graph)` は今まで通り pair/open/row を一度だけ決める。
* node ごとの connectivity rows 全体から deterministic な row order を作る。
* context-only row も row order には含める。
* offset を適用するのは emitted spans が参照する materialized rows だけにする。
* layout/geom は port 位置の変化に自然追従する。

禁止:

* lowering / pass-through / draw を M24 に混ぜる。
* branch/cross kind enum または label を作る。
* geometry から junction kind を推測する。
* existing span/layout/seed を row separation に使う。
* context-only row の port/span を生成する。

## bb2 milestone 25

bb2 は pass-through node mode と lowering intent を rules/layout に保存する。実際の lowered geometry、support visual、draw/render はまだ作らない。

対応:

* `BundleNodeMode::kPassThrough` は saved junction context のある existing node に限定して受ける。
* kPassThrough は pair/open/row の決定には使わない。
* pass-through 対象 row は generated route 上の active row から一意に決める。
* 対象 row が一意でなければ unsupported にする。
* intent は `SpanLayoutRule` / `SpanLayoutEntry` の pass/lower fields に保存する。
* layout/geom はこれまで通り direct output を作り、lowered shape は作らない。

禁止:

* actual lowered curve geometry を M25 に混ぜる。
* support arm / insulator / attachment visual を作る。
* draw/render cache を触る。
* branch/cross kind enum を作る。
* existing span/layout/seed や v1/recalc/materialization を intent 決定に使う。

## bb2 junction v1 vertical slice

bb2 は existing junction を saved graph context として読み、port identity scope と minimal lowering geometry まで縦に通す。M21-M25 の境界は維持し、小さい整理 milestone は増やさない。

対応:

* saved graph は topology authority とし、context link は saved graph incident edge から作る。
* existing span/layout/seed から topology、row、level、port identity を復元しない。
* port binding resolution は `row_key + lane_index` だけではなく、bundle template / port kind / port layer compatibility を必要条件にする。
* 同じ port に複数 binding を許す場合は runtime index が全 binding を返し、incompatible scope の cross-binding は unsupported にする。
* duplicate same edge + bundle は unsupported とし、span geometry 比較や既存 span 差分補完はしない。
* pair/open/row は `pairs make(graph)` で一度だけ決め、kPassThrough / lowering / row separation は pair/open を変えない。
* row separation と lowering は placement/layout/geom の話として扱い、topology authority にはしない。
* lowering V1 は rules/layout に intent を保存し、layout endpoint / curve / bounds へ deterministic vertical offset を反映する。
* context-only row は ordering/intent の入力になり得るが、port/span は materialize しない。

禁止:

* T/cross/branch enum または kind label を追加する。
* v1/recalc/materialization を bb2 generation 中に読む。
* draw/render/support arm/insulator/attachment visual をこの slice に混ぜる。
* topology を Span / layout / seed から再構成する。

## bb2 gate 3 fix

context link は判断入力であり、save target ではない。`save_graph` でも new link と context link を分離する。

対応:

* `edge.is_new=true` の link だけ `save_backbone_edge` を呼ぶ。
* `edge.is_new=false` の context link は `edge.saved` から既存 saved edge ref を読むだけにする。
* context link の `edge.saved` が invalid なら unsupported とする。

禁止:

* context link の saved edge を node pair / pole pair / geometry / span / layout / seed から推測する。
* context link に対して `save_backbone_edge` を呼ぶ。

## bb2 milestone 26

bb2 は same edge bundle + lane の span binding を saved backbone graph で一意に扱う。duplicate request は no-op ではなく unsupported とし、span を geometry や layout から探さない。

対応:

* `SavedBackboneSpanBinding` を保存する。
* binding は `edge_bundle_id + lane_index -> span_id` を持つ。
* `bind_backbone_span` は同じ `edge_bundle_id + lane_index` の duplicate を拒否する。
* bb2 `save_graph` は `span.lane` を渡して span binding を作る。
* 既存 `edge_bundle.span_ids` は frontier 用の span list として残すが、duplicate 判定の正本にはしない。

禁止:

* span を curve 類似、port 位置、layout、seed、materialization result から探す。
* existing span から不足 lane を補完する。
* duplicate span binding を黙って no-op にする。

## bb2 milestone 27

bb2 は lowering V1 の placement authority として support group を持つ。support group は pair/open/row を決めず、intent で lower_required になった materialized row の配置だけを決める。

対応:

* `groups` は `intent` の後に作る。
* group は `row_members`, `group_axis`, `vertical_order`, `lower_offset_m` だけを持つ。
* `rules` は group を読み、lowering flag / support_group_id / lower offset を保存する。
* `layout` は rule に保存された group offset を consume して endpoint / support world を下げる。
* `geom` は layout endpoint を読むだけで curve / bounds に反映する。

禁止:

* support group が topology / pair / open / row を決める。
* mount / tip / arm / insulator / attachment など visual 寄りの語を group に入れる。
* draw/render/support visual を M27 に混ぜる。
* existing span/layout/seed から support group を作る。

## bb2 milestone 28

bb2 は layout/geom から最小 draw output を保存する。draw は topology、pair/open/row、lowering intent を新規判断せず、既に保存された layout/geom を読む consumer とする。

対応:

* `draw` は `layout` と `geom` の後に作る。
* render cache は `DetailCurve` の sample points から距離配列を作る。
* visual cache は `SpanLayoutEntry` の lowered endpoint から最小 placeholder を作る。
* `cache_span_visual` / `cache_span_render` は direct cache store として使う。

禁止:

* draw が saved graph / pair/open/row / node mode / support group を再判断する。
* draw が v1/recalc rebuild 経路を呼ぶ。
* geometry 比較や position proximity から support visual を推測する。
* support arm / insulator / attachment の本格仕様を M28 に混ぜる。
