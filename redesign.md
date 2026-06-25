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
* `avoid_radius_m` は `avoid_points` が空なら no-op として扱う。
* route の単一 segment にだけ交差する 1 avoid point は deterministic detour として扱う。
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

* `BundleNodeMode::kPassThrough` は current route の一意な interior pair、または saved junction context のある existing node に限定して受ける。
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

## bb2 milestone 29

bb2 は saved backbone graph を持たない既存 scene を暗黙 migration しない。既存 pole を `node_specs` で指定する場合、その pole は saved graph node を持っている必要がある。

対応:

* all-new route はこれまで通り bb2 が saved graph を作る。
* bb2 生成済み pole は saved graph node を持つため、追加 route の既存 node として使える。
* saved graph node を持たない既存 pole は unsupported にする。
* migration/import は M29 では実装しない。

禁止:

* span / layout / seed / curve / port 位置から saved graph を推測する。
* existing pole を見つけた時に bb2 がその場で saved graph node を作って取り込む。
* v1 scene を fallback として暗黙に bb2 graph へ変換する。

## M29後 bb2 品質修正

M30 へ進む前に、M29 後に残った境界品質を固定する。

対応:

* lowered layout は `support_world` を port の world position に保つ。
* lowered layout は `endpoint_world` だけを `lower_offset` 分下げる。
* draw placeholder は `LayoutEndpoint.support_world -> endpoint_world` の線分だけを使う。
* draw は `branch_down_offset_m` から support point を復元しない。
* duplicate same edge bundle + lane は emit 前の `check(ps)` で saved span binding を読んで reject する。
* `bind_backbone_span` の duplicate failure は保存時 invariant として残す。

禁止:

* draw が lowering offset から support を再構成する。
* duplicate reject のために port/span/bundle を先に生成して rollback 前提にする。
* duplicate 判定で span geometry、existing layout/seed、port position を読む。

## M29後 bb2 アーキテクチャ監査

監査日: 2026-06-16

結果: 1 件の境界修正後に PASS。

修正:

* `pairs make(graph)` は context link の `dir` を node position から再構成しない。
* new link の `dir` は引き続き input route geometry から計算する。
* context link の `dir` は `prepare()` で読み込んだ saved graph edge direction のまま維持する。

確認:

* existing context に対する topology authority は引き続き `SavedBackboneGraph` である。
* `pairs make(graph)` は引き続き pair/open/row の決定点である。
* context link は new edge として emit も save もしない。
* duplicate same edge bundle + lane は saved span binding から emit 前に reject する。
* layout は rules/group data を consume し、geom は layout を consume し、draw は layout/geom を consume する。
* bb2 production source は v1 backbone pipeline、recalc、materialization、support layout contract/projection、authority、seed、fallback、infer、guess、legacy、grouped span generation、support layout save entrypoint を参照しない。

既知の非ブロッカー:

* `BackboneLoweringKind::kBranchSupport` は既存 layout enum 値として引き続き使っている。bb2 では T/cross/branch の topology label ではなく、pair/open/row の決定にも使わない。

## bb2 対応生成範囲

フェーズ: 実用 mainline の対応範囲凍結。

## bb2 supported generation scope

Supported:

* supported requests run through bb2 and produce saved graph outputs.
* direct rules / layout / geom / draw output is generated without recalc.

Unsupported:

* existing scenes without `SavedBackboneGraph`.
* topology import from span / layout / seed / curve / port position.

Scope contract:

* `SavedBackboneGraph` is topology authority.
* `pairs make(graph)` is connectivity authority.
* duplicate same edge_bundle + lane is rejected before topology emit.
* lowered layout keeps `support_world` at the original support point and applies lowering to `endpoint_world`.
* GenerateFromBackboneSpec does not fall back to v1.

対応:

* input route は 2 点以上の polyline である。
* `interval_m > 0` の場合、各 input segment 上に deterministic な中間 generated node を挿入する。
* source segment 両端が同じ ownerless support kind の場合、`interval_m` / avoid detour の挿入 node は同じ ownerless support kind を継承する。
* support node は pole、明示的な new midair route point、または明示的な new building route point である。
* `SupportKind::kMidair`、`SupportKind::kBuilding`、`SupportKind::kGround` に既存 `node_id` が無い場合、new route point は明示的な ownerless support node として扱える。
* existing ownerless route point を受けるのは、`node_id` が pole id を持たず support kind も一致する saved backbone node の場合に限る。
* `SupportKind::kPole` かつ `node_id` なしの明示 `node_specs` は generated pole node である。
* generated pole の `node_specs` は、初期 pole yaw を設定する non-zero `tangent_hint` を持てる。
* route node は new pole、またはすでに `SavedBackboneGraph` node を持つ existing pole のいずれかである。
* template、layer、count、pole port band を mutation 前に resolve できるなら、1 個以上の bundle spec を許可する。
* `pole_type_id` 欠落を受けるのは、要求 bundle template 全体が 1 つの有効な共通 `related_pole_type_id` を共有する場合に限る。
* fixed-count bundle template は、明示 `count` が template の fixed count と完全一致する場合だけ受ける。
* range-count bundle template は、明示 `count` が template の min/max 範囲内にある場合だけ受ける。
* existing context は `SavedBackboneGraph` の node / edge / edge_bundle / binding record からのみ読む。
* same saved edge でも bundle-compatible scope が異なれば、edge_bundle と generated output を追加できる。
* duplicate same edge_bundle + lane は `AddPole` / `AddPort` / `AddBundle` / `AddSpan` 前に reject する。
* `constraints.lateral_offset_m` は port placement offset である。
* `avoid_points` が空なら `constraints.avoid_radius_m` は no-op として受ける。
* `constraints.avoid_points` は、`avoid_radius_m <= 0` の場合、または正の半径を持つ全 avoid point が要求 route と交差しない場合に no-op として受ける。
* route の単一 segment にだけ交差する positive avoid point は、input route の該当 segment を deterministic detour に変換して受ける。
* 複数 positive avoid point が同じ source segment にだけ交差する場合、detour point は source segment 上の `t` 順で route に挿入する。
* 複数 positive avoid point が複数 source segment に交差する場合、各 source segment ごとに detour point を `t` 順で route に挿入する。
* 同じ source segment 上で同じ detour 位置になる duplicate avoid point は、1つの detour node に coalesce する。
* `interval_m` と avoid detour が同じ segment にある場合、interval point と detour point は source segment 上の `t` 順で route に挿入する。
* `interval_m` の挿入位置と avoid detour が同じ source `t` にある場合、obstacle 内の interval point は作らず detour node を優先する。
* internal route vertex に positive avoid point が重なる場合、その vertex を upstream で deterministic detour point に置き換える。
* 明示 support node と完全一致する avoid point は、existing / new のどちらでも support を動かさず no-op として受ける。
* `PortPlacementBand` preflight は materialize 対象の active bundle だけを検査する。
* `pole_placement.pin_endpoints` と `pin_vertices` は newly generated pole にだけ適用する。
* `BundleNodeMode::kNotPresent` は validation 後の no-op として受ける。
* `BundleNodeMode::kPassThrough` は、current-route 上で target row intent が一意な interior pair、または target row intent が曖昧でない限定 saved-junction scope にだけ受ける。
* generated output には poles / bundles / ports / spans、saved backbone graph binding、span rules、layout、geom、最小 draw cache を含む。

非対応:

* saved ownerless support node が無いケース、および未対応の support kind。
* saved backbone node を持たない existing pole node。
* v1/manual scene からの saved graph migration。
* span / layout / seed / curve / port position から推測する graph import。
* 存在しない existing pole id を参照する明示 node spec。
* zero-length tangent hint。
* endpoint 付近 avoid や障害物回り込み探索を含む一般 avoid routing。
* 明示 `count` が不一致な fixed-count bundle template。
* 明示 `count` が template の min/max 範囲外にある range-count bundle template。
* duplicate same edge_bundle + lane request。
* 曖昧な pass-through / lowering target。
* 完全な support arm、insulator、attachment、render styling semantics。
* 要求 bundle template 群が 1 つの有効な共通 related pole type を提供しない場合の `pole_type_id` 欠落。

境界:

* `GenerateFromBackboneSpec` は bb2 を使い、unsupported request で v1 へ fallback しない。
* `SavedBackboneGraph` は topology authority である。
* `pairs make(graph)` は connectivity authority である。
* row separation と support group は placement authority のみである。
* layout は `support_world` を元の support / port point に保ち、`endpoint_world` だけを下げる。
* geom は layout を consume する。
* draw は layout / geom を consume し、`branch_down_offset_m` から `support_world` を補修しない。
* existing span / layout / seed / materialization result と position proximity は意味入力ではない。
* pole placement pinning は topology、connectivity、row placement、existing pole に影響しない。
* tangent hint は generated pole yaw にだけ影響し、topology、connectivity、row placement、existing pole には影響しない。
* `pin_vertices` は interval 挿入された auto node ではなく、元の clicked vertex に適用する。

## bb2 シナリオ受け入れパック

フェーズ: 実用 mainline のシナリオ網羅。

各シナリオ受け入れは、結果確認に加えて少なくとも 1 つの authority または否定境界 check を含む。

シナリオ:

* 単純な 2 点線: generated topology、rules、layout、geom、draw、saved graph が存在し、bb2 production source は v1 generation dependency を持たない。
* 3 点 polyline: link / pair / open / row は一度だけ決まり、rules は connectivity を再計算せず topology と placement group を consume する。
* 複数 bundle: bundle spec は同じ pair/open/row authority を共有したまま複数 output を生成する。
* existing pole 継続: saved graph node は受理し、saved graph を持たない manual/v1 pole は reject し、existing span は意味入力にしない。
* A-B-C 上の existing branch B-D: emit するのは new route だけで、context A-B/B-C link を再 emit / 再 save しない。
* A-B-C 上の existing cross D-B-E: pair+pair context を T/cross/branch kind label なしで扱う。
* 異なる bundle を持つ同一 saved edge: saved edge は共有し、edge_bundle は bundle 単位で扱う。
* duplicate same edge_bundle + lane: request は unsupported で、state は不変のままである。
* pass-through lowering: pair/open authority は不変で、layout、geom、draw が lowering result を consume する。
* new-route 内部 pass-through: 中点に `kPassThrough` を持つ 3 点 route は、current route がちょうど 1 つの target pair row を与える場合に対応し、このケースでは saved junction context を必要としない。
* generated pole pinning: `pin_endpoints` は generated route endpoint だけを manual pole として materialize し、`pin_vertices` は元の clicked vertex を manual として materialize する。existing pole と interval 挿入された auto node は mutate しない。
* interval route generation: `interval_m` は同じ bb2 graph pipeline 上で中間 node と span を materialize し、auto interval node は pinning 上の clicked vertex として扱わない。
* ownerless interval node: source segment 両端が同じ ownerless support の場合、interval 挿入 node は pole ではなく同じ ownerless support として保存する。
* 明示 new pole node spec: `SupportKind::kPole` かつ `node_id` なしは明示 generated pole marker として受ける。欠落した non-invalid id は引き続き reject する。
* fixed bundle count の完全一致: fixed-count template は template count と等しい冗長な明示 count を受けるが、異なる count は mutation 前に reject する。
* point なし avoid radius: `avoid_radius_m` 単体は no-op として受ける。
* zero-radius avoid point: `avoid_radius_m <= 0` の `avoid_points` は無効化された no-op として受け、layout や geom を変えない。
* 交差しない正半径 avoid point: route と交差しない正の半径の `avoid_points` は no-op として受ける。
* simple avoid detour: route の単一 segment 上に positive avoid point がある場合、bb2 は route 上流で detour node を挿入する。
* multi-segment avoid detour: 複数 source segment 上に positive avoid point がある場合、bb2 は各 source segment 上流で detour node を挿入する。
* duplicate avoid detour: 同じ detour 位置になる duplicate avoid point は zero-length link を作らないよう upstream で1点に畳む。
* endpoint avoid no-op: route first/last endpoint と完全一致する avoid point は固定 endpoint を動かさず no-op として受ける。
* explicit support avoid no-op: 明示 support node と完全一致する avoid point は、existing / new のどちらでも support を detour せず no-op として受ける。
* interval/avoid collision: interval point と avoid detour が同じ source `t` にある場合、detour node を採用し、obstacle 内の interval node は作らない。
* 明示 range bundle count: range-count template は min/max 内の要求 lane count を materialize し、範囲外 count は mutation 前に reject する。
* generated pole tangent hint: non-zero tangent hint は generated pole yaw を設定し、zero hint は mutation 前に reject し、pair/open/row は route 由来のまま保つ。
* bundle 由来 pole type: `pole_type_id` が欠け、要求 bundle が 1 つの有効な related pole type を共有する場合、generated pole はそれを使う。related pole type が混在する場合は mutation 前に reject する。
* new midair route point: 明示 `SupportKind::kMidair` は pole を作らず、pole id を持たない backbone node を保存し、input point height に ownerless port/span を materialize する。
* existing midair route point: saved backbone node id を持つ明示 `SupportKind::kMidair` は、pole を作らず span/layout/position から推測もせず、その node から延長する。
* new building route point: 明示 `SupportKind::kBuilding` は pole を作らず、support kind `Building` の backbone node を保存し、input point height に ownerless port/span を materialize する。
* building pick route point: `PickHitKind::kBuilding` は新しい ownerless `SupportKind::kBuilding` route point に resolve する。building hit id は任意で、saved backbone node id としては扱わない。
* ground pick route point: `PickHitKind::kGround` は新しい ownerless `SupportKind::kGround` route point に resolve する。anchor や attachment visual は作らない。

## bb2 変更境界の強化

フェーズ: emit 前変更境界。

生成順:

* `GenerateFromBackboneSpec` は `build()` の前に `prepare()` と `check()` を実行する。
* `build()` は `graph` から `pairs` を作る。
* duplicate saved span binding の preflight は intent、group、topology emit より前に走る。
* intent と support group は topology emit より前に計算する。
* topology emit が最初の object mutation step である。
* saved backbone graph binding は topology emit の後に行う。
* rules、layout、geom、draw は graph save の後に derive する。

変更境界:

* request / saved graph / template / band data から不足情報を判定できる invalid input は、topology emit 前に停止しなければならない。
* duplicate same edge_bundle + lane は saved graph binding から判明するため、`AddPole` / `AddPort` / `AddBundle` / `AddSpan` 前に reject しなければならない。
* emit 後の saved graph binding failure は、通常の duplicate policy ではなく internal invariant failure として扱う。

既知の非ブロッカー:

* bb2 は topology emit 後の internal invariant failure に対する rollback を未実装である。現在対応している duplicate と unsupported-input のケースは emit 前に preflight している。

## bb2 draw 消費境界

フェーズ: 最小 draw output 強化。

draw の責務:

* wire render cache は保存済み geom curve sample から生成する。
* lowered placeholder visual は layout の `support_world -> endpoint_world` から生成する。
* draw は決定済みの値を転送する consumer であり、topology、connectivity、placement、lowering を決めない。

禁止:

* draw は saved graph topology を読まない。
* draw は pair/open/row connectivity を読まない。
* draw は lowering を決めるために support group data を読まない。
* draw は `branch_down_offset_m` から `support_world` を再構成しない。
* draw は完全な support arm、insulator、attachment、render styling semantics を作らない。

viewer 注記:

* viewer/raylib の利用可否は bb2 core acceptance の一部ではない。viewer 依存欠落は別途記録し、`wire_core_tests` の bb2 acceptance は block しない。
* viewer/raylib availability is not part of bb2 core acceptance; missing viewer dependencies do not block `wire_core_tests` bb2 acceptance.

## bb2 GenerateFromBackboneSpec ゲート方針

フェーズ: mainline ゲート強制。

方針:

* 対応済み `BackboneSpec` request は bb2 の `prepare()` / `check()` / `build()` を通す。
* unsupported request は unsupported を返し、v1 generation へ fallback しない。
* existing pole に対して saved graph が欠落している場合は unsupported である。
* v1/manual scene migration は未実装である。
* existing span / layout / seed / curve / port position からの import は禁止する。

受け入れ境界:

* 対応 request は bb2 saved graph output を生成する。
* unsupported request は topology object と saved graph を変更しない。
* `SavedBackboneGraph` を持たない manual/v1 existing pole は引き続き reject する。

## bb2 実用 mainline アーキテクチャ監査

## bb2 usable mainline architecture audit

監査日: 2026-06-16

結果: PASS。

Result: PASS.

確認した owner:

* topology の owner: `SavedBackboneGraph`。
* topology owner: `SavedBackboneGraph`.
* connectivity の owner: `pairs make(graph)`。
* connectivity owner: `pairs make(graph)`.
* placement の owner: row separation と support group。
* port identity の owner: bundle-compatible scope を持つ saved row-port binding。
* span duplicate の owner: 最終 guard として bind invariant を持つ saved span binding preflight。
* lowering の owner: intent / support group / rules / layout。
* geom の owner: layout consumer。
* draw の owner: layout / geom consumer。
* draw owner: layout / geom consumer.
* unsupported の owner: v1 fallback を伴わない `prepare()` / `check()` / preflight。

確認した境界:

* downstream は span / layout / seed / curve / port position から topology を再構成しない。
* context link は判断入力のみであり、emit/save target ではない。
* draw は topology、pair/open/row、placement、lowering を決めない。
* bb2 production source は v1 backbone pipeline、recalc、materialization、support layout contract/projection、authority、seed、fallback、infer、guess、legacy、grouped span generation、support layout save entrypoint を参照しない。

残っている非ブロッカー:

* v1/recalc/inspection は bb2 の外側で引き続き旧 contract を保持している。
* v1/manual scene migration は引き続き unsupported である。
* 完全な support arm / insulator / attachment semantics は引き続き unsupported である。

## bb2 segment pick without bundle policy

対応日: 2026-06-18

supported scenario:

* `PickHitKind::kSegment` の dry-run branch pick は、selected bundle template が未指定でも `SupportKind::kMidair` の route point として解決できる。
* `PickHitKind::kSegment` の既定 branch pick も、selected bundle template が未指定なら debug virtual node を作らず ownerless midair route point として解決する。
* `PickHitKind::kNode` が saved ownerless backbone node を指す場合、その saved graph node の support kind / position を route input として解決できる。
* `PickHitKind::kSegment` が bb2 saved span を指す場合、endpoint id が pick payload に無くても saved graph binding から ownerless endpoint node へ snap できる。
* 解決結果を `BackboneSpec.path.node_specs` に渡した場合、bb2 は ownerless midair node / port / span output を生成する。
* selected bundle template がある segment pick で transient midair node id が返る場合、bb2 はその pending support node を route input として読める。
* `create_midair_node=true` を明示した segment pick は、selected bundle template が未指定でも pending ownerless midair node id を返し、後続 bb2 generation の route input として使える。
* selected bundle template 未指定の explicit midair pick は bundle mode を作らない。空の bundle policy は生成対象を制限せず、後続 request の bundle spec をそのまま materialize する。
* pending support node は既存 topology ではなく現在の pick input であり、generation 後は新しい saved ownerless backbone node として保存する。
* pending support node が bundle mode を持つ場合、`kNotPresent` の bundle は materialization 対象から外す。
* pending support node が bundle mode を持つ場合、bundle mode に無い request bundle も materialization 対象から外す。selected bundle policy は選択 bundle だけを生成対象にする。
* selected bundle template がある building / ground pick は pending ownerless surface node id を返し、後続 bb2 generation では選択 bundle だけを生成対象にする。
* selected bundle policy で inactive になった bundle に `kPassThrough` node mode が指定された場合、no-op で無視せず unsupported にする。
* selected bundle policy を持つ pending ownerless node が saved ownerless backbone node になった後も、その bundle policy は後続 continuation で保持する。
* saved selected ownerless node からの continuation は route 方向に関係なく selected bundle だけを生成対象にする。
* saved selected ownerless node で inactive bundle の `kPassThrough` が指定された場合も、no-op 前に unsupported にする。
* mixed selected bundle の midair branch は許可 bundle だけを生成し、全 bundle が disallowed の場合は成功 no-op とする。
* segment pick の hit position が地面/投影点の z を持つ場合でも、source span が分かるなら midair route point の高さは source span の port 高さから決める。
* source edge 上に作った saved ownerless node は source edge endpoint と `source_edge_t` を保持し、以後の branch generation では saved edge context を pair/open/row/placement 入力として読む。
* saved graph 由来の context node は `support_kind` と source-edge metadata を graph context に保持したまま読む。
* render cache は generated span の bundle template から cable template appearance（radius/color/material）を読む。
* support visual placeholder は layout の `support_world -> endpoint_world` と visual settings の radius を転写する。
* `VisualSettings.enable_support_structures=false` の場合、bb2 は rules/layout/geom/render を生成したまま support visual placeholder だけを生成しない。
* `VisualSettings.support_arm_radius_m` の変更は `UpdateVisualSettings` で変更として扱われ、bb2 support visual placeholder に反映される。

境界:

* selected bundle template がある場合の midair branch policy は維持する。
* selected bundle template がある surface pick policy も active bundle 選択にだけ使い、building / ground topology を pole や attachment 仕様から推測しない。
* bundle mode は active bundle 選択にだけ使い、pair/open/row の正本には使わない。
* saved ownerless node の bundle policy は SavedBackboneNode に保存する。後続 generation は span/layout/seed/position proximity から policy を復元しない。
* source span は pick geometry の高さ補正にだけ使い、topology / pair / row / port identity を span/layout/seed から復元しない。
* source edge context link は generation/save target ではなく判断入力であり、saved edge を split したり context span/port を生成しない。
* context node は saved graph node の support/source metadata を落とさず、pole 前提の local node へ潰さない。
* render は geom curve と span bundle の template appearance を転写するだけで、topology / pair / row / lowering を判断しない。
* support visual placeholder は full support arm / insulator / attachment semantics ではなく、既に決まった layout 線分の表示属性だけを持つ。
* support visual の有効/無効は draw 出力の表示設定であり、topology / pair / row / lowering / layout / geom を変えない。
* support visual の半径変更は表示属性の変更であり、topology / pair / row / lowering / layout / geom を変えない。
* segment pick は topology を span/layout/seed から推測するための入口ではない。
* bb2 generation は引き続き saved graph / explicit route input を正本にする。

## bb2 same edge different bundle with pass-through

対応日: 2026-06-19

supported scenario:

* saved edge A-B に既存 bundle がある状態で、同じ A-B に別 bundle を追加できる。
* その追加 request に `BundleNodeMode::kPassThrough` が付いていても、saved edge は共有し、別 `edge_bundle` と generated span を作る。
* pass-through mode は pair/open/row を変えず、layout/geom intent だけに影響する。

境界:

* pass-through の事前検査は saved graph incident と current route incident を見る。
* saved edge + current route link で対象 row が一意になる場合は supported とする。
* existing span/layout/seed/port position から target row を推測しない。
* same edge + same bundle duplicate policy は引き続き unsupported。

## bb2 stale segment-pick midair duplicate request

対応日: 2026-06-19

supported scenario:

* segment pick から作った pending ownerless midair node id を使って branch を生成した後、同じ branch が stale source id + explicit saved target pole で再送されても新しい node / port / bundle / span / saved graph を増やさない。
* pending ownerless node は、source saved edge と `source_edge_t` が一致する既存 saved ownerless node に解決する。
* duplicate same edge + bundle + lane policy は emit 前に効き、request は unsupported で止まる。

境界:

* pending node の解決は position proximity ではなく、saved graph の source edge identity と `source_edge_t` で行う。
* existing span/layout/seed から topology / pair / row / port identity を復元しない。
* stale pending id は新規 topology ではなく、既に保存済みの ownerless backbone node への参照として扱う。

## bb2 ownerless-only route without pole type

対応日: 2026-06-19

supported scenario:

* 両端が ownerless midair node の route は、new pole を作らないため `pole_type_id` が未指定でも生成できる。
* LV + Communication のように bundle template の related pole type が異なる場合でも、ownerless-only route では pole type ambiguity として reject しない。
* bundle / span / rules / layout / geom / draw は通常通り生成する。

境界:

* pole type は new pole を materialize する場合だけ必要。
* ownerless port placement は pole band を読まない。
* bundle spec は connectivity を変えず、pair/open/row は引き続き `pairs make(graph)` で一度だけ決める。

## bb2 missing port band preflight

対応日: 2026-06-19

supported boundary:

* new pole を含む route で必要な `PortPlacementBand` が無い場合、bb2 は topology mutation 前に unsupported で止まる。
* reject 後に pole / port / bundle / span / saved graph を増やさない。

境界:

* port band は port placement の前提であり、emit_ports で初めて発覚させない。
* missing band を固定値や geometry で補わない。
* ownerless-only route は引き続き pole band を読まない。

## bb2 segment-pick midair pass-through

対応日: 2026-06-19

supported scenario:

* segment pick 由来の pending ownerless midair node から branch を生成するとき、同じ route point に `BundleNodeMode::kPassThrough` を指定できる。
* pending node が saved node になる前でも、`source_edge` context がある場合は pass-through gate を通す。
* layout / geom は lowered result を保存する。

境界:

* `source_edge` context は `ResolveBranchPick` が作った saved graph edge identity から読む。
* existing span / layout / seed / port position から pass-through target を推測しない。
* pass-through は pair/open/row を変えず、intent / layout / geom にだけ影響する。

## bb2 selected saved ownerless node pick

対応日: 2026-06-20

supported scenario:

* selected bundle がある状態で saved ownerless building node を node pick し、その node から branch を生成できる。
* node pick でも selected bundle policy は pending support node に載せ、bb2 は saved backbone node context と bundle policy の両方を読む。
* LV + Communication request でも selected Communication だけを materialize する。
* selected policy は初回 branch 後も `SavedBackboneNode.bundle_modes` に bind され、同じ saved ownerless node からの後続 continuation でも保持される。

境界:

* selected policy は active bundle 選択にだけ使い、pair/open/row を変えない。
* saved ownerless node context は SavedBackboneGraph から読む。
* span/layout/seed/position proximity から topology や bundle policy を復元しない。
### C600 selected existing pole pick

bb2 treats selected bundle policy on an existing pole pick as transient route input. `ResolveBranchPick` may return a pending pole `SupportNode` carrying the selected bundle modes so the current `GenerateFromBackboneSpec` call materializes only the selected bundle. The policy is not stored on the saved pole backbone node; pole-backed `SavedBackboneNode.bundle_modes` remains empty. Ownerless saved nodes still persist bundle policy because they are route attachment identities, not ordinary pole topology nodes.

### C601 context-only bundle policy boundary

bb2 does not let bundle policy stored on a context-only saved node filter the bundles of a later materialized route. Context nodes remain pair/open/row inputs, but active bundle selection is based on the current route nodes only. This keeps a selected-bundle midair branch from accidentally suppressing LV/Communication generation on a later continuation from the generated endpoint pole.

### C602 context-only pole band boundary

bb2 does not let a context-only pole's `PortPlacementBand` availability reject a later materialized route. Port band preflight is limited to route nodes that will own generated ports. Context poles remain connectivity/placement inputs and do not become port placement prerequisites for unrelated new links.

### C603 route-local generated pole yaw

Generated pole yaw in bb2 is derived from the materialized route links, not from adjacent entries in the internal graph node vector. Context-only nodes may be present for pair/open/row decisions, but they do not affect the visual orientation of newly generated route poles.

### C604 large avoid detour radius

bb2 supports deterministic avoid detours whose radius is larger than the simple `radius + clearance` offset can satisfy. The detour offset is chosen so the two generated adjacent route segments clear the requested radius. If the avoid radius reaches a fixed route endpoint, bb2 rejects the request instead of moving the endpoint or using downstream geometry repair.

### C605 saved graph route query

`FindBackboneRoute` uses `SavedBackboneGraph` when queried endpoints resolve to saved backbone nodes. Ownerless bb2 nodes do not have pole ids, so public route queries must be explained from saved graph node/edge/edge_bundle state instead of span-derived edges. `BuildBackboneResult` / `BuildBackboneEdges` are saved-graph-only compatibility reads; public query must not reconstruct topology from spans/layout/ports.

### C609 acute corner lowering

bb2 lowers bent HighVoltage pair rows in the intent layer after pair/open/row has been resolved. This is a placement decision, not a junction kind label: pair/open/row is not recomputed and T/cross/branch enums are not introduced.

The lowered result is stored in layout/geom. Ports stay at their band height through `support_world`; `endpoint_world`, curve samples, and bounds carry the lower offset. Draw remains a consumer of layout/geom and does not repair lowering from visual-side data.

### C610 rules-only lowering refresh

bb2 rules-only spans keep their saved `SpanLayoutRule` when pole refresh invalidates projected layout. Refresh rebuilds layout/geom from the saved rule and the current endpoint ports; it does not require a v1 support-layout seed and does not infer lowering from draw/materialization state.

For lowered endpoints, refresh preserves `support_world` at the current port height and applies the lower offset only to `endpoint_world`, curve samples, and bounds.

## bb2 status cut 2026-06-23

目的:

* 進捗を C 番号ではなく supported scenario で説明する。
* viewer 未確認、旧テスト制約未移植、`pipeline.cpp` 肥大化を明示し、次の作業が「テストを増やしただけ」にならないようにする。
* この節は現状説明であり、新しい supported scope を宣言しない。

現状評価:

* bb2 は v1 fallback なしの generation mainline としてかなり広がった。
* ただし `core/src/generation/bb2/pipeline.cpp` は約 2500 行で、責務の追跡性は落ちている。
* `core/tests/bb2.cpp` は約 7000 行で、C 番号増加だけでは進捗説明にならない。
* viewer 代表 scene はまだ固定確認されていないため、「見た目まで順調」とは言わない。
* 旧 v1 テスト全体は bb2 完了条件ではないが、守っていた重要制約の移植状況は説明可能にする必要がある。

### supported scenario matrix

| Scenario | Status | Evidence | Notes |
|---|---|---|---|
| 2 点 line | supported | C368, C524, C539 | saved graph / rules / layout / geom / draw を生成する |
| 3 点 polyline / corner | supported | C388-C393, C525, C609-C610 | acute HV lowering は layout/geom に保存し、refresh 後も維持する |
| interval route | supported | C545, C580, C584, C586 | interval node は graph pipeline に入る。ownerless 継承も対応済み |
| avoid detour | supported, scoped | C552, C559, C579, C582-C588, C604 | deterministic detour のみ。一般 routing は未対応 |
| multiple bundle on one graph | supported | C400-C405, C526 | bundle spec は pair/open/row を変えない |
| existing pole continuation with saved graph | supported | C394-C396, C458-C461, C527-C528 | saved graph が無い existing pole は unsupported |
| same edge different bundle | supported | C464, C530, C574 | saved edge を共有し edge_bundle を追加する |
| duplicate same edge/bundle/lane | unsupported, preflight | C463, C466, C490, C520, C531 | mutation 前 reject を正とする |
| ownerless midair / building / ground route point | supported, scoped | C553-C558, C576, C584 | pole を作らず saved backbone node と ownerless port/span を作る |
| saved ownerless continuation | supported | C554, C562-C568, C591-C593, C605-C607 | saved graph node/source edge metadata を読む |
| selected bundle policy on picks | supported, scoped | C589-C601 | active bundle 選択だけに使い、pair/open/row は変えない |
| pass-through / lowering intent | supported, scoped | C481-C486, C532, C543, C574, C578 | pair/open/row は不変。layout/geom/draw が consume する |
| minimal draw/render | supported, minimal | C511-C514, C536-C538, C569-C572 | full support arm / insulator / attachment semantics は未対応 |
| BuildBackboneResult / route query from saved graph | supported | C605-C608 | span/layout/seed から topology を復元しない |
| v1/manual existing scene migration | unsupported | C515, C517, C541 | SavedBackboneGraph が無い scene は bb2 で暗黙 import しない |

### representative viewer scenes

bb2 の「実用として順調」を言うには、次の scene を viewer で確認する。core test pass だけでは完了扱いにしない。

| Scene | Purpose | Expected visual |
|---|---|---|
| simple line, LV/HV/Communication | terminal row separation と template appearance | route 直交方向に row が開き、wire が一点へ潰れない |
| 3 point acute HV corner | corner lowering | port は band height、wire endpoint/curve は一段 lower |
| A-B-C then B-D branch | saved graph context branch | context A-B/B-C は再生成されず、B-D だけ materialize される |
| A-B-C then D-B-E cross | pair+pair without kind label | cross/row separation が見え、T/cross enum 前提に戻らない |
| segment pick midair branch | ownerless route point | source span height を使い、地面 z から線が出ない |
| selected bundle midair/building pick | active bundle policy | 選択 bundle だけ materialize される |
| duplicate request | mutation boundary | 見た目も state count も増えず unsupported になる |

viewer gate:

* 代表 scene を見ていない間は「core architecture は進んだ」とだけ言い、「見た目まで順調」とは言わない。
* viewer で違和感が出た場合、core geometry か viewer 表示かを切り分ける。

headless viewer validators:

| Test | Scene | Fixed evidence | Status |
|---|---|---|---|
| V16 | simple line with LV/HV/Communication | `saved_backbone_result`, `span_layout`, curve, bounds, visual cache, render cache exist for generated spans | pass |
| V17 | A-B-C then B-D existing branch | context A-B/B-C are not regenerated; B-D outputs are display-readable; saved graph frontier exposes 3 edges | pass |
| V18 | pass-through/lowering branch | lowered endpoint is visible from layout and support visual placeholder reads layout points | pass |
| V19 | 3 point acute HV corner | lowered endpoint remains visible from layout/geom without lowering materialized ports | pass |
| V20 | A-B-C then D-B-E cross | pair+pair context produces display-readable new links without regenerating context links | pass |
| V21 | segment pick midair branch | `ResolveBranchPick` feeds a midair node, generated branch has display-readable outputs at source span height | pass |
| V22 | selected building pick | selected bundle policy materializes Communication only and outputs remain display-readable | pass |

not yet visually fixed:

* Manual visual capture review is still separate from these headless validators.
* These validators prove viewer-readable saved graph/layout/geom/draw data exists; they do not replace human inspection of the rendered frame.

### pipeline.cpp split plan

`pipeline.cpp` は約 2500 行で、これ以上の機能追加前に分割候補を固定する。rename 祭りはしない。namespace と短いファイル名で読ませる。

| File | Moves from current pipeline.cpp | Owner |
|---|---|---|
| `graph.cpp` | `prepare`, route expansion, avoid/interval insertion, saved context node read | input graph / saved graph context |
| `connect.cpp` | `make(const graph&)`, `pair/open/row`, row keys | connectivity authority |
| `intent.cpp` | `make(const pairs&)`, pass-through/lowering intent | placement intent, not topology |
| `group.cpp` | `make(const pairs&, const intent&)`, row separation/support groups | placement authority |
| `emit.cpp` | `emit_poles`, `emit_bundles`, `emit_ports`, `emit_spans`, `emit` | materialized topology output |
| `rule.cpp` | `make(const topo&, const groups&)` | saved span rules |
| `layout.cpp` | `make(const rules&)`, layout save support group cache collection | layout consumer |
| `geom.cpp` | `line`, `make(const layout&)`, bounds/curve creation | geometry consumer |
| `draw.cpp` | render/visual placeholder, `make(const layout&, const geom&)` | draw consumer |
| `save.cpp` | `save_graph`, `save(rules/layout/geom/draw)`, lane assignment publication | saved graph/cache boundary |

分割ルール:

* public class/method surface は急に増やさない。
* `pairs make(graph)` の単一確定点は維持する。
* draw は topology / pair/open/row / lowering を読まない。
* 分割 commit は behavior 変更を含めない。behavior 変更と同じ commit にしない。

### old test constraint migration top 20

旧テストをそのまま合否基準にしない。守っていた制約を bb2 の構造へ移す。

| Old case | Constraint | bb2 status | bb2 target |
|---|---|---|---|
| C110 | explicit existing pole node を重複生成しない | migrated | C394-C396 |
| C111/C112/C113/C115 | existing midair/support node continuation | partially migrated | C554, C562-C568, C605-C607 |
| C100/C103 | non-pole route point を pole 前提へ戻さない | migrated for scoped kinds | C553-C558 |
| C104-C107 | segment pick midair behavior and non-destructive dry-run | partially migrated | C560-C568, C587 |
| C185 | HV terminal row separation | partially migrated | C524/C569 周辺。viewer scene 必須 |
| C188 | HV hint 無し terminal row separation | not explicitly migrated | candidate after viewer |
| C189 | CommunicationPole + all templates terminal row | partially covered, not trusted visually | candidate viewer scene |
| C190 | Communication multi-lane terminal row | not explicitly migrated | candidate viewer scene |
| C191 | preserved multi-lane endpoint semantics | not migrated | candidate after draw/geom audit |
| C192 | clicked existing CommunicationPole all-template row separation | not trusted | candidate viewer + bb2 saved graph scenario |
| C204 | acute HV corner one-step lower | migrated structurally | C609 |
| C205 | lower identity survives pole refresh | migrated structurally | C610 |
| C206 | regeneration keeps acute lowering and cleans stale generated ports | not migrated | likely explicit rederive/regeneration milestone |
| C207/C208 | interval route extension lane order | not migrated | candidate after viewer |
| C209 | moderate corner lowering | not explicitly migrated | candidate if viewer shows gap |
| C210/C211/C212 | all-template branch/cross on CommunicationPole lowering | partially migrated by bb2 branch/cross, not visual-trusted | representative viewer scenes |
| C244-C249 | downstream does not re-decide endpoint/order/side | partially migrated by authority tests | audit when splitting `pipeline.cpp` |
| C250-C256 | lowered support group identity through refresh/unrelated updates | partially migrated | C518-C520, C610; more needed after viewer |
| C277 | high-low smooth span composite curve | not migrated | geom milestone, do not sneak into status cut |
| C298/C299 | requested CommunicationPole type and HV category height | partially migrated | C406-C408, C602; viewer scene still needed |

Top 20 handling rule:

* `migrated` は bb2 構造で守る制約があるという意味で、旧期待値をそのまま採用した意味ではない。
* `not trusted visually` は core test が通っても viewer 確認なしに完了扱いしない。
* `not migrated` は次の fail scenario 候補だが、viewer で症状を見てから選ぶ。

### next decision

次に production code を触る前に、どちらかを選ぶ。

1. viewer gate: representative viewer scenes を作って目視する。
2. split gate: `pipeline.cpp` を behavior 変更なしで分割し、追跡可能性を戻す。

どちらもやらずに C611 以降の scenario を増やすのは、スロー化の兆候として停止する。

## bb2 mainline checkpoint / v1 deletion map 2026-06-24

目的:

* bb2 をさらに広げず、v1 削除可能性を call graph で確認する。
* 物理削除は proven dead なものだけに限定する。
* `bb2` rename は v1 本流が物理的に外れるまで行わない。

entrypoint:

* `CoreState::GenerateFromBackboneSpec` は `generation::bb2::pipeline` の `prepare/check/build` だけを呼ぶ。
* v1 `BackbonePipeline` へ戻る fallback はない。
* `GenerateSimpleLine` / `GenerateSimpleLineFromPoints` は `BackboneSpec` を作って `GenerateFromBackboneSpec` に委譲する。

classification:

| Class | Meaning | Action |
|---|---|---|
| A | bb2 本流からも viewer/public query/tests からも未使用 | 物理削除可 |
| B | viewer/public query がまだ読む | 先に neutral bb2/SavedGraph read へ移す |
| C | tests だけが読む | 旧制約を bb2 構造へ移植してから削除 |
| D | v1/recalc 専用として残る | bb2 本流から隔離し、mainline と混ぜない |
| E | bb2 未対応 scenario のため残る | supported にするか unsupported を固定してから削除判断 |

deletion map:

| Target | Class | Current readers | bb2 generation reads it? | Next cut |
|---|---:|---|---:|---|
| `core/src/generation/backbone_pipeline/*` | removed | old generation tests, compiled old pipeline helpers | no | C365-C367 を退役し、旧 wrapper pipeline は物理削除済み |
| `core/src/generation/bundle_spans/*` | removed | old grouped span engine, one old helper test | no | C282 を退役し、grouped span engine は物理削除済み。bb2 は pair/row/support group/rules/layout/geom を正とする |
| `core/src/generation/build_backbone/build_request.cpp` | removed | old request planner | no | bb2 `prepare/check` が supported scope を持つため削除済み |
| `core/src/generation/build_backbone/build_span_layout_rules.*` | removed | old authority/rules bridge | no | bb2 rules は direct `SpanLayoutRules` 生成なので削除済み |
| `core/src/generation/route/build_support_chain.cpp` | removed | old request planner support resolution | no | bb2 は saved graph / explicit node spec を読むため削除済み |
| `CoreState::BuildBackboneEdges` | saved-only compat | old tests / public query name | no | span-derived edge reconstruction は削除済み。互換名だが `BuildSavedBackboneResult().edges` だけを返す |
| `CoreState::BuildBackboneResult` span-derived fallback | removed | old tests / public query name | no | `BuildSavedBackboneResult` へ委譲するだけに変更済み。span/layout/port position から topology を復元しない |
| `support_layout_projection` / `support_layout_contract` | B/D | viewer render overlay, old tests, recalc | no | viewer を `span_layout` / `span_layout_state` へ寄せる。recalc/v1 専用 API として隔離 |
| `SpanSupportLayoutDecisionSeed` and authority contract | C/D | recalc/materialization, old tests, validator | no | bb2 は `SpanLayoutRules` / `SpanLayoutEntry` を正とする。v1/recalc tests の制約分類後に削除判断 |
| `core/src/recalc/support_layout_materialization.*` | D | recalc pipeline and old support layout materialization tests | no | bb2 generation からは切断済み。v1/recalc 削除または隔離までは残す |
| `core/src/recalc/detail_curve_*` | D | recalc pipeline, old curve tests | no | bb2 geom は独自 direct output。post-edit rederive 方針が決まるまで v1/recalc 側に隔離 |
| viewer `support_layout_projection` read | B | none in `viewer/src` | no | render overlay は neutral `span_layout` へ移行済み。残る viewer 旧依存は debug/inspection 表示の `inspect_support_layout` |
| viewer `build_backbone_result` overlay | B | none in normal viewer overlay/query/capture | no | viewer は `saved_backbone_result` / `BuildSavedBackboneResult` へ移行済み。残る旧 `BuildBackboneResult` は adapter の v1 API として未使用化対象 |
| old tests using `Commit(run_recalc)` / support layout contract | C | `core/tests/generation.cpp`, `geometry.cpp`, `bundle_visuals.cpp`, others | no | old test constraint migration top 20 から順に、bb2 の SavedGraph/rules/layout/geom/draw 基準へ移す |

current deletion result:

* `backbone_pipeline`, `bundle_spans`, `build_backbone` request/rules, and route support-chain old planner are physically deleted.
* C365-C367 and C282 were retired because they asserted old pipeline/grouped engine internals rather than bb2 mainline contracts.
* `SegmentLaneAssignment` remains because bb2 publishes lane assignment snapshots for viewer/debug compatibility; it is no longer backed by the old grouped span engine.
* `BuildBackboneResult` / `BuildBackboneEdges` no longer reconstruct topology from spans; both read the saved graph view only.
* remaining deletion work is no longer blocked by those old generation families; next cuts are v1 recalc/support-layout inspection contracts and old test constraints.

mainline readiness:

| Item | Status | Reason |
|---|---|---|
| bb2 supported scope | usable v0 | C368-C610 bb2 acceptance pass。viewer 代表 scene は引き続き gate |
| v1 fallback from `GenerateFromBackboneSpec` | removed | entrypoint は bb2 pipeline only |
| v1 old pipeline physical deletion | done for generation mainline | `backbone_pipeline`, grouped span engine, and old request planner have been removed |
| viewer old contract dependency | mostly cut | normal overlay and DrawPath capture read neutral `span_layout`, rules, curve, bounds, visual/render caches, and saved graph. Remaining `inspect_support_layout` calls are legacy/recalc debug only |
| public query old span-derived fallback | removed | `BuildBackboneResult` and `BuildBackboneEdges` are saved-graph-only compatibility reads |
| `bb2` rename | blocked | recalc/public query/test remnants remain; renaming now would still mix bb2 with v1 inspection/recalc concepts |

next cuts:

1. support layout inspection boundary: keep remaining `inspect_support_layout` calls inside explicit legacy/recalc debug sections only; do not use it for bb2 capture or normal viewer validation.
2. old test constraint migration: migrate only high-value constraints from the top 20 list; do not chase old implementation expectations.
3. after B/C shrink, delete or isolate remaining v1 recalc/support-layout public query paths as explicit slices.

validation:

* `git diff --check`: pass.
* `wire_viewer`: build pass with VS18 CMake.
* `wire_viewer_tests`: V01-V22 pass.
* `wire_core_tests bb2`: C368-C610 pass.
* full `wire_core_tests`: fails in old v1/recalc tests; this is not a bb2 blocker, but each failure must be classified before using it as a fix target.

full old suite classification 2026-06-24:

* Observed command: `build-vs18-coretests/core/Debug/wire_core_tests.exe`.
* Result: exit 1, 197 failures.
* bb2 failures: 0 by failure-name scan; C368-C610 pass in the bb2 focused run.
* Failure log: `build-vs18-coretests/core/Debug/wire_core_tests_failures_latest.txt`.

| Category | Count | Meaning | bb2 action |
|---|---:|---|---|
| recalc/cache/render/visual/inspection/support-layout | 89 | Old dirty queue, recalc cache, render visual, support-layout contract, override/inspection behavior | Do not fix by reintroducing recalc into bb2. Extract constraints only when they apply to bb2 direct rules/layout/geom/draw. |
| old backbone/grouped-span/junction/lowering | 157 | Old v1 pipeline, grouped span engine, junction relation, lowering/order/side authority expectations | Do not restore v1 fallback. Migrate only the constraint if it is still valid under SavedBackboneGraph + pair/open/row + support group. |
| uncategorized by coarse scan | 6 | Forced reverse direction, validation authority, optical/coiled render, context profile wobble | Inspect individually before acting. None is a reason to keep bb2 tied to v1. |

classification rule:

* A failing old test is not a fix target until its constraint is named in bb2 terms.
* v1 implementation-detail expectations such as authority/seed/projection, grouped span internal order, or recalc side effects remain v1-only until migrated.
* bb2 blocker examples are limited to constraints that should hold in the new structure: unsupported input leaves state unchanged, saved graph explains topology, generated outputs have rules/layout/geom/draw, duplicate requests do not mutate state, and viewer-required outputs are present.

current mainline state:

* bb2 can be called as the generation entrypoint without v1 fallback.
* normal viewer overlay/query/capture now read `saved_backbone_result` instead of span-derived `BuildBackboneResult`.
* viewer render overlay and DrawPath capture use neutral `span_layout` / `span_layout_state` / `span_layout_rules` for bb2 checks; old `inspect_support_layout` remains legacy/recalc debug only.
* A-class old generation families were physically deleted in earlier checkpoints; remaining old pieces are recalc/inspection/test constraint work, not bb2 generation fallback.
* Rename from `bb2` to mainline remains blocked until old pipeline/recalc/public-query remnants are either deleted or explicitly isolated.

## bb2 recalc exit map 2026-06-25

目的:

* recalc を一気に削除せず、bb2 mainline から切る順序を固定する。
* viewer normal path / bb2 post-edit / public query が recalc なしで動く状態を先に作る。
* recalc 依存の旧 test は、期待構造ではなく守っていた制約を bb2 構造へ移植する。

baseline:

* working tree: clean.
* `git diff --check`: pass.
* `wire_viewer_tests`: V01-V22 pass.
* `wire_core_tests`: build pass.
* `wire_core_tests bb2`: C368-C610 pass.

classification:

| Target | Class | Current callers | bb2 mainline needed? | Next cut |
|---|---:|---|---:|---|
| `CoreState::Commit(run_recalc)` / `ProcessDirtyQueues` | B/D | viewer auto/manual recalc, state load/update paths, old workflow/geometry/generation tests | no | add bb2 direct derive entrypoint, then cut viewer normal path from auto recalc |
| viewer `legacy_auto_recalc` loop | B | `viewer/src/main.cpp`, `viewer/src/panels.cpp` UI | no | replace normal bb2 refresh with explicit direct derive; keep manual recalc only as legacy/debug until removed |
| `viewer_core_state::Commit` adapter | B/D | viewer auto/manual recalc and validation paths | no | keep for validation/debug; do not make bb2 post-edit depend on it |
| `support_layout_projection` / `support_layout_contract` | C/D | old tests, recalc pipeline, v1 inspection API | no | keep v1/recalc-only until old constraints are classified; bb2 observes `span_layout` / `span_layout_state` |
| `inspect_support_layout` | B/D | viewer legacy/recalc debug panels and old tests | no | normal viewer/capture already neutral; remaining calls stay isolated until legacy debug is removed |
| `cache_span_support_layout` / `cache_span_support_layout_seed` | C/D | recalc/materialization, core test hook, old validation tests | no | bb2 uses `cache_span_layout` / `cache_span_rules`; delete after old support-layout constraint migration |
| `core/src/recalc/detail_curve_*` | D | recalc curve rebuild and old curve tests | no | bb2 geom/draw already direct; post-edit direct derive must not call these |
| `core/src/recalc/support_layout_materialization.*` | D | recalc support layout materialization and old tests | no | isolate as v1/recalc until support-layout tests are migrated or retired |
| `CoreView::last_recalc_stats` / `RecalcStats` UI | B/D | viewer stats panel and old tests | no | keep as legacy/debug metric; not a bb2 progress signal |
| old tests using `Commit(run_recalc)` / support layout inspection | C | `generation.cpp`, `geometry.cpp`, `bundle_visuals.cpp`, `workflow.cpp`, `state_services.cpp`, `template_policy.cpp` | no | classify by family; migrate only constraints that still apply to SavedGraph/rules/layout/geom/draw |

exit order:

1. Add a bb2 direct derive entrypoint for saved rules/layout/geom/draw outputs.
2. Use that entrypoint for supported bb2 post-edit/rederive scenarios without `Commit(run_recalc)`.
3. Cut viewer normal bb2 refresh from `legacy_auto_recalc`; leave manual recalc as legacy/debug only.
4. Classify old recalc tests by constraint family before changing expectations.
5. Delete or isolate remaining recalc callers only after their active viewer/public/test dependency is gone.

stop conditions:

* topology, pair/open/row, port identity, or lowering would need to be inferred from span/layout/seed/curve/port position.
* bb2 post-edit would need `Commit(run_recalc)` or support-layout materialization to produce outputs.
* viewer normal display would need old `support_layout_contract` or `inspect_support_layout`.
* an old recalc test failure cannot be classified as either bb2 constraint or v1 implementation detail.

### old recalc test constraint families

Observed active old-test dependency count: about 250 reads/calls across `generation.cpp`, `geometry.cpp`, `bundle_visuals.cpp`, `workflow.cpp`, `state_services.cpp`, and `template_policy.cpp`.

Do not migrate these one test at a time. Classify and move by family:

| Family | Old surface | bb2 action |
|---|---|---|
| state unchanged on reject | `Commit` side effects / dirty queue checks | keep; assert through bb2 unsupported preflight and saved graph counts |
| saved topology explainability | `BuildBackboneResult`, junction inspection, span-derived fallback | already migrated to SavedBackboneGraph/public saved result |
| rules/layout/geom/draw existence | `support_layout_projection`, `inspect_support_layout`, recalc cache | migrate to `span_layout_rules`, `span_layout`, curve/bounds/draw caches |
| lowering survives refresh | `Commit(run_recalc)` + `inspect_support_layout` | migrate only if expressible as saved rule + layout/geom direct derive; do not restore support-layout authority seed |
| branch/cross row separation | old relation kind / support layout endpoint role | migrate as pair/open/row + support group placement; no T/cross/branch enum |
| attachment/socket support layout | support layout endpoint socket fields | not bb2 mainline yet; keep v1/recalc-only until attachment requirement is fixed |
| supplemental/coiled cable render | recalc detail curve supplemental paths | not bb2 mainline yet; keep v1/recalc-only until draw/geom requirement is fixed |
| validation of stale support-layout authority | mutated seed/projection/support-group caches | v1/recalc-only; bb2 equivalent is direct cache consistency, not seed/projection repair |
| context profile/template style recalc | `UpdateContextProfile` / cable template + Commit | migrate only as draw/render template output if needed for supported viewer scenarios |
| old grouped span order/authority | grouped span internals, support authority fields | retire or rewrite; do not reintroduce grouped span engine |

Priority:

1. migrate only viewer-visible supported bb2 constraints.
2. isolate v1/recalc-only tests behind their old surface.
3. delete old tests only after their constraint is either migrated or declared v1-only.

### direct derive entrypoint

`CoreState::DeriveGeneratedSpanOutputs(span_id)` is the bb2 direct derive entrypoint.

It consumes:

* existing span object
* saved `SpanLayoutRule`
* current endpoint ports referenced by that rule

It writes:

* `SpanLayoutEntry`
* `DetailCurve`
* `BoundsCacheEntry`
* minimal visual/render caches

It does not:

* call `Commit`
* call `ProcessDirtyQueues`
* read support layout contract/authority/seed
* call support-layout materialization
* infer topology, row, port identity, or span identity from geometry

This is not yet the full viewer normal refresh path. The next cut is to route supported bb2 post-edit/rederive cases through this entrypoint before keeping or removing viewer auto recalc.

first supported post-edit slice:

* `SetPortWorldPositionManual` / `ResetPortPositionToAuto` still mark connected spans dirty for v1 compatibility.
* Spans with saved backbone binding and saved `SpanLayoutRule` are immediately derived through `DeriveGeneratedSpanOutputs`.
* Direct-derived bb2 spans are removed from geometry/bounds/render dirty queues.
* Spans without saved backbone binding remain on the old dirty queue path until their constraints are migrated or retired.

viewer normal path cut:

* viewer `legacy_auto_recalc` now defaults to off.
* the UI labels recalc controls as legacy recalc.
* manual legacy recalc remains available for old debug/validation, but bb2 supported generation and port edit display outputs are expected to exist without it.
* validation: `build-vs18-viewer-fetch` config/build pass, and viewer tests V01-V23 pass.
