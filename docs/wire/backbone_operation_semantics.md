# Backbone 操作意味論

この文書は、操作前の状態と操作の組合せごとに、期待する状態遷移を定義する。
実装済みコードやtest結果ではなく、実装前に合意する要件の正本である。
実装の回帰一覧は`domains/wire/tests/spec_ledger.md`、未実装の保留条件は
`docs/merge_readiness.md`が担当する。

## セルの状態

| 表記 | 意味 |
|---|---|
| `C` | 操作成功。継続テーブルへ接続を記録する |
| `O` | 操作成功。推測せず未接続openのまま残す |
| `K` | 操作成功。既存の接続記録を維持し、派生出力だけ再計算する |
| `U` | mutation前に`unsupported`として拒否する |
| `-` | その状態では操作対象にならない |
| `D#` | 要件未定義。決定してこの表を更新するまで実装しない |

`C`は接続の有無だけを表す。NodePatch、jumper、row、fixtureは接続記録と
現在の幾何から導出するため、`C`の判定理由に角度を使わない。

## 既存状態

| 状態 | 定義 |
|---|---|
| `S0` | 対象nodeに同placement/laneのincident edgeがない |
| `S1` | 未接続openが1本ある |
| `S2` | 通常角として表示中の接続済みpairだけがある |
| `S3` | 鋭角として表示中の接続済みpairだけがある |
| `S4` | 接続済みpairと未接続openが1本ある |
| `S5` | 接続済みpairと未接続openが2本以上ある |
| `SM` | source edge中間のownerless node |

状態はBundle placementとlaneごとに評価する。同じnodeでもtemplateまたは
placementが違うedgeは別の評価単位であり、category名や高さでまとめない。

## 操作×状態

| 操作 | `S0` | `S1` | `S2` | `S3` | `S4` | `S5` | `SM` |
|---|---|---|---|---|---|---|---|
| `add_one_edge` 新規edgeを1本追加 | `O` | `C` | `O` | `O` | `C` | `O` | template policyに従い`O`または`U` |
| `add_two_edges` 同一操作で2 edgeを追加 | `C` | `O` | `C` | `C` | `O` | `O` | `D3` |
| `move_pole_angle` poleを移動して角度を変更 | `-` | `K` | `K` | `K` | `K` | `K` | `D3` |
| `update_placement` placement設定を更新 | `-` | `K` | `K` | `K` | `K` | `K` | `K` |
| `save_load` save/load | `K` | `K` | `K` | `K` | `K` | `K` | `K` |
| `regenerate` regenerate | `K` | `K` | `K` | `K` | `K` | `K` | `K` |
| `connect_two_open` 明示的に2 openを接続 | `-` | `-` | `-` | `-` | `-` | `D2` | `D2` |

## セル必須観点

`C` / `O` / `K` / `U` の各セルは、case ID の有無だけでなく、次の観点を
`domains/wire/tests/spec_ledger.md` の aspect coverage で満たす。
この表は「何を見ればそのセルを検証したと言えるか」の正本である。

| Cell | Required aspects |
|---|---|
| BOS:add_one_edge:S0 | `open_state` `support_level` |
| BOS:add_one_edge:S1 | `pair_connectivity` `continuity_table` `support_level` `curve_endpoint` |
| BOS:add_one_edge:S2 | `open_state` `support_level` |
| BOS:add_one_edge:S3 | `open_state` `ambiguity_open` |
| BOS:add_one_edge:S4 | `pair_connectivity` `continuity_table` `support_level` |
| BOS:add_one_edge:S5 | `open_state` `ambiguity_open` |
| BOS:add_one_edge:SM | `template_policy` `selected_policy` `source_projection` `open_state` |
| BOS:add_two_edges:S0 | `pair_connectivity` `continuity_table` `port_identity` `curve_endpoint` |
| BOS:add_two_edges:S1 | `open_state` `ambiguity_open` |
| BOS:add_two_edges:S2 | `open_state` `ambiguity_open` |
| BOS:add_two_edges:S3 | `open_state` `ambiguity_open` |
| BOS:add_two_edges:S4 | `open_state` `ambiguity_open` |
| BOS:add_two_edges:S5 | `open_state` `ambiguity_open` |
| BOS:move_pole_angle:S1 | `open_state` `port_identity` `curve_endpoint` |
| BOS:move_pole_angle:S2 | `continuity_table` `port_identity` `representation_switch` `row_fixture` `nodepatch` `curve_endpoint` |
| BOS:move_pole_angle:S3 | `continuity_table` `port_identity` `representation_switch` `row_fixture` `jumper` `curve_endpoint` |
| BOS:move_pole_angle:S4 | `continuity_table` `port_identity` `representation_switch` `support_level` |
| BOS:move_pole_angle:S5 | `continuity_table` `port_identity` `representation_switch` `support_level` |
| BOS:update_placement:S1 | `placement_identity` `support_level` `curve_endpoint` |
| BOS:update_placement:S2 | `placement_identity` `support_level` `curve_endpoint` |
| BOS:update_placement:S3 | `placement_identity` `support_level` `curve_endpoint` |
| BOS:update_placement:S4 | `placement_identity` `support_level` `curve_endpoint` |
| BOS:update_placement:S5 | `placement_identity` `support_level` `curve_endpoint` |
| BOS:update_placement:SM | `placement_identity` `source_projection` `curve_endpoint` |
| BOS:save_load:S0 | `save_load` `derived_equivalence` |
| BOS:save_load:S1 | `save_load` `continuity_table` |
| BOS:save_load:S2 | `save_load` `continuity_table` `port_identity` `derived_equivalence` |
| BOS:save_load:S3 | `save_load` `continuity_table` `jumper` `derived_equivalence` |
| BOS:save_load:S4 | `save_load` `continuity_table` `support_level` |
| BOS:save_load:S5 | `save_load` `continuity_table` `support_level` |
| BOS:save_load:SM | `save_load` `source_projection` |
| BOS:regenerate:S0 | `regenerate` `derived_equivalence` |
| BOS:regenerate:S1 | `regenerate` `continuity_table` |
| BOS:regenerate:S2 | `regenerate` `continuity_table` `port_identity` `derived_equivalence` |
| BOS:regenerate:S3 | `regenerate` `continuity_table` `jumper` `derived_equivalence` |
| BOS:regenerate:S4 | `regenerate` `continuity_table` `support_level` |
| BOS:regenerate:S5 | `regenerate` `continuity_table` `support_level` |
| BOS:regenerate:SM | `regenerate` `source_projection` |

## セル必須入力代表

`セル必須観点`は何を検査するかを定義する。この表は、どの入力形状・操作履歴を
最低限通すかを定義する。case IDだけではなく、この代表集合を
`domains/wire/tests/spec_ledger.md` の representative coverage で満たす。

| Cell | Required representatives |
|---|---|
| BOS:add_one_edge:S0 | `single_edge` `no_existing_incident` |
| BOS:add_one_edge:S1 | `incremental` `existing_open_one` `normal_angle` `sharp_angle` `draw_forward` `draw_reverse` `branch_down_enabled` `branch_down_disabled` |
| BOS:add_one_edge:S2 | `incremental` `existing_pair_normal` `single_added_edge` `branch_down_enabled` |
| BOS:add_one_edge:S3 | `incremental` `existing_pair_sharp` `single_added_edge` `ambiguity_candidates` |
| BOS:add_one_edge:S4 | `incremental` `existing_pair_plus_open_one` `single_added_edge` `occupied_lower_levels` `branch_down_enabled` |
| BOS:add_one_edge:S5 | `incremental` `existing_pair_plus_open_many` `ambiguity_candidates` |
| BOS:add_one_edge:SM | `incremental` `source_edge_midspan` `selected_template_allowed` `selected_template_rejected` |
| BOS:add_two_edges:S0 | `one_shot` `two_edge_same_operation` `normal_angle` `sharp_angle` `viewer_default_bundle_set` `branch_down_enabled` `branch_down_disabled` |
| BOS:add_two_edges:S1 | `one_shot` `two_edge_same_operation` `existing_open_one` `ambiguity_candidates` |
| BOS:add_two_edges:S2 | `one_shot` `two_edge_same_operation` `existing_pair_normal` `sharp_angle` `occupied_lower_levels` `branch_down_enabled` |
| BOS:add_two_edges:S3 | `one_shot` `two_edge_same_operation` `existing_pair_sharp` `ambiguity_candidates` |
| BOS:add_two_edges:S4 | `one_shot` `two_edge_same_operation` `existing_pair_plus_open_one` `ambiguity_candidates` |
| BOS:add_two_edges:S5 | `one_shot` `two_edge_same_operation` `existing_pair_plus_open_many` `ambiguity_candidates` |
| BOS:move_pole_angle:S1 | `move_pole` `existing_open_one` |
| BOS:move_pole_angle:S2 | `move_pole` `existing_pair_normal` `move_to_sharp` `move_back_to_normal` `model_assembly` |
| BOS:move_pole_angle:S3 | `move_pole` `existing_pair_sharp` `move_to_normal` `move_back_to_sharp` `model_assembly` |
| BOS:move_pole_angle:S4 | `move_pole` `existing_pair_plus_open_one` `representation_switch` |
| BOS:move_pole_angle:S5 | `move_pole` `existing_pair_plus_open_many` `representation_switch` |
| BOS:update_placement:S1 | `placement_update` `existing_open_one` |
| BOS:update_placement:S2 | `placement_update` `existing_pair_normal` `same_template_multi_placement` |
| BOS:update_placement:S3 | `placement_update` `existing_pair_sharp` |
| BOS:update_placement:S4 | `placement_update` `existing_pair_plus_open_one` `same_template_multi_placement` |
| BOS:update_placement:S5 | `placement_update` `existing_pair_plus_open_many` |
| BOS:update_placement:SM | `placement_update` `source_edge_midspan` |
| BOS:save_load:S0 | `save_load` `no_existing_incident` |
| BOS:save_load:S1 | `save_load` `existing_open_one` |
| BOS:save_load:S2 | `save_load` `existing_pair_normal` `migration_shared_port` |
| BOS:save_load:S3 | `save_load` `existing_pair_sharp` `migration_shared_port` |
| BOS:save_load:S4 | `save_load` `existing_pair_plus_open_one` |
| BOS:save_load:S5 | `save_load` `existing_pair_plus_open_many` |
| BOS:save_load:SM | `save_load` `source_edge_midspan` |
| BOS:regenerate:S0 | `regenerate` `no_existing_incident` |
| BOS:regenerate:S1 | `regenerate` `existing_open_one` |
| BOS:regenerate:S2 | `regenerate` `existing_pair_normal` |
| BOS:regenerate:S3 | `regenerate` `existing_pair_sharp` |
| BOS:regenerate:S4 | `regenerate` `existing_pair_plus_open_one` |
| BOS:regenerate:S5 | `regenerate` `existing_pair_plus_open_many` |
| BOS:regenerate:SM | `regenerate` `source_edge_midspan` |

### 接続候補の規則

生成操作の完了時に、影響nodeの未接続endpointを同じ解決処理へ渡す。

1. `SavedBackboneRowContinuity`に既に含まれるendpointは候補から除外する。
2. 同じBundle placement identity、互換lane、同じnodeの未接続endpointだけを候補とする。
3. 候補が相互に1つだけなら接続し、`SavedBackboneRowContinuity`へ記録する。
4. 候補が0または2つ以上なら操作は成功させ、未接続openのまま残す。
5. 角度、距離、edge ID、生成順、配列順、category、高さは相手選択に使わない。

同一操作で追加された2 edgeもこの規則を通す。操作内の隣接という理由だけで
既存の未接続endpointより優先しない。

### 表現の規則

接続済みendpointの表現は、継続テーブルと現在の角度から導出する。

| 条件 | 派生表現 |
|---|---|
| 通常角 | 共有row/fixtureとNodePatch |
| 鋭角 | edgeごとの2 open row/fixtureとlane別jumper |
| 未接続 | edge固有open row。NodePatch/jumperなし |

角度は表現だけを変え、`SavedBackboneRowContinuity`の相手を変更しない。
generated Portは常にedge endpointごとに別identityを持つ。通常角pairでもPortを
共有せず、共有するのは派生row/fixtureだけとする。通常角pairの2 Portのworld位置は、
同じrow layout決定から導出して浮動小数bitまで一致させる。2つの独立計算結果を
後段で近づけたり、一方を他方へコピーして補正したりしない。

endpoint fixtureはPort単位ではなく派生した`(row, lane)`単位で生成する。
通常角pairでは2 Portに対して1 fixture、鋭角pairでは2つの派生open rowに対して
2 fixtureを生成する。角度変更ではPort identityとcontinuityを維持し、row、fixture、
NodePatch、jumperだけを再導出する。

`SavedBackboneRowKey`は`(node_id, edge_id)`だけを保存し、1つのedge endpointを識別する。
pair/openの表現、peer edge、row axisは保存しない。接続済みpeerは
`SavedBackboneRowContinuity`から解決し、現在のincident edge幾何と既存のcorner規則を
1つの派生処理へ渡して、共有rowまたは2 open rowを決める。通常角のrow axisは従来どおり
2 edgeの単位接線二等分線から決め、鋭角の各row axisは各edge方向から決める。

support levelはcontinuity単位ではなく物理fixture row単位で割り当てる。
通常pairは共有rowとして1 levelを使う。鋭角pairは2つのopen rowとjumperへ派生するため、
2つの異なる空きlevelを使い、各rowが別support groupを持つ。
既に低いlevelが別rowで埋まっている場合は、鋭角pairの2 rowへ次の2つの空きlevelを割り当てる。
jumperはcontinuityを表すだけで、placement levelを共有させない。

### 既存saveの移行

共有Portを含む既存`wire_state_v2`はload時にedge endpoint別Portへ移行する。
片側endpointへ決定的に新しいObjectIdを割り当て、そのedge bundleのPort bindingと
Span endpoint参照を同じIDへ書き換える。pair row keyは、各bindingが所有する
edge bundleのedge IDを使う`(node_id, edge_id)`へ分割する。高さ、edge IDの大小、
Bundle ID、container順から
接続相手を推測せず、既存のrow continuityとbindingだけを使う。

分割対象または書き換えるendpointを一意に特定できないsaveは、部分適用せず
明示errorで停止する。移行前後でNodePatch、jumper、fixture、layout endpoint、
curve endpointの意味値を一致させ、load後の再saveは新形式へ正規化する。

## 決定済みセル

| ID | 決定 | 必須検証 |
|---|---|---|
| `D1` | Portと`SavedBackboneRowKey(node_id, edge_id)`は常にedge endpoint別identityとし、pair/open、peer edge、row/fixtureはcontinuityと現在幾何から導出する | 通常角2 Portのworld位置bit一致、laneあたりfixture 1個、角度変更時のPort/RowKey不変、NodePatch↔jumper切替、v2共有Port/pair key migration前後の派生出力等価 |

## 未定義セル

| ID | 未決定事項 | 決めない場合の影響 |
|---|---|---|
| `D2` | 複数openから2本を明示指定するCore APIとUI操作 | `S5`の曖昧状態は自動解消しない |
| `D3` | ownerless中間nodeで複数edgeを追加、明示接続、repositionする操作契約 | midair branchの複合junctionを安全に実装できない |

## 変更手順

1. 新機能またはバグ修正の対象となる操作と既存状態を特定する。
2. 対象セルが`D#`なら、Input、正本、派生、失敗時動作を決めて表を更新する。
3. `C/O/K/U`へ確定したセルごとにfail-first testを追加する。
4. 実装後、対応testを`domains/wire/tests/spec_ledger.md`へ登録する。
5. 隣接セルに同じ変更が波及しないことを確認する。

未定義セルをtestの期待値や現在のコードから埋めてはならない。
