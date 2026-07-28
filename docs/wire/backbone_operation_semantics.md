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

## 実行時coverage契約

`C` / `O` / `K` / `U` の各セルは、test case ID や手書きタグではなく、test実行中に
構築された `CoreState` から状態を分類してcoverageへ記録する。matrixに行または列を追加し、
対応する実状態を構築しなければcore test全実行を失敗させる。

coverageとして認めるcaseは、次の独立evidenceを1つ以上持つ。

- `oracle`: 手計算値または固定fixture期待値
- `anchor`: 正本入力やstable IDとの関係
- `presence`: 必須出力の存在数。silent dropを許可しない
- `differential`: 同じ意味の別操作列、save/load前後等の比較

派生値同士の一致だけを示す`derived_equality`は、単独ではcoverage evidenceにしない。
source textを検査するcaseは`SourceGuard` familyへ分離し、操作×状態coverageには使用しない。

## 入口境界

core API の全required cellに加え、実アプリがbranch生成へ渡すpayloadは次のentryで検証する。
この表はweb testが実行時に直接読み、required cellを実payloadで通さなければ失敗する。

| Cell | Required entries |
|---|---|
| BOS:add_one_edge:S1 | `wasm_adapter` `viewer_action` |
| BOS:add_one_edge:SM | `wasm_adapter` `viewer_action` |

`wasm_adapter`は実WASM stateでpole snapとsource-edge midspanを解決し、生成結果まで検証する。
`viewer_action`は`ViewerActions`へ同じhit payload形状を渡し、解決済みnode identityが
node specへそのまま渡ることを検証する。adapter/viewerは位置や`hitId`からnode identityを再推測しない。
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
