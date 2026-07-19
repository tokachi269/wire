# Backbone 操作意味論

この文書は、操作前の状態と操作の組合せごとに、期待する状態遷移を定義する。
実装済みコードやtest結果ではなく、実装前に合意する要件の正本である。
実装の回帰一覧は`core/tests/spec_ledger.md`、未実装の保留条件は
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
| 新規edgeを1本追加 | `O` | `C` | `O` | `O` | `C` | `O` | template policyに従い`O`または`U` |
| 同一操作で2 edgeを追加 | `C` | `O` | `C` | `C` | `O` | `O` | `D3` |
| poleを移動して角度を変更 | `-` | `K` | `D1` | `D1` | `D1` | `D1` | `D3` |
| placement設定を更新 | `-` | `K` | `K` | `K` | `K` | `K` | `K` |
| save/load | `K` | `K` | `K` | `K` | `K` | `K` | `K` |
| regenerate | `K` | `K` | `K` | `K` | `K` | `K` | `K` |
| 明示的に2 openを接続 | `-` | `-` | `-` | `-` | `-` | `D2` | `D2` |

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
5本のincident edgeは、2接続pairと未接続1本なら3 support levelを使う。
1 levelは1接続pairまたは1未接続openを収容する。鋭角pairの2 open rowは
同じlevelを使うが、fixtureを潰さないため別support groupを持つ。

## 未定義セル

| ID | 未決定事項 | 決めない場合の影響 |
|---|---|---|
| `D1` | 通常角の共有Portを、角度変更時にedge endpoint別Portへ分離し、戻したとき再共有するか。代替はgenerated Portを常にedge endpoint別に保持し、共有row/fixtureだけを派生する設計 | 現在は通常pairが1 Portを共有するため、continuityを維持したまま非ゼロ長jumperへ切り替えられない |
| `D2` | 複数openから2本を明示指定するCore APIとUI操作 | `S5`の曖昧状態は自動解消しない |
| `D3` | ownerless中間nodeで複数edgeを追加、明示接続、repositionする操作契約 | midair branchの複合junctionを安全に実装できない |

## 変更手順

1. 新機能またはバグ修正の対象となる操作と既存状態を特定する。
2. 対象セルが`D#`なら、Input、正本、派生、失敗時動作を決めて表を更新する。
3. `C/O/K/U`へ確定したセルごとにfail-first testを追加する。
4. 実装後、対応testを`core/tests/spec_ledger.md`へ登録する。
5. 隣接セルに同じ変更が波及しないことを確認する。

未定義セルをtestの期待値や現在のコードから埋めてはならない。
