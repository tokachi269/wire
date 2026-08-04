# Road backlog

現在の標準機能から外した項目。目的と再開条件だけを残す。詳細設計と未実装型はここに書かない。
現行の契約は `architecture.md` / `operation_semantics.md` / `supported_operations.md` を見る。

| 項目 | 目的 | 再開条件 |
|---|---|---|
| 手動線マーキング | 自動白線で表現できない個別の線を引く | どの線が自動で出せてどれが出せないかを実際の断面で確認してから。標準UI・public APIから外し、保存fieldだけ残している |
| 手動面マーキング | ゼブラ等の面を個別に置く | 同上 |
| 区間途中の部分削除 | segment内の範囲を消す | 通常削除(segment単位)で足りない場面が実際に出てから。保存対象ではないので公開経路ごと削除済み |
| raw lane topology編集 | 曖昧なjunctionで車線接続を明示指定する | junctionの自動解決が曖昧になる実例を集めてから。意味的な選択(どの車線からどの車線へ)をUIで表現できる形で設計する。`lane_connections` / `boundary_continuations` はjunction生成が内部利用するため保存fieldは残る |
| 道路沿い配置(街路樹・標識・建物) | 道路に沿ってpropを並べる | consumerを実装する時点で、そのconsumerの要件から参照型を設計する。先行実装は削除済み |
| 高架・terrain・構造物 | 立体交差と地形 | 未着手 |

## 保存互換性

手動線・手動面・lane connection・boundary continuationは `SavedRoadGraph` に残り、archive version 11 で読み書きできる。
標準UIとpublic APIから外しただけで、既存workspaceは開ける。保存fieldの削除は別途migration方針を決めてから行う。
