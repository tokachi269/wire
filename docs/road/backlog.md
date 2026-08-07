# Road backlog

現在の標準機能から外した項目。目的と再開条件だけを残す。詳細設計と未実装型はここに書かない。
現行の契約は `architecture.md` / `operation_semantics.md` / `supported_operations.md` を見る。

| 項目 | 目的 | 再開条件 |
|---|---|---|
| 手動線マーキング | 自動白線で表現できない個別の線を引く | どの線が自動で出せてどれが出せないかを実際の断面で確認してから。標準UI・public APIから外し、保存fieldだけ残している |
| 手動面マーキング | ゼブラ等の面を個別に置く | 同上 |
| 区間途中の部分削除 | segment内の範囲を消す | 通常削除(segment単位)で足りない場面が実際に出てから。保存対象ではないので公開経路ごと削除済み |
| raw lane topology編集 | 曖昧なjunctionで車線接続を明示指定する | junctionの自動解決が曖昧になる実例を集めてから。意味的な選択(どの車線からどの車線へ)をUIで表現できる形で設計する。`lane_connections` / `boundary_continuations` はjunction生成が内部利用するため保存fieldは残る |
| Branch / Merge | 本線から車線を分岐させる / 本線車線へ合流させる | 「どの車線から分岐するか」を意味で選べるUIを設計してから。raw BoundaryIdのdropdownと既定値の自動選択がPOC条件に反したため削除した。内部lane topologyはjunction生成が使うので残っている |
| 道路沿い配置(街路樹・標識・建物) | 道路に沿ってpropを並べる | consumerを実装する時点で、そのconsumerの要件から参照型を設計する。先行実装は削除済み。参照線の区別は `architecture.md` の「Edge datum と envelope」 |
| 縦断・地形・立体交差 | 縦断プロファイルと路面roll、地形追従と切り込み、高さの違う道路の非接続交差、地上と高架の構造mode境界 | 平面の断面・接続契約が安定してから。`VerticalProfile` / `RollProfile`はこのときに初めてschemaへ追加する |
| 高架の連続形状 | 高架壁と床版側面・下面の押し出し、地面マスクの無効化と遷移 | 縦断が入ってから。橋脚等の個別モデル配置は別 |
| ユーザー定義Area | ロータリー・広場の形状生成。outerとhole 1個から | 交差点とcorridorが安定してから。内部の交通意味論は対象外 |
| wire連携 | 道路脇の基準Path、side、offset、許可区間だけを公開する`RoadsideGuide` | wire側にconsumerができてから。roadとwireは相互に内部構造を読まない |
| 白線幅の所有権移動 | style IDから物理幅を導く表をWebへ移す | 幅の保存場所を決めてから。現在archiveはstyle IDだけを持ち、幅は `marking_width_m` がCore内で決めている。移すとarchive schema変更とmigrationが必要 |
| wire製品カタログの所有権移動 | 電柱型・束・ケーブル・取付テンプレートをWebへ移す | road断面と同じ扱いにする。`CoreState()` が4カタログを自動登録し、Web(`defaultBundlePreset.ts`)が固定ID 101/102/104を仮定している。archiveは定義を保存しているのでLoad互換は取れるが、wire test 457箇所のCoreState生成と4種のWASM入力schemaが必要 |

## 端部構造

L字溝・縁石・median edgeは`BoundaryProfile`のcontourとして実装済み。契約は`architecture.md`の
「Boundary profile」。**edge datumはcontourのlateral 0**であって、L字溝の道路側垂直面をそこへ置く。
残りは次の3点。

1. **断面の外端にprofileを置けない。** `boundaries.size() + 1 == strips.size()`のままなので、
   boundaryは必ず2つのstripの間にある。歩道のない側のL字溝は宣言できず、
   `architecture.md`の「歩道がない場合」の張り出しも表現できない。gapごとに1つ
   (`strips.size() + 1`個)へ変えると外端も実boundaryになり、合成ID 1 / 999も消える。
   `merge_boundary_policies`のindexとjunctionの外端判定が連動する。
2. **接続が断面styleの完全一致を要求する。** `connection_geometry_from_gates`は
   `surface_styles`が違うと`NotImplemented`。片側だけ溝がある、左右で溝が違う、
   溝つき道路と縁石道路をつなぐ、はいずれもcornerで通らない。
3. **`Mesh`に法線とUVがない。** `derived_types/derived_road.hpp`は頂点とindexだけを持つ。
   L字溝のhard edgeは表現できない。**現状hard edgeは未対応。** Unrealへ出す前に片付ける。

埋設部・基礎の下面(structural envelope)、面ごとの勾配mode、socket、蓋やgratingのinstanceは
まだ設計していない。下面が要るときはcontourをもう1本足す形にし、閉曲線・hole・booleanへは広げない。

## 却下

再提案する場合は、ここに書いた理由が変わったことを示す。

- 車両のroute choice・右左折判断: 交通対応時の導出物。既存のlane geometry pathへ交通判断を混ぜない
- アウトインアウト: 道路境界を動かす場合と車両経路だけを動かす場合を分けられていない
- `BoundaryProfile`の丸み種別: 未使用のjoin欄を先に置かない。丸み編集を実装するときに追加する
- `NodeConnectionPolicyOverride`の追加値: Auto / ForcePassThrough / ForceCorner / ForceJunction以外を先に定義しない
- 高架壁と地上curbの自動変換: 明示的な構造切替がなければunsupportedのままにする
- 交通・信号・5叉路以上の自動整形・複雑車線分岐・短距離多重遷移・排水計算: 全phaseで非目標

## 保存互換性

手動線・手動面・lane connection・boundary continuationは `SavedRoadGraph` に残り、archive version 13 で読み書きできる。
version 12 の workspace も開ける。boundary の width/height を2点profileへ解決し、その幅を左隣stripへ移す。
標準UIとpublic APIから外しただけで、既存workspaceは開ける。保存fieldの削除は別途migration方針を決めてから行う。
