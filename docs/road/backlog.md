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
| 端部構造(L字溝) | 道路端に断面を持つ構造を置き、道路幅と歩道幅の境界を明示する | 下の「端部構造」の衝突を解消してから。契約は `architecture.md` の「Edge datum と envelope」にあり、実装はない |
| 縦断・地形・立体交差 | 縦断プロファイルと路面roll、地形追従と切り込み、高さの違う道路の非接続交差、地上と高架の構造mode境界 | 平面の断面・接続契約が安定してから。`VerticalProfile` / `RollProfile`はこのときに初めてschemaへ追加する |
| 高架の連続形状 | 高架壁と床版側面・下面の押し出し、地面マスクの無効化と遷移 | 縦断が入ってから。橋脚等の個別モデル配置は別 |
| ユーザー定義Area | ロータリー・広場の形状生成。outerとhole 1個から | 交差点とcorridorが安定してから。内部の交通意味論は対象外 |
| wire連携 | 道路脇の基準Path、side、offset、許可区間だけを公開する`RoadsideGuide` | wire側にconsumerができてから。roadとwireは相互に内部構造を読まない |
| 白線幅の所有権移動 | style IDから物理幅を導く表をWebへ移す | 幅の保存場所を決めてから。現在archiveはstyle IDだけを持ち、幅は `marking_width_m` がCore内で決めている。移すとarchive schema変更とmigrationが必要 |
| wire製品カタログの所有権移動 | 電柱型・束・ケーブル・取付テンプレートをWebへ移す | road断面と同じ扱いにする。`CoreState()` が4カタログを自動登録し、Web(`defaultBundlePreset.ts`)が固定ID 101/102/104を仮定している。archiveは定義を保存しているのでLoad互換は取れるが、wire test 457箇所のCoreState生成と4種のWASM入力schemaが必要 |

## 端部構造

`architecture.md` の edge datum 契約は決めただけで、実装は現行と衝突している。着手前に次を解消する。
番号順に依存する。

1. **断面の横基準が幅の合計から出ている。** `geometry/section.cpp:171` と `:282` は
   `lateral = -total_width * 0.5` で断面を総幅の中央に置く。端部構造の幅を変えると車道が
   中心線に対して動く。edge datum を保持するには断面へ明示的な横基準を持たせる必要がある。
2. **`BoundaryProfile` が 2 点の斜面しか表せない。** `derive_boundaries` は boundary ごとに
   前後 2 sample しか出さない。溝底・垂直面・上面・受け部・地中面を区別できない。edge datum
   相対座標の閉じた断面 polygon が要る。
3. **交差点が独自に中心を再計算する。** `geometry/junction.cpp:249` の `section_center_m` は
   道路外端 2 点の中点。端部構造を足すと外端が動き、junction 側の基準も動く。segment の datum を
   接続 gate 経由で junction へ渡す経路が要る。同ファイルは side boundary が 2 個を超えると
   `NotImplemented` で拒否するので、複数点の断面はそのままでは通らない。
4. **接続が断面 style の完全一致を要求する。** `connection_geometry_from_gates` は
   `surface_styles` が違うと `NotImplemented`。片側だけ溝がある、左右で溝が違う、という組合せは
   corner で通らない。
5. **遷移が幅を線形補間する。** `interpolate_section` は strip / boundary の幅を線形に混ぜるので、
   溝が幅 0 から生えて途中で不正な断面になる。端部構造は遷移で連続かどうかを別に決める。
6. **archive に置き場がない。** version 11 の boundary field は
   `.boundary_id .role .width_m .height_m .marking.*` だけ。断面 polygon、面ごとの勾配、socket、
   地中面を保存する field はない。version 12 と migration 方針を先に決める。
7. **`Mesh` に法線と UV がない。** `derived_types/derived_road.hpp:54` は頂点と index だけを持つ。
   L 字溝の hard edge は表現できない。**現状 hard edge は未対応。** Unreal へ出す前に片付ける。

上のうち 1 と 3 は「端部構造 profile を変えても道路中心線が動かない」を満たすために両方要る。
2 が済むまで 4・5 は評価できない。6 は 2 の形が決まってから。

## 却下

再提案する場合は、ここに書いた理由が変わったことを示す。

- 車両のroute choice・右左折判断: 交通対応時の導出物。既存のlane geometry pathへ交通判断を混ぜない
- アウトインアウト: 道路境界を動かす場合と車両経路だけを動かす場合を分けられていない
- `BoundaryProfile`の丸み種別: 未使用のjoin欄を先に置かない。丸み編集を実装するときに追加する
- `NodeConnectionPolicyOverride`の追加値: Auto / ForcePassThrough / ForceCorner / ForceJunction以外を先に定義しない
- 高架壁と地上curbの自動変換: 明示的な構造切替がなければunsupportedのままにする
- 交通・信号・5叉路以上の自動整形・複雑車線分岐・短距離多重遷移・排水計算: 全phaseで非目標

## 保存互換性

手動線・手動面・lane connection・boundary continuationは `SavedRoadGraph` に残り、archive version 11 で読み書きできる。
標準UIとpublic APIから外しただけで、既存workspaceは開ける。保存fieldの削除は別途migration方針を決めてから行う。
