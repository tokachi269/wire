# Backbone / bb2 mainline 設計

## 目的

`bb2` を supported backbone generation の mainline candidate として扱う。

この文書は現在の設計契約だけを示す。過去の作業履歴、C 番号、実行済み plan、日付付き記録は `docs/archive/redesign_history_2026-06.md` に退避する。

## 現行方針

* `GenerateFromBackboneSpec` は supported request を `bb2` で処理する。
* v1 fallback は許可しない。未対応入力は明示的に `unsupported` とする。
* 正本、派生、表示を混ぜない。
* 下流は上流の意味を再判断しない。
* existing span / layout / seed / curve / port 位置 / materialization result から topology、pair、row、port identity、lowering、draw を推測しない。
* supported scenario が増えた場合だけ進捗とする。docs / tests の増加だけでは進捗扱いにしない。
* `bb2` rename は、v1 / recalc / support-layout 依存が本流から外れるまで保留する。

## レイヤと決定者

| レイヤ | 決定者 | 責務 | 禁止 |
|---|---|---|---|
| topology | `SavedBackboneGraph` | node / edge / edge_bundle / binding / frontier を保存する | span / layout / seed から graph を復元する |
| connectivity | `pairs make(graph)` | link / pair / open / row を一度だけ決める | T / cross / branch enum を作る |
| placement | support group / row placement | row separation、vertical order、lowering offset を決める | topology / pair / open / row を変える |
| port identity | saved row-port binding | row key / lane / bundle-compatible scope から port を解決する | 位置近似や existing layout で port を探す |
| span policy | saved span binding preflight | same edge_bundle + lane の重複を mutation 前に拒否する | 既存 span geometry を比較する |
| rules | `SpanLayoutRules` | span の layout intent を保存する | authority / seed / projection を作る |
| layout | `SpanLayoutEntry` | rules と support group を world endpoint へ写す | lowering を再判断する |
| geom | curve / bounds | layout から deterministic に派生する | route geometry から意味を補完する |
| draw | visual / render cache | layout / geom の結果を最小表示へ転写する | topology / pair / row / lowering を判断する |

## bb2 mainline v0

### 対応範囲

`bb2` は v1 fallback なしで通る範囲だけを supported とする。

* 2点以上の route。
* polyline。
* new pole。
* saved graph を持つ existing pole。
* new / saved midair node。
* building / ground / supported segment-pick route point。
* single bundle / multiple bundles。
* selected bundle policy。
* exact / range bundle count の明示指定。
* pole band による port height。
* lateral offset。
* simple deterministic avoid detour。
* existing junction への branch / cross 追加。ただし T / cross / branch kind は作らない。
* pass-through / lowering intent と、その layout / geom / draw 反映。
* generated span の direct derive。recalc dirty queue は通さない。

avoid support は simple deterministic detour に限定する。general routing、collision solving、obstacle avoidance ではない。

### 非対応範囲

次は supported ではない。

* SavedBackboneGraph が無い existing scene を暗黙に取り込むこと。
* span / layout / seed / curve / port 位置から graph を推測する migration。
* general routing / collision solving / obstacle avoidance。
* support arm / insulator / attachment の本格 semantic。
* v1 grouped span engine。
* v1 support-layout authority / seed / projection を bb2 の中心に戻すこと。
* recalc / materialization を bb2 generation の通常経路にすること。
* draw が topology / connectivity / lowering を決めること。

### 出力保証

supported request の生成直後に、bb2 は次を保存する。

* `SavedBackboneGraph` の node / edge / edge_bundle / binding。
* `SpanLayoutRules`。
* `SpanLayoutEntry`。
* curve / bounds。
* 最小 visual / render cache。

これらは recalc 後に揃う副作用ではなく、bb2 generation の出力である。

## 生成フロー

現在の `bb2` は次の順序を守る。

1. `prepare`
2. `check`
3. `pairs make(graph)`
4. duplicate / span binding preflight。duplicate same edge_bundle + lane はここで拒否する。
5. intent / support group
6. `emit` による topology object 生成
7. saved backbone graph への保存
8. rules 保存
9. layout 保存
10. geom 保存
11. draw 保存

mutation 後にしか分からない failure を通常経路にしない。避けられない場合は fallback や rollback で隠さず、未解決の設計衝突として扱う。

## 正本ルール

`SavedBackboneGraph` は topology authority。

* node は saved backbone node。support object と position を持つ。
* edge は node 間の物理 segment。
* edge_bundle は edge 上に載る bundle 単位。
* span binding は edge_bundle + lane から generated span へ結ぶ。
* port binding は row key + lane + bundle-compatible scope から materialized port へ結ぶ。
* frontier は pole / span などから incident graph を引くための index。

`pairs make(graph)` は connectivity authority。

* link は graph edge の incident。new link と context link を区別する。
* pair は同じ route 上で連続する incoming / outgoing incident。
* open は接続相手を持たない incident。
* row は port placement unit。pair または open を source に持つ。

support group は placement authority。

* materialized row separation、group axis、vertical order、lower offset を決めてよい。
* topology、pair / open / row、edge continuity、context link の生成対象化を決めてはいけない。

## 既存物の扱い

既存物は再利用制約または context であり、意味決定の正本ではない。

許可:

* saved graph node として existing pole / midair / building node を使う。
* saved graph incident edge を context link として使う。
* saved row-port binding に一致する port を使う。
* saved edge_bundle / span binding で duplicate を検出する。

禁止:

* existing span から pair / row / lowering を決める。
* existing layout / seed から side / support / pass-through を決める。
* 位置が近い port / span を同一物として採用する。
* context link を生成・保存対象にする。

## derive / post-edit refresh 境界

Direct derive は、saved bb2 rules と ports から layout / geom / draw を再導出する入口。

layout は `support_world` と `endpoint_world` を分ける。

* `support_world`: port / support の元位置。
* `endpoint_world`: lowering などを反映した wire endpoint。

守ること:

* topology / pair / row を決めない。
* support-layout contract / seed / projection を読まない。
* recalc dirty queue / materialization を通らない。
* generated span の post-edit refresh を bb2 direct output に戻す。

Direct derive は recalc を強化するためではなく、bb2 が recalc に戻らないための足場。

## viewer / public query 境界

viewer normal path は bb2 neutral outputs を読む。

通常表示で読むもの:

* saved backbone graph。
* `SpanLayoutRules`。
* `SpanLayoutEntry`。
* curve / bounds。
* visual / render cache。

`inspect_support_layout` は manual debug / v1 inspection 用に限定する。viewer normal path が旧 support-layout contract を読む場合は削除候補とする。

public backbone query は saved graph を正本として読む。旧名の API が残っていても、span-derived fallback を戻してはいけない。

## v1 / recalc / support-layout 残存境界

recalc は bb2 の通常生成経路ではない。

残ってよい範囲:

* v1 専用。
* 旧テスト。
* 手動 debug。
* validation 専用。

残してはいけない範囲:

* bb2 generation。
* viewer normal path。
* supported scenario の post-edit 更新。

残存依存は family 単位で判断する。詳細な caller、削除可否、次の切断先は `docs/backbone_legacy_map.md` を正とする。

| 分類 | 判断 |
|---|---|
| A | bb2 本流から未使用。物理削除候補 |
| B | viewer / public query が読む。先に neutral output へ移す |
| C | tests だけが読む。制約を bb2 構造へ移植してから退役 |
| D | v1 専用として隔離 |
| E | bb2 未対応 scenario のため残っている。supported 化か unsupported 固定を決める |

現在の大きい残件:

* recalc pipeline。
* support-layout materialization。
* support-layout authority / seed / projection inspection。
* 旧テストの v1 実装詳細 assert。
* manual debug / capture replay の旧 inspection 依存。

## 禁止事項

* v1 fallback を作る。
* `legacy` を通常経路名にする。
* `fallback` / `infer` / `guess` / `rescue` 的な補完経路を作る。
* `manager` / `helper` / `processor` で責務を曖昧にする。
* v1 由来の `authority` / `seed` / `projection` を bb2 中心語に戻す。
* old grouped span engine を bb2 の本流に戻す。
* draw で topology / pair / row / lowering を判断する。

## 旧テストの扱い

旧テストは捨てない。ただし旧実装詳細を bb2 の完了条件にしない。

bb2 に移植する制約:

* unsupported 入力で state が変わらない。
* generated object id が一貫して引ける。
* endpoint / layout / curve / bounds / draw が欠けない。
* duplicate で増殖しない。
* 同じ input で deterministic。
* public query / viewer が必要な情報を読める。

bb2 に移植しない期待値:

* authority / seed / projection object の存在そのもの。
* grouped span engine の内部順序。
* recalc / materialization の副作用。
* v2 で禁止した fallback 前提の期待値。

## 次の判断

次の作業は、supported scenario が増えるか、v1 normal-path 依存が減る場合だけ採用する。

優先順:

1. viewer normal path から残った旧 inspection 依存を切る。
2. capture / debug が必要な情報を neutral output で読めるようにする。
3. old test family から bb2 に必要な制約だけを移植する。
4. v1 専用化できた旧経路を物理削除する。
5. 明確に failing な practical scenario を 1 つ supported にする。

docs / tests だけ増える作業が続く場合は停止する。
