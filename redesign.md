# Backbone生成パイプライン再設計仕様 改訂版

## 目的

backbone から span 生成までの流れを、処理順と責務が明確な形に作り直す。
現行の `plan / commit / execute / materialization / authority / decision` などの抽象語中心の構造は前提にしない。
生成対象として自然な概念で整理する。

この仕様では、次を重視する。

* 何をどの順番で確定するか
* どの段が何を入力にして何を出力するか
* 後段が前段の意味を作り直さないこと
* pole を主役にしないこと
* grouped span 側で pair や side を勝手に発明しないこと
* backbone 自体が後段の共通基準を持つこと

---

## スコープ

対象は backbone 入力から grouped span 生成まで。

含むもの:

* backbone path 入力
* backbone node / edge の確定
* junction での役割決定
* pole facing 解決
* bundle/span 生成
* span ごとの layout rule 保存

含まないもの:

* viewer
* render
* attachment template のデータ定義自体
* CoreState 全体の再設計
* curve 表現の全面見直し

---

## backbone が持つ正本

backbone は単なる node / edge の集合ではなく、**後段が向きや pair を再発明しなくて済むための正本**を持つ。

### backbone が持つもの

* nodes
* edges
* junctions
* **input direction**

  * backbone path を入力した向き
  * クリック順の方向を正とする
* **build direction**

  * 生成で使う正方向
  * 初期値は input direction
  * 必要なら外部指定や内部ルールで反転可能
* main pair
* cross pair
* junction roles

### backbone が持たないもの

* final pole facing
* lane order
* final port order
* socket の resolved 結果
* materialization 後の world geometry

重要:

* 順序番号を正本にしない
* 正方向は `input direction` と `build direction` で持つ
* downstream は local geometry から正方向を再生成しない

---

## 処理順

処理順はこれを正とする。

1. backbone node / edge を確定する
2. backbone の正方向を確定する
3. junction の役割を決める
4. pole facing を解決する
5. bundle/span を生成する
6. span ごとの layout rule を保存する
7. 後段は保存済み rule を使って geometry / world を作る

重要:

* 4 の後に pole facing を変えてはいけない
* 5 の後に pair / side / lowering の意味を変えてはいけない
* 7 は consume のみで、意味決定をしない

補足:

* pole facing は主工程ではなく、junction roles と backbone の build direction から解く従属値
* node / edge の順序番号は不要
* 正方向は input/build direction を使う

---

## 主な構成

### BackbonePipeline

本流の入口。

工程は短い名前で持つ。

* `prepare()`
* `check()`
* `build()`

`prepare()` の中で前提を揃える。
`check()` は入力不備の確認だけに使う。
`build()` の中で生成まで進める。

### BackboneBuilder

backbone path から node / edge と方向基準を作る。

### JunctionRoleResolver

各 junction で

* ThroughMain
* SideBranch
* CrossUnderpass
* CornerContinuation
  を決める。

### PoleFacingResolver

junction role と外部指定と backbone の build direction から pole facing を解決する。
pole facing は主工程ではなく従属値として扱う。

### BundleSpanBuilder

backbone、junction role、pole facing を使って bundle/span を生成する。
grouped span 系の枝処理はこの中で使う。

### SpanLayoutRuleBuilder

生成した span ごとに、後段が使う端点 rule を保存する。

---

## 各段の責務

### 1. BackboneBuilder

入力:

* backbone path
* node spec
* pole type
* interval
* 向き指定

出力:

* backbone nodes
* backbone edges
* node metadata
* pole/node ids
* input direction
* build direction

責務:

* backbone の node / edge を確定する
* 必要な pole を state に作る
* input direction を記録する
* build direction を確定する

禁止:

* junction role を決める
* pole facing を決める
* pair / lane / span を決める

---

### 2. JunctionRoleResolver

入力:

* backbone nodes / edges
* build direction
* junction raw facts

出力:

* junction roles
* main pair
* cross pair

責務:

* 各 junction の隣接方向に役割を付ける
* through/main/branch/cross/corner を決める
* main pair / cross pair をここで正本化する
* 役割決定はここで完結する

禁止:

* pole facing を決める
* lowering 可否を geometry ベースで丸める
* grouped span 向けの pair companion を downstream 前提で足す

---

### 3. PoleFacingResolver

入力:

* junction roles
* main pair / cross pair
* build direction
* 外部 facing 指定

出力:

* pole facing

責務:

* pole ごとの facing を1本決める

優先順位:

1. 外部指定
2. cross は本線向き
3. 分岐でなければ二等分線
4. それ以外のみ単純 fallback

重要:

* pole facing は主役ではない
* backbone と junction の正本から解く派生値
* 重い planner にしない

禁止:

* pole を主役にして junction role を逆算する
* pair / lane / span を pole facing から決める
* pole facing を backbone の正本の代わりに使う

---

### 4. BundleSpanBuilder

入力:

* backbone
* junction roles
* pole facing
* bundle spec

出力:

* generated spans
* lane/port assignment
* endpoint layout rules

責務:

* grouped span 系の処理を使って実 span を生成する
* lowering / side / pair / lane / port をここで決める

禁止:

* junction roles を再解釈する
* explicit pair がないのに pair を発明する
* lane preparation で side sign を発明する
* build direction を local geometry で上書きする

---

### 5. SpanLayoutRuleBuilder

入力:

* generated spans
* endpoint layout rules
* lane assignment

出力:

* span layout rules

責務:

* 各 span の start/end 端点 rule を保存する
* 後段がこれを読む

禁止:

* 新しい意味決定
* rule family の補正
* pair family の再解釈
* junction を見て pair rule を寄せ直すこと

---

### 6. 後段 geometry/world 処理

入力:

* saved span layout rules

出力:

* layout
* curve
* bounds
* world geometry

責務:

* 保存済み rule を使って geometry/world を作る

禁止:

* pair rule の正規化
* junction を見て rule を寄せ直す
* authority/decision の作り直し
* build direction や pair family の再解釈

---

## grouped span 枝の方針

grouped span 側では次を正とする。

* explicit pair だけを使う
* explicit pair が無ければ pair を作らない
* borrowed pair を downstream で作らない
* lane preparation は side sign を発明しない
* materialization は意味を作り直さない
* local geometry から backbone 正方向を再発明しない

### grouped span 内の役割

* endpoint height rule を決める
* endpoint side / pair / axis を決める
* lane / row を決める
* span を作る

---

## 命名方針

### クラス名

* `BackbonePipeline`
* `BackboneBuilder`
* `JunctionRoleResolver`
* `PoleFacingResolver`
* `BundleSpanBuilder`
* `SpanLayoutRuleBuilder`

### 工程名

* `prepare`
* `check`
* `build`

詳細処理は補助メソッド側で具体名にする。

* `buildBackbone`
* `resolveJunctionRoles`
* `resolvePoleFacing`
* `buildBundles`
* `saveSpanLayoutRules`

禁止:

* `Authority`
* `Decision`
* `Projection`
* `Materialization`
* `Commit`
* `Execute`
* `Planner`
* `Policy`

---

## データ構造方針

抽象語ではなく、対象物ベースで持つ。

例:

* `BackboneGraph`
* `JunctionRoles`
* `PoleFacing`
* `EndpointLayoutRule`
* `SpanLayoutRules`
* `InputDirection`
* `BuildDirection`

避ける例:

* `AuthorityView`
* `DecisionSeed`
* `ProjectionView`
* `MaterializationInputs`

---

## テスト方針

正本は仕様。
テストはその検証器とする。

### 1. 仕様テスト

* build direction は input direction から初期化される
* pole facing は junction roles の後に決まる
* materialization は pair family を変更しない
* grouped span consumer は explicit pair が無ければ pair を作らない

### 2. 症状テスト

* T で main と branch の高さ関係
* cross center が本線基準で揃う
* lane twist が起きない
* borrowed pair を勝手に使わない
* 向き反転が build direction に従って一貫する

### 3. 移行テスト

* capture replay
* accepted visual cases

注意:

* 旧テストはそのまま正本扱いしない
* 設計と衝突するテストは見直し対象にする

---

## 実装方針

全面上書きではなく、新系統を横に作る。

* `BackbonePipeline`
* `BackboneBuilder`
* `JunctionRoleResolver`
* `PoleFacingResolver`
* `BundleSpanBuilder`
* `SpanLayoutRuleBuilder`

この新系統を入口からつないで比較可能にする。
通ったら旧 generation 本流を物理削除する。

---

## 完了条件

* 処理順が backbone → junction → facing → spans → rules 保存 で明確
* backbone が input/build direction を持つ
* pole facing が主工程ではなく従属値になっている
* grouped span で pair / side を勝手に発明しない
* 後段が意味を作り直さない
* クラス名とメソッド名を見て流れが分かる
* 仕様テストと症状テストが通る
* 旧本流を削除できる見込みが立つ