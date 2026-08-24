# Repository architecture

この文書はdomainに依存しないリポジトリ全体の設計契約である。
wire固有のtopology、placement、curve、model規則は`wire/architecture.md`と`wire/models.md`、
road固有の契約は`road/architecture.md`を正本とする。domain文書は固有語彙を追加できるが、
本書のauthority境界、依存方向、transaction規則を弱めてはならない。

## Domain着手条件

新しいdomain、subsystem、pipelineを実装する前に、同じリポジトリの既存実装を調査する。
production実装より先に、domain文書へ次の対応表を作る。

| 項目 | 記載内容 |
|---|---|
| 既存の決定者 | 類似するauthority、operation、pipeline、utilityと所在 |
| 新domainでの対応 | 再利用する型・処理、または新設するowner |
| 差異 | 既存実装をそのまま使えない意味上の理由 |
| Input | 外部要求、session draft、adapter payload |
| 正本 | 保存するidentityとユーザー決定値 |
| 派生 | 正本から再構築できる値とcache |
| 操作owner | 正本を変更できるAPIとatomicity |
| Decision ownership | Decision、Owner、Inputs、Authoritative output、downstream consumers |

対応が未決定の項目を現在の実装、test、UIから推測して埋めない。
類似実装を再利用しない場合は、名前の違いではなく意味と責務の違いを記録する。

## State layers

状態は次のlayerに分ける。

| layer | 内容 | 保存 |
|---|---|---|
| Input | 外部request、import値、操作引数 | 原則保存しない |
| Session draft | 未確定pick、preview、操作途中のhandle | 保存しない |
| Authoritative | identity、topology、ユーザーが確定した意味入力 | 保存する |
| Derived | connectivity評価、layout、curve、mesh、marking、bounds | 保存しない |
| Runtime cache | index、lookup、render cache、計測値 | 保存しない |

正本を書き換えてよいのは、それを所有するoperation APIだけである。
派生、viewer、adapter、testから正本へ書き戻さない。loadは正本を復元した後、通常のbuild経路で
派生とruntime cacheを再構築する。

## Decision ownership

domain architectureは、意味を決める責務を必要に応じて次の標準形で記録する。

```text
Decision -> Owner -> Inputs -> Authoritative output -> downstream consumers
```

同じDecisionのOwnerは1つだけとする。downstream consumerはAuthoritative outputを消費し、
同じInputsからDecisionを再判定しない。repository共通文書は標準形だけを定め、domain固有の
Decision一覧やowner名は各domain architectureに置く。

## Identity and decisions

- topology、接続、所有関係はIDで表す。位置の近接、名前、配列順、生成順からidentityを推測しない。
- 同じ意味の決定者は1つにする。下流は決定済み結果を消費し、同じ入力から再判定しない。
- 種別enumは保存すべきユーザー概念にだけ使う。現在の派生表現を分類するためのenumを正本へ追加しない。
- context、pick、projectionは判断入力であり、明示されたidentityの代用や保存対象にしない。
- 不足したidentityや未定義状態をfallbackで補完しない。validationまたはunsupportedとして返す。

## Operations and build

各domainは操作前状態と操作の組合せをoperation semantics文書で定義する。
未定義セルはproduction実装へ進めない。

正本変更はtrial stateへ適用し、統一build入口で派生とindexを再構築し、全stageのvalidation成功後にだけ
本stateへcommitする。失敗時は正本、派生、session draftの消費状態を変更前と一致させる。

pipeline stageは責務で分ける。operation固有分岐をmaterializationやviewerへ持ち込まない。
下流で不足が判明した場合、上流正本を補正せず、ownerへ戻すかunsupportedにする。

## Persistence

- identityとauthoritative値だけをversion付きで保存する。
- 未知field、欠落、重複、truncation、構文不正を拒否する。
- loadは新しいtrial stateでparse、index構築、通常build、validationを完了してからcommitする。
- save -> load -> saveはauthoritative byte一致を基本契約とする。
- schema migrationはversionごとの明示処理とし、geometryや名前から欠損identityを推測しない。

## Geometry and materialization

編集入力とcanonical geometryを分ける。UIのtool modeやpreview表現を、そのまま正本のkindへしない。
curve、frame、station、surface、meshは各domainのcanonical authorityから一方向に導出する。

materializationはcoreが決定したworld-space geometryとmaterial semanticsを消費する。
viewer側で高さ、接続、material、可視性の意味を再判定しない。

## Shared code

共有実装は、2つ以上の実消費者があり、意味と失敗契約が一致した後に`foundation`へ移す。
将来共有できそうという理由だけで抽象化しない。一方、既存の共有実装を検索せずにdomain内へ重複実装を作らない。

共有候補をdomain内へ一時配置する場合は、owner、抽出条件、非対象をdomain architectureへ記載する。

## Dependency direction

```text
domain authoritative types
  -> domain operations / build
  -> domain derived geometry
  -> adapter / viewer / export
```

domain間の直接依存は設計文書で明示された場合だけ許可する。共有概念は一方のdomainを経由せず、
意味が安定してからfoundationへ置く。adapterとviewerはdomain正本を所有しない。
