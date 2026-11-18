# Repository architecture

この文書では、ドメインに依存しないリポジトリ共通の設計契約を定義する。wire固有のtopology、placement、curve、モデル規則は`wire/architecture.md`と`wire/models.md`、road固有の契約は`road/architecture.md`を正本とする。

ドメイン文書は固有語彙を追加できるが、本書で定める正本の境界、依存方向、トランザクション規則を弱めてはならない。

## Domain着手条件

新しいドメイン、subsystem、pipelineを実装する前に、同じリポジトリの既存実装を調査する。製品コードの実装より先に、ドメイン文書へ次の対応表を作る。

| 項目 | 記載内容 |
|---|---|
| 既存の決定者 | 類似するauthority、操作、pipeline、utilityと所在 |
| 新ドメインでの対応 | 再利用する型・処理、または新設する担当箇所 |
| 差異 | 既存実装をそのまま使えない意味上の理由 |
| Input | 外部要求、セッションの一時データ、adapter payload |
| 正本 | 保存する識別情報とユーザー決定値 |
| 派生 | 正本から再構築できる値とキャッシュ |
| 操作担当箇所 | 正本を変更できるAPIと原子性 |
| Decision ownership | Decision、Owner、Inputs、Authoritative出力、downstream consumers |

対応が未決定の項目を現在の実装、テスト、UIから推測して埋めない。類似実装を再利用しない場合は、意味と責務がどう異なるかを記録する。名前の違いだけを理由にしない。

## State layers

状態は次のlayerに分ける。

| layer | 内容 | 保存 |
|---|---|---|
| Input | 外部request、import値、操作引数 | 原則保存しない |
| Session draft | 未確定pick、preview、操作途中のhandle | 保存しない |
| Authoritative | 識別情報、topology、ユーザーが確定した意味入力 | 保存する |
| Derived | connectivity評価、layout、curve、mesh、marking、bounds | 保存しない |
| Runtimeキャッシュ | index、lookup、renderキャッシュ、計測値 | 保存しない |

正本の変更は、その状態を管理する操作APIだけが行う。派生、viewer、adapter、テストから正本へ書き戻さない。loadは正本を復元した後、通常のbuild経路で派生と実行時キャッシュを再構築する。

## Decision ownership

ドメインarchitectureは、意味を決める責務を必要に応じて次の標準形で記録する。

```text
Decision -> Owner -> Inputs -> Authoritative output -> downstream consumers
```

1つの判断を担当する箇所は1つに限定する。下流の利用側は、正本として決定した出力を使う。同じ入力から判断をやり直してはならない。リポジトリ共通文書は標準形だけを定め、ドメイン固有の判断の一覧と担当箇所は、各ドメインの設計文書に記載する。

## Identity and decisions

- topology、接続、所有関係はIDで表す。位置の近接、名前、配列順、生成順から識別情報を推測しない。
- 同じ意味の決定者は1つにする。下流は決定済み結果を消費し、同じ入力から再判定しない。
- 種別enumは保存すべきユーザー概念にだけ使う。現在の派生表現を分類するためのenumを正本へ追加しない。
- context、pick、projectionは判断入力であり、明示された識別情報の代用や保存対象にしない。
- 不足した識別情報や未定義状態をfallbackで補完しない。検証またはunsupportedとして返す。

## Operations and build

各ドメインは操作前状態と操作の組合せを操作意味論文書で定義する。未定義セルは製品コードの実装へ進めない。

正本変更は試行用の状態へ適用し、統一build入口で派生とindexを再構築し、全段階の検証が成功した後にだけ本状態へ反映する。失敗時は正本、派生、セッションの一時データの消費状態を変更前と一致させる。

pipeline段階は責務で分ける。操作固有分岐をmaterializationやviewerへ持ち込まない。下流で不足が判明した場合、上流正本を補正せず、担当箇所へ戻すかunsupportedにする。

## Persistence

- 識別情報とauthoritative値だけを保存する。保存互換を保証するドメインだけがversioningを持つ。
- 未知フィールド、欠落、重複、truncation、構文不正を拒否する。
- loadは新しい試行用の状態でparse、index構築、通常build、検証を完了してからcommitする。
- save -> load -> saveはauthoritative byte一致を基本契約とする。
- versioningを持たないドメインは現行スキーマだけを読み書きし、互換分岐を置かない。将来、保存互換を保証する時点でversioningと明示migrationを導入し、幾何や名前から欠損識別情報を推測しない。

## Geometry and materialization

編集入力とcanonical幾何を分ける。UIのtool modeやpreview表現を、そのまま正本のkindへしない。curve、frame、station、surface、meshは各ドメインのcanonical authorityから一方向に導出する。

materializationはcoreが決定したworld-space幾何とmaterial意味論を消費する。viewer側で高さ、接続、material、可視性の意味を再判定しない。

## Shared code

共有実装は、2つ以上の実消費者があり、意味と失敗契約が一致した後に`foundation`へ移す。将来共有できそうという理由だけで抽象化しない。一方、既存の共有実装を検索せずにドメイン内へ重複実装を作らない。

共有候補をドメイン内へ一時配置する場合は、担当箇所、抽出条件、非対象をドメインarchitectureへ記載する。

## Dependency direction

```text
domain authoritative types
  -> domain operations / build
  -> domain derived geometry
  -> adapter / viewer / export
```

ドメイン間の直接依存は設計文書で明示された場合だけ許可する。共有概念は一方のドメインを経由せず、意味が安定してからfoundationへ置く。adapterとviewerはドメイン正本を所有しない。
