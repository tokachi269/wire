# Generation naming

ドメインをまたいで同じ意味を持つ動詞だけを統一する。ドメイン固有の名詞は無理に共通化しない。この文書と各ドメイン文書が矛盾した場合は、この文書の動詞定義を優先する。

## 動詞

| 動詞 | 意味 | 例 |
|---|---|---|
| `generate` | 正本からドメインの全派生状態、または個別の派生形状・出力を作る | `generate_road(graph)`、`generate_junction_geometry` |
| `derive` | 正本または前段の結果から決定論的に導出する | `derive_segments`、`derive_markings` |
| `resolve` | 複数の入力・policy・overrideから1つの意味を決める | `resolve_connections` |
| `emit` | 解決済み幾何を頂点・index・描画データへ変換する | `emit_surface_mesh` |
| `validate` | 入力または生成結果を検査する | `validate_derived` |

`materialize`は生成系の用語として使わない。`build`は道路・電線の派生生成処理の名前に使わない(CMakeターゲットや通常のオブジェクト構築を指す`build`は対象外)。

工程ごとに型・表・ファイル・カウンタを増やさない。非公開関数への分割は自由とする。

## domain固有語(統一しない)

| wire | road |
|---|---|
| Pole / Port / Span / Bundle / Row / Lane / BackboneGraph | RoadNode / RoadSegment / RoadLayoutTemplate / RoadLayoutTransition / Junction / ConnectionGate / ApproachKey / Marking |

同じ語を別の意味で使わないことだけを守る。意味が違うものを共通名に寄せない。

## 共通する構造上の語

| 語 | 意味 |
|---|---|
| authoritative | 保存する識別情報とユーザー決定値 |
| derived | 正本から再構築できる値。保存しない |
| trial | 編集を適用した一時状態。成功時だけcommitする |
| unsupported | 対応範囲外。fallbackせず明示的に失敗する |

## 失敗分類

`CommitFailureCategory`はroadとwireで名前・値・意味を一致させる。別定義のまま同じ集合を保つ。enumの名前と、各失敗をどの値に分類するかを一致させる。

| 分類 | 意味 |
|---|---|
| `kRequirementConstraint` | 製品仕様として禁じている操作 |
| `kInvalidInput` | 外部入力が不正(ID不在、非有限値、enum範囲外、寸法が非正) |
| `kNotImplemented` | 入力は正しいが現在の対応範囲外。`unsupported`と同義 |
| `kStateConflict` | 現在の状態と両立しない |
| `kInternalError` | 正しい正本から派生が欠ける等の内部不整合 |

対応予定の入力で起きた幾何生成の失敗を`kNotImplemented`へ分類してはならない。値を同じに保つためだけの共通型は作らない。両ドメインが同じ集合を維持していることをテストで固定する。

## 表記

コードに実在する名前は原文のまま書き、`` ` ``で囲む。訳さず、カナにもしない。型、関数、フィールド、enum値、ファイル名、テスト名、設定キーがこれにあたる。

文中の一般名詞は日本語で書く。英語の一般名詞を助詞でつながない。同じ語を英字とカナで混ぜて書かない。

開発で定着した技術用語や検証機構の名前は、無理に訳さない。arch lint、assertion、oracle、decision ownerなどは英語のまま使う。同じ仕組みに複数の訳語を付けず、文書の正本、実装責務、データの所有元は文脈に応じて区別する。

外来語は、日本語として定着しているものだけカナで書く。テスト、データ、キャッシュ、テンプレート、ドメイン、シナリオ、ユーザーはカナとする。

`` ` ``とコードフェンスは、コピーして実行・検索できる文字列にだけ使う。日本語の説明や図を入れない。構造を示すときは表か箇条書きにする。

同じ語を両方の表記で使っている語は次で統一する。英語の用語句の一部(`route scope`、`layout settings`、`session draft`等)はその句ごと英語のまま書く。

| 文中で単独の名詞として書くとき | 使う表記 |
|---|---|
| test、data、model、template、domain、scenario、cache、user、code、case、file、schema、repository、version、metadata、class、project | カナ(テスト、データ、モデル、テンプレート、ドメイン、シナリオ、キャッシュ、ユーザー、コード、ケース、ファイル、スキーマ、リポジトリ、バージョン、メタデータ、クラス、プロジェクト) |
| route、layout、session、source、build、error、rule、click、command、tool、catalog、view、region、panel、pointer、matrix | 英語 |
| document | 文書 |

viewerの画面を説明する文では、UIの部品名(パネル、リージョン、レイアウト)はカナで書いてよい。
