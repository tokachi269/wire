# Generation naming

domainをまたいで同じ意味を持つ動詞だけを統一する。ドメイン固有の名詞は無理に共通化しない。
本書と各domain文書が矛盾した場合は本書の動詞定義を優先する。

## 動詞

| 動詞 | 意味 | 例 |
|---|---|---|
| `generate` | 正本からdomainの全派生state、または個別の派生形状・出力を作る | `generate_road(graph)`、`generate_junction_geometry` |
| `derive` | 正本または前段の結果から決定論的に導出する | `derive_segments`、`derive_markings` |
| `resolve` | 複数の入力・policy・overrideから1つの意味を決める | `resolve_connections` |
| `emit` | 解決済みgeometryを頂点・index・描画データへ変換する | `emit_surface_mesh` |
| `validate` | 入力または生成結果を検査する | `validate_derived` |

`materialize`は生成系の用語として使わない。`build`は道路・電線の派生生成処理の名前に使わない
(CMakeターゲットや通常のオブジェクト構築を指す`build`は対象外)。

工程ごとに型・Table・ファイル・カウンタを増やさない。
private関数への分割は自由とする。

## domain固有語(統一しない)

| wire | road |
|---|---|
| Pole / Port / Span / Bundle / Row / Lane / BackboneGraph | RoadNode / RoadSegment / RoadLayoutTemplate / RoadLayoutTransition / Junction / ConnectionGate / ApproachKey / Marking |

同じ語を別の意味で使わないことだけを守る。意味が違うものを共通名に寄せない。

## 共通する構造上の語

| 語 | 意味 |
|---|---|
| authoritative | 保存するidentityとユーザー決定値 |
| derived | 正本から再構築できる値。保存しない |
| trial | 編集を適用した一時state。成功時だけcommitする |
| unsupported | 対応範囲外。fallbackせず明示的に失敗する |

## 失敗分類

`CommitFailureCategory`はroadとwireで名前・値・意味を一致させる。別定義のまま同じ集合を保つ。
enum名だけを揃えるのではなく、どの失敗がどれに入るかを一致させる。

| 分類 | 意味 |
|---|---|
| `kRequirementConstraint` | 製品仕様として禁じている操作 |
| `kInvalidInput` | 外部入力が不正(ID不在、非有限値、enum範囲外、寸法が非正) |
| `kNotImplemented` | 入力は正しいが現在の対応範囲外。`unsupported`と同義 |
| `kStateConflict` | 現在のstateと両立しない |
| `kInternalError` | 正しい正本から派生が欠ける等の内部不整合 |

対応予定の入力で幾何生成に失敗したものを`kNotImplemented`へ逃がさない。
値を同じに保つためだけの共通型は作らない。両domainが同じ集合を維持していることをテストで固定する。
