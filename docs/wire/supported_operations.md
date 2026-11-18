# Wireの対応操作

本書は、明示的なWire確定操作の対応範囲を定める。ポインターのガイドでは、backbone生成、model assembly、scene出力を実行しない。

## 通常描画で対応する構成

通常の操作では、新しい独立route、最後に生成した終点からの延長、session終了後の新しい独立route、明示的なPole終点の再利用、対応済みmidair branch pick、T字・cross生成、modelを含む複数Bundle生成に対応する。routeの完了後は、一時anchor、preview、pending support状態を残さない。

実WASMの回帰fixtureは、modelを含む独立した12本のrouteを作り、それぞれ2区間を確定する。previewの原子性と最終scene出力を検証する。ViewerActionsでは別途、Click、Enter、Escapeの繰り返しと、toolを切り替えるsessionを検証する。

## 製品要件による制約

`RequirementConstraint`に分類するのは、要求されたmidair branchをBundle templateが禁止する場合など、明示的なtemplate方針またはtopology方針に限る。古いrow、model fixtureの欠落、解決できない保存binding、session状態の不一致は、製品要件による制約ではない。

## 失敗時の責務

### CoreのEditResult

`EditResult`は人間向けの`error`文字列に加えて、機械可読な`CommitFailureCategory`と`reason_code`を返す。

旧説明にあった次の3分類は、現在のenum値ではない。外部入力不正、未対応、内部不整合という診断の性質を表していた。

- `kValidation`: 外部入力が不正で、ユーザー入力またはadapter payloadを直せばよいもの。
- `kUnsupported`: 入力は読めたが、当時の仕様で扱わないもの。旧説明では分類に迷う既存errorもここへ寄せていた。
- `kInternal`: 保存済み正本や派生再構築の整合が壊れており、通常操作では起きてはいけないもの。

現在は下表と[確定操作の失敗分類](../editor/commit_failure_categories.md)の5分類を使う。Coreは`core_edit_types.hpp`の`ClassifyCommitFailure`で分類し、未知のerrorは`InternalError`とする。旧説明の曖昧なerrorを未対応へ寄せる方針は適用しない。

既存の`error`文字列は診断情報として維持する。境界adapterは`effective_failure_category()`で分類済み値を読み、表示層は文字列prefixを再解釈しない。下表は描画操作の境界で失敗を担当する箇所を示し、Coreの分類とは別の正本を作らない。

| 担当 | 入力条件 | 分類 | 通常描画への影響 |
|---|---|---|---|
| anchor・要求の検証 | 対象IDの欠落、非有限値の点、anchorと同じ終点 | `InvalidInput` | 明示的なClickまたはEnter時だけ表示する |
| template方針 | midair node/branchやtopology方針による明示的な禁止 | `RequirementConstraint` | 許可されたtemplateまたは対象を使う |
| topology/layoutの解決 | 意味のあるtopologyだが未実装 | `NotImplemented` | 入力不正ではなく、実装の保留事項として扱う |
| 描画session | 参照するanchorやversionが古くなった | `StateConflict` | anchorを更新して再試行する |
| backbone/model assembly | 保存binding、row、fixture、生成endpointが不変条件に違反した | `InternalError` | 対応済み経路の不具合として扱う |

過去のroute繰り返し生成の失敗は`InternalError`だった。局所生成のcontextが接続済みendpointのrow frameを置き換え、model assemblyがfixture dataの不整合を報告していた。現在はendpoint contextを維持し、modelを含むroute繰り返し生成のfixtureで再発を防ぐ。

## sessionの動作

最初のClickでanchorを受理する。次のClickで1区間を確定し、解決済みの終点を継続anchorとして保持する。Enterは表示中のガイドを確定してsessionを終了する。Escapeは一時状態だけを破棄する。toolの切り替えでは、それまでの一時sessionを終了する。全ての主操作は明示的な結果を返し、描画可能なクリックを黙って無視しない。
