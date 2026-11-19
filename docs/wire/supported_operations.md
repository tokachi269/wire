# Wireの対応操作

本書は、明示的なWire確定操作の対応範囲を定める。ポインターのガイドでは、backbone生成、モデルassembly、scene出力を実行しない。

## 通常描画で対応する構成

通常の操作では、新しい独立route、最後に生成した終点からの延長、session終了後の新しい独立route、明示的なPole終点の再利用、対応済みmidair branch pick、T字・cross生成、モデルを含む複数Bundle生成に対応する。routeの完了後は、一時anchor、preview、pending support状態を残さない。

実WASMの回帰fixtureは、モデルを含む独立した12本のrouteを作り、それぞれ2区間を確定する。previewの原子性と最終scene出力を検証する。ViewerActionsでは別途、Click、Enter、Escapeの繰り返しと、toolを切り替えるsessionを検証する。

## 製品要件による制約

`RequirementConstraint`に分類するのは、要求されたmidair branchをBundleテンプレートが禁止する場合など、明示的なテンプレート方針またはtopology方針に限る。古いrow、モデルfixtureの欠落、解決できない保存binding、session状態の不一致は、製品要件による制約ではない。

## 失敗時の責務

分類とreason codeの意味は[確定操作の失敗分類](../editor/commit_failure_categories.md)を正本とする。Coreは`core_edit_types.hpp`の`ClassifyCommitFailure`で分類し、未知のerrorは`InternalError`とする。下表は描画操作の境界でWireのどこが失敗を担当するかを示す。

| 担当 | 入力条件 | 分類 | 通常描画への影響 |
|---|---|---|---|
| anchor・要求の検証 | 対象IDの欠落、非有限値の点、anchorと同じ終点 | `InvalidInput` | 明示的なClickまたはEnter時だけ表示する |
| テンプレート方針 | midair node/branchやtopology方針による明示的な禁止 | `RequirementConstraint` | 許可されたテンプレートまたは対象を使う |
| topology/layoutの解決 | 意味のあるtopologyだが未実装 | `NotImplemented` | 入力不正ではなく、実装の保留事項として扱う |
| 描画session | 参照するanchorやバージョンが古くなった | `StateConflict` | anchorを更新して再試行する |
| backbone/model assembly | 保存binding、row、fixture、生成endpointが不変条件に違反した | `InternalError` | 対応済み経路の不具合として扱う |

## sessionの動作

最初のClickでanchorを受理する。次のClickで1区間を確定し、解決済みの終点を継続anchorとして保持する。Enterは表示中のガイドを確定してsessionを終了する。Escapeは一時状態だけを破棄する。toolの切り替えでは、それまでの一時sessionを終了する。全ての主操作は明示的な結果を返し、描画可能なクリックを無視せず明示的に結果を返す。
