# AGENTS.md
In Code Mode, within each bounded stage, run independent, functions.exec-available tool calls concurrently in one functions.exec call. Use await Promise.allSettled([...]) when partial results are useful, and inspect every result; use await Promise.all([...]) only when any failure should abort the batch. Keep dependencies, waits/resumes, approvals, conflicting or interdependent mutations, and adaptive investigations where each result may change the next step sequential. Do not split otherwise batchable inspections across outer tool calls.

## 目的
見た目の自然さと内部整合の両方を重視する。内部整合が取れていても、人間が見て不自然なら未完了とする。

## 基本原則
- KISS を優先し、必要以上の抽象化・分岐・互換層を増やさない。
- 不明な点は不明のまま残し、想像で補完しない。判断に迷う変更は実施せず理由つきで報告する。
- 旧経路・互換分岐・不要処理は削除候補として扱う。
- ユーティリティや変換を新設する前に、既存の共有実装を検索する。重複定義を作らない。

## 命名
- 強い中心語を置き、修飾語を積んで説明しない。manager/helper/processor のような逃げ語を避ける。
- 名前は近傍の namespace・型・ファイル文脈で読ませ、新語や言い換えを増やさない。

## 実装の判断
- まず直近の指示から決定事項・非対象・受け入れ条件を整理する。
- production変更前に`docs/architecture.md`、`docs/testing.md`、該当domainのarchitecture、該当operation semanticsを読む。
- architecture semanticsを本ファイルへ複製しない。新しいdecisionは、実装前にcanonical architecture文書でownerを決める。
- 操作または対応状態を増やす変更は、先に該当domainの操作×状態意味論表を更新する。未定義セルを現在の実装やtestから推測して埋めない。
- 局所修正で済むなら局所で直す。ただし同じ意味を別の場所で再判定しているなら構造改善を優先する。
- bool や番兵値を安易に増やさない。新しい例外分岐を足す前に、既存の責務境界が正しいか確認する。

## UI変更
- 機能要件を満たすための局所的なUI追加は、既存の構成・操作体系に沿う範囲で実施してよい。roadmap・phase要件・core機能の記載だけを根拠に、画面全体や既存操作を再設計しない。
- 明示指示なしに、既存のタブ・ボタン・パネルを削除、移動、統合、置換しない。操作手順、クリック順序、既定動作、画面上の主従関係も変えない。
- 新しい操作モード、常設領域、既存操作の集約など、UXへ大きく影響する変更は実装前に確認する。複数のUI経路があり操作感が変わる場合は、未決定として推測で選ばない。

## 進捗
- 最優先は納期。品質確認は前へ進むための最小十分な範囲に絞る。
- 検証済みの修正はその作業単位で commit してから次へ進む。未コミット差分に別件を積まない。
- verificationや文書の追加数を進捗としない。守れるcontract、supported scenario、退役できた重複で評価する。
- 整理作業は test 数ではなく family 単位の退役・物理削除・依存除去を進捗とする。
- production 変更なしの作業が続いたら停止し、逃げの整理フェーズに入っていないか確認する。

## 検証
- verification policyの正本は`docs/testing.md`とする。testやledgerをarchitecture semanticsの正本として扱わない。
- bug fix終了時に、incident reproducerを永久保存する必要があるか`docs/testing.md`のregression lifecycleで評価する。
- 検証は変更範囲に見合うものを選ぶ。docs だけの変更で全件テストを回さない。
- viewer で違和感がある場合、core 幾何の問題か viewer 表示の問題かを切り分ける。判断ロジックは core に置き、viewer には持たせない。

## 報告
出力時に必要に応じて書く:
- 新しい概念・フィールドを1つ追加した場合、修正が波及した決定箇所の数と場所
- 観測したこと / 変えたこと / 寄せた責務 / 守った不変条件 / 未完と保留(保留は docs/merge_readiness.md に登録)
- 次に見るべき点

同じ種類の指摘が繰り返された場合は、個別対応で済ませず AGENTS.md への恒久ルール追加を簡潔に提案する。

## wire 固有
- wireのarchitectureは`docs/wire/architecture.md`、操作意味論は`docs/wire/backbone_operation_semantics.md`、モデル・座標規約は`docs/wire/models.md`を読む。domain契約を本ファイルへ複製しない。
- pipeline.cpp に新しい flag・分岐を足す前に、責務を既存の owner に寄せられるか確認する。

## road 固有
- roadの移行先authorityとbuild境界は docs/road/architecture.md、操作意味論は docs/road/operation_semantics.md、対応範囲は docs/road/supported_operations.md、対象外と将来の範囲は docs/road/backlog.md を正本とする。

## ここに書かないもの
直近の個別バグ、今回だけの仕様、作業中の閾値、過去ログの要約は都度の指示文に書く。

## ビルド
実績あるコマンドは docs/command_cheatsheet.md を参照。
