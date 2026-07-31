# AGENTS.md
In Code Mode, within each bounded stage, run independent, functions.exec-available tool calls concurrently in one functions.exec call. Use await Promise.allSettled([...]) when partial results are useful, and inspect every result; use await Promise.all([...]) only when any failure should abort the batch. Keep dependencies, waits/resumes, approvals, conflicting or interdependent mutations, and adaptive investigations where each result may change the next step sequential. Do not split otherwise batchable inspections across outer tool calls.

## 目的
見た目の自然さと内部整合の両方を重視する。内部整合が取れていても、人間が見て不自然なら未完了とする。

## 基本原則
- 状態は Input / 正本 / 派生に分ける。正本を書き換えてよいのは、それを所有する操作 API だけ。派生層・viewer・テストから正本のフィールドへ直接代入しない。派生は正本から一方向に計算し、上流へ書き戻さない。
- KISS を優先し、必要以上の抽象化・分岐・互換層を増やさない。
- 同じ意味の決定者は 1 箇所に寄せる。下流は上流の決定を再解釈しない。
- 位置の近接・名前文字列・種別分岐から同一性や接続を推測しない。識別は ID で行う。
- fallback で黙って通さない。不足・退化は明示エラーか unsupported にする。
- 不明な点は不明のまま残し、想像で補完しない。判断に迷う変更は実施せず理由つきで報告する。
- 旧経路・互換分岐・不要処理は削除候補として扱う。
- ユーティリティや変換を新設する前に、既存の共有実装を検索する。重複定義を作らない。

## 命名
- 強い中心語を置き、修飾語を積んで説明しない。manager/helper/processor のような逃げ語を避ける。
- 名前は近傍の namespace・型・ファイル文脈で読ませ、新語や言い換えを増やさない。

## 実装の判断
- まず直近の指示から決定事項・非対象・受け入れ条件を整理する。
- 新しいdomain、subsystem、pipelineへ着手する前に、同じリポジトリの類似authority、operation、pipeline、utilityを調査する。production実装より先に、既存の決定者 / 新domainでの対応 / 再利用箇所 / 新設理由 / Input / 正本 / 派生 / ownerをdomain architectureへ記載する。対応が未決定なら実装を始めない。
- リポジトリ共通architectureの正本は docs/architecture.md とする。domain固有文書は固有語彙を追加できるが、共通のauthority境界、ID規則、依存方向、transaction契約を弱めない。
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
- docs/test の増加だけを進捗としない。supported scenario が増えたときだけ進捗とする。
- 整理作業は test 数ではなく family 単位の退役・物理削除・依存除去を進捗とする。
- production 変更なしの作業が続いたら停止し、逃げの整理フェーズに入っていないか確認する。

## 検証
- バグ修正と新規 scenario は fail-first。まず現状 fail することを確認してから直す。既に pass するものに test を足すだけの作業は採用しない。
- 新規 test は `WIRE_TEST_EXPECT` のような失敗理由を出せる helper を使う。既存 test は触った関数単位で理由付きへ移行する。全件一括移行を進捗扱いしない。
- skip されたテストを通過と数えない。報告では skip 0 を確認事項に含める。
- 挙動変更ゼロを謳うリファクタは、bit 一致・既存テスト無変更通過など等価性の証明方法を明示する。
- test が常に正本とは限らない。失敗時は test の期待値より観測事実とアーキテクチャ上の正しさを優先する。旧テストは期待値ではなく制約を抽出して移植する。
- 検証は変更範囲に見合うものを選ぶ。docs だけの変更で全件テストを回さない。
- viewer で違和感がある場合、core 幾何の問題か viewer 表示の問題かを切り分ける。判断ロジックは core に置き、viewer には持たせない。

## 報告
出力時に必要に応じて書く:
- 新しい概念・フィールドを1つ追加した場合、修正が波及した決定箇所の数と場所
- 観測したこと / 変えたこと / 寄せた責務 / 守った不変条件 / 未完と保留(保留は docs/merge_readiness.md に登録)
- 次に見るべき点

同じ種類の指摘が繰り返された場合は、個別対応で済ませず AGENTS.md への恒久ルール追加を簡潔に提案する。

## wire 固有
- wireのarchitecture・authority(topology / connectivity / placement / 派生の一方向性)の正本は docs/wire/architecture.md、操作意味論は docs/wire/backbone_operation_semantics.md、モデル・座標規約は docs/wire/models.md。本ファイルと矛盾したら共通architecture、domain architectureの順に優先する。
- Generation 以外は support_orientation_rule を変えない。Materialization 以外は socket を変えない。Visual は world-space geometry を読むだけ。
- context link は判断入力であり、生成・保存対象にしない。
- pipeline.cpp に新しい flag・分岐を足す前に、責務を既存の owner に寄せられるか確認する。

## road 固有
- roadの移行先authorityとbuild境界は docs/road/architecture.md、操作意味論は docs/road/operation_semantics.md、要求範囲は docs/road/plan.md を正本とする。
- roadの現行実装を設計根拠にしない。docs/road/architecture.mdに未移行と記載された責務へ機能を追加する前に、該当authority境界を移行する。

## ここに書かないもの
直近の個別バグ、今回だけの仕様、作業中の閾値、過去ログの要約は都度の指示文に書く。

## ビルド
実績あるコマンドは docs/command_cheatsheet.md を参照。
