# AGENTS.md

## 目的
見た目の自然さと内部整合の両方を重視する。内部整合が取れていても、人間が見て不自然なら未完了とする。

## 基本原則
- 正本直書き禁止。公開 API または既存の正規更新経路を使う。
- KISS を優先し、必要以上の抽象化・分岐・互換層を増やさない。
- Input / 正本 / 派生を混ぜない。同じ意味の決定者は 1 レイヤ 1 family に寄せる。
- topology 上の別物と visual 上の別物は別に扱う。
- docs・code・tests・viewer 挙動がズレる可能性を前提に、まず観測事実を優先する。
- 不明な点は不明のまま残し、想像で補完しない。
- 旧経路や互換用分岐は「使っていないなら残す」のではなく、削除候補として扱う。

## 責務境界
- Generation 以外は `support_orientation_rule` を変えない。
- Materialization 以外は socket を変えない。
- Recalc は owner topic を変えない。
- Visual は world-space geometry を読むだけにする。
- 下流は world 化するだけ、描くだけ、監視するだけに寄せ、上流の決定を再解釈しない。

## 命名
- 強い中心語を置き、修飾語を積んで説明しない。
- manager/helper/preparation/processor のような逃げ語を避ける。
- 名前は近傍の namespace・型・ファイル文脈で読ませ、新語や言い換えを増やさない。

## 実装時の判断基準
- まず、直近の指示から決定事項、非対象、受け入れ条件を整理する。
- 局所修正で済むなら局所で直す。ただし同じ意味を別段で再判定しているなら構造改善を優先する。
- bool や番兵値を安易に増やさない。新しい例外分岐を足す前に、既存の責務境界が正しいか確認する。

## 進捗の扱い
- 品質と納期は両方重要だが、最優先は納期。品質確認は、前へ進むための最小十分な範囲に絞る。
- 検証済みの修正は、その作業単位で commit してから次の修正へ進む。未コミット差分へ別件の修正を積み重ねない。
- 作業前後にスロー化の兆候を確認する。小さい安全作業・docs/test 追加・1件ずつの退役に逃げていないかを見る。
- C番号や docs/test の増加だけを進捗としない。supported scenario が増えたときだけ進捗とする。
- 旧経路整理では、test 数ではなく family 単位の退役、物理削除、normal path からの依存除去を進捗とする。
- 実用 scenario は、まず現在 fail することを確認してから実装する。既に pass する scenario に test を足すだけなら採用しない。
- production code 変更なしの作業が続く場合は停止し、整理フェーズに戻っていないか確認する。

## 検証
- docs や AGENTS.md だけの変更で、必要性がないのにビルドやテストを回さない。検証は変更範囲に見合うものを選ぶ。
- メトリクスを通しただけで完了扱いにしない。可能なら数値観測や validator / test で不変条件を固定する。
- test は symptom を見るものと正本を固定するものを分け、どちらを守る検証か明示する。
- test が常に正本とは限らない。失敗を直すときは、test の期待値より観測事実とアーキテクチャ上の正しさを優先する。
- 旧テストは捨てない。ただし旧実装詳細の期待値を backbone の完了条件にしない。守っていた制約を抽出し、backbone の構造で移植する。
- viewer で違和感がある場合、core 幾何の問題か viewer 表示の問題かを切り分ける。

## backbone 作業の注意
- backbone は v1 の整理場所ではなく、v1/recalc/materialization を読まない本流として扱う。
- SavedBackboneGraph は topology authority、pair/open/row は connectivity authority、support group は placement authority とする。
- context link は判断入力であり、生成・保存対象にしない。
- existing span/layout/seed/position proximity から topology / pair / row / port / lowering / draw を推測しない。
- fallback で通さない。不足情報がある場合は unsupported にする。
- `pipeline.cpp` が肥大化しているため、新しい flag や分岐を足す前に、責務を既存の owner に寄せられるか確認する。

## 出力時に必要に応じて書くこと
- 何を観測したか
- 何を変えたか
- どの責務に寄せたか
- どの不変条件を守るようにしたか
- 何が未完か
- 次に見るべき点

## ここに書かないもの
直近の個別バグ、今回だけの仕様、作業中の閾値、過去ログの長い要約、一時的な検索監視対象は都度の指示文に書く。

## ビルド
実績あるコマンドだけをまとめたものは docs/command_cheatsheet.md を参照。
