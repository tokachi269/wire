# AGENTS.md

## 目的
このリポジトリでは、見た目の自然さと内部整合の両方を重視する。  
内部整合が取れていても、人間が見て不自然なら未完了とする。

## 基本原則
- 正本直書き禁止。公開 API または既存の正規更新経路を使う。
- Manual 保持優先。全体再生成を既定にしない。
- KISS を優先し、必要以上の抽象化や分岐を増やさない。
- Input / 正本 / 派生を混ぜない。
- topology distinct と visual distinct は別に扱う。
- 処理は可能な限りパイプラインとして構成し、各段の入力・出力・責務を明確に分ける。
- docs・code・tests・viewer 挙動がズレる可能性を前提に、まず観測事実を優先する。
- 不明な点は不明のまま残し、想像で補完しない。
  1. Generation 以外は `support_orientation_rule` を変えない
  2. Materialization 以外は socket を変えない
  3. Recalc は owner topic を変えない
  4. Visual は world-space geometry を読むだけ
- 変数名やメソッド名の命名は強い中心語を置き、修飾語を積んで説明しない。manager/helper/preparation/processor のような逃げ語を避ける。名前は近傍の namespace・型・ファイル文脈で読めるようにし、新語や言い換えを増やさない。

## 設計方針
- 文章ルールを増やすより、型・API・validator で守る。
- 不正状態を型として作れないようにする。
- 生の更新口を増やさない。中間構造や可変コンテナへの直接追加を避ける。
- 後方互換が不要な変更では、互換用分岐や変換層を足さず現行の正規経路に統一する。
- fallback は必要最小限にし、複数段で同じ意味を再解釈しない。
- 同じ topic に対して、owner は 1 レイヤ 1 family だけにする。
- 同じ意味の決定を複数層で別々に持たない。決定結果は可能な限り一箇所で確定し、下流はそれを使う。
- 下流は world 化するだけ、描くだけ、監視するだけに寄せ、上流の決定を再解釈しない。
- 旧経路や互換用分岐は「使っていないなら残す」のではなく、削除候補として扱う。

## 実装時の判断基準
- まず、直近の指示から以下を整理する。
  - 決定事項
  - 非対象
  - 受け入れ条件
- 局所修正で済むなら局所で直す。
- ただし、同じ意味を別段で再判定しているなら構造改善を優先する。
- bool や番兵値を安易に増やさない。
- 新しい例外分岐を足す前に、既存の責務境界が正しいかを確認する。

## 検証
- メトリクスを通しただけで完了扱いにしない。
- 可能なら数値観測を残す。
- 可能なら validator / test で不変条件を固定する。
- test は symptom を見るものと Authority を固定するものを分け、どちらを守る検証か明示する。
- refresh / recalc 後に意味が痩せていないかを確認する。
- viewer で違和感がある場合、core 幾何の問題か viewer 表示の問題かを切り分ける。
- Tests are not always authoritative in this repo; prioritize observed behavior and architectural correctness over test assumptions when resolving failures.

## 出力時に必要に応じて書くこと
- 何を観測したか
- 何を変えたか
- どの責務に寄せたか
- どの不変条件を守るようにしたか
- 何が未完か
- 次に見るべき点

## ここに書かないもの
- 直近の個別バグ
- 今回だけの仕様
- 作業中の閾値
- 過去ログの長い要約
- 一時的な検索監視対象  
これらは都度の指示文に書く。

## KISS原則の強調
- KISS を優先し、`GenerateFromBackboneSpec` 周辺は plan → validate → commit の薄い orchestrator を保つ。
- 共通 pipeline と policy/resolver の責務分離を重視する。

## ビルド
- 実績あるコマンドだけをまとめたものは docs/command_cheatsheet.md を参照。