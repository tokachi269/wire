# 描画操作

RoadとWireの描画は同じイベント契約に従う。ドメイン固有の幾何、snap、検証、確定要求は、それぞれのドメインが所有する。

## 状態

描画sessionは、次の3つの一時状態のいずれかを取る。

- `idle`: 開始anchorがない。
- `anchored`: 開始anchorはあるが、区間候補がない。
- `guide`: 現在のポインター入力から区間候補が得られている。ガイドはドメインの完全な生成が成功することまでは保証しない。

確定済みのドメインentityは描画sessionの状態ではない。preview、ガイド、ポインターgesture、開始anchorは保存しない。

## クリック（Click）

最初の主操作で開始anchorを設定する。anchorがある状態の主操作では、現在表示中のガイドをドメインの操作へ渡す。完全な生成と検証を実行するのは、この確定境界だけである。確定に成功すると、その終点を次の開始anchorとし、次の区間のpreviewを始める。

確定要求とガイドは同じ入力要求を使う。ガイドのために完全な操作を先行実行せず、ガイドの有無を主操作の実行条件にもしない。

主操作のClickは毎回、`anchor-accepted`、`commit-succeeded`、`commit-rejected`、`operation-applied`、または理由付きの`ignored`を記録して返す。単に処理を抜けるだけでは操作結果にならない。

## Enter

ガイドがある場合、EnterはClickと同じ経路でその区間を確定し、成功した場合だけ描画sessionを終了する。ガイドがなければsessionを終了する。確定に失敗した場合はガイドを維持し、失敗を報告する。Enterも明示的な操作結果を記録する。非activeな描画sessionへの入力は、明示的に扱い`ignored/session-inactive`として報告する。

## Escape

Escapeは未確定のpreview、anchor、ポインターgestureを破棄し、描画sessionを終了する。確定済みのドメイン操作は削除せず、取り消さない。activeなsessionがない場合は`ignored/session-inactive`を返し、存在しなかったsessionを終了したとは報告しない。

## 操作結果

両ドメインは共通の`DrawActionResult`を返す。値は`anchor-accepted`、`commit-succeeded`、理由コード付きの`commit-rejected`、`session-ended`、`operation-applied`、または理由コード付きの`ignored`である。

viewerはこの結果と`CommitFailureCategory`から何が起きたかを判断し、error文字列からは判断しない。

`web/tests/drawSessionContract.ts`が共通の期待動作を定義し、RoadとWireの両方が実行する。

- 非activeな描画sessionでのEscapeは`ignored`となり、確定失敗を残さない。
- 確定を拒否した場合、anchorを保持し、何も確定せず、分類と理由コードを報告する。
- ポインター移動は、直前の確定操作が報告した失敗を上書きしない。
- 拒否後も同じanchorから確定でき、再試行が可能である。
- EnterとEscapeは操作結果を明示して返す。boolだけで結果を表してはならない。

ドメイン固有のtool状態、snap、要求は各ドメインに置く。共有するのは結果を表す語彙だけである。

## 取り消し（Undo）

Ctrl+Zは最後に確定した区間を取り消す操作である。Escapeを取り消しのshortcutにはしない。

## ツール切り替え

toolを変更すると描画sessionの一時状態だけを破棄する。previewを確定せず、既に確定したentityも削除しない。

## ポインター移動

anchor設定後のポインター移動は、軽量ガイドを作るか、候補なしの状態にする。完全なCore生成、将来の確定失敗の分類、ユーザー向け確定errorの通知は行わない。ガイドは現在の入力で置き換え、古い候補を現在のものとして表示し続けない。

Roadのガイドは、候補の理解に必要な入力pathと幅の範囲を示す。Wireのガイドは入力routeとsnap markerを示す。ドメイン幾何、support配置、junction生成、モデルassemblyは確定時の処理である。

Add Laneでは、2車線から3車線への変化開始位置、3車線が完成する位置、3車線を維持する終点の順に選ぶ。ポインター移動で更新するのは局所位置と影響範囲のガイドだけである。editorは選択した道路上の点をsegment内の`t`へ変換し、section planとtransitionの検証はConfirm時だけ実行する。

確定を拒否した場合は3つの選択を全て保持する。

Bezier道路のhandleは、開始anchor、引き継いだ開始接線、終点、弦長から決まる。前frameのポインター位置は入力にしない。同じ終点へ至る2つのポインター経路は、同じガイド形状と確定形状になる。

## 原子性

1区間の確定は、1つのatomicなドメイン操作である。Roadはsegment、node、corridor所属、connection、surface、markingを一緒に確定する。Wireはroute topology、support binding、生成したwire幾何を一緒に確定する。ガイドとtopologyを別々の確定段階にはしない。

## ビルド識別情報

Web bundleとWASM moduleには同じGit commitとpackageバージョンを埋め込む。起動時に両者を比較してからworkspaceを読み込み、sceneをmountする。不一致なら編集を止めて両方の識別情報を報告する。これはドメインの確定失敗には分類しない。

text入力にfocusがある間、EnterとEscapeはそのcontrolが処理する。IME変換中は描画commandを送らない。keyboard入力は、activeな描画ドメインの1つだけへ送る。
