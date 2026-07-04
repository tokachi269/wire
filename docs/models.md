# Model方針

このドキュメントは、自動生成placeholder(円柱pole等)から自作モデル(電柱、碍子、変圧器、端子函、広告等)へ
品質を上げるための設計方針をまとめる。特定のゲームエンジンは前提にしない。

## 位置づけ

architecture.mdの境界を変えない。
coreはbackend非依存のcurve、bounds、primitive、style参照を出力し、mesh assetはadapter側で解決する。
**coreはmeshを読まない。** モデル対応はcoreの変更ではなく「参照の先」の整備である。

## 3層構造

```text
mesh asset(fbx/glTF等)   adapter側。coreは読まない
model descriptor           meshから抽出+手動overrideした数値契約。engine非依存
core template              PoleTypeDefinition / AttachmentTemplate等。descriptorから値を受ける
```

descriptorはengine非依存の数値契約であり、エンジン決定前に固定できる。
bbox等の抽出はdescriptorを生成するimportツールの仕事で、coreには入れない。

## 接続点: markerを正、bboxは粗い初期値

- 接続点(socket)はDCC内のnamed marker(empty等)と命名規約を正本にする。
  bboxから取れるのは全高・接地半径などの粗い値だけで、碍子の線受けや函の端子位置は原理的に出ない。
- 調整はアプリ内の数値入力ではなくDCC内のmarker移動で行う。
- geometry推測でsocketを補完しない。markerが無ければそのsocketは無い。

## 測定とoverrideの分離

- descriptor = 抽出結果(測定レイヤ) + 手動override(補正レイヤ)の合成。2つを混ぜて保存しない。
- overrideはmarker名・band名など安定キーで持つ。

## 再読込

- 再抽出は測定レイヤだけを置き換える。overrideは残す。
- 差分report(marker消失、寸法変化)を出す。消えたmarkerを参照するoverrideは補完せずconflictとして報告する。
- core templateへの反映は既存post-edit経路に乗せる。placement-only差分はkReposition/kReshape、
  構造差分はunsupported/regenerate。**モデル再読込専用の更新経路は作らない。**

## テーパー

- descriptorにradius(h)を数点サンプル(接地・各band高さ・頂部)で持つ。連続関数や断面解析はしない。
- coreはbandごとのradius値として消費する(center hintのclearance等、既存C80系)。
- 精度が要る高さだけoverrideで補正する。

## 自作と手続き生成の境界

**他の形状に密着追従する必要があるものは手続き生成に残す。単体で完結する形は自作モデルにする。**

| 種別 | 扱い |
|---|---|
| 電柱本体、碍子、変圧器、端子函、広告板 | 自作mesh |
| ベルト・バンド類(テーパー円周に密着) | radius(h)から手続き生成 |
| 腕金(crossarm) | 自作mesh + 手続きベルトで留める |

ベルトを自作meshのスケールで合わせる方式は、テーパーがある限り微調整が無限に発生するため採用しない。

## 表面占有(重なり回避)

- 電柱meshは素のテーパー柱にし、柱表面を占有するもの(昇降ボルト、バンド、広告、端子函)は
  すべてplacement対象にする。占有は pole ごとの(高さ区間 × 角度区間)リストで管理する。
- mesh同士の実行時衝突判定はしない。宣言済み区間の照合のみで解く(placement reserveと同じ考え方)。
- ボルト等を焼き込んだmeshを使う場合は、descriptorにkeep-out zone(側・高さ範囲・角度)を宣言させる。

## span内モデル(玉碍子・端子函)

既存機構を使う。新概念は追加しない。

- 正体は span 上の Attachment + socket + `AttachmentLineInteractionMode::kReplaceWithInternalPath`。
  元curveの`replaced_interval`をhiddenにし、socket間をつなぎ直す。
- 玉碍子: socket 2つ + 区間置換。線はsocket Aで終端し、碍子長ぶんhidden、socket Bから再開。
- 端子函: in/out socketで幹線を函へ落として出す。引込線用socketを追加すれば降り線の接続点になる。
- socket位置はmodel descriptorのmarkerから供給する(上記3層構造と同一の経路)。

### 決定済みの派生規則

- モデルposeはcurveから一方向導出する: pose = 弧長s位置 + tangent frame。socket世界位置はposeから導出し、
  線の端がsocketへ合わせに行く。モデル位置へcurveを寄せる逆方向は禁止。
- 1 spanに複数置いた場合、replaced intervalの重複はvalidatorで報告する。補完しない。

### 未決(別タスクで決める)

- internal path無しの置換(モデル本体が区間を埋める)を合法にするか。現状validatorはReplace/Addに
  internal pathを要求する。玉碍子はpath無しが正しいため、許可する方向で検討する。
- 点荷重によるsag変形(重い函で弛みが折れ線化)は現行scope外。やる場合はcurve profile hint拡張(C277系)。

## 依存する解除項目

merge_readiness.mdのunsupported保留一覧のうち、モデル対応を進めると優先度が上がるもの:

| 項目 | 理由 |
|---|---|
| `UpdateAttachmentTemplate`の使用中拒否 | モデル再読込でsocket位置が変わるたびに拒否に当たる。幾何のみの差分はkReshapeで通す緩和が必要 |
| regenerateのuser attachment保持 | span上の玉碍子・端子函が増えるほど「置いた線はtemplate編集不可」の面が広がる。attachmentはspan id + 弧長キーで引き継ぐ設計を別milestoneで積む |

## 進め方

1. descriptor契約(marker命名規約・項目)を文書で固定する
2. 電柱1・碍子1・変圧器1の3体だけdescriptorを手書きしてviewerで回す(抽出ツールは書かない)
3. 再読込・override・conflict報告のループを固定してから、bbox/marker抽出を自動化する
4. ベルト手続き生成と表面占有solverはcrossarm実装と同じmilestoneに入れる

importパイプラインの実装から始めない。契約 → 手書き検証 → 自動化の順とする。
