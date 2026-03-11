# DrawPath 可視期待値マトリクス

## 目的

この文書は、次の境界を固定するための整理メモです。

- `DrawPath` は現状の主要な利用入口である。
- 「コードや runtime がある」ことと、「DrawPath 利用者が実際に体験できる」ことは同じではない。
- 判定基準は内部実装の完成度ではなく、現時点の DrawPath 通常利用で何が見えるかに置く。

この文書でいう「DrawPath 通常利用」は次を指す。

- `viewer` が clicked point と `node_specs` から通常の `BackboneSpec` を組み立てる経路
- 明示 override 編集や attachment / socket authoring を行わない経路

## 固定方針

- DrawPath 利用者が通常生成と現行 viewer を通して観測できるものだけを「見えるべきもの」に数える。
- DrawPath request に必要入力が流れていない高度機能は、「DrawPath でできている扱い」にしない。
- viewer 上の違和感は次のどれかとして分類する。
  - 明確なバグ
  - 仕様未達 / 見た目不十分
  - DrawPath 未接続
  - 現時点ではまだ見えなくてよい高度機能

## 非対象

- DrawPath attachment / socket authoring の完成
- `support style / mirror / flow classification` の formal override 導入
- `Backbone` と `detail` の責務再定義

## A. DrawPath 通常利用で見えるべきもの

### A1. Backbone / 配置の基本体験

- Pole は clicked path に沿って自然に並ぶべき
- 明示 pick した pole / support node は重複生成されず自然に再利用されるべき
- Midair extension を含む path でも detail chain が成立するべき

現状分類: 実装済み

根拠:

- `C36` DrawPath 点位置の正確性
- `C110-C113` explicit node reuse / midair extension

### A2. main / branch の粗い構造差

- main と branch は最低限でも見分けがつくべき
- 既存 main chain 継続は branch 追加後も main らしく見えるべき
- branch support と branch down offset は、隠れ metadata ではなく見た目差として読めるべき

現状分類: 一部実装済みだが見た目としてまだ弱い

根拠:

- `C133-C140` main-chain forward、branch support ports、down offset、edge flow
- `C164` support layout が branch-support 入力を集約

### A3. Pole / support / port 軸の読みやすさ

- 直線継続では support / port 列が route 軸に揃うべき
- 十字 / 直交 junction では support 列が対角線化しないべき
- branch を足しても既存 main support 軸は壊れないべき

現状分類: core と inspection 面では実装済み

根拠:

- `C173-C176`

### A4. cable 形状の基本品質

- 最低限の sag 感が見えるべき
- near-straight で不自然な横 wobble が出ないべき
- branch span は support 近傍の出方に寄せ、全体で横へ流れすぎないべき

現状分類: テスト済みケースでは成立するが、全 viewer capture では未保証

根拠:

- `C141-C149`
- `C161-C163`

### A5. basic support-layout の観測性

- DrawPath 利用者または debugger は、その span が plain support endpoint を使っているか読めるべき
- `SupportLayout` と `DetailCurve` は同じ endpoint 由来を説明できるべき

現状分類: inspection / logging 面では実装済み

根拠:

- `C165`
- `C177-C178`

## B. DrawPath 通常利用ではまだ見えなくてよいもの

### B1. attachment / socket authoring

- explicit endpoint attachment 選択
- explicit endpoint socket 選択
- attachment internal path replacement
- DrawPath authored behavior としての attachment-driven endpoint escape

現状分類: core 基盤あり、DrawPath 未接続

### B2. 高度 override 駆動の制御

- DrawPath 通常利用での endpoint socket override 編集
- formal mirror override
- formal flow-classification override
- support-style override

現状分類: 一部は器のみ、一部は未実装

### B3. 深い inspector / mod surface

- readonly debug surface は存在してよい
- ただし、これは DrawPath の最低限 UX 要件ではない

現状分類: debug 用としては実装済み、通常利用の必須体験ではない

## C. DrawPath 利用で本来見えるべきなのに、今は弱い / 見えていないもの

### C1. main vs branch の見た目差

- core は main/branch を分類し branch support も導出しているが、viewer を casual に見たときに「差が十分か」はまだ弱い可能性がある

分類: 見た目として不十分

理由:

- core 側には branch support / down offset の規則とテストがある
- viewer 側では debug panel を開かずに差が明確かまではまだ保証していない

### C2. 実キャプチャでの branch 横回り込み

- long-branch suppression 自体はあるが、ユーザーが感じた違和感をすべて潰せているわけではない

分類: 見た目として不十分

理由:

- 既存テストは機構の存在を保証する
- しかし DrawPath capture 全体を受けた見た目品質までは固定していない

### C3. main-chain continuity が「main らしく見える」こと

- support-axis 修正や main-chain forward は入っているが、viewer 上で end-to-end に main らしく見えるかはまだ手確認依存が残る

分類: 見た目として不十分

理由:

- core の方向 / support 軸ロジックは改善済み
- viewer 側受け入れ条件はまだ debug / inspection 中心で、capture ベースの確証が薄い

### C4. attachment/socket を DrawPath でも「できている扱い」にすること

- これは見積もり上の一番大きい誤分類ポイント

分類: DrawPath 未接続

理由:

- runtime はある
- しかし DrawPath request path に attachment / socket 入力が乗らないため、通常 DrawPath は plain support endpoint へフォールバックする

## 状態マトリクス

| 項目 | DrawPath 利用者が今期待してよいこと | 現状分類 | 備考 |
|---|---|---|---|
| clicked path に沿った pole 配置 | 期待してよい | 実装済み | DrawPath の基礎価値 |
| picked node reuse / midair reuse | 期待してよい | 実装済み | 通常 authoring 動作 |
| main / branch の粗い分離 | 期待してよい | 見た目として不十分 | core 基盤はあるが見た目保証が弱い |
| straight/cross/orthogonal support 軸 | 期待してよい | 実装済み | core と inspection では保証強化済み |
| branch support / branch down offset | 期待してよい | 見た目として不十分 | 挙動はあるが casual view で弱い可能性あり |
| basic sag / 横 wobble 抑制 | 期待してよい | 一部実装済み | 良い invariant はあるが capture 網羅不足 |
| attachment 入力なしの plain support fallback | inspection で読めてよい | 実装済み | DrawPath 未接続診断に重要 |
| DrawPath からの explicit attachment/socket 利用 | まだ見えなくてよい | DrawPath 未接続 | runtime はあるが入口がない |
| DrawPath からの attachment internal path 挙動 | まだ見えなくてよい | DrawPath 未接続 | 通常 DrawPath 入力対象外 |
| support style 切替 | まだ見えなくてよい | 未実装 / 器のみ | formal override 未導入 |
| mirror override | まだ見えなくてよい | 器のみ | override surface は placeholder |
| flow classification override | まだ見えなくてよい | 器のみ | override surface は placeholder |

## DrawPath 利用者向け優先順位

### P1. 今すぐ詰めるべきもの

1. default viewer view での main vs branch の見分けやすさ
2. DrawPath capture でまだ目立つ branch 横回り込み
3. junction で main chain が main らしく見えること
4. core にある basic support-layout 判断が viewer で読みにくい箇所

### P2. 次点

1. support-layout trace の見やすさ
2. endpoint origin の見やすさ
3. branch down offset の可視確認
4. main-axis 選択の可視確認

### P3. 後回し

1. DrawPath attachment/socket authoring
2. attachment internal path 露出
3. advanced override UI
4. support style バリエーション

## 最小改善プラン

1. DrawPath で作った viewer capture を `main / branch / support / port / curve` の受け入れ面として使う
2. default inspector で `flow`, `support axis`, `endpoint origin`, `branch down offset` をコードを読まずに説明できる状態を強める
3. 高度機能を広げる前に、残る見た目違和感へ capture-shaped invariants を追加する
4. attachment/socket は DrawPath へ explicit input path を足すまで、「基盤あり / 通常利用未接続」の箱に置く

## 現時点の要約

- DrawPath 通常利用で「できる」と言ってよいもの
  - route-following pole placement
  - main/branch classification の基礎
  - straight/cross/orthogonal での非対角 support 軸
  - basic sag と departure 制約つき curve
  - attachment 入力なし時の plain-support fallback 可視化
- DrawPath 通常利用でまだ「できる」と言わないもの
  - authored attachment/socket endpoint behavior
  - authored attachment internal path behavior
  - advanced override-driven support style / mirror / flow control
- 現状の最重要課題は attachment の高度化ではない
- DrawPath 基本体験の見た目 readability を上げることである
