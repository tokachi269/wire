# Web viewer レイアウト方針

このドキュメントは、web viewer(wasm + three.js + Svelte)の画面構成を定める。
W4(パネル移植)はこの仕様に従い、個別判断でレイアウトを決めない。
操作の集合は既存 viewer から引き継ぐが、配置・グルーピング・表示タイミングは作り直す。
panels.cpp の1:1移植は禁止する。

## 現行の破綻(修正対象)

現行 desktop viewer は次の情報設計の欠陥を持つ。web viewer で再現しない。

- 全パネルが縦一列に積まれ、3D ビューへ重なる。レイアウト領域(region)が無い。
- Outliner が Poles/Ports/Spans をフラットに全行インライン展開し、Ports が pole 数×band 数で
  数百行になり画面を占有する(`DrawObjectList` の無制限 `Selectable`)。
- Bundle placement とPole band詳細が同じ表に混ざり、生成するBundle数を編集できない。
- 生成の全体設定と、選択オブジェクトの編集が同じ列に常時同居する。累進的開示が無い。

## リージョン構成

固定リージョンとし、パネルを 3D ビューに重ねない。v1 は固定レイアウトで、
自由なドッキング再配置は提供しない。

| region | 位置 | 内容 | 幅の目安 |
|---|---|---|---|
| top bar | 上・全幅 | mode 切替、generate、save capture、pause | 全幅 |
| left | 左 | scene tree(outliner) | 狭・固定 |
| center | 中央 | 3D viewport(遮らない) | 可変・最大 |
| right | 右 | draw path、scene settings、template editor、selection inspector | 中・固定 |
| bottom | 下・折りたたみ可 | diagnostics / log / error | 全幅・低 |

## 選択状態 → 右 inspector

右 region は常時表示する操作群と、選択に応じて内容を差し替える `SelectionInspector` を分ける。
選択詳細を Outliner に混ぜない。

| 選択 | inspector 内容 |
|---|---|
| 未選択 | scene: 生成設定、geometry/layout、visual cache、camera |
| pole | pole placement(表)、tilt、pole template apply、advanced(折りたたみ) |
| span | cable/bundle template、endpoint/branch override |
| port | port 詳細(read 中心) |

## Outliner(left)

- 階層表示: Pole →(展開)その Ports / Span / Midair node。**Ports は既定折りたたみ**。
- 高さ固定 + スクロール + filter(検索)。無制限インライン展開をしない。
- 選択したものだけ強調する。全項目の一括ハイライトをしない。

## Bundle placement

- 行 = 生成テンプレート内のBundle placement、列 = Bundle / Height / Offset / Spread / Count。
- Height / Offsetはpole local中心軸からの絶対値。初期行はPole templateのband既定位置から設定し、
  Heightの高い順（HV先頭）に並べる。未使用のDropは表示しない。
- categoryはBundleTemplateの属性であり、行identityや複製可否には使わない。全categoryで同じ操作を使う。
- 行のhoverまたはkeyboard focusで`＋`を表示し、その配置recordを直後へ複製する。
- `＋`後は並べ替えず、保持しているplacement順をそのまま表示する。削除操作は置かない。
- 同じBundleTemplateを複数行から参照できる。各行はGenerate時に別Bundle identityになる。
- Pole typeの`Port bands`と`Anchor slots`はplacement表ではない。template editorのadvanced詳細として
  既定折りたたみにし、Bundle placementへ平均値を再入力しない。

## 累進的開示

- Advanced Port Bands、Anchor Slots、Junction Debug、Visual Cache 内部値、capture の詳細は既定折りたたみ。Repro Trace の保存ボタンだけは top bar に置く。
- P1(編集操作と主要表示)を最初に見せる。P2(debug/capture 系)は畳む。

## 反映モデル

- スライダの drag policy は operations.md の更新クラス列に従う:
  kRedraw/kReshape/kReposition は生反映(スロットル)、regenerate 級・構造差分は release-only か無効化。
- kReshape の生反映では `preferred_visible_span_ids` を必ず渡す(既存 `UpdateCableTemplate` 機構)。
- 明示 Apply ボタンは regenerate 級・構造差分にのみ置く。
- 操作中フラグは store 側に持つ(`interaction`)。preview と commit は同じ core API を呼び、
  ログ/undo/capture は commit にだけ紐づける。core に transient/preview 概念を追加しない。
- **暗黙の連結呼び出しを禁止する。** 現行の「Apply Pole/Bundle Template 成功後に
  `ApplyBundleRelatedPoleTypeToExistingPoles` を自動で呼ぶ」挙動は移植しない。related 適用は独立ボタンにする。
- 全操作の `EditResult.error` を bottom diagnostics に一級表示する。ログに握り潰さない。

## 非対象

- mesh/glTF 表示、テーマの細部装飾、ドッキングの自由再配置。
- capability の事前判定 UI(ボタン無効化+理由)。v1 は error 表示で足りる。将来 core の
  preflight query が入った時に検討する。

## W0 / W4 との関係

- W0 の操作監査表(operations.md)に次の列を持たせる:
  group(生成/pole/span/visual/debug)、region(top/left/right/bottom)、
  visibility-context(always / pole 選択時 / span 選択時)、control 種別(slider/toggle/button/text)。
- W4 は operations.md と本ドキュメントの両方に従う。表と本仕様に無い配置を実装時に足さない。
