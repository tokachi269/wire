# Web viewer レイアウト方針

この文書は、web viewer(wasm + three.js + Svelte)の画面構成を定める。W4(パネル移植)はこの仕様に従い、個別判断でレイアウトを決めない。操作の集合は既存viewerから引き継ぐが、配置・グルーピング・表示タイミングは作り直す。panels.cppの1:1移植は禁止する。

## 現行の破綻(修正対象)

現行desktop viewerには、次の情報設計上の問題がある。web viewerで再現しない。

- 全パネルが縦一列に積まれ、3Dビューへ重なる。レイアウト領域(region)が無い。
- OutlinerがPoles/Ports/Spansをフラットに全行インライン展開し、Portsがpole数×band数で
  数百行になり画面を占有する(`DrawObjectList`の無制限`Selectable`)。
- Bundle placementとPole band詳細が同じ表に混ざり、生成するBundle数を編集できない。
- 生成の全体設定と、選択オブジェクトの編集が同じ列に常時同居する。段階的開示が無い。

## リージョン構成

固定リージョンとし、パネルを 3Dビューに重ねない。v1は固定レイアウトで、自由なドッキング再配置は提供しない。

| region | 位置 | 内容 | 幅の目安 |
|---|---|---|---|
| top bar | 上・全幅 | モード切替、generate、pause | 全幅 |
| left | 左 | scene tree(outliner) | 狭・固定 |
| center | 中央 | 3D viewport(遮らない) | 可変・最大 |
| right | 右 | draw path、scene settings、テンプレートの編集画面、selection inspector | 中・固定 |
| bottom | 下・折りたたみ可 | diagnostics / log / error | 全幅・低 |

## 選択状態と右側のinspector

右リージョンでは、常時表示する操作群と、選択に応じて内容を差し替える`SelectionInspector`を分ける。選択詳細をOutlinerに混ぜない。

| 選択 | inspectorの内容 |
|---|---|
| 未選択 | scene: 生成設定、geometry/layout、visualキャッシュ、camera |
| pole | pole placement(表)、tilt、poleテンプレートapply、advanced(折りたたみ) |
| span | cable/bundleテンプレート、endpoint/branch override |
| port | port詳細(read中心) |

## Outliner(left)

- 階層表示: Pole →(展開)そのPorts / Span / Midair node。Portsは既定折りたたみ。
- 高さ固定 + スクロール + filter(検索)。無制限インライン展開をしない。
- 選択したものだけ強調する。全項目の一括ハイライトをしない。

## Bundle placement

- 行 = 生成テンプレート内のBundle placement、列 = Bundle / Height / Offset / Spread / Count。
- Height / Offsetはpole local中心軸からの絶対値。初期行はPoleテンプレートのband既定位置から設定し、
  Heightの高い順（HV先頭）に並べる。未使用のDropは表示しない。
- categoryはBundleTemplateの属性であり、行識別情報や複製可否には使わない。全categoryで同じ操作を使う。
- 行へポインターを重ねるか、キーボードでフォーカスを移すと`＋`を表示する。その配置レコードを直後へ複製する。
- `＋`後は並べ替えず、保持しているplacement順をそのまま表示する。削除操作は置かない。
- 同じBundleTemplateを複数行から参照できる。生成時には、各行に別のIDを持つBundleを作る。
- Pole typeの`Port bands`と`Anchor slots`はplacement表ではない。テンプレートの編集画面のadvanced詳細として
  既定折りたたみにし、Bundle placementへ平均値を再入力しない。

## 段階的開示

- Advanced Port Bands、Anchor Slots、Junction Debug、Visual Cache内部値は既定折りたたみにする。
- P1(編集操作と主要表示)を最初に見せる。

## 反映モデル

- スライダのdrag policyはoperations.mdの更新クラス列に従う。
  kRedraw/kReshape/kRepositionは、更新頻度を制限しながら操作中も反映する。regenerate級の変更や構造差分は、操作終了時だけ反映するか、操作を無効にする。
- kReshapeで操作中の変更を反映するときは、`preferred_visible_span_ids`を必ず渡す(既存`UpdateCableTemplate`機構)。
- 明示Applyボタンはregenerate級・構造差分にのみ置く。
- 操作中フラグはstore側に持つ(`interaction`)。previewとcommitは同じCore APIを呼び、
  ログの記録とUndoへの登録はcommit時だけ行う。Coreに一時状態やプレビューの概念を追加しない。
- 暗黙の連続呼び出しを禁止する。現行の「Apply Pole/Bundle Template成功後に
  `ApplyBundleRelatedPoleTypeToExistingPoles`を自動で呼ぶ」挙動は移植しない。related適用は独立ボタンにする。
- 全操作の`EditResult.error`を画面下部の診断領域に直接表示する。ログにだけ記録して表示を省略しない。

## 非対象

- mesh/glTF表示、テーマの細部装飾、ドッキングの自由再配置。
- capabilityの事前判定UI(ボタン無効化+理由)。v1はerror表示で足りる。将来coreの
  preflight queryが入った時に検討する。

## W0 / W4 との関係

- W0の操作監査表(operations.md)に次の列を持たせる。
  group(生成/pole/span/visual/debug)、region(top/left/right/bottom)、
  visibility-context(always / pole選択時 / span選択時)、control種別(slider/toggle/button/text)。
- W4はoperations.mdと本ドキュメントの両方に従う。表と本仕様に無い配置を実装時に足さない。
