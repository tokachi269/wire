# Viewer 操作インベントリ

この文書は desktop viewer の操作を監査し、web viewer へ移す範囲を固定する。
desktop viewer の実装を写経せず、`panels -> store/actions -> bridge -> core` の依存方向で再実装する。

## 判定規則

- `そのまま`: 現行の責務と呼び出し単位を維持できる。
- `直して移植`: 操作の目的は維持するが、暗黙連結、エラー非表示、更新粒度を修正する。
- `捨てる`: desktop 固有、debug/capture、実験機能、または正本を迂回する操作。
- P1 は W4 の対象、P2 は対象外とする。W2/W3 で先に実装する表示・Draw Path も表では P1 とする。
- `preview` と `commit` は同じ core API を呼び、`preview` はログへ残さない。

更新クラスと drag policy:

| 更新クラス | drag policy |
|---|---|
| UI / camera | client 内で生反映 |
| `kRedraw` | 生反映、約30 Hz |
| `kReshape` | 生反映、約30 Hz。`UpdateCableTemplate` は `preferred_visible_span_ids` を渡す |
| `kReposition` | 生反映、約15 Hz。release 時に最終値を再送 |
| `kRegenerate` / unsupported | drag 不可。release 時だけ commit |

## 主要 UI

| UI 項目 | 呼ぶ API / action | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Left/Right Panel / Workspace Width | web layout store | toggle / slider | UI | 生反映 | 直して移植 | desktop の unified/window mode は持たず、左右panelの表示と幅だけを管理する | P1 |
| Camera FOV | renderer camera action | slider | camera | 生反映 | そのまま | renderer 所有で core 非依存 | P1 |
| Ground Grid | renderer store | toggle | UI | 生反映 | そのまま | world geometryを変えずGridHelperの表示だけ切り替える | P1 |
| Reset Workspace | workspace cache action | button | Core + UI | commit | 直して移植 | factory CoreStateと既定UIへ戻し、同じ内容でcacheを置換する | P1 |
| Walk Speed / Mouse Sensitivity / Start-Stop Walk | なし | slider / button | desktop input | - | 捨てる | web は OrbitControls の orbit/pan/zoomを採用し walk mode は非対象 | P2 |
| Clear Selection | selection action | button | UI | 即時 | そのまま | store の選択だけを消す | P1 |
| Outliner: Poles / Ports / Spans / Midair SupportNodes | read model + selection action | selectable list | UI | 即時 | そのまま | topology を解釈せず公開 view の項目を表示する | P1 |
| Related entity links | selection action | small button | UI | 即時 | そのまま | 公開 inspection の参照先へ選択を移すだけ | P1 |

## Draw Path

| UI 項目 | 呼ぶ API / action | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Enable DrawPath Pick | draw store | toggle | UI | 生反映 | 直して移植 | W3 では Draw Path tool の有効状態として扱う | P1 |
| Show Backbone Overlay | renderer store | toggle | `kRedraw`相当 | 生反映 | そのまま | store 内の派生出力表示切替だけ | P1 |
| Draw branch pick | scene raycast / `ResolveBranchPick` | scene pick | UI | 生反映 | 直して移植 | sceneが公開hit kind/idを渡し、coreがbranch接続点を解決する。JSで近傍topologyを推測しない | P1 |
| Draw Plane Z | draw store | number | UI | 生反映 | そのまま | ray-plane 交差だけに使う | P1 |
| Path Interval | `GenerateFromBackboneSpec` | number | `kRegenerate` | release commit | そのまま | generation input | P1 |
| Clicked Points Only | `GenerateFromBackboneSpec` | toggle | `kRegenerate` | release commit | そのまま | `interval_m` の有無へ変換する | P1 |
| Show Preview | renderer store | toggle | `kRedraw`相当 | 生反映 | そのまま | JS は点列を表示するだけで wire geometry を補完しない | P1 |
| Keep Path After Generate | draw store | toggle | UI | 生反映 | そのまま | action 成功後の入力点列保持だけ | P1 |
| Path PoleType | generation action input | select | `kRegenerate` | release commit | そのまま | 明示 template id を渡す | P1 |
| Bundle placement / Count / Height / Offset / Spread | `BackboneSpec::bundles` | ordered records | `kRegenerate` | release commit | 直して移植 | Height/Offsetはpole-local絶対値。全category共通。同じBundleTemplateを複数配置でき、行右上の小さい`＋`は配置recordを直後へ複製する。JSでSpan topologyを作らない | P1 |
| Direction Mode / Flip Direction | generation action input | select / button | `kRegenerate` | release commit | そのまま | `BackboneSpec` の明示入力 | P1 |
| Generate From Path | `GenerateFromBackboneSpec` | button | `kRegenerate` | commit | 直して移植 | W3 の action。`EditResult.error`を必ず表示する | P1 |
| Undo Last Point / Clear Path | draw store | button | UI | 即時 | そのまま | committed core state は変更しない | P1 |
| Bundle Population Rules | `UpdateBundleTemplate(population_rules)` | folded rule editor | `kReshape` | number preview / count commit | 直して移植 | population は BundleTemplate の性質。Draw Path の global toggle/seed は置かず、rule が空なら追加線なし | P1 |
| Save Repro Trace | web viewer action | header button | debug/capture | explicit | 直して移植 | 操作順、生成入力、anchor と VisualCurvePart の種類別件数だけを text download する。curve sample と UI log は含めない | P2 |

## Geometry / layout / orientation

| UI 項目 | 呼ぶ core API | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Curve Samples | `UpdateGeometrySettings` | slider | `kReshape` | 約30 Hz | そのまま | curve 派生を更新する | P1 |
| Sag Enabled / Sag Factor | `UpdateGeometrySettings` | toggle / number | `kReshape` | 約30 Hz | そのまま | layout endpoint は変えず geom/draw を更新する | P1 |
| Pole Clearance | `UpdateGeometrySettings` | number | `kReshape` | 約30 Hz | 直して移植 | API が受ける設定値としてのみ扱い、JS で clearance を解かない | P1 |
| Apply Geometry | `UpdateGeometrySettings` | button | `kReshape` | commit | 直して移植 | web では各 control の preview/commit actionへ分解し、Apply button は置かない | P1 |
| Angle Correction Enabled / Corner Threshold / Min-Max Side Scale | `UpdateLayoutSettings` | toggle / number | `kRegenerate` | release commit | そのまま | backbone span 存在時も統一 regenerate で反映する。error は core 結果を表示する | P1 |
| Apply Layout | `UpdateLayoutSettings` | button | `kRegenerate` | commit | 直して移植 | form commit action とする。unsupported を隠さない | P1 |
| Max Tilt | generation input / `ApplyPoleTilt` | number | `kReposition` | 約15 Hz | 直して移植 | 新規 Draw Path は `BackboneSpec.pole_placement`、既存 pole 編集は独立 action | P1 |
| Apply Tilt To Selected Poles | `ApplyPoleTilt(selected ids)` | button | `kReposition` | commit | そのまま | 選択対象を明示する | P1 |
| Select Poles / Midair / Spans | selection filter store | toggle | UI | 生反映 | そのまま | core mutation なし | P1 |
| Apply Tilt To All Poles | `ApplyPoleTilt(all pole ids)` | button | `kReposition` | commit | 直して移植 | 全件対象を action が明示収集する。Draw Path の後処理にはしない | P1 |
| Reset All Span Reference Lengths | `ResetAllSpanReferenceLengths` | button | `kReshape` | commit | そのまま | 独立した明示操作 | P1 |

## Cable template

| UI 項目 | 呼ぶ core API | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Cable Template / Name | template read / `UpdateCableTemplate` | select / read-only text | mixed | release commit | そのまま | stable id で選択する。現行desktopもnameはread-only | P1 |
| Outer Diameter / Bend Stiffness / Min Bend Radius | `UpdateCableTemplate` | number | `kReshape` | 約30 Hz | そのまま | `preferred_visible_span_ids` を必ず渡す | P1 |
| Cable Material | `UpdateCableTemplate` | select | `kRedraw` | 約30 Hz | そのまま | render/visual のみ | P1 |
| Requires Insulator / Insulator Attach Height | `UpdateCableTemplate` | toggle / number | `kRedraw` | commit | そのまま | endpoint detail curveが設定を消費する | P1 |
| Sag Factor / Slack Factor | `UpdateCableTemplate` | number | `kReshape` | 約30 Hz | そのまま | geom/draw の再導出 | P1 |
| Cable Continuity | `UpdateCableTemplate` | select | `kRegenerate` | release commit | そのまま | decision 差分。現行保留条件を表示する | P1 |
| CurveOffset Straight Supplemental | `UpdateCableTemplate` | toggle | `kReshape` | 約30 Hz | そのまま | explicit supplemental visual setting | P1 |
| Supplemental Lateral / Vertical Offset | `UpdateCableTemplate` | number | `kReshape` | 約30 Hz | そのまま | JS で offset geometry を作らない | P1 |
| Supplemental Wobble Amplitude / Wavelength / Phase / Endpoint Envelope | `UpdateCableTemplate` | number | `kReshape` | 約30 Hz | そのまま | core の curve 出力を再取得する | P1 |
| Apply Cable Template | `UpdateCableTemplate` | button | mixed | commit | 直して移植 | web では field の更新クラスに従う action に分ける | P1 |

## Bundle template

| UI 項目 | 呼ぶ core API | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Bundle Template / Name / Description | template read / `UpdateBundleTemplate` | select / text | mixed | release commit | そのまま | stable id で編集する | P1 |
| Bundle Cable Template / Related Pole Template / Default Layer | `UpdateBundleTemplate` | select | `kRegenerate` | release commit | 直して移植 | definition 更新だけを行い related pole 適用を暗黙に呼ばない | P1 |
| Midair Node / Midair Branch | `UpdateBundleTemplate` | toggle | `kRegenerate` | release commit | そのまま | branch pick policyが消費する | P1 |
| Span Visual Assembly / Support Band | `UpdateBundleTemplate` | folded editor | `kReshape` | commit | 直して移植 | helix、containment、wander、twistの正本設定 | P1 |
| Apply Bundle Template | `UpdateBundleTemplate` | button | mixed | commit | 直して移植 | `ApplyBundleRelatedPoleTypeToExistingPoles` の自動呼出を禁止する | P1 |
| Apply related pole type | `ApplyBundleRelatedPoleTypeToExistingPoles` | 独立 button | `kRegenerate` | commit | 直して移植 | 現行は Bundle/Pole Apply 後に暗黙実行される。web では独立操作にし error を表示する | P1 |

## Pole template

| UI 項目 | 呼ぶ core API | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Pole Template Name / Description | `UpdatePoleTypeDefinition` | text | definition | release commit | そのまま | stable id の definition 更新 | P1 |
| Pole Default Height | `UpdatePoleTypeDefinition` | number | `kReposition` | 約15 Hz | そのまま | placement-only 差分 | P1 |
| Pole Placement: category Height / Offset / Spread | `UpdatePoleTypeDefinition` | number | `kReposition` | 約15 Hz | そのまま | band 配置差分。構造差分を混ぜない | P1 |
| Advanced Port Bands: Band Id / Category / Layer / Side / Role | `UpdatePoleTypeDefinition` | integer / select | `kRegenerate` | release commit | そのまま | identity/placement structure 差分 | P1 |
| Advanced Port Bands: Lateral Center-Min-Max / Height Center-Min-Max / Priority / Min Spacing | `UpdatePoleTypeDefinition` | number | mixed | release commit | そのまま |既存構造内の位置差分だけなら reposition、その他は core の判定に従う | P1 |
| Advanced Port Bands: Allow Multiple / Overflow Policy / Enabled | `UpdatePoleTypeDefinition` | toggle / select | `kRegenerate` | release commit | そのまま | structure/decision 差分 | P1 |
| Add / Remove Port Band | `UpdatePoleTypeDefinition` | button | `kRegenerate` | commit | そのまま | active backbone 使用時も統一 regenerate で反映する。error は core 結果を表示する | P1 |
| Anchor Slot: Slot Id / Usage / Local XYZ / Priority / Enabled | `UpdatePoleTypeDefinition` | integer / select / number / toggle | `kRegenerate` | release commit | そのまま | support structure 差分 | P1 |
| Add / Remove Anchor Slot | `UpdatePoleTypeDefinition` | button | `kRegenerate` | commit | そのまま | active backbone 使用時も統一 regenerate で反映する。error は core 結果を表示する | P1 |
| Apply Pole Template | `UpdatePoleTypeDefinition` | button | mixed | commit | 直して移植 | `ApplyBundleRelatedPoleTypeToExistingPoles` の自動呼出を禁止する | P1 |

## Visual / inspection

| UI 項目 | 呼ぶ core API / source | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Enable Support Structures / Enable Insulators | `UpdateVisualSettings` | toggle | `kRedraw` | 約30 Hz | そのまま | visual cache の決定済み値だけを表示する | P1 |
| Support Center Threshold / Arm Extra / Insulator Radius-Length | `UpdateVisualSettings` | number | `kRedraw` | 約30 Hz | そのまま | core visual output を再取得する | P1 |
| Solid Support Render | renderer store | toggle | `kRedraw` | 生反映 | そのまま | backend 表示方式だけ | P1 |
| Apply Visual Cache Settings | `UpdateVisualSettings` | button | `kRedraw` | commit | 直して移植 | web では各 control の preview/commit に分解する | P1 |
| Clear Pole Orientation Override | `ClearPoleOrientationOverride` | button | `kReposition` | commit | そのまま | 独立操作、error 表示必須 | P1 |
| Clear Span Socket Override A/B | `ClearSpanEndpointSocketOverride` | button | `kRegenerate` | commit | そのまま | backbone span でも統一 regenerate で反映する。error は core 結果を表示する | P1 |
| Clear Branch Down Offset Override | `ClearSpanBranchDownOffsetOverride` | button | `kRegenerate` | commit | そのまま | backbone span でも統一 regenerate で反映する。error は core 結果を表示する | P1 |
| Selected entity summary / neutral span output | 公開 `CoreView` / wasm read model | read-only | read | - | 直して移植 | W2/W4 では主要 id・種類・出力有無に絞り、desktop の全文 dump は移さない | P1 |
| Pole Height Debug: Show Ports / Supports / Bundles | なし | toggle | debug | - | 捨てる | debug canvas 専用 | P2 |
| Pole Height marker drag | `SetPortWorldPositionManual` | invisible drag | `kReposition` | - | 捨てる | debug canvas から正本を手動化する操作で通常編集ではない | P2 |
| Show Span AABB / Segment AABB / Visual Curve Controls / Highlight Selected Bundle | renderer debug store | toggle | debug | - | 捨てる | W4 は P2 debug を実装しない | P2 |
| Backbone Junction Debug / validation dump / Stats | 公開 query | read-only | debug | - | 捨てる | desktop 診断表示。error panel は別途 P1 として持つ | P2 |

## 監査結果

直接 control callsite は次の130件を照合した。

| file | button | slider | input | checkbox | combo | invisible button | 合計 |
|---|---:|---:|---:|---:|---:|---:|---:|
| `viewer/src/panels.cpp` | 20 | 5 | 44 | 25 | 15 | 2 | 111 |
| `viewer/src/draw_path.cpp` | 5 | 0 | 5 | 6 | 3 | 0 | 19 |
| `viewer/src/render_overlay.cpp` | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

`Selectable` は combo の候補と outliner の反復項目なので、独立操作数には加算していない。
`InputTextString` と動的 label の related link / object list は、それぞれ上表の template text、related links、
outliner として監査した。

現行の既知問題:

- `Apply Bundle Template` は成功後に `ApplyBundleRelatedPoleTypeToExistingPoles` を暗黙実行する。
- `Apply Pole Template` も成功後に同 API を暗黙実行する。
- 後段が失敗すると先行した template 更新は既に成功しているのに、UI は一つの Apply が失敗したように見える。
- web viewer では template 更新と related pole 適用を別 action / button にし、各 `EditResult.error` を表示する。
- template の未対応 decision/structure 差分は [merge_readiness.md](merge_readiness.md) の条件で unsupported になり得る。
  `UpdateLayoutSettings` と span override は backbone span でも統一 regenerate で反映する。事前推測せず core の error を表示する。

## W4 実装結果

P1 の実装済み family:

- Draw Path: ground click、pole/bundle選択、count、interval、clicked-only、direction、preview、
  overlay、keep path、undo、clear、generation error。
- camera/workspace: OrbitControls、FOV、panel表示、panel幅。
- geometry/layout/visual: 更新クラスに従う preview/commit、Esc cancel、error表示。
- orientation: selected/all pole tilt、span reference length reset、selection filter。
- Cable/Bundle/Pole template: current値の読込と明示 update。Pole band/anchor slotを含む。
- related pole type: 独立button。Bundle/Pole template updateから暗黙呼出ししない。
- outliner/inspection: pole/port/span/midair一覧、主要値、選択解除。
- override clear: pole orientation、span socket A/B、branch down。

P1 だが意図的に除外した項目:

- Related entity links: W1の最小inspection境界にlinkを含めていない。ID関係をJSで再構成しない。
- Unified UI toggle: webは単一workspaceを唯一の構成とし、旧window modeを持たない。
- direct object edit: desktop側も無効であり、Draw Pathと明示template/update actionだけを入口にする。

P2 は Repro Trace のみ実装済みである。walk modeは未実装である。

## Workspace persistence

web viewerはversion付きworkspace documentをbrowser local storageへ自動保存する。
documentはCoreのauthoritative save文字列と、編集中path・template選択・表示・panel配置等の
Web設定を同じ単位で保持し、次回起動時にCoreを先にloadしてからWeb設定とderived sceneを復元する。
parts、catalog、timing、log等の再取得可能な値は保存しない。

Store変更は250ms debounceで保存し、page unloadでは同期flushする。`Reset`は起動時のfactory
CoreStateと既定Web設定へ戻し、その状態でcacheを置換する。手動Save/LoadはCore archiveの
入出力として残し、Load後のstateは次のcache flushに含める。
