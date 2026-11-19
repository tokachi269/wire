# Viewer 操作インベントリ

この文書はdesktop viewerの操作を監査し、web viewerへ移す範囲を固定する。desktop viewerの実装をそのまま複製せず、`panels -> store/actions -> bridge -> core`の依存方向で再実装する。

## 判定規則

- そのまま: 現行の責務と呼び出し単位を維持できる。
- 直して移植: 操作の目的は維持するが、暗黙連結、エラー非表示、更新粒度を修正する。
- 捨てる: desktop固有、debug、実験機能、または正本を迂回する操作。
- P1はW4の対象、P2は対象外とする。W2/W3で先に実装する表示・Draw Pathも表ではP1とする。
- `preview`と`commit`は同じcore APIを呼び、`preview`はログへ残さない。

更新クラスごとのdrag時の反映方法を示す。

| 更新クラス | drag policy |
|---|---|
| UI / camera | client内で生反映 |
| `kRedraw` | 生反映、約30 Hz |
| `kReshape` | 生反映、約30 Hz。`UpdateCableTemplate`は`preferred_visible_span_ids`を渡す |
| `kReposition` | 生反映、約15 Hz。release時に最終値を再送 |
| `kRegenerate` / unsupported | drag不可。release時だけcommit |

## 主要 UI

| UI項目 | 呼ぶAPI / action | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Left/Right Panel / Workspace Width | web layout store | toggle / slider | UI | 生反映 | 直して移植 | desktopのunified/window modeは持たず、左右panelの表示と幅だけを管理する | P1 |
| Camera FOV | renderer camera action | slider | camera | 生反映 | そのまま | renderer所有でcore非依存 | P1 |
| Ground Grid | renderer store | toggle | UI | 生反映 | そのまま | world幾何を変えずGridHelperの表示だけ切り替える | P1 |
| Reset Workspace | workspaceキャッシュaction | button | Core + UI | commit | 直して移植 | factory CoreStateと既定UIへ戻し、同じ内容でキャッシュを置換する | P1 |
| Walk Speed / Mouse Sensitivity / Start-Stop Walk | なし | slider / button | desktop入力 | - | 捨てる | webはOrbitControlsのorbit/pan/zoomを採用しwalk modeは非対象 | P2 |
| Clear Selection | selection action | button | UI | 即時 | そのまま | storeの選択だけを消す | P1 |
| Outliner: Poles / Ports / Spans / Midair SupportNodes | readモデル + selection action | selectable list | UI | 即時 | そのまま | topologyを解釈せず公開viewの項目を表示する | P1 |
| Related entity links | selection action | small button | UI | 即時 | そのまま | 公開inspectionの参照先へ選択を移すだけ | P1 |

## Draw Path

| UI項目 | 呼ぶAPI / action | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Enable DrawPath Pick | draw store | toggle | UI | 生反映 | 直して移植 | W3ではDraw Path toolの有効状態として扱う | P1 |
| Show Backbone Overlay | renderer store | toggle | `kRedraw`相当 | 生反映 | そのまま | store内の派生出力表示切替だけ | P1 |
| Draw branch pick | scene raycast / `ResolveBranchPick` | scene pick | UI | 生反映 | 直して移植 | sceneが公開hit kind/idを渡し、coreがbranch接続点を解決する。JSで近傍topologyを推測しない | P1 |
| Draw Plane Z | draw store | number | UI | 生反映 | そのまま | ray-plane交差だけに使う | P1 |
| Path Interval | `GenerateFromBackboneSpec` | number | `kRegenerate` | release commit | そのまま | generation入力 | P1 |
| Clicked Points Only | `GenerateFromBackboneSpec` | toggle | `kRegenerate` | release commit | そのまま | `interval_m`の有無へ変換する | P1 |
| Show Preview | renderer store | toggle | `kRedraw`相当 | 生反映 | そのまま | JSは点列を表示するだけでwire幾何を補完しない | P1 |
| Keep Path After Generate | draw store | toggle | UI | 生反映 | そのまま | action成功後の入力点列保持だけ | P1 |
| Path PoleType | generation action入力 | select | `kRegenerate` | release commit | そのまま | 明示テンプレートidを渡す | P1 |
| Bundle placement / Count / Height / Offset / Spread | `BackboneSpec::bundles` | ordered records | `kRegenerate` | release commit | 直して移植 | Height/Offsetはpole-local絶対値。全category共通。同じBundleTemplateを複数配置でき、行右上の小さい`＋`は配置recordを直後へ複製する。JSでSpan topologyを作らない | P1 |
| Direction Mode / Flip Direction | generation action入力 | select / button | `kRegenerate` | release commit | そのまま | `BackboneSpec`の明示入力 | P1 |
| Generate From Path | `GenerateFromBackboneSpec` | button | `kRegenerate` | commit | 直して移植 | W3のaction。`EditResult.error`を必ず表示する | P1 |
| Undo Last Point / Clear Path | draw store | button | UI | 即時 | そのまま | committed core状態は変更しない | P1 |
| Bundle Population Rules | `UpdateBundleTemplate(population_rules)` | folded rule editor | `kReshape` | number preview / count commit | 直して移植 | populationはBundleTemplateの性質。Draw Pathのglobal toggle/seedは置かず、ruleが空なら追加線なし | P1 |

## Geometry / layout / orientation

| UI項目 | 呼ぶcore API | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Curve Samples | `UpdateGeometrySettings` | slider | `kReshape` | 約30 Hz | そのまま | curve派生を更新する | P1 |
| Sag Enabled / Sag Factor | `UpdateGeometrySettings` | toggle / number | `kReshape` | 約30 Hz | そのまま | layout endpointは変えずgeom/drawを更新する | P1 |
| Pole Clearance | `UpdateGeometrySettings` | number | `kReshape` | 約30 Hz | 直して移植 | APIが受ける設定値としてのみ扱い、JSでclearanceを解かない | P1 |
| Apply Geometry | `UpdateGeometrySettings` | button | `kReshape` | commit | 直して移植 | webでは各controlのpreview/commit actionへ分解し、Apply buttonは置かない | P1 |
| Angle Correction Enabled / Corner Threshold / Min-Max Side Scale | `UpdateLayoutSettings` | toggle / number | `kRegenerate` | release commit | そのまま | backbone span存在時も統一regenerateで反映する。errorはcore結果を表示する | P1 |
| Apply Layout | `UpdateLayoutSettings` | button | `kRegenerate` | commit | 直して移植 | form commit actionとする。unsupportedを隠さない | P1 |
| Max Tilt | generation入力 / `ApplyPoleTilt` | number | `kReposition` | 約15 Hz | 直して移植 | 新規Draw Pathは`BackboneSpec.pole_placement`、既存pole編集は独立action | P1 |
| Apply Tilt To Selected Poles | `ApplyPoleTilt(selected ids)` | button | `kReposition` | commit | そのまま | 選択対象を明示する | P1 |
| Select Poles / Midair / Spans | selection filter store | toggle | UI | 生反映 | そのまま | core状態変更なし | P1 |
| Apply Tilt To All Poles | `ApplyPoleTilt(all pole ids)` | button | `kReposition` | commit | 直して移植 | 全件対象をactionが明示収集する。Draw Pathの後処理にはしない | P1 |
| Reset All Span Reference Lengths | `ResetAllSpanReferenceLengths` | button | `kReshape` | commit | そのまま | 独立した明示操作 | P1 |

## Cable template

| UI項目 | 呼ぶcore API | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Cable Template / Name | テンプレートread / `UpdateCableTemplate` | select / read-only text | mixed | release commit | そのまま | stable idで選択する。現行desktopもnameはread-only | P1 |
| Outer Diameter / Bend Stiffness / Min Bend Radius | `UpdateCableTemplate` | number | `kReshape` | 約30 Hz | そのまま | `preferred_visible_span_ids`を必ず渡す | P1 |
| Cable Material | `UpdateCableTemplate` | select | `kRedraw` | 約30 Hz | そのまま | render/visualのみ | P1 |
| Requires Insulator / Insulator Attach Height | `UpdateCableTemplate` | toggle / number | `kRedraw` | commit | そのまま | endpoint detail curveが設定を消費する | P1 |
| Sag Factor | `UpdateCableTemplate` | number | `kReshape` | 約30 Hz | そのまま | geom/drawの再導出 | P1 |
| Cable Continuity | `UpdateCableTemplate` | select | `kRegenerate` | release commit | そのまま | decision差分。現行保留条件を表示する | P1 |
| CurveOffset Straight Supplemental | `UpdateCableTemplate` | toggle | `kReshape` | 約30 Hz | そのまま | explicit supplemental visual setting | P1 |
| Supplemental Lateral / Vertical Offset | `UpdateCableTemplate` | number | `kReshape` | 約30 Hz | そのまま | JSでoffset幾何を作らない | P1 |
| Supplemental Wobble Amplitude / Wavelength / Phase / Endpoint Envelope | `UpdateCableTemplate` | number | `kReshape` | 約30 Hz | そのまま | coreのcurve出力を再取得する | P1 |
| Apply Cable Template | `UpdateCableTemplate` | button | mixed | commit | 直して移植 | webではフィールドの更新クラスに従うactionに分ける | P1 |

## Bundle template

| UI項目 | 呼ぶcore API | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Bundle Template / Name / Description | テンプレートread / `UpdateBundleTemplate` | select / text | mixed | release commit | そのまま | stable idで編集する | P1 |
| Bundle Cable Template / Related Pole Template / Default Layer | `UpdateBundleTemplate` | select | `kRegenerate` | release commit | 直して移植 | definition更新だけを行いrelated pole適用を暗黙に呼ばない | P1 |
| Midair Node / Midair Branch | `UpdateBundleTemplate` | toggle | `kRegenerate` | release commit | そのまま | branch pick policyが消費する | P1 |
| Span Visual Assembly / Support Band | `UpdateBundleTemplate` | folded editor | `kReshape` | commit | 直して移植 | helix、containment、wander、twistの正本設定 | P1 |
| Apply Bundle Template | `UpdateBundleTemplate` | button | mixed | commit | 直して移植 | `ApplyBundleRelatedPoleTypeToExistingPoles`の自動呼出を禁止する | P1 |
| Apply related pole type | `ApplyBundleRelatedPoleTypeToExistingPoles` | 独立button | `kRegenerate` | commit | 直して移植 | 現行はBundle/Pole Apply後に暗黙実行される。webでは独立操作にしerrorを表示する | P1 |

## Pole template

| UI項目 | 呼ぶcore API | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Pole Template Name / Description | `UpdatePoleTypeDefinition` | text | definition | release commit | そのまま | stable idのdefinition更新 | P1 |
| Pole Default Height | `UpdatePoleTypeDefinition` | number | `kReposition` | 約15 Hz | そのまま | placement-only差分 | P1 |
| Pole Placement: category Height / Offset / Spread | `UpdatePoleTypeDefinition` | number | `kReposition` | 約15 Hz | そのまま | band配置差分。構造差分を混ぜない | P1 |
| Advanced Port Bands: Band Id / Category / Layer / Side / Role | `UpdatePoleTypeDefinition` | integer / select | `kRegenerate` | release commit | そのまま | identity/placement structure差分 | P1 |
| Advanced Port Bands: Lateral Center-Min-Max / Height Center-Min-Max / Priority / Min Spacing | `UpdatePoleTypeDefinition` | number | mixed | release commit | そのまま |既存構造内の位置差分だけならreposition、その他はcoreの判定に従う | P1 |
| Advanced Port Bands: Allow Multiple / Overflow Policy / Enabled | `UpdatePoleTypeDefinition` | toggle / select | `kRegenerate` | release commit | そのまま | structure/decision差分 | P1 |
| Add / Remove Port Band | `UpdatePoleTypeDefinition` | button | `kRegenerate` | commit | そのまま | active backbone使用時も統一regenerateで反映する。errorはcore結果を表示する | P1 |
| Anchor Slot: Slot Id / Usage / Local XYZ / Priority / Enabled | `UpdatePoleTypeDefinition` | integer / select / number / toggle | `kRegenerate` | release commit | そのまま | support structure差分 | P1 |
| Add / Remove Anchor Slot | `UpdatePoleTypeDefinition` | button | `kRegenerate` | commit | そのまま | active backbone使用時も統一regenerateで反映する。errorはcore結果を表示する | P1 |
| Apply Pole Template | `UpdatePoleTypeDefinition` | button | mixed | commit | 直して移植 | `ApplyBundleRelatedPoleTypeToExistingPoles`の自動呼出を禁止する | P1 |

## Visual / inspection

| UI項目 | 呼ぶcore API / source | control | 更新クラス | drag policy | 判定 | 理由 | 優先度 |
|---|---|---|---|---|---|---|---|
| Enable Insulators | `UpdateVisualSettings` | toggle | `kRedraw` | 約30 Hz | そのまま | visualキャッシュの決定済み値だけを表示する | P1 |
| Support Center Threshold / Arm Extra / Insulator Radius-Length | `UpdateVisualSettings` | number | `kRedraw` | 約30 Hz | そのまま | core visual出力を再取得する | P1 |
| Solid Support Render | renderer store | toggle | `kRedraw` | 生反映 | そのまま | backend表示方式だけ | P1 |
| Apply Visual Cache Settings | `UpdateVisualSettings` | button | `kRedraw` | commit | 直して移植 | webでは各controlのpreview/commitに分解する | P1 |
| Clear Pole Orientation Override | `ClearPoleOrientationOverride` | button | `kReposition` | commit | そのまま | 独立操作、error表示必須 | P1 |
| Clear Span Socket Override A/B | `ClearSpanEndpointSocketOverride` | button | `kRegenerate` | commit | そのまま | backbone spanでも統一regenerateで反映する。errorはcore結果を表示する | P1 |
| Clear Branch Down Offset Override | `ClearSpanBranchDownOffsetOverride` | button | `kRegenerate` | commit | そのまま | backbone spanでも統一regenerateで反映する。errorはcore結果を表示する | P1 |
| Selected entity summary / neutral span出力 | 公開`CoreView` / wasm readモデル | read-only | read | - | 直して移植 | W2/W4では主要id・種類・出力有無に絞り、desktopの全文dumpは移さない | P1 |
| Pole Height Debug: Show Ports / Supports / Bundles | なし | toggle | debug | - | 捨てる | debug canvas専用 | P2 |
| Pole Height marker drag | `SetPortWorldPositionManual` | invisible drag | `kReposition` | - | 捨てる | debug canvasから正本を手動化する操作で通常編集ではない | P2 |
| Show Span AABB / Segment AABB / Visual Curve Controls / Highlight Selected Bundle | renderer debug store | toggle | debug | - | 捨てる | W4はP2 debugを実装しない | P2 |
| Backbone Junction Debug / 検証dump / Stats | 公開query | read-only | debug | - | 捨てる | desktop診断表示。error panelは別途P1として持つ | P2 |

## 監査結果

直接control callsiteは次の130件を照合した。

| file | button | slider | 入力 | checkbox | combo | invisible button | 合計 |
|---|---:|---:|---:|---:|---:|---:|---:|
| `viewer/src/panels.cpp` | 20 | 5 | 44 | 25 | 15 | 2 | 111 |
| `viewer/src/draw_path.cpp` | 5 | 0 | 5 | 6 | 3 | 0 | 19 |
| `viewer/src/render_overlay.cpp` | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

`Selectable`はcomboの候補とoutlinerの反復項目なので、独立操作数には加算していない。`InputTextString`と動的labelのrelated link / object listは、それぞれ上表のテンプレートtext、related links、outlinerとして監査した。

現在、次の問題を把握している。

- `Apply Bundle Template`は成功後に`ApplyBundleRelatedPoleTypeToExistingPoles`を暗黙実行する。
- `Apply Pole Template`も成功後に同APIを暗黙実行する。
- 後段が失敗すると先行したテンプレート更新は既に成功しているのに、UIは一つのApplyが失敗したように見える。
- web viewerではテンプレート更新とrelated pole適用を別action / buttonにし、各`EditResult.error`を表示する。
- テンプレートの未対応decision/structure差分は [merge_readiness.md](../merge_readiness.md) の条件でunsupportedになり得る。
  `UpdateLayoutSettings`とspan overrideはbackbone spanでも統一regenerateで反映する。事前推測せずcoreのerrorを表示する。

## W4 実装結果

P1の実装済みfamily。

- Draw Path: ground click、pole/bundle選択、count、interval、clicked-only、direction、preview、
  overlay、keep path、undo、clear、generation error。
- camera/workspace: OrbitControls、FOV、panel表示、panel幅。
- geometry/layout/visual: 更新クラスに従うpreview/commit、Esc cancel、error表示。
- orientation: selected/all pole tilt、span reference length reset、selection filter。
- Cable/Bundle/Poleテンプレート: current値の読込と明示update。Pole band/anchor slotを含む。
- related pole type: 独立button。Bundle/Poleテンプレートupdateから暗黙呼出ししない。
- outliner/inspection: pole/port/span/midair一覧、主要値、選択解除。
- override clear: pole orientation、span socket A/B、branch down。

P1だが意図的に除外した項目。

- Related entity links: W1の最小inspection境界にlinkを含めていない。ID関係をJSで再構成しない。
- Unified UI toggle: webは単一workspaceを唯一の構成とし、旧window modeを持たない。
- direct object edit: desktop側も無効であり、Draw Pathと明示template/update actionだけを入口にする。

## Workspace persistence

web viewerはバージョン付きworkspace documentをbrowser local storageへ自動保存する。documentはCoreのauthoritative save文字列と、編集中path・テンプレート選択・表示・panel配置等のWeb設定を同じ単位で保持し、次回起動時にCoreを先にloadしてからWeb設定とderived sceneを復元する。

parts、catalog、timing、log等の再取得可能な値は保存しない。

Store変更は250ms debounceで保存し、page unloadでは同期flushする。`Reset`は起動時のfactory CoreStateと既定Web設定へ戻し、その状態でキャッシュを置換する。手動Save/LoadはCore archiveの入出力として残し、Load後の状態は次のキャッシュflushに含める。

### Smoke確認

- Saveで状態textを保存し、別操作後にLoadすると保存時のpole / span / curve表示へ戻る。
- バージョン不一致またはtruncatedな状態textをLoadするとerrorが表示され、現在の表示と状態は変わらない。
