# Core Test Case Matrix (Phase4.8)

## Scope and policy
- 観測根拠: 公開API戻り値、`view()`、`connection_index()`、`relation_index()`、`find_*cache()`、`slot_selection_debug_records()`、`last_path_direction_debug()`、`last_lane_assignments()`、`last_generation_backbone()`、`Validate()` のみ。
- 期待値粒度: `Exact` は決定論のみ、`Invariant` は不変条件のみ。
- モック方針: ドメインロジックのモック禁止（本スイートはモック未使用）。
- 時間/並行: 実時間待ち・非決定並行を使わない。

| Case ID | 目的 | 前提 | 入力 | 期待結果 | 観測点 | 壊れた時に守りたいユーザ価値 |
|---|---|---|---|---|---|---|
| C01 | ID生成の単調増加 | 新規IdGenerator | next/peek/reset | Exact: 連番・重複なし | 戻り値 | 永続ID衝突防止 |
| C02 | ObjectStore整合 | 空Store | insert/find/remove/upsert | Exact: 件数・参照整合 | 公開API | 参照崩壊防止 |
| C03 | Span追加初期Dirty | 有効Port2点 | AddSpan | Exact: runtime生成+dirty | runtime | 再計算漏れ防止 |
| C04 | MovePole局所Dirty | 独立Span2本 | MovePole | Invariant: 関連Spanのみdirty | runtime | 無関係再計算抑制 |
| C05 | SplitSpan整合 | Span1本 | SplitSpan | Invariant: 旧削除+新2本+新Port | stores/index/Validate | 分岐崩壊防止 |
| C06 | PoleType適用 | PoleTypeあり | ApplyPoleType | Invariant: owner整合でPort生成 | PoleDetail | テンプレ適用維持 |
| C07 | PoleType差分 | PoleType2種 | 各Type適用 | Invariant: slot構成差 | source_slot | 型差分維持 |
| C08 | 未使用slot優先 | PoleType適用2Pole | AddConnectionByPole×2 | Invariant: 別slot選択 | result.slot | 線重なり低減 |
| C09 | Pole接続整合 | PoleType適用2Pole | AddConnectionByPole | Invariant: Span/index/dirty整合 | spans/index/runtime | 主導線成立 |
| C10 | 同一Pole接続拒否 | Pole1本 | 同一Pole接続後に正常操作 | Exact: fail+復帰可 | error/後続成功 | 誤操作耐性 |
| C11 | 柱起点引込 | PoleType適用済 | AddDropFromPole | Invariant: service span生成 | spans/ports | 引込崩壊防止 |
| C12 | 線起点引込 | Span1本 | AddDropFromSpan | Invariant: split+drop整合 | spans/index | 分岐引込維持 |
| C13 | 直線幾何決定性 | 同一状態 | 再生成 | Exact: 点列一致 | CurveCache | 再現性 |
| C14 | サグ基本 | Span1本 | line→sag | Invariant: 端点一致+中点低下 | CurveCache | 接続維持 |
| C15 | Version追随局所性 | 独立Span2本 | Port移動→再計算 | Exact: 対象のみ追随 | runtime | 局所更新維持 |
| C16 | Bounds有効性 | Span1本 | 再計算 | Invariant: AABB有効 | BoundsCache | 前段データ健全 |
| C17 | Bounds追随 | Span1本 | sag設定変更 | Invariant: AABB更新 | BoundsCache | 表示追随 |
| C18 | デモ密度 | make_demo_state | なし | Invariant: 最低件数以上 | edit_state | 初期視認性 |
| C19 | 道路Pole生成 | PoleTypeあり | GeneratePolesAlongRoad | Invariant: 本数/Type/RoadAuto | poles/generation | 自動配置信頼性 |
| C20 | 短polyline拒否 | PoleTypeあり | GenerateSimpleLine(点1) | Exact: fail+状態不変 | error/count | 入力ミス耐性 |
| C21 | interval不正拒否 | PoleTypeあり | GenerateSimpleLine(interval<=0) | Exact: fail+状態不変 | error/count | 設定ミス耐性 |
| C22 | 存在しないPort拒否 | 空状態 | AddSpan(無効ID) | Exact: fail+状態不変 | error/count | 参照不正耐性 |
| C23 | Split t不正拒否 | Span1本 | SplitSpan(t=0) | Exact: fail+復帰可 | error/spans | 失敗後復帰 |
| C24 | 隣接Pole自動接続 | Pole列 | GenerateSpansBetweenPoles | Exact: n-1生成 | result/index | 欠線防止 |
| C25 | 複数パス増加 | Pole列 | GenerateSpansBetweenPoles×6 | Exact: 毎回n-1増加 | span総数 | 頭打ち回帰防止 |
| C26 | 第3slot利用 | PoleType適用列 | 低圧自動接続×3 | Invariant: 3slot以上利用 | source_slot集合 | 2本固定回帰防止 |
| C27 | SimpleLine統合 | 有効折れ線 | GenerateSimpleLine→再計算 | Invariant: 生成+Version追随 | result/runtime/cache | 一発生成成立 |
| C28 | through連続性 | 直線入力 | GenerateSimpleLine | Invariant: 中間Pole同Port再利用 | span端点Port | 幹線連続維持 |
| C29 | 表示ID採番 | 新規CoreState | Pole/Port/Span追加 | Exact: prefix別連番 | display_id | UI追跡性 |
| C30 | Pole文脈分類 | 直線+折れ線 | GeneratePolesAlongRoad | Invariant: Terminal/Straight/Corner | pole.context | 文脈基盤維持 |
| C31 | 角補正有界 | 補正ON | 折れ線生成 | Invariant: sideScale有界+有限 | pole/port | 補正暴走防止 |
| C32 | 文脈別選定 | 3Pole | Trunk接続+Branch接続 | Invariant: 選定傾向差 | slot_id | 分岐競合低減 |
| C33 | 決定的タイブレーク | 同一入力2回 | AddConnectionByPole | Exact: 同slot+debug整合 | debug records | 再現性 |
| C34 | Corner文脈統合 | 折れ線 | GenerateSimpleLine | Invariant: CornerPass含有 | span.context | 角付き路線維持 |
| C35 | 内外補正差 | 左折/右折 | GeneratePolesAlongRoad | Invariant: 外側オフセット>内側 | turn_sign/slot座標 | 角圧縮の低減 |
| C61 | 鋭角自動拡幅 | 鋭角/鈍角の同一テンプレート比較 | GenerateSimpleLineFromPoints | Invariant: 鋭角の左右レーン間隔が鈍角より広い（カテゴリ非依存） | corner poleのlocal Y差 | 鋭角での線間距離不足防止 |
| C62 | 群レーンねじれ抑制 | U字Guide + HV3 lane | GenerateFromBackboneSpec(HV_3PH) | Invariant: 区間ごとのlane順逆転数が0 | lane assignment port local Y順 | 群配線のクロス抑制 |
| C76 | 鋭角コーナーlane順反転抑制 | 鋭角コーナーを含むGuide + COMM4 lane | GenerateFromBackboneSpec(COMM,count=4) | Invariant: 区間ごとのlane順逆転数が0 | lane assignmentのport local Y順 | 鋭角時の見た目破綻防止 |
| C99 | HV3キャプチャ形状の反転回帰 | ねじれ再現点列（6点）+ HV3 | GenerateFromBackboneSpec(HV_3PH) | Invariant: 区間ごとのlane順逆転数が0 | last_lane_assignments / port local Y順 | 実運用形状でのねじれ再発防止 |
| C100 | Midair SupportNode保持 | 3点Pathの中点をMidair指定 | GenerateFromBackboneSpec(LV) | Invariant: `last_generation_backbone.nodes` にMidairノードとtangent hintが残る | last_generation_backbone.nodes | Pole固定前提への逆戻り防止 |
| C101 | HV空中分岐規格フラグ固定 | HV template | GenerateFromBackboneSpec(HV_3PH, legacy branch mode値を注入) | Exact: fail（unsupported mode）かつ `allow_midair_branch=false` | error / bundle_templates | 高圧規格逸脱の混入防止 |
| C102 | Bundle別接続モード共存 | 同一Midair点 + HV/COMM複数bundle | GenerateFromBackboneSpec(HV+COMM) | Invariant: 同一SupportNodeでHV=PassThrough/COMM=NotPresentを同時保持 | last_generation_backbone.nodes[].bundle_modes | ノード単位固定分岐モデルの混入防止 |
| C103 | 非Poleノード経由の詳細生成安定性 | 3点Pathの中点をBuilding指定 | GenerateFromBackboneSpec(LV) | Invariant: 生成が失敗/クラッシュせずSpan生成される | result.generated_span_ids | 非Pole入力での生成停止防止 |
| C104 | Segment端点吸着 | Segmentクリック（端点近傍） | ResolveBranchPick(PickResult, snap_radius) | Invariant: 端点Nodeに吸着し、Midairノードを増やさない | ResolveBranchPick結果 / last_generation_backbone.nodes | 端点近傍で余計な空中ノードを作らない |
| C105 | Segment中間でMidair生成 | Segmentクリック（端点から十分離れる） | ResolveBranchPick(PickResult, snap_radius) | Invariant: Midair SupportNodeが生成される | ResolveBranchPick結果 / last_generation_backbone.nodes | 空中分岐入力を安定して扱える |
| C106 | Pick経由HV空中分岐禁止 | Segment中間クリック + HV template | ResolveBranchPick(PickResult, HV) | Exact: fail（midair branch禁止） | error | 高圧規格逸脱の混入防止 |
| C107 | Midair Dry-run非破壊 | Segment中間クリック + create_midair_node=false | ResolveBranchPick(PickResult, LV) | Invariant: Midair解決しても`last_generation_backbone.nodes`が増えない | ResolveBranchPick結果 / last_generation_backbone.nodes | ホバー評価で状態が汚れる回帰防止 |
| C63 | mirror導入の非悪化（Y/Z/Layer） | 同一路線をmirror無効/有効で比較 | GenerateFromBackboneSpec + template.allow_mirror override | Invariant: 有効時の複合指標（Y逆転+Z逆転+layer jump）が無効時以下 | lane assignment/port座標/template layer | 生成品質調整で見た目悪化させない |
| C64 | Guide頂点の強制Manual解除 | 3頂点Guide | GenerateFromBackboneSpec(既定設定) | Exact: 中間頂点/端点PoleともにAuto（既定） | pole placement_mode | DrawPath点=強制Pinの回避 |
| C65 | pin_verticesオプション | 3頂点Guide | GenerateFromBackboneSpec(pin_vertices=true) | Exact: 中間頂点PoleもManual | pole placement_mode | ピン留め挙動の明示制御 |
| C66 | Pole Pin/Unpin切替 | 既存Pole | SetPolePlacementMode(Auto→Manual→Auto) | Exact: mode遷移とuser_edited整合 | pole fields | ユーザー明示ピン留め運用 |
| C67 | セッション局所再生成でManual Pole保持 | 既存session生成 + 中間PoleをManual化 | RegenerateSessionAutoParts(session,newGuide) | Invariant: Manual Pole位置不変、Auto部分のみ更新 | pole位置/mode/span数 | 軽微変更で手直し消失防止 |
| C68 | セッション局所再生成でManual Port保持 | 既存session生成 + PortをManual化 | RegenerateSessionAutoParts(session,newGuide) | Invariant: Manual Port位置不変 | port位置/mode | ポート手直し保護 |
| C72 | セッション局所再生成でAuto Pole配下Manual Port保持 | 既存session生成 + Auto Pole配下PortをManual化 | RegenerateSessionAutoParts(session,newGuide) | Invariant: owner PoleはAutoのまま、Manual Port位置不変 | pole mode/port位置 | Port手直しをPole Pin必須にしない |
| C69 | 他セッション非干渉 | session1/session2を別生成 | RegenerateSessionAutoParts(session1,...) | Invariant: session2のPole/Span不変 | pole/span存在と位置 | 局所更新の安全性 |
| C36 | DrawPath点直配置 | クリック点3 | GenerateSimpleLineFromPoints | Exact: Pole数=点数,位置一致,yaw一致 | pole position/yaw | DrawPath直感性 |
| C37 | 幾何based side選定 | 2Pole(左右) | AddConnectionByPole(Branch) | Invariant: 右手前でRight,左手前でLeft | selected slot side | 偶奇依存排除 |
| C38 | 高圧3相群生成 | 有効Path | GenerateFromBackboneSpec(HV_3PH) | Invariant: 3レーン×区間数生成, lane記録あり | span数/bundle/lane_assignments | 高圧ねじれ抑制 |
| C39 | 方向強制モード | 有効Path | GenerateFromBackboneSpec(direction=Reverse) | Exact: Reverseが採用される | direction_debug/先頭Pole | 手動比較可能性 |
| C40 | Pole flip_180 | 接続済Pole | SetPoleFlip180(true) | Invariant: 配下Port更新+接続Span dirty | port位置/runtime dirty | 局所向き修正性 |
| C41 | debug記録クリアの無害性 | 生成/接続でdebug記録あり | clear_slot_selection_debug_records + clear_path_direction_debug_records | Exact: 記録だけ消え、Entity件数/ID/整合は不変 | counts/ID集合/Validate | デバッグ操作で本体破壊しない |
| C42 | 再計算の非破壊性 | Spanを含む状態 | UpdateGeometrySettings→ProcessDirtyQueues | Invariant: cache/version更新のみでEntity件数/ID不変 | counts/ID集合/runtime/cache | キャッシュ再生成で正本が歪まない |
| C43 | 鋭角時Port展開軸補正 | クリック点3(コーナー内角<75°) | GenerateSimpleLineFromPoints | Invariant: 中間Poleのside軸が内角二等分線に直交し、内側へ向かない | pole yaw/context(sharp_theta,b,side_dir) | 鋭角での線間距離潰れ抑制 |
| C108 | 鋭角向きの入口間一致 | 同一acute pathをSimple/Backboneで生成 | GenerateSimpleLineFromPoints / GenerateFromBackboneSpec | Invariant: 中間Poleのyawとsharp debugが一致 | pole yaw/context(sharp_side_dir,sharp_bisector_dir) | 入口ごとの別向き決定の再混入防止 |
| C109 | HV3 captureの隣接交差なし | capture再現8点 path | GenerateFromBackboneSpec(HV_3PH) | Invariant: 隣接segment間で別導体polylineがXY交差しない | lane assignments / port world positions | fallback generated port経路のねじれ再発防止 |
| C44 | Bundle参照API整合 | Span1本+Bundle作成 | AddBundle + AddSpan + GetSpansByBundle | Invariant: Spanがbundle参照を保持し検索整合 | span fields/query API/Validate | 複数本配線の正本一貫性維持 |
| C45 | 不正Bundle参照拒否 | Span1本 | AddSpan(bundle_id不正) | Exact: failし状態保全, 診断文言あり | error/span fields | 不正参照でデータ破壊しない |
| C47 | DrawPath Bundle生成(HV標準) | 有効Path+PoleType | GenerateFromBackboneSpec(HV, lane=0) | Invariant: Bundle作成+標準3並列+全Spanにbundle_id | result IDs/span fields/Validate | 束単位生成の成立 |
| C48 | DrawPath Bundle生成方向モード | 有効Path | GenerateFromBackboneSpec(Forward/Reverse/Auto) | Invariant: 全モードで生成成功 | result/generated spans | 束単位向き指定の入口 |
| C49 | DrawPath Bundle生成の異常系 | 新規CoreState | polyline不足/interval<=0/未知template/未知PoleType | Exact: fail+状態不変+復帰成功 | error/counts/後続成功 | 入力不正耐性と運用安定 |
| C90 | Backbone bundles必須 | BackboneSpecでbundles空+legacy値のみ設定 | GenerateFromBackboneSpec | Exact: bundles[]必須エラー | error | legacy経路の逆流防止 |
| C91 | 接続時テンプレ必須 | 2Pole + auto_create_bundle=true + template未指定 | AddConnectionByPole | Exact: bundle_template_id必須エラー | error | category依存自動生成の再混入防止 |
| C92 | 接続時テンプレ優先 | 2Pole + template=HV + category引数をLVで呼ぶ | AddConnectionByPole | Invariant: Span/Port/BundleがHV規格で生成される | span layer / port category / bundle kind,count | category引数の隠れ分岐混入防止 |
| C93 | 接続時bundle/template競合拒否 | 2Pole + 既存LV bundle + template=HV | AddConnectionByPole | Exact: mismatchエラー | error | 二重入力の矛盾混入防止 |
| C94 | 接続時template指定+auto_create無効 | 2Pole + use_template=true + auto_create=false + bundle_id未指定 | AddConnectionByPole | Exact: bundle_id必須エラー | error | 暗黙no-opの混入防止 |
| C95 | 接続時span_layer上書き競合拒否 | 2Pole + template=HV + span_layer=LV | AddConnectionByPole | Exact: span_layer conflictエラー | error | layer二重指定の矛盾混入防止 |
| C96 | Dropはテンプレ既定利用 | PoleType適用済Pole | AddDropFromPole(kDrop) | Invariant: Bundle spacing/layerがテンプレ既定に一致 | span layer / bundle spacing | ハードコード設定の再混入防止 |
| C73 | 固定テンプレcount上書き拒否 | BackboneSpec + HV_3PH | GenerateFromBackboneSpec(bundle=count指定) | Exact: fail（上書き不可） | error | 固定規格の強制 |
| C74 | 非HVテンプレ固定1本 | BackboneSpec + LV既定テンプレ | GenerateFromBackboneSpec(bundle=count未指定) | Invariant: 区間ごとに1本生成 | generated span数/bundle count | 規格固定で入力削減 |
| C75 | 非HV固定テンプレcount上書き拒否 | BackboneSpec + LV既定テンプレ | GenerateFromBackboneSpec(bundle=count指定) | Exact: fail（上書き不可） | error | 固定規格の強制 |
| C77 | 複数テンプレ同時生成 | BackboneSpec + LV/COMM複合指定 | GenerateFromBackboneSpec(bundles複数) | Invariant: bundleが複数生成され、Span数が合算本数に一致 | result.bundle_ids/generated spans | 複数束同時入力の成立 |
| C50 | Port初期モード | 新規Port追加 | AddPort | Exact: position_mode=Auto | port fields | 新規Port生成規則の固定 |
| C51 | Port手修正/解除 | 接続済Port | SetPortWorldPositionManual→ResetPortPositionToAuto | Invariant: Manual化→Auto復帰, 関連SpanのみDirty | port/runtime | 手直し維持と復帰性 |
| C52 | Manual保護 | 手修正Portあり | SetPoleFlip180 | Invariant: Manual Port位置が維持される | port position/mode | 軽微再生成で手修正消失防止 |
| C53 | BackboneSpec境界手動点安定 | BackboneSpec初回生成済 | BackboneSpec延長して再GenerateFromBackboneSpec | Invariant: 既存Manual境界Poleの位置/Mode不変 | pole position/mode | 軽微変更で手直し消失防止 |
| C54 | BackboneSpec局所更新 | BackboneSpec生成済 | 同一BackboneSpec再実行→延長再実行 | Invariant: 同一入力で重複増殖なし、延長で末端のみ追加 | span数/生成結果 | 全再生成回帰防止 |
| C55 | Backbone経路 | Bundle付きSpan生成済 | BuildBackboneEdges/FindBackboneRoute | Invariant: bundle付きedge構築と経路取得 | backbone edge/route | ルート計算基盤維持 |
| C56 | 鋭角閾値境界 | 非対称3点角（コーナー内角基準） | コーナー内角74度/75度/76度でGenerateSimpleLineFromPoints | Invariant: `内角<=75`で補正適用、`内角>75`で非適用 | middle pole context.sharp_orientation_applied | 閾値バグによる向き破綻防止 |
| C60 | Guide再利用頂点の向き再評価 | 既存頂点Poleを再利用可能なGuide | 同一Guideを再生成（頂点PoleのYawを事前に崩す） | Invariant: override無しなら再利用頂点Poleのside軸が二等分線直交方向へ再評価される | vertex pole yaw/context(sharp_*) | 再生成で角向きが古いまま残る不具合防止 |
| C57 | Guide重複点ロバスト | PoleTypeあり | 重複点含むGuideでGenerateFromBackboneSpec | Invariant: 成功しPole座標有限、path Z維持 | generated pole positions/Validate | 入力ノイズ耐性 |
| C58 | Reverse対称性 | 同一Guide | Forward/ReverseでGenerateFromBackboneSpec | Invariant: 生成Pole位置集合が一致（順序非依存） | generated pole positions set | 方向モードで幾何が破綻しない |
| C59 | Avoid制約尊重 | PoleTypeあり | avoid_points/avoid_radius指定GenerateFromBackboneSpec | Invariant: 生成Poleが禁止半径内に入らない | pole positions vs avoid radius | 回避制約の信頼性 |

## LLM self-review
- 実装依存か: private順序/内部関数呼び出し順には依存しない。
- 期待値は観測可能か: すべて公開APIと公開状態で観測。
- モック過多か: モック未使用。
- 異常系が入っているか: C10/C20/C21/C22/C23/C49で失敗診断・状態保全・復帰を検証。
- フレーク要因がないか: 実時間待ち/非決定乱数なし。

