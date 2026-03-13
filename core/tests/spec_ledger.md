# Core Spec Ledger (Phase4.8)

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
| C14 | サグ基本 | Span1本 | line→sag | Invariant: 端点一致+中点がworld up方向に低下 | CurveCache | 接続維持 |
| C15 | Version追随局所性 | 独立Span2本 | Port移動→再計算 | Exact: 対象のみ追随 | runtime | 局所更新維持 |
| C16 | Bounds有効性 | Span1本 | 再計算 | Invariant: AABB有効 | BoundsCache | 前段データ健全 |
| C17 | Bounds追随 | Span1本 | sag設定変更 | Invariant: AABBがworld up方向の変化に追随して更新 | BoundsCache | 表示追随 |
| C18 | デモ密度 | make_demo_state | なし | Invariant: 最低件数以上 | edit_state | 初期視認性 |
| C19 | 道路Pole生成 | PoleTypeあり | GeneratePolesAlongRoad | Invariant: 本数/Type/RoadAuto | poles/generation | 自動配置信頼性 |
| C20 | 短polyline拒否 | PoleTypeあり | GenerateSimpleLine(点1) | Exact: fail+状態不変 | error/count | 入力ミス耐性 |
| C21 | interval不正拒否 | PoleTypeあり | GenerateSimpleLine(interval<=0) | Exact: fail+状態不変 | error/count | 設定ミス耐性 |
| C22 | 存在しないPort拒否 | 空状態 | AddSpan(無効ID) | Exact: fail+状態不変 | error/count | 参照不正耐性 |
| C23 | Split t不正拒否 | Span1本 | SplitSpan(t=0) | Exact: fail+復帰可 | error/spans | 失敗後復帰 |
| C24 | 隣接Pole自動接続 | Pole列 | GenerateSpansBetweenPoles | Exact: n-1生成 | result/index | 欠線防止 |
| C25 | 複数パス増加 | Pole列 | GenerateSpansBetweenPoles×6 | Exact: 毎回n-1増加 | span総数 | 頭打ち回帰防止 |
| C26 | 第3slot利用 | PoleType適用列 | 低圧自動接続×3 | Invariant: 3slot以上利用 | source_slot集合 | 2本固定回帰防止 |
| C27 | SimpleLine統合 | 有効折れ線 | GenerateSimpleLine→GenerateFromBackboneSpec→再計算 | Invariant: 生成+Version追随+last_generation_backbone更新 | result/runtime/cache/backbone | 一発生成成立 |
| C28 | through連続性 | 直線入力 | GenerateSimpleLine | Invariant: 中間Pole同Port再利用 | span端点Port | 幹線連続維持 |
| C29 | 表示ID採番 | 新規CoreState | Pole/Port/Span追加 | Exact: prefix別連番 | display_id | UI追跡性 |
| C30 | Pole文脈分類 | 直線+折れ線 | GeneratePolesAlongRoad | Invariant: Terminal/Straight/Corner | pole.context | 文脈基盤維持 |
| C31 | 角補正有界 | 補正ON | 折れ線生成 | Invariant: sideScale有界+有限 | pole/port | 補正暴走防止 |
| C32 | 文脈別選定 | 3Pole | Trunk接続+Branch接続 | Invariant: 選定傾向差 | slot_id | 分岐競合低減 |
| C33 | 決定的タイブレーク | 同一入力2回 | AddConnectionByPole | Exact: 同slot+debug整合 | debug records | 再現性 |
| C34 | Corner文脈統合 | 折れ線 | GenerateSimpleLine | Invariant: ガイド頂点PoleがCorner文脈のまま生成される | pole.context | 角付き路線維持 |
| C35 | 内外補正差 | 左折/右折 | GeneratePolesAlongRoad | Invariant: 外側オフセット>内側 | turn_sign/slot座標 | 角圧縮の低減 |
| C61 | 鋭角自動拡幅 | 鋭角/鈍角の同一テンプレート比較 | GenerateSimpleLineFromPoints | Invariant: 鋭角の左右レーン間隔が鈍角より広い（カテゴリ非依存） | corner poleのlocal Y差 | 鋭角での線間距離不足防止 |
| C62 | 群レーンねじれ抑制 | U字Guide + HV3 lane | GenerateFromBackboneSpec(HV_3PH) | Invariant: 区間ごとのlane順逆転数が0 | lane assignment port local Y順 | 群配線のクロス抑制 |
| C76 | 鋭角コーナーlane順反転抑制 | 鋭角コーナーを含むGuide + COMM4 lane | GenerateFromBackboneSpec(COMM,count=4) | Invariant: 区間ごとのlane順逆転数が0 | lane assignmentのport local Y順 | 鋭角時の見た目破綻防止 |
| C99 | HV3キャプチャ形状の反転回帰 | ねじれ再現点列（6点）+ HV3 | GenerateFromBackboneSpec(HV_3PH) | Invariant: 区間ごとのlane順逆転数が0 | last_lane_assignments / port local Y順 | 実運用形状でのねじれ再発防止 |
| C100 | Midair SupportNode保持 | 3点Pathの中点をMidair指定 | GenerateFromBackboneSpec(LV) | Invariant: `last_generation_backbone.nodes` にMidairノードとtangent hintが残る | last_generation_backbone.nodes | Pole固定前提への逆戻り防止 |
| C110 | 明示Pole node再利用 | 既存Poleを始点にした2本目Path | GenerateFromBackboneSpec(LV, node_specs.node_id) | Invariant: 明示node_idのPoleを再利用し、重複Poleを生成しない | generated pole ids / backbone route | DrawPathの既存node接続を座標一致頼みに戻さない |
| C111 | 明示SupportNode再利用 | 既存Midairを始点にした2本目Path | GenerateFromBackboneSpec(LV, node_specs.node_id) | Invariant: 明示node_idのSupportNodeを次のbackboneでも再利用する | last_generation_backbone.nodes | DrawPathの空中分岐接続を新規node化しない |
| C112 | Midair始点延長の詳細生成 | 既存Midairを始点にした短い2点Path | GenerateFromBackboneSpec(LV, node_specs.node_id) | Invariant: 新規 branch 側に pole/span が少なくとも1本ずつ生成される | generated_pole_ids / generated_span_ids | 中間レイキャスト始点で詳細生成が空振りしない |
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
| C70 | Guide再利用頂点Port再投影 | 既存Poleを再利用するGuide | 角向き変更を伴うGuide再生成 | Invariant: corner向き変更時にtemplate-owned Portが再投影される | owned port位置 / pole yaw | 再利用PoleでPort位置が古いまま残る不具合防止 |
| C71 | MovePoleでAuto Port再投影とManual保護 | 既存Pole + Manual Portあり | MovePole | Invariant: Auto Portだけが再投影され、Manual Portは維持される | port位置 / runtime dirty | Pole移動で手修正Portが壊れない |
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
| C109 | HV3 captureの内部共有pole順序連続 | capture再現8点 path | GenerateFromBackboneSpec(HV_3PH) | Invariant: terminal fan-out を除く内部の隣接segment間では共有pole上のlane順序が連続する | lane assignments / shared-pole local ordering | perpendicular row 方針でも内部のmain-chain continuityが崩れない回帰防止 |
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
| C98 | Backbone延長時の境界導体順保持 | 既存bundle pathを延長するBackbone | GenerateFromBackboneSpecで既存末端から延長 | Invariant: 延長境界で導体順が反転せず保持される | lane assignments / boundary ports | Backbone延長で境界の見た目が崩れない |
| C73 | 固定テンプレcount上書き拒否 | BackboneSpec + HV_3PH | GenerateFromBackboneSpec(bundle=count指定) | Exact: fail（上書き不可） | error | 固定規格の強制 |
| C74 | 非HVテンプレ固定1本 | BackboneSpec + LV既定テンプレ | GenerateFromBackboneSpec(bundle=count未指定) | Invariant: 区間ごとに1本生成 | generated span数/bundle count | 規格固定で入力削減 |
| C75 | 非HV固定テンプレcount上書き拒否 | BackboneSpec + LV既定テンプレ | GenerateFromBackboneSpec(bundle=count指定) | Exact: fail（上書き不可） | error | 固定規格の強制 |
| C77 | 複数テンプレ同時生成 | BackboneSpec + LV/COMM複合指定 | GenerateFromBackboneSpec(bundles複数) | Invariant: bundleが複数生成され、Span数が合算本数に一致 | result.bundle_ids/generated spans | 複数束同時入力の成立 |
| C78 | Pole tiltでAuto Port再投影と見た目追随 | 既存Pole | SetPoleTilt | Invariant: Auto Port再投影とSpan visual cache更新が同じ経路で起きる | port位置 / visual cache | 傾き変更後に線の見た目が古いまま残らない |
| C79 | 参照長によるサグ安定 | 既存Pole + Span | Pole tilt後に再計算 | Invariant: reference_lengthによりサグ深さが視覚的に安定する | curve cache / reference_length | 傾き変更でたるみが不自然に跳ねない |
| C80 | center slotのPole非重なり | center slotを持つPoleType | PoleType適用でPort生成 | Invariant: center slotはPole中心線から半径+クリアランス分だけ離れる | port local/world位置 | center PortがPoleに埋まらない |
| C81 | insulatorは電力系のみ表示 | 電力線と非電力線の両方がある | 再計算 | Invariant: insulator visualは電力系でのみ生成される | visual cache | 非電力線に碍子が誤表示されない |
| C82 | T-junctionの一次session優先 | 2本のDrawPathがT字で接続 | GenerateFromBackboneSpecを2回実行 | Invariant: 最初のsessionがjunction primary order=0を維持する | BackboneResult.junctions | 後から足した経路で主系統順が壊れない |
| C83 | Cross junction順序の安定 | 交差junctionを再構築できる状態 | BuildBackboneResultを繰り返し実行 | Invariant: incident orderが再構築後も安定する | BackboneResult.junctions | junction順序が再計算で揺れない |
| C84 | 後続pathでjunction優先順を上書きしない | 既存path + 後から追加したDrawPath | GenerateFromBackboneSpecを追加実行 | Invariant: prioritized_session_idが初回sessionのまま維持される | BackboneResult.junctions | 後続pathで既存junctionの主従が壊れない |
| C85 | mirror処理は2択に留まる | grouped bundle path | GenerateFromBackboneSpec(HV/COMM) | Invariant: mirror解決が反転あり/なしの2択だけで、任意並び替えを導入しない | lane assignments | permutation追加による複雑化を防ぐ |
| C86 | acute pattern suiteの反転なし | 複数acute pathパターン | GenerateFromBackboneSpec(COMM,count=4) | Invariant: 全パターンでlane順逆転数が0 | lane assignments | 鋭角パターン群での見た目破綻防止 |
| C87 | HV3 acute pathの相ねじれ防止 | acute path + HV_3PH | GenerateFromBackboneSpec(HV_3PH) | Invariant: 3相の導体順が区間をまたいでねじれない | lane assignments / local Y順 | 高圧3相の相順崩れ防止 |
| C88 | Backbone HV3 acute pathのtemplate経路trace可視化 | acute backbone path + HV_3PH | GenerateFromBackboneSpec(HV_3PH) | Invariant: template生成経路でも各route poleにsupport axis/layout yaw traceが出る | pole inspection / edge orientations | Backbone入口でもsupport axis観測が欠けない |
| C89 | 3相ポリシーのカテゴリ非依存性 | 複数カテゴリの3相bundle | GenerateFromBackboneSpec(3相bundle群) | Invariant: 3相ポリシーがカテゴリ名に依存せず同じ規則で働く | lane assignments | カテゴリ分岐の混入防止 |
| C50 | Port初期モード | 新規Port追加 | AddPort | Exact: position_mode=Auto | port fields | 新規Port生成規則の固定 |
| C51 | Port手修正/解除 | 接続済Port | SetPortWorldPositionManual→ResetPortPositionToAuto | Invariant: Manual化→Auto復帰, 関連SpanのみDirty | port/runtime | 手直し維持と復帰性 |
| C52 | Manual保護 | 手修正Portあり | SetPoleFlip180 | Invariant: Manual Port位置が維持される | port position/mode | 軽微再生成で手修正消失防止 |
| C53 | BackboneSpec境界手動点安定 | BackboneSpec初回生成済 | BackboneSpec延長して再GenerateFromBackboneSpec | Invariant: 既存Manual境界Poleの位置/Mode不変 | pole position/mode | 軽微変更で手直し消失防止 |
| C54 | BackboneSpec局所更新 | BackboneSpec生成済 | 同一BackboneSpec再実行→延長再実行 | Invariant: 同一入力で重複増殖なし、延長で末端のみ追加 | span数/生成結果 | 全再生成回帰防止 |
| C55 | Backbone経路 | Bundle付きSpan生成済 | BuildBackboneEdges/FindBackboneRoute | Invariant: bundle付きedge構築と経路取得 | backbone edge/route | ルート計算基盤維持 |
| C56 | 鋭角閾値境界 | 非対称3点角（コーナー内角基準） | コーナー内角74度/75度/76度でGenerateSimpleLineFromPoints | Invariant: `内角<=75`で補正適用、`内角>75`で非適用 | middle pole context.sharp_orientation_applied | 閾値バグによる向き破綻防止 |
| C60 | Guide再利用頂点の向き再評価 | 既存頂点Poleを再利用可能なGuide | 同一Guideを再生成（頂点PoleのYawを事前に崩す） | Invariant: override無しなら再利用頂点Poleのside軸が二等分線直交方向へ再評価される | vertex pole yaw/context(sharp_*) | 再生成で角向きが古いまま残る不具合防止 |
| C57 | Guide重複点ロバスト | PoleTypeあり | 重複点含むGuideでGenerateFromBackboneSpec | Invariant: 成功しPole座標有限、pathのworld up方向高さを維持 | generated pole positions/Validate | 入力ノイズ耐性 |
| C58 | Reverse対称性 | 同一Guide | Forward/ReverseでGenerateFromBackboneSpec | Invariant: 生成Pole位置集合が一致（順序非依存） | generated pole positions set | 方向モードで幾何が破綻しない |
| C59 | Avoid制約尊重 | PoleTypeあり | avoid_points/avoid_radius指定GenerateFromBackboneSpec | Invariant: 生成Poleが禁止半径内に入らない | pole positions vs avoid radius | 回避制約の信頼性 |
| C113 | Midair始点延長の先頭 support-detail 区間保持 | 既存Midairを始点にした2点Path | GenerateFromBackboneSpec(LV, node_specs.node_id) | Invariant: 最初の support-to-detail 区間が route に含まれる | FindBackboneRoute / generated spans | Midair始点で先頭1区間だけ抜ける回帰防止 |
| C114 | Midair branch の source span 高さ再利用 | source span の中間から branch | ResolveBranchPick + GenerateFromBackboneSpec(LV) | Invariant: backbone pick は抽象高さでも detail は source span 高さから始まる | branch port位置 / source span位置 | 中間分岐が 0m から出る見た目破綻防止 |
| C115 | Midair 1クリック延長で余計な bridge を増やさない | 既存Midairから終点1点へ延長 | GenerateFromBackboneSpec(LV, node_specs.node_id) | Invariant: 直結1区間で構成され、追加 bridge が出ない | generated_pole_ids / generated_span_ids / route | 1クリック延長で余計な中間生成をしない |
| C116 | midair branch 禁止 template は生成だけを skip する | source-edge midair branch + 禁止template | GenerateFromBackboneSpec(HV_3PH) | Invariant: request 全体は失敗せず、禁止templateの bundle/span だけ生成されない | generated ids / counts | 規格違反 bundle だけを除外し他 bundle を巻き込まない |
| C117 | Path input では midair policy を入力段階で強制しない | midair pick + HV template + enforce=false | ResolveBranchPick(PickResult, HV) | Invariant: template branch policy を強制せず Midair 解決できる | ResolveBranchPick結果 / last_generation_backbone.nodes | DrawPath の入力段階で不要に操作を止めない |
| C118 | mixed template の midair branch は許可 bundle だけ生成 | source-edge midair branch + LV/HV 混在 | ResolveBranchPick + GenerateFromBackboneSpec(LV+HV) | Invariant: allow_midair_branch=true の bundle だけが生成される | bundle_ids / generated spans | 混在生成で disallow bundle を巻き込まない |
| C119 | CableTemplate太さ変更の見た目反映 | 既存線 + CableTemplate.outer_diameter変更 | UpdateCableTemplate | Invariant: 依存 span の wire render radius が更新される | span render cache | CableTemplateの太さ変更が既存線へ反映される |
| C120 | CableTemplate碍子要否の見た目切替 | 既存電力線 + requires_insulator変更 | UpdateCableTemplate | Invariant: 依存 span の insulator visual が切り替わる | span visual cache | CableTemplateの碍子設定が既存線へ反映される |
| C121 | Template責務の分離 | 型定義 | compile-time traits | Exact: allow_midair_branch は BundleTemplate 側のみで、CableTemplate と entity は bezier入力や arc-length table を持たない | type traits | 正本とテンプレ責務の混線防止 |
| C122 | CableTemplate編集で Pole tilt を上書きしない | Pole tilt 設定済み + CableTemplate編集 | SetPoleTilt + UpdateCableTemplate | Invariant: pole rotation X/Y は不変 | pole world_transform | Pole傾きをテンプレ変更が壊さない |
| C123 | BundleTemplate の topology 変更は regeneration_required を立てる | 既存bundle + topology変更 | UpdateBundleTemplate | Invariant: 依存 bundle が regeneration_required になり dependency state に載る | bundles / template_dependency_state | topology変更を visible更新だけで済ませない |
| C124 | BundleTemplate の visual-only 変更は dirty のみに留める | 既存bundle + cable_template_id変更 | UpdateBundleTemplate | Invariant: 依存 span は dirty になるが regeneration_required は立たない | span runtime / template_dependency_state | 見た目変更を topology再生成に混ぜない |
| C125 | ApplyPoleTilt は選択された Pole 実体だけを更新する | 2本のPoleの片方だけ選択 | ApplyPoleTilt(selection, value) | Invariant: 指定Poleだけ rotation X/Y が変わる | pole world_transform | Pole tilt を実体値として局所更新できる |
| C126 | WorldUp と lateral 軸の整合 | 代表的な forward vector | WorldUp + ComputeLateralAxis | Invariant: lateral が forward/up と直交し正規化される | coord utils | 軸依存の手書き計算を1定義に寄せる |
| C127 | PoleFrame の local/world roundtrip | tilt付き transform + local point | BuildPoleFrame + LocalPointToWorld + WorldPointToLocal | Invariant: tilt 下でも local/world roundtrip が安定する | pole frame math | Pole傾き適用の回転順序ずれ防止 |
| C128 | uベース曲線APIの端点拘束 | 端点位置と端点接線が既知 | BuildDetailCurve + EvaluatePosition/EvaluateTangent | Invariant: u=0/1 で端点位置を満たし、端点接線が意図方向へ揃う | DetailCurve | 曲線生成を u ベースに固定する |
| C129 | sベース配置APIと sag 合成 | 端点固定 + sag 付き曲線 | BuildDetailCurve + PositionAtLength | Invariant: sag 後も端点位置は不変で、長さ基準の中点配置が使える | DetailCurve.arc_length_table | 正確な配置を s ベースで扱える |
| C141 | 懸垂寄り support slope | 端点同高 + sag 付き曲線 | BuildDetailCurve | Invariant: sag 付き曲線は支点で完全水平に寝ず、両端で上下向きの傾きが出る | DetailCurve.EvaluateTangent | quartic 的な平坦 sag に戻る回帰防止 |
| C142 | 近直線での横揺れ抑止強化 | 同高ほぼ直線 + 軽い逆向き横tangent | BuildDetailCurve | Invariant: 近直線 span は lateral tangent 差があっても平面内でほぼ真っ直ぐに保たれる | DetailCurve.sample_points | 微小な横成分で左右にぐにゃる回帰防止 |
| C143 | SmoothPass の制御付き横曲げ | 同方向へ緩く曲がる長尺 span | BuildDetailCurve | Invariant: SmoothPass は全 lateral 成分を潰さず、制御された平面曲がりを保つ | DetailCurve.quality / sample_points | NearStraight 向け横抑制が緩い継続カーブまで潰す回帰防止 |
| C130 | OffsetEndpoint 端点 | 支点と派生 endpoint offset が既知 | BuildDetailCurve(OffsetEndpoint) | Invariant: 曲線端点は支点そのものではなく派生 offset 位置になる | DetailCurve | 支点近傍で急に折れない離脱表現の土台 |
| C137 | Attachment表示offset非干渉 | Span + Attachment(display offset) | AddAttachment→Commit | Invariant: 正本 attachment の表示offsetは detail curve 端点を変えない | Attachment / DetailCurve | 接続情報と表示補正の責務混線防止 |
| C131 | 鋭角/競合接線での品質劣化安全策 | 反対向きに近い接線ヒント | BuildDetailCurve | Invariant: tangent fallback が働き、過度な膨張や逆行を抑える | DetailCurve.quality / sample_points | 高度最適化なしで見た目破綻を抑える |
| C148 | SharpCorner の compact 化 | 鋭角 corner-pass + 強い横tangent | BuildDetailCurve | Invariant: SharpCorner は handle を縮めて G1 に落ち、横ピークを抑える | DetailCurve.quality / sample_points | 鋭角部まで SmoothPass 的に膨らむ回帰防止 |
| C144 | 長尺継続 span の G2優先採用 | 長い pass-through span + 素直な端点接線 | BuildDetailCurve(PreferG2) | Invariant: continuity は G2 になり、smooth pass-through 理由が debug で追える | DetailCurve.quality | G2優先方針が単なる係数でなく判定として残ることを固定 |
| C145 | 短スパンの G1劣化 | 短い span + PreferG2 | BuildDetailCurve | Invariant: 短スパンでは無理に G2 を維持せず G1 と ShortSpan 理由へ落ちる | DetailCurve.quality | 短スパンで過拘束 cubic を押し込む回帰防止 |
| C146 | branch pass の G1優先と端点拘束保持 | branch pass + offset endpoint | BuildDetailCurve | Invariant: branch pass は G1 を採用しつつ offset endpoint と端点接線を保つ | DetailCurve.quality / EvaluatePosition / EvaluateTangent | branch で G2 に固執して support 離脱が崩れる回帰防止 |
| C147 | PreferG1 は明示選択 | 長尺 span + PreferG1 + sag | BuildDetailCurve | Invariant: PreferG1 は failed G2 ではなく explicit G1 として扱われ、sag と arc-length はそのまま動く | DetailCurve.quality / arc_length_table | continuity 設定と実挙動の意味ずれ防止 |
| C149 | ViaAttachment の endpoint 優先 | offset endpoint を使う attachment 離脱 | BuildDetailCurve(OffsetEndpoint) | Invariant: ViaAttachment は attachment endpoint を端点に使い、smooth pass を強制せず endpoint-priority G1 へ落ちる | DetailCurve.quality / EvaluatePosition | attachment 接続を支点中心通過へ戻す回帰防止 |
| C132 | GPU用距離属性の焼き込み | span再計算済み | Commit + find_span_render_cache | Invariant: 累積長と正規化距離が render cache に入り、毎フレーム CPU 逆引きを不要にする | span render cache | 長さ依存エフェクトの受け皿を持つ |
| C133 | 本線優先Pole向き | 既存main + 後からbranch | GenerateFromBackboneSpec(HV) を2回 | Invariant: 共有junction Pole の yaw が branch ではなく既存 main chain に従う | pole yaw / pole orientation debug | junction primary が Pole向きへ反映されない回帰防止 |
| C134 | Branch support port 生成 | 既存main + HV branch | GenerateFromBackboneSpec(HV) | Invariant: branch bundle が `flow=Branch` になり branch-support port を使う | last_lane_assignments / port placement_source | branch を main support と同列 mirror に戻す回帰防止 |
| C135 | Branch down offset は layer 非改変 | 既存main + HV branch | GenerateFromBackboneSpec(HV) | Invariant: branch port の高さは main より低いが template layer は維持される | branch/main port z / template_layer | support offset ではなく layer 書換えで解く実装の混入防止 |
| C136 | HV3 main port 安定性 | 既存HV main + HV branch | GenerateFromBackboneSpec(HV) を2回 | Invariant: branch 追加後も既存 main bundle の中心 port 集合が変わらない | main bundle center port ids | branch 追加で既存 HV_3PH の左右順が壊れる回帰防止 |
| C138 | Mixed route の edge 単位 flow | 既存main + main→branch 混在 path | GenerateFromBackboneSpec(LV) | Invariant: 先頭 edge は Main、分岐 edge は Branch として別処理される | last_lane_assignments.flow_kind / flow_decision_rule | route 1値の branch 判定が main 区間まで侵食する回帰防止 |
| C139 | Branch support の派生配置 | 既存main + HV branch + recalc | GenerateFromBackboneSpec→Commit | Invariant: branch support placement が visual cache に派生生成される | span visual cache.branch_supports | branch support が port 配置だけで終わる回帰防止 |
| C140 | Near-straight branch でも topology 優先 | 既存main + ほぼ直進角の branch | GenerateFromBackboneSpec(LV) | Invariant: 幾何角度に引っ張られず Branch 判定を維持する | last_lane_assignments.flow_kind / flow_decision_rule | order/topology ではなく角度しきい値で branch 判定が揺れる回帰防止 |
| C150 | 新規chainは orientation fallback | 既存main文脈なしの新規3点chain | GenerateFromBackboneSpec(LV) | Invariant: 中央Poleは main chain / primary 根拠が無いので fallback rule のまま | pole_orientation_debug_records | main文脈が無い新規Poleまで本線向きへ決め打ちする回帰防止 |
| C173 | 直線 DrawPath の support axis は路線直交へ保つ | 直線3点 + HV3 | GenerateFromBackboneSpec(HV_3PH) | Invariant: 中央Poleの support axis と port row は route 軸ではなくその直交軸へ揃う | PoleInspectionView.support_axis_dir / port world positions | support 配置軸が route 軸へ寝て viewer 上で相列が重なって見える回帰防止 |
| C174 | 十字 junction の support axis は対角線にならず main 軸直交を保つ | 水平 trunk + 垂直 path で共有中心Pole | GenerateFromBackboneSpec を2回 | Invariant: 共有中心Poleの support axis は primary main 軸の直交軸を維持し、port row が diagonal へ倒れない | PoleInspectionView / port world positions | cross junction で support 列が二等分方向や route 平行へ傾く回帰防止 |
| C175 | 直交 DrawPath corner は bisector を使わない | 直交3点 + HV3 | GenerateFromBackboneSpec(HV_3PH) | Invariant: corner Pole の support axis は cardinal 軸のどちらかを選び、対角線にならない | PoleInspectionView.support_axis_dir | 直交 path の中間Poleで support 配置軸が bisector になる回帰防止 |
| C176 | branch 追加で main support axis の直交関係を崩さない | 既存水平 trunk + 垂直 branch | GenerateFromBackboneSpec を2回 | Invariant: branch 後も共有Poleの support axis と main port row は main 軸ではなく main 軸の直交軸を保つ | PoleInspectionView / port world positions | branch 追加で main support が route 平行や diagonal 化する回帰防止 |
| C151 | worldspace variation の連続性 | 近接/遠方の world position | EvaluateHierarchicalVariation | Invariant: 近い位置の bias 差は小さく、遠い位置では差が広がる | HierarchicalVariationSample.world_bias | ID乱数で worldspace を離散化する回帰防止 |
| C152 | same flow の共有 bias と pole/local 差 | 同一chainの複数 span + variation 有効 | GenerateFromBackboneSpec→Commit | Invariant: 同一 flow_key の span は同じ flow_bias を持つが final 値は完全一致しない | DetailCurve.quality.sag_variation | 隣接 span が完全独立/完全一致の両極へ崩れる回帰防止 |
| C153 | global seed 再現性 | 同一seed + 同一入力を2回 | GenerateFromBackboneSpec→Commit | Invariant: derived variation の final 値が一致する | DetailCurve.quality.sag_variation | 再生成で揺らぎが毎回変わる回帰防止 |
| C154 | seed 変更で derived output が変わる | seed だけ変更した同一入力 | GenerateFromBackboneSpec→Commit | Invariant: variation final 値が変化する | DetailCurve.quality.sag_variation | seed を変えても見た目揺らぎが固定化する不具合防止 |
| C155 | variation は topology と mirror を変えない | seed違いの同一 trunk+branch 入力 | GenerateFromBackboneSpec | Invariant: flow_kind / flow_rule / mirror 系は seed に依存しない | last_lane_assignments | topology 規則へランダムが混入する回帰防止 |
| C156 | PassThrough attachment は外部線を残す | span + PassThrough template attachment | AddAttachment→Commit | Invariant: hidden interval も replacement path も増やさず、外部線は全長可視のまま | DetailCurve.visible_intervals / replacement_paths | attachment を置いただけで線を不意に欠損させる回帰防止 |
| C157 | HideSegment attachment は外部線区間を隠す | span + HideSegment template attachment | AddAttachment→Commit | Invariant: 該当区間だけ hidden になり、visible interval は分割されるが replacement path は無い | DetailCurve.hidden_intervals / visible_intervals | 隠すだけの部品を internal path 必須にしてしまう回帰防止 |
| C158 | ReplaceWithInternalPath attachment は内部線路へ置換する | span + ReplaceWithInternalPath template attachment | AddAttachment→Commit | Invariant: 外部線区間を隠しつつ replacement path が生成される | DetailCurve.hidden_intervals / replacement_paths | 置換系 attachment が単なる hide で終わる回帰防止 |
| C159 | socket endpoint 接続で隙間を詰める | span endpoint + attachment socket override | endpoint_attachment/socket 設定→Commit | Invariant: detail curve 端点が support 原点ではなく attachment socket 世界座標へ移る | Span.endpoint_attachment_* / DetailCurve.EvaluatePosition | support 中心から離れた socket 接続で隙間が残る回帰防止 |
| C160 | runtime は attachment 名に依存しない | HideSegment template を任意名へ変更 | UpdateAttachmentTemplate→Commit | Invariant: runtime は template 名変更後も explicit interaction mode どおりに動く | AttachmentTemplate.line_interaction_mode | importer 補助の命名規約が runtime 仕様へ逆流する回帰防止 |
| C164 | support layout は branch support 拘束を集約する | branch support source port + branch span | Commit | Invariant: support layout は branch/main support 種別、endpoint、departure、down offset を保持し、curve/visual は同じ layout を参照する | SpanSupportLayoutEntry / DetailCurve.start_constraint / BranchSupportPlacement | support 情報が grouped span / recalc / visual に分散して再解釈ズレする回帰防止 |
| C165 | support layout は attachment socket endpoint を捕捉する | endpoint attachment socket override | Commit | Invariant: support layout の endpoint/departure 入力が socket 接続後の curve 入力と一致する | SpanSupportLayoutEntry.start / DetailCurve.start_constraint | attachment socket が curve にだけ入り support 前段から見えなくなる回帰防止 |
| C166 | inspector surface は span/support layout/detail curve を概念単位で見せる | branch span + support layout + detail curve | Commit→inspect_* / collect_decision_trace | Invariant: authoritative/derived/detail-derived と関連リンク、主要 trace topic が取得でき、内部途中構造に依存しない | SpanInspectionView / SupportLayoutInspectionView / DetailCurveInspectionView / DecisionTraceEntry | 値はあるが意味や決定理由が追えない状態へ逆戻りする回帰防止 |
| C167 | inspector surface は pole/template/override/junction を共通面で参照できる | manual yaw override pole + degree3 junction | Commit→inspect_* | Invariant: pole/template/override/junction が concept-level view として取得でき、override は direct edit でなく surface として見える | PoleInspectionView / TemplateInspectionView / OverrideInspectionView / JunctionInspectionView | 正本/override/derived 境界が inspector 面で崩れる回帰防止 |
| C168 | Pole orientation override の往復 | 既存 auto yaw の pole | SetPoleManualYawOverride / SetPoleFlip180 / ClearPoleOrientationOverride | Invariant: override 中は auto と final が区別され、解除で auto yaw に戻る | PoleInspectionView / OverrideInspectionView / DecisionTrace | 派生 yaw を直編集して自動へ戻れなくなる回帰防止 |
| C169 | Span socket override の往復 | endpoint attachment を持つ span | SetSpanEndpointSocketOverride / ClearSpanEndpointSocketOverride | Invariant: socket override 中は support layout endpoint が切り替わり、解除で自動 socket 選択へ戻る | SpanSupportLayoutEntry / OverrideInspectionView | endpoint socket を source に焼き込んで戻せなくなる回帰防止 |
| C170 | Branch down offset override の往復 | branch support source を持つ span | SetSpanBranchDownOffsetOverride / ClearSpanBranchDownOffsetOverride | Invariant: override 中は support layout の down offset が変わり、解除で policy-derived 値へ戻る | SpanSupportLayoutEntry / OverrideInspectionView | branch down offset を layer 書換えや派生直編集で持ってしまう回帰防止 |
| C171 | public header smoke surface | 公開ヘッダのみを含む最小 consumer | include public headers + instantiate public types | Invariant: 公開ヘッダが self-sufficient で、concept-level view を内部 storage に触れずに使える | EntityRef / SupportLayoutEndpointView / CoreView inspection surface | 公開ヘッダの依存漏れや内部 storage 断面への依存が利用者ビルドへ漏れる回帰防止 |
| C172 | support layout と detail curve は同じ resolved policy を使う | DrawPath trunk + branch span | GenerateFromBackboneSpec→inspect_* / collect_decision_trace | Invariant: flow/continuity/variation の resolved 入力が support layout と detail curve で一致し、trace では support/tangent/continuity/sag が分離表示される | SupportLayoutInspectionView / DetailCurveInspectionView / DecisionTraceEntry | span-level policy が前段/後段で別々に再解釈されて見た目不具合の主因が追えなくなる回帰防止 |
| C177 | DrawPath 通常経路は plain support endpoint へ落ちる | simple Backbone path + LV bundle | GenerateFromBackboneSpec | Invariant: attachment入力なしでは support layout endpoint source が `PlainSupport` になり、attachment request / resolved socket は未設定のまま | SupportLayoutInspectionView | DrawPath 未接続の attachment を「効いている扱い」にする誤認防止 |
| C178 | DrawPath 通常経路は attachment 未入力を trace できる | simple Backbone path + LV bundle | GenerateFromBackboneSpec→collect_decision_trace | Invariant: support layout trace に `AttachmentEndpointSelection` が出て、`PlainSupport` と `request=None` が読める | DecisionTraceEntry | DrawPath で attachment 未接続なのか反映失敗なのかを区別できない回帰防止 |
| C179 | DrawPath branch curve は support 離脱を局所化する | 既存水平 trunk + 垂直 branch | GenerateFromBackboneSpec を2回→Commit | Invariant: branch span は `BranchPass` / `BranchChordPriority` を維持しつつ、support 離脱長を局所 budget に抑えて chord へ早く戻り、inspection/trace でも lateral 抑制値が読める | DetailCurveInspectionView / CurveCacheEntry.sample_points / DecisionTraceEntry | branch support の向きを span 全体へ引きずって横回り込みが長く残る回帰防止 |
| C180 | DrawPath の main/branch 差は inspection と trace で読める | 既存水平 trunk + 垂直 branch | GenerateFromBackboneSpec を2回→inspect_span / inspect_detail_curve / collect_decision_trace | Invariant: main span は Main / 非branch-support、branch span は Branch / branch-support / BranchPass / lateral suppression として観測でき、trace でも両者が分かれる | SpanInspectionView / DetailCurveInspectionView / DecisionTraceEntry | flow は内部にあるが DrawPath 利用時の観測面では main と branch の差が読めない回帰防止 |
| C181 | endpoint refresh service は relation index から owned endpoint を集める | pole A/B と owned port/anchor | service を直接呼ぶ | Invariant: service は pole A の relation index に載る port/anchor だけを返し、他 pole の endpoint を混ぜない | state_internal::EndpointRefreshService / RelationIndex | pole 更新時に全走査前提の責務が残り、対象 endpoint を局所化できない回帰防止 |
| C182 | endpoint refresh service は対象 pole の owned endpoint だけ更新する | pole A/B と owned port/anchor、Aだけ移動 | service を直接呼ぶ | Invariant: pole A の owned auto endpoint だけ再投影され、pole B 側は不変のまま | state_internal::EndpointRefreshService / ChangeSet | endpoint refresh が他 pole まで巻き込む回帰防止 |
| C183 | override resolution service は formal override を優先解決する | pole override + span socket/down offset override | public API で override 設定→service を直接呼ぶ | Invariant: manual yaw / flip / socket / branch down offset は service から同じ最終採用値として読める | state_internal::OverrideResolutionService | override 解決規則が CoreState 各所へ散って precedence がずれる回帰防止 |
| C184 | ApplyPoleType は relation-index-owned endpoint を再利用する | pole A/B に同じ pole type を適用後、Aへ再適用 | ApplyPoleType を再実行 | Invariant: pole A は既存 slot port / anchor を relation index から再利用し、全体 count と pole B の owned endpoint は増えない | CoreState::ApplyPoleType / RelationIndex | pole type 再適用が全 port/anchor 走査に依存し、無関係 pole まで巻き込む回帰防止 |
| C186 | template mutation service は編集対象 template にぶら下がる bundle だけ再生成要求する | LV bundle と COMM bundle を並存させ、LV template だけ変更 | service を直接呼ぶ | Invariant: edited template に一致する bundle だけ `regeneration_required` と dependency queue に入る | state_internal::TemplateMutationService / TemplateDependencyState | template 更新責務が CoreState 内で散り、無関係 bundle まで巻き込む回帰防止 |
| C187 | attachment template 更新は実際にその template を使う span だけ dirty にする | attachment を持つ span と持たない span | service を直接呼ぶ | Invariant: edited attachment template を参照する span だけ geometry/render dirty になる | state_internal::TemplateMutationService / ChangeSet | attachment template 更新が全 attachment / 全 span を無差別に汚す回帰防止 |
| C185 | HV3 DrawPath terminal は route 直交に開く | 2点直線 + HV_3PH bundle | GenerateFromBackboneSpec | Invariant: 開始/終了 pole の HV row は topology distinct と visual distinct を満たし、viewer 上で route 直交方向へ分離する | SegmentLaneAssignment / VisualSeparationMetrics / AxisRelationMetrics / port world positions | endpoint pole で grouped lane が route 平行に寝て三相が一点へ寄って見える回帰防止 |
| C188 | HV slot が無い terminal でも generated row は route 直交に開く | 2点直線 + HV_3PH bundle + HV slot 無し pole type | GenerateFromBackboneSpec | Invariant: 開始/終了 pole は generated row で 3 相を route 直交方向へ分離する | SegmentLaneAssignment / VisualSeparationMetrics / AxisRelationMetrics / port world positions | template 依存の生成が無い pole で三相が一点へ寄せる回帰防止 |
| C189 | communication pole + 全 template 選択でも HV3 terminal row は route 直交を保つ | 2点直線 + CommunicationPole + LV/HV/COMM/OPT bundle | GenerateFromBackboneSpec | Invariant: communication pole 上でも HV3 terminal は visual distinct を維持し、row axis は route 直交から崩れない | SegmentLaneAssignment / VisualSeparationMetrics / AxisRelationMetrics | pole type や mixed templates の組み合わせで HV3 terminal row が再び route 平行に寝る回帰防止 |
| C190 | communication 多条 terminal row は route 直交を保つ | 2点直線 + CommunicationPole + COMM_BUNDLE count=3 | GenerateFromBackboneSpec | Invariant: communication 多条 terminal でも topology distinct と visual distinct を両立し、row axis は route 直交を保つ | SegmentLaneAssignment / VisualSeparationMetrics / AxisRelationMetrics | 通信 bundle では terminal row 修正が効かず viewer 上で線が一点へ寄る回帰防止 |
| C191 | preserved multi-lane template は HV 名に依存せず offset endpoint を使う | Communication bundle template を preserve_conductor_identity=true, count=3 に変更 | UpdateBundleTemplate→GenerateFromBackboneSpec | Invariant: category が Communication でも preserved multi-lane policy なら support layout/detail curve の endpoint mode は OffsetEndpoint になる | BundleTemplate.preserve_conductor_identity / SpanSupportLayoutEntry / DetailCurveInspectionView | endpoint mode が HV category の直書きに依存し、活用例以外の multi-lane template で geometry 規則が効かない回帰防止 |
| C192 | clicked existing communication poles + 全 template 選択でも HV terminal 分離を維持する | 既存 CommunicationPole 2本を node_specs で再利用し LV/HV/COMM/OPT bundle を同時生成 | GenerateFromBackboneSpec | Invariant: clicked point only 相当の既存 pole 再利用経路でも HV terminal の port/endpoint/wire spacing と row axis が崩れない | BackboneInputSpec.node_specs / VisualSeparationMetrics / AxisRelationMetrics | fresh generated poles だけ通って viewer の既存 pole 再利用経路で三相が一点へ寄る回帰防止 |
| C161 | 長い branch span は横回り込みを抑える | 長尺 branch + 強い横向き support tangent | BuildDetailCurve(BranchPass) | Invariant: branch は chord 優先で support departure を局所化し、大きな sideways runout を抑える | DetailCurve.quality / sample_points | branch support の横向きが span 全体を左右へ引っ張る回帰防止 |
| C162 | branch の局所 departure は短スパンほど強い | 短尺/長尺 branch を同一 tangent で比較 | BuildDetailCurve(BranchPass) | Invariant: 短スパンの方が support weight と lateral limit が高く、長スパンでは chord 優先が強まる | DetailCurve.quality | branch の横成分抑制が span 長に応じて変わらない回帰防止 |
| C163 | main の sag は branch より強く読める | 同一 chord/hint の main と branch | BuildDetailCurve | Invariant: endpoint 拘束を保ったまま main の中央下がりが branch より強い | DetailCurve.sag_amplitude / EvaluatePosition | branch まで main と同じ sag 読みになって垂れすぎる回帰防止 |

## LLM self-review
- 実装依存か: private順序/内部関数呼び出し順には依存しない。
- 期待値は観測可能か: すべて公開APIと公開状態で観測。
- モック過多か: モック未使用。
- 異常系が入っているか: C10/C20/C21/C22/C23/C49で失敗診断・状態保全・復帰を検証。
- フレーク要因がないか: 実時間待ち/非決定乱数なし。
