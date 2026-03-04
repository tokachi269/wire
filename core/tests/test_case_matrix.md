# Core Test Case Matrix (Phase4.8)

## Scope and policy
- 観測根拠: 公開API戻り値、`view()`、`connection_index()`、`find_*cache()`、`slot_selection_debug_records()`、`last_path_direction_debug()`、`last_lane_assignments()`、`Validate()` のみ。
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
| C62 | 群レーンねじれ抑制 | U字Guide + HV3 lane | GenerateGroupedLine(allow_lane_mirror=true) | Invariant: 区間ごとのlane順逆転数が0 | lane assignment port local Y順 | 群配線のクロス抑制 |
| C63 | mirror導入の非悪化（Y/Z/Layer） | 同一路線をmirror無効/有効で比較 | GenerateGroupedLine(allow_lane_mirror=false/true) | Invariant: 有効時の複合指標（Y逆転+Z逆転+layer jump）が無効時以下 | lane assignment/port座標/template layer | 生成品質調整で見た目悪化させない |
| C64 | Guide頂点の強制Manual解除 | 3頂点Guide | GenerateFromGuide(既定設定) | Exact: 中間頂点PoleはAuto、端点のみManual（既定） | pole placement_mode | DrawPath点=強制Pinの回避 |
| C65 | pin_verticesオプション | 3頂点Guide | GenerateFromGuide(pin_vertices=true) | Exact: 中間頂点PoleもManual | pole placement_mode | ピン留め挙動の明示制御 |
| C66 | Pole Pin/Unpin切替 | 既存Pole | SetPolePlacementMode(Auto→Manual→Auto) | Exact: mode遷移とuser_edited整合 | pole fields | ユーザー明示ピン留め運用 |
| C67 | セッション局所再生成でManual Pole保持 | 既存session生成 + 中間PoleをManual化 | RegenerateSessionAutoParts(session,newGuide) | Invariant: Manual Pole位置不変、Auto部分のみ更新 | pole位置/mode/span数 | 軽微変更で手直し消失防止 |
| C68 | セッション局所再生成でManual Port保持 | 既存session生成 + PortをManual化 | RegenerateSessionAutoParts(session,newGuide) | Invariant: Manual Port位置不変 | port位置/mode | ポート手直し保護 |
| C69 | 他セッション非干渉 | session1/session2を別生成 | RegenerateSessionAutoParts(session1,...) | Invariant: session2のPole/Span不変 | pole/span存在と位置 | 局所更新の安全性 |
| C36 | DrawPath点直配置 | クリック点3 | GenerateSimpleLineFromPoints | Exact: Pole数=点数,位置一致,yaw一致 | pole position/yaw | DrawPath直感性 |
| C37 | 幾何based side選定 | 2Pole(左右) | AddConnectionByPole(Branch) | Invariant: 右手前でRight,左手前でLeft | selected slot side | 偶奇依存排除 |
| C38 | 高圧3相群生成 | 有効Path | GenerateGroupedLine(HV,3) | Invariant: 3レーン×区間数生成, lane記録あり | span数/bundle/lane_assignments | 高圧ねじれ抑制 |
| C39 | 方向強制モード | 有効Path | GenerateGroupedLine(Reverse) | Exact: Reverseが採用される | direction_debug/先頭Pole | 手動比較可能性 |
| C40 | Pole flip_180 | 接続済Pole | SetPoleFlip180(true) | Invariant: 配下Port更新+接続Span dirty | port位置/runtime dirty | 局所向き修正性 |
| C41 | debug記録クリアの無害性 | 生成/接続でdebug記録あり | clear_slot_selection_debug_records + clear_path_direction_debug_records | Exact: 記録だけ消え、Entity件数/ID/整合は不変 | counts/ID集合/Validate | デバッグ操作で本体破壊しない |
| C42 | 再計算の非破壊性 | Spanを含む状態 | UpdateGeometrySettings→ProcessDirtyQueues | Invariant: cache/version更新のみでEntity件数/ID不変 | counts/ID集合/runtime/cache | キャッシュ再生成で正本が歪まない |
| C43 | 鋭角時Port展開軸補正 | クリック点3(コーナー内角<75°) | GenerateSimpleLineFromPoints | Invariant: 中間Poleのside軸が内角二等分線に直交し、内側へ向かない | pole yaw/context(sharp_theta,b,side_dir) | 鋭角での線間距離潰れ抑制 |
| C44 | Bundle参照API整合 | Span1本+Bundle作成 | AddBundle + AddSpan + GetSpansByBundle | Invariant: Spanがbundle参照を保持し検索整合 | span fields/query API/Validate | 複数本配線の正本一貫性維持 |
| C45 | 不正Bundle参照拒否 | Span1本 | AddSpan(bundle_id不正) | Exact: failし状態保全, 診断文言あり | error/span fields | 不正参照でデータ破壊しない |
| C46 | 既存Span互換性 | Bundle未設定Span | ProcessDirtyQueues | Invariant: 従来再計算/Validate維持 | runtime/Validate/query API | 既存データ互換維持 |
| C47 | DrawPath Bundle生成(HV標準) | 有効Path+PoleType | GenerateBundleFromPath(HV, lane=0) | Invariant: Bundle作成+標準3並列+全Spanにbundle_id | result IDs/span fields/Validate | 束単位生成の成立 |
| C48 | DrawPath Bundle生成方向モード | 有効Path | GenerateBundleFromPath(Forward/Reverse/Auto) | Invariant: 全モードで生成成功 | result/generated spans | 束単位向き指定の入口 |
| C49 | DrawPath Bundle生成の異常系 | 新規CoreState | polyline不足/interval<=0/未知category/未知PoleType | Exact: fail+状態不変+復帰成功 | error/counts/後続成功 | 入力不正耐性と運用安定 |
| C50 | Port初期モード | 新規Port追加 | AddPort | Exact: position_mode=Auto | port fields | 既存互換維持 |
| C51 | Port手修正/解除 | 接続済Port | SetPortWorldPositionManual→ResetPortPositionToAuto | Invariant: Manual化→Auto復帰, 関連SpanのみDirty | port/runtime | 手直し維持と復帰性 |
| C52 | Manual保護 | 手修正Portあり | SetPoleFlip180 | Invariant: Manual Port位置が維持される | port position/mode | 軽微再生成で手修正消失防止 |
| C53 | BackboneSpec境界手動点安定 | BackboneSpec初回生成済 | BackboneSpec延長して再GenerateFromGuide | Invariant: 既存Manual境界Poleの位置/Mode不変 | pole position/mode | 軽微変更で手直し消失防止 |
| C54 | BackboneSpec局所更新 | BackboneSpec生成済 | 同一BackboneSpec再実行→延長再実行 | Invariant: 同一入力で重複増殖なし、延長で末端のみ追加 | span数/生成結果 | 全再生成回帰防止 |
| C55 | Backbone経路 | Bundle付きSpan生成済 | BuildBackboneEdges/FindBackboneRoute | Invariant: bundle付きedge構築と経路取得 | backbone edge/route | ルート計算基盤維持 |
| C56 | 鋭角閾値境界 | 非対称3点角（コーナー内角基準） | コーナー内角74度/75度/76度でGenerateSimpleLineFromPoints | Invariant: `内角<=75`で補正適用、`内角>75`で非適用 | middle pole context.sharp_orientation_applied | 閾値バグによる向き破綻防止 |
| C60 | Guide再利用頂点の向き再評価 | 既存頂点Poleを再利用可能なGuide | 同一Guideを再生成（頂点PoleのYawを事前に崩す） | Invariant: override無しなら再利用頂点Poleのside軸が二等分線直交方向へ再評価される | vertex pole yaw/context(sharp_*) | 再生成で角向きが古いまま残る不具合防止 |
| C57 | Guide重複点ロバスト | PoleTypeあり | 重複点含むGuideでGenerateFromGuide | Invariant: 成功しPole座標有限、path Z維持 | generated pole positions/Validate | 入力ノイズ耐性 |
| C58 | Reverse対称性 | 同一Guide | Forward/ReverseでGenerateFromGuide | Invariant: 生成Pole位置集合が一致（順序非依存） | generated pole positions set | 方向モードで幾何が破綻しない |
| C59 | Avoid制約尊重 | PoleTypeあり | avoid_points/avoid_radius指定GenerateFromGuide | Invariant: 生成Poleが禁止半径内に入らない | pole positions vs avoid radius | 回避制約の信頼性 |

## LLM self-review
- 実装依存か: private順序/内部関数呼び出し順には依存しない。
- 期待値は観測可能か: すべて公開APIと公開状態で観測。
- モック過多か: モック未使用。
- 異常系が入っているか: C10/C20/C21/C22/C23/C49で失敗診断・状態保全・復帰を検証。
- フレーク要因がないか: 実時間待ち/非決定乱数なし。

