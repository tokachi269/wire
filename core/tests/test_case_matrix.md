# Core Test Case Matrix (Phase4.7)

## Scope and policy
- 観測根拠: 公開APIの戻り値、`edit_state()`、`connection_index()`、`find_*cache()`、`slot_selection_debug_records()`、`Validate()`のみを使用。
- 期待値粒度: `Exact`は決定論の値に限定、変わりやすい仕様は`Invariant`(不変条件/整合条件)で検証。
- モック方針: ドメインロジックはモック禁止。境界モックのみ許容（本スイートはモック未使用）。
- 時間/並行: 実時間待ち・非決定処理は使用しない。

| Case ID | 目的 | 前提 | 入力 | 期待結果 | 観測点 | 壊れた時に守りたいユーザ価値 |
|---|---|---|---|---|---|---|
| C01 | ID発行の単調増加/リセット後整合を保証 | 新規IdGenerator | next/peek/reset | Exact: 連番・重複なし | IdGenerator戻り値 | 永続ID衝突防止 |
| C02 | ObjectStoreの追加/取得/削除整合を保証 | 空Store | insert/find/remove/upsert | Exact: 件数と参照整合維持 | ObjectStore公開API | 編集操作後の参照破綻防止 |
| C03 | Span追加時Runtime初期化を保証 | 有効Port2点 | AddSpan | Exact: runtime作成+dirty付与 | find_span_runtime_state | 再計算漏れ防止 |
| C04 | Pole移動の局所Dirtyを保証 | 独立Span2本 | MovePole | Invariant: 関連Spanのみdirty | runtime dirty bits | 無関係再計算の抑制 |
| C05 | SplitSpanの構造整合を保証 | Span1本 | SplitSpan(0.5) | Invariant: 旧削除+新2Span+新Port | stores/index/Validate | 分岐操作の破綻防止 |
| C06 | PoleType適用でテンプレPort生成を保証 | PoleType定義あり | ApplyPoleType | Invariant: owner整合でPort生成 | GetPoleDetail | テンプレ適用の可視編集維持 |
| C07 | PoleType差でPort構成差分を保証 | PoleType2種あり | 2Poleへ各Type適用 | Invariant: slot構成差が出る | source_slot_id集合 | タイプ差の設計意図維持 |
| C08 | 自動割当が未使用slot優先を保証 | 同Typeの2Pole | AddConnectionByPole×2 | Invariant: 連続接続で別slot使用 | result.slot_a_id | 線追加時の重なり低減 |
| C09 | Pole接続APIの基本整合を保証 | PoleType適用済2Pole | AddConnectionByPole | Invariant: Span生成+index/dirty更新 | spans/ports/index/runtime | 主導線操作の成立 |
| C10 | 同一Pole接続拒否と復帰可能性を保証 | Pole1本 | AddConnectionByPole(p,p)後に正常接続 | Exact: fail診断+後続成功 | error/counts/API戻り値 | 誤操作後も継続編集可能 |
| C11 | 柱起点引込の最小整合を保証 | PoleType適用済Pole | AddDropFromPole | Invariant: service span+free endpoint | spans/ports/Validate | 引込生成の破綻防止 |
| C12 | 線起点引込の最小整合を保証 | 既存Span1本 | AddDropFromSpan | Invariant: split+drop構造整合 | spans/index/Validate | 分岐引込で崩壊しない |
| C13 | 直線幾何の決定性を保証 | 同一状態 | ProcessDirtyQueues再生成 | Exact: 点列一致 | CurveCache | 同入力同出力の再現性 |
| C14 | サグ基本不変条件を保証 | Span1本 | line→sag切替 | Invariant: 端点一致+中点下降 | CurveCache points | 見た目変更でも接続維持 |
| C15 | Geometry/Bounds/Render追随と局所性を保証 | 独立Span2本 | Port移動→再計算 | Exact: 対象のみversion追随 | runtime versions | 局所再計算設計の維持 |
| C16 | Bounds生成有効性を保証 | Span1本 | 再計算 | Invariant: whole/segment AABB有効 | BoundsCache | 後続カリング前提の健全性 |
| C17 | BoundsがGeometry変化へ追随することを保証 | Span1本 | sag設定変更 | Invariant: AABBが更新 | BoundsCache.whole | 見た目変更の追随性 |
| C18 | デモ状態の初期密度を保証 | make_demo_state | なし | Invariant: Pole/Port/Spanが最低数以上 | edit_state件数 | 初期確認時の視認性 |
| C19 | 道路沿いPole生成の基本整合を保証 | PoleTypeあり | GeneratePolesAlongRoad | Invariant: 本数/Type/RoadAuto整合 | poles/generation | 自動配置の信頼性 |
| C20 | 異常系:短いpolyline拒否を保証 | PoleTypeあり | GenerateSimpleLine(点1) | Exact: fail+状態不変+回復可能 | error/counts/再実行 | 入力ミスで状態汚染しない |
| C21 | 異常系:interval不正拒否を保証 | PoleTypeあり | GenerateSimpleLine(interval<=0) | Exact: fail+状態不変+回復可能 | error/counts/再実行 | 不正設定で破綻しない |
| C22 | 異常系:存在しないPort接続拒否を保証 | 空状態 | AddSpan(無効ID) | Exact: fail+状態不変+回復可能 | error/counts | 参照不正で壊れない |
| C23 | 異常系:Split t不正拒否を保証 | Span1本 | SplitSpan(t=0)後に正常split | Exact: fail診断+回復可能 | error/spans/API | 失敗後の復帰性 |
| C24 | 隣接Pole自動接続の基本を保証 | Pole列 | GenerateSpansBetweenPoles | Exact: n-1本生成+dirty付与 | result/runtime/index | 自動接続の最低品質 |
| C25 | 複数パス接続で本数増加を保証 | Pole列 | GenerateSpansBetweenPoles×6 | Exact: 毎回n-1追加 | span総数/result | 頭打ち回帰の防止 |
| C26 | 低圧自動接続で第3slot利用を保証 | PoleType適用Pole列 | GenerateSpansBetweenPoles×3 | Invariant: 3つ以上のslotが使われる | first pole source_slot_id集合 | 「2本しか出ない」回帰防止 |
| C27 | GenerateSimpleLine統合整合を保証 | 有効折れ線 | GenerateSimpleLine→再計算 | Invariant: Pole/Span生成+Version追随 | result/runtime/cache | 一発生成導線の成立 |
| C28 | 中間Poleのthrough連続性を保証 | 直線入力 | GenerateSimpleLine | Invariant: 中間Poleで同Port再利用 | span端点Port | 幹線連続の見た目維持 |
| C29 | 表示IDのprefix別採番を保証 | 新規CoreState | Pole/Port/Span追加 | Exact: P/PT/SPが個別連番 | entity.display_id | UI上の追跡容易性 |
| C30 | Pole文脈分類を保証 | 直線+折れ線入力 | GeneratePolesAlongRoad | Invariant: Terminal/Straight/Corner判定 | pole.context | 文脈依存配置の基盤維持 |
| C31 | 角度補正値の有界性と有限性を保証 | 補正ON設定 | 折れ線生成 | Invariant: sideScale範囲内+有限値 | pole.context/port値 | 補正暴走による崩壊防止 |
| C32 | 文脈別スロット選定傾向を保証 | 3Pole | Trunk接続+Branch接続 | Invariant: Branchで選択傾向差 | slot_a_id比較 | 分岐時の競合低減 |
| C33 | 決定的タイブレークとデバッグ整合を保証 | 同一入力2回 | AddConnectionByPole | Exact: 同slot選択+候補整合 | slot_selection_debug_records | 再現性と調整可能性 |
| C34 | DrawPath系生成へのCorner文脈統合を保証 | 折れ線入力 | GenerateSimpleLine | Invariant: CornerPass spanが含まれる | span.placement_context/Validate | 角付き路線の品質維持 |
| C35 | 角の内外補正が回転方向で変わることを保証 | 角付き道路入力 | GeneratePolesAlongRoad(左折/右折) | Invariant: 外側slotのオフセットが内側より大きい | pole.context.corner_turn_sign + slot200/201座標差 | 角で線が詰まりすぎる見た目を防ぐ |
| C36 | DrawPath生成でクリック点をそのままPole化し向きを自動計算することを保証 | クリック点3つ | GenerateSimpleLineFromPoints | Exact: Pole数=点数, Pole位置一致, yawが経路接線に一致 | pole_ids/position/rotation_euler_deg.z | DrawPath操作が直感どおり再現される |

## LLM self-review (before/after coding)
- 実装依存か: private関数や呼び出し順は検証していない。公開API・公開状態のみ観測。
- 期待値は観測可能か: すべて戻り値・公開データ・Validation・デバッグ記録から観測可能。
- モックが多すぎないか: モック未使用。
- 異常系が入っているか: C10/C20/C21/C22/C23で失敗診断・状態不変・復帰可能性を検証。
- フレーク要因がないか: 時間待ち/非決定依存なし、同一入力で同一結果を確認。
