# Codex共有コンテキスト（現行）

この文書は、チャットをまたいで設計意図を維持するための共有メモです。
仕様確定文書ではなく、現時点の意思決定と優先順位を明示します。

## 1. 何を作っているか
- 電柱・電線ネットワークの生成/編集/検証基盤。
- 自動生成を主役にしつつ、必要な箇所だけ手直しできる構成を目指す。
- core はエンジン非依存、viewer は検証用。

## 2. 優先順位（固定）
1. 見た目の自然さ
2. 自動生成の一貫性（手で1本ずつ作る前提にしない）
3. 正本モデルの整合（更新経路が追えること）
4. 局所再生成（無関係要素を巻き込まない）
5. 将来拡張性（保存/探索/UE連携）

## 3. 現在の正本モデル
- `Pole`, `Port`, `Anchor`, `Bundle`, `Span`, `Attachment`
- 複数本配線は `Bundle + 複数 Span`
- `lane` は workflow/debug の内部概念で、公開正本 API にはしない
- Bundle本数は `BundleTemplate` で決める（固定テンプレは上書き禁止、可変テンプレは範囲制約）。

## 4. 重要な境界ルール
- DrawPath は入力ツール。正本ではない。
- 生成器の中心入力は `BackboneSpec`。
- `BackboneSpec.bundles[]` で束テンプレを指定する。
- テンプレ配置ヒントと `Port`（実接続点）を混同しない。
- 更新は編集 API 経由のみ。外部から mutable 参照で正本を直接触らない。

## 5. Manual/Auto ルール
- Manual はユーザー明示操作で作る。
- DrawPath 点を既定で強制 Manual 化しない。
- 再生成時は Manual Pole/Port を保持し、Auto 部分を優先更新する。

## 6. Backbone の位置づけ
- Backbone は Pole 間骨格とルート探索の層。
- Port/Span 詳細編集や見た目ルール本体は詳細層の責務。
- 入力条件（Spec）と生成結果（Result）は型で分離する。

## 6.5. 自動決定 / 正本 / override / 導出結果 の境界（2026-03-09 固定）
- 目的:
  - Pole向き / branch support / down offset / attachment / DetailCurve で値責務を混ぜない。
  - 将来 override を足しても、正本と導出の境界を壊さない。
- 分類ルール:
  - `正本`: 接続・所有・入力意味として保持する値。再生成で消えない。
  - `明示override可能な正本`: 既定は自動決定だが、将来ユーザーが固定しうる値。override は formal override 層を正とし、entity 側に mirror が残る場合でも互換目的に限定する。
  - `自動決定される導出結果`: topology / template / junction から再計算できる結果。重い正本概念にしない。
  - `詳細形状層の派生データ`: 見た目曲線・距離属性・attachment内部経路など。cache/recalc/render に閉じる。
- 項目別の固定:
  - `Backbone / junction order / primary`: 自動決定される導出結果
    - 置き場: `BackboneResult`, `JunctionInfo`, `JunctionIncident`
  - `Pole forward / yaw`: 明示override可能な正本
    - 置き場: 正本は `Pole.orientation_control`
    - 自動採用結果は `PoleOrientationDebugRecord` に留める
  - `main / branch classification`: 自動決定される導出結果
    - 置き場: `BackboneFlowKind`, `BackboneEdgeOrientation`, `SegmentLaneAssignment`
  - `main support / branch support`: 自動決定される導出結果
    - 置き場: support 配置ロジック、`PortPlacementSourceKind`
  - `branch down offset`: 自動決定される導出結果
    - 置き場: support/attachment 側の配置値
    - 禁止: layer 書き換え
  - `mirror decision`: 自動決定される導出結果
    - 置き場: lane assignment / edge orientation debug
  - `attachment endpoint / socket相当`: 正本と詳細形状派生に分離
    - 正本: どの socket/attachment に接続しているか
    - 派生: attachment内部 guide path / endpoint offset / 特殊曲線
  - `DetailCurve / arc-length table / distance attributes`: 詳細形状層の派生データ
    - 置き場: `CurveCacheEntry.detail`, render cache
- 既に formal override 層へ導入済み:
  - Pole yaw / forward 固定
  - branch down offset の span 単位 override
  - attachment socket 選択
- 将来 override 候補:
  - branch support style 選択
  - branch down offset の template/style override
  - mirror 選択
  - flow classification の例外指定
- 禁止:
  - 導出結果を独立した重い正本概念へ昇格させること
  - 未実装 override の受け皿がないからといって、導出値を entity 正本へ直書きすること
  - attachment 内部経路や DetailCurve 制御点を正本へ保存すること

## 6.6. 公開概念とアクセス境界（2026-03-11 固定）
- 目的:
  - inspector / debug / mod API で派生値の直編集や内部途中構造の露出を防ぐ。
  - `Pole`, `Span`, `SupportLayout`, `DetailCurve` などを、同じ作法で「参照可能だがどこまで触れるか」が分かる形にする。
- 固定方針:
  - source は `Pole`, `Span`, `Bundle`, `Template`, `Override`
  - readonly derived は `Junction`, `BackboneEdge`, `SupportNode`, `SupportLayout`, `DetailCurve`
  - 自動決定結果を変えたい場合は direct edit ではなく override を通す
  - `ObjectStore` mutable 実体、recalc の途中変数、grouped span の局所補助配列は外に出さない
- 参照先:
  - 詳細な分類表は `docs/core_model_architecture.md` の「公開概念とアクセス境界」を正本とする

## 7. 既知の注意点
- 正本と派生を混在させると破綻しやすい。
- 生成都合の一時情報を Entity に埋め込みすぎない。
- 文字列用語（Guide/テンプレ配置ヒント/lane）が混ざると実装者解釈が割れるため、用語は docs/wire.md に合わせる。

## 8. 現在の実装フォーカス
- 正本安定化（更新経路の集約）
- Backbone と詳細層の責務分離維持
- template 編集が既存見た目へ反映される基盤の維持
- DetailCurve による拘束付き見た目曲線と `u`/`s` 分離の土台整備
- support layout / attachment interaction / override / inspection の公開面整理
- viewer 側の検証用テストと visible-first 更新の維持

## 9. 参照順序（新規チャット時）
1. `README.md`
2. `docs/wire.md`
3. `docs/core_model_architecture.md`
4. `docs/core_model_inventory.md`
5. `docs/chat_handoff_checklist.md`

## 10. Current Snapshot（2026-03-11）
- いま動くもの:
  - BackboneSpec 経路での HV_3PH 鋭角パスにおける lane ねじれ抑制。
  - `CableTemplate` / `BundleTemplate` / Pole 実体値の責務分離。
  - template 編集時の visible-first dirty 更新。
  - `DetailCurve` 派生層による拘束付き見た目曲線の基盤。
  - `u` ベース曲線評価と `s` ベース配置 API の分離。
  - render cache への arc-length 距離属性焼き込み。
  - `SupportLayout` を detail curve 前段の派生中間構造として明示化。
  - attachment line interaction (`PassThrough / HideSegment / ReplaceWithInternalPath`) と optional internal path。
  - concept-level inspection surface（`Pole / Span / SupportLayout / DetailCurve / Junction / Template / Override`）。
  - formal override 層（Pole yaw、endpoint socket、branch down offset）。
  - viewer で pole / midair support / span の個別選択と矩形選択、選択 pole tilt が可能。
  - `wire_core_tests` は 169/169 PASS。
  - `wire_viewer_tests` は 8/8 PASS。
- いま壊れているもの:
  - `wire_viewer.exe` が起動中だと viewer 本体の再リンクが `LNK1168` で止まる。
- 既知リスク:
  - ねじれ判定は「pole局所Yの順序反転」基準。厳密XY交差（扇状近傍を含む）は評価主軸にしない。
  - `DetailCurve` は見た目曲線の近似基盤であり、厳密懸垂/弾性線/張力釣り合いは未導入。
  - `GenerateGroupedLine` 互換入口は削除済みだが、docs の古い記述が残っていないかは継続確認が必要。
- 未着手/保留:
  - shader 側での arc-length 正規化距離属性の実利用。
  - support style / mirror / flow classification の formal override。
  - viewer 側の可視デバッグ（mirror 適用区間 / junction order 表示）の恒久UI化。

## 11. Decision Log（直近）
- 2026-03-11 / Accepted:
  - 決定: `SupportLayout` を detail curve 前段の派生中間構造として明示し、curve 生成は port のその場計算ではなく support layout 集約結果を入力に受ける。
  - 理由: main support / branch support / endpoint / departure / down offset の責務を port 生成や grouped span の局所ロジックから切り出し、branch 曲線や attachment/socket 導入時の原因切り分けをしやすくするため。
  - 影響: `SupportLayout`, `SpanSupportLayoutEntry`, support layout inspector, curve constraint 再構成。
  - 覆す条件: support layout より前の層で拘束確定を共有した方が整合・性能・可視化の全てで有利だと確認された場合。
- 2026-03-11 / Accepted:
  - 決定: `Pole / Span / SupportLayout / DetailCurve / Junction / Template / Override` を concept-level inspection surface で readonly 参照し、内部 SoA や途中変数は直接 inspector に出さない。
  - 理由: 「値はあるが意味が追えない」状態を避け、作った人以外でも決定理由と関連先を追えるようにするため。
  - 影響: `EntityRef`, `EntityMeta`, inspection views, `DecisionTrace`, viewer inspector。
  - 覆す条件: readonly view では運用上不足し、正式な external API / mod API を別面として分離する必要が明確になった場合。
- 2026-03-11 / Accepted:
  - 決定: override を formal layer として導入し、少なくとも `Pole yaw/forward`, `endpoint socket`, `branch down offset` は「自動値」と別に保持して解決する。
  - 理由: 派生結果の direct edit を避けつつ、明示介入と自動復帰を両立するため。
  - 影響: `OverrideState`, inspection override surface, decision trace の override resolution, dirty/recalc 解決経路。
  - 覆す条件: formal override 層では対象ごとの dirty/recalc 粒度や UI 要件に耐えず、より細かい override 単位への再分解が必要になった場合。
  - 例外: entity 側に互換 mirror が一部残る。これは authoritative source ではなく compat のためで、将来削減対象。
- 2026-03-11 / Accepted:
  - 決定: 公開概念とアクセス境界を固定し、`SupportLayout` と `DetailCurve` を readonly derived、`Override` を source、内部 SoA や recalc 途中値は非公開とする。
  - 理由: inspector / debug / 将来の mod API が正本と派生を混ぜずに成り立つようにするため。
  - 影響: `docs/core_model_architecture.md` のアクセス境界表、inspection surface、public API tightening 方針。
  - 覆す条件: 外部公開要件の拡大により、現在の readonly surface では不足し、public contract を別パッケージとして再構成する必要が出た場合。
- 2026-03-09 / Accepted:
  - 決定: 曲線生成は `u`、正確な配置は `s`、毎フレーム表示変形は GPU 距離属性を使う。
  - 理由: attachment / 等間隔配置 / visible 区間制御に実長ベースが必要だが、毎フレーム CPU 逆引きは避けたいため。
  - 影響: `DetailCurve`, `CurveCacheEntry`, `SpanRenderCacheEntry`, viewer attachment 表示。
  - 覆す条件: arc-length table だけでは配置精度や表示要件を満たせず、別の詳細形状表現が必要になった場合。
- 2026-03-09 / Accepted:
  - 決定: 見た目曲線は「端点拘束付き cubic 基準曲線 + 後段 sag 合成」で作る。
  - 理由: 端点位置/接線拘束と中央たるみを、厳密懸垂なしで安定に両立させるため。
  - 影響: `detail_curve.cpp` の基準曲線生成、品質劣化ルール、ViaAttachment の扱い。
  - 覆す条件: 実運用で支点近傍の不自然さが残り、より高次の拘束や別モデルが必要になった場合。
- 2026-03-09 / Accepted:
  - 決定: arc-length table と制御点は正本へ入れず、詳細形状の派生 cache に置く。
  - 理由: 正本と派生を混ぜると template 編集・再計算・将来拡張で破綻しやすいため。
  - 影響: `CurveCacheEntry.detail`, `C121`, `C128-C132`。
  - 覆す条件: 保存対象として detail curve 自体を永続化する要件が生まれた場合。
- 2026-03-06 / Accepted:
  - 決定: ねじれ評価の主指標を区間法線ではなく「pole局所Y順序反転」に統一。
  - 理由: 鋭角区間で軸が反転し、同一配線でも偽陽性が出るため。
  - 影響: `service.cpp` の mirror 評価軸、`generation.cpp` の inversion 観測軸。
  - 覆す条件: 実運用キャプチャで局所Y基準でも連続的にねじれ見えが残る場合。
- 2026-03-06 / Accepted:
  - 決定: lane ねじれ対策は mirror 2択のみ（任意 permutation 不採用）を維持。
  - 理由: 自由並び替えは挙動が不安定化しやすく、編集予測性を下げるため。
  - 影響: 生成器の割当ロジック、テスト観点（C85/C86/C99）。
  - 覆す条件: mirror 2択で要件ケースを満たせない失敗が複数パターンで確認された場合。
- 2026-03-06 / Accepted:
  - 決定: `ensure_ports` と `evaluate_increment` の side 軸は pole yaw を優先し、fallback のみヒント軸を使う。
  - 理由: ポート生成軸と評価軸の不一致が mirror 誤判定を誘発していたため。
  - 影響: 鋭角パスでの mirror 選択安定化、C76/C86/C87/C88/C99 の回帰抑制。
  - 覆す条件: yaw 優先で既存の直線・鈍角ケースに退行が出る場合。

## 12. 48h Task Board
1. P1: 自動決定 / override / 導出の境界をコードへ反映
   - Done: Pole向き、support分離、branch down offset、mirror の各値で「正本に持つ値」と「derived/debug に置く値」を明文化し、必要な型の置き場を固定する。
   - 依存: `Pole.orientation_control` と support/detail/recalc の責務維持。
2. P2: Pole向き / main-branch / branch support の実装精度向上
   - Done: main continuation と `order/primary` を優先にした pole forward、edge/junction 単位の main/branch 分類、branch support 実体の強化。
   - 依存: 6.5 の境界固定。
3. P3: attachment/socket 境界の次段整理
   - Done: 正本接続情報と detail curve 側 endpoint escape をさらに分離し、socket 導入の受け皿を作る。
   - 依存: attachment 正本モデルの最小単位決定。

## 13. 次回開始パック（そのまま貼付可）
- ゴール:
  - formal override の対象拡張（support style / mirror / flow classification）か、public readonly 面の tightening を進める。
  - Backbone と詳細層の責務分離を維持したまま viewer/debug 手確認まで閉じる。
- 現在状態:
  - `wire_core_tests` は 169/169 PASS、`wire_viewer_tests` は 8/8 PASS。
  - `SupportLayout`、inspection、override、attachment interaction は概念面まで実装済み。
  - viewer 本体は実行中プロセスがあると `LNK1168` で再リンク不能。
- 直近決定:
  - 曲線生成は `u`、正確な配置は `s`、毎フレーム表示変形は GPU 距離属性。
  - 見た目曲線は「cubic 基準曲線 + 後段 sag 合成」。
  - arc-length table と制御点は正本ではなく派生 cache。
- 次の48h候補:
  - viewer 本体の再リンクと inspection / override UI の手動確認。
  - GPU 距離属性の最初の実利用。
  - support style / mirror / flow classification override の formal 導入。
- 制約:
  - 正本直書き禁止、公開API経由のみ。
- テンプレ配置ヒント(候補) / `Port`(実体) の用語混同禁止。
  - Manual保持優先、全体再生成を既定にしない。
  - arc-length table / 制御点 / detail curve を正本へ保存しない。
