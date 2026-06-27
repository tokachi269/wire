# 要求仕様書（整理版）

電柱・電線ネットワーク基盤（ネイティブ C++ core + 軽量 viewer）

## 1. 目的
- ゲームエンジン連携前提で、電柱・電線ネットワークをネイティブ C++ で生成・編集・検証できる基盤を維持する。
- 見た目だけでなく、接続整合、局所再計算、デバッグ観測、将来拡張に耐える設計を保つ。

## 2. 現在の対象
- `core` ライブラリ
- `viewer`（開発用）
- 編集正本: `Pole / Port / Anchor / Bundle / Span / Attachment`
- 生成入力: `BackboneSpec`（DrawPath はこの入力を作るツール）
- 部分再計算: Dirty/Version
- 骨格探索: Backbone（Pole 間）

## 3. 非対象（現段階）
- 高精度物理（厳密カテナリー、風揺れ）
- 厳密電気計算
- 保存読込フォーマットの固定
- Undo/Redo
- 道路システム本統合

## 4. 固定方針
### 4.1 座標と精度
- 座標系: UE 準拠
- 計算精度: `double`

### 4.2 ID
- 64bit 永続 ID
- 再利用しない
- 表示 ID（`PL-000001` など）を持つ

### 4.3 正本と派生
- 正本（PersistCore）: `Pole / Port / Anchor / Bundle / Span / Attachment`
- 派生（DerivedCache）: `CurveCache / BoundsCache / DirtyQueue / SpanRuntimeState`
- セッション情報（SessionDebug）: 選定ログ、評価ログ、統計

### 4.4 パイプライン（固定）
処理は次の順で流す。

1. Input Normalization
2. Relation / Decision
3. Materialization
4. Detail Curve
5. Presentation / Inspection

各段は前段の正本を消費するだけとし、後段で前段の意味を再判定しない。
補正・再解釈・再導出で辻褄を合わせる実装は禁止する。

## 5. 用語ルール
- テンプレ配置ヒント: テンプレート側の候補情報（実体ではない）
- `Port`: 実接続点（実体）
- `Bundle`: 複数本配線の正本単位
- `lane`: workflow/debug 用の内部概念（公開正本 API では扱わない）
- `Guide`: 新規の中心用語としては使わない。`BackboneSpec` を使う。

## 6. 生成と編集
### 6.1 生成入力
- DrawPath はツール入力。
- 生成器は `BackboneSpec` を受け取る。
- `BackboneSpec.bundles[]` で束テンプレを指定する（`bundle_template_id` 必須）。
- 生成結果は `BackboneResult` で扱う（入力条件と分離）。

### 6.2 生成原則
- DrawPath 点は既定で強制 Manual にしない。
- Manual はユーザー操作（Pin/Unpin）で明示する。
- 複数本は `Bundle + 複数 Span` で表現する。
- 固定テンプレ（例: 高圧3本）は `count` 上書きを許可しない。
- 可変テンプレ（例: 通信束）はテンプレ範囲内でのみ `count` を受け付ける。

### 6.3 再生成原則
- Manual Pole / Manual Port は保持する。
- Auto 部分を優先更新する。
- 影響範囲を局所化し、無関係要素を巻き込まない。

### 6.4 テンプレ責務の分担
- `BundleTemplate` は束のルールと、関連する `CableTemplate` / `PoleTypeDefinition` への参照を持つ。
- `CableTemplate` は線の見た目と detail 用派生入力を持つ。
- `PoleTypeDefinition` は pole 本体高さと `PortPlacementBand` / `AnchorSlotTemplate` を持ち、pole 上幾何の正本とする。
- `PortPlacementBand` は中間結果ではなく、auto port 配置の正本である。
- viewer の category 単位 pole placement UI は編集補助であり、保存時は `PortPlacementBand` に落とし込む。
- `BundleTemplate.related_pole_type_id` は bundle 主語の編集参照であり、pole 幾何そのものを bundle 側へ移さない。

## 7. 更新整合ルール
- 変更は編集 API 経由のみ。
- Pole transform 変更時は同一経路で次を実行する。
1. Pole 更新
2. 配下 Auto Port 再投影（Manual は保持）
3. 関連 Span を Dirty 化
4. direct derive で派生出力を追随
- 参照整合は `ValidateFast()` / `Validate()` で検出する。
- `UpdatePoleTypeDefinition(...)` は、すでにその `pole_type_id` を持つ既存 pole にだけ再適用する。
- `UpdateBundleTemplate(...)` は bundle 定義を更新するが、既存 pole instance の `pole_type_id` は自動変更しない。
- bundle 主語で既存 pole instance に関連 pole type を反映したい場合は `ApplyBundleRelatedPoleTypeToExistingPoles(...)` を使う。

## 8. Backbone の責務
- Backbone は Pole 間接続の骨格表現を扱う。
- Port/Span 詳細編集や見た目規則本体は詳細層の責務。
- ルート探索は Backbone を優先し、必要時のみ詳細に降りる。

## 9. lowered support の固定経路
- 正規経路: `SpanLayoutRules -> SupportGroupDecision -> SpanLayoutEntry -> geom/draw`
- `support_group_id` は decision の正本フィールドを使い、downstream で再計算しない。
- orientation / side / lower_required は upstream decision を正とし、derive/viewer で再推論しない。

## 10. 鋭角コーナー補正（現行）
- 鋭角判定は `corner interior angle <= 74°`。
- 鋭角時は Port 列の side 軸を角の二等分線に直交する向きへ補正する。
- tilted pole の再利用時も、world XY 固定ではなく pole の support plane 上で同じ sharp 判定を使う。
- デバッグで `theta / bisector / side_dir` を観測できること。

## 11. viewer 最低要件
- Pole/Port/Span/Bundle の可視化
- DrawPath 入力と生成実行
- Pole の Pin/Unpin
- Dirty/Version/再計算件数の観測
- 主要デバッグ値の表示

## 12. テスト方針
- 公開 API の観測可能事実だけを根拠にする。
- 正常系 + 異常系 + 復帰可能性を必ず含める。
- 決定論は Exact、可変要素は Invariant で検証する。
- ケース観点は `core/tests/spec_ledger.md` に集約する。

## 13. フェーズ位置づけ
- Phase 0-4: 基盤、編集、Dirty/Version、幾何表示
- Phase 4.x: 設計棚卸し
- Phase 4.9: 公開面整理（inspection / override / support layout / DetailCurve 方針の固定）
- Phase 4.8 系: Bundle 正本化、BackboneSpec 生成、Manual 保持、局所再生成
- Phase 5+: 保存読込、レイキャスト、カリング/LOD

## 14. 参照ドキュメント
- `README.md`
- `docs/core_model_inventory.md`
- `docs/core_model_architecture.md`
- `docs/chat_handoff_checklist.md`
- `docs/command_cheatsheet.md`
