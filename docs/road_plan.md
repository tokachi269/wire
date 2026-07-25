# 道路生成 方針・要件書(v2・自己完結版)

この文書は単体で新規チャット・新規リポジトリへ引き継ぐことを想定して書かれている。
前提となる外部ログ(AI との検討履歴)は破棄してよい。必要な結論はすべて本文に含む。

## 0. 背景(引き継ぎ用の最小文脈)

- 大目標: **町全体の自動生成**。既存の第一ドメインとして「wire」(電柱・電線網の生成/編集システム。C++ core + wasm + three.js/Svelte viewer)が稼働中。道路は第二ドメイン。
- wire で確立済みで、町全体に共通させる規約(**town 共通規約**と呼ぶ):
  1. 状態は Input / 正本(authoritative)/ 派生に分類。正本を書けるのは所有する操作 API のみ。派生は一方向導出で保存しない
  2. 操作はトランザクション(事前検証 preflight → trial 状態で実行 → 成功時のみ commit)。全外部入力の有限性・範囲検証は preflight に一元化
  3. エラーは3分類: validation(入力不正)/ unsupported(仕様上の拒否)/ internal(バグ検出)。fallback で黙って通さない
  4. 永続化は正本のみ、versioned text(key=value 行形式)、未知 version は明示 reject、保存境界で圧縮
  5. 識別は ID。位置の近接・頂点番号・名前文字列で同一性を推測しない
  6. 「操作×状態の意味論表」を維持し、未定義セルは実装禁止(unsupported)
  7. テストは fail-first・理由付き失敗メッセージ・seed 付き fuzz+不変量検査・skip を通過と数えない
- wire の反省: 要件の意味論を決めずに実装が先行して手戻りが大きかった/削除操作を後回しにした/投機的な型を先行させて死にデータ化した。本計画はこれらを初日から避ける。

## 1. 目的と品質バー

- Cities: Skylines 2 の道路のような「構成を組み替えられる道路」を、少数の機構で生成する。
- 品質バー: **街路レベルで眺めて自然。対応範囲では端部・遷移まで完成。曖昧なケースは補完せず unsupported で止める。**
- 交通シミュレーション・信号制御・土木 CAD 精度は対象外。

## 2. 設計原則

1. **挙動は段階導入、データ設計は先に正しく。** 将来機能が「スキーマ変更」ではなく「データ追加+消費コードの追加」で済む形を初版スキーマで担保する。ただし消費者のいない実装は置かない(スキーマ定義と docs のみ)。
2. 決定者一箇所: 断面評価の結果(境界・外周・アンカー)を、路面・白線・マスク・将来の車両経路が同一テーブルから消費する。独自再計算を作らない。
3. **専用型を増やさない**: 「線」は用途ごとに固有型を作らず共通 Path 一種(§4)。「面」も共通 Area 一種(§5)。

## 3. wire との構造的な違い

| | wire | 道路 |
|---|---|---|
| node | 支持点(電柱)そのもの。曲がるたびに node | **接続点・端点のみ**。カーブは node にしない |
| segment | 2 node 間=上面視で直線 | 2 node 間に**任意の平面曲線**(S字も1 segment) |
| 面要素 | なし | **あり**(交差点・ロータリー・駅前広場 = Area) |
| 接続 | lane 継続テーブル | 断面要素の対応表+接続記録(思想は同じ、型は共有しない) |

backbone の思想(正本グラフ→一方向導出、接続は操作時に決定して記録、再生成は reconcile)は引き継ぐ。構造・型は引き継がない。

## 4. 共通プリミティブ: Path

```text
Path = 順序付き primitive 列、 primitive = Line | Arc | Bezier
```

- 用途: segment の平面線形/交差点・Area の縁/マーキング中心線/縁石などメッシュ導出の案内線/将来の車両経路。**直線と曲線の混在可。**
- レーン白線・車両パス・縁石線に固有の曲線型を作らない。用途属性は Path を参照する側が持つ。
- 評価は弧長 s。フレームは world-up 基準(Frenet 禁止)。

## 5. データモデル(初版スキーマで固定)

```text
正本(SavedRoadGraph)
├─ RoadNode     { id, position }                      ← 接続点・端点のみ
├─ RoadSegment  { id, node_a, node_b,
│                 alignment: Path(平面),
│                 vertical: スロットのみ(P3),
│                 section_timeline }
├─ SectionTimeline   { s区間ごとの CrossSectionTemplate 参照 + SectionTransition }
├─ SectionTransition { from/to template, s範囲, 要素対応表, anchor(中心線|左端|右端) }
│      ← lane 増減・歩道消滅などのスムーズ接続の正本。挙動は P2
├─ CrossSectionTemplate
│    └─ elements: 順序付き [ SurfaceBand | BoundaryProfile | SideStructure ]
│         プロファイルは開いた列または**閉ループ**(高架の360度壁・床版裏面・箱桁)
├─ PavedArea    { id, outer: 縁要素列, holes: [縁要素列](島),
│                 surface style, gates: [segment 接続口] }
│      ← 面の正本。縁要素 = segment端の断面 | 自由 Path+縁石プロファイル
│         交差点 = node から自動生成される Area(編集は approach 単位)
│         ロータリー・駅前広場 = ユーザー定義 Area(中央島・複数島 = holes)
│         「片面非表示の道路を円形に引く」偽装は採らない
└─ Marking
     ├─ LineMarking { anchor(要素境界ID+役割+s範囲) | 自由 Path, style }
     └─ AreaMarking { 自前フレーム(位置+向き), 形状パラメータ, style }
          ← ゼブラ・導流帯。向きは道路軸から独立(斜め交差点でも縞はまっすぐ)

導出(保存しない)
├─ 断面評価テーブル(sごとの境界横位置・高さ・外周・アンカー)← 唯一の決定者
├─ StripMesh / SideStructureMesh(閉プロファイルは筒押し出し)
├─ AreaMesh(縁ループ+holes の三角形分割、縁石は縁 Path から押し出し)
├─ MarkingMesh(Line=リボン、Area=フレーム内で縞生成→路面へ投影)
├─ TerrainMaskPolygon(断面・Area 外周から)
└─ LanePath(将来。導出物のみ。専用正本を持たない)
```

## 6. 要件 → データ上の置き場所

| 要件 | 置き場所 |
|---|---|
| 1 segment 内で左右カーブ | RoadSegment.alignment = Path |
| 高架の360度壁・裏面 | 断面テンプレの閉プロファイル |
| lane 増減のスムーズ接続 | SectionTransition |
| ゼブラ、斜め交差点で縞まっすぐ | AreaMarking の独立フレーム |
| 交差点内の線に直線混在 | 内部線 = Path(Line+Arc 列) |
| 白線・車パス・縁石線の型統一 | すべて Path 参照 |
| ロータリー・駅前広場・中央島 | PavedArea(holes=島、gates=接続口)。segment では表現しない |

## 7. フェーズ計画(挙動の段階導入)

**P0 — 歩ける骨格**: 曲線 alignment の単独 segment/固定断面テンプレ1種(歩道|縁石|車道)/StripMesh+中央線・外側線+マスク導出/編集再生成・save/load・**segment 削除**・決定論+不変量/交差点なし(重なりは検出して警告)。

**P1 — 接続**: T字・十字の自動 Area(corner radius・停止線・ゼブラ1種)/既存 node への接続。同一テンプレ断面同士のみ。

**P2 — 断面の可変**: テンプレ編集のデータ化/SectionTransition 実装(歩道テーパ・片側拡幅・分離帯先端終了)/境界別 MarkingRule。

**P3 — 面と立体**: ユーザー定義 PavedArea(ロータリー・広場、島1個から)/閉プロファイル押し出し(高架)/vertical+地形/`RoadsideGuide` 出力で wire 接続。

**非目標(全フェーズ)**: 交通・信号・5叉路以上の自動整形・複雑車線分岐・短距離多重遷移。ロータリーは「Area としての形状生成」まで(内部の通行意味論はやらない)。

## 8. 保留・却下の記録

- LanePath・右左折経路: 導出物としてのみ将来対応。専用正本・専用曲線型を作らない
- アウトインアウト: lane lateral offset = f(s) のデータ追加で表現可能なことのみ確認、実装保留
- ProfileJoin の丸み種別: スキーマに join 欄は置くが P0 は Sharp のみ
- ApproachOverride 全項目: P1 は corner_radius のみ
- 高架壁⇔地上縁石の自動変換: 明示的な構造切替(unsupported 遷移)

## 9. 共有基盤の方針(3バケット)

共有候補は次の3問で機械的に判定する: **① API にドメイン概念が混ざらないか ② 既に安定しているか ③ 消費者が2つ見えているか。** 3つ Yes なら A、規約で足りるなら B、ドメイン形状なら C。

**A. 今すぐ共有 — 同一リポジトリ内 `foundation` static library(wire も road もこれに依存)**
- 数学(Vec3d・frame・hash)/**Path(Line|Arc|Bezier — 新規にここへ書く。wire の曲線はケーブル専用なので移さない)**
- EditResult+エラー3分類/ID 生成・display id/永続化 archive 機構(key=value・version・field 列挙。format と version 文字列は各ドメイン)
- テスト harness(登録・理由付き失敗・fuzz 骨格)/計測カウンター
- wire という実消費者が API を検証済みのため、今共有しても投機にならない。foundation の変更は両ドメインのテストを通すことを義務付ける

**B. 規約として共有、コードは各ドメイン(小さい or ドメイン形状)**
- トランザクション(preflight→trial→commit)/preflight 検証様式/意味論表・spec_ledger・merge_readiness の運用。正本は AGENTS.md 汎用部+本書 §0

**C. 共有しない(ドメイン概念)**
- **部分更新(reconcile)を汎用化しない。** wire の reconcile は bundle/row/port が編み込まれた最難関ドメインロジックであり、道路の部分更新は構造的に単純(segment 編集→影響 mesh 再導出)。汎用フレームワーク化は「単純な側を複雑な側に通す」逆転を生む
- graph 型・生成パイプライン・断面(道路)と band(wire)の統一もしない

グレーな候補(例: 曲線サンプリングが両者で収束した場合)のみ「後で抽出」を適用する。viewer は共有前提(同一 Svelte/three シェル・bridge 様式・workspace 永続。ドメイン別は scene 部分と操作パネルのみ)。

wire との実行時結合は `RoadsideGuide`(道路脇の基準 Path+側+オフセット)1本のみ。相互に内部構造を読まない。

## 10. 意味論表の初期セル(P0)

| 操作 \ 状態 | 空 | 既存 segment あり | 自己交差入力 |
|---|---|---|---|
| 道路を引く | 生成 | 独立生成(接続なし=P0) | unsupported |
| 点/曲線を編集 | - | 再生成 | unsupported |
| segment 削除 | - | 定義済み(P0 実装) | - |
| save/load | 空を復元 | bit 一致 | - |

P1 以降、接続・交差・Area のセルを埋めてから実装する。

## 11. 未決事項

1. リポジトリ配置: 推奨=同一リポジトリ内の独立モジュール(wire core への include 依存ゼロを lint で強制)。完全別リポジトリでも可(統合時のビルド統合コストを許容するなら)
2. 断面テンプレ初期値(車道幅・歩道幅・縁石高)
3. P0 の重なり検出の扱い(警告のみ or unsupported)
4. Path 編集 UI(クリック点列→自動 Arc 化で開始し、ハンドル編集は後、で良いか)
