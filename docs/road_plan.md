# 道路生成 方針・要件書(v3・自己完結版)

この文書は単体で新規チャット・新規リポジトリへ引き継ぐことを想定して書かれている。
前提となる外部ログ(AIとの検討履歴)は破棄してよい。必要な結論はすべて本文に含む。

## 0. 背景(引き継ぎ用の最小文脈)

- 上位製品の範囲と名称は未決定。現在は複数の生成・編集ドメインを同一リポジトリで扱う。既存の第一ドメインは`city::wire`(電柱・電線網。C++ core + wasm + three.js/Svelte viewer)、道路は`city::road`。
- `city::wire`で確立済みで、各ドメインに共通させる規約(**リポジトリ共通規約**と呼ぶ):
  1. 状態はInput / 正本(authoritative) / 派生に分類。正本を書けるのは所有する操作APIのみ。派生は一方向導出で保存しない
  2. 操作はトランザクション(事前検証preflight → trial状態で実行 → 成功時のみcommit)。全外部入力の有限性・範囲検証はpreflightに一元化
  3. エラーは3分類: validation(入力不正) / unsupported(仕様上の拒否) / internal(バグ検出)。fallbackで黙って通さない
  4. 永続化は正本のみ。versioned text(key=value行形式)、未知versionは明示reject、保存境界で圧縮
  5. 識別はID。位置の近接・頂点番号・名前文字列で同一性を推測しない
  6. 「操作×状態の意味論表」を維持し、未定義セルは実装禁止(unsupported)
  7. テストはfail-first・理由付き失敗メッセージ・seed付きfuzz+不変量検査・skipを通過と数えない
- wireの反省: 要件の意味論を決めずに実装が先行して手戻りが大きかった/削除操作を後回しにした/投機的な型を先行させて死にデータ化した。本計画はこれらを初日から避ける。

## 1. 目的と品質バー

- Cities: Skylines 2の道路のような「横断構成を組み替えられ、異なる道路同士を破綻なく接続できる道路」を、少数の機構で生成する。
- 品質バー: **街路レベルで眺めて自然。対応範囲では端部・遷移まで完成。曖昧なケースは補完せずunsupportedで止める。**
- 機能を限定することと、対応範囲の品質を下げることを混同しない。
- 交通シミュレーション・信号制御・排水計算・土木CAD精度は対象外。横断勾配、路面の丸み、側溝、縁石、高架壁などは見た目の形状として扱う。
- 個別モデル・テクスチャ・設備の網羅は初期設計の中心に置かない。先に、形状・接続・編集の契約を成立させる。

## 2. 設計原則

1. **挙動は段階導入、先に固定するのは契約であって将来の全スキーマではない。**
   - 初期に固定する: 正本/派生の所有、ID参照、接続境界、導出方向、保存versionとmigration方針
   - 初期に固定しない: 消費者のいない将来フィールド、未実装機能のスロット、用途が未確定な専用型
   - 将来の正当なスキーマ変更はversion migrationで扱う
2. **決定者一箇所。** 断面評価の結果(境界・外周・アンカー・接続口)を、路面・白線・交差点・地面マスク・将来の車両経路が同一テーブルから消費する。各生成器が独自に幅や高さを再計算しない。
3. **意味モデルと描画メッシュを分ける。** 車線・歩道・分離帯などは意味上別要素として保持するが、同じ材質・連続面なら描画メッシュは結合してよい。
4. **自動生成物と手動正本を分ける。** 自動中央線・外側線・交差点面は保存しない。ユーザーが置いた自由線・ゼブラ・広場などは正本として保存する。
5. **専用型を増やしすぎない。** 曲線表現はroad内の共通Pathを使う。ただし所有者・座標空間・用途属性は参照側が持つ。異なる所有や意味を無理に一つの正本型へ統合しない。

## 3. wireとの構造的な違い

| | wire | 道路 |
|---|---|---|
| node | 支持点(電柱)そのもの。曲がるたびにnode | **接続点・端点のみ**。カーブはnodeにしない |
| segment | 2 node間=上面視で直線 | 2 node間に**任意の平面曲線**(S字も1 segment) |
| 面要素 | なし | **あり**(自動交差点面、ロータリー、駅前広場など) |
| 接続 | lane継続テーブル | 断面要素対応+ConnectionGate。思想は同じだが型は共有しない |

backboneの思想(正本グラフ→一方向導出、接続は操作時に決定して記録、再生成はreconcile)は引き継ぐ。構造・型は引き継がない。

## 4. road内共通プリミティブ: Path

```text
Path = 順序付きprimitive列
primitive = Line | Bezier
```

- 初期配置はroadモジュール内。別の実消費者が現れ、APIが安定してからfoundationへの抽出を判断する。
- 保存Pathは原則として**所有者ローカルの2D曲線**。segmentの平面線形、Areaの縁、手動マーキング中心線などに使う。
- 評価は弧長s。Path自体は道路の高さ・roll・world-upフレームを決めない。
- 3D位置と姿勢は所有者の評価器が決める。

```text
RoadSurfaceEvaluator
= horizontal Path
+ phase導入後のvertical profile
+ phase導入後のroll profile
+ cross-section evaluation
→ 3D位置・接線・横方向・上方向
```

- Frenet frameを道路姿勢の決定者にしない。world-up基準またはねじれを抑えたフレームをRoadSurfaceEvaluatorで構成する。
- 同じXY位置に地上道路と高架道路が存在できるため、自由Pathは必ず所有するsurface/Areaを参照する。無所属Pathをワールド路面へ最近傍投影しない。

## 5. データモデルと所有

以下は概念上の到達形である。各フィールド・型は消費コードが入るフェーズで追加し、未使用スロットとして先行実装しない。

```text
正本(SavedRoadGraph)
├─ RoadNode
│    { id, position }
│    ← 接続点・端点のみ
│
├─ RoadSegment
│    { id, node_a, node_b, alignment: Path, section_timeline }
│
├─ SectionTimeline
│    { 区間ごとのCrossSectionTemplate参照、必要なphaseでSectionTransition参照 }
│
├─ CrossSectionTemplate
│    ├─ surface_chain
│    │    └─ 横断方向に並ぶ[SurfaceBand, BoundaryProfile, SurfaceBand, ...]
│    ├─ edge_structures
│    │    └─ 外周または指定境界へ付くSideStructure
│    └─ structural_shells
│         └─ 高架床版下面・箱桁など、路面チェーンと別所有の開/閉プロファイル
│
├─ SectionTransition                 ← P2で追加
│    { from/to、station範囲、要素対応表、anchor、各要素の開始/終了規則 }
│
├─ JunctionDefinition                ← P1で追加
│    { node_id、approachごとの接続設定、corner radius、許可されたoverride }
│    ← 自動交差点面そのものは保存しない
│
├─ UserPavedArea                     ← P5で追加
│    { id、owner frame、outer、holes、surface style、connection gates }
│    ← ロータリー・駅前広場などユーザーが作る面
│
└─ ManualMarking                     ← 必要なphaseで追加
     ├─ ManualLineMarking
     │    { owner_surface_id、anchor参照またはowner-local Path、style }
     └─ ManualAreaMarking
          { owner_surface_id、自前2D frame、形状パラメータ、style }
```

### 5.1 横断断面の役割

```text
SurfaceBand
- 車道、歩道、路肩、自転車帯、中央分離帯、排水用の浅い帯など
- 幅、表面高さプロファイル、意味role、surface styleを持つ

BoundaryProfile
- 隣接SurfaceBandの接続形状
- 段差なし、鋭角、縁石、浅い溝、丸みなど
- P0は必要な固定形状だけ。未実装join種別のフィールドは先に置かない

SideStructure
- 高架壁、遮音壁、ガードレール、擁壁など
- surface_chainの外周または安定したboundary IDへ付く

StructuralShell
- 高架床版側面・下面、箱桁など
- surface_chainと同じ順序列に混ぜない
```

車線・歩道・分離帯は意味上別要素として保持する。描画時は、同一材質かつ法線・UV・衝突用途を共有できる隣接面をまとめてよい。

## 6. 導出データと接続境界

```text
導出(保存しない)
├─ SectionEvaluationTable
│    { stationごとの各boundary横位置・高さ・role・外周・anchor }
├─ ConnectionGate
├─ SegmentSurfaceMesh / SideStructureMesh / StructuralShellMesh
├─ JunctionArea / JunctionAreaMesh
├─ UserPavedAreaMesh
├─ AutoMarking / MarkingMesh
├─ TerrainMaskPolygon
└─ LanePath(将来。交通対応時のみ)
```

### 6.1 ConnectionGate

異なる道路断面や交差点を破綻なく接続するため、segment端の接続情報を一度だけ評価する。

```text
ConnectionGate
├─ gate_id
├─ segment_id / end role
├─ station
├─ 3D frame(位置・接線・横方向・上方向)
├─ 断面要素の順序
├─ 各boundaryの3D位置・法線・role・element ID
├─ surface外周
└─ marking anchor
```

所有境界:

```text
segment mesh   = gate直前までを所有
auto junction  = gate境界から内側を所有
gate上の位置   = SectionEvaluationTableから一度だけ算出し、両方が共有
```

segment側とjunction側が別々に幅・高さ・頂点を計算してはならない。接続面の隙間、高さ差、法線差を不変量テストで検出する。

### 6.2 自動面とユーザー面

- `JunctionArea`は`JunctionDefinition + ConnectionGate[]`から導出する。正本として保存しない。
- `UserPavedArea`はユーザー入力そのものなので正本として保存する。
- 自動交差点とユーザー広場を同じ正本型として扱わない。

### 6.3 自動マーキングと手動マーキング

- 中央線・車線境界線・外側線などは断面要素間のBoundary roleとMarkingRuleから導出する。
- 停止線・ゼブラはP1ではJunctionDefinitionから自動導出してよい。
- ユーザーが編集・追加した自由線やAreaMarkingだけをManualMarkingとして保存する。
- MarkingMeshは路面テクスチャへ焼き込まず、リボンメッシュ、デカール、シェーダー等の描画方式から独立した意味データを使う。

## 7. 断面遷移

異なる断面をnode一点で急変させず、接続前の明示的な`SectionTransition`で整える。

```text
通常segment
→ SectionTransition
→ ConnectionGate
→ JunctionAreaまたは次segment
```

要素ごとの遷移結果は次のいずれかに限定する。

```text
- 継続
- 幅・高さ・横位置を変えながら継続
- 徐々に終了
- 徐々に発生
- 専用端面で終了
- 自動接続不可(unsupported)
```

初期対応例:

- 歩道あり→なし: 歩道幅を徐々にゼロへする
- curbあり→なし: curb高さ・幅を徐々にゼロへする
- 浅い側溝あり→なし: 深さを徐々にゼロへする
- 2車線→3車線: anchorを固定し、追加側を明示して片側拡幅する
- 中央分離帯あり→なし: 専用の先端終了規則を使う
- 高架壁あり→なし: 壁終端を作る
- 高架壁→地上curb: 暗黙変換せず、明示的な構造切替がなければunsupported

断面要素は頂点番号ではなく、安定したelement ID・role・boundary IDで対応付ける。

## 8. station参照と編集意味論

弧長sを保存する機能を導入するときは、alignment編集時の保持規則を明示する。単なる`double s`を正本へ置いて意味を曖昧にしない。

```text
StationRef
├─ FromStart(distance)
├─ FromEnd(distance)
└─ Ratio(u)
```

- 終端手前の車線減少などは`FromEnd`
- 始点から固定距離の設備・遷移は`FromStart`
- 道路全体に対する相対位置を保つものは`Ratio`
- どの参照方式を使うかは操作APIが明示的に決定し、再生成時に推測しない
- P0でStationRefの実消費者がなければ型を実装しない。P2導入前に意味論表を固定する

## 9. 要件→データ上の置き場所

| 要件 | 置き場所 |
|---|---|
| 1 segment内で左右カーブ | `RoadSegment.alignment = Path` |
| 歩道・車道・分離帯・側溝 | `CrossSectionTemplate.surface_chain` |
| curb・段差・丸み | `BoundaryProfile` |
| 高架壁・遮音壁 | `edge_structures` |
| 高架床版側面・裏面・箱桁 | `structural_shells` |
| lane増減・歩道消滅 | `SectionTransition` |
| 異なる道路と交差点の接続 | `ConnectionGate + JunctionDefinition` |
| 自動中央線・外側線 | Boundary roleから導出する`AutoMarking` |
| 手動白線・自由線 | owner付き`ManualLineMarking` |
| ゼブラ、斜め交差点で縞を独立方向に保つ | `ManualAreaMarking`または自動AreaMarkingのowner-local frame |
| ロータリー・駅前広場・中央島 | `UserPavedArea(holesあり)`。segmentで偽装しない |
| 地面非表示 | 最終Section/Area外周から導出する`TerrainMaskPolygon` |
| 高さの違う道路の非接続交差 | P3のvertical評価+接続明示。XY交差だけでnodeを作らない |

## 10. アウト・イン・アウトとLanePath

「道路の車線形状」と「車両が車線内を通る経路」を分ける。

```text
A. 車線境界自体がアウト・イン・アウト形状
   → boundary lateral offsetの道路形状
   → 白線・車線中心も追従

B. 白線は通常形状で、車両だけ車線内をアウト・イン・アウト
   → 将来のLanePath
   → 交通シミュレーション非目標の間は実装しない
```

- Aは`boundary lateral offset = f(s)`のようなデータで将来表現可能だが、初期実装しない。
- Bは交通機能が必要になった時点で導出物として追加する。専用正本や専用曲線型を先に置かない。

## 11. フェーズ計画(挙動の段階導入)

### P0 — 歩ける骨格

- road内Pathによる単独曲線segment
- 固定断面テンプレ1種(日本の一般的な都市部2車線):
  - 歩道2.0m | curb0.2m/段差0.15m | 車道6.0m(3.0m×2) | curb0.2m/段差0.15m | 歩道2.0m
  - 車道は中央から左右へ2%の横断勾配、歩道は外側へ1%の横断勾配
  - 浅い側溝はP0では独立SurfaceBandにしない。必要なら固定BoundaryProfileの見た目としてだけ扱い、編集可能な側溝種別はP2以降
- SectionEvaluationTable
- SurfaceMesh、固定中央線・外側線、TerrainMask導出
- alignment編集→全派生再生成
- Path編集UIは初期からCities系道路ツール相当の引き方にする。開始点指定、ライブプレビュー、終点確定、直線/Bezier(S字含む)の作成とハンドル編集をP0対象に含める
- save/load、segment削除
- 決定論テスト、不変量テスト、自己交差reject
- 交差点なし。既存segmentとの重なりはP0では判定しない。接続も警告も発生させず、独立segmentとして生成する

### P1 — 同一断面の接続

- ConnectionGate
- 既存nodeへの接続
- 同一CrossSectionTemplate同士のT字・十字
- JunctionDefinitionからJunctionAreaを自動導出
- 接続角度45〜135度、最小segment長8m、corner radius初期値4m
- 停止線、ゼブラ1種
- segment/gate/junctionの隙間・法線・所有重複を不変量検査

### P2 — 断面の可変と手動マーキング

- CrossSectionTemplate編集の正本化
- StationRefの意味論と実装
- SectionTransition
- 歩道テーパ、片側拡幅、車線増減、分離帯先端終了
- boundary別MarkingRule
- ManualLineMarking / ManualAreaMarking

### P3 — 縦断・地形・立体交差

- VerticalProfileとRollProfileをこのphaseで初めてスキーマへ追加
- 横断クラウンと断面全体rollを分離
- 地形追従、切り込み用TerrainMask
- 高さの異なる道路の非接続交差
- 地上区間と高架区間の構造mode境界

### P4 — 高架の連続形状

- edge_structuresによる高架壁
- structural_shellsによる床版側面・下面・閉プロファイル押し出し
- 高架壁の開始・終了
- 地面マスクの無効化・遷移
- 橋脚等の個別モデル配置は別計画。P4の完了条件に含めない

### P5 — ユーザー定義Area

- UserPavedArea
- outer + hole 1個から開始
- ロータリー・広場の形状生成
- gateによるsegment接続
- 内部の交通意味論は実装しない

### P6 — wire連携

- `RoadsideGuide`出力
- 道路脇の基準Path、side、offset、許可区間だけを公開
- wireとroadは相互に内部構造を読まない

**非目標(全フェーズ)**: 交通・信号・5叉路以上の自動整形・複雑車線分岐・短距離多重遷移・排水計算。ロータリーはAreaとしての形状生成まで。

## 12. 保留・却下の記録

- LanePath・右左折経路: 交通対応時の導出物。専用正本・専用曲線型を先に作らない
- アウトインアウト: 道路境界を動かす場合と車両経路だけを動かす場合を分離。両方とも実装保留
- BoundaryProfileの丸み種別: P0は必要な固定断面だけ。未使用join欄は置かず、丸み編集を実装するphaseで追加
- ApproachOverride: P1はcorner radiusのみ。未使用override群を先に定義しない
- 高架壁⇔地上curbの自動変換: 明示的な構造切替がなければunsupported
- 個別モデル・テクスチャ: 道路形状・接続契約の後。初期文書で網羅しない

## 13. 共有基盤の方針(3バケット)

共有候補は次の3問で判定する。

1. APIにドメイン概念が混ざらないか
2. 既に安定しているか
3. 実消費者が2つあるか

3つYesならA、規約で足りるならB、ドメイン形状ならC。

### A. 今すぐ共有

同一リポジトリ内`foundation` static library。`city::wire`も`city::road`も依存可能。ただし共有対象が抽出されるまで空のlibraryは作らない。

- 既に安定している数学型(Vec3d等)、hash
- EditResultとエラー3分類
- ID生成・display ID
- 永続化archive機構(key=value、version、field列挙。format/version文字列は各ドメイン)
- テストharness、理由付き失敗、fuzz骨格、計測カウンター

foundation変更時は両ドメインのテストを通す。namespaceは`city::foundation`とする。

### B. 規約として共有、コードは各ドメイン

- トランザクション(preflight→trial→commit)
- preflight検証様式
- 意味論表、spec_ledger、merge_readinessの運用
- 正本はAGENTS.md汎用部+本書§0

### C. 共有しない

- graph型、生成パイプライン、断面とwire bandの統一
- 部分更新reconcileの汎用化
- Junction、SectionTransition、ConnectionGate等の道路概念

`Path(Line|Bezier)`は新規かつ初期消費者がroadだけなので、まずroad内に置く。別の実消費者が現れ、APIが収束した場合のみfoundationへ抽出する。

viewerは共有前提(同一Svelte/threeシェル、bridge様式、workspace永続)。ドメイン別なのはscene部分と操作パネル。

wireとの実行時結合は`RoadsideGuide`一本のみ。

## 14. 意味論表の初期セル(P0)

| 操作 \ 状態 | 空 | 既存segmentあり | 自己交差入力 | 既存segmentと重なる |
|---|---|---|---|---|
| 道路を引く | 生成 | 独立生成(接続なし=P0) | unsupported | 判定しない。独立生成 |
| 点/曲線を編集 | - | 再生成 | unsupported | 判定しない。再生成 |
| segment削除 | - | 定義済み(P0実装) | - | - |
| save/load | 空を復元 | 正本bit一致 | - | - |

P1以降、接続・交差・Area・遷移のセルを埋めてから実装する。

## 15. P0不変量

- 全IDが一意で参照先が存在する
- Path primitive列が連続し、弧長評価が有限
- 断面評価のboundary順序が横断方向に一貫する
- 生成メッシュにNaN/Inf、範囲外index、ゼロ面積triangleがない
- 同一正本からの再生成が決定論的
- save/load後の正本がbit一致
- unsupported/validation失敗時に正本と派生が変化しない
- segment削除後にsegment由来の派生物が残らない
- TerrainMask外周が最終断面外周と同じ決定表を消費する

P1ではConnectionGate共有、junctionとの隙間・重複・法線不一致を追加する。

## 16. 未決事項

1. マテリアル境界と描画メッシュ結合の最小ルール(意味要素は分けたまま、どこまで同一mesh/material groupへまとめるか)

## 17. 決定済み事項

1. リポジトリ配置: `domains/wire` (`city::wire`)と`domains/road` (`city::road`)を独立配置する。roadからwireへのinclude依存ゼロをlintで強制する。共有実装は条件を満たしたものだけ`foundation` (`city::foundation`)へ置く
2. P0固定断面: 日本の一般的な都市部2車線を初期値にする。車線3.0m×2、歩道2.0m×2、curb幅0.2m/段差0.15m、車道横断勾配2%、歩道横断勾配1%。浅い側溝はP0では独立SurfaceBandにしない
3. P0の既存segment重なり: 判定しない。warningにもunsupportedにもせず、接続なしの独立segmentとして扱う
4. Path編集UI: Bezierハンドル編集を後回しにしない。P0からCities系道路ツール相当のライブプレビュー、直線/Bezier作成、ハンドル編集を対象にする
5. P1接続範囲: 同一断面のT字・十字だけを対象に、接続角度45〜135度、最小segment長8m、corner radius初期値4mとする
