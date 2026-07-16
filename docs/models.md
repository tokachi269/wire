# Model方針

このドキュメントは、自動生成placeholder(円柱pole等)から自作モデル(電柱、碍子、変圧器、端子函、広告等)へ
品質を上げるための設計方針をまとめる。特定のゲームエンジンは前提にしない。

## 位置づけ

architecture.mdの境界を変えない。
coreはbackend非依存のcurve、bounds、model assembly、style参照を出力し、mesh assetはadapter側で解決する。
**coreはmeshを読まない。** adapterはローカル測定値を渡し、world配置と電線接続点はcoreが決める。

## 3層構造

```text
mesh asset(fbx/glTF等)   adapter側。coreは読まない
model descriptor           meshから抽出+手動overrideした一時的な数値契約。engine非依存
model assembly template    配置に使うpart、socket、local transform。core正本
core template              PoleTypeDefinition / BundleTemplate等。assembly idを参照する
```

descriptorはengine非依存の数値契約であり、エンジン決定前に固定できる。
bbox等の抽出はdescriptorを生成するimportツールの仕事で、coreには入れない。

BundleTemplate の identity は `BundleTemplateId` とし、`BundleKind` は分類タグとしてだけ使う。
asset / descriptor / viewer は kind から template を一意に推測しない。同じ kind の template が複数ある場合も、
参照・編集・population/regenerate scope は必ず `BundleTemplateId` で渡す。

## 接続点: markerを正、bboxは粗い初期値

- 接続点(socket)はDCC内のnamed marker(empty等)、またはasset固有adapterが明示するローカルanchorを正本にする。
  bboxから取れるのは全高・接地半径などの粗い値だけで、碍子の線受けや函の端子位置は原理的に出ない。
- 調整はアプリ内の数値入力ではなくDCC内のmarker移動で行う。
- scene内の近傍geometryからsocketを補完しない。markerが無いassetは、生成前bootstrapでadapterが
  model descriptorへ安定名付きanchorを明示する。

## 測定とoverrideの分離

- descriptor = 抽出結果(測定レイヤ) + 手動override(補正レイヤ)の合成。2つを混ぜて保存しない。
- overrideはmarker名・band名など安定キーで持つ。

## 再読込

- 再抽出は測定レイヤだけを置き換える。overrideは残す。
- 差分report(marker消失、寸法変化)を出す。消えたmarkerを参照するoverrideは補完せずconflictとして報告する。
- core templateへの反映は既存post-edit経路に乗せる。placement-only差分はkReposition/kReshape、
  構造差分はunsupported/regenerate。**モデル再読込専用の更新経路は作らない。**

## 断面とテーパー

- 電柱は円柱に限定しない(四角柱等がある)。descriptorにはsection(h)を数点サンプル
  (接地・各band高さ・頂部)で持つ: shape(circle / rect / octagon等)+ 寸法。連続関数や断面解析はしない。
- coreはbandごとの外接円半径として消費する(center hintのclearance等、既存C80系)。
  方位依存の半径はv1では扱わず、見た目上必要になった時に精緻化する。
- 四角柱は面が向きを持つため、バンド・広告のplacementを面にスナップする拡張は将来課題としてメモに留める。
- 精度が要る高さだけoverrideで補正する。
- v1のmesh beltは円形poleに限定し、配置高さの`pole_radius_at_height_m`へ径方向scaleする。
  非円形断面や幅方向の完全なテーパー追従は対応済みとしない。
- belt adapterはGLBのring axisと基準内周半径を一度だけ確定し、ring断面を内周半径1.0へ正規化する。
  Coreはbelt中心のpole-local高さで半径を一度だけ評価し、`target pole radius / source inner radius`
  に相当する同一scaleをring断面の2軸へ適用する。belt幅方向のscaleは常に1.0とする。
- この近似ではbelt幅内のpole taper差を無視し、板厚の径方向成分も半径比に応じて伸縮する。
  下端・上端半径、頂点単位の径方向変形、専用shader、高さ別belt assetは持たない。

## 電柱本体の長さ

- 電柱本体は長さ別meshを基本的に持たない。十分長い本体meshを使い、地面下への埋まり量で見える高さを吸収する。
- pole adapterは接地高さの基準外半径を一度だけ測定し、柱断面を半径1.0へ正規化する。
  Coreの`kPoleHeight`はpole kindの論理基部半径を断面2軸へ、PoleType基準高に対する高さ比を長手軸へ適用する。
  これによりbeltとpole meshは同じ`pole_radius_at_height_m`を基準に表示される。
- 取付位置はmesh下端基準にしない。地面基準またはpole local height基準で扱い、埋め込み量でport、socket、金具位置がずれないようにする。
- core/templateが持つのは高さ、見える高さ、中心軸、local frame、section/radius等の数値契約であり、meshの下端や三角形面を取付基準にしない。
- 何角柱として描くか、LODをどう切り替えるかはrender/asset側の都合とする。部品配置はmesh面番号ではなく、pole local frame上の高さ、角度、半径方向offsetで表す。
- 長さ別meshやend-middle-end分割を検討する対象は、電柱本体ではなく、横柱、腕金、吊り金具、クランプ列、バンド付き部品、端部形状を持つケーブル装飾などに限定する。

## 自作modelとfitの境界

| 種別 | 扱い |
|---|---|
| 電柱本体、碍子、変圧器、端子函、広告板 | 自作mesh |
| ベルト・バンド類(狭い円形断面用v1) | 自作mesh + 配置高さの径方向scale |
| 腕金(crossarm) | 自作mesh。beltと同じrow assemblyに含める |

beltのfitは単一transformで表せる円形柱近似に限定する。非円形断面や幅方向の変形が必要なら、
このfit modeを拡大解釈せず別scenarioとして設計する。

## pole側modelのowner

```text
Pole                         -> pole visual assembly(body、電線非関連装飾)
Pole + SavedBackboneRowKey   -> row fixture assembly(crossarm、belt)
Port                         -> endpoint fixture assembly(insulator、clamp相当)
Attachment                   -> span途中のinline model
```

Coreのmodel contractには用途名を持ち込まない。assemblyはpart、local transform、限定fit mode、
任意のwire socketだけを持つ。wire socket無しassemblyは表示だけで、curve endpointを変更しない。

model assemblyのworld materializationはbackboneのlayout endpoint resolverが所有し、初回生成とpost-editで
同じ経路を使う。`VisualModelInstance`はderived cacheであり、Pole、row、PortやSavedBackboneGraphへ
model instance identityを追加しない。

row fixtureは任意の`endpoint_mount_socket`を公開できる。このsocketのworld位置とrow rootの差分だけを
各Portのendpoint fixture rootへ加え、腕金の柱表面取付位置を碍子へ伝える。endpoint fixtureの回転は
従来どおりPoleFrame/layout yawが所有し、row partのGLB軸補正rotationやscaleを親子継承しない。
endpoint fixture表示とlayoutのwire endpointは同じ`resolve_endpoint_placement`結果を使用する。

v1のwire socketはcurve endpointの位置を決める。socketのlocal directionはdescriptor/assemblyへ保持するが、
現行のcurve tangent authorityにはしない。挿し込み型fixtureもadapterが測定長とwire socketを渡し、
Coreは用途名やGLB軸を知らず同じPort endpoint placementで配置する。方向をG1拘束へ使う場合は、continuity policyとnode patchの契約を
含めて別scenarioとして設計する。

## 表面占有(重なり回避)

- 電柱meshは素のテーパー柱にし、柱表面を占有するもの(昇降ボルト、バンド、広告、端子函)は
  すべてplacement対象にする。占有は pole ごとの(高さ区間 × 角度区間)リストで管理する。
- mesh同士の実行時衝突判定はしない。宣言済み区間の照合のみで解く(placement reserveと同じ考え方)。
- ボルト等を焼き込んだmeshを使う場合は、descriptorにkeep-out zone(側・高さ範囲・角度)を宣言させる。

## span内モデル(玉碍子・端子函)

既存機構を使う。新概念は追加しない。

- 正体は span 上の Attachment + socket + `AttachmentLineInteractionMode::kReplaceWithInternalPath`。
  元curveの`replaced_interval`をhiddenにし、socket間をつなぎ直す。
- `internal_path`が無い置換は合法。モデル本体が区間を埋める前提として、元curveの区間だけをhiddenにする。
- 玉碍子: socket 2つ + 区間置換。線はsocket Aで終端し、碍子長ぶんhidden、socket Bから再開。
- 端子函: in/out socketで幹線を函へ落として出す。引込線用socketを追加すれば降り線の接続点になる。
- socket位置はmodel descriptorのmarkerから供給する(上記3層構造と同一の経路)。
- descriptorから`AttachmentTemplate`へはpure functionで変換する。`line_in` / `line_out` markerを
  main line置換の必須socketとし、`drop` markerは補助socketとして保持する。欠けたmarkerは補完せずconflictにする。
- 既存sceneへの反映は`UpdateAttachmentTemplate`を使う。モデル再読込専用の登録・更新経路は作らない。

### 決定済みの派生規則

- モデルposeはcurveから一方向導出する: pose = 弧長s位置 + tangent frame。socket世界位置はposeから導出し、
  線の端がsocketへ合わせに行く。モデル位置へcurveを寄せる逆方向は禁止。
- 1 spanに複数置いた場合、replaced intervalの重複はvalidatorで報告する。補完しない。

### 未決(別タスクで決める)

- 点荷重によるsag変形(重い函で弛みが折れ線化)は現行scope外。やる場合はcurve profile hint拡張(C277系)。

## 依存する解除項目

merge_readiness.mdのunsupported保留一覧のうち、モデル対応を進めると優先度が上がるもの:

| 項目 | 理由 |
|---|---|
| `UpdateAttachmentTemplate`の構造差分 | socket追加/削除/id変更、mode変更、internal path本数/socket参照/kind変更は、使用中 attachment の意味を変える再読込 conflict とする。conflict解決の正本/退役規則が設計されるまで mutation 前に拒否し、構造 lifecycle として別設計にする |
| `UpdateAttachmentTemplate`の幾何差分 | socket位置/方向、internal path local_points/coil値は既存更新経路でkReshapeし、対象spanを再導出する |
| endpoint attachment生成/退役規則 | `CableTemplate.default_endpoint_attachment_template_id` は backbone pipeline が `AttachmentOrigin::kDefaultEndpoint` の endpoint attachment だけを reconcile する。user attachment は保持し、退役spanにuser attachmentがあれば regenerate は mutation 前に unsupported とする |

## レンダラ移行との順序

wasm + three.js への移行を予定しているため、モデル作業を2つに分ける。

- エンジン非依存部分(descriptor契約、marker規約、socket接続、replaced_interval、表面占有)は
  レンダラを待たずに進める。
- mesh を画面に出す部分(glTF load、instancing、material、LOD)は
  three.js 側で初実装する。**desktop viewer(raylib)に mesh loading を実装しない**(捨てる投資になる)。
- three.js 化自体は、既存出力(VisualCurvePart / render cache / placeholder)だけを描く最小縦スライスを
  モデルより先に通す。wasm境界を渡るのは core の派生出力のみとし、JS側に判断を持たせない。

## 進め方

1. descriptor契約(marker命名規約・項目)を文書で固定する
2. 電柱1・碍子1・変圧器1の3体だけdescriptorを手書きしてviewerで回す(抽出ツールは書かない)
3. 再読込・override・conflict報告のループを固定してから、bbox/marker抽出を自動化する
4. crossarmとmesh beltを同じrow assemblyとして表示し、円形pole向けradial fitを確認する

importパイプラインの実装から始めない。契約 → 手書き検証 → 自動化の順とする。
