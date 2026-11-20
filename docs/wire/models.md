# モデル方針

この文書は、円柱で表した電柱などの自動生成placeholderを、電柱、碍子、変圧器、端子函、広告などの自作モデルへ置き換える際の設計方針をまとめる。特定のゲームエンジンは前提にしない。

## 位置づけ

`architecture.md`と`../architecture.md`の境界を変えない。Coreはbackendに依存しないcurve、bounds、model assembly、styleへの参照を出力する。mesh assetはadapter側で解決し、Coreはmeshを読まない。adapterはローカル座標での測定値を渡し、ワールド座標での配置と電線接続点はCoreが決める。

## モデルの3層構造とCoreテンプレート

| 層 | 内容 |
|---|---|
| mesh asset(fbx/glTF等) | adapter側。coreは読まない |
| model descriptor | meshから抽出し手動overrideを足した一時的な数値契約。engine非依存 |
| model assembly template | 配置に使うpart、socket、local transform。core正本 |
| core template | `PoleTypeDefinition` / `BundleTemplate`等。assembly idを参照する |

descriptorはエンジンに依存しない数値契約であり、エンジンを決める前に固定できる。bbox等の抽出は、descriptorを作るインポート処理で行い、Coreには入れない。

BundleTemplateの識別情報は`BundleTemplateId`とし、`BundleKind`は分類タグとしてだけ使う。asset / descriptor / viewerはkindからテンプレートを一意に推測しない。同じkindのテンプレートが複数ある場合も、参照・編集・population/regenerate scopeは必ず`BundleTemplateId`で渡す。

`bundle_templates`のkey、`Bundle.bundle_template_id`、backbone spec / saved graph / binding、random bundle rule、regenerate scopeも`BundleTemplateId`を使う。

kindで探すAPIはgrouping/filterとして複数idを返し、`BundleKind`から一意のテンプレートを引かない。

## 接続点: markerを正、bboxは粗い初期値

- 接続点(socket)はDCC内のnamed marker(empty等)、またはasset固有のadapterが明示するローカルanchorを正本にする。
  bboxから取れるのは全高・接地半径などの粗い値だけで、碍子の線受けや函の端子位置は原理的に出ない。
- 調整はDCC内でmarkerを移動して行う。アプリ内の数値入力では調整しない。
- scene内の近傍幾何からsocketを補完しない。markerが無いassetは、生成前bootstrapでadapterが
  model descriptorへ安定名付きanchorを明示する。

## 測定とoverrideの分離

- descriptor = 抽出結果(測定レイヤ) + 手動override(補正レイヤ)の合成。2つを混ぜて保存しない。
- overrideはmarker名・band名など安定キーで持つ。

## 再読込

- 再抽出は測定レイヤだけを置き換える。overrideは残す。
- 差分report(marker消失、寸法変化)を出す。消えたmarkerを参照するoverrideは補完せずconflictとして報告する。
- coreテンプレートへの反映は既存post-edit経路に乗せる。placement-only差分はkReposition/kReshape、
  構造差分はunsupported/regenerate。モデル再読込専用の更新経路は作らない。

## 断面とテーパー

- 電柱は円柱に限定しない(四角柱等がある)。descriptorにはsection(h)を数点サンプル
  (接地・各band高さ・頂部)で持つ: shape(circle / rect / octagon等)+ 寸法。連続関数や断面解析はしない。
- coreはbandごとの外接円半径として消費する(center hintのclearance等、既存C80系)。
  方位依存の半径はv1では扱わず、見た目上必要になった時に精緻化する。
- 四角柱は面が向きを持つため、バンド・広告のplacementを面にスナップする拡張は将来課題としてメモに留める。
- 精度が要る高さだけoverrideで補正する。
- v1のmesh beltは円形poleに限定し、配置高さの`pole_radius_at_height_m`へ径方向scaleする。
  非円形断面や幅方向の完全なテーパー追従は対応済みとしない。
- beltモデルはadapterが渡すsource authoring radiusに合う寸法でauthoringする。現行assetではその値が
  pole mesh下端径に由来していても、Coreはmesh下端、bbox、`h=-2m`を取付基準として読まない。
  adapterはdescriptorの`radialReference`としてsource authoring radiusを固定し、ring断面をその半径へ正規化する。
- beltのlocal mount anchorはcenterとする。Coreは配置高さでtarget pole radiusを
  `pole_radius_at_height_m`から求め、最終radial scaleが
  `target pole radius / source authoring radius`に一致するように同一scaleをring断面の2軸へ適用する。
  belt幅方向のscaleは常に1.0とする。
- Contract tags: `belt_mount_anchor_center`, `core_does_not_use_mesh_lower_h_or_bbox`,
  `belt_radial_scale_target_over_source`, `no_belt_clearance_field`, `no_belt_vertex_deformation`.
- この近似ではbelt幅内のtaperをasset形状に委ね、板厚の径方向成分も半径比に応じて伸縮する。
  clearanceフィールド、下端・上端半径、頂点単位の径方向変形、専用shader、高さ別belt assetは持たない。

## 変圧器のバンド

beltは円環1つなので配置高さの`pole_radius_at_height_m`へradial scaleできる。変圧器のバンドは電柱とタンクの両方を1周する閉ループであり、2つの円の共通接線で形が決まるため、単一transformでは電柱径に追従しない。したがってバンドは焼き込みとし、fit modeは`kRigid`に限定する。

- 生成元は`tools/models/pole_transformer_m_01.ts`、焼き込みは`tools/models/export_pole_transformer.mjs`が行う。
- exporterはpole primitiveのtop diameterとtaper ratioから公称取付高さでの半径を求め、模型内部のtaper係数と併せて渡す。バンドが2段あっても各段の高さで正しい電柱径になる。
- 公称取付高さはexporterの`MOUNT_HEIGHT_M`とadapterの`TRANSFORMER_MOUNT_HEIGHT_M`が同じ値を持つ。変えるときは両方を直して再exportする。
- 取付高さが公称から1m動いても電柱半径の差は7mm程度で、バンドの板厚と同程度に収まる。高さ別assetは持たない。
- 腕金は焼き込まない。既存の`hv_crossarm`をassemblyの別partとして置き、長さはlocal transformのscaleで決める。
- 電柱への付き方と、どこまでを1モデルに含めるかは`pole_fittings.md`が決める。
- assetを焼き直したらassemblyの`version`を上げる。descriptorはbounds差分を検出するが、bootstrapは既存assemblyとの差分を新しいversionなしでは受け付けず、保存済みworkspaceの復元が失敗する。

## 電柱本体の長さ

- 電柱本体は長さ別meshを基本的に持たない。十分長い本体meshを使い、地面下への埋まり量で見える高さを吸収する。
- pole adapterはBlender primitive生成設定のtop diameter、visible height、taper ratioから線形半径profileを
  固定し、`PoleTypeDefinition`へ渡す。Coreはmeshを読まず、この完成済み数値を`pole_radius_at_height_m`の正本として使う。
- `kPoleHeight`はassetが持つ断面寸法を保ち、PoleType基準高に対する高さ比だけを長手軸へ適用する。
  表面fixtureとbeltも同じ半径profileを使うため、pole meshと別のテーパー式を再解釈しない。
- mesh adapterが存在しないheadless Coreでは、半径profileが未設定の場合だけpole kindの既定近似を使う。
- 取付位置はmesh下端基準にしない。地面基準またはpole local height基準で扱い、埋め込み量でport、socket、金具位置がずれないようにする。
- domains/wire/templateが持つのは高さ、見える高さ、中心軸、local frame、section/radius等の数値契約であり、meshの下端や三角形面を取付基準にしない。
- 何角柱として描くか、LODをどう切り替えるかはrender/asset側の都合とする。部品配置は、pole local frame上の高さ、角度、半径方向offsetで表す。meshの面番号は使わない。
- 長さ別meshやend-middle-end分割を検討する対象は、横柱、腕金、吊り金具、クランプ列、バンド付き部品、端部形状を持つケーブル装飾などに限定する。電柱本体は対象にしない。

## 自作モデルとfitの境界

| 種別 | 扱い |
|---|---|
| 電柱本体、碍子、変圧器、端子函、広告板 | 自作mesh |
| ベルト・バンド類(狭い円形断面用v1) | 自作mesh + 配置高さの径方向scale |
| 腕金(crossarm) | 自作mesh。beltと同じrow assemblyに含める |

beltのfitは単一transformで表せる円形柱近似に限定する。非円形断面や幅方向の変形が必要なら、このfit modeを拡大解釈せず別シナリオとして設計する。

## モデル配置のownerと親参照

| owner | 配置するモデル |
|---|---|
| `Pole` | pole visual assembly(body、電線非関連装飾) |
| `SavedBackboneRowKey` | `PlacementRule::at_row`のrow fixture assembly(crossarm、belt) |
| `Port` | `PlacementRule::at_endpoint`のendpoint fixture assembly(insulator、clamp相当) |
| Span anchor | `PlacementRule::interval`のspan上assembly(テスト用。既定テンプレートでは未使用) |
| Entity-layer `Attachment` | span途中のinline model |
| Core derived decoration | `Pole` / `SupportNode` / `Span`由来の局所detail modelと短い配線をCore visual outputへ派生 |

Coreのモデル契約には用途名を持ち込まない。assemblyが持つのは、part、local transform、限定されたfit mode、名前付きsocketだけである。socketを持たないassemblyは表示だけに使い、curve endpointを変更しない。

Core derived decorationはauthoritative entityではない。Bundle、lane、socket binding、生成済みモデルID、local cable ID、collision結果、equipment listは保存しない。ただしWireドメインの意味を使うため、viewer adapterではなくCore visual generationが、名前付きsocket、既存Bundle/Port/row/carrier情報、CableTemplate appearanceから`VisualModelInstance` / `VisualCurvePart`を再導出する。

既存のentity-layer span Attachmentの保存形式と意味は変えない。

model assemblyのworld materializationはCoreが所有し、初回生成とpost-editで同じ経路を使う。asset adapterはGLB/node/Empty由来のモデルとsocketのメタデータを渡すだけで、Wireの接続先、curve、materialは判断しない。

GLBを読む責務はasset adapterに留める。adapterはモデルのkey、local transform、名前付きsocketのメタデータを`ModelAssemblyTemplate` / `ModelAssemblyPart` / `ModelAssemblySocket`としてCoreへ渡すが、`connect_*`socketからcarrierを探したりlocal cable curveやmaterialを決めたりしない。

socket名のWire上の意味はCoreだけが解釈し、CoreはGLB parserを持たない。`VisualModelInstance`はderivedキャッシュであり、Pole、row、PortやSavedBackboneGraphへモデルinstanceの識別情報を追加しない。

deformable cable patternもmesh asset自体のparse、catalog、authoring extent、局所deformationはadapter側に置く。Coreはsagを適用したcurveとWire semanticだけを返し、GLB asset識別情報を`VisualModelInstance`や保存スキーマへ追加しない。

未完成のsupport detail catalogは、Coreの派生・保存契約を変えずviewerのpresentation availabilityとして非表示にしてよい。この境界で配置、接続先、curve、material意味論を再計算してはいけない。

lowered endpointのsocket/endpoint契約は[`generation.md`](generation.md)のPlacementを正本とする。旧`support_world -> endpoint_world`の`SupportArm` placeholderは生成しない。

row fixtureとPort fixtureは汎用のmodel assemblyから派生し、用途を表す旧来のboolや高さのscalarを判断に使わない。viewerが不足したfixtureを推測して補ってはいけない。

配置されるinstanceの親参照は次の3種だけに限定する。

| 親参照 | 意味 |
|---|---|
| `MountRef::pole_frame` | `PoleFrame`やlayout yawから作ったroot frame |
| `MountRef::span_anchor` | curve上の純関数anchor frame |
| `MountRef::instance_socket` | 親instance IDとsocket名 |

world transformはmount graphの解決関数が親チェーンを辿って合成する。深さは固定しない。socket欠落や循環はunsupportedとして明示し、近傍幾何やモデル名から補完しない。row/endpointの現行2配置もこの機構へ載せる。row fixtureは`pole_frame`、endpoint fixtureはrow側の`endpoint_mount_socket`がある場合は`instance_socket`、無い場合は`pole_frame`を親にする。

wire endpointはendpoint fixtureの`wire_socket`から得るが、layoutが決めた接続点へsocketが一致するようfixture rootを解く。socket local offsetでPort高さやbranch down policyを再決定しない。

## 回転とpole local frame

`rotation_euler_deg`は`RotateEulerXYZDeg`の実装順を正本とする。local pointへX回転、Y回転、Z回転をこの順で適用する(`X -> Y -> Z`)。逆変換はZ、Y、Xの逆順で戻す。

`BuildPoleFrame`は次の手順でpole local frameを作る。

```text
origin  = pole.world_transform.position
forward = RotateEulerXYZDeg(WorldForward, rotation_euler_deg)
lateral = RotateEulerXYZDeg(WorldLateral, rotation_euler_deg)
up      = RotateEulerXYZDeg(WorldUp, rotation_euler_deg)
```

その後、`layout_yaw_deg - rotation_euler_deg.z`だけ、すでに傾いた`up`軸まわりに`forward`と`lateral`を回す。`up`はlayout yawでは変えない。つまりtiltはpole instanceの物理姿勢、layout yawは同じpole上のrow/port配置軸をpole軸まわりに回す派生入力である。

`WorldForward`はlocal X、`WorldLateral`はlocal Y、`WorldUp`はlocal Zの基準軸である。Port、row fixture、endpoint fixture、belt fitはこの`BuildPoleFrame`の`forward/lateral/up`を読む。

adapter/viewer側でEuler順やlayout yaw合成を再解釈しない。

同じ派生rowに属するPort位置、row fixture、endpoint fixture、wire socket、curve endpointは、同じ`layout_yaw_deg`から作った`PoleFrame`を使用する。pair promotionでrow frameが変わる場合はPort識別情報を維持したまま位置を再導出し、binding yawとPort位置を一つのexact frame更新として適用する。

派生物同士の一致だけでなく、Portをそのframeへ戻したlocal lateralが正本のband、placement lateral、lane spacingから得るanchorと一致することを不変条件とする。

テンプレートの配置指定は`PlacementRule`のリストで表し、bit flagは使わない。

| 指定 | 意味 |
|---|---|
| `PlacementRule::at_row` | 既存のrow fixture相当 |
| `PlacementRule::at_endpoint` | 既存のendpoint fixture相当 |
| `PlacementRule::interval` | spacingとphaseでspan上にanchor frame列を作る純関数 |

外部APIの`rowFixtureAssemblyId` / `endpointFixtureAssemblyId`は互換入力として残すが、意味解釈はadapter 1箇所で`PlacementRule`へ変換する。モデルmaterializationは旧フィールドを直接読まない。

`interval`は現時点では機構証明用で、既定テンプレートには載せない。姿勢は配置規則の種類ではなく`world_up` / `align_tangent`のorientation policyとして直交させる。

v1のwire socketはcurve endpointの位置を決める。socketのdirectionはdescriptor/assemblyへ保持するが、現行のcurve tangent authorityにはしない。挿し込み型のfixtureもadapterが測定長とwire socketを渡し、Coreは部品の用途名やGLB軸を知らず同じMountRef/PlacementRule経路で配置する。

方向をG1拘束へ使う場合は、continuity policyとnode patchの契約を含めて別シナリオとして設計する。

## 表面占有(重なり回避)

- 電柱meshは素のテーパー柱にし、柱表面を占有するもの(昇降ボルト、バンド、広告、端子函)は
  すべてplacement対象にする。占有は、poleごとに高さ区間と角度区間の組を列挙して管理する。
- mesh同士の実行時衝突判定はしない。宣言済み区間の照合のみで解く(placement reserveと同じ考え方)。
- ボルト等を焼き込んだmeshを使う場合は、descriptorにkeep-out zone(側・高さ範囲・角度)を宣言させる。

## span内モデル(玉碍子・端子函)

既存機構を使う。新概念は追加しない。

- 正体はspan上のAttachment + socket + `AttachmentLineInteractionMode::kReplaceWithInternalPath`。
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

merge_readiness.mdのunsupported保留一覧のうち、モデル対応を進めると優先度が上がるもの。

| 項目 | 理由 |
|---|---|
| `UpdateAttachmentTemplate`の構造差分 | socket追加/削除/id変更、mode変更、internal path本数/socket参照/kind変更は、使用中attachmentの意味を変える再読込conflictとする。conflict解決の正本/退役規則が設計されるまで状態変更前に拒否し、構造lifecycleとして別設計にする |
| `UpdateAttachmentTemplate`の幾何差分 | socket位置/方向、internal path local_points/coil値は既存更新経路でkReshapeし、対象spanを再導出する |
| endpoint attachment生成/退役規則 | `CableTemplate.default_endpoint_attachment_template_id`はbackbone pipelineが`AttachmentOrigin::kDefaultEndpoint`のendpoint attachmentだけをreconcileする。user attachmentは保持し、退役spanにuser attachmentがあればregenerateは状態変更前にunsupportedとする |

## レンダラ移行との順序

wasm + three.jsへの移行を予定しているため、モデル作業を2つに分ける。

- エンジン非依存部分(descriptor契約、marker規約、socket接続、replaced_interval、表面占有)は
  レンダラを待たずに進める。
- meshを画面に出す部分(glTF load、instancing、material、LOD)は
  three.js側で初実装する。desktop viewer(raylib)にmesh loadingを実装しない(捨てる投資になる)。
- three.js化自体は、既存出力(VisualCurvePart / renderキャッシュ / placeholder)だけを描く最小縦スライスを
  モデルより先に通す。wasm境界を渡るのはcoreの派生出力のみとし、JS側に判断を持たせない。

## 進め方

1. descriptor契約(marker命名規約・項目)を文書で固定する
2. 電柱1・碍子1・変圧器1の3体だけdescriptorを手書きしてviewerで回す(抽出ツールは書かない)
3. 再読込・override・conflict報告のループを固定してから、bbox/marker抽出を自動化する
4. crossarmとmesh beltを同じrow assemblyとして表示し、円形pole向けradial fitを確認する

importパイプラインの実装から始めない。契約 → 手書き検証 → 自動化の順とする。
