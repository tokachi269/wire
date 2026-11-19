# Road architecture

この文書は`city::road`のarchitectureを定義する。P0-P2テストの通過だけでは設計準拠を意味しない。現行実装を根拠に未定義の意味を補完せず、下記の担当箇所と依存方向を維持する。

共通契約は`../architecture.md`、操作×状態は`operation_semantics.md`、対応範囲は`supported_operations.md`、対象外と将来の範囲は`backlog.md`を参照する。

## Naming

生成系の動詞は`../architecture_naming.md`に従う。roadの派生生成入口は`generate_road`であり、`derive` / `resolve` / `emit` / `validate`を責務の名前として使う。

road固有の名詞(`RoadNode`、`RoadSegment`、`RoadLayoutTemplate`、`RoadLayoutTransition`、`Junction`、`ConnectionGate`、`ApproachKey`、`Marking`)はwireへ揃えない。

## State ownership

### Input

- Straight / Curvedなどのtool mode
- viewport click、drag中handle、preview point
- snap結果の`segment_id + segment_distance_m`または`node_id`
- section、transition、marking、policy操作request

tool modeとpreviewは保存しない。adapterはrequestへ型変換するだけで、接続、transition action、ID、fallbackを決めない。

### Authoritative

- `RoadNode`: ID、確定XY位置、標高。segment endpointの水平位置とendpoint elevationの唯一の正本
- `RoadSegment`: 局所編集単位のID、endpoint node ID、局所`SegmentShape`、section timeline参照
- `RoadCorridor`: 共通道路定義を参照する方向付き`RoadSegment`列。分岐を含まない
- `SegmentShape`: endpointから出るhandle vector、内部knot、内部handle。endpoint座標は持たない
- `SegmentShape.intent`: tool由来の編集意図。Straightはnode移動時のlinear handle再導出とinspectionだけに使い、generation / emitの別経路を作らない
- `RoadLayoutTemplate`と`RoadLayoutTransition`の意味入力
- `LaneBand`: テンプレート内で安定した`LaneId`、所属surface、横方向範囲、segment正本方向に対する進行方向
- `LaneConnection`: `LaneEndpointKey(segment_id, lane_id, endpoint_role)`間の確定した車線接続
- `BoundaryContinuation`: `BoundaryEndpointKey(segment_id, boundary_id, endpoint_role)`間の確定した境界継続
- manual markingの所有元IDとowner-local distance / lateral値
- surface / marking style識別情報は`SurfaceStyleId` / `MarkingStyleId`で保存する
- ユーザーが明示した`NodeConnectionPolicyOverride`
- 新規道路作成時に確定した`RoadSegment.corner_radius_m`
- next ID状態

`RoadSegment`へendpoint座標やendpoint elevationを保存しない。一回で確定したPathは一つのsegmentとして複数spanを持てる。確定済みsegmentの延長は新segmentを作り、既存shapeへspanを追記しない。degree 2 nodeは明示的な確定境界として正当であり、同一道路定義でも自動統合しない。

自動junctionの存在、connection kind、gate、setback、meshは保存しない。corner radiusはsegmentごとの作成時設定として保存する。共有テンプレートにはせず、sidebarの変更も既存道路へ遡及させない。

### 断面カタログの所有

道路製品として提供する断面の一覧、幅・勾配・路面・車線・境界・白線の具体値、表示名、並び順、初期選択はWebが所有する。正本は`web/src/road_templates.ts`。Coreは`RoadLayoutTemplate`型、その検証、ID採番、保存、生成、共有テンプレート編集を所有し、具体的なpreset catalogueは持たない。

各presetのjunction corner radius初期値もWebが所有する。新規道路へ渡された後は`RoadSegment.corner_radius_m`が正本となり、splitは元segmentから、corridor延長は終端segmentから継承する。junctionでは隣接approach pairごとに小さい保存値を丸みの上限として使い、作成順、平均、connection全体の固定fallbackへ畳み込まない。

- 新規workspaceはWebが各presetを`AddRoadLayoutTemplate`へ渡し、返ったIDを保持する。
  Web側のpreset keyはCoreへ渡さず、保存もしない。
- 既存workspaceのLoadは保存済み`layout_templates`をそのまま使う。Web presetを再注入せず、
  保存済みの寸法をWebの最新値へ更新しない。Web presetの定義を変えても既存archiveは変わらない。
- 表示名はWebがそのworkspaceで登録したIDに対してだけ持つ。Loadで入った断面やADD LANEが
  派生させた断面はIDで表示する。表示名とpreset keyは`SavedRoadGraph`へ保存しない。
- ADD LANEが作る変更後断面はCoreが作る。これはWeb presetではない。
- テンプレートIDは互換性判定に使わない。接続可否はendpointの実断面で判定する。

`RoadState`は断面を持たない状態から始まる。存在しないテンプレートIDで道路を作る要求は従来どおり明示的に失敗し、Coreが既定断面を補うことはない。

### Vertical profile

roadは水平線形と縦断を分ける。`SegmentShape`と`Path`はXY上のhorizontal alignmentであり、道路を一つの3D Bezierとして保存しない。MVPのvertical profileはsegment両端の`RoadNode.elevation_m`をalignment distanceで線形補間するconstant gradeである。

endpoint elevationはnodeだけが所有し、segment、section、derived sample、meshへ重複保存しない。

生成側はsegment distanceごとに3D road frameを導出する。frameは`position`、`tangent`、`lateral`、`normal`を持つ。`position.xy`はhorizontal alignment、`position.z`はvertical profileで決まり、`tangent`はhorizontal tangentにlongitudinal gradeを合成した3D方向である。

bankingはまだ持たないため、`lateral`は水平横方向、`normal`はtangentとlateralから決まる基準上方向である。

asphalt、sidewalk、shoulder、median、curb、L-gutter、BoundaryProfile、marking、lane pathは同じroad frameを使う。断面profileの`height_m`とcross slopeはframe上のlocal heightであり、world Zへ直接固定しない。

概念上は`vertical profile elevation + section/profile local height`がsurface位置になる。longitudinal grade、cross slope、将来のbankingは別概念として扱い、cross slopeを縦断勾配の代用にしない。

splitはsplit前vertical profileをそのsegment distanceで評価し、新しいRoadNodeの`elevation_m`へ保存する。branchをsegment途中から作る場合も、split nodeとbranch free endpointのdefault elevationはsource segmentの評価値を継承する。

corridor extensionは既存終端node elevationを新segment startへ継承し、end elevation指定がなければ水平延長として同じ値を使う。Add Laneは断面幅とlane topologyだけを変え、node elevationやsegment vertical profileを変更しない。

XY上で道路が交差しても、それだけではjunctionを作らない。接続はnode共有または明示操作だけで決まる。高さが違う道路を位置近接やXY交差から同一nodeへbindしない。junction / connectionのgateはapproachの3D frameから得る。

現行junction surfaceはresolved gate / perimeterの3D点を使うが、高度な縦断曲線、banking、terrain cut/fill、bridge/tunnel幾何は別課題である。

`RoadNode.elevation_m`はauthoritative archiveへ保存する。evaluated Z sample、3D frame、mesh vertex Z、lane path point Z、marking ZはDerivedであり保存しない。

### Coreに残す具体値と、その理由

断面カタログを外した後もroad Coreは数値を持つ。所有権を次で区別する。

| 値 | 場所 | 分類 | Coreに残す理由 |
|---|---|---|---|
| `builtin_surface_styles` / `builtin_marking_styles` | `common_types.hpp` | 意味語彙 | archiveが保存するのはstyle IDであり、これが路面と線の同一性そのもの。表示上のmaterialはWebが`road_material_key`で解決する |
| 白線幅 0.12 / 0.16 / 0.35 / 0.10 m (`marking_width_m`) | `generation/markings.cpp` | 保留 | style IDから物理幅を導いている。archiveに幅がないため、移すとスキーマ変更が必要。下記「保留」参照 |
| 停止線・横断歩道の寸法 | `generation/markings.cpp` | 保留 | 同上 |
| `distance_epsilon` 1e-9、`kDegenerateStripWidthM` 0.05 | `geometry/geometry.hpp` / `generation/markings.cpp` | アルゴリズム定数 | 数値安定性と退化判定。workspaceごとに変える意味がない |
| `kCurveSamples` 24、`kConnectionCurveSamples` 8、`kJunctionCurveSamples` 6 | `geometry/alignment.hpp` / `geometry/junction.cpp` | アルゴリズム定数 | 派生幾何を決めるため、変更するとlocalityテストのbit一致契約が壊れる。表示品質設定ではない |
| `kMarkingElevationM` 0.025 | `generation/markings.cpp` | アルゴリズム定数 | z-fighting回避。生成結果の一部として保存互換に関わる |
接続segmentに一律の最小長は置かない。`resolve_connections`が断面幅・角度・接続種別から必要setbackを導出し、segment長を超えるapproachまたは両端gateが重なるsegmentだけを`kNotImplemented`として拒否する。
| `kSnapDistancePointToleranceM` 0.6 | `operations/create.cpp` | 製品規則 | 明示snap距離と入力endpointの一致許容。UIのsnap半径ではなく、Coreが受け取った2値の整合検査 |
| 保存された断面・車線幅・境界寸法 | `SavedRoadGraph` | 保存された具体値 | workspaceを再現するために必要。Webが作った値でも確定後はCoreが保存する |

#### 保留: style IDから導く物理寸法

`marking_width_m`はstyle IDから白線幅を返す。幅はarchiveに保存されていないため、Webへ移すと過去のworkspaceが同じ線幅で再現できない。選択肢は次の3つで、いずれもarchiveスキーマ変更かrequest契約変更を伴うため、今回は実施しない。

- `AutoMarkingPolicy`へ幅を保存する (スキーマ変更。保存互換を保証する段階ならversioningが必要)
- 新規workspace作成時にWebがstyle表をCoreへ登録する (断面テンプレートと同じ方式、スキーマ変更必要)
- Core固定のまま、日本の道路標示規格に由来する値として文書化する (現状)

### Derived

- topology indexとnode degree
- `CanonicalAlignment`: `node_a.position + SegmentShape + node_b.position`から作るcubic Bezier span列
- arc-length table、distance、tangent、frame、bounds
- `ApproachKey`: `node_id + segment_id + endpoint_role`の完全一致でsegment端を識別するderived識別情報
- `DerivedSegment`: CanonicalAlignment、semantic / surface distance、`SectionEvaluation`
- `ResolvedConnection`: PassThrough / Corner / Junction / Unsupported、approach order、corner policy、適用override、reason
- `ResolvedApproach`: endpoint section ID、auto値、override後のresolved値、gate distance、`ConnectionGate`
- `DerivedMarking`: 所有元、境界ID、role、style、幅、world points / polygon
- `RenderStyleRef`付きsurface、curb、sidewalk、marking、terrain mask、mesh、viewer payload

StraightとCurvedは同じ`SegmentShape`とcubic Bezier APIを使う。Straight requestは操作planで、`P1 = P0 + (P3 - P0) / 3`、`P2 = P0 + 2 * (P3 - P0) / 3`のlinear cubicへ正規化する。

`SegmentShape.intent`はStraightとして保存するが、sampling、connection、emitはBezierを共通に読む。後から直線かどうかを制御点の近接比較で判定して編集意味を決めない。Bezierの接線はcubic derivativeから求め、endpointでは隣接control handleの方向をそのまま使う。

G1なspan境界は同じ接線を持つ。明示的な非G1境界は前後接線を平均して隠さず、両側のoffset lineが交わるmiter frameとしてsurface、lane、markingへ共通供給する。

角度を持つStraight segment同士の丸みは`RoadSegment.shape`へ混入しない。degree 2の屈曲nodeでは、各segmentをgate distanceまで直線として使い、その間のcorner curveを`ResolvedConnection.connection_geometry`として導出する。

corner生成は隣接segmentの制御点を書き換えない。

一回で確定したPathは、複数spanを内部に持つ一つのRoadSegmentへ正規化する。確定済みsegmentをcorridor終端から延長した場合は、既存segmentを書き換えず、新しいRoadSegmentをcorridor末尾へ追加する。自動curve previewだけが既存corridor終端の解析接線を新spanの開始handleへ反映する。

操作は受け取ったcontrol pointを再整形せず、明示編集されたhandleをそのまま正本へ保存する。

## Operations

全public操作は次の一経路だけを使う。

```text
Request
  -> Preflight(current authoritative, request)
  -> OperationPlan
  -> Apply(plan, trial authoritative + trial next_id)
  -> generate_road(trial authoritative)
  -> invariant
  -> Commit(authoritative + derived + next_id)
```

検証、unsupported、generate、不変条件のどこで失敗しても、authoritative serialization bytes、derived deterministic hash、next ID、inspection / query結果は操作前と完全一致する。

public操作から別public操作を呼ばない。ApplyはplanにないID確保、接続判断、fallbackを行わない。splitは`target_segment_id + segment_distance_m`を使い、target識別情報やdistanceを座標から再推測しない。

degree 1のcorridor終端への同一道路延長は`ExtendCorridorFromEnd`として新しいnodeとsegmentを同じplanで追加する。既存segment、既存node位置、既存handleをreplaceしない。corridor先頭への延長は、既存corridor distance参照の移行規則がないためunsupportedとする。

segment間の滑らかさは隣接する二つのsegmentだけを入力とするderived connection幾何が担当する。

`EditSegmentPath`相当の外部入力はcoreで次へ分ける。

- `EditSegmentShape`: handle、internal knotだけを変更し、node位置を変更しない
- `MoveNode`: `RoadNode.position`を変更する。Straight intentを持つincident segmentは新しい両端node位置からlinear handleを再導出し、Curve intentのhandleはそのsegment-local shapeとして維持する

## Generate

```text
authoritative graph
  -> edit plan
  -> trial
  -> generate_road
       derive_segments      canonical alignment / semantic distances / section evaluation
       resolve_connections  node decision / auto values / user override / gate / connection geometry
       derive_markings      boundary lines / manual markings / junction markings
       emit_geometry        surface, connection, junction and marking meshes
  -> validate
  -> commit
```

`generate_road(graph)`は純粋関数であり、`RoadState`へ途中結果を書かない。完成した`DerivedRoad`だけを返し、失敗時は部分的な派生を公開しない。工程を細分するかどうかは実装都合であり、契約は担当箇所と依存方向だけを固定する。

工程数は削減目標ではない。各工程は入力・出力・利用側・失敗分類を説明できることだけを求める。利用側のない工程と、削除した機能のためだけの分岐は置かない。

| 工程 | 入力 | 出力 | 利用側 |
|---|---|---|---|
| `derive_node_incidence` | graph nodes / segments | node別のincident endpoint | `resolve_connections` |
| `derive_segment_shapes` | node位置 + `SegmentShape` | `CanonicalAlignment`、長さ | `derive_segment_sections`、`resolve_connections`、viewer |
| `resolve_connections` | incidence、alignment、policy override、approach override | `ResolvedConnection`(種別・approach順・auto値・resolved値・gate) | `derive_segment_sections`、`resolve_connection_geometry`、inspection |
| `derive_segment_sections` | alignment、テンプレート、transition、gate distance | semantic / surface distance、`SectionEvaluation` | `resolve_connection_geometry`、`derive_markings`、`emit_geometry` |
| `derive_topology_paths` | lane connection、境界continuation | junction内のlane / 境界経路 | `derive_segment_lane_paths`、`derive_markings` |
| `derive_segment_lane_paths` | section評価、topology経路 | segment内のlane経路 | scene payload、`derive_markings` |
| `resolve_connection_geometry` | `ResolvedConnection`、gate断面 | corner / junction surface | `emit_geometry` |
| `derive_markings` | section境界、marking policy、junction override | `DerivedMarking` | `emit_geometry` |
| `emit_geometry` | 解決済み幾何のみ | mesh、terrain mask | viewer |

`SavedRoadGraph`の13フィールドはすべて生成側に読み手がある。保留機能の`manual_lines` / `manual_areas`は標準UIと公開APIから外したが、既存workspaceを開くために保存と生成の読み取りを維持する。`lane_connections` / `boundary_continuations`はjunction生成が内部で使う。

失敗分類は全工程共通で、入力不正は`kInvalidInput`、現在の対応範囲外は`kNotImplemented`、正しい正本から派生が欠ける場合は`kInternalError`とする。保留機能のデータを持たない通常道路が、その機能の処理を理由に失敗してはならない。

### derive_segments

`CanonicalAlignment`、semantic distance、surface distance、`SectionEvaluation`を`DerivedSegment`が所有する。sampling planやsection tableを独立した長寿命readモデルとして公開しない。

単独segmentとdegree 1終端は、connection側の解決を要求せずにこの段階だけで面とmarkingに必要な形状を持つ。

### resolve_connections

degree 2以上の接続nodeについて、接続種別、approach順、auto setback、ユーザーoverride、gate frame、corner / junction幾何を一度に決める。autoとresolvedを別工程・別tableにしない。`ResolvedApproach`がauto値とresolved値を両方持ち、inspectionはそこから読む。

degree 0/1のnodeにはconnection entity、gate、layoutを作らない。

接続前にpair、mapping、gateを正本へ用意する構造は持たない。必要な接続はgraphから導出する。一般的な斜交接続を固定の最小・最大角度だけで拒否しない。junctionのauto setbackは各approachの実断面外半幅、隣接approachとの交差角、共通corner radiusの接線長から個別に導出し、必要距離をsegment内へ確保できない場合や有限な境界幾何を生成できない場合だけを`unsupported`とする。

Junction幾何はテンプレートIDや表示styleで断面を分類しない。gateごとに左右のroad outer、curb、shoulder、carriageway edgeを境界roleとstrip adjacencyから解決し、存在する帯だけを生成する。片側だけcurbがある断面も同じ解決器を通し、候補が複数で意味が一意にならない側だけをunsupportedとする。

異なる断面sideを接続するときは`carriageway edge`と`road outer`を両端の固定対応とし、片側だけにあるshoulder等の中間faceはcarriageway側の幅0から展開する。gutterの輪郭点をroad outerへ読み替えず、各gateに存在するgutter上面幅とroad outer位置を維持する。

carriagewayとshoulderは同じAsphalt surface regionとしてcurb innerまで一度だけemitし、carriageway edgeは、shoulderを示す意味上のmarkingとして保持する。surfaceの分割には使わない。

### derive_markings

線は所有元segmentと境界IDで識別する。断面遷移では同じ境界IDが続く限り線を継続し、現れた位置で開始、消えた位置で終了する。隣接車線が退化している区間には線を出さない。配列indexや位置近接で別境界へつなぎ替えない。

CenterLineとLaneSeparatorは交差点内でdefault gate終了とし、`JunctionMarkingOverride`が明示したsource / targetだけが接続線を作る。CarriagewayEdgeは隣接sideのrole/styleが一致する場合、解決済みjunction perimeter curveへ自動接続する。

片側だけに存在する線はgate終了とし、明示`BoundaryContinuation`と同じendpoint pairへ自動線を重複生成しない。最も近い線や角度差最小の線を自動選択しない。

### emit_geometry

解決済み幾何だけを受け取り、頂点・index・normal・UV・mesh groupと有限性検査を行う。接続種別、setback、override適用、境界対応、marking範囲、所有元、style、線幅を決めない。

surfaceの表向きは幾何生成処理が決める。section順と元segmentのface normalから`ResolvedSurfaceStrip.winding`を導出し、junctionの上面regionは幾何生成処理がperimeter順を確定する。`emit_geometry`は三角形のnormalをworld-upと比較してindexを反転しない。

これにより、水平上面、横断勾配、gutterの立面、将来の高架下面を同じemit規則で扱い、立面の微小なZ成分を上面と誤認しない。

generated `Mesh`はposition、index、normal、uv0、material groupを持つ。現行emitはstyleごとにmeshを分けるが、各meshにもmaterial groupを持たせ、将来mesh統合やreference mesh由来のgroupへ移行できるようにする。

normalは完成したtriangleから積算して派生し、normalの都合で幾何やUV topologyを変えない。

roadのUVはnetwork/corridorの累積距離を使わない。上流segmentの長さを次segmentのUV originへ渡す依存鎖を作らず、branchやjunctionの先までUV continuityのために探索しない。Coreが決めるのは幾何をどの2次元座標へparameterizeするかまでで、texture file、repeat scale、normal / roughness / overlayの使い方はviewer material側が決める。

asphalt、sidewalk、median等の面的surfaceは`MeshUvMapping::kWorld`を使い、UVを0..1へ道路幅・道路長で正規化しない。segment surfaceではUをそのsegment上のpatch-local距離m、Vを断面横方向mとして出す。

transitionでDerived上のpatchが分かれても、各patch内のU/Vはmeter単位であり、幅が6mから12mへ変わっただけで同じV spanへ正規化しない。junction interiorのように単一の道路方向がない面はworld XYのmeter座標を使う。

方向を持つsweep surface、つまりgutter、curb、将来のparapet、高架側面、retaining wall等は`MeshUvMapping::kPatchQuantized`を使う。設定は`PatchUvSettings`で表し、現行defaultは`reference_length_m=64.0`、`seam_divisions=4`である。

safe seamの長さは`reference_length_m / seam_divisions`で、patch lengthを最も近いsafe seam数へ丸める。短いpatchでも最低1 intervalを割り当て、Uは1を超えてよい。現在のprocedural sectionではUをpatch長手方向、Vを評価済み断面profileのcontour累積距離から生成する。

L-gutterのように同じlateralで高さだけが違う立面も、Vの幅を持つ。これは将来のauthored profile/reference mesh UVを保存スキーマなしで置き換えられる派生defaultである。

一つのsweep幾何patchは一つのlongitudinal U mappingを共有する。同じsample indexに属するtop / wall / channel等のprofile faceは同じUを持ち、faceごとにarc lengthやsafe seam量子化をやり直さない。

sweepを含むsegment境界surfaceでは`surface_segment_distances_m`のsegment-local距離を共通U基準に使う。corner / junction境界stripでは、同じ点数のresolved strip群から局所的な代表距離列を作り、各stripへ共有する。

これはconnectionまたはjunction内のresolved幾何だけを見る処理で、corridorやnetworkを辿らない。

`RoadLayoutTransition`を持つsegmentでは、segment start、transition start、transition completion、segment endをDerived UV patch境界にする。sweep patchは各patchでU=0から始まり、前patchのU_endを次patchへ引き継がない。

planar surfaceはpatchが分かれてもmeter単位のscaleを維持する。Add Laneも同じsection transitionなので、transition前、taper、completion後のDerived patchへ分かれる。UV patch分割はDerived/emit上の分割であり、RoadSegmentやcorridorをsplitしない。

marking line/ribbonはgutter用のsafe seam方式へ合わせない。Uはribbon長手方向の局所距離m、Vはribbon幅方向mで、点線phase同期やshader dash生成はviewer/material側の将来課題とする。面markingとjunction interiorはworld XYのmeter座標を使い、junction/corridorの前後関係を読まない。

junction interiorは解決済みperimeterを境界頂点として保持し、追加した中心点からfan状に三角形を張る。corner subdivision由来のperimeter頂点をinterior mesh側でも使うため、corner tessellationとinterior triangulationは境界で一致する。

UVのためにroad topologyを追加しない。

## Style ownership

| 層 | 契約 |
|---|---|
| authoritative / request | `SurfaceStyleId` / `MarkingStyleId`を保存・入力する。自由文字列を識別情報に使わない |
| built-in catalog | road内の最小built-in styleだけを`common_types.hpp`で定義する |
| derived / emit | `RenderStyleRef(domain, value)`だけを運ぶ。表示名文字列へ戻さない |
| viewer adapter | `RenderStyleRef`をviewer material key文字列へ変換する唯一の境界。未知IDはfallbackしない |

manual markingのstyle IDは保存だけでなくemit入力へ渡し、生成meshの`RenderStyleRef`へ反映する。BoundaryProfileには個別styleフィールドを持たせない。curb等の描画styleはBoundaryRoleからsection評価で導出する。

## Identity

- 接続と所有はIDで決める。同位置、eps内、配列index、頂点番号、名前から識別情報を推測しない。
- segment端の識別情報は`ApproachKey(node_id, segment_id, endpoint_role)`の完全一致だけで決める。
  node移動、shape編集、approach角度順、segment格納順で変化しない。
- 境界 / element対応はstable ID、role、explicit transition mappingで決める。
- lane endpointは`LaneEndpointKey`、境界endpointは`BoundaryEndpointKey`の完全一致だけで識別する。
  lane/boundaryの横方向順序、world位置、距離、sample indexから接続先を推測しない。
- `almost_same`やdistance-epsはBezier連続性、ゼロ長、平行性など数値幾何だけに使える。
- 同位置に複数nodeが存在しても別識別情報である。

## Lane topology ownership

`LaneConnection`と`BoundaryContinuation`はauthoritativeである。ユーザー操作または操作で確定したsource/target endpointを保存し、generation時に幾何の近さから作り直さない。`LaneConnectionKind`と`BoundaryContinuationKind`は生成とinspectionの意味を補助するが、識別情報の代わりにはしない。

高速道路、出口、合流専用の保存型は作らず、一般道の車線追加、分岐、合流、交差点movementも同じlane/boundary接続で表す。segment splitでは、元segment start endpointに属するtopologyは元segmentへ残し、元segment end endpointに属するtopologyはsplit後のend側segmentへIDでremapする。

segment deleteでは、削除segmentをsourceまたはtargetに持つ`LaneConnection`と`BoundaryContinuation`を同じOperationPlanで削除する。orphan topologyをgenerateや検証へ持ち越さない。

車線中心と道路境界は別の正本である。lane接続からshoulderや道路外端の継続を暗黙導出せず、必要な境界は`BoundaryContinuation`で明示する。lane/boundaryのworld pathはこれらの正本から派生し、保存しない。

外側lane追加は、追加側と反対にある直近の断面境界を`RoadLayoutTransition`の固定基準とする。curbやmedian edgeのように幅と高さを持つ境界も、その境界の断面順で左側のsampleを固定する。反対側に境界がない単一strip断面では断面外端を固定する。

既存laneの内側境界を同じIDの断面sampleから一度だけ解決し、その外側にだけ新laneを展開する。全断面の中心合わせやworld位置の近接再bindは行わない。新laneの幅はtransition中に0から指定幅へ単調に増加し、既存shoulder幅は維持したまま外側へ移動する。

複数laneが同じcarriageway stripを共有していても、lane範囲を含む断面順から外側laneを解決する。laneとstripの一対一対応やstrip全幅占有をADD LANEの前提にしない。固定境界は選択側の内側方向にある最初の安定境界を使う。

ADD LANEの変化開始と3車線完成位置は同一corridor上の`SegmentPosition`として操作へ渡す。Coreが各位置をcorridor累積距離へ解決し、開始から完成への向きを操作方向にする。taperは1 segment内だけに作り、segment境界を跨ぐtaperのためにpartialテンプレートを多数生成しない。

完成断面は操作方向のcorridor終端まで維持する。ユーザーへ追加の終端入力を要求せず、開始・完成位置を作るためだけにもsegmentを分割せず、追加の終端入力も持たない。物理距離やcorridor-globalなLaneAddition entityを保存しない。境界anchorで決まった横方向原点は操作方向のcorridor終端までの後続segmentへ派生伝播し、追加前から存在するlaneを再中心化しない。

通常幅以降のsegmentはtransitionを所有しないため、後続segmentは通常の完成断面として扱う。このanchor伝播は現行の非局所派生契約であり、正本ではない。新しいtransition意味を追加する前に、操作が後続segmentの完成テンプレートoffsetを明示確定できるかを優先して検討する。

接続可否はendpointの実断面から判定する。テンプレートIDだけでは決めない。同じlane識別情報とsemantic sideを一意に対応できれば、side profileが異なるdegree 2接続も生成する。laneまたは境界対応が曖昧な場合だけnode手前のsection transitionか明示topologyを要求する。

junctionでは各approachの断面を独立して解決し、未指定laneはgateで終了する。AddLaneは3/4-way junctionの接続先を横順やlane数から推測しない。1件のLaneConnectionを全断面の解決済み判定には使わない。道路中心や最寄りlaneから接続先を補わない。

新segmentをtargetにする分岐と、新segmentをsourceにする合流は同じ操作契約を使う。`resolve_connections`はLaneConnectionからapproach全体のauto lateral shiftを導出しない。laneごとに必要な横方向差は`DerivedLanePath`で処理し、approach lateral shiftはmanual overrideだけが変更できる。

Junction gateのauto setbackは、角度順で両隣にあるapproachだけから決める。各pairのcornerへ向く実断面side lineを`EndpointRole`とtangentから組み立て、その交点を両approachへ射影する。反対側や非隣接approachの幅を最大値として流用せず、左右非対称断面も一方のreachだけを使う近似式へ落とさない。

corner radiusは隣接pairの保存値の小さい方を希望上限として解決し、connection全体へaverage・作成順・固定fallbackで畳み込まない。0mはgateをradius分だけ押し広げないという意味であり、road outerのsharp cornerを許す。

gate確定後の各断面境界は、その境界自身の端点とapproach tangentの交点からG1 controlを導出する。したがってroad outerに接線余地がなくても、内側のcarriageway/sidewalk境界に余地があれば曲線を維持する。全境界をconnection共通controlで直線化または丸めない。

鋭角ではradiusが0でもside line交点自体が遠方になり得るため、90度T字のmixed-radius問題とは分けて扱う。`DerivedLanePath`と`DerivedBoundaryPath`は保存せず、authoritative connection IDから毎回生成する。

Splitの両側境界から`DerivedSeparationArea`を作るが、lane connectionから境界接続を逆算しない。

laneと境界の多重度は同じ意味に揃える。Continuationは一対一、Splitは同一sourceから複数target、Mergeは複数sourceから同一targetを許可する。同一source-target pairは常に重複としてrejectする。対向道路でも接続単位は`LaneEndpointKey`であり、道路全体の方向分類からbranch対象を広げない。

選択方向と反対のlaneおよびmedian境界は、明示された接続以外では既存mainline上に維持する。

交差点内の進行は`LaneConnectionKind::kJunctionMovement`として同じauthoritative topologyへ保存するが、幾何は`resolve_junction_movement_path`がjunction gate間だけに生成する。

merge/splitの長い接続区間resolverとは分け、共通のG1 cubic primitiveだけを共有する。source/targetは常に`LaneEndpointKey`で明示し、turn roleや角度から推測しない。

segment間の接続白線は`DerivedBoundaryPath`を唯一の幾何入力とする。marking生成は境界endpointやlane centerから別Bezierを作らない。Split区間はsource境界のrole/style、Merge区間はtarget境界のrole/styleを使い、Continuationでrole/styleが変わる場合はunsupportedとする。

接続線はjunction ownershipでgate間だけを所有する。

## Approach override lifecycle

authoritativeに保存するのは`ApproachGeometryOverride`のmanual setback / manual lateral shiftだけである。全フィールドがAutoの空overrideは保存しない。単独segmentとdegree 1 endpointにはconnection/overrideを要求しない。

degree 2のCorner/PassThrough、degree 3/4のJunctionでだけ`ResolvedConnection`を導出する。

- extension: 既存segmentの`ApproachKey`は不変で、新segment endpointに新しい`ApproachKey`を作る。
  既存manualフィールドは維持し、Autoフィールドは最新auto値へ追従する。
- node move / shape edit: `ApproachKey`は維持され、`ResolvedConnection`だけを再導出する。
- segment split: 元segment start側overrideは元segmentに残る。元segment end側overrideは新しい外側segment endへIDでmappingする。内部nodeへ複製しない。
- segment delete / node delete: 該当segment/nodeのoverrideは同じOperationPlanで削除し、orphanを残さない。
- connection kind変更: 同じ`ApproachKey`かつフィールドが有効なら維持する。layout対象外になった手動overrideはgenerateでunsupportedになり、無視せず明示的に結果を返す。

## Persistence target

Road archiveは現行スキーマだけを読み書きする。保存対象はauthoritative状態とnext IDだけとする。CanonicalAlignment、connection、gate、section evaluation、junction幾何、mesh、mask、auto markingは保存しない。

resolved connection、derived marking、mesh、maskも保存しない。ApproachGeometryOverrideはmanualフィールドがある場合だけ保存する。`LaneConnection`と`BoundaryContinuation`はID順に保存し、lane/boundary path幾何は保存しない。

Web/WASM境界はCoreが返す軽量なlane world pathをhoverと選択表示にだけ使う。編集requestは選択済みの`LaneEndpointKey` / `BoundaryEndpointKey`を明示して渡す。AddLaneはeditorで選択した3つの道路位置をsegment IDとsegment-local `t`を持つ`SegmentPosition`として渡す。

Coreは3点を同じcorridor上の累積距離へ解決し、既存nodeの選択を要求しない。Webはlane接続curve、境界対応、接続相手を再計算しない。標準道路描画のpointer moveは入力Bezierと断面幅からローカルguideだけを表示し、Core操作を実行しない。

ClickまたはEnterの明示commitで初めてCore generationと検証を行う。control-point編集やlane編集の明示previewは試行用の状態で対応するCore操作を実行できる。preview結果は確定結果と同じresolved mesh経路を使い、preview前後でauthoritative状態を変更しない。

AddLaneの固定基準はCoreがlaneの方向・追加側・断面順から決める。Webはrole、配列順、数値IDから候補を推測しない。

形式は一行一フィールドのnamed indexed `key=value`で、カンマ位置に意味を持たせない。saveフィールド順は固定し、top-level entityはID順にcanonical serializeする。順序に意味がある`SegmentShape.internal_knots`やPath spanは保存順を維持する。

doubleはload後に同じbinary doubleへ戻る表現で保存する。

バージョンフィールドとmigration分岐は持たない。loadは現行スキーマとしてparse、フィールド構造検証、型変換、`ValidateAuthoritativeGraph`、`generate_road`、不変条件検査を通過した場合だけ新状態を返す。duplicate key、missing key、unknown key、count不一致、enum範囲外、非有限値、重複ID、欠損参照をrejectする。

## Preflight and validation

操作preflightはrequestの全フィールドについて、ID存在、有限性、enum範囲、style ID、正であるべき寸法を検査する。正しい入力だが現在扱えない接続形状・transition付きsplit・prepend distance移動は`kUnsupported`に残す。

valid authoritativeから派生が欠ける、emitへ不整合なresolved幾何が渡る等は`kInternal`とする。`ValidateAuthoritativeGraph`はloadと操作trial後に共通利用し、保存正本の完全性だけを見る。generateでしか判断できない幾何対応可否は混ぜない。

## Segment locality

`RoadSegment`は局所追加、局所削除、split、shape編集、局所所有物の単位である。長い道路全体や一回の描画セッションを表さない。shape編集の意味上の影響範囲は対象segment、その両端connection、対象segmentを参照する局所所有物に限定する。

## RoadCorridor

`RoadCorridor`は`RoadCorridorId`、終端延長時の既定断面である`layout_template_id`、順序付き`DirectedSegmentRef`を正本として持つ。実体のない別`RoadDefinitionId`は作らない。`reversed=false`はsegment startからend、`true`はendからstartへ進む。

corridor内のsegmentは端点IDで連続し、重複せず、他corridorへ重複所属しない。位置近接は連続性判定に使わない。断面変更を含むcorridorではsegmentごとの`layout_template`が異なってよい。テンプレートIDの一致を接続可否やcorridor所属の判定に使わない。

`layout_template_id`は終端延長時に使う既定断面を表す。corridor全体の断面を表す値にはしない。generationはsegmentごとの`RoadSegment.layout_template`と`RoadLayoutTransition`から断面を評価し、corridorの`layout_template_id`で既存segmentを再解釈しない。

corridor distanceは先頭から単調増加する。内部segment境界は後続segmentのlocal 0、corridor終端は末尾segmentのlocal lengthへ解決する。reversed segmentでは`local = length - corridor-local`とする。長さ、累積distance、解決結果は派生であり保存しない。

分岐は既存corridorへ枝を混入せず、新しい`RoadCorridor`を作る。共有nodeは許可するが、本線corridorのsegment列、方向、始点、distance、周期配置位相は変更しない。

## Corridor editing

`SplitSegmentAtDistance`はDe Casteljau分割を使い、元segment IDをstart側、新IDをend側へ割り当てる。corridor内では元参照を二つの向き付き参照へ置換し、reversed参照ではcorridor方向に沿う順序へ反転する。

splitはsegment shape、corner radius、endpoint-owned approach override、manual marking、auto marking override、junction marking override、LaneConnection、BoundaryContinuationの移行を同じOperationPlanに載せる。

各操作が個別にendpoint近接で再bindしてはならない。split前後でcorridor全長とworld位置を維持する。

通常削除はsegment単位で行う。corridorが分断される場合、始点側が元corridor IDを維持し、終点側へ新IDを割り当てる。空corridorは保存しない。segment-local markingはsegment distanceでstart/end側へ明示移行し、境界を跨ぐmanual lineは現段階ではunsupportedとする。

approach overrideとjunction marking overrideはendpoint ID対応で移行し、一意に対応しないものはunsupportedとする。通常のViewer削除からこの操作を呼ばない。

`RoadSegment`はユーザーが一つとして選択・編集・削除する確定区間である。一回で確定したPathは複数Bezier spanを内部に持ってよい。span、sample、mesh、marking runの境界をRoadSegment識別情報へ昇格しない。確定済みsegmentを後から延長する操作は新しいRoadSegmentを追加する。

Viewerの通常削除はhover hitが持つ`RoadSegmentId`のsegment全体をハイライトし、同じIDを一回のクリックで`DeleteSegment`へ渡す。Coreは選択segmentだけを削除し、結果として必要なcorridor分断を決定論的に行う。通常削除はrange境界、split node、replacement segmentを生成しない。

道路沿いへ街路樹・標識・建物を並べるための参照型と周期配置policyは、利用側を実装する時点でその利用側の要件から設計する。先行して型だけを置かない。

## Section axes

断面は物理`RoadLayoutStrip`、利用上の`StripFunction`、表示`SurfaceStyleId`、`LaneBand`によるlane割当を別軸として扱う。styleからfunctionを推測しない。`Shoulder`は独立した水平stripを表す。

curb幅や段差の代わりには使わない。車道外側線はcarriageway利用領域とshoulderのsemantic境界へ置き、shoulderがない場合だけ外側境界を使う。

`LaneBand`は車両へ割り当てるsemanticな横方向区間であり、幅は`lateral_end_m - lateral_start_m`、中心はその中点である。markingはsemantic境界上の表示であり、線幅やplacementを変えてもlane幅・lane中心は変わらない。

1つのcarriageway stripへ複数の`LaneBand`を割り当ててよく、lane 1本を物理strip 1本へ対応させない。

断面遷移では`LaneId`だけを継続識別情報とする。同じIDは横方向区間を補間し、targetだけにあるIDは出生、sourceだけにあるIDは消滅として幅0のedgeから、またはedgeへ補間する。出生・消滅edgeは同じIDで継続する隣接laneのedgeかsurface strip外端から一意に解ける場合だけ採用し、配列index・中心位置・近接では推測しない。

同じIDのsurface stripまたは進行方向が変わる遷移、およびcollapse edgeが一意でない遷移はunsupportedとする。物理外形の変化とlane数の変化は独立であり、固定外形内の再配分と外側拡幅の両方を同じ補間で扱う。

現行スキーマは局所segment、corridor、directed ref、section strip、lane allocation、lane connection、境界continuation、layoutのalignment offset、境界profile、segment corner radius、RoadNode elevationをnamedフィールドで保存する。

欠落フィールドやバージョンフィールドを含むarchiveは現行スキーマではないためrejectする。保存互換を保証する段階になった時点でversioningとmigrationを改めて導入する。corridor長、累積distance、connection幾何、evaluated elevation、3D frameは派生なので保存しない。

## Boundary profile

`BoundaryProfile`は`ProfilePoint`(lateral, height)の順序付きcontourと、点間の面ごとの`SurfaceStyleId`を持つ。contourは断面と同じ向き(左→右)で単調非減少、`segment_styles.size() + 1 == contour.size()`。

車線境界線は1点0面、縁石は2点1面、L字溝は上面・垂直面・溝底・路側lipの5点4面。

layout widthを持つのはstripだけで、境界は0を消費する。 contourは両隣のstripの内側へ食い込む。歩道2.0mにL字溝の上面0.2mを置いても歩道幅は2.0mのままで、見える歩行面が1.8mになる。L字溝の寸法を変えてもlayout width、alignment offset、車線位置は動かない。

contourのlateral 0がedge datumであり、端部構造ではそこへ道路側の垂直面を置く。高さはdatum相対で、contour先頭点が直前stripの走行高さを引き継ぐ。stripのcross slopeは自分の面が実際に届く範囲へかかるので、profileとの間に段差も隙間も生じない。profile自体は路面勾配で回転しない。

左右の向きは断面座標のままで、Coreは反転しない。左右で違う端部構造を置くこともでき、どちらも同じ型で表す。埋設部・基礎の下面、法線・UV、蓋やgratingのinstanceは対象外。

## Layout alignment origin

`RoadLayoutTemplate`は`alignment_offset_from_left_m`を持つ。segmentの進行方向へ断面を見たときの左外端からRoadSegment alignmentまでの横方向距離であり、`0 <= offset <= total layout width`を満たす。

断面内の位置はすべて左外端から積み上げ、alignmentを0とする横座標で報告する。alignmentが断面の中央にあるとは限らない。この値は総幅から毎回推定せず、境界roleからもmesh bboxからも推測せず、交差点で外端2点の中点から再計算もしない。交差点はsegment側で確定したframeをconnection gate経由で受け取り、その位置に対するlateral sampleとして断面を置く。

gate上の断面外端が偶然どこにあっても、それがalignmentの定義になることはない。

片側だけ幅が増えても、反対側とalignmentは動かない。Add Laneは追加した側にだけ幅を足し、左へ足したときはoffsetへ同じ幅を加算する。任意のstrip幅を変えるテンプレート編集では、Coreはどの幅を固定すべきか推測しない。offsetは完成したテンプレートの一部として明示的に渡される。

遷移中は幅と同じ距離位置でoffsetも評価する。遷移のanchorは、その上で「どの境界を固定するか」を明示指定する既存の仕組みとして残る。

alignment originはedge datumではない。alignment originはRoadSegmentのpathが断面のどこを通るか、edge datumは将来の端部構造における道路側垂直面を指す。実装があるのはalignment originだけで、edge datumと端部構造profileは未実装である。

## Edge datum と envelope

L字溝のように幅と立体断面を持つ端部構造をいつか置くために、「道路幅がどこまでを意味するか」と「meshがどこまで張り出すか」を先に決める。この契約は未実装である。 現行実装との衝突は`backlog.md`の「端部構造」に一覧がある。

### edge datum

端部構造の道路側を向いた垂直面の、道路側表面を横方向の基準面とする。これをedge datumと呼ぶ。壁厚の中心でも外側面でもない。

- 道路側の意味上の幅はedge datumまで
- 歩道幅はedge datumから外側
- 端部構造の下面と溝部分は道路側surfaceの一部
- 上面は歩道側surfaceの一部
- 上面幅は歩道幅を消費する。歩道 2.0mに上面 0.2mを置いたら残りの歩道面は 1.8mで、合計は 2.0mのまま

端部構造を足したことで道路幅や歩道幅を単純加算で広げない。溝を道路幅の外へ後付けしない。

### 三つの envelope

| envelope | 意味 | 用途 |
|---|---|---|
| layout envelope | 道路幅・歩道幅を測る意味上の外端。edge datumとlayout datumが属する | 車線、白線、幅の検証 |
| visible envelope | 上面・縁石など目に見えるmeshの最外端 | 電柱・標識など地上物の離隔 |
| structural envelope | 地中の基礎や下部張り出しを含む構造上の最外端 | 建物壁・敷地境界・埋設物との離隔 |

三者は一致しなくてよい。歩道がある場合は普通layout envelopeが最も外側になり、歩道がない場合はvisible envelopeがlayout envelopeの外へ出る。

### 歩道がない場合

歩道がなくてもedge datumを内側へ自動移動させない。

- 道路のlayout上の幅はedge datumまで
- 上面はlayout幅の外へ張り出してよい
- 張り出した範囲はvisible envelopeに含める

幅制限のために外へ出せない場合、mesh生成時に内側へ寄せたり車道幅を縮めたりしない。上位が「より小さい端部構造を使う」「道路幅を変える」「端部構造を使わない」「要求をunsupportedとして拒否する」のいずれかを明示的に選ぶ。敷地制限の自動解決は行わない。

### 交差点と接続部

segment、corner、junctionでedge datumの意味を変えない。segment上のedge datumをconnection gate経由でjunctionのdatum curveへ連続させ、その両側に端部構造のsurfaceを派生させる。端部構造の形に合わせてdatumを局所的に内側や外側へ動かさない。

交差点で端部構造のjoinを実装しない範囲は、生成前に接続形状ごと明示的にunsupportedとして拒否する。segment meshだけ作ってjunctionで端部構造を消す、別の境界形状へfallbackする、といった処理はしない。

### 将来の object 配置基準

電柱・壁・金網・標識を置くとき、道路端を生成meshのbounding boxから推測してはならない。 次を区別する。

| 参照線 | 用途 |
|---|---|
| edge datum | 歩道の開始位置 |
| layout datum / layout envelope | 車線、白線 |
| road-side visible surface edge | 路面の見た目の端 |
| visible outer edge | 電柱の離隔 |
| structural outer edge | 建物壁・敷地境界の離隔 |
| profile固有socket | 金網、蓋 |

これらは設計上の区別であり、利用側ができるまで公開APIにはしない。

### 採用しなかった代替案

| 案 | 却下理由 |
|---|---|
| 端部構造を道路幅の外へ単純加算する | 歩道 2.0mに上面 0.2mを足すと 2.2mになる。設定幅と実際の幅が一致しなくなる |
| 歩道なしのときだけdatumを内側へ移す | datumの意味が断面構成で変わる。歩道の有無で車道幅が動く |
| meshのbboxを道路端として使う | 派生結果から意味を逆算することになる。基礎や張り出しを足すたびに「道路端」が動く |
| 交差点でdatumを局所補正する | segmentとjunctionで幅の意味が食い違う。接続部だけずれる |
| width / height一組でL字溝を斜面として近似する | 溝底・垂直面・上面を区別できない。hard edgeも蓋の受け部も表現できない |

テストはproductionと同じ`ValidateGraphInvariants`を代表シナリオの観測点とseed付き操作列の各stepで呼ぶ。テスト専用の別不変条件や、派生値同士だけを比較する代替検査を作らない。

## Module boundaries

```text
include/city/road/input_types/
include/city/road/authoritative_types/
include/city/road/derived_types/
src/generation/     generate, segments, connections, markings, emit
src/geometry/
src/operations/
src/persistence/
```

`src/geometry/`はalignment、section、junctionの純粋幾何を持つ。ソースファイルの分割は実装責務を固定するために行い、工程数と一致させない。emitはauthoritative headerをincludeせず、adapterはauthoritative型を構築しない。幾何はgraphを読むがderivedを書かない。
