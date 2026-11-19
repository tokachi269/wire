# Backbone 操作意味論

標準wire描画は`docs/editor/draw_interaction.md`に従う。開始anchor後の各Clickは完全なroute区間を1つ確定し、Enterは有効previewを必要なら確定してsessionを終了する。Escapeは未確定previewだけを破棄し、確定済みwire topologyはUndoだけで取り消す。

この文書は、操作前の状態と操作の組合せごとに、期待する状態遷移を定義する。実装前に合意する要件の正本であり、実装済みコードやテスト結果から要件を定義しない。実装の回帰一覧は`domains/wire/tests/spec_ledger.md`、未実装の保留条件は`docs/merge_readiness.md`が担当する。

## セルの状態

| 表記 | 意味 |
|---|---|
| `C` | 操作成功。継続テーブルへ接続を記録する |
| `O` | 操作成功。推測せず未接続openのまま残す |
| `K` | 操作成功。既存の接続記録を維持し、派生出力だけ再計算する |
| `U` | 状態変更前に`unsupported`として拒否する |
| `-` | その状態では操作対象にならない |
| `D#` | 要件未定義。決定してこの表を更新するまで実装しない |

`C`は接続の有無だけを表す。NodePatch、jumper、row、fixtureは接続記録と現在の幾何から導出するため、`C`の判定理由に角度を使わない。

## 既存状態

| 状態 | 定義 |
|---|---|
| `S0` | 対象nodeに同placement/laneのincident edgeがない |
| `S1` | 未接続openが1本ある |
| `S2` | 通常角として表示中の接続済みpairだけがある |
| `S3` | 鋭角として表示中の接続済みpairだけがある |
| `S4` | 接続済みpairと未接続openが1本ある |
| `S5` | 接続済みpairと未接続openが2本以上ある |
| `SM` | source edge中間のownerless node |

状態はBundle placementとlaneごとに評価する。同じnodeでもテンプレートまたはplacementが違うedgeは別の評価単位であり、category名や高さでまとめない。

## 操作×状態

| 操作 | `S0` | `S1` | `S2` | `S3` | `S4` | `S5` | `SM` |
|---|---|---|---|---|---|---|---|
| `add_one_edge`新規edgeを1本追加 | `O` | `C` | `O` | `O` | `C` | `O` | テンプレートpolicyに従い`O`または`U` |
| `add_two_edges`同一操作で2 edgeを追加 | `C` | `O` | `C` | `C` | `O` | `O` | `D3` |
| `move_pole_angle` poleを移動して角度を変更 | `-` | `K` | `K` | `K` | `K` | `K` | `D3` |
| `update_placement` placement設定を更新 | `-` | `K` | `K` | `K` | `K` | `K` | `K` |
| `save_load` save/load | `K` | `K` | `K` | `K` | `K` | `K` | `K` |
| `regenerate` regenerate | `K` | `K` | `K` | `K` | `K` | `K` | `K` |
| `connect_two_open`明示的に2 openを接続 | `-` | `-` | `-` | `-` | `-` | `D2` | `D2` |

### exact Bundle lane topology reconcile

`UpdateBackboneBundleConductorCount`は、exact `Bundle`のlane topologyを保存済みrow continuity componentごとにreconcileする編集後操作である。接続状態の意味は変えない。存続laneは全状態で`K`、増加laneは同じcomponentの既存row/continuity判断を使って追加し、減少laneは末尾から退役する。

lane permutationやorder変更は扱わない。

`kRange`はテンプレート範囲内の要求countだけを受理する。`kFixed`の個別変更、必須binding不足、退役spanのuser Attachment、退役Portのmanual position、scope外でも使用中のPortは失敗とし、全component成功前にauthorityへ反映しない。

### exact Bundle lifecycle

`RetireBackboneBundle`は、exact `Bundle`識別情報と、それを参照する全edge bundle componentをauthoritative topologyから完全に退役する編集後操作である。別の接続状態へ切り替える操作にはしない。削除担当箇所は既存pipelineの`retire_untouched`に置く。

`S1`～`S5`、branch、cross、sharpをcategoryで除外せず、対象BundleのSpan、専用Port、binding、row continuity、default endpoint Attachmentを同じtrialで退役し、残存authorityからderived出力と実行時indexを再構築する。

`SM`ではsource-edge側のexact edge bundle識別情報を参照する存続Spanがある場合、そのsource Bundle退役を拒否する。sourceを残したbranch Bundleの退役は可能とする。必須binding不足、user Attachment、manual Port、Span override、scope外Spanが使用するPort、またはsource dependencyをexact識別情報で確定できない状態は、authority不変で失敗する。

`AddBackboneBundleInstance`は接続相手を新しく決める操作ではない。exact anchor `Bundle`の全edge bundle membership、row continuity、lane index relationを保存済みauthorityから読み、同じテンプレート、conductor count、lane policyを持つ新しいexact `Bundle`へ同型複製する。

callerが指定できるのは新しい非zero `placement_key`とexplicit height / lateral / spacingだけである。新Bundle、EdgeBundle、Port、Span、binding、default endpoint Attachmentは全componentを1つのtrialで生成し、anchorとpeerの識別情報を変更または共有しない。

新instanceのPort / Spanは常に別ObjectIdとする。Attachment、manual Port、Span overrideは複製対象にしない。

branch / cross / sharpはcategoryで除外せず、anchorの保存済みrelationをそのまま複製できる限り扱う。pipelineへはanchor continuityを一時的なexplicit constraintとして渡し、通常の候補規則による再pairingを複製の期待値の根拠にしない。

生成後のphysical edge membershipとlane continuityがanchorと同型でなければcommitしない。anchor scopeまたは必須bindingの欠落、placement key重複、テンプレート / count / lane relation不整合はauthority不変で失敗する。

source-edge dependencyを持つanchorは、exact source Bundle mappingを入力できない現段階では`SM`のfail-closed制約として状態変更前に`unsupported`で拒否する。

`ReconcileBackboneBundleInstances`は接続状態やscope membershipを推測する操作ではない。callerが明示するexact current Bundle IDsと、nonzero placement keyを持つdesired concrete `BackboneBundleSpec`をkeyで分類し、intersectionは識別情報を維持してplacement / permitted countだけを更新し、desired-onlyはcaller指定のcurrent-scope anchorから追加し、current-onlyはexact retireする。

desired-only anchorは同じテンプレート / lane topologyを持つ必要があり、0 -> 1、source-edge mapping、テンプレートmigration、lane order / pairing policy変更は推測せず拒否する。

desiredはexplicit placementで、keyはscope内一意、layerはテンプレートdefault、spacingはpositive finiteとする。入力順は意味を持たず、placement key順の安定処理とする。addを全て終えてからretireするためretire予定anchorも利用できるが、個別操作が保存したmembership / continuity / support assignmentを並べ替えや補正で書き換えない。

全処理は1つのouter `CoreState` trial内で行い、最終desired集合、SavedBackboneGraph、derived/runtime、検証が整合した場合だけ一度commitする。late retire conflictやsource-edge add unsupportedを含む任意の失敗では、先行survive / addも本状態へ反映しない。

### persisted Bundle variation Apply

route bundle variationのdecision ownerはWire generation入力resolverである。入力は人間が記述したrandom bundle rule、route seed、preferred side、`PoleTypeDefinition` / `BundleTemplate`定義で、出力はconcreteな`BackboneBundleSpec` entriesである。

利用側は既存backbone generation pipelineだけとし、pipeline / layout / curve / viewerはrandomizationを再判断しない。

`GenerateBackboneBundleVariation`はresolver入力と生成されたexact live Bundle scopeを、通常topologyと同じtrialで関連付ける。descriptorはexplicit Applyの入力であり、loadやregenerateはresolveしない。

persistent検証はdescriptor構造とsaved relationを検査し、current resolver resultとの一致を要求しない。Generate / Applyトランザクションだけが実際のresolved resultとcommit instanceの一致を検査する。

`ApplyBackboneBundleVariation`は保存scopeをcurrent、descriptorのresolve結果をdesiredとして既存`ReconcileBackboneBundleInstances`へ渡し、descriptor更新とconcrete topology更新を1つのouter trialでcommitする。

存続placement keyのBundle識別情報を維持し、branch / cross / sharpの追加instanceはlive anchorの保存membershipとcontinuityだけを複製する。placement keyはvariation scope内のcorrelationであり、別variationの同値keyとは`Bundle::id`で区別する。

ruleのconductor countはtopology上の導体数であり、見た目の束を作るためには変動させない。ruleのheight / lateral envelopeは各Bundle instanceを独立にsampleする入力である。resolverはBundle間の最小間隔や収容上限を推測せず、同じ位置へ近づくことを理由にdensityを拒否しない。

相互clearanceが必要なfeatureはrandom resolverの隠れ制約ではなく、物理配置のdecision ownerと入力を別途決める。HVの3相配置はrandomization対象外とする。非HVのsupported placementは連続座標をそのまま使わず、同じroute-wide sideにある有限のsupport slotへ寄せる。

load、regenerate、通常updateは保存済みconcrete placementを使い、rerollしない。同一BundleのplacementをPoleごとに変えず、同一route内でsideを反転しない。road-facing sideは外部から与える単純なsignであり、WireはRoadドメインや道路の意味を解釈しない。

1 -> 0ではconcrete Bundleを完全退役し、live `SavedBackboneEdgeBundle`に所有されなくなったedge/nodeも同じtrialで回収する。hidden skeleton、membership snapshot、row-continuity replay recordは保存しない。

その後の0 -> 1はinitial 0 -> 1と同じく`U`で、過去path、category、幾何からmembershipを推測しない。Density Applyは同じテンプレート / lane topologyのlive instanceをanchorに追加できる範囲をsupportedとし、高レベル既定ruleはeditable groupを1本以上に保つ。

recipe-backed Bundleの個別placement/count/add/retire/通常extensionは`U`とし、exact scopeを持つvariation操作だけがouter trial内でinstance ownership associationを一時的に外して既存generic操作を合成する。

いずれの失敗もdescriptorとphysical authorityの両方を不変に保つ。

`ExtendBackboneBundleVariation`はcallerのBundle specを受け付けず、保存済みexact instanceからfull concrete scopeをCore内で再構築して既存pipelineへ渡す。callerはpathとexact node referenceだけを指定する。

partial scope入力、stale exact scope、live instanceのないvariation、source-edge mappingの推測はfail-closedとし、既存descriptorとphysical authorityを変更しない。

variation extensionのbranch pickもCoreがvariation IDからexact live instanceを引き、その`BundleTemplate`集合だけを既存`ResolveBranchPick`へ渡す。descriptorにruleがあっても0 instanceのテンプレートはphysical branch scopeへ含めない。

`SavedBackboneEdge.route / order`は初回保存時のroute-local導出補助であり、physical edge識別情報やsame-node-pair再利用のcompatibilityではない。同じphysical edgeを別route順序から使う各occurrenceのroute/orderは`SavedBackboneEdgeBundle`が所有する。

same-node-pair edgeのreuseはlive topology内の既存識別情報を使う。Bundleを持たないedgeのreuse契約はv1には存在しない。

WASM/Webは選択Spanのexact `bundle_id`から保存済みdescriptorを参照し、選択中scopeの調整を`ApplyBackboneBundleVariation`へ渡す。adapterはcategory、テンプレート名、幾何からscopeを推測しない。

初回生成もWebでresolveしたconcrete specを通常Generateへ渡さず、descriptorをCoreのvariation生成入口へ渡す。既存routeの調整値は保存済みdescriptorに対する相対値であり、neutral Applyはrulesを変えない。

描画中routeでは同じroute controlが保存済みvariation IDへatomic Applyし、確定済み全区間と次のpreview / commit入力を同じdescriptorへ更新する。Finishはdraw sessionだけを終了し、直前のrouteをvariation編集targetとして保持する。

recipe-backed routeの選択はtargetをそのexact variationへ切り替え、新規routeの最初のanchor確定だけがtargetをnext routeへ戻す。Webは1組のcontrolをtargetに応じて初期descriptor生成または保存済みvariation Applyへ振り分ける。

個別Bundle編集は拒否し、viewerはrecipeを解釈せずApply後の通常sceneだけを描画する。

### Other post-edit regenerate operations

現対応は`UpdateBundleTemplate`のfixed count増減と`kTopology` policy差分、`UpdateCableTemplate`のbackbone continuity policy / default endpoint attachment decision差分、`UpdatePoleTypeDefinition`のactive backbone pole構造差分、`ApplyBundleRelatedPoleTypeToExistingPoles`のrelated pole type適用、backbone spanのendpoint socket / branch-down override、`UpdateLayoutSettings`の全backbone route再導出である。

同一edgeに複数edge_bundleがある場合はsaved edge_bundles順を生成時のbundle spec順として扱い、group offsetを再構成する。3点以上routeの接続はsaved row continuityとsaved nodeからpipeline graphを復元し、row表現は現在幾何から再導出する。

row key / laneが一致するbindingは再利用し、不一致のbindingはretire + emitでreconcileする。

存続するuser attachmentはspan idとともに保持し、退役spanにuser attachmentがあれば状態変更前に`unsupported`で拒否する。`AttachmentOrigin::kDefaultEndpoint`はtrial内で退役できる。

`UpdateCableTemplate`のcontinuity policyとdefault endpoint attachmentはroute scopeごとに同じ入口を通し、既存spanのcurve decisionとauto endpoint attachmentを編集後テンプレートへreconcileする。

non-backbone spanを含むdecision差分は未対応として拒否する。

`UpdatePoleTypeDefinition`は対象typeをactive backbone poleが使用中でもplacement-only差分なら`kReposition`として既存auto portを再配置し、layout -> geom -> drawを再導出する。

band追加・削除、enabled/side/layer/role/priority変更、anchor slot変更などの構造差分は、対象poleのincident edgeをroute-local bundle scopeに展開し、統一regenerateでemitから再解決する。

manual portはテンプレートplacement更新では動かさない。統一regenerateでも存続laneのmanual portはworld positionとmanual markerを保持し、退役laneのmanual portは状態変更前に拒否する。

backbone spanのendpoint socket / branch-down overrideは`override_state`が正本である。APIは本状態を直接書かず、試行用の状態にoverrideを入れて対象spanのedgeを統一regenerateする。

layout ruleはoverride解決を消費し、socketはendpoint source / resolved socket、branch-downはendpoint offsetとcurveに反映する。

`UpdateLayoutSettings`はlayout settingsを試行用の状態に入れ、row continuityで接続されたedge_bundle componentとbundle instanceごとのscopeを統一regenerateする。scope復元はsaved row continuity componentを正本とし、保存済みedgeのroute/orderは判定入力にしない。

拡張条件は操作ごとに異なる。`UpdateCableTemplate`はnon-backbone spanを含むdecision更新、`UpdateBundleTemplate`はfresh生成が拒否する新policyまたはnon-backbone spanを含むscope、`UpdatePoleTypeDefinition`はmanual port退役拒否の緩和、endpoint socket / branch-down overrideは対象範囲をspan-local以外へ広げる場合、`UpdateLayoutSettings`はlayout settingsが新しい生成入力を持つ場合に、それぞれ別シナリオとして設計してから広げる。

## 実行時coverage契約

`C` / `O` / `K` / `U`の各セルは、テストケースIDや手書きタグではなく、テスト実行中に構築された`CoreState`から状態を分類してcoverageへ記録する。matrixに行または列を追加し、対応する実状態を構築しなければcoreテスト全実行を失敗させる。

coverageとして認める各Observationは、そのcellの操作結果について次の独立した証拠を1つ以上持つ。

- `oracle`: 手計算値または固定fixture期待値
- `anchor`: 正本入力やstable IDとの関係
- `presence`: 必須出力の存在数。silent dropを許可しない
- `differential`: 同じ意味の別操作列、save/load前後等の比較

派生値同士の一致だけを示す`derived_equality`は、単独ではcoverage evidenceにしない。source textを検査するケースは`SourceGuard` familyへ分離し、操作×状態coverageには使用しない。

### coverageと不変条件

coverageは「操作×状態のrequired cellへ到達したか」だけを表す。cellへ到達した状態が内部整合していることは、別の共通不変条件で検査する。

`Observe` / `ObserveEmpty` / `ObserveMidspan`はcoverageを記録する前に、全bindingについて次を検査する。

- 正本のplacement、band、laneから求めたPort anchorと、binding yawのpole frame上のPort位置
- endpoint fixture anchorと対応Port
- row fixture frameとbinding yaw
- layout endpoint、モデルsocket、curve endpoint

不変条件違反は`anchor` evidenceとして、binding、Port、期待値、実測値を含む理由で失敗する。派生値同士が一致するだけでは、正本anchorとのずれを通過させない。

## 入口境界

core APIの全required cellはCoreテストが`(cell, core_api)`として強制する。これに加え、実アプリがbranch生成へ渡すpayloadは次の入口で検証する。この表はwebテストが実行時に直接読み、required cellを実payloadで通さなければ失敗する。

| Cell | Required entries |
|---|---|
| BOS:add_one_edge:S1 | `wasm_adapter` `viewer_action` |
| BOS:add_one_edge:SM | `wasm_adapter` `viewer_action` |

`wasm_adapter`は実WASM状態でpole snapとsource-edge midspanを解決し、生成結果まで検証する。`viewer_action`は`ViewerActions`へ同じhit payload形状を渡し、解決済みnode識別情報がnode specへそのまま渡ることを検証する。

adapter/viewerは位置や`hitId`からnode識別情報を再推測しない。
### 接続候補の規則

生成操作の完了時に、影響nodeの未接続endpointを同じ解決処理へ渡す。

1. `SavedBackboneRowContinuity`に既に含まれるendpointは候補から除外する。
2. 同じBundle placement識別情報、互換lane、同じnodeの未接続endpointだけを候補とする。
3. 候補が相互に1つだけなら接続し、`SavedBackboneRowContinuity`へ記録する。
4. 候補が0または2つ以上なら操作は成功させ、未接続openのまま残す。
5. 角度、距離、edge ID、生成順、配列順、category、高さは相手選択に使わない。

同一操作で追加された2 edgeもこの規則を通す。操作内の隣接という理由だけで既存の未接続endpointより優先しない。peer edge確定後、`kPermutableHomogeneous`かつconductor識別情報を保存しないmulti-lane rowは、両rowの最終Port列方向からsame/reverseの1 bitでlane対応を決める。

これは相手選択ではなく、選択済みpeer間の物理lane対応である。fixed orderまたはconductor識別情報を保存するテンプレートはlane indexを維持する。

### 表現の規則

接続済みendpointの表現は、継続テーブルと現在の角度から導出する。

| 条件 | 派生表現 |
|---|---|
| 通常角 | 共有row/fixtureとNodePatch |
| 鋭角 | edgeごとの2 open row/fixtureとlane別jumper |
| 未接続 | edge固有open row。NodePatch/jumperなし |

角度は表現だけを変え、`SavedBackboneRowContinuity`のpeer edgeを変更しない。generated Portは常にedge endpointごとに別識別情報を持つ。通常角pairでもPortを共有せず、共有するのは派生row/fixtureだけとする。

通常角pairの2 Portのworld位置は、同じrow layout決定から導出して浮動小数bitまで一致させる。2つの独立計算結果を後段で近づけたり、一方を他方へコピーして補正したりしない。

`add_one_edge`で`S1`のopen endpointを通常pairへ接続するpromotionでは、既存Port ID、edge endpoint別binding、continuityを維持する。一方、open時の物理frameは保存せず、通常pairの二等分frameを現在のincident edge幾何から再導出する。

既存Port位置、bindingの`layout_yaw_deg`、row fixture、endpoint fixture、curve endpointはこの同じframeへ追従する。Port位置だけを旧open frameへ残すこと、またはbinding yawだけを更新することは禁止する。

manual Portを移動する必要がある場合は推測で動かさず`unsupported`とする。

endpoint fixtureは派生した`(row, lane)`単位で生成する。Port単位では生成しない。通常角pairでは2 Portに対して1 fixture、鋭角pairでは2つの派生open rowに対して2 fixtureを生成する。角度変更ではPort識別情報とcontinuityを維持し、row、fixture、NodePatch、jumperだけを再導出する。

`SavedBackboneRowKey`は`(node_id, edge_id)`だけを保存し、1つのedge endpointを識別する。pair/openの表現、peer edge、row axisは保存しない。接続済みpeerは`SavedBackboneRowContinuity`から解決し、現在のincident edge幾何と既存のcorner規則を1つの派生処理へ渡して、共有rowまたは2 open rowを決める。

通常角のrow axisは従来どおり2 edgeの単位接線二等分線から決め、鋭角の各row axisは各edge方向から決める。

support levelは物理fixture row単位で割り当てる。continuity単位では割り当てない。通常pairは共有rowとして1 levelを使う。鋭角pairは2つのopen rowとjumperへ派生するため、2つの異なる空きlevelを使い、各rowが別support groupを持つ。

既に低いlevelが別rowで埋まっている場合は、鋭角pairの2 rowへ次の2つの空きlevelを割り当てる。jumperはcontinuityを表すだけで、placement levelを共有させない。

通常角pairの相手は`placement_key`で限定しない。別ルートのkeyであっても、同じnode、同じテンプレート、同じ名目取付位置、同じlane数とspacingを持つ保存済みopen rowとは、通常角のとき共有rowへ合流する。合流したedgeは既存の`Bundle`へ属し、その共有rowのsupport levelとsupport groupを引き継ぎ、laneごとの`SavedBackboneRowContinuity`を追加する。要求のplacement keyは新しい`Bundle`を作らない。

合流相手は同じ`placement_key`を優先する。同じkeyの候補があればそれを使い、無い場合にだけ同じ名目取付位置の別keyを使う。同じ名目取付位置に候補が2つ以上ある場合は合流せず、それぞれ独立したrowとして扱う。保存済みpair rowは合流相手にしない。鋭角、異なる取付位置、異なるlane構成も合流しない。これらは従来どおり別rowとして次の空きlevelを使う。

### save schema

saveスキーマとload原子性は[`state_and_persistence.md`](state_and_persistence.md)を正本とする。

## 決定済みセル

| ID | 決定 | 必須検証 |
|---|---|---|
| `D1` | Portと`SavedBackboneRowKey(node_id, edge_id)`は常にedge endpoint別識別情報とし、pair/open、peer edge、row/fixtureはcontinuityと現在幾何から導出する | 通常角2 Portのworld位置bit一致、laneあたりfixture 1個、角度変更時のPort/RowKey不変、NodePatch↔jumper切替、v2共有Port/pair key migration前後の派生出力等価 |

## 未定義セル

| ID | 未決定事項 | 決めない場合の影響 |
|---|---|---|
| `D2` | 複数openから2本を明示指定するCore APIとUI操作 | `S5`の曖昧状態は自動解消しない |
| `D3` | ownerless中間nodeで複数edgeを追加、明示接続、repositionする操作契約 | midair branchの複合junctionを安全に実装できない |

## 変更手順

1. 新機能またはバグ修正の対象となる操作と既存状態を特定する。
2. 対象セルが`D#`なら、Input、正本、派生、失敗時動作を決めて表を更新する。
3. `C/O/K/U`へ確定したセルごとにfail-firstテストを追加する。
4. 実装後、対応テストを`domains/wire/tests/spec_ledger.md`へ登録する。
5. 隣接セルに同じ変更が波及しないことを確認する。

未定義セルをテストの期待値や現在のコードから埋めてはならない。
