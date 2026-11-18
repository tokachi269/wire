# Core仕様台帳（Phase 4.8）

このledgerは現行のevidence、テスト群、authority guard mapping、machine coverageのindexであり、設計上の意味の正本ではない。Wireの意味は`docs/wire/architecture.md`、`docs/wire/backbone_operation_semantics.md`、関連するモデル文書を正本とする。

## 対象範囲と方針
- 観測根拠: 公開API戻り値、`view()`、`connection_index()`、`relation_index()`、`find_*cache()`、`port_resolution_debug_records()`、`last_path_direction_debug()`、`last_generation_edge_orientations()`、`SavedBackboneResult()`、`Validate()`のみ。`last_generation_backbone`系はmidair入力まわりの一時snapshot観測としてのみ扱う。
- 期待値粒度: `Exact`は決定論のみ、`Invariant`は不変条件のみ。
- モック方針: ドメインロジックのモック禁止（本スイートはモック未使用）。
- 時間/並行: 実時間待ち・非決定並行を使わない。
- 分類ルール: `分類=Symptom`は最終症状やユーザ価値を直接固定する行、`分類=Authority`は正本・ownership・decision originを固定する行。
- 永続化Boundary: `CoreStateAuthoritativeStorage`へフィールドを追加する変更は、serializer・deserializer・roundtripテストの更新を伴う。

## 実行時coverage方針

- 操作×状態coverageはこのledgerへ手書きしない。`C836`が正本状態を実行時分類し、full `wire_core_tests`の終了時に意味論文書のrequired cellと照合する。
- coverageとして認めるケースは`oracle` / `anchor` / `presence` / `differential`のいずれかを実行時記録する。`derived_equality`単独は認めない。
- source textを検査するケースは`SourceGuard` familyで登録し、操作×状態coverageには数えない。
- Web入口coverageは同じ意味論文書の`入口境界`をテストから直接読み、WASM adapterとViewerActionsの実payloadを通す。
- `legacy_unclassified`は既存ケースの根拠種別が未移行であることを明示する値であり、実行時coverage evidenceではない。

過去の有効性監査は[`audits/`](audits/)に保管する。この台帳には現行の証明とindexだけを記載する。

## Backbone正本guardの検査範囲

matrix / aspectは外部仕様と観測点を表す。authority guardは、各観点を決めるproduction式が分散しないことを固定する。`Required owner tokens`は担当箇所に必須、`Unique production tokens`は`domains/wire/src`と`domains/wire/include`内で担当箇所にだけ存在できる。`Unique production tokens`には実装式を書く。関数名はcall siteが正当に名前を書くため固定できない。tokenはbacktickで囲む。囲み漏れはlintが拒否する。

`Forbidden owner tokens`は担当箇所内で使ってはいけない分岐語彙を示す。

| Authority | Owner | Required担当箇所tokens | Unique production tokens | Forbidden担当箇所tokens |
|---|---|---|---|---|
| row_support_placement | `domains/wire/src/generation/backbone/pipeline.cpp` | `enable_branch_down_offset` `branch_endpoint_offset_m * static_cast<double>(support_level)` `endpoint->branch_down_offset_m` | `branch_endpoint_offset_m * static_cast<double>(support_level)` | `BundleKind::kHighVoltage` `row_height_offsets` `stable_row_slot_plan` `kRowHeightSeparationM` |
| layout_endpoint_lowering_projection | `domains/wire/include/city/wire/span_layout_types.hpp` | `ApplyEndpointLayoutRule` `dst.branch_down_offset_m = rule.branch_down_offset_m` | `dst.branch_down_offset_m = rule.branch_down_offset_m` |  |
| sharp_representation_threshold | `domains/wire/src/generation/backbone/row_representation.cpp` | `kSharpCornerInteriorAngleMaxDeg` | `kSharpCornerInteriorAngleMaxDeg` |  |
| sharp_jumper_derivation | `domains/wire/src/generation/backbone/curve_parts.cpp` | `endpoint.jumper_peer_port_id` `VisualCurvePartKind::kJumper` |  | `Length(peer->point - endpoint.point)` |
| saved_row_support_assignment | `domains/wire/src/state/backbone.cpp` | `binding.support_level = support_level` `binding.support_group_id = support_group_id` | `binding.support_level =` `binding.support_group_id =` |  |

Evidence列は実行時に記録したassertion種別を書く。宣言した種別は実際に記録されていなければならず、記録されない種別は書かない。未分類は`legacy_unclassified`単独とし、`source_guard`はSourceGuard familyの登録と一致させる。この照合はtest runnerが全件実行後に行う。

| Case ID | Family | 分類 | 目的 | Evidence | 壊れた時に守りたいユーザ価値 |
|---|---|---|---|---|---|
| C750 | Behavior | Invariant | authoritative saveはバージョン付きで決定的かつ編集を反映する | `legacy_unclassified` | 保存結果の非決定性と編集の取りこぼしを防ぐ |
| C751 | Behavior | Invariant | 元の状態と初回loadの派生意味値がbit一致し、初回loadと再loadの完全比較も維持する | `presence` `differential` | layout・curve sample・bounds・visual curve・run IDを比較する。元の状態との比較では契約どおり実行時バージョンと診断用計測値を除外する。VisualModelInstanceCache全体はこのsnapshotの対象外 |
| C752 | Behavior | Symptom | load後の再saveは元byteと一致する | `legacy_unclassified` | 保存と再読込の反復でプロジェクトfileを変質させない |
| C753 | Behavior | Symptom | load後もID衝突なく編集を継続できる | `legacy_unclassified` | 読込後の編集で既存objectを上書きしない |
| C754 | Behavior | Boundary | 不正loadは正本・派生出力・保留branchとその再試行結果を変更しない | `presence` `anchor` `differential` | 保存byteだけでは見えないキャッシュ破損、pending消費、識別情報採番の変化を防ぐ |
| C762 | Behavior | Boundary | ModelDescriptorからassembly partへstable socketを値コピーする | `legacy_unclassified` | transient descriptorへのpointer保持やscene幾何からのsocket再推測を防ぐ |
| C763 | Behavior | Boundary | assembly登録は検証後だけ反映し、wire socketとendpoint mount socketを含めsave/load対象になる | `legacy_unclassified` | adapterが正本mapを直書きする、不完全assemblyを部分反映する、またはmount追加で旧saveを読めなくする回帰を防ぐ |
| C01 | Behavior | Symptom | ID生成の単調増加 | `legacy_unclassified` | 永続ID衝突防止 |
| C02 | Behavior | Symptom | ObjectStore整合 | `legacy_unclassified` | 参照崩壊防止 |
| C03 | Behavior | Symptom | Span追加初期Dirty | `legacy_unclassified` | 再計算漏れ防止 |
| C04 | Behavior | Symptom | MovePole局所更新 | `legacy_unclassified` | 無関係再計算抑制 |
| C06 | Behavior | Authority | PoleType適用 | `legacy_unclassified` | テンプレ適用維持 |
| C07 | Behavior | Symptom | PoleType差分 | `legacy_unclassified` | 型差分維持 |
| C19 | Behavior | Symptom | interval Pole生成 | `legacy_unclassified` | interval自動配置信頼性 |
| C20 | Behavior | Symptom | 短polyline拒否 | `legacy_unclassified` | 入力ミス耐性 |
| C22 | Behavior | Symptom | 存在しないPort拒否 | `legacy_unclassified` | 参照不正耐性 |
| C28 | Behavior | Symptom | through連続性 | `legacy_unclassified` | 幹線連続をPort共有へ符号化する回帰防止 |
| C29 | Behavior | Symptom | 表示ID採番 | `legacy_unclassified` | UI追跡性 |
| C61 | Behavior | Invariant | 鋭角は倍率拡幅ではなく径間別rowで実幅を保つ | `legacy_unclassified` | 鋭角でゼロへ近づく投影幅を大倍率で追う、またはheight差をrow分離の代理にする回帰防止 |
| C101 | Behavior | Symptom | HV空中分岐規格フラグ固定 | `legacy_unclassified` | 高圧規格逸脱の混入防止 |
| C105 | Behavior | Authority | Segment中間でMidair生成 | `legacy_unclassified` | 空中分岐入力を安定して扱える |
| C106 | Behavior | Authority | Pick経由HV空中分岐禁止 | `legacy_unclassified` | 高圧規格逸脱の混入防止 |
| C107 | Behavior | Authority | Midair Dry-run非破壊 | `legacy_unclassified` | ホバー評価で状態が汚れる回帰防止 |
| C64 | Behavior | Symptom | Guide頂点の強制Manual解除 | `legacy_unclassified` | DrawPath点=強制Pinの回避 |
| C65 | Behavior | Symptom | pin_verticesオプション | `legacy_unclassified` | ピン留め挙動の明示制御 |
| C66 | Behavior | Symptom | Pole Pin/Unpin切替 | `legacy_unclassified` | ユーザー明示ピン留め運用 |
| C71 | Behavior | Symptom | MovePoleでAuto Port再投影とManual保護 | `legacy_unclassified` | Pole移動で手修正Portが壊れない |
| C44 | Behavior | Authority | Bundle参照API整合 | `legacy_unclassified` | 複数本配線の正本一貫性維持 |
| C45 | Behavior | Symptom | 不正Bundle参照拒否 | `legacy_unclassified` | 不正参照でデータ破壊しない |
| C90 | Behavior | Symptom | Backbone bundles必須 | `legacy_unclassified` | legacy経路の逆流防止 |
| C74 | Behavior | Symptom | 非HVテンプレ固定1本 | `legacy_unclassified` | 規格固定で入力削減 |
| C75 | Behavior | Symptom | 非HV固定テンプレcount上書き拒否 | `legacy_unclassified` | 固定規格の強制 |
| C77 | Behavior | Symptom | 複数テンプレ同時生成 | `legacy_unclassified` | 複数束同時入力の成立 |
| C80 | Behavior | Symptom | center hintのPole非重なり | `legacy_unclassified` | center PortがPoleに埋まらない |
| C50 | Behavior | Symptom | Port初期モード | `legacy_unclassified` | 新規Port生成規則の固定 |
| C52 | Behavior | Symptom | Manual保護 | `legacy_unclassified` | 軽微再生成で手修正消失防止 |
| C53 | Behavior | Symptom | BackboneSpec境界手動点安定 | `legacy_unclassified` | 軽微変更で手直し消失防止。既存support同座標の暗黙再利用は禁止 |
| C55 | Behavior | Symptom | Backbone経路 | `legacy_unclassified` | ルート計算基盤維持 |
| C58 | Behavior | Symptom | Reverse対称性 | `legacy_unclassified` | 方向モードで幾何が破綻しない |
| C59 | Behavior | Symptom | Avoid制約尊重 | `legacy_unclassified` | 回避制約の信頼性 |
| C117 | Behavior | Authority | Path入力ではmidair policyを入力段階で強制しない | `legacy_unclassified` | DrawPathの入力段階で不要に操作を止めない |
| C817 | Behavior | Boundary | stale pending node idはload後に状態変更前拒否される | `legacy_unclassified` | セッションの一時データをauthoritative保存したり、load後に壊れたnode idを生成入力として黙って受ける回帰防止 |
| C818 | Behavior | Invariant | pending support nodeはclearで破棄され生成成功で消費される | `legacy_unclassified` | path cancel後のpending残留や生成成功後のdraft再利用を防ぐ |
| C819 | Behavior | Boundary | path point座標の非有限値は状態変更前拒否する | `legacy_unclassified` | raycast由来のNaNがpreflightを抜けて後段のfallbackやNaN幾何を作る回帰防止 |
| C820 | Behavior | Boundary | pole tilt入力の非有限値は状態変更前拒否する | `legacy_unclassified` | 外部入力のinf tiltがpole transformやlayoutへ伝播する回帰防止 |
| C821 | SourceGuard | Boundary | BackboneSpec numeric入力検証はフィールド追加漏れを検出する | `source_guard` | 外部入力フィールド追加時にpreflight有限性検証を足し忘れる回帰防止 |
| C822 | Behavior | Boundary | EditResult error kindはvalidation/unsupported/internalを機械可読に分類する | `legacy_unclassified` | viewerやadapterがerror文字列prefixを再解釈し、入力不正・未対応・内部不整合を混同する回帰防止 |
| C823 | SourceGuard | Boundary | テスト失敗diagnostics helperが使える状態で、代表backboneシナリオへ適用されている | `source_guard` | 複数手順テストが再びboolだけに戻り、どの操作境界で壊れたか分からない状態へ退行するのを防ぐ |
| C824 | Behavior | Invariant | seed付きroute fuzzは成功時に共通不変条件を満たし、拒否時は状態を変えない | `legacy_unclassified` | 手書きシナリオだけでは漏れる参照切れ、NaN幾何、拒否時状態変更を短時間で検出する |
| C825 | SourceGuard | Boundary | silent fallback候補の分類表が維持される | `source_guard` | fallbackを発見しても分類せず温存し、後続変更で同じ黙示補完を増やす回帰防止 |
| C826 | SourceGuard | Boundary | core epsilonはsupport配下の共有ヘッダ1つから用途名付き定数を使う | `source_guard` | 退化判定・角度判定・位置比較のepsilonが再び局所aliasやmagic numberへ戻る回帰防止 |
| C827 | SourceGuard | Boundary | モデルrotationとPoleFrame座標規約がmodels.mdに記録される | `source_guard` | DCC/adapter/viewerが回転順やpole local軸を推測し、fixture/socket位置がずれる回帰防止 |
| C828 | SourceGuard | Boundary | CoreState mutable accessorは本体headerから削除し、残る正本mutable accessはfriend実装として監査する | `source_guard` | 正本mutable参照の裏口が分類なしで増え、決定者一箇所を破る回帰防止 |
| C829 | SourceGuard | Boundary | core政策定数は物理/固定仕様/見た目調整候補へ分類される | `source_guard` | 調整したくなる見た目定数と物理・固定仕様定数が同列のまま増殖する回帰防止 |
| C830 | SourceGuard | Boundary | domains/wire/srcのEditResult error literalは登録済みprefixを持ち、未知prefixはInternal扱いになる | `source_guard` | 文字列prefixなしerrorが黙ってUnsupportedに分類され、入力不正・未対応・内部不整合の区別が崩れる回帰防止 |
| C831 | SourceGuard | Boundary | NormalizedOr/unit_or/safe_unit系fallback呼出はsource上で分類される | `source_guard` | fallback分類がdocsだけにあり、呼出追加時に意図不明な黙示補完が増える回帰防止 |
| C832 | SourceGuard | Boundary | ResolveBranchPick pure化はbridge協定変更としてmerge readinessに保留登録する | `source_guard` | 副作用を持つ照会APIのpure化保留が忘れられ、未定義のまま次のUI/API変更へ混ざる回帰防止 |
| C833 | Behavior | Invariant | branch-down override 0は最終lowering値としてfixture socketとcurve endpointまで届く | `legacy_unclassified` | layoutが0 overrideを持つのにモデルassemblyがautomatic loweringを復活させる二重決定者の回帰防止 |
| C834 | Behavior | Invariant | 鋭角pairの物理row別support levelはendpoint fixture socketとcurve endpointまで届く | `legacy_unclassified` | support_levelだけ通ってmodel/socket/curveが下がらない、または鋭角2rowを同じ高さへ潰す回帰防止 |
| C835 | Behavior | Invariant | 鋭角pairはcontinuityを保ったまま2つの物理rowを別support levelへ配置する | `legacy_unclassified` | jumper continuityをplacement level共有として扱い、鋭角2rowが同じ高さになる回帰防止 |
| C121 | Behavior | Authority | Template責務の分離 | `legacy_unclassified` | 正本とテンプレ責務の混線防止 |
| C122 | Behavior | Symptom | CableTemplate編集でPole tiltを上書きしない | `legacy_unclassified` | Pole傾きをテンプレ変更が壊さない |
| C123 | Behavior | Invariant | BundleTemplateのtopology変更はbackbone scopeをregenerateする | `legacy_unclassified` | topology変更をrejectまたはvisual-onlyで済ませる回帰防止 |
| C124 | Behavior | Symptom | BundleTemplateのvisual-only変更はdirtyのみに留める | `legacy_unclassified` | 見た目変更をtopology再生成に混ぜない |
| C125 | Behavior | Symptom | ApplyPoleTiltは選択されたPole実体だけを更新する | `legacy_unclassified` | Pole tiltを実体値として局所更新できる |
| C202 | Behavior | Symptom | Pole tiltはincident span方向へ寄る | `legacy_unclassified` | Pole tiltが接続方向と無関係に倒れて見える回帰防止 |
| C203 | Behavior | Symptom | Pole tilt量はpull imbalanceに従う | `legacy_unclassified` | 懸垂の効きが弱く、直線と角度付きで同じ傾き量になる回帰防止 |
| C126 | Behavior | Symptom | WorldUpとlateral軸の整合 | `legacy_unclassified` | 軸依存の手書き計算を1定義に寄せる |
| C127 | Behavior | Symptom | PoleFrameのlocal/world roundtrip | `legacy_unclassified` | Pole傾き適用の回転順序ずれ防止 |
| C128 | Behavior | Symptom | uベース曲線APIの端点拘束 | `legacy_unclassified` | 曲線生成をuベースに固定する |
| C129 | Behavior | Symptom | sベース配置APIとsag合成 | `legacy_unclassified` | 正確な配置をsベースで扱える |
| C141 | Behavior | Symptom | parabolic sagがsupportで実勾配を持つ | `legacy_unclassified` | supportで平坦になるdecorative sagへ戻る回帰防止 |
| C142 | Behavior | Symptom | 近直線での横揺れ抑止強化 | `legacy_unclassified` | 微小な横成分で左右にぐにゃる回帰防止 |
| C143 | Behavior | Symptom | SmoothPassの横揺れ抑止 | `legacy_unclassified` | 継続カーブで不要な左右揺れを残す回帰防止 |
| C130 | Behavior | Symptom | OffsetEndpoint端点 | `legacy_unclassified` | 支点近傍で急に折れない離脱表現の土台 |
| C131 | Behavior | Symptom | 鋭角/競合接線での品質劣化安全策 | `legacy_unclassified` | 高度最適化なしで見た目破綻を抑える |
| C148 | Behavior | Symptom | SharpCornerのcompact化 | `legacy_unclassified` | 鋭角部までSmoothPass的に膨らむ回帰防止 |
| C144 | Behavior | Symptom | 長尺継続spanのG2優先採用 | `legacy_unclassified` | G2優先方針が単なる係数でなく判定として残ることを固定 |
| C145 | Behavior | Symptom | 短スパンのG1劣化 | `legacy_unclassified` | 短スパンで過拘束cubicを押し込む回帰防止 |
| C146 | Behavior | Symptom | branch passのG1優先と端点拘束保持 | `legacy_unclassified` | branchでG2に固執してsupport離脱が崩れる回帰防止 |
| C147 | Behavior | Symptom | PreferG1は明示選択 | `legacy_unclassified` | continuity設定と実挙動の意味ずれ防止 |
| C149 | Behavior | Symptom | ViaAttachmentのendpoint優先 | `legacy_unclassified` | attachment接続を支点中心通過へ戻す回帰防止 |
| C151 | Behavior | Symptom | worldspace variationの連続性 | `legacy_unclassified` | ID乱数でworldspaceを離散化する回帰防止 |
| C171 | Behavior | Symptom | public header smoke surface | `legacy_unclassified` | 公開ヘッダの依存漏れや内部storage断面への依存が利用者ビルドへ漏れる回帰防止 |
| C181 | Behavior | Symptom | GetPoleDetailは対象poleのowned endpointだけを返す | `legacy_unclassified` | owned endpointの公開観測面が他poleを混ぜたりrelation index直参照前提に戻る回帰防止 |
| C182 | Behavior | Symptom | MovePoleは対象poleのowned endpointだけ更新する | `legacy_unclassified` | pole移動が他poleのowned endpointまで巻き込む回帰防止 |
| C200 | Behavior | Authority | bundleのbranch down offset policy変更はtopology regenerate | `legacy_unclassified` | branch-down policy変更がno-opまたはvisual-only扱いになる回帰防止 |
| C277 | Behavior | Symptom | 高低差の大きいsmooth spanはcomposite curveへ切り替える | `legacy_unclassified` | 高低差spanが単一cubicへ戻り、support近傍で不自然に直線化・急変する回帰防止 |
| C161 | Behavior | Symptom | 長いbranch spanは横回り込みを抑える | `legacy_unclassified` | branch supportの横向きがspan全体を左右へ引っ張る回帰防止 |
| C162 | Behavior | Symptom | branchの局所departureは短スパンほど強い | `legacy_unclassified` | branchの横成分抑制がspan長に応じて変わらない回帰防止 |
| C163 | Behavior | Invariant | pass種別は端点処理を変えても指定された物理sag量を再加重しない | `oracle` | branch/mainの見た目差を理由にsag量へ隠れ倍率を掛ける回帰防止 |
| C287 | Behavior | Symptom | CommunicationPoleはCore所有の日本配電profileを保つ | `legacy_unclassified` | Web起動時状態変更撤去後にCore既定profileが古い値へ戻る回帰防止 |
| C289 | Behavior | Symptom | pole type高さ更新は既存pole本体高さとowned auto portsへ再適用される | `legacy_unclassified` | pole type編集後も既存poleの本体高さやport高さがstaleのまま残る回帰防止 |
| C296 | Behavior | Authority | default poleテンプレートのHV categoryは単一高さを使う | `legacy_unclassified` | HV bandごとに高さが微妙にずれていてcategory UIの初期値だけ高く見える回帰防止 |
| C297 | Behavior | Invariant | related pole typeのpost-editはbackboneをregenerateして適用する | `legacy_unclassified` | 旧直接適用や拒否でSavedGraphと実体を不整合にする回帰防止 |
| C304 | Behavior | Authority | style context resolverはsemantic keyごとにdeterministicでroute-level傾向を共有する | `legacy_unclassified` | realism resolverがtransient idや呼び出し順に依存して不安定になり、route相関や再現性を失う回帰防止 |
| C309 | Behavior | Symptom | CommunicationPoleのcommunication / optical既定bandは中央sideを保つ | `legacy_unclassified` | CommunicationPoleの既定bandが再びleft/rightに戻り、光ケーブルや通信ケーブルがpole配置で左右へ逃げる回帰防止 |
| C310 | Behavior | Symptom | defaultテンプレートはoptical-with-supportの分岐typeを登録しない | `legacy_unclassified` | opticalだけ支持線有無でbundle typeが分かれてviewer上のtypeが増える回帰防止 |
| C313 | Behavior | Symptom | CommunicationPoleのdefault Pole Placement値はCore所有のviewer profileと一致する | `legacy_unclassified` | Web起動時patch廃止後にdefaultテンプレートとPole Placement UIの初期値がずれる回帰防止 |
| C783 | Behavior | Authority | Core defaultsがviewer startup semanticを所有する | `legacy_unclassified` | Webが起動時にupdateCableTemplate/updatePoleTemplate/updateGeometrySettingsで正本を書き換える経路の復活防止 |
| C784 | SourceGuard | Boundary | fixture placement planは操作単位で完全構築し、対象外memberを0 offsetで補わない | `source_guard` | fixture plan構築がaffected span数やendpoint port数に比例して全キャッシュ走査を繰り返す、または操作対象外memberを0 offsetで黙って平均へ混ぜる回帰防止 |
| C357 | Behavior | Invariant | cable decision更新はbackbone出力をregenerateする | `legacy_unclassified` | decision変更をstale成功や未再導出にしない |
| C368 | Behavior | Invariant | backbone milestone 1は単純line sliceを生成する | `legacy_unclassified` | v2が入口で最小縦スライスを通せない回帰防止 |
| C369 | Behavior | Invariant | backboneは生成spanごとにSpanLayoutRulesを保存する | `legacy_unclassified` | v2が旧authority/seedではなく保存済みrulesを正本にする境界確認 |
| C370 | SourceGuard | Boundary | backboneは旧契約 / recalc内部に依存しない | `source_guard` | v2が旧support-layout/recalc wrapperへ戻る回帰防止 |
| C371 | Behavior | Boundary | backboneは未対応入力をfallbackせず拒否する | `legacy_unclassified` | 未対応機能をv1 fallbackで埋めて境界が崩れる回帰防止 |
| C372 | Behavior | Boundary | backboneはrules保存で追加入力を要求しない | `legacy_unclassified` | backboneがrules-only正本から旧seed/authority契約へ戻る回帰防止 |
| C373 | Behavior | Boundary | backboneはrecalcなしでrulesからlayoutを保存する | `legacy_unclassified` | backboneが生成後にdirty/recalc/materializationへ戻らない境界確認 |
| C374 | Behavior | Invariant | backbone layout導出は同一入力で決定的 | `legacy_unclassified` | キャッシュや既存状態でlayout派生が揺れる回帰防止 |
| C375 | Behavior | Boundary | backboneはrecalcなしでlayoutからcurveを保存する | `legacy_unclassified` | backboneがcurve生成でrecalc/materializationへ戻らない境界確認 |
| C376 | Behavior | Invariant | backbone curve導出は同一入力で決定的 | `legacy_unclassified` | キャッシュや既存状態でcurve派生が揺れる回帰防止 |
| C377 | Behavior | Boundary | backboneはrecalcなしでcurveからboundsを保存する | `legacy_unclassified` | backboneがbounds生成でrecalc/rebuildへ戻らない境界確認 |
| C378 | Behavior | Invariant | backbone bounds導出は同一入力で決定的 | `legacy_unclassified` | キャッシュや既存状態でbounds派生が揺れる回帰防止 |
| C379 | Behavior | Boundary | backbone milestone 1の必須出力を固定する | `legacy_unclassified` | milestone 1の要求仕様が曖昧になりdrawキャッシュまで必須扱いされる回帰防止 |
| C380 | Behavior | Boundary | backbone milestone 1はdrawキャッシュを保存する | `legacy_unclassified` | draw V1が生成直後の派生出力から抜ける回帰防止 |
| C381 | SourceGuard | Boundary | backbone milestone 1はrecalc契約を持たない | `source_guard` | recalc不使用が口約束になり旧契約がbackboneに戻る回帰防止 |
| C382 | SourceGuard | Boundary | backbone geomはpipeline上 1層として扱う | `source_guard` | curve/boundsがpipelineの独立工程へ戻り、milestone 1の派生層が細切れになる回帰防止 |
| C383 | SourceGuard | Boundary | backbone drawはpipeline層として扱う | `source_guard` | drawがlayout/geom以外の工程へ混ざる回帰防止 |
| C384 | SourceGuard | Boundary | backbone topology出力は`topo` 1層として扱う | `source_guard` | topology生成結果がgraphやspan専用コンテナへ散って責務が読めなくなる回帰防止 |
| C385 | SourceGuard | Boundary | backbone topology emissionはpoles/bundles/ports/spansに分かれる | `source_guard` | `emit()`がtopology作成の大きな便利箱へ戻る回帰防止 |
| C386 | SourceGuard | Boundary | backboneはlink/pair/open/rowを別概念として持つ | `source_guard` | pair/open/rowが別名の便利箱になり、junction分岐が再発する回帰防止 |
| C387 | SourceGuard | Boundary | backboneはpairsをgraphから一箇所で作る | `source_guard` | pair確定がemit/rules/layoutに分散する回帰防止 |
| C388 | SourceGuard | Invariant | backbone 3点routeはlink/pair/openモデルで生成される | `source_guard` | 折れ線対応がsegment直書きに戻り、pair正本を通らない回帰防止 |
| C389 | SourceGuard | Boundary | backbone row axisはpairs出力が担当箇所になる | `source_guard` | port placementがroute幾何からaxisを再発明する回帰防止 |
| C390 | SourceGuard | Boundary | backboneは不正なpair incident入力を拒否する | `source_guard` | pairが曖昧でもfallbackで補ってdownstreamに流す回帰防止 |
| C391 | SourceGuard | Boundary | backboneはjunction kind labelを持たない | `source_guard` | T/cross/branch分岐が別enumとして戻る回帰防止 |
| C392 | Behavior | Invariant | backboneは3点routeの全spanに必須出力を保存する | `legacy_unclassified` | polyline対応でrules-only/layout-only/geom境界が崩れる回帰防止 |
| C393 | Behavior | Invariant | backbone 3点route出力は決定的 | `legacy_unclassified` | pair/row確定がvector順や既存状態で揺れる回帰防止 |
| C394 | Behavior | Boundary | backbone existing pole nodeはpoleを重複生成しない | `legacy_unclassified` | existing node対応が新規生成と混ざり、同じ位置にpoleを複製する回帰防止 |
| C395 | Behavior | Invariant | backbone `is_new`はpair出力に影響しない | `legacy_unclassified` | existing/newの違いがpair/row axisの意味決定に混ざる回帰防止 |
| C396 | Behavior | Boundary | existing pole continuationは既存spanを意味決定に再入力せず必要な派生だけを更新する | `legacy_unclassified` | existing pole周辺の既存span/layoutをtopology決定へ再入力する、またはPort reflow後に既存派生をstaleのまま残す回帰防止 |
| C397 | Behavior | Boundary | backboneはmissing saved midair node specを拒否する | `legacy_unclassified` | saved graphなしにexisting midair reuseを推測する回帰防止 |
| C398 | Behavior | Boundary | backboneは存在しないexisting pole idを拒否する | `legacy_unclassified` | 不正node idをfallbackで新規pole化する回帰防止 |
| C399 | Behavior | Invariant | backbone existing pole node sequenceは決定的 | `legacy_unclassified` | existing pole nodeの処理が既存状態順やキャッシュで揺れる回帰防止 |
| C400 | Behavior | Invariant | backboneは同じpairs graph上で複数bundleを生成する | `legacy_unclassified` | multiple bundleがv1 fallbackへ戻る回帰防止 |
| C401 | Behavior | Invariant | backbone複数bundle 3点routeは全spanに必須出力を保存する | `legacy_unclassified` | multiple bundle対応でrules/layout/geom境界が崩れる回帰防止 |
| C402 | Behavior | Boundary | backbone bundle specはpair出力に影響しない | `legacy_unclassified` | bundle specがpair/row axisの意味決定へ混ざる回帰防止 |
| C403 | Behavior | Invariant | backbone existing pole nodeとmultiple bundleは併用できる | `legacy_unclassified` | existing nodeとmultiple bundleの組み合わせがv1 fallbackやpole複製へ戻る回帰防止 |
| C404 | Behavior | Boundary | backboneはempty bundle requestを拒否する | `legacy_unclassified` | bundle無し入力を曖昧に成功させる回帰防止 |
| C405 | SourceGuard | Boundary | backbone pairs makeはbundle specを読まない | `source_guard` | bundle数や種類でpair/axisが変わる回帰防止 |
| C406 | Behavior | Invariant | backbone port heightはpole bandを使う | `legacy_unclassified` | 固定高さ 9.32へ戻る回帰防止 |
| C407 | Behavior | Invariant | backbone multiple bundle heightはbundleごとのpole bandを使う | `legacy_unclassified` | 複数bundleを同一高さへ潰す回帰防止 |
| C408 | Behavior | Invariant | backbone existing pole heightはactual pole typeを使う | `legacy_unclassified` | existing poleをrequest type扱いする回帰防止 |
| C409 | Behavior | Boundary | backboneはmissing port bandを拒否する | `legacy_unclassified` | band不在時に固定値やfallbackで補う回帰防止 |
| C410 | SourceGuard | Boundary | placement heightはpairsに影響しない | `source_guard` | height決定がpair/row axis正本へ混ざる回帰防止 |
| C411 | Behavior | Invariant | backbone lateral offsetはrow axis方向へ動く | `legacy_unclassified` | lateral offsetの符号や方向が別resolverで揺れる回帰防止 |
| C412 | Behavior | Invariant | backbone lateral offset符号は決定的 | `legacy_unclassified` | row axis符号が状態順で反転する回帰防止 |
| C413 | SourceGuard | Boundary | backbone lateral offsetはpairsに影響しない | `source_guard` | placement offsetがpair/row axis正本へ混ざる回帰防止 |
| C414 | Behavior | Boundary | backboneは 2点route + 1 avoid pointのsimple detourを生成する | `legacy_unclassified` | route上avoid pointが無条件unsupportedになりviewer操作が止まる、またはdownstreamが幾何fallbackで避ける回帰防止 |
| C415 | SourceGuard | Boundary | backboneは空のlevels層を持たない | `source_guard` | 将来用の空箱が便利箱化する回帰防止 |
| C416 | Behavior | Invariant | backbone `kNotPresent` node modeはno-op | `legacy_unclassified` | no-op入力が生成意味へ混ざる回帰防止 |
| C417 | Behavior | Boundary | backboneはtarget rowが無い`kPassThrough` node modeを拒否する | `legacy_unclassified` | pass-through意味決定を未仕様のまま受ける回帰防止 |
| C418 | Behavior | Boundary | backboneはrequest外bundleのnode modeを拒否する | `legacy_unclassified` | no-opをbroken入力ignoreと誤解する回帰防止 |
| C419 | Behavior | Boundary | backboneは範囲外node mode point indexを拒否する | `legacy_unclassified` | 入力point indexと内部graph node idを混同する回帰防止 |
| C420 | SourceGuard | Boundary | backbone node modeはpairsに影響しない | `source_guard` | node modeがpair/row axis正本へ混ざる回帰防止 |
| C421 | SourceGuard | Boundary | backbone topo rowはrow sourceを運ぶ | `source_guard` | topo.rowsがport実体だけを持ち、後段がrow意味を推測し始める回帰防止 |
| C422 | SourceGuard | Boundary | backbone rulesはtopoとgroupsを読む | `source_guard` | rulesがpairs.linksのendpoint row mappingを再び読む、またはlowering placementをintent直読へ戻す回帰防止 |
| C423 | SourceGuard | Boundary | backbone topo spanはendpoint row indexを運ぶ | `source_guard` | span endpoint row対応がrules側へ漏れる回帰防止 |
| C424 | Behavior | Boundary | backboneはsaved backbone graph nodes/edgesを保存する | `legacy_unclassified` | Pole/Port/Spanからbackboneを復元するだけの構造へ戻る回帰防止 |
| C425 | Behavior | Boundary | backbone backbone edgeは複数spanをedge bundle経由で持てる | `legacy_unclassified` | pole間multiple spansをedge bundle正本へ束ねられない回帰防止 |
| C426 | Behavior | Boundary | backbone existing poleはsaved graph nodeを再利用する | `legacy_unclassified` | existing pole接続でgraph nodeが重複する回帰防止 |
| C427 | Behavior | Boundary | backbone graph indexはgraphと出力を結ぶ | `legacy_unclassified` | 変更対象収集の土台になるindexが保存されない回帰防止 |
| C428 | Behavior | Boundary | backbone pole frontierはsaved graph incidentを集める | `legacy_unclassified` | 変更対象を全span scanやflag scanで集める回帰防止 |
| C429 | Behavior | Boundary | backbone span frontierはedge bundle配下spanを集める | `legacy_unclassified` | Spanからgraph edgeを復元せずedge bundleを読む境界の回帰防止 |
| C430 | SourceGuard | Boundary | backbone frontierはsaved graph indexを読む | `source_guard` | frontierが旧span-derived topology scanへ戻る回帰防止 |
| C431 | Behavior | Boundary | backbone edge bundleはsaved backbone unit | `legacy_unclassified` | backbone graphがspan直下ではなくedge-bundle正本を持つ境界の回帰防止 |
| C432 | Behavior | Boundary | backbone multiple bundleは複数edge_bundleを作る | `legacy_unclassified` | bundleをbackbone attachmentとして保存できない回帰防止 |
| C433 | Behavior | Boundary | backboneは同じpole pairのedgeを再利用する | `legacy_unclassified` | 同じ物理segmentが重複edgeになる回帰防止 |
| C434 | Behavior | Boundary | backbone reverse duplicate same bundleはrejectする | `legacy_unclassified` | reverse duplicateがspanを無制限に増やす回帰防止 |
| C435 | Behavior | Boundary | backbone edgeメタデータはduplicate rejectで上書きされない | `legacy_unclassified` | duplicate reject時にedge正本を後書きする回帰防止 |
| C436 | Behavior | Boundary | backbone frontierはedge_bundle経由で読む | `legacy_unclassified` | frontierがedge-level span直読みへ戻る回帰防止 |
| C437 | SourceGuard | Boundary | backbone layout保存はdirect store | `source_guard` | backbone layout保存が旧support layout/recalc経路へ戻る回帰防止 |
| C438 | Behavior | Boundary | backbone direct layout保存は追加入力を要求しない | `legacy_unclassified` | direct layout保存がseed/authority契約を生やす回帰防止 |
| C439 | SourceGuard | Boundary | backbone sourceはsupport layout entrypointを呼ばない | `source_guard` | backboneが旧support layout entrypointへ戻る回帰防止 |
| C440 | SourceGuard | Boundary | backboneは退役担当箇所以外でauthoritative backboneを直接読まない | `source_guard` | saved graph担当箇所境界が崩れ、退役以外の段が内部vectorを読む回帰防止 |
| C441 | SourceGuard | Boundary | backbone edge保存はsaved refを返す | `source_guard` | backboneがedge idだけ受け取りgraph vector検索へ戻る回帰防止 |
| C442 | SourceGuard | Boundary | backbone edge_forwardはsaved refから決める | `source_guard` | edge direction判定がCoreState担当箇所境界を越える回帰防止 |
| C443 | Behavior | Boundary | backbone edge resolution境界はduplicate reject後も維持される | `legacy_unclassified` | API境界変更でduplicate policyが抜ける回帰防止 |
| C444 | SourceGuard | Boundary | backbone layout sourceはneutral型を使う | `source_guard` | backbone layout層が旧support layout型名へ戻る回帰防止 |
| C445 | SourceGuard | Boundary | backbone layout保存口はneutral入口を受ける | `source_guard` | direct layout保存口が旧support layout signature / recalc経路へ戻る回帰防止 |
| C446 | Behavior | Boundary | backbone neutral layout境界は生成結果を変えない | `legacy_unclassified` | layout型名変更でbackbone derived outputsやno-authority境界が崩れる回帰防止 |
| C447 | Behavior | Boundary | backbone layout viewはneutral layoutを読む | `legacy_unclassified` | backboneの正規layout観測口が旧projection APIに戻る回帰防止 |
| C448 | SourceGuard | Boundary | backboneテストはneutral layout readを使う | `source_guard` | acceptanceが旧projection観測口を正本扱いし続ける回帰防止 |
| C449 | SourceGuard | Boundary | backbone neutral layout readはauthorityを露出しない | `source_guard` | neutral read APIが旧authority契約を再公開する回帰防止 |
| C450 | Behavior | Boundary | backbone layout状態はneutral | `legacy_unclassified` | no-authority観測が旧契約APIに戻る回帰防止 |
| C451 | SourceGuard | Boundary | backboneテストはsupport layout契約を読まない | `source_guard` | backbone acceptanceが旧契約観測口を正本扱いする回帰防止 |
| C452 | SourceGuard | Boundary | backbone layout状態は旧契約名を露出しない | `source_guard` | neutral状態viewが旧契約語を再公開する回帰防止 |
| C453 | SourceGuard | Boundary | backbone layout状態はseed pathを読まない | `source_guard` | neutral状態viewが旧seed/contract経路へ戻る回帰防止 |
| C454 | SourceGuard | Boundary | backboneキャッシュ状態はspan layoutキャッシュ名を使う | `source_guard` | neutralキャッシュ担当箇所が旧support layoutフィールド名へ戻る回帰防止 |
| C455 | SourceGuard | Boundary | backbone neutral layout APIはspan layoutキャッシュを読む | `source_guard` | neutral APIが旧キャッシュ担当箇所名へ戻る回帰防止 |
| C456 | SourceGuard | Boundary | backbone sourceは旧layoutキャッシュ名を避ける | `source_guard` | backbone production pathが旧layout cache/read名へ戻る回帰防止 |
| C457 | Behavior | Boundary | backbone layoutキャッシュ境界は生成結果を変えない | `legacy_unclassified` | キャッシュ担当箇所renameでbackbone derived outputsやno-input境界が崩れる回帰防止 |
| C458 | Behavior | Boundary | backboneはexisting BにB-Dを追加できる | `legacy_unclassified` | existing junction contextを読めず新規branchがunsupportedに戻る回帰防止 |
| C459 | Behavior | Boundary | backboneはexisting B経由のD-B-Eをkindなしで追加できる | `legacy_unclassified` | cross相当入力をT/cross kind分岐なしで扱えない回帰防止 |
| C460 | Behavior | Boundary | backbone context linkはemitされない | `legacy_unclassified` | saved graph contextをtopology生成対象として誤emitする回帰防止 |
| C461 | Behavior | Boundary | backbone same-edge requestはduplicate contextをskipする | `legacy_unclassified` | same edge追加がcontext duplicateでambiguous/rejectされる回帰防止 |
| C462 | SourceGuard | Boundary | backbone existing contextはjunction kind labelを追加しない | `source_guard` | existing junction sliceがT/cross/branch分岐として再発する回帰防止 |
| C463 | Behavior | Boundary | backbone duplicate same edge + bundleはrejectする | `legacy_unclassified` | 同一edge + bundleの追加生成が無制限にspanを増やす回帰防止 |
| C464 | Behavior | Boundary | backbone different bundleは同じedgeに追加できる | `legacy_unclassified` | duplicate policyが別bundle追加まで拒否する回帰防止 |
| C465 | SourceGuard | Boundary | backbone duplicate policyはexisting spanを読まない | `source_guard` | duplicate検出がspan resolution/差分補完へ滑る回帰防止 |
| C466 | Behavior | Boundary | backbone duplicate rejectは状態を変えない | `legacy_unclassified` | unsupported pathが途中までtopology/cacheを生成する回帰防止 |
| C467 | Behavior | Boundary | backbone row-port bindingを保存する | `legacy_unclassified` | 将来port resolutionが座標推測に戻る回帰防止 |
| C468 | Behavior | Boundary | backbone existing context追加でもnew row-port bindingだけ保存する | `legacy_unclassified` | context rowをmaterialized portと誤って結びつける回帰防止 |
| C469 | Behavior | Boundary | backbone row-port duplicateはresolveせずrejectする | `legacy_unclassified` | existing port resolution仕様なしにbindingを重複作成する回帰防止 |
| C470 | SourceGuard | Boundary | backbone row-port識別情報は位置近似を使わない | `source_guard` | port識別情報が座標近似やlayout依存に戻る回帰防止 |
| C471 | Behavior | Boundary | compatible pairもedge endpoint別Port bindingを持つ | `legacy_unclassified` | shared Port IDを接続正本へ戻す回帰防止 |
| C472 | Behavior | Boundary | backbone port resolutionはsaved bindingを要求する | `legacy_unclassified` | 座標近似でv1由来portを拾う回帰防止 |
| C473 | Behavior | Boundary | 隣接spanは各edge endpoint Portを参照する | `legacy_unclassified` | Spanがpeer edgeのPortを共有する回帰防止 |
| C474 | SourceGuard | Boundary | backbone port resolutionはambiguous bindingをrejectする | `source_guard` | 曖昧なresolutionを恣意的に選ぶ回帰防止 |
| C475 | SourceGuard | Boundary | backbone port resolutionはexisting layoutを読まない | `source_guard` | port resolutionがlayout/materialization/位置推測に戻る回帰防止 |
| C476 | Behavior | Invariant | backbone branch rowはkindなしでPortの論理anchorとHV bundle水平中心を維持し、resolved support levelで最終endpointだけを分離する | `legacy_unclassified` | branch相当入力で物理rowが潰れる、Port Zを別decisionで動かす、またはbundle中心を横へ動かす回帰防止 |
| C477 | SourceGuard | Boundary | backbone cross rowはkindなしでcontinuityから導出される | `source_guard` | cross相当入力で未接続openが残る、またはcross enumが再発する回帰防止 |
| C478 | Behavior | Invariant | backbone row height separationはdeterministic | `legacy_unclassified` | row height separationがunordered contextや生成順で揺れる回帰防止 |
| C479 | SourceGuard | Boundary | backbone row height separationはpairsを変えない | `source_guard` | port placement policyがpair/open/row source決定へ混ざる回帰防止 |
| C480 | Behavior | Invariant | backbone context rowは既存Portを動かさずcontext側port/spanを再materializeせず、branch-down無効bundleの新rowへsupport offsetを混入しない | `legacy_unclassified` | context-only rowを新規port/spanとして生成する、既存Portをreflowする、またはflag無効bundleを高さ分離する回帰防止 |
| C481 | Behavior | Boundary | backbone pass-through modeは限定範囲で受ける | `legacy_unclassified` | node modeを無制限に受けてpair/row決定へ混ぜる回帰防止 |
| C482 | Behavior | Boundary | backbone pass-throughはexplicit intentを保存する | `legacy_unclassified` | pass-through指定が保存されず後段推測へ戻る回帰防止 |
| C483 | SourceGuard | Boundary | backbone pass-through ambiguous targetはrejectする | `source_guard` | 幾何や順序で曖昧なtarget rowを選ぶ回帰防止 |
| C484 | Behavior | Boundary | backbone lowering drawはlayoutだけを読む | `legacy_unclassified` | drawがlowering/topologyを再判断する回帰防止 |
| C485 | SourceGuard | Boundary | backbone lowering intentはexisting spanを読まない | `source_guard` | intent決定がv1/recalc/materializationへ戻る回帰防止 |
| C486 | Behavior | Invariant | backbone pass-through intentはdeterministic | `legacy_unclassified` | pass-through intentがsaved graph traversal順で揺れる回帰防止 |
| C487 | Behavior | Boundary | backbone port resolutionはbundle-compatible scopeを要求する | `legacy_unclassified` | row_key/laneだけで異種bundle portを共有する回帰防止 |
| C488 | Behavior | Boundary | compatible pair bindingはscopeを維持して別Portへ分かれる | `legacy_unclassified` | compatibilityをPort共有と混同する回帰防止 |
| C489 | Behavior | Boundary | generated Portのbinding indexはedge endpoint専用 | `legacy_unclassified` | 1 Portへ複数edge bindingを再導入する回帰防止 |
| C490 | Behavior | Boundary | backbone duplicate same edge/bundle/laneは重複生成しない | `legacy_unclassified` | duplicate requestがspan/portを増やす回帰防止 |
| C491 | Behavior | Invariant | backbone branch lowering v1はgeomに反映される | `legacy_unclassified` | lowering intentが保存だけでgeomに反映されない回帰防止 |
| C492 | Behavior | Invariant | backbone cross lowering v1はnew linksだけに反映される | `legacy_unclassified` | context linkを再生成する、またはloweringをcontext出力へ混ぜる回帰防止 |
| C493 | SourceGuard | Boundary | backbone pass-throughはpair/openを変えない | `source_guard` | pass-throughがconnectivity authorityへ混ざる回帰防止 |
| C494 | Behavior | Boundary | backbone lowering v1 drawは再判断しない | `legacy_unclassified` | drawがlowering intentを新規判断する回帰防止 |
| C495 | SourceGuard | Boundary | backbone lowering v1はexisting spanを読まない | `source_guard` | loweringがv1/recalc/materializationへ戻る回帰防止 |
| C496 | Behavior | Invariant | backbone junction v1はdeterministic | `legacy_unclassified` | existing junction出力がtraversal/orderで揺れる回帰防止 |
| C497 | Behavior | Invariant | backbone context rowはsupport-level constraintに使い未存在portをmaterializeしない | `legacy_unclassified` | context-only rowを新規port/spanとして生成する、またはflag無効bundleのPort anchorを動かす回帰防止 |
| C498 | Behavior | Boundary | backbone saved graphはtopology authorityのまま | `legacy_unclassified` | topologyをspans/layout/seedから復元する構造へ戻る回帰防止 |
| C499 | Behavior | Boundary | backbone context linkはsave targetではない | `legacy_unclassified` | context linkが`save_backbone_edge`対象になりnew linkと混ざる回帰防止 |
| C500 | SourceGuard | Boundary | backbone context linkはsaved edge refを要求する | `source_guard` | context linkをnode pair / 幾何 / span/layoutから推測して保存する回帰防止 |
| C501 | SourceGuard | Boundary | backbone Gate 3契約はsave_graphでも維持される | `source_guard` | emitでは分離していてもsave境界でcontext/newが混ざる回帰防止 |
| C502 | Behavior | Boundary | backbone span bindingはlane識別情報を保存し、edge bundleをpair/id indexで参照する | `legacy_unclassified` | edge_bundle.span_idsだけでlaneが分からない、またはsave_graphがbundle/spanごとに全edge_bundlesを線形探索する回帰防止 |
| C503 | SourceGuard | Boundary | backbone duplicate span binding laneはrejectする | `source_guard` | same edge_bundle + laneのspanが複数保存される回帰防止 |
| C504 | SourceGuard | Boundary | backbone span resolutionはgeometry/layoutを読まない | `source_guard` | span resolutionがcurve類似・layout・位置推測へ滑る回帰防止 |
| C505 | SourceGuard | Boundary | backbone save_graphはspan binding失敗を伝播する | `source_guard` | duplicate span bindingを保存境界で黙殺する回帰防止 |
| C506 | SourceGuard | Boundary | backbone support groupはplacement layer | `source_guard` | support groupが便利箱化してtopology/visual責務を持つ回帰防止 |
| C507 | SourceGuard | Boundary | backbone support groupはintent後に作る | `source_guard` | lowering placementがintent/rules/layoutに分散する回帰防止 |
| C508 | Behavior | Boundary | backbone support groupがlowered rulesを駆動する | `legacy_unclassified` | loweringがgroupを経由せずrulesに直書きされる回帰防止 |
| C509 | SourceGuard | Boundary | backbone support groupはvisual語を持たない | `source_guard` | support groupがdraw/support visualの責務を先取りする回帰防止 |
| C510 | SourceGuard | Boundary | backbone layoutはfixture placement planを読む | `source_guard` | layoutがplacement planを迂回する、またはsocket解決後にloweringを二重適用する回帰防止 |
| C511 | Behavior | Boundary | backbone drawはgeomから保存される | `legacy_unclassified` | drawがgeom以外の経路や未保存状態へ戻る回帰防止 |
| C512 | SourceGuard | Boundary | backbone drawはtopologyを読まない | `source_guard` | drawがtopology / pair / row / loweringを再判断する回帰防止 |
| C513 | Behavior | Boundary | backbone lowered layoutはobsolete SupportArmを出さない | `legacy_unclassified` | loweringをmodel/socket配置で消費した後に旧SupportArmが復活する回帰防止 |
| C514 | SourceGuard | Boundary | backbone draw saveはdirect store | `source_guard` | draw保存がrecalc/rebuild経路へ戻る回帰防止 |
| C515 | Behavior | Boundary | backboneはsaved graphの無いexisting poleを拒否する | `legacy_unclassified` | v1/手動sceneを暗黙migrationする回帰防止 |
| C516 | Behavior | Boundary | backbone生成済みpoleはexisting nodeとして使える | `legacy_unclassified` | M29 gateがbackbone正規saved graph contextまで拒否する回帰防止 |
| C517 | SourceGuard | Boundary | backbone migration gateはoutputsから推測しない | `source_guard` | span/layout/seed/curve/positionからgraphを復元する回帰防止 |
| C518 | Behavior | Boundary | backbone lowered layoutはsupport_worldとendpoint_worldを最終fixture高さへ一致させる | `legacy_unclassified` | モデルを元高さに残し線端だけを下げる旧契約の復活を防ぐ |
| C519 | SourceGuard | Boundary | backbone drawはlowering placeholderを出さない | `source_guard` | SupportArmで旧support/endpoint差を埋める経路の復活を防ぐ |
| C520 | SourceGuard | Boundary | backbone duplicate span binding preflightはemit前に走る | `source_guard` | duplicate rejectが生成後rollbackや保存時不変条件だけに依存する回帰防止 |
| C521 | SourceGuard | Boundary | backbone context linkはsaved dirを保つ | `source_guard` | context linkの保存済み向きをconnectivity段でnode幾何から再生成する回帰防止 |
| C522 | SourceGuard | Boundary | backbone supported scopeは文書化されている | `source_guard` | supported scopeが暗黙化し、実装判断が場当たりになる回帰防止 |
| C523 | SourceGuard | Boundary | backbone scope gateはentrypointと一致する | `source_guard` | supported scope freezeと入口実装がズレる回帰防止 |
| C524 | SourceGuard | Boundary | backbone simple lineシナリオはmainline出力とauthorityを見る | `source_guard` | 単純線が結果だけ通り、実際はv1/recalcに戻る回帰防止 |
| C525 | Behavior | Boundary | backbone polylineシナリオはconnectivity一点決定を守る | `legacy_unclassified` | polyline対応でdownstreamがconnectivityを再判断する回帰防止 |
| C526 | Behavior | Boundary | backbone multiple bundleシナリオはconnectivityを共有する | `legacy_unclassified` | bundle specごとにpair/open/rowを分岐させる回帰防止 |
| C527 | Behavior | Boundary | backbone existing pole continuationシナリオはsaved graphだけを見る | `legacy_unclassified` | existing接続がsaved graphではなくspan/layout/seed推測へ戻る回帰防止 |
| C528 | Behavior | Boundary | backbone branchシナリオはnew linkだけemitする | `legacy_unclassified` | context linkを生成・保存対象に混ぜる回帰防止 |
| C529 | SourceGuard | Boundary | backbone crossシナリオはkind labelなしで通る | `source_guard` | T/cross/branch enumが再発する回帰防止 |
| C530 | Behavior | Boundary | backbone same-edge different-bundleシナリオはedgeを共有する | `legacy_unclassified` | 同じedgeのbundle追加がduplicate扱いになる、または異種portを共有する回帰防止 |
| C531 | Behavior | Boundary | backbone duplicateシナリオは状態変更なしでrejectする | `legacy_unclassified` | duplicate rejectが生成後rollback前提になる回帰防止 |
| C532 | Behavior | Boundary | backbone pass-through loweringシナリオは利用側chainを守る | `legacy_unclassified` | pass-throughがconnectivityを変える、またはdrawがloweringを再判断する回帰防止 |
| C533 | SourceGuard | Boundary | backbone build状態変更orderは固定されている | `source_guard` | 状態変更前に分かる失敗がemit後へ戻る回帰防止 |
| C534 | Behavior | Boundary | backbone invalid inputsはemit前に止まる | `legacy_unclassified` | unsupported入力が途中までobjectを生成する回帰防止 |
| C535 | SourceGuard | Boundary | backbone duplicate preflightは状態変更境界 | `source_guard` | duplicate rejectが保存時不変条件やrollbackに依存する回帰防止 |
| C536 | Behavior | Boundary | backbone draw利用側出力は最小 | `legacy_unclassified` | drawがmodel/socketで解決済みのloweringを再解釈する回帰防止 |
| C537 | SourceGuard | Boundary | backbone draw sourceはdecision入力を読まない | `source_guard` | drawがtopology/connectivity/loweringを再判断する回帰防止 |
| C538 | SourceGuard | Boundary | backbone viewer depsはcore draw gateではない | `source_guard` | viewer環境不備でcore backbone acceptanceを止める回帰防止 |
| C539 | SourceGuard | Boundary | backbone supported requestはsaved graph出力を作る | `source_guard` | supported scopeがv1 fallbackや未保存graphに戻る回帰防止 |
| C540 | Behavior | Boundary | backbone unsupported requestはv1出力を作らない | `legacy_unclassified` | unsupportedがv1-style出力を途中生成する回帰防止 |
| C541 | SourceGuard | Boundary | backbone manual existing pole without graphはgate reject | `source_guard` | v1/manual sceneを暗黙migrationする回帰防止 |
| C542 | Behavior | Boundary | backbone usable mainline architecture auditはpass | `legacy_unclassified` | usable mainline化の境界がまた曖昧になる回帰防止 |
| C543 | Behavior | Boundary | backboneは新規route interiorの`kPassThrough`をsupportedシナリオとして通す | `legacy_unclassified` | C番号ではなくsupportedシナリオが増えたことを固定し、saved graph限定gateへ戻る回帰防止 |
| C544 | Behavior | Boundary | backboneはgenerated pole pinningをsupportedシナリオとして反映する | `legacy_unclassified` | pole placement入力を無視する、またはexisting poleを副作用でmanual化する回帰防止 |
| C545 | Behavior | Boundary | backboneは`interval_m`で中間poleを生成する | `legacy_unclassified` | interval入力を無視し続ける、またはauto interval nodeをmanual vertex扱いする回帰防止 |
| C546 | Behavior | Boundary | backboneはexplicit new-pole node specをsupportedシナリオとして受ける | `legacy_unclassified` | explicit new pole指定を壊れたexisting idと誤判定する、または不正idを新規pole fallbackする回帰防止 |
| C547 | Behavior | Boundary | backboneはfixedテンプレートのexact count指定をsupportedシナリオとして受ける | `legacy_unclassified` | UIが固定数を明示しただけでrejectする、またはmismatched countをlane数変更として受ける回帰防止 |
| C548 | Behavior | Boundary | backboneはavoid radius単独指定をno-op supportedシナリオとして受ける | `legacy_unclassified` | radiusだけのUI入力を不必要にrejectする回帰防止 |
| C549 | Behavior | Boundary | backboneはrangeテンプレートのexplicit countをsupportedシナリオとして受ける | `legacy_unclassified` | range countを無視する、または範囲外countをそのまま生成する回帰防止 |
| C550 | Behavior | Boundary | backboneはgenerated poleのtangent hintをsupportedシナリオとして反映する | `legacy_unclassified` | tangent hintを無視する、またはtopology/connectivity/row placementに混ぜる回帰防止 |
| C551 | Behavior | Boundary | backboneはmissing pole typeをbundleテンプレートから解決する | `legacy_unclassified` | pole type未指定UI入力を不必要にrejectする、または混在bundleからpole typeを推測する回帰防止 |
| C552 | Behavior | Boundary | backboneはzero-radius avoid pointsをno-op supportedシナリオとして受ける | `legacy_unclassified` | disabled avoid UI入力を不必要にrejectする回帰防止 |
| C553 | Behavior | Boundary | backboneはnew midair route pointをsupportedシナリオとして受ける | `legacy_unclassified` | midair入力をv1 fallbackへ戻す、またはpoleとして誤生成する回帰防止 |
| C554 | Behavior | Boundary | backboneはexisting saved midair route pointをsupportedシナリオとして受ける | `legacy_unclassified` | existing midairをspan/layout/座標近似から推測する、またはpoleとして誤生成する回帰防止 |
| C555 | Behavior | Boundary | backboneはnew building route pointをsupportedシナリオとして受ける | `legacy_unclassified` | building入力をv1 fallbackへ戻す、またはpoleとして誤生成する回帰防止 |
| C556 | Behavior | Boundary | backboneはbuilding pickをnew building route pointとして受ける | `legacy_unclassified` | viewer building pickがPole扱いやsaved node id推測へ戻る回帰防止 |
| C557 | Behavior | Boundary | backboneはobject idの無いbuilding pickを受ける | `legacy_unclassified` | viewer raycastがstable building idを持たない場合に不必要にrejectする回帰防止 |
| C558 | Behavior | Boundary | backboneはground pickをnew ground route pointとして受ける | `legacy_unclassified` | ground pickをv1 fallbackへ戻す、またはattachment/anchor仕様なしに推測生成する回帰防止 |
| C559 | Behavior | Boundary | backboneはrouteと交差しないpositive avoidをno-opとして受ける | `legacy_unclassified` | clear avoidが不要にrouteを変える回帰防止 |
| C560 | Behavior | Boundary | backboneはselected bundle未指定のsegment dry-run pickをmidair route pointとして受ける | `legacy_unclassified` | UIのpath point解決をbundle選択前に不必要に止める、またはspan/layout/seedからtopologyを推測する回帰防止 |
| C561 | Behavior | Boundary | backboneはselected bundle未指定の既定segment pickをownerless midair route pointとして受ける | `legacy_unclassified` | UIがbundle選択前の既定pickをvirtual node idとして返し、backboneがsaved node不在でrejectする回帰防止 |
| C562 | Behavior | Boundary | backboneはsaved midair node pickから延長できる | `legacy_unclassified` | saved midair nodeをPoleと誤解してbackbone extensionがsaved graph missingで止まる回帰防止 |
| C563 | Behavior | Boundary | backboneはsaved ownerless span endpointへsegment pickからsnapできる | `legacy_unclassified` | viewer segment pick payloadがendpoint idsを持たない場合にownerless endpoint snapが効かず、backbone extensionが止まる回帰防止 |
| C564 | Behavior | Boundary | backboneはselected bundle付きsegment pickのtransient midair nodeから生成できる | `legacy_unclassified` | selected bundle指定時だけvirtual node idが返り、backboneがsaved node不在で止まる回帰防止 |
| C565 | Behavior | Boundary | backboneはmixed selected midair branchで許可bundleだけ生成する | `legacy_unclassified` | 禁止bundleを巻き込んで生成する、またはmixed request全体をrejectする回帰防止 |
| C566 | Behavior | Boundary | backboneは全bundle disallowedのselected midair branchをno-opにする | `legacy_unclassified` | 禁止bundleだけのrequestが途中までpole/port/spanを生成する回帰防止 |
| C567 | Behavior | Boundary | backboneはsegment pick midair branchでsource projectionを使う | `legacy_unclassified` | viewer hit点のz=0をそのまま使い、中間分岐が地面から出る回帰防止 |
| C568 | Behavior | Boundary | source-edge midair branchはcurrent source curve projectionを使う | `legacy_unclassified` | source edge上のbranchが既存main contextを失う、支持点高さを接続点高さへ混ぜる、またはendpointがsource curveから離れる回帰防止 |
| C569 | Behavior | Boundary | backbone renderキャッシュはcableテンプレートappearanceを使う | `legacy_unclassified` | backboneのrenderキャッシュがdefault radius/colorのままになり、テンプレート表示属性が初期生成に反映されない回帰防止 |
| C570 | Behavior | Boundary | backbone loweringはSupportArm visualを出さない | `legacy_unclassified` | 旧SupportArmがvisual設定経由で復活する回帰防止 |
| C571 | Behavior | Boundary | insulator visual disabledでもloweringは最終socketで解決する | `legacy_unclassified` | insulator visual設定を無効化してもlowering/layout/geomを壊す、または旧placeholderを復活させる回帰防止 |
| C572 | SourceGuard | Boundary | obsolete SupportArm設定はproduction surfaceへ存在しない | `source_guard` | dead compatibilityとして旧SupportArm familyを残す回帰防止 |
| C573 | Behavior | Boundary | backbone saved context nodeはsupportメタデータを保持する | `legacy_unclassified` | ownerless/building/ground context nodeがsupport/sourceメタデータを失い、後続のcontext/placement判断がpole前提に痩せる回帰防止 |
| C574 | Behavior | Boundary | backboneはexisting edgeに別bundleをpass-through mode付きで追加できる | `legacy_unclassified` | same-edge bundle追加でnode modeがduplicate edge/span判定やcontext解釈を壊す回帰防止 |
| C575 | Behavior | Boundary | backboneはstale segment-pick midair duplicate requestを状態変更前にrejectする | `legacy_unclassified` | stale pending idが新しいownerless nodeとspanを再生成し、duplicate policyをすり抜ける回帰防止 |
| C576 | Behavior | Boundary | backbone ownerless-only multiple bundle routeはpole typeを要求しない | `legacy_unclassified` | ownerless-only routeが不要なpole type ambiguityに引きずられてunsupportedになる回帰防止 |
| C577 | Behavior | Boundary | backbone missing port bandはtopology状態変更前にrejectする | `legacy_unclassified` | missing bandがemit_portsまで進んでpartial topologyを残す回帰防止 |
| C578 | Behavior | Boundary | backbone segment pick midair branchはpass-throughを受ける | `legacy_unclassified` | segment pick直後のownerless branchがsaved node未作成という理由だけでpass-through unsupportedになる回帰防止 |
| C579 | Behavior | Boundary | backboneはpolyline上の単一avoid detourをsupportedシナリオとして受ける | `legacy_unclassified` | avoid対応が 2点lineに限定され、polyline操作が不要にunsupportedになる回帰防止 |
| C580 | Behavior | Boundary | backboneはintervalとpositive avoid detourをsegment orderで生成する | `legacy_unclassified` | interval pointとdetour pointの順序が逆転し、routeが折り返す回帰防止 |
| C581 | Behavior | Boundary | backboneはinactive bundleのmissing port bandを無視する | `legacy_unclassified` | 生成しないbundleのband不足でsupported requestが不自然にrejectされる回帰防止 |
| C582 | Behavior | Boundary | backboneは同一segment上の複数avoid pointをsupportedシナリオとして受ける | `legacy_unclassified` | avoid pointが複数あるだけでviewer操作が不要にunsupportedになる回帰防止 |
| C583 | Behavior | Boundary | backboneは複数segment上のavoid pointをsupportedシナリオとして受ける | `legacy_unclassified` | polylineの複数segmentにavoid pointがあるだけでviewer操作が不要にunsupportedになる回帰防止 |
| C584 | Behavior | Boundary | backboneはownerless routeのinterval挿入点をownerless nodeとして扱う | `legacy_unclassified` | ownerless-only routeの中間interval pointが暗黙poleになり、不要なpole type要求やpole生成が起きる回帰防止 |
| C585 | Behavior | Boundary | backboneは重複avoid pointを1つのdetour nodeに畳む | `legacy_unclassified` | duplicate UI payloadが同一点detour nodeを複数作り、zero-length linkでsupported操作が失敗する回帰防止 |
| C586 | Behavior | Boundary | backboneはinterval pointとavoid detourが同じsource位置にある場合detourを優先する | `legacy_unclassified` | interval pointがobstacle中心に残り、avoid detourを作ってもroute_clearでrejectされる回帰防止 |
| C587 | Behavior | Boundary | backboneはselected bundle未指定でも明示midair node作成を受ける | `legacy_unclassified` | selected bundle未指定のviewer操作がinvalid node idになり、後続backbone node_specsへ渡せない回帰防止 |
| C588 | Behavior | Boundary | backboneはinternal corner上のavoid pointをdeterministic detourとして受ける | `legacy_unclassified` | corner上のobstacleがsegment t=0/1としてdetour対象から外れ、route_clearでsupported viewer操作が止まる回帰防止 |
| C589 | Behavior | Boundary | backboneはselected-bundle midair policyで未選択bundleを生成しない | `legacy_unclassified` | viewerの選択bundleから作ったmidair branchが、未選択bundleまで一緒に生成する回帰防止 |
| C590 | Behavior | Boundary | backboneはselected policyでinactiveになったbundleのpass-throughをno-op前に拒否する | `legacy_unclassified` | viewerの選択範囲で生成対象外になったbundleのnode modeが黙って無視される回帰防止 |
| C591 | Behavior | Boundary | backboneはsaved selected midair nodeの継続生成でrequest bundleを使う | `legacy_unclassified` | 中間分岐のselected policyが保存後のcontinuationまで絞り、テンプレート指定どおりの延長を妨げる回帰防止 |
| C592 | Behavior | Boundary | backboneはsaved selected midair nodeのreverse continuationでもrequest bundleを使う | `legacy_unclassified` | reverse continuationだけsaved policyが残り、テンプレート指定どおりの延長を妨げる回帰防止 |
| C593 | Behavior | Boundary | backboneはsaved selected midair nodeでrequest pass-throughを受ける | `legacy_unclassified` | saved node化後の古いselected policyが明示requestを拒否する回帰防止 |
| C594 | Behavior | Boundary | backboneはroute endpointと完全一致するavoid pointをno-opとして扱う | `legacy_unclassified` | endpointと同一点のUI payloadがroute_clearで不要にunsupportedになる回帰防止 |
| C595 | Behavior | Boundary | backboneは明示existing supportと完全一致するavoid pointをno-opとして扱う | `legacy_unclassified` | UI payloadがsupport自体をavoid pointに含めた時に、既存supportをdetourしたりunsupportedへ落ちる回帰防止 |
| C596 | Behavior | Boundary | backboneは明示new supportと完全一致するavoid pointをno-opとして扱う | `legacy_unclassified` | clicked supportと同じavoid payloadで明示supportがdetourされたりunsupportedへ落ちる回帰防止 |
| C597 | Behavior | Boundary | backboneはselected building pickで選択bundleだけを生成する | `legacy_unclassified` | viewerの選択bundleからbuilding route pointを作った時に、未選択bundleまで一緒に生成する回帰防止 |
| C598 | Behavior | Boundary | backboneはselected saved building node pickで選択bundleだけを生成する | `legacy_unclassified` | viewerのnode pick経由だけselected bundle policyが失われ、未選択bundleまで生成する回帰防止 |
| C599 | Behavior | Boundary | backboneはselected saved building nodeのbundle policyをbranch後も保持する | `legacy_unclassified` | selected saved nodeの初回branchだけ選択bundleになり、後続continuationで未選択bundleが復活する回帰防止 |
| C600 | Behavior | Boundary | pole id pickはsaved node idに正規化しselected policyをpoleへ刻印しない | `legacy_unclassified` | pole pickにmidair/selected policyを刻印し、HVなど未選択扱いの欠落を伝播させる回帰防止 |
| C601 | Behavior | Boundary | backboneはcontext-only nodeのselected policyで新規route bundleを絞らない | `legacy_unclassified` | context入力だけのselected policyが後続routeのbundle生成を誤って削る回帰防止 |
| C602 | Behavior | Boundary | saved bandを持たないpole type適用は状態変更前に拒否し、無関係routeは影響を受けない | `legacy_unclassified` | binding band不足をfallbackで補完する回帰と、拒否が無関係routeを汚す回帰を防止 |
| C603 | Behavior | Boundary | backboneはcontext nodeでgenerated endpoint pole yawを曲げない | `legacy_unclassified` | context nodeがroute nodeの次要素として扱われ、新規endpoint poleの向きがbranch方向から外れる回帰防止 |
| C604 | Behavior | Boundary | backboneは大きいavoid半径でもdeterministic detourを半径外へ出す | `legacy_unclassified` | detour point自体は動いたが折れ線segmentがまだobstacle半径内を通り、後段preflightでsupported入力がrejectされる回帰防止 |
| C605 | Behavior | Boundary | backboneのroute queryはsaved ownerless backbone graphを読む | `legacy_unclassified` | ownerless endpointがpole idを持たないため、span-derived SavedBackboneEdgesだけではpublic route queryが空になる回帰防止 |
| C608 | Behavior | Boundary | SavedBackboneResultはsaved pole nodeを重複表示しない | `legacy_unclassified` | saved graph nodeとmaterialized pole id nodeが二重計上され、public queryのnode数・junction判断が膨らむ回帰防止 |
| C607 | Behavior | Boundary | SavedBackboneResultはsaved ownerless nodeの今回route indexを保つ | `legacy_unclassified` | saved graph正本を優先した結果、debug routeメタデータが上書きされ、明示non-pole supportをpublic queryで再特定できなくなる回帰防止 |
| C606 | Behavior | Boundary | SavedBackboneResultはsaved ownerless nodeを説明する | `legacy_unclassified` | backbone saved ownerless topologyがpublic queryから見えず旧観測API経由の実用確認ができない回帰防止 |
| C609 | Behavior | Boundary | 通常のbendはlowering intentを作らない | `legacy_unclassified` | route bendをbranch/cross conflictと誤認してloweringする回帰防止 |
| C610 | Behavior | Boundary | row conflictは許可されたbundleのjunction endpointだけをoffsetする | `legacy_unclassified` | pair idをrow idとして扱う、またはmulti-row nodeの全bundle/全endpointを下げる回帰防止 |
| C611 | Behavior | Boundary | backbone direct deriveはsaved ruleからspan outputsを復元する | `legacy_unclassified` | post-edit/rederiveが`Commit(run_recalc)`やsupport-layout materializationに戻る回帰防止 |
| C612 | SourceGuard | Boundary | backbone direct deriveはrecalc/materialization経路を呼ばない | `source_guard` | direct derive entrypointが名前だけで内部はrecalc wrapperになる回帰防止 |
| C613 | Behavior | Boundary | backbone port editはCommitなしでgenerated span outputsを再導出する | `legacy_unclassified` | viewer normal pathがport edit後の表示更新を`Commit(run_recalc)`に依存する回帰防止 |
| C614 | Behavior | Boundary | backbone update planは4分類だけを使う | `legacy_unclassified` | setterやderive側が操作名別dirty enumを増やす回帰防止 |
| C615 | Behavior | Boundary | kRegenerateはlocal fallback実行しない | `legacy_unclassified` | route/island regeneration仕様なしでfallback実装する回帰防止 |
| C616 | Behavior | Boundary | kRepositionはSavedBackboneGraph識別情報を変えない | `legacy_unclassified` | port/pole移動がtopology再決定やgraph再保存に戻る回帰防止 |
| C617 | Behavior | Boundary | kReshapeはlayoutを書き換えない | `legacy_unclassified` | sag変更でplacement/layoutを再判定する回帰防止 |
| C618 | Behavior | Boundary | kRedrawはlayout/geomを書き換えない | `legacy_unclassified` | 表示設定変更で幾何やplacementを再導出する回帰防止 |
| C619 | Behavior | Boundary | kRepositionはaffected spanだけ更新する | `legacy_unclassified` | affected setが全span更新に戻る回帰防止 |
| C620 | SourceGuard | Boundary | update境界に操作別kindを作らない | `source_guard` | PoleTilt/PoleYaw/Sag等の細粒度enumが増殖する回帰防止 |
| C621 | Behavior | Boundary | sagはgeom/drawだけをreshapeする | `legacy_unclassified` | backbone wireが直線固定へ戻る、またはsagがtopology/layoutを再決定する回帰防止 |
| C622 | Behavior | Boundary | pipeline/update timingは診断専用 | `legacy_unclassified` | global profilerやtiming起点の生成分岐を作らず性能観測の入口を固定する |
| C623 | Behavior | Boundary | backbone layout settingsはregenerateで派生出力を更新する | `legacy_unclassified` | layout settingsをstale成功または全拒否へ戻す回帰防止 |
| C624 | Behavior | Differential | variation settingsは既存Spanをreshapeしfresh生成と一致する | `legacy_unclassified` | sag variation変更がstale成功またはtopology再生成へ戻る回帰防止 |
| C625 | Behavior | Boundary | context profileはstale成功しない | `legacy_unclassified` | 未接続profileの成功扱い防止 |
| C626 | Behavior | Boundary | cable shape/render更新はdirect derive | `legacy_unclassified` | dirty markerなしで出力更新を保証 |
| C627 | SourceGuard | Boundary | 旧topology状態変更APIはpublic surfaceに残さない | `source_guard` | 旧APIがgraph外topology経路として復活する回帰防止 |
| C628 | Behavior | Boundary | active backbone pole typeはplacement-onlyなら再導出する | `legacy_unclassified` | Pole Placement UIを既存線へ反映する回帰防止。構造差分はC713が統一regenerateとfresh等価性で固定する |
| C629 | Behavior | Invariant | parabolic main-span curveの意味入力を固定する | `legacy_unclassified` | caller側Bezier handleや横揺れをauthorityにする回帰防止 |
| C630 | Behavior | Invariant | curve sample frameはfiniteかつorthonormal | `legacy_unclassified` | Frenet由来のnear-straight twistやNaN防止 |
| C631 | Behavior | Invariant | reverse traversalでもcanonical lateral frameを維持する | `legacy_unclassified` | save/loadやedge traversal反転でlane左右が反転する回帰防止 |
| C632 | Behavior | Invariant | tessellationはlength/sagに応じる | `legacy_unclassified` | caller固定sample countへの逆戻り防止 |
| C633 | Behavior | Invariant | zero-length/vertical curveを決定的に処理する | `legacy_unclassified` | near-world-up fallback不定・silent method fallback防止 |
| C634 | Behavior | Boundary | terminal nodeはNodePatchCurveを作らない | `legacy_unclassified` | terminalまでspan-local attachment blendを作る回帰防止 |
| C760 | Behavior | Invariant | terminal EdgeBodyはresolved port endpointに直接接続する | `legacy_unclassified` | patch無しを理由にterminal wireがportから浮く回帰を防ぐ |
| C761 | Behavior | Invariant | default OPTICAL bundleはmember endpoint追従のsupportと6 samples/turnのhelixを出力し、COMMは通常線のままにする | `legacy_unclassified` | Helixの既定対象がCOMMへ戻る、固定高support bandで断面が肥大化する、端部だけ動いて懸垂部が残る、またはsamplingが過剰になる回帰を防ぐ |
| C635 | Behavior | Boundary | simple 2-edge continuous nodeはNodePatchCurveを作る | `legacy_unclassified` | 前後spanがそれぞれ同じjunction接続部を丸める回帰防止 |
| C636 | Behavior | Boundary | EdgeBodyCurveはNodePatch境界で止まる | `legacy_unclassified` | edge bodyがnode patch内まで食い込み、接続部ownershipが混ざる回帰防止 |
| C637 | Behavior | Boundary | NodePatchCurveとEdgeBodyCurveは元の懸垂曲線へG1接続する | `legacy_unclassified` | trim後bodyをHermiteで再構築して端部へ曲率を集中させる、またはpatchとbodyだけを同じ誤接線へ揃える回帰防止 |
| C638 | Behavior | Boundary | visual curve part samplesはfinite | `legacy_unclassified` | NodePatch/EdgeBody導入でNaN/infや空debug幾何を保存する回帰防止 |
| C639 | Behavior | Boundary | NodePatchCurveは直線chordではない | `legacy_unclassified` | 接続部の丸みが消え、patchがdebug名だけの折れ線に戻る回帰防止 |
| C640 | Behavior | Boundary | NodePatchCurveのBezier debug controlsを観測できる | `legacy_unclassified` | 見た目違和感の調査でviewer側が推測や補正を始める回帰防止 |
| C641 | Behavior | Boundary | Pole tilt後もvisual curve partsが所有endpointに追従する | `legacy_unclassified` | direct derive後に古いcurve partキャッシュを残さない |
| C659 | Behavior | Boundary | draw-time tiltはport/span生成前にsupport frameへ入る | `legacy_unclassified` | DrawPath後のfinal Apply API依存や、見た目だけtiltしてendpointがuntiltedのまま残る回帰防止 |
| C642 | Behavior | Boundary | EdgeBodyCurveは正式sag curveを共有する | `legacy_unclassified` | quartic sag二重実装と固定8 sampleへの回帰防止 |
| C643 | Behavior | Boundary | NodePatchCurveはattachment参照を保持した内側fillet | `legacy_unclassified` | attachment通過制約による外振りへの回帰防止 |
| C644 | Behavior | Boundary | patch境界はmain sag実接線の延長上でattachmentより下がる | `legacy_unclassified` | attachmentと同高の水平patchや固定down offsetへの回帰防止 |
| C645 | Behavior | Boundary | NodePatch内側filletは局所曲率を維持する | `legacy_unclassified` | filletがstraight chordへ退化する回帰防止 |
| C646 | Behavior | Boundary | NodePatchはturn内側で単調に曲がる | `legacy_unclassified` | 左折前に右へ外振りする回帰防止 |
| C647 | Behavior | Boundary | NodePatchはincident cableのappearanceを使う | `legacy_unclassified` | source_span_idを持たないpatchだけviewer既定の太さ・色へ落ちる回帰防止 |
| C655 | SourceGuard | Boundary | NodePatch groupingはband識別情報を含み、pair選択はrow continuityだけを読む | `source_guard` | 同テンプレートの別Bundle、上下band、別placement slotを同じ接続部としてpatch化する回帰防止 |
| C658 | Behavior | Boundary | CableCurveはendpoint tangent hintsを実サンプルに使う | `legacy_unclassified` | NodePatch境界tangentをEdgeBodyがメタデータだけ持ち、描画samplesでは無視する回帰防止 |
| C688 | Behavior | Invariant | 片側hintのCableCurveはhint無し端の自然なsag端点接線を保つ | `legacy_unclassified` | 3点以上routeの終端spanだけ弦方向接線+端点微分ゼロの装飾sagへ切り替わる回帰防止 |
| C730 | Behavior | Invariant | BundleKindはBundleTemplate識別情報ではない | `legacy_unclassified` | BundleKind keyed mapへ戻る回帰防止 |
| C731 | Behavior | Invariant | BackboneSpecは同kind別テンプレートを区別する | `legacy_unclassified` | BackboneBundleSpecがBundleKind識別情報に戻る回帰防止 |
| C732 | Behavior | Invariant | route bundle ruleの所有元はBundleTemplateId | `legacy_unclassified` | BundleKind単位でrandom bundle ruleが混線する回帰防止 |
| C733 | Behavior | Invariant | regenerate scopeはBundleTemplateId | `legacy_unclassified` | regenerate scopeがkindで集まる回帰防止 |
| C734 | Behavior | Invariant | cableテンプレートlookupはkind basedではない | `legacy_unclassified` | CableTemplate lookupがBundleKindに戻る回帰防止 |
| C735 | SourceGuard | Boundary | source guardはBundleKind識別情報残存を拒否する | `source_guard` | 永続化前にenum識別情報を焼き戻す回帰防止 |
| C736 | SourceGuard | Boundary | unsupported保留docsは対応済みbackbone更新を戻さない | `source_guard` | docsだけが古いunsupportedを再導入し、viewer後追い作業を誤誘導する回帰防止 |
| C737 | Behavior | Boundary | overlay edge endpoint snapはsaved node spec idを返す | `legacy_unclassified` | web overlay edge endpointから伸ばす操作がunknown node referenceで止まる回帰防止 |
| C837 | Behavior | Boundary | source bundle付きendpoint completionは別bundleのopen rowをpromotion候補にしない | `legacy_unclassified` | 終点へつなぐ操作で同種別の別route bundleを同じplacementへ混ぜ、promoted placement maps to multiple bundlesで拒否する回帰防止 |
| C691 | Behavior | Invariant | run idはthrough sectionを接続する | `legacy_unclassified` | section scope idのままspan境界で見た目上1本のcableが分断される回帰防止 |
| C692 | Behavior | Invariant | run idは世代跨ぎterminal extensionを接続する | `legacy_unclassified` | 別生成回の連続sectionが別run扱いになる回帰防止 |
| C693 | Behavior | Invariant | branchとdead-endは別runになる | `legacy_unclassified` | jumperやbranchをpatch連続として誤集約する回帰防止 |
| C695 | Behavior | Invariant | 識別情報を持つcable sectionのrun idは決定的 | `legacy_unclassified` | 識別情報なしsupport/helixをrunへ偽装せず、pointer/order依存の不安定run id回帰を防止 |
| C696 | SourceGuard | Boundary | run idはvisual derive層の派生値で、section indexから決定する | `source_guard` | run識別情報をtopology正本へ保存する、またはsectionごとの線形探索へ戻る回帰防止 |
| C660 | Behavior | Boundary | regenerate fixed count increaseはSavedGraph識別情報を維持して下流だけ更新する | `legacy_unclassified` | route-local regenerateが入力replayや位置推測でgraphを作り直す回帰防止 |
| C661 | Behavior | Invariant | pair row axisは径間長ではなく単位接線二等分で決める | `legacy_unclassified` | pass-through cornerの横並びが長い径間へ引っ張られる回帰防止 |
| C662 | Behavior | Invariant | pair row axisはincident spanのlane順を反転しない | `legacy_unclassified` | row axis変更で片側spanのlane順がねじれる回帰防止 |
| C663 | Behavior | Invariant | 3相HV鋭角continuityは最終Port列に従ってlaneをmirrorし、径間別dead-end rowと3 Jumperを導出する | `oracle` `anchor` | span内部だけ直ってcross-edge continuity/Jumperがsame-laneのままねじれる回帰防止 |
| C664 | SourceGuard | Boundary | 鋭角pole facingはpairsのcorner decisionを消費し再判定しない | `source_guard` | pole yawとrow/jumperが別々に鋭角を解釈する回帰防止 |
| C665 | Behavior | Boundary | source-edge projectionは派生curve上に置く | `legacy_unclassified` | 途中分岐が既存cableから浮く回帰防止 |
| C666 | Behavior | Boundary | 世代を跨ぐterminal continuationをcontinuityで接続する | `legacy_unclassified` | point proximityや同一port偶然一致へ依存する回帰防止 |
| C667 | Behavior | Boundary | branch追加後もthrough patchを維持する | `legacy_unclassified` | group size 3で既存through丸めが消える回帰防止 |
| C697 | Behavior | Invariant | saved edgeはlateral offset生成入力をecho保存する | `legacy_unclassified` | 保存済みgraphから復元できないplacement constraintをregenerate前処理で使えるようにする |
| C668 | Behavior | Invariant | regenerate fixed count increaseはsaved lateral offsetを消費する | `legacy_unclassified` | placement echoを使わず既定offsetで増加再生成する回帰防止 |
| C669 | Behavior | Invariant | multi-bundle regenerateはcount減少も対象bundleだけを変えfreshと一致する | `legacy_unclassified` | multi-bundle count減少で非対象bundleを動かす、または対象bundleのgroup offsetを誤る回帰防止 |
| C670 | Behavior | Invariant | regenerateはpair rowを含む3点routeをfreshと一致させる | `legacy_unclassified` | regenerateがendpoint Port識別情報を共有へ戻す、またはpair lane位置を分離する回帰防止 |
| C671 | SourceGuard | Boundary | regenerateはpipeline段階を再利用し専用emitを持たない | `source_guard` | M1 regenerateが第2 pipelineへ戻る回帰防止 |
| C672 | Behavior | Invariant | regenerateは存続laneのmanual portを保持する | `legacy_unclassified` | 手編集済みportをregenerateが黙って上書きまたは拒否する回帰防止 |
| C673 | Behavior | Invariant | regenerateは存続spanのuser attachmentを保持する | `legacy_unclassified` | 存続spanのユーザー追加attachmentをregenerateが不要に拒否または喪失する回帰防止 |
| C698 | Behavior | Invariant | regenerate fixed count decreaseは退役laneを削除し残存出力をfresh count1と一致させる | `legacy_unclassified` | count減少をmigration複製ではなくreconcile付きregenerateで扱う |
| C699 | Behavior | Invariant | regenerate fixed count decreaseはsaved lateral offsetを消費する | `legacy_unclassified` | regenerate前処理がplacement echoを使わず既定offsetで再生成する回帰防止 |
| C700 | Behavior | Boundary | regenerate fixed count decreaseは退役対象attachmentを状態変更前に拒否する | `legacy_unclassified` | ユーザーattachment付きspanを暗黙退役する回帰防止 |
| C701 | SourceGuard | Boundary | regenerateは派生出力からtopologyを推測しない | `source_guard` | 差分再生成がspan/layout/curve/port位置からtopologyを復元する第2 pipelineへ逸れる回帰防止 |
| C702 | Behavior | Invariant | regenerate fixed countは増減混在後もfresh count1と一致する | `legacy_unclassified` | 増加だけ旧migration、減少だけregenerateという分岐を残す回帰防止 |
| C742 | Behavior | Invariant | fixed count減少はメタデータrenameと同時でも統一regenerateを通る | `legacy_unclassified` | count減少側だけメタデータ一致を要求し、増加側と受理範囲が分岐する回帰防止 |
| C743 | SourceGuard | Boundary | BundleTemplateフィールドdiffの決定者はclassifier一箇所だけである | `source_guard` | routing blockがフィールド条件を再列挙し、分類器と受理範囲が再びdriftする回帰防止 |
| C744 | SourceGuard | Boundary | lowered support group keyの収集は一箇所だけである | `source_guard` | generationとpost-edit deriveがgroup key/default/basisを別々に転記してdriftする回帰防止 |
| C745 | SourceGuard | Boundary | 旧population wrapとCableTemplate coil familyはproduction surfaceに残らない | `source_guard` | 旧carrier-following wrapまたはCableTemplate coilが新assemblyと並存する回帰防止 |
| C746 | Behavior | Invariant | generationはisolated trialを成功時だけmove commitし、66 pole級状態でcopyは総時間の20%以内 | `legacy_unclassified` | 空状態だけを測ってcopy costを過小評価する、またはpipeline後半失敗が本状態へ漏れる回帰防止 |
| C747 | Behavior | Invariant | range count policyは保存済みconductor countを検証し、範囲内なら出力を再生成せず、範囲外なら対象bundleを示して状態不変で拒否する | `legacy_unclassified` | range化を不要にregenerateする、または範囲外の既存bundleを黙って不整合にする回帰防止 |
| C748 | Behavior | Invariant | bundle policy差分はscopeをfresh等価へreconcileし、fresh-invalid入力は状態変更前に拒否する | `legacy_unclassified` | topology policyをrejectし続ける、scope外を再導出する、freshでは拒否される入力をpartial状態変更する回帰防止 |
| C749 | Behavior | Invariant | Offset=0はband既定位置から追加移動せず、他bundleにも影響されない | `legacy_unclassified` | bundle index由来の隠れた横offset、band中心無視、HV全laneの単一band展開への回帰防止 |
| C789 | Behavior | Invariant | multi-route同一band rowはPortを共通の論理anchorに保ちながら、同じルートの枝と別ルート識別子の両方でsupport levelを適用した最終endpoint間隔を保つ | `oracle` `anchor` | 論理Portの人工的Z分離を復活させる、物理fixture/socketのrow separationを失う、または別ルートが同じbandへ重なる回帰を防ぐ |
| C790 | Behavior | Boundary | 既存supportと同一座標のpath pointは明示node referenceを要求する | `legacy_unclassified` | 既存supportを暗黙選択または重複poleとして後段処理へ進め、pair/occupancy探索を肥大化させる回帰防止 |
| C703 | SourceGuard | Boundary | bundle count migrationシンボルはdomains/wire/srcから消える | `source_guard` | 差分別migration操作が残る回帰防止 |
| C704 | SourceGuard | Boundary | regenerateはpost-edit APIから直接呼ばれupdate plan経由で実行しない | `source_guard` | pipeline内にmigration語が残る、またはplanが差分入力なしでregenerate実行経路になる回帰防止 |
| C705 | Behavior | Invariant | saved edge_bundleの順序はrequest bundle spec順と一致する | `legacy_unclassified` | multi-bundle regenerateがgroup offsetを保存済みedge_bundles順から復元する前提を固定 |
| C706 | Behavior | Invariant | multi-bundle regenerateは対象bundleだけを増減しfreshと一致する | `legacy_unclassified` | multi-bundle group offsetを単一bundleとして再生成したり、非対象bundleを動かす回帰防止 |
| C707 | Behavior | Invariant | 3点routeの継続はrow continuity tableに保存される | `legacy_unclassified` | regenerateがsaved edge route/orderではなくrow continuityを継続正本として使うことを固定 |
| C708 | Behavior | Invariant | 3点route regenerateはcount減少後freshと一致し、scope外routeを触らない | `legacy_unclassified` | pair rowを含むroute-local regenerateが一部edgeだけを再生成する、またはscope外routeを再導出する回帰防止 |
| C709 | Behavior | Invariant | 3点route regenerateはcount増加後freshと一致し、scope外routeを触らない | `legacy_unclassified` | lane増加時にendpoint別識別情報とpair位置一致を崩す、またはscope外routeを再導出する回帰防止 |
| C710 | Behavior | Invariant | multi-bundle 3点route regenerateはfreshと一致し、scope外routeを触らない | `legacy_unclassified` | R1のgroup offset復元とR2のpair row復元を同時に満たし、scope外routeを再導出しない |
| C711 | Behavior | Invariant | regenerate count減少は存続laneのattachmentを保持する | `legacy_unclassified` | count減少で存続spanのattachmentまで拒否または削除する回帰防止 |
| C712 | Behavior | Invariant | cable decision差分はregenerateでfreshと一致する | `legacy_unclassified` | decision差分をstale成功やdirect deriveに逃がさず統一regenerateへ通す |
| C738 | Behavior | Invariant | default endpoint attachment差分はauto endpointだけをreconcileする | `legacy_unclassified` | default endpoint差分を未消費またはuser attachmentまで置換し、scope外を再導出する回帰防止 |
| C713 | Behavior | Invariant | pole type structural差分はregenerateでfreshと一致する | `legacy_unclassified` | poleテンプレート構造差分をrejectやstale成功にせず統一regenerateへ通す |
| C714 | Behavior | Boundary | regenerateは退役laneのmanual portを状態変更前に拒否する | `legacy_unclassified` | 手編集済みportを退役時に暗黙削除する回帰防止 |
| C715 | Behavior | Invariant | backbone span branch-down overrideはregenerateでlayout/curveに反映する | `legacy_unclassified` | backbone span overrideを拒否または保存だけして派生出力をstaleにする回帰防止 |
| C716 | Behavior | Invariant | backbone span endpoint socket overrideはregenerateでlayoutに反映する | `legacy_unclassified` | socket overrideを非backbone専用の保存状態に閉じ込める回帰防止 |
| C739 | Behavior | Invariant | span override regenerateは別routeを触らない | `legacy_unclassified` | span-local overrideが全route regenerateや雑なscope収集へ広がる回帰防止 |
| C740 | Behavior | Invariant | geomで構築済みのbase curveをvisual curve partsが優先再利用する | `legacy_unclassified` | pipelineとvisualが同じbase spanのfull curveを二重構築する回帰防止 |
| C741 | Behavior | Invariant | A-B-CへC-Dを追加したscoped visual rebuildは隣接B connectionを保持して全量再構築と一致する | `differential` | read contextの反対側nodeをwrite scopeへ昇格し、既存NodePatch/Jumperを削除する回帰防止 |
| C717 | Behavior | Invariant | layout settings差分はregenerateでfreshと一致する | `legacy_unclassified` | layout settings差分を部分deriveやstale成功に逃がさず統一regenerateへ通す |
| C727 | SourceGuard | Boundary | pipeline execution入口はbuild(build_input) だけにする | `source_guard` | regenerateが第2 pipeline化し、通常生成と別段階列を持つ回帰防止 |
| C728 | SourceGuard | Boundary | pipelineはrun mode / skip flagを持たない | `source_guard` | 操作固有条件や検証skipがpipeline mode分岐として復活する回帰防止 |
| C729 | SourceGuard | Boundary | regenerate sourceはpipeline出力と退役を手組みしない | `source_guard` | regenerate.cppが第2 pipelineやfallback更新経路、退役ループを持つ回帰防止 |
| C718 | Behavior | Boundary | viewer hit world heightはsource-edge branchの正本ではない | `legacy_unclassified` | viewer preview座標やsource projectionを支持点高さとして固定する回帰防止 |
| C719 | Behavior | Invariant | source-edge branch endpointはcurrent curve projectionに追従する | `legacy_unclassified` | source-edge branch endpointが作成時port位置にstale固定される回帰防止 |
| C720 | SourceGuard | Boundary | source-edge pipeline前半はcurve projectionを読まない | `source_guard` | topology/port materialization段がsource cable幾何を必須入力に戻す回帰防止 |
| C721 | Behavior | Invariant | source-edge識別情報はprojection update後も残る | `legacy_unclassified` | projection更新をworld height保存へすり替える回帰防止 |
| C722 | Behavior | Boundary | unresolved source-edge referenceは状態変更前に失敗する | `legacy_unclassified` | source識別情報不解決時に途中までentityを作る回帰防止 |
| C723 | Behavior | Invariant | source-edge branchはsource sagを変えない | `legacy_unclassified` | branch追加をsource cable物理荷重として循環させる回帰防止 |
| C724 | Behavior | Invariant | sourceテンプレートsag変更はbranch projectionを更新する | `legacy_unclassified` | sag/template更新でbranch endpointがstaleになる回帰防止 |
| C725 | Behavior | Invariant | source layout settings変更後もbranch projectionはcurrentのまま | `legacy_unclassified` | layout更新でsource-edge branch endpointがstaleになる、またはownerless source-edge route regenerateが拒否される回帰防止 |
| C726 | Behavior | Boundary | source-edge branch projectionはprior curveキャッシュを要求しない | `legacy_unclassified` | pipeline後半がprior curveキャッシュに依存し、一発生成やキャッシュ欠落時にbranch endpointを解けない回帰防止 |
| C674 | SourceGuard | Boundary | backbone portのbandはsaved binding、非backbone portは共有selectorが決める | `source_guard` | 下流がband識別情報を再推測する回帰防止 |
| C675 | SourceGuard | Boundary | port layout yawはsaved bindingを読みdebug recordを決定入力にしない | `source_guard` | debug観測値がproduction配置を決める回帰防止 |
| C676 | Behavior | Invariant | generationとpost-editは同じsaved layout yawを使う | `legacy_unclassified` | generationとendpoint refreshのyaw決定者が分岐する回帰防止 |
| C677 | SourceGuard | Boundary | corner side scaleとinner side判定は1定義だけを持つ | `source_guard` | 同じ角補正式がstate/template/refreshで分岐する回帰防止 |
| C678 | Behavior | Invariant | ModelDescriptor mergeは決定的でconflictを補完しない | `legacy_unclassified` | import/adapter前にdescriptor契約をpure logicとして固定し、fallback補完や状態依存へ流れる回帰を防ぐ |
| C679 | Behavior | Invariant | 使用中AttachmentTemplateの幾何差分はcurveを再導出する | `legacy_unclassified` | モデル再読込のsocket位置変更が全拒否され、手動で置いたattachmentの曲線がstaleになる回帰を防ぐ |
| C680 | Behavior | Boundary | 使用中AttachmentTemplateの構造差分は状態変更前に拒否する | `legacy_unclassified` | socket識別情報変更を幾何更新として通し、既存attachmentの接続意味を壊す回帰を防ぐ |
| C681 | Behavior | Invariant | path無しReplaceはhidden intervalのみ生成する | `legacy_unclassified` | モデル本体が区間を埋める前提のattachmentをinternal path必須に戻す回帰を防ぐ |
| C682 | Behavior | Boundary | 同一spanのreplaced interval重複はErrorとする | `legacy_unclassified` | 置換区間同士が重なって線の可視区間が曖昧になる回帰を防ぐ |
| C683 | Behavior | Invariant | ModelDescriptorからpath無しAttachmentTemplateを生成する | `legacy_unclassified` | DCC marker descriptorを状態やviewerに依存せずcoreテンプレートへ変換できることを固定する |
| C684 | Behavior | Invariant | 端子函descriptorのdrop socketは補助socketとして保持する | `legacy_unclassified` | 追加socketをtopologyやline pathと誤解して補完生成する回帰を防ぐ |
| C685 | Behavior | Invariant | descriptor-builtテンプレートは既存更新経路で使用中spanを更新する | `legacy_unclassified` | モデル再読込専用経路を作らず、通常テンプレート更新経路で使用中attachmentを更新する契約を固定する |
| C755 | Behavior | Invariant | 鋭角Jumperはincident spanへG1接続する | `legacy_unclassified` | 鋭角接続がport間chord接線で折れる既存不具合の固定 |
| C756 | SourceGuard | Boundary | write/readは型別wrapperを持たず同じarchive visitorから導出する | `source_guard` | フィールド列挙一元化後にwrite/read二重familyが再発することを防ぐ |
| C757 | Behavior | Invariant | load後authoritativeは同じarchiveフィールド列挙で直接一致する | `legacy_unclassified` | writer/reader双方から同じフィールドが落ちbyte一致だけ通る盲点を防ぐ |
| C758 | Behavior | Invariant | enabled assemblyはlogical spanごとにsupport pathとhelixを1本ずつ派生する | `legacy_unclassified` | Bundle全laneを一体化したりauthoritative topologyを増やす回帰を防ぐ |
| C759 | SourceGuard | Boundary | support primary curve・helix・containment・wander・twistはspan visual assemblyだけが決定する | `source_guard` | category別support実装やvisual assemblyをtopology／curve_partsへ分散させる回帰を防ぐ |
| C764 | Behavior | Invariant | straight HVはPoleとderived row/lane単位でモデルを派生しwire socketをcurve端点にする | `legacy_unclassified` | Port単位fixture二重化、Web後合わせ、laneごとの腕金重複、belt基準混同、post-edit時のstaleモデルを防ぐ |
| C765 | Behavior | Invariant | branch loweringはfixture placementへ一度だけ適用し、最終モデルsocketをcurve endpointにする | `legacy_unclassified` | モデルを元高さに残して線端だけ下げる、またはedge endpoint Portごとにfixtureを二重生成する回帰を防ぐ |
| C766 | Behavior | Invariant | row mount・endpoint fixture・接続線は一つのendpoint placementを共有し、band lateral変更へ追従する。beltは柱軸に残る | `legacy_unclassified` | 腕金のモデルoffsetだけが碍子・線へ伝わらない、腕金だけがband offsetへ追従しない、またはbeltまで柱から外れる回帰を防ぐ |
| C767 | Behavior | Invariant | default HV/LVのsupport pathはOpticalと同じprimary curve生成を使い、接続曲線境界で切れず解決済みendpointへ届く | `legacy_unclassified` | EdgeBody複製によるnode patch境界での切断、カテゴリ別curve family、碍子socket無視、support-onlyでmember幾何を変える回帰を防ぐ |
| C769 | Behavior | Invariant | 同じBundleTemplateを複数配置してもBundle単位で識別情報・絶対配置・node patch・中間分岐attachmentを分離する | `legacy_unclassified` | テンプレートid集合へ畳む、同テンプレートの最初のsource attachmentを使う、support/helix/node patchだけ旧位置や別Bundleへ混ざる回帰を防ぐ |
| C770 | Behavior | Invariant | Bundle placementのheight更新はPortを新しいexplicit base heightへ置き、保存済みsupport level/groupによるrow placementを維持して碍子・腕金・線端へ反映する | `oracle` `anchor` | placement更新で旧Port offsetを別authorityとして温存する、またはモデルsocket補正を線端から落として交差部の碍子・腕金と線を分離する回帰を防ぐ |
| C771 | Behavior | Invariant | incremental cross completionは既存endpoint識別情報を維持して2 continuityへ収束する | `legacy_unclassified` | degree fourのsaved/current差で未接続openが残る、または接続時に既存endpoint frameを動かす回帰を防ぐ |
| C772 | Behavior | Boundary | incremental pairingは複数の未接続候補を推測せず新edgeをopenで残す | `legacy_unclassified` | first-match、最小角度選択、曖昧時の操作拒否を防ぐ |
| C773 | Behavior | Invariant | sharp cornerのincremental completionはcontinuityを記録し2 dead-end rowとjumperを導出する | `legacy_unclassified` | sharp接続を未接続として保存する、または通常NodePatchとして描く回帰を防ぐ |
| C774 | Behavior | Invariant | bundle/port scopeが違うincremental completionは既存Portを誤共有しない | `legacy_unclassified` | row keyだけで異scopeのport bindingを再利用する回帰を防ぐ |
| C775 | Behavior | Invariant | incremental canonical pair rowはsave/load後も維持される | `differential` `anchor` | 保存形式やload再派生でcanonical pairがopenへ戻る、または保存Spanと再導出endpointの向きが分裂する回帰を防ぐ |
| C776 | Behavior | Invariant | incremental canonical pair rowはregenerate後も維持される | `legacy_unclassified` | regenerateがsaved open由来のpairを3rowへ戻す回帰を防ぐ |
| C777 | Behavior | Invariant | reverse入力directionでもincremental completionは同じcontinuityと派生rowへ収束する | `legacy_unclassified` | 入力方向によってcontinuityやrow配置が反転する回帰を防ぐ |
| C778 | Behavior | Invariant | multi-templateのincremental completionは各互換scopeを1回だけ接続し、別scopeや別openを曖昧扱いしない | `legacy_unclassified` | endpoint bindingをscope外まで汎用探索して曖昧化する回帰を防ぐ |
| C779 | Behavior | Invariant | same-template multi-placementのincremental completionはplacement_keyで既存Bundleへ接続しPortはedge別に保つ | `legacy_unclassified` | duplicate placement混同、shared Port再導入、HV欠落を防ぐ |
| C780 | Behavior | Invariant | 同じテンプレート・同じ数値配置でもplacement_keyが違えば別placementとして扱い、request順序に依存しない | `legacy_unclassified` | height/offset/spacingやrequest配列順でmatchingする回帰を防ぐ |
| C781 | Behavior | Invariant | completed crossからさらにedgeを伸ばしても既存spanをretireしない | `legacy_unclassified` | 既存Bundle IDを継続したedge bundleをcontext edge側までretire scopeに含め、元のspanを削除する回帰を防ぐ |
| C782 | Behavior | Invariant | 接続済みsharp pairがあるjunctionへさらに鋭角edgeを追加しても既存pairを奪わず新規edgeをopenで残す | `legacy_unclassified` | 接続済みedgeを候補へ戻す、角度で再pairする、または追加操作を拒否する回帰を防ぐ |
| C785 | Behavior | Invariant | HV 3相のincremental pair promotionは既存Port識別情報を維持し、通常pairの二等分frameへPort位置とbinding yawを再導出する。lane対応は識別情報 / complete reverseに限定し、context側正本を変更せず、必須frameを再構築できない失敗時は正本を不変に保つ | `legacy_unclassified` | lane対応反転、contextの通常生成、失敗時の部分更新、bindingとPortのframe分裂、shared Port再導入を防ぐ |
| C786 | SourceGuard | Boundary | hash mix helperはproductionで1定義だけを持つ | `source_guard` | hash helper copy familyがcurve / 幾何に再発することを防ぐ |
| C787 | SourceGuard | Boundary | BundleTemplate categoryと既定配置はCore/WASM payloadを正本にし、JSはlayerから推測しない | `source_guard` | webがSpanLayer→category対応表やband matchingを複製し、Core/WASMの分類とdriftする回帰を防ぐ |
| C788 | SourceGuard | Boundary | belt radial fitとendpoint socket placementはmaterializationが所有する | `source_guard` | curve_partsやrendererがsocket/mesh幾何を再解釈する、またはbelt半径をmesh実体からCoreで読む回帰を防ぐ |
| C791 | Behavior | Boundary | 大きめの既存sceneへrouteを1本追加してもfixture pipelineは操作単位でだけ走る | `legacy_unclassified` | scene規模に比例してplan構築・materialization・fallbackが再発する回帰をwall-clockに依存せず検出する |
| C792 | Behavior | Invariant | incrementalで新しい独立rowを追加しても既存Port anchorは動かさず、新rowはband anchorを保ったまま次のsupport level/groupを使い、そのresolved offsetをendpointへ適用する | `anchor` | Port高さのstable-slot再決定、既存row reflow、support levelとendpoint loweringの分裂を防ぐ |
| C795 | Behavior | Invariant | model-aware HV pair promotionはPort/Span識別情報を維持し、正本placement anchorから同一pair frameでPort/row fixture/endpoint fixture/curveを導出してNodePatch laneをcollapseさせない | `legacy_unclassified` | モデルありpromotionでfixtureとPortのframe分裂、3相collapse、shared Port再導入を防ぐ |
| C796 | Behavior | Invariant | explicit placement heightはincremental row reflowで変更されず、mixed bundleとsame-template duplicateを混同しない | `legacy_unclassified` | explicit heightをrow spacingやstable slotでband既定値へずらす、same-template duplicateが混ざる、pair promotionで既存row高さが変わる回帰を防ぐ |
| C797 | Behavior | Boundary | row continuity tableはrouteとpair promotionの各lane継続を保存する | `legacy_unclassified` | NodePatchの存在をcurve層がbundle/port/位置epsから推測し続ける回帰を防ぐ |
| C798 | Behavior | Invariant | viewer既定T字branchはHVを生成し、branch-downはテンプレートflagだけで決まる | `legacy_unclassified` | 画面上snap済みの端点をCoreの3D距離判定でmidair扱いしてHVを除外する、category名でloweringする、またはT追加で既存Port高さを動かす回帰を防ぐ |
| C800 | Behavior | Boundary | row continuity graph lintはroute/branch/cross出力の参照整合を検査する | `legacy_unclassified` | 不正参照や重複endpointをcurve層のsilent dropで隠す回帰を防ぐ |
| C802 | Behavior | Invariant | viewer既定T字branchは既存電柱がpath終点でもHVを生成し、branch-downはテンプレートflagだけで決まる | `legacy_unclassified` | T branchの向き違いでHVだけ欠落する、category名でloweringする、またはT追加で既存Port高さを動かす回帰を防ぐ |
| C808 | Behavior | Invariant | T字branch loweringはHV categoryではなくBundleTemplate flagだけに従う | `legacy_unclassified` | HV専用分岐で高さを変える、またはflagを無視して非対象テンプレートのPort自体をrow reflowで下げる回帰を防ぐ |
| C809 | Behavior | Invariant | 1 support levelは1 continuity pairまたは1未接続endpointだけを収容し、incremental追加では既存levelを維持して次の空きlevelへ進む | `legacy_unclassified` | 追加rowをすべて固定の2段目へ置く、接続時に段を変える、または通常状態の高さ・ID順から段を再推測する回帰を防ぐ |
| C810 | Behavior | Invariant | 通常pairはedge endpoint別Portを持ち、共有row/fixtureだけを導出する | `legacy_unclassified` | shared Port、別計算の位置ずれ、fixture二重生成、通常repositionでのPort再生成を防ぐ |
| C813 | Behavior | Invariant | MovePoleで通常/鋭角閾値を跨いでもendpoint識別情報を維持し表現だけを再導出する | `legacy_unclassified` | 保存pair/open表現やcached ruleにより角度変更後も古いrow/fixture/patchが残る回帰を防ぐ |
| C814 | Behavior | Invariant | 同一操作の2 edgeを既存openより優先して接続しない | `legacy_unclassified` | 操作内隣接を特別扱いして曖昧候補から勝手にpairを選ぶ回帰を防ぐ |
| C815 | Behavior | Invariant | 鋭角pairの段違い高さは一筆書きか段階追加かで変わらない | `legacy_unclassified` | 鋭角の2 derived rowを同じ高さへ潰す、片側だけをloweringする、または操作履歴でpair高さが変わる回帰を防ぐ |
| C816 | Behavior | Invariant | 通常pairはpair化後も割当済みsupport levelとbundle height更新を2本で維持する | `legacy_unclassified` | support level集合だけ合っていて、pair化したedgeがlevel 0へ戻る、後続pairと段を取り違える、またはheight更新がpair片側だけにしか効かない回帰を防ぐ |
| C803 | Behavior | Invariant | mount graphはinstance socket親参照を深さ4まで合成し、欠落socketと循環を明示エラーにする | `legacy_unclassified` | row/endpointの2段専用伝搬に戻り、3段以上のモデル配置が動かない回帰を防ぐ |
| C804 | SourceGuard | Boundary | 旧row/endpoint fixtureフィールドは入力adapterだけでPlacementRule化し、MountRef 3種とinterval anchor純関数を文書化する | `source_guard` | 新しい配置種ごとにBundleTemplate専用フィールドやmodel_assembly分岐を増やす、またはビットフラグ組合せで意味を曖昧にする回帰を防ぐ |
| C805 | Behavior | Invariant | route/orderはactive route link内だけの継続補助で、保存済みcontext edgeはsaved pair/row continuityを正本にする | `legacy_unclassified` | 世代違いの保存済みcontext edgeが同じroute/orderを持つだけで継続候補になり、restore時にroute continuity ambiguityを発生させる回帰を防ぐ |
| C807 | SourceGuard | Boundary | pipeline/regenerate/updateはpair/open/row continuityをroute/orderから推測しない | `source_guard` | K完了後にH2の暫定same_route限定やregenerate/updateのroute/order判定が戻り、継続性テーブルと並走する回帰を防ぐ |
| C793 | SourceGuard | Boundary | derive/updateはcached ruleをcanonical span順でplan化し、affected spanはspan_id lookupで解決する | `source_guard` | unordered_map列挙順やaffected spanごとの線形rules探索、support group代表の偶然順序依存が戻る回帰を防ぐ |
| C794 | SourceGuard | Boundary | ViewerActionContextのtimer/cancel/interaction/persistence/factory状態はprivateで、action moduleはmethod経由で操作する | `source_guard` | 役割分割後に巨大共有contextへpublic mutable状態が戻り、action module間でtimerやcancel状態を自由変更する回帰を防ぐ |
| C838 | Behavior | Invariant | span bindingだけからedge_bundle_spans / span_edge_bundle / binding indexを再構築する | `legacy_unclassified` | SavedBackboneEdgeBundle::span_idsを再び所属正本として復活させる回帰を防ぐ |
| C839 | Behavior | Invariant | span bindingのlane→span対応はsave/load後も一致し、span_idsは保存しない | `legacy_unclassified` | lane順をvector挿入順やedge bundle内span一覧から復元する回帰を防ぐ |
| C840 | Behavior | Boundary | 同一edge bundle内のduplicate lane bindingはload時に拒否する | `legacy_unclassified` | lane識別情報が曖昧な保存状態をsilent acceptする回帰を防ぐ |
| C841 | Behavior | Boundary | 同一spanの複数backbone span bindingはload時に拒否する | `legacy_unclassified` | spanが複数edge bundle/laneへ所属する二重authority状態を受け入れる回帰を防ぐ |
| C843 | Behavior | Boundary | continuity構築対象のlaneでPortBinding / SpanBinding / Span endpoint relationが不足・不整合なら、既存continuityを変更せず操作を失敗させる | `legacy_unclassified` | continuity不要bundleの対象外判定と、必須relation欠落のsilent skipを混同する回帰を防ぐ |
| C844 | Behavior | Boundary | promotion frame更新はexact PortBindingを必須識別情報とし、missing / ambiguous / Port relation不成立を正本不変のまま拒否する | `legacy_unclassified` | missing exact bindingをno-op successとしてpromotionを継続する回帰を防ぐ |
| C845 | Behavior | Invariant | permutable 3-lane rowの識別情報 / complete reverse双方で、PortBinding laneが参照するPortとplacement_band_idは同じphysical placementを表す | `legacy_unclassified` | mirror時にlogical span laneのbandをphysical Portへ保存してfixture・curve placementを分裂させる回帰を防ぐ |
| C846 | SourceGuard | Boundary | save_graphのPortBinding保存はplacement_band_idをlogical span laneではなく、選択したPortと同じphysical trow laneから読む | `source_guard` | behavior fixtureが未到達のmirror形状でspan.lane参照が再導入される回帰を防ぐ |
| C847 | Behavior | Invariant | 同一physical edge上のHV/LVが別peer edgeへ接続する独立edge-bundle continuity componentはauthoritative検証、save/load、derived rebuildを維持する | `anchor` | edge-bundle識別情報をload前にphysical edgeへcollapseし、合法な保存状態をambiguousとして読めなくする回帰を防ぐ |
| C848 | Behavior | Invariant | scoped regenerateは対象edge-bundle componentを先に選び、無関係componentをauthority・出力の判断入力にしない | `legacy_unclassified` | 同じphysical edge上の別bundle continuityがHV/LV個別regenerateを失敗または変質させる回帰を防ぐ |
| C849 | Behavior | Boundary | 同一edge-bundle component内で3方向へ分岐するsaved continuityはload時に明示失敗し、既存正本を変更しない | `legacy_unclassified` | component識別情報修正をambiguityのsilent skipやfallbackへ変える回帰を防ぐ |
| C850 | Behavior | Invariant | 同じfixed BundleTemplateを使う独立Bundle placementは、各Bundle識別情報とedge-bundle continuity componentをscopeに3→4更新し、単独実行・配置順反転と同じ意味結果を得る | `legacy_unclassified` | テンプレート全体を一括regenerateして独立componentを混同する、または無関係componentを判断入力にする回帰を防ぐ |
| C851 | Behavior | Invariant | 同じfixed BundleTemplateを使う独立Bundle placementの4→3更新は、各componentのlane 3だけをretireし、残存PortBinding・SpanBinding・continuityをcomponent内に保つ | `legacy_unclassified` | component横断でSpan/Portを削除する、または残存relationを別Bundleへ結ぶ回帰を防ぐ |
| C866 | Behavior | Invariant | range Bundleの個別count更新はexact Bundle識別情報とlane prefix識別情報を保ち、無関係Bundleを判断入力・変更対象にせず、save/load後もbindingを再構築できる | `presence` `anchor` `differential` | テンプレート共有Bundleを巻き込む、存続laneを置換する、退役lane relationを残す回帰を防ぐ |
| C867 | Behavior | FailureAtomicity | range Bundleの退役laneにuser Attachmentまたはmanual Portがあるcount減少はauthority不変で拒否する | `presence` `differential` | user-owned relationを暗黙削除する、または失敗後に一部だけretireする回帰を防ぐ |
| C868 | Behavior | Boundary | fixed BundleTemplateの個別Bundle count変更とrange外count要求はauthority不変で拒否する | `presence` `differential` | count policyを個別overrideとして迂回する回帰を防ぐ |
| C869 | Behavior | Invariant | 同一3点route上のexact Bundleだけを完全退役し、Bundle・EdgeBundle・Span・専用Port・binding・continuity・default Attachment・derived出力・実行時indexを消す一方、peer Bundle識別情報とtopologyを変えずsave/load後も同じ状態を保つ | `presence` `anchor` `differential` | exact scope外を巻き込む、退役relationを残す、load後に削除済みtopologyを復活させる回帰を防ぐ |
| C870 | Behavior | Boundary | cross/sharpを含むexact Bundleの全saved componentを識別情報で退役して対象derived出力だけを消す。source-edge branch Bundleはsourceを残して退役でき、source Bundleに存続projectionがある場合はauthority不変で拒否する | `presence` `oracle` `differential` | branch/cross/sharp/source-edgeを一律unsupportedにする、derived接続曲線を残す、source dependencyを切断する回帰を防ぐ |
| C871 | Behavior | FailureAtomicity | exact BundleのSpanにuser AttachmentまたはSpan override、もしくはbound Portにmanual editがある退役要求はauthority不変で拒否する | `presence` `differential` | user-owned authorityを暗黙削除する、または部分退役後に失敗する回帰を防ぐ |
| C872 | Behavior | Boundary | exact Bundle scopeの必須SpanBindingが不足する退役要求はfail-closedでauthorityを変更しない | `presence` `differential` | incomplete binding scopeをsilent skipしてBundleだけ削除する回帰を防ぐ |
| C873 | Behavior | Invariant | exact anchor Bundleから別placement識別情報を追加し、全physical edge membershipとlane continuityを同型に保つ一方、Bundle/Port/Span識別情報を共有せずpeerを変更しない。save/load後も維持し、追加Bundleの退役でanchor topologyへ戻る | `anchor` `presence` `differential` | テンプレート一致だけの再利用、Port/Span共有、部分component生成、stale binding/cacheを防ぐ |
| C874 | Behavior | Invariant | branch/cross/sharpのexact anchor relationをcategoryや通常pair候補から再判断せず新Bundleへ同型複製し、Jumper/NodePatchは既存derived経路から生成する | `anchor` `oracle` `presence` | branch除外、cross component混同、sharp continuityの再判定、parallel derived実装を防ぐ |
| C875 | Behavior | FailureAtomicity | placement_key重複とexact source Bundle mappingを持たないsource-edge dependent anchorは状態変更前に拒否してauthorityを変更しない | `differential` `presence` | 識別情報衝突とsource dependency推測、部分状態変更を防ぐ |
| C876 | Behavior | Boundary | anchorの必須SpanBindingまたはmulti-lane row continuityが部分欠落したBundle instance追加はfail-closedでauthorityを変更しない | `differential` `presence` | incomplete scopeやlane relationをsilent skipし、欠落したmembershipを複製成功扱いする回帰を防ぐ |
| C877 | Behavior | Invariant | exact currentとdesired concrete Bundle集合が同一ならsemantic状態を変えず、同keyのplacementとkRange count変更ではBundle ObjectIdと存続lane識別情報を維持する | `anchor` `differential` | no-opでID/counterを進める、survivorをretire/addへ置換する、count変更で存続laneを置換する回帰を防ぐ |
| C878 | Behavior | Invariant | current/desiredのkey差分をouter trial内でadd/retireし、A+B、Aへの縮小、A+BからA+Cへの置換、2 -> 4 -> 2を通じて存続Bundle識別情報とexact membershipを維持する | `anchor` `presence` | partial commit、scope外retire、survivor識別情報置換、stale relationを防ぐ |
| C879 | Behavior | Invariant | 3点以上およびbranch/cross/sharpのsaved relationを既存Add/Retireだけでreconcileし、desired/current入力順を反転しても同じkey別semantic topologyを得てsave/load後も維持する | `metamorphic` `anchor` `differential` | 入力vector順を新しいBundle ordering authorityにする、branch membershipやpairingを再判断する回帰を防ぐ |
| C880 | Behavior | FailureAtomicity | survive placement/countとaddの後にlate retireがuser authorityで失敗する場合、およびsource-edge addがunsupportedとなる場合もouterトランザクション全体を元authorityへ戻す | `differential` `presence` | nested generic操作の途中commitを本状態へ漏らす回帰を防ぐ |
| C881 | Behavior | Boundary | duplicate/nonzero key、template/layer/count/spacing、current-scope外anchor、anchor lane topology不一致を状態変更前またはouter trial内で拒否し、scope外の同値keyは別correlationとして扱う | `differential` `presence` | category/template/geometryによるscopeやanchor推測、placement keyのglobal識別情報化、silent normalization、暗黙テンプレートmigrationを防ぐ |
| C882 | Behavior | Invariant | Generate/Applyトランザクションはその場のresolved resultとcommitするplacement識別情報を厳密対応させる一方、loadはstructurally validなdescriptorとsaved exact concrete scopeを復元しcurrent resolverを再実行しない | `anchor` `differential` `presence` | session-only recipe、トランザクション時の非resolver識別情報、resolver変更によるload不能、load時reroll、scope識別情報欠落を防ぐ |
| C883 | Behavior | Differential | explicit Applyは同じplacement keyのBundle識別情報を維持してdensity/height/lateralを更新し、失敗時はdescriptorとconcrete authorityを同時に元へ戻す | `anchor` `differential` `presence` | Apply途中のpartial commit、survivor置換、descriptorとconcrete topologyのずれを防ぐ |
| C884 | Behavior | Boundary | 1 -> 0はBundleと物理topologyを退役し、live exact sourceのない0 -> 1は状態不変のままunsupportedにする | `anchor` `differential` | v1対象外のdormant skeleton/replayを残す、または削除済みmembershipを推測して復活させる回帰を防ぐ |
| C885 | Behavior | Invariant | exact variation extensionで保存したbranch/cross/sharp membershipをexplicit Applyの追加instanceへ複製し、通常derived経路でconnection出力を再構築する | `anchor` `differential` `presence` | branch membership推測、random専用pairing、sharp/cross continuity欠落を防ぐ |
| C886 | Behavior | Boundary | 同じseedの別variationは同じplacement keyを持てるが、Bundle識別情報、descriptor、Apply scopeは完全に独立する | `anchor` `differential` | placement keyをrepo全体のBundle識別情報として扱い、別routeを誤更新するfalse collisionを防ぐ |
| C888 | Behavior | Boundary | variation branch pickはvariation IDからexact live Bundle scopeを解決し、0-instance descriptor ruleをphysical branch modeへ含めない | `presence` `anchor` | Webがrecipe rulesから物理scopeを再構成し、存在しないHV等のpolicyで正当なbranch pickを拒否する回帰を防ぐ |
| C892 | SourceGuard | Boundary | generic pipelineとloaded-output rebuildはvariation retentionをscanせず、live topologyをEdgeBundle relationだけから導出する | `source_guard` | variation固有retentionやdormant replayがpipeline/loadへ戻る回帰を防ぐ |
| C894 | Behavior | Boundary | 現行sharp row契約はcategoryによらずcornerに2 Portを置きJumperで接続する。Communicationの最終viewer sampleで各Jumper endpointは対応するmain body endpointと一致し、両curveのendpoint-away tangentは逆向き同一直線となる | `anchor` `oracle` | 非HVだけ未定義にpoint-through化すること、短い接続線がmain bodyへ非G1で接続する回帰を防ぐ |
| C852 | Behavior | Invariant | row角度差が1e-6 degreeより大きく1e-6 radianより小さい場合も、support levelは実角度順で決まりpath入力方向に依存しない | `metamorphic` `oracle` | atan2のradian値をdegree toleranceと比較し、ID順tieへ約57.3倍広げる回帰を防ぐ |
| C853 | Behavior | Invariant | Portはband/explicit placementの論理anchorを保持し、support level/groupから一度resolveしたbranch downだけが物理endpointへ適用され、save/loadとscoped placement regenerateでも同じ意味結果を保つ | `differential` `oracle` `anchor` | row offset、stable slot、support level loweringが独立にPort/endpoint Zを決めて相互補償する回帰を防ぐ |
| C854 | SourceGuard | Boundary | productionのrow height経路にrow-height offset、stable slot、bundle update時の保存offsetを残さず、row ordering角度はatan2直後にdegreeへ正規化する | `source_guard` | 別経路のno-op故障を他経路が補償する構造とradian/degree混在が再導入される回帰を防ぐ |
| C855 | Behavior | Invariant | 非Autoのmain spanも単一のsag_factorをspan全体へ1回適用し、`chord length * ratio`の物理sag量と独立式`4u(1-u)`のparabolic profileを使う | `oracle` `presence` | continuity policyによるprofile分岐、sag ratio二重加算、長さ・pass・剛性による隠れ倍率を防ぐ |
| C856 | Behavior | Invariant | 同じroute seedとruleはBundle数・placement_key・height・lateralを完全同一に解決し、代表的な別seedは異なる結果になる | `legacy_unclassified` | redrawでrerollする、またはseedを無視して常に同じ配置になる回帰を防ぐ |
| C857 | Behavior | Invariant | route bundle variationは全Bundleを指定sideとrule envelope内に保ち、横位置にもvariationを持つ。resolverはBundle間のclearance policyを推測しない | `legacy_unclassified` | 高さだけの縦積み、左右反転、または入力にない配置制約の再導入を防ぐ |
| C858 | Behavior | Boundary | Communication/Opticalだけのruleは指定テンプレートだけを通常Bundle・Port・Span・saved bindingとして生成する | `legacy_unclassified` | category推測でHV/LVを追加する、またはvisual-only線へ戻る回帰を防ぐ |
| C860 | Behavior | Invariant | acute cornerを通るrandomized multi-bundle routeはbackbone不変条件を満たし、save/load後もconcrete placementを維持する | `anchor` | load時rerollとlane/continuity twist再発を防ぐ |
| C861 | Behavior | Invariant | incremental extensionは同じresolved placementとsource Bundleを使い、既存spanとconnection出力を保つ | `anchor` | extension時rerollと既存visual update scope欠落を防ぐ |
| C862 | Behavior | Invariant | 固定seed群でLV・Communication・OpticalのBundle構成または配置が実際に変わり、Communicationは1 conductor Bundleを複数生成する一方、HV 3相配置は不変 | `legacy_unclassified` | 見た目の束をtopology laneで増やす、またはHVまでrandom化する回帰を防ぐ |
| C863 | Behavior | Invariant | 近いCommunication/Opticalは1本のsupport rowを共有しper-Bundle row fixtureを残さず、visual member数を増やしてもSpan・Port・endpoint fixture・support数は増えない | `legacy_unclassified` | Bundleごとの横棒・belt乱造、visual memberによるattachment増殖、direct fixtureの浮遊を防ぐ |
| C864 | Behavior | Invariant | 非HV logical centerlineは通常のsag curveを維持する。3-member visual bundleは線径由来のcompactな三角断面を全長とendpointで保ち、NodePatchを含む接続曲線にも同じ固定断面を引き継ぐ。非HV sagだけが明示spread内で変化する | `legacy_unclassified` | random wander/phase/twistの復活、endpointで一点へ収束する扇形、接続部だけcenter 1本へ戻る回帰、横一列、member衝突・離脱、HV形状変化を防ぐ |
| C865 | Behavior | Invariant | 保存済みplacementからのmember断面はderived rebuildとsave/loadで同一で、Opticalも同じlogical-span packingから3 member・support・helixを生成し、member外周を共有helix containment内へ保つ | `legacy_unclassified` | rebuild/load時reroll、Communication/Opticalの断面処理分裂、helix外へのmember離脱を防ぐ |
| C895 | Behavior | Differential | Webで公開するDensity 5xとHeight/Lateral spreadの全域はresolverが通常specへ解決でき、既存variationへatomic Applyすると本数および各spread由来のconcrete placementが変化する | `differential` `presence` `anchor` | 選択可能なcontrolがunsupportedになり、またはdescriptorだけ変わってconcrete placementへ反映されない回帰を防ぐ |
| C896 | Behavior | Invariant | Coreはsag済みVisualCurvePartとhelix axis/resolved radiusだけを決定し、asset catalogを知らずにrebuild/save-loadで同じcurve意味論を維持する | `oracle` `differential` `presence` | asset presentation decisionのCore逆流、helix幾何semanticの欠落、GLB導入によるlogical topology変化を防ぐ |
| C836 | Behavior | Invariant | 操作×状態表の各確定セルを実際の正本状態から分類し、各Observation tokenへその操作結果の独立した証拠を結び付けて実行する | `oracle` `presence` `anchor` | テストケース内の別cellやObserve自体の検証条件で未検証cellをcoverage済みにする、または接続状態だけ正しく派生frameが分裂する回帰を防ぐ |

## 廃止済み旧pipeline検査

- 旧ケース 365〜367は登録済みsuiteから削除した。これらはbackbone本流ではなく、移行中の`BackboneBuilder`、support-layout authority seed、materialization surfaceを固定していた。
- これらの制約は現在、saved graph、pair/open/row authority、再計算・再materialization禁止、layout/geometry/drawの利用側chainを検査するbackbone側の証明で保護している。

## LLM自己レビュー

- 実装依存か: private順序/内部関数呼び出し順には依存しない。
- 期待値は観測可能か: すべて公開APIと公開状態で観測。
- モック過多か: モック未使用。
- 異常系が入っているか: C10/C20/C21/C22/C23/C49で失敗診断・状態保全・復帰を検証。
- フレーク要因がないか: 実時間待ち/非決定乱数なし。
