# Core仕様台帳（Phase 4.8）

このledgerは現行のevidence、test family、authority guard mapping、machine coverageのindexであり、
architecture semanticsの正本ではない。Wireの意味は`docs/wire/architecture.md`、
`docs/wire/backbone_operation_semantics.md`、関連するmodel文書を正本とする。

## 対象範囲と方針
- 観測根拠: 公開API戻り値、`view()`、`connection_index()`、`relation_index()`、`find_*cache()`、`port_resolution_debug_records()`、`last_path_direction_debug()`、`last_generation_edge_orientations()`、`SavedBackboneResult()`、`Validate()` のみ。`last_generation_backbone` 系は midair 入力まわりの一時 snapshot 観測としてのみ扱う。
- 期待値粒度: `Exact` は決定論のみ、`Invariant` は不変条件のみ。
- モック方針: ドメインロジックのモック禁止（本スイートはモック未使用）。
- 時間/並行: 実時間待ち・非決定並行を使わない。
- 分類ルール: `分類=Symptom` は最終症状やユーザ価値を直接固定する行、`分類=Authority` は正本・ownership・decision origin を固定する行。
- 永続化 Boundary: `CoreStateAuthoritativeStorage` へ field を追加する変更は、serializer・deserializer・roundtrip test の更新を伴う。

## 実行時coverage方針

- 操作×状態coverageはこのledgerへ手書きしない。`C836`が正本状態を実行時分類し、full `wire_core_tests`の終了時に意味論文書のrequired cellと照合する。
- coverageとして認めるcaseは`oracle` / `anchor` / `presence` / `differential`のいずれかを実行時記録する。`derived_equality`単独は認めない。
- source textを検査するcaseは`SourceGuard` familyで登録し、操作×状態coverageには数えない。
- Web entry coverageは同じ意味論文書の`入口境界`をtestから直接読み、WASM adapterとViewerActionsの実payloadを通す。
- `legacy_unclassified`は既存caseの根拠種別が未移行であることを明示する値であり、runtime coverage evidenceではない。

過去の有効性監査は[`audits/`](audits/)に保管する。この台帳には現行の証明とindexだけを記載する。

## Backbone正本guardの検査範囲

matrix / aspect は外部仕様と観測点を表す。authority guard は、各観点を決める
production式が分散しないことを固定する。`Required owner tokens` は owner に必須、
`Unique production tokens` は `domains/wire/src` と `domains/wire/include` 内で owner にだけ存在できる。
`Forbidden owner tokens` は owner 内で使ってはいけない分岐語彙を示す。

| Authority | Owner | Required owner tokens | Unique production tokens | Forbidden owner tokens |
|---|---|---|---|---|
| row_support_placement | `domains/wire/src/generation/backbone/pipeline.cpp` | `enable_branch_down_offset` `branch_endpoint_offset_m * static_cast<double>(support_level)` `endpoint->branch_down_offset_m` | `branch_endpoint_offset_m * static_cast<double>(support_level)` | `BundleKind::kHighVoltage` `row_height_offsets` `stable_row_slot_plan` `kRowHeightSeparationM` |
| layout_endpoint_lowering_projection | `domains/wire/include/city/wire/span_layout_types.hpp` | `ApplyEndpointLayoutRule` `dst.branch_down_offset_m = rule.branch_down_offset_m` | `ApplyEndpointLayoutRule` |  |
| sharp_representation_threshold | `domains/wire/src/generation/backbone/row_representation.cpp` | `kSharpCornerInteriorAngleMaxDeg` | `kSharpCornerInteriorAngleMaxDeg` |  |
| sharp_jumper_derivation | `domains/wire/src/generation/backbone/curve_parts.cpp` | `endpoint.jumper_peer_port_id` `VisualCurvePartKind::kJumper` |  | `Length(peer->point - endpoint.point)` |

| Case ID | Family | 分類 | 目的 | Evidence | 壊れた時に守りたいユーザ価値 |
|---|---|---|---|---|---|
| C750 | Behavior | Invariant | authoritative save は version 付きで決定的かつ編集を反映する | `legacy_unclassified` | 保存結果の非決定性と編集の取りこぼしを防ぐ |
| C751 | Behavior | Invariant | authoritative load は派生出力をbit一致で再導出する | `legacy_unclassified` | cacheを保存せず同じ表示と形状を復元する |
| C752 | Behavior | Symptom | load後の再saveは元byteと一致する | `legacy_unclassified` | 保存と再読込の反復でproject fileを変質させない |
| C753 | Behavior | Symptom | load後もID衝突なく編集を継続できる | `legacy_unclassified` | 読込後の編集で既存objectを上書きしない |
| C754 | Behavior | Boundary | 不正loadは本stateを変更しない | `legacy_unclassified` | 壊れたfileの部分適用を防ぐ |
| C762 | Behavior | Boundary | ModelDescriptorからassembly partへstable socketを値コピーする | `legacy_unclassified` | transient descriptorへのpointer保持やscene geometryからのsocket再推測を防ぐ |
| C763 | Behavior | Boundary | assembly登録はvalidation後だけ反映し、wire socketとendpoint mount socketを含めsave/load対象になる | `legacy_unclassified` | adapterが正本mapを直書きする、不完全assemblyを部分反映する、またはmount追加で旧saveを読めなくする回帰を防ぐ |
| C01 | Behavior | Symptom | ID生成の単調増加 | `legacy_unclassified` | 永続ID衝突防止 |
| C02 | Behavior | Symptom | ObjectStore整合 | `legacy_unclassified` | 参照崩壊防止 |
| C03 | Behavior | Symptom | Span追加初期Dirty | `legacy_unclassified` | 再計算漏れ防止 |
| C04 | Behavior | Symptom | MovePole局所更新 | `legacy_unclassified` | 無関係再計算抑制 |
| C06 | Behavior | Authority | PoleType適用 | `legacy_unclassified` | テンプレ適用維持 |
| C07 | Behavior | Symptom | PoleType差分 | `legacy_unclassified` | 型差分維持 |
| C19 | Behavior | Symptom | interval Pole生成 | `legacy_unclassified` | interval 自動配置信頼性 |
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
| C117 | Behavior | Authority | Path input では midair policy を入力段階で強制しない | `legacy_unclassified` | DrawPath の入力段階で不要に操作を止めない |
| C817 | Behavior | Boundary | stale pending node id はload後にmutation前拒否される | `legacy_unclassified` | session draftをauthoritative保存したり、load後に壊れたnode idを生成入力として黙って受ける回帰防止 |
| C818 | Behavior | Invariant | pending support node はclearで破棄され生成成功で消費される | `legacy_unclassified` | path cancel後のpending残留や生成成功後のdraft再利用を防ぐ |
| C819 | Behavior | Boundary | path point座標の非有限値はmutation前拒否する | `legacy_unclassified` | raycast由来のNaNがpreflightを抜けて後段のfallbackやNaN geometryを作る回帰防止 |
| C820 | Behavior | Boundary | pole tilt入力の非有限値はmutation前拒否する | `legacy_unclassified` | 外部入力のinf tiltがpole transformやlayoutへ伝播する回帰防止 |
| C821 | SourceGuard | Boundary | BackboneSpec numeric input validationはfield追加漏れを検出する | `source_guard` | 外部入力field追加時にpreflight有限性検証を足し忘れる回帰防止 |
| C822 | Behavior | Boundary | EditResult error kindはvalidation/unsupported/internalを機械可読に分類する | `legacy_unclassified` | viewerやadapterがerror文字列prefixを再解釈し、入力不正・未対応・内部不整合を混同する回帰防止 |
| C823 | SourceGuard | Boundary | test failure diagnostics helperが使える状態で、代表backboneシナリオへ適用されている | `source_guard` | 複数手順testが再びboolだけに戻り、どの操作境界で壊れたか分からない状態へ退行するのを防ぐ |
| C824 | Behavior | Invariant | seed付きroute fuzzは成功時に共通invariantを満たし、拒否時はstateを変えない | `legacy_unclassified` | 手書きscenarioだけでは漏れる参照切れ、NaN geometry、拒否時mutationを短時間で検出する |
| C825 | SourceGuard | Boundary | silent fallback候補の分類表が維持される | `source_guard` | fallbackを発見しても分類せず温存し、後続変更で同じ黙示補完を増やす回帰防止 |
| C826 | SourceGuard | Boundary | core epsilonはsupport配下の共有ヘッダ1つから用途名付き定数を使う | `source_guard` | 退化判定・角度判定・位置比較のepsilonが再び局所aliasやmagic numberへ戻る回帰防止 |
| C827 | SourceGuard | Boundary | model rotationとPoleFrame座標規約がmodels.mdに記録される | `source_guard` | DCC/adapter/viewerが回転順やpole local軸を推測し、fixture/socket位置がずれる回帰防止 |
| C828 | SourceGuard | Boundary | CoreState mutable accessorは本体headerから削除し、残る正本mutable accessはfriend実装として監査する | `source_guard` | 正本mutable参照の裏口が分類なしで増え、決定者一箇所を破る回帰防止 |
| C829 | SourceGuard | Boundary | core政策定数は物理/固定仕様/見た目調整候補へ分類される | `source_guard` | 調整したくなる見た目定数と物理・固定仕様定数が同列のまま増殖する回帰防止 |
| C830 | SourceGuard | Boundary | domains/wire/srcのEditResult error literalは登録済みprefixを持ち、未知prefixはInternal扱いになる | `source_guard` | 文字列prefixなしerrorが黙ってUnsupportedに分類され、入力不正・未対応・内部不整合の区別が崩れる回帰防止 |
| C831 | SourceGuard | Boundary | NormalizedOr/unit_or/safe_unit系fallback呼出はsource上で分類される | `source_guard` | fallback分類がdocsだけにあり、呼出追加時に意図不明な黙示補完が増える回帰防止 |
| C832 | SourceGuard | Boundary | ResolveBranchPick pure化はbridge協定変更としてmerge readinessに保留登録する | `source_guard` | 副作用を持つ照会APIのpure化保留が忘れられ、未定義のまま次のUI/API変更へ混ざる回帰防止 |
| C833 | Behavior | Invariant | branch-down override 0 は最終 lowering 値として fixture socket と curve endpoint まで届く | `legacy_unclassified` | layout が0 overrideを持つのにmodel assemblyがautomatic loweringを復活させる二重決定者の回帰防止 |
| C834 | Behavior | Invariant | 鋭角pairの物理row別support levelはendpoint fixture socketとcurve endpointまで届く | `legacy_unclassified` | support_levelだけ通ってmodel/socket/curveが下がらない、または鋭角2rowを同じ高さへ潰す回帰防止 |
| C835 | Behavior | Invariant | 鋭角pairはcontinuityを保ったまま2つの物理rowを別support levelへ配置する | `legacy_unclassified` | jumper continuityをplacement level共有として扱い、鋭角2rowが同じ高さになる回帰防止 |
| C121 | Behavior | Authority | Template責務の分離 | `legacy_unclassified` | 正本とテンプレ責務の混線防止 |
| C122 | Behavior | Symptom | CableTemplate編集で Pole tilt を上書きしない | `legacy_unclassified` | Pole傾きをテンプレ変更が壊さない |
| C123 | Behavior | Invariant | BundleTemplate の topology 変更は backbone scope を regenerate する | `legacy_unclassified` | topology変更をrejectまたはvisual-onlyで済ませる回帰防止 |
| C124 | Behavior | Symptom | BundleTemplate の visual-only 変更は dirty のみに留める | `legacy_unclassified` | 見た目変更を topology再生成に混ぜない |
| C125 | Behavior | Symptom | ApplyPoleTilt は選択された Pole 実体だけを更新する | `legacy_unclassified` | Pole tilt を実体値として局所更新できる |
| C202 | Behavior | Symptom | Pole tilt は incident span 方向へ寄る | `legacy_unclassified` | Pole tilt が接続方向と無関係に倒れて見える回帰防止 |
| C203 | Behavior | Symptom | Pole tilt 量は pull imbalance に従う | `legacy_unclassified` | 懸垂の効きが弱く、直線と角度付きで同じ傾き量になる回帰防止 |
| C126 | Behavior | Symptom | WorldUp と lateral 軸の整合 | `legacy_unclassified` | 軸依存の手書き計算を1定義に寄せる |
| C127 | Behavior | Symptom | PoleFrame の local/world roundtrip | `legacy_unclassified` | Pole傾き適用の回転順序ずれ防止 |
| C128 | Behavior | Symptom | uベース曲線APIの端点拘束 | `legacy_unclassified` | 曲線生成を u ベースに固定する |
| C129 | Behavior | Symptom | sベース配置APIと sag 合成 | `legacy_unclassified` | 正確な配置を s ベースで扱える |
| C141 | Behavior | Symptom | parabolic sagがsupportで実勾配を持つ | `legacy_unclassified` | supportで平坦になるdecorative sagへ戻る回帰防止 |
| C142 | Behavior | Symptom | 近直線での横揺れ抑止強化 | `legacy_unclassified` | 微小な横成分で左右にぐにゃる回帰防止 |
| C143 | Behavior | Symptom | SmoothPass の横揺れ抑止 | `legacy_unclassified` | 継続カーブで不要な左右揺れを残す回帰防止 |
| C130 | Behavior | Symptom | OffsetEndpoint 端点 | `legacy_unclassified` | 支点近傍で急に折れない離脱表現の土台 |
| C131 | Behavior | Symptom | 鋭角/競合接線での品質劣化安全策 | `legacy_unclassified` | 高度最適化なしで見た目破綻を抑える |
| C148 | Behavior | Symptom | SharpCorner の compact 化 | `legacy_unclassified` | 鋭角部まで SmoothPass 的に膨らむ回帰防止 |
| C144 | Behavior | Symptom | 長尺継続 span の G2優先採用 | `legacy_unclassified` | G2優先方針が単なる係数でなく判定として残ることを固定 |
| C145 | Behavior | Symptom | 短スパンの G1劣化 | `legacy_unclassified` | 短スパンで過拘束 cubic を押し込む回帰防止 |
| C146 | Behavior | Symptom | branch pass の G1優先と端点拘束保持 | `legacy_unclassified` | branch で G2 に固執して support 離脱が崩れる回帰防止 |
| C147 | Behavior | Symptom | PreferG1 は明示選択 | `legacy_unclassified` | continuity 設定と実挙動の意味ずれ防止 |
| C149 | Behavior | Symptom | ViaAttachment の endpoint 優先 | `legacy_unclassified` | attachment 接続を支点中心通過へ戻す回帰防止 |
| C151 | Behavior | Symptom | worldspace variation の連続性 | `legacy_unclassified` | ID乱数で worldspace を離散化する回帰防止 |
| C171 | Behavior | Symptom | public header smoke surface | `legacy_unclassified` | 公開ヘッダの依存漏れや内部 storage 断面への依存が利用者ビルドへ漏れる回帰防止 |
| C181 | Behavior | Symptom | GetPoleDetail は対象 pole の owned endpoint だけを返す | `legacy_unclassified` | owned endpoint の公開観測面が他 pole を混ぜたり relation index 直参照前提に戻る回帰防止 |
| C182 | Behavior | Symptom | MovePole は対象 pole の owned endpoint だけ更新する | `legacy_unclassified` | pole 移動が他 pole の owned endpoint まで巻き込む回帰防止 |
| C200 | Behavior | Authority | bundle の branch down offset policy 変更は topology regenerate | `legacy_unclassified` | branch-down policy 変更が no-opまたはvisual-only扱いになる回帰防止 |
| C277 | Behavior | Symptom | 高低差の大きい smooth span は composite curve へ切り替える | `legacy_unclassified` | 高低差 span が単一 cubic へ戻り、support 近傍で不自然に直線化・急変する回帰防止 |
| C161 | Behavior | Symptom | 長い branch span は横回り込みを抑える | `legacy_unclassified` | branch support の横向きが span 全体を左右へ引っ張る回帰防止 |
| C162 | Behavior | Symptom | branch の局所 departure は短スパンほど強い | `legacy_unclassified` | branch の横成分抑制が span 長に応じて変わらない回帰防止 |
| C163 | Behavior | Invariant | pass種別は端点処理を変えても指定された物理sag量を再加重しない | `oracle` | branch/mainの見た目差を理由にsag量へ隠れ倍率を掛ける回帰防止 |
| C287 | Behavior | Symptom | CommunicationPole はCore所有の日本配電profileを保つ | `legacy_unclassified` | Web起動時mutation撤去後にCore既定profileが古い値へ戻る回帰防止 |
| C289 | Behavior | Symptom | pole type 高さ更新は既存 pole 本体高さと owned auto ports へ再適用される | `legacy_unclassified` | pole type 編集後も既存 pole の本体高さや port 高さが stale のまま残る回帰防止 |
| C296 | Behavior | Authority | default pole template の HV category は単一高さを使う | `legacy_unclassified` | HV band ごとに高さが微妙にずれていて category UI の初期値だけ高く見える回帰防止 |
| C297 | Behavior | Invariant | related pole type の post-edit は backbone を regenerate して適用する | `legacy_unclassified` | 旧直接適用や拒否で SavedGraph と実体を不整合にする回帰防止 |
| C304 | Behavior | Authority | style context resolver は semantic key ごとに deterministic で route-level 傾向を共有する | `legacy_unclassified` | realism resolver が transient id や呼び出し順に依存して不安定になり、route 相関や再現性を失う回帰防止 |
| C309 | Behavior | Symptom | CommunicationPole の communication / optical 既定 band は中央 side を保つ | `legacy_unclassified` | CommunicationPole の既定 band が再び left/right に戻り、光ケーブルや通信ケーブルが pole 配置で左右へ逃げる回帰防止 |
| C310 | Behavior | Symptom | default template は optical-with-support の分岐 type を登録しない | `legacy_unclassified` | optical だけ支持線有無で bundle type が分かれて viewer 上の type が増える回帰防止 |
| C313 | Behavior | Symptom | CommunicationPole の default Pole Placement 値はCore所有のviewer profileと一致する | `legacy_unclassified` | Web起動時patch廃止後にdefault templateとPole Placement UIの初期値がずれる回帰防止 |
| C783 | Behavior | Authority | Core defaults がviewer startup semanticを所有する | `legacy_unclassified` | Webが起動時にupdateCableTemplate/updatePoleTemplate/updateGeometrySettingsで正本を書き換える経路の復活防止 |
| C784 | SourceGuard | Boundary | fixture placement plan は operation 単位で完全構築し、対象外memberを0 offsetで補わない | `source_guard` | fixture plan 構築が affected span 数や endpoint port 数に比例して全cache走査を繰り返す、または operation対象外memberを0 offsetで黙って平均へ混ぜる回帰防止 |
| C357 | Behavior | Invariant | cable decision 更新は backbone 出力を regenerate する | `legacy_unclassified` | decision変更を stale 成功や未再導出にしない |
| C368 | Behavior | Invariant | backbone milestone 1 は単純 line slice を生成する | `legacy_unclassified` | v2 が入口で最小縦スライスを通せない回帰防止 |
| C369 | Behavior | Invariant | backbone は生成 span ごとに SpanLayoutRules を保存する | `legacy_unclassified` | v2 が旧 authority/seed ではなく保存済み rules を正本にする境界確認 |
| C370 | SourceGuard | Boundary | backbone は旧 contract / recalc 内部に依存しない | `source_guard` | v2 が旧 support-layout/recalc wrapper へ戻る回帰防止 |
| C371 | Behavior | Boundary | backbone は未対応入力を fallback せず拒否する | `legacy_unclassified` | 未対応機能を v1 fallback で埋めて境界が崩れる回帰防止 |
| C372 | Behavior | Boundary | backbone は rules 保存で追加入力を要求しない | `legacy_unclassified` | backbone が rules-only 正本から旧 seed/authority contract へ戻る回帰防止 |
| C373 | Behavior | Boundary | backbone は recalc なしで rules から layout を保存する | `legacy_unclassified` | backbone が生成後に dirty/recalc/materialization へ戻らない境界確認 |
| C374 | Behavior | Invariant | backbone layout 導出は同一入力で決定的 | `legacy_unclassified` | cache や既存状態で layout 派生が揺れる回帰防止 |
| C375 | Behavior | Boundary | backbone は recalc なしで layout から curve を保存する | `legacy_unclassified` | backbone が curve 生成で recalc/materialization へ戻らない境界確認 |
| C376 | Behavior | Invariant | backbone curve 導出は同一入力で決定的 | `legacy_unclassified` | cache や既存状態で curve 派生が揺れる回帰防止 |
| C377 | Behavior | Boundary | backbone は recalc なしで curve から bounds を保存する | `legacy_unclassified` | backbone が bounds 生成で recalc/rebuild へ戻らない境界確認 |
| C378 | Behavior | Invariant | backbone bounds 導出は同一入力で決定的 | `legacy_unclassified` | cache や既存状態で bounds 派生が揺れる回帰防止 |
| C379 | Behavior | Boundary | backbone milestone 1 の必須出力を固定する | `legacy_unclassified` | milestone 1 の要求仕様が曖昧になり draw cache まで必須扱いされる回帰防止 |
| C380 | Behavior | Boundary | backbone milestone 1 は draw cache を保存する | `legacy_unclassified` | draw V1 が生成直後の派生出力から抜ける回帰防止 |
| C381 | SourceGuard | Boundary | backbone milestone 1 は recalc contract を持たない | `source_guard` | recalc 不使用が口約束になり旧 contract が backbone に戻る回帰防止 |
| C382 | SourceGuard | Boundary | backbone geom は pipeline 上 1 層として扱う | `source_guard` | curve/bounds が pipeline の独立工程へ戻り、milestone 1 の派生層が細切れになる回帰防止 |
| C383 | SourceGuard | Boundary | backbone draw は pipeline 層として扱う | `source_guard` | draw が layout/geom 以外の工程へ混ざる回帰防止 |
| C384 | SourceGuard | Boundary | backbone topology 出力は `topo` 1 層として扱う | `source_guard` | topology 生成結果が graph や span 専用コンテナへ散って責務が読めなくなる回帰防止 |
| C385 | SourceGuard | Boundary | backbone topology emission は poles/bundles/ports/spans に分かれる | `source_guard` | `emit()` が topology 作成の大きな便利箱へ戻る回帰防止 |
| C386 | SourceGuard | Boundary | backbone は link/pair/open/row を別概念として持つ | `source_guard` | pair/open/row が別名の便利箱になり、junction 分岐が再発する回帰防止 |
| C387 | SourceGuard | Boundary | backbone は pairs を graph から一箇所で作る | `source_guard` | pair 確定が emit/rules/layout に分散する回帰防止 |
| C388 | SourceGuard | Invariant | backbone 3点 route は link/pair/open model で生成される | `source_guard` | 折れ線対応が segment 直書きに戻り、pair 正本を通らない回帰防止 |
| C389 | SourceGuard | Boundary | backbone row axis は pairs 出力が owner になる | `source_guard` | port placement が route geometry から axis を再発明する回帰防止 |
| C390 | SourceGuard | Boundary | backbone は不正な pair incident 入力を拒否する | `source_guard` | pair が曖昧でも fallback で補って downstream に流す回帰防止 |
| C391 | SourceGuard | Boundary | backbone は junction kind label を持たない | `source_guard` | T/cross/branch 分岐が別 enum として戻る回帰防止 |
| C392 | Behavior | Invariant | backbone は3点 route の全 span に必須出力を保存する | `legacy_unclassified` | polyline 対応で rules-only/layout-only/geom 境界が崩れる回帰防止 |
| C393 | Behavior | Invariant | backbone 3点 route 出力は決定的 | `legacy_unclassified` | pair/row 確定が vector 順や既存状態で揺れる回帰防止 |
| C394 | Behavior | Boundary | backbone existing pole node は pole を重複生成しない | `legacy_unclassified` | existing node 対応が新規生成と混ざり、同じ位置に pole を複製する回帰防止 |
| C395 | Behavior | Invariant | backbone `is_new` は pair 出力に影響しない | `legacy_unclassified` | existing/new の違いが pair/row axis の意味決定に混ざる回帰防止 |
| C396 | Behavior | Boundary | existing pole continuation は既存 span を意味決定に再入力せず必要な派生だけを更新する | `legacy_unclassified` | existing pole 周辺の既存 span/layout を topology決定へ再入力する、またはPort reflow後に既存派生をstaleのまま残す回帰防止 |
| C397 | Behavior | Boundary | backbone は missing saved midair node spec を拒否する | `legacy_unclassified` | saved graph なしに existing midair reuse を推測する回帰防止 |
| C398 | Behavior | Boundary | backbone は存在しない existing pole id を拒否する | `legacy_unclassified` | 不正 node id を fallback で新規 pole 化する回帰防止 |
| C399 | Behavior | Invariant | backbone existing pole node sequence は決定的 | `legacy_unclassified` | existing pole node の処理が既存状態順や cache で揺れる回帰防止 |
| C400 | Behavior | Invariant | backbone は同じ pairs graph 上で複数 bundle を生成する | `legacy_unclassified` | multiple bundle が v1 fallback へ戻る回帰防止 |
| C401 | Behavior | Invariant | backbone 複数 bundle 3点 route は全 span に必須出力を保存する | `legacy_unclassified` | multiple bundle 対応で rules/layout/geom 境界が崩れる回帰防止 |
| C402 | Behavior | Boundary | backbone bundle spec は pair 出力に影響しない | `legacy_unclassified` | bundle spec が pair/row axis の意味決定へ混ざる回帰防止 |
| C403 | Behavior | Invariant | backbone existing pole node と multiple bundle は併用できる | `legacy_unclassified` | existing node と multiple bundle の組み合わせが v1 fallback や pole 複製へ戻る回帰防止 |
| C404 | Behavior | Boundary | backbone は empty bundle request を拒否する | `legacy_unclassified` | bundle 無し入力を曖昧に成功させる回帰防止 |
| C405 | SourceGuard | Boundary | backbone pairs make は bundle spec を読まない | `source_guard` | bundle 数や種類で pair/axis が変わる回帰防止 |
| C406 | Behavior | Invariant | backbone port height は pole band を使う | `legacy_unclassified` | 固定高さ 9.32 へ戻る回帰防止 |
| C407 | Behavior | Invariant | backbone multiple bundle height は bundle ごとの pole band を使う | `legacy_unclassified` | 複数 bundle を同一高さへ潰す回帰防止 |
| C408 | Behavior | Invariant | backbone existing pole height は actual pole type を使う | `legacy_unclassified` | existing pole を request type 扱いする回帰防止 |
| C409 | Behavior | Boundary | backbone は missing port band を拒否する | `legacy_unclassified` | band 不在時に固定値や fallback で補う回帰防止 |
| C410 | SourceGuard | Boundary | placement height は pairs に影響しない | `source_guard` | height 決定が pair/row axis 正本へ混ざる回帰防止 |
| C411 | Behavior | Invariant | backbone lateral offset は row axis 方向へ動く | `legacy_unclassified` | lateral offset の符号や方向が別 resolver で揺れる回帰防止 |
| C412 | Behavior | Invariant | backbone lateral offset 符号は決定的 | `legacy_unclassified` | row axis 符号が状態順で反転する回帰防止 |
| C413 | SourceGuard | Boundary | backbone lateral offset は pairs に影響しない | `source_guard` | placement offset が pair/row axis 正本へ混ざる回帰防止 |
| C414 | Behavior | Boundary | backbone は 2点 route + 1 avoid point の simple detour を生成する | `legacy_unclassified` | route 上 avoid point が無条件 unsupported になり viewer 操作が止まる、または downstream が geometry fallback で避ける回帰防止 |
| C415 | SourceGuard | Boundary | backbone は空の levels 層を持たない | `source_guard` | 将来用の空箱が便利箱化する回帰防止 |
| C416 | Behavior | Invariant | backbone `kNotPresent` node mode は no-op | `legacy_unclassified` | no-op 入力が生成意味へ混ざる回帰防止 |
| C417 | Behavior | Boundary | backbone は target row が無い `kPassThrough` node mode を拒否する | `legacy_unclassified` | pass-through 意味決定を未仕様のまま受ける回帰防止 |
| C418 | Behavior | Boundary | backbone は request 外 bundle の node mode を拒否する | `legacy_unclassified` | no-op を broken input ignore と誤解する回帰防止 |
| C419 | Behavior | Boundary | backbone は範囲外 node mode point index を拒否する | `legacy_unclassified` | input point index と内部 graph node id を混同する回帰防止 |
| C420 | SourceGuard | Boundary | backbone node mode は pairs に影響しない | `source_guard` | node mode が pair/row axis 正本へ混ざる回帰防止 |
| C421 | SourceGuard | Boundary | backbone topo row は row source を運ぶ | `source_guard` | topo.rows が port 実体だけを持ち、後段が row 意味を推測し始める回帰防止 |
| C422 | SourceGuard | Boundary | backbone rules は topo と groups を読む | `source_guard` | rules が pairs.links の endpoint row mapping を再び読む、または lowering placement を intent 直読へ戻す回帰防止 |
| C423 | SourceGuard | Boundary | backbone topo span は endpoint row index を運ぶ | `source_guard` | span endpoint row 対応が rules 側へ漏れる回帰防止 |
| C424 | Behavior | Boundary | backbone は saved backbone graph nodes/edges を保存する | `legacy_unclassified` | Pole/Port/Span から backbone を復元するだけの構造へ戻る回帰防止 |
| C425 | Behavior | Boundary | backbone backbone edge は複数 span を edge bundle 経由で持てる | `legacy_unclassified` | pole 間 multiple spans を edge bundle 正本へ束ねられない回帰防止 |
| C426 | Behavior | Boundary | backbone existing pole は saved graph node を再利用する | `legacy_unclassified` | existing pole 接続で graph node が重複する回帰防止 |
| C427 | Behavior | Boundary | backbone graph index は graph と出力を結ぶ | `legacy_unclassified` | 変更対象収集の土台になる index が保存されない回帰防止 |
| C428 | Behavior | Boundary | backbone pole frontier は saved graph incident を集める | `legacy_unclassified` | 変更対象を全 span scan や flag scan で集める回帰防止 |
| C429 | Behavior | Boundary | backbone span frontier は edge bundle 配下 span を集める | `legacy_unclassified` | Span から graph edge を復元せず edge bundle を読む境界の回帰防止 |
| C430 | SourceGuard | Boundary | backbone frontier は saved graph index を読む | `source_guard` | frontier が旧 span-derived topology scan へ戻る回帰防止 |
| C431 | Behavior | Boundary | backbone edge bundle は saved backbone unit | `legacy_unclassified` | backbone graph が span 直下ではなく edge-bundle 正本を持つ境界の回帰防止 |
| C432 | Behavior | Boundary | backbone multiple bundle は複数 edge_bundle を作る | `legacy_unclassified` | bundle を backbone attachment として保存できない回帰防止 |
| C433 | Behavior | Boundary | backbone は同じ pole pair の edge を再利用する | `legacy_unclassified` | 同じ物理 segment が重複 edge になる回帰防止 |
| C434 | Behavior | Boundary | backbone reverse duplicate same bundle は reject する | `legacy_unclassified` | reverse duplicate が span を無制限に増やす回帰防止 |
| C435 | Behavior | Boundary | backbone edge metadata は duplicate reject で上書きされない | `legacy_unclassified` | duplicate reject 時に edge 正本を後書きする回帰防止 |
| C436 | Behavior | Boundary | backbone frontier は edge_bundle 経由で読む | `legacy_unclassified` | frontier が edge-level span 直読みへ戻る回帰防止 |
| C437 | SourceGuard | Boundary | backbone layout 保存は direct store | `source_guard` | backbone layout 保存が旧 support layout/recalc 経路へ戻る回帰防止 |
| C438 | Behavior | Boundary | backbone direct layout 保存は追加入力を要求しない | `legacy_unclassified` | direct layout 保存が seed/authority contract を生やす回帰防止 |
| C439 | SourceGuard | Boundary | backbone source は support layout entrypoint を呼ばない | `source_guard` | backbone が旧 support layout entrypoint へ戻る回帰防止 |
| C440 | SourceGuard | Boundary | backbone は退役 owner 以外で authoritative backbone を直接読まない | `source_guard` | saved graph owner 境界が崩れ、退役以外の段が内部 vector を読む回帰防止 |
| C441 | SourceGuard | Boundary | backbone edge 保存は saved ref を返す | `source_guard` | backbone が edge id だけ受け取り graph vector 検索へ戻る回帰防止 |
| C442 | SourceGuard | Boundary | backbone edge_forward は saved ref から決める | `source_guard` | edge direction 判定が CoreState owner 境界を越える回帰防止 |
| C443 | Behavior | Boundary | backbone edge resolution 境界は duplicate reject 後も維持される | `legacy_unclassified` | API 境界変更で duplicate policy が抜ける回帰防止 |
| C444 | SourceGuard | Boundary | backbone layout source は neutral 型を使う | `source_guard` | backbone layout 層が旧 support layout 型名へ戻る回帰防止 |
| C445 | SourceGuard | Boundary | backbone layout 保存口は neutral entry を受ける | `source_guard` | direct layout 保存口が旧 support layout signature / recalc 経路へ戻る回帰防止 |
| C446 | Behavior | Boundary | backbone neutral layout 境界は生成結果を変えない | `legacy_unclassified` | layout 型名変更で backbone derived outputs や no-authority 境界が崩れる回帰防止 |
| C447 | Behavior | Boundary | backbone layout view は neutral layout を読む | `legacy_unclassified` | backbone の正規 layout 観測口が旧 projection API に戻る回帰防止 |
| C448 | SourceGuard | Boundary | backbone tests は neutral layout read を使う | `source_guard` | acceptance が旧 projection 観測口を正本扱いし続ける回帰防止 |
| C449 | SourceGuard | Boundary | backbone neutral layout read は authority を露出しない | `source_guard` | neutral read API が旧 authority contract を再公開する回帰防止 |
| C450 | Behavior | Boundary | backbone layout state は neutral | `legacy_unclassified` | no-authority 観測が旧 contract API に戻る回帰防止 |
| C451 | SourceGuard | Boundary | backbone tests は support layout contract を読まない | `source_guard` | backbone acceptance が旧 contract 観測口を正本扱いする回帰防止 |
| C452 | SourceGuard | Boundary | backbone layout state は旧 contract 名を露出しない | `source_guard` | neutral 状態 view が旧 contract 語を再公開する回帰防止 |
| C453 | SourceGuard | Boundary | backbone layout state は seed path を読まない | `source_guard` | neutral 状態 view が旧 seed/contract 経路へ戻る回帰防止 |
| C454 | SourceGuard | Boundary | backbone cache state は span layout cache 名を使う | `source_guard` | neutral cache owner が旧 support layout field 名へ戻る回帰防止 |
| C455 | SourceGuard | Boundary | backbone neutral layout API は span layout cache を読む | `source_guard` | neutral API が旧 cache owner 名へ戻る回帰防止 |
| C456 | SourceGuard | Boundary | backbone source は旧 layout cache 名を避ける | `source_guard` | backbone production path が旧 layout cache/read 名へ戻る回帰防止 |
| C457 | Behavior | Boundary | backbone layout cache 境界は生成結果を変えない | `legacy_unclassified` | cache owner rename で backbone derived outputs や no-input 境界が崩れる回帰防止 |
| C458 | Behavior | Boundary | backbone は existing B に B-D を追加できる | `legacy_unclassified` | existing junction context を読めず新規 branch が unsupported に戻る回帰防止 |
| C459 | Behavior | Boundary | backbone は existing B 経由の D-B-E を kind なしで追加できる | `legacy_unclassified` | cross 相当入力を T/cross kind 分岐なしで扱えない回帰防止 |
| C460 | Behavior | Boundary | backbone context link は emit されない | `legacy_unclassified` | saved graph context を topology 生成対象として誤 emit する回帰防止 |
| C461 | Behavior | Boundary | backbone same-edge request は duplicate context を skip する | `legacy_unclassified` | same edge 追加が context duplicate で ambiguous/reject される回帰防止 |
| C462 | SourceGuard | Boundary | backbone existing context は junction kind label を追加しない | `source_guard` | existing junction slice が T/cross/branch 分岐として再発する回帰防止 |
| C463 | Behavior | Boundary | backbone duplicate same edge + bundle は reject する | `legacy_unclassified` | 同一 edge + bundle の追加生成が無制限に span を増やす回帰防止 |
| C464 | Behavior | Boundary | backbone different bundle は同じ edge に追加できる | `legacy_unclassified` | duplicate policy が別 bundle 追加まで拒否する回帰防止 |
| C465 | SourceGuard | Boundary | backbone duplicate policy は existing span を読まない | `source_guard` | duplicate 検出が span resolution/差分補完へ滑る回帰防止 |
| C466 | Behavior | Boundary | backbone duplicate reject は state を変えない | `legacy_unclassified` | unsupported path が途中まで topology/cache を生成する回帰防止 |
| C467 | Behavior | Boundary | backbone row-port binding を保存する | `legacy_unclassified` | 将来 port resolution が座標推測に戻る回帰防止 |
| C468 | Behavior | Boundary | backbone existing context 追加でも new row-port binding だけ保存する | `legacy_unclassified` | context row を materialized port と誤って結びつける回帰防止 |
| C469 | Behavior | Boundary | backbone row-port duplicate は resolve せず reject する | `legacy_unclassified` | existing port resolution 仕様なしに binding を重複作成する回帰防止 |
| C470 | SourceGuard | Boundary | backbone row-port identity は位置近似を使わない | `source_guard` | port identity が座標近似や layout 依存に戻る回帰防止 |
| C471 | Behavior | Boundary | compatible pairもedge endpoint別Port bindingを持つ | `legacy_unclassified` | shared Port IDを接続正本へ戻す回帰防止 |
| C472 | Behavior | Boundary | backbone port resolution は saved binding を要求する | `legacy_unclassified` | 座標近似で v1 由来 port を拾う回帰防止 |
| C473 | Behavior | Boundary | 隣接spanは各edge endpoint Portを参照する | `legacy_unclassified` | Spanがpeer edgeのPortを共有する回帰防止 |
| C474 | SourceGuard | Boundary | backbone port resolution は ambiguous binding を reject する | `source_guard` | 曖昧な resolution を恣意的に選ぶ回帰防止 |
| C475 | SourceGuard | Boundary | backbone port resolution は existing layout を読まない | `source_guard` | port resolution が layout/materialization/位置推測に戻る回帰防止 |
| C476 | Behavior | Invariant | backbone branch rowはkindなしでPortの論理anchorとHV bundle水平中心を維持し、resolved support levelで最終endpointだけを分離する | `oracle` `anchor` | branch相当入力で物理rowが潰れる、Port Zを別decisionで動かす、またはbundle中心を横へ動かす回帰防止 |
| C477 | SourceGuard | Boundary | backbone cross row は kind なしでcontinuityから導出される | `source_guard` | cross 相当入力で未接続openが残る、または cross enum が再発する回帰防止 |
| C478 | Behavior | Invariant | backbone row height separation は deterministic | `legacy_unclassified` | row height separation が unordered context や生成順で揺れる回帰防止 |
| C479 | SourceGuard | Boundary | backbone row height separation は pairs を変えない | `source_guard` | port placement policy が pair/open/row source 決定へ混ざる回帰防止 |
| C480 | Behavior | Invariant | backbone context rowは既存Portを動かさずcontext側port/spanを再materializeせず、branch-down無効bundleの新rowへsupport offsetを混入しない | `oracle` `anchor` | context-only rowを新規port/spanとして生成する、既存Portをreflowする、またはflag無効bundleを高さ分離する回帰防止 |
| C481 | Behavior | Boundary | backbone pass-through mode は限定範囲で受ける | `legacy_unclassified` | node mode を無制限に受けて pair/row 決定へ混ぜる回帰防止 |
| C482 | Behavior | Boundary | backbone pass-through は explicit intent を保存する | `legacy_unclassified` | pass-through 指定が保存されず後段推測へ戻る回帰防止 |
| C483 | SourceGuard | Boundary | backbone pass-through ambiguous target は reject する | `source_guard` | geometry や順序で曖昧な target row を選ぶ回帰防止 |
| C484 | Behavior | Boundary | backbone lowering draw は layout だけを読む | `legacy_unclassified` | draw が lowering/topology を再判断する回帰防止 |
| C485 | SourceGuard | Boundary | backbone lowering intent は existing span を読まない | `source_guard` | intent 決定が v1/recalc/materialization へ戻る回帰防止 |
| C486 | Behavior | Invariant | backbone pass-through intent は deterministic | `legacy_unclassified` | pass-through intent が saved graph traversal 順で揺れる回帰防止 |
| C487 | Behavior | Boundary | backbone port resolution は bundle-compatible scope を要求する | `legacy_unclassified` | row_key/lane だけで異種 bundle port を共有する回帰防止 |
| C488 | Behavior | Boundary | compatible pair bindingはscopeを維持して別Portへ分かれる | `legacy_unclassified` | compatibilityをPort共有と混同する回帰防止 |
| C489 | Behavior | Boundary | generated Portのbinding indexはedge endpoint専用 | `legacy_unclassified` | 1 Portへ複数edge bindingを再導入する回帰防止 |
| C490 | Behavior | Boundary | backbone duplicate same edge/bundle/lane は重複生成しない | `legacy_unclassified` | duplicate request が span/port を増やす回帰防止 |
| C491 | Behavior | Invariant | backbone branch lowering v1 は geom に反映される | `legacy_unclassified` | lowering intent が保存だけで geom に反映されない回帰防止 |
| C492 | Behavior | Invariant | backbone cross lowering v1 は new links だけに反映される | `legacy_unclassified` | context link を再生成する、または lowering を context 出力へ混ぜる回帰防止 |
| C493 | SourceGuard | Boundary | backbone pass-through は pair/open を変えない | `source_guard` | pass-through が connectivity authority へ混ざる回帰防止 |
| C494 | Behavior | Boundary | backbone lowering v1 draw は再判断しない | `legacy_unclassified` | draw が lowering intent を新規判断する回帰防止 |
| C495 | SourceGuard | Boundary | backbone lowering v1 は existing span を読まない | `source_guard` | lowering が v1/recalc/materialization へ戻る回帰防止 |
| C496 | Behavior | Invariant | backbone junction v1 は deterministic | `legacy_unclassified` | existing junction output が traversal/order で揺れる回帰防止 |
| C497 | Behavior | Invariant | backbone context rowはsupport-level constraintに使い未存在portをmaterializeしない | `oracle` `anchor` | context-only rowを新規port/spanとして生成する、またはflag無効bundleのPort anchorを動かす回帰防止 |
| C498 | Behavior | Boundary | backbone saved graph は topology authority のまま | `legacy_unclassified` | topology を spans/layout/seed から復元する構造へ戻る回帰防止 |
| C499 | Behavior | Boundary | backbone context link は save target ではない | `legacy_unclassified` | context link が `save_backbone_edge` 対象になり new link と混ざる回帰防止 |
| C500 | SourceGuard | Boundary | backbone context link は saved edge ref を要求する | `source_guard` | context link を node pair / geometry / span/layout から推測して保存する回帰防止 |
| C501 | SourceGuard | Boundary | backbone Gate 3 契約は save_graph でも維持される | `source_guard` | emit では分離していても save 境界で context/new が混ざる回帰防止 |
| C502 | Behavior | Boundary | backbone span bindingはlane identityを保存し、edge bundleをpair/id indexで参照する | `legacy_unclassified` | edge_bundle.span_idsだけでlaneが分からない、またはsave_graphがbundle/spanごとに全edge_bundlesを線形探索する回帰防止 |
| C503 | SourceGuard | Boundary | backbone duplicate span binding lane は reject する | `source_guard` | same edge_bundle + lane の span が複数保存される回帰防止 |
| C504 | SourceGuard | Boundary | backbone span resolution は geometry/layout を読まない | `source_guard` | span resolution が curve 類似・layout・位置推測へ滑る回帰防止 |
| C505 | SourceGuard | Boundary | backbone save_graph は span binding failure を伝播する | `source_guard` | duplicate span binding を保存境界で黙殺する回帰防止 |
| C506 | SourceGuard | Boundary | backbone support group は placement layer | `source_guard` | support group が便利箱化して topology/visual 責務を持つ回帰防止 |
| C507 | SourceGuard | Boundary | backbone support group は intent 後に作る | `source_guard` | lowering placement が intent/rules/layout に分散する回帰防止 |
| C508 | Behavior | Boundary | backbone support group が lowered rules を駆動する | `legacy_unclassified` | lowering が group を経由せず rules に直書きされる回帰防止 |
| C509 | SourceGuard | Boundary | backbone support group は visual 語を持たない | `source_guard` | support group が draw/support visual の責務を先取りする回帰防止 |
| C510 | SourceGuard | Boundary | backbone layout は fixture placement plan を読む | `source_guard` | layout が placement plan を迂回する、または socket 解決後に lowering を二重適用する回帰防止 |
| C511 | Behavior | Boundary | backbone draw は geom から保存される | `legacy_unclassified` | draw が geom 以外の経路や未保存状態へ戻る回帰防止 |
| C512 | SourceGuard | Boundary | backbone draw は topology を読まない | `source_guard` | draw が topology / pair / row / lowering を再判断する回帰防止 |
| C513 | Behavior | Boundary | backbone lowered layout は obsolete SupportArm を出さない | `legacy_unclassified` | loweringをmodel/socket配置で消費した後に旧SupportArmが復活する回帰防止 |
| C514 | SourceGuard | Boundary | backbone draw save は direct store | `source_guard` | draw 保存が recalc/rebuild 経路へ戻る回帰防止 |
| C515 | Behavior | Boundary | backbone は saved graph の無い existing pole を拒否する | `legacy_unclassified` | v1/手動 scene を暗黙 migration する回帰防止 |
| C516 | Behavior | Boundary | backbone 生成済み pole は existing node として使える | `legacy_unclassified` | M29 gate が backbone 正規 saved graph context まで拒否する回帰防止 |
| C517 | SourceGuard | Boundary | backbone migration gate は outputs から推測しない | `source_guard` | span/layout/seed/curve/position から graph を復元する回帰防止 |
| C518 | Behavior | Boundary | backbone lowered layout は support_world と endpoint_world を最終fixture高さへ一致させる | `legacy_unclassified` | modelを元高さに残し線端だけを下げる旧契約の復活を防ぐ |
| C519 | SourceGuard | Boundary | backbone draw は lowering placeholder を出さない | `source_guard` | SupportArmで旧support/endpoint差を埋める経路の復活を防ぐ |
| C520 | SourceGuard | Boundary | backbone duplicate span binding preflight は emit 前に走る | `source_guard` | duplicate reject が生成後 rollback や保存時 invariant だけに依存する回帰防止 |
| C521 | SourceGuard | Boundary | backbone context link は saved dir を保つ | `source_guard` | context link の保存済み向きを connectivity 段で node geometry から再生成する回帰防止 |
| C522 | SourceGuard | Boundary | backbone supported scope は文書化されている | `source_guard` | supported scope が暗黙化し、実装判断が場当たりになる回帰防止 |
| C523 | SourceGuard | Boundary | backbone scope gate は entrypoint と一致する | `source_guard` | supported scope freeze と入口実装がズレる回帰防止 |
| C524 | SourceGuard | Boundary | backbone simple line scenario は mainline 出力と authority を見る | `source_guard` | 単純線が結果だけ通り、実際は v1/recalc に戻る回帰防止 |
| C525 | Behavior | Boundary | backbone polyline scenario は connectivity 一点決定を守る | `legacy_unclassified` | polyline 対応で downstream が connectivity を再判断する回帰防止 |
| C526 | Behavior | Boundary | backbone multiple bundle scenario は connectivity を共有する | `legacy_unclassified` | bundle spec ごとに pair/open/row を分岐させる回帰防止 |
| C527 | Behavior | Boundary | backbone existing pole continuation scenario は saved graph だけを見る | `legacy_unclassified` | existing 接続が saved graph ではなく span/layout/seed 推測へ戻る回帰防止 |
| C528 | Behavior | Boundary | backbone branch scenario は new link だけ emit する | `legacy_unclassified` | context link を生成・保存対象に混ぜる回帰防止 |
| C529 | SourceGuard | Boundary | backbone cross scenario は kind label なしで通る | `source_guard` | T/cross/branch enum が再発する回帰防止 |
| C530 | Behavior | Boundary | backbone same-edge different-bundle scenario は edge を共有する | `legacy_unclassified` | 同じ edge の bundle 追加が duplicate 扱いになる、または異種 port を共有する回帰防止 |
| C531 | Behavior | Boundary | backbone duplicate scenario は mutation なしで reject する | `legacy_unclassified` | duplicate reject が生成後 rollback 前提になる回帰防止 |
| C532 | Behavior | Boundary | backbone pass-through lowering scenario は consumer chain を守る | `legacy_unclassified` | pass-through が connectivity を変える、または draw が lowering を再判断する回帰防止 |
| C533 | SourceGuard | Boundary | backbone build mutation order は固定されている | `source_guard` | mutation 前に分かる失敗が emit 後へ戻る回帰防止 |
| C534 | Behavior | Boundary | backbone invalid inputs は emit 前に止まる | `legacy_unclassified` | unsupported 入力が途中まで object を生成する回帰防止 |
| C535 | SourceGuard | Boundary | backbone duplicate preflight は mutation boundary | `source_guard` | duplicate reject が保存時 invariant や rollback に依存する回帰防止 |
| C536 | Behavior | Boundary | backbone draw consumer output は最小 | `legacy_unclassified` | draw がmodel/socketで解決済みのloweringを再解釈する回帰防止 |
| C537 | SourceGuard | Boundary | backbone draw source は decision input を読まない | `source_guard` | draw が topology/connectivity/lowering を再判断する回帰防止 |
| C538 | SourceGuard | Boundary | backbone viewer deps は core draw gate ではない | `source_guard` | viewer 環境不備で core backbone acceptance を止める回帰防止 |
| C539 | SourceGuard | Boundary | backbone supported request は saved graph output を作る | `source_guard` | supported scope が v1 fallback や未保存 graph に戻る回帰防止 |
| C540 | Behavior | Boundary | backbone unsupported request は v1 output を作らない | `legacy_unclassified` | unsupported が v1-style output を途中生成する回帰防止 |
| C541 | SourceGuard | Boundary | backbone manual existing pole without graph は gate reject | `source_guard` | v1/manual scene を暗黙 migration する回帰防止 |
| C542 | Behavior | Boundary | backbone usable mainline architecture audit は pass | `legacy_unclassified` | usable mainline 化の境界がまた曖昧になる回帰防止 |
| C543 | Behavior | Boundary | backbone は新規 route interior の `kPassThrough` を supported scenario として通す | `legacy_unclassified` | C番号ではなく supported scenario が増えたことを固定し、saved graph 限定 gate へ戻る回帰防止 |
| C544 | Behavior | Boundary | backbone は generated pole pinning を supported scenario として反映する | `legacy_unclassified` | pole placement 入力を無視する、または existing pole を副作用で manual 化する回帰防止 |
| C545 | Behavior | Boundary | backbone は `interval_m` で中間 pole を生成する | `legacy_unclassified` | interval 入力を無視し続ける、または auto interval node を manual vertex 扱いする回帰防止 |
| C546 | Behavior | Boundary | backbone は explicit new-pole node spec を supported scenario として受ける | `legacy_unclassified` | explicit new pole 指定を壊れた existing id と誤判定する、または不正 id を新規 pole fallback する回帰防止 |
| C547 | Behavior | Boundary | backbone は fixed template の exact count 指定を supported scenario として受ける | `legacy_unclassified` | UI が固定数を明示しただけで reject する、または mismatched count を lane 数変更として受ける回帰防止 |
| C548 | Behavior | Boundary | backbone は avoid radius 単独指定を no-op supported scenario として受ける | `legacy_unclassified` | radius だけの UI 入力を不必要に reject する回帰防止 |
| C549 | Behavior | Boundary | backbone は range template の explicit count を supported scenario として受ける | `legacy_unclassified` | range count を無視する、または範囲外 count をそのまま生成する回帰防止 |
| C550 | Behavior | Boundary | backbone は generated pole の tangent hint を supported scenario として反映する | `legacy_unclassified` | tangent hint を無視する、または topology/connectivity/row placement に混ぜる回帰防止 |
| C551 | Behavior | Boundary | backbone は missing pole type を bundle template から解決する | `legacy_unclassified` | pole type 未指定 UI 入力を不必要に reject する、または混在 bundle から pole type を推測する回帰防止 |
| C552 | Behavior | Boundary | backbone は zero-radius avoid points を no-op supported scenario として受ける | `legacy_unclassified` | disabled avoid UI 入力を不必要に reject する回帰防止 |
| C553 | Behavior | Boundary | backbone は new midair route point を supported scenario として受ける | `legacy_unclassified` | midair 入力を v1 fallback へ戻す、または pole として誤生成する回帰防止 |
| C554 | Behavior | Boundary | backbone は existing saved midair route point を supported scenario として受ける | `legacy_unclassified` | existing midair を span/layout/座標近似から推測する、または pole として誤生成する回帰防止 |
| C555 | Behavior | Boundary | backbone は new building route point を supported scenario として受ける | `legacy_unclassified` | building 入力を v1 fallback へ戻す、または pole として誤生成する回帰防止 |
| C556 | Behavior | Boundary | backbone は building pick を new building route point として受ける | `legacy_unclassified` | viewer building pick が Pole 扱いや saved node id 推測へ戻る回帰防止 |
| C557 | Behavior | Boundary | backbone は object id の無い building pick を受ける | `legacy_unclassified` | viewer raycast が stable building id を持たない場合に不必要に reject する回帰防止 |
| C558 | Behavior | Boundary | backbone は ground pick を new ground route point として受ける | `legacy_unclassified` | ground pick を v1 fallback へ戻す、または attachment/anchor 仕様なしに推測生成する回帰防止 |
| C559 | Behavior | Boundary | backbone は route と交差しない positive avoid を no-op として受ける | `legacy_unclassified` | clear avoid が不要に route を変える回帰防止 |
| C560 | Behavior | Boundary | backbone は selected bundle 未指定の segment dry-run pick を midair route point として受ける | `legacy_unclassified` | UI の path point 解決を bundle 選択前に不必要に止める、または span/layout/seed から topology を推測する回帰防止 |
| C561 | Behavior | Boundary | backbone は selected bundle 未指定の既定 segment pick を ownerless midair route point として受ける | `legacy_unclassified` | UI が bundle 選択前の既定 pick を virtual node id として返し、backbone が saved node 不在で reject する回帰防止 |
| C562 | Behavior | Boundary | backbone は saved midair node pick から延長できる | `legacy_unclassified` | saved midair node を Pole と誤解して backbone extension が saved graph missing で止まる回帰防止 |
| C563 | Behavior | Boundary | backbone は saved ownerless span endpoint へ segment pick から snap できる | `legacy_unclassified` | viewer segment pick payload が endpoint ids を持たない場合に ownerless endpoint snap が効かず、backbone extension が止まる回帰防止 |
| C564 | Behavior | Boundary | backbone は selected bundle 付き segment pick の transient midair node から生成できる | `legacy_unclassified` | selected bundle 指定時だけ virtual node id が返り、backbone が saved node 不在で止まる回帰防止 |
| C565 | Behavior | Boundary | backbone は mixed selected midair branch で許可 bundle だけ生成する | `legacy_unclassified` | 禁止 bundle を巻き込んで生成する、または mixed request 全体を reject する回帰防止 |
| C566 | Behavior | Boundary | backbone は全 bundle disallowed の selected midair branch を no-op にする | `legacy_unclassified` | 禁止 bundle だけの request が途中まで pole/port/span を生成する回帰防止 |
| C567 | Behavior | Boundary | backbone は segment pick midair branch で source projection を使う | `legacy_unclassified` | viewer hit 点の z=0 をそのまま使い、中間分岐が地面から出る回帰防止 |
| C568 | Behavior | Boundary | source-edge midair branch は current source curve projection を使う | `legacy_unclassified` | source edge 上の branch が既存 main context を失う、支持点高さを接続点高さへ混ぜる、または endpoint が source curve から離れる回帰防止 |
| C569 | Behavior | Boundary | backbone render cache は cable template appearance を使う | `legacy_unclassified` | backbone の render cache が default radius/color のままになり、template 表示属性が初期生成に反映されない回帰防止 |
| C570 | Behavior | Boundary | backbone lowering は SupportArm visual を出さない | `legacy_unclassified` | 旧SupportArmがvisual設定経由で復活する回帰防止 |
| C571 | Behavior | Boundary | insulator visual disabled でも lowering は最終socketで解決する | `legacy_unclassified` | insulator visual 設定を無効化しても lowering/layout/geom を壊す、または旧placeholderを復活させる回帰防止 |
| C572 | SourceGuard | Boundary | obsolete SupportArm設定はproduction surfaceへ存在しない | `source_guard` | dead compatibilityとして旧SupportArm familyを残す回帰防止 |
| C573 | Behavior | Boundary | backbone saved context node は support metadata を保持する | `legacy_unclassified` | ownerless/building/ground context node が support/source metadata を失い、後続の context/placement 判断が pole 前提に痩せる回帰防止 |
| C574 | Behavior | Boundary | backbone は existing edge に別 bundle を pass-through mode 付きで追加できる | `legacy_unclassified` | same-edge bundle 追加で node mode が duplicate edge/span 判定や context 解釈を壊す回帰防止 |
| C575 | Behavior | Boundary | backbone は stale segment-pick midair duplicate request を mutation 前に reject する | `legacy_unclassified` | stale pending id が新しい ownerless node と span を再生成し、duplicate policy をすり抜ける回帰防止 |
| C576 | Behavior | Boundary | backbone ownerless-only multiple bundle route は pole type を要求しない | `legacy_unclassified` | ownerless-only route が不要な pole type ambiguity に引きずられて unsupported になる回帰防止 |
| C577 | Behavior | Boundary | backbone missing port band は topology mutation 前に reject する | `legacy_unclassified` | missing band が emit_ports まで進んで partial topology を残す回帰防止 |
| C578 | Behavior | Boundary | backbone segment pick midair branch は pass-through を受ける | `legacy_unclassified` | segment pick 直後の ownerless branch が saved node 未作成という理由だけで pass-through unsupported になる回帰防止 |
| C579 | Behavior | Boundary | backbone は polyline 上の単一 avoid detour を supported scenario として受ける | `legacy_unclassified` | avoid 対応が 2点 line に限定され、polyline 操作が不要に unsupported になる回帰防止 |
| C580 | Behavior | Boundary | backbone は interval と positive avoid detour を segment order で生成する | `legacy_unclassified` | interval point と detour point の順序が逆転し、route が折り返す回帰防止 |
| C581 | Behavior | Boundary | backbone は inactive bundle の missing port band を無視する | `legacy_unclassified` | 生成しない bundle の band 不足で supported request が不自然に reject される回帰防止 |
| C582 | Behavior | Boundary | backbone は同一 segment 上の複数 avoid point を supported scenario として受ける | `legacy_unclassified` | avoid point が複数あるだけで viewer 操作が不要に unsupported になる回帰防止 |
| C583 | Behavior | Boundary | backbone は複数 segment 上の avoid point を supported scenario として受ける | `legacy_unclassified` | polyline の複数 segment に avoid point があるだけで viewer 操作が不要に unsupported になる回帰防止 |
| C584 | Behavior | Boundary | backbone は ownerless route の interval 挿入点を ownerless node として扱う | `legacy_unclassified` | ownerless-only route の中間 interval point が暗黙 pole になり、不要な pole type 要求や pole 生成が起きる回帰防止 |
| C585 | Behavior | Boundary | backbone は重複 avoid point を1つの detour node に畳む | `legacy_unclassified` | duplicate UI payload が同一点 detour node を複数作り、zero-length link で supported 操作が失敗する回帰防止 |
| C586 | Behavior | Boundary | backbone は interval point と avoid detour が同じ source 位置にある場合 detour を優先する | `legacy_unclassified` | interval point が obstacle 中心に残り、avoid detour を作っても route_clear で reject される回帰防止 |
| C587 | Behavior | Boundary | backbone は selected bundle 未指定でも明示 midair node 作成を受ける | `legacy_unclassified` | selected bundle 未指定の viewer 操作が invalid node id になり、後続 backbone node_specs へ渡せない回帰防止 |
| C588 | Behavior | Boundary | backbone は internal corner 上の avoid point を deterministic detour として受ける | `legacy_unclassified` | corner 上の obstacle が segment t=0/1 として detour 対象から外れ、route_clear で supported viewer 操作が止まる回帰防止 |
| C589 | Behavior | Boundary | backbone は selected-bundle midair policy で未選択 bundle を生成しない | `legacy_unclassified` | viewer の選択 bundle から作った midair branch が、未選択 bundle まで一緒に生成する回帰防止 |
| C590 | Behavior | Boundary | backbone は selected policy で inactive になった bundle の pass-through を no-op 前に拒否する | `legacy_unclassified` | viewer の選択範囲で生成対象外になった bundle の node mode が黙って無視される回帰防止 |
| C591 | Behavior | Boundary | backbone は saved selected midair node の継続生成で request bundle を使う | `legacy_unclassified` | 中間分岐の selected policy が保存後の continuation まで絞り、テンプレート指定どおりの延長を妨げる回帰防止 |
| C592 | Behavior | Boundary | backbone は saved selected midair node の reverse continuation でも request bundle を使う | `legacy_unclassified` | reverse continuation だけ saved policy が残り、テンプレート指定どおりの延長を妨げる回帰防止 |
| C593 | Behavior | Boundary | backbone は saved selected midair node で request pass-through を受ける | `legacy_unclassified` | saved node 化後の古い selected policy が明示 request を拒否する回帰防止 |
| C594 | Behavior | Boundary | backbone は route endpoint と完全一致する avoid point を no-op として扱う | `legacy_unclassified` | endpoint と同一点の UI payload が route_clear で不要に unsupported になる回帰防止 |
| C595 | Behavior | Boundary | backbone は明示 existing support と完全一致する avoid point を no-op として扱う | `legacy_unclassified` | UI payload が support 自体を avoid point に含めた時に、既存 support を detour したり unsupported へ落ちる回帰防止 |
| C596 | Behavior | Boundary | backbone は明示 new support と完全一致する avoid point を no-op として扱う | `legacy_unclassified` | clicked support と同じ avoid payload で明示 support が detour されたり unsupported へ落ちる回帰防止 |
| C597 | Behavior | Boundary | backbone は selected building pick で選択 bundle だけを生成する | `legacy_unclassified` | viewer の選択 bundle から building route point を作った時に、未選択 bundle まで一緒に生成する回帰防止 |
| C598 | Behavior | Boundary | backbone は selected saved building node pick で選択 bundle だけを生成する | `legacy_unclassified` | viewer の node pick 経由だけ selected bundle policy が失われ、未選択 bundle まで生成する回帰防止 |
| C599 | Behavior | Boundary | backbone は selected saved building node の bundle policy を branch 後も保持する | `legacy_unclassified` | selected saved node の初回 branch だけ選択 bundle になり、後続 continuation で未選択 bundle が復活する回帰防止 |
| C600 | Behavior | Boundary | pole id pick は saved node id に正規化し selected policy を pole へ刻印しない | `legacy_unclassified` | pole pick に midair/selected policy を刻印し、HVなど未選択扱いの欠落を伝播させる回帰防止 |
| C601 | Behavior | Boundary | backbone は context-only node の selected policy で新規 route bundle を絞らない | `legacy_unclassified` | context 入力だけの selected policy が後続 route の bundle 生成を誤って削る回帰防止 |
| C602 | Behavior | Boundary | saved band を持たない pole type 適用は mutation 前に拒否し、無関係 route は影響を受けない | `legacy_unclassified` | binding band 不足を fallback で補完する回帰と、拒否が無関係 route を汚す回帰を防止 |
| C603 | Behavior | Boundary | backbone は context node で generated endpoint pole yaw を曲げない | `legacy_unclassified` | context node が route node の次要素として扱われ、新規 endpoint pole の向きが branch 方向から外れる回帰防止 |
| C604 | Behavior | Boundary | backbone は大きい avoid 半径でも deterministic detour を半径外へ出す | `legacy_unclassified` | detour point 自体は動いたが折れ線 segment がまだ obstacle 半径内を通り、後段 preflight で supported 入力が reject される回帰防止 |
| C605 | Behavior | Boundary | backbone の route query は saved ownerless backbone graph を読む | `legacy_unclassified` | ownerless endpoint が pole id を持たないため、span-derived SavedBackboneEdges だけでは public route query が空になる回帰防止 |
| C608 | Behavior | Boundary | SavedBackboneResult は saved pole node を重複表示しない | `legacy_unclassified` | saved graph node と materialized pole id node が二重計上され、public query の node 数・junction 判断が膨らむ回帰防止 |
| C607 | Behavior | Boundary | SavedBackboneResult は saved ownerless node の今回 route index を保つ | `legacy_unclassified` | saved graph 正本を優先した結果、debug route metadata が上書きされ、明示 non-pole support を public query で再特定できなくなる回帰防止 |
| C606 | Behavior | Boundary | SavedBackboneResult は saved ownerless node を説明する | `legacy_unclassified` | backbone saved ownerless topology が public query から見えず旧観測 API 経由の実用確認ができない回帰防止 |
| C609 | Behavior | Boundary | 通常の bend は lowering intent を作らない | `legacy_unclassified` | route bend を branch/cross conflict と誤認して lowering する回帰防止 |
| C610 | Behavior | Boundary | row conflict は許可された bundle の junction endpoint だけを offset する | `legacy_unclassified` | pair idをrow idとして扱う、またはmulti-row nodeの全bundle/全endpointを下げる回帰防止 |
| C611 | Behavior | Boundary | backbone direct derive は saved rule から span outputs を復元する | `legacy_unclassified` | post-edit/rederive が `Commit(run_recalc)` や support-layout materialization に戻る回帰防止 |
| C612 | SourceGuard | Boundary | backbone direct derive は recalc/materialization 経路を呼ばない | `source_guard` | direct derive entrypoint が名前だけで内部は recalc wrapper になる回帰防止 |
| C613 | Behavior | Boundary | backbone port edit は Commit なしで generated span outputs を再導出する | `legacy_unclassified` | viewer normal path が port edit 後の表示更新を `Commit(run_recalc)` に依存する回帰防止 |
| C614 | Behavior | Boundary | backbone update plan は4分類だけを使う | `legacy_unclassified` | setterやderive側が操作名別 dirty enum を増やす回帰防止 |
| C615 | Behavior | Boundary | kRegenerate は local fallback 実行しない | `legacy_unclassified` | route/island regeneration 仕様なしで fallback 実装する回帰防止 |
| C616 | Behavior | Boundary | kReposition は SavedBackboneGraph identity を変えない | `legacy_unclassified` | port/pole 移動が topology 再決定や graph 再保存に戻る回帰防止 |
| C617 | Behavior | Boundary | kReshape は layout を書き換えない | `legacy_unclassified` | sag 変更で placement/layout を再判定する回帰防止 |
| C618 | Behavior | Boundary | kRedraw は layout/geom を書き換えない | `legacy_unclassified` | 表示設定変更で geometry や placement を再導出する回帰防止 |
| C619 | Behavior | Boundary | kReposition は affected span だけ更新する | `legacy_unclassified` | affected set が全 span 更新に戻る回帰防止 |
| C620 | SourceGuard | Boundary | update boundary に操作別 kind を作らない | `source_guard` | PoleTilt/PoleYaw/Sag 等の細粒度 enum が増殖する回帰防止 |
| C621 | Behavior | Boundary | sag は geom/draw だけを reshape する | `legacy_unclassified` | backbone wire が直線固定へ戻る、または sag が topology/layout を再決定する回帰防止 |
| C622 | Behavior | Boundary | pipeline/update timing は診断専用 | `legacy_unclassified` | global profiler や timing 起点の生成分岐を作らず性能観測の入口を固定する |
| C623 | Behavior | Boundary | backbone layout settings は regenerate で派生出力を更新する | `legacy_unclassified` | layout settings を stale 成功または全拒否へ戻す回帰防止 |
| C624 | Behavior | Boundary | variation settings は stale 成功しない | `legacy_unclassified` | 未接続設定の成功扱い防止 |
| C625 | Behavior | Boundary | context profile は stale 成功しない | `legacy_unclassified` | 未接続profileの成功扱い防止 |
| C626 | Behavior | Boundary | cable shape/render 更新は direct derive | `legacy_unclassified` | dirty markerなしで出力更新を保証 |
| C627 | SourceGuard | Boundary | 旧 topology mutation API は public surface に残さない | `source_guard` | 旧 API が graph 外 topology 経路として復活する回帰防止 |
| C628 | Behavior | Boundary | active backbone pole type は placement-only なら再導出する | `legacy_unclassified` | Pole Placement UI を既存線へ反映する回帰防止。構造差分は C713 が統一 regenerate と fresh 等価性で固定する |
| C629 | Behavior | Invariant | parabolic main-span curve の意味入力を固定する | `legacy_unclassified` | caller側Bezier handleや横揺れをauthorityにする回帰防止 |
| C630 | Behavior | Invariant | curve sample frame はfiniteかつorthonormal | `legacy_unclassified` | Frenet由来のnear-straight twistやNaN防止 |
| C631 | Behavior | Invariant | reverse traversalでもcanonical lateral frameを維持する | `legacy_unclassified` | save/loadやedge traversal反転でlane左右が反転する回帰防止 |
| C632 | Behavior | Invariant | tessellationはlength/sagに応じる | `legacy_unclassified` | caller固定sample countへの逆戻り防止 |
| C633 | Behavior | Invariant | zero-length/vertical curveを決定的に処理する | `legacy_unclassified` | near-world-up fallback不定・silent method fallback防止 |
| C634 | Behavior | Boundary | terminal node は NodePatchCurve を作らない | `legacy_unclassified` | terminal まで span-local attachment blend を作る回帰防止 |
| C760 | Behavior | Invariant | terminal EdgeBody は resolved port endpoint に直接接続する | `legacy_unclassified` | patch無しを理由に terminal wire が port から浮く回帰を防ぐ |
| C761 | Behavior | Invariant | default OPTICAL bundle はmember endpoint追従のsupportと6 samples/turnのhelixを出力し、COMMは通常線のままにする | `legacy_unclassified` | Helixの既定対象がCOMMへ戻る、固定高support bandで断面が肥大化する、端部だけ動いて懸垂部が残る、またはsamplingが過剰になる回帰を防ぐ |
| C635 | Behavior | Boundary | simple 2-edge continuous node は NodePatchCurve を作る | `legacy_unclassified` | 前後 span がそれぞれ同じ junction 接続部を丸める回帰防止 |
| C636 | Behavior | Boundary | EdgeBodyCurve は NodePatch 境界で止まる | `legacy_unclassified` | edge body が node patch 内まで食い込み、接続部 ownership が混ざる回帰防止 |
| C637 | Behavior | Boundary | NodePatchCurve と EdgeBodyCurve は元の懸垂曲線へG1接続する | `legacy_unclassified` | trim後bodyをHermiteで再構築して端部へ曲率を集中させる、またはpatchとbodyだけを同じ誤接線へ揃える回帰防止 |
| C638 | Behavior | Boundary | visual curve part samples は finite | `legacy_unclassified` | NodePatch/EdgeBody 導入で NaN/inf や空 debug geometry を保存する回帰防止 |
| C639 | Behavior | Boundary | NodePatchCurve は直線 chord ではない | `legacy_unclassified` | 接続部の丸みが消え、patch が debug 名だけの折れ線に戻る回帰防止 |
| C640 | Behavior | Boundary | NodePatchCurve の Bezier debug controls を観測できる | `legacy_unclassified` | 見た目違和感の調査で viewer 側が推測や補正を始める回帰防止 |
| C641 | Behavior | Boundary | Pole tilt 後も visual curve parts が所有 endpoint に追従する | `legacy_unclassified` | direct derive 後に古い curve part cache を残さない |
| C659 | Behavior | Boundary | draw-time tilt は port/span 生成前に support frame へ入る | `legacy_unclassified` | DrawPath 後の final Apply API 依存や、見た目だけ tilt して endpoint が untilted のまま残る回帰防止 |
| C642 | Behavior | Boundary | EdgeBodyCurve は正式 sag curve を共有する | `legacy_unclassified` | quartic sag 二重実装と固定8 sampleへの回帰防止 |
| C643 | Behavior | Boundary | NodePatchCurve はattachment参照を保持した内側fillet | `legacy_unclassified` | attachment通過制約による外振りへの回帰防止 |
| C644 | Behavior | Boundary | patch boundary は main sag実接線の延長上でattachmentより下がる | `legacy_unclassified` | attachmentと同高の水平patchや固定down offsetへの回帰防止 |
| C645 | Behavior | Boundary | NodePatch内側filletは局所曲率を維持する | `legacy_unclassified` | filletがstraight chordへ退化する回帰防止 |
| C646 | Behavior | Boundary | NodePatchはturn内側で単調に曲がる | `legacy_unclassified` | 左折前に右へ外振りする回帰防止 |
| C647 | Behavior | Boundary | NodePatchはincident cableのappearanceを使う | `legacy_unclassified` | source_span_idを持たないpatchだけviewer既定の太さ・色へ落ちる回帰防止 |
| C655 | SourceGuard | Boundary | NodePatch grouping は band identity を含み、pair選択はrow continuityだけを読む | `source_guard` | 同templateの別Bundle、上下band、別placement slotを同じ接続部としてpatch化する回帰防止 |
| C656 | Behavior | Boundary | NodePatch は base section と extra section を混ぜない | `legacy_unclassified` | base線とpopulation線がjunctionで接続される回帰防止 |
| C657 | Behavior | Boundary | extra section は同一 instance だけを接続する | `legacy_unclassified` | extra instance 0と1がjunctionで交差接続される、または同じinstanceがspan境界で切れる回帰防止 |
| C648 | Behavior | Invariant | 同じexplicit seedは同じspan section populationを返す | `legacy_unclassified` | redrawで追加線が並べ替わる回帰防止 |
| C649 | Behavior | Invariant | logical span identityだけではpopulation配置seedを変えない | `legacy_unclassified` | 同じ生成Bundle上の追加線が電柱間ごとに別位置へ飛び、junctionで切れる回帰防止 |
| C650 | Behavior | Invariant | reserveはcandidate pairを拒否する | `legacy_unclassified` | reserved領域へ線を置く回帰防止 |
| C651 | Behavior | Invariant | spacing不足はcandidateを拒否する | `legacy_unclassified` | 密集線が同位置へ重なる回帰防止 |
| C652 | Behavior | Boundary | 片endpoint失敗はsection全体をomitする | `legacy_unclassified` | endpointを独立solveして線がねじれる回帰防止 |
| C653 | Behavior | Boundary | band identity重複を拒否する | `legacy_unclassified` | vector順で曖昧bandを選ぶ回帰防止 |
| C654 | Behavior | Boundary | span section populationはlogical topologyを変更しない | `legacy_unclassified` | 見た目用追加線がtopologyまたは別curve familyになる回帰防止 |
| C658 | Behavior | Boundary | CableCurve は endpoint tangent hints を実サンプルに使う | `legacy_unclassified` | NodePatch境界tangentをEdgeBodyがmetadataだけ持ち、描画samplesでは無視する回帰防止 |
| C688 | Behavior | Invariant | 片側hintのCableCurveはhint無し端の自然なsag端点接線を保つ | `legacy_unclassified` | 3点以上routeの終端spanだけ弦方向接線+端点微分ゼロの装飾sagへ切り替わる回帰防止 |
| C730 | Behavior | Invariant | BundleKind は BundleTemplate identity ではない | `legacy_unclassified` | BundleKind keyed map へ戻る回帰防止 |
| C731 | Behavior | Invariant | BackboneSpec は同 kind 別 template を区別する | `legacy_unclassified` | BackboneBundleSpec が BundleKind identity に戻る回帰防止 |
| C732 | Behavior | Invariant | population rule owner は BundleTemplateId | `legacy_unclassified` | BundleKind 単位で population rule が混線する回帰防止 |
| C733 | Behavior | Invariant | regenerate scope は BundleTemplateId | `legacy_unclassified` | regenerate scope が kind で集まる回帰防止 |
| C734 | Behavior | Invariant | cable template lookup は kind based ではない | `legacy_unclassified` | CableTemplate lookup が BundleKind に戻る回帰防止 |
| C735 | SourceGuard | Boundary | source guard は BundleKind identity 残存を拒否する | `source_guard` | 永続化前に enum identity を焼き戻す回帰防止 |
| C736 | SourceGuard | Boundary | unsupported保留docsは対応済みbackbone更新を戻さない | `source_guard` | docsだけが古い unsupported を再導入し、viewer後追い作業を誤誘導する回帰防止 |
| C737 | Behavior | Boundary | overlay edge endpoint snap は saved node spec id を返す | `legacy_unclassified` | web overlay edge endpoint から伸ばす操作が unknown node reference で止まる回帰防止 |
| C837 | Behavior | Boundary | source bundle付きendpoint completionは別bundleのopen rowをpromotion候補にしない | `legacy_unclassified` | 終点へつなぐ操作で同種別の別route bundleを同じplacementへ混ぜ、promoted placement maps to multiple bundlesで拒否する回帰防止 |
| C691 | Behavior | Invariant | run id は through section を接続する | `legacy_unclassified` | section scope idのままspan境界で見た目上1本のcableが分断される回帰防止 |
| C692 | Behavior | Invariant | run id は世代跨ぎterminal extensionを接続する | `legacy_unclassified` | 別生成回の連続sectionが別run扱いになる回帰防止 |
| C693 | Behavior | Invariant | branchとdead-endは別runになる | `legacy_unclassified` | jumperやbranchをpatch連続として誤集約する回帰防止 |
| C694 | Behavior | Invariant | population instance ごとに run を接続する | `legacy_unclassified` | baseとpopulationまたは別instanceを同じrunへ混ぜる回帰防止 |
| C695 | Behavior | Invariant | identityを持つcable sectionのrun idは決定的 | `legacy_unclassified` | identityなしsupport/helixをrunへ偽装せず、pointer/order依存の不安定run id回帰を防止 |
| C696 | SourceGuard | Boundary | run idはvisual derive層の派生値で、section indexから決定する | `source_guard` | run identityをtopology正本へ保存する、またはsectionごとの線形探索へ戻る回帰防止 |
| C660 | Behavior | Boundary | regenerate fixed count increase は SavedGraph identity を維持して下流だけ更新する | `legacy_unclassified` | route-local regenerate が入力replayや位置推測でgraphを作り直す回帰防止 |
| C661 | Behavior | Invariant | pair row axis は径間長ではなく単位接線二等分で決める | `legacy_unclassified` | pass-through cornerの横並びが長い径間へ引っ張られる回帰防止 |
| C662 | Behavior | Invariant | pair row axis は incident span の lane順を反転しない | `legacy_unclassified` | row axis変更で片側spanのlane順がねじれる回帰防止 |
| C663 | Behavior | Invariant | 3相HV鋭角 continuity は最終Port列に従ってlaneをmirrorし、径間別dead-end rowと3 Jumperを導出する | `oracle` `anchor` | span内部だけ直ってcross-edge continuity/Jumperがsame-laneのままねじれる回帰防止 |
| C664 | SourceGuard | Boundary | 鋭角pole facingはpairsのcorner decisionを消費し再判定しない | `source_guard` | pole yawとrow/jumperが別々に鋭角を解釈する回帰防止 |
| C665 | Behavior | Boundary | source-edge projectionは派生curve上に置く | `legacy_unclassified` | 途中分岐が既存cableから浮く回帰防止 |
| C666 | Behavior | Boundary | 世代を跨ぐterminal continuationをcontinuityで接続する | `legacy_unclassified` | point proximityや同一port偶然一致へ依存する回帰防止 |
| C667 | Behavior | Boundary | branch追加後もthrough patchを維持する | `legacy_unclassified` | group size 3で既存through丸めが消える回帰防止 |
| C697 | Behavior | Invariant | saved edge は lateral offset 生成入力を echo 保存する | `legacy_unclassified` | 保存済み graph から復元できない placement constraint を regenerate 前処理で使えるようにする |
| C668 | Behavior | Invariant | regenerate fixed count increase は saved lateral offset を消費する | `legacy_unclassified` | placement echo を使わず既定 offset で増加再生成する回帰防止 |
| C669 | Behavior | Invariant | multi-bundle regenerate は count 減少も対象 bundle だけを変え fresh と一致する | `legacy_unclassified` | multi-bundle count 減少で非対象 bundle を動かす、または対象 bundle の group offset を誤る回帰防止 |
| C670 | Behavior | Invariant | regenerate は pair row を含む3点routeを fresh と一致させる | `legacy_unclassified` | regenerateがendpoint Port identityを共有へ戻す、またはpair lane位置を分離する回帰防止 |
| C671 | SourceGuard | Boundary | regenerate は pipeline stage を再利用し専用 emit を持たない | `source_guard` | M1 regenerate が第2 pipeline へ戻る回帰防止 |
| C672 | Behavior | Invariant | regenerate は存続 lane の manual port を保持する | `legacy_unclassified` | 手編集済み port を regenerate が黙って上書きまたは拒否する回帰防止 |
| C673 | Behavior | Invariant | regenerate は存続 span の user attachment を保持する | `legacy_unclassified` | 存続 span のユーザー追加 attachment を regenerate が不要に拒否または喪失する回帰防止 |
| C698 | Behavior | Invariant | regenerate fixed count decrease は退役 lane を削除し残存出力を fresh count1 と一致させる | `legacy_unclassified` | count 減少を migration 複製ではなく reconcile 付き regenerate で扱う |
| C699 | Behavior | Invariant | regenerate fixed count decrease は saved lateral offset を消費する | `legacy_unclassified` | regenerate 前処理が placement echo を使わず既定 offset で再生成する回帰防止 |
| C700 | Behavior | Boundary | regenerate fixed count decrease は退役対象 attachment を mutation 前に拒否する | `legacy_unclassified` | ユーザー attachment 付き span を暗黙退役する回帰防止 |
| C701 | SourceGuard | Boundary | regenerate は派生出力から topology を推測しない | `source_guard` | 差分再生成が span/layout/curve/port 位置から topology を復元する第2 pipeline へ逸れる回帰防止 |
| C702 | Behavior | Invariant | regenerate fixed count は増減混在後も fresh count1 と一致する | `legacy_unclassified` | 増加だけ旧 migration、減少だけ regenerate という分岐を残す回帰防止 |
| C742 | Behavior | Invariant | fixed count 減少は metadata rename と同時でも統一 regenerate を通る | `legacy_unclassified` | count 減少側だけ metadata 一致を要求し、増加側と受理範囲が分岐する回帰防止 |
| C743 | SourceGuard | Boundary | BundleTemplate field diff の決定者は classifier 一箇所だけである | `source_guard` | routing block が field 条件を再列挙し、分類器と受理範囲が再び drift する回帰防止 |
| C744 | SourceGuard | Boundary | lowered support group key の収集は一箇所だけである | `source_guard` | generation と post-edit derive が group key/default/basis を別々に転記して drift する回帰防止 |
| C745 | SourceGuard | Boundary | 旧population wrapとCableTemplate coil familyはproduction surfaceに残らない | `source_guard` | 旧carrier-following wrapまたはCableTemplate coilが新assemblyと並存する回帰防止 |
| C746 | Behavior | Invariant | generation は isolated trial を成功時だけ move commit し、66 pole級stateで copy は総時間の20%以内 | `legacy_unclassified` | 空stateだけを測ってcopy costを過小評価する、またはpipeline後半failureが本stateへ漏れる回帰防止 |
| C747 | Behavior | Invariant | range count policy は保存済み conductor count を検証し、範囲内なら出力を再生成せず、範囲外なら対象 bundle を示して state 不変で拒否する | `legacy_unclassified` | range化を不要にregenerateする、または範囲外の既存 bundle を黙って不整合にする回帰防止 |
| C748 | Behavior | Invariant | bundle policy差分はscopeをfresh等価へreconcileし、fresh-invalid入力はmutation前に拒否する | `legacy_unclassified` | topology policyをrejectし続ける、scope外を再導出する、freshでは拒否される入力をpartial mutationする回帰防止 |
| C749 | Behavior | Invariant | Offset=0はband既定位置から追加移動せず、他bundleにも影響されない | `legacy_unclassified` | bundle index由来の隠れた横offset、band中心無視、HV全laneの単一band展開への回帰防止 |
| C789 | Behavior | Invariant | multi-route同一band rowはPortを共通の論理anchorに保ちながら、support levelを適用した最終endpoint間隔を保つ | `oracle` `anchor` | 論理Portの人工的Z分離を復活させる、または物理fixture/socketのrow separationを失う回帰を防ぐ |
| C790 | Behavior | Boundary | 既存supportと同一座標のpath pointは明示node referenceを要求する | `legacy_unclassified` | 既存supportを暗黙選択または重複poleとして後段処理へ進め、pair/occupancy探索を肥大化させる回帰防止 |
| C703 | SourceGuard | Boundary | bundle count migration シンボルは domains/wire/src から消える | `source_guard` | 差分別 migration operation が残る回帰防止 |
| C704 | SourceGuard | Boundary | regenerate は post-edit API から直接呼ばれ update plan 経由で実行しない | `source_guard` | pipeline 内に migration 語が残る、または plan が差分入力なしで regenerate 実行経路になる回帰防止 |
| C705 | Behavior | Invariant | saved edge_bundle の順序は request bundle spec 順と一致する | `legacy_unclassified` | multi-bundle regenerate が group offset を保存済み edge_bundles 順から復元する前提を固定 |
| C706 | Behavior | Invariant | multi-bundle regenerate は対象 bundle だけを増減し fresh と一致する | `legacy_unclassified` | multi-bundle group offset を単一bundleとして再生成したり、非対象 bundle を動かす回帰防止 |
| C707 | Behavior | Invariant | 3点routeの継続はrow continuity tableに保存される | `legacy_unclassified` | regenerate が saved edge route/order ではなく row continuity を継続正本として使うことを固定 |
| C708 | Behavior | Invariant | 3点route regenerate は count 減少後 fresh と一致し、scope外routeを触らない | `legacy_unclassified` | pair row を含む route-local regenerate が一部 edge だけを再生成する、またはscope外routeを再導出する回帰防止 |
| C709 | Behavior | Invariant | 3点route regenerate は count 増加後 fresh と一致し、scope外routeを触らない | `legacy_unclassified` | lane増加時にendpoint別identityとpair位置一致を崩す、またはscope外routeを再導出する回帰防止 |
| C710 | Behavior | Invariant | multi-bundle 3点route regenerate は fresh と一致し、scope外routeを触らない | `legacy_unclassified` | R1 の group offset 復元と R2 の pair row 復元を同時に満たし、scope外routeを再導出しない |
| C711 | Behavior | Invariant | regenerate count 減少は存続 lane の attachment を保持する | `legacy_unclassified` | count 減少で存続 span の attachment まで拒否または削除する回帰防止 |
| C712 | Behavior | Invariant | cable decision 差分は regenerate で fresh と一致する | `legacy_unclassified` | decision 差分を stale 成功や direct derive に逃がさず統一 regenerate へ通す |
| C738 | Behavior | Invariant | default endpoint attachment 差分は auto endpoint だけを reconcile する | `legacy_unclassified` | default endpoint差分を未消費またはuser attachmentまで置換し、scope外を再導出する回帰防止 |
| C713 | Behavior | Invariant | pole type structural 差分は regenerate で fresh と一致する | `legacy_unclassified` | pole template 構造差分を reject や stale 成功にせず統一 regenerate へ通す |
| C714 | Behavior | Boundary | regenerate は退役 lane の manual port を mutation 前に拒否する | `legacy_unclassified` | 手編集済み port を退役時に暗黙削除する回帰防止 |
| C715 | Behavior | Invariant | backbone span branch-down override は regenerate で layout/curve に反映する | `legacy_unclassified` | backbone span override を拒否または保存だけして派生出力を stale にする回帰防止 |
| C716 | Behavior | Invariant | backbone span endpoint socket override は regenerate で layout に反映する | `legacy_unclassified` | socket override を非backbone専用の保存状態に閉じ込める回帰防止 |
| C739 | Behavior | Invariant | span override regenerate は別routeを触らない | `legacy_unclassified` | span-local override が全route regenerate や雑なscope収集へ広がる回帰防止 |
| C740 | Behavior | Invariant | geomで構築済みのbase curveをvisual curve partsが優先再利用する | `legacy_unclassified` | pipelineとvisualが同じbase spanのfull curveを二重構築する回帰防止 |
| C741 | Behavior | Invariant | A-B-CへC-Dを追加したscoped visual rebuildは隣接B connectionを保持して全量再構築と一致する | `differential` | read contextの反対側nodeをwrite scopeへ昇格し、既存NodePatch/Jumperを削除する回帰防止 |
| C717 | Behavior | Invariant | layout settings 差分は regenerate で fresh と一致する | `legacy_unclassified` | layout settings 差分を部分 derive や stale 成功に逃がさず統一 regenerate へ通す |
| C727 | SourceGuard | Boundary | pipeline execution entry は build(build_input) だけにする | `source_guard` | regenerate が第2 pipeline 化し、通常生成と別stage列を持つ回帰防止 |
| C728 | SourceGuard | Boundary | pipeline は run mode / skip flag を持たない | `source_guard` | operation固有条件や検証skipが pipeline mode 分岐として復活する回帰防止 |
| C729 | SourceGuard | Boundary | regenerate source は pipeline output と退役を手組みしない | `source_guard` | regenerate.cpp が第2 pipeline や fallback 更新経路、退役ループを持つ回帰防止 |
| C718 | Behavior | Boundary | viewer hit world height は source-edge branch の正本ではない | `legacy_unclassified` | viewer preview 座標や source projection を支持点高さとして固定する回帰防止 |
| C719 | Behavior | Invariant | source-edge branch endpoint は current curve projection に追従する | `legacy_unclassified` | source-edge branch endpoint が作成時 port 位置に stale 固定される回帰防止 |
| C720 | SourceGuard | Boundary | source-edge pipeline 前半は curve projection を読まない | `source_guard` | topology/port materialization 段が source cable geometry を必須入力に戻す回帰防止 |
| C721 | Behavior | Invariant | source-edge identity は projection update 後も残る | `legacy_unclassified` | projection 更新を world height 保存へすり替える回帰防止 |
| C722 | Behavior | Boundary | unresolved source-edge reference は mutation 前に失敗する | `legacy_unclassified` | source identity 不解決時に途中まで entity を作る回帰防止 |
| C723 | Behavior | Invariant | source-edge branch は source sag を変えない | `legacy_unclassified` | branch 追加を source cable 物理荷重として循環させる回帰防止 |
| C724 | Behavior | Invariant | source template sag 変更は branch projection を更新する | `legacy_unclassified` | sag/template 更新で branch endpoint が stale になる回帰防止 |
| C725 | Behavior | Invariant | source layout settings 変更後も branch projection は current のまま | `legacy_unclassified` | layout更新で source-edge branch endpoint が stale になる、または ownerless source-edge route regenerate が拒否される回帰防止 |
| C726 | Behavior | Boundary | source-edge branch projection は prior curve cache を要求しない | `legacy_unclassified` | pipeline 後半が prior curve cache に依存し、一発生成や cache 欠落時に branch endpoint を解けない回帰防止 |
| C674 | SourceGuard | Boundary | backbone port の band は saved binding、非 backbone port は共有 selector が決める | `source_guard` | 下流が band identity を再推測する回帰防止 |
| C675 | SourceGuard | Boundary | port layout yaw は saved binding を読み debug record を決定入力にしない | `source_guard` | debug 観測値が production 配置を決める回帰防止 |
| C676 | Behavior | Invariant | generation と post-edit は同じ saved layout yaw を使う | `legacy_unclassified` | generation と endpoint refresh の yaw 決定者が分岐する回帰防止 |
| C677 | SourceGuard | Boundary | corner side scale と inner side 判定は1定義だけを持つ | `source_guard` | 同じ角補正式が state/template/refresh で分岐する回帰防止 |
| C678 | Behavior | Invariant | ModelDescriptor merge は決定的で conflict を補完しない | `legacy_unclassified` | import/adapter 前に descriptor 契約を pure logic として固定し、fallback 補完や state 依存へ流れる回帰を防ぐ |
| C679 | Behavior | Invariant | 使用中 AttachmentTemplate の幾何差分は curve を再導出する | `legacy_unclassified` | モデル再読込の socket 位置変更が全拒否され、手動で置いた attachment の曲線が stale になる回帰を防ぐ |
| C680 | Behavior | Boundary | 使用中 AttachmentTemplate の構造差分は mutation 前に拒否する | `legacy_unclassified` | socket identity 変更を geometry 更新として通し、既存 attachment の接続意味を壊す回帰を防ぐ |
| C681 | Behavior | Invariant | path無し Replace は hidden interval のみ生成する | `legacy_unclassified` | モデル本体が区間を埋める前提の attachment を internal path 必須に戻す回帰を防ぐ |
| C682 | Behavior | Boundary | 同一 span の replaced interval 重複は Error とする | `legacy_unclassified` | 置換区間同士が重なって線の可視区間が曖昧になる回帰を防ぐ |
| C683 | Behavior | Invariant | ModelDescriptor から path無し AttachmentTemplate を生成する | `legacy_unclassified` | DCC marker descriptor を state や viewer に依存せず core template へ変換できることを固定する |
| C684 | Behavior | Invariant | 端子函 descriptor の drop socket は補助 socket として保持する | `legacy_unclassified` | 追加 socket を topology や line path と誤解して補完生成する回帰を防ぐ |
| C685 | Behavior | Invariant | descriptor-built template は既存更新経路で使用中 span を更新する | `legacy_unclassified` | モデル再読込専用経路を作らず、通常 template 更新経路で使用中 attachment を更新する契約を固定する |
| C686 | Behavior | Boundary | BundleTemplate の population rule は派生線だけを追加する | `legacy_unclassified` | 追加平行線を topology や saved graph として保存する回帰防止 |
| C687 | Behavior | Boundary | population rule 差分は kReshape として再導出する | `legacy_unclassified` | rule 変更を regenerate 級として拒否したり topology_change に混ぜる回帰防止 |
| C755 | Behavior | Invariant | 鋭角Jumperはincident spanへG1接続する | `legacy_unclassified` | 鋭角接続がport間chord接線で折れる既存不具合の固定 |
| C756 | SourceGuard | Boundary | write/readは型別wrapperを持たず同じarchive visitorから導出する | `source_guard` | field列挙一元化後にwrite/read二重familyが再発することを防ぐ |
| C757 | Behavior | Invariant | load後authoritativeは同じarchive field列挙で直接一致する | `legacy_unclassified` | writer/reader双方から同じfieldが落ちbyte一致だけ通る盲点を防ぐ |
| C758 | Behavior | Invariant | enabled assemblyはlogical spanごとにsupport pathとhelixを1本ずつ派生する | `legacy_unclassified` | Bundle全laneを一体化したりauthoritative topologyを増やす回帰を防ぐ |
| C759 | SourceGuard | Boundary | support primary curve・helix・containment・wander・twistはspan visual assemblyだけが決定する | `source_guard` | category別support実装やvisual assemblyをtopology／curve_partsへ分散させる回帰を防ぐ |
| C764 | Behavior | Invariant | straight HVはPoleとderived row/lane単位でmodelを派生しwire socketをcurve端点にする | `legacy_unclassified` | Port単位fixture二重化、Web後合わせ、laneごとの腕金重複、belt基準混同、post-edit時のstale modelを防ぐ |
| C765 | Behavior | Invariant | branch loweringはfixture placementへ一度だけ適用し、最終model socketをcurve endpointにする | `legacy_unclassified` | modelを元高さに残して線端だけ下げる、またはedge endpoint Portごとにfixtureを二重生成する回帰を防ぐ |
| C766 | Behavior | Invariant | row mount・endpoint fixture・接続線は一つのendpoint placementを共有し、band lateral変更へ追従する。beltは柱軸に残る | `legacy_unclassified` | 腕金のmodel offsetだけが碍子・線へ伝わらない、腕金だけがband offsetへ追従しない、またはbeltまで柱から外れる回帰を防ぐ |
| C767 | Behavior | Invariant | default HV/LVのsupport pathはOpticalと同じprimary curve生成を使い、接続曲線境界で切れず解決済みendpointへ届く | `legacy_unclassified` | EdgeBody複製によるnode patch境界での切断、カテゴリ別curve family、碍子socket無視、support-onlyでmember geometryを変える回帰を防ぐ |
| C769 | Behavior | Invariant | 同じBundleTemplateを複数配置してもBundle単位でidentity・絶対配置・node patch・中間分岐attachmentを分離する | `legacy_unclassified` | template id集合へ畳む、同templateの最初のsource attachmentを使う、support/helix/node patchだけ旧位置や別Bundleへ混ざる回帰を防ぐ |
| C770 | Behavior | Invariant | Bundle placementのheight更新はPortを新しいexplicit base heightへ置き、保存済みsupport level/groupによるrow placementを維持して碍子・腕金・線端へ反映する | `oracle` `anchor` | placement更新で旧Port offsetを別authorityとして温存する、またはmodel socket補正を線端から落として交差部の碍子・腕金と線を分離する回帰を防ぐ |
| C771 | Behavior | Invariant | incremental cross completion は既存endpoint identityを維持して2 continuityへ収束する | `legacy_unclassified` | degree four のsaved/current差で未接続openが残る、または接続時に既存endpoint frameを動かす回帰を防ぐ |
| C772 | Behavior | Boundary | incremental pairing は複数の未接続候補を推測せず新edgeをopenで残す | `legacy_unclassified` | first-match、最小角度選択、曖昧時の操作拒否を防ぐ |
| C773 | Behavior | Invariant | sharp corner の incremental completion はcontinuityを記録し2 dead-end rowとjumperを導出する | `legacy_unclassified` | sharp接続を未接続として保存する、または通常NodePatchとして描く回帰を防ぐ |
| C774 | Behavior | Invariant | bundle/port scope が違う incremental completion は既存Portを誤共有しない | `legacy_unclassified` | row keyだけで異scopeのport bindingを再利用する回帰を防ぐ |
| C775 | Behavior | Invariant | incremental canonical pair row は save/load 後も維持される | `differential` `anchor` | 保存形式やload再派生でcanonical pairがopenへ戻る、または保存Spanと再導出endpointの向きが分裂する回帰を防ぐ |
| C776 | Behavior | Invariant | incremental canonical pair row は regenerate 後も維持される | `legacy_unclassified` | regenerateがsaved open由来のpairを3rowへ戻す回帰を防ぐ |
| C777 | Behavior | Invariant | reverse input direction でも incremental completion は同じcontinuityと派生rowへ収束する | `legacy_unclassified` | 入力方向によってcontinuityやrow配置が反転する回帰を防ぐ |
| C778 | Behavior | Invariant | multi-template の incremental completion は各互換scopeを1回だけ接続し、別scopeや別openを曖昧扱いしない | `legacy_unclassified` | endpoint bindingをscope外まで汎用探索して曖昧化する回帰を防ぐ |
| C779 | Behavior | Invariant | same-template multi-placement の incremental completion は placement_key で既存Bundleへ接続しPortはedge別に保つ | `legacy_unclassified` | duplicate placement混同、shared Port再導入、HV欠落を防ぐ |
| C780 | Behavior | Invariant | 同じtemplate・同じ数値配置でもplacement_keyが違えば別placementとして扱い、request順序に依存しない | `legacy_unclassified` | height/offset/spacingやrequest配列順でmatchingする回帰を防ぐ |
| C781 | Behavior | Invariant | completed cross からさらにedgeを伸ばしても既存spanをretireしない | `legacy_unclassified` | 既存Bundle IDを継続したedge bundleをcontext edge側までretire scopeに含め、元のspanを削除する回帰を防ぐ |
| C782 | Behavior | Invariant | 接続済みsharp pairがあるjunctionへさらに鋭角edgeを追加しても既存pairを奪わず新規edgeをopenで残す | `legacy_unclassified` | 接続済みedgeを候補へ戻す、角度で再pairする、または追加操作を拒否する回帰を防ぐ |
| C785 | Behavior | Invariant | HV 3相のincremental pair promotionは既存Port identityを維持し、通常pairの二等分frameへPort位置とbinding yawを再導出する。lane対応はidentity / complete reverseに限定し、context側正本を変更せず、必須frameを再構築できない失敗時は正本を不変に保つ | `legacy_unclassified` | lane対応反転、contextの通常生成、失敗時の部分更新、bindingとPortのframe分裂、shared Port再導入を防ぐ |
| C786 | SourceGuard | Boundary | hash mix helper は production で1定義だけを持つ | `source_guard` | hash helper copy family が curve / geometry に再発することを防ぐ |
| C787 | SourceGuard | Boundary | BundleTemplate category と既定配置は Core/WASM payload を正本にし、JS は layer から推測しない | `source_guard` | web が SpanLayer→category 対応表や band matching を複製し、Core/WASM の分類とdriftする回帰を防ぐ |
| C788 | SourceGuard | Boundary | belt radial fit と endpoint socket placement は materialization が所有する | `source_guard` | curve_parts や renderer が socket/mesh geometry を再解釈する、またはbelt半径をmesh実体からCoreで読む回帰を防ぐ |
| C791 | Behavior | Boundary | 大きめの既存sceneへrouteを1本追加してもfixture pipelineはoperation単位でだけ走る | `legacy_unclassified` | scene規模に比例してplan構築・materialization・fallbackが再発する回帰をwall-clockに依存せず検出する |
| C792 | Behavior | Invariant | incrementalで新しい独立rowを追加しても既存Port anchorは動かさず、新rowはband anchorを保ったまま次のsupport level/groupを使い、そのresolved offsetをendpointへ適用する | `oracle` `anchor` | Port高さのstable-slot再決定、既存row reflow、support levelとendpoint loweringの分裂を防ぐ |
| C795 | Behavior | Invariant | model-aware HV pair promotionはPort/Span identityを維持し、正本placement anchorから同一pair frameでPort/row fixture/endpoint fixture/curveを導出してNodePatch laneをcollapseさせない | `legacy_unclassified` | modelありpromotionでfixtureとPortのframe分裂、3相collapse、shared Port再導入を防ぐ |
| C796 | Behavior | Invariant | explicit placement heightはincremental row reflowで変更されず、mixed bundleとsame-template duplicateを混同しない | `legacy_unclassified` | explicit heightをrow spacingやstable slotでband既定値へずらす、same-template duplicateが混ざる、pair promotionで既存row高さが変わる回帰を防ぐ |
| C797 | Behavior | Boundary | row continuity table はrouteとpair promotionの各lane継続を保存する | `legacy_unclassified` | NodePatchの存在をcurve層がbundle/port/位置epsから推測し続ける回帰を防ぐ |
| C798 | Behavior | Invariant | viewer既定T字branchはHVを生成し、branch-downはtemplate flagだけで決まる | `legacy_unclassified` | 画面上snap済みの端点をCoreの3D距離判定でmidair扱いしてHVを除外する、category名でloweringする、またはT追加で既存Port高さを動かす回帰を防ぐ |
| C799 | Behavior | Boundary | v1 load はrow continuityをmigrationしv2として保存する | `legacy_unclassified` | K4のv1互換読み込みとv2永続化の境界を固定する |
| C800 | Behavior | Boundary | row continuity graph lint はroute/branch/cross出力の参照整合を検査する | `legacy_unclassified` | 不正参照や重複endpointをcurve層のsilent dropで隠す回帰を防ぐ |
| C801 | Behavior | Boundary | v2 row continuityの壊れた参照はsilent dropせず拒否する | `legacy_unclassified` | テーブル参照切れを黙って読み飛ばして接続曲線を消す回帰を防ぐ |
| C802 | Behavior | Invariant | viewer既定T字branchは既存電柱がpath終点でもHVを生成し、branch-downはtemplate flagだけで決まる | `legacy_unclassified` | T branchの向き違いでHVだけ欠落する、category名でloweringする、またはT追加で既存Port高さを動かす回帰を防ぐ |
| C808 | Behavior | Invariant | T字branch loweringはHV categoryではなくBundleTemplate flagだけに従う | `legacy_unclassified` | HV専用分岐で高さを変える、またはflagを無視して非対象templateのPort自体をrow reflowで下げる回帰を防ぐ |
| C809 | Behavior | Invariant | 1 support levelは1 continuity pairまたは1未接続endpointだけを収容し、incremental追加では既存levelを維持して次の空きlevelへ進む | `legacy_unclassified` | 追加rowをすべて固定の2段目へ置く、接続時に段を変える、または通常stateの高さ・ID順から段を再推測する回帰を防ぐ |
| C810 | Behavior | Invariant | 通常pairはedge endpoint別Portを持ち、共有row/fixtureだけを導出する | `legacy_unclassified` | shared Port、別計算の位置ずれ、fixture二重生成、通常repositionでのPort再生成を防ぐ |
| C811 | Behavior | Invariant | 共有Portを含むv2 saveはedge endpoint別Portへ決定的に移行し、派生幾何を変えない | `differential` `anchor` | 実workspaceを読めなくする、ID順や高さで誤分割する、またはmigrationで線・碍子・腕金の位置を変える回帰を防ぐ |
| C812 | Behavior | Boundary | shared Port migrationはcontinuityで分割先を一意に特定できないv2 saveを部分適用しない | `legacy_unclassified` | 高さ、ID順、binding順による推測migrationや部分適用を防ぐ |
| C813 | Behavior | Invariant | MovePoleで通常/鋭角閾値を跨いでもendpoint identityを維持し表現だけを再導出する | `legacy_unclassified` | 保存pair/open表現やcached ruleにより角度変更後も古いrow/fixture/patchが残る回帰を防ぐ |
| C814 | Behavior | Invariant | 同一操作の2 edgeを既存openより優先して接続しない | `legacy_unclassified` | 操作内隣接を特別扱いして曖昧候補から勝手にpairを選ぶ回帰を防ぐ |
| C815 | Behavior | Invariant | 鋭角pairの段違い高さは一筆書きか段階追加かで変わらない | `legacy_unclassified` | 鋭角の2 derived rowを同じ高さへ潰す、片側だけをloweringする、または操作履歴でpair高さが変わる回帰を防ぐ |
| C816 | Behavior | Invariant | 通常pairはpair化後も割当済みsupport levelとbundle height更新を2本で維持する | `legacy_unclassified` | support level集合だけ合っていて、pair化したedgeがlevel 0へ戻る、後続pairと段を取り違える、またはheight更新がpair片側だけにしか効かない回帰を防ぐ |
| C803 | Behavior | Invariant | mount graphはinstance socket親参照を深さ4まで合成し、欠落socketと循環を明示エラーにする | `legacy_unclassified` | row/endpointの2段専用伝搬に戻り、3段以上のモデル配置が動かない回帰を防ぐ |
| C804 | SourceGuard | Boundary | 旧row/endpoint fixture fieldは入力adapterだけでPlacementRule化し、MountRef 3種とinterval anchor純関数を文書化する | `source_guard` | 新しい配置種ごとにBundleTemplate専用fieldやmodel_assembly分岐を増やす、またはビットフラグ組合せで意味を曖昧にする回帰を防ぐ |
| C805 | Behavior | Invariant | route/orderはactive route link内だけの継続補助で、保存済みcontext edgeはsaved pair/row continuityを正本にする | `legacy_unclassified` | 世代違いの保存済みcontext edgeが同じroute/orderを持つだけで継続候補になり、restore時にroute continuity ambiguityを発生させる回帰を防ぐ |
| C806 | SourceGuard | Boundary | v2保存はbackbone edge/edge_bundleのroute/order導出補助を永続化しない | `source_guard` | 生成request内だけの補助値をv2正本として保存し、世代跨ぎrestoreやregenerateがroute/order推測へ戻る回帰を防ぐ |
| C807 | SourceGuard | Boundary | pipeline/regenerate/updateはpair/open/row continuityをroute/orderから推測しない | `source_guard` | K完了後にH2の暫定same_route限定やregenerate/updateのroute/order判定が戻り、継続性テーブルと並走する回帰を防ぐ |
| C793 | SourceGuard | Boundary | derive/updateはcached ruleをcanonical span順でplan化し、affected spanはspan_id lookupで解決する | `source_guard` | unordered_map列挙順やaffected spanごとの線形rules探索、support group代表の偶然順序依存が戻る回帰を防ぐ |
| C794 | SourceGuard | Boundary | ViewerActionContextのtimer/cancel/interaction/persistence/factory状態はprivateで、action moduleはmethod経由で操作する | `source_guard` | 役割分割後に巨大共有contextへpublic mutable stateが戻り、action module間でtimerやcancel状態を自由変更する回帰を防ぐ |
| C768 | Behavior | Boundary | 新field導入前のauthoritative stateは暗黙supportと既定Bundle placementを維持し現行形式へ移行できる | `legacy_unclassified` | IndexedDBに残る旧workspaceのrestore不能化、旧helixから必須supportが消える回帰を防ぐ |
| C838 | Behavior | Invariant | span binding だけから edge_bundle_spans / span_edge_bundle / binding index を再構築する | `legacy_unclassified` | SavedBackboneEdgeBundle::span_ids を再び所属正本として復活させる回帰を防ぐ |
| C839 | Behavior | Invariant | span binding の lane→span 対応は save/load 後も一致し、span_ids は保存しない | `legacy_unclassified` | lane順をvector挿入順やedge bundle内span一覧から復元する回帰を防ぐ |
| C840 | Behavior | Boundary | 同一 edge bundle 内の duplicate lane binding は load 時に拒否する | `legacy_unclassified` | lane identity が曖昧な保存状態をsilent acceptする回帰を防ぐ |
| C841 | Behavior | Boundary | 同一 span の複数 backbone span binding は load 時に拒否する | `legacy_unclassified` | span が複数edge bundle/laneへ所属する二重authority状態を受け入れる回帰を防ぐ |
| C842 | Behavior | Boundary | 旧saveに残る edge_bundle span_ids は読み捨て、再保存では出力しない | `legacy_unclassified` | 互換読み込みのために削除済みspan_idsを新しいauthoritative stateへ戻す回帰を防ぐ |
| C843 | Behavior | Boundary | continuity構築対象のlaneでPortBinding / SpanBinding / Span endpoint relationが不足・不整合なら、既存continuityを変更せずoperationを失敗させる | `legacy_unclassified` | continuity不要bundleの対象外判定と、必須relation欠落のsilent skipを混同する回帰を防ぐ |
| C844 | Behavior | Boundary | promotion frame更新はexact PortBindingを必須identityとし、missing / ambiguous / Port relation不成立を正本不変のまま拒否する | `legacy_unclassified` | missing exact bindingをno-op successとしてpromotionを継続する回帰を防ぐ |
| C845 | Behavior | Invariant | permutable 3-lane rowのidentity / complete reverse双方で、PortBinding laneが参照するPortとplacement_band_idは同じphysical placementを表す | `legacy_unclassified` | mirror時にlogical span laneのbandをphysical Portへ保存してfixture・curve placementを分裂させる回帰を防ぐ |
| C846 | SourceGuard | Boundary | save_graphのPortBinding保存はplacement_band_idをlogical span laneではなく、選択したPortと同じphysical trow laneから読む | `source_guard` | behavior fixtureが未到達のmirror形状でspan.lane参照が再導入される回帰を防ぐ |
| C847 | Behavior | Invariant | 同一physical edge上のHV/LVが別peer edgeへ接続する独立edge-bundle continuity componentはauthoritative validation、save/load、derived rebuildを維持する | `differential` `presence` | edge-bundle identityをload前にphysical edgeへcollapseし、合法な保存状態をambiguousとして読めなくする回帰を防ぐ |
| C848 | Behavior | Invariant | scoped regenerateは対象edge-bundle componentを先に選び、無関係componentをauthority・出力の判断入力にしない | `differential` `anchor` | 同じphysical edge上の別bundle continuityがHV/LV個別regenerateを失敗または変質させる回帰を防ぐ |
| C849 | Behavior | Boundary | 同一edge-bundle component内で3方向へ分岐するsaved continuityはload時に明示失敗し、既存正本を変更しない | `anchor` | component identity修正をambiguityのsilent skipやfallbackへ変える回帰を防ぐ |
| C850 | Behavior | Invariant | 同じfixed BundleTemplateを使う独立Bundle placementは、各Bundle identityとedge-bundle continuity componentをscopeに3→4更新し、単独実行・配置順反転と同じ意味結果を得る | `differential` `anchor` | template全体を一括regenerateして独立componentを混同する、または無関係componentを判断入力にする回帰を防ぐ |
| C851 | Behavior | Invariant | 同じfixed BundleTemplateを使う独立Bundle placementの4→3更新は、各componentのlane 3だけをretireし、残存PortBinding・SpanBinding・continuityをcomponent内に保つ | `presence` `anchor` | component横断でSpan/Portを削除する、または残存relationを別Bundleへ結ぶ回帰を防ぐ |
| C852 | Behavior | Invariant | row角度差が1e-6 degreeより大きく1e-6 radianより小さい場合も、support levelは実角度順で決まりpath入力方向に依存しない | `metamorphic` `oracle` | atan2のradian値をdegree toleranceと比較し、ID順tieへ約57.3倍広げる回帰を防ぐ |
| C853 | Behavior | Invariant | Portはband/explicit placementの論理anchorを保持し、support level/groupから一度resolveしたbranch downだけが物理endpointへ適用され、save/loadとscoped placement regenerateでも同じ意味結果を保つ | `differential` `oracle` `anchor` | row offset、stable slot、support level loweringが独立にPort/endpoint Zを決めて相互補償する回帰を防ぐ |
| C854 | SourceGuard | Boundary | productionのrow height経路にrow-height offset、stable slot、bundle update時の保存offsetを残さず、row ordering角度はatan2直後にdegreeへ正規化する | `source_guard` | 別経路のno-op faultを他経路が補償する構造とradian/degree混在が再導入される回帰を防ぐ |
| C855 | Behavior | Invariant | 非Autoのmain spanも単一のsag_factorをspan全体へ1回適用し、`chord length * ratio`の物理sag量と独立式`4u(1-u)`のparabolic profileを使う | `oracle` `presence` | continuity policyによるprofile分岐、sag ratio二重加算、長さ・pass・剛性による隠れ倍率を防ぐ |
| C836 | Behavior | Invariant | 操作×状態表の各確定セルを実際の正本状態から分類し、各観測点でrow frame coherenceを検査して実行する | `oracle` `presence` `anchor` | case名と手書き表だけで未構築stateをcoverage済みにする、または接続状態だけ正しく派生frameが分裂する回帰を防ぐ |

## 廃止済み旧pipeline検査

- 旧case 365〜367は登録済みsuiteから削除した。これらはbackbone本流ではなく、移行中の`BackboneBuilder`、support-layout authority seed、materialization surfaceを固定していた。
- これらの制約は現在、saved graph、pair/open/row authority、再計算・再materialization禁止、layout/geometry/drawのconsumer chainを検査するbackbone側の証明で保護している。

## LLM自己レビュー

- 実装依存か: private順序/内部関数呼び出し順には依存しない。
- 期待値は観測可能か: すべて公開APIと公開状態で観測。
- モック過多か: モック未使用。
- 異常系が入っているか: C10/C20/C21/C22/C23/C49で失敗診断・状態保全・復帰を検証。
- フレーク要因がないか: 実時間待ち/非決定乱数なし。
