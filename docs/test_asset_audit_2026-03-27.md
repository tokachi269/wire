# テスト資産の棚卸しと再編方針

## 前提と観測

- 基準 commit は `feb5819e8ec29020453e4f44cd2442e1d522507e`。
- `HEAD`、追跡先 ref、`git ls-remote origin codex/relation-index-corestate` は同一だった。`git fetch` は `.git/FETCH_HEAD` の permission で失敗したため、remote 側 commit は `ls-remote` で確認した。
- 登録済みテストは 277 件。集中箇所は `generation.cpp` 119 件、`geometry.cpp` 65 件。
- 登録外だが test asset として効いている compile-only 資産が `core/tests/public_headers_smoke.cpp` に 1 件ある。
- `core/tests/spec_ledger.md` は観測点をかなり制限しているが、`last_generation_backbone` 系だけは「一時 snapshot 観測」と明記されている。ここに依存するケースは、仕様テストとしては残しにくい。
- `core/tests/state_services.cpp` は `../src/state/internal_services.hpp` と `core_test_hook.hpp` を直接使っており、現在の責務分割に強く依存している。

## 現在のパイプライン境界案

### 1. relation 判定

- 主責務:
  - route / node / incident から junction relation を決める。
  - `ThroughPair` の accept/reject、`SideBranch` / `CornerContinuation` / `CrossUnderpass`、`same_level_feasible`、`default_lower_required` を確定する。
- 観測した実装:
  - `core/src/generation/from_backbone.cpp`
  - `core/src/generation/grouped_span_lowering.cpp`
- この段で決めること:
  - relation kind
  - continuity class
  - through-pair membership
  - same-level feasibility と理由
  - default lower の必要性
- 次段で再判定してはいけないこと:
  - port 配置や local geometry を見て relation を作り直すこと
  - refresh / recalc / inspection で `ThroughPair` を再推論すること

### 2. decision 決定

- 主責務:
  - relation と policy を入力に、authoritative な endpoint / segment decision を作る。
  - order decision、flow kind、lowering kind、support-group identity、side axis、orientation basis を確定する。
- 観測した実装:
  - `core/src/generation/grouped_spans.cpp`
  - `core/src/generation/grouped_span_lowering.cpp`
  - `core/src/generation/grouped_span_orientation.cpp`
- この段で決めること:
  - `EndpointContinuityDecision`
  - `SupportGroupDecision`
  - `SegmentLaneAssignment`
  - `BackboneEdgeOrientation`
- 次段で再判定してはいけないこと:
  - port source から lower / branch / cross を逆算すること
  - visual / refresh 側で chosen order / side / basis を上書きすること

### 3. lane/port 準備

- 主責務:
  - authoritative decision を消費して必要な port を準備する。
  - constrained solver や row scaffold を使って実体 port を揃える。
- 観測した実装:
  - `core/src/generation/grouped_span_lane_preparation.cpp`
  - `core/src/state/endpoint_refresh_service.cpp`
- この段で決めること:
  - port の実体化方法
  - endpoint ごとの port 再投影
  - 同一 decision を満たすための solver 使用有無
- 次段で再判定してはいけないこと:
  - relation / lower / support-group identity を port の置かれ方から再構成すること
  - refresh で authoritative decision を pole-local 判定に戻すこと

### 4. 実体化 / recalc

- 主責務:
  - decision を support layout / lowered support placement / detail curve / visual cache に写像する。
  - validator と inspection が authority を読む面を作る。
- 観測した実装:
  - `core/src/recalc/support_layout_materialization.cpp`
  - `core/src/recalc/recalc_pipeline.cpp`
  - `core/src/validation/validator.cpp`
  - `core/src/state/inspection.cpp`
- この段で決めること:
  - support layout endpoint / group placement の geometry
  - detail curve profile hint の適用
  - visual cache の grouped support 表現
  - validator / inspection surface
- 次段で再判定してはいけないこと:
  - endpoint semantic copy から support-group authority を組み直すこと
  - visual 側で grouped support を endpoint 単位へ割り戻すこと
  - inspection が stale port/source を正本扱いすること

## 棚卸し結果

表の見方:

- 分類:
  - `A`: 残すべきテスト
  - `B`: 捨てる候補
  - `C`: 書き直し候補
- 守っているもの:
  - `仕様` / `契約` / `実装詳細` / `不明`

| テスト名 | 分類 | 守っているもの | 理由 | 推奨対応 |
|---|---|---|---|---|
| `C171 public header smoke surface` | A | 契約 | 公開 header 群が self-contained で、公開 view/inspection 型が最低限使えることを見ている。実装再編後も意味が残る。 | 残す |
| `public_headers_smoke.cpp` | B | 実装詳細 | `C171` とほぼ同じ compile smoke を登録外 TU で重複している。守る意味が分散している。 | 削除 |
| `C01 ID生成の単調増加`; `C02 ObjectStore整合` | A | 契約 | 純粋で独立した基盤契約。パイプライン再編の影響を受けにくい。 | 残す |
| `C03 Span追加初期Dirty`; `C04 MovePole局所Dirty`; `C05 SplitSpan整合`; `C09 Pole接続整合`; `C10 同一Pole接続拒否`; `C11 柱起点引込`; `C12 線起点引込` | A | 契約 | 編集 API の整合、dirty、失敗後復帰を見ている。ユーザ操作と再計算境界を直接守る。 | 残す |
| `C06 PoleType適用`; `C07 PoleType差分`; `C08 未使用候補優先` | A | 契約 | pole type 適用と port 選定の契約を見ている。helper 呼び出し順ではなく観測可能結果を見ている。 | 残す |
| `C18 デモ密度` | B | 不明 | 初期 viewer 都合の密度閾値で、仕様よりデモ状態に依存している。壊れても責務境界の異常とは直結しにくい。 | 削除 |
| `C20 短polyline拒否`; `C21 interval不正拒否`; `C22 存在しないPort拒否`; `C23 Split t不正拒否` | A | 仕様 | 異常入力で fail し、状態が壊れないことを見ている。再編後も必要な API 仕様。 | 残す |
| `C24 隣接Pole自動接続`; `C27 SimpleLine統合`; `C28 through連続性`; `C29 表示ID採番`; `C42 再計算の非破壊性` | A | 仕様/契約 | 生成結果、through 継続、表示 ID、derived cache 再計算の非破壊性を見ている。最終意味と重要契約に寄っている。 | 残す |
| `C25 複数パス増加`; `C26 第3候補利用` | C | 契約 | 意図は「未使用候補優先」だが、現在は pass 回数や候補 ordinal に寄りすぎている。allocator 再編で壊れやすい。 | 書き直し |
| `C41 debug記録クリアの無害性` | B | 実装詳細 | debug record の存在前提が強く、再発防止より debug 管理都合に近い。 | 削除 |
| `C44 Bundle参照API整合`; `C45 不正Bundle参照拒否`; `C78 Pole tiltでAuto Port再投影と見た目追随`; `C79 参照長によるサグ安定`; `C80 center hintのPole非重なり`; `C81 insulatorは電力系のみ表示`; `C119 CableTemplate太さ変更の見た目反映`; `C120 CableTemplate碍子要否の見た目切替`; `C121 Template責務の分離`; `C122 CableTemplate編集で Pole tilt を上書きしない`; `C125 ApplyPoleTilt は選択された Pole 実体だけを更新する`; `C202 Pole tilt は incident span 方向へ寄る`; `C203 Pole tilt 量は pull imbalance に従う` | A | 仕様/契約 | visual/tilt/template の結果が最終見え方へ直結している。内部 helper ではなく状態遷移と見え方を見ている。 | 残す |
| `C13 直線幾何決定性`; `C14 サグ基本`; `C15 Version追随局所性`; `C16 Bounds有効性`; `C17 Bounds追随` | A | 契約 | curve/bounds/runtime version の基本不変条件。再発したときの実害が明確。 | 残す |
| `C19 道路Pole生成`; `C30 Pole文脈分類`; `C31 角補正有界`; `C34 Corner文脈統合`; `C35 内外補正差`; `C36 DrawPath点直配置`; `C37 幾何based side選定`; `C43 鋭角時Port展開軸補正`; `C56 鋭角閾値境界`; `C57 Guide重複点ロバスト`; `C58 Reverse対称性`; `C59 Avoid制約尊重`; `C60 Guide再利用頂点の向き再評価`; `C61 鋭角自動拡幅`; `C70 Guide再利用頂点Port再投影`; `C71 MovePoleでAuto Port再投影とManual保護`; `C108 鋭角向きの入口間一致` | A | 仕様/契約 | geometry と pole/port 向きの規則を直接見ている。ユーザが見て不自然になる再発を止める効きが強い。 | 残す |
| `C32 文脈別選定` | A | 契約 | trunk/branch で port 選定傾向が変わるという契約を見ている。private helper には依存していない。 | 残す |
| `C33 決定的タイブレーク` | C | 契約 | 決定論自体は重要だが、debug record の整合まで一つのテストで固定している。trace surface への依存が強い。 | 書き直し |
| `C126 WorldUp と lateral 軸の整合`; `C127 PoleFrame の local/world roundtrip` | A | 契約 | 純粋関数の座標契約。責務が独立しており補助テストとして残しやすい。 | 残す |
| `C128 uベース曲線APIの端点拘束`; `C129 sベース配置APIと sag 合成`; `C130 OffsetEndpoint 端点`; `C131 鋭角/競合接線での品質劣化安全策`; `C141 懸垂寄り support slope`; `C142 近直線での横揺れ抑止強化`; `C143 SmoothPass の横揺れ抑止`; `C144 長尺継続 span の G2優先採用`; `C145 短スパンの G1劣化`; `C146 branch pass の G1優先と端点拘束保持`; `C147 PreferG1 は明示選択`; `C148 SharpCorner の compact 化`; `C149 ViaAttachment の endpoint 優先`; `C161 長い branch span は横回り込みを抑える`; `C162 branch の局所 departure は短スパンほど強い`; `C163 main の sag は branch より強く読める`; `C277 高低差の大きい smooth span は composite curve へ切り替える` | A | 契約 | detail curve の純粋な品質契約。内部関数分割の変更に耐えやすい。 | 残す |
| `C132 GPU用距離属性の焼き込み`; `C137 Attachment表示offset非干渉`; `C156 PassThrough attachment は外部線を残す`; `C157 HideSegment attachment は外部線区間を隠す`; `C158 ReplaceWithInternalPath attachment は内部線路へ置換する`; `C159 socket endpoint 接続で隙間を詰める`; `C160 runtime は attachment 名に依存しない`; `C164 support layout は branch support 拘束を集約する`; `C165 support layout は attachment socket endpoint を捕捉する`; `C168 Pole orientation override の往復`; `C169 Span socket override の往復`; `C170 Branch down offset override の往復`; `C172 support layout と detail curve は同じ resolved policy を使う`; `C254 support layout は stale な branch-support port より authoritative assignment を優先する`; `C278 support layout 由来の高低差 hint は recalc で composite curve に伝播する` | A | 契約 | attachment/support-layout/override の境界契約を見ている。中間実装より意味の伝播を見ている。 | 残す |
| `C166 inspector surface は span/support layout/detail curve を概念単位で見せる`; `C167 inspector surface は pole/template/override/junction を共通面で参照できる`; `C257 inspection support layout は assignment fallback をしない`; `C258 inspection backbone は rebuilt 結果を正とする`; `C259 junction relation 面は relation snapshot を正とする` | C | 契約 | inspection 面の意図は重要だが、現在は surface の作り方と表示都合が混ざっている。境界を `inspection reads authority, not fallback` に絞って作り直すべき。 | 書き直し |
| `C82 T-junctionの一次session優先`; `C83 Cross junction順序の安定`; `C84 後続pathでjunction優先順を上書きしない` | C | 契約 | junction order の意図は relation 判定段の契約だが、今は session 順序と rebuilt ordering に強く依存している。 | 書き直し |
| `C104 Segment端点吸着`; `C105 Segment中間でMidair生成`; `C106 Pick経由HV空中分岐禁止`; `C107 Midair Dry-run非破壊`; `C117 Path input では midair policy を入力段階で強制しない` | C | 契約 | pick/input resolution の意図は重要だが、`last_generation_backbone` 系 snapshot 依存が強い。入力解決 API 契約へ落とし直すべき。 | 書き直し |
| `C40 Pole flip_180`; `C50 Port初期モード`; `C51 Port手修正/解除`; `C52 Manual保護`; `C53 BackboneSpec境界手動点安定`; `C54 BackboneSpec局所更新`; `C55 Backbone経路`; `C64 Guide頂点の強制Manual解除`; `C65 pin_verticesオプション`; `C66 Pole Pin/Unpin切替`; `C67 セッション局所再生成でManual Pole保持`; `C68 セッション局所再生成でManual Port保持`; `C69 他セッション非干渉`; `C71 MovePoleでAuto Port再投影とManual保護`; `C72 セッション局所再生成でAuto Pole配下Manual Port保持`; `C206 セッション再生成でacute低下とgenerated port cleanupを維持`; `C207 セッション再生成の経路延長でもHV lane orderを維持` | A | 仕様/契約 | 手修正保護、局所再生成、session 隔離を見ている。ユーザ価値に直結し、再編後も必須。 | 残す |
| `C73 固定テンプレcount上書き拒否`; `C74 非HVテンプレ固定1本`; `C75 非HV固定テンプレcount上書き拒否`; `C77 複数テンプレ同時生成`; `C90 Backbone bundles必須`; `C91 接続時テンプレ必須`; `C92 接続時テンプレ優先`; `C93 接続時bundle/template競合拒否`; `C94 接続時template指定+auto_create無効`; `C95 接続時span_layer上書き競合拒否`; `C96 Dropはテンプレ既定利用`; `C101 HV空中分岐規格フラグ固定`; `C123 BundleTemplate の topology 変更は regeneration_required を立てる`; `C124 BundleTemplate の visual-only 変更は dirty のみに留める`; `C191 preserved multi-lane template は HV 名に依存せず offset endpoint を使う` | A | 仕様/契約 | template policy と invalid input 拒否の契約。実装順序ではなく API/結果を守っている。 | 残す |
| `C102 Bundle別接続モード共存` | C | 契約 | 意図は重要だが、現在は generation snapshot の node bundle mode 観測に寄っている。bundle-mode authority の契約面を先に定義したい。 | 書き直し |
| `C181 endpoint refresh service は relation index から owned endpoint を集める`; `C182 endpoint refresh service は対象 pole の owned endpoint だけ更新する`; `C183 override resolution service は formal override を優先解決する`; `C184 ApplyPoleType は relation-index-owned endpoint を再利用する`; `C186 template mutation service は編集対象 template にぶら下がる bundle だけ再生成要求する`; `C187 attachment template 更新は実際にその template を使う span だけ dirty にする`; `C200 bundle の branch down offset policy 変更は topology change` | C | 実装詳細 | 意図自体は必要だが、`internal_services.hpp` と `core_test_hook` に強く依存している。public mutation boundary か validator 契約へ寄せ直すべき。 | 書き直し |
| `C260 non-radial support basis は authoritative side axis を必須にする`; `C261 grouped lowered support は radial basis を許さない`; `C262 grouped support identity は一意の placement と pair 決定に対応する`; `C281 validation は grouped endpoint の semantic copy より support-group authority を主語にする` | A | 契約 | validator が不変条件を固定している。今回の再編方針と直接一致する。 | 残す |
| `C263 inspection は authoritative support-group decision と grouped placement を読む`; `C280 grouped support materialization は endpoint copy ではなく layout-owned authority を消費する` | C | 契約 | 守りたい意味は中核だが、現状は cache mutation と inspection surface の組み合わせに寄っている。materialization boundary 契約として再構成したい。 | 書き直し |
| `C38 高圧3相群生成`; `C39 方向強制モード`; `C47 DrawPath Bundle生成(HV標準)`; `C48 DrawPath Bundle生成方向モード`; `C49 DrawPath Bundle生成の異常系`; `C89 3相ポリシーのカテゴリ非依存性` | A | 仕様/契約 | backbone/grouped generation の基本仕様。最終結果や異常系を見ている。 | 残す |
| `C62 群レーンねじれ抑制`; `C63 mirror導入の非悪化`; `C76 鋭角コーナーlane順反転抑制`; `C86 acute pattern suiteの反転なし`; `C87 HV3 acute pathの相ねじれ防止`; `C98 Backbone延長時の境界導体順保持`; `C99 HV3キャプチャ形状の反転回帰`; `C109 HV3 captureの内部共有pole順序連続`; `C208 interval 挿入ありの経路延長でも境界 lane order を維持` | A | 仕様/契約 | 実際に再発しやすい lane order / inversion 系を直接検知する。強い仕様テストとして残す価値が高い。 | 残す |
| `C88 Backbone HV3 acute pathのtemplate経路trace可視化`; `C178 DrawPath 通常経路は attachment 未入力を trace できる` | B | 実装詳細 | trace の読みやすさや可視化面を主に固定している。再発防止の主軸にしない方がよい。 | 削除 |
| `C100 Midair SupportNode保持`; `C103 非Poleノード経由の詳細生成安定性`; `C110 明示Pole node再利用`; `C111 明示SupportNode再利用`; `C112 Midair始点延長の詳細生成`; `C113 Midair始点延長の先頭 support-detail 区間保持`; `C114 Midair branch の source span 高さ再利用`; `C115 Midair 1クリック延長で余計な bridge を増やさない`; `C116 midair branch 禁止 template は生成だけを skip する`; `C118 mixed template の midair branch は許可 bundle だけ生成` | C | 契約 | Midair 入力まわりの意図は重要だが、一時 backbone snapshot と source-edge 構造へ密結合している。input resolution / generated result 契約に分離したい。 | 書き直し |
| `C133 本線優先Pole向き`; `C136 HV3 main port 安定性`; `C138 Mixed route の edge 単位 flow`; `C139 Branch support の派生配置`; `C140 Near-straight branch でも topology 優先`; `C150 新規chainは orientation fallback`; `C173 直線 DrawPath の support axis は路線直交へ保つ`; `C174 十字 junction の support axis は対角線にならず main 軸直交を保つ`; `C175 直交 DrawPath corner は bisector を使わない`; `C176 branch 追加で main support axis の直交関係を崩さない`; `C177 DrawPath 通常経路は plain support endpoint へ落ちる`; `C179 DrawPath branch curve は support 離脱を局所化する`; `C185 HV3 DrawPath terminal は route 直交に開く`; `C188 HV hint が無い terminal でも generated row は route 直交に開く`; `C189 communication pole + 全 template 選択でも HV3 terminal row は route 直交を保つ`; `C190 communication 多条 terminal row は route 直交を保つ`; `C192 clicked existing communication poles + 全 template 選択でも HV terminal 分離を維持する`; `C237 point-like branch の orientation は非回帰`; `C279 隣接 branch root の lowered support は route-local peer を基準に bisector を選ぶ` | A | 仕様/契約 | support axis / terminal row / route-local orientation の最終意味を見ている。見た目の自然さに直結する。 | 残す |
| `C134 bundle-like branch root は grouped lowered support を持つ`; `C135 Branch down offset は layer 非改変`; `C193 HV3 bundle branch は bundle 全体で 1 段 lower を共有する`; `C195 bundle-like branch lower は分岐根だけに留める`; `C196 Branch support visual は branch に直交する`; `C197 DefaultSingle branch は自動高さ変更しない`; `C198 Communication multi-bundle branch は policy blocked lowered decision を観測する`; `C199 HV3 bundle branch は両方の標準 pole type で同じ 1 段 lower を使う`; `C204 HV3 corner continuation は middle pole で 1 段 lower を使う`; `C205 corner continuation の 1 段 lower identity は pole refresh 後も維持される`; `C209 moderate corner でも既定閾値で 1 段 lower が入る`; `C210 全 template 選択の branch でも communication pole 上の HV は 1 段 lower を維持する`; `C211 全 template 選択の cross でも communication pole 上の underpass を維持する`; `C212 共有 trunk 付き capture 相当の 4点 path で HV の branch root と corner endpoint は同じ 1 段 lower を共有する`; `C217 through pair accepted でも same-level 不可なら cross を下げる`; `C218 shape 上 lower 必要でも category policy が無効なら blocking を観測できる`; `C219 semantic main と local corner を分けたまま feasibility lowering できる`; `C220 acute merge は run 境界でも feasibility 根拠を保つ`; `C221 recalc 後も same-level/lowering 起源が support layout に残る`; `C222 same-level infeasible な corner は通常 solver の constrained band へ寄る`; `C223 through accepted でも cross infeasible は solver 制約で下げる`; `C224 policy blocked conflict は recalc 後も unresolved として残る`; `C225 refresh 後も placement constraint origin を保持する`; `C226 mirror は constrained solver 使用判断も変えない`; `C227 cross relation は support layout recalc でも潰れ切らない`; `C228 bundle-like HV3 branch は ThroughPair 外を既定 lower にする`; `C229 bundle-like HV3 corner continuation は既定 lower にする`; `C230 bundle-like HV3 cross は ThroughPair だけ same-level 候補に残す`; `C231 point-like branch は feasibility ベースの same-level を維持する`; `C232 bundle-rule + policy blocked は unresolved のまま残る`; `C233 bundle-rule origin は refresh 後も失われにくい`; `C234 cross lowered pair は junction pair 基準で 1 つの side group を使う`; `C235 constrained lowered support は non-radial authoritative basis を使う`; `C236 bundle-like lowered branch root は bisector 優先で向きを決める`; `C238 refresh 後も lowered side/orientation origin は失われにくい`; `C244 support layout は authoritative endpoint decision をそのまま使う`; `C245 refresh は authoritative endpoint decision を上書きしない`; `C246 cross lowered pair は authoritative shared side choice を維持する`; `C247 constrained placement は authoritative orientation basis を使う`; `C248 HV3 authoritative order decision は refresh 後も残る`; `C249 edge orientation は authoritative order decision から導出される`; `C250 bundle-like branch lower は pole 局所条件で決まる`; `C251 cross underpass support は左右へ分裂せず 1 つの side group を共有する`; `C252 refresh は pole-local lower と shared support identity を維持する`; `C253 lowered corner support は接続した lowered 線群だけで non-radial basis を決める`; `C255 grouped lowered support は bundle の各 lane から見える`; `C256 無関係な後続生成で既存 lowered bundle の意味が劣化しない`; `C267 non-lowered cross spans は lowered support group を露出しない`; `C268 acute-corner を含む bundle の flat span は lowered support group を継承しない`; `C271 grouped lowered support の visual cache は single placement を使う`; `C272 bundle-like non-through の lower_required は grouped placement の高さへ必ず反映され refresh 後も不変`; `C273 bundle-like non-through の高さは ThroughMain 対比で 1 段 lower に縮退する`; `C275 lowered bundle の downstream corner でも support pair と向きは support group 正本を保ち refresh 後も不変`; `C276 pair-based lowered support は shared axis/basis を保ちつつ opposite poles で sign を反転できる` | A | 契約 | relation -> decision -> materialization の主問題を直接守っている。今回の再編の土台にすべきテスト群。 | 残す |
| `C194 preserved continuity が無い junction は straightest pair を優先する`; `C201 point-like 十字交差は same-level continuity を残せる`; `C213 直角 junction は ThroughPair を受理しない`; `C214 local corner は semantic main に射影されても local through ではない`; `C215 別 route 合流後の corner も relation 根拠で acute lowering を持てる`; `C216 mirror は relation / lowering 根拠を変えない`; `C239 HV3 same-level は non-fixed order decision を保持する`; `C240 HV3 lowered 側でも non-fixed order decision を維持できる`; `C241 identity-preserving bundle は non-fixed order evaluation を使わない`; `C242 refresh 後も chosen order decision を保持`; `C243 point-like route は order permutation 非対象`; `C269 cross-like 既存ノードからの single-edge route は main として扱う`; `C270 explicit middle anchor の bent route は既存 straight chain に負けず main-like を保つ`; `C274 cross-like reuse では既存 straight continuity を新規 crossing より優先する` | A | 契約 | relation 判定と order decision の契約を見ている。下流実装を変えても意味が残る。 | 残す |
| `C264 inspection は all-template branch の HV 1 段 lower を保つ`; `C265 inspection は capture 相当 path の branch root と corner を同じ 1 段 lower で読む`; `C266 inspection span は lane snapshot の flow と turn をそのまま読む`; `C180 DrawPath の main/branch 差は inspection と trace で読める` | C | 契約 | 重要な意味だが、inspection / last-lane snapshot surface の表現まで一体で固定している。relation/decision 契約と inspection contract を分離して作り直したい。 | 書き直し |

## 新しいテスト体系案

### 1. 仕様テスト

少数で強いケースだけを残す。候補は次。

- `C10` 同一Pole接続拒否
- `C20`, `C21`, `C22`, `C23` 失敗時の状態保全
- `C24`, `C27`, `C28` 生成結果と through 継続
- `C61`, `C62`, `C76`, `C99` ねじれ・鋭角・lane order 非回帰
- `C67`, `C68`, `C71`, `C72`, `C206`, `C207` manual/regen 保護
- `C101` HV midair branch policy
- `C134`, `C193`, `C204`, `C217`, `C228`, `C251`, `C272`, `C273` lower / grouped support の本命ケース
- `C269`, `C270`, `C274` main-like / cross-like relation の本命ケース

### 2. 契約テスト

パイプライン境界ごとに分ける。

- relation 判定:
  - `C194`, `C201`, `C213`, `C214`, `C215`, `C216`, `C219`, `C220`
  - ThroughPair accept/reject、semantic main と local continuity の分離、same-level feasibility
- decision 決定:
  - `C221` から `C249`
  - authoritative endpoint decision、order decision、orientation basis、policy blocked unresolved
- lane/port 準備:
  - `C32`, `C61`, `C71`, `C177`, `C179`, `C225`, `C236`
  - port 選定、constrained solver、refresh が authority を消費すること
- materialization / recalc:
  - `C244` から `C256`, `C267`, `C268`, `C271`, `C272`, `C273`, `C275`, `C276`, `C278`
  - support layout, grouped placement, visual cache, detail curve への authority 伝播
- validator:
  - `C260`, `C261`, `C262`, `C281`
  - authority と copy を混同しない structural invariant

### 3. 補助テスト

責務が独立しているものだけ残す。

- `C01`, `C02`
- `C126`, `C127`
- `C128` から `C149`, `C161`, `C162`, `C163`, `C277`
- `C151` から `C155`
- `C171`

## 今すぐ削除してよいテスト群

- `public_headers_smoke.cpp`
  - `C171` と重複。登録外 TU に compile smoke を分散させる理由がない。
- `C18 デモ密度`
  - デモ状態の見え方都合で、仕様や契約を守る軸が弱い。
- `C41 debug記録クリアの無害性`
  - debug 管理の都合に寄っており、今回の再発防止の主題から外れる。
- `C88 Backbone HV3 acute pathのtemplate経路trace可視化`
  - trace 可視化の読みやすさを固定しており、仕様保護より surface 依存が強い。
- `C178 DrawPath 通常経路は attachment 未入力を trace できる`
  - trace 文脈の可視性であって、パイプライン境界の契約ではない。

## 再編の優先順

1. `B` を先に外す。重複と低価値ノイズを減らす。
2. `C` のうち `last_generation_backbone` 依存と `internal_services.hpp` 直呼びを最優先で置き換える。
3. `A` の relation/decision/materialization 系を新しい土台へ寄せて固定する。
4. その後に inspection 向け契約を必要最小限だけ戻す。

## 未完

- ケース数が多いため、今回の棚卸しは「ケース単位の分類」は行ったが、「各テストを新ファイルへどう分割するか」までは落としていない。
- `C` の書き直し先 API 名は、relation/decision/materialization の境界をコード側で明示した後に決めるべき。

## 次に見るべき点

- `core/src/generation/from_backbone.cpp`
  - relation 判定の authority surface を固定できるか。
- `core/src/generation/grouped_spans.cpp`
  - `EndpointContinuityDecision` / `SupportGroupDecision` を下流が必ず通る形に寄せられるか。
- `core/src/recalc/support_layout_materialization.cpp`
  - endpoint semantic copy ではなく layout-owned authority を唯一の正本にできるか。
- `core/src/state/inspection.cpp`
  - inspection を「意味の観測面」に限定し、trace/readability テストを減らせるか。
