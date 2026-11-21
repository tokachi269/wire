# 現在の保留と検証状況

merge前のゲートは[merge_readiness.md](merge_readiness.md)にある。この文書は作業が進むたびに更新する状況ボードであり、設計上の意味の正本ではない。各行の正本は参照先の設計文書とする。

## PR #9の外部解析待ち

持ち込みレビューレポートでは、[PR #9](https://github.com/tokachi269/wire/pull/9)の`d1a99e2e`に対するSonarCloud Quality Gateは、Security / ReliabilityがともにCで失敗したと報告されている。この環境では外部チェックの最新結果を再取得しておらず、修正後の再解析も未実施である。merge前に次を確認する。

- `javascript:S3403`: Node.jsでの`FileReader`存在確認を`typeof`形式へ変更した。この環境でも、生成する12個のGLBが変更前後でSHA-256一致となった。再解析で指摘が解消するかを確認する。
- `pythonsecurity:S8707`の4件: レポートでは、`tools/backbone_semantics.py`の入力・出力と、`tools/architecture_observation.py`のmanifest・出力へのCLI引数が対象となっている。信頼されたローカルCLIとして使う前提では誤検出候補だが、別の自動化から未検証の引数を渡す場合の権限境界は未確認である。CMakeはソースツリー外への出力を使うため、一律のリポジトリ内制限は追加していない。呼出し側の権限モデルを確認した上で、SonarCloud上の判定を確定する。

## 残blocker

### 柱上機材の制作移行

制作方針の正本は[wire/pole_fittings.md](wire/pole_fittings.md)。現行M01の固定支持腕と碍子はまだTypeScriptの相対配置であり、Blenderを正本とした往復編集は未実装である。次段階は1つのblend内のCollectionとROOTから1パターンを出力し、部品移動・再出力・外部socketの追従を既存viewerで確認すること。固定配線のLINES読込と、共有Meshを持つ2パターンの編集・実行時選択は、その後の独立した段階として残る。

固定部品と可変rowの一般的な衝突回避は今回のroot回転修正では保証しない。タンク、支持腕、固定配線の造形は、往復編集が通った後にBlenderで調整する。part補正値や機材専用のCore分岐で補わない。

外部socketの番号について、制作規約はcategory内のレーン番号と説明するが、現行Coreはcategory内の端点列をlane・edge Bundle・Spanの順で並べて参照する。曲がり角では同じlaneの両脚が並ぶため、この説明との差を別途確認する。今回の修正は番号の解釈を変えず、実際に選択されたsocketへの位置追従を検証する。

### その他のblocker

| ID | 操作 | 現状 | 解除条件 |
|---|---|---|---|
| R2 | `ResolveBranchPick`のpure化 | `ResolveBranchPick`は現在、viewerのdraw bridgeから次の生成処理へ渡すpending support nodeも作成する。副作用のない照会APIへ分離するには、bridgeの受け渡し契約を変更する必要がある | `ResolveBranchPick`は解決結果だけを返し、pending support nodeの作成・消費は別の確定操作へ移す。web bridgeとclear/cancel lifecycleを同時に更新し、既存のpending lifecycleテストを通す |
| R3 | road authored profile / reference mesh UV sourceがない | `Mesh`はnormal、uv0、material groupを持つが、現在のUVにはprocedural defaultを使っている。Blender等で制作したgutter/highway reference meshからcross section、V、material groupを取り込む経路はまだない | reference mesh/profile asset importerを追加し、procedural default UVを制作データ由来のメタデータへ置き換えられることを実メッシュで確認する |

## unsupported保留の一覧

不正入力を恒久的に拒否するpipelineの`unsupported`は、この表に含めない。ここでは、未実装のため保留している操作だけを扱う。viewerでは事前に判別せず、Apply後のerror logでunsupportedを確認する。

| 分類 | 操作 | viewer到達 | 残る条件 | 解除条件 |
|---|---|---|---|---|
| D3. 生成処理が入力を未使用 | `UpdateContextProfile` | viewer未接続 | backbone spanあり | regenerateへの対応ではない。生成処理が`ResolveStyleContext`を実際の出力生成で使うようになった時点で解除する。現在の拒否は、変更が出力へ反映されないまま成功することを防ぐためである |
| R1. Road section not expressible | 断面外端の`BoundaryProfile` | viewer到達可(preset作成時) | `boundaries.size() + 1 == strips.size()` | 境界をgapごと(`strips.size() + 1`個)へ変え、外端も実境界にする。合成ID 1 / 999が消え、`merge_boundary_policies`のindexとjunctionの外端判定が連動する。歩道のない側のL字溝と、`road/architecture.md`の「歩道がない場合」の張り出しがこれで解ける |
| R2. Road connection mapping | 断面styleが異なる道路同士の接続 | viewer到達可 | `AddSegmentConnectedTo`等 | `connection_geometry_from_gates`が`surface_styles`完全一致を要求している。片側だけ溝がある、左右で溝が違う、溝つきと縁石をつなぐ、が通らない。gate間のsurface対応をID/roleで解決してから解除する |

## 正本を別に持つ制限

| 操作 | 制限 | 正本 |
|---|---|---|
| `UpdateCableTemplate` | non-backbone spanを含むdecision差分は状態変更前にunsupported。backbone-only scopeは統一regenerateが所有する | [wire/backbone_operation_semantics.md](wire/backbone_operation_semantics.md) のpost-edit regenerate契約 |
| `UpdateAttachmentTemplate` | 使用中attachmentの構造差分(socket増減/id変更、mode変更、internal path本数/socket参照/kind変更)は、モデル再読込conflictとして解決するまで状態変更前にunsupported | [wire/models.md](wire/models.md) の`UpdateAttachmentTemplate`構造差分 |
| Road archive | 現段階では保存互換を保証しない。バージョンフィールドとmigration分岐を持たず、現行スキーマだけを読み書きする | [road/architecture.md](road/architecture.md) のPersistence target |

既に保留から外したもの: `UpdateBundleTemplate`のfixed count増減、`kTopology` policy差分、range化(`count_rule` fixed/range、`min_count` / `max_count` / `default_count`: 保存済み`conductor_count`が新policy内なら出力不変、外なら状態変更前拒否)、multi-bundle、3点以上route、存続attachment保持、退役attachment拒否、存続manual port保持、退役manual port拒否、`population_rules` / `support_wire_pole_band_id` / `cable_template_id` detail差分、`related_pole_type_id`の定義更新のみ、metadata/name、`UpdateCableTemplate`のbackbone continuity policy decision差分、geometry/render差分、source-edge branch projection追従、`UpdatePoleTypeDefinition`のactive backbone pole構造差分、`ApplyBundleRelatedPoleTypeToExistingPoles`、backbone spanのendpoint socket / branch-down override、`UpdateLayoutSettings`。

E. Docs stale only: `docs/viewer/operations.md`に残っていたbackbone span / active backbone / decision差分の旧unsupported表記は実装とspec ledgerに合わないため削除した。以後はviewerが事前にunsupported判定せず、coreのerrorを表示する。

## 決定済みの制限

- `AddConnectionByPole`、`AddDropFromPole`、`AddDropFromSpan`、`SplitSpan`は公開API、
  実装、専用result/options型を削除済みである。

これらは未解決blockerではない。旧topology操作が必要なら、`SavedBackboneGraph`を更新する新しい操作として別途設計する。

## 検証で確認した事項

| ID | 対象 | 観測した不足 | 次に確認すること |
|---|---|---|---|
| V1・対応済み | Wireの元の状態から初回loadへの派生等価性 | 旧C751は再loadの比較だけで、Span 54の`layout.end.endpoint_node_id`が0から55へ変わる差を見逃していた | 保存処理が採番済みnode IDを一時graphへ反映するよう修正した。C751は元の状態→初回loadの意味値も比較する。モデルのinstance cache全体は比較対象外。根拠は[2026-11監査記録](../domains/wire/tests/audits/2026-11.md)を参照 |
| V2・未対応 | `MovePole`の失敗時の状態不変 | 端の電柱を隣の電柱の位置へ移すとゼロ長として拒否するが、保存状態は変更される。マトリクスの成功経路とC813では検出できない | 失敗前後の正本・派生出力を既存の移動契約で直接比較し、更新全体の原子性を修正する |
| V3・未対応 | ワークスペース読込全体の原子性 | 正常なWireと不正なRoadを含む文書は読込失敗になるが、Wireだけが置き換わり、表示用ストアは旧状態に残る | Core単体の読込契約と区別し、複数ドメインをまたぐ入口全体で失敗時の状態不変を保証する |
| V4・未対応 | 連続生成後の保存・再導出の等価性 | 実WASMのクリック経路で3区間を連続生成すると、保存・再読込で一部のCableRun IDが変わる。座標とモデル配置の差は観測していない | 一括生成に限らず連続操作後の派生値を比較し、run IDを変える再導出範囲と原因を調べる |
| V5・逐次追加に対応済み | 別ルートが共有する取付位置の支持段 | 添付データでは電柱13の高圧支持列が段0へ重なり、別ルートの線の中心間距離が約1.34mmだった。旧共通不変条件も通過した | 同じ名目取付位置の保存済み段をルートのkeyをまたいで予約し、C809と共通不変条件を強化した。同じ構成の再生成では段0・1・2となり、補助曲線を除く高圧線の最短距離は約537mm。既存archiveの自動修復、同一操作で同時追加する同位置の複数Bundle、異なる取付位置間の一般的な衝突回避は対象外。根拠は[2026-11監査記録](../domains/wire/tests/audits/2026-11.md)を参照 |

既にblockerから外したもの: `MovePole`で通常/鋭角閾値を跨ぐ場合の表現再導出。`C813_backbone_move_pole_rederives_pair_representation`が、endpoint Port IDとrow keyを維持したままfixture 1↔2、NodePatch↔jumperを切り替え、通常角へ戻せることを検証する。

### Future architecture evaluation

具体的なmodifiabilityシナリオが必要になった時点でSAAM等のscenario-based evaluationを検討する。今回はframework、スキーマ、placeholder systemを先回りして実装しない。

## 未確認のまま残っている実行結果

2026-08-25時点でRoad architecture契約テストの2件(`reverse_input_has_equivalent_geometry`、`same_angle_approaches_use_id_tie_break`)がFAILだった。その後の再実行を記録していないため、現在の状態は未確認である。Road側を触る作業で実行して結果を更新する。
