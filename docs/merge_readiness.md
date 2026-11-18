# Main merge readiness

この文書は、mainへmergeする前の判断基準を示す。

## 必須check

- Coreの既定テストが通る
- backbone acceptanceが通る
- viewerのテストが通る
- viewer applicationがbuildできる
- `tools/arch_lint.py`が通る
- `tools/test_family_lint.py`が通る
- `git diff --check`が通る
- working treeが意図した状態である
- authoritative状態のsave -> loadが、再保存byteと再導出したlayout / curve / bounds / visualのbit-exact roundtripテストを通る

実行コマンドは[command_cheatsheet.md](command_cheatsheet.md)を参照する。

## test evidence debt

2026-08-25時点のspec ledger集計。

- `derived_equality`単独ケース: 0
- `legacy_unclassified`: 422

この集計は過去のsnapshotであり、現在値や品質目標ではない。`legacy_unclassified`は一括書換えせず、重要な契約の検証に関係するケースから実際の検証条件と期待値の根拠を確認する。matrix観測に使うケースは分類し、その他はproduction/testを触った作業単位で扱う。

進捗は、新たに検出・防止できる故障と、明らかになった保証範囲で判断する。残件数や分類率を進捗目標にしない。

## architecture条件

- `SavedBackboneGraph`がtopology authorityである
- pair/open/rowの決定者が一箇所である
- layout/geom/drawが上流判断を再実行しない
- viewer/inspectionが状態を補正しない
- v1/recalc/materializationへのfallbackがない
- post-edit regenerateと通常生成`GenerateFromBackboneSpec`は入力 / 識別情報 / binding / 構造条件を満たさないunsupported requestを状態変更前に拒否し、派生幾何固有の後半失敗でも本状態を変更しない
- post-edit successがstale出力を残さない
- wire coreが外部ドメイン型へ依存しない
- bundleテンプレート識別情報は`BundleTemplateId`であり、`BundleKind`はcategory/tagとしてのみ使う

## 決定済みの制限

- `AddConnectionByPole`、`AddDropFromPole`、`AddDropFromSpan`、`SplitSpan`は公開API、
  実装、専用result/options型を削除済みである。

これらは未解決blockerではない。旧topology操作が必要なら、`SavedBackboneGraph`を更新する新しい操作として別途設計する。

## unsupported保留の一覧

pipelineの入力検証系`unsupported`(不正入力の恒久拒否)はこの表の対象外。ここに載るのは「実装が無いため止めている」操作だけである。viewerはこれらを事前に判別せず、Apply後のerror logで初めてunsupportedが見える。

| 分類 | 操作 | viewer到達 | 残る条件 | 解除条件 |
|---|---|---|---|---|
| D3. Generator入力not consumed | `UpdateContextProfile` | viewer未接続 | backbone spanあり | regenerateではない。生成側が`ResolveStyleContext`を実際の出力生成で消費した時点で解除する。現在の拒否はstale success防止 |
| R1. Road section not expressible | 断面外端の`BoundaryProfile` | viewer到達可(preset作成時) | `boundaries.size() + 1 == strips.size()` | 境界をgapごと(`strips.size() + 1`個)へ変え、外端も実境界にする。合成ID 1 / 999が消え、`merge_boundary_policies`のindexとjunctionの外端判定が連動する。歩道のない側のL字溝と、`road/architecture.md`の「歩道がない場合」の張り出しがこれで解ける |
| R2. Road connection mapping | 断面styleが異なる道路同士の接続 | viewer到達可 | `AddSegmentConnectedTo`等 | `connection_geometry_from_gates`が`surface_styles`完全一致を要求している。片側だけ溝がある、左右で溝が違う、溝つきと縁石をつなぐ、が通らない。gate間のsurface対応をID/roleで解決してから解除する |

## 決定済みの制限

| 操作 | 制限 | 正本 |
|---|---|---|
| `UpdateCableTemplate` | non-backbone spanを含むdecision差分は状態変更前にunsupported。backbone-only scopeは統一regenerateが所有する | [wire/backbone_operation_semantics.md](wire/backbone_operation_semantics.md) のpost-edit regenerate契約 |
| `UpdateAttachmentTemplate` | 使用中attachmentの構造差分(socket増減/id変更、mode変更、internal path本数/socket参照/kind変更)は、モデル再読込conflictとして解決するまで状態変更前にunsupported | [wire/models.md](wire/models.md) の`UpdateAttachmentTemplate`構造差分 |
| Road archive | 現段階では保存互換を保証しない。バージョンフィールドとmigration分岐を持たず、現行スキーマだけを読み書きする | [road/architecture.md](road/architecture.md) のPersistence target |

既に保留から外したもの: `UpdateBundleTemplate`のfixed count増減、`kTopology` policy差分、range化(`count_rule` fixed/range、`min_count` / `max_count` / `default_count`: 保存済み`conductor_count`が新policy内なら出力不変、外なら状態変更前拒否)、multi-bundle、3点以上route、存続attachment保持、退役attachment拒否、存続manual port保持、退役manual port拒否、`population_rules` / `support_wire_pole_band_id` / `cable_template_id` detail差分、`related_pole_type_id`の定義更新のみ、metadata/name、`UpdateCableTemplate`のbackbone continuity policy decision差分、geometry/render差分、source-edge branch projection追従、`UpdatePoleTypeDefinition`のactive backbone pole構造差分、`ApplyBundleRelatedPoleTypeToExistingPoles`、backbone spanのendpoint socket / branch-down override、`UpdateLayoutSettings`。

E. Docs stale only: `docs/viewer/operations.md`に残っていたbackbone span / active backbone / decision差分の旧unsupported表記は実装とspec ledgerに合わないため削除した。以後はviewerが事前にunsupported判定せず、coreのerrorを表示する。

## 残blocker

| ID | 操作 | 現状 | 解除条件 |
|---|---|---|---|
| R2 | `ResolveBranchPick`のpure化 | `ResolveBranchPick`は現在、viewerのdraw bridgeが次のgenerationに渡すpending support nodeを作る操作でもある。照会APIとしてpureに分離するにはbridge協定変更が必要 | `ResolveBranchPick`は副作用なしの解決だけを返し、pending support nodeの作成/消費は別の確定操作へ移す。web bridgeとclear/cancel lifecycleを同時に更新し、既存pending lifecycleテストを通す |
| R3 | road authored profile / reference mesh UV sourceがない | `Mesh`はnormal、uv0、material groupを持つが、現行UVはprocedural defaultである。Blender等で制作したgutter/highway reference meshからcross section、V、material groupを取り込む経路はまだない | reference mesh/profile asset importerを追加し、procedural default UVをauthoredメタデータへ置換できることを実メッシュで確認する |

## 検証で確認した事項

| ID | 対象 | 観測した不足 | 次に確認すること |
|---|---|---|---|
| V1・対応済み | Wireの元の状態から初回loadへの派生等価性 | 旧C751は再loadの比較だけで、Span 54の`layout.end.endpoint_node_id`が0から55へ変わる差を見逃していた | 保存担当箇所が採番済みnode IDを一時graphへ反映するよう修正した。C751は元の状態→初回loadの意味値も比較する。モデルinstanceキャッシュ全体は対象外。根拠は[2026-11監査記録](../domains/wire/tests/audits/2026-11.md)を参照 |
| V2・未対応 | `MovePole`の失敗時の状態不変 | 端の電柱を隣の電柱の位置へ移すとゼロ長として拒否するが、保存状態は変更される。マトリクスの成功経路とC813では検出できない | 失敗前後の正本・派生出力を既存の移動契約で直接比較し、更新全体の原子性を修正する |
| V3・未対応 | ワークスペース読込全体の原子性 | 正常なWireと不正なRoadを含む文書は読込失敗になるが、Wireだけが置き換わり、表示用ストアは旧状態に残る | Core単体の読込契約と区別し、複数ドメインをまたぐ入口全体で失敗時の状態不変を保証する |
| V4・未対応 | 連続生成後の保存・再導出の等価性 | 実WASMのクリック経路で3区間を連続生成すると、保存・再読込で一部のCableRun IDが変わる。座標とモデル配置の差は観測していない | 一括生成に限らず連続操作後の派生値を比較し、run IDを変える再導出範囲と原因を調べる |

既にblockerから外したもの: `MovePole`で通常/鋭角閾値を跨ぐ場合の表現再導出。`C813_backbone_move_pole_rederives_pair_representation`が、endpoint Port IDとrow keyを維持したままfixture 1↔2、NodePatch↔jumperを切り替え、通常角へ戻せることを検証する。

### Future architecture evaluation

具体的なmodifiabilityシナリオが必要になった時点でSAAM等のscenario-based evaluationを検討する。今回はframework、スキーマ、placeholder systemを先回りして実装しない。

## 直近の検証状況

2026-08-25時点で確認した結果。

- Road coreテスト: 101 cases PASS
- Road architecture lint: PASS
- リポジトリarchitecture lint: PASS
- test-family lint: 585 registered cases / 29 source files PASS
- Web/WASM build: PASS
- `web/tests/wasm.test.ts`: 59 cases PASS
- `git diff --check`: PASS
- Road architecture契約テスト: 2 cases FAIL
  - `reverse_input_has_equivalent_geometry`: `reverse surface mesh geometry differs`
  - `same_angle_approaches_use_id_tie_break`: `road junction perimeter has zero signed area`

Road architecture契約の2件が残るため、現時点では必須check完了ではない。この記録は実画面確認、mainとの差分・競合確認、未記載の必須checkを代替しない。

## merge可能条件

必須checkが全て通った時点でsource上はmerge可能と判断する。実画面の最終確認とmainとの競合確認は別途行う。
