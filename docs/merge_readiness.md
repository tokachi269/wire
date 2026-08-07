# Main merge readiness

この文書は、mainへmergeする前の判断基準を示す。

## 必須check

- default core testsが通る
- backbone acceptanceが通る
- viewer testsが通る
- viewer applicationがbuildできる
- `tools/arch_lint.py`が通る
- `tools/test_family_lint.py`が通る
- `git diff --check`が通る
- working treeが意図した状態である
- authoritative state の save -> load が、再保存byteと再導出した layout / curve / bounds / visual のbit-exact roundtrip testを通る

実行コマンドは[command_cheatsheet.md](command_cheatsheet.md)を参照する。

## test evidence debt

2026-07-29時点のfull core test実測:

- `derived_equality`単独case: 0
- `legacy_unclassified`: 419

`legacy_unclassified`は一括書換えしない。matrix観測に使うcaseは即時分類し、その他は
production/testを触った作業単位で分類する。case総数の増加ではなく、この残件数の減少を
進捗として扱う。

## architecture条件

- `SavedBackboneGraph`がtopology authorityである
- pair/open/rowの決定者が一箇所である
- layout/geom/drawが上流判断を再実行しない
- viewer/inspectionがstateを補正しない
- v1/recalc/materializationへのfallbackがない
- post-edit regenerate と通常生成 `GenerateFromBackboneSpec` は input / identity / binding / 構造条件を満たさない unsupported request を mutation 前に拒否し、派生 geometry 固有の後半失敗でも本 state を変更しない
- post-edit successがstale outputを残さない
- wire coreが外部domain型へ依存しない
- bundle template identity は `BundleTemplateId` であり、`BundleKind` は category/tag としてのみ使う

## 決定済みの制限

- `AddConnectionByPole`、`AddDropFromPole`、`AddDropFromSpan`、`SplitSpan`はpublic API、
  実装、専用result/options型を削除済みである。

これらは未解決blockerではない。旧topology operationが必要なら、
`SavedBackboneGraph`を更新する新しいoperationとして別途設計する。

## unsupported保留の一覧

pipelineの入力validation系`unsupported`(不正入力の恒久拒否)はこの表の対象外。
ここに載るのは「実装が無いため止めている」操作だけである。
viewerはこれらを事前に判別せず、Apply後のerror logで初めてunsupportedが見える。

| 分類 | 操作 | viewer到達 | 残る条件 | 解除条件 |
|---|---|---|---|---|
| D2. Generator input not consumed | `UpdateVariationSettings` | viewer未接続 | backbone spanあり | regenerateではない。backbone派生がvariation settingsを実際の出力生成で消費した時点で解除する。現在の拒否は stale success 防止 |
| D3. Generator input not consumed | `UpdateContextProfile` | viewer未接続 | backbone spanあり | regenerateではない。生成側が`ResolveStyleContext`を実際の出力生成で消費した時点で解除する。現在の拒否は stale success 防止 |
| R1. Road section not expressible | 断面外端の`BoundaryProfile` | viewer到達可(preset作成時) | `boundaries.size() + 1 == strips.size()` | boundaryをgapごと(`strips.size() + 1`個)へ変え、外端も実boundaryにする。合成ID 1 / 999が消え、`merge_boundary_policies`のindexとjunctionの外端判定が連動する。歩道のない側のL字溝と、`road/architecture.md`の「歩道がない場合」の張り出しがこれで解ける |
| R2. Road connection mapping | 断面styleが異なる道路同士の接続 | viewer到達可 | `AddSegmentConnectedTo` 等 | `connection_geometry_from_gates`が`surface_styles`完全一致を要求している。片側だけ溝がある、左右で溝が違う、溝つきと縁石をつなぐ、が通らない。gate間のsurface対応をID/roleで解決してから解除する |

## 決定済みの制限

| 操作 | 制限 | 正本 |
|---|---|---|
| `UpdateCableTemplate` | non-backbone span を含む decision差分は mutation 前に unsupported。backbone-only scope は統一 regenerate が所有する | [wire/architecture.md](wire/architecture.md) の transaction / regenerate 契約 |
| `UpdateAttachmentTemplate` | 使用中 attachment の構造差分(socket増減/id変更、mode変更、internal path本数/socket参照/kind変更)は、モデル再読込 conflict として解決するまで mutation 前に unsupported | [wire/models.md](wire/models.md) の `UpdateAttachmentTemplate` 構造差分 |

既に保留から外したもの: `UpdateBundleTemplate` の fixed count 増減、`kTopology` policy差分、range化(`count_rule` fixed/range、`min_count` / `max_count` / `default_count`: 保存済み`conductor_count`が新policy内なら出力不変、外ならmutation前拒否)、multi-bundle、3点以上route、存続attachment保持、退役attachment拒否、存続manual port保持、退役manual port拒否、`population_rules` / `support_wire_pole_band_id` / `cable_template_id` detail差分、`related_pole_type_id` の定義更新のみ、metadata/name、`UpdateCableTemplate` の backbone continuity policy decision差分、geometry/render差分、source-edge branch projection追従、`UpdatePoleTypeDefinition` の active backbone pole 構造差分、`ApplyBundleRelatedPoleTypeToExistingPoles`、backbone span の endpoint socket / branch-down override、`UpdateLayoutSettings`。

E. Docs stale only: `docs/viewer/operations.md` に残っていた backbone span / active backbone / decision差分の旧unsupported表記は実装と spec ledger に合わないため削除した。以後は viewer が事前にunsupported判定せず、core の error を表示する。

## 残blocker

| ID | 操作 | 現状 | 解除条件 |
|---|---|---|---|
| R2 | `ResolveBranchPick` の pure 化 | `ResolveBranchPick` は現在、viewer の draw bridge が次の generation に渡す pending support node を作る操作でもある。照会APIとして pure に分離するには bridge 協定変更が必要 | `ResolveBranchPick` は副作用なしの解決だけを返し、pending support node の作成/消費は別の確定操作へ移す。web bridge と clear/cancel lifecycle を同時に更新し、既存 pending lifecycle tests を通す |
| R3 | road `Mesh` に法線とUVがない | `derived_types/derived_road.hpp`の`Mesh`は頂点とindexだけを持つ。L字溝の垂直面と上面の稜が hard edge にならない | `Mesh`が法線を持ち、profileの点で分割された面が別々の法線を受け取る。Unrealへ出す前に必要 |
| J2 | `MovePole`で接続済みpairの角度が通常/鋭角閾値を跨ぐ | endpoint Port identityとcontinuityは維持されるが、repositionは保存済みlayout ruleを再利用するため、row/fixtureとNodePatch/jumperの表現切替を再導出しない | `SavedBackboneRowContinuity`と現在幾何から通常/鋭角表現を再導出し、Port ID不変のままfixture 1↔2、NodePatch↔jumperを切り替えるfail-first testを通す |

J2がsource上の既知blockerである。必須check、実画面の最終確認、mainとの差分・競合確認も必要である。

## merge可能条件

必須checkが全て通った時点でsource上はmerge可能と判断する。
実画面の最終確認とmainとの競合確認は別途行う。
