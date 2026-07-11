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

実行コマンドは[command_cheatsheet.md](command_cheatsheet.md)を参照する。

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
| B. Non-backbone update path pending | `UpdateCableTemplate` | `Apply Cable Template` | non-backbone span を含む decision差分 | backbone span の continuity policy decision差分は統一 regenerate で対応済み(C357/C712)。non-backbone の構造 decision 更新は別経路を設計する |
| C. Structural template lifecycle pending | `UpdateAttachmentTemplate` | viewer未接続 | 使用中attachmentあり + 構造差分(socket増減/id変更、mode変更、internal path本数/socket参照/kind変更) | 幾何差分(socket位置/方向、internal path local_points/coil値)はkReshapeで対象spanを再導出する。構造差分は実用 scenario が出た時点で正本と退役規則を設計する |
| D. Generator input not consumed | `UpdateVariationSettings` | viewer未接続 | backbone spanあり | regenerateではない。backbone派生がvariation settingsを実際の出力生成で消費した時点で解除する。現在の拒否は stale success 防止 |
| D. Generator input not consumed | `UpdateContextProfile` | viewer未接続 | backbone spanあり | regenerateではない。生成側が`ResolveStyleContext`を実際の出力生成で消費した時点で解除する。現在の拒否は stale success 防止 |

既に保留から外したもの: `UpdateBundleTemplate` の fixed count 増減、`kTopology` policy差分、range化(`count_rule` fixed/range、`min_count` / `max_count` / `default_count`: 保存済み`conductor_count`が新policy内なら出力不変、外ならmutation前拒否)、multi-bundle、3点以上route、存続attachment保持、退役attachment拒否、存続manual port保持、退役manual port拒否、`population_rules` / `support_wire_pole_band_id` / `cable_template_id` detail差分、`related_pole_type_id` の定義更新のみ、metadata/name、`UpdateCableTemplate` の backbone continuity policy decision差分、geometry/render差分、source-edge branch projection追従、`UpdatePoleTypeDefinition` の active backbone pole 構造差分、`ApplyBundleRelatedPoleTypeToExistingPoles`、backbone span の endpoint socket / branch-down override、`UpdateLayoutSettings`。

E. Docs stale only: `docs/viewer_operations.md` に残っていた backbone span / active backbone / decision差分の旧unsupported表記は実装と spec ledger に合わないため削除した。以後は viewer が事前にunsupported判定せず、core の error を表示する。

## 残blocker

source上の既知blockerはない。必須check、実画面の最終確認、mainとの差分・競合確認は必要である。

## merge可能条件

必須checkが全て通った時点でsource上はmerge可能と判断する。
実画面の最終確認とmainとの競合確認は別途行う。
