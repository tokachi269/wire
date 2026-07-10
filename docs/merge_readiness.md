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
- post-edit regenerate は input / identity / binding / 構造条件を満たさない unsupported request を mutation 前に拒否し、派生 geometry 固有の後半失敗でも本 state を変更しない。通常生成 `GenerateFromBackboneSpec` の transaction 境界は未完であり、完了条件として通過扱いにしない
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
| A. Regenerate scenario pending | `UpdateBundleTemplate` | `Apply Bundle Template` | range化: `count_rule` の fixed/range 切替、`min_count` / `max_count` / `default_count` 変更。count の生成数・退役範囲・fresh 等価性を fixed count 増減と同じ規則でまだ定義していない | range を実用 scenario として扱う時点で、scope 収集、退役 span/port/attachment/manual port 規則、fresh 等価性を追加する |
| A. Regenerate scenario pending | `UpdateBundleTemplate` | `Apply Bundle Template` | bundle policy変更: `category`、`default_layer`、`preserve_conductor_identity`、`allow_mirror`、`allow_midair_node`、`allow_midair_branch`、`enable_branch_down_offset`、`branch_endpoint_offset_m`、`order_decision_policy`、`row_layout_axis_mode`、`support_style`、`branch_policy`、`continuity_policy`。pair/row/lane/endpoint/branch decision を変えるため、既存 lane 数だけを増減する fixed count と同一扱いにできない | policy ごとに regenerate 入力へ保存済み topology をどう再解釈するかを scenario 化する |
| B. Non-backbone update path pending | `UpdateCableTemplate` | `Apply Cable Template` | non-backbone span を含む decision差分 | backbone span の continuity policy decision差分は統一 regenerate で対応済み(C357/C712)。non-backbone の構造 decision 更新は別経路を設計する |
| C. Structural template lifecycle pending | `UpdateAttachmentTemplate` | viewer未接続 | 使用中attachmentあり + 構造差分(socket増減/id変更、mode変更、internal path本数/socket参照/kind変更) | 幾何差分(socket位置/方向、internal path local_points/coil値)はkReshapeで対象spanを再導出する。構造差分は実用 scenario が出た時点で正本と退役規則を設計する |
| D. Generator input not consumed | `UpdateCableTemplate` | `Apply Cable Template` | `default_endpoint_attachment_template_id` 変更。decision regenerate は通るが、`ensure_default_endpoint_attachments_for_span` は post-edit 経路から呼ばれず、endpoint attachment の生成/退役/置換がまだ反映されない | endpoint attachment を生成入力として pipeline / regenerate が消費し、既存 attachment の保持・退役規則を定義する |
| D. Generator input not consumed | `UpdateVariationSettings` | viewer未接続 | backbone spanあり | regenerateではない。backbone派生がvariation settingsを実際の出力生成で消費した時点で解除する。現在の拒否は stale success 防止 |
| D. Generator input not consumed | `UpdateContextProfile` | viewer未接続 | backbone spanあり | regenerateではない。生成側が`ResolveStyleContext`を実際の出力生成で消費した時点で解除する。現在の拒否は stale success 防止 |

`UpdateBundleTemplate` の policy 未対応内訳:

| field | fixed count 増減と同じ扱いにできない理由 |
|---|---|
| fixed ↔ range | lane 数が単一値でなくなり、生成数・退役範囲・fresh 等価性の基準が変わる |
| `min_count` / `max_count` / `default_count` | range の実 conductor count 解決規則と退役規則が未定義 |
| `category` | pole band / row / grouping の解決対象が変わり、既存 lane の増減だけでは済まない |
| `default_layer` | span layer と route grouping の保存済み identity に影響する |
| `preserve_conductor_identity` | lane identity の引き継ぎ規則そのものを変える |
| `allow_mirror` | row/lane の反転許可を再判定する必要がある |
| `allow_midair_node` | source/pending node の受理条件を変えるため既存 topology の意味が変わる |
| `allow_midair_branch` | branch materialization 可否を変えるため既存 branch scope の意味が変わる |
| `enable_branch_down_offset` | endpoint lowering/branch-down layout decision を再解決する必要がある |
| `branch_endpoint_offset_m` | branch endpoint 位置の decision 入力が変わり、curve/layoutだけで閉じない |
| `order_decision_policy` | pair/order の決定規則が変わり、saved route の再解釈が必要 |
| `row_layout_axis_mode` | pair row axis の決定規則が変わり、port row と jumper が再解釈対象になる |
| `support_style` | support/node patch まわりの構造 decision に影響する |
| `branch_policy` | branch と through の connectivity decision が変わる |
| `continuity_policy` | bundle-level continuity decision を route全体で fresh と照合する scenario が未定義 |

既に保留から外したもの: `UpdateBundleTemplate` の fixed count 増減、multi-bundle、3点以上route、存続attachment保持、退役attachment拒否、存続manual port保持、退役manual port拒否、`population_rules` / `support_wire_pole_band_id` / `cable_template_id` detail差分、`related_pole_type_id` の定義更新のみ、metadata/name、`UpdateCableTemplate` の backbone continuity policy decision差分、geometry/render差分、source-edge branch projection追従、`UpdatePoleTypeDefinition` の active backbone pole 構造差分、`ApplyBundleRelatedPoleTypeToExistingPoles`、backbone span の endpoint socket / branch-down override、`UpdateLayoutSettings`。

E. Docs stale only: `docs/viewer_operations.md` に残っていた backbone span / active backbone / decision差分の旧unsupported表記は実装と spec ledger に合わないため削除した。以後は viewer が事前にunsupported判定せず、core の error を表示する。

## 残blocker

source上の既知blockerはない。必須check、実画面の最終確認、mainとの差分・競合確認は必要である。

## merge可能条件

必須checkが全て通った時点でsource上はmerge可能と判断する。
実画面の最終確認とmainとの競合確認は別途行う。
