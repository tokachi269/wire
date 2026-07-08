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
- unsupported requestがmutation前に拒否される
- post-edit successがstale outputを残さない
- wire coreが外部domain型へ依存しない

## 決定済みの制限

- `AddConnectionByPole`、`AddDropFromPole`、`AddDropFromSpan`、`SplitSpan`はpublic API、
  実装、専用result/options型を削除済みである。

これらは未解決blockerではない。旧topology operationが必要なら、
`SavedBackboneGraph`を更新する新しいoperationとして別途設計する。

## unsupported保留の一覧

pipelineの入力validation系`unsupported`(不正入力の恒久拒否)はこの表の対象外。
ここに載るのは「実装が無いため止めている」操作だけで、根本原因は
`execute_update_plan`の`kRegenerate`未接続、または該当 scenario の統一 regenerate 未対応に集約される。
viewerはこれらを事前に判別せず、Apply後のerror logで初めてunsupportedが見える。

| 操作 | viewer到達 | 拒否条件 | 解除条件 |
|---|---|---|---|
| `UpdatePoleTypeDefinition` | `Apply Pole Template`(Pole Placement含む) | active backbone poleが対象typeを使用 | placement-only差分(band高さ/横位置、pole高さ)はkRepositionで通す。band増減・enabled/side/layer/role/priority変更、anchor slot変更などの構造差分は統一 regenerate で対応済み(C713)。存続 lane の manual port は保持し、退役 lane の manual port は mutation 前に拒否する(C672/C714) |
| `UpdateBundleTemplate` | `Apply Bundle Template` | topology級差分 + 既存bundleあり | fixed count増減は単一 saved route に限り統一 regenerate で対応。multi-bundle と pair row を含む3点以上routeも saved graph から再構成する。存続spanのattachmentは保持し、退役spanのattachmentはmutation前に拒否する。range化、policy変更は scenario 未対応。visual-only/detail差分は現状も通る |
| `UpdateCableTemplate` | `Apply Cable Template` | non-backbone span を含む decision差分 / default endpoint attachment の構造影響 | backbone span の continuity policy decision差分は統一 regenerate で対応済み(C357/C712)。geometry/render差分は現状も通る。non-backbone 対象と default endpoint attachment の構造影響は mutation前 unsupported |
| `ApplyBundleRelatedPoleTypeToExistingPoles` | template Apply後に自動呼出 | 対象poleがbackbone node | 対象 pole の type 差し替え + incident edge の統一 regenerate で対応済み(C297)。暗黙連結ではなく独立 API |
| `UpdateLayoutSettings` | `Apply Layout` | backbone span bindingが1つでも存在 | generation入力のためregenerate級。state全体一括拒否の粒度は再検討候補 |
| `SetSpanEndpointSocketOverride` / Clear | 非backbone span向けのみ | backbone span | socketはgeneration所有。該当 family の migration 実装 |
| `SetSpanBranchDownOffsetOverride` / Clear | 非backbone span向けのみ | backbone span | 同上 |
| `UpdateAttachmentTemplate` | viewer未接続 | 使用中attachmentあり + 構造差分(socket増減/id変更、mode変更、internal path本数/socket参照/kind変更) | 幾何差分(socket位置/方向、internal path local_points/coil値)はkReshapeで対象spanを再導出する。構造差分は該当 family の migration 実装まで保留 |
| `UpdateVariationSettings` | viewer未接続 | backbone spanあり | regenerateではない。backbone派生がvariation settingsを消費した時点で解除 |
| `UpdateContextProfile` | viewer未接続 | backbone spanあり | 同上。生成側が`ResolveStyleContext`を消費した時点で解除 |

## 残blocker

source上の既知blockerはない。必須check、実画面の最終確認、mainとの差分・競合確認は必要である。

## merge可能条件

必須checkが全て通った時点でsource上はmerge可能と判断する。
実画面の最終確認とmainとの競合確認は別途行う。
