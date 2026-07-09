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
- bundle template identity は `BundleTemplateId` であり、`BundleKind` は category/tag としてのみ使う

## 決定済みの制限

- `AddConnectionByPole`、`AddDropFromPole`、`AddDropFromSpan`、`SplitSpan`はpublic API、
  実装、専用result/options型を削除済みである。

これらは未解決blockerではない。旧topology operationが必要なら、
`SavedBackboneGraph`を更新する新しいoperationとして別途設計する。

## unsupported保留の一覧

pipelineの入力validation系`unsupported`(不正入力の恒久拒否)はこの表の対象外。
ここに載るのは「実装が無いため止めている」操作だけで、根本原因は
該当 scenario の統一 regenerate 未対応、または variation/context の生成入力未消費に集約される。
viewerはこれらを事前に判別せず、Apply後のerror logで初めてunsupportedが見える。

| 操作 | viewer到達 | 拒否条件 | 解除条件 |
|---|---|---|---|
| `UpdateBundleTemplate` | `Apply Bundle Template` | range化、bundle policy変更 | fixed count増減、multi-bundle、3点以上route、存続attachment保持、退役attachment拒否は対応済み。range/policy を実用 scenario として扱う時点で統一 regenerate の scope と fresh 等価性を追加する |
| `UpdateCableTemplate` | `Apply Cable Template` | non-backbone span を含む decision差分 / default endpoint attachment の構造影響 | backbone span の continuity policy decision差分は統一 regenerate で対応済み(C357/C712)。geometry/render差分は現状も通る。non-backbone 対象と default endpoint attachment の構造影響は mutation前 unsupported |
| `UpdateAttachmentTemplate` | viewer未接続 | 使用中attachmentあり + 構造差分(socket増減/id変更、mode変更、internal path本数/socket参照/kind変更) | 幾何差分(socket位置/方向、internal path local_points/coil値)はkReshapeで対象spanを再導出する。構造差分は実用 scenario が出た時点で正本と退役規則を設計する |
| `UpdateVariationSettings` | viewer未接続 | backbone spanあり | regenerateではない。backbone派生がvariation settingsを消費した時点で解除 |
| `UpdateContextProfile` | viewer未接続 | backbone spanあり | 同上。生成側が`ResolveStyleContext`を消費した時点で解除 |

## 残blocker

source上の既知blockerはない。必須check、実画面の最終確認、mainとの差分・競合確認は必要である。

## merge可能条件

必須checkが全て通った時点でsource上はmerge可能と判断する。
実画面の最終確認とmainとの競合確認は別途行う。
