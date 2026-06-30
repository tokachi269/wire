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

実行コマンドは[operations.md](operations.md)を参照する。

## architecture条件

- `SavedBackboneGraph`がtopology authorityである
- pair/open/rowの決定者が一箇所である
- layout/geom/drawが上流判断を再実行しない
- viewer/inspectionがstateを補正しない
- v1/recalc/materializationへのfallbackがない
- unsupported requestがmutation前に拒否される
- post-edit successがstale outputを残さない
- wire coreが外部domain型へ依存しない

## 残blocker

### `UpdatePoleTypeDefinition`

現状はdefinition更新後に既存poleへ`ApplyPoleType()`を順次適用する。
active generated spanを含む場合に、途中失敗のatomicityとlayout/geom/drawの追随を説明し切れていない。
次のどちらかを固定するまでmerge blockerとする。

- mutation前に対象をpreflightし、全対象を`kReposition`でdirect deriveする
- topology/identity変更が必要な場合はdefinitionを変更する前に`unsupported`を返す

### legacy topology public API

`AddConnectionByPole`、`AddDrop*`、`SplitSpan`はmutation前`unsupported`でstateを壊さない。
normal pathにはいないためarchitecture blockerではないが、mainへ残すpublic surfaceとして許容するか、宣言ごと削除するかをmerge前に決める。

## merge可能条件

上記blockerを解消または明示的に受容し、必須checkが全て通った時点でsource上はmerge可能と判断する。
実画面の最終確認とmainとの競合確認は別途行う。

