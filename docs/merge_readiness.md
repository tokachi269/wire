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

- `UpdatePoleTypeDefinition`は、対象typeをactive backbone poleが使用中ならmutation前に
  `unsupported`を返す。非backbone poleだけが使用するtypeは従来どおり更新できる。
- `AddConnectionByPole`、`AddDropFromPole`、`AddDropFromSpan`、`SplitSpan`はpublic API、
  実装、専用result/options型を削除済みである。

これらは未解決blockerではない。active pole typeの一括migrationや旧topology operationが必要なら、
`SavedBackboneGraph`を更新する新しいoperationとして別途設計する。

## 残blocker

source上の既知blockerはない。必須check、実画面の最終確認、mainとの差分・競合確認は必要である。

## merge可能条件

必須checkが全て通った時点でsource上はmerge可能と判断する。
実画面の最終確認とmainとの競合確認は別途行う。
