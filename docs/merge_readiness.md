# Main merge readiness

この文書は、mainへmergeする前のゲートを示す。現在の保留、blocker、検証で見つかった不足は[status.md](status.md)にある。

柱上機材のBlender制作移行は段階実装中である。M01の1パターンは制作blendとGLBの往復・外部socket追従をデータ上で検証済みだが、新GLBのviewer上の最終確認は未実施である。固定配線と複数パターンは次段階として残る。残る確認は[status.mdの柱上機材](status.md#柱上機材の制作移行)、制作契約は[wire/pole_fittings.md](wire/pole_fittings.md)を参照する。

## 必須の確認

- Coreの既定テストが通る
- backbone acceptanceが通る
- viewerのテストが通る
- viewer applicationがbuildできる
- `tools/arch_lint.py`が通る
- `tools/test_family_lint.py`が通る
- `git diff --check`が通る
- working treeが意図した状態である
- 正本のsave -> loadで、再保存したbyte列と、再導出したlayout / curve / bounds / visualがbit-exact roundtripのテストを通る

実行コマンドは[command_cheatsheet.md](command_cheatsheet.md)を参照する。

## architecture条件

- `SavedBackboneGraph`がtopology authorityである
- pair/open/rowの決定者が一箇所である
- layout/geom/drawが上流判断を再実行しない
- viewer/inspectionが状態を補正しない
- v1/recalc/materializationへのfallbackがない
- post-edit regenerateと通常生成の`GenerateFromBackboneSpec`は、入力、識別情報、binding、構造条件を満たさない要求を状態変更前にunsupportedとして拒否する。後半の派生幾何処理で失敗した場合も、本状態を変更しない
- post-editの成功時に古い出力を残さない
- Wire Coreが他ドメインの型へ依存しない
- Bundleのテンプレートを識別するには`BundleTemplateId`を使い、`BundleKind`はcategory/tagとしてだけ使う

## merge可能条件

必須の確認が全て通った時点で、source上はmerge可能と判断する。実画面の最終確認とmainとの競合確認は別途行う。
