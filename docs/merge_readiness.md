# Main merge readiness

この文書は、mainへmergeする前のゲートを示す。現在の保留、blocker、検証で見つかった不足は[status.md](status.md)にある。

## 必須の確認

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

## merge可能条件

必須checkが全て通った時点でsource上はmerge可能と判断する。実画面の最終確認とmainとの競合確認は別途行う。
