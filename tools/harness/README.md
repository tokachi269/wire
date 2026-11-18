# 移植可能なarchitecture lint

`architecture_lint.py`は、リポジトリに依存せずソースの層と境界を検査するエンジンである。次の設定を持つmanifestを受け取る。

- `scan.roots`
- `scan.extensions`
- `scan.exclude_patterns`
- `layers[].name`
- `layers[].patterns`
- `layers[].forbidden_tokens`

走査した全ファイル、各ファイルを1つの層へ割り当てた結果、未分類・複数分類・禁止tokenのエラーを返す。宣言した走査起点は必須であり、対象ファイルが0件の場合も失敗する。プロジェクト固有のドメイン語彙は持たない。

C++の引用符付きliteral includeは、ソース相対、リポジトリ相対、走査対象内で一意なsuffixの順に参照先を解決する。解決先のリポジトリ相対pathにも、既存の禁止tokenを適用する。architecture observationも同じ解決処理を使う。

compilerのinclude検索順やpreprocessor全体は再現しない。macro include、候補が複数ある参照、走査対象外の参照は、この検査では保証しない。

パターンはslash区切りで、1つのpath segment内の`*`と`?`を扱う。完全な`**` segmentは0個以上の入れ子のdirectoryに一致する。`source/**/*.ext`は直下から任意の深さまでを含み、再帰的な除外にも同じ意味を適用する。文字クラスなどshell固有のglob機能は、共通契約には含めない。

manifestの不正はlintの診断として返す。Rootとextensionは空でないstring list、layer名は空でなく一意、patternと禁止tokenはstring listとする。走査起点を暗黙に任意扱いする動作は設けない。

各プロジェクトは固有のmanifestとcomposition rootを持つ。composition rootで設定を読み込み、`lint_architecture`を呼ぶ。その後、プロジェクト固有の文書、操作意味論の検証範囲、安定した正本境界を検査する。

合成fixtureによる自己テストは次のコマンドで実行する。

```powershell
python tools/harness/test_architecture_lint.py
```

テストでは一時プロジェクトを使う。正常な分類、必須の走査起点、空の走査結果、manifest不正、未分類・複数分類、禁止token、入れ子のpattern、再帰的な除外、複数の走査起点、composition rootからの呼び出し委譲を検査する。
