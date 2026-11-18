# 移植可能なarchitecture lint

`architecture_lint.py`はsource境界を検査するrepository非依存engineである。次の値を持つmanifestを受け取る。

- `scan.roots`
- `scan.extensions`
- `scan.exclude_patterns`
- `layers[].name`
- `layers[].patterns`
- `layers[].forbidden_tokens`

Scanした全file、各fileのexactly-one-layer分類、未分類・複数分類・forbidden tokenのerrorを返す。宣言したscan rootは必須であり、設定された対象fileが0件のscanはfailする。Project固有のdomain語彙は持たない。

C++の引用符付きliteral includeは、source相対、repository相対、scan対象内で一意なsuffixの順に参照先を解決する。解決済みのrepository相対pathにも既存のforbidden tokenを適用する。architecture observationも同じ解決処理を使う。compilerのinclude search pathやpreprocessor全体は再現しないため、macro include、候補が複数ある参照、scan対象外の参照はこの検査の保証範囲外である。

Patternはslash区切りで、1つのpath segment内の`*`と`?`をsupportする。完全な`**` segmentは0個以上のnested directoryにmatchするため、`source/**/*.ext`はdirect childから任意の深さまでを含む。Recursive exclusionも同じ意味論を使う。Character classなどshell固有のglob機能はportable contractに含めない。

Manifest errorはlint diagnosticとして返す。Rootとextensionは空でないstring list、layer nameは空でなく一意、patternとforbidden tokenはstring listでなければならない。暗黙のoptional-root behaviorは存在しない。

各projectは固有のmanifestとcomposition rootを持つ。Composition rootはproject設定をloadし、`lint_architecture`を呼び、その後にproject固有のdocument、semantic coverage、stable authorityの検査を追加する。

合成fixtureによるself-testは次で実行する。

```powershell
python tools/harness/test_architecture_lint.py
```

Testはtemporary projectを使い、valid classification、required root、empty scan、malformed manifest、unclassified/multiply classified source、forbidden token、nested pattern、recursive exclusion、multiple scan root、composition-root delegationを検査する。
