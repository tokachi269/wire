# テスト体系

## test family

| family | 目的 |
|---|---|
| default core tests | public behavior、validation、geometry、state serviceを含む通常回帰 |
| backbone acceptance | generation、SavedGraph、binding、layout/geom/draw、update境界 |
| viewer tests | input policy、座標変換、selection、代表sceneの表示出力 |
| architecture lint | source layer分類と禁止依存 |
| test family lint | 登録testのowner分類 |

backbone acceptanceのfilterは`backbone`である。旧`bb2` filter aliasは持たない。

```powershell
build-vs18-coretests\domains\wire\Debug\wire_core_tests.exe backbone
```

## fixture

backbone fixtureは`domains/wire/tests/backbone/fixtures.*`に置く。
testはv1 topology API、existing span geometry、position proximityから入力正本を組み立てない。
代表sceneは結果だけでなく、topology/connectivity/placementのownerも検証する。

## test family manifest

`domains/wire/tests/test_family_manifest.json`はtest sourceのownerをfamily単位で管理する。
新しい登録test sourceが未分類、または複数familyへ分類された場合はlintを失敗させる。
C番号は履歴識別子として維持するが、C番号の増加を進捗指標にしない。

## failure diagnostics

新規testは `WIRE_TEST_EXPECT(condition, reason)` で主要な前提と不変条件を検査する。
既存testを触る場合は、その関数内の入口条件や複数手順の主要境界から理由付きへ移行する。
全既存testの機械的一括移行は進捗ではない。複数手順のfailは、最初に壊れた操作名や期待した不変条件を
reasonに残す。

## 操作×状態coverageと不変条件

操作×状態coverageは、意味論表のrequired cellへ実際に到達したことを保証する。
内部整合の正しさはcoverageへ詰め込まず、各観測点で共通不変条件を実行して保証する。

- wire: `Observe` / `ObserveEmpty` / `ObserveMidspan`がrow frame coherenceを検査してから
  `(cell, entry)`を記録する
- core entry: 全required cellを`core_api`で実行する
- WASM / viewer entry: `docs/wire/backbone_operation_semantics.md`の入口境界表を
  web testが読み、実WASM stateと`ViewerActions` payloadで必須cellを実行する
- road: productionのedit/load境界とtestの代表観測点、seed付き操作列で、同じ
  `ValidateGraphInvariants`を使用する。test専用の別invariantは作らない

`derived_equality`だけのcaseは独立evidenceにならない。full core testは該当case一覧を
終了時に出力し、件数を`docs/merge_readiness.md`へ記録する。

### canonical backbone acceptance

正常な`CoreState`を生成、更新、regenerate、loadしたbackbone acceptanceは、安定した観測点ごとに
次の順で検査する。

```text
canonical successful backbone scenario
    -> WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state)
    -> scenario-specific oracle
```

`WIRE_TEST_EXPECT_BACKBONE_INVARIANTS`は`backbone_common_invariants_pass()`を呼ぶ唯一の通常test入口で、
scene全体の最低成立条件を`Anchor`として記録する。個別scenarioが守る形状、identity、差分、件数、
方向等は`Oracle`または`Differential`等として後段に残し、common invariantへ吸収しない。

対象はstraight、corner、branch、cross、multi-lane、incremental、regenerate、save/load、model/socket、
midair、production-like configuration等の代表的な成功scenarioである。新しいcanonical successful
scenarioも原則この入口を通す。複数操作を持つtestは、initial generation後、edit後、load後等のうち、
仕様上安定した最終stateだけを観測する。

invalid input/reject、SourceGuard、pure unit、parser/template単体、intentional intermediate stateには
一律適用しない。全caseへの機械的追加やmacro文字列のgrep件数をcoverage根拠にしない。
適用漏れが実際に再発するまでは専用lintやclassification hierarchyを追加しない。

現在の`backbone_common_invariants_pass()`がauthoritative reference、layout/endpoint、row/frame、
model/cache、visual geometry、connection、HV crossingまで含むため、別の
`WIRE_TEST_EXPECT_SCENE_BASELINE`は設けない。共通でないmulti-levelの具体高さや曲線形状等は、
引き続きscenario固有oracleが所有する。

## test effectiveness

test count、line coverage、branch coverageだけでは、test suiteの価値は評価できない。
coverageは「そのコードを通った」ことは示せるが、「そのコードが壊れたとき検出できる」
ことまでは保証しない。重要なのは、重要な故障を実際に入れたとき、そのtest familyが
意味のある場所で失敗できることである。

通常のmutation testingは、return値変更、比較演算子変更、条件反転のようなsyntactic
mutationでもpassしてしまう0点testの発見に有効である。一方、実際のregressionは、
connection visualを丸ごと生成しない、incrementalだけ古い状態を使う、owner transformを
modelへ伝播しない、production bootstrap時だけ意味が変わる、connectivityをsilent skipする、
といったsemanticな壊れ方をする。test effectiveness評価ではsyntactic mutationだけでなく、
現在codeへ小さなtemporary semantic faultを入れて、既存test familyが検出できるかを確認する。
fault injection自体はcommitしない。

評価単位は個別testではなく契約である。例えば「continuous connectionはvisual connectionを持つ」
という契約は、fresh、continuation、branch、midair、reverse、regenerate、save/load、
production configurationのどこから守られているかを見る。C番号やtest件数を増やすことを目的にせず、
重要契約をsuite全体がどの程度守れているかを評価する。

過去に実際に起きた不具合は、最も価値の高いfault modelの一つである。実際の過去commitを
そのままbuildする必要はない。現在codeへ過去故障と意味的に同じtemporary mutationを入れ、
現在のtest suiteが検出できるか確認してよい。安全に再現可能なら古いcommitを使ってもよい。

全test pass後に実使用で見つかったregressionは、test effectiveness不足の証拠として扱う。
bug修正時は、新しいregression testを追加しただけで完了にしない。なぜ既存testが逃したかを、
common invariant不足、production fidelity不足、scenario不足、semantic assertion不足、
condition complexity、実装詳細だけを見たassert、特定fixtureへ閉じた過去fixのいずれかとして
分類し、必要なら既存common invariantやproduction-like sanityへ戻す。

## architecture guard

`tools/arch_manifest.json`と`tools/arch_lint.py`は次を検出する。

- 未分類source/header
- viewerからcore private headerへの依存
- geometry/validationからgeneration privateへの逆依存
- recalc/support-layout familyの復活
- domains/road/rail/building/city domain identityのcore流入
- `docs/wire/backbone_operation_semantics.md` の操作×状態表と
  `domains/wire/tests/spec_ledger.md` の `BOS:<operation>:<state>` coverage 対応漏れ

source scanは安定した境界だけに使う。広い単語grepをtest semanticsの代わりにしない。

## 旧test

旧testの期待値をそのままbackboneの合格基準にしない。
守っていた制約を抽出し、次のいずれかに分類する。

- そのまま維持
- backboneの正本/派生出力で書き換え
- v1実装詳細としてfamily退役
- 現設計と衝突するため削除

unsupported testはstate unchangedまで確認する。
post-edit testはmarkerではなく、実際のlayout/geom/draw更新またはmutation前rejectを確認する。
