# Wire検証

Wire検証はリポジトリ共通方針[`../testing.md`](../testing.md)に従う。この文書では、Wire固有のテスト群、fixture、網羅性の確認方法、診断用の入口を定義する。共通の検証方針は再定義しない。

## Test family

| Family | 目的 |
|---|---|
| Coreの既定テスト | 公開APIの動作、検証、幾何、状態を扱うサービス |
| backbone acceptance | generation、SavedGraph、binding、layout/geometry/draw、update境界 |
| viewerのテスト | 入力方針、座標変換、選択操作、代表的なsceneの出力 |
| arch lint | sourceの層分類と禁止された依存関係 |
| テスト群lint | 登録済みのテストsourceと担当familyの対応 |

Backbone acceptanceの実行時は、filterに`backbone`を指定する。旧aliasの`bb2`は存在しない。

```powershell
build-vs18-coretests\domains\wire\Debug\wire_core_tests.exe backbone
```

## Fixtureとfamily manifest

Backbone fixtureは`domains/wire/tests/backbone/fixtures.*`に置く。テストではv1 topology API、既存spanの幾何、位置の近さから正本となる入力を構築しない。代表的なsceneでは、最終出力に加えてtopology、connectivity、placementのdecision ownerも検証する。

`domains/wire/tests/test_family_manifest.json`では、登録済みのテストsourceをそれぞれ1つのfamilyへ割り当てる。未分類または複数のfamilyへの割当はlintで失敗とする。C番号は過去からの識別子であり、進捗指標ではない。

## 失敗診断

新しいテストの主要な前提条件と不変条件には`WIRE_TEST_EXPECT(condition, reason)`を使う。既存テストを変更した場合は、その関数内の重要な操作境界で失敗理由付きのassertionを使う。テスト群全体のassertionを機械的に置き換える作業は進捗にしない。

## 操作×状態coverage

操作×状態のcoverageでは、要求された操作意味論のセルへ実際に到達したことを確認する。共通不変条件はcoverageのメタデータへ埋め込まず、安定した各観測点で状態の正しさを検証する。

- `Observe`、`ObserveEmpty`、`ObserveMidspan`は`(cell, entry)`を記録する前にrow-frame coherenceを検査する。
- 各`Observe`が返すtokenへ、その操作実行後の`oracle`、`anchor`、`presence`、`differential`を結び付ける。テストケース内の別cellに対するassertion、fixture構築成功、Observe自体はそのcellのevidenceにしない。
- 正本表から生成した期待動作も実行時へ渡す。`VerifyOutcome`は観測時と操作後の継続関係をnode、edge bundle、laneで比較し、既存関係の保存と成功時の`C/O/K`の一致を検査する。証拠を持っていても動作を照合していない観測は未検証として扱う。
- Coreはすべてのrequired cellを`core_api`経由で実行する。
- WASMとviewerのテストは`backbone_operation_semantics.md`の入口境界表を読み、実際のpayloadを使って操作を実行する。
- `derived_equality`だけを記録するケースは独立した証拠ではない。Coreテストの全件実行では、該当ケースを出力する。

この照合が守るのは接続の遷移である。`K`の派生更新、失敗時の正本・派生・セッションの一時データ不変、対象外への非干渉、Port/Span識別情報、保存と再導出の等価性は、該当する既存の契約テストが検証する。セル到達や共通不変条件の成功から、これらの操作別契約まで検証済みとは扱わない。

調査で確認した対応と故障注入結果は[`../../domains/wire/tests/audits/2026-11.md`](../../domains/wire/tests/audits/2026-11.md)に記録する。

### 正本に対するbackbone acceptance

成功した`CoreState`のgeneration、update、regenerate、loadは、安定した各観測点を次の順で検査する。

```text
canonical successful backbone scenario
  -> WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state)
  -> scenario-specific oracle
```

`WIRE_TEST_EXPECT_BACKBONE_INVARIANTS`は`backbone_common_invariants_pass()`への通常のテスト入口である。Scene全体の最低条件をAnchorとして記録する。形状、識別情報、差分、個数、方向に関するassertionは、各シナリオの独立したoracleとして残す。

代表的な成功シナリオにはstraight、corner、branch、cross、multi-lane、incremental、regenerate、save/load、model/socket、midair、production-like構成がある。観測対象は、処理が安定した最終状態とする。

全ての中間状態を対象にはしない。不正入力や拒否経路のテスト、SourceGuard、純粋な単体テスト、parserやテンプレートだけのテスト、意図的な中間状態へ共通不変条件を機械的に適用しない。

現行の共通不変条件はauthoritative reference、layout/endpoint、row/frame、model/cache、visual幾何、connection、high-voltage crossingを覆う。具体的な多段高さと曲線形状は、各シナリオの独立した期待値で検証する。

## Wireのarch lint

`tools/arch_lint.py`と`tools/arch_manifest.json`では、Wireのsource分類、非公開実装への禁止された依存、廃止済みsource familyの残存、Wire Coreへの他ドメインの型・名前の混入、操作×状態とBOS coverageの対応、authority guardのownerを検査する。

安定したsourceの検査はStructural evidenceとして扱い、動作テストの代替にはしない。

## 移植できる実例

共通のbehavioral probeの実例として、差分再構築と全量再構築の比較は`C741_scoped_visual_curve_rebuild_matches_full_rebuild`、load失敗時の原子性は`C754_authoritative_load_rejects_invalid_text_without_mutation`、独立した構成要素の処理順に関する検証は`C780_backbone_incremental_duplicate_values_are_order_independent_by_placement_key`を参照できる。

これらは実例であり、共通方針の再定義ではない。

## 旧テスト

旧テストの期待値をBackbone acceptanceの正本にしない。契約を抽出し、現行の正本と派生出力に対して維持する、書き直す、実装familyとともに廃止する、現行設計と競合するため削除する、のいずれかに分類する。未対応操作のテストでは、状態が変わらないことも検証する。

編集後のテストでは、派生出力が実際に更新されること、または状態を変える前に操作が拒否されることを検証する。
