# Wire検証

Wire検証はリポジトリ共通方針[`../testing.md`](../testing.md)に従う。この文書では、Wire固有のテスト群、fixture、網羅性の確認方法、診断用の入口を定義する。共通の検証方針は再定義しない。

## Test family

| Family | 目的 |
|---|---|
| Coreの既定テスト | public behavior、検証、幾何、状態service |
| backbone acceptance | generation、SavedGraph、binding、layout/geometry/draw、update境界 |
| viewerのテスト | 入力policy、座標変換、selection、代表的scene出力 |
| architecture lint | ソースの層分類と禁止dependency |
| テスト群lint | 登録済みテストsourceのownership |

Backbone acceptance filterは`backbone`であり、旧`bb2` aliasは存在しない。

```powershell
build-vs18-coretests\domains\wire\Debug\wire_core_tests.exe backbone
```

## Fixtureとfamily manifest

Backbone fixtureは`domains/wire/tests/backbone/fixtures.*`に置く。Testはv1 topology API、既存span幾何、位置の近接からauthoritative入力を構築しない。代表sceneでは最終出力だけでなくtopology、connectivity、placementのdecision担当箇所も検証する。

`domains/wire/tests/test_family_manifest.json`は登録済みテストsourceをちょうど1つのfamilyへ割り当てる。未分類または複数分類はlint失敗になる。C番号は過去からの識別子であり、進捗指標ではない。

## 失敗診断

新しいテストの主要preconditionと不変条件には`WIRE_TEST_EXPECT(condition, reason)`を使う。既存テストを変更した場合、その関数内の重要な操作境界を失敗理由付き検証条件へ移行する。Suite全体の機械的検証条件置換を進捗にしない。

## 操作×状態coverage

操作×状態coverageは、要求された操作意味論のセルへ実際に到達したことを証明する。共通不変条件はcoverageメタデータへ埋め込まず、安定した各観測点の状態の正しさを証明する。

- `Observe`、`ObserveEmpty`、`ObserveMidspan`は`(cell, entry)`を記録する前にrow-frame coherenceを検査する。
- 各`Observe`が返すtokenへ、その操作実行後の`oracle`、`anchor`、`presence`、`differential`を結び付ける。テストケース内の別cellに対する検証条件、fixture構築成功、Observe自体はそのcellのevidenceにしない。
- 正本表から生成した期待動作も実行時へ渡す。`VerifyOutcome`は観測時と操作後の継続関係をnode、edge bundle、laneで比較し、既存関係の保存と成功時の`C/O/K`の一致を検査する。証拠を持っていても動作を照合していない観測は未検証として扱う。
- Coreはすべてのrequired cellを`core_api`経由で実行する。
- WASMとviewerのテストは`backbone_operation_semantics.md`の`入口境界`表を読み、実payloadを実行する。
- `derived_equality`だけを記録するケースは独立した証拠ではない。Full coreテスト出力は該当ケースを列挙する。

この照合が守るのは接続の遷移である。`K`の派生更新、失敗時の正本・派生・セッションの一時データ不変、対象外への非干渉、Port/Span識別情報、保存と再導出の等価性は、該当する既存の契約テストが検証する。セル到達や共通不変条件の成功から、これらの操作別契約まで検証済みとは扱わない。

調査で確認した対応と故障注入結果は[`../../domains/wire/tests/audits/2026-11.md`](../../domains/wire/tests/audits/2026-11.md)に記録する。

### 正本backbone acceptance

成功した`CoreState`のgeneration、update、regenerate、loadは、安定した各観測点を次の順で検査する。

```text
canonical successful backbone scenario
  -> WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state)
  -> scenario-specific oracle
```

`WIRE_TEST_EXPECT_BACKBONE_INVARIANTS`は`backbone_common_invariants_pass()`への通常のテスト入口である。Scene全体の最低条件をAnchorとして記録する。形状、識別情報、差分、個数、方向の検証条件は、各シナリオに独立した期待値の根拠として残す。

代表的な成功シナリオにはstraight、corner、branch、cross、multi-lane、incremental、regenerate、save/load、model/socket、midair、production-like構成がある。観測対象は、処理が安定した最終状態とする。

全ての中間状態を対象にはしない。Invalid入力、rejection、SourceGuard、pure unit、parser/template-only、意図的な中間状態へ共通不変条件を機械的に適用しない。

現行の共通不変条件はauthoritative reference、layout/endpoint、row/frame、model/cache、visual幾何、connection、high-voltage crossingを覆う。具体的な多段高さと曲線形状は、各シナリオの独立した期待値で検証する。

## Wire architecture検査

`tools/arch_lint.py`と`tools/arch_manifest.json`はWire source分類、非公開実装への依存禁止、retired source family、ドメイン識別情報漏洩、操作×状態/BOS coverage mapping、authority guard担当箇所を検査する。

安定したsource検査はStructural evidenceであり、behaviorテストを代替しない。

## Portable catalogの実例

共通behavioral probe catalogの例として、incremental-vs-full differentialは`C741_scoped_visual_curve_rebuild_matches_full_rebuild`、load失敗時の原子性は`C754_authoritative_load_rejects_invalid_text_without_mutation`、独立component orderingは`C780_backbone_incremental_duplicate_values_are_order_independent_by_placement_key`を参照できる。

これらは実例であり、共通方針の再定義ではない。

## Legacy test

Legacyテストの期待値をBackbone acceptanceの正本にしない。契約を抽出し、現行authority/derived出力に対して維持、書き直し、実装familyとともに退役、現行designと競合するため削除、のいずれかに分類する。Unsupported-operationテストでは状態不変も検証する。

Post-editテストでは実際のderived出力更新、または状態変更前のrejectionを検証する。
