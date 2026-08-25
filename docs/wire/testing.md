# Wire検証

Wire検証はrepository共通方針[`../testing.md`](../testing.md)に従う。この文書が所有するのはWire固有のtest family、fixture、coverage機構、診断entryだけであり、共通検証方針は再定義しない。

## Test family

| Family | 目的 |
|---|---|
| default core tests | public behavior、validation、geometry、state service |
| backbone acceptance | generation、SavedGraph、binding、layout/geometry/draw、update境界 |
| viewer tests | input policy、座標変換、selection、代表的scene output |
| architecture lint | source layer分類と禁止dependency |
| test family lint | 登録済みtest sourceのownership |

Backbone acceptance filterは`backbone`であり、旧`bb2` aliasは存在しない。

```powershell
build-vs18-coretests\domains\wire\Debug\wire_core_tests.exe backbone
```

## Fixtureとfamily manifest

Backbone fixtureは`domains/wire/tests/backbone/fixtures.*`に置く。Testはv1 topology API、既存span geometry、位置の近接からauthoritative inputを構築しない。代表sceneでは最終outputだけでなくtopology、connectivity、placementのdecision ownerも検証する。

`domains/wire/tests/test_family_manifest.json`は登録済みtest sourceをちょうど1つのfamilyへ割り当てる。未分類または複数分類はlint failureになる。C番号は過去からの識別子であり、進捗指標ではない。

## 失敗診断

新testの主要preconditionとinvariantには`WIRE_TEST_EXPECT(condition, reason)`を使う。既存testを変更した場合、その関数内の重要なoperation境界を失敗理由付きassertionへ移行する。Suite全体の機械的assertion置換を進捗にしない。

## 操作×状態coverage

操作×状態coverageは、要求されたsemantics cellへ実際に到達したことを証明する。共通invariantはcoverage metadataへ埋め込まず、安定した各観測点のstate correctnessを証明する。

- `Observe`、`ObserveEmpty`、`ObserveMidspan`は`(cell, entry)`を記録する前にrow-frame coherenceを検査する。
- 各`Observe`が返すtokenへ、そのoperation実行後の`oracle`、`anchor`、`presence`、`differential`を結び付ける。case内の別cellに対するassertion、fixture構築成功、Observe自体はそのcellのevidenceにしない。
- Coreはすべてのrequired cellを`core_api`経由で実行する。
- WASMとviewer testは`backbone_operation_semantics.md`の`入口境界`表を読み、実payloadを実行する。
- `derived_equality`だけを記録するcaseは独立evidenceではない。Full core test outputは該当caseを列挙する。

### 正本backbone acceptance

成功した`CoreState`のgeneration、update、regenerate、loadは、安定した各観測点を次の順で検査する。

```text
canonical successful backbone scenario
  -> WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state)
  -> scenario-specific oracle
```

`WIRE_TEST_EXPECT_BACKBONE_INVARIANTS`は`backbone_common_invariants_pass()`への通常のtest entryである。Scene全体の最低条件をAnchorとして記録する。Shape、identity、differential、count、direction assertionは独立oracleとして各scenarioに残す。

代表的な成功scenarioにはstraight、corner、branch、cross、multi-lane、incremental、regenerate、save/load、model/socket、midair、production-like構成がある。意図した中間stateすべてではなく、安定したfinal stateを観測する。Invalid input、rejection、SourceGuard、pure unit、parser/template-only、意図的な中間stateへ共通invariantを機械的に適用しない。

現行の共通invariantはauthoritative reference、layout/endpoint、row/frame、model/cache、visual geometry、connection、high-voltage crossingを覆う。具体的なmulti-level heightとcurve shapeはscenario固有oracleが所有する。

## Wire architecture検査

`tools/arch_lint.py`と`tools/arch_manifest.json`はWire source分類、private dependency禁止、retired source family、domain identity漏洩、操作×状態/BOS coverage mapping、authority guard ownerを検査する。安定したsource検査はStructural evidenceであり、behavior testを代替しない。

## Portable catalogの実例

共通behavioral probe catalogの例として、incremental-vs-full differentialは`C741_scoped_visual_curve_rebuild_matches_full_rebuild`、failed-load atomicityは`C754_authoritative_load_rejects_invalid_text_without_mutation`、独立component orderingは`C780_backbone_incremental_duplicate_values_are_order_independent_by_placement_key`を参照できる。これらは実例であり、共通方針の再定義ではない。

## Legacy test

Legacy testの期待値をBackbone acceptanceの正本にしない。契約を抽出し、現行authority/derived outputに対して維持、書き直し、実装familyとともに退役、現行designと競合するため削除、のいずれかに分類する。Unsupported-operation testではstate不変も検証する。Post-edit testでは実際のderived output更新、またはmutation前のrejectionを検証する。
