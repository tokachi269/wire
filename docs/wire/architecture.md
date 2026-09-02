# Wire architecture

Wire domainはsupport node間のwire topologyを生成・編集し、保存済みidentityとbindingからlayout、geometry、visual outputを派生する。移行中に`bb2`と呼んでいた実装も、production codeと現行testでは`backbone`を正とする。repository共通contractは[`../architecture.md`](../architecture.md)を参照する。

## High-level structure

```text
BackboneSpec
  -> backbone generation
  -> SavedBackboneGraph
  -> pair / open / row
  -> SpanLayoutRules
  -> support group / SpanLayoutEntry
  -> DetailCurve / bounds
  -> visual / render cache
  -> derived decoration materialization
  -> viewer / export adapter
```

`SavedBackboneGraph`はwire topologyでありcity topologyではない。`Pole`は物理support entityであってtopology rootではなく、support nodeはpole、ownerless point、external anchorを表せる。spanはport間の生成結果でありtopology authorityではない。

## Architecture views

詳細contractのownerは次の通り。入口文書へ同じ完全なcontractを複写しない。

| View | 正本 |
|---|---|
| authoritative/derived state、session draft、persistence、public view lifetime | [`state_and_persistence.md`](state_and_persistence.md) |
| generation pipeline、placement、identity mapping、build/rebuild、validation boundary | [`generation.md`](generation.md) |
| 操作×状態、接続・表現、post-edit lifecycle | [`backbone_operation_semantics.md`](backbone_operation_semantics.md) |
| supported interactionとfailure ownership | [`supported_operations.md`](supported_operations.md) |
| model descriptor、assembly、socket、座標規約、asset adapter境界 | [`models.md`](models.md) |
| cable curve、visual parts、span visual assembly、render materialization | [`cable_visual.md`](cable_visual.md) |
| Wire固有test family、fixture、coverage、architecture inspection | [`testing.md`](testing.md) |

## Essential architecture contracts

- Topologyのauthorityは`SavedBackboneGraph`である。生成済みspan、layout、curve、bounds、visual、port位置からtopologyを復元しない。詳細ownerは[`state_and_persistence.md`](state_and_persistence.md)とする。
- Pair/open/row、placement、geometry、visualは各stageの決定済み出力を下流が消費する。同じ意味をpipeline、materialization、viewerで再判断しない。stage境界は[`generation.md`](generation.md)を正本とする。
- 操作前stateと操作の組合せは[`backbone_operation_semantics.md`](backbone_operation_semantics.md)を正本とし、未定義セルを実装やtestから推測して埋めない。
- 通常生成とpost-edit regenerateは同じpipelineを使い、全stage成功時だけ本stateへcommitする。preflightで検出できない後半failureでもatomicityを失わない。詳細は[`generation.md`](generation.md)が所有する。
- 保存対象はidentityとauthoritative storageだけである。runtime/debug cacheは保存せず、loadで既存pipelineから再導出する。schemaとroundtrip contractは[`state_and_persistence.md`](state_and_persistence.md)が所有する。
- `BundleTemplateId`がbundle template identityであり、`BundleKind`はcategory/tagである。model/socket/asset境界を含む完全なcontractは[`models.md`](models.md)が所有する。
- viewer / exporterはCoreのderived outputを消費し、topology、pair、row、lowering、connection、material semanticsを補完しない。modelは[`models.md`](models.md)、deformable cableは[`cable_visual.md`](cable_visual.md)を正本とする。

## Domain boundary

wire coreはroad、rail、building、terrain、cityのdomain型を知らない。外部systemはworld position、wire template/profile、opaqueなexternal anchor tokenへ解決してからCoreを呼ぶ。

## Structural guards

- public API: `domains/wire/include/city/wire`
- private generation: `domains/wire/src/generation/backbone`
- state ownership: `CoreState`
- read-only query: `CoreView`とconst query
- dependency and layer guard: `tools/arch_manifest.json` / `tools/arch_lint.py`
- test ownership guard: `tools/test_family_lint.py`

architecture observationのDSMやdeltaはreview evidenceであり、上記guardの再実装ではない。実行方法は[`../command_cheatsheet.md`](../command_cheatsheet.md)を参照する。
