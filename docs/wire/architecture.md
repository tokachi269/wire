# Wire architecture

Wireドメインはsupport node間の電線のtopologyを生成・編集し、保存済みの識別情報とbindingからlayout、幾何、表示用の出力を導出する。移行中に`bb2`と呼んでいた実装も、製品コードと現行テストでは`backbone`を正とする。リポジトリ共通契約は[`../architecture.md`](../architecture.md)を参照する。

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

`SavedBackboneGraph`はWireの接続構造を表す。city全体の接続構造は扱わない。`Pole`は物理的なsupport entityであり、接続構造の起点としては使わない。support nodeはpole、ownerless point、external anchorを表せる。spanはport間の生成結果として扱い、接続構造の正本にはしない。

## Architecture views

詳細な契約は次の文書で定義する。この入口文書へ同じ内容を複写しない。

| View | 正本 |
|---|---|
| 正本と派生状態、sessionの一時データ、永続化、公開viewの参照寿命 | [`state_and_persistence.md`](state_and_persistence.md) |
| generation pipeline、配置、識別情報の対応、生成と再構築、検証境界 | [`generation.md`](generation.md) |
| 操作×状態、接続・表現、post-edit lifecycle | [`backbone_operation_semantics.md`](backbone_operation_semantics.md) |
| 対応する操作と失敗時の責務 | [`supported_operations.md`](supported_operations.md) |
| model descriptor、model assembly、socket、座標規約、asset adapterの境界 | [`models.md`](models.md) |
| cable curve、visual parts、span visual assembly、render materialization | [`cable_visual.md`](cable_visual.md) |
| Wire固有のテスト群、fixture、coverage、設計上の検査 | [`testing.md`](testing.md) |

## Essential architecture contracts

- Topologyのauthorityは`SavedBackboneGraph`である。生成済みspan、layout、curve、bounds、visual、port位置からtopologyを復元しない。詳細の正本は[`state_and_persistence.md`](state_and_persistence.md)とする。
- Pair/open/row、placement、幾何、visualの各段階では、前段で決定した出力を使う。同じ意味をpipeline、materialization、viewerで再判断しない。段階境界は[`generation.md`](generation.md)を正本とする。
- 操作前状態と操作の組合せは[`backbone_operation_semantics.md`](backbone_operation_semantics.md)を正本とし、未定義セルを実装やテストから推測して埋めない。
- 通常生成とpost-edit regenerateは同じpipelineを使い、全段階成功時だけ本状態へcommitする。preflightで検出できない後半失敗でも原子性を失わない。詳細の正本は[`generation.md`](generation.md)とする。
- 保存対象は識別情報とauthoritative storageだけである。実行時やデバッグ用のキャッシュは保存せず、loadで既存pipelineから再導出する。スキーマとroundtrip契約の正本は[`state_and_persistence.md`](state_and_persistence.md)とする。
- `BundleTemplateId`がbundleテンプレート識別情報であり、`BundleKind`はcategory/tagである。model/socket/asset境界を含む完全な契約の正本は[`models.md`](models.md)とする。
- viewer / exporterはCoreのderived出力を消費し、topology、pair、row、lowering、connection、material意味論を補完しない。モデルは[`models.md`](models.md)、deformable cableは[`cable_visual.md`](cable_visual.md)を正本とする。

## Domain boundary

Wire Coreはroad、rail、building、terrain、cityのドメイン型を参照しない。外部システムは入力をworld position、wire template/profile、内部構造を解釈しないexternal anchor tokenへ解決してからCoreを呼ぶ。

## Structural guards

- 公開API: `domains/wire/include/city/wire`
- private generation: `domains/wire/src/generation/backbone`
- 状態ownership: `CoreState`
- read-only query: `CoreView`とconst query
- dependency and layer guard: `tools/arch_manifest.json` / `tools/arch_lint.py`
- テストownership guard: `tools/test_family_lint.py`

architecture observationのDSMやdeltaは、レビューの判断材料として使う。上記guardを再実装するものではない。実行方法は[`../command_cheatsheet.md`](../command_cheatsheet.md)を参照する。
