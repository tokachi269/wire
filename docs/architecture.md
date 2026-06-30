# アーキテクチャ

このドキュメントは、現在の backbone 生成本流の契約をまとめる。
現在の実装は移行中に `bb2` と呼んでいたが、production code と現行testは `backbone` を正とする。

## 全体構造

```text
BackboneSpec
  -> backbone generation
  -> SavedBackboneGraph
  -> pair / open / row
  -> SpanLayoutRules
  -> support group / SpanLayoutEntry
  -> DetailCurve / bounds
  -> visual / render cache
  -> viewer / export adapter
```

## 正本と派生

| 領域 | 決定者 | 責務 |
|---|---|---|
| topology | `SavedBackboneGraph` | node、edge、edge bundle、port/span binding、frontier |
| connectivity | `pairs make(graph)` | pair、open、row |
| placement | support group / row placement | row separation、vertical order、lowering offset |
| rules | `SpanLayoutRules` | span layout intent |
| layout | `SpanLayoutEntry` | `support_world` と `endpoint_world` |
| geom | `DetailCurve` / bounds | layout endpointからの形状派生 |
| draw | visual / render cache | layout/geomからの表示出力 |

生成済みのspan、layout、curve、bounds、visual、port位置からtopologyを復元してはいけない。
同じ意味を複数段で再判断せず、下流は上流の決定済み値だけを消費する。

## backbone generation

`GenerateFromBackboneSpec()` は `core/src/generation/backbone` のpipelineだけを呼ぶ。
未対応入力はv1へfallbackせず、mutation前に`unsupported`を返す。

処理順は次の通り。

1. inputの`prepare`と`check`
2. graphからpair/open/rowを確定
3. duplicate edge bundle/span bindingをpreflight
4. intentとsupport groupを確定
5. pole、bundle、port、spanを生成
6. `SavedBackboneGraph`とbindingを保存
7. rules、layout、geom、drawを保存

context linkは判断入力であり、生成・保存対象ではない。
T/cross/branchのkind enumは作らず、pair/open/rowの組合せで表す。

## post-edit update

更新分類は次の4種類だけとする。

| `UpdateKind` | 変更範囲 | 再導出 |
|---|---|---|
| `kRegenerate` | topology / identity / connectivity | generation。安全にできなければ`unsupported` |
| `kReposition` | support/endpoint位置 | layout -> geom -> draw |
| `kReshape` | curve/bounds/shape | geom -> draw |
| `kRedraw` | visual/render | drawのみ |

操作名ごとのdirty enumは追加しない。
post-edit APIは、派生出力を更新して成功するか、mutation前に拒否する。
staleなlayout/geom/drawを残したまま成功してはいけない。

`UpdatePoleTypeDefinition`は、対象typeをactive backbone poleが使用中ならmutation前に拒否する。
非backbone poleだけへのdefinition再適用は許可する。active objectのtemplate migrationは現行scopeに含めない。

## validationとinspection

`ValidateFast()`、`Validate()`、inspectionはstateを観測して問題を報告する。
不足情報の補完、topologyの推測、state mutationは行わない。

## viewerとrender/export

viewerは`SavedBackboneGraph`、rules、layout、geom、visual/render cacheを読むconsumerである。
viewerが不足したtopology、pair、row、loweringを推測または補正してはいけない。

coreはbackend非依存のcurve、bounds、primitive、style参照を出力する。
UE、Blender、viewer、exporter固有のasset/material型はadapter側で解決する。

## wire domain境界

wire coreはroad、rail、building、terrain、cityのdomain型を知らない。
外部systemはworld position、wire template/profile、opaqueなexternal anchor tokenへ解決してからcoreを呼ぶ。
`SavedBackboneGraph`はwire topologyであり、city topologyではない。

## Poleの位置づけ

`Pole`は物理support entityであり、topology rootではない。
support nodeはpole、ownerless point、external anchorを表せる。
spanはport間の生成結果であり、topology authorityではない。

## 境界guard

- public API: `core/include/wire/core`
- private generation: `core/src/generation/backbone`
- state ownership: `CoreState`
- read-only query: `CoreView`とconst query
- dependency guard: `tools/arch_lint.py`
- test ownership guard: `tools/test_family_lint.py`
