# Wire state and persistence

この文書はWireのauthoritative/derived境界、session draft、永続化、public viewの参照寿命を所有する。操作ごとの状態遷移は[`backbone_operation_semantics.md`](backbone_operation_semantics.md)、生成・再導出のstage境界は[`generation.md`](generation.md)を正本とする。

## Authority map

| 領域 | 決定者 | 責務 |
|---|---|---|
| topology | `SavedBackboneGraph` | node、edge、edge bundle、port/span binding、frontier |
| connectivity | `pairs make(graph)` | pair、open、row |
| placement | support group / row placement | row separation、vertical order、lowering offset |
| rules | `SpanLayoutRules` | span layout intent |
| layout | `SpanLayoutEntry` | `support_world` と `endpoint_world` |
| geom | `DetailCurve` / bounds | layout endpointからの形状派生 |
| draw | visual / render cache | layout/geomからの表示出力 |
| derived decoration | Core visual generation | support/span周辺の局所設備・短い配線・inline deviceをWire domainの意味から派生 |
| settings | `CoreStateAuthoritativeStorage` | geometry / visual / variation / context / layout のユーザー設定 |

生成済みのspan、layout、curve、bounds、visual、port位置からtopologyを復元してはいけない。同じ意味を複数段で再判断せず、下流は上流の決定済み値だけを消費する。ユーザーがUpdate APIで設定しderived出力に影響する値はauthoritativeに置き、runtime cacheにmirrorを持たない。

`Pole`は物理support entityでありtopology rootではない。support nodeはpole、ownerless point、external anchorを表せる。spanはport間の生成結果でありtopology authorityではない。

## Session draft

`ResolveBranchPick()`が作るpending support nodeは、次のdraw requestへpick結果を渡すためのsession draftである。authoritative topologyではなく、保存対象ではない。

pending support nodeの生存期間は次の通り。

- `ResolveBranchPick()`は必要な場合だけpending support nodeを作る。dry-runは作らない。
- `GenerateFromBackboneSpec()`がpending nodeを参照して成功した場合、そのpending nodeは消費済みとして削除する。
- draw pathのcancel / clearは`ClearPendingSupportNodes()`を呼び、未消費pendingを破棄する。
- save / loadはpendingを保持しない。load後に古いpending node idがrequestに残っていた場合、preflightはmutation前に`unknown node reference`として拒否する。

生成失敗時は本stateへcommitしないため、pendingの消費も行わない。retryやclearは同じsession draft契約に従う。

## Persistence

保存対象はidentityとauthoritative storageのみとし、runtime cacheとdebug storageは保存しない。loadは保存済みtopology / binding / settingsから既存pipelineを通してlayout、curve、bounds、visualを再導出する。

同一stateのsave -> loadではauthoritativeの再保存byteが一致し、派生出力の意味値と浮動小数bitが一致することをroundtrip等価性とする。runtime固有のversionや計測値、container addressは等価性に含めない。

永続形式はversionを完全一致で判定する。未知field、必須field欠落、truncation、重複field、構文不正は拒否し、部分的に読み飛ばさない。loadは新しいtrial stateでparse、index再構築、派生再導出、validationを完了してからmember-wise move commitする。どの段階で失敗しても、本stateは変更前と同一でなければならない。

### Save schema

正式v1前のarchive migrationは保証しない。現行`wire_state_v6`だけをexact-key schemaとして読み書きし、旧development schemaはunsupported versionとしてmutation前に拒否する。`save -> load -> save`は現行schemaのauthoritative byte一致を維持し、欠落fieldを位置、名前、container順から推測しない。

## Public view lifetime

`CoreState` / `CoreView`から取得したpointer、reference、viewは、同じ`CoreState`に対する次のnon-const operationまで有効である。non-const operationの成功・失敗を跨いだアドレス同一性は保証しない。長期保持が必要なconsumerは`ObjectId`を保持し、操作後に再取得する。

| API / result | 参照元 storage | 無効になり得る操作 | mutation 跨ぎ安定性 |
|---|---|---|---|
| `SpanLayoutView` / `SpanLayoutRulesView` | `SpanLayoutCache.records_by_span`の`unordered_map`内`optional` | record erase、layout/rule再保存、storage代入。insertのrehashはiteratorを無効化する | 保証しない |
| `CoreView`のPole / Port / Span / Bundle / Attachment pointer、`PoleDetailInfo` | `EditState`の`ObjectStore` | vectorのinsert/reallocation、eraseの末尾要素移動、storage代入 | 保証しない |
| backbone node / edge / binding pointerとCoreViewのmap/vector reference | `SavedBackboneGraph`、runtime index、debug vector/map | vector insert/erase、map erase/rehash、storage代入 | 保証しない |
| curve / bounds / visual / render cache pointer、visual curve cache reference | runtime cacheの`unordered_map` / vector | cache entry erase/再保存、vector更新、storage代入。rehashはiteratorを無効化する | 保証しない |
| inspection result内pointer (`PoleDetailInfo`等) | 上記viewが指すstorage | 上記と同じ | 保証しない |

現在public contractとしてmutationを跨ぐ参照安定性を保証するconsumerはない。既存testもvalue / id / `ChangeSet`を観測し、pointer addressをinvariantにしてはならない。
