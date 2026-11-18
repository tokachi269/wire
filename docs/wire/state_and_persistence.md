# Wire state and persistence

この文書では、Wireの正本と派生の境界、セッションの一時データ、永続化、公開viewの参照寿命を定義する。操作ごとの状態遷移は[`backbone_operation_semantics.md`](backbone_operation_semantics.md)、生成・再導出の段階境界は[`generation.md`](generation.md)を正本とする。

## Authority map

| 領域 | 決定者 | 責務 |
|---|---|---|
| topology | `SavedBackboneGraph` | node、edge、edge bundle、port/span binding、frontier |
| connectivity | `pairs make(graph)` | pair、open、row |
| placement | support group / row placement | row separation、vertical order、lowering offset |
| rules | `SpanLayoutRules` | span layout intent |
| layout | `SpanLayoutEntry` | `support_world`と`endpoint_world` |
| geom | `DetailCurve` / bounds | layout endpointからの形状派生 |
| draw | visual / render cache | layout/geomからの表示出力 |
| derived decoration | Core visual generation | support/span周辺の局所設備・短い配線・inline deviceをWireドメインの意味から派生 |
| settings | `CoreStateAuthoritativeStorage` | 幾何 / visual / variation / context / layoutのユーザー設定 |

生成済みのspan、layout、curve、bounds、visual、port位置からtopologyを復元してはいけない。同じ意味を複数段で再判断せず、下流は上流の決定済み値だけを消費する。ユーザーがUpdate APIで設定しderived出力に影響する値はauthoritativeに置き、実行時キャッシュにmirrorを持たない。

`Pole`は物理support entityでありtopology rootではない。support nodeはpole、ownerless point、external anchorを表せる。spanはport間の生成結果でありtopology authorityではない。

## Session draft

`ResolveBranchPick()`が作るpending support nodeは、次のdraw requestへpick結果を渡すためのセッションの一時データである。正本のtopologyには含めず、保存もしない。

pending support nodeの生存期間は次の通り。

- `ResolveBranchPick()`は必要な場合だけpending support nodeを作る。dry-runは作らない。
- `GenerateFromBackboneSpec()`がpending nodeを参照して成功した場合、そのpending nodeは消費済みとして削除する。
- draw pathのcancel / clearは`ClearPendingSupportNodes()`を呼び、未消費pendingを破棄する。
- save / loadはpendingを保持しない。load後に古いpending node idがrequestに残っていた場合、preflightは状態変更前に`unknown node reference`として拒否する。

生成失敗時は本状態へcommitしないため、pendingの消費も行わない。retryやclearは同じセッションの一時データ契約に従う。

## Persistence

保存対象は識別情報とauthoritative storageのみとし、実行時キャッシュとdebug storageは保存しない。loadは保存済みtopology / binding / settingsから既存pipelineを通してlayout、curve、bounds、visualを再導出する。

同一状態のsave -> loadではauthoritativeの再保存byteが一致し、派生出力の意味値と浮動小数bitが一致することをroundtrip等価性とする。実行時固有のバージョンや計測値、container addressは等価性に含めない。

永続形式はバージョンを完全一致で判定する。未知フィールド、必須フィールド欠落、truncation、重複フィールド、構文不正は拒否し、部分的に読み飛ばさない。loadは新しい試行用の状態でparse、index再構築、派生再導出、検証を完了してからmember-wise move commitする。どの段階で失敗しても、本状態は変更前と同一でなければならない。

### Save schema

正式v1前のarchive migrationは保証しない。現行`wire_state_v6`だけをexact-keyスキーマとして読み書きし、旧developmentスキーマはunsupportedバージョンとして状態変更前に拒否する。`save -> load -> save`は現行スキーマのauthoritative byte一致を維持し、欠落フィールドを位置、名前、container順から推測しない。

`SavedBackbonePortBinding`の`support_level`と`support_group_id`は保存値をそのまま読む。loadは段の振り直しや欠落値の補完を行わず、範囲外の値は状態変更前に拒否する。段の割当と物理占有の判断は[`generation.md`](generation.md)のrow placementだけが持つ。

## Public view lifetime

`CoreState` / `CoreView`から取得したpointer、reference、viewは、同じ`CoreState`に対する次のnon-const操作まで有効である。non-const操作の成功・失敗を跨いだアドレス同一性は保証しない。長期保持が必要な利用側は`ObjectId`を保持し、操作後に再取得する。

| API / result | 参照元storage | 無効になり得る操作 | 状態変更後の参照安定性 |
|---|---|---|---|
| `SpanLayoutView` / `SpanLayoutRulesView` | `SpanLayoutCache.records_by_span`の`unordered_map`内`optional` | record erase、layout/rule再保存、storage代入。insertのrehashはiteratorを無効化する | 保証しない |
| `CoreView`のPole / Port / Span / Bundle / Attachment pointer、`PoleDetailInfo` | `EditState`の`ObjectStore` | vectorのinsert/reallocation、eraseの末尾要素移動、storage代入 | 保証しない |
| backbone node / edge / binding pointerとCoreViewのmap/vector reference | `SavedBackboneGraph`、実行時index、debug vector/map | vector insert/erase、map erase/rehash、storage代入 | 保証しない |
| curve / bounds / visual / renderキャッシュpointer、visual curveキャッシュreference | 実行時キャッシュの`unordered_map` / vector | キャッシュ要素の削除・再保存、vector更新、storage代入。rehashはiteratorを無効化する | 保証しない |
| inspection result内pointer (`PoleDetailInfo`等) | 上記viewが指すstorage | 上記と同じ | 保証しない |

現在public契約として状態変更を跨ぐ参照安定性を保証する利用側はない。既存テストもvalue / id / `ChangeSet`を観測し、pointer addressを不変条件にしてはならない。
