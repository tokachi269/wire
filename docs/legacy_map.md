# Legacy map

このドキュメントは、残存する旧経路と削除済みfamilyの現在地だけを示す。
作業履歴はGitで確認する。

## 残存

| family | 状態 | normal path | 消す条件 |
|---|---|---|---|
| `AddConnectionByPole` / `AddDrop*` / `SplitSpan` | `core/src/state/legacy/topology.cpp`でmutation前`unsupported` | いない | SavedGraphを更新する新operationを定義するか、public宣言を削除する |
| `UpdatePoleTypeDefinition` | template更新後に既存pole/portへ再適用 | post-edit API | active generated spanを含む場合のatomicityとdirect deriveを固定する |
| `pending_support_nodes` | 未保存pickを次のrequestへ渡すtransient input | DrawPath input | 保存済みnodeと混同しない限り維持 |

## 隔離

| family | 状態 | 方針 |
|---|---|---|
| 旧topology allocator test | 登録解除済み | object shapeを復活させず、必要な制約だけbackbone testで守る |
| v1実装依存test | retired familyとして分類 | 期待構造ではなく制約を移植する |

## 削除済み

次のfamilyは削除済みで、互換wrapperやfallbackとして戻さない。

- recalc directory、dirty queue、`ProcessDirtyQueues`
- `DirtyBits`、`regeneration_required`、`TemplateDependencyState`
- support-layout authority / seed / projection / materialization
- grouped span generation、旧backbone pipeline
- span/layout/curve/port位置からのtopology復元
- capture replayとlegacy recalc UI
- span-derived backbone public query
- road/building固有のcore identity型

## 旧テストの扱い

旧testは一律に捨てない。

- 現在も有効なvalidation、state unchanged、identity、determinismは維持する。
- v1内部構造に依存するassertはbackboneの正本・派生境界へ置き換える。
- recalc順序、authority object、projection object自体を要求するtestは退役する。
- 1件ずつ延命せず、family単位で移植または削除する。

