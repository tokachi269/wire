# Legacy map

このドキュメントは、残存する旧経路と削除済みfamilyの現在地だけを示す。作業履歴はGitで確認する。

## 撤回した決定

現行の編集後操作の意味は[backbone_operation_semantics.md](backbone_operation_semantics.md)、セッションの一時データは[state_and_persistence.md](state_and_persistence.md)を正本とする。ここには撤回した決定だけを残す。

| family | 状態 | normal path | 消す条件 |
|---|---|---|---|
| M4 trial削除 | 撤回済み。preflightを増やしてもtrialは削除せず、同等の失敗保証を持つ代替トランザクション方式へ置換できた場合だけ退役する | トランザクション契約 | MutationPlan / journal / copy-on-write等で全段階失敗時の本状態不変を保証できたとき |

## 隔離

| family | 状態 | 方針 |
|---|---|---|
| 旧topology allocatorテスト | 登録解除済み | object shapeを復活させず、必要な制約だけbackboneテストで守る |
| v1実装依存テスト | retired familyとして分類 | 期待構造ではなく制約を移植する |

## 削除済み

次のfamilyは削除済みで、互換wrapperやfallbackとして戻さない。

- recalc directory、dirty queue、`ProcessDirtyQueues`
- `DirtyBits`、`regeneration_required`、`TemplateDependencyState`
- `migrate_backbone_bundle_fixed_count_increase`、`bundle_count_migration.cpp`
- support-layout authority / seed / projection / materialization
- grouped span generation、旧backbone pipeline
- span/layout/curve/port位置からのtopology復元
- capture replayとlegacy recalc UI
- span-derived backbone public query
- global cable population config / enable flag / seed
- `AddConnectionByPole`、`AddDropFromPole`、`AddDropFromSpan`、`SplitSpan`の公開API、実装、専用型
- domains/road/building固有のcore識別情報型
- `SegmentLaneAssignment`、`last_generation_lane_assignments`、旧generationテストsuite

## viewer/debug境界

通常viewerとcaptureは、neutral layout / geom / draw / `SavedBackboneGraph`を読む。support-layout authority、seed、projectionを読む表示は、退役済みのdebug処理群として扱う。通常の処理経路には含めない。

## 旧テストの扱い

旧テストは一律に捨てない。

- 現在も有効な検証、状態unchanged、識別情報、determinismは維持する。
- v1内部構造に依存するassertはbackboneの正本・派生境界へ置き換える。
- recalc順序、authority object、projection object自体を要求するテストは退役する。
- 1件ずつ延命せず、family単位で移植または削除する。
