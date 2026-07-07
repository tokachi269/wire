# Legacy map

このドキュメントは、残存する旧経路と削除済みfamilyの現在地だけを示す。
作業履歴はGitで確認する。

## 残存

| family | 状態 | normal path | 消す条件 |
|---|---|---|---|
| `UpdatePoleTypeDefinition` | active backbone pole使用中でもplacement-only差分は`kReposition`で反映。構造差分はmutation前`unsupported` | post-edit API | band増減やrole/side変更などの構造差分を扱う regenerate scenario が必要になった時だけ、SavedGraph identityを維持する対応を追加する |
| `UpdateBundleTemplate` | fixed count増減は単一 saved route に限り、統一 regenerate でSavedGraph identityを維持して pipeline 段を部分再実行し、port/span/rules/layout/geom/drawを再生成または退役する。multi-bundle は saved edge_bundles 順で group offset を再構成し、pair row は saved route/order から再確定する。存続spanのattachmentは保持し、退役spanのattachmentは拒否する。`population_rules` 差分はtemplate-owned derived outputとして`kReshape`で反映する | post-edit API | range化、policy変更を扱う scenario が必要になった時だけ拡張する |
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
- `migrate_backbone_bundle_fixed_count_increase`、`bundle_count_migration.cpp`
- support-layout authority / seed / projection / materialization
- grouped span generation、旧backbone pipeline
- span/layout/curve/port位置からのtopology復元
- capture replayとlegacy recalc UI
- span-derived backbone public query
- global cable population config / enable flag / seed
- `AddConnectionByPole`、`AddDropFromPole`、`AddDropFromSpan`、`SplitSpan`のpublic API、実装、専用型
- road/building固有のcore identity型
- `SegmentLaneAssignment`、`last_generation_lane_assignments`、旧generation test suite

## viewer/debug境界

通常viewerとcaptureは、neutral layout / geom / draw / `SavedBackboneGraph` を読む。
support-layout authority、seed、projectionを読む表示は通常pathではなく、退役済みdebug familyとして扱う。

## 旧テストの扱い

旧testは一律に捨てない。

- 現在も有効なvalidation、state unchanged、identity、determinismは維持する。
- v1内部構造に依存するassertはbackboneの正本・派生境界へ置き換える。
- recalc順序、authority object、projection object自体を要求するtestは退役する。
- 1件ずつ延命せず、family単位で移植または削除する。
