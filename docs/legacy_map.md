# Legacy map

このドキュメントは、残存する旧経路と削除済みfamilyの現在地だけを示す。
作業履歴はGitで確認する。

## 残存

| family | 状態 | normal path | 消す条件 |
|---|---|---|---|
| `UpdatePoleTypeDefinition` | active backbone pole使用中でもplacement-only差分は`kReposition`で反映。構造差分はmutation前`unsupported` | post-edit API | band増減やrole/side変更などの構造差分を扱うtemplate migrationが必要になった時だけ、SavedGraph identityを維持する新operationを設計する |
| `UpdateBundleTemplate` | fixed count増加、かつ単純 open row の 2-pole route だけ、SavedGraph identityを維持して pipeline 段を部分再実行し、port/span/rules/layout/geom/drawを再生成する。`population_rules` 差分はtemplate-owned derived outputとして`kReshape`で反映する | post-edit API | count減少、range化、policy変更、3点以上route、lateral/group/loweringを保存していない配置を扱うroute-local regenerateが必要になった時だけ拡張する |
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
