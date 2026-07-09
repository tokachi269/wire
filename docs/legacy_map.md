# Legacy map

このドキュメントは、残存する旧経路と削除済みfamilyの現在地だけを示す。
作業履歴はGitで確認する。

## 残存

| family | 状態 | normal path | 消す条件 |
|---|---|---|---|
| `UpdatePoleTypeDefinition` | active backbone pole使用中でもplacement-only差分は`kReposition`で反映。構造差分は対象 pole の incident edge を統一 regenerate し、SavedGraph identityを維持して port/span/rules/layout/geom/draw を再導出する。存続 lane の manual port は保持し、退役 lane の manual port は拒否する | post-edit API | manual port 退役拒否を緩和する場合だけ別 scenario として扱う |
| `UpdateCableTemplate` | backbone span の continuity policy decision差分は統一 regenerate で route scope を再生成し、既存 span の curve decision を編集後 cable template に合わせる。non-backbone span を含む decision差分は mutation前 unsupported。default endpoint attachment 変更は post-edit 経路で attachment 生成/退役/置換をまだ消費しない | post-edit API | default endpoint attachment の生成/退役規則を扱う scenario が必要になった時だけ拡張する |
| `UpdateBundleTemplate` | fixed count増減は単一 saved route に限り、統一 regenerate でSavedGraph identityを維持して pipeline 段を部分再実行し、port/span/rules/layout/geom/drawを再生成または退役する。multi-bundle は saved edge_bundles 順で group offset を再構成し、pair row は saved route/order から再確定する。存続spanのattachmentは保持し、退役spanのattachmentは拒否する。`population_rules` 差分はtemplate-owned derived outputとして`kReshape`で反映する | post-edit API | range化、bundle policy変更を扱う scenario が必要になった時だけ拡張する |
| `SetSpanEndpointSocketOverride` / `SetSpanBranchDownOffsetOverride` | backbone span でも `override_state` を正本として trial に書き、対象 span の edge を統一 regenerate して layout/geom/draw を再導出する | post-edit API | override の対象範囲を span-local 以外へ広げる場合だけ別 scenario として扱う |
| `UpdateLayoutSettings` | backbone span が存在しても、layout settings を trial に書いて全 route/bundle scope を統一 regenerate し、保存済み topology identity を維持して layout/geom/draw を再導出する | post-edit API | layout settings が新しい生成入力を持つ場合は同じ scope 収集に追加する |
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
