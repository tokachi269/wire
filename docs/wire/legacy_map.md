# Legacy map

このドキュメントは、残存する旧経路と削除済みfamilyの現在地だけを示す。作業履歴はGitで確認する。

## 残存

| family | 状態 | normal path | 消す条件 |
|---|---|---|---|
| `UpdatePoleTypeDefinition` | active backbone pole使用中でもplacement-only差分は`kReposition`で反映。構造差分は対象poleのincident edgeを統一regenerateし、SavedGraph識別情報を維持してport/span/rules/layout/geom/drawを再導出する。存続laneのmanual portは保持し、退役laneのmanual portは拒否する | post-edit API | manual port退役拒否を緩和する場合だけ別シナリオとして扱う |
| `UpdateCableTemplate` | backbone spanのcontinuity policyとdefault endpoint attachment差分は統一regenerateでroute scopeを再生成し、curve decisionと`AttachmentOrigin::kDefaultEndpoint`を編集後cableテンプレートに合わせる。non-backbone spanを含むdecision差分は状態変更前unsupported | post-edit API | non-backbone decision更新の実用シナリオが必要になった時だけ拡張する |
| `UpdateBundleTemplate` | fixed count増減とclassifierの`kTopology` policy差分は統一regenerateでrow-continuity component scopeをreconcileし、row key/lane一致bindingを再利用、不一致bindingを退役+再生成する。range化(`count_rule` fixed/range、min/max/default)は全既存bundleの`conductor_count`が新policy内なら出力を再生成せず受理し、外なら状態変更前に拒否する。multi-bundleはsaved edge_bundles順でgroup offsetを再構成し、pair rowはsaved row continuityから再確定する。存続spanのattachmentは保持し、退役spanのuser attachmentは拒否する。`population_rules`差分はtemplate-owned derived出力として`kReshape`で反映する | post-edit API | fresh生成が拒否する新policy、またはnon-backbone spanを含むscopeを扱うシナリオが必要になった時だけ拡張する |
| `SetSpanEndpointSocketOverride` / `SetSpanBranchDownOffsetOverride` | backbone spanでも`override_state`を正本としてtrialに書き、対象spanのedgeを統一regenerateしてlayout/geom/drawを再導出する | post-edit API | overrideの対象範囲をspan-local以外へ広げる場合だけ別シナリオとして扱う |
| `UpdateLayoutSettings` | backbone spanが存在しても、layout settingsをtrialに書いて全route/bundle scopeを統一regenerateし、保存済みtopology識別情報を維持してlayout/geom/drawを再導出する | post-edit API | layout settingsが新しい生成入力を持つ場合は同じscope収集に追加する |
| `pending_support_nodes` | `ResolveBranchPick`が作るセッションの一時データ。保存せず、生成成功で消費し、path clear/cancelで破棄する | DrawPath入力 | 保存済みnodeと混同しない限り維持 |
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
