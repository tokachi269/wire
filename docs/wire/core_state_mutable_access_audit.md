# CoreStateの変更可能なaccess経路の監査

状態: 完了済み監査の固定記録。本書はR8の調査結果を記録するもので、継続更新する網羅性マトリクスではない。

R8では変更可能なaccess経路を分類した。通常の規則は、production codeが正本を変更する際には`CoreState`の操作APIを使い、testの内部状態設定には`CoreStateTestHook`を使うことである。

## `core_state.hpp`から削除したもの

次のprivate accessorはproductionにもtestにも呼び出し元がなく、削除した。

- `connection_index_access`
- `relation_index_access`
- `span_runtime_states_access`
- `cache_state_access`
- `next_generation_session_id_access`
- `path_direction_debug_records_access`

## productionに残るfriend access

| storage | 呼び出し元 | 分類 | 理由・今後の対応 |
|---|---|---|---|
| `authoritative_.edit_state` | `domains/wire/src/generation/backbone/pipeline.cpp` | 監査済みのproduction friend実装詳細 | backbone pipelineは`CoreState`のfriendとして宣言され、`emit_ports`でPortを変更する。R8では汎用の変更可能accessorを閉じ、この明示的なfriend accessをproduction側に残る接点とした。generationが所有する、より狭い操作への置換は今後のrefactorであり、R8には含めない。 |

`core_state.hpp`へ新しい`*_access()` accessorを追加しない。test専用のaccessは`CoreStateTestHook`へ置く。
