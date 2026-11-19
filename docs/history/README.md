# 記録

ここにあるのは完了した調査の固定記録である。正本ではない。更新しない。

現行の契約は`docs/`の各設計文書を参照する。ここに書かれた分類や一覧は、記録した時点の状態を示すだけで、網羅性を保証しない。新しい調査を始めたときは、その結果を同じ形式で追加し、既存の記録を書き換えない。

| 記録 | 対象 |
|---|---|
| [core_policy_constants.md](core_policy_constants.md) | 変更しやすい定数の分類(T3) |
| [core_silent_fallback_audit.md](core_silent_fallback_audit.md) | 退化した幾何を既定値へ置き換える経路の分類(R5) |
| [core_state_mutable_access_audit.md](core_state_mutable_access_audit.md) | 変更可能なaccess経路の分類(R8) |

日付つきの検証記録は`domains/wire/tests/audits/`にある。
