# テスト体系

## test family

| family | 目的 |
|---|---|
| default core tests | public behavior、validation、geometry、state serviceを含む通常回帰 |
| backbone acceptance | generation、SavedGraph、binding、layout/geom/draw、update境界 |
| viewer tests | input policy、座標変換、selection、代表sceneの表示出力 |
| architecture lint | source layer分類と禁止依存 |
| test family lint | 登録testのowner分類 |

backbone acceptanceのfilterは`backbone`である。旧`bb2` filter aliasは持たない。

```powershell
build-vs18-coretests\domains\wire\Debug\wire_core_tests.exe backbone
```

## fixture

backbone fixtureは`domains/wire/tests/backbone/fixtures.*`に置く。
testはv1 topology API、existing span geometry、position proximityから入力正本を組み立てない。
代表sceneは結果だけでなく、topology/connectivity/placementのownerも検証する。

## test family manifest

`domains/wire/tests/test_family_manifest.json`はtest sourceのownerをfamily単位で管理する。
新しい登録test sourceが未分類、または複数familyへ分類された場合はlintを失敗させる。
C番号は履歴識別子として維持するが、C番号の増加を進捗指標にしない。

## failure diagnostics

新規testは `WIRE_TEST_EXPECT(condition, reason)` で主要な前提と不変条件を検査する。
既存testを触る場合は、その関数内の入口条件や複数手順の主要境界から理由付きへ移行する。
全既存testの機械的一括移行は進捗ではない。複数手順のfailは、最初に壊れた操作名や期待した不変条件を
reasonに残す。

## 操作×状態coverageと不変条件

操作×状態coverageは、意味論表のrequired cellへ実際に到達したことを保証する。
内部整合の正しさはcoverageへ詰め込まず、各観測点で共通不変条件を実行して保証する。

- wire: `Observe` / `ObserveEmpty` / `ObserveMidspan`がrow frame coherenceを検査してから
  `(cell, entry)`を記録する
- core entry: 全required cellを`core_api`で実行する
- WASM / viewer entry: `docs/wire/backbone_operation_semantics.md`の入口境界表を
  web testが読み、実WASM stateと`ViewerActions` payloadで必須cellを実行する
- road: productionのedit/load境界とtestの代表観測点、seed付き操作列で、同じ
  `ValidateGraphInvariants`を使用する。test専用の別invariantは作らない

`derived_equality`だけのcaseは独立evidenceにならない。full core testは該当case一覧を
終了時に出力し、件数を`docs/merge_readiness.md`へ記録する。

## architecture guard

`tools/arch_manifest.json`と`tools/arch_lint.py`は次を検出する。

- 未分類source/header
- viewerからcore private headerへの依存
- geometry/validationからgeneration privateへの逆依存
- recalc/support-layout familyの復活
- domains/road/rail/building/city domain identityのcore流入
- `docs/wire/backbone_operation_semantics.md` の操作×状態表と
  `domains/wire/tests/spec_ledger.md` の `BOS:<operation>:<state>` coverage 対応漏れ

source scanは安定した境界だけに使う。広い単語grepをtest semanticsの代わりにしない。

## 旧test

旧testの期待値をそのままbackboneの合格基準にしない。
守っていた制約を抽出し、次のいずれかに分類する。

- そのまま維持
- backboneの正本/派生出力で書き換え
- v1実装詳細としてfamily退役
- 現設計と衝突するため削除

unsupported testはstate unchangedまで確認する。
post-edit testはmarkerではなく、実際のlayout/geom/draw更新またはmutation前rejectを確認する。
