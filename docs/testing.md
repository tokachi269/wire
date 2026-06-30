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
build-vs18-coretests\core\Debug\wire_core_tests.exe backbone
```

## fixture

backbone fixtureは`core/tests/backbone/fixtures.*`に置く。
testはv1 topology API、existing span geometry、position proximityから入力正本を組み立てない。
代表sceneは結果だけでなく、topology/connectivity/placementのownerも検証する。

## test family manifest

`core/tests/test_family_manifest.json`はtest sourceのownerをfamily単位で管理する。
新しい登録test sourceが未分類、または複数familyへ分類された場合はlintを失敗させる。
C番号は履歴識別子として維持するが、C番号の増加を進捗指標にしない。

## architecture guard

`tools/arch_manifest.json`と`tools/arch_lint.py`は次を検出する。

- 未分類source/header
- viewerからcore private headerへの依存
- geometry/validationからgeneration privateへの逆依存
- recalc/support-layout familyの復活
- road/rail/building/city domain identityのcore流入

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

