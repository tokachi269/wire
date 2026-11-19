# wire

電柱と電線のネットワークを生成・編集・検証するC++製Coreと、開発用viewerを提供する。

## 現在の構成

- `core`: dataモデル、backbone generation、post-edit derive、検証
- `viewer`: core出力を表示・操作する開発用利用側
- `tools`: architecture/test family lint
- `docs`: 現行設計、残存legacy、merge条件、テスト、実行手順

`SavedBackboneGraph`をtopology authorityとし、rules、layout、geom、drawを一方向に派生する。viewerは不足したtopologyやplacementを推測しない。

## ドキュメント

1. [リポジトリ共通の設計](docs/architecture.md)
2. [Wireの設計](docs/wire/architecture.md)
3. [merge前の確認事項](docs/merge_readiness.md)
4. [検証方針](docs/testing.md)
5. [エージェントによる開発支援の共通手順](docs/engineering/agent_harness.md)
6. [実行コマンド](docs/command_cheatsheet.md)
7. [docsの索引](docs/README.md)

## Quick check

```powershell
python tools\arch_lint.py
python tools\test_family_lint.py
& .\build-vs18-coretests\domains\wire\Debug\wire_core_tests.exe
& .\build-vs18-coretests\domains\wire\Debug\wire_core_tests.exe backbone
```

詳細なbuild/viewerコマンドは[command_cheatsheet.md](docs/command_cheatsheet.md)を参照する。
