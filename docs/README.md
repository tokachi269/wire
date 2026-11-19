# docs

現行の正本は次のとおり。作業履歴と過去planはGitで確認する。docsに日記を追加しない。

| 範囲 | 入口 | 内容 |
|---|---|---|
| リポジトリ共通 | [architecture.md](architecture.md) | authority、依存方向、トランザクション契約 |
| 検証 | [testing.md](testing.md) | 契約中心の検証方針、evidence、lintとの分担 |
| 命名 | [architecture_naming.md](architecture_naming.md) | ドメインをまたぐ動詞の定義 |
| merge判断 | [merge_readiness.md](merge_readiness.md) | mainへmergeする前のゲート |
| Wire | [wire/architecture.md](wire/architecture.md) | Wireの設計文書への入口 |
| Road | [road/architecture.md](road/architecture.md) | Roadの設計文書への入口 |
| Viewer | [viewer/operations.md](viewer/operations.md) | viewer操作契約とlayout |
| 描画の共通契約 | [editor/draw_interaction.md](editor/draw_interaction.md) | RoadとWireが共有するイベント契約 |
| エージェント手順 | [engineering/agent_harness.md](engineering/agent_harness.md) | 開発支援の共通手順 |
| 実行コマンド | [command_cheatsheet.md](command_cheatsheet.md) | 実績あるローカル実行コマンド |

各ドメインの文書一覧は入口文書の参照先に従う。索引をここと各ディレクトリで二重に持たない。
