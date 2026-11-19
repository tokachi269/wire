#!/usr/bin/env python3
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import arch_lint


def nl_join(lines: tuple[str, ...]) -> str:
    return chr(10).join(lines)


class RepositoryArchitectureChecksTest(unittest.TestCase):
    def write(self, root: Path, relative: str, content: str) -> None:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    def test_required_architecture_heading_missing_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write(
                root,
                "docs/architecture.md",
                "\n".join(
                    (
                        "# Repository architecture",
                        "## Domain着手条件",
                        "## State layers",
                        "## Operations and build",
                    )
                ),
            )
            errors = arch_lint.check_architecture_documents(root)
        self.assertIn(
            "docs/architecture.md: required architecture contract is missing '## Dependency direction'",
            errors,
        )

    def test_malformed_operation_state_table_fails(self) -> None:
        semantics = """
## 操作×状態
| Operation | `S1` | `S2` |
|---|---|---|
| `add` | `C` |
"""
        _, errors = arch_lint.parse_backbone_semantics_cells(semantics)
        self.assertTrue(
            any("malformed operation-state row" in error for error in errors)
        )

    def test_backbone_semantics_parser_follows_declared_state_order(self) -> None:
        from backbone_semantics import parse_backbone_semantics_cells

        semantics = """
## 操作×状態
| Operation | `SM` | `S1` |
|---|---|---|
| `add_one_edge` | `O` | `C` |
"""
        required, errors = parse_backbone_semantics_cells(semantics)
        self.assertEqual([], errors)
        self.assertEqual(
            {"BOS:add_one_edge:SM": "O", "BOS:add_one_edge:S1": "C"}, required
        )

    def test_backbone_semantics_preserves_alternative_outcomes_and_rejects_typos(self) -> None:
        semantics = """
## 操作×状態
| Operation | `SM` | `S1` |
|---|---|---|
| `add_one_edge` | template policyに従い`O`または`U` | `D2` |
"""
        required, errors = arch_lint.parse_backbone_semantics_cells(semantics)
        self.assertEqual([], errors)
        self.assertEqual({"BOS:add_one_edge:SM": "OU"}, required)
        for invalid in ("`KK`", "`C`または`KK`", "`C`または`D2`", "`C`または`-`"):
            with self.subTest(invalid=invalid):
                _, errors = arch_lint.parse_backbone_semantics_cells(semantics.replace("`D2`", invalid))
                self.assertTrue(any("unknown operation-state outcome" in error for error in errors))

    def test_authority_unique_owner_markdown_reaches_checker(self) -> None:
        ledger = "\n".join(
            (
                "## Backbone正本guardの検査範囲",
                "| Guard | Owner | Required | Unique | Forbidden |",
                "|---|---|---|---|---|",
                "| decision_owner | `domains/wire/src/owner.cpp` | - | `decision_token` | - |",
            )
        )
        guards, parse_errors = arch_lint.parse_backbone_authority_guards(ledger)
        self.assertEqual([], parse_errors)
        production_text = {
            "domains/wire/src/owner.cpp": "decision_token",
            "domains/wire/src/duplicate.cpp": "decision_token",
        }
        errors = arch_lint.check_authority_guard_owners(guards, production_text)
        self.assertTrue(
            any(
                "decision_token appears outside owner" in error
                and "duplicate.cpp" in error
                for error in errors
            )
        )

    def test_authority_guard_keeps_non_identifier_expression_tokens(self) -> None:
        ledger = nl_join((
            "## Backbone正本guardの検査範囲",
            "| Guard | Owner | Required | Unique | Forbidden |",
            "|---|---|---|---|---|",
            "| decision_owner | `domains/wire/src/owner.cpp` | - | `value.level = Resolve(x)` | - |",
        ))
        guards, parse_errors = arch_lint.parse_backbone_authority_guards(ledger)
        self.assertEqual([], parse_errors)
        self.assertEqual({"value.level = Resolve(x)"}, guards[0]["unique"])
        errors = arch_lint.check_authority_guard_owners(
            guards,
            {
                "domains/wire/src/owner.cpp": "value.level = Resolve(x)",
                "domains/wire/src/duplicate.cpp": "value.level = Resolve(x)",
            },
        )
        self.assertTrue(
            any("appears outside owner" in error and "duplicate.cpp" in error for error in errors)
        )

    def test_authority_guard_unquoted_token_is_rejected(self) -> None:
        ledger = nl_join((
            "## Backbone正本guardの検査範囲",
            "| Guard | Owner | Required | Unique | Forbidden |",
            "|---|---|---|---|---|",
            "| decision_owner | `domains/wire/src/owner.cpp` | - | value.level = Resolve(x) | - |",
        ))
        guards, parse_errors = arch_lint.parse_backbone_authority_guards(ledger)
        self.assertEqual([], guards)
        self.assertTrue(any("unquoted unique token" in error for error in parse_errors))

    def test_document_notation_rejects_japanese_in_code_spans(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write(root, "docs/sample.md", nl_join((
                "# sample",
                "`入口境界`を参照する。",
                "```text",
                "state -> 日本語の説明",
                "```",
            )))
            errors = arch_lint.check_document_notation(root)
        self.assertTrue(any("inline code span contains Japanese" in error for error in errors))
        self.assertTrue(any("code fence contains Japanese" in error for error in errors))

    def test_document_notation_allows_ascii_code(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write(root, "docs/sample.md", nl_join((
                "# sample",
                "`SavedBackboneGraph`を正本とする。",
                "```text",
                "src/geometry/",
                "```",
            )))
            errors = arch_lint.check_document_notation(root)
        self.assertEqual([], errors)

    def test_bindings_required_tokens_are_each_connected(self) -> None:
        required_tokens = (
            "preview_placements",
            "CoreState trial = *state_",
            "GenerateFromBackboneSpec(spec)",
            'result.set("failureCategory"',
            'result.set("reasonCode"',
            "wireBuildSourceHash",
            "wireBuildVersion",
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for missing in required_tokens:
                with self.subTest(missing=missing):
                    self.write(
                        root,
                        "web/wasm/bindings.cpp",
                        "\n".join(token for token in required_tokens if token != missing),
                    )
                    errors = arch_lint.check_draw_interaction_contract(root)
                    self.assertIn(
                        "web/wasm/bindings.cpp: shared draw interaction contract "
                        f"is missing {missing!r}",
                        errors,
                    )

    def test_required_draw_contract_missing_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            errors = arch_lint.check_draw_interaction_contract(Path(temporary))
        self.assertIn(
            "docs/editor/draw_interaction.md: shared draw interaction source is missing",
            errors,
        )

    def test_forbidden_obsolete_draw_path_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            self.write(root, "web/src/App.svelte", "Generate Path")
            errors = arch_lint.check_draw_interaction_contract(root)
        self.assertIn(
            "web/src/App.svelte: obsolete draw interaction path remains: 'Generate Path'",
            errors,
        )


if __name__ == "__main__":
    unittest.main()
