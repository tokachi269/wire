#!/usr/bin/env python3
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import arch_lint


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

    def test_authority_unique_owner_violation_fails(self) -> None:
        guards = [
            {
                "name": "decision_owner",
                "owner": "domains/wire/src/owner.cpp",
                "required": set(),
                "unique": {"decision_token"},
                "forbidden": set(),
            }
        ]
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
