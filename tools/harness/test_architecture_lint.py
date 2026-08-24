#!/usr/bin/env python3
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


HARNESS_ROOT = Path(__file__).resolve().parent
TOOLS_ROOT = HARNESS_ROOT.parent
sys.path.insert(0, str(HARNESS_ROOT))
sys.path.insert(0, str(TOOLS_ROOT))

from architecture_lint import lint_architecture
import arch_lint


class ArchitectureLintTest(unittest.TestCase):
    def manifest(self) -> dict[str, object]:
        return {
            "version": 1,
            "scan": {
                "roots": ["src"],
                "extensions": [".src"],
                "exclude_patterns": ["src/generated/**"],
            },
            "layers": [
                {
                    "name": "core",
                    "patterns": ["src/core/*.src"],
                    "forbidden_tokens": ["ui_dependency"],
                },
                {
                    "name": "ui",
                    "patterns": ["src/ui/*.src"],
                    "forbidden_tokens": [],
                },
            ],
        }

    def run_lint(
        self,
        files: dict[str, str],
        manifest: dict[str, object] | None = None,
    ):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for relative, content in files.items():
                path = root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8")
            return lint_architecture(root, manifest or self.manifest())

    def test_valid_project_passes(self) -> None:
        result = self.run_lint(
            {
                "src/core/model.src": "authoritative model",
                "src/ui/view.src": "render model",
            }
        )
        self.assertEqual([], result.errors)
        self.assertEqual(2, len(result.classified))

    def test_unclassified_source_fails(self) -> None:
        result = self.run_lint({"src/other/unknown.src": "unknown"})
        self.assertTrue(any("expected exactly one layer" in error and "none" in error for error in result.errors))

    def test_multiply_classified_source_fails(self) -> None:
        manifest = self.manifest()
        manifest["layers"] = [
            {
                "name": "all_sources",
                "patterns": ["src/**/*.src"],
                "forbidden_tokens": [],
            },
            {
                "name": "shared",
                "patterns": ["src/shared/*.src"],
                "forbidden_tokens": [],
            },
        ]
        result = self.run_lint({"src/shared/value.src": "shared"}, manifest)
        self.assertTrue(any("all_sources" in error and "shared" in error for error in result.errors))

    def test_layer_forbidden_token_fails(self) -> None:
        result = self.run_lint({"src/core/model.src": "import ui_dependency"})
        self.assertTrue(any("core forbids 'ui_dependency'" in error for error in result.errors))

    def test_excluded_generated_source_passes(self) -> None:
        result = self.run_lint(
            {
                "src/core/model.src": "authoritative model",
                "src/generated/unclassified.src": "generated",
            }
        )
        self.assertEqual([], result.errors)
        self.assertEqual(["src/core/model.src"], result.files)

    def test_repository_composition_uses_generic_engine(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = root / "src" / "core" / "model.src"
            path.parent.mkdir(parents=True)
            path.write_text("authoritative model", encoding="utf-8")
            result = arch_lint.run_generic_architecture_checks(root, self.manifest())
        self.assertEqual([], result.errors)
        self.assertEqual({"src/core/model.src": "core"}, result.classified)


if __name__ == "__main__":
    unittest.main()
