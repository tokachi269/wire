#!/usr/bin/env python3
from __future__ import annotations

import contextlib
import copy
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


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
            return lint_architecture(
                root, self.manifest() if manifest is None else manifest
            )

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

    def test_missing_required_scan_root_fails(self) -> None:
        manifest = self.manifest()
        manifest["scan"]["roots"] = ["src", "missing"]
        result = self.run_lint({"src/core/model.src": "authoritative model"}, manifest)
        self.assertTrue(any("scan root does not exist: missing" in error for error in result.errors))

    def test_zero_scanned_files_fails(self) -> None:
        result = self.run_lint({"src/ignored.txt": "not a configured source"})
        self.assertTrue(any("scan matched zero files" in error for error in result.errors))

    def test_duplicate_layer_names_fail(self) -> None:
        manifest = self.manifest()
        manifest["layers"].append(copy.deepcopy(manifest["layers"][0]))
        result = self.run_lint({"src/core/model.src": "authoritative model"}, manifest)
        self.assertTrue(any("duplicate layer name: core" in error for error in result.errors))

    def test_malformed_manifest_reports_diagnostics(self) -> None:
        cases = (
            ("scan", None, "manifest.scan must be an object"),
            ("roots", "src", "manifest.scan.roots must be a non-empty list"),
            ("roots", [""], "manifest.scan.roots[0] must be a non-empty string"),
            ("extensions", ".src", "manifest.scan.extensions must be a non-empty list"),
            ("extensions", [""], "manifest.scan.extensions[0] must be a non-empty string"),
            ("layer_name", "", "manifest.layers[0].name must be a non-empty string"),
            ("patterns", "src/*.src", "manifest.layers[0].patterns must be a list"),
            ("forbidden_tokens", "bad", "manifest.layers[0].forbidden_tokens must be a list"),
        )
        for field, value, expected in cases:
            with self.subTest(field=field, value=value):
                manifest = self.manifest()
                if field == "scan":
                    manifest["scan"] = value
                elif field in {"roots", "extensions"}:
                    manifest["scan"][field] = value
                elif field == "layer_name":
                    manifest["layers"][0]["name"] = value
                else:
                    manifest["layers"][0][field] = value
                result = self.run_lint({"src/core/model.src": "authoritative model"}, manifest)
                self.assertIn(expected, result.errors)

    def test_recursive_pattern_classifies_direct_and_nested_files(self) -> None:
        manifest = self.manifest()
        manifest["layers"] = [
            {
                "name": "package",
                "patterns": ["src/pkg/**/*.src"],
                "forbidden_tokens": [],
            }
        ]
        result = self.run_lint(
            {
                "src/pkg/direct.src": "direct",
                "src/pkg/nested/one.src": "nested",
                "src/pkg/nested/deeper/two.src": "deeper",
            },
            manifest,
        )
        self.assertEqual([], result.errors)
        self.assertEqual(3, len(result.classified))

    def test_recursive_exclusion_ignores_deep_generated_files(self) -> None:
        result = self.run_lint(
            {
                "src/core/model.src": "authoritative model",
                "src/generated/one/two/unclassified.src": "generated",
            }
        )
        self.assertEqual([], result.errors)
        self.assertEqual(["src/core/model.src"], result.files)

    def test_multiple_scan_roots_are_combined(self) -> None:
        manifest = self.manifest()
        manifest["scan"]["roots"] = ["model", "interface"]
        manifest["layers"] = [
            {"name": "model", "patterns": ["model/*.src"], "forbidden_tokens": []},
            {"name": "interface", "patterns": ["interface/*.src"], "forbidden_tokens": []},
        ]
        result = self.run_lint(
            {
                "model/value.src": "model",
                "interface/view.src": "interface",
            },
            manifest,
        )
        self.assertEqual([], result.errors)
        self.assertEqual(2, len(result.classified))

    def test_repository_composition_uses_generic_engine(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            path = root / "src" / "core" / "model.src"
            path.parent.mkdir(parents=True)
            path.write_text("import ui_dependency", encoding="utf-8")
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(self.manifest()), encoding="utf-8")
            argv = [
                "arch_lint.py",
                "--root",
                str(root),
                "--manifest",
                str(manifest_path),
            ]
            with (
                mock.patch.object(sys, "argv", argv),
                mock.patch.object(arch_lint, "check_architecture_documents", return_value=[]),
                mock.patch.object(arch_lint, "check_backbone_semantics_coverage", return_value=[]),
                mock.patch.object(arch_lint, "check_draw_interaction_contract", return_value=[]),
                mock.patch.object(arch_lint, "check_road_architecture", return_value=[]),
                contextlib.redirect_stderr(io.StringIO()),
            ):
                exit_code = arch_lint.main()
        self.assertEqual(1, exit_code)

    def test_repository_entry_reports_malformed_json(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path = root / "manifest.json"
            manifest_path.write_text("{", encoding="utf-8")
            argv = [
                "arch_lint.py",
                "--root",
                str(root),
                "--manifest",
                str(manifest_path),
            ]
            stderr = io.StringIO()
            with (
                mock.patch.object(sys, "argv", argv),
                contextlib.redirect_stderr(stderr),
            ):
                exit_code = arch_lint.main()
        self.assertEqual(1, exit_code)
        self.assertIn("cannot load manifest", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
