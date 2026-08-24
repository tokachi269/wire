# Portable architecture lint

`architecture_lint.py` is a repository-independent engine for source boundary checks. It accepts a manifest value
with:

- `scan.roots`
- `scan.extensions`
- `scan.exclude_patterns`
- `layers[].name`
- `layers[].patterns`
- `layers[].forbidden_tokens`

It returns every scanned file, the exactly-one-layer classification, and errors for unclassified, multiply
classified, or forbidden-token sources. It contains no project domain vocabulary.

Projects keep their own manifest and composition root. A composition root loads project configuration, calls
`lint_architecture`, then adds project-specific document, semantic-coverage, and stable authority checks.

Run the synthetic self-tests with:

```powershell
python tools/harness/test_architecture_lint.py
```

The tests use temporary projects and cover valid classification, unclassified sources, multiple classifications,
forbidden tokens, exclusions, and composition-root delegation.
