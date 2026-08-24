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
classified, or forbidden-token sources. Declared scan roots are required, and a scan that yields no configured files
fails. It contains no project domain vocabulary.

Patterns are slash-separated and support `*` and `?` within one path segment. A complete `**` segment matches zero
or more nested directories, so `source/**/*.ext` includes direct children and arbitrarily deep descendants.
Recursive exclusions use the same semantics. Character classes and other shell-specific glob features are not part
of the portable contract.

Manifest errors are returned as lint diagnostics. Roots and extensions are non-empty string lists; layer names are
non-empty and unique; patterns and forbidden tokens are string lists. There is no implicit optional-root behavior.

Projects keep their own manifest and composition root. A composition root loads project configuration, calls
`lint_architecture`, then adds project-specific document, semantic-coverage, and stable authority checks.

Run the synthetic self-tests with:

```powershell
python tools/harness/test_architecture_lint.py
```

The tests use temporary projects and cover valid classification, required roots, empty scans, malformed manifests,
unclassified and multiply classified sources, forbidden tokens, nested patterns, recursive exclusions, multiple
scan roots, and composition-root delegation.
