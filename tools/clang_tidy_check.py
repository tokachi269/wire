#!/usr/bin/env python3
"""Run clang-tidy with visible progress from a CMake compilation database."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys


DEFAULT_SOURCE_ROOTS = (
    Path("domains/wire/src"),
    Path("domains/wire/tests"),
    Path("domains/road/src"),
    Path("domains/road/tests"),
    Path("viewer/src"),
)

VS_CLANG_TIDY_CANDIDATES = (
    Path("C:/Program Files/Microsoft Visual Studio/18/Community/VC/Tools/Llvm/x64/bin/clang-tidy.exe"),
    Path("C:/Program Files/Microsoft Visual Studio/18/Community/VC/Tools/Llvm/bin/clang-tidy.exe"),
    Path("C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/x64/bin/clang-tidy.exe"),
    Path("C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/bin/clang-tidy.exe"),
)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def resolve_clang_tidy(explicit: str | None) -> str:
    if explicit:
        return explicit
    found = shutil.which("clang-tidy")
    if found:
        return found
    for candidate in VS_CLANG_TIDY_CANDIDATES:
        if candidate.exists():
            return str(candidate)
    raise RuntimeError("clang-tidy was not found")


def load_compile_files(build_dir: Path) -> list[Path]:
    db = build_dir / "compile_commands.json"
    if not db.exists():
        raise RuntimeError(f"missing compile_commands.json: {db}")
    with db.open("r", encoding="utf-8") as handle:
        commands = json.load(handle)
    files = []
    for command in commands:
        file_value = command.get("file")
        if isinstance(file_value, str) and file_value.endswith(".cpp"):
            files.append(Path(file_value).resolve())
    return sorted(set(files))


def in_roots(path: Path, roots: tuple[Path, ...]) -> bool:
    for root in roots:
        try:
            path.relative_to(root)
            return True
        except ValueError:
            continue
    return False


def select_files(all_files: list[Path], requested: list[str]) -> list[Path]:
    root = repo_root()
    roots = tuple((root / item).resolve() for item in DEFAULT_SOURCE_ROOTS)
    requested_roots = tuple((root / item).resolve() for item in requested)
    active_roots = requested_roots if requested_roots else roots
    return [path for path in all_files if in_roots(path, active_roots)]


def run_one(clang_tidy: str, build_dir: Path, file_path: Path) -> tuple[Path, int, str]:
    completed = subprocess.run(
        [clang_tidy, "-p", str(build_dir), str(file_path)],
        cwd=repo_root(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    return file_path, completed.returncode, completed.stdout


def main() -> int:
    parser = argparse.ArgumentParser(description="Run clang-tidy with progress.")
    parser.add_argument("--build-dir", default="build-clang-tidy", help="CMake build directory with compile_commands.json")
    parser.add_argument("--clang-tidy", default=None, help="clang-tidy executable")
    parser.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1), help="parallel clang-tidy jobs")
    parser.add_argument("--path", action="append", default=[], help="limit to a source subtree; repeatable")
    parser.add_argument("--limit", type=int, default=0, help="debug limit for the first N selected files")
    args = parser.parse_args()

    root = repo_root()
    build_dir = (root / args.build_dir).resolve()
    clang_tidy = resolve_clang_tidy(args.clang_tidy)
    files = select_files(load_compile_files(build_dir), args.path)
    if args.limit > 0:
      files = files[:args.limit]
    if not files:
        print("clang-tidy-check: no source files selected", file=sys.stderr, flush=True)
        return 1

    print(f"clang-tidy-check: {len(files)} files, jobs={args.jobs}, build={build_dir}", flush=True)
    failures: list[tuple[Path, str]] = []
    completed_count = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, args.jobs)) as executor:
        futures = [executor.submit(run_one, clang_tidy, build_dir, path) for path in files]
        for future in concurrent.futures.as_completed(futures):
            file_path, code, output = future.result()
            completed_count += 1
            rel = file_path.relative_to(root)
            status = "PASS" if code == 0 else "FAIL"
            print(f"[{completed_count}/{len(files)}] {status} {rel}", flush=True)
            if code != 0:
                failures.append((rel, output))

    if failures:
        print(f"clang-tidy-check: failed ({len(failures)} files)", file=sys.stderr, flush=True)
        for rel, output in failures:
            print(f"\n--- {rel} ---", file=sys.stderr, flush=True)
            print(output.rstrip(), file=sys.stderr, flush=True)
        return 1

    print(f"clang-tidy-check: pass ({len(files)} files)", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f"clang-tidy-check: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
