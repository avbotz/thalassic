#!/usr/bin/env bash
# Format (or check) every source file this repository owns.
#
#   pixi run format          rewrite files in place
#   pixi run lint            check only; non-zero exit if anything is off
#
# Ruff for Python (ruff.toml), clang-format for C and C++ (.clang-format)
# File list comes from `git ls-files`, which lists submodule directories but not their contents, so upstream code is # never reformatted.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1

ours() { git ls-files --cached --others --exclude-standard "$@"; }

mapfile -t PY_FILES < <(ours '*.py')
mapfile -t CC_FILES < <(ours '*.c' '*.h' '*.cpp' '*.hpp')

status=0
run() {
    local label="$1"; shift
    echo "==> $label (${#PY_FILES[@]} py, ${#CC_FILES[@]} c/c++)"
    "$@" || status=1
}

if [ "$CHECK" -eq 1 ]; then
    run "ruff format" ruff format --check "${PY_FILES[@]}"
    run "ruff check" ruff check "${PY_FILES[@]}"
    run "clang-format" clang-format --dry-run --Werror "${CC_FILES[@]}"
else
    run "ruff format" ruff format "${PY_FILES[@]}"
    run "ruff check --fix" ruff check --fix "${PY_FILES[@]}"
    run "clang-format -i" clang-format -i "${CC_FILES[@]}"
fi

if [ "$status" -ne 0 ]; then
    if [ "$CHECK" -eq 1 ]; then
        echo
        echo "Formatting or lint problems above. Most of them: pixi run format" >&2
    fi
    exit "$status"
fi
