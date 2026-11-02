#!/usr/bin/env bash
# Uses ruff for Python, clang-format for C/C++, and shfmt for shell scripts.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1

# --cached still lists a file deleted from the working tree until the deletion is
# staged, so keep only the files that exist.
ours() {
  git ls-files --cached --others --exclude-standard "$@" |
    while IFS= read -r file; do [ ! -e "$file" ] || printf '%s\n' "$file"; done
}

mapfile -t PY_FILES < <(ours '*.py')
mapfile -t CC_FILES < <(ours '*.c' '*.h' '*.cpp' '*.hpp')

status=0
run() {
  echo "==> $1"
  shift
  "$@" || status=1
}

if [ "$CHECK" -eq 1 ]; then
  run "ruff format" ruff format --check "${PY_FILES[@]}"
  run "ruff check" ruff check "${PY_FILES[@]}"
  run "clang-format" clang-format --dry-run --Werror "${CC_FILES[@]}"
  run "shfmt" shfmt -l scripts deploy
else
  run "ruff format" ruff format "${PY_FILES[@]}"
  run "ruff check --fix" ruff check --fix "${PY_FILES[@]}"
  run "clang-format -i" clang-format -i "${CC_FILES[@]}"
  run "shfmt" shfmt -w scripts deploy
fi

if [ "$status" -ne 0 ]; then
  if [ "$CHECK" -eq 1 ]; then
    echo
    echo "Formatting or lint problems." >&2
  fi
  exit "$status"
fi
