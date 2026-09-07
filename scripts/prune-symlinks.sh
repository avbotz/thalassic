#!/usr/bin/env bash
# Delete dangling symlinks from the build and install folders left by deleted files after symlink-install.
# Refer to https://github.com/colcon/colcon-core/issues/633 and https://github.com/colcon/colcon-core/issues/692 for more info.

set -euo pipefail

dry_run=false

cd "$(dirname "${BASH_SOURCE[0]}")/.."

dirs=()

if [ -d "build" ]; then
    dirs+=("build")
fi

if [ -d "install" ]; then
    dirs+=("install")
fi

if [ ${#dirs[@]} -eq 0 ]; then
    echo "prune-symlinks: nothing to search"
    exit 0
fi

# -xtype l matches links whose target is missing (and link loops, which are
# equally useless). Null-delimited so paths with spaces survive; the Spinnaker
# SDK ships a few.
count=0
while IFS= read -r -d '' link; do
    echo "removing $link -> $(readlink "$link")"
    rm -f "$link"
    count=$((count + 1))
done < <(find "${dirs[@]}" -xtype l -print0)

echo "prune-symlinks: removed $count dangling symlink(s)"
