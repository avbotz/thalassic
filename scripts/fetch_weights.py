#!/usr/bin/env python3
"""Download the models listed in weights/manifest.toml into weights/.

Missing models are downloaded. Models whose checksum differs are downloaded again,
and the old copy is kept as <file>.old.

    fetch_weights.py                # every file in the manifest
    fetch_weights.py torp.onnx      # just these
    fetch_weights.py --check        # report only: exits 1 unless all are present and match
"""

import argparse
import hashlib
import sys
import tomllib
import urllib.request
from pathlib import Path

WEIGHTS = Path(__file__).resolve().parent.parent / "weights"
MANIFEST = WEIGHTS / "manifest.toml"
CHUNK = 1 << 20
# Kept in weights/ but never listed: TensorRT engines are built on the Jetson
# from the ONNX files, and the rest are this script's own.
UNLISTED = {".engine", ".old", ".part"}


def sha256(path: Path) -> str:
    with path.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def download(url: str, path: Path, expected: str) -> None:
    part = path.with_name(path.name + ".part")
    try:
        digest = hashlib.sha256()
        with urllib.request.urlopen(url, timeout=60) as response, part.open("wb") as out:
            while chunk := response.read(CHUNK):
                digest.update(chunk)
                out.write(chunk)
        if digest.hexdigest() != expected:
            raise ValueError(
                f"the download's sha256 is {digest.hexdigest()}, the manifest's {expected}"
            )
        if path.exists():
            path.replace(path.with_name(path.name + ".old"))
        part.replace(path)
    finally:
        part.unlink(missing_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument("files", nargs="*", help="manifest entries to fetch (default: all)")
    parser.add_argument(
        "--check",
        action="store_true",
        help="only report; exit 1 unless every file is present and matches",
    )
    args = parser.parse_args()

    manifest = tomllib.loads(MANIFEST.read_text())
    unknown = sorted(set(args.files) - manifest.keys())
    if unknown:
        print(f"not in {MANIFEST.name}: {', '.join(unknown)}", file=sys.stderr)
        return 2

    failed = False
    for name in args.files or manifest:
        entry = manifest[name]
        path = WEIGHTS / name
        if path.exists() and sha256(path) == entry["sha256"]:
            print(f"ok           {name}")
            continue
        state = "differs" if path.exists() else "missing"
        if args.check:
            print(f"{state:<12} {name}")
            failed = True
            continue
        print(f"downloading  {name} ({state})", flush=True)
        try:
            download(entry["url"], path, entry["sha256"])
        except (OSError, ValueError) as error:
            print(f"failed       {name}: {error}", file=sys.stderr)
            failed = True

    if not args.files:
        extra = sorted(
            p.name
            for p in WEIGHTS.iterdir()
            if p.is_file() and p != MANIFEST and p.suffix not in UNLISTED and p.name not in manifest
        )
        if extra:
            print(f"not in {MANIFEST.name} (left alone): {', '.join(extra)}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
