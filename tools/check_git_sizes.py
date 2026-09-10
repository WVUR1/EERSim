"""Check staged Git blobs before committing/pushing; LFS pointers are small blobs."""
from pathlib import Path
import subprocess
import sys

MIB = 1024 * 1024


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    entries = subprocess.check_output(
        ["git", "ls-files", "--stage", "-z"], cwd=root
    ).split(b"\0")
    blobs = []
    for entry in filter(None, entries):
        metadata, name = entry.split(b"\t", 1)
        mode, oid, stage = metadata.split()
        if mode == b"160000":  # A submodule stores a commit, not a file blob.
            continue
        blobs.append((oid, name.decode("utf-8", errors="replace")))
    if not blobs:
        print("No files staged/tracked; run git add before this check.")
        return 1
    output = subprocess.check_output(
        ["git", "cat-file", "--batch-check=%(objectsize)"],
        cwd=root, input=b"\n".join(oid for oid, _ in blobs) + b"\n",
    ).splitlines()
    files = [(int(size), name) for size, (_, name) in zip(output, blobs)]
    oversized = [(size, name) for size, name in files if size > 100 * MIB]
    for size, name in sorted(files, reverse=True):
        if size > 50 * MIB:
            level = "ERROR" if size > 100 * MIB else "WARNING"
            print(f"{level}: {name}: {size / MIB:.2f} MiB")
    largest, name = max(files)
    print(f"Checked {len(files)} Git blobs; largest: {name} ({largest / MIB:.2f} MiB).")
    if oversized:
        print("GitHub blocks files over 100 MiB. Unstage and ignore them, or track them with Git LFS.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
