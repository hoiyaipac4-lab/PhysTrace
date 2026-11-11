"""Pack and restore a verified Orbax checkpoint in Release-sized tar parts.

The input file list is the historical model manifest with relative paths, sizes,
and SHA-256 digests. The local pack step neither converts nor loads tensors.
"""
import argparse
from contextlib import nullcontext
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import tarfile


BLOCK = 8 * 1024**2
RELEASE_LIMIT = 2 * 1024**3
DEFAULT_PART_BYTES = 1900 * 1024**2


def safe_name(name):
    path = PurePosixPath(name)
    if path.is_absolute() or not path.parts or ".." in path.parts or "\\" in name or ":" in name:
        raise ValueError(f"Unsafe archive path: {name}")
    return path


def file_digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


class PartWriter:
    def __init__(self, directory, limit, asset_stem):
        self.directory = directory
        self.limit = limit
        self.asset_stem = asset_stem
        self.parts = []
        self.stream = None

    def write(self, data):
        consumed = 0
        while consumed < len(data):
            if self.stream is None:
                name = f"{self.asset_stem}.tar.part{len(self.parts) + 1:03d}"
                self.stream = (self.directory / name).open("xb")
                self.parts.append({"name": name, "bytes": 0})
                self.digest = hashlib.sha256()
            part = self.parts[-1]
            size = min(len(data) - consumed, self.limit - part["bytes"])
            chunk = data[consumed:consumed + size]
            self.stream.write(chunk)
            self.digest.update(chunk)
            part["bytes"] += size
            consumed += size
            if part["bytes"] == self.limit:
                self.close()
        return consumed

    def close(self):
        if self.stream is not None:
            self.stream.close()
            self.parts[-1]["sha256"] = self.digest.hexdigest()
            self.stream = None


class PartReader:
    def __init__(self, directory, parts):
        self.paths = iter(directory / p["name"] for p in parts)
        self.stream = None

    def read(self, size):
        chunks = []
        while size > 0:
            if self.stream is None:
                path = next(self.paths, None)
                if path is None:
                    break
                self.stream = path.open("rb")
            chunk = self.stream.read(size)
            if chunk:
                size -= len(chunk)
                chunks.append(chunk)
            else:
                self.close()
        return b"".join(chunks)

    def close(self):
        if self.stream is not None:
            self.stream.close()
            self.stream = None


class HashReader:
    def __init__(self, stream):
        self.stream = stream
        self.digest = hashlib.sha256()

    def read(self, size):
        data = self.stream.read(size)
        self.digest.update(data)
        return data


def read_file_list(document):
    files = document["files"]
    paths = {}
    for row in files:
        name = row["path"]
        safe_name(name)
        if name in paths or row["bytes"] < 0 or len(row["sha256"]) != 64:
            raise ValueError(f"Invalid or duplicate file record: {name}")
        paths[name] = row
    if not paths:
        raise ValueError("Empty checkpoint manifest")
    return paths


def pack(root, model_info, output, part_bytes):
    if not 0 < part_bytes < RELEASE_LIMIT:
        raise ValueError("Each part must be smaller than 2 GiB")
    model = json.loads(model_info.read_text(encoding="utf-8-sig"))
    files = read_file_list(model)
    model_id = model["model"]
    if not re.fullmatch(r"[a-z0-9][a-z0-9_-]*", model_id):
        raise ValueError(f"Invalid model identifier: {model_id!r}")
    asset_stem = f"phystrace-{model_id}-12000"
    file_manifest_sha = hashlib.sha256(
        json.dumps(model["files"], sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    if model.get("checkpoint_file_manifest_sha256", file_manifest_sha) != file_manifest_sha:
        raise ValueError("Checkpoint manifest SHA-256 mismatch")
    if output.exists():
        raise FileExistsError(output)
    pending = output.with_name(output.name + ".partial")
    pending.mkdir(parents=True, exist_ok=False)
    writer = PartWriter(pending, part_bytes, asset_stem)
    try:
        with tarfile.open(fileobj=writer, mode="w|", bufsize=BLOCK, copybufsize=BLOCK) as archive:
            for name, item in files.items():
                source = root.joinpath(*safe_name(name).parts)
                if not source.is_file() or source.stat().st_size != item["bytes"]:
                    raise ValueError(f"Source size or file mismatch: {name}")
                member = tarfile.TarInfo(name)
                member.size = item["bytes"]
                member.mode = 0o644
                with source.open("rb") as stream:
                    reader = HashReader(stream)
                    archive.addfile(member, reader)
                if reader.digest.hexdigest() != item["sha256"]:
                    raise ValueError(f"Source SHA-256 mismatch: {name}")
    finally:
        writer.close()

    release = {
        "schema": "phystrace_checkpoint_release_v1",
        "model": model_id,
        "checkpoint_file_manifest_sha256": file_manifest_sha,
        "parts": writer.parts,
        "files": model["files"],
    }
    (pending / f"{asset_stem}-release-manifest.json").write_text(
        json.dumps(release, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    pending.rename(output)
    print(f"Packed {len(files)} files into {len(writer.parts)} parts.")


def restore(manifest, output=None, expected_manifest_sha=None):
    release = json.loads(manifest.read_text(encoding="utf-8-sig"))
    if release["schema"] != "phystrace_checkpoint_release_v1":
        raise ValueError("Unsupported release manifest")
    expected = read_file_list(release)
    manifest_sha = hashlib.sha256(
        json.dumps(release["files"], sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    if manifest_sha != release["checkpoint_file_manifest_sha256"]:
        raise ValueError("Embedded checkpoint manifest SHA-256 mismatch")
    if expected_manifest_sha is not None and manifest_sha != expected_manifest_sha:
        raise ValueError("Release does not match the supplied checkpoint manifest SHA-256")
    parts = release["parts"]
    if not parts or len({part["name"] for part in parts}) != len(parts):
        raise ValueError("Missing or duplicate part names")
    for part in parts:
        name = safe_name(part["name"])
        if len(name.parts) != 1 or not 0 < part["bytes"] < RELEASE_LIMIT:
            raise ValueError(f"Invalid part: {name}")
        path = manifest.parent / part["name"]
        if path.stat().st_size != part["bytes"] or file_digest(path) != part["sha256"]:
            raise ValueError(f"Release part verification failed: {path}")
    pending = None
    if output is not None:
        if output.exists():
            raise FileExistsError(output)
        pending = output.with_name(output.name + ".partial")
        pending.mkdir(parents=True, exist_ok=False)
    reader = PartReader(manifest.parent, parts)
    seen = set()
    try:
        with tarfile.open(fileobj=reader, mode="r|", bufsize=BLOCK) as archive:
            for member in archive:
                name = member.name
                path = safe_name(name)
                if name not in expected or name in seen or not member.isfile():
                    raise ValueError(f"Unexpected archive entry: {name}")
                item = expected[name]
                if member.size != item["bytes"]:
                    raise ValueError(f"Wrong extracted size: {name}")
                stream_context = nullcontext()
                if pending is not None:
                    destination = pending.joinpath(*path.parts)
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    stream_context = destination.open("xb")
                digest = hashlib.sha256()
                with archive.extractfile(member) as content, stream_context as stream:
                    for chunk in iter(lambda: content.read(BLOCK), b""):
                        if stream is not None:
                            stream.write(chunk)
                        digest.update(chunk)
                if digest.hexdigest() != item["sha256"]:
                    raise ValueError(f"Wrong extracted SHA-256: {name}")
                seen.add(name)
    finally:
        reader.close()
    if seen != set(expected):
        raise ValueError(f"Missing files: {sorted(set(expected) - seen)}")
    if pending is not None:
        pending.rename(output)
        print(f"Restored and verified {len(seen)} files.")
    else:
        print(f"Verified {len(seen)} archive entries.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    packing = commands.add_parser("pack")
    packing.add_argument("--root", type=Path, required=True, help="Root containing the models/ directory")
    packing.add_argument("--model-info", type=Path, required=True)
    packing.add_argument("--output", type=Path, required=True)
    packing.add_argument("--part-bytes", type=int, default=DEFAULT_PART_BYTES)
    restoring = commands.add_parser("restore")
    restoring.add_argument("--manifest", type=Path, required=True)
    restoring.add_argument("--output", type=Path, required=True)
    restoring.add_argument("--expected-manifest-sha")
    verifying = commands.add_parser("verify")
    verifying.add_argument("--manifest", type=Path, required=True)
    verifying.add_argument("--expected-manifest-sha")
    args = parser.parse_args()
    if args.command == "pack":
        pack(args.root, args.model_info, args.output, args.part_bytes)
    else:
        restore(args.manifest, getattr(args, "output", None), args.expected_manifest_sha)


if __name__ == "__main__":
    main()
