"""Preview or restore this task's files, refusing to overwrite later edits.

The baseline archive is read directly; no duplicate ROS package tree is created.
Default is read-only. --root supports isolated patch/rollback verification.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
import zipfile


def digest(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--root', type=Path)
    args = parser.parse_args()
    artifacts = Path(__file__).resolve().parent
    workspace = (args.root or artifacts.parents[1]).resolve()
    manifest = json.loads((artifacts/'changes_manifest.json').read_text())
    archive_path = artifacts/'baseline_sources.zip'
    if digest(archive_path.read_bytes()) != manifest['baseline_archive_sha256']:
        raise SystemExit('Baseline archive hash mismatch; refusing restoration.')
    operations = []
    with zipfile.ZipFile(archive_path) as archive:
        for entry in manifest['files']:
            target = workspace/entry['path']
            if not target.resolve().is_relative_to(workspace) or target.is_symlink():
                raise SystemExit(f'Unsafe target: {target}')
            expected = entry['before_sha256']
            current = digest(target.read_bytes()) if target.exists() else None
            if current == expected:
                operations.append((entry, target, None, 'already_baseline'))
                continue
            if current != entry['after_sha256']:
                raise SystemExit(f'Later edit or unexpected file: {target}; source left intact.')
            before = archive.read(entry['path']) if expected is not None else None
            if before is not None and digest(before) != expected:
                raise SystemExit(f'Baseline entry hash mismatch: {target}')
            operations.append((entry, target, before, 'restore' if before is not None else 'remove_added'))
        # Validate every target before changing any target.
        for entry, target, before, action in operations:
            print(f'{action}: {entry["path"]}')
            if not args.apply or action == 'already_baseline':
                continue
            if before is None:
                target.unlink()
            else:
                mode = target.stat().st_mode & 0o777
                with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as stream:
                    stream.write(before)
                    temporary = Path(stream.name)
                os.chmod(temporary, mode)
                os.replace(temporary, target)
    print('Restored task baseline.' if args.apply else 'Preview only; use --apply to restore.')


if __name__ == '__main__':
    main()
