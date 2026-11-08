"""Apply the delivered diff to isolated originals, then verify safe rollback."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[3]
ARTIFACTS = ROOT/'docs/research'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def main():
    manifest = json.loads((ARTIFACTS/'changes_manifest.json').read_text())
    checks = {}
    with tempfile.TemporaryDirectory(prefix='rda-patch-verify-') as directory:
        stage = Path(directory)
        with zipfile.ZipFile(ARTIFACTS/'baseline_sources.zip') as archive:
            for entry in manifest['files']:
                if entry['before_sha256'] is not None:
                    archive.extract(entry['path'], stage)
        def patch(dry=False):
            command = ['patch', '-p1', '-i', str(ARTIFACTS/'changes.patch')]
            if dry:
                command.append('--dry-run')
            return subprocess.run(command, cwd=stage, text=True, capture_output=True)
        def restore(apply=False):
            command = [sys.executable, str(ARTIFACTS/'restore_baseline.py'), '--root', str(stage)]
            if apply:
                command.append('--apply')
            return subprocess.run(command, text=True, capture_output=True)
        result = patch(True)
        checks['patch_dry_run'] = result.returncode == 0
        if result.returncode:
            raise RuntimeError(result.stdout+result.stderr)
        checks['patch_apply'] = patch().returncode == 0
        checks['applied_hashes_match'] = all(sha(stage/e['path']) == e['after_sha256'] for e in manifest['files'])
        checks['rollback_preview'] = restore().returncode == 0
        first = stage/manifest['files'][0]['path']
        original = first.read_bytes()
        first.write_bytes(original+b'\n// later user edit\n')
        state_before = {e['path']: sha(stage/e['path']) for e in manifest['files']}
        rejected = restore(True)
        checks['refused_later_edit_without_any_write'] = rejected.returncode != 0 and all(
            sha(stage/name) == value for name, value in state_before.items())
        first.write_bytes(original)
        checks['rollback_apply'] = restore(True).returncode == 0
        checks['restored_before_hashes_and_removed_only_added'] = all(
            sha(stage/e['path']) == e['before_sha256'] for e in manifest['files'])
        checks['idempotent_rollback'] = restore(True).returncode == 0
    result = {'date': '2026-11-06', 'checks': checks, 'files': len(manifest['files']),
              'source_workspace_modified_by_verification': False, 'isolated_outside_src': True}
    (ARTIFACTS/'validation/patch_and_rollback.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return not all(checks.values())


if __name__ == '__main__':
    raise SystemExit(main())
