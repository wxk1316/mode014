"""Build and verify a source overlay/evidence ZIP without moving any ROS package.

Uses the frozen source manifest; excludes full-text/web caches, bytecode and the
separately preserved baseline ZIP. Standard library only, no ROS or sockets.
"""
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[3]
RESEARCH = ROOT / 'docs/research'
ARCHIVE = RESEARCH / 'delivery_20261106.zip'
MANIFEST = RESEARCH / 'delivery_manifest.json'
REPORT = RESEARCH / 'validation/delivery_package.json'


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> None:
    source_manifest = json.loads((RESEARCH / 'changes_manifest.json').read_text())
    changed = {ROOT / entry['path'] for entry in source_manifest['files']}
    for entry in source_manifest['files']:
        if sha((ROOT / entry['path']).read_bytes()) != entry['after_sha256']:
            raise SystemExit(f"Frozen source changed: {entry['path']}")
    if sha((RESEARCH / 'changes.patch').read_bytes()) != source_manifest['patch_sha256']:
        raise SystemExit('Frozen patch hash mismatch')
    if sha((RESEARCH / 'baseline_sources.zip').read_bytes()) != source_manifest['baseline_archive_sha256']:
        raise SystemExit('Separately preserved baseline hash mismatch')

    members = set(changed)
    members.update(p for p in RESEARCH.iterdir()
                   if p.is_file() and p.suffix in {'.md', '.json', '.py', '.patch'})
    members.update(p for p in (RESEARCH / 'agent_notes').glob('*.md'))
    # Generated numeric evidence and author-written scripts, not paper caches.
    members.update(p for p in (RESEARCH / 'validation').rglob('*')
                   if p.is_file() and p.suffix in {'.json', '.py', '.sh', '.log'}
                   and '__pycache__' not in p.parts)
    members.update(p for p in (RESEARCH / 'dynamic_encounter_traces').rglob('*.json'))
    members.update(p for p in (ROOT / 'tests').rglob('*')
                   if p.is_file() and p.suffix in {'.py', '.cpp'}
                   and '__pycache__' not in p.parts)
    members.update((ROOT / 'docs/validation').glob('*.py'))
    members.update({ROOT / 'docs/算法客观理解与论文检索任务书.md',
                    ROOT / 'docs/实车日志记录与分析指南.md'})
    members.difference_update({MANIFEST, REPORT})
    payload = {p.relative_to(ROOT).as_posix(): p.read_bytes() for p in sorted(members)}
    for name, data in payload.items():
        if name.endswith('.py'):
            ast.parse(data, filename=name)
    manifest = {
        'date': '2026-11-06', 'kind': 'source overlay and offline evidence',
        'archive': ARCHIVE.name, 'standalone_ros_workspace': False,
        'changed_source_and_test_files': len(changed),
        'baseline_required_separately_for_reproduction_and_rollback': {
            'path': 'docs/research/baseline_sources.zip',
            'sha256': source_manifest['baseline_archive_sha256']},
        'excluded': ['paper_sources/', 'perception_sources/', 'paper/full-web caches',
                     '__pycache__/', '*.pyc', 'baseline_sources.zip'],
        'scope': 'offline model/oracle/fake ROS; no real-vehicle validation',
        'members': [{'path': name, 'bytes': len(data), 'sha256': sha(data)}
                    for name, data in sorted(payload.items())]}
    manifest_data = (json.dumps(manifest, ensure_ascii=False, indent=2) + '\n').encode()
    MANIFEST.write_bytes(manifest_data)
    payload[MANIFEST.relative_to(ROOT).as_posix()] = manifest_data
    with zipfile.ZipFile(ARCHIVE, 'w', compression=zipfile.ZIP_DEFLATED,
                         compresslevel=9) as archive:
        for name, data in sorted(payload.items()):
            info = zipfile.ZipInfo(name, date_time=(2026, 11, 6, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = ((ROOT / name).stat().st_mode & 0xFFFF) << 16
            archive.writestr(info, data)
    with zipfile.ZipFile(ARCHIVE) as archive:
        assert archive.testzip() is None
        assert len(archive.namelist()) == len(set(archive.namelist())) == len(payload)
        assert set(archive.namelist()) == set(payload)
        for name, data in payload.items():
            assert archive.read(name) == data == (ROOT / name).read_bytes(), name
        assert all(entry['path'] in archive.namelist() for entry in source_manifest['files'])
        assert not any('paper_sources/' in name or 'perception_sources/' in name
                       or '__pycache__/' in name or name.endswith(('.pyc', '.html', '.raw', '.pdf'))
                       or name.endswith('baseline_sources.zip') for name in archive.namelist())
    report = {'date': '2026-11-06', 'archive': ARCHIVE.relative_to(ROOT).as_posix(),
              'archive_bytes': ARCHIVE.stat().st_size,
              'archive_sha256': sha(ARCHIVE.read_bytes()),
              'manifest_sha256': sha(manifest_data), 'members_verified': len(payload),
              'changed_files_included': len(changed), 'all_hashes_and_crc_passed': True,
              'python_scripts_parsed': sum(name.endswith('.py') for name in payload),
              'full_text_and_baseline_excluded': True,
              'runtime_source_changes_during_packaging': False}
    REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
