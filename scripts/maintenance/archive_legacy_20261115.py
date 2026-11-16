#!/usr/bin/env python3
"""Explicit, recoverable document/launcher reorganization; never runs a launcher.

This one-off mechanical migration is kept with its manifest for reproducibility.
--apply refuses existing archive targets. --verify only reads and updates the
maintenance manifest; it cannot touch training source, models, logs or packages.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import tarfile
from urllib.parse import unquote, urlsplit

DEPLOY = Path('/home/dong/rl_sar_b2_z1')
TRAIN = Path('/home/dong/b2_z1_VD2')
ROOTS = (TRAIN, DEPLOY)
NAME = 'organization_20261115'
LINK = re.compile(r'(!?\[[^\]\n]*\]\()([^\s)]+)(\))')


def plan():
    rows = []
    documents = {
        TRAIN: {
            'vd5': ['VD5_GITHUB_SYNC_20261110.md', 'VD5_STARTUP_ADAPTATION_20261110.md',
                    'VD5_TASK_TRACKING_REVIEW_20261110.md', 'VD5_WORLD_SEAM_COMMANDS_20261109.md'],
            'vd9': ['VD9_CONTINUOUS_TRAINING_20261112.md', 'VD9_TRAINING_PLAN_20261112.md'],
            'vd10': ['VD10_GROUND_CONTACT_FIX_20261113.md', 'VD10_HOLLOW_GUN_MODEL_20261112.md'],
            'vd11': ['VD11_MOTION_QUALITY_20261113.md'],
            'vd12': ['VD12_GEOMETRY_GUARD_20261114.md'],
        },
        DEPLOY: {
            'vd1': ['VD1_MUJOCO.md', 'VD1_MUJOCO_NO_PRIVILEGE_AUDIT.md',
                    'VD1_DUAL_VISION_DEBUG.md', 'VD1_FLAT_WALK_DEBUG.md', 'VD1_TRUTH_CURVE_DEBUG.md'],
            'vd5': ['VD5_MUJOCO_SIM2SIM_20261110.md'],
            'project': ['B2_Z1_PROJECT_PLAN_DETAILED_20261113.md'],
        },
    }
    launchers = {
        TRAIN: ['train_vd3.sh', 'train_vd3_symmetry.sh', 'train_vd4.sh', 'train_vd5.sh',
                'train_vd5_startup.sh', 'train_vd6.sh', 'train_vd7.sh', 'train_vd8.sh',
                'train_vd9.sh', 'train_vd10.sh', 'train_vd11.sh', 'train_vd12.sh',
                'play_vd3.sh', 'play_vd3_curve.sh'],
        DEPLOY: ['debug_vd1_flat_walk.sh', 'debug_vd1_truth_curve.sh', 'debug_vd1_dual_vision.sh',
                 'debug_vd3_rear_vision.sh', 'play_vd3_mujoco.sh', 'play_vd3_front_path.sh',
                 'debug_vd5_gait_compare.sh', 'debug_vd5_mujoco.sh',
                 'debug_vd5_work_distance.sh', 'debug_vd5_rear_preview.sh'],
    }
    for root, versions in documents.items():
        for version, names in versions.items():
            rows.extend((root/'docs'/name, root/'docs/archive'/version/name, 'document') for name in names)
    for root, names in launchers.items():
        for name in names:
            version = re.search(r'vd\d+', name)[0]
            rows.append((root/'scripts'/name, root/'scripts/archive'/version/name, 'launcher'))
    return rows


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def markdowns(root):
    paths = list(root.glob('README*.md'))
    for part in ('docs', 'scripts', 'tests'):
        paths.extend((root/part).rglob('*.md'))
    return sorted(set(paths))


def local_target(file, url):
    parsed = urlsplit(url)
    if not parsed.path or parsed.scheme or parsed.netloc or url.startswith('#'):
        return None
    target = unquote(parsed.path)
    target = re.sub(r':\d+$', '', target)
    return Path(os.path.abspath(file.parent/target))


def broken_links():
    broken = set()
    for root in ROOTS:
        for file in markdowns(root):
            for match in LINK.finditer(file.read_text()):
                target = local_target(file, match[2])
                if target is not None and not target.exists():
                    broken.add((str(file), str(target)))
    return sorted(broken)


def protected_files():
    # Do not follow checkpoint/log trees. Protect all actual training code/assets
    # and imported Python modules, plus the running entry and IDE configuration.
    paths = [TRAIN/'scripts/train_vd13.sh', TRAIN/'CODE_VERSION.json']
    for base in (TRAIN/'source', TRAIN/'scripts/rsl_rl', TRAIN/'tests', DEPLOY/'tests'):
        paths.extend(p for p in base.rglob('*') if p.is_file()
                     and '__pycache__' not in p.parts and p.suffix != '.md')
    paths.extend(p for p in (DEPLOY/'scripts').rglob('*.py') if 'maintenance' not in p.parts)
    archived = {source for source, _, kind in plan() if kind == 'launcher'}
    for root in ROOTS:
        paths.extend(p for p in (root/'scripts').rglob('*.sh') if p not in archived)
        paths.extend(p for p in (root/'.vscode').glob('*') if p.is_file() and not volatile_ide_file(p))
        index = subprocess.check_output(
            ['git', '-C', str(root), 'rev-parse', '--git-path', 'index'], text=True
        ).strip()
        paths.append(root/index)
    return {str(p): digest(p) for p in paths if p.exists()}


def volatile_ide_file(path):
    path = Path(path)
    return path.parent.name == '.vscode' and path.name in {
        'browse.vc.db', 'browse.vc.db-shm', 'browse.vc.db-wal', 'browse.vc.db.lock'
    }


def rewrite_markdown(text, before, after, mappings):
    def replace(match):
        target = local_target(before, match[2])
        if target is None:
            return match[0]
        target = mappings.get(target, target)
        parsed = urlsplit(match[2])
        path = str(target) if parsed.path.startswith('/') else os.path.relpath(target, after.parent)
        line_suffix = re.search(r':\d+$', parsed.path)
        if line_suffix:
            path += line_suffix[0]
        if parsed.query:
            path += '?'+parsed.query
        if parsed.fragment:
            path += '#'+parsed.fragment
        return match[1]+path+match[3]

    text = LINK.sub(replace, text)
    # Preserve old script commands through symlinks; only document paths move.
    for source, target in mappings.items():
        if source.suffix == '.md':
            text = text.replace(str(source), str(target))
            for root in ROOTS:
                if source.is_relative_to(root):
                    text = text.replace(str(source.relative_to(root)), str(target.relative_to(root)))
    return text


def launcher_text(source, original=None):
    text = source.read_text() if original is None else original
    locator = '$(dirname -- "${BASH_SOURCE[0]}")'
    resolved = '$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")'
    if text.count(locator) != 1:
        raise ValueError('Unexpected launcher path expression: '+str(source))
    if locator+'/..' in text:
        text = text.replace(locator+'/..', resolved+'/../../..')
    else:
        text = text.replace(locator, resolved+'/../..')
    warning = ("# Historical launcher; original options retained, not the VD13 entry.\n"
               "printf '[历史入口] "+source.name+"：仅供旧版本复现；当前 VD13 见 docs/README.md。\\n' >&2\n")
    first, rest = text.split('\n', 1)
    return first+'\n'+warning+rest


def apply():
    rows = plan()
    for source, target, kind in rows:
        if not source.is_file() or source.is_symlink() or target.exists():
            raise ValueError('Refusing missing, already-moved, or occupied path: '+str(source)+' -> '+str(target))
        if kind == 'launcher':
            launcher_text(source)  # Validate all substitutions before moving anything.
    archive_workspace = TRAIN/'docs/archive/workspace/TRAINING_WORKSPACE_20261105.md'
    if archive_workspace.exists():
        raise FileExistsError(archive_workspace)
    for root in ROOTS:
        for suffix in ('.json', '_before.tar.gz'):
            if (root/'docs/maintenance'/(NAME+suffix)).exists():
                raise FileExistsError('Existing audit output; this migration must not be reapplied')
    before_protected = protected_files()
    before_links = broken_links()
    originals = {p: p.read_text() for root in ROOTS for p in markdowns(root)}
    shell_text = {s: launcher_text(s) for s, _, kind in rows if kind == 'launcher'}
    for root in ROOTS:
        audit = root/'docs/maintenance'
        audit.mkdir(parents=True, exist_ok=True)
        selected = [s for s, _, _ in rows if s.is_relative_to(root)]
        with tarfile.open(audit/(NAME+'_before.tar.gz'), 'x:gz') as tar:
            for file in sorted(set([p for p in originals if p.is_relative_to(root)]+selected)):
                tar.add(file, arcname=str(file.relative_to(root)), recursive=False)
        data = dict(date='2026-11-15', status='snapshot_complete', workspace=str(root),
                    protected_sha256=before_protected, broken_markdown_before=before_links,
                    preserved=['source', 'models/assets', 'logs', 'outputs', 'checkpoints',
                               'deployment packages', 'patches', 'tests', 'IDE', 'Git index'],
                    moves=[dict(source=str(s), target=str(t), kind=k, sha256_before=digest(s),
                                mode=stat.S_IMODE(s.stat().st_mode), compatibility_symlink=k=='launcher')
                           for s,t,k in rows if s.is_relative_to(root)])
        (audit/(NAME+'.json')).write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')

    mappings = {s:t for s,t,_ in rows}
    workspace = TRAIN/'docs/TRAINING_WORKSPACE.md'
    archive_workspace.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(workspace, archive_workspace)
    originals[archive_workspace] = originals[workspace]
    for source, target, kind in rows:
        target.parent.mkdir(parents=True, exist_ok=True)
        source.rename(target)
        if kind == 'launcher':
            target.write_text(shell_text[source])
            source.symlink_to(os.path.relpath(target, source.parent))

    for old_path, original in originals.items():
        new_path = mappings.get(old_path, old_path)
        origin = workspace if old_path == archive_workspace else old_path
        updated = rewrite_markdown(original, origin, new_path, mappings)
        if old_path in mappings or old_path == archive_workspace:
            owner = next(r for r in ROOTS if new_path.is_relative_to(r))
            link = os.path.relpath(owner/'docs/README.md', new_path.parent)
            updated = ('> 历史归档：内容和命令对应原记录日期，不是当前VD13操作指令。'
                       f'当前状态见[文档入口]({link})。\n\n'+updated)
        if new_path.read_text() != updated:
            new_path.write_text(updated)
    verify()


def verify():
    path = DEPLOY/'docs/maintenance'/(NAME+'.json')
    data = json.loads(path.read_text())
    # Live C/C++ symbol databases are IDE caches, not configuration. Retain their
    # initial hashes in the audit, but never rewind/delete them or fail a source
    # invariant simply because the editor updated its own index during cleanup.
    excluded = [p for p in data['protected_sha256'] if volatile_ide_file(p)]
    changed = [p for p,h in data['protected_sha256'].items() if p not in excluded
               and (not Path(p).is_file() or digest(Path(p)) != h)]
    moved = {str(s):str(t) for s,t,_ in plan()}
    baseline = {(moved.get(p,p), moved.get(t,t)) for p,t in data['broken_markdown_before']}
    workspace = str(TRAIN/'docs/TRAINING_WORKSPACE.md')
    archived_workspace = str(TRAIN/'docs/archive/workspace/TRAINING_WORKSPACE_20261105.md')
    baseline.update((archived_workspace, moved.get(t,t))
                    for p,t in data['broken_markdown_before'] if p == workspace)
    broken = broken_links()
    added = sorted(set(broken)-baseline)
    links = []
    shell_errors = []
    for root in ROOTS:
        with tarfile.open(root/'docs/maintenance'/(NAME+'_before.tar.gz')) as snapshot:
            for source, target, kind in plan():
                if kind != 'launcher' or not source.is_relative_to(root):
                    continue
                member = snapshot.getmember(str(source.relative_to(root)))
                original = snapshot.extractfile(member).read().decode()
                if target.read_text() != launcher_text(source, original):
                    shell_errors.append(str(target)+': non-mechanical change')
                if stat.S_IMODE(target.stat().st_mode) != member.mode:
                    shell_errors.append(str(target)+': permission changed')
                for location in (source, target):
                    result = subprocess.run(['bash', '-n', str(location)], capture_output=True, text=True)
                    if result.returncode:
                        shell_errors.append(str(location)+': '+result.stderr)
    for source, target, kind in plan():
        if kind == 'launcher' and (not source.is_symlink() or source.resolve() != target):
            links.append(str(source))
    for root in ROOTS:
        manifest = root/'docs/maintenance'/(NAME+'.json')
        record = json.loads(manifest.read_text())
        record.update(status='verified' if not (changed or added or links or shell_errors) else 'verification_failed',
                      protected_changed=changed, compatibility_link_errors=links,
                      protected_static_files=len(data['protected_sha256'])-len(excluded),
                      volatile_ide_excluded=excluded, launcher_errors=shell_errors,
                      new_broken_markdown=added, broken_markdown_after=broken)
        for move in record['moves']:
            move['sha256_after'] = digest(Path(move['target']))
        manifest.write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(protected_files=len(data['protected_sha256'])-len(excluded), protected_changed=changed,
                          volatile_ide_excluded=excluded, launcher_errors=shell_errors,
                          compatibility_link_errors=links, new_broken_markdown=added,
                          existing_broken_markdown=len(broken), moves=len(plan())), ensure_ascii=False, indent=2))
    if changed or added or links or shell_errors:
        raise SystemExit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    if args.apply and args.verify:
        parser.error('Choose apply or verify')
    if args.apply:
        apply()
    elif args.verify:
        verify()
    else:
        print(json.dumps([dict(source=str(s), target=str(t), kind=k) for s,t,k in plan()], indent=2))
