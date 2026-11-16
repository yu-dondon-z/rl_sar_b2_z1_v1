#!/usr/bin/env python3
"""Compare deployment FK to checkpoint's archived URDF, without Isaac/robot I/O."""
import hashlib
import json
from pathlib import Path
import tarfile
import xml.etree.ElementTree as ET

import numpy as np

from analyze_recorded_tcp import ARM, fk


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3)*np.cos(angle)+(1.-np.cos(angle))*np.outer(axis, axis)+np.sin(angle)*cross


def main():
    archive = Path('/home/dong/b2_z1/logs/rsl_rl/b2_z1_amp/2026-09-28_15-07-31_va15_curriculum_from2500/source.tar.gz')
    entry = 'source/b2_z1/b2_z1/assets/b2_z1/b2_z1_no_stick.urdf'
    with tarfile.open(archive) as source:
        urdf = source.extractfile(entry).read()
    root = ET.fromstring(urdf)
    parents = {j.find('child').get('link'):j for j in root.findall('joint')}
    chain = []
    link = 'dummy_target'
    while link != 'base_link':
        joint = parents[link]
        chain.insert(0, joint)
        link = joint.find('parent').get('link')
    mapping = {f'joint{i+1}':index for i, index in enumerate(ARM)}
    rng = np.random.default_rng(256)
    q = np.zeros((1000, 18))
    for j in chain:
        if j.get('type') != 'fixed':
            limits = j.find('limit')
            q[:, mapping[j.get('name')]] = rng.uniform(float(limits.get('lower')), float(limits.get('upper')), len(q))
    archive_positions = []
    for sample in q:
        p, r = np.zeros(3), np.eye(3)
        for j in chain:
            origin = j.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            roll, pitch, yaw = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
            p += r @ xyz
            r = r @ rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll)
            if j.get('type') != 'fixed':
                axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
                r = r @ rotation(axis, sample[mapping[j.get('name')]])
        archive_positions.append(p)
    error = np.linalg.norm(np.array(archive_positions)-fk(q), axis=1)
    report = dict(schema='va15_archived_tcp_geometry_v1', archive=str(archive), entry=entry,
                  archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
                  urdf_sha256=hashlib.sha256(urdf).hexdigest(), poses=len(q),
                  maximum_position_difference_m=float(error.max()), tolerance_m=1.e-6,
                  passed=bool(error.max()<1.e-6), reference='base_link -> dummy_target',
                  physical_tool_calibration=False, hardware_qualified=False)
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
