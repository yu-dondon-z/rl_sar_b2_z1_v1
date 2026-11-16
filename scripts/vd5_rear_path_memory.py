"""Short rear path memory using sent commands, never simulator motion/road truth.

This is command dead reckoning for a bounded path buffer, not measured odometry.
Speed tracking error and slip therefore remain sources of error.
"""
from collections import deque
import math

import numpy as np


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array(((c, -s), (s, c)))


class RearPathMemory:
    def __init__(self, horizon_s):
        self.horizon_s = float(horizon_s)
        if not math.isfinite(self.horizon_s) or not 0. < self.horizon_s <= .8:
            raise ValueError('rear path memory must be in (0, 0.8] seconds')
        self.motion = deque()
        self.pose = np.zeros(3)
        self.frames = deque()
        self.identity = None
        self.last_stamp = None

    def advance(self, now, dt, sent_command):
        """Integrate the command sent during the preceding control interval.

        dt is the control interval; wall timestamps only associate captured frames
        with this ledger. Slow rendering does not count as extra robot motion.
        """
        if self.motion and now <= self.motion[-1][0]:
            raise ValueError('rear memory control clock moved backwards')
        if not self.motion:
            self.motion.append((now-dt, self.pose.copy()))
        command = np.asarray(sent_command, dtype=float)
        yaw_delta = float(command[2]*dt)
        if abs(command[2]) < 1.e-9:
            local_delta = command[:2]*dt
        else:
            a = math.sin(yaw_delta)/command[2]
            b = (1.-math.cos(yaw_delta))/command[2]
            local_delta = np.array(((a, -b), (b, a))) @ command[:2]
        self.pose[:2] += rotation(self.pose[2]) @ local_delta
        self.pose[2] += yaw_delta
        self.motion.append((now, self.pose.copy()))
        while len(self.motion) > 2 and self.motion[1][0] < now-2.:
            self.motion.popleft()
        self._expire(now)

    def _expire(self, now):
        self.frames = deque(frame for frame in self.frames
                            if now-frame['stamp'] <= self.horizon_s)

    def _pose_at(self, stamp):
        if not self.motion or stamp < self.motion[0][0] or stamp > self.motion[-1][0]+.002:
            raise ValueError('rear capture has no command-motion history')
        for (t0, p0), (t1, p1) in zip(self.motion, list(self.motion)[1:]):
            if t0 <= stamp <= t1:
                return p0+(p1-p0)*(stamp-t0)/(t1-t0)
        return self.pose.copy()

    def ingest(self, points, plane, basis, generation, sequence, stamp, now):
        identity = (generation, sequence)
        if self.identity is not None:
            if identity < self.identity:
                raise ValueError('rear path memory capture identity moved backwards')
            if identity == self.identity:
                if stamp != self.last_stamp:
                    raise ValueError('rear path memory repeated frame changed timestamp')
                return
            if generation != self.identity[0]:
                self.frames.clear()
        self.identity, self.last_stamp = identity, stamp
        if now-stamp > self.horizon_s:
            return
        try:
            pose = self._pose_at(stamp)
        except ValueError:
            # The first frame can predate actor startup. It may initialize the
            # old direct controller but must not be assigned a made-up pose.
            return
        points = np.asarray(points, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 6
                or not np.isfinite(points).all()):
            raise ValueError('rear memory requires measured xyz points')
        origin = -plane[3]*plane[:3]
        xy = (points-origin) @ basis[:, :2]
        if np.ptp(xy[:, 0]) < .10 or np.max(np.abs(xy[:, 1])) > .35:
            raise ValueError('rear memory visible support is short or outside working envelope')
        stored = xy @ rotation(pose[2]).T+pose[:2]
        # Disagreeing overlapping observations must not leave two parallel
        # paths to be averaged. A new measurement replaces conflicting history.
        for frame in self.frames:
            old = (frame['points']-pose[:2]) @ rotation(pose[2])
            order = np.argsort(old[:, 0])
            overlap = (xy[:, 0] >= old[:, 0].min()) & (xy[:, 0] <= old[:, 0].max())
            if np.count_nonzero(overlap) >= 6:
                residual = xy[overlap, 1]-np.interp(xy[overlap, 0], old[order, 0], old[order, 1])
                if np.median(np.abs(residual)) > .025:
                    self.frames.clear()
                    break
        self.frames.append(dict(points=stored, pose=pose, stamp=float(stamp), sequence=int(sequence)))
        self._expire(now)

    def paths(self, plane, basis, now):
        """Return continuous measured support in the present nominal base frame.

        No extrapolation and no bridging gaps over 2.5 cm. The current depth
        plane supplies height and tilt; only planar command motion is buffered.
        """
        self._expire(now)
        points, sources = [], []
        for frame in self.frames:
            displacement = float(np.linalg.norm(self.pose[:2]-frame['pose'][:2]))
            if displacement > .16 or abs(self.pose[2]-frame['pose'][2]) > .15:
                continue
            xy = (frame['points']-self.pose[:2]) @ rotation(self.pose[2])
            points.extend(xy)
            sources.extend([(frame['stamp'], frame['sequence'], displacement)]*len(xy))
        if len(points) < 6:
            return []
        points = np.asarray(points)
        sources = np.asarray(sources)
        bins = np.floor(points[:, 0]/.005).astype(int)
        grouped, metadata = [], []
        for key in np.unique(bins):
            selected = bins == key
            yy = points[selected, 1]
            if np.ptp(yy) > .03:
                continue
            grouped.append(np.median(points[selected], axis=0))
            metadata.append(sources[selected])
        if len(grouped) < 6:
            return []
        grouped = np.asarray(grouped)
        lengths = np.linalg.norm(np.diff(grouped, axis=0), axis=1)
        cuts = np.flatnonzero(lengths > .025)+1
        origin = -plane[3]*plane[:3]
        paths = []
        for indices in np.split(np.arange(len(grouped)), cuts):
            if len(indices) < 6 or np.ptp(grouped[indices, 0]) < .12:
                continue
            source = np.concatenate([metadata[i] for i in indices])
            xyz = origin+grouped[indices] @ basis[:, :2].T
            paths.append((xyz, dict(
                rear_memory_source_age_max_s=float(now-source[:, 0].min()),
                rear_memory_source_age_min_s=float(now-source[:, 0].max()),
                rear_memory_source_capture_timestamp_min=float(source[:, 0].min()),
                rear_memory_source_sequences=sorted(set(source[:, 1].astype(int).tolist())),
                rear_memory_command_displacement_max_m=float(source[:, 2].max()),
                rear_memory_point_count=len(indices),
                rear_memory_motion_source='sent_base_commands_only',
                rear_memory_max_age_s=self.horizon_s)))
        return paths
