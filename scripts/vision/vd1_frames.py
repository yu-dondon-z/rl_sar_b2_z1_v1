"""Pose-free paired camera publication for the VD1 image-only controller.

The input RGB arrays use RGB channel order, and rear depth is aligned optical
Z in metres.  Calibration describes fixed camera mounting only.  A private
capture history binds returned measurements to frames actually published by
this instance; reading a plausible JSON file alone is insufficient.
"""

from __future__ import annotations

from collections import OrderedDict
import json
import math
import os
from pathlib import Path
import time

import cv2
import numpy as np

from vd1_control import SCHEMA, _check_payload


FRAME_SCHEMA = "b2_z1_dual_camera_frame_v1"


def _finite(value, shape):
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError("invalid fixed camera calibration")
    return array


def _calibration(value):
    if not isinstance(value, dict):
        raise ValueError("camera calibration must be a dictionary")
    _check_payload(value)
    position = _finite(value.get("position"), (3,))
    quaternion = _finite(value.get("quaternion_wxyz"), (4,))
    intrinsic = _finite(value.get("intrinsic_matrix"), (3, 3))
    if abs(float(np.linalg.norm(quaternion)) - 1) > 1.e-3:
        raise ValueError("camera mounting quaternion must be unit wxyz")
    if intrinsic[0, 0] <= 0 or intrinsic[1, 1] <= 0 or not np.allclose(intrinsic[2], [0, 0, 1]):
        raise ValueError("invalid camera intrinsic matrix")
    return {"position": position.tolist(), "quaternion_wxyz": quaternion.tolist(),
            "intrinsic_matrix": intrinsic.tolist()}


def _markers(value):
    if value is None:
        value = {}
    if not isinstance(value, dict):
        raise ValueError("marker calibration must be a dictionary")
    _check_payload(value)
    defaults = {"marker_radius_m": .035, "marker_axial_spacing_m": .10,
                "marker_tip_extension_m": .10}
    aliases = {"radius_m": "marker_radius_m", "axial_spacing_m": "marker_axial_spacing_m",
               "tip_extension_m": "marker_tip_extension_m"}
    for name, item in value.items():
        key = aliases.get(name, name)
        if key not in defaults:
            raise ValueError(f"unknown fixed marker calibration: {name}")
        number = float(item)
        minimum_valid = number >= 0 if key == "marker_tip_extension_m" else number > 0
        if not math.isfinite(number) or not minimum_valid or number > 1:
            raise ValueError("marker radius/spacing must be positive; tip extension may be zero")
        defaults[key] = number
    return defaults


def _atomic_json(path, payload):
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, allow_nan=False), encoding="utf-8")
    os.replace(temporary, path)


def _atomic_image(path, array):
    temporary = path.with_name(f".{path.stem}.{os.getpid()}.tmp.png")
    if not cv2.imwrite(str(temporary), array):
        raise RuntimeError(f"cannot publish camera image: {path}")
    os.replace(temporary, path)


class VD1FramePublisher:
    """Publish only pairs for which both renderer frame counters advanced.

    ``validate_packet`` returns ``(accepted, reason)`` and checks provenance,
    not image-detector validity or age.  The visual controller owns those
    checks.  Construct a new publisher on simulation reset so that its new
    generation also invalidates in-flight frames from the previous episode.
    """

    def __init__(self, exchange, front_calibration, rear_calibration,
                 marker_calibration, generation=None, slots=8):
        self.exchange = Path(exchange).expanduser().resolve()
        self.front_calibration = _calibration(front_calibration)
        self.rear_calibration = _calibration(rear_calibration)
        self.marker_calibration = _markers(marker_calibration)
        self.generation = time.time_ns() if generation is None else generation
        if type(self.generation) is not int or self.generation < 0:
            raise ValueError("capture generation must be a nonnegative integer")
        if type(slots) is not int or slots < 2:
            raise ValueError("at least two camera image slots are required")
        self.slots = slots
        self._sequence = 0
        self._last_frames = (-1, -1)
        self._last_timestamp = None
        self._captures = OrderedDict()
        for stream in ("front", "rear"):
            (self.exchange / stream).mkdir(parents=True, exist_ok=True)

    def publish(self, front_rgb, rear_rgb, rear_depth_m, *, front_frame, rear_frame,
                capture_timestamp, sim_time_s, front_gravity_b=None,
                rear_arm_joint_positions=None):
        if any(type(frame) is not int or frame < 0 for frame in (front_frame, rear_frame)):
            raise ValueError("renderer frame counters must be nonnegative integers")
        if front_frame <= self._last_frames[0] or rear_frame <= self._last_frames[1]:
            return False
        capture_timestamp, sim_time_s = float(capture_timestamp), float(sim_time_s)
        if (not math.isfinite(capture_timestamp) or not math.isfinite(sim_time_s)
                or sim_time_s < 0 or capture_timestamp < 0):
            raise ValueError("camera clocks must be finite and nonnegative")
        if self._last_timestamp is not None and capture_timestamp < self._last_timestamp:
            raise ValueError("camera capture clock moved backwards")
        if front_gravity_b is not None:
            front_gravity_b = _finite(front_gravity_b, (3,))
            if not .9 <= np.linalg.norm(front_gravity_b) <= 1.1:
                raise ValueError('front IMU gravity must be a unit vector')
        if rear_arm_joint_positions is not None:
            rear_arm_joint_positions = _finite(rear_arm_joint_positions, (6,))
        images = {}
        for name, image in (("front", front_rgb), ("rear", rear_rgb)):
            image = np.asarray(image)
            if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or min(image.shape[:2]) < 1:
                raise ValueError("camera RGB must be a nonempty HxWx3 uint8 RGB array")
            images[name] = image
        depth = np.asarray(rear_depth_m, dtype=np.float64)
        if depth.shape != images["rear"].shape[:2]:
            raise ValueError("rear depth must be aligned to rear RGB")
        millimetres = np.zeros(depth.shape, dtype=np.uint16)
        measured = np.isfinite(depth) & (depth > 0)
        millimetres[measured] = np.rint(np.clip(depth[measured] * 1000., 0, 65535)).astype(np.uint16)
        sequence, slot = self._sequence, self._sequence % self.slots
        captures = {}
        for stream, frame, calibration in (("front", front_frame, self.front_calibration),
                                           ("rear", rear_frame, self.rear_calibration)):
            directory = self.exchange / stream
            rgb_name = f"rgb_slot_{slot}.png"
            _atomic_image(directory / rgb_name, cv2.cvtColor(images[stream], cv2.COLOR_RGB2BGR))
            metadata = {
                "schema": FRAME_SCHEMA, "frame_id": "b2_base", "stream": stream,
                "camera_name": "front_rgb" if stream == "front" else "rear_d435i",
                "sequence": sequence, "camera_frame": frame, "capture_generation": self.generation,
                "capture_timestamp": capture_timestamp, "capture_timestamp_basis": "host_receive_monotonic",
                "capture_sim_time_s": sim_time_s, "timestamp_ns": time.time_ns(),
                "rgb_file": rgb_name,
                "image_size": [int(images[stream].shape[1]), int(images[stream].shape[0])],
                "intrinsic_matrix": calibration["intrinsic_matrix"],
                "camera_to_base": {"position": calibration["position"],
                                   "quaternion_wxyz": calibration["quaternion_wxyz"]},
            }
            if stream == "rear":
                depth_name = f"depth_slot_{slot}_mm.png"
                _atomic_image(directory / depth_name, millimetres)
                metadata.update(depth_file=depth_name, depth_unit_m=.001, depth_aligned_to_rgb=True,
                                **self.marker_calibration)
                if rear_arm_joint_positions is not None:
                    metadata.update(arm_joint_positions_rad=rear_arm_joint_positions.tolist(),
                                    arm_joint_names=['joint%d' % i for i in range(1, 7)],
                                    arm_encoder_timestamp=capture_timestamp)
            elif front_gravity_b is not None:
                metadata['imu_gravity_b'] = front_gravity_b.tolist()
            captures[stream] = metadata
        # Publish metadata after both streams' images are complete.  A reader
        # between these two atomic updates sees unequal sequence IDs and waits.
        for stream, metadata in captures.items():
            directory = self.exchange / stream
            _atomic_json(directory / f"frame_slot_{slot}.json", metadata)
            _atomic_json(directory / "latest_frame.json", metadata)
        self._captures[sequence] = {
            "generation": self.generation,
            "front_timestamp": captures["front"]["capture_timestamp"],
            "rear_timestamp": captures["rear"]["capture_timestamp"],
        }
        while len(self._captures) > 128:
            self._captures.popitem(last=False)
        self._last_frames = (front_frame, rear_frame)
        self._last_timestamp = capture_timestamp
        self._sequence += 1
        return True

    def validate_packet(self, packet):
        """Check capture identity against private history, never file metadata."""
        try:
            if not isinstance(packet, dict):
                raise ValueError("missing paired visual packet")
            _check_payload(packet)
            if packet.get("schema") != SCHEMA or packet.get("capture_generation") != self.generation:
                raise ValueError("wrong packet schema or capture generation")
            sequences = []
            for stream in ("front", "rear"):
                observation = packet.get(stream)
                if not isinstance(observation, dict):
                    raise ValueError(f"missing {stream} capture identity")
                sequence = observation.get("sequence")
                if type(sequence) is not int or sequence not in self._captures:
                    raise ValueError("capture was not published or has left the history")
                if observation.get("capture_generation") != self.generation:
                    raise ValueError("capture belongs to another generation")
                timestamp = observation.get("capture_timestamp")
                if (not isinstance(timestamp, (int, float)) or isinstance(timestamp, bool)
                        or not math.isfinite(timestamp)):
                    raise ValueError("capture timestamp must be finite")
                expected = self._captures[sequence][f"{stream}_timestamp"]
                if abs(timestamp - expected) > 1.e-6:
                    raise ValueError("capture timestamp differs from the published frame")
                sequences.append(sequence)
            if sequences[0] != sequences[1]:
                raise ValueError("front/rear captures are not a published pair")
            return True, ""
        except (KeyError, TypeError, ValueError) as error:
            return False, str(error)
