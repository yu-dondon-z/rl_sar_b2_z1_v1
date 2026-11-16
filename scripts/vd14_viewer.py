"""Spectator-only model: long drawing range without changing robot cameras."""
from pathlib import Path

import mujoco


class SpectatorState:
    def __init__(self, scene_path, source_data, *, far_m=250.):
        self.model = mujoco.MjModel.from_xml_path(str(Path(scene_path)))
        self.model.vis.map.zfar = far_m/self.model.stat.extent
        self.data = mujoco.MjData(self.model)
        self.update(source_data)

    def update(self, source_data):
        # This model is for viewing ONLY. No forces, contacts, sensors, UI edits
        # or state ever return from it to physics, perception or the actor.
        for name in ('qpos', 'qvel', 'act', 'mocap_pos', 'mocap_quat'):
            getattr(self.data, name)[:] = getattr(source_data, name)
        self.data.time = source_data.time
        mujoco.mj_forward(self.model, self.data)
