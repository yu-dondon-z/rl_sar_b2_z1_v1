"""Explicit mount75 backend; does not mutate the historical VD12 backend."""
import play_vd12_mujoco as legacy
from vd12_geometry_guard import ASSETS, load_pure_module, MujocoGunGeometryGuard as BaseGuard
from vd1_mujoco_scene import VD13_URDF_SHA256, VD10_TCP_ROTATION, validate_vd13_model

evaluation = load_pure_module('vd13_evaluation_contract',
    ASSETS.parents[3]/'scripts/diagnostics/vd13_play_contract.py')
DEFAULT_CHECKPOINT = evaluation.DEFAULT_CHECKPOINT
URDF = evaluation.URDF
URDF_SHA256 = VD13_URDF_SHA256
TCP_POSITION, TCP_ROTATION = evaluation.TCP_POSITION, VD10_TCP_ROTATION
ROBOT_VERSION, WORK_DISTANCE_M = 'vd13', evaluation.WORK_DISTANCE_M
shared = legacy.shared
load_yaml = legacy.load_yaml
processed_joint_target = legacy.processed_joint_target
GeometryLatch = legacy.GeometryLatch


def validate_checkpoint(checkpoint, agent, saved, profile):
    evaluation.validate_mount_checkpoint(checkpoint, agent, saved, profile)
    return legacy.validate_checkpoint(checkpoint, agent, saved, profile,
        expected=evaluation.EXPERIMENT, startup_profile=evaluation.STARTUP_PROFILE,
        work_distance=WORK_DISTANCE_M)


def validate_saved_settings(saved):
    result = legacy.validate_saved_settings(saved, command_class='VD13SafeSeamCommand')
    result.update(mount_profile_checked=True, tcp_position_m=list(TCP_POSITION),
                  work_distance_m=WORK_DISTANCE_M)
    return result


def validate_model(model):
    return validate_vd13_model(model)


class MujocoGunGeometryGuard(BaseGuard):
    def __init__(self, model):
        super().__init__(model, urdf_path=URDF)
