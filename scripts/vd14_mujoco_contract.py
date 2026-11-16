"""VD14 policy identity on the unchanged, validated VD13 physical model."""
import play_vd12_mujoco as legacy
import vd13_mujoco_contract as mount_backend
from vd12_geometry_guard import ASSETS, load_pure_module

evaluation = load_pure_module('vd14_evaluation_contract',
    ASSETS.parents[3] / 'scripts/diagnostics/vd14_play_contract.py')
DEFAULT_CHECKPOINT = evaluation.DEFAULT_CHECKPOINT
URDF, URDF_SHA256 = mount_backend.URDF, mount_backend.URDF_SHA256
TCP_POSITION, TCP_ROTATION = mount_backend.TCP_POSITION, mount_backend.TCP_ROTATION
ROBOT_VERSION, WORK_DISTANCE_M = 'vd13', mount_backend.WORK_DISTANCE_M
shared, load_yaml = legacy.shared, legacy.load_yaml
processed_joint_target, GeometryLatch = legacy.processed_joint_target, legacy.GeometryLatch
MujocoGunGeometryGuard, validate_model = mount_backend.MujocoGunGeometryGuard, mount_backend.validate_model


def validate_checkpoint(checkpoint, agent, saved, profile):
    evaluation.validate_posture_checkpoint(checkpoint, agent, saved, profile)
    return legacy.validate_checkpoint(checkpoint, agent, saved, profile,
        expected=evaluation.EXPERIMENT, startup_profile=evaluation.STARTUP_PROFILE,
        work_distance=WORK_DISTANCE_M)


def validate_saved_settings(saved):
    result = mount_backend.validate_saved_settings(saved)
    result.update(posture_profile_checked=True)
    return result
