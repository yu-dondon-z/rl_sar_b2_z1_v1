"""Dry-run launcher regressions; never start vision, simulation, or hardware."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = DEPLOY_ROOT / "scripts/play_vd1_mujoco.py"


class VD1MujocoLauncherTests(unittest.TestCase):
    def setUp(self):
        self.output = tempfile.TemporaryDirectory(prefix="vd1_launcher_test_", dir="/dev/shm")
        self.addCleanup(self.output.cleanup)

    def dry_run(self, *arguments, success=True, launcher=LAUNCHER, explicit_interpreters=True):
        # Existing Python executables satisfy the dry-run's path checks. They
        # are recorded as command arguments but are never launched by it.
        interpreters = (["--mujoco-python", sys.executable, "--vision-python", sys.executable]
                        if explicit_interpreters else [])
        result = subprocess.run(
            [sys.executable, str(launcher), "--dry-run", *interpreters,
             "--output-root", self.output.name, *arguments],
            cwd=str(DEPLOY_ROOT), capture_output=True, text=True, timeout=20,
        )
        if not success:
            self.assertNotEqual(result.returncode, 0, result.stdout)
            return result
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        sessions = list(Path(self.output.name).glob("session_*/session.json"))
        self.assertEqual(len(sessions), 1, result.stdout)
        session = json.loads(sessions[0].read_text(encoding="utf-8"))
        printed = json.loads(result.stdout[result.stdout.index("{"):])
        self.assertEqual(session["state"], "dry_run")
        self.assertFalse(session["hardware_started"])
        self.assertNotIn("play_pid", session)
        self.assertNotIn("vision_pid", session)
        self.assertEqual(printed["play"], session["play"])
        self.assertEqual(printed["vision"], session["vision"])
        return session

    def option(self, command, name):
        return command[command.index(name) + 1]

    def assert_speed_and_curve(self, session, speed, stage, curvature):
        self.assertEqual(session["forward_speed_mps"], speed)
        self.assertEqual(session["curvature_stage"], stage)
        self.assertEqual(session["maximum_curvature_per_m"], curvature)
        self.assertEqual(float(self.option(session["play"], "--speed")), speed)
        self.assertEqual(float(self.option(session["play"], "--curvature-max")), curvature)

    def test_defaults_request_point_four_and_gentle_curve_with_windows(self):
        session = self.dry_run()
        self.assert_speed_and_curve(session, 0.4, 1, 0.05)
        self.assertNotIn("--headless", session["play"])
        self.assertIn("--display", session["vision"])
        self.assertEqual(session["external_sensors"], ["front_rgb", "rear_rgbd"])
        self.assertFalse(session["critic_used"])

    def test_vd3_visual_deployment_selects_reviewed_mirror_2000(self):
        session = self.dry_run('--robot-version', 'vd3')
        self.assertEqual(Path(session['policy']), DEPLOY_ROOT /
                         'policy/b2_z1_no_gun/mujoco_vd3_mirror_2000/policy.pt')
        self.assertEqual(session['policy_sha256'],
                         '51eda90b2b77204ef66d949523486250e88c7062aeb1e86f0dc5478924291c78')
        self.assertEqual(self.option(session['play'], '--policy'), session['policy'])
        self.assertEqual(self.option(session['play'], '--robot-version'), 'vd3')

    def test_vd3_debug_opens_both_windows_with_reviewed_physics(self):
        session = self.dry_run('--show', '--max-steps', '0',
                               explicit_interpreters=False,
                               launcher=DEPLOY_ROOT/'scripts/debug_vd3_rear_vision.py')
        self.assert_speed_and_curve(session, 0.2, 4, 0.40)
        self.assertNotIn('--headless', session['play'])
        self.assertIn('--display', session['vision'])
        self.assertEqual(self.option(session['play'], '--max-steps'), '0')
        self.assertEqual(self.option(session['play'], '--velocity-constraint'), 'impulse')
        self.assertEqual(float(self.option(session['play'], '--physics-dt')), 0.005)
        self.assertIn('--physx-implicit-position', session['play'])
        self.assertEqual(session['external_sensors'], ['front_rgb', 'rear_rgbd'])
        self.assertEqual(session['tool_target_mode'], 'rear_seam_direct')
        self.assertFalse(session['visual_tcp_feedback'])

    def test_vd3_rejects_a_different_actor_before_starting(self):
        fake = Path(self.output.name) / 'wrong_actor.pt'
        fake.write_bytes(b'not the reviewed actor')
        result = self.dry_run('--robot-version', 'vd3', '--policy', str(fake), success=False)
        self.assertIn('SHA256 differs from the verified VD3 export', result.stderr)
        self.assertEqual(list(Path(self.output.name).glob('session_*')), [])

    def test_explicit_point_four_and_gentle_curve_are_forwarded(self):
        session = self.dry_run("--speed", "0.40", "--curvature-stage", "1")
        self.assert_speed_and_curve(session, 0.4, 1, 0.05)

    def test_old_point_two_straight_baseline_remains_available(self):
        session = self.dry_run("--speed", "0.20", "--curvature-stage", "0")
        self.assert_speed_and_curve(session, 0.2, 0, 0.0)

    def test_front_estimation_selects_independent_projection_and_same_actor(self):
        session = self.dry_run('--robot-version', 'vd3', '--visual-controller', 'guarded',
                               '--front-path-estimation', '--front-nominal-height-m', '.55')
        self.assertTrue(session['front_path_estimation'])
        self.assertFalse(session['body_motion_estimation'])
        self.assertIn('--front-path-estimation', session['play'])
        self.assertEqual(float(self.option(session['vision'], '--front-nominal-height-m')), .55)
        self.assertEqual(session['base_guidance_source'], 'front_rgb_curve_estimator')
        self.assertEqual(session['tool_target_mode'], 'rear_seam_direct')

    def test_front_estimation_rejects_incompatible_modes(self):
        for options in (['--front-path-estimation'],
                        ['--visual-controller', 'guarded', '--front-path-estimation', '--front-only-guidance'],
                        ['--visual-controller', 'guarded', '--front-path-estimation', '--front-nominal-height-m', 'nan']):
            self.dry_run(*options, success=False)

    def test_dual_debug_requests_point_three_curvature_and_zero_gun_mass(self):
        session = self.dry_run(launcher=DEPLOY_ROOT/'scripts/debug_vd1_dual_vision.py')
        self.assert_speed_and_curve(session, 0.4, 5, 0.3)
        self.assertEqual(session['gun_mass_mode'], 'zero')
        self.assertTrue(session['gun_mass_experiment'])
        self.assertEqual(self.option(session['play'], '--gun-mass-mode'), 'zero')
        self.assertEqual(self.option(session['play'], '--road-contact'), 'heightfield')
        self.assertEqual(self.option(session['play'], '--road-surface'), 'tape')
        self.assertEqual(session['road_surface'], 'tape')
        self.assertEqual(session['road_contact'], 'flat_plane')
        self.assertEqual(self.option(session['play'], '--visual-controller'), 'guarded')
        self.assertEqual(Path(session['vision'][1]).name, 'run_vd1_dual_camera_bridge.py')
        self.assertEqual(session['front_component_selection'], 'longest_measured_forward_support')
        self.assertEqual(session['tool_target_mode'], 'rear_seam_direct')
        self.assertFalse(session['visual_tcp_feedback'])
        self.assertFalse(session['requires_tcp_markers'])
        self.assertFalse(session['tool_markers_in_scene'])

    def test_original_gun_mass_can_be_selected_for_comparison(self):
        session = self.dry_run('--gun-mass-mode', 'original',
                               launcher=DEPLOY_ROOT/'scripts/debug_vd1_dual_vision.py')
        self.assertEqual(session['gun_mass_mode'], 'original')
        self.assertFalse(session['gun_mass_experiment'])
        self.assertEqual(self.option(session['play'], '--gun-mass-mode'), 'original')

    def test_previous_dual_debug_curvature_can_be_restored(self):
        session = self.dry_run('--curvature-stage', '2',
                               launcher=DEPLOY_ROOT/'scripts/debug_vd1_dual_vision.py')
        self.assert_speed_and_curve(session, 0.4, 2, 0.1)

    def test_physical_crack_can_be_selected_explicitly(self):
        session = self.dry_run('--road-surface', 'crack',
                               launcher=DEPLOY_ROOT/'scripts/debug_vd1_dual_vision.py')
        self.assertEqual(session['road_surface'], 'crack')
        self.assertEqual(session['road_contact'], 'heightfield')
        self.assertEqual(self.option(session['play'], '--road-surface'), 'crack')

    def test_physics_defaults_are_unchanged(self):
        session = self.dry_run()
        self.assertEqual(session['gun_mass_mode'], 'original')
        self.assertFalse(session['gun_mass_experiment'])
        self.assertEqual(session["velocity_constraint"], "off")
        self.assertFalse(session["velocity_constraint_experimental"])
        self.assertEqual(self.option(session["play"], "--velocity-constraint"), "off")
        self.assertEqual(session["physics_timestep_s"], 0.001)
        self.assertEqual(float(self.option(session["play"], "--physics-dt")), 0.001)
        self.assertEqual(session["leg_pd_period_s"], 0.005)
        self.assertEqual(session["actor_period_s"], 0.02)
        self.assertEqual(session["physics_substeps_per_pd"], 5)
        self.assertFalse(session["physx_implicit_position"])
        self.assertNotIn("--physx-implicit-position", session["play"])

    def test_previous_explicit_physics_options_are_still_forwarded(self):
        session = self.dry_run(
            "--velocity-constraint", "impulse", "--physics-dt", "0.005",
            "--physx-implicit-position",
        )
        self.assertEqual(self.option(session["play"], "--velocity-constraint"), "impulse")
        self.assertEqual(session["velocity_constraint"], "impulse")
        self.assertTrue(session["velocity_constraint_experimental"])
        self.assertEqual(float(self.option(session["play"], "--physics-dt")), 0.005)
        self.assertEqual(session["physics_timestep_s"], 0.005)
        self.assertEqual(session["physics_substeps_per_pd"], 1)
        self.assertIn("--physx-implicit-position", session["play"])
        self.assertTrue(session["physx_implicit_position"])

    def test_road_contact_and_guidance_ablation_reach_runner(self):
        session = self.dry_run('--road-contact', 'heightfield', '--visual-controller', 'guarded',
                               '--front-only-guidance')
        self.assertEqual(session['road_contact'], 'heightfield')
        self.assertTrue(session['front_only_guidance'])
        self.assertEqual(self.option(session['play'], '--road-contact'), 'heightfield')
        self.assertIn('--front-only-guidance', session['play'])

    def test_invalid_speed_is_rejected_before_creating_a_session(self):
        for value in ("0", "-0.1", "nan", "inf", "1.01"):
            with self.subTest(speed=value):
                result = self.dry_run("--speed=" + value, success=False)
                self.assertIn("--speed must be finite and within (0, 1] m/s", result.stderr)
                self.assertEqual(list(Path(self.output.name).iterdir()), [])


if __name__ == "__main__":
    unittest.main()
