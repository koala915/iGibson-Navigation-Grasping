#!/usr/bin/env python3
"""Offline/static tests for the v24 candidate package; no model loading or hardware."""

import contextlib
import hashlib
import importlib.util
import io
import json
import sys
import unittest
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
V23_SCRIPT = ROOT / "grasp" / "v23" / "x3plus_real_grasp.py"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    # dataclasses resolves postponed annotations through sys.modules while the
    # class decorator runs, matching normal import machinery.
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(name, None)
        raise
    return module


class CandidatePackageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = json.loads((HERE / "manifest.json").read_text(encoding="utf-8"))
        cls.launcher = load_module("v24_candidate_launcher", HERE / "run_candidate.py")

        v23_dir = str(V23_SCRIPT.parent)
        sys.path.insert(0, v23_dir)
        try:
            cls.runtime = load_module("v24_test_v23_runtime", V23_SCRIPT)
        finally:
            sys.path.remove(v23_dir)

    def test_artifact_hashes_match_manifest(self):
        for key in ("model", "vecnormalize"):
            record = self.manifest["artifacts"][key]
            self.assertEqual(record["sha256"], sha256(HERE / record["file"]))

    def test_contract_home_and_envelope_are_e1(self):
        contract = self.manifest["contract"]
        self.assertEqual("obs_28_incremental", contract["name"])
        self.assertEqual(28, contract["observation_dim"])
        self.assertEqual(6, contract["action_dim"])
        self.assertEqual([0.0, -0.275, -1.42, -1.42, 0.0],
                         self.manifest["grasp_home"]["sim_rad"])
        self.assertEqual([0.205, 0.28],
                         self.manifest["target_envelope"]["trained_x_range_m"])
        self.assertEqual([-0.07, 0.065],
                         self.manifest["target_envelope"]["trained_y_range_m"])

    def test_new_weight_hardware_gates_start_false(self):
        gates = self.manifest["hardware_gates"]
        values = [value for key, value in gates.items() if key != "note"]
        self.assertTrue(values)
        self.assertTrue(all(value is False for value in values))
        self.assertEqual("candidate", self.manifest["status"])

    def test_launcher_injects_locked_identity(self):
        args = self.launcher.build_runtime_argv(["--obj-x", "0.24"])
        self.assertEqual(str(self.launcher.MODEL), args[args.index("--model") + 1])
        self.assertEqual(str(self.launcher.VECNORMALIZE),
                         args[args.index("--vecnorm") + 1])
        self.assertEqual("obs_28_incremental", args[args.index("--contract") + 1])
        self.assertEqual(str(self.launcher.MANIFEST),
                         args[args.index("--release-manifest") + 1])

    def test_launcher_injects_v23_hardware_profile(self):
        args = self.launcher.build_runtime_argv([])
        expected = {
            "--pose-tol-deg": "3.0",
            "--entry-xy-mm": "10.0",
            "--floor-finger-error-mm": "15.0",
            "--tcp-forward-error-mm": "20.0",
            "--jaw-track-fraction": "0.5",
            "--jaw-max-lag-deg": "15.0",
            "--s6-stall-grasp-steps": "2",
        }
        for option, value in expected.items():
            with self.subTest(option=option):
                self.assertEqual(value, args[args.index(option) + 1])

        profile = self.manifest["deployment_profile"]
        self.assertEqual(15.0, profile["floor_finger_error_mm"])
        self.assertEqual(20.0, profile["tcp_forward_error_mm"])
        self.assertEqual(0.5, profile["jaw_track_fraction"])

    def test_launcher_rejects_identity_overrides(self):
        for args in (
            ["--model", "other.zip"],
            ["--vecnorm=other.pkl"],
            ["--contract", "obs_28_absolute"],
            ["--release-manifest", "other.json"],
            ["--floor-finger-error-mm", "0"],
            ["--tcp-forward-error-mm=0"],
            ["--jaw-track-fraction", "0"],
        ):
            with self.subTest(args=args), self.assertRaises(SystemExit):
                self.launcher.build_runtime_argv(args)

    def test_launcher_parses_on_jetson_python_36(self):
        source = (HERE / "run_candidate.py").read_text(encoding="utf-8")
        self.assertNotIn("from __future__ import annotations", source)
        self.assertNotIn(":=", source)

    def test_release_gate_uses_v24_manifest_and_stays_candidate(self):
        cfg = self.runtime.DeployConfig()
        cfg.model_path = str(self.launcher.MODEL)
        cfg.vecnorm_path = str(self.launcher.VECNORMALIZE)
        cfg.contract_name = "obs_28_incremental"
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(self.runtime._release_gate_ok(
                cfg, unlock=False, manifest_path=str(self.launcher.MANIFEST)))
            self.assertTrue(self.runtime._release_gate_ok(
                cfg, unlock=True, manifest_path=str(self.launcher.MANIFEST)))

    def test_release_override_never_waives_hash(self):
        cfg = self.runtime.DeployConfig()
        cfg.model_path = str(ROOT / "grasp" / "v23" / "models" /
                             "candidate_v23_seed23401_ckpt250000.zip")
        cfg.vecnorm_path = str(self.launcher.VECNORMALIZE)
        cfg.contract_name = "obs_28_incremental"
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(self.runtime._release_gate_ok(
                cfg, unlock=True, manifest_path=str(self.launcher.MANIFEST)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
