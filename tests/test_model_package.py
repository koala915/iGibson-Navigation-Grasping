import io
import json
import subprocess
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from model_tools.model_package import (
    ModelPackageError,
    extract_archive_safely,
    install_archive,
    sha256_file,
    verify_package_directory,
)


def make_manifest(model_sha, vec_sha, model_id="grasp-test-v1"):
    return {
        "schema_version": 1,
        "model_id": model_id,
        "task": "grasp",
        "status": "sim-approved",
        "source": {"repository": "owner/repo", "commit": "abcdef1"},
        "files": {
            "model": {"path": "model.zip", "sha256": model_sha},
            "vecnormalize": {"path": "vecnormalize.pkl", "sha256": vec_sha},
        },
        "contract": {
            "observation_dim": 28,
            "action_dim": 6,
            "arm_action_mode": "absolute",
            "grasp_home_sim_rad": [0, -0.4, -1.4, -1.4, 0],
            "grasp_home_api_deg": [90, 67.08, 9.79, 9.79, 90, 30],
            "arm_hw_invert": [False, False, False, False, False],
        },
        "hardware_gates": {"dry_run": False},
    }


class ModelPackageTests(unittest.TestCase):
    def _package_dir(self, root):
        package = root / "package"
        package.mkdir()
        model = package / "model.zip"
        vecnorm = package / "vecnormalize.pkl"
        model.write_bytes(b"model")
        vecnorm.write_bytes(b"vecnorm")
        manifest = make_manifest(sha256_file(model), sha256_file(vecnorm))
        (package / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
        return package

    def _archive(self, package, output):
        with tarfile.open(str(output), "w:gz") as archive:
            archive.add(str(package), arcname=package.name)

    def test_valid_directory(self):
        with tempfile.TemporaryDirectory() as temp:
            package = self._package_dir(Path(temp))
            manifest = verify_package_directory(package)
            self.assertEqual(manifest["model_id"], "grasp-test-v1")

    def test_hash_mismatch_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            package = self._package_dir(Path(temp))
            (package / "model.zip").write_bytes(b"changed")
            with self.assertRaisesRegex(ModelPackageError, "SHA256 mismatch"):
                verify_package_directory(package)

    def test_missing_vecnormalize_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            package = self._package_dir(Path(temp))
            (package / "vecnormalize.pkl").unlink()
            with self.assertRaisesRegex(ModelPackageError, "missing files.vecnormalize"):
                verify_package_directory(package)

    def test_wrong_grasp_dimensions_fail(self):
        with tempfile.TemporaryDirectory() as temp:
            package = self._package_dir(Path(temp))
            manifest_path = package / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["contract"]["observation_dim"] = 27
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with self.assertRaisesRegex(ModelPackageError, "must be 28"):
                verify_package_directory(package)

    def test_grasp_package_requires_explicit_action_semantics(self):
        with tempfile.TemporaryDirectory() as temp:
            package = self._package_dir(Path(temp))
            manifest_path = package / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            del manifest["contract"]["arm_action_mode"]
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with self.assertRaisesRegex(ModelPackageError, "arm_action_mode"):
                verify_package_directory(package)

    def test_legacy_wrapper_refuses_incremental_package(self):
        with tempfile.TemporaryDirectory() as temp:
            package = self._package_dir(Path(temp))
            manifest_path = package / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["contract"]["arm_action_mode"] = "incremental"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            wrapper = ROOT / "model_tools" / "run_grasp_package.py"
            result = subprocess.run(
                [sys.executable, str(wrapper), str(package)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                universal_newlines=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("legacy absolute-action runtime", result.stdout)

    def test_hardware_approved_requires_all_gates(self):
        with tempfile.TemporaryDirectory() as temp:
            package = self._package_dir(Path(temp))
            manifest_path = package / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["status"] = "hardware-approved"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            with self.assertRaisesRegex(ModelPackageError, "failed gates"):
                verify_package_directory(package)

    def test_install_and_preserve_replaced_version(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            package = self._package_dir(root)
            archive = root / "package.tar.gz"
            self._archive(package, archive)
            install_root = root / "installed"
            target, _, backup = install_archive(archive, install_root)
            self.assertTrue((target / "manifest.json").is_file())
            self.assertIsNone(backup)
            target2, _, backup2 = install_archive(archive, install_root, replace=True)
            self.assertEqual(target, target2)
            self.assertTrue((backup2 / "manifest.json").is_file())

    def test_rejects_archive_traversal(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            archive_path = root / "bad.tar.gz"
            with tarfile.open(str(archive_path), "w:gz") as archive:
                info = tarfile.TarInfo("../escape")
                payload = b"bad"
                info.size = len(payload)
                archive.addfile(info, io.BytesIO(payload))
            with self.assertRaisesRegex(ModelPackageError, "unsafe archive path"):
                extract_archive_safely(archive_path, root / "out")

    def test_candidate_wrapper_refuses_real_before_loading_model(self):
        with tempfile.TemporaryDirectory() as temp:
            package = self._package_dir(Path(temp))
            wrapper = Path(__file__).resolve().parent.parent / "model_tools" / "run_grasp_package.py"
            result = subprocess.run(
                [sys.executable, str(wrapper), str(package), "--", "--real"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("hardware-approved is required", result.stdout)

    def test_export_is_reproducible_and_installable(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            model = root / "source-model.zip"
            vecnorm = root / "source-vec.pkl"
            model.write_bytes(b"model payload")
            vecnorm.write_bytes(b"vecnorm payload")
            project = Path(__file__).resolve().parent.parent
            exporter = project / "model_tools" / "export_model_package.py"
            template = project / "model_tools" / "grasp_v18_manifest.template.json"
            outputs = [root / "one.tar.gz", root / "two.tar.gz"]
            for output in outputs:
                result = subprocess.run(
                    [
                        sys.executable, str(exporter),
                        "--manifest-template", str(template),
                        "--model", str(model),
                        "--vecnorm", str(vecnorm),
                        "--output", str(output),
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    universal_newlines=True,
                )
                self.assertEqual(result.returncode, 0, result.stdout)
            self.assertEqual(outputs[0].read_bytes(), outputs[1].read_bytes())
            target, manifest, _ = install_archive(outputs[0], root / "installed")
            self.assertEqual(manifest["model_id"], "grasp-v18.0.0-candidate")
            self.assertEqual(verify_package_directory(target)["status"], "sim-approved")


if __name__ == "__main__":
    unittest.main()
