#!/usr/bin/env python3
"""Shared validation and safe archive helpers for X3Plus model packages.

This module intentionally uses only the Python standard library so it can run
on the Jetson before Stable-Baselines3 or the robot stack is imported.
"""
from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import tarfile
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple


SCHEMA_VERSION = 1
ALLOWED_TASKS = {"grasp", "navigation"}
ALLOWED_STATUSES = {"candidate", "sim-approved", "hardware-approved", "retired"}
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class ModelPackageError(RuntimeError):
    """Raised when a model package is malformed or fails verification."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ModelPackageError(message)


def _safe_relative_path(value: Any, field: str) -> Path:
    _require(isinstance(value, str) and bool(value.strip()), f"{field} must be a non-empty string")
    path = Path(value)
    _require(not path.is_absolute(), f"{field} must be relative")
    _require(".." not in path.parts, f"{field} must not contain '..'")
    _require(path.name not in {"", ".", ".."}, f"{field} is invalid")
    return path


def load_manifest(path: Path) -> Dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ModelPackageError(f"could not read manifest {path}: {exc}") from exc
    _require(isinstance(data, dict), "manifest root must be a JSON object")
    return data


def validate_manifest(manifest: Mapping[str, Any]) -> Dict[str, Any]:
    """Validate schema and return a shallow plain-dict copy."""
    _require(manifest.get("schema_version") == SCHEMA_VERSION,
             f"schema_version must be {SCHEMA_VERSION}")
    model_id = manifest.get("model_id")
    _require(isinstance(model_id, str) and re.fullmatch(r"[a-z0-9][a-z0-9._-]{2,63}", model_id or "") is not None,
             "model_id must be 3-64 lowercase letters, digits, dot, underscore, or dash")
    task = manifest.get("task")
    _require(task in ALLOWED_TASKS, f"task must be one of {sorted(ALLOWED_TASKS)}")
    status = manifest.get("status")
    _require(status in ALLOWED_STATUSES, f"status must be one of {sorted(ALLOWED_STATUSES)}")

    source = manifest.get("source")
    _require(isinstance(source, dict), "source must be an object")
    _require(isinstance(source.get("repository"), str) and "/" in source["repository"],
             "source.repository must be owner/name")
    _require(isinstance(source.get("commit"), str) and
             re.fullmatch(r"[0-9a-fA-F]{7,40}", source["commit"]) is not None,
             "source.commit must be a 7-40 character Git SHA")

    files = manifest.get("files")
    _require(isinstance(files, dict), "files must be an object")
    required_roles = ("model", "vecnormalize")
    for role in required_roles:
        spec = files.get(role)
        _require(isinstance(spec, dict), f"files.{role} must be an object")
        _safe_relative_path(spec.get("path"), f"files.{role}.path")
        digest = spec.get("sha256")
        _require(isinstance(digest, str) and SHA256_RE.fullmatch(digest) is not None,
                 f"files.{role}.sha256 must be a lowercase SHA256")

    contract = manifest.get("contract")
    _require(isinstance(contract, dict), "contract must be an object")
    for key in ("observation_dim", "action_dim"):
        _require(isinstance(contract.get(key), int) and contract[key] > 0,
                 f"contract.{key} must be a positive integer")

    if task == "grasp":
        _require(contract.get("observation_dim") == 28,
                 "grasp contract observation_dim must be 28")
        _require(contract.get("action_dim") == 6,
                 "grasp contract action_dim must be 6")
        _require(contract.get("arm_action_mode") in {"absolute", "incremental"},
                 "grasp contract arm_action_mode must be 'absolute' or 'incremental'")
        for key, length in (("grasp_home_sim_rad", 5), ("grasp_home_api_deg", 6)):
            values = contract.get(key)
            _require(isinstance(values, list) and len(values) == length and
                     all(isinstance(value, (int, float)) for value in values),
                     f"contract.{key} must contain {length} numbers")
        invert = contract.get("arm_hw_invert")
        _require(isinstance(invert, list) and len(invert) == 5 and
                 all(isinstance(value, bool) for value in invert),
                 "contract.arm_hw_invert must contain 5 booleans")

    gates = manifest.get("hardware_gates", {})
    _require(isinstance(gates, dict) and all(isinstance(value, bool) for value in gates.values()),
             "hardware_gates must map names to booleans")
    if status == "hardware-approved":
        _require(bool(gates), "hardware-approved packages must declare hardware_gates")
        failed = sorted(name for name, passed in gates.items() if not passed)
        _require(not failed, "hardware-approved package has failed gates: " + ", ".join(failed))

    return dict(manifest)


def verify_package_directory(package_dir: Path) -> Dict[str, Any]:
    package_dir = package_dir.resolve()
    manifest_path = package_dir / "manifest.json"
    manifest = validate_manifest(load_manifest(manifest_path))
    for role, spec in manifest["files"].items():
        if not isinstance(spec, dict) or "path" not in spec or "sha256" not in spec:
            continue
        relative = _safe_relative_path(spec["path"], f"files.{role}.path")
        file_path = (package_dir / relative).resolve()
        _require(os.path.commonpath([str(package_dir), str(file_path)]) == str(package_dir),
                 f"files.{role}.path escapes package directory")
        _require(file_path.is_file(), f"missing files.{role}: {relative}")
        actual = sha256_file(file_path)
        _require(actual == spec["sha256"],
                 f"SHA256 mismatch for {relative}: expected {spec['sha256']}, got {actual}")
    return manifest


def _validate_tar_members(archive: tarfile.TarFile) -> List[tarfile.TarInfo]:
    members = archive.getmembers()
    _require(bool(members), "model package archive is empty")
    for member in members:
        path = Path(member.name)
        _require(not path.is_absolute() and ".." not in path.parts,
                 f"unsafe archive path: {member.name}")
        _require(not member.issym() and not member.islnk(),
                 f"links are not allowed in model packages: {member.name}")
        _require(member.isfile() or member.isdir(),
                 f"unsupported archive member: {member.name}")
    return members


def extract_archive_safely(archive_path: Path, destination: Path) -> Path:
    """Extract archive and return the directory containing manifest.json."""
    destination.mkdir(parents=True, exist_ok=True)
    try:
        with tarfile.open(str(archive_path), "r:gz") as archive:
            members = _validate_tar_members(archive)
            archive.extractall(str(destination), members=members)
    except (OSError, tarfile.TarError) as exc:
        raise ModelPackageError(f"could not extract {archive_path}: {exc}") from exc

    manifests = list(destination.rglob("manifest.json"))
    _require(len(manifests) == 1,
             f"archive must contain exactly one manifest.json, found {len(manifests)}")
    return manifests[0].parent


def install_archive(archive_path: Path, install_root: Path, replace: bool = False) -> Tuple[Path, Dict[str, Any], Optional[Path]]:
    """Verify then atomically install an archive, preserving replaced versions."""
    install_root = install_root.resolve()
    install_root.mkdir(parents=True, exist_ok=True)
    temp_dir = Path(tempfile.mkdtemp(prefix=".model-install-", dir=str(install_root)))
    backup = None
    try:
        extracted = extract_archive_safely(archive_path.resolve(), temp_dir)
        manifest = verify_package_directory(extracted)
        target = install_root / manifest["model_id"]
        if target.exists():
            if not replace:
                raise ModelPackageError(f"{target} already exists; pass --replace to preserve and replace it")
            backup_base = target.name + ".backup-" + time.strftime("%Y%m%d-%H%M%S")
            backup = install_root / backup_base
            suffix = 1
            while backup.exists():
                backup = install_root / (backup_base + "-" + str(suffix))
                suffix += 1
            target.rename(backup)
        try:
            extracted.rename(target)
        except Exception:
            if backup is not None and backup.exists() and not target.exists():
                backup.rename(target)
            raise
        return target, manifest, backup
    finally:
        if temp_dir.exists():
            shutil.rmtree(str(temp_dir), ignore_errors=True)


def deployment_paths(package_dir: Path, manifest: Mapping[str, Any]) -> Tuple[Path, Path]:
    return (
        package_dir / manifest["files"]["model"]["path"],
        package_dir / manifest["files"]["vecnormalize"]["path"],
    )
