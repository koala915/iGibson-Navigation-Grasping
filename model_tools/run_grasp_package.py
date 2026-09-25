#!/usr/bin/env python3
"""Fail-closed wrapper that runs grasp deployment with a verified model package."""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from model_tools.model_package import deployment_paths, verify_package_directory


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package_dir", type=Path)
    parser.add_argument("deploy_args", nargs=argparse.REMAINDER,
                        help="arguments after -- are passed to x3plus_real_grasp.py")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    deploy_args = list(args.deploy_args)
    if deploy_args and deploy_args[0] == "--":
        deploy_args.pop(0)
    forbidden = {"--model", "--vecnorm", "--grasp-home-deg"}
    supplied = [value for value in deploy_args if value.split("=", 1)[0] in forbidden]
    if supplied:
        raise SystemExit("model package wrapper owns these options: " + ", ".join(supplied))

    package_dir = args.package_dir.resolve()
    manifest = verify_package_directory(package_dir)
    action_mode = manifest["contract"].get("arm_action_mode")
    if action_mode != "absolute":
        raise SystemExit(
            "this wrapper launches the legacy absolute-action runtime and refuses "
            f"a {action_mode!r} package; use the versioned v21/v23 launcher for "
            "incremental models"
        )
    is_real = "--real" in deploy_args
    if is_real:
        if manifest["status"] != "hardware-approved":
            raise SystemExit(
                f"refusing --real with {manifest['status']} package; hardware-approved is required"
            )
        failed = sorted(name for name, passed in manifest.get("hardware_gates", {}).items() if not passed)
        if failed:
            raise SystemExit("refusing --real; failed hardware gates: " + ", ".join(failed))

    model, vecnorm = deployment_paths(package_dir, manifest)
    home = ",".join(str(value) for value in manifest["contract"]["grasp_home_api_deg"])
    script = Path(__file__).resolve().parent.parent / "grasp" / "x3plus_real_grasp.py"
    command = [
        sys.executable, str(script),
        "--model", str(model),
        "--vecnorm", str(vecnorm),
        "--grasp-home-deg", home,
    ] + deploy_args
    print("verified package:", manifest["model_id"], manifest["status"])
    print("launching:", " ".join(command))
    raise SystemExit(subprocess.call(command))


if __name__ == "__main__":
    main()
